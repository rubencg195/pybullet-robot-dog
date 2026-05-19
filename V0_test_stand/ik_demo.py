#!/usr/bin/env python3
"""Drive the test-stand leg along a scripted path with IK.

Orange polyline = target, green trail = where the foot actually lands. Paths:
``circle`` (default, in XZ), ``line`` (sweep along X), ``step`` (flat stance +
lofted swing). Same camera and ``--record`` / ``--snapshot`` flags as
``test_stand.py``.

Examples::

    python V0_test_stand/ik_demo.py --path step --speed 0.3
    python V0_test_stand/ik_demo.py --record recordings/ik.gif --loops 2
"""

import sys
import os
import time
import argparse

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pybullet as p
import pybullet_data

from common.kinematics import (
    LegConfig,
    forward_kinematics,
    forward_kinematics_full,
    inverse_kinematics,
)
from common.debug_visualizer import DebugVisualizer
from common.view_capture import rgba_from_debug_view

STAND_HEIGHT = 0.35
URDF_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "urdf", "leg_test_stand.urdf"
)

CAM_STAND = dict(
    cameraDistance=0.52,
    cameraYaw=-90.0,
    cameraPitch=-20.0,
    cameraTargetPosition=[0.0, 0.0, STAND_HEIGHT - 0.06],
)
CAM_ISO = dict(
    cameraDistance=0.5,
    cameraYaw=45.0,
    cameraPitch=-30.0,
    cameraTargetPosition=[0.0, -0.03, STAND_HEIGHT - 0.12],
)
CAM_CORONAL = dict(
    cameraDistance=0.52,
    cameraYaw=0.0,
    cameraPitch=-20.0,
    cameraTargetPosition=[0.0, 0.0, STAND_HEIGHT - 0.06],
)
CAM_PRESETS = {"stand": CAM_STAND, "iso": CAM_ISO, "coronal": CAM_CORONAL}

# Nominal stance: slight hip flexion + knee bend so the foot is well
# inside the workspace and the circle/path has room around it.
# With L2=L3=60mm, full extension = 120mm; keep foot well inside.
NOMINAL_ANGLES = (0.0, 0.4, -0.8)


# ── trajectory generators ──────────────────────────────────────────────

def _circle_path(center, radius, n=200):
    """Vertical circle in the XZ plane."""
    normal = np.array([0.0, 1.0, 0.0])
    # u and v span the circle plane
    u = np.array([1.0, 0.0, 0.0])
    v = np.array([0.0, 0.0, 1.0])
    return [
        center + radius * (np.cos(2 * np.pi * i / n) * u
                           + np.sin(2 * np.pi * i / n) * v)
        for i in range(n)
    ]


def _line_path(center, half_length, n=200):
    """Forward/backward sweep along X, bouncing back."""
    fwd = [center + np.array([half_length * (2 * t / n - 1), 0, 0])
           for t in range(n // 2)]
    bwd = list(reversed(fwd))
    return fwd + bwd


def _step_path(center, step_length, step_height, n=200):
    """Elliptical stepping trajectory: flat stance, arched swing."""
    points = []
    for i in range(n):
        t = 2 * np.pi * i / n
        x = step_length / 2 * np.cos(t)
        z = step_height / 2 * np.sin(t)
        z = max(z, 0.0)  # flat ground phase
        points.append(center + np.array([x, 0.0, z]))
    return points


# ── main ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="IK demo — trace paths with inverse kinematics"
    )
    parser.add_argument(
        "--path", choices=["circle", "line", "step"], default="circle",
        help="Trajectory shape (default: circle)",
    )
    parser.add_argument("--radius", type=float, default=0.03,
                        help="Circle radius or half-length in m (default: 0.03)")
    parser.add_argument("--speed", type=float, default=0.5,
                        help="Speed multiplier (default: 0.5)")
    parser.add_argument("--loops", type=int, default=0,
                        help="Number of loops, 0 = infinite (default: 0)")
    parser.add_argument("--record", metavar="FILE", default=None,
                        help="Save GUI view to GIF (debug camera)")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--width", type=int, default=800)
    parser.add_argument("--height", type=int, default=600)
    parser.add_argument(
        "--snapshot", metavar="FILE", default=None,
        help="Save one PNG from debug camera on exit",
    )
    parser.add_argument(
        "--camera",
        choices=tuple(CAM_PRESETS.keys()),
        default="stand",
        help="stand=profile (default); iso; coronal",
    )
    parser.add_argument(
        "--urdf",
        default=None,
        metavar="PATH",
        help="Alternate leg URDF (default: V0). Use V1 mesh URDF when meshes exist.",
    )
    args = parser.parse_args()

    urdf_path = os.path.abspath(args.urdf) if args.urdf else URDF_PATH

    if args.record or args.snapshot:
        try:
            from PIL import Image  # noqa: F401
        except ImportError:
            print("Install Pillow for --record / --snapshot: pip install Pillow")
            sys.exit(1)

    # ── PyBullet setup ──────────────────────────────────────────────────
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    cam = CAM_PRESETS[args.camera]
    p.resetDebugVisualizerCamera(**cam)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

    p.loadURDF("plane.urdf")
    print(f"[IK demo] URDF: {urdf_path}")
    robot = p.loadURDF(urdf_path, [0, 0, STAND_HEIGHT], useFixedBase=True)

    joints = {}
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        if info[2] == p.JOINT_REVOLUTE:
            joints[info[1].decode()] = i

    # Four-bar loop closure (if coupler_link and tibia_link present)
    def _find_link(name):
        for i in range(p.getNumJoints(robot)):
            if p.getJointInfo(robot, i)[12].decode() == name:
                return i
        return -1

    coupler_idx = _find_link("coupler_link")
    tibia_idx = _find_link("tibia_link")
    if coupler_idx >= 0 and tibia_idx >= 0:
        cid = p.createConstraint(
            robot, coupler_idx, robot, tibia_idx,
            p.JOINT_POINT2POINT, [0, 0, 0],
            [0, 0, -0.060], [0, 0, 0])
        p.changeConstraint(cid, maxForce=500)
        if "coupler_passive" in joints:
            p.setJointMotorControl2(robot, joints["coupler_passive"],
                                    p.VELOCITY_CONTROL, targetVelocity=0, force=0)

    cfg = LegConfig()
    viz = DebugVisualizer()
    base = np.array([0.0, 0.0, STAND_HEIGHT])

    # ── build target trajectory (in hip frame) ──────────────────────────
    nominal_foot = forward_kinematics(*NOMINAL_ANGLES, cfg)

    if args.path == "circle":
        targets = _circle_path(nominal_foot, args.radius)
    elif args.path == "line":
        targets = _line_path(nominal_foot, args.radius)
    else:
        targets = _step_path(nominal_foot,
                             step_length=0.03, step_height=0.02)

    # draw full target path once (orange, persistent)
    world_targets = [t + base for t in targets]
    viz.draw_path(world_targets, color=(1, 0.5, 0), width=1.5, closed=True)

    # ── animation loop ──────────────────────────────────────────────────
    idx = 0
    loop_count = 0
    ik_failures = 0
    recording = args.record is not None
    capture_interval = 1.0 / args.fps if recording else 0.0
    last_capture = 0.0
    frames_pil: list = []

    print(f"IK demo: path={args.path}  radius/half={args.radius}  "
          f"speed={args.speed}  loops={'inf' if args.loops == 0 else args.loops}")
    if recording:
        print(f"Recording GUI view → {args.record} at {args.fps} fps.")
    print("Press Ctrl+C or close the window to stop.\n")

    try:
        while True:
            try:
                if not p.isConnected():
                    break
            except Exception:
                break

            target = np.array(targets[idx])
            target_w = target + base

            result = inverse_kinematics(target, cfg)

            if result is not None:
                q1, q2, q3 = result

                p.setJointMotorControl2(
                    robot, joints["hip_abduction"],
                    p.POSITION_CONTROL, q1, force=10, maxVelocity=10)
                p.setJointMotorControl2(
                    robot, joints["hip_flexion"],
                    p.POSITION_CONTROL, q2, force=10, maxVelocity=10)
                p.setJointMotorControl2(
                    robot, joints["knee_passive"],
                    p.POSITION_CONTROL, q3, force=10, maxVelocity=10)

                hip, shoulder, knee, foot = forward_kinematics_full(
                    q1, q2, q3, cfg)
                foot_w = foot + base

                viz.add_trail_point(foot_w, "actual", color=(0, 1, 0))
                viz.draw_leg_skeleton(
                    [hip + base, shoulder + base, knee + base, foot_w],
                    color=(1, 1, 0), width=2,
                )

                err_mm = np.linalg.norm(target - foot) * 1000
                viz.update_text(
                    f"q1={np.degrees(q1):+6.1f}\u00b0  "
                    f"q2={np.degrees(q2):+6.1f}\u00b0  "
                    f"q3={np.degrees(q3):+6.1f}\u00b0",
                    "ik_angles", base + [0.12, 0, 0.07],
                )
                viz.update_text(
                    f"Target err: {err_mm:.2f} mm   "
                    f"Failures: {ik_failures}",
                    "error", base + [0.12, 0, 0.04],
                )
            else:
                ik_failures += 1
                viz.update_text(
                    f"IK UNREACHABLE  target=({target[0]:+.3f}, "
                    f"{target[1]:+.3f}, {target[2]:+.3f})",
                    "ik_angles", base + [0.12, 0, 0.07], color=(1, 0, 0),
                )

            viz.draw_point(target_w, color=(1, 0, 0), size=0.008,
                           lifetime=0.1)

            p.stepSimulation()

            if recording:
                now_cap = time.monotonic()
                if now_cap - last_capture >= capture_interval:
                    from PIL import Image

                    rgba = rgba_from_debug_view(p, args.width, args.height)
                    frames_pil.append(Image.fromarray(rgba[:, :, :3]))
                    last_capture = now_cap

            idx += 1
            if idx >= len(targets):
                idx = 0
                loop_count += 1
                if args.loops > 0 and loop_count >= args.loops:
                    break

            time.sleep(1.0 / (240.0 * args.speed))

    except (KeyboardInterrupt, SystemExit):
        pass

    if args.snapshot:
        try:
            if p.isConnected():
                from PIL import Image

                rgba = rgba_from_debug_view(p, args.width, args.height)
                out = os.path.abspath(args.snapshot)
                parent = os.path.dirname(out)
                if parent:
                    os.makedirs(parent, exist_ok=True)
                Image.fromarray(rgba[:, :, :3]).save(out)
                print(f"Snapshot saved → {out}")
        except Exception as exc:
            print(f"Snapshot failed: {exc}")

    if recording and frames_pil:
        _save_gif(frames_pil, args.record, args.fps)

    try:
        if p.isConnected():
            p.disconnect()
    except Exception:
        pass
    print(f"\nCompleted {loop_count} loops.  IK failures: {ik_failures}")


def _save_gif(images, path, fps):
    if not images:
        return
    out = os.path.abspath(path)
    parent = os.path.dirname(out)
    if parent:
        os.makedirs(parent, exist_ok=True)
    images[0].save(
        out, format="GIF", save_all=True, append_images=images[1:],
        duration=int(1000 / fps), loop=0,
    )
    print(f"Saved {len(images)} frames → {out}")


if __name__ == "__main__":
    main()
