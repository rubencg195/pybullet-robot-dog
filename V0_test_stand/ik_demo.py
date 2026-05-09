#!/usr/bin/env python3
"""V0 IK Demo — trace target paths with inverse kinematics.

Draws a target trajectory (orange), solves IK for each point, and
drives the leg to follow it.  The actual foot path (green) is drawn
on top so you can compare tracking accuracy.

Trajectory types
----------------
  circle  — vertical circle in the XZ plane (default)
  line    — forward/backward sweep in X
  step    — elliptical walking-step cycle (flat stance, arched swing)

Usage
-----
    python V0_test_stand/ik_demo.py
    python V0_test_stand/ik_demo.py --path step --speed 0.3
    python V0_test_stand/ik_demo.py --path circle --record recordings/ik_circle.gif
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

STAND_HEIGHT = 0.35
URDF_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "urdf", "leg_test_stand.urdf"
)

# Match test_stand.py defaults (front = coronal from +X).
CAM_FRONT = dict(
    cameraDistance=0.52,
    cameraYaw=-90.0,
    cameraPitch=-20.0,
    cameraTargetPosition=[0.0, 0.0, STAND_HEIGHT - 0.06],
)
CAM_SIDE = dict(
    cameraDistance=0.5,
    cameraYaw=45.0,
    cameraPitch=-30.0,
    cameraTargetPosition=[0.0, -0.03, STAND_HEIGHT - 0.12],
)

# Nominal stance: slight hip flexion + knee bend so the foot is well
# inside the workspace and the circle/path has room around it.
NOMINAL_ANGLES = (0.0, 0.3, -0.6)


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
    parser.add_argument("--record", default=None, help="Save as GIF")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--width", type=int, default=800)
    parser.add_argument("--height", type=int, default=600)
    parser.add_argument(
        "--camera",
        choices=("front", "side"),
        default="front",
        help="front = coronal from +X; side = old yaw=45 view",
    )
    args = parser.parse_args()

    # ── PyBullet setup ──────────────────────────────────────────────────
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    cam = CAM_FRONT if args.camera == "front" else CAM_SIDE
    p.resetDebugVisualizerCamera(**cam)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

    p.loadURDF("plane.urdf")
    robot = p.loadURDF(URDF_PATH, [0, 0, STAND_HEIGHT], useFixedBase=True)

    joints = {}
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        if info[2] == p.JOINT_REVOLUTE:
            joints[info[1].decode()] = i

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
                             step_length=0.06, step_height=0.04)

    # draw full target path once (orange, persistent)
    world_targets = [t + base for t in targets]
    viz.draw_path(world_targets, color=(1, 0.5, 0), width=1.5, closed=True)

    # ── animation loop ──────────────────────────────────────────────────
    idx = 0
    loop_count = 0
    ik_failures = 0
    frames: list[np.ndarray] = []

    print(f"IK demo: path={args.path}  radius/half={args.radius}  "
          f"speed={args.speed}  loops={'inf' if args.loops == 0 else args.loops}")
    print("Press Ctrl+C or close the window to stop.\n")

    try:
        while True:
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
                    robot, joints["knee_flexion"],
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

            if args.record:
                _, _, rgba, _, _ = p.getCameraImage(
                    args.width, args.height, renderer=p.ER_TINY_RENDERER)
                frames.append(
                    np.reshape(rgba, (args.height, args.width, 4)))

            idx += 1
            if idx >= len(targets):
                idx = 0
                loop_count += 1
                if args.loops > 0 and loop_count >= args.loops:
                    break

            time.sleep(1.0 / (240.0 * args.speed))

    except (KeyboardInterrupt, SystemExit):
        pass

    if args.record and frames:
        _save_gif(frames, args.record, args.fps)

    p.disconnect()
    print(f"\nCompleted {loop_count} loops.  IK failures: {ik_failures}")


def _save_gif(frames, path, fps):
    try:
        from PIL import Image

        images = [Image.fromarray(f[:, :, :3]) for f in frames]
        if images:
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
            images[0].save(
                path, save_all=True, append_images=images[1:],
                duration=int(1000 / fps), loop=0,
            )
            print(f"Saved {len(images)} frames → {path}")
    except ImportError:
        print("Install Pillow for GIF recording:  pip install Pillow")


if __name__ == "__main__":
    main()
