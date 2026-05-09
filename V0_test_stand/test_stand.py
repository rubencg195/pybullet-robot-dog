#!/usr/bin/env python3
"""V0 Test Stand — interactive single-leg control with joint sliders.

Loads a SpotMicro-inspired 3-DOF leg on a fixed test stand.  The PyBullet
**Params** panel shows **sliders**: three joints in **degrees** plus
**Clear Trail**.  Default camera **stand** matches the physical test-stand
**profile** (sagittal-style: yaw −90°, pitch −20°, target near hip).  Recording
uses the **same** debug camera as the GUI (like ``interactive_robot_arm.py``).

Usage
-----
    python V0_test_stand/test_stand.py
    python V0_test_stand/test_stand.py --camera iso
    python V0_test_stand/test_stand.py --record recordings/v0_session.gif --fps 15
    python V0_test_stand/test_stand.py --snapshot recordings/README_v0_still.png
"""

import sys
import os
import time
import argparse

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pybullet as p
import pybullet_data

from common.kinematics import LegConfig, forward_kinematics_full
from common.debug_visualizer import DebugVisualizer
from common.view_capture import rgba_from_debug_view

STAND_HEIGHT = 0.35
URDF_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "urdf", "leg_test_stand.urdf"
)

JOINT_ORDER = ["hip_abduction", "hip_flexion", "knee_flexion"]
LOG_INTERVAL = 120  # print to terminal every N simulation steps

# **stand** — test-stand profile (matches typical side/profile screenshot: yaw −90, pitch −20).
CAM_STAND = dict(
    cameraDistance=0.52,
    cameraYaw=-90.0,
    cameraPitch=-20.0,
    cameraTargetPosition=[0.0, 0.0, STAND_HEIGHT - 0.06],
)
# **iso** — corner / isometric (older default).
CAM_ISO = dict(
    cameraDistance=0.5,
    cameraYaw=45.0,
    cameraPitch=-30.0,
    cameraTargetPosition=[0.0, -0.03, STAND_HEIGHT - 0.12],
)
# **coronal** — face the leg from +X (width / abduction opening toward camera).
CAM_CORONAL = dict(
    cameraDistance=0.52,
    cameraYaw=0.0,
    cameraPitch=-20.0,
    cameraTargetPosition=[0.0, 0.0, STAND_HEIGHT - 0.06],
)

CAM_PRESETS = {"stand": CAM_STAND, "iso": CAM_ISO, "coronal": CAM_CORONAL}


def _find_link_index(robot_id, link_name):
    for i in range(p.getNumJoints(robot_id)):
        if p.getJointInfo(robot_id, i)[12].decode() == link_name:
            return i
    return -1


def main():
    parser = argparse.ArgumentParser(
        description="SpotMicro leg test stand — interactive FK explorer"
    )
    parser.add_argument("--record", metavar="FILE", default=None,
                        help="Record the GUI view to an animated GIF (debug camera)")
    parser.add_argument("--fps", type=int, default=15, help="GIF frame rate when --record")
    parser.add_argument("--width", type=int, default=800, help="Capture / snapshot width")
    parser.add_argument("--height", type=int, default=600, help="Capture / snapshot height")
    parser.add_argument(
        "--snapshot", metavar="FILE", default=None,
        help="Save one PNG from the debug camera on exit (README still image)",
    )
    parser.add_argument(
        "--camera",
        choices=tuple(CAM_PRESETS.keys()),
        default="stand",
        help="stand=test-stand profile (default); iso=yaw 45; coronal=from +X",
    )
    args = parser.parse_args()

    if args.record or args.snapshot:
        try:
            from PIL import Image  # noqa: F401
        except ImportError:
            print("Install Pillow for --record / --snapshot: pip install Pillow")
            sys.exit(1)

    print("=" * 60)
    print("  V0 TEST STAND — SpotMicro Leg Simulation")
    print("=" * 60)
    print(f"  URDF path : {URDF_PATH}")
    print(f"  Stand height : {STAND_HEIGHT} m")
    print(f"  URDF exists  : {os.path.isfile(URDF_PATH)}")
    print()

    # ── PyBullet setup ──────────────────────────────────────────────────
    print("[INIT] Connecting to PyBullet GUI...")
    cid = p.connect(p.GUI)
    print(f"[INIT] PyBullet connected  (client id = {cid})")

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    cam = CAM_PRESETS[args.camera]
    p.resetDebugVisualizerCamera(**cam)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    print(f"[INIT] Camera preset: {args.camera}  ({cam})")

    print("[INIT] Loading ground plane...")
    plane_id = p.loadURDF("plane.urdf")
    print(f"[INIT] Ground plane loaded  (body id = {plane_id})")

    print(f"[INIT] Loading leg URDF from {URDF_PATH} ...")
    robot = p.loadURDF(URDF_PATH, [0, 0, STAND_HEIGHT], useFixedBase=True)
    print(f"[INIT] Leg loaded  (body id = {robot})")

    # ── discover revolute joints ────────────────────────────────────────
    num_joints = p.getNumJoints(robot)
    print(f"[INIT] Total joints in URDF: {num_joints}")
    joints = {}
    for i in range(num_joints):
        info = p.getJointInfo(robot, i)
        jtype = {0: "REVOLUTE", 1: "PRISMATIC", 2: "SPHERICAL",
                 3: "PLANAR", 4: "FIXED"}.get(info[2], f"UNKNOWN({info[2]})")
        jname = info[1].decode()
        lname = info[12].decode()
        print(f"  joint[{i}]  name={jname:<20s}  type={jtype:<10s}  "
              f"child_link={lname:<16s}  limits=[{info[8]:.3f}, {info[9]:.3f}]")
        if info[2] == p.JOINT_REVOLUTE:
            joints[jname] = {"idx": i, "lower": info[8], "upper": info[9]}

    print(f"[INIT] Revolute joints found: {list(joints.keys())}")

    # Params panel sliders: joint targets in **degrees** (converted to rad for PyBullet).
    sliders = {}
    for name in JOINT_ORDER:
        j = joints[name]
        lo_deg = float(np.degrees(j["lower"]))
        hi_deg = float(np.degrees(j["upper"]))
        label = name.replace("_", " ").title() + " (deg)"
        sliders[name] = p.addUserDebugParameter(label, lo_deg, hi_deg, 0.0)
    print(f"[INIT] GUI sliders (degrees) for: {JOINT_ORDER}")

    clear_btn = p.addUserDebugParameter("Clear Trail", 0, 1, 0)
    last_clear = p.readUserDebugParameter(clear_btn)

    # ── helpers ─────────────────────────────────────────────────────────
    cfg = LegConfig()
    print(f"[INIT] LegConfig: L1={cfg.L1}m  L2={cfg.L2}m  L3={cfg.L3}m  "
          f"side_sign={cfg.side_sign}")

    viz = DebugVisualizer()
    base = np.array([0.0, 0.0, STAND_HEIGHT])
    foot_link_idx = _find_link_index(robot, "foot_link")
    print(f"[INIT] foot_link index = {foot_link_idx}")

    # verify initial foot position
    if foot_link_idx >= 0:
        init_state = p.getLinkState(robot, foot_link_idx)
        print(f"[INIT] Initial foot world pos (PyBullet): "
              f"({init_state[0][0]:.4f}, {init_state[0][1]:.4f}, {init_state[0][2]:.4f})")

    hip0, sh0, kn0, ft0 = forward_kinematics_full(0, 0, 0, cfg)
    print(f"[INIT] FK at zero angles — foot (hip frame): "
          f"({ft0[0]:+.4f}, {ft0[1]:+.4f}, {ft0[2]:+.4f})")
    print(f"[INIT] FK at zero angles — foot (world):     "
          f"({ft0[0]+base[0]:+.4f}, {ft0[1]+base[1]:+.4f}, {ft0[2]+base[2]:+.4f})")
    print()

    step_count = 0
    max_fk_err = 0.0
    t_start = time.time()

    print("[RUN ] Simulation running — move sliders in the PyBullet GUI window.")
    print(f"[RUN ] Terminal log every {LOG_INTERVAL} steps (~{LOG_INTERVAL/240:.1f}s).")
    if args.record:
        print(f"[RUN ] Recording GUI view → {args.record} at {args.fps} fps "
              f"({args.width}x{args.height}).")
    if args.snapshot:
        print(f"[RUN ] Will save still PNG on exit → {args.snapshot}")
    print("[RUN ] Press Ctrl+C or close the window to exit.")
    print("-" * 60)

    recording = args.record is not None
    capture_interval = 1.0 / args.fps if recording else 0.0
    last_capture = 0.0
    frames_pil: list = []

    try:
        while True:
            try:
                if not p.isConnected():
                    print("\n[RUN ] GUI window closed.")
                    break
            except Exception:
                break

            # clear-trail button
            cv = p.readUserDebugParameter(clear_btn)
            if cv != last_clear:
                last_clear = cv
                viz.clear_trail("foot")
                print("[EVENT] Trail cleared")

            # read sliders (degrees → radians, clamp to URDF limits)
            q_raw = []
            for name in JOINT_ORDER:
                lo, hi = joints[name]["lower"], joints[name]["upper"]
                rad = np.radians(p.readUserDebugParameter(sliders[name]))
                q_raw.append(float(np.clip(rad, lo, hi)))
            q1, q2, q3 = q_raw

            # drive joints
            for name, q in zip(JOINT_ORDER, (q1, q2, q3)):
                p.setJointMotorControl2(
                    robot,
                    joints[name]["idx"],
                    p.POSITION_CONTROL,
                    targetPosition=q,
                    force=10,
                    maxVelocity=5.0,
                )

            p.stepSimulation()
            step_count += 1

            # FK (our math)
            hip, shoulder, knee, foot = forward_kinematics_full(q1, q2, q3, cfg)
            hip_w = hip + base
            shoulder_w = shoulder + base
            knee_w = knee + base
            foot_w = foot + base

            # FK verification against PyBullet
            if foot_link_idx >= 0:
                pb_foot = np.array(p.getLinkState(robot, foot_link_idx)[0])
                fk_err = np.linalg.norm(foot_w - pb_foot)
            else:
                pb_foot = foot_w
                fk_err = 0.0

            max_fk_err = max(max_fk_err, fk_err)

            # periodic terminal log
            if step_count % LOG_INTERVAL == 0:
                elapsed = time.time() - t_start
                fps_actual = step_count / elapsed if elapsed > 0 else 0
                print(
                    f"[STEP {step_count:>6d}]  "
                    f"q=({np.degrees(q1):+6.1f}, {np.degrees(q2):+6.1f}, {np.degrees(q3):+6.1f}) deg  "
                    f"foot_hip=({foot[0]:+.4f}, {foot[1]:+.4f}, {foot[2]:+.4f})  "
                    f"foot_world=({foot_w[0]:+.4f}, {foot_w[1]:+.4f}, {foot_w[2]:+.4f})  "
                    f"reach={np.linalg.norm(foot):.4f}m  "
                    f"fk_err={fk_err:.6f}m  "
                    f"pb_foot=({pb_foot[0]:.4f}, {pb_foot[1]:.4f}, {pb_foot[2]:.4f})  "
                    f"fps={fps_actual:.0f}"
                )

            # visualise
            viz.draw_leg_skeleton([hip_w, shoulder_w, knee_w, foot_w])
            viz.add_trail_point(foot_w, "foot", color=(0, 1, 0))

            for pos, c in [
                (hip_w, (1, 0.3, 0.3)),
                (shoulder_w, (0.3, 0.3, 1)),
                (knee_w, (1, 0.7, 0)),
                (foot_w, (0, 1, 0)),
            ]:
                viz.draw_point(pos, c, size=0.006)

            viz.draw_coordinate_frame(hip_w, size=0.025)

            # HUD
            viz.update_text(
                f"q1={np.degrees(q1):+6.1f}\u00b0  "
                f"q2={np.degrees(q2):+6.1f}\u00b0  "
                f"q3={np.degrees(q3):+6.1f}\u00b0",
                "angles",
                base + [0.12, 0.0, 0.07],
            )
            viz.update_text(
                f"Foot (hip):  x={foot[0]:+.4f}  y={foot[1]:+.4f}  z={foot[2]:+.4f} m",
                "foot_local",
                base + [0.12, 0.0, 0.04],
            )
            viz.update_text(
                f"Reach: {np.linalg.norm(foot):.4f} m    FK err: {fk_err:.6f} m",
                "reach",
                base + [0.12, 0.0, 0.01],
            )

            # optional recording — same debug camera as on screen (cf. interactive_robot_arm.py)
            if recording:
                now_cap = time.monotonic()
                if now_cap - last_capture >= capture_interval:
                    from PIL import Image

                    rgba = rgba_from_debug_view(p, args.width, args.height)
                    frames_pil.append(Image.fromarray(rgba[:, :, :3]))
                    last_capture = now_cap

            time.sleep(1.0 / 240.0)

    except (KeyboardInterrupt, SystemExit):
        pass

    # ── summary ─────────────────────────────────────────────────────────
    elapsed = time.time() - t_start
    print()
    print("-" * 60)
    print(f"[DONE] Steps: {step_count}   Elapsed: {elapsed:.1f}s   "
          f"Avg FPS: {step_count / elapsed if elapsed > 0 else 0:.0f}")
    print(f"[DONE] Max FK error: {max_fk_err:.6f} m")

    # Still image (debug camera) — useful for README figures
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
                print(f"[DONE] Snapshot saved → {out}")
        except Exception as exc:
            print(f"[WARN] Snapshot failed: {exc}")

    if recording and frames_pil:
        _save_gif(frames_pil, args.record, args.fps)
    elif recording and not frames_pil:
        print("[WARN] No GIF frames captured (session very short?).")

    try:
        if p.isConnected():
            p.disconnect()
    except Exception:
        pass
    print("[DONE] PyBullet disconnected.  Goodbye.")


def _save_gif(images, path, fps):
    """Save a list of PIL.Image RGB frames."""
    from PIL import Image

    if not images:
        return
    out = os.path.abspath(path)
    parent = os.path.dirname(out)
    if parent:
        os.makedirs(parent, exist_ok=True)
    images[0].save(
        out,
        format="GIF",
        save_all=True,
        append_images=images[1:],
        duration=int(1000 / fps),
        loop=0,
    )
    print(f"\nSaved {len(images)} frames → {out}")


if __name__ == "__main__":
    main()
