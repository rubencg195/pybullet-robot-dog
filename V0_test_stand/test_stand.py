#!/usr/bin/env python3
"""Single SpotMicro-style leg on a fixed stand with four-bar knee drive.

The knee is driven by a crank+coupler four-bar linkage (parallel link style).
PyBullet's Params panel drives hip_abduction, hip_flexion, and knee_drive.
The knee_passive joint is constrained by a point-to-point loop closure.

Default camera preset ``stand`` is the side/profile rig view; use ``coronal``
or ``iso`` for other angles.  ``--record`` / ``--snapshot`` grab the same
view as the GUI.

Examples::

    python V0_test_stand/test_stand.py
    python V0_test_stand/test_stand.py --camera iso --record out.gif --fps 15
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

JOINT_ORDER = ["hip_abduction", "hip_flexion", "knee_drive"]
LOG_INTERVAL = 120

CAM_STAND = dict(
    cameraDistance=0.45,
    cameraYaw=-90.0,
    cameraPitch=-20.0,
    cameraTargetPosition=[0.0, 0.0, STAND_HEIGHT - 0.04],
)
CAM_ISO = dict(
    cameraDistance=0.42,
    cameraYaw=45.0,
    cameraPitch=-30.0,
    cameraTargetPosition=[0.0, -0.03, STAND_HEIGHT - 0.08],
)
CAM_CORONAL = dict(
    cameraDistance=0.45,
    cameraYaw=0.0,
    cameraPitch=-20.0,
    cameraTargetPosition=[0.0, 0.0, STAND_HEIGHT - 0.04],
)

CAM_PRESETS = {"stand": CAM_STAND, "iso": CAM_ISO, "coronal": CAM_CORONAL}

# Four-bar geometry (metres) — matches URDF
GROUND_LINK = 0.033
CRANK_LEN = 0.029
FEMUR_LEN = 0.060
COUPLER_LEN = 0.060


def _find_link_index(robot_id, link_name):
    for i in range(p.getNumJoints(robot_id)):
        if p.getJointInfo(robot_id, i)[12].decode() == link_name:
            return i
    return -1


def _create_fourbar_constraint(robot):
    """Close the four-bar loop: end of coupler_link → knee pivot on tibia_link."""
    coupler_idx = _find_link_index(robot, "coupler_link")
    tibia_idx = _find_link_index(robot, "tibia_link")

    if coupler_idx < 0 or tibia_idx < 0:
        print("[WARN] Could not find coupler_link or tibia_link for constraint")
        return None

    cid = p.createConstraint(
        parentBodyUniqueId=robot,
        parentLinkIndex=coupler_idx,
        childBodyUniqueId=robot,
        childLinkIndex=tibia_idx,
        jointType=p.JOINT_POINT2POINT,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, -0.060],
        childFramePosition=[0, 0, 0],
    )
    p.changeConstraint(cid, maxForce=500)
    return cid


def main():
    parser = argparse.ArgumentParser(
        description="SpotMicro leg test stand — four-bar knee drive"
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
    parser.add_argument(
        "--urdf",
        default=None,
        metavar="PATH",
        help="Alternate leg URDF (default: V0 four-bar).",
    )
    args = parser.parse_args()

    urdf_path = os.path.abspath(args.urdf) if args.urdf else URDF_PATH

    if args.record or args.snapshot:
        try:
            from PIL import Image  # noqa: F401
        except ImportError:
            print("Install Pillow for --record / --snapshot: pip install Pillow")
            sys.exit(1)

    label = "V1 CAD (mesh)" if "V1_test_stand" in urdf_path else "V0 four-bar"
    print("=" * 60)
    print(f"  TEST STAND — {label}")
    print("=" * 60)
    print(f"  URDF path : {urdf_path}")
    print(f"  Stand height : {STAND_HEIGHT} m")
    print(f"  URDF exists  : {os.path.isfile(urdf_path)}")
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

    print(f"[INIT] Loading leg URDF from {urdf_path} ...")
    robot = p.loadURDF(urdf_path, [0, 0, STAND_HEIGHT], useFixedBase=True)
    print(f"[INIT] Leg loaded  (body id = {robot})")

    # ── discover joints ──────────────────────────────────────────────────
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

    # ── create four-bar loop closure constraint ─────────────────────────
    print("[INIT] Creating four-bar loop closure constraint...")
    fourbar_cid = _create_fourbar_constraint(robot)
    if fourbar_cid is not None:
        print(f"[INIT] Four-bar constraint created (id={fourbar_cid})")
    else:
        print("[WARN] Four-bar constraint NOT created — mechanism may be unconstrained")

    # ── GUI sliders ─────────────────────────────────────────────────────
    sliders = {}
    for name in JOINT_ORDER:
        if name not in joints:
            print(f"[WARN] Joint '{name}' not found in URDF — skipping slider")
            continue
        j = joints[name]
        lo_deg = float(np.degrees(j["lower"]))
        hi_deg = float(np.degrees(j["upper"]))
        label = name.replace("_", " ").title() + " (deg)"
        sliders[name] = p.addUserDebugParameter(label, lo_deg, hi_deg, 0.0)
    print(f"[INIT] GUI sliders (degrees) for: {list(sliders.keys())}")

    clear_btn = p.addUserDebugParameter("Clear Trail", 0, 1, 0)
    last_clear = p.readUserDebugParameter(clear_btn)

    # ── helpers ─────────────────────────────────────────────────────────
    cfg = LegConfig()
    print(f"[INIT] LegConfig: L1={cfg.L1}m  L2={cfg.L2}m  L3={cfg.L3}m  "
          f"side_sign={cfg.side_sign}")

    viz = DebugVisualizer()
    base = np.array([0.0, 0.0, STAND_HEIGHT])
    foot_link_idx = _find_link_index(robot, "foot_link")
    tibia_link_idx = _find_link_index(robot, "tibia_link")
    crank_link_idx = _find_link_index(robot, "crank_link")
    coupler_link_idx = _find_link_index(robot, "coupler_link")
    femur_link_idx = _find_link_index(robot, "femur_link")
    print(f"[INIT] Link indices: foot={foot_link_idx} tibia={tibia_link_idx} "
          f"femur={femur_link_idx} crank={crank_link_idx} coupler={coupler_link_idx}")

    if foot_link_idx >= 0:
        init_state = p.getLinkState(robot, foot_link_idx)
        print(f"[INIT] Initial foot world pos (PyBullet): "
              f"({init_state[0][0]:.4f}, {init_state[0][1]:.4f}, {init_state[0][2]:.4f})")

    hip0, sh0, kn0, ft0 = forward_kinematics_full(0, 0, 0, cfg)
    print(f"[INIT] FK at zero angles — foot (hip frame): "
          f"({ft0[0]:+.4f}, {ft0[1]:+.4f}, {ft0[2]:+.4f})")
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
            q_vals = {}
            for name in JOINT_ORDER:
                if name not in sliders:
                    continue
                lo, hi = joints[name]["lower"], joints[name]["upper"]
                rad = np.radians(p.readUserDebugParameter(sliders[name]))
                q_vals[name] = float(np.clip(rad, lo, hi))

            # drive active joints (hip_abduction, hip_flexion, knee_drive)
            for name, q in q_vals.items():
                p.setJointMotorControl2(
                    robot,
                    joints[name]["idx"],
                    p.POSITION_CONTROL,
                    targetPosition=q,
                    force=12,
                    maxVelocity=5.0,
                )

            # disable motor on passive joints so constraint drives them
            if "knee_passive" in joints:
                p.setJointMotorControl2(
                    robot, joints["knee_passive"]["idx"],
                    p.VELOCITY_CONTROL, targetVelocity=0, force=0)
            if "coupler_passive" in joints:
                p.setJointMotorControl2(
                    robot, joints["coupler_passive"]["idx"],
                    p.VELOCITY_CONTROL, targetVelocity=0, force=0)

            p.stepSimulation()
            step_count += 1

            # get world positions of key links for visualisation
            positions = {}
            for name, idx in [("foot", foot_link_idx), ("tibia", tibia_link_idx),
                              ("femur", femur_link_idx), ("crank", crank_link_idx),
                              ("coupler", coupler_link_idx)]:
                if idx >= 0:
                    positions[name] = np.array(p.getLinkState(robot, idx)[0])

            foot_w = positions.get("foot", base)
            knee_w = positions.get("tibia", base)
            femur_w = positions.get("femur", base)

            # hip/shoulder world position (base of femur)
            hip_w = base.copy()
            shoulder_w = base + np.array([0, -0.055, 0])
            if "hip_abduction" in joints:
                sh_state = p.getLinkState(robot, joints["hip_abduction"]["idx"])
                shoulder_w = np.array(sh_state[0])

            # get actual joint states for FK comparison
            q1 = p.getJointState(robot, joints["hip_abduction"]["idx"])[0] if "hip_abduction" in joints else 0
            q2 = p.getJointState(robot, joints["hip_flexion"]["idx"])[0] if "hip_flexion" in joints else 0
            q3_passive = p.getJointState(robot, joints["knee_passive"]["idx"])[0] if "knee_passive" in joints else 0

            # FK using our math (effective q3 is the passive knee angle)
            _, _, _, foot_fk = forward_kinematics_full(q1, q2, q3_passive, cfg)
            foot_fk_w = foot_fk + base

            fk_err = np.linalg.norm(foot_w - foot_fk_w)
            max_fk_err = max(max_fk_err, fk_err)

            # periodic terminal log
            if step_count % LOG_INTERVAL == 0:
                elapsed = time.time() - t_start
                fps_actual = step_count / elapsed if elapsed > 0 else 0
                q_knee_drive = q_vals.get("knee_drive", 0)
                print(
                    f"[STEP {step_count:>6d}]  "
                    f"q=({np.degrees(q1):+6.1f}, {np.degrees(q2):+6.1f}, "
                    f"knee_drv={np.degrees(q_knee_drive):+6.1f}, "
                    f"knee_eff={np.degrees(q3_passive):+6.1f}) deg  "
                    f"foot=({foot_w[0]:+.4f}, {foot_w[1]:+.4f}, {foot_w[2]:+.4f})  "
                    f"fk_err={fk_err:.6f}m  fps={fps_actual:.0f}"
                )

            # visualise main chain
            viz.draw_leg_skeleton([hip_w, shoulder_w, knee_w, foot_w])
            viz.add_trail_point(foot_w, "foot", color=(0, 1, 0))

            # visualise four-bar linkage
            if "crank" in positions and "coupler" in positions:
                crank_pivot_w = shoulder_w + np.array([-0.033, 0, 0])
                crank_end_w = positions["crank"]
                coupler_end_w = positions["coupler"]
                # draw crank and coupler
                viz.draw_leg_skeleton(
                    [crank_pivot_w, crank_end_w],
                    color=(0.95, 0.55, 0.1), width=2, lifetime=0.05)
                viz.draw_leg_skeleton(
                    [crank_end_w, coupler_end_w],
                    color=(0.25, 0.65, 0.35), width=2, lifetime=0.05)
                # draw closing link (coupler end → knee)
                viz.draw_leg_skeleton(
                    [coupler_end_w, knee_w],
                    color=(0.5, 0.5, 0.5), width=1, lifetime=0.05)

            # joint markers
            for pos, c in [
                (hip_w, (1, 0.3, 0.3)),
                (shoulder_w, (0.3, 0.3, 1)),
                (knee_w, (1, 0.7, 0)),
                (foot_w, (0, 1, 0)),
            ]:
                viz.draw_point(pos, c, size=0.005)

            viz.draw_coordinate_frame(hip_w, size=0.020)

            # HUD
            q_knee_drive = q_vals.get("knee_drive", 0)
            viz.update_text(
                f"q1={np.degrees(q1):+6.1f}\u00b0  "
                f"q2={np.degrees(q2):+6.1f}\u00b0  "
                f"knee_drv={np.degrees(q_knee_drive):+6.1f}\u00b0  "
                f"knee_eff={np.degrees(q3_passive):+6.1f}\u00b0",
                "angles",
                base + [0.10, 0.0, 0.06],
            )
            viz.update_text(
                f"Foot: x={foot_w[0]:+.4f}  y={foot_w[1]:+.4f}  z={foot_w[2]:+.4f} m",
                "foot_pos",
                base + [0.10, 0.0, 0.03],
            )
            viz.update_text(
                f"FK err: {fk_err:.5f} m    Four-bar active",
                "reach",
                base + [0.10, 0.0, 0.00],
            )

            # optional recording
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
