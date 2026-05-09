#!/usr/bin/env python3
"""V0 Test Stand — interactive single-leg control with joint sliders.

Loads a SpotMicro-inspired 3-DOF leg on a fixed test stand and gives
you three GUI sliders (hip abduction, hip flexion, knee).  The foot
trail is drawn in green; a yellow skeleton overlay and joint-angle
HUD update every frame.

Usage
-----
    python V0_test_stand/test_stand.py
    python V0_test_stand/test_stand.py --record recordings/session.gif
"""

import sys
import os
import time
import argparse

import numpy as np

# Allow imports from the repo root regardless of cwd.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pybullet as p
import pybullet_data

from common.kinematics import LegConfig, forward_kinematics_full
from common.debug_visualizer import DebugVisualizer

STAND_HEIGHT = 0.35
URDF_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "urdf", "leg_test_stand.urdf"
)

JOINT_ORDER = ["hip_abduction", "hip_flexion", "knee_flexion"]


def _find_link_index(robot_id, link_name):
    for i in range(p.getNumJoints(robot_id)):
        if p.getJointInfo(robot_id, i)[12].decode() == link_name:
            return i
    return -1


def main():
    parser = argparse.ArgumentParser(
        description="SpotMicro leg test stand — interactive FK explorer"
    )
    parser.add_argument("--record", default=None, help="Save session as GIF")
    parser.add_argument("--fps", type=int, default=15, help="GIF frame rate")
    parser.add_argument("--width", type=int, default=800, help="Capture width")
    parser.add_argument("--height", type=int, default=600, help="Capture height")
    args = parser.parse_args()

    # ── PyBullet setup ──────────────────────────────────────────────────
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, -0.03, STAND_HEIGHT - 0.12],
    )
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

    p.loadURDF("plane.urdf")
    robot = p.loadURDF(URDF_PATH, [0, 0, STAND_HEIGHT], useFixedBase=True)

    # ── discover revolute joints ────────────────────────────────────────
    joints = {}
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        if info[2] == p.JOINT_REVOLUTE:
            joints[info[1].decode()] = {
                "idx": i,
                "lower": info[8],
                "upper": info[9],
            }

    sliders = {}
    for name in JOINT_ORDER:
        j = joints[name]
        sliders[name] = p.addUserDebugParameter(
            name.replace("_", " ").title(), j["lower"], j["upper"], 0.0
        )

    clear_btn = p.addUserDebugParameter("Clear Trail", 0, 1, 0)
    last_clear = p.readUserDebugParameter(clear_btn)

    # ── helpers ─────────────────────────────────────────────────────────
    cfg = LegConfig()
    viz = DebugVisualizer()
    base = np.array([0.0, 0.0, STAND_HEIGHT])
    foot_link_idx = _find_link_index(robot, "foot_link")

    frames: list[np.ndarray] = []

    print("Test stand running.  Move the sliders in the PyBullet GUI.")
    print("Press Ctrl+C or close the window to exit.\n")

    try:
        while True:
            # clear-trail button
            cv = p.readUserDebugParameter(clear_btn)
            if cv != last_clear:
                last_clear = cv
                viz.clear_trail("foot")

            # read sliders
            q1 = p.readUserDebugParameter(sliders["hip_abduction"])
            q2 = p.readUserDebugParameter(sliders["hip_flexion"])
            q3 = p.readUserDebugParameter(sliders["knee_flexion"])

            # drive joints
            for name in JOINT_ORDER:
                angle = p.readUserDebugParameter(sliders[name])
                p.setJointMotorControl2(
                    robot,
                    joints[name]["idx"],
                    p.POSITION_CONTROL,
                    targetPosition=angle,
                    force=10,
                    maxVelocity=5.0,
                )

            p.stepSimulation()

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
                fk_err = 0.0

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

            # optional recording
            if args.record:
                _, _, rgba, _, _ = p.getCameraImage(
                    args.width, args.height, renderer=p.ER_TINY_RENDERER
                )
                frames.append(
                    np.reshape(rgba, (args.height, args.width, 4))
                )

            time.sleep(1.0 / 240.0)

    except (KeyboardInterrupt, SystemExit):
        pass

    if args.record and frames:
        _save_gif(frames, args.record, args.fps)

    p.disconnect()


def _save_gif(frames, path, fps):
    try:
        from PIL import Image

        images = [Image.fromarray(f[:, :, :3]) for f in frames]
        if images:
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
            images[0].save(
                path,
                save_all=True,
                append_images=images[1:],
                duration=int(1000 / fps),
                loop=0,
            )
            print(f"\nSaved {len(images)} frames → {path}")
    except ImportError:
        print("\nInstall Pillow for GIF recording:  pip install Pillow")


if __name__ == "__main__":
    main()
