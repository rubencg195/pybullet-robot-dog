"""Capture the PyBullet *debug* (GUI) camera — matches what you see on screen.

See aws-pybullet-environment ``interactive_robot_arm.py`` for the same pattern:
https://github.com/rubencg195/aws-pybullet-environment/blob/main/scripts/interactive_robot_arm.py
"""

from __future__ import annotations

import numpy as np


def rgba_from_debug_view(p, width: int, height: int, *, fov: float = 60.0,
                         near: float = 0.1, far: float = 100.0) -> np.ndarray:
    """Return H×W×4 RGBA uint8 using the current debug-visualizer camera."""
    cam = p.getDebugVisualizerCamera()
    view = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=cam[11],
        distance=cam[10],
        yaw=cam[8],
        pitch=cam[9],
        roll=0,
        upAxisIndex=2,
    )
    proj = p.computeProjectionMatrixFOV(
        fov=fov,
        aspect=float(width) / float(height),
        nearVal=near,
        farVal=far,
    )
    _, _, rgba, _, _ = p.getCameraImage(
        width,
        height,
        viewMatrix=view,
        projectionMatrix=proj,
        renderer=p.ER_TINY_RENDERER,
    )
    return np.asarray(rgba, dtype=np.uint8).reshape((height, width, 4))
