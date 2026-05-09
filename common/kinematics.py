"""
Forward and inverse kinematics for a 3-DOF SpotMicro-style leg.

Coordinate system (hip-centric, right-hand rule):
    X: forward
    Y: lateral (positive = left)
    Z: vertical (positive = up)

Joint chain:
    1. Hip abduction  (q1) — rotates around X axis
    2. Hip flexion     (q2) — rotates around Y axis (after L1 lateral offset)
    3. Knee flexion    (q3) — rotates around Y axis

At zero angles the leg hangs straight down:
    foot = (0, side_sign * L1, -(L2 + L3))
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class LegConfig:
    """SpotMicro-inspired leg dimensions and side selection."""
    L1: float = 0.055   # shoulder lateral offset  [m]
    L2: float = 0.107   # upper leg (femur) length [m]
    L3: float = 0.130   # lower leg (tibia) length [m]
    side_sign: float = -1.0  # -1 = right leg, +1 = left leg


def forward_kinematics(q1: float, q2: float, q3: float,
                       cfg: LegConfig | None = None) -> np.ndarray:
    """Return foot position [x, y, z] in the hip frame."""
    if cfg is None:
        cfg = LegConfig()

    s1, c1 = np.sin(q1), np.cos(q1)
    s2, c2 = np.sin(q2), np.cos(q2)
    s23 = np.sin(q2 + q3)
    c23 = np.cos(q2 + q3)

    # Sagittal-plane reach (vertical component before abduction rotation)
    D = cfg.L2 * c2 + cfg.L3 * c23

    x = -cfg.L2 * s2 - cfg.L3 * s23
    y = cfg.side_sign * cfg.L1 * c1 + D * s1
    z = cfg.side_sign * cfg.L1 * s1 - D * c1

    return np.array([x, y, z])


def forward_kinematics_full(q1: float, q2: float, q3: float,
                            cfg: LegConfig | None = None):
    """Return positions of all four key points: hip, shoulder, knee, foot.

    Each is an [x, y, z] array in the hip frame.  Useful for drawing the
    leg skeleton and placing joint markers.
    """
    if cfg is None:
        cfg = LegConfig()

    s1, c1 = np.sin(q1), np.cos(q1)
    s2, c2 = np.sin(q2), np.cos(q2)
    s23 = np.sin(q2 + q3)
    c23 = np.cos(q2 + q3)

    hip = np.array([0.0, 0.0, 0.0])

    shoulder = np.array([
        0.0,
        cfg.side_sign * cfg.L1 * c1,
        cfg.side_sign * cfg.L1 * s1,
    ])

    D_knee = cfg.L2 * c2
    knee = np.array([
        -cfg.L2 * s2,
        cfg.side_sign * cfg.L1 * c1 + D_knee * s1,
        cfg.side_sign * cfg.L1 * s1 - D_knee * c1,
    ])

    D_foot = cfg.L2 * c2 + cfg.L3 * c23
    foot = np.array([
        -cfg.L2 * s2 - cfg.L3 * s23,
        cfg.side_sign * cfg.L1 * c1 + D_foot * s1,
        cfg.side_sign * cfg.L1 * s1 - D_foot * c1,
    ])

    return hip, shoulder, knee, foot


def inverse_kinematics(target, cfg: LegConfig | None = None,
                       knee_sign: float = -1.0):
    """Solve for joint angles (q1, q2, q3) that place the foot at *target*.

    Parameters
    ----------
    target : array-like, shape (3,)
        Desired foot [x, y, z] in the hip frame.
    cfg : LegConfig, optional
    knee_sign : float
        -1.0 for natural backward-bent knee, +1.0 for forward knee.

    Returns
    -------
    np.ndarray of shape (3,) — [q1, q2, q3] in radians, or None if
    the target is outside the reachable workspace.
    """
    if cfg is None:
        cfg = LegConfig()

    px, py, pz = float(target[0]), float(target[1]), float(target[2])

    # --- Knee angle (q3) via law of cosines in the leg sagittal plane ------
    r_sq = px * px + py * py + pz * pz
    d_sq = r_sq - cfg.L1 ** 2
    if d_sq < 0:
        return None  # inside the shoulder-offset sphere

    cos_q3 = (d_sq - cfg.L2 ** 2 - cfg.L3 ** 2) / (2.0 * cfg.L2 * cfg.L3)
    if abs(cos_q3) > 1.0 + 1e-6:
        return None  # out of reach
    cos_q3 = np.clip(cos_q3, -1.0, 1.0)
    q3 = knee_sign * np.arccos(cos_q3)

    # --- Hip flexion (q2) --------------------------------------------------
    D = np.sqrt(max(d_sq, 0.0))  # positive magnitude
    A = cfg.L2 + cfg.L3 * np.cos(q3)
    B = cfg.L3 * np.sin(q3)
    q2 = np.arctan2(-px, D) - np.arctan2(B, A)

    # --- Hip abduction (q1) ------------------------------------------------
    d_yz_sq = py * py + pz * pz
    if d_yz_sq < cfg.L1 ** 2 - 1e-8:
        return None
    q1 = (np.arctan2(py, -pz)
          - np.arctan2(cfg.side_sign * cfg.L1,
                       np.sqrt(max(d_yz_sq - cfg.L1 ** 2, 0.0))))

    return np.array([q1, q2, q3])
