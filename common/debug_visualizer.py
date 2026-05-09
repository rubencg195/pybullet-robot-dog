"""PyBullet debug-drawing utilities for leg visualisation."""

import pybullet as p
import numpy as np


class DebugVisualizer:
    """Manages persistent trails, skeleton overlays, markers, and HUD text."""

    def __init__(self):
        self._trails: dict[str, list[tuple[list, int]]] = {}
        self._text_items: dict[str, int] = {}

    # ── trails ──────────────────────────────────────────────────────────

    def add_trail_point(self, position, trail_name="default",
                        color=(0, 1, 0), width=2, max_points=2000):
        """Append a point; draw a line segment from the previous point."""
        trail = self._trails.setdefault(trail_name, [])
        pos = list(position)

        if trail:
            prev = trail[-1][0]
            if np.linalg.norm(np.array(pos) - np.array(prev)) > 1e-4:
                lid = p.addUserDebugLine(prev, pos, list(color), width,
                                         lifeTime=0)
                trail.append((pos, lid))
        else:
            trail.append((pos, -1))

        if len(trail) > max_points:
            excess = len(trail) - max_points
            for _, lid in trail[:excess]:
                if lid >= 0:
                    p.removeUserDebugItem(lid)
            self._trails[trail_name] = trail[excess:]

    def clear_trail(self, trail_name="default"):
        for _, lid in self._trails.get(trail_name, []):
            if lid >= 0:
                p.removeUserDebugItem(lid)
        self._trails[trail_name] = []

    def clear_all_trails(self):
        for name in list(self._trails):
            self.clear_trail(name)

    # ── instantaneous markers ───────────────────────────────────────────

    def draw_point(self, position, color=(1, 0, 0), size=0.008,
                   lifetime=0.05):
        """Small 3-axis cross at *position*."""
        pos = np.array(position)
        for axis in (np.array([1, 0, 0]),
                     np.array([0, 1, 0]),
                     np.array([0, 0, 1])):
            p.addUserDebugLine((pos - size * axis).tolist(),
                               (pos + size * axis).tolist(),
                               list(color), 2, lifeTime=lifetime)

    def draw_sphere_marker(self, position, color=(1, 0, 0), radius=0.006,
                           lifetime=0.05):
        """Visual-only sphere via a tiny debug line (cheaper than shapes)."""
        self.draw_point(position, color, size=radius, lifetime=lifetime)

    def draw_leg_skeleton(self, joint_positions, color=(1, 1, 0), width=3,
                          lifetime=0.05):
        """Lines connecting sequential joint world positions."""
        for i in range(len(joint_positions) - 1):
            p.addUserDebugLine(list(joint_positions[i]),
                               list(joint_positions[i + 1]),
                               list(color), width, lifeTime=lifetime)

    def draw_coordinate_frame(self, position, size=0.03, lifetime=0.05):
        """RGB axes at *position* (X=red, Y=green, Z=blue)."""
        pos = list(position)
        for i, col in enumerate(([1, 0, 0], [0, 1, 0], [0, 0, 1])):
            end = list(position)
            end[i] += size
            p.addUserDebugLine(pos, end, col, 2, lifeTime=lifetime)

    # ── HUD text ────────────────────────────────────────────────────────

    def update_text(self, text, key, position, color=(1, 1, 1), size=1.0):
        """Create or replace debug text identified by *key*."""
        pos = list(position)
        col = list(color)
        if key in self._text_items:
            new_id = p.addUserDebugText(
                text, pos, textColorRGB=col, textSize=size,
                replaceItemUniqueId=self._text_items[key])
        else:
            new_id = p.addUserDebugText(text, pos, textColorRGB=col,
                                        textSize=size)
        self._text_items[key] = new_id

    # ── path geometry ───────────────────────────────────────────────────

    def draw_path(self, points, color=(1, 0.5, 0), width=1.5, lifetime=0,
                  closed=True):
        """Draw a polyline (or closed loop) through *points*."""
        ids = []
        n = len(points)
        for i in range(n):
            j = (i + 1) % n if closed else i + 1
            if j >= n:
                break
            lid = p.addUserDebugLine(list(points[i]), list(points[j]),
                                     list(color), width, lifeTime=lifetime)
            ids.append(lid)
        return ids
