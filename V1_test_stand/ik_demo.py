#!/usr/bin/env python3
"""IK path demo using the V1 mesh URDF (same as V0 once meshes exist)."""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parent
_V0_IK = _REPO / "V0_test_stand" / "ik_demo.py"
_URDF = _HERE / "urdf" / "leg_test_stand_cad.urdf"
_MESH_DIR = _HERE / "meshes"

_REQUIRED = (
    "base_plate.stl",
    "shoulder_link.stl",
    "upper_leg.stl",
    "lower_leg.stl",
    "foot.stl",
)


def main() -> None:
    missing = [n for n in _REQUIRED if not (_MESH_DIR / n).is_file()]
    if missing:
        print(
            "V1 CAD meshes missing — same list as test_stand. Use V0 ik_demo for now:\n"
            "  bash scripts/run_ik_demo.sh\n",
            file=sys.stderr,
        )
        sys.exit(1)

    saved = sys.argv[:]
    sys.argv = [str(_V0_IK), "--urdf", str(_URDF)] + saved[1:]
    try:
        runpy.run_path(str(_V0_IK), run_name="__main__")
    finally:
        sys.argv = saved


if __name__ == "__main__":
    main()
