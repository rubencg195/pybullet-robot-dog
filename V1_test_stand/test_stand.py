#!/usr/bin/env python3
"""Same interactive test stand as V0, but loads the mesh URDF in urdf/.

You need five STLs under meshes/ (see urdf/leg_test_stand_cad.urdf). Until they
exist, this exits with a short checklist — use V0 for day-to-day sim.
"""

from __future__ import annotations

import runpy
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parent
_V0_SCRIPT = _REPO / "V0_test_stand" / "test_stand.py"
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
            "V1 CAD meshes are not in the repo yet. Drop these into "
            f"{_MESH_DIR.relative_to(_REPO)}/ :\n\n"
            + "\n".join(f"  • {m}" for m in missing)
            + "\n\nJoint names and origins match V0 so `common/kinematics.py` "
            "still applies if your link frames line up. Until the STLs exist, run:\n"
            "  bash scripts/run_test_stand.sh\n",
            file=sys.stderr,
        )
        sys.exit(1)

    saved = sys.argv[:]
    sys.argv = [str(_V0_SCRIPT), "--urdf", str(_URDF)] + saved[1:]
    try:
        runpy.run_path(str(_V0_SCRIPT), run_name="__main__")
    finally:
        sys.argv = saved


if __name__ == "__main__":
    main()
