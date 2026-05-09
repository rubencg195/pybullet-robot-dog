#!/usr/bin/env bash
# Diagnose why V0 PyBullet sim may not start. Run from repo root:
#   bash scripts/check_v0_env.sh

set +H
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

echo "=============================================="
echo "  V0 environment check (pybullet-robot-dog)"
echo "=============================================="
echo "Repo: $ROOT"
echo "DISPLAY: ${DISPLAY:-<unset>}"
echo

# Prefer explicit override, then Miniconda (common no-sudo install), then system python3.
CANDIDATES=()
if [[ -n "${PY_ROBOT_DOG:-}" ]]; then
  CANDIDATES+=("$PY_ROBOT_DOG")
fi
if [[ -x "$HOME/miniconda3/bin/python" ]]; then
  CANDIDATES+=("$HOME/miniconda3/bin/python")
fi
if command -v python3 >/dev/null 2>&1; then
  CANDIDATES+=("$(command -v python3)")
fi

GOOD_PY=""
for try in "${CANDIDATES[@]}"; do
  [[ -n "$try" ]] || continue
  [[ -x "$try" ]] || continue
  if "$try" -c "import numpy, pybullet" 2>/dev/null; then
    GOOD_PY="$try"
    break
  fi
done

ok=0

if [[ -n "$GOOD_PY" ]]; then
  echo "[OK]   Python with numpy + pybullet: $GOOD_PY"
  echo "       $("$GOOD_PY" --version 2>&1)"
  echo "       numpy $($GOOD_PY -c 'import numpy; print(numpy.__version__)')"
else
  echo "[FAIL] No candidate interpreter could import both numpy and pybullet."
  echo "       Tried: ${CANDIDATES[*]}"
  ok=1
fi

if command -v g++ >/dev/null 2>&1; then
  echo "[OK]   g++ found: $(command -v g++)  ($(g++ --version | head -1))"
elif [[ -n "$GOOD_PY" ]] && [[ "$GOOD_PY" == *miniconda3* ]]; then
  echo "[INFO] g++ not installed — not required for this Miniconda PyBullet (prebuilt binary)."
else
  echo "[WARN] g++ not found — pip install pybullet usually needs build-essential."
  echo "       Fix: sudo apt-get install -y build-essential python3-dev"
  echo "       Or install Miniconda and: conda install -y -c conda-forge pybullet numpy pillow"
fi

if [[ -z "$GOOD_PY" ]]; then
  if ! python3 -c "import numpy" 2>/dev/null; then
    echo "[FAIL] System python3: numpy missing."
  fi
  if ! python3 -c "import pybullet" 2>/dev/null; then
    echo "[FAIL] System python3: pybullet missing."
  fi
fi

echo
if [[ "$ok" -eq 0 ]]; then
  echo "All import checks passed. Run the sim (unbuffered logs):"
  echo "  PY_ROBOT_DOG=\"$GOOD_PY\" bash scripts/run_test_stand.sh"
  echo "  # or:  \"$GOOD_PY\" -u V0_test_stand/test_stand.py"
  exit 0
fi

echo "Fix the [FAIL] items above (or install Miniconda + conda-forge pybullet), then re-run."
exit 1
