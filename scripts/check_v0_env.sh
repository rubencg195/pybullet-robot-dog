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
echo "Python: $(command -v python3)  ($(python3 --version 2>&1))"
echo "DISPLAY: ${DISPLAY:-<unset>}"
echo

ok=0

if command -v g++ >/dev/null 2>&1; then
  echo "[OK]   g++ found: $(command -v g++)  ($(g++ --version | head -1))"
else
  echo "[FAIL] g++ not found — PyBullet usually builds from source and needs a C++ compiler."
  echo "       Fix (Ubuntu/WSL):  sudo apt-get install -y build-essential python3-dev"
  ok=1
fi

if python3 -c "import numpy" 2>/dev/null; then
  echo "[OK]   numpy: $(python3 -c 'import numpy; print(numpy.__version__)')"
else
  echo "[FAIL] numpy not installed."
  echo "       Fix:  python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt"
  ok=1
fi

if python3 -c "import pybullet" 2>/dev/null; then
  ver="$(python3 -c 'import pybullet as p; print(getattr(p, "__version__", "unknown"))')"
  echo "[OK]   pybullet import works (version: $ver)"
else
  echo "[FAIL] pybullet not importable."
  echo "       If pip install pybullet failed with:"
  echo "         error: command 'x86_64-linux-gnu-g++' failed: No such file or directory"
  echo "       install build-essential (see g++ line above), then:"
  echo "         pip install -r requirements.txt"
  ok=1
fi

echo
if [[ "$ok" -eq 0 ]]; then
  echo "All checks passed. Try:"
  echo "  python3 V0_test_stand/test_stand.py"
  exit 0
fi

echo "Fix the [FAIL] items above, then re-run this script."
exit 1
