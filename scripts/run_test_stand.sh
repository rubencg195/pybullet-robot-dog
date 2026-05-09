#!/usr/bin/env bash
# Run V0 test stand with a Python that has PyBullet (Miniconda or PY_ROBOT_DOG).
# Usage: bash scripts/run_test_stand.sh  [-- args passed to test_stand.py]

set +H
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if [[ -n "${PY_ROBOT_DOG:-}" ]]; then
  PY="$PY_ROBOT_DOG"
elif [[ -x "$HOME/miniconda3/bin/python" ]] && "$HOME/miniconda3/bin/python" -c "import pybullet" 2>/dev/null; then
  PY="$HOME/miniconda3/bin/python"
else
  PY="$(command -v python3)"
fi

exec "$PY" -u V0_test_stand/test_stand.py "$@"
