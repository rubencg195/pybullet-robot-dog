#!/usr/bin/env bash
set +H
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
if [[ -n "${PY_ROBOT_DOG:-}" ]]; then PY="$PY_ROBOT_DOG"
elif [[ -x "$HOME/miniconda3/bin/python" ]] && "$HOME/miniconda3/bin/python" -c "import pybullet" 2>/dev/null; then
  PY="$HOME/miniconda3/bin/python"
else PY="$(command -v python3)"; fi
exec "$PY" -u V1_test_stand/ik_demo.py "$@"
