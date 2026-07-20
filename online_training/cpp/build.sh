#!/usr/bin/env bash
# Build the residual_core python extension (reuses Eigen from src_gym/deps —
# run src_gym/build.sh once first if deps are missing).
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
VENV_PY="$HERE/../../.venv/bin/python"

cmake -S "$HERE" -B "$HERE/build" \
      -Dpybind11_DIR="$("$VENV_PY" -m pybind11 --cmakedir)" \
      -DPYTHON_EXECUTABLE="$VENV_PY"
cmake --build "$HERE/build" -j

cp "$HERE/build/"residual_core*.so "$HERE/.."
echo "== Done: $(ls "$HERE/.."/residual_core*.so) =="
