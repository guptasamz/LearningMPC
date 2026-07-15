#!/usr/bin/env bash
# Build the lmpc_core python extension.
# Installs Eigen 3.4.0, OSQP v0.6.3 and osqp-eigen v0.8.1 into ./deps
# (one-time), then builds cpp/lmpc_core.cpp against the venv's pybind11.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
DEPS="$HERE/deps"
SRC="$DEPS/src"
VENV_PY="$HERE/../.venv/bin/python"

mkdir -p "$SRC"

if [ ! -f "$DEPS/share/eigen3/cmake/Eigen3Config.cmake" ]; then
  echo "== Installing Eigen 3.4.0 =="
  [ -d "$SRC/eigen" ] || git clone --depth 1 --branch 3.4.0 https://gitlab.com/libeigen/eigen.git "$SRC/eigen"
  cmake -S "$SRC/eigen" -B "$SRC/eigen/build" -DCMAKE_INSTALL_PREFIX="$DEPS" \
        -DEIGEN_BUILD_DOC=OFF -DBUILD_TESTING=OFF -DEIGEN_BUILD_PKGCONFIG=OFF > /dev/null
  cmake --install "$SRC/eigen/build" > /dev/null
fi

if [ ! -d "$DEPS/lib/cmake/osqp" ]; then
  echo "== Installing OSQP v0.6.3 =="
  [ -d "$SRC/osqp" ] || git clone --depth 1 --branch v0.6.3 --recursive https://github.com/osqp/osqp.git "$SRC/osqp"
  cmake -S "$SRC/osqp" -B "$SRC/osqp/build" -DCMAKE_INSTALL_PREFIX="$DEPS" \
        -DCMAKE_BUILD_TYPE=Release -DUNITTESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON > /dev/null
  cmake --build "$SRC/osqp/build" -j > /dev/null
  cmake --install "$SRC/osqp/build" > /dev/null
fi

if [ ! -d "$DEPS/lib/cmake/OsqpEigen" ]; then
  echo "== Installing osqp-eigen v0.8.1 =="
  [ -d "$SRC/osqp-eigen" ] || git clone --depth 1 --branch v0.8.1 https://github.com/robotology/osqp-eigen.git "$SRC/osqp-eigen"
  cmake -S "$SRC/osqp-eigen" -B "$SRC/osqp-eigen/build" -DCMAKE_INSTALL_PREFIX="$DEPS" \
        -DCMAKE_PREFIX_PATH="$DEPS" -DCMAKE_BUILD_TYPE=Release \
        -DOSQP_EIGEN_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON > /dev/null
  cmake --build "$SRC/osqp-eigen/build" -j > /dev/null
  cmake --install "$SRC/osqp-eigen/build" > /dev/null
fi

echo "== Building lmpc_core =="
cmake -S "$HERE" -B "$HERE/build" \
      -Dpybind11_DIR="$("$VENV_PY" -m pybind11 --cmakedir)" \
      -DPYTHON_EXECUTABLE="$VENV_PY"
cmake --build "$HERE/build" -j

cp "$HERE/build/"lmpc_core*.so "$HERE/"
echo "== Done: $(ls "$HERE"/lmpc_core*.so) =="
