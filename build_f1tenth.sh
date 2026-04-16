#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(readlink -f "$(dirname "$0")")"
cd "$SCRIPT_DIR"

echo "[INFO] Building f1tenth workspace"

colcon build \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

echo "[INFO] Sourcing workspace"
source install/setup.bash

echo "[INFO] Build completed successfully"