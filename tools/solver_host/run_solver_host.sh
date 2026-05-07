#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
OUT_DIR="$ROOT_DIR/build/solver_host"
OUT_BIN="$OUT_DIR/solver_host"

mkdir -p "$OUT_DIR"
cc -std=c11 -Wall -Wextra -Wpedantic -Wno-strict-prototypes \
  -I"$ROOT_DIR/tools/solver_host/include" \
  -I"$ROOT_DIR/platform/stm32f405/Core/Inc" \
  -I"$ROOT_DIR/params/f413_preorder" \
  "$ROOT_DIR/tools/solver_host/solver_host.c" \
  "$ROOT_DIR/platform/stm32f405/Core/Src/solver.c" \
  "$ROOT_DIR/platform/stm32f405/Core/Src/path.c" \
  "$ROOT_DIR/platform/stm32f405/Core/Src/maze_grid.c" \
  "$ROOT_DIR/platform/stm32f405/Core/Src/solver_params.c" \
  "$ROOT_DIR/params/f413_preorder/shortest_run_params_split.c" \
  -lm -o "$OUT_BIN"
"$OUT_BIN" "$@"
