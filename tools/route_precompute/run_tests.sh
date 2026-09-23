#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
OUT_DIR="$ROOT_DIR/build/route_precompute"
OUT_BIN="$OUT_DIR/verify_f413_route_motion_table"
CC_BIN=${CC:-cc}

python3 "$ROOT_DIR/tools/route_precompute/generate.py" --check
mkdir -p "$OUT_DIR"
"$CC_BIN" -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic \
  -I"$ROOT_DIR/common/route" \
  -I"$ROOT_DIR/platform/stm32f405/Core/Inc" \
  -I"$ROOT_DIR/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc" \
  -I"$ROOT_DIR/params/f413_preorder" \
  -I"$ROOT_DIR/tools/route_precompute" \
  "$ROOT_DIR/tools/route_precompute/verify_f413_route_motion_table.c" \
  "$ROOT_DIR/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_route_motion_table.c" \
  "$ROOT_DIR/common/route/motion_time.c" \
  "$ROOT_DIR/params/f413_preorder/shortest_run_params_split.c" \
  -lm -o "$OUT_BIN"
"$OUT_BIN"
