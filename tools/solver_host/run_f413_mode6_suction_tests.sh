#!/usr/bin/env sh
set -eu
TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
TEST_OUT_DIR="$TASK_ROOT/build/solver_host"
mkdir -p "$TEST_OUT_DIR"
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -Wno-unused-function -O1 -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I"$TASK_ROOT/common/route" -I"$TASK_ROOT/params/mini_r3_0" \
  -I"$TASK_ROOT/platform/stm32f405/Core/Inc" \
  -I"$TASK_ROOT/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc" \
  "$TASK_ROOT/tools/solver_host/f413_mode6_suction_tests.c" \
  "$TASK_ROOT/common/route/legacy_path_codec.c" \
  "$TASK_ROOT/common/route/motion_time.c" \
  "$TASK_ROOT/params/mini_r3_0/shortest_run_params_split.c" \
  -lm -o "$TEST_OUT_DIR/f413_mode6_suction_tests"
ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
  "$TEST_OUT_DIR/f413_mode6_suction_tests"
