#!/usr/bin/env sh
set -eu
TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
TEST_OUT_DIR="$TASK_ROOT/build/solver_host"
mkdir -p "$TEST_OUT_DIR"
# Includes the full mode3..7 ladder; retained name for existing callers.
for PROFILE in mini_r3_0 f413_preorder; do
  LEGACY_PROFILE=0
  if [ "$PROFILE" = f413_preorder ]; then LEGACY_PROFILE=1; fi
  for TEST_MODE in 3 4 5 6 7; do
    "${CC:-cc}" -DTEST_MODE="$TEST_MODE" -DLEGACY_PROFILE="$LEGACY_PROFILE" -std=c11 -Wall -Wextra -Werror -Wno-unused-function -O1 -g \
      -fsanitize=address,undefined -fno-omit-frame-pointer \
      -I"$TASK_ROOT/common/route" -I"$TASK_ROOT/params/$PROFILE" \
      -I"$TASK_ROOT/platform/stm32f405/Core/Inc" \
      -I"$TASK_ROOT/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc" \
      "$TASK_ROOT/tools/solver_host/f413_mode6_suction_tests.c" \
      "$TASK_ROOT/common/route/legacy_path_codec.c" \
      "$TASK_ROOT/common/route/motion_time.c" \
      "$TASK_ROOT/params/$PROFILE/shortest_run_params_split.c" \
      -lm -o "$TEST_OUT_DIR/f413_mode${TEST_MODE}_${PROFILE}_suction_tests"
    ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
      "$TEST_OUT_DIR/f413_mode${TEST_MODE}_${PROFILE}_suction_tests"
  done
done
