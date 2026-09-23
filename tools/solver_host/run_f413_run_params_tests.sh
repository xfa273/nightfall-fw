#!/usr/bin/env sh
set -eu
TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
TEST_OUT_DIR="$TASK_ROOT/build/solver_host"
mkdir -p "$TEST_OUT_DIR"
for PROFILE in f413_preorder mini_r3_0; do
  "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -Wno-unused-function -O1 -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$TASK_ROOT/common/route" -I"$TASK_ROOT/params/$PROFILE" \
    -I"$TASK_ROOT/platform/stm32f405/Core/Inc" \
    -I"$TASK_ROOT/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc" \
    "$TASK_ROOT/tools/solver_host/f413_run_params_tests.c" \
    "$TASK_ROOT/common/route/legacy_path_codec.c" "$TASK_ROOT/common/route/motion_time.c" \
    "$TASK_ROOT/params/$PROFILE/shortest_run_params_split.c" \
    -lm -o "$TEST_OUT_DIR/f413_run_params_$PROFILE"
  ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
    "$TEST_OUT_DIR/f413_run_params_$PROFILE"
  "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -O1 -g -DSTM32F413xx \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -I"$TASK_ROOT/tools/hil/control_stubs" -I"$TASK_ROOT/tools/hil/nvm_stubs" \
    -I"$TASK_ROOT/params/$PROFILE" -I"$TASK_ROOT/board/f413" -I"$TASK_ROOT/nvm" \
    -I"$TASK_ROOT/platform/stm32f405/Core/Inc" \
    -I"$TASK_ROOT/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc" \
    "$TASK_ROOT/tools/hil/f413_control_velocity_tests.c" -lm \
    -o "$TEST_OUT_DIR/f413_control_velocity_$PROFILE"
  ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
    "$TEST_OUT_DIR/f413_control_velocity_$PROFILE"
done
