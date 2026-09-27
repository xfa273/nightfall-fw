#!/usr/bin/env sh
set -eu
TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
TEST_OUT_DIR="$TASK_ROOT/build/hil_host"
mkdir -p "$TEST_OUT_DIR"
for profile in mini_r3_0 f413_preorder; do
  for kind in offset offset_wall; do
    "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -Wno-unused-function -O1 -g -DSTM32F413xx \
      -fsanitize=address,undefined -fno-omit-frame-pointer \
      -I"$TASK_ROOT/params/$profile" -I"$TASK_ROOT/nvm" -I"$TASK_ROOT/common/route" \
      -I"$TASK_ROOT/tools/hil/nvm_stubs" -I"$TASK_ROOT/board/f413" \
      -I"$TASK_ROOT/platform/trace" \
      -I"$TASK_ROOT/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc" \
      -I"$TASK_ROOT/platform/stm32f405/Core/Inc" \
      "$TASK_ROOT/tools/hil/f413_chained_${kind}_tests.c" \
      "$TASK_ROOT/params/$profile/shortest_run_params_split.c" \
      "$TASK_ROOT/common/route/legacy_path_codec.c" "$TASK_ROOT/common/route/motion_time.c" \
      -lm -o "$TEST_OUT_DIR/f413_chained_${kind}_tests_$profile"
    ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
      "$TEST_OUT_DIR/f413_chained_${kind}_tests_$profile"
  done
done
