#!/usr/bin/env sh
set -eu
TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
TEST_OUT_DIR="$TASK_ROOT/build/hil_host"
mkdir -p "$TEST_OUT_DIR"
case "$(uname -s)" in
  Darwin) GC_FLAGS="-Wl,-dead_strip" ;;
  *) GC_FLAGS="-Wl,--gc-sections" ;;
esac
for PROFILE in f413_preorder mini_r3_0; do
  "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -Wno-unused-function -Wno-array-bounds -O1 -g \
    -DSTM32F413xx -fsanitize=address,undefined -fno-omit-frame-pointer \
    -ffunction-sections -fdata-sections \
    -I"$TASK_ROOT/params/$PROFILE" -I"$TASK_ROOT/nvm" \
    -I"$TASK_ROOT/tools/hil/nvm_stubs" -I"$TASK_ROOT/board/f413" \
    -I"$TASK_ROOT/platform/trace" -I"$TASK_ROOT/platform/stm32f405/Core/Inc" \
    -I"$TASK_ROOT/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc" \
    "$TASK_ROOT/tools/hil/f413_search_distance_tests.c" \
    "$TASK_ROOT/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_front_match.c" \
    "$TASK_ROOT/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_run_features.c" \
    "$TASK_ROOT/params/$PROFILE/search_run_params_split.c" \
    $GC_FLAGS -lm -o "$TEST_OUT_DIR/f413_search_distance_tests_$PROFILE"
  ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
    UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
    "$TEST_OUT_DIR/f413_search_distance_tests_$PROFILE"
done
