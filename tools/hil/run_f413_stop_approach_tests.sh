#!/usr/bin/env sh
set -eu
TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
TEST_OUT_DIR="$TASK_ROOT/build/hil_host"
mkdir -p "$TEST_OUT_DIR"
case "$(uname -s)" in
  Darwin) GC_FLAGS="-Wl,-dead_strip" ;;
  *) GC_FLAGS="-Wl,--gc-sections" ;;
esac
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -Wno-unused-function -Wno-array-bounds -O1 -g \
  -DSTM32F413xx \
  -fsanitize=address,undefined -fno-omit-frame-pointer -ffunction-sections -fdata-sections \
  -I"$TASK_ROOT/params/mini_r3_0" -I"$TASK_ROOT/nvm" \
  -I"$TASK_ROOT/tools/hil/nvm_stubs" -I"$TASK_ROOT/board/f413" \
  -I"$TASK_ROOT/platform/trace" -I"$TASK_ROOT/platform/stm32f405/Core/Inc" \
  -I"$TASK_ROOT/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc" \
  "$TASK_ROOT/tools/hil/f413_stop_approach_tests.c" \
  "$TASK_ROOT/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_front_match.c" \
  $GC_FLAGS -lm -o "$TEST_OUT_DIR/f413_stop_approach_tests"
ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
  UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  "$TEST_OUT_DIR/f413_stop_approach_tests"
for CONTROL_TEST_NAME in f413_control_stop_tests f413_control_gain_tests f413_control_velocity_tests; do
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -O1 -g -DSTM32F413xx \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I"$TASK_ROOT/tools/hil/control_stubs" -I"$TASK_ROOT/tools/hil/nvm_stubs" \
  -I"$TASK_ROOT/params/mini_r3_0" -I"$TASK_ROOT/board/f413" -I"$TASK_ROOT/nvm" \
  -I"$TASK_ROOT/platform/stm32f405/Core/Inc" \
  -I"$TASK_ROOT/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc" \
  "$TASK_ROOT/tools/hil/${CONTROL_TEST_NAME}.c" -lm \
  -o "$TEST_OUT_DIR/${CONTROL_TEST_NAME}"
ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
  UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  "$TEST_OUT_DIR/${CONTROL_TEST_NAME}"
done
