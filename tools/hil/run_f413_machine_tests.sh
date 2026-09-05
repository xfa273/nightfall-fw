#!/usr/bin/env sh
set -eu
TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
TEST_OUT_DIR="$TASK_ROOT/build/hil_host"
mkdir -p "$TEST_OUT_DIR"
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -Wpedantic -O1 -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -DNIGHTFALL_F413_RUNTIME_CONFIG=1 -I"$TASK_ROOT/params/f413_preorder" \
  -I"$TASK_ROOT/board/f413" -I"$TASK_ROOT/nvm" \
  -I"$TASK_ROOT/platform/stm32f405/Core/Inc" \
  -I"$TASK_ROOT/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc" \
  "$TASK_ROOT/board/f413/f413_machine.c" \
  "$TASK_ROOT/board/f413/f413_registry.c" \
  "$TASK_ROOT/params/f413_preorder/profile.c" \
  "$TASK_ROOT/params/mini_r3_0/profile.c" \
  "$TASK_ROOT/nvm/nvm_identity.c" \
  "$TASK_ROOT/platform/stm32f405/Core/Src/sensor_distance.c" \
  "$TASK_ROOT/tools/hil/f413_machine_tests.c" \
  -o "$TEST_OUT_DIR/f413_machine_tests"
for machine in 0 2 3; do
  ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
    UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
    "$TEST_OUT_DIR/f413_machine_tests" "$machine"
done
