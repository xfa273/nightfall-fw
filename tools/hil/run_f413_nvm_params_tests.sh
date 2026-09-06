#!/usr/bin/env sh
set -eu
TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
TEST_OUT_DIR="$TASK_ROOT/build/hil_host"
mkdir -p "$TEST_OUT_DIR"
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -Wpedantic -O1 -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer -DSTM32F413xx \
  -I"$TASK_ROOT/tools/hil/nvm_stubs" -I"$TASK_ROOT/nvm" \
  -I"$TASK_ROOT/board/f413" \
  -I"$TASK_ROOT/platform/stm32f405/Core/Inc" \
  "$TASK_ROOT/nvm/nvm_params.c" "$TASK_ROOT/tools/hil/f413_nvm_params_tests.c" \
  -o "$TEST_OUT_DIR/f413_nvm_params_tests"
ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
  "$TEST_OUT_DIR/f413_nvm_params_tests"
