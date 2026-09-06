#!/usr/bin/env sh
set -eu
TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
TEST_OUT_DIR="$TASK_ROOT/build/hil_host"
F413_CORE="$TASK_ROOT/platform/stm32f413/HM_Nightfall_f413_preorder/Core"
mkdir -p "$TEST_OUT_DIR"
for guard in 0 1; do
  "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -Wpedantic -O1 -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer -DSTM32F413xx \
    -DNIGHTFALL_F413_DESTRUCTIVE_NVM_DIAGNOSTICS="$guard" \
    -I"$TASK_ROOT/tools/hil/nvm_stubs" -I"$TASK_ROOT/nvm" \
    -I"$TASK_ROOT/board/f413" -I"$TASK_ROOT/platform/trace" \
    -I"$TASK_ROOT/platform/stm32f405/Core/Inc" -I"$F413_CORE/Inc" \
    "$F413_CORE/Src/f413_diag.c" "$F413_CORE/Src/f413_trace_diag.c" \
    "$TASK_ROOT/nvm/nvm_params.c" "$TASK_ROOT/nvm/nvm_trace_log.c" \
    "$TASK_ROOT/tools/hil/f413_nvm_guard_tests.c" \
    -o "$TEST_OUT_DIR/f413_nvm_guard_tests_$guard"
  ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
    "$TEST_OUT_DIR/f413_nvm_guard_tests_$guard"
done
