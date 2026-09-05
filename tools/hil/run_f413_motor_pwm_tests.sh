#!/usr/bin/env sh
set -eu

TASK_ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
TEST_OUT_DIR="$TASK_ROOT/build/hil_host"
CC_BIN=${CC:-cc}
mkdir -p "$TEST_OUT_DIR"

# Host only: no probe, UART, firmware flashing, or motor command is used.
# Exercise both the current mini r3 default and explicit legacy wiring.
for wiring in 1 0; do
  set --
  if [ "$wiring" = 0 ]; then
    set -- -DNIGHTFALL_F413_MOTOR_LEFT_FORWARD_IN2_HIGH=0
  fi
  "$CC_BIN" -std=c11 -Wall -Wextra -Werror -Wpedantic -O1 -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -DTEST_EXPECT_LEFT_FORWARD_HIGH="$wiring" "$@" \
    -I"$TASK_ROOT/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc" \
    "$TASK_ROOT/tools/hil/f413_motor_pwm_tests.c" \
    -o "$TEST_OUT_DIR/f413_motor_pwm_$wiring"
  ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
    UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
    "$TEST_OUT_DIR/f413_motor_pwm_$wiring"
done
