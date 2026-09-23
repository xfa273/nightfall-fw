#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
OUT_DIR="$ROOT_DIR/build/solver_host"
OUT_BIN="$OUT_DIR/slalom_time_planner_tests"
CC_BIN=${CC:-cc}
SANITIZE=${SANITIZE:-1}

mkdir -p "$OUT_DIR"

SANITIZER_FLAGS=
if [ "$SANITIZE" = "1" ]; then
  SANITIZER_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer"
fi

# SANITIZER_FLAGS intentionally expands into separate compiler arguments.
# shellcheck disable=SC2086
"$CC_BIN" -std=c11 -Wall -Wextra -Werror -Wpedantic -O1 -g -fno-inline \
  $SANITIZER_FLAGS \
  -I"$ROOT_DIR/common/route" \
  -I"$ROOT_DIR/tools/solver_host" \
  "$ROOT_DIR/tools/solver_host/slalom_time_planner_tests.c" \
  "$ROOT_DIR/tools/solver_host/slalom_profile_baseline.c" \
  "$ROOT_DIR/common/route/slalom_time_planner.c" \
  "$ROOT_DIR/common/route/route_clearance.c" \
  "$ROOT_DIR/common/route/orthogonal_time_planner.c" \
  "$ROOT_DIR/common/route/motion_time.c" \
  -lm -o "$OUT_BIN"

if [ "$SANITIZE" = "1" ]; then
  ASAN_OPTIONS=${ASAN_OPTIONS:-detect_leaks=0:halt_on_error=1} \
  UBSAN_OPTIONS=${UBSAN_OPTIONS:-halt_on_error=1:print_stacktrace=1} \
    "$OUT_BIN"
else
  "$OUT_BIN"
fi
