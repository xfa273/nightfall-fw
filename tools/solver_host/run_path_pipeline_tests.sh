#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
OUT_DIR="$ROOT_DIR/build/solver_host"
OUT_BIN="$OUT_DIR/test_path_pipeline"
CC_BIN=${CC:-cc}
SANITIZE=${SANITIZE:-1}

SANITIZER_FLAGS=
if [ "$SANITIZE" = "1" ]; then
  SANITIZER_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer -O1 -g"
fi

mkdir -p "$OUT_DIR"
# SANITIZER_FLAGS intentionally expands into separate compiler arguments.
# shellcheck disable=SC2086
"$CC_BIN" -std=c11 -Wall -Wextra -Werror -Wpedantic \
  $SANITIZER_FLAGS \
  -I"$ROOT_DIR/common/route" \
  -I"$ROOT_DIR/tools/solver_host/include" \
  -I"$ROOT_DIR/params/mini_r1_0" \
  -I"$ROOT_DIR/platform/stm32f405/Core/Inc" \
  "$ROOT_DIR/common/route/legacy_path_codec.c" \
  "$ROOT_DIR/platform/stm32f405/Core/Src/path.c" \
  "$ROOT_DIR/tools/solver_host/path_pipeline_tests.c" \
  -o "$OUT_BIN"

if [ "$SANITIZE" = "1" ]; then
  ASAN_OPTIONS=${ASAN_OPTIONS:-detect_leaks=0:halt_on_error=1} \
  UBSAN_OPTIONS=${UBSAN_OPTIONS:-halt_on_error=1:print_stacktrace=1} \
    "$OUT_BIN"
else
  "$OUT_BIN"
fi
