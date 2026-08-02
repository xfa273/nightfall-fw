#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
OUT_DIR="$ROOT_DIR/build/solver_host"
OUT_BIN="$OUT_DIR/test_legacy_path_validator"
CC_BIN=${CC:-cc}
SANITIZE=${SANITIZE:-1}

SANITIZER_FLAGS=
if [ "$SANITIZE" = "1" ]; then
  SANITIZER_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer -O1 -g"
fi

mkdir -p "$OUT_DIR"
# SANITIZER_FLAGS intentionally expands into separate compiler arguments.
# shellcheck disable=SC2086
"$CC_BIN" -std=c11 -Wall -Wextra -Wpedantic -Werror \
  $SANITIZER_FLAGS \
  -I"$ROOT_DIR/tools/solver_host" \
  -I"$ROOT_DIR/platform/stm32f405/Core/Inc" \
  "$ROOT_DIR/tools/solver_host/legacy_path_validator.c" \
  "$ROOT_DIR/tools/solver_host/test_legacy_path_validator.c" \
  -o "$OUT_BIN"

if [ "$SANITIZE" = "1" ]; then
  ASAN_OPTIONS=${ASAN_OPTIONS:-detect_leaks=0:halt_on_error=1} \
  UBSAN_OPTIONS=${UBSAN_OPTIONS:-halt_on_error=1:print_stacktrace=1} \
    "$OUT_BIN"
else
  "$OUT_BIN"
fi
