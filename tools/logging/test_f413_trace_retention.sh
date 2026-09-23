#!/bin/sh
set -eu
repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build_dir=$(mktemp -d /tmp/nightfall-trace-retention.XXXXXX)
trap 'rm -rf -- "$build_dir"' EXIT HUP INT TERM
core="$repo_root/platform/stm32f413/HM_Nightfall_f413_preorder/Core"
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -O1 -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -I"$repo_root/tools/logging/trace_stubs" -I"$repo_root/tools/hil/nvm_stubs" \
  -I"$repo_root/nvm" -I"$repo_root/platform/trace" -I"$core/Inc" \
  "$repo_root/nvm/nvm_trace_log.c" "$core/Src/f413_trace_compact.c" \
  "$core/Src/f413_trace_log.c" "$core/Src/f413_trace_diag.c" \
  "$repo_root/tools/logging/f413_trace_retention_host_test.c" \
  -o "$build_dir/trace_retention"
ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 \
  "$build_dir/trace_retention" "$build_dir"
TRACE_RETENTION_FIXTURES="$build_dir" python3 -m unittest discover \
  -s "$repo_root/tools/logging" -p 'test_trace_retention.py' -v
