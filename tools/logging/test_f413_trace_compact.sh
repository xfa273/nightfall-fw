#!/bin/sh
set -eu

repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build_dir=$(mktemp -d /tmp/nightfall-trace-compact-test.XXXXXX)
trap 'rm -rf -- "$build_dir"' EXIT HUP INT TERM

cc -std=c11 -Wall -Wextra -Werror \
  -I"$repo_root/nvm" \
  -I"$repo_root/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc" \
  "$repo_root/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_trace_compact.c" \
  "$repo_root/tools/logging/f413_trace_compact_host_test.c" \
  -o "$build_dir/f413_trace_compact_host_test"

"$build_dir/f413_trace_compact_host_test"
