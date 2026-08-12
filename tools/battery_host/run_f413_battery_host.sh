#!/bin/sh
set -eu

repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/nightfall-battery-host.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

cc -std=c11 -Wall -Wextra -Werror -pedantic \
  -I"$repo_root/tools/battery_host/include" \
  -I"$repo_root/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc" \
  -I"$repo_root/params/f413_preorder" \
  "$repo_root/platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_battery.c" \
  "$repo_root/tools/battery_host/test_f413_battery.c" \
  -o "$build_dir/test_f413_battery"

"$build_dir/test_f413_battery"
