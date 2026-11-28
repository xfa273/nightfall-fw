#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
OUT="$ROOT/build/f405_time_planner"
mkdir -p "$OUT"
flags=(-std=c11 -O2 -g -Wall -Wextra -Werror)
if [[ "${SANITIZE:-1}" == 1 ]]; then flags+=(-fsanitize=address,undefined -fno-omit-frame-pointer); fi
"${CC:-cc}" "${flags[@]}" \
  -I"$ROOT/common/route" -I"$ROOT/tools/solver_host" \
  -I"$ROOT/platform/stm32f405/Core/Inc" -I"$ROOT/params/classic_r1_0" \
  "$ROOT/tools/f405_time_planner/host.c" \
  "$ROOT/common/route/compact_orthogonal.c" "$ROOT/common/route/motion_time.c" \
  "$ROOT/common/route/orthogonal_time_planner.c" "$ROOT/tools/solver_host/maze_ascii.c" \
  "$ROOT/platform/stm32f405/Core/Src/f405_orthogonal_preview.c" \
  "$ROOT/params/classic_r1_0/shortest_run_params_split.c" -lm -o "$OUT/planner"
if [[ $# == 0 ]]; then set -- --self-test; fi
exec "$OUT/planner" "$@"
