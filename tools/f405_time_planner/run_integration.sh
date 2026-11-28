#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
OUT="$ROOT/build/f405_time_planner"
mkdir -p "$OUT"
flags=(-std=c11 -O2 -g -Wall -Wextra -Werror -ffunction-sections -fdata-sections)
if [[ "${SANITIZE:-1}" == 1 ]]; then flags+=(-fsanitize=address,undefined -fno-omit-frame-pointer); fi
if [[ "$(uname -s)" == Darwin ]]; then flags+=(-Wl,-dead_strip); else flags+=(-Wl,--gc-sections); fi
"${CC:-cc}" "${flags[@]}" -DNIGHTFALL_CLASSIC_TIME_PLANNER=1 \
  -include "$ROOT/tools/f405_time_planner/host_platform.h" \
  -I"$ROOT/common/route" -I"$ROOT/tools/solver_host" \
  -I"$ROOT/platform/stm32f405/Core/Inc" -I"$ROOT/params/classic_r1_0" \
  "$ROOT/tools/f405_time_planner/integration.c" \
  "$ROOT/platform/stm32f405/Core/Src/run.c" \
  "$ROOT/platform/stm32f405/Core/Src/solver.c" \
  "$ROOT/platform/stm32f405/Core/Src/solver_params.c" \
  "$ROOT/platform/stm32f405/Core/Src/maze_grid.c" \
  "$ROOT/platform/stm32f405/Core/Src/f405_time_path.c" \
  "$ROOT/platform/stm32f405/Core/Src/f405_orthogonal_preview.c" \
  "$ROOT/common/route/compact_orthogonal.c" "$ROOT/common/route/motion_time.c" \
  "$ROOT/common/route/orthogonal_time_planner.c" "$ROOT/tools/solver_host/maze_ascii.c" \
  "$ROOT/params/classic_r1_0/shortest_run_params_split.c" -lm -o "$OUT/integration"
"$OUT/integration" "$@" > "$OUT/integration.log"
