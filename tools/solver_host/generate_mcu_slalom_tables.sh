#!/usr/bin/env sh
set -eu
ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
CHECK_ONLY=0
if [ "${1:-}" = "--check" ]; then
  CHECK_ONLY=1
  shift
fi
if [ "$#" -gt 1 ]; then
  echo "usage: $0 [--check] [OUTPUT.c]" >&2
  exit 2
fi
OUTPUT_PATH=${1:-"$ROOT_DIR/common/route/mcu_slalom_tables.c"}
BUILD_DIR="$ROOT_DIR/build/solver_host"
mkdir -p "$BUILD_DIR"
compile_inputs() {
  "$@" \
  -I "$ROOT_DIR/common/route" -I "$ROOT_DIR/tools/solver_host" \
  -I "$ROOT_DIR/tools/solver_host/include" -I "$ROOT_DIR/platform/stm32f405/Core/Inc" \
  -I "$ROOT_DIR/params/f413_preorder" \
  "$ROOT_DIR/tools/solver_host/generate_mcu_slalom_tables.c" \
  "$ROOT_DIR/tools/solver_host/slalom_time_plan_host.c" \
  "$ROOT_DIR/tools/solver_host/slalom_profile_baseline.c" \
  "$ROOT_DIR/tools/solver_host/maze_ascii.c" \
  "$ROOT_DIR/common/route/motion_time.c" \
  "$ROOT_DIR/common/route/orthogonal_time_planner.c" \
  "$ROOT_DIR/common/route/route_clearance.c" \
  "$ROOT_DIR/common/route/slalom_plan_legacy_codec.c" \
  "$ROOT_DIR/common/route/legacy_path_codec.c" \
  "$ROOT_DIR/params/f413_preorder/shortest_run_params_split.c" ${TABLE_LINK_LIBRARY:+"$TABLE_LINK_LIBRARY"}
}
TABLE_LINK_LIBRARY=
compile_inputs "${CC:-cc}" -std=c11 -MM > "$BUILD_DIR/mcu_slalom_tables.d"
INPUTS_SHA256=$(python3 - "$ROOT_DIR" "$BUILD_DIR/mcu_slalom_tables.d" <<'PY'
import hashlib
from pathlib import Path
import shlex
import sys

root = Path(sys.argv[1]).resolve()
dependencies = Path(sys.argv[2]).read_text().replace('\\\n', '')
inputs = {root / 'tools/solver_host/generate_mcu_slalom_tables.sh'}
for line in dependencies.splitlines():
    for name in shlex.split(line.split(':', 1)[1]):
        path = Path(name).resolve()
        if path.is_relative_to(root):
            inputs.add(path)
digest = hashlib.sha256()
for path in sorted(inputs):
    data = path.read_bytes()
    digest.update(path.relative_to(root).as_posix().encode() + b'\0')
    digest.update(str(len(data)).encode() + b'\0' + data)
print(digest.hexdigest())
PY
)
TABLE_LINK_LIBRARY=-lm
compile_inputs "${CC:-cc}" -std=c11 -O2 -Wall -Wextra -Wpedantic -Wno-strict-prototypes \
  -o "$BUILD_DIR/generate_mcu_slalom_tables"
TEMP_OUTPUT=$(mktemp "$BUILD_DIR/mcu_slalom_tables.XXXXXX")
trap 'rm -f "$TEMP_OUTPUT"' EXIT HUP INT TERM
"$BUILD_DIR/generate_mcu_slalom_tables" "$TEMP_OUTPUT" "$INPUTS_SHA256"
if [ "$CHECK_ONLY" -eq 1 ]; then
  if ! cmp -s "$TEMP_OUTPUT" "$OUTPUT_PATH"; then
    echo "MCU slalom tables are stale: regenerate with tools/solver_host/generate_mcu_slalom_tables.sh" >&2
    exit 1
  fi
  echo "MCU slalom tables match their inputs ($INPUTS_SHA256)"
else
  cp "$TEMP_OUTPUT" "$OUTPUT_PATH"
fi
