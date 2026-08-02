#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
BIN="$ROOT_DIR/build/solver_host/solver_host"
VALID_MAZE="$ROOT_DIR/tools/solver_host/testdata/open4.maze"
TMP_MAZE=$(mktemp "${TMPDIR:-/tmp}/nightfall-solver-host-maze.XXXXXX")
trap 'rm -f "$TMP_MAZE"' EXIT HUP INT TERM

SANITIZE=0 "$ROOT_DIR/tools/solver_host/run_solver_host.sh" --help >/dev/null

checks=0

expect_status()
{
  expected=$1
  label=$2
  shift 2
  set +e
  "$BIN" "$@" >/dev/null 2>&1
  actual=$?
  set -e
  checks=$((checks + 1))
  if [ "$actual" -ne "$expected" ]; then
    echo "$label: exit=$actual expected=$expected" >&2
    exit 1
  fi
}

expect_status 2 "mode overflow is rejected" --mode 258
expect_status 2 "case overflow is rejected" --case 257
expect_status 2 "time-plan origin is rejected" --time-plan --origin bottom-left
expect_status 2 "time-plan max-steps is rejected" --time-plan --max-steps 1
expect_status 2 "legacy turn-set is rejected" --turn-set profile
expect_status 2 "max-steps without explore is rejected" --max-steps 1
expect_status 2 "both planners are rejected" --time-plan --slalom-time-plan
expect_status 2 "slalom planner requires maze" --slalom-time-plan
expect_status 2 "slalom planner rejects explicit mode" --maze "$VALID_MAZE" --slalom-time-plan --mode 2
expect_status 2 "slalom planner rejects case 7" --maze "$VALID_MAZE" --slalom-time-plan --case 7
expect_status 2 "slalom profile requires planner" --slalom-profile f413-preorder-mode2
expect_status 2 "unknown slalom profile is rejected" --maze "$VALID_MAZE" --slalom-time-plan --slalom-profile missing
expect_status 0 "F413 slalom default case is valid" --maze "$VALID_MAZE" --slalom-time-plan --compare-orthogonal --assert-valid --summary-only
expect_status 0 "F405 mini mode5 case9 is valid" --maze "$VALID_MAZE" --slalom-time-plan --slalom-profile f405-mini-mode5 --case 9 --compare-orthogonal --assert-valid --summary-only

# More than 2*MAZE_SIZE+1 non-empty lines used to be silently truncated to 16x16.
i=0
while [ "$i" -lt 35 ]; do
  printf 'x\n' >>"$TMP_MAZE"
  i=$((i + 1))
done
expect_status 1 "oversized legacy maze is rejected" --maze "$TMP_MAZE" --explore-sim

echo "solver_host_cli_tests: all $checks checks passed"
