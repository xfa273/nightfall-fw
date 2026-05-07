#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
OUT_DIR="$ROOT_DIR/build/solver_host/kerilab_data"
BASE_URL="https://raw.githubusercontent.com/kerikun11/micromouse-maze-data/master"
MODE="${MODE:-2}"
CASE="${CASE:-1}"

if [ "$#" -gt 0 ]; then
  MAZES="$*"
else
  MAZES="data/32MM2019HX.maze data/32MM2021HX.maze data/32MM2022HX.maze data/32MM2023HX.maze data/32_test_01.maze"
fi

if ! command -v curl >/dev/null 2>&1; then
  echo "curl is required" >&2
  exit 2
fi

mkdir -p "$OUT_DIR"
fail_count=0

for maze in $MAZES; do
  file="$OUT_DIR/$(basename "$maze")"
  url="$BASE_URL/$maze"
  if [ ! -f "$file" ]; then
    echo "[fetch] $url"
    curl -fsSL "$url" -o "$file"
  fi
  echo "[run] $maze mode=$MODE case=$CASE"
  if ! "$ROOT_DIR/tools/solver_host/run_solver_host.sh" --maze "$file" --mode "$MODE" --case "$CASE"; then
    fail_count=$((fail_count + 1))
  fi
  echo
 done

if [ "$fail_count" -ne 0 ]; then
  echo "[summary] failed=$fail_count" >&2
  exit 1
fi

echo "[summary] all passed"
