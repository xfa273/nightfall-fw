#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
DATA_REV="762ed2b68735ea29148c6a1251a90ed0651ff26b"
OUT_DIR="$ROOT_DIR/build/solver_host/kerilab_data/$DATA_REV"
BASE_URL="https://raw.githubusercontent.com/kerikun11/micromouse-maze-data/$DATA_REV"
MODE="${MODE:-2}"
CASE="${CASE:-2}"
TURN_SET="${TURN_SET:-profile}"
RUN_ALL="${ALL:-0}"

REPRESENTATIVE_MAZES="\
data/16MM2012CX.maze \
data/16MM2020CX.maze \
data/32MM2008HX.maze \
data/32MM2010HX.maze \
data/32MM2023HX.maze"

ALL_COMPETITION_MAZES="\
data/16MM2012CX.maze \
data/16MM2013CX.maze \
data/16MM2014CX.maze \
data/16MM2015CX.maze \
data/16MM2016CX.maze \
data/16MM2017CX.maze \
data/16MM2018CX.maze \
data/16MM2019CX.maze \
data/16MM2020CX.maze \
data/32MM2008HX.maze \
data/32MM2009HX.maze \
data/32MM2010HX.maze \
data/32MM2011HX.maze \
data/32MM2012HX.maze \
data/32MM2013HX.maze \
data/32MM2014HX.maze \
data/32MM2015HX.maze \
data/32MM2016HX.maze \
data/32MM2017HX.maze \
data/32MM2018HX.maze \
data/32MM2019HX.maze \
data/32MM2021HX.maze \
data/32MM2022HX.maze \
data/32MM2023HX.maze"

if [ "$#" -gt 0 ]; then
  MAZES="$*"
elif [ "$RUN_ALL" = "1" ]; then
  MAZES="$ALL_COMPETITION_MAZES"
else
  MAZES="$REPRESENTATIVE_MAZES"
fi

if ! command -v curl >/dev/null 2>&1; then
  echo "curl is required" >&2
  exit 2
fi

mkdir -p "$OUT_DIR"
fail_count=0
run_count=0

for maze in $MAZES; do
  file="$OUT_DIR/$(basename "$maze")"
  url="$BASE_URL/$maze"
  if [ ! -f "$file" ]; then
    echo "[fetch] $url"
    download_file="$file.download.$$"
    if ! curl -fsSL "$url" -o "$download_file"; then
      rm -f "$download_file"
      exit 1
    fi
    mv "$download_file" "$file"
  fi
  run_count=$((run_count + 1))
  echo "[run] $maze mode=$MODE case=$CASE turn_set=$TURN_SET time-plan=yes assert-valid=yes"
  if ! "$ROOT_DIR/tools/solver_host/run_solver_host.sh" \
      --maze "$file" \
      --mode "$MODE" \
      --case "$CASE" \
      --time-plan \
      --turn-set "$TURN_SET" \
      --assert-valid; then
    fail_count=$((fail_count + 1))
  fi
  echo
done

if [ "$fail_count" -ne 0 ]; then
  echo "[summary] total=$run_count failed=$fail_count data_rev=$DATA_REV" >&2
  exit 1
fi

echo "[summary] total=$run_count failed=0 data_rev=$DATA_REV"
