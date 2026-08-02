#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
DATA_REV="762ed2b68735ea29148c6a1251a90ed0651ff26b"
DATA_DIR="$ROOT_DIR/build/solver_host/kerilab_data/$DATA_REV"
BASE_URL="https://raw.githubusercontent.com/kerikun11/micromouse-maze-data/$DATA_REV"
BIN="$ROOT_DIR/build/solver_host/solver_host"
RESULT_TSV="$ROOT_DIR/build/solver_host/slalom_kerilab_matrix.tsv"

PROFILES=${PROFILES:-"f413-preorder-mode2 f405-mini-mode2 f405-mini-mode3 f405-mini-mode4 f405-mini-mode5"}
CASES=${CASES:-"8 9"}
MAZES=${MAZES:-"16MM2012CX.maze 16MM2013CX.maze 16MM2014CX.maze 16MM2015CX.maze 16MM2016CX.maze 16MM2017CX.maze 16MM2018CX.maze 16MM2019CX.maze 16MM2020CX.maze 32MM2008HX.maze 32MM2009HX.maze 32MM2010HX.maze 32MM2011HX.maze 32MM2012HX.maze 32MM2013HX.maze 32MM2014HX.maze 32MM2015HX.maze 32MM2016HX.maze 32MM2017HX.maze 32MM2018HX.maze 32MM2019HX.maze 32MM2021HX.maze 32MM2022HX.maze 32MM2023HX.maze"}

if ! command -v curl >/dev/null 2>&1; then
  echo "curl is required" >&2
  exit 2
fi

mkdir -p "$DATA_DIR"
RESULT_TMP=$(mktemp "$ROOT_DIR/build/solver_host/slalom_kerilab_matrix.tsv.tmp.XXXXXX")
cleanup_result_tmp() {
  if [ -n "${RESULT_TMP:-}" ]; then
    rm -f "$RESULT_TMP"
  fi
}
trap cleanup_result_tmp 0 1 2 15

"$ROOT_DIR/tools/solver_host/run_slalom_profile_audit.sh" >/dev/null
SANITIZE=0 "$ROOT_DIR/tools/solver_host/run_solver_host.sh" --help >/dev/null

for maze in $MAZES; do
  file="$DATA_DIR/$maze"
  if [ ! -f "$file" ]; then
    download_file="$file.download.$$"
    echo "[fetch] $BASE_URL/data/$maze"
    if ! curl -fsSL "$BASE_URL/data/$maze" -o "$download_file"; then
      rm -f "$download_file"
      exit 1
    fi
    mv "$download_file" "$file"
  fi
done

printf 'profile\tcase\tmaze\tgoal_entry_us\torthogonal_goal_entry_us\timprovement_us\tactions\tdiagonal_actions\tlegacy_geometry\n' >"$RESULT_TMP"

configurations=0
failures=0
not_slower=0
strict_improvements=0
diagonal_routes=0
legacy_compatible=0
legacy_terminal_diagonal=0
completed=0

for profile in $PROFILES; do
  for case_index in $CASES; do
    for maze in $MAZES; do
      file="$DATA_DIR/$maze"
      configurations=$((configurations + 1))
      echo "[matrix] $configurations profile=$profile case=$case_index maze=$maze"
      set +e
      summary=$(
        "$BIN" --maze "$file" --slalom-time-plan \
          --slalom-profile "$profile" --case "$case_index" \
          --compare-orthogonal --assert-valid --summary-only 2>&1
      )
      status=$?
      set -e
      if [ "$status" -ne 0 ]; then
        echo "$summary" >&2
        failures=$((failures + 1))
        continue
      fi

      goal_entry=$(printf '%s\n' "$summary" | sed -n 's/.* goal_entry_us=\([0-9][0-9]*\).*/\1/p')
      orthogonal_entry=$(printf '%s\n' "$summary" | sed -n 's/.* orthogonal_goal_entry_us=\([0-9][0-9]*\).*/\1/p')
      improvement=$(printf '%s\n' "$summary" | sed -n 's/.* improvement_us=\([0-9][0-9]*\).*/\1/p')
      actions=$(printf '%s\n' "$summary" | sed -n 's/.* actions=\([0-9][0-9]*\).*/\1/p')
      diagonal_actions=$(printf '%s\n' "$summary" | sed -n 's/.* diagonal_actions=\([0-9][0-9]*\).*/\1/p')
      legacy_geometry=$(printf '%s\n' "$summary" | sed -n 's/.* legacy_geometry=\([^ ]*\).*/\1/p')
      if [ -z "$goal_entry" ] || [ -z "$orthogonal_entry" ] ||
         [ -z "$improvement" ] || [ -z "$actions" ] ||
         [ -z "$diagonal_actions" ] || [ -z "$legacy_geometry" ]; then
        echo "failed to parse: $summary" >&2
        failures=$((failures + 1))
        continue
      fi

      completed=$((completed + 1))
      not_slower=$((not_slower + 1))
      if [ "$improvement" -gt 0 ]; then
        strict_improvements=$((strict_improvements + 1))
      fi
      if [ "$diagonal_actions" -gt 0 ]; then
        diagonal_routes=$((diagonal_routes + 1))
      fi
      if [ "$legacy_geometry" = "ok" ]; then
        legacy_compatible=$((legacy_compatible + 1))
      elif [ "$legacy_geometry" = "terminal-diagonal-unsupported" ]; then
        legacy_terminal_diagonal=$((legacy_terminal_diagonal + 1))
      else
        echo "unexpected legacy geometry status: $legacy_geometry ($profile case=$case_index maze=$maze)" >&2
        failures=$((failures + 1))
        continue
      fi
      printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "$profile" "$case_index" "$maze" "$goal_entry" \
        "$orthogonal_entry" "$improvement" "$actions" \
        "$diagonal_actions" "$legacy_geometry" >>"$RESULT_TMP"
    done
  done
done

if [ "$failures" -ne 0 ]; then
  echo "[matrix-summary] configurations=$configurations failed=$failures data_rev=$DATA_REV" >&2
  exit 1
fi

min_diagonal_routes=${MIN_DIAGONAL_ROUTES:-$configurations}
min_strict_improvements=${MIN_STRICT_IMPROVEMENTS:-$configurations}
min_legacy_compatible=${MIN_LEGACY_COMPATIBLE:-1}
min_legacy_terminal_diagonal=${MIN_LEGACY_TERMINAL_DIAGONAL:-1}
if [ "$completed" -ne "$configurations" ] ||
   [ "$not_slower" -ne "$configurations" ] ||
   [ "$diagonal_routes" -lt "$min_diagonal_routes" ] ||
   [ "$strict_improvements" -lt "$min_strict_improvements" ] ||
   [ $((legacy_compatible + legacy_terminal_diagonal)) -ne "$configurations" ] ||
   [ "$legacy_compatible" -lt "$min_legacy_compatible" ] ||
   [ "$legacy_terminal_diagonal" -lt "$min_legacy_terminal_diagonal" ]; then
  echo "[matrix-summary] regression-threshold-failed configurations=$configurations completed=$completed diagonal_not_slower=$not_slower strict_improvements=$strict_improvements minimum_strict=$min_strict_improvements diagonal_routes=$diagonal_routes minimum_diagonal=$min_diagonal_routes legacy_compatible=$legacy_compatible minimum_legacy_compatible=$min_legacy_compatible legacy_terminal_diagonal=$legacy_terminal_diagonal minimum_legacy_terminal_diagonal=$min_legacy_terminal_diagonal data_rev=$DATA_REV" >&2
  exit 1
fi

mv "$RESULT_TMP" "$RESULT_TSV"
RESULT_TMP=

echo "[matrix-summary] configurations=$configurations failed=0 diagonal_not_slower=$not_slower strict_improvements=$strict_improvements diagonal_routes=$diagonal_routes legacy_compatible=$legacy_compatible legacy_terminal_diagonal=$legacy_terminal_diagonal data_rev=$DATA_REV"
echo "[matrix-summary] results=$RESULT_TSV"
