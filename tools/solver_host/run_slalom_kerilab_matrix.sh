#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
DATA_REV="762ed2b68735ea29148c6a1251a90ed0651ff26b"
DATA_DIR="$ROOT_DIR/build/solver_host/kerilab_data/$DATA_REV"
BASE_URL="https://raw.githubusercontent.com/kerikun11/micromouse-maze-data/$DATA_REV"
BIN="$ROOT_DIR/build/solver_host/solver_host"
RESULT_TSV="$ROOT_DIR/build/solver_host/slalom_kerilab_matrix.tsv"

DEFAULT_PROFILES="f413-preorder-mode2 f405-mini-mode2 f405-mini-mode3 f405-mini-mode4 f405-mini-mode5"
DEFAULT_CASES="8 9"
DEFAULT_MAZES="16MM2012CX.maze 16MM2013CX.maze 16MM2014CX.maze 16MM2015CX.maze 16MM2016CX.maze 16MM2017CX.maze 16MM2018CX.maze 16MM2019CX.maze 16MM2020CX.maze 32MM2008HX.maze 32MM2009HX.maze 32MM2010HX.maze 32MM2011HX.maze 32MM2012HX.maze 32MM2013HX.maze 32MM2014HX.maze 32MM2015HX.maze 32MM2016HX.maze 32MM2017HX.maze 32MM2018HX.maze 32MM2019HX.maze 32MM2021HX.maze 32MM2022HX.maze 32MM2023HX.maze"
matrix_selection_overridden=0
if [ "${PROFILES+x}" = x ] || [ "${CASES+x}" = x ] ||
   [ "${MAZES+x}" = x ]; then
  matrix_selection_overridden=1
fi
PROFILES=${PROFILES:-$DEFAULT_PROFILES}
CASES=${CASES:-$DEFAULT_CASES}
MAZES=${MAZES:-$DEFAULT_MAZES}

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

printf 'profile\tcase\tmaze\tstatus\tgoal_entry_us\torthogonal_goal_entry_us\timprovement_us\tactions\tdiagonal_actions\tzero_step_diagonal_turns\treduced_turns\tlow_turns\tcrawl_turns\tsmall90_actions\tlegacy_geometry\n' >"$RESULT_TMP"

configurations=0
failures=0
not_slower=0
strict_improvements=0
diagonal_routes=0
legacy_compatible=0
legacy_terminal_diagonal=0
zero_step_diagonal_turns=0
reduced_turns=0
low_turns=0
crawl_turns=0
small90_actions=0
no_feasible_terminal=0
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
        if [ "$maze" = "32MM2009HX.maze" ] &&
           printf '%s\n' "$summary" |
             grep -q 'planner=no-feasible-terminal'; then
          no_feasible_terminal=$((no_feasible_terminal + 1))
          printf '%s\t%s\t%s\tno-feasible-terminal\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\n' \
            "$profile" "$case_index" "$maze" >>"$RESULT_TMP"
        else
          echo "$summary" >&2
          failures=$((failures + 1))
        fi
        continue
      fi

      goal_entry=$(printf '%s\n' "$summary" | sed -n 's/.* goal_entry_us=\([0-9][0-9]*\).*/\1/p')
      orthogonal_entry=$(printf '%s\n' "$summary" | sed -n 's/.* orthogonal_goal_entry_us=\([0-9][0-9]*\).*/\1/p')
      improvement=$(printf '%s\n' "$summary" | sed -n 's/.* improvement_us=\([0-9][0-9]*\).*/\1/p')
      actions=$(printf '%s\n' "$summary" | sed -n 's/.* actions=\([0-9][0-9]*\).*/\1/p')
      diagonal_actions=$(printf '%s\n' "$summary" | sed -n 's/.* diagonal_actions=\([0-9][0-9]*\).*/\1/p')
      zero_step_turns=$(printf '%s\n' "$summary" | sed -n 's/.* zero_step_diagonal_turns=\([0-9][0-9]*\).*/\1/p')
      reduced=$(printf '%s\n' "$summary" | sed -n 's/.* reduced_turns=\([0-9][0-9]*\).*/\1/p')
      low=$(printf '%s\n' "$summary" | sed -n 's/.* low_turns=\([0-9][0-9]*\).*/\1/p')
      crawl=$(printf '%s\n' "$summary" | sed -n 's/.* crawl_turns=\([0-9][0-9]*\).*/\1/p')
      small90=$(printf '%s\n' "$summary" | sed -n 's/.* small90_actions=\([0-9][0-9]*\).*/\1/p')
      legacy_geometry=$(printf '%s\n' "$summary" | sed -n 's/.* legacy_geometry=\([^ ]*\).*/\1/p')
      if [ -z "$goal_entry" ] || [ -z "$orthogonal_entry" ] ||
         [ -z "$improvement" ] || [ -z "$actions" ] ||
         [ -z "$diagonal_actions" ] || [ -z "$zero_step_turns" ] ||
         [ -z "$reduced" ] || [ -z "$low" ] || [ -z "$crawl" ] ||
         [ -z "$small90" ] ||
         [ -z "$legacy_geometry" ]; then
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
      zero_step_diagonal_turns=$((zero_step_diagonal_turns + zero_step_turns))
      reduced_turns=$((reduced_turns + reduced))
      low_turns=$((low_turns + low))
      crawl_turns=$((crawl_turns + crawl))
      small90_actions=$((small90_actions + small90))
      if [ "$legacy_geometry" = "ok" ]; then
        legacy_compatible=$((legacy_compatible + 1))
      elif [ "$legacy_geometry" = "terminal-diagonal-unsupported" ]; then
        legacy_terminal_diagonal=$((legacy_terminal_diagonal + 1))
      else
        echo "unexpected legacy geometry status: $legacy_geometry ($profile case=$case_index maze=$maze)" >&2
        failures=$((failures + 1))
        continue
      fi
      printf '%s\t%s\t%s\tok\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "$profile" "$case_index" "$maze" "$goal_entry" \
        "$orthogonal_entry" "$improvement" "$actions" \
        "$diagonal_actions" "$zero_step_turns" "$reduced" "$low" \
        "$crawl" "$small90" "$legacy_geometry" >>"$RESULT_TMP"
    done
  done
done

if [ "$failures" -ne 0 ]; then
  echo "[matrix-summary] configurations=$configurations failed=$failures data_rev=$DATA_REV" >&2
  exit 1
fi

# Pin the complete fixed matrix exactly. Diagnostic subsets have clustered
# ties, so their adoption minima default to zero unless explicitly supplied.
if [ "$matrix_selection_overridden" -eq 0 ]; then
  default_min_diagonal=230
  default_max_diagonal=230
  default_min_legacy_compatible=65
  default_max_legacy_compatible=65
  default_min_legacy_terminal=165
  default_max_legacy_terminal=165
  default_min_no_feasible_terminal=10
  default_max_no_feasible_terminal=10
  default_min_zero_step=1303
  default_max_zero_step=1303
  default_min_reduced=268
  default_max_reduced=268
  default_min_low=219
  default_max_low=219
  default_min_crawl=49
  default_max_crawl=49
else
  default_min_diagonal=0
  default_max_diagonal=$configurations
  default_min_legacy_compatible=0
  default_max_legacy_compatible=$configurations
  default_min_legacy_terminal=0
  default_max_legacy_terminal=$configurations
  default_min_no_feasible_terminal=0
  default_max_no_feasible_terminal=$configurations
  default_min_zero_step=0
  default_max_zero_step=65535
  default_min_reduced=0
  default_max_reduced=65535
  default_min_low=0
  default_max_low=65535
  default_min_crawl=0
  default_max_crawl=65535
fi
min_diagonal_routes=${MIN_DIAGONAL_ROUTES:-$default_min_diagonal}
min_strict_improvements=${MIN_STRICT_IMPROVEMENTS:-$default_min_diagonal}
max_diagonal_routes=${MAX_DIAGONAL_ROUTES:-$default_max_diagonal}
max_strict_improvements=${MAX_STRICT_IMPROVEMENTS:-$default_max_diagonal}
min_legacy_compatible=${MIN_LEGACY_COMPATIBLE:-$default_min_legacy_compatible}
max_legacy_compatible=${MAX_LEGACY_COMPATIBLE:-$default_max_legacy_compatible}
min_legacy_terminal_diagonal=${MIN_LEGACY_TERMINAL_DIAGONAL:-$default_min_legacy_terminal}
max_legacy_terminal_diagonal=${MAX_LEGACY_TERMINAL_DIAGONAL:-$default_max_legacy_terminal}
min_no_feasible_terminal=${MIN_NO_FEASIBLE_TERMINAL:-$default_min_no_feasible_terminal}
max_no_feasible_terminal=${MAX_NO_FEASIBLE_TERMINAL:-$default_max_no_feasible_terminal}
min_zero_step=${MIN_ZERO_STEP_DIAGONAL_TURNS:-$default_min_zero_step}
max_zero_step=${MAX_ZERO_STEP_DIAGONAL_TURNS:-$default_max_zero_step}
min_reduced=${MIN_REDUCED_TURNS:-$default_min_reduced}
max_reduced=${MAX_REDUCED_TURNS:-$default_max_reduced}
min_low=${MIN_LOW_TURNS:-$default_min_low}
max_low=${MAX_LOW_TURNS:-$default_max_low}
min_crawl=${MIN_CRAWL_TURNS:-$default_min_crawl}
max_crawl=${MAX_CRAWL_TURNS:-$default_max_crawl}
if [ $((completed + no_feasible_terminal)) -ne "$configurations" ] ||
   [ "$not_slower" -ne "$completed" ] ||
   [ "$diagonal_routes" -lt "$min_diagonal_routes" ] ||
   [ "$diagonal_routes" -gt "$max_diagonal_routes" ] ||
   [ "$strict_improvements" -lt "$min_strict_improvements" ] ||
   [ "$strict_improvements" -gt "$max_strict_improvements" ] ||
   [ "$zero_step_diagonal_turns" -lt "$min_zero_step" ] ||
   [ "$zero_step_diagonal_turns" -gt "$max_zero_step" ] ||
   [ "$reduced_turns" -lt "$min_reduced" ] ||
   [ "$reduced_turns" -gt "$max_reduced" ] ||
   [ "$low_turns" -lt "$min_low" ] ||
   [ "$low_turns" -gt "$max_low" ] ||
   [ "$crawl_turns" -lt "$min_crawl" ] ||
   [ "$crawl_turns" -gt "$max_crawl" ] ||
   [ "$small90_actions" -ne 0 ] ||
   [ "$no_feasible_terminal" -lt "$min_no_feasible_terminal" ] ||
   [ "$no_feasible_terminal" -gt "$max_no_feasible_terminal" ] ||
   [ $((legacy_compatible + legacy_terminal_diagonal)) -ne "$completed" ] ||
   [ "$legacy_compatible" -lt "$min_legacy_compatible" ] ||
   [ "$legacy_compatible" -gt "$max_legacy_compatible" ] ||
   [ "$legacy_terminal_diagonal" -lt "$min_legacy_terminal_diagonal" ] ||
   [ "$legacy_terminal_diagonal" -gt "$max_legacy_terminal_diagonal" ]; then
  echo "[matrix-summary] regression-threshold-failed configurations=$configurations completed=$completed no_feasible_terminal=$no_feasible_terminal minimum_no_feasible_terminal=$min_no_feasible_terminal maximum_no_feasible_terminal=$max_no_feasible_terminal diagonal_not_slower=$not_slower strict_improvements=$strict_improvements minimum_strict=$min_strict_improvements maximum_strict=$max_strict_improvements diagonal_routes=$diagonal_routes minimum_diagonal=$min_diagonal_routes maximum_diagonal=$max_diagonal_routes zero_step_diagonal_turns=$zero_step_diagonal_turns minimum_zero_step=$min_zero_step maximum_zero_step=$max_zero_step reduced_turns=$reduced_turns minimum_reduced=$min_reduced maximum_reduced=$max_reduced low_turns=$low_turns minimum_low=$min_low maximum_low=$max_low crawl_turns=$crawl_turns minimum_crawl=$min_crawl maximum_crawl=$max_crawl small90_actions=$small90_actions legacy_compatible=$legacy_compatible minimum_legacy_compatible=$min_legacy_compatible maximum_legacy_compatible=$max_legacy_compatible legacy_terminal_diagonal=$legacy_terminal_diagonal minimum_legacy_terminal_diagonal=$min_legacy_terminal_diagonal maximum_legacy_terminal_diagonal=$max_legacy_terminal_diagonal data_rev=$DATA_REV" >&2
  exit 1
fi

mv "$RESULT_TMP" "$RESULT_TSV"
RESULT_TMP=

echo "[matrix-summary] configurations=$configurations completed=$completed no_feasible_terminal=$no_feasible_terminal failed=0 diagonal_not_slower=$not_slower strict_improvements=$strict_improvements diagonal_routes=$diagonal_routes zero_step_diagonal_turns=$zero_step_diagonal_turns reduced_turns=$reduced_turns low_turns=$low_turns crawl_turns=$crawl_turns small90_actions=$small90_actions legacy_compatible=$legacy_compatible legacy_terminal_diagonal=$legacy_terminal_diagonal data_rev=$DATA_REV"
echo "[matrix-summary] results=$RESULT_TSV"
