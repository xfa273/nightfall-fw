#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
EXPORTER="$SCRIPT_DIR/export_plotjuggler_csv.py"
DEFAULT_LOGS_DIR="$SCRIPT_DIR/logs"
DEFAULT_LAYOUT="$SCRIPT_DIR/plotjuggler/nightfall_f413_tune.xml"

INPUT="${MICROMOUSE_LOG_DIR:-$DEFAULT_LOGS_DIR}"
LAYOUT="$DEFAULT_LAYOUT"
DRY_RUN=0
PICK=0

usage() {
  cat <<EOF
Usage: $0 [options] [csv-or-log-dir]

Options:
  --pick              Select a CSV file with the macOS file picker
  --layout PATH       PlotJuggler layout XML (default: $DEFAULT_LAYOUT)
  --dry-run           Print the command without launching PlotJuggler
  -h, --help          Show this help

Examples:
  $0
  $0 tools/logging/logs/stm32_log_20260510_153411.csv
  $0 --pick
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pick)
      PICK=1
      shift
      ;;
    --layout)
      if [[ $# -lt 2 ]]; then
        echo "[ERROR] --layout requires a path." >&2
        exit 1
      fi
      LAYOUT="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      break
      ;;
    -* )
      echo "[ERROR] Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
    *)
      INPUT="$1"
      shift
      ;;
  esac
done

if [[ $# -gt 0 ]]; then
  INPUT="$1"
fi

if [[ "$PICK" -eq 1 ]]; then
  INPUT="$(osascript <<'APPLESCRIPT'
set chosenFile to choose file with prompt "Select Nightfall CSV log" of type {"csv"}
POSIX path of chosenFile
APPLESCRIPT
)"
fi

if [[ ! -f "$EXPORTER" ]]; then
  echo "[ERROR] Exporter not found: $EXPORTER" >&2
  exit 1
fi

if [[ ! -f "$LAYOUT" ]]; then
  echo "[ERROR] Layout not found: $LAYOUT" >&2
  exit 1
fi

if [[ -n "${PLOTJUGGLER_BIN:-}" ]]; then
  PLOTJUGGLER="$PLOTJUGGLER_BIN"
elif command -v plotjuggler >/dev/null 2>&1; then
  PLOTJUGGLER="$(command -v plotjuggler)"
elif [[ -x /Applications/PlotJuggler.app/Contents/MacOS/plotjuggler ]]; then
  PLOTJUGGLER="/Applications/PlotJuggler.app/Contents/MacOS/plotjuggler"
else
  echo "[ERROR] PlotJuggler executable not found." >&2
  echo "[HINT] Install it with: brew install --cask plotjuggler" >&2
  echo "[HINT] Or set PLOTJUGGLER_BIN=/path/to/plotjuggler" >&2
  exit 1
fi

if [[ "$INPUT" == *.plotjuggler.csv && -f "$INPUT" ]]; then
  PJ_CSV="$INPUT"
else
  EXPORT_LOG="$(mktemp)"
  python3 "$EXPORTER" "$INPUT" >"$EXPORT_LOG"
  cat "$EXPORT_LOG"
  PJ_CSV="$(awk -F'output: ' '/^\[PlotJuggler\] output:/ {print $2}' "$EXPORT_LOG" | tail -1)"
  rm -f "$EXPORT_LOG"
fi

if [[ -z "${PJ_CSV:-}" || ! -f "$PJ_CSV" ]]; then
  echo "[ERROR] PlotJuggler CSV was not generated: ${PJ_CSV:-}" >&2
  exit 1
fi

CMD=("$PLOTJUGGLER" --nosplash --datafile "$PJ_CSV" --layout "$LAYOUT" --window_title "Nightfall PlotJuggler")

printf '[PlotJuggler] command:'
printf ' %q' "${CMD[@]}"
printf '\n'

if [[ "$DRY_RUN" -eq 1 ]]; then
  exit 0
fi

exec "${CMD[@]}"
