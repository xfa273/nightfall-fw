#!/bin/bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/ai/maybe_apply_codex_result.sh --task-id <task-id> [--yes]

Options:
  --task-id <task-id>   Target handoff id
  --yes                 Apply without interactive confirmation
  -h, --help            Show help
USAGE
}

TASK_ID=""
ASSUME_YES=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --task-id)
      TASK_ID="$2"
      shift 2
      ;;
    --yes)
      ASSUME_YES=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [[ -z "$TASK_ID" ]]; then
  echo "Error: --task-id is required" >&2
  exit 2
fi

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Error: must run inside git repository" >&2
  exit 1
fi

ROOT_DIR="$(git rev-parse --show-toplevel)"
HANDOFF_DIR="$ROOT_DIR/docs/ai/HANDOFFS"
WORKLOG_FILE="$ROOT_DIR/docs/ai/WORKLOG.md"
META_FILE="$HANDOFF_DIR/${TASK_ID}-meta.env"

if [[ ! -f "$META_FILE" ]]; then
  echo "Error: meta file not found: $META_FILE" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$META_FILE"

if [[ -z "${DIFF_FILE:-}" || ! -f "$DIFF_FILE" ]]; then
  echo "Error: diff file not found: ${DIFF_FILE:-N/A}" >&2
  exit 1
fi

if [[ ! -s "$DIFF_FILE" ]]; then
  echo "No diff to apply: $DIFF_FILE"
  exit 0
fi

echo "Target diff: $DIFF_FILE"
if [[ "$ASSUME_YES" -ne 1 ]]; then
  read -r -p "Apply this diff to current working tree? [y/N] " answer
  case "$answer" in
    y|Y|yes|YES)
      ;;
    *)
      echo "Canceled."
      exit 0
      ;;
  esac
fi

git -C "$ROOT_DIR" apply --check --3way "$DIFF_FILE"
git -C "$ROOT_DIR" apply --3way "$DIFF_FILE"

echo "Applied diff successfully."

UTC_NOW="$(date -u +"%Y%m%d-%H%M%S")"
if [[ -f "$WORKLOG_FILE" ]]; then
  echo "| $UTC_NOW | $TASK_ID | apply diff | n/a | yes | source: $DIFF_FILE |" >> "$WORKLOG_FILE"
fi
