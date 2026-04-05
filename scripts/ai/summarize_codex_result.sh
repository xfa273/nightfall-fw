#!/bin/bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/ai/summarize_codex_result.sh --task-id <task-id>
USAGE
}

TASK_ID=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --task-id)
      TASK_ID="$2"
      shift 2
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
META_FILE="$HANDOFF_DIR/${TASK_ID}-meta.env"

if [[ ! -f "$META_FILE" ]]; then
  echo "Error: meta file not found: $META_FILE" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "$META_FILE"

if [[ -z "${RESULT_FILE:-}" ]]; then
  echo "Error: RESULT_FILE is missing in meta" >&2
  exit 1
fi

CHANGED_COUNT=0
if [[ -f "${CHANGED_FILES_FILE:-}" ]]; then
  CHANGED_COUNT="$(grep -c '.' "$CHANGED_FILES_FILE" || true)"
fi

LAST_MESSAGE_TAIL=""
if [[ -f "${LAST_MESSAGE_FILE:-}" ]]; then
  LAST_MESSAGE_TAIL="$(tail -n 120 "$LAST_MESSAGE_FILE")"
fi

RAW_LOG_TAIL=""
if [[ -f "${RAW_LOG_FILE:-}" ]]; then
  RAW_LOG_TAIL="$(tail -n 80 "$RAW_LOG_FILE")"
fi

cat > "$RESULT_FILE" <<EOF
# Codex Handoff Result: ${TASK_ID}

## Status
- codex_exit: ${CODEX_EXIT:-unknown}
- changed_files_count: ${CHANGED_COUNT}

## Paths
- packet: ${PACKET_FILE:-N/A}
- prompt: ${PROMPT_FILE:-N/A}
- raw_log: ${RAW_LOG_FILE:-N/A}
- last_message: ${LAST_MESSAGE_FILE:-N/A}
- diff: ${DIFF_FILE:-N/A}
- changed_files: ${CHANGED_FILES_FILE:-N/A}

## Changed Files
\`\`\`
$(cat "${CHANGED_FILES_FILE:-/dev/null}" 2>/dev/null || true)
\`\`\`

## Codex Last Message (tail)
\`\`\`
${LAST_MESSAGE_TAIL}
\`\`\`

## Raw Log (tail)
\`\`\`
${RAW_LOG_TAIL}
\`\`\`
EOF

echo "[summarize_codex_result] updated: $RESULT_FILE"
