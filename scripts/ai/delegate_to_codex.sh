#!/bin/bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/ai/delegate_to_codex.sh --title <title> [options]

Required:
  --title <title>               Human-readable task title

Prompt input (one of these is required):
  --prompt <text>               Task prompt text
  --prompt-file <path>          Path to prompt markdown/text file

Optional:
  --task-id <id>                Stable task id (default: utc timestamp + title slug)
  --targets <glob_or_list>      Target files/directories hint for Codex
  --success-criteria <text>     Success criteria summary
  --model <model>               codex model name
  --profile <profile>           codex profile name
  --sandbox <mode>              read-only|workspace-write|danger-full-access (default: workspace-write)
  --keep-worktree               Keep worktree after execution (default: keep)
  --cleanup-worktree            Remove worktree after execution
  --auto-apply                  Apply generated diff to current working tree
  -h, --help                    Show this help

Examples:
  scripts/ai/delegate_to_codex.sh \
    --title "Refactor trace init" \
    --prompt-file /tmp/task.md \
    --targets "platform/trace/**" \
    --success-criteria "Debug-stm32f405/413 build success"

  scripts/ai/delegate_to_codex.sh \
    --title "Fix nvm schema guard" \
    --prompt "nvm読み込み時の境界チェックを追加し、関連テストを実行"
USAGE
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Error: required command not found: $1" >&2
    exit 1
  fi
}

slugify() {
  echo "$1" \
    | tr '[:upper:]' '[:lower:]' \
    | sed -E 's/[^a-z0-9]+/-/g; s/^-+//; s/-+$//' \
    | cut -c1-48
}

TASK_TITLE=""
TASK_ID=""
PROMPT_TEXT=""
PROMPT_FILE=""
TARGETS=""
SUCCESS_CRITERIA=""
MODEL=""
PROFILE=""
SANDBOX_MODE="workspace-write"
KEEP_WORKTREE=1
AUTO_APPLY=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --title)
      TASK_TITLE="$2"
      shift 2
      ;;
    --task-id)
      TASK_ID="$2"
      shift 2
      ;;
    --prompt)
      PROMPT_TEXT="$2"
      shift 2
      ;;
    --prompt-file)
      PROMPT_FILE="$2"
      shift 2
      ;;
    --targets)
      TARGETS="$2"
      shift 2
      ;;
    --success-criteria)
      SUCCESS_CRITERIA="$2"
      shift 2
      ;;
    --model)
      MODEL="$2"
      shift 2
      ;;
    --profile)
      PROFILE="$2"
      shift 2
      ;;
    --sandbox)
      SANDBOX_MODE="$2"
      shift 2
      ;;
    --keep-worktree)
      KEEP_WORKTREE=1
      shift
      ;;
    --cleanup-worktree)
      KEEP_WORKTREE=0
      shift
      ;;
    --auto-apply)
      AUTO_APPLY=1
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

if [[ -z "$TASK_TITLE" ]]; then
  echo "Error: --title is required" >&2
  exit 2
fi

if [[ -z "$PROMPT_TEXT" && -z "$PROMPT_FILE" ]]; then
  echo "Error: either --prompt or --prompt-file is required" >&2
  exit 2
fi

if [[ -n "$PROMPT_TEXT" && -n "$PROMPT_FILE" ]]; then
  echo "Error: use either --prompt or --prompt-file, not both" >&2
  exit 2
fi

case "$SANDBOX_MODE" in
  read-only|workspace-write|danger-full-access)
    ;;
  *)
    echo "Error: invalid --sandbox value: $SANDBOX_MODE" >&2
    exit 2
    ;;
esac

require_cmd git
require_cmd codex

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Error: this script must be run inside a git repository" >&2
  exit 1
fi

ROOT_DIR="$(git rev-parse --show-toplevel)"
AI_DOCS_DIR="$ROOT_DIR/docs/ai"
HANDOFF_DIR="$AI_DOCS_DIR/HANDOFFS"
WORKLOG_FILE="$AI_DOCS_DIR/WORKLOG.md"
WORKTREE_BASE="$ROOT_DIR/.ai/worktrees"
SCRIPTS_DIR="$ROOT_DIR/scripts/ai"
ROOT_AGENTS_FILE="$ROOT_DIR/AGENTS.md"
ROOT_STATE_FILE="$ROOT_DIR/docs/ai/STATE.md"
ROOT_POLICY_FILE="$ROOT_DIR/docs/NIGHTFALL_FW_DEV_POLICY.md"

mkdir -p "$HANDOFF_DIR" "$WORKTREE_BASE" "$SCRIPTS_DIR"

if [[ ! -f "$WORKLOG_FILE" ]]; then
  cat > "$WORKLOG_FILE" <<'EOF'
# AI Worklog

| UTC Timestamp | Task ID | Title | Codex Exit | Applied | Notes |
| --- | --- | --- | --- | --- | --- |
EOF
fi

if [[ -n "$PROMPT_FILE" ]]; then
  if [[ ! -f "$PROMPT_FILE" ]]; then
    echo "Error: prompt file not found: $PROMPT_FILE" >&2
    exit 1
  fi
  PROMPT_TEXT="$(cat "$PROMPT_FILE")"
fi

TIMESTAMP_UTC="$(date -u +"%Y%m%d-%H%M%S")"
if [[ -z "$TASK_ID" ]]; then
  TASK_ID="${TIMESTAMP_UTC}-$(slugify "$TASK_TITLE")"
fi

if [[ -z "$TASK_ID" ]]; then
  echo "Error: failed to generate task id" >&2
  exit 1
fi

BASE_BRANCH="$(git -C "$ROOT_DIR" rev-parse --abbrev-ref HEAD)"
BASE_SHA="$(git -C "$ROOT_DIR" rev-parse HEAD)"
BRANCH_NAME="codex/$TASK_ID"
WORKTREE_DIR="$WORKTREE_BASE/$TASK_ID"

if [[ -e "$WORKTREE_DIR" ]]; then
  echo "Error: worktree already exists: $WORKTREE_DIR" >&2
  exit 1
fi

PACKET_FILE="$HANDOFF_DIR/${TASK_ID}-packet.md"
PROMPT_MERGED_FILE="$HANDOFF_DIR/${TASK_ID}-codex-prompt.md"
RAW_LOG_FILE="$HANDOFF_DIR/${TASK_ID}-raw.log"
LAST_MESSAGE_FILE="$HANDOFF_DIR/${TASK_ID}-last-message.md"
RESULT_FILE="$HANDOFF_DIR/${TASK_ID}-result.md"
META_FILE="$HANDOFF_DIR/${TASK_ID}-meta.env"
DIFF_FILE="$HANDOFF_DIR/${TASK_ID}.diff"
CHANGED_FILES_FILE="$HANDOFF_DIR/${TASK_ID}-changed-files.txt"

ROOT_STATUS="$(git -C "$ROOT_DIR" status --short || true)"

cat > "$PACKET_FILE" <<EOF
# Task Packet: $TASK_ID

## Title
$TASK_TITLE

## Created (UTC)
$TIMESTAMP_UTC

## Base Branch / SHA
- branch: $BASE_BRANCH
- sha: $BASE_SHA

## Targets
${TARGETS:-N/A}

## Success Criteria
${SUCCESS_CRITERIA:-N/A}

## Root Working Tree Status (at dispatch)
\`\`\`
${ROOT_STATUS:-clean}
\`\`\`

## User Prompt
\`\`\`
$PROMPT_TEXT
\`\`\`
EOF

cat > "$PROMPT_MERGED_FILE" <<EOF
You are Codex CLI running as a delegated worker for the nightfall-fw repository.

Follow these constraints strictly:
1. Read and obey AGENTS.md in repository root.
2. Use docs/NIGHTFALL_FW_DEV_POLICY.md as the primary policy.
3. Focus only on the requested task; do not perform unrelated refactors.
4. Do not run git commit/push.
5. Run relevant local verification commands when possible.
6. Provide a concise final summary with:
   - changed files
   - what was validated (build/test)
   - remaining risks / next actions

Important context path fallback:
- Worktree root: $WORKTREE_DIR
- Authoritative root (added via --add-dir): $ROOT_DIR
- If AGENTS.md or docs/ai/STATE.md are missing in worktree, read:
  - $ROOT_AGENTS_FILE
  - $ROOT_STATE_FILE
  - $ROOT_POLICY_FILE

Task metadata:
- task_id: $TASK_ID
- title: $TASK_TITLE
- base_branch: $BASE_BRANCH
- base_sha: $BASE_SHA
- targets: ${TARGETS:-N/A}
- success_criteria: ${SUCCESS_CRITERIA:-N/A}

Primary task instructions:
$PROMPT_TEXT
EOF

if ! codex login status >/dev/null 2>&1; then
  echo "Warning: codex login status check failed. You may need to run: codex login" >&2
fi

echo "[delegate_to_codex] Creating worktree: $WORKTREE_DIR"
git -C "$ROOT_DIR" worktree add -b "$BRANCH_NAME" "$WORKTREE_DIR" HEAD >/dev/null

CODEX_ARGS=(exec -C "$WORKTREE_DIR" --sandbox "$SANDBOX_MODE" --add-dir "$ROOT_DIR" --output-last-message "$LAST_MESSAGE_FILE")
if [[ -n "$MODEL" ]]; then
  CODEX_ARGS+=(--model "$MODEL")
fi
if [[ -n "$PROFILE" ]]; then
  CODEX_ARGS+=(--profile "$PROFILE")
fi
CODEX_ARGS+=(-)

echo "[delegate_to_codex] Running Codex..."
set +e
codex "${CODEX_ARGS[@]}" < "$PROMPT_MERGED_FILE" > "$RAW_LOG_FILE" 2>&1
CODEX_EXIT=$?
set -e

git -C "$WORKTREE_DIR" diff --name-only > "$CHANGED_FILES_FILE"
git -C "$WORKTREE_DIR" diff --binary > "$DIFF_FILE"

SAFE_TITLE="${TASK_TITLE//\"/\\\"}"
cat > "$META_FILE" <<EOF
TASK_ID="$TASK_ID"
TASK_TITLE="$SAFE_TITLE"
CREATED_UTC="$TIMESTAMP_UTC"
BASE_BRANCH="$BASE_BRANCH"
BASE_SHA="$BASE_SHA"
WORKTREE_DIR="$WORKTREE_DIR"
BRANCH_NAME="$BRANCH_NAME"
PACKET_FILE="$PACKET_FILE"
PROMPT_FILE="$PROMPT_MERGED_FILE"
RAW_LOG_FILE="$RAW_LOG_FILE"
LAST_MESSAGE_FILE="$LAST_MESSAGE_FILE"
RESULT_FILE="$RESULT_FILE"
DIFF_FILE="$DIFF_FILE"
CHANGED_FILES_FILE="$CHANGED_FILES_FILE"
CODEX_EXIT="$CODEX_EXIT"
ROOT_DIR="$ROOT_DIR"
EOF

if [[ -x "$SCRIPTS_DIR/summarize_codex_result.sh" ]]; then
  "$SCRIPTS_DIR/summarize_codex_result.sh" --task-id "$TASK_ID" || true
fi

TABLE_TITLE="${TASK_TITLE//|/\\|}"
echo "| $TIMESTAMP_UTC | $TASK_ID | $TABLE_TITLE | $CODEX_EXIT | no | handoff: $RESULT_FILE |" >> "$WORKLOG_FILE"

if [[ "$AUTO_APPLY" -eq 1 ]]; then
  if [[ -x "$SCRIPTS_DIR/maybe_apply_codex_result.sh" ]]; then
    "$SCRIPTS_DIR/maybe_apply_codex_result.sh" --task-id "$TASK_ID" --yes
  else
    echo "[delegate_to_codex] Skip auto-apply: maybe_apply_codex_result.sh not executable"
  fi
fi

if [[ "$KEEP_WORKTREE" -eq 0 ]]; then
  echo "[delegate_to_codex] Removing worktree: $WORKTREE_DIR"
  git -C "$ROOT_DIR" worktree remove "$WORKTREE_DIR" --force >/dev/null
fi

echo "[delegate_to_codex] done"
echo "  task_id: $TASK_ID"
echo "  codex_exit: $CODEX_EXIT"
echo "  packet: $PACKET_FILE"
echo "  result: $RESULT_FILE"
echo "  diff: $DIFF_FILE"
echo "  raw_log: $RAW_LOG_FILE"

exit "$CODEX_EXIT"
