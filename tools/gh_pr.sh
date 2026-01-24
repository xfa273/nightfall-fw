#!/bin/bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  tools/gh_pr.sh [--base <branch>] [--draft] [--title <title>] [--body <body>]
                [--merge] [--auto] [--method <merge|squash|rebase>] [--delete-branch]

Examples:
  # push current branch, create PR (if not exists)
  tools/gh_pr.sh

  # create PR and enable auto-merge (merge commit)
  tools/gh_pr.sh --merge --auto --method merge

Notes:
  - Requires GitHub CLI (gh): https://cli.github.com/
  - Requires authentication: gh auth login
USAGE
}

BASE="main"
DRAFT=0
TITLE=""
BODY=""
DO_MERGE=0
AUTO_MERGE=0
METHOD="merge"
DELETE_BRANCH=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --base) BASE="$2"; shift 2;;
    --draft) DRAFT=1; shift;;
    --title) TITLE="$2"; shift 2;;
    --body) BODY="$2"; shift 2;;
    --merge) DO_MERGE=1; shift;;
    --auto) AUTO_MERGE=1; shift;;
    --method) METHOD="$2"; shift 2;;
    --delete-branch) DELETE_BRANCH=1; shift;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown arg: $1"; usage; exit 2;;
  esac
done

if ! command -v gh >/dev/null 2>&1; then
  echo "Error: gh not found. Install: brew install gh"
  exit 1
fi

if ! gh auth status >/dev/null 2>&1; then
  echo "Error: gh is not authenticated. Run: gh auth login"
  exit 1
fi

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Error: not a git repository."
  exit 1
fi

HEAD_BRANCH="$(git rev-parse --abbrev-ref HEAD)"
if [[ "$HEAD_BRANCH" == "$BASE" ]]; then
  echo "Error: current branch is base branch ('$BASE'). Create a feature/fix branch first."
  exit 1
fi

if [[ -n "$(git status --porcelain)" ]]; then
  echo "Error: working tree is dirty. Commit or stash changes first."
  exit 1
fi

# Push current branch (create upstream if needed)
git push -u origin HEAD

PR_URL=""
if PR_URL=$(gh pr view --head "$HEAD_BRANCH" --base "$BASE" --json url -q .url 2>/dev/null); then
  echo "PR already exists: $PR_URL"
else
  CREATE_ARGS=(--base "$BASE" --head "$HEAD_BRANCH")
  if [[ "$DRAFT" -eq 1 ]]; then
    CREATE_ARGS+=(--draft)
  fi
  if [[ -n "$TITLE" ]]; then
    CREATE_ARGS+=(--title "$TITLE")
  fi
  if [[ -n "$BODY" ]]; then
    CREATE_ARGS+=(--body "$BODY")
  else
    CREATE_ARGS+=(--fill)
  fi

  PR_URL=$(gh pr create "${CREATE_ARGS[@]}")
  echo "Created PR: $PR_URL"
fi

if [[ "$DO_MERGE" -eq 1 ]]; then
  MERGE_FLAG="--merge"
  case "$METHOD" in
    merge) MERGE_FLAG="--merge";;
    squash) MERGE_FLAG="--squash";;
    rebase) MERGE_FLAG="--rebase";;
    *) echo "Error: invalid --method '$METHOD' (merge|squash|rebase)"; exit 2;;
  esac

  MERGE_ARGS=($MERGE_FLAG)
  if [[ "$AUTO_MERGE" -eq 1 ]]; then
    MERGE_ARGS+=(--auto)
  fi
  if [[ "$DELETE_BRANCH" -eq 1 ]]; then
    MERGE_ARGS+=(--delete-branch)
  fi

  echo "Merging PR ($METHOD)..."
  gh pr merge "${MERGE_ARGS[@]}" "$PR_URL"
fi
