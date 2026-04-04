#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_APP="$SCRIPT_DIR/web_visualizer.py"
VENV_PY="$SCRIPT_DIR/venv/bin/python"

export SYSTEM_VERSION_COMPAT=0
export STREAMLIT_BROWSER_GATHER_USAGE_STATS=false

PY_BIN="$VENV_PY"
if [[ ! -x "$PY_BIN" ]]; then
  if command -v python3 >/dev/null 2>&1; then
    PY_BIN="python3"
  else
    echo "[ERROR] python3 not found." >&2
    exit 1
  fi
fi

# 互換: 旧来の呼び出し（--web）を許容。指定が無くてもWeb版を起動する。
if [[ $# -gt 0 && "$1" == "--web" ]]; then
  shift
fi

if [[ ! -f "$WEB_APP" ]]; then
  echo "[ERROR] Web visualizer script not found: $WEB_APP" >&2
  exit 1
fi

if ! env SYSTEM_VERSION_COMPAT=0 "$PY_BIN" -c 'import streamlit, plotly' >/dev/null 2>&1; then
  echo "[ERROR] Web dependencies are missing (streamlit/plotly)." >&2
  echo "[INFO] Python used: $PY_BIN" >&2
  echo "[HINT] Install deps with:" >&2
  echo "       $PY_BIN -m pip install -r \"$SCRIPT_DIR/requirements.txt\"" >&2
  echo "[HINT] If pip is missing:" >&2
  echo "       $PY_BIN -m ensurepip --upgrade" >&2
  exit 1
fi

if [[ $# -gt 0 && -f "$1" ]]; then
  export MICROMOUSE_WEB_DEFAULT_CSV="$1"
fi

exec env SYSTEM_VERSION_COMPAT=0 "$PY_BIN" -m streamlit run "$WEB_APP" --server.headless false --browser.gatherUsageStats false
