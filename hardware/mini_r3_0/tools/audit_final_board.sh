#!/bin/zsh
set -euo pipefail

# Four-layer release wrapper (F.Cu/In1.Cu/In2.Cu/B.Cu).  A legacy two-layer
# candidate is still accepted for a normal audit; only the stack gate fails.
SCRIPT_DIR="${0:A:h}"
KI_PY="/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3.9"
REPO_ROOT="${SCRIPT_DIR:h:h:h}"
DEFAULT_PROJECT_DIR="$REPO_ROOT/hardware/mini_r3_0/cad/kicad/HM_Nightfall-mini-3a"
PROJECT_DIR="${MINI3_PROJECT_DIR:-$DEFAULT_PROJECT_DIR}"

if [[ ! -x "$KI_PY" ]]; then
  print -u2 "KiCad bundled Python not found: $KI_PY"
  exit 1
fi
if [[ ! -d "$PROJECT_DIR" ]]; then
  print -u2 "Project directory not found: $PROJECT_DIR"
  print -u2 "Set MINI3_PROJECT_DIR to the canonical KiCad project directory."
  exit 1
fi

exec "$KI_PY" "$SCRIPT_DIR/audit_final_board.py" \
  --project-dir "$PROJECT_DIR" "$@"
