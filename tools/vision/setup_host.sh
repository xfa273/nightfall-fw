#!/bin/sh
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_root=$(CDPATH= cd -- "$script_dir/../.." && pwd)
venv_dir=${NIGHTFALL_VISION_VENV:-"$repo_root/.venv-vision"}
python_bin=${PYTHON:-python3}

if ! command -v "$python_bin" >/dev/null 2>&1; then
  echo "[VISION-SETUP][ERROR] Python 3 was not found: $python_bin" >&2
  exit 2
fi
if ! command -v ffmpeg >/dev/null 2>&1 ||
   ! command -v ffprobe >/dev/null 2>&1; then
  echo "[VISION-SETUP][ERROR] ffmpeg and ffprobe are required" >&2
  exit 2
fi

"$python_bin" -m venv "$venv_dir"
"$venv_dir/bin/python" -m pip install \
  --disable-pip-version-check \
  -r "$script_dir/requirements.txt"

echo "[VISION-SETUP] Python: $venv_dir/bin/python"
