#!/bin/sh
set -eu

usage() {
  echo "Usage: $0 SERIAL [OUTPUT_DIR]" >&2
}

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  usage
  exit 2
fi

serial=$1
output_root=${2:-/tmp/nightfall-stock-slowmo}
timeout_seconds=${STOCK_CAPTURE_TIMEOUT_SECONDS:-300}
stable_polls_required=${STOCK_CAPTURE_STABLE_POLLS:-3}
adb_command=${ADB:-}
python_command=${PYTHON:-}

case "$timeout_seconds" in
  ''|*[!0-9]*)
    echo "[STOCK-SLOWMO][ERROR] timeout must be an integer" >&2
    exit 2
    ;;
esac
case "$stable_polls_required" in
  ''|*[!0-9]*)
    echo "[STOCK-SLOWMO][ERROR] stable polls must be an integer" >&2
    exit 2
    ;;
esac
if [ "$timeout_seconds" -lt 10 ] || [ "$stable_polls_required" -lt 2 ]; then
  echo "[STOCK-SLOWMO][ERROR] timeout/stable-poll setting is too small" >&2
  exit 2
fi

if [ -z "$adb_command" ]; then
  adb_command=$(command -v adb || true)
fi
if [ -n "$adb_command" ] && [ ! -x "$adb_command" ]; then
  adb_command=$(command -v "$adb_command" || true)
fi
if [ -z "$adb_command" ] || [ ! -x "$adb_command" ]; then
  echo "[STOCK-SLOWMO][ERROR] adb not found; set ADB" >&2
  exit 2
fi

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repo_root=$(CDPATH= cd -- "$script_dir/../../.." && pwd)
if [ -z "$python_command" ] && [ -x "$repo_root/.venv-vision/bin/python" ]; then
  python_command=$repo_root/.venv-vision/bin/python
fi
if [ -z "$python_command" ]; then
  python_command=$(command -v python3 || true)
fi
if [ -n "$python_command" ] && [ ! -x "$python_command" ]; then
  python_command=$(command -v "$python_command" || true)
fi
if [ -z "$python_command" ] || [ ! -x "$python_command" ]; then
  echo "[STOCK-SLOWMO][ERROR] Python not found; set PYTHON" >&2
  exit 2
fi
if ! command -v ffprobe >/dev/null 2>&1; then
  echo "[STOCK-SLOWMO][ERROR] ffprobe is required" >&2
  exit 2
fi

device_state=$("$adb_command" -s "$serial" get-state 2>/dev/null || true)
if [ "$device_state" != "device" ]; then
  echo "[STOCK-SLOWMO][ERROR] device is not ready: $serial" >&2
  exit 2
fi

newest_stock_video() {
  "$adb_command" -s "$serial" shell \
    'for file in /sdcard/DCIM/Camera/*.mp4 \
      /sdcard/DCIM/Camera/*.MP4; do
       if [ -f "$file" ]; then
         stat -c "%Y|%s|%n" "$file"
       fi
     done' \
    | tr -d '\r' \
    | sort -t '|' -k1,1nr \
    | head -n 1
}

baseline=$(newest_stock_video)
baseline_path=$(printf '%s\n' "$baseline" | cut -d '|' -f 3-)
started_epoch=$(date +%s)

device_model=$(
  "$adb_command" -s "$serial" shell getprop ro.product.model \
    | tr -d '\r'
)
safe_model=$(printf '%s' "$device_model" | sed 's/[^A-Za-z0-9._-]/_/g')
safe_serial=$(printf '%s' "$serial" | sed 's/[^A-Za-z0-9._-]/_/g')
collected_at=$(date -u '+%Y%m%dT%H%M%SZ')
session_dir=$output_root/${safe_model}_${safe_serial}_${collected_at}
if [ -e "$session_dir" ]; then
  echo "[STOCK-SLOWMO][ERROR] output already exists: $session_dir" >&2
  exit 2
fi
mkdir -p "$session_dir"

"$adb_command" -s "$serial" shell am force-stop com.android.camera
"$adb_command" -s "$serial" shell am start -W \
  -n com.android.camera/.Camera >/dev/null

echo "[STOCK-SLOWMO] Xiaomi Camera is open on $device_model."
echo "[STOCK-SLOWMO] Select Slow motion, choose 120 or 240 fps,"
echo "[STOCK-SLOWMO] then record for several seconds and stop."
echo "[STOCK-SLOWMO] Waiting up to ${timeout_seconds}s for a new MP4..."

deadline=$((started_epoch + timeout_seconds))
candidate_path=
candidate_size=
stable_polls=0
while :; do
  newest=$(newest_stock_video)
  newest_mtime=$(printf '%s\n' "$newest" | cut -d '|' -f 1)
  newest_size=$(printf '%s\n' "$newest" | cut -d '|' -f 2)
  newest_path=$(printf '%s\n' "$newest" | cut -d '|' -f 3-)
  if [ -n "$newest_path" ] \
    && [ "$newest_path" != "$baseline_path" ] \
    && [ "$newest_mtime" -ge "$started_epoch" ]
  then
    if [ "$newest_path" = "$candidate_path" ] \
      && [ "$newest_size" = "$candidate_size" ] \
      && [ "$newest_size" -gt 0 ]
    then
      stable_polls=$((stable_polls + 1))
    else
      candidate_path=$newest_path
      candidate_size=$newest_size
      stable_polls=0
    fi
    if [ "$stable_polls" -ge "$stable_polls_required" ]; then
      break
    fi
  fi
  if [ "$(date +%s)" -ge "$deadline" ]; then
    echo "[STOCK-SLOWMO][ERROR] no completed new MP4 was detected" >&2
    exit 2
  fi
  sleep 1
done

video_name=$(basename "$candidate_path")
video_path=$session_dir/$video_name
"$adb_command" -s "$serial" pull "$candidate_path" "$video_path" >/dev/null
if [ ! -s "$video_path" ]; then
  echo "[STOCK-SLOWMO][ERROR] pulled video is empty" >&2
  exit 2
fi

ffprobe_json=$session_dir/ffprobe_full.json
ffprobe -v error -count_frames -show_streams -show_format \
  -of json "$video_path" > "$ffprobe_json"

capture_fps=$(
  "$python_command" - "$ffprobe_json" <<'PY'
import json
import pathlib
import sys

payload = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
tags = payload.get("format", {}).get("tags", {})
value = tags.get("com.android.capture.fps")
if value is None:
    raise SystemExit(2)
number = float(value)
if number < 120:
    raise SystemExit(2)
print(f"{number:g}")
PY
) || {
  echo "[STOCK-SLOWMO][ERROR] MP4 has no valid high-speed capture tag" >&2
  exit 2
}

timing_report=$session_dir/timing_qa.json
"$python_command" "$repo_root/tools/vision/video_timing_qa.py" \
  "$video_path" \
  --expected-fps "$capture_fps" \
  --report-json "$timing_report"

camera_dump=$(
  "$adb_command" -s "$serial" shell dumpsys package com.android.camera \
    | tr -d '\r'
)
camera_version=$(
  printf '%s\n' "$camera_dump" \
    | sed -n 's/^[[:space:]]*versionName=//p' \
    | head -n 1
)
report_path=$session_dir/stock_capture_report.json
"$python_command" - \
  "$ffprobe_json" "$timing_report" "$report_path" \
  "$serial" "$device_model" "$camera_version" \
  "$candidate_path" "$video_name" "$capture_fps" <<'PY'
import datetime
import json
import pathlib
import sys

(
    ffprobe_path,
    timing_path,
    output_path,
    serial,
    model,
    camera_version,
    remote_path,
    video_name,
    capture_fps,
) = sys.argv[1:]
ffprobe = json.loads(pathlib.Path(ffprobe_path).read_text(encoding="utf-8"))
timing = json.loads(pathlib.Path(timing_path).read_text(encoding="utf-8"))
report = {
    "schema": "nightfall_xiaomi_stock_slowmo_capture_v1",
    "generated_at_utc": datetime.datetime.now(
        datetime.timezone.utc
    ).isoformat(),
    "device": {
        "adb_serial": serial,
        "model": model,
        "stock_camera_version": camera_version,
    },
    "capture": {
        "method": "Xiaomi stock Camera slow-motion mode",
        "capture_fps": float(capture_fps),
        "remote_path": remote_path,
        "local_video": video_name,
    },
    "ffprobe": ffprobe,
    "timing_qa": timing,
}
pathlib.Path(output_path).write_text(
    json.dumps(report, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
PY

echo "[STOCK-SLOWMO] Session: $session_dir"
echo "[STOCK-SLOWMO] Video: $video_path"
echo "[STOCK-SLOWMO] Capture FPS: $capture_fps"
echo "[STOCK-SLOWMO] Report: $report_path"
