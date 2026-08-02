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
output_root=${2:-/tmp/nightfall-hfr-recordings}
package_name=com.nightfall.hfrrecorder
expected_schema=nightfall_android_hfr_recording_v1
expected_version_code=7
expected_version_name=0.3.4
camera_id=${HFR_CAMERA_ID:-0}
width=${HFR_WIDTH:-1920}
height=${HFR_HEIGHT:-1080}
fps=${HFR_FPS:-120}
duration_seconds=${HFR_DURATION_SECONDS:-5}
bitrate=${HFR_BITRATE:-40000000}
exposure_us=${HFR_EXPOSURE_US:-0}
iso=${HFR_ISO:-400}
enable_preview=${HFR_ENABLE_PREVIEW:-1}
optical_trigger=${HFR_OPTICAL_TRIGGER:-0}
optical_trigger_score=${HFR_OPTICAL_TRIGGER_SCORE:-180}
optical_trigger_hot_pixels=${HFR_OPTICAL_TRIGGER_HOT_PIXELS:-2}
optical_stop_tail_ms=${HFR_OPTICAL_STOP_TAIL_MS:-900}
record_timeout=${HFR_TIMEOUT_SECONDS:-90}
adb_command=${ADB:-}
python_command=${PYTHON:-}

for value_name in \
  width height fps duration_seconds bitrate exposure_us iso enable_preview \
  optical_trigger optical_trigger_score optical_trigger_hot_pixels \
  optical_stop_tail_ms record_timeout
do
  eval "value=\${$value_name}"
  case "$value" in
    ''|*[!0-9]*)
      echo "[HFR-RECORDER][ERROR] $value_name must be an integer" >&2
      exit 2
      ;;
  esac
done
if [ "$enable_preview" != "0" ] && [ "$enable_preview" != "1" ]; then
  echo "[HFR-RECORDER][ERROR] HFR_ENABLE_PREVIEW must be 0 or 1" >&2
  exit 2
fi
if [ "$optical_trigger" != "0" ] && [ "$optical_trigger" != "1" ]; then
  echo "[HFR-RECORDER][ERROR] HFR_OPTICAL_TRIGGER must be 0 or 1" >&2
  exit 2
fi
if [ "$enable_preview" = "1" ]; then
  enable_preview_boolean=true
else
  enable_preview_boolean=false
fi
if [ "$optical_trigger" = "1" ]; then
  optical_trigger_boolean=true
else
  optical_trigger_boolean=false
fi
if [ "$optical_trigger" = "1" ] && [ "$enable_preview" != "1" ]; then
  echo "[HFR-RECORDER][ERROR] optical trigger requires preview" >&2
  exit 2
fi
if [ "$optical_trigger_score" -lt 1 ] \
  || [ "$optical_trigger_hot_pixels" -lt 1 ]; then
  echo "[HFR-RECORDER][ERROR] optical trigger thresholds must be positive" >&2
  exit 2
fi
if [ "$optical_stop_tail_ms" -gt 5000 ]; then
  echo "[HFR-RECORDER][ERROR] optical stop tail must be <= 5000 ms" >&2
  exit 2
fi
if [ "$width" -lt 320 ] || [ "$height" -lt 240 ]; then
  echo "[HFR-RECORDER][ERROR] recording dimensions are too small" >&2
  exit 2
fi
if [ "$fps" -lt 30 ] || [ "$fps" -gt 480 ]; then
  echo "[HFR-RECORDER][ERROR] HFR_FPS must be in 30..480" >&2
  exit 2
fi
if [ "$duration_seconds" -lt 1 ] || [ "$duration_seconds" -gt 60 ]; then
  echo "[HFR-RECORDER][ERROR] duration must be in 1..60 seconds" >&2
  exit 2
fi
if [ "$record_timeout" -le "$duration_seconds" ]; then
  echo "[HFR-RECORDER][ERROR] timeout must exceed duration" >&2
  exit 2
fi

if [ -z "$adb_command" ]; then
  adb_command=$(command -v adb || true)
fi
if [ -n "$adb_command" ] && [ ! -x "$adb_command" ]; then
  adb_command=$(command -v "$adb_command" || true)
fi
if [ -z "$adb_command" ] || [ ! -x "$adb_command" ]; then
  echo "[HFR-RECORDER][ERROR] adb not found; set ADB" >&2
  exit 2
fi

if [ -z "$python_command" ]; then
  python_command=$(command -v python3 || true)
fi
if [ -n "$python_command" ] && [ ! -x "$python_command" ]; then
  python_command=$(command -v "$python_command" || true)
fi
if [ -z "$python_command" ] || [ ! -x "$python_command" ]; then
  echo "[HFR-RECORDER][ERROR] python3 not found; set PYTHON" >&2
  exit 2
fi

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
apk=$script_dir/recorder/build/outputs/apk/debug/recorder-debug.apk
if [ ! -f "$apk" ]; then
  echo "[HFR-RECORDER][ERROR] recorder APK is missing; run" >&2
  echo "  tools/vision/android_camera_probe/build_debug.sh" >&2
  exit 2
fi

device_state=$("$adb_command" -s "$serial" get-state 2>/dev/null || true)
if [ "$device_state" != "device" ]; then
  echo "[HFR-RECORDER][ERROR] device is not ready: $serial" >&2
  exit 2
fi
device_model=$(
  "$adb_command" -s "$serial" shell getprop ro.product.model \
    | tr -d '\r'
)
safe_model=$(printf '%s' "$device_model" | sed 's/[^A-Za-z0-9._-]/_/g')
safe_serial=$(printf '%s' "$serial" | sed 's/[^A-Za-z0-9._-]/_/g')
collected_at=$(date -u '+%Y%m%dT%H%M%SZ')
record_nonce=${safe_serial}-${collected_at}-$$
session_dir=$output_root/${safe_model}_${safe_serial}_${collected_at}
if [ -e "$session_dir" ]; then
  echo "[HFR-RECORDER][ERROR] output already exists: $session_dir" >&2
  exit 2
fi
mkdir -p "$session_dir"
temporary_report=$(mktemp "$session_dir/.hfr-report.XXXXXX")

cleanup() {
  if [ -n "${temporary_report:-}" ] && [ -e "$temporary_report" ]; then
    rm -f -- "$temporary_report"
  fi
}
trap cleanup EXIT HUP INT TERM

package_dump=$(
  "$adb_command" -s "$serial" shell dumpsys package "$package_name" \
    2>/dev/null | tr -d '\r' || true
)
current_version_code=$(
  printf '%s\n' "$package_dump" \
    | sed -n 's/^[[:space:]]*versionCode=\([0-9][0-9]*\).*/\1/p' \
    | head -n 1
)
current_version_name=$(
  printf '%s\n' "$package_dump" \
    | sed -n 's/^[[:space:]]*versionName=//p' \
    | head -n 1
)
if [ "$current_version_code" = "$expected_version_code" ] \
  && [ "$current_version_name" = "$expected_version_name" ]
then
  echo "[HFR-RECORDER] Using installed $expected_version_name on $device_model."
else
  echo "[HFR-RECORDER] Installing $expected_version_name on $device_model..."
  # Stop a foreground copy before package replacement.  Otherwise Android may
  # restore the old task without the recording extras while `install -r` races
  # the explicit launch below.
  "$adb_command" -s "$serial" shell am force-stop "$package_name" \
    >/dev/null 2>&1 || true
  "$adb_command" -s "$serial" install -r "$apk" >/dev/null
  package_dump=$(
    "$adb_command" -s "$serial" shell dumpsys package "$package_name" \
      | tr -d '\r'
  )
fi
installed_version_code=$(
  printf '%s\n' "$package_dump" \
    | sed -n 's/^[[:space:]]*versionCode=\([0-9][0-9]*\).*/\1/p' \
    | head -n 1
)
installed_version_name=$(
  printf '%s\n' "$package_dump" \
    | sed -n 's/^[[:space:]]*versionName=//p' \
    | head -n 1
)
if [ "$installed_version_code" != "$expected_version_code" ] \
  || [ "$installed_version_name" != "$expected_version_name" ]; then
  echo "[HFR-RECORDER][ERROR] recorder app version mismatch" >&2
  exit 2
fi

if ! printf '%s\n' "$package_dump" \
  | grep -q 'android.permission.CAMERA: granted=true'
then
  echo "[HFR-RECORDER][WARN] Allow camera access on the phone when prompted." >&2
fi

"$adb_command" -s "$serial" shell am force-stop "$package_name"
# Pixel's camera service may retain a just-closed constrained-HFR client for a
# short time, especially after package replacement or an interrupted armed
# session. Starting immediately can fail once with CameraDevice error 2
# (maximum cameras in use), so allow the HAL to publish the disconnect first.
sleep 2
"$adb_command" -s "$serial" shell am start -W \
  -n "$package_name/.MainActivity" \
  --ez auto_record true \
  --es record_nonce "$record_nonce" \
  --es camera_id "$camera_id" \
  --ei width "$width" \
  --ei height "$height" \
  --ei fps "$fps" \
  --ei duration_seconds "$duration_seconds" \
  --ei bitrate "$bitrate" \
  --ei exposure_us "$exposure_us" \
  --ei iso "$iso" \
  --ez enable_preview "$enable_preview_boolean" \
  --ez optical_trigger "$optical_trigger_boolean" \
  --ei optical_trigger_score "$optical_trigger_score" \
  --ei optical_trigger_hot_pixels "$optical_trigger_hot_pixels" \
  --ei optical_stop_tail_ms "$optical_stop_tail_ms" >/dev/null

if [ "$optical_trigger" = "1" ]; then
  echo "[HFR-RECORDER] Armed ${width}x${height}@${fps} for optical" \
    "START/STOP (max_recording=${duration_seconds}s," \
    "exposure_us=${exposure_us}, iso=${iso}, score=${optical_trigger_score}," \
    "hot_pixels=${optical_trigger_hot_pixels})."
  echo "[HFR-RECORDER] Wait about 2 seconds for WAIT_RISE_1_OF_3 on Pixel" \
    "before starting the mouse."
else
  echo "[HFR-RECORDER] Recording ${width}x${height}@${fps}" \
    "for ${duration_seconds}s (exposure_us=${exposure_us}, iso=${iso}," \
    "preview=${enable_preview_boolean})."
fi

report_has_expected_nonce() {
  "$python_command" - "$temporary_report" \
    "$expected_schema" "$record_nonce" <<'PY'
import json
import pathlib
import sys

try:
    report = json.loads(
        pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
    )
except (OSError, UnicodeError, json.JSONDecodeError):
    raise SystemExit(1)

if report.get("schema") != sys.argv[2]:
    raise SystemExit(1)
if report.get("record_nonce") != sys.argv[3]:
    raise SystemExit(1)
PY
}

record_deadline=$(($(date +%s) + record_timeout))
while :; do
  if "$adb_command" -s "$serial" exec-out \
    run-as "$package_name" cat "files/hfr_report.json" \
    > "$temporary_report" 2>/dev/null \
    && report_has_expected_nonce
  then
    break
  fi
  if [ "$(date +%s)" -ge "$record_deadline" ]; then
    echo "[HFR-RECORDER][ERROR] no fresh report after" \
      "$record_timeout seconds" >&2
    exit 2
  fi
  sleep 1
done

report_path=$session_dir/hfr_report.json
mv "$temporary_report" "$report_path"
temporary_report=

record_status=$(
  "$python_command" - "$report_path" <<'PY'
import json
import pathlib
import sys

report = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
print(report.get("status", "missing"))
if report.get("status") != "complete":
    print(
        f"[HFR-RECORDER][ERROR] {report.get('error')}",
        file=sys.stderr,
    )
PY
)
if [ "$record_status" != "complete" ]; then
  echo "[HFR-RECORDER] Diagnostic report: $report_path" >&2
  exit 2
fi

for artifact in \
  hfr_capture.mp4 capture_results.jsonl encoder_samples.jsonl
do
  "$adb_command" -s "$serial" exec-out \
    run-as "$package_name" cat "files/$artifact" \
    > "$session_dir/$artifact"
  if [ ! -s "$session_dir/$artifact" ]; then
    echo "[HFR-RECORDER][ERROR] empty artifact: $artifact" >&2
    exit 2
  fi
done

if command -v ffprobe >/dev/null 2>&1; then
  ffprobe -v error -count_frames -select_streams v:0 \
    -show_entries \
    stream=codec_name,width,height,r_frame_rate,avg_frame_rate,nb_read_frames \
    -of json "$session_dir/hfr_capture.mp4" \
    > "$session_dir/ffprobe_stream.json"
fi

echo "[HFR-RECORDER] Session: $session_dir"
echo "[HFR-RECORDER] Report: $report_path"
