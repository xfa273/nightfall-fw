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
output_root=${2:-sessions/hfr-tests/pixel8/manual-runs}
package_name=com.nightfall.hfrrecorder
expected_schema=nightfall_android_hfr_recording_v1
expected_version_code=22
expected_version_name=0.5.8
adb_command=${ADB:-}
python_command=${PYTHON:-}

if [ -z "$adb_command" ]; then
  adb_command=$(command -v adb || true)
fi
if [ -n "$adb_command" ] && [ ! -x "$adb_command" ]; then
  adb_command=$(command -v "$adb_command" || true)
fi
if [ -z "$python_command" ]; then
  python_command=$(command -v python3 || true)
fi
if [ -n "$python_command" ] && [ ! -x "$python_command" ]; then
  python_command=$(command -v "$python_command" || true)
fi
if [ -z "$adb_command" ] || [ ! -x "$adb_command" ]; then
  echo "[HFR-MANUAL][ERROR] adb not found; set ADB" >&2
  exit 2
fi
if [ -z "$python_command" ] || [ ! -x "$python_command" ]; then
  echo "[HFR-MANUAL][ERROR] python3 not found; set PYTHON" >&2
  exit 2
fi
if [ "$("$adb_command" -s "$serial" get-state 2>/dev/null || true)" \
  != "device" ]; then
  echo "[HFR-MANUAL][ERROR] device is not ready: $serial" >&2
  exit 2
fi

package_dump=$(
  "$adb_command" -s "$serial" shell dumpsys package "$package_name" \
    2>/dev/null | tr -d '\r' || true
)
version_code=$(
  printf '%s\n' "$package_dump" \
    | sed -n 's/^[[:space:]]*versionCode=\([0-9][0-9]*\).*/\1/p' \
    | head -n 1
)
version_name=$(
  printf '%s\n' "$package_dump" \
    | sed -n 's/^[[:space:]]*versionName=//p' \
    | head -n 1
)
if [ "$version_code" != "$expected_version_code" ] \
  || [ "$version_name" != "$expected_version_name" ]; then
  echo "[HFR-MANUAL][ERROR] expected app $expected_version_name" \
    "($expected_version_code), found $version_name ($version_code)" >&2
  exit 2
fi

device_model=$(
  "$adb_command" -s "$serial" shell getprop ro.product.model | tr -d '\r'
)
safe_model=$(printf '%s' "$device_model" | sed 's/[^A-Za-z0-9._-]/_/g')
safe_serial=$(printf '%s' "$serial" | sed 's/[^A-Za-z0-9._-]/_/g')
mkdir -p "$output_root"

run_names=$(
  "$adb_command" -s "$serial" shell run-as "$package_name" \
    find files/manual_runs -mindepth 1 -maxdepth 1 -type d -print \
    2>/dev/null \
    | tr -d '\r' \
    | sed 's|.*/||' \
    | sort || true
)
if [ -z "$run_names" ]; then
  echo "[HFR-MANUAL] No retained Pixel-started runs found."
  exit 0
fi

collected=0
skipped=0
diagnostic=0
for run_name in $run_names; do
  case "$run_name" in
    ''|*[!A-Za-z0-9._-]*)
      echo "[HFR-MANUAL][ERROR] unsafe run directory: $run_name" >&2
      exit 2
      ;;
  esac
  session_dir=$output_root/${safe_model}_${safe_serial}_${run_name}
  report_path=$session_dir/hfr_report.json
  if [ -s "$report_path" ] \
    && "$python_command" - "$report_path" "$expected_schema" "$run_name" <<'PY'
import json
import pathlib
import sys

report = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
if report.get("schema") != sys.argv[2] or report.get("record_nonce") != sys.argv[3]:
    raise SystemExit(1)
PY
  then
    echo "[HFR-MANUAL] Already collected: $run_name"
    skipped=$((skipped + 1))
    continue
  fi
  if [ -e "$session_dir" ]; then
    echo "[HFR-MANUAL][ERROR] output exists but does not match: $session_dir" >&2
    exit 2
  fi
  mkdir -p "$session_dir"
  remote_root=files/manual_runs/$run_name
  "$adb_command" -s "$serial" exec-out run-as "$package_name" \
    cat "$remote_root/hfr_report.json" > "$report_path"
  status=$(
    "$python_command" - "$report_path" "$expected_schema" "$run_name" <<'PY'
import json
import pathlib
import sys

report = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
if report.get("schema") != sys.argv[2]:
    raise SystemExit("unexpected report schema")
if report.get("record_nonce") != sys.argv[3]:
    raise SystemExit("report nonce does not match directory")
print(report.get("status", "missing"))
PY
  )
  for artifact in capture_results.jsonl encoder_samples.jsonl; do
    "$adb_command" -s "$serial" exec-out run-as "$package_name" \
      cat "$remote_root/$artifact" > "$session_dir/$artifact"
  done
  if [ "$status" = "complete" ]; then
    for artifact in capture_results.jsonl encoder_samples.jsonl; do
      if [ ! -s "$session_dir/$artifact" ]; then
        echo "[HFR-MANUAL][ERROR] empty $artifact for $run_name" >&2
        exit 2
      fi
    done
    "$adb_command" -s "$serial" exec-out run-as "$package_name" \
      cat "$remote_root/hfr_capture.mp4" > "$session_dir/hfr_capture.mp4"
    if [ ! -s "$session_dir/hfr_capture.mp4" ]; then
      echo "[HFR-MANUAL][ERROR] empty video for $run_name" >&2
      exit 2
    fi
    if command -v ffprobe >/dev/null 2>&1; then
      ffprobe -v error -count_frames -select_streams v:0 \
        -show_entries \
        stream=codec_name,width,height,r_frame_rate,avg_frame_rate,duration,nb_read_frames \
        -of json "$session_dir/hfr_capture.mp4" \
        > "$session_dir/ffprobe_stream.json"
    fi
    collected=$((collected + 1))
    echo "[HFR-MANUAL] Collected complete run: $run_name"
  else
    diagnostic=$((diagnostic + 1))
    echo "[HFR-MANUAL] Collected $status diagnostic: $run_name"
  fi
done

echo "[HFR-MANUAL] Complete: collected=$collected" \
  "diagnostic=$diagnostic skipped=$skipped output=$output_root"
