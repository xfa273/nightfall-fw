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
output_dir=${2:-/tmp/nightfall-camera-probe}
package_name=com.nightfall.cameraprobe
report_path=files/camera_capabilities.json
expected_schema=nightfall_android_camera_probe_v2
expected_version_code=2
expected_version_name=0.2.0
probe_timeout=${PROBE_TIMEOUT_SECONDS:-120}
adb_command=${ADB:-}
python_command=${PYTHON:-}

case "$probe_timeout" in
  ''|*[!0-9]*)
    echo "[CAMERA-PROBE][ERROR] PROBE_TIMEOUT_SECONDS must be an integer" >&2
    exit 2
    ;;
esac
if [ "$probe_timeout" -le 0 ]; then
  echo "[CAMERA-PROBE][ERROR] PROBE_TIMEOUT_SECONDS must be positive" >&2
  exit 2
fi

if [ -z "$adb_command" ]; then
  adb_command=$(command -v adb || true)
fi
if [ -n "$adb_command" ] && [ ! -x "$adb_command" ]; then
  adb_command=$(command -v "$adb_command" || true)
fi
if [ -z "$adb_command" ] || [ ! -x "$adb_command" ]; then
  echo "[CAMERA-PROBE][ERROR] adb not found; set ADB" >&2
  exit 2
fi

if [ -z "$python_command" ]; then
  python_command=$(command -v python3 || true)
fi
if [ -n "$python_command" ] && [ ! -x "$python_command" ]; then
  python_command=$(command -v "$python_command" || true)
fi
if [ -z "$python_command" ] || [ ! -x "$python_command" ]; then
  echo "[CAMERA-PROBE][ERROR] python3 not found; set PYTHON" >&2
  exit 2
fi

device_state=$("$adb_command" -s "$serial" get-state 2>/dev/null || true)
if [ "$device_state" != "device" ]; then
  echo "[CAMERA-PROBE][ERROR] device is not ready: $serial" >&2
  exit 2
fi

device_model=$(
  "$adb_command" -s "$serial" shell getprop ro.product.model \
    | tr -d '\r'
)
if [ -z "$device_model" ]; then
  device_model=unknown-model
fi
device_fingerprint=$(
  "$adb_command" -s "$serial" shell getprop ro.build.fingerprint \
    | tr -d '\r'
)
if [ -z "$device_fingerprint" ]; then
  echo "[CAMERA-PROBE][ERROR] unable to read device fingerprint" >&2
  exit 2
fi

package_dump=$(
  "$adb_command" -s "$serial" shell dumpsys package "$package_name" \
    | tr -d '\r'
)
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
  echo "[CAMERA-PROBE][ERROR] expected probe app" \
    "$expected_version_name ($expected_version_code)," \
    "found ${installed_version_name:-none}" \
    "(${installed_version_code:-none})" >&2
  echo "Install the current app-debug.apk before collecting." >&2
  exit 2
fi

safe_model=$(printf '%s' "$device_model" | sed 's/[^A-Za-z0-9._-]/_/g')
safe_serial=$(printf '%s' "$serial" | sed 's/[^A-Za-z0-9._-]/_/g')
collected_at=$(date -u '+%Y%m%dT%H%M%SZ')
probe_nonce=${safe_serial}-${collected_at}-$$

mkdir -p "$output_dir"
output_path=$output_dir/${safe_model}_${safe_serial}_${collected_at}_camera_capabilities.json
if [ -e "$output_path" ]; then
  echo "[CAMERA-PROBE][ERROR] output already exists: $output_path" >&2
  exit 2
fi

temporary_path=$(mktemp "$output_dir/.camera-capabilities.XXXXXX")
cleanup() {
  if [ -n "${temporary_path:-}" ] && [ -e "$temporary_path" ]; then
    rm -f -- "$temporary_path"
  fi
}
trap cleanup EXIT HUP INT TERM

"$adb_command" -s "$serial" shell am force-stop "$package_name"
if ! "$adb_command" -s "$serial" shell am start -W \
  -n "$package_name/.MainActivity" \
  --es probe_nonce "$probe_nonce" > /dev/null
then
  echo "[CAMERA-PROBE][ERROR] unable to start the probe app" >&2
  exit 2
fi

echo "[CAMERA-PROBE] Probe started on $device_model."
echo "[CAMERA-PROBE] If prompted, unlock the phone and grant CAMERA."

report_has_expected_nonce() {
  "$python_command" - "$temporary_path" \
    "$expected_schema" "$probe_nonce" <<'PY'
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
if report.get("probe_nonce") != sys.argv[3]:
    raise SystemExit(1)
PY
}

probe_deadline=$(($(date +%s) + probe_timeout))
while :; do
  if "$adb_command" -s "$serial" exec-out \
    run-as "$package_name" cat "$report_path" \
    > "$temporary_path" 2>/dev/null \
    && report_has_expected_nonce
  then
    break
  fi
  if [ "$(date +%s)" -ge "$probe_deadline" ]; then
    echo "[CAMERA-PROBE][ERROR] no fresh report after" \
      "$probe_timeout seconds" >&2
    echo "Unlock the phone, grant CAMERA, then run the collector again." >&2
    exit 2
  fi
  sleep 1
done

"$python_command" - "$temporary_path" \
  "$device_model" "$device_fingerprint" "$expected_schema" "$probe_nonce" \
  "$expected_version_code" "$expected_version_name" <<'PY'
import json
import pathlib
import sys
from datetime import datetime

path = pathlib.Path(sys.argv[1])
expected_model = sys.argv[2]
expected_fingerprint = sys.argv[3]
expected_schema = sys.argv[4]
expected_nonce = sys.argv[5]
expected_version_code = int(sys.argv[6])
expected_version_name = sys.argv[7]

try:
    report = json.loads(path.read_text(encoding="utf-8"))
except (OSError, UnicodeError, json.JSONDecodeError) as error:
    raise SystemExit(f"[CAMERA-PROBE][ERROR] invalid JSON: {error}")

if report.get("schema") != expected_schema:
    raise SystemExit(
        "[CAMERA-PROBE][ERROR] unexpected or stale schema: "
        f"{report.get('schema')!r}"
    )
if report.get("probe_nonce") != expected_nonce:
    raise SystemExit("[CAMERA-PROBE][ERROR] probe nonce mismatch")
generated_at = report.get("generated_at_utc")
if not generated_at:
    raise SystemExit("[CAMERA-PROBE][ERROR] generated_at_utc is missing")
try:
    generated_datetime = datetime.fromisoformat(
        generated_at.replace("Z", "+00:00")
    )
except (AttributeError, ValueError) as error:
    raise SystemExit(
        f"[CAMERA-PROBE][ERROR] invalid generated_at_utc: {error}"
    )
if generated_datetime.tzinfo is None:
    raise SystemExit(
        "[CAMERA-PROBE][ERROR] generated_at_utc has no timezone"
    )

probe_app = report.get("probe_app")
if not isinstance(probe_app, dict):
    raise SystemExit("[CAMERA-PROBE][ERROR] probe_app is missing")
if probe_app.get("version_name") != expected_version_name \
        or probe_app.get("version_code") != expected_version_code:
    raise SystemExit(
        "[CAMERA-PROBE][ERROR] probe app version mismatch: "
        f"{probe_app.get('version_name')!r} "
        f"({probe_app.get('version_code')!r})"
    )
if not isinstance(report.get("cameras"), list):
    raise SystemExit("[CAMERA-PROBE][ERROR] cameras is missing")

device = report.get("device")
if not isinstance(device, dict):
    raise SystemExit("[CAMERA-PROBE][ERROR] device is missing")
reported_model = device.get("model")
if reported_model != expected_model:
    raise SystemExit(
        "[CAMERA-PROBE][ERROR] report/device mismatch: "
        f"JSON={reported_model!r}, adb={expected_model!r}"
    )
reported_fingerprint = device.get("fingerprint")
if reported_fingerprint != expected_fingerprint:
    raise SystemExit(
        "[CAMERA-PROBE][ERROR] report/fingerprint mismatch"
    )

print(f"[CAMERA-PROBE] Generated: {generated_at}")
PY

mv "$temporary_path" "$output_path"
temporary_path=

echo "[CAMERA-PROBE] Report: $output_path"
