#!/bin/sh
set -eu

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
  echo "Usage: $0 SERIAL [OUTPUT_DIR]" >&2
  exit 2
fi

serial=$1
output_dir=${2:-sessions/hfr-tests/pixel8/optical-runs}
script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

HFR_FPS=${HFR_FPS:-240} \
HFR_BITRATE=${HFR_BITRATE:-72000000} \
HFR_DURATION_SECONDS=${HFR_DURATION_SECONDS:-60} \
HFR_EXPOSURE_US=${HFR_EXPOSURE_US:-1000} \
HFR_ISO=${HFR_ISO:-800} \
HFR_ENABLE_PREVIEW=1 \
HFR_OPTICAL_TRIGGER=1 \
HFR_OPTICAL_TRIGGER_SCORE=${HFR_OPTICAL_TRIGGER_SCORE:-180} \
HFR_OPTICAL_TRIGGER_HOT_PIXELS=${HFR_OPTICAL_TRIGGER_HOT_PIXELS:-2} \
HFR_OPTICAL_STOP_TAIL_MS=${HFR_OPTICAL_STOP_TAIL_MS:-900} \
HFR_TIMEOUT_SECONDS=${HFR_TIMEOUT_SECONDS:-300} \
  "$script_dir/record_test.sh" "$serial" "$output_dir"
