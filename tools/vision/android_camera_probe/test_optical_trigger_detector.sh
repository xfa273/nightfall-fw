#!/bin/sh
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
build_dir=$(mktemp -d /tmp/nightfall-optical-trigger-test.XXXXXX)
trap 'rm -rf -- "$build_dir"' EXIT HUP INT TERM

java_root=${JAVA_HOME:-}
if [ -z "$java_root" ] \
  && [ -x /opt/homebrew/opt/openjdk@17/libexec/openjdk.jdk/Contents/Home/bin/javac ]; then
  java_root=/opt/homebrew/opt/openjdk@17/libexec/openjdk.jdk/Contents/Home
fi
if [ -z "$java_root" ] || [ ! -x "$java_root/bin/javac" ]; then
  echo "[OPTICAL-TRIGGER-TEST][ERROR] JDK 17 not found; set JAVA_HOME" >&2
  exit 2
fi

"$java_root/bin/javac" -d "$build_dir" \
  "$script_dir/recorder/src/main/java/com/nightfall/hfrrecorder/OpticalTriggerDetector.java" \
  "$script_dir/recorder/src/main/java/com/nightfall/hfrrecorder/MotionGateDetector.java" \
  "$script_dir/recorder/src/main/java/com/nightfall/hfrrecorder/WifiTransferProtocol.java" \
  "$script_dir/host_tests/OpticalTriggerDetectorHostTest.java" \
  "$script_dir/host_tests/MotionGateDetectorHostTest.java" \
  "$script_dir/host_tests/WifiTransferProtocolHostTest.java"
"$java_root/bin/java" -cp "$build_dir" \
  com.nightfall.hfrrecorder.OpticalTriggerDetectorHostTest
"$java_root/bin/java" -cp "$build_dir" \
  com.nightfall.hfrrecorder.MotionGateDetectorHostTest
"$java_root/bin/java" -cp "$build_dir" \
  com.nightfall.hfrrecorder.WifiTransferProtocolHostTest
