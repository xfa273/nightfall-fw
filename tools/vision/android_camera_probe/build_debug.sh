#!/bin/sh
set -eu

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
java_root=${JAVA_HOME:-}
android_sdk_root=${ANDROID_HOME:-${ANDROID_SDK_ROOT:-}}

if [ -z "$java_root" ] && [ -x /usr/libexec/java_home ]; then
  java_root=$(/usr/libexec/java_home -v 17 2>/dev/null || true)
fi
if [ -z "$java_root" ]; then
  for candidate in \
    /opt/homebrew/opt/openjdk@17/libexec/openjdk.jdk/Contents/Home \
    /usr/local/opt/openjdk@17/libexec/openjdk.jdk/Contents/Home \
    /usr/lib/jvm/java-17-openjdk \
    /usr/lib/jvm/java-17-openjdk-* \
    /opt/android-studio/jbr \
    "/Applications/Android Studio.app/Contents/jbr/Contents/Home" \
    "${HOME:-}/Applications/Android Studio.app/Contents/jbr/Contents/Home"
  do
    if [ -x "$candidate/bin/java" ]; then
      java_root=$candidate
      break
    fi
  done
fi
if [ -z "$java_root" ]; then
  java_command=$(command -v java || true)
  if [ -n "$java_command" ]; then
    java_root=$(
      "$java_command" -XshowSettings:properties -version 2>&1 \
        | sed -n 's/^[[:space:]]*java.home = //p' \
        | head -n 1
    )
  fi
fi

if [ -z "$android_sdk_root" ]; then
  for candidate in \
    /opt/homebrew/share/android-commandlinetools \
    /usr/local/share/android-commandlinetools \
    "${HOME:-}/Library/Android/sdk" \
    "${HOME:-}/Android/Sdk"
  do
    if [ -d "$candidate/platforms/android-36" ]; then
      android_sdk_root=$candidate
      break
    fi
  done
fi

if [ -z "$java_root" ] || [ ! -x "$java_root/bin/java" ]; then
  echo "[CAMERA-PROBE][ERROR] JDK 17 not found; set JAVA_HOME" >&2
  exit 2
fi
if [ -z "$android_sdk_root" ] \
  || [ ! -d "$android_sdk_root/platforms/android-36" ]; then
  echo "[CAMERA-PROBE][ERROR] Android API 36 SDK not found; set ANDROID_HOME" >&2
  exit 2
fi
if [ ! -d "$android_sdk_root/build-tools/36.0.0" ]; then
  echo "[CAMERA-PROBE][ERROR] Android build-tools 36.0.0 not found" >&2
  exit 2
fi

echo "[CAMERA-PROBE] JAVA_HOME=$java_root"
echo "[CAMERA-PROBE] ANDROID_HOME=$android_sdk_root"

cd "$script_dir"
JAVA_HOME=$java_root \
  ANDROID_HOME=$android_sdk_root \
  ANDROID_SDK_ROOT=$android_sdk_root \
  ./gradlew --no-daemon :app:assembleDebug :recorder:assembleDebug

echo "[CAMERA-PROBE] APK: $script_dir/app/build/outputs/apk/debug/app-debug.apk"
echo "[HFR-RECORDER] APK:" \
  "$script_dir/recorder/build/outputs/apk/debug/recorder-debug.apk"
