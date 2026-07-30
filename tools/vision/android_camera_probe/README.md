# Nightfall Android Camera2 capability probe

This small, dependency-free debug APK enumerates the actual Camera2
high-speed size/FPS matrix exposed by a phone. It also records manual-sensor,
stabilization, timestamp, request/result-key, and high-speed encoder-profile
capabilities. Schema `nightfall_android_camera_probe_v2` includes the probe
generation time and app version. A failure for one camera is recorded in that
camera's entry instead of discarding successful entries for the other cameras.
The current probe release is version `0.2.0`, version code `2`.

It is a static capability probe, not the final recorder. A real high-speed
capture must still verify applied exposure, sensor timestamps, frame duration,
rolling-shutter skew, encoder PTS, and dropped frames.

## Build

The repository build host uses Android API 36, Android build-tools 36.0.0,
AGP 9.2, Gradle 9.4.1, and JDK 17:

```sh
tools/vision/android_camera_probe/build_debug.sh
```

`build_debug.sh` first honors `JAVA_HOME`, then looks for a JDK using
`/usr/libexec/java_home`, common Homebrew/Linux/Android Studio locations, and
the `java` executable on `PATH`. It first honors `ANDROID_HOME`, then
`ANDROID_SDK_ROOT`, followed by common macOS and Linux SDK locations. The
Android build-tools version and Gradle distribution checksum are pinned.

## Install and collect

Enable USB debugging and find each phone's serial:

```sh
adb devices -l
```

Install the probe on one phone:

```sh
SERIAL=REPLACE_WITH_ADB_SERIAL
adb -s "$SERIAL" install -r \
  tools/vision/android_camera_probe/app/build/outputs/apk/debug/app-debug.apk
```

Start a fresh probe and collect it with a phone-specific filename:

```sh
tools/vision/android_camera_probe/collect_report.sh "$SERIAL"
```

The collector force-starts the probe with a unique nonce and waits up to 120
seconds for the matching report. Unlock the phone and grant the camera
permission if prompted. The app deletes any prior report before requesting
permission or starting a probe, then publishes the new JSON with an atomic
rename. Manual `am start` or a press of `Refresh camera report` is not needed.
The collector rejects an installed app other than version `0.2.0`, version
code `2`. Override the wait using `PROBE_TIMEOUT_SECONDS` when necessary.

The default output directory is `/tmp/nightfall-camera-probe`. An optional
second argument selects the session artifact directory:

```sh
tools/vision/android_camera_probe/collect_report.sh \
  "$SERIAL" /path/to/session-artifacts
```

The filename includes the model, adb serial, and UTC collection time, so Pixel
8 and Xiaomi 13 Ultra reports cannot overwrite one another. The collector
checks JSON syntax, schema, launch nonce, app version, required metadata, and
that the JSON model and build fingerprint match the connected adb device.

Run the install/collect sequence separately with each serial when both phones
are connected. Keep collected reports with session artifacts, not in Git.
