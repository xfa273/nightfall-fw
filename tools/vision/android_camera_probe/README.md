# Nightfall Android Camera2 capability probe

This small, dependency-free debug APK enumerates the actual Camera2
high-speed size/FPS matrix exposed by a phone. It also records manual-sensor,
stabilization, timestamp, request/result-key, and high-speed encoder-profile
capabilities. Schema `nightfall_android_camera_probe_v2` includes the probe
generation time and app version. A failure for one camera is recorded in that
camera's entry instead of discarding successful entries for the other cameras.
The current probe release is version `0.2.0`, version code `2`.

The `app` module is the static capability probe. The separate experimental
`recorder` module attempts a real Camera2 constrained-high-speed recording and
writes CaptureResult and encoded-sample JSONL sidecars. A real capture must
still verify applied exposure, sensor timestamps, frame duration, encoder PTS,
decoded content, and dropped frames. Rolling-shutter skew is recorded only
when the device exposes it.

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

## Experimental Camera2 HFR recording

Build both APKs with `build_debug.sh`, then request a five-second 1080p/120
capture:

```sh
tools/vision/android_camera_probe/record_test.sh "$SERIAL" \
  /path/to/session-artifacts
```

The collector installs version `0.3.0` of
`com.nightfall.hfrrecorder`, starts a nonce-tagged recording, and pulls:

- `hfr_capture.mp4`
- `capture_results.jsonl`
- `encoder_samples.jsonl`
- `hfr_report.json`
- `ffprobe_stream.json`

The recorder enables a simultaneous `TextureView` preview by default and
disables EIS and OIS for measurement use. Automatic exposure is the default.
A manual trial can be requested only after an automatic trial succeeds:

```sh
HFR_EXPOSURE_US=1000 HFR_ISO=400 \
  tools/vision/android_camera_probe/record_test.sh "$SERIAL" \
  /path/to/session-artifacts
```

The explicit equivalent of the default preview-enabled Pixel test is:

```sh
HFR_FPS=240 HFR_BITRATE=72000000 HFR_ENABLE_PREVIEW=1 \
  tools/vision/android_camera_probe/record_test.sh "$SERIAL" \
  /path/to/session-artifacts
```

The report records whether the preview surface was enabled. A successful
recording-only session does not prove that preview plus recording works; test
the exact surface combination on each device. Set `HFR_ENABLE_PREVIEW=0` only
for a recording-surface-only diagnostic.

## F413 optical-trigger capture

The recorder can arm a preview-only high-speed session and wait for the F413
mouse to emit its visible-LED token. The firmware drives all three status LEDs
with a 300 ms ON, 200 ms OFF, 300 ms ON, 200 ms OFF, 600 ms ON pattern. It
emits one token before starting the trace/motion and another after stopping.
The Pixel compares consecutive preview frames, discards the first 0.8 seconds
while auto exposure settles, calibrates local preview noise for 1.2 seconds,
and requires three brightening edges at the same location at
the expected spacing.

Arm and collect one run with:

```sh
tools/vision/android_camera_probe/capture_optical_run.sh \
  "$SERIAL" /path/to/session-artifacts
```

The wrapper selects 1080p/240, 72 Mbps, 1.000 ms, ISO 800, and a 60-second
maximum recording by default. It waits up to five minutes for a complete
START/STOP pair. The start token is decoded while the preview-only request is
running; MediaRecorder is enabled before the firmware's final LED pulse and
300 ms guard complete. The stop token leaves a default 900 ms video tail.
`hfr_report.json` records both optical detection times and the delay from start
detection to MediaRecorder start.

The 2026-08-01 stationary Pixel 8 integration trial completed without the
external lamp: start detection to recording was 28.1 ms, stop detection to
recording stop was 908.9 ms with the configured 900 ms tail, and the 6.23 s
session had 0 capture failures. Sensor timestamps measured 239.99998 fps; the
H.264 MP4 contained 1,453 frames at 239.9808 fps. Both the start and stop final
LED pulses, plus an LED-off tail, were visually present in the saved video.
Ambient-light success proves the trigger path, not markerless trajectory
quality; illuminate the maze for moving runs and repeat this test after the
final camera/light installation is fixed.

No UART connection to the mouse is used during a floor run. While developing
with a wired and stationary mouse, UART `;` emits one token without starting
the motors, allowing the optical path to be tested twice for START and STOP.
The mouse and its three visible status LEDs must be inside the Pixel frame.
After arming, wait until the screen reports `WAIT_FIRST_RISE` before starting
the mouse so the preview-noise calibration is complete.

The thresholds can be overridden for a measured installation:

```sh
HFR_OPTICAL_TRIGGER_SCORE=500 \
HFR_OPTICAL_TRIGGER_HOT_PIXELS=2 \
HFR_OPTICAL_STOP_TAIL_MS=900 \
  tools/vision/android_camera_probe/capture_optical_run.sh "$SERIAL" /path/out
```

The configured score is a lower bound; the app raises it automatically when
preview noise measured during arming is higher. Always repeat the non-motor
UART-token test after changing height, exposure, ISO, or illumination.

### Pixel 8 verified HFR path

On the tested Pixel 8 (`shiba`; initially Android 16/API 36 and currently
Android 17/API 37), rear camera ID 0 exposes
fixed 120 and 240 fps at both 1280x720 and 1920x1080, a hardware H.264
1920x1080/240 profile, manual sensor control, REALTIME sensor timestamps, and
rolling-shutter-skew results.

The preview plus MediaRecorder path completed a five-second
1920x1080/240 test at 72 Mbps with requested 1.000 ms exposure, ISO 400, and
EIS/OIS off. CaptureResult reported 1,144 unique sensor timestamps at measured
240.000 fps, actual exposure 0.999635 ms, ISO 400, and 4.542720 ms rolling
shutter skew. The MP4 contained 1,141 decoded frames at measured 239.981 fps,
with no PTS gaps and no identical or near-identical adjacent frames; strict
timing/content QA passed.

With only the MediaRecorder surface, CaptureResult callbacks represented
roughly one result per high-speed request batch. Adding the preview surface
produced per-frame-rate callbacks and is therefore the selected starting point
for live LED detection and sensor/encoded timestamp mapping.

Do not interpret a static Camera2 HFR matrix as proof that a session works.
On the tested Xiaomi 13 Ultra (`2304FPN6DG`, MIUI
`V14.0.5.0.TMAMIXM`, Android 13), camera ID 0 advertises fixed
1080p/120, 240, and 480 fps. The vendor HAL nevertheless rejected every
120 fps constrained session during `configureStreams`, including:

- preview plus MediaCodec recording surfaces;
- a MediaCodec recording surface alone;
- a MediaRecorder recording surface alone; and
- a preview surface alone.

The device log reports a zero HAL buffer count and
`Unsupported set of inputs/outputs provided`. This is a device/firmware
Camera2 interoperability failure, not evidence that the advertised mode is
usable. Keep the diagnostic report with session artifacts. The verified Pixel
8 custom-recorder path is preferred over this Xiaomi public Camera2 path.

## Xiaomi stock-camera fallback

Xiaomi's privileged stock Camera can use the same hardware even when the
public Camera2 HFR session is rejected. Start a collector and then operate the
phone:

```sh
tools/vision/android_camera_probe/collect_stock_slowmo.sh "$SERIAL" \
  /path/to/session-artifacts
```

Select **Slow motion**, choose 120 or 240 fps, record for several seconds, and
stop. The collector detects the new file under `DCIM/Camera`, waits for its
size to stabilize, pulls it, reads `com.android.capture.fps`, and runs
`video_timing_qa.py` against that capture rate. It rejects an MP4 without a
valid high-speed capture tag.

An existing Xiaomi 13 Ultra stock clip named with `HSR_240` was independently
checked as 1280x720 H.264 with 1,337 frames. Its MP4 declared
`com.android.capture.fps=240`; measured median PTS cadence was 239.981 fps,
with no PTS gaps or decoded adjacent duplicates. Thus the stock path preserves
real-time HFR frames on this firmware rather than silently retiming them to
30 fps. Each new session must still pass the same check. The stock path does
not provide per-frame Camera2 CaptureResult exposure or sensor timestamps, so
use an optical sync event when aligning it to firmware trace.
