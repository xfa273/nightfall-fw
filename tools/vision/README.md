# Video trajectory and turn-tuning tools

The operational design, current limitations, and hardware-side next steps are
documented in
[`docs/SMARTPHONE_TURN_TUNING.md`](../../docs/SMARTPHONE_TURN_TUNING.md).

Reusable tools in this directory are:

- `aruco_trajectory.py`: carried-ArUco reference extractor for the existing
  8×8 test board
- `markerless_trajectory.py`: fixed board markers plus green PCB, foreground,
  and one red cue; no vehicle marker is used
- `desk_green_pair_probe.py`: raw-pixel feasibility probe that tracks the two
  separated green PCB regions before a marked maze fixture is available
- `generate_fixed_aruco_print_pack.py`: print-ready A4 PDF and vector SVG
  generator for the four fixed maze-plane markers
- `video_timing_qa.py`: checks encoded frame PTS cadence, gaps, and identical
  192-pixel-wide grayscale fingerprints of decoded neighbors
- `fuse_trace_video.py`: aligns video yaw rate or speed with a firmware trace
- `board_layout.py`: validates a measured fixed-marker/grid layout in mm
- `android_camera_probe/`: Camera2 capability probe, experimental HFR recorder,
  and Xiaomi stock-camera slow-motion collector
- `../tuning/turn_video_tune.py`: converts video endpoints to the firmware turn
  frame and prints a bounded candidate without editing source files

The Android real-time HFR recorder and F413 three-pulse START/four-pulse STOP
optical trigger are implemented, with a markerless green-body motion gate
between them. Pixel 8 has passed a preview plus 1080p/240
manual-exposure recording trial with per-frame-rate CaptureResult metadata,
and both optical edges passed a stationary no-motor integration trial;
the tested Xiaomi 13 Ultra firmware rejects all public Camera2 constrained-HFR
sessions.
The more-than-four-marker solver, multi-LED solver, session-wide QA runner,
trace-phase segmentation, and whole-path fitter are not implemented yet.

## Host setup and timing QA

FFmpeg supplies frame PTS and ADB is used for the static Android probe.
Create the persistent, Git-ignored Python environment with:

```sh
brew install ffmpeg
brew install --cask android-platform-tools
tools/vision/setup_host.sh
```

The Android build additionally needs JDK 17, Android command-line tools, API
36, and build-tools 36.0.0.

The probe additionally needs SDK packages `platforms;android-36` and
`build-tools;36.0.0`; see
[`android_camera_probe/README.md`](android_camera_probe/README.md).

Check an expected real-time 120 fps file before pose extraction:

```sh
./.venv-vision/bin/python tools/vision/video_timing_qa.py \
  /path/to/capture.mp4 \
  --expected-fps 120 \
  --fps-tolerance-percent 1.0 \
  --maximum-gap-rate 0.001 \
  --maximum-cadence-deviation-percent 10 \
  --maximum-cadence-deviation-rate 0.001 \
  --maximum-content-duplicate-rate 0.001 \
  --report-json /path/to/timing_report.json
```

This checks encoded PTS, both short and long cadence deviations, and exact
equality of decoded frames after resizing them to at most 192 pixels wide and
converting them to grayscale. It is not a full-resolution, full-colour byte
comparison. It cannot prove sensor capture rate or detect all duplicated,
interpolated, or retimed frames. A production recorder must save sensor
timestamp, frame duration, exposure, ISO, rolling-shutter skew, and the
encoded-PTS mapping in a sidecar.

For the Xiaomi stock-camera fallback, use:

```sh
tools/vision/android_camera_probe/collect_stock_slowmo.sh \
  "$SERIAL" /path/to/session-artifacts
```

The collector waits for a new stock Camera MP4, validates the
`com.android.capture.fps` tag, and runs the same timing/content QA. The tested
1080p/240 Xiaomi clip contained 2,823/2,823 valid decoded frames, measured
239.981 fps median PTS cadence, no large PTS gaps, and no identical adjacent
decoded frames.

## Unmarked desk feasibility probe

Before the marked maze fixture is ready, a short hand-moved desk clip can
check whether the green PCB remains visible:

```sh
./.venv-vision/bin/python \
  tools/vision/desk_green_pair_probe.py /path/to/stock_hfr.mp4 \
  --output-dir /path/to/desk-probe \
  --render-fps 60
```

The current Nightfall body presents two separated green PCB components. The
probe pairs them using component area, rigid separation, and motion prediction;
their midpoint is the raw-pixel position and their line is an undirected
heading. The desk test on Xiaomi 13 Ultra tracked all 2,823 frames even with a
hand partly occluding the mouse. Median pair separation was 148.6 px.

Outputs are `trajectory_image_px.csv`, `trajectory_image_px.png`, an optional
annotated MP4, and `desk_green_pair_report.json`. These values have no metric
scale, board homography, lens correction, or absolute front/back direction.
They prove visibility only. Use fixed perimeter markers and
`markerless_trajectory.py` for maze measurements.

## Markerless vehicle pose

`markerless_trajectory.py` never detects the carried ID. Its current pipeline
is:

1. per-frame rectification from fixed IDs 5, 4, and 6;
2. a 41-frame temporal-median background;
3. a green PCB component of at least 250 canonical pixels as the mandatory
   per-frame search seed;
4. nearby background difference as the foreground body;
5. one red connected component as the tracked point and directed heading cue;
6. gated foreground principal-axis continuity as a cue fallback.

This is not a general contour-only tracker. The current red tracker is not a
multi-LED pose solver, and a frame without a usable green component is
invalid. Cue lever arm, recent cue distance, principal-axis anisotropy, and
maximum yaw rate gate `heading_valid` independently of position validity.
`qa_report.json` records accepted-cue lever-arm percentiles as
`accepted_cue_lever_arm_px`.

Without `--background-video`, the median background assumes that the vehicle
does not occupy the same pixel in half or more of the sampled run frames.
Prefer a separate empty-maze clip captured with unchanged camera/exposure.

The fixed-marker transform is specific to the existing test board:

- IDs 5, 7, 4, and 6 are top-left, top-right, bottom-right, and bottom-left.
- Only 5, 4, and 6 enter the homography. ID 7 is detection-count QA only.
- Marker centers are assumed to form a square, all markers have the same
  orientation and size, and with the defaults the black-side/center-span ratio
  must be about 4.95%.

Do not place arbitrary perimeter markers and treat the legacy transform as
metric. New setups should copy and then replace the measured values in
`board_layout_4x4_example.json`. Its `nightfall_vision_board_layout_v1` schema
maps marker center, black-side length, rotation, and the analysis grid in mm.
The current solver still uses IDs 5, 4, and 6 for homography; ID 7 is QA-only.
Coordinates are +x right and +y forward/up in the rectified view. At
`rotation_deg: 0`, every printed marker's top edge faces +y and its top edge
runs left-to-right along +x; positive rotation is counter-clockwise in the
maze plane. `canvas_bounds_mm` must be square because the canonical output uses
one metric scale on both axes.

The example's `-55..775 mm` bounds are an 830 mm analysis crop, not a paper
edge, fixture edge, or required-FOV edge. The outer black marker edges reach
`-50..770 mm`. Provide at least 7 mm of white quiet zone outside them; 10 mm is
recommended, giving an approximately 840 mm physical visible span. Add vehicle
travel and placement margin beyond that when selecting the camera FOV.

Generate the fixed-marker print pack with:

```sh
./.venv-vision/bin/python tools/vision/generate_fixed_aruco_print_pack.py
```

The A4 PDF has a recommended 60 mm black-side page and a compact 40 mm page.
Print exactly one page at `100%` / `Actual Size`; do not mix sizes. Verify the
printed 100 mm scale line and measure the black outer square. The four distinct
`DICT_4X4_50` markers are ID 5 top-left, ID 7 top-right, ID 4 bottom-right, and
ID 6 bottom-left. Keep at least the supplied white quiet zone and place all
four on the same plane as the maze floor.

For 1080-pixel short-side video, aim for at least 40 pixels across a marker's
black side:

```text
black_side_mm >= visible_short_span_mm * 40 / 1080
```

Thus, 60 mm is suitable up to about a 1.5 m visible short-side span. A wider
view, such as a complete full-size 16x16 maze, needs larger markers and a
different print layout.

For a measured 4×4 setup, run:

```sh
./.venv-vision/bin/python \
  tools/vision/markerless_trajectory.py /path/to/capture.mp4 \
  --board-layout /path/to/board_layout.json \
  --background-video /path/to/empty_background.mp4 \
  --initial-yaw-deg 180 \
  --cue-yaw-offset-deg 180 \
  --maximum-missing-fraction 0.01 \
  --maximum-heading-invalid-fraction 0.01
```

In legacy mode without `--board-layout`, `--grid-cells` is the number of
visible analysis-grid pitches, not the nominal cell count of the complete
maze. A view containing 4×4 standard 180 mm cells has eight 90 mm pitches and
uses `--grid-cells 8 --cell-size-mm 90 --cell-size-confirmed`.

`--cue-yaw-offset-deg` must be calibrated for the chosen cue. The anchor passed
to `turn_video_tune.py` has the direction **control reference to tracked
point**, resolved along the vehicle-right and vehicle-forward axes. For a cue
23 mm behind the control reference, use `--anchor-forward-mm -23`, not `+23`.

Use `--position-source green` when the foreground silhouette touches maze
walls and its centroid becomes biased. This tracks the green-PCB centroid and
still uses the foreground principal axis for cue-less heading. The PCB centroid
is not the vehicle control reference, so measure its fixed right/forward offset
and pass the corresponding anchor correction to downstream turn analysis.

For a rigidly mounted camera, a hand may briefly hide multiple board markers
while starting the vehicle. In that case, set both
`--maximum-consecutive-homography-fallback-frames` and
`--maximum-homography-fallback-fraction` to explicit bounded values. The
tracker reuses the last marker-supported transform during that interval. Keep
the default five-frame consecutive limit when the camera can move.

The transform does not remove lens distortion or height parallax. Before using
millimetre results for parameter changes, calibrate the exact HFR crop,
undistort it, compensate the known cue height, and validate position/yaw across
the field with a temporary ground-truth fixture.

On `IMG_1592.mov`, fixed board markers were the only pose-estimation markers and
ID 3 was used only as post-hoc ground truth. The prototype tracked all 515
frames; a stationary calibration followed by active-run hold-out evaluation
gave about 6.3 mm position RMSE and 4.0 degrees yaw RMSE. This 720p/30 fps
one-video result establishes feasibility only and does not meet the intended
automatic-tuning gate: its detected start pose window has a 21.4 mm/s median
speed and is rejected by the default 10 mm/s stationarity limit.

## Trace alignment

Align one short, distinctive turn and require the stricter operational
correlation gate:

```sh
./.venv-vision/bin/python \
  tools/vision/fuse_trace_video.py \
  /path/to/trajectory.csv /path/to/trace.csv \
  --minimum-correlation 0.70 \
  --yaw-sign same \
  --minimum-correlation-margin 0.02 \
  --output-dir /path/to/analysis/fused
```

The output is `/path/to/analysis/fused/fused.csv` plus
`sync_report.json`. The fused CSV keeps every source trace CSV data row and
column, then adds the video measurements. Parsed `#key=value` comment metadata
is stored in `sync_report.json`; blank lines, other comments, and comment order
are not copied to `fused.csv`. Inspect the selected offset, `video_sign`,
second peak margin, signal gain, bias, and RMSE as well as correlation. The
default expects both Nightfall and video yaw to be CCW-positive. An inverted
sign is a calibration failure, not permission to proceed.

Repeated similar turns can produce an ambiguous correlation peak. Motion
correlation is a provisional synchronizer; start/end LED patterns tied to
firmware ticks are the planned robust synchronizer.

## Endpoint candidate

The current candidate tool uses final position and yaw only. It does not use
the fused trace, path shape, duration, or trace phase in the fit.

Use five identical dedicated trials whose motion boundaries match the turn
model and have at least 150 ms of stationary pose before and after. Do not use
an unsegmented run containing approach/exit straights or multiple turns.
The command below hard-gates each 150 ms pose window at 80% time coverage
(at least 120 ms of samples) and at a median speed no greater than 10 mm/s.
These gates do not replace the requirement to record the full stationary
period.

```sh
./.venv-vision/bin/python \
  tools/tuning/turn_video_tune.py \
  trial01/trajectory.csv trial02/trajectory.csv \
  trial03/trajectory.csv trial04/trajectory.csv \
  trial05/trajectory.csv \
  --runner shortest --mode 2 --code 501 \
  --anchor-forward-mm -23.0 --anchor-right-mm 0.7 \
  --minimum-valid-fraction 0.99 \
  --minimum-heading-valid-fraction 0.99 \
  --maximum-pose-window-speed-mm-s 10 \
  --minimum-pose-window-coverage 0.8 \
  --minimum-fit-trials 5 \
  --maximum-endpoint-std-mm 2.0 \
  --maximum-yaw-std-deg 1.0 \
  --maximum-turn-yaw-error-deg 30 \
  --vary angle \
  --propose-fit --feedback-gain 0.5 \
  --report-json turn_report.json
```

The anchor numbers are examples from `IMG_1592.mov`; measure and replace them
for the selected cue and control reference. `--vary angle` intentionally
overrides the fitter's multi-parameter default for the first staged tuning
pass. The fitter prints an experiment candidate only. It does not edit params,
build, flash, spin the fan, or start a run. Any floor/maze trial requires
explicit authorization for that specific motion under
`docs/ai/HIL_SAFETY.md`.

## Carried-ArUco reference extraction

`aruco_trajectory.py` extracts a mouse trajectory from an overhead video of
the Nightfall 8×8 tracking board.

The default layout is:

- ID 5: board top-left
- ID 7: board top-right
- ID 4: board bottom-right
- ID 6: board bottom-left
- ID 3: marker carried by the mouse

The fixed markers provide a per-frame planar transform, so camera drift and
perspective do not appear as mouse motion. The carried marker is tracked from
its known ArUco bit pattern. Green PCB pixels are used only to constrain the
initial search and any low-confidence recovery search.

The transform consistently uses IDs 5, 4, and 6, which span the board and are
readable throughout the supplied recording. ID 7 is still detected and drawn
for QA, but is not mixed intermittently into the transform because glare makes
its detection much less stable.

## Setup

```sh
tools/vision/setup_host.sh
```

## Run

Without a known physical scale:

```sh
./.venv-vision/bin/python tools/vision/aruco_trajectory.py \
  /path/to/video.mov
```

With a 90 mm grid pitch that has not been independently measured:

```sh
./.venv-vision/bin/python tools/vision/aruco_trajectory.py \
  /path/to/video.mov \
  --cell-size-mm 90
```

Add `--cell-size-confirmed` only when the pitch has been measured or otherwise
confirmed. Cell coordinates remain available regardless of metric scale.

The coordinate system has its origin at the bottom-left grid intersection,
with +x to the right and +y upward. Yaw is zero toward +x and increases
counter-clockwise. The default `--yaw-offset-deg 90` matches the ID 3 mounting
in `IMG_1592.mov`; use the measured marker-to-vehicle rotation for other
mountings. The reported position is the marker center, not a separately
calibrated geometric vehicle center.

## Outputs

The default output directory is `<video stem>_trajectory` beside the input:

- `trajectory.csv`: raw and smoothed positions, yaw, velocity, marker score,
  fixed-marker IDs, and homography residual
- `trajectory.png`: time-colored top-view path
- `trajectory_overlay.mp4`: trajectory over the source view
- `trajectory_topview.mp4`: stabilized top view with the trajectory
- `reference_topview.png`: stabilized reference frame
- `calibration.json`: marker/grid coordinate mapping
- `qa_report.json`: detection rates and trajectory quality metrics

Metric columns in the CSV are blank unless `--cell-size-mm` is supplied.
If the local OpenCV build cannot encode MP4, the two videos use `.avi`
instead; `qa_report.json` always records the actual filenames.

Generated CSV, image, JSON, and video files are run artifacts and should stay
outside Git. Only the reusable files under `tools/vision/` belong in the
repository.

## Notes

- The detector uses OpenCV `DICT_4X4_50`. Larger OpenCV 4×4 dictionaries share
  these low-numbered code words, so the exact dictionary used to print the
  markers cannot be inferred from the video alone.
- The planar transform removes camera motion but does not model lens
  distortion. If sub-millimetre accuracy is required, calibrate the camera and
  undistort frames first.
- `--no-videos` skips the two MP4 renders when only numeric output is needed.
