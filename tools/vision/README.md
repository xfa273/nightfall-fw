# ArUco trajectory extraction

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
python3 -m venv /tmp/nightfall-vision-venv
/tmp/nightfall-vision-venv/bin/python -m pip install -r tools/vision/requirements.txt
```

## Run

Without a known physical scale:

```sh
/tmp/nightfall-vision-venv/bin/python tools/vision/aruco_trajectory.py \
  /path/to/video.mov
```

With a 90 mm grid pitch that has not been independently measured:

```sh
/tmp/nightfall-vision-venv/bin/python tools/vision/aruco_trajectory.py \
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
