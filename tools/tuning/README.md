# tools/tuning

Host-side tuning helpers live here.

## `turn_tune.py`

`turn_tune.py` simulates and replays F413 turn trajectories in the same
local coordinate frame:

- `x_mm`: right side of the mouse is positive
- `y_mm`: forward is positive
- `theta_deg`: left turn is positive, right turn is negative

The simulator reproduces the current F413 smooth omega-profile turn shape:
in-offset straight segment, constant-velocity omega-profile turn core, and
out-offset straight segment. It reads F413 preorder params by default.
For `shortest`, the default `--angle-policy configured` uses the angle field in
the selected params, matching mode2 case1/2 where angle accumulation is off.
Pass `--angle-policy runtime` for case0 and case4+ angle-accumulator runs, where
the executed angle is exactly 45/90/135/180 degrees even when the historical
params field contains a small angle correction. `turn_clearance.py` targets the
angle-accumulator shortest-run workflow and therefore selects `runtime` by
default.

Examples:

```sh
python3 tools/tuning/turn_tune.py simulate --runner shortest --mode 2 --code 501
python3 tools/tuning/turn_tune.py simulate --runner search --search-index 0 --side right
python3 tools/tuning/turn_tune.py replay tools/logging/logs/trace_bin_20260620_210552.csv --runner shortest --mode 2 --code 502 --compare-sim
python3 tools/tuning/turn_tune.py contact tools/logging/logs/trace_bin_20260621_150116.csv --window target-angle --tread 34.5
python3 tools/tuning/turn_tune.py fit --runner shortest --mode 2 --code 501 --target-x 104 --target-y 96 --target-theta -90
```

Notes:

- `simulate` is the ideal firmware plan from params.
- `replay` integrates the logged `target_velocity_mm_s` / `target_omega_mdps`
  and `real_velocity_mm_s` / `real_omega_mdps` streams. The logged target omega
  is the controller reference, so it may include heading correction outside the
  pure omega profile.
- `contact` compares signed left/right encoder deltas with IMU omega over a
  selected trace window. Use it to distinguish wheel-side tracking error from
  tire/floor yaw compliance or slip.
- `fit` prints suggested C initializer assignments only. It does not edit params.
  By default it varies velocity, alpha, in/out offsets, and the angle field.
  Do not fit the angle field for an angle-accumulator run because that field is
  not the executed angle in that configuration.

## `turn_clearance.py`

`turn_clearance.py` adds a physical acceptance gate to the nominal turn model.
It was independently implemented after reviewing the workflow of
<https://github.com/pidream/turn_simulator01>: the reference tool draws vehicle
width and 6 mm posts, while this implementation evaluates the full front/rear/
left/right rectangle, explicit wall panels and standalone posts, and reports a
numeric minimum clearance.

The reported values are deliberately separated:

- `raw_min_clearance_mm`: pure geometric body-to-obstacle gap; zero or less is
  contact/overlap.
- `uncertainty_mm`: mechanical tolerance + simulator/video position error +
  heading error at the furthest body corner + interpolation allowance.
- `effective_min_clearance_mm = raw - uncertainty`.
- A candidate passes only when `effective_min_clearance_mm` is at least
  `required_margin_mm` (3 mm by default) and its endpoint and heading errors
  are within the command-specific limits, and every trajectory model is inside
  an explicitly safety-qualified calibration scope.

The default footprint is the user-confirmed completed-machine 70 x 39 mm
rectangle: 35 mm front/rear and 19.5 mm left/right from the blue-label centre.
The user also confirmed that this reference coincides with the machine/turn
centre. The measurement is recorded in
`data/mini_r2_0_footprint.json`; use `--robot-*-mm` only when evaluating a
different machine envelope or a later mechanical revision. These measurements
exactly match the former bootstrap values, so confirming them changes their
provenance but does not change previously calculated clearance numbers.
The default 3 mm position/model uncertainty is a bootstrap value rounded up
from the largest endpoint standard deviation (2.81 mm) in the final three-run
R135-in batch. It is not universal; video validation of the known contact run
misses physical contact by about 5.1 mm before uncertainty is deducted, so use
`--position-uncertainty-mm 5` until label-height pose calibration and absolute
scene registration are validated for that camera/scene.

### Recommended workflow

1. Evaluate the current firmware parameters and visualize the complete swept
   body:

   ```sh
   python3 tools/tuning/turn_clearance.py evaluate \
     --mode 2 --code 901 \
     --position-uncertainty-mm 5 \
     --plot sessions/tuning/r135-current.png \
     --report-json sessions/tuning/r135-current.json
   ```

2. Search angular acceleration and solve/quantize the in/out offsets for the
   canonical KERI endpoint. Candidates that do not keep the requested margin
   are rejected. The command only prints a candidate; it never edits params:

   ```sh
   python3 tools/tuning/turn_clearance.py search \
     --mode 2 --code 901 \
     --alpha-min 6000 --alpha-max 14000 --alpha-step 250 \
     --offset-quantum-mm 0.5 \
     --position-uncertainty-mm 5 \
     --plot sessions/tuning/r135-search.png \
     --report-json sessions/tuning/r135-search.json
   ```

   With no calibrated full-path artifact, the zero-slip firmware model is a
   rough-design diagnostic only. It may rank candidates, but it cannot set
   `safe_recommendation_available`, emit C assignments, or return status 0.
   This prevents the ideal model from bypassing a known measured trajectory
   mismatch. The full 6000--14000/250 grid evaluates all endpoint-valid offset
   pairs and currently takes about 70--90 seconds on the development Mac; a
   fixed-alpha check takes about 2--3 seconds.

   R135-in at 500 mm/s has a historical velocity-heading-lag artifact
   (`data/mode2_d135_in_empirical_model.json`). Its legacy `K=0.03303 s^2/m`
   value fits only the final normalized core endpoint. Re-auditing the same
   trajectories gives `K=0.0261` after correcting the trajectory start and
   `K=0.0178` with 2.89 mm RMSE for a full-path fit. This disagreement means a
   single non-zero K is not a swept-clearance safety model. The following
   command is retained only to reproduce the historical diagnostic comparison:

   ```sh
   python3 tools/tuning/turn_clearance.py search \
     --mode 2 --code 901 \
     --slip-angle-coefficient 0.03303 \
     --slip-model-json tools/tuning/data/mode2_d135_in_empirical_model.json \
     --robust-slip-angle-coefficient 0 \
     --position-uncertainty-mm 5 \
     --alpha-min 6000 --alpha-max 14000 --alpha-step 250
   ```

   No non-zero model in this artifact is safety-qualified, even when mode,
   turn, speed and alpha match. The tool must therefore print
   `best_diagnostic_do_not_apply`, omit C assignments, and return status 1.
   `--slip-model-json` binds provenance and scope; it does not turn an artifact
   whose `qualification.safety_qualified` is false into a safety model.
   Extrapolated candidates and all candidates from this diagnostic-only
   artifact cannot return a safe result.

3. Apply only the `recommended` object when
   `safe_recommendation_available=true`, and only after its absolute-scene
   repeated-video validation. Never apply `best_diagnostic`, even when its
   geometric `margin_passed` field is true. Any mode2 turn-param change must
   be followed by regeneration and verification of the PC-generated route
   motion table before building or running firmware:

   ```sh
   python3 tools/route_precompute/generate.py
   tools/route_precompute/run_tests.sh
   ```

4. After recording 3--5 identical trials, first run the normalized shape gate.
   Apply the same envelope test to each measured trajectory. The angular-rate
   curve is time-aligned to the firmware profile so the slow beginning of the
   turn is retained; measured positions/headings then form the swept path and
   the configured entry offset is back-projected to the primitive origin:

   ```sh
   python3 tools/tuning/turn_clearance.py video path/to/run{1,2,3}/trajectory.csv \
     --mode 2 --code 901 \
     --position-uncertainty-mm 5 \
     --plot sessions/tuning/r135-video.png \
     --report-json sessions/tuning/r135-video.json
   ```

   Multiple paths are evaluated as one batch and the worst repeat is reported.
   A truncated turn, missing exit distance, insufficient pre/post-roll, or a
   valid-pose gap above 50 ms is rejected as incomplete analysis.

   Normalized mode deliberately removes the trial's absolute translation and
   rotation so it can compare turn shape and repeatability. It is not an
   absolute body-to-wall clearance result. For the final physical gate, provide
   a `--scene-json` in the same board-millimetre frame as the trajectory CSV and
   retain placement with:

   ```sh
   python3 tools/tuning/turn_clearance.py video path/to/run{1,2,3}/trajectory.csv \
     --mode 2 --code 901 \
     --registration-mode absolute \
     --scene-json path/to/measured-scene.json \
     --position-uncertainty-mm 5
   ```

   Vision yaw zero points along +board-x, whereas scene theta zero points along
   +world-y, so absolute mode adds `--absolute-yaw-offset-deg -90` by default.
   Change that option only if the supplied trajectory uses another convention.

5. Accept a parameter set only when every repeat passes clearance, endpoint and
   heading checks on both sides required by the intended route. Use the worst
   repeat, not only the median endpoint. Keep a 1 ms trace from at least one
   repeat to verify that a clearance failure is not control saturation/contact.

The built-in scene places all nearby 6 x 6 mm posts explicitly and adds a
conservative local set of inner/outer wall panels. For a specific test maze,
provide `--scene-json`. The schema is the same as the `scene` object in a JSON
report: `start_pose`, `target_local`, `target_pose`, and axis-aligned obstacle
rectangles with `id`, `kind`, `min_x_mm`, `min_y_mm`, `max_x_mm`, and
`max_y_mm`. Small-90 uses the union of every local wall compatible with its
required approach/exit openings. Its actual wall-end placement is still
run-time dependent, so a measured scene is required for final acceptance.

The evaluator adaptively interpolates every centre pose until the maximum
possible body-corner motion is below `--max-corner-step-mm`. This matters for
240 fps video, where the body can rotate several degrees between frames.
Commands return status 1 when analysis succeeds but any clearance/endpoint/
heading gate fails, and status 2 for invalid or incomplete analysis.
JSON and normal CLI output explicitly distinguish `normalized-shape` from
`absolute-clearance`; only the latter can set `absolute_clearance_qualified`.

Geometry, between-frame collision, runtime-angle, D135 contact/clearance, and
manifest regressions can be run with:

```sh
python3 tools/tuning/test_turn_clearance.py
```

## Calibration data

The current R135-in corpus can support diagnostics without new runs, but it
cannot safety-qualify a non-zero full-path model. Nine historical parameter
sets (27 non-contact videos) and the latest known-contact run are indexed in
`data/mode2_d135_in_manifest.json`. Parameter records, reports, trajectory
sets, and the explicit contact sources carry SHA-256 digests while the large
raw session files remain ignored; clean-checkout tests validate the catalog
and a populated local checkout also checks directly listed sources.
Historical reports are not safe to consume by filename alone: some older
composite sub-mode reports retained a default turn code. New datasets should
record exact params, effective runtime angle, firmware SHA/dirty state,
mode/case/sub/path codes, scene/footprint, trajectory hash, and whether contact
occurred.

The next R135-in calibration must hold speed at 500 mm/s, use an exact runtime
angle of 135 degrees, keep positive fixed entry/exit segments, and run in an
open fixture with enough straight distance before and after the turn. Capture
five right-turn repeats at each alpha of 8000, 10000, 12000 and
14000 deg/s2; retain a 1 ms machine trace for at least one repeat at every
alpha. Before interpreting absolute body-to-wall clearance, record the heights
of the blue- and red-label tracking planes above the maze floor (one value is
enough if they are equal), or provide stationary known-position samples across
the camera field, so homography parallax can be corrected. Fit and validate the
complete 10--130 degree swept path on held-out repeats, not only its endpoint.
After selecting a simulator candidate, capture three to five right repeats and
three to five mirrored left repeats in the representative walled scene before
changing the production turn parameters.
