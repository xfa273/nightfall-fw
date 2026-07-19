# F413 Sensor Distance Conversion Design

## Purpose

F413 currently uses wall sensor delta ADC values directly for wall detection, wall control, wall-end detection, and front-wall position matching. This makes the control response nonlinear with respect to real wall distance.

The goal is to add a distance-domain sensor layer that converts the existing offset-corrected ADC delta into millimeters, while preserving the current raw-ADC behavior until the conversion is validated with logs and HIL.

## Reference Implementations

When checking public micromouse implementations for this topic, prioritize:

- Naophis repositories: https://github.com/Naophis?tab=repositories
- satoshihamasuna repositories: https://github.com/satoshihamasuna?tab=repositories
- kerikun11 repositories: https://github.com/kerikun11?tab=repositories

Observed useful patterns:

- satoshihamasuna `mouse_type8`: converts each IR sensor from ADC to distance using per-sensor monotonic tables and clamps outside the valid range. Wall-fix motion uses front-left/front-right distances, with distance average controlling translation and distance difference controlling rotation.
- kerikun11 `micromouse-kerise-firmware`: separates wall detection from wall distance, uses averaged distance values, validity/hysteresis, and calibration references. Front-wall attachment is a bounded, timeout-controlled motion based on front sensor distance errors.
- Naophis `ExiaNona`: uses `front_dist`/`front_mid_dist` and millimeter-domain offsets for search slalom and pivot/front-wall correction decisions.

## LTR-209 Sensor Considerations

The F413 machine uses Lite-On LTR-209 phototransistors.

Datasheet reference: https://xonstorage.z8.web.core.windows.net/pdf/liteon_ltr209_apr22_xonlink.pdf

Relevant device traits:

- NPN phototransistor, lensed clear end-looking package.
- Narrow optical acceptance: half angle is about 8 degrees to the half-power point.
- Matched to 940 nm infrared emitters.
- The phototransistor response is a detector-current response to irradiance; in a micromouse, the measured ADC-to-distance curve additionally includes LED radiation pattern, wall reflectance, sensor angle, wall angle, ADC/load circuit, and saturation.

Implications for conversion:

- A single first-principles formula should not be trusted as the primary calibration.
- The conversion must be per-channel and measured on the actual machine.
- Close range can saturate or bend sharply; far range has low slope and becomes noisy.
- Extrapolated distances are useful for logs, but should not be used for control.
- Shape-preserving interpolation is preferred over ordinary cubic splines, because overshoot would create false wall distances.

## Current Nightfall State

Existing pieces that should be reused:

- `platform/stm32f405/Core/Src/sensor_distance.c`
  - Per-channel LUT conversion for FL/FR/L/R.
  - Front-sum conversion.
  - Optional 3-point distance-domain warp for FL/FR/front-sum.
- F413 already links `sensor_distance.c`.
- `nvm_params_distance_load_and_apply()` on F413 already loads 3-point warp data and applies it to `sensor_distance`.
- `f413_wall_sensor.c` already performs ambient/off subtraction and per-channel NVM offset subtraction:
  - `delta = on - off - saved_offset`
  - this should remain the input to distance conversion.

Current gaps:

- `f413_wall_sensor_snapshot_t` exposes only raw/delta ADC values and boolean wall flags.
- F413 wall detection/control/search code still consumes `fr_delta`, `fl_delta`, `r_delta`, `l_delta` directly.
- Existing NVM distance blob stores only front-related warp data, not side-channel correction.
- Existing trace schema logs ADC values but not distance values.

## Signal Pipeline

Use this pipeline:

1. Raw ADC off/on acquisition in `f413_wall_sensor.c`.
2. Offset-corrected delta ADC:

   ```text
   delta_adc = max(0, on_adc - off_adc - nvm_dark_offset)
   ```

3. Distance conversion:

   ```text
   mm_unwarped = LUT(delta_adc)
   mm = channel_warp(mm_unwarped)
   ```

4. Validity/confidence evaluation:
   - invalid if saturated
   - invalid if below no-wall delta threshold
   - invalid if outside calibrated LUT range beyond a small margin
   - degraded confidence when extrapolated

Do not use raw `on_adc` as the conversion input. Environment-light drift should be handled before conversion by the current off/on subtraction and dark-offset calibration.

## Firmware API

Keep the existing ADC snapshot API unchanged and add a distance wrapper.

Proposed new module:

- `f413_wall_distance.h`
- `f413_wall_distance.c`

Proposed snapshot:

```c
typedef struct {
  f413_wall_sensor_snapshot_t adc;

  float fr_mm;
  float fl_mm;
  float front_sum_mm;
  float r_mm;
  float l_mm;

  float fr_mm_unwarped;
  float fl_mm_unwarped;
  float front_sum_mm_unwarped;
  float r_mm_unwarped;
  float l_mm_unwarped;

  uint16_t valid_mask;
  uint16_t extrapolated_mask;
  uint16_t saturated_mask;

  bool front_valid;
  bool right_valid;
  bool left_valid;
} f413_wall_distance_snapshot_t;
```

Suggested functions:

```c
void f413_wall_distance_init(void);
bool f413_wall_distance_read_snapshot(f413_wall_distance_snapshot_t* out);
bool f413_wall_distance_front_present(const f413_wall_distance_snapshot_t* s);
bool f413_wall_distance_side_present(const f413_wall_distance_snapshot_t* s, bool right);
```

`f413_wall_distance_init()` should call `sensor_distance_init()` and then `nvm_params_distance_load_and_apply()`. If NVM distance data is missing, default LUTs are still used.

Implementation status:

- `sensor_distance` supports two LUT interpolation modes:
  - `SENSOR_DISTANCE_INTERP_LINEAR`: legacy default.
  - `SENSOR_DISTANCE_INTERP_PCHIP`: shape-preserving cubic interpolation over the current monotonic LUT.
- `f413_wall_distance_init()` selects PCHIP for F413 distance snapshots.
- F413 exposes a non-motor UART diagnostic command:
  - `n`: average 512 distinct completed sensor frames, then print only
    `FR`, `FL`, `R`, and `L` in the fitter's sensor order.
  - `:`: print the detailed calibration view with standard deviation, min/max,
    converted distance, and validity masks.
  - The detailed view's highlighted calibration row uses the fitter's exact order:
    `DIST_MM,FR,FL,R,L`; replace `DIST_MM` with the fixture distance.
- The F413 front-wall match path now uses calibrated PCHIP distance values:
  - front-sum distance controls translation.
  - individual FR/FL distance difference controls yaw.
  - both normal exploration matching and the continuous mode1 test use the same path.
- Wall detection, side-wall control, wall-end detection, and search wall writes still use the
  existing raw ADC paths until their distance conversions are separately calibrated.

## Conversion Model

Existing `sensor_distance.c` behavior:

- Convert ADC to distance by finding the surrounding LUT segment and applying linear interpolation directly in `(adc, mm)` space.
- For FL/FR/front-sum only, optionally apply a 3-point distance-domain PCHIP warp after LUT conversion.
- The 3-point warp is useful for global scale/offset correction, but it cannot remove local interpolation error caused by the optical sensor curve.

Design target:

- Use monotonic per-channel LUTs, but do not assume a coarse, uniformly spaced table is accurate enough.
- Generate the firmware LUT offline from calibration data with an error-bounded adaptive knot selection:
  - keep more points where the ADC-vs-distance curve has high curvature.
  - keep fewer points where the curve is smooth.
  - report the worst interpolation error in millimeters before accepting the table.
- Clamp or mark invalid outside the calibrated range for control use.
- Keep unwarped values in logs for diagnosis.
- Apply 3-point distance-domain warp only after LUT conversion, and use it only for per-unit/per-environment correction.

This matches the practical shape used by satoshihamasuna while reusing the Nightfall `sensor_distance` module.

Candidate interpolation models for the calibration tool:

1. Baseline ADC-domain linear interpolation
   - Current firmware behavior.
   - Cheapest runtime path.
   - Acceptable only if the generated adaptive LUT meets the error target.

2. Transformed-domain linear interpolation
   - Interpolate using a transformed sensor axis such as:

     ```text
     x = log(delta_adc)
     ```

     or, if the measured data is closer to an inverse-power curve:

     ```text
     x = 1 / sqrt(delta_adc)
     ```

   - This can reduce the number of LUT points, but adds runtime math or requires an additional transformed lookup table.

3. Monotone cubic interpolation
   - Use PCHIP in ADC or transformed-ADC domain.
   - Preserves monotonicity better than a normal spline.
   - Better suited to the LTR-209 reflected-light curve than coarse ADC-domain linear interpolation.
   - Requires precomputed slopes and careful monotonicity enforcement, but runtime cost is still small for four wall sensors on STM32F413.

Recommended firmware direction:

- Keep the existing ADC-domain linear interpolation as the fallback and as the first comparison baseline.
- Add a general per-channel monotone PCHIP mode as the preferred control-path candidate.
- Generate PCHIP slopes offline from the calibrated, monotonic table; firmware should only do segment search and cubic evaluation.
- Use the same measured calibration data to compare:
  - ADC-domain linear interpolation with adaptive knots.
  - log-ADC-domain linear interpolation.
  - inverse-power / `1 / sqrt(delta_adc)` style transformed interpolation.
  - ADC-domain PCHIP.
- Select PCHIP only when the validation report shows lower max error than adaptive linear without noise amplification.

Practical initial choice:

- Front-wall position matching: prefer front-sum PCHIP for translation and individual FL/FR PCHIP for yaw balance.
- Side wall control: start with side-channel PCHIP only after logs show stable side distance; raw derivative can still be better for wall-cut timing.
- Wall existence: use distance thresholds only behind a feature flag until raw/distance decisions agree in successful exploration logs.

Future compact model if needed:

- Add a log-model option similar to kerikun11:

  ```text
  mm = a * log(delta_adc) + b
  ```

  This can reduce table size, but LUT is easier to validate and safer for first integration.

## Calibration Strategy

Separate three things:

1. Dark/environment offset calibration
   - already implemented as off/on subtraction plus saved dark offsets.
   - keep this mandatory before distance calibration.

2. Base LUT creation
   - collect controlled data for each channel.
   - generate monotonic tables offline.
   - compile the table into firmware or params.

3. Per-unit / per-environment warp
   - save a small set of anchor corrections in NVM.
   - start with existing 3-point warp for front channels/front sum.
   - later extend NVM schema for side-channel warp if side distance proves useful.

Recommended measurement points:

- Front sensors: cover the actual correction range at 5 mm intervals. For the F413 fixture,
  the calibrated sensor-to-wall range is 2 to 62 mm at `2 + 5*n` mm.
- Side sensors: 25 to 80 mm, every 5 mm.
- For each point, record at least:
  - off ADC mean/std
  - on ADC mean/std
  - delta ADC mean/std
  - battery ADC
  - sample count

Use at least a few hundred samples per point, because the resulting table becomes a control primitive.

### F413 Front Calibration Applied 2026-07-19

The committed source data is:

- `params/f413_preorder/sensor_distance_calibration.csv`
- regulated bench supply
- fixture positions `2 + 5*n` mm from 2 through 62 mm
- repeated 7, 12, 17, and 22 mm measurements are retained as separate rows and averaged by
  the fitter
- the intermediate 2.5 mm-offset series is excluded because the fixture wall angle is less
  reliable at those positions

The generated FR, FL, and front-sum tables are in
`params/f413_preorder/sensor_distance_lut.c`. The fitter's PCHIP validation reported:

- FR: sample maximum error 0.241 mm; leave-one-out maximum error 0.580 mm
- FL: sample maximum error 0.211 mm; leave-one-out maximum error 0.373 mm

Front-wall matching uses a 7.0 mm target. It deliberately consumes the unwarped profile-LUT
values because the existing F413 NVM distance-warp anchors are not calibrated against this
fixture. The detailed `:` diagnostic prints profile-LUT and NVM-warped values separately.

Suggested CSV format for the host fitter:

```csv
distance_mm,fr_delta,fl_delta,r_delta,l_delta
40,4080,4090,,
40,4076,4087,,
45,3890,3340,,
...
25,,,3420,4090
25,,,3417,4086
```

Notes:

- Use `delta` values after off/on subtraction and saved dark offset correction.
- Front and side measurements can share one CSV; leave unused sensor columns blank.
- Keep separate rows for repeated samples. The fitter uses them to estimate noise.
- Use millimeters from the actual wall/reference plane used by the run logic, not from an arbitrary sensor package datum.

## Tooling

Add a host tool after the firmware logging path exists:

- `tools/logging/fit_sensor_distance.py`

Responsibilities:

- read distance-calibration CSV
- check monotonicity
- apply monotonic regression if needed
- generate C arrays or params snippets
- plot raw delta vs distance
- report conversion error and repeatability
- generate `params/f413_preorder/sensor_distance_lut.c`

Typical command:

```sh
python3 tools/logging/fit_sensor_distance.py sensor_distance_cal.csv \
  --emit-c params/f413_preorder/sensor_distance_lut.c \
  --max-error-mm 1.0
```

The generated profile function overrides the weak default
`sensor_distance_load_profile_luts()` hook. On the next F413 build,
`sensor_distance_init()` loads the built-in fallback table first and then
replaces the calibrated channels with the generated profile tables.

The first validation should compare:

- old raw ADC value
- converted distance
- repeated measurements under different ambient light
- repeated measurements after power cycle

## Trace And Logging

Phase 1 should not change the trace schema used by existing analyzers.

Instead:

- add a debug dump/test mode that prints both raw ADC and converted mm values.
- add distance fields to exploratory CSV only after the conversion is useful.
- when adding distance to FRAM trace, bump the trace schema and update CSV tools together.

For search logs, keep raw wall-read values and add distance values later, because raw values are still needed to debug threshold regressions.

## Rollout Plan

1. Add distance snapshot API and debug output.
   - no behavior change.
   - compare raw and converted distance in logs.

2. Add sensor-distance calibration capture mode.
   - fixed machine, no motor required.
   - collect tables for front and side sensors.

3. Use distance only in the front-wall match test mode.
   - target in mm.
   - translation error: average of FR/FL distance errors.
   - rotation error: FR/FL distance difference.
   - no heavy LPF in the control path; use bounded command and stable-count exit.

4. Add distance-domain logs to normal exploration.
   - compare wall read decisions against distance-domain thresholds.

5. Switch wall detection thresholds to distance domain behind a feature flag.

6. Switch wall control error to millimeter domain after enough logs.
   - both walls: compare left/right distance errors from wall-center reference.
   - single wall: use one-sided distance error with a separate gain.

7. Revisit wall-end detection.
   - raw derivative may remain best for edge timing.
   - use distance derivative only after raw/distance logs prove it is better.

## Safety / Fallback

Every runtime use must have a raw-ADC fallback until validated:

- missing NVM distance params: use default LUTs, mark source as default
- invalid distance: disable distance-based correction for that cycle
- saturated sensor: disable distance-based correction for that cycle
- out-of-range/extrapolated: allow logging, avoid control unless explicitly enabled

Do not replace search wall writes or wall control in one step.

## Acceptance Criteria

Before using distance conversion in normal exploration:

- converted distance is monotonic over the intended range.
- repeated measurements at the same distance are within about 1 mm near front-wall match distance.
- a 5 mm movement of the front wall changes the converted distance promptly and in the correct direction.
- front-wall match works without the previous delayed/oscillatory response.
- logs show raw ADC and distance-domain decisions agree on simple maze cases.
