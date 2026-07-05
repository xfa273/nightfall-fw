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

## Conversion Model

Initial model:

- Use monotonic per-channel LUTs with linear interpolation.
- Clamp or mark invalid outside the calibrated range for control use.
- Keep unwarped values in logs for diagnosis.
- Apply 3-point distance-domain warp only after LUT conversion.

This matches the practical shape used by satoshihamasuna while reusing the Nightfall `sensor_distance` module.

Future model if needed:

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

- Front sensors: 40 to 125 mm, every 5 mm.
- Side sensors: 25 to 80 mm, every 5 mm.
- For each point, record at least:
  - off ADC mean/std
  - on ADC mean/std
  - delta ADC mean/std
  - battery ADC
  - sample count

Use at least a few hundred samples per point, because the resulting table becomes a control primitive.

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

