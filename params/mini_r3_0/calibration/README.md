# mini_r3 wall distance calibration

## Active optical-shielding measurements, 2026-09-12 (t0.5)

The user added light-blocking tape between each LED and phototransistor and
remeasured all four channels. `front_centre_20260912.csv` and
`side_centre_20260912.csv` are the **active** curated source fixtures; the
2026-09-06 files/sections below are historical and must not extend these LUTs.
They retain the user-supplied means without smoothing, rescaling or an additional
offset subtraction. Row interpretation is the existing UART `:` contract:
`distance_mm,fr_delta,fl_delta,r_delta,l_delta`. Offsets must remain the same as
during acquisition; do not restore the old 708/681/551/570 values merely because
they were used before shielding. This LUT update does not itself save offsets.

- Distances remain **body centre to wall**, with no sensor-position correction.
- Front sweep: use only columns2/3 (FR/FL). All9 points40..80 mm, step5 mm,
  are retained. FR+FL is summed at matching distances. At45 mm FR=2508,
  FL=2494, sum=5002. ADC ranges FR565..3048, FL595..3043, sum1160..6091.
- Side sweeps: use column4 only from R and column5 only from L; they are
  independent placements, not a simultaneous left/right measurement.
  All12 points23,30,35,...80 mm are retained. At45 mm R=640, L=733.
  ADC ranges R150..2507, L176..2919. Cross-channel readings are not LUT knots.
- Keep PCHIP, front target45 mm/too-close42.5 mm, raw side control, all control
  bases/gains/thresholds, NVM schema and the destructive-diagnostic guard unchanged.
  Do not turn the side45 mm measurements into saved wall-control bases here.
- Values outside each measured range remain extrapolated/invalid, including
  front distances beyond80 mm. The old85..110 mm front points are removed from
  the active table because they predate the optical change.
- The unchanged side delta>300 validity gate rejects **both** L/R65..80 mm
  samples as low-signal, even though these knots are in the LUT. No-wall and
  raw saturation checks are also retained. mini_r2/F405 tables are unchanged.

Reproduce the active tables and run the real firmware converter regression:

```sh
python3 tools/logging/fit_sensor_distance.py \
  params/mini_r3_0/calibration/front_centre_20260912.csv \
  --sensors fr,fl --emit-c build/mini_r3_front_20260912_generated.c
python3 tools/logging/fit_sensor_distance.py \
  params/mini_r3_0/calibration/side_centre_20260912.csv \
  --sensors r,l --emit-c build/mini_r3_side_20260912_generated.c
diff -u params/mini_r3_0/sensor_distance_lut.c build/mini_r3_front_20260912_generated.c
diff -u params/mini_r3_0/side_distance_lut.c build/mini_r3_side_20260912_generated.c
sh tools/hil/run_f413_machine_tests.sh
sh tools/hil/run_f413_nvm_params_tests.sh
sh tools/hil/run_f413_nvm_guard_tests.sh
```

All supplied channel sequences are strictly decreasing. No monotonic correction
or knot reduction is needed. Exact knot reproduction and leave-one-out fitting
are numerical checks, not held-out physical accuracy; repeated jig placement
and new distances near45 mm still need independent verification. See
`docs/MINI_R3_COMMISSIONING.md` for acquisition-offset provenance and HIL status.

## Historical front measurements, 2026-09-06 (t0.3/t0.4)

`front_centre_20260906.csv` is the curated source fixture for the FR/FL LUT,
transcribed from the user's jig measurements on mini_r3_0_unit001. It is not a
raw trace capture. Columns contain the reported averaged, offset-corrected
`fr_delta` and `fl_delta`; do not subtract the no-wall offsets a second time.
The pre-flash readback on 2026-09-06 has offsets FR=708 and FL=681
(R=551, L=570), updated since the earlier provisional 700/664 calibration.
These existing values are preserved, not recalibrated by the LUT update.

## Reference and scope

- `distance_mm` is **robot body centre to wall**, as set on the user's jig.
  The requested half-size cell-centering target is 45 mm. Do not introduce an
  extra sensor-position or wall-thickness correction to these supplied values.
- All 15 measured points, 40..110 mm at 5 mm spacing, are retained exactly.
  FR, FL and FR+FL each have a LUT; the sum is formed at matching distances.
  Existing shape-preserving PCHIP interpolation is retained.
- The supplied no-wall row FR=0, FL=0 is an absence check, not a zero-distance
  knot. The supplied R/L columns are deliberately excluded: a front-wall jig
  distance is not a lateral sensor calibration distance.
- Input ranges are FR 198..2491, FL 209..2536, sum 407..5027. The low-level
  converter can return an extrapolated number outside these ranges, but the
  wall-distance validity mask rejects it. Raw ADC saturation also invalidates
  a measurement. Check validity, not just the displayed number.
- These means alone do not provide repeated-placement accuracy or noise.
  Recheck 45 mm and held-out distances (especially near 40..50 mm) before floor
  front-alignment tests. The front calibration did not change side LUTs/bases;
  the subsequent side LUT is documented below. All mini_r2 settings remain unchanged.

## Historical front reproduction

From the repository root, generate a comparison file (not a new calibration):

```sh
python3 tools/logging/fit_sensor_distance.py \
  params/mini_r3_0/calibration/front_centre_20260906.csv \
  --sensors fr,fl --emit-c build/mini_r3_front_generated.c
```

This reproduces the old table only; it no longer matches the active t0.5 source.

No monotonic adjustment is needed. The fitter's `std=0,n=1` means one supplied
mean per distance, not noiseless hardware; exact fit at the input knots is not an
independent accuracy test.

Profile `mini-r3-front-centre-t0.3` marks only the front distance reference as
`body_centre`. Its alignment target is 45 mm; the too-close/backoff threshold is
42.5 mm, preserving the previous 2.5 mm margin, pending floor validation. Existing
v1 FRAM distance warps have no reference/source-LUT identifier, so the F413 loader
skips them for this profile and clears RAM warps without changing FRAM. Thus
`distance=MISS` / `params=0` is expected even though the flash LUT is active.
A future centre-reference warp needs versioned provenance before enabling it.

mini_r2 retains `legacy_profile`, its old LUT and 7 mm alignment target, and
compatible legacy FRAM warps. To migrate it, use a documented old-to-centre
reference offset only if the old datum is known; otherwise remeasure with the
centre-reference jig. Do not infer a universal +38 mm sensor offset merely from
the change of alignment target.

## Historical side LUT, 2026-09-06 (t0.4)

`side_centre_20260906.csv` contains **two independent jig sweeps**, matched by
body-centre-to-wall distance for convenience, not simultaneous left/right walls.
The user's original rows are `distance_mm,fr_delta,fl_delta,r_delta,l_delta`:
use only the **fifth column** from the `l_delta` sweep and **fourth column** from
the `r_delta` sweep. All other channels are excluded. No additional no-wall
subtraction or front-sum generation is performed on these rows.

All 12 points per side are retained exactly: 23, 30, 35, ... 80 mm. The first
interval is 7 mm, the rest 5 mm. At 45 mm, L=720 and R=673. ADC ranges are
L=188..2498 and R=165..2475. PCHIP is retained without smoothing/point reduction.
The separate generated `side_distance_lut.c` is loaded alongside the unchanged
front LUT by the r3 profile hook, allowing either sweep to be regenerated without
overwriting the other.

```sh
python3 tools/logging/fit_sensor_distance.py \
  params/mini_r3_0/calibration/side_centre_20260906.csv \
  --sensors r,l --emit-c build/mini_r3_side_generated.c
```

Profile `mini-r3-wall-centre-t0.4` marks both front and side LUT references
`body_centre`. The front table/target/warp gate is unchanged from t0.3. Side
distances are currently used only for conversion/diagnostic display, **not**
feedback: lateral control and wall-end detection still use corrected ADC deltas,
with unchanged bases, thresholds and gains. Do not turn the 45 mm means into
stored side-control bases as part of this LUT-only update.

The existing distance-validity gate additionally requires side delta **>300**.
Thus all 23..80 mm knots convert, but the supplied L readings at 70..80 mm and
R readings at 65..80 mm carry a low-signal/invalid flag even while in LUT range.
No-wall/out-of-range and raw ADC saturation are rejected independently. This
policy is deliberately unchanged; future distance-based control must review
signal quality/validity and gains, not merely substitute millimetres for ADC.
Validate held-out jig distances and repeated placement before that integration.
