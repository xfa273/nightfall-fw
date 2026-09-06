# mini_r3 front distance calibration

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
  front-alignment tests. Side LUTs/bases and all mini_r2 settings remain unchanged.

## Reproduce

From the repository root, generate a comparison file (not a new calibration):

```sh
python3 tools/logging/fit_sensor_distance.py \
  params/mini_r3_0/calibration/front_centre_20260906.csv \
  --sensors fr,fl --emit-c build/mini_r3_front_generated.c
diff -u params/mini_r3_0/sensor_distance_lut.c build/mini_r3_front_generated.c
sh tools/hil/run_f413_machine_tests.sh
sh tools/hil/run_f413_nvm_params_tests.sh
```

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
