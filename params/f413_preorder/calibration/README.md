# mini_r2 front distance calibration

## Active body-centre sweep, 2026-10-04 (t0.5)

The user explicitly identified `front_centre_20261004.csv` as **mini_r2**
measurements with the body centre as the distance origin, like mini_r3.
Apply them to the current common F413 firmware/profile, not the separate
August restoration checkout `build/r2_august_goal01` / commit `99e44d9`.
The date identifies acceptance of the supplied data; no independent acquisition
timestamp, repeat count, unit identity or offset readback was supplied.

Columns follow the UART contract: distance, FR, FL, R, L. Only FR/FL form
front LUTs; FR+FL is summed at matching distances. Retain all 15 points at
40..110 mm, step 5 mm. The R/L columns are preserved as provenance, not lateral
calibration knots. No extra offset subtraction, averaging or smoothing is applied.
All three sequences decrease strictly; interpolation remains PCHIP.

| Body centre to wall | FR | FL | FR+FL |
| --- | ---: | ---: | ---: |
| 45 mm (alignment target) | 1421 | 1816 | 3237 |
| 80 mm | 277 | 409 | 686 |
| 90 mm (nominal turn entry) | 199 | 306 | 505 |
| 95 mm | 174 | 266 | 440 |
| 110 mm | 112 | 185 | 297 |

Active input ranges are FR112..1922, FL185..2442, sum297..4364. Outside them,
the existing extrapolation/invalid flags remain. The unchanged strict delta>160
per-channel guard rejects the supplied FR readings at100/105/110 mm; the sum
also fails its>320 guard at110 mm. These points still define interpolation but
are not valid front-control observations. Raw saturation and wall-presence
checks remain required. The supplied40..95 mm pairs pass the signal/range gates.

Set `front_distance_body_centre=true`, `F_ALIGN_TARGET_MM=45` and
`F_ALIGN_TOO_CLOSE_MM=42.5`, preserving the2.5 mm close margin. Existing turn
references derive from the same target: nominal90 mm, search82/80 mm. There is
no sensor-position or wall-thickness correction. The old LUT at2..62 mm and
7 mm target had a different, insufficiently documented origin; the numeric
target change does not establish a universal+38 mm physical offset or permit
a same-distance intensity comparison with the old measurements.

The existing centre-reference gate skips v1 FRAM distance warps, which contain
no reference/source-LUT identity, and clears RAM warps. It neither reads nor
rewrites the distance blob. `distance=MISS` / `params=0` is consequently expected
while the compiled LUT is active. Sensor offsets, saved wall bases, maze and
identity remain untouched; a future warp needs compatible, versioned provenance.

The r2 motion policy remains0 (August-compatible algorithms within the current
firmware), with its existing gains, geometry, goals and side LUT/reference.
No r3 algorithm is opted in. mini_r3 profiles/LUTs and F405 sources are unchanged.
The historical `../sensor_distance_calibration.csv` remains an archive.

## Reproduction and validation

```sh
python3 tools/logging/fit_sensor_distance.py \
  params/f413_preorder/calibration/front_centre_20261004.csv \
  --sensors fr,fl --emit-c build/mini_r2_front_20261004_generated.c
diff -u params/f413_preorder/sensor_distance_lut.c build/mini_r2_front_20261004_generated.c
sh tools/hil/run_f413_machine_tests.sh
sh tools/hil/run_f413_nvm_params_tests.sh
python3 tools/hil/run_f413_motion_compat_tests.py
python3 tools/hil/run_f413_runtime_goal_tests.py
sh tools/route_precompute/run_tests.sh
cmake --build --preset Debug-stm32f413
cmake --build --preset Debug-stm32f405
```

Host checks passed: all knots; monotone, bounded interpolation across every
integer ADC; range/low-signal/saturation/single-invalid-channel rejection;
front-entry accessor and82 mm crossing; profile isolation and side conversion;
old-warp rejection without NVM read/write; runtime goal/profile selection;
both profiles' historical control/search/path/mode comparisons;14,210 route
checks and both MCU builds. Compatibility comparisons use the same **current**
parameters on both sides, verifying algorithm preservation, not old-calibration
equivalence. F413 RAM274400 bytes / Flash388996 bytes in the working build.

No ST-LINK/UART/flash/reset/motor/fan/NVM operation was performed. No physical
alignment or floor run has validated this calibration; numeric knot agreement
does not prove running stability or establish the cause of the earlier failure.
