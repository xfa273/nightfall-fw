# mini_r3_0_unit001 commissioning

## Confirmed construction (user, 2026-09-06)

- Same external envelope and drivetrain geometry as mini_r2: retain 70 x 39 mm,
  35 mm front/rear overhang, 34.5 mm tread, 200 encoder counts/wheel revolution
  and the existing 14.13 mm effective tire-diameter starting value. Tire wear,
  mounting and load still need a short floor-distance confirmation.
- Changes: NFP-D0612-1-3.7 -> NFP-D0812-1-3.7 drive motors, suction fan,
  2S/3S-capable power hardware. R34/R35 physically replaced by 0 ohm;
  untracked CAD still says 220k. Left leads swapped on unit001.
- Same IMU/encoders. User confirms IMU on the centreline, 2.5 mm behind yaw centre.
  Main KiCad PCB has U4 at `(148.501098,107.4,180deg) F.Cu`; r2 has
  `(148.501098,105.003598,0deg) F.Cu`. Same pad1 corner/pin mapping confirms
  a relative 180-degree yaw mounting, not a footprint-origin convention change.
  CAD position delta is 2.396402 mm; use the user's yaw-centre-relative 2.5 mm
  dimension, not the difference between two CAD component coordinates.
- Forward acceleration is **-Y**, gyro yaw remains **+Z**. The bias-removed
  forward reading is referred to the yaw centre with
  `a_centre = a_sensor + omega_rad_s^2 * r_forward_mm`, where r=-2.5 mm.
  There is no forward angular-acceleration term for a centreline IMU. This does
  not correct gravity/tilt or cross-axis sensitivity. Turn control continues to
  use encoder velocity; neither gains nor turn parameters were tuned in air.
- R4/R5 battery divider changed from 22k/10k to 100k/27k. Nominal voltage is
  `adc/4095 * Vref * 127/27` (Vref assumed 3.3V, not precision-calibrated).
  3S 12.6V would produce about 2.679V at the ADC; this is a design calculation,
  not a 3S hardware qualification. Cell count must not be inferred from the
  unchanged machine ID. Cell-specific battery warnings/cutoff and 3S motor/fan
  limits remain work before 3S operation; BAT_WARN_ADC_THR has no F413 consumer.

Sources: the untracked `hardware/mini_r{2,3}_0/cad/kicad/` main PCB files;
`docs/datasheets/ism330dhcx.pdf` DS13012 Rev7; user-confirmed physical modifications.
Do not overwrite the CAD or claim its BOM includes the manual resistor rework.

## Autonomous fixed-machine scope

- User explicitly confirms lifted/secured, 8V/2A bench supply.
- Validate identity, IMU configuration/static bias, wall ADC stability, stop input,
  calibration provenance and FRAM log read/write path.
- Back up the preceding trace before any new run capture. Calibration/map/identity
  remain unchanged unless a separately documented repair is justified.
- Bounded motor sweep `{` checks 6/12/18% single-wheel L/R forward/reverse, then
  12% both forward/reverse, 300ms drive + 300ms coast each. MCU enforces the
  duration independent of host connectivity; every phase stops before printing.
  No PI/gyro feedback runs on a mechanically restrained chassis.
- Supply gate 7..9V using the board divider and nominal Vref; pressed stop,
  absent/stale ADC (>30ms), or encoder direction/stall aborts the sequence.
  Never use this sweep at 3S. It is a low-duty functional comparison, not a
  motor-current, stall-current, thermal, loaded efficiency or EMI qualification.
- `|` prints NVM status, sensor calibration and raw 256-byte prefixes of both
  calibration areas. It is read-only; it does not establish calibration quality.
- Known-distance wall/LUT/baseline/offset calibration cannot be completed with
  the current unspecified wall fixture. Do not save a new calibration from it.

## Gates after fixed-machine verification

1. Known wall placements: offsets, centre baselines, distance LUT/warp; then verify
   front-wall alignment. The current warped distance preview must be qualified
   before allowing wall-based motion.
2. User-authorized floor motion: distance scale, velocity/FF/distance loops,
   omega/angle loops, wall control, small90/U-turn and wall-end correction.
3. Small-maze exploration/map persistence, low-speed basic shortest, repeatability.
4. r3-specific turn/clearance tools and precomputed tables before KERI routes;
   the r2 table remains blocked for r3. Supply and fan-run integration are separate.

## Calibration finding

Read-only `|` found the exact historical NVM I/O-test fixtures: distance FL
`x=230/420/680, y=180/360/540`, FR `235/425/685`, front-sum `465/845/1365`,
and sensor bases `1111/1222/1333`, offsets `200/210/220/230`, gyro offset `1.25`.
These are not measurements. The sensor driver already ignored the sensor fixture,
but distance conversion applied the distance fixture and produced negative mm.
The F413 loader now rejects that exact fixture and clears only the live warp;
stored calibration bytes, real r2 calibrations and F405 behavior are unchanged.
`distance=MISS` is intentional until real calibration is saved. Unwarped r3 seed
LUT distances are still provisional, not a substitute for fixture calibration.
Raw prefixes are backed up in `tools/logging/logs/mini_r3_fixed_bench_20260906.log`.
Host regression: `sh tools/hil/run_f413_nvm_params_tests.sh`.

## Fixed-machine results, 2026-09-06

Tested application: `40e96ea DIRTY=1`, profile `0x00030001`, tune
`mini-r3-bench-t0.2`; RAM 274120 B / 320 KiB, Flash 365628 B / 1 MiB.
The dirty flag includes pre-existing user changes, not an unrecorded calibration.
Application sectors 0..6 were written and verified; identity/protected Flash was
not rewritten. ELF SHA-256:
`8d696117abe71dfa1136956e2d0f4058835526e458faafe691e6b014c52acf87`;
BIN SHA-256:
`e84a8e6c998504efcbd01c7b47a6695288f81929d3700e3cbfb00c092ab9c59c`.

### Motor and encoder

All 14 phases passed on each of three sweeps. Each entry below is the mean
signed encoder count during **300 ms drive only**, excluding the 300 ms coast;
these are not steady-state, loaded wheel-speed or gain measurements.

| Drive duty | Left forward | Right forward | Left reverse | Right reverse |
| --- | ---: | ---: | ---: | ---: |
| 6%, one wheel | +487.0 | +429.7 | -355.3 | -338.3 |
| 12%, one wheel | +1283.0 | +1226.0 | -1108.0 | -1074.7 |
| 18%, one wheel | +1998.3 | +1927.3 | -1844.7 | -1801.0 |
| 12%, both wheels | +1292.7 | +1227.3 | -1114.7 | -1072.7 |

- Correct signs at all duties; the unpowered wheel remained exactly zero in
  all single-wheel phases. Both motors' stronger direction is now forward.
- At 12% single-wheel drive, L/R forward ranges were 1280..1285 / 1218..1232;
  reverse magnitudes were 1104..1110 / 1073..1077. Mean L/R mismatch is about
  4.5% forward and 3.1% reverse, relative to their pair mean. At 6%, right-forward
  startup varied 412..439 counts and mismatch was larger; do not extrapolate a
  single fixed left/right multiplier from this test.
- Minimum sampled VBAT was ADC 2072, nominal 7.854 V; the 7..9 V gate never
  tripped. This does not measure motor current or rule out short supply spikes.
- TIM2 remains PSC=0 / ARR=1000 (about 99.9 kHz). No PWM, PID/FF, turn or fan-run
  parameter was tuned from the unloaded results.
- Intermediate `a99db92` sweep aborted before enabling any drive because HAL
  rejected re-starting an already-running encoder. `40e96ea` preserves running
  counters; the three successful sweeps above are on that fix.

### IMU, wall sensors, fan and logging

- IMU WHO_AM_I=0x6B and CTRL1_XL/CTRL2_G/CTRL3_C=0x74/0x71/0x44 passed before
  and after motion: 833 Hz, accel +/-16 g, gyro +/-4000 dps, BDU/increment.
  Static yaw bias was 0.17 dps; the 8-second angle observation ended at -0.1 deg
  (rounded display). Earlier static acceleration observation after the mounting
  fix ended at vx/vy/vz=-1/-14/+10 mm/s after 8 s. Neither proves dynamic IMU
  axis/scale/offset compensation; confirm forward acceleration and actual yaw
  on a later unrestrained, explicitly authorized motion test.
- Fixed-wall 512-sample deltas FR/FL/R/L=718/691/596/621, standard deviations
  2.55/3.85/3.36/3.39 counts, no saturation/extrapolation/low-signal flag.
  Rejected dummy warp gives `params=0`, FR/FL/SUM=23.54/30.10/26.98 mm in both
  unwarped and warped previews. Those distances have **not** been fixture-verified.
  Current effective offsets remain zero; side bases 1941/1989 are seed fallback
  values, not calibrated to this machine. `sensor=OK` describes a readable blob,
  not its suitability: the stored sensor dummy is ignored by the wall driver.
- Fan OP mode9/case4 completed 20/50/80% PWM for 1.2 s each and disabled PWM.
  SWD observed CCR1=200 during the first phase and zero at final check. No tach,
  airflow, motor-current or temperature measurement was available; this confirms
  the software output sequence, not suction performance or fan-induced IMU noise.
- The old 222-record trace was backed up before new capture. UART `x` wrote a
  stationary trace: 1212 records, no overflow/dropped/clipped/NVM errors. Reset
  and full binary readback validated payload checksum and contiguous seq 0..1211.
  Span 1528 ms, intervals 1 ms x894 / 2 ms x317: this main-loop idle test is
  **not** verification of the active controller's 1 kHz sampling. All logged
  motor outputs were zero; idle gyro/accel controller fields are not live.
  CSV dump includes machine/profile, tune, firmware SHA/dirty and mounting metadata.
- Initial binary capture used 6 s and stopped after 52403 bytes of the 126112-byte
  frame; retry with `--timeout 35` returned a complete 126184-byte raw file.
  The decoder's generic `magic not found` error was a truncated-transfer symptom,
  not FRAM corruption. Use a sufficiently long timeout for future full dumps.
- Both calibration prefixes (256 B each, covering complete current blobs) were
  compared before/after: byte-identical. Identity/calibration/maze were not saved
  or erased. Only the backed-up trace area was replaced by the idle capture.
- Final reset reached mode0 idle. SWD confirmed TIM2 CCER/CCR1/CCR3=0, both DIR
  pins and STBY low, TIM10 CCER/CCR1=0, CFSR/HFSR=0, VDD 3.24 V. Final `i,w,p`
  passed, VBAT ADC=2112, all walls detected without saturation, switch released.
  UART was closed. A pressed-switch functional test needs the user's physical
  operation and is not claimed from the released-state reading.

Logs under `tools/logging/logs/`:

- `mini_r3_fixed_precheck_20260906.log` (before mounting/warp fixes).
- `mini_r3_fixed_bench_20260906.log` (intermediate firmware, calibration backup).
- `mini_r3_fixed_verified_20260906.log` (three sweeps, fan, idle trace/CSV).
- `mini_r3_fixed_final_20260906.log`; boot `f413_boot_20260906_125459.log` and
  `f413_boot_20260906_125954.log`.
- `mini_r3_before_fixed_20260906.raw/.csv` (old trace backup).
- `mini_r3_fixed_idle_retry_20260906.raw/.csv` (complete new trace; the file
  without `_retry` is the incomplete first capture, retained as diagnostic evidence).

Host checks passed: F413 and both F405 builds; ASan/UBSan machine/measurement
tests (boot variants 0/2/3), 524288 PWM mapping cases, real/dummy/invalid NVM
calibration tests; route table freshness and `git diff --check`.

### Next user-assisted step

Keep 8 V / 2 A and the fan off initially. Establish known wall geometry to
calibrate dark offsets, side-centre baselines and front-distance conversion,
then verify these at unused distances. This precedes any wall-based floor motion.
After that, confirm a released/pressed stop input and actual IMU axis directions,
then proceed to low-speed floor distance/velocity tuning and turns in that order.
Normal fan-run integration, fan-on tuning, battery cell-specific safeguards and
3S output limits remain explicit later work; do not switch to 3S on this result.
