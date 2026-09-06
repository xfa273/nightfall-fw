# mini_r3_0_unit001 commissioning

Current front calibration: `mini-r3-front-centre-t0.3`, body-centre-to-wall LUT
40..110 mm; see the final section for the 45 mm non-motor jig verification.
Earlier seed-LUT and provisional-offset results below are historical.

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

## Provisional no-wall offset and OP input recovery, 2026-09-06

Current calibration label: `mini-r3-open-wall-temp-20260906`. This section
supersedes the zero-offset/dummy-sensor status above, not the historical results.
Firmware/profile remain `40e96ea DIRTY=1` / `mini-r3-bench-t0.2`; no code change,
build or application flash was needed.

The user removed walls in every direction and requested a provisional no-wall
reference to restore hand-operated mode selection before building a distance jig.
Two 512-sample blocks returned on-minus-off means FR=701/702, FL=665, R=562,
L=575, with standard deviations 2.47..4.15 counts. Thus the old OP condition
`FR>=150 && FL<=250` could not become true: even uncovered FL was about 665.
The thresholds had not changed; the missing background subtraction was the blocker.

After backing up both 256-byte calibration prefixes, executed existing **non-motor**
OP mode9/case9 (1500 ms settle, 10 samples at 10 ms intervals). Saved:

| Channel | Provisional no-wall offset | Corrected no-wall mean / maximum (512 samples) |
| --- | ---: | ---: |
| FR | 700 | 2 / 11 |
| FL | 664 | 2 / 13 |
| R | 563 | 1 / 14 |
| L | 574 | 1 / 12 |

Offsets are subtracted from LED-on minus LED-off ADC, with negative results
clamped to zero. OP thresholds remain FR>=150 / FL<=250; exploration wall
thresholds and all runtime profiles are unchanged. The old sensor blob was the
known dummy, so the existing editor initialized defaults: stored centre bases
and stored gyro offset are now zero, not a measured centre/gyro calibration.
Effective side-control fallback remains L=1941/R=1989. Do not save the no-wall
reading as a wall-centre base (mode9/case8).

Readback validated the 68-byte sensor blob (checksum 0x1CD), and reset loaded
the same offsets. Corrected `w` detects front/right/left=0/0/0 with no saturation.
In safe mode0, the user covered only FR and released it three times; UART logged
three `execute mode=0 idle` events, and the user confirmed successful sound/LED
response. No UART `E` was sent during this physical-input check.

Sensor area was updated intentionally; distance calibration prefix was
byte-identical, trace still held 1212 records, and identity/maze were not written.
Before/after prefixes are in `mini_r3_open_wall_20260906.log`; post-reset data and
physical-input events are in `mini_r3_open_wall_verified_20260906.log` under
`tools/logging/logs/`. Sensor-prefix SHA-256 changed from
`059f6a046b8a8b8dc1e9ebf89f108be2f1e1407ca3c586e287e75e7d7ddee760` to
`00afa8d579a418414e21d1de763794428f19a982858c7f1569e7b2a5dd528695`.
The original complete blob can be reconstructed from the saved prefix for a
reviewed NVM restore; restoring dummy calibration is not normal operation.
Final SWD motor/fan enable/PWM, DIR/STBY and fault registers were zero; UART closed.

This is an offset for the **current physical setup**, not a qualified dark/optical
calibration. Large no-wall background remains unexplained, and earlier uncorrected
wall-present flags alone did not establish wall/no-wall contrast. On the distance
jig, fix the mounting height, surrounding surfaces and lighting, recheck the no-wall
offset, then collect distance-versus-corrected-delta tables and independently check
wall thresholds/centre bases. The current seed LUT distances are not valid calibration
results, and low/extrapolation flags at no-wall are expected. Keep floor wall-control
tests gated on that work.

## Front body-centre LUT and 45 mm jig verification, 2026-09-06

The user supplied 15 front-wall jig means at 40..110 mm in 5 mm steps and
explicitly changed the reference to **robot body centre to wall**, with a 45 mm
half-size cell-centering target. Used only `fr_delta` and `fl_delta`, already
offset-corrected. The no-wall FR/FL=0/0 row is absence, not a LUT knot; R/L values
from this front-wall fixture are not lateral-distance calibration data.
Curated source, provenance and reproduction command:
`params/mini_r3_0/calibration/README.md` and `front_centre_20260906.csv`.

All 15 FR/FL points are retained without smoothing or monotonic adjustment;
FR+FL is generated at the same distances. Existing PCHIP interpolation remains.
FR=2317, FL=2387 and sum=4704 each map exactly to 45 mm. The profile marks only
the front reference `body_centre`, changes `F_ALIGN_TARGET_MM` 7 -> 45 and
`F_ALIGN_TOO_CLOSE_MM` 4.5 -> 42.5, retaining the 2.5 mm backoff margin. Side
tables/bases, wall thresholds, motor/IMU/turn settings, mini_r2 and F405 behavior
are unchanged. Floor position/yaw-loop performance is not validated by this test.

The v1 NVM distance warp has no reference/source-LUT identity. For a centre-based
profile it is skipped, even on diagnostic load, and RAM warps are cleared without
writing FRAM. Thus `distance=MISS` and `params=0` are expected, not a missing flash
LUT. Genuine legacy warps remain accepted on mini_r2. Future centre-based warps
need a versioned provenance/migration path; do not use old sensor-origin anchors.
New front-reference CSV metadata is explicitly labelled `dump_time_profile`,
not a claim about the origin of a previously recorded trace.

The user closed the measurement terminal and reset the jig to 45 mm. Before any
flash, `p`, read-only `|`, `:` confirmed the existing sensor offsets were now
FR=708, FL=681, R=551, L=570, not the preceding temporary 700/664/563/574 values.
These user-side updates were preserved. Pre-flash corrected means FR=2310,
FL=2375 reproduced the supplied data, but the old LUT returned 1.33/4.30 mm.

Built and flashed `b0a5a88 DIRTY=1`, profile `0x00030001`, tune
`mini-r3-front-centre-t0.3` through ST-LINK `066CFF545771485067013914`.
Only application sectors 0..6 were erased/programmed; verification and software
reset succeeded. VDD 3.24 V, RAM 274120 B / 320 KiB, Flash 366260 B / 1 MiB.
The dirty flag covers the pre-existing unrelated worktree changes.
ELF SHA-256 `03ae228b692c95d63176aac28e5892701cd3100f02259360b151ab009c8e5c70`;
BIN SHA-256 `6520df472acd2a9c6b4cfa78425e0c22af159421cda2e565ec365ad06bad8767`.

UART `/dev/cu.usbmodem211202`, 921600 8N1, captured reset to mode0 and the new
`body_centre`, target 45.00 / too-close 42.50 boot line. Command sequence after
reset was `:`, `i`, `:`, read-only `|`, `w`, `p`; no motor/fan/run, calibration,
identity, maze or trace-writing command was issued.

| 45 mm jig, 512 fresh samples each | FR | FL | Front sum |
| --- | ---: | ---: | ---: |
| First mean delta | 2317 | 2385 | 4702 |
| First distance, mm | 45.00 | 45.04 | 45.02 |
| Second mean delta | 2316 | 2381 | 4697 |
| Second distance, mm | 45.02 | 45.12 | 45.06 |

Blocks took 1024/1025 ms; FR/FL standard deviations were 3.81/5.30 and
3.73/5.18 counts. Both `valid=0x1F`, extrapolation/saturation/low-signal=0, and
unwarped/warped front previews identical. Side-valid flags do not qualify the
unchanged side distance tables. This verifies repeat acquisition at one measured
distance, not independent full-range or sub-0.1 mm physical accuracy.

IMU WHO_AM_I=0x6B and configuration 0x74/0x71/0x44 passed. Final wall snapshot
VBAT ADC=2102 (nominal about 7.97 V), switch released. Read-only hot-plug SWD
confirmed TIM2 CCER/CCR1/CCR3=0, TIM10 CCER/CCR1=0, left/right DIR and STBY low,
CFSR/HFSR=0. UART was closed in mode0 idle.

Both full 256-byte calibration prefixes were byte-identical before/after:

- Distance SHA-256 `2682a96bfb5aaf9a7614c3f6dcbba9e82a3d8b19153fb9b4ceef1adc74f87c7f`.
- Sensor SHA-256 `b1b8a0fdbd929eaa00983c23555e46ed66440018fbcfd1586c56fe1f6498376c`.

Trace still has 1212 records, schema 0x00060000, 104-byte records; it was not
overwritten/dumped as new calibration evidence. Logs under `tools/logging/logs/`:
`mini_r3_front_centre_pre_20260906.log` and
`mini_r3_front_centre_verified_20260906.log` (includes boot and both averages).

Host ASan/UBSan tests pass for unknown/r2/r3 boot, all 15 actual C conversion
anchors, every in-range integer ADC (bounded monotonic PCHIP), range/no-wall/raw
saturation rejection, unchanged side conversion/r2 7 mm target, and real/dummy/
reference-incompatible NVM warp behavior without writes. F413 and both F405
builds, 524288 PWM mapping cases, route-table freshness, generator/source exact
comparison and `git diff --check` passed.

Next: collect held-out distances (e.g. near 42.5/47.5 mm and further out) to check
interpolation/placement sensitivity, then lateral-distance LUTs and side bases
using the appropriate fixture. The 40..110 mm front validity interval is not
extended by extrapolated preview numbers. For mini_r2, use a documented datum
translation only if the old reference is known; otherwise remeasure with this
centre-reference convention before migrating its unchanged legacy profile.
