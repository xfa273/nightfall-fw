# mini_r3_0_unit001 commissioning

Current source calibration: `mini-r3-wall-centre-t0.5`, remeasured after optical
shielding on2026-09-12, body-centre front LUT40..80 mm and sides23..80 mm.
Motor-recheck boot on2026-09-12 confirms `aba28d5 DIRTY=1` / t0.5 already on
unit001, with diagnostic writes LOCKED; no flash was performed in that recheck.
Stored/effective offsets are FR82/FL66/R27/L43. Acquisition-offset correspondence
and a non-motor45 mm HIL check remain pending; do not restore pre-shielding values.
Left drive now fails both directions in the bounded open-loop test; see the final
section before any further drive/closed-loop testing.
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

## Side body-centre LUT, 2026-09-06

Added the user's separate L/R jig sweeps, each 12 points at 23,30,35,...80 mm.
Original column order is distance,FR,FL,R,L: only L from the `l_delta` sweep and
R from the `r_delta` sweep are retained. Their independent means are stored in
`params/mini_r3_0/calibration/side_centre_20260906.csv` and generate the separate
`side_distance_lut.c`; front tables are byte-for-byte unchanged. PCHIP, all knots
and the non-uniform first interval are preserved. L720/R673 map exactly to 45 mm.

`mini-r3-wall-centre-t0.4` selects front and side centre-reference LUTs together.
Side reference metadata is separate from front, including a dump-time source
label on CSV meta. Lateral feedback and wall-end detection still use ADC deltas;
no side baseline, threshold, gain, NVM schema/warp policy or r2/F405 behavior was
changed. Existing >300 side-distance validity gating is retained, so the supplied
L70..80/R65..80 mm knots convert but are low-signal/invalid. A future distance
controller must review this policy and its gains with physical validation.

Host ASan/UBSan tests cover all 24 actual C side knots, every in-range integer
ADC (bounded monotonic PCHIP), range/no-wall/saturation rejection, the 300/301
validity boundary, unchanged front knots and legacy r2 side conversion. Machine
and NVM tests, F413 and both F405 builds, both generator/source exact comparisons,
route-table freshness and diff checks passed.

After the user closed their UART terminal, read-only `|` backed up both 256-byte
calibration prefixes. Built/flashed `d4ff9d6 DIRTY=1` (profile0x30001) through
ST-LINK066CFF545771485067013914, application sectors0..6 only, verify and software
reset successful. RAM274120 B, Flash366764 B, VDD3.24V. ELF SHA-256
`7645c3399b7e67ba9f61461aafb5d43726bf977f04df226f36ec7859e5a1fe8f`;
BIN SHA-256 `fb1b4ebd9781415e769fd5bf6781c4c7a8828eb1734f379fd30d11cd2494b555`.
Pre-existing unrelated worktree edits account for DIRTY=1.

UART `/dev/cu.usbmodem211202`,921600 captured reset to mode0, front/side
`body_centre`, `side_control=adc_delta`, and unchanged front target45/42.5.
Post-reset command sequence: `:`, `i`, read-only `|`, `p` only. One 512-sample
1025ms block gave FR/FL/R/L=0/0/173/29, with R/L SD2.75/3.46 counts. Right delta173
converts to78.70mm but is correctly low-signal/invalid; left29 is out of range
(extrapolated102.90mm is not a usable distance). Valid0, extrap0x1D, low0x1F,
saturation0. Fixture distance was not newly specified, so this is conversion and
validity-path HIL, **not** an independent physical accuracy check. IMU ID0x6B and
configuration74/71/44 passed, stop switch released.

Both complete calibration prefixes were unchanged: sensor SHA-256
`b1b8a0fdbd929eaa00983c23555e46ed66440018fbcfd1586c56fe1f6498376c`, distance
`2682a96bfb5aaf9a7614c3f6dcbba9e82a3d8b19153fb9b4ceef1adc74f87c7f`.
Offsets remain FR708/FL681/R551/L570; side bases were not saved. Trace still1212
records. No motor/fan/run/identity/calibration/maze/trace-writing command was
issued. Hotplug SWD confirmed TIM2 CCER/CCR1/CCR3=0, TIM10 CCER/CCR1=0,
DIR/STBY low, CFSR/HFSR=0. UART closed in mode0. Logs:
`mini_r3_side_centre_pre_20260906.log`, `mini_r3_side_centre_verified_20260906.log`
under `tools/logging/logs/`. Next physical work remains held-out/repeated-placement
distance verification and separately authorized raw side-base/control tuning.

## Recurrent sensor-calibration overwrite: diagnosis only, 2026-09-06

User reported unstable mode entry with all sensors approximately wall-free.
No firmware change, build, flash or calibration write was requested/performed.
Software reset to safe mode0 retained `d4ff9d6 DIRTY=1`, tune t0.4; read-only
`|`, `w`, `:`, `:`, `p` on UART921600 showed the sensor blob is now exactly the
NVM diagnostic fixture: bases1111/1222/1333, offsets FR220/FL230/R200/L210,
imu_z1.25. The wall driver recognizes/rejects that entire pattern and applies
defaults: **effective FR/FL/R/L offsets all0**, not those stored dummy numbers.

Two fresh512-sample1024ms blocks both averaged FR721/FL694/R597/L648. Their
pooled extrema were FR706..735, FL681..710, R585..608, L634..664; noise SD
2.27..3.79 counts is small relative to the missing background subtraction.
OP enters only when FR>=150 and FL<=250 for3 polls at40ms, so uncovered FL694
blocks entry. Distance LUTs are not consumed by OP. Baseline restoration to
the last verified offsets708/681/551/570 would nominally leave13/13/46/78
counts at this setup; do not silently recalibrate approximately wall-free data
and thereby change the ADC origin used by the measured tables.

The complete256-byte sensor prefix SHA changed from the last verified
`b1b8a0fdbd929eaa00983c23555e46ed66440018fbcfd1586c56fe1f6498376c` to
`059f6a046b8a8b8dc1e9ebf89f108be2f1e1407ca3c586e287e75e7d7ddee760`, exactly
the earlier dummy blob. Trace count also changed1212 ->16. UART `a` and `s`
call the diagnostic sensor save path, which writes this data into the real
calibration area; the current evidence establishes the matching payload, not
which command/source/time caused the overwrite. No such command was sent here
or during either LUT update. Safe prefix backup from the previous side-LUT HIL
can reconstruct the whole68-byte good sensor blob for a reviewed restore.

Current evidence log: `tools/logging/logs/mini_r3_offset_diagnosis_20260906.log`.
ST-LINK066CFF545771485067013914 software reset and hotplug read only; final
TIM2/TIM10 enable/PWM0, DIR/STBYlow,CFSR/HFSR0,VDD3.24V, switch released,
UART closed mode0. No motor/fan/run/trace/NVM write. Proposed next task is exact
calibration restore plus an explicit guard on destructive UART diagnostic saves;
both await user direction. The old trace backup remains available separately.

## Exact calibration recovery and destructive-diagnostic guard, 2026-09-06

The user explicitly approved restoring the previous good calibration and
preventing diagnostic overwrites. No motor, fan or run was authorized/issued.
The approximate no-wall scene was **not** recalibrated; changing its ADC origin
would invalidate the relationship to the measured front/side LUT samples.

Restored only the sensor blob (68 bytes, schema0x00010001, checksum0x1D6) to
the exact bytes in `mini_r3_side_centre_verified_20260906.log`: offsets
FR708/FL681/R551/L570, stored bases L/R/F0/0/0, gyro offset0 and reserved0.
Effective side bases still fall back to L1941/R1989; they remain unqualified.
The sensor256-byte prefix again hashes to
`b1b8a0fdbd929eaa00983c23555e46ed66440018fbcfd1586c56fe1f6498376c`.
The distance256-byte prefix remains
`2682a96bfb5aaf9a7614c3f6dcbba9e82a3d8b19153fb9b4ceef1adc74f87c7f`;
its historical warp fixture is still rejected, and compiled body-centre LUTs,
thresholds, control parameters and all identity data were not changed.

Recovery used a temporary boot block inside `main.c` USER CODE, after SPI/UART
initialization and before wall/control timers start. It required all of:

- UID `00280047-31335117-34313932`, valid mini board0x30000/unit1 identity,
  runtime profile0x30001, and the expected FRAM sensor-area layout;
- the entire currently observed diagnostic68-byte blob, including header,
  checksum0x59A, padding and reserved bytes, to compare exactly;
- normal sensor save/load and byte-for-byte readback to succeed (otherwise
  safe halt with outputs never enabled).

F413 sensor save uses the FRAM backend whose erase is a no-op; only the68-byte
sensor write was performed. No identity/maze/distance/trace restore was attempted.
The temporary image reported exact recovery PASS. The hook and its includes
were then removed completely (`main.c` has no final diff), and the normal
protected application was rebuilt/flashed/verified. There is **no persistent
automatic restore** that could reintroduce these values after future calibration.
The local recovery patch is retained only as an ignored audit artifact:
`tools/logging/logs/mini_r3_sensor_recovery_20260906.patch`, SHA256
`8fbe8499e74b264eda5e4fdcab0c801f1eb73d5f91a05de2337eeeeff80954f5`.

Normal builds default to `NIGHTFALL_F413_DESTRUCTIVE_NVM_DIAGNOSTICS=OFF`.
The actual diagnostic entry points refuse `a/d/s/m/t` test writes and
`q/Q/r/k` trace format/synthetic append/self-test before any NVM mutation or
trace-abort side effect. This also protects non-UART callers. An explicitly
opted-in maintenance build is required to enable them; there is no UART unlock.
Intentional OP calibration, normal maze saves and run logging are unchanged.
Older firmware remains unsafe for these letters. This closes the demonstrated
overwrite paths, but does not identify which earlier input/source caused it.

Host validation: ASan/UBSan tests link the real diagnostic entry points, sensor
serializer and trace serializer to mock NVM. Default locked build preserves
every NVM byte and produces eight refusals (q/Q share one entry point), while
ordinary sensor calibration remains writable. The opt-in path is also tested
in host memory only. Machine/LUT, NVM warp, exhaustive PWM tests, route-table
freshness, both F405 builds, F413 build and diff checks passed.

HIL used ST-LINK066CFF545771485067013914, target VDD3.24V, UART
`/dev/cu.usbmodem211202`921600, software reset and application sectors0..6
only; protected Flash sectors12..15 were not erased/programmed. Final image
reports `d5099d8 DIRTY=1`, t0.4, correct unit/profile and `[NVM-GUARD] LOCKED`.
Only after verifying that build/boot, sent `a,d,s,m,t,q,Q,r,k` separately and
received all nine refusals. Before/after `|` dumps match both calibration
prefixes exactly and trace count remains16. The temporary restore hook is not
present, so it cannot conceal an overwrite on a later reset.

Post-restore512 fresh samples in1024ms, current approximately wall-free scene:

| Channel | Effective offset | Corrected mean | Min..max | StdDev |
| --- | ---: | ---: | --- | ---: |
| FR | 708 | 10 | 0..26 | 4.03 |
| FL | 681 | 14 | 3..26 | 3.32 |
| R | 551 | 45 | 30..59 | 3.75 |
| L | 570 | 76 | 62..85 | 3.09 |

`w` confirms front/right/left=0/0/0, no saturation, and the stored offsets
are applied. Distance validity is0 with all signals low, as expected with no
walls. The FL<=250 entry condition has ample margin again; FR still needs a
hand to exceed150. Independent held-out distance/placement validation remains
pending, and side wall control is still raw ADC with unqualified bases.

Artifacts in `tools/logging/logs/`: `mini_r3_calibration_restore_pre_20260906.log`,
`mini_r3_calibration_restore_20260906.log`,
`mini_r3_calibration_guard_verified_20260906.log`.
Temporary recovery ELF SHA256
`02d3d569003492cef8128c7f3f7abc214b85525563e397e68438dae17656fc0b`
(Flash368060B); final normal ELF
`ff4d100fd43a0528948a36f3b0f51eed97eca506742434cd8a08ec919e1a4dbb`,
BIN `c022d94eb67b941cd924c63bb6fdd5413f72baec87b34a07cde84af456b6e68c`
(RAM274120B, Flash367140B). The reported dirty flag reflects pre-existing
unrelated worktree edits, not a remaining recovery hook.

A further software reset of the final normal firmware retained both exact
calibration prefixes and trace count16; `w` still applied the correct offsets
and detected no walls. SWD checks during final HIL found TIM2/TIM10 enables
and PWM compares0, DIR/STBYlow, CFSR/HFSR0. Left at mode0 idle with PUSH
released and UART closed. Requested FR-only hand/LED/beep confirmation from
the user; no reply or hand-entry event had been received by this handoff, so
physical OP-response confirmation is still pending.

## Shielding-adjusted wall LUTs prepared, 2026-09-12

User added light-blocking tape between LEDs and phototransistors to reduce
no-wall readings, then supplied three new jig sweeps. Interpreted columns as
the existing `distance_mm,fr_delta,fl_delta,r_delta,l_delta` UART contract;
requested confirmation that these remain corrected deltas and whether offsets
were recalibrated after shielding. No new no-wall data or numerical offsets
were supplied, so no baseline was inferred or saved. Preserve the acquisition
offsets rather than restoring the historical708/681/551/570.

Source commit `3cc013e`, profile `mini-r3-wall-centre-t0.5`:

- Front sweep uses only FR/FL, nine points40..80 mm in5 mm steps; FR+FL sums
  at matching distances. At45 mm: FR2508, FL2494, sum5002. Old85..110 mm
  points are **not** mixed in after the optical change.
- R sweep uses only column4; L only column5. Each has12 points23,30,35,...80 mm.
  At45 mm R640/L733. The other channels are not lateral calibration data.
- Curated source fixtures are `front_centre_20260912.csv` and
  `side_centre_20260912.csv` in `params/mini_r3_0/calibration/`. Old CSVs remain
  historical; no additional offset subtraction, smoothing or knot reduction.
- All references stay body-centre. PCHIP, target45/too-close42.5, raw side
  control, side bases/gains, wall/low-signal thresholds, NVM schema and guards
  are unchanged. Both side sweeps65..80 mm now fall below the existing
  delta>300 validity gate. Front beyond80 mm is extrapolated/invalid.
- F405 and mini_r2 profiles/tables are unchanged. This update does not qualify
  floor alignment, side-distance control, or unmeasured front distances.

Verification passed: fitter-generated C matches both sources exactly; real
F413 host converter/ASan/UBSan tests cover all9 front pairs+sum, all24 side
knots, every in-range integer ADC, monotonicity/bounds, endpoint rejection,
old front-tail rejection, no-wall/saturation, low-signal gates and unchanged
r2 behavior. NVM warp and destructive-diagnostic guard tests, F413/bothF405
builds, route freshness and diff checks also passed. Fitter leave-one-out is
only a numerical interpolation check, not independent physical accuracy.

Built artifact `3cc013e DIRTY=1`: RAM274120B, Flash367068B;
ELF SHA256 `0276550f1facb8e5f963d6b02d7fe404de60f9aa0a836b88c16faa9a55f6221e`,
BIN SHA256 `c1252462d1a375c445e49f40201b07ed891a49785602e0d200ec1d60d3a4558c`.
The CMake diagnostic-write option isOFF. No CubeMX/platform/NVM code changed.

HIL **not performed**: `/dev/cu.usbmodem211202` exists but is open in the
user's `tools/logging/serial_terminal.py` PID84832. Did not interrupt or share
that connection, reset, flash, issue UART commands or write NVM. Asked the user
to close it and, if possible, place the front wall45 mm from body centre.
Next: back up `|` calibration prefixes and read `w` offsets/identity before
application-only flash; preserve all NVM; compare prefixes and use `:` at45 mm
afterwards. All checks non-motor/non-fan. Acquisition-offset confirmation and
independent/repeated-placement physical validation remain pending.

## Left motor stopped: recheck, 2026-09-12

User reported no apparent left rotation during a running test and freshly
reconfirmed lifted/secured, bench8V/2A. Used the existing application without
source changes, rebuild, flash or NVM writes. ST-LINK066CFF545771485067013914,
UID00280047-31335117-34313932, VDD3.24V; UART `/dev/cu.usbmodem211202`,921600.
Boot after software reset reports `aba28d5 DIRTY=1`, mini_r3_0_unit001,
profile0x30001, t0.5, both forward IN2 high, encoder signs1/-1, PWM PSC0,
and `[NVM-GUARD] LOCKED`. Thus the previous section's pending-flash observation
has been superseded, but how/when the intervening flash/calibration occurred
was not observed. Current offsets are FR82/FL66/R27/L43, bases0, imu_z0.

Before reset or drive, read-only `|` and full `V` backed up the previous2112
trace records (schema0x60000,104 bytes/record; CSV37 columns, seq0..2111, done
marker received). Context is mode2/case0/sub1, test80, large90 turn. All left
encoder samples are0 while left command reaches1000; right encoder spans
-20..+21 counts/sample. Gyro angle remains within-0.157..+0.209deg despite
the turn target. This is not a suitable turn-gain test on a fixed chassis;
do not reproduce a closed-loop turn that cannot achieve its gyro target.

After reset to mode0 and `p,w`, command sequence was `6,p,8,7,9`; then, at
the user's explicit request to watch again, `6,8`. Each motor command is
12% for500ms followed by disable and300ms coast, with one wheel driven only:

| Direction | Signed encoder counts including coast | Repeat |
| --- | ---: | ---: |
| Left forward | 0 | 0 |
| Left reverse | 0 | 0 |
| Right forward | +2421 | not repeated |
| Right reverse | -2146 | not repeated |

The unpowered wheel always read0. `OK` in this legacy test means pulse
completion, not an encoder/motor pass. User confirmed that the repeated left
tests produced no movement at all and no visible supply-current change.
This is not merely an encoder-reporting symptom or the old weak-one-direction
asymmetry. No higher-duty pulse, fan, manual encoder window or run was issued.

Read-only SWD shows expected pin configuration: PA5 AF1 left TIM2_CH1,
PB10 AF1 right TIM2_CH3, PA4/PB1 DIR and PB2 STBY outputs; both encoders
enabled with PA6/7 and PB6/7 AF2. TIM2 PSC0/ARR1000 and PWM1 channel modes
match source. The drive helper, PWM mapping and encoder init have not changed
since the successful0906 baseline. Two attempted short-pulse SWD captures
only caught the disabled state; **active PWM compare values/waveforms were
not verified**. These checks do not prove electrical output from MCU/driver.

Prioritize power-OFF inspection of left motor leads and their board/motor-end
solder joints, free wheel/gear movement, and the left driver U2 plus reworked
0-ohm R35. Local main PCB nets confirm U2=left, U3=right, R35=left SR-to-GND2,
R34=right SR-to-GND2; CAD still says220k and was not edited. U2 nFAULT is
unconnected, so firmware cannot report its hardware fault status. Motor/lead
continuity and driver supply/input/output remain unmeasured; no failed part
is conclusively identified and no software repair is justified yet.

Final `w,|,p`: VBAT2103 (nominal7.97V), all wall flags0, released switch;
trace count remains2112. Both256-byte calibration prefixes byte-identical
before/after: sensor SHA256
`ef65dd1b727fee87a65d1ab159834bcf6ba640e77124df4011651d40bd773cde`, distance
`2682a96bfb5aaf9a7614c3f6dcbba9e82a3d8b19153fb9b4ceef1adc74f87c7f`.
Final SWD TIM2/TIM10 enables/compares0, DIR/STBYlow, CFSR/HFSR0. Left at
mode0 idle and closed UART. Evidence including original full trace and pulse
results: `tools/logging/logs/mini_r3_left_motor_recheck_20260912.log`.

### Subsequent idle U2 heating report: suspend powered testing

User reports no apparent mechanical fault, U2 heating merely on power-up,
and about0.23A at8V; historical healthy idle current is unknown. This is a
user observation, not measured by Codex. Requested disconnecting both bench
supply and debugger and allowing cooling; completion not yet confirmed.
No live device access, reset, UART, flash, motor/fan or NVM operation followed
this report. Input power is1.84W for the whole robot, not measured U2 loss;
being well below the2A supply limit does not clear a local thermal fault.

MPS MP6551 Rev1.0 datasheet pp4/10 specifies EN1=EN2 low enters sleep with
outputs off and internal circuitry disabled; typical sleep current10.5uA
and no-load awake current2.5mA are specified at VIN4V/25C, not measured at8V
here. SR-to-GND shorting is explicitly supported, so R35=0ohm is not itself
an invalid setting or a sufficient explanation for idle heating. Prior MCU
STBY-low readback does not verify voltage at U2 pins4/14 or its GND contact.
U2 internal damage, solder bridges or EN/GND contact faults are suspects;
heat conducted from adjacent components is not excluded. Check unpowered
assembly first, then isolated motor/board output continuity if needed.
Local PCB confirms left output pads TP3/TP4, right TP1/TP2. Motor windings
can normally measure low resistance, and body diodes/parallel board paths
affect in-circuit readings; do not condemn a driver from a single beep.
Further powered isolation, removal or replacement requires a reviewed next
step; no faulty part or original damage mechanism has been established.

Source: [MPS MP6551 datasheet](https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MP6551GQB/).

User subsequently reports diode-mode readings, black lead on GND then reversed:
TP3=0.507V/0.317V, TP4=0.506V/0.317V. These are not near-zero hard-short
readings, but do not clear U2's powered thermal fault. Both outputs match;
motor isolation was requested but not explicitly confirmed, and connected
windings can couple the outputs. Body diodes and shared-rail board paths also
prevent interpreting these as isolated transistor tests. Next request is the
same TP1/TP2-to-GND measurements on the previously functioning right side,
with all power/debugger removed and matched motor-isolation conditions.
No re-power, live tool access, firmware change or component removal performed
by Codex; U2 damage/assembly fault remains unconfirmed.

User then explicitly confirmed the left motor leads were disconnected and
provided the requested right-side comparison, in TP1/TP2 order with black lead
on GND then reversed: TP1=0.420V/0.344V, TP2=0.422V/0.345V. Left output
similarity is therefore not due to its motor winding. Each driver has nearly
matching output readings; left-to-right differences are84..87mV in the first
polarity and27..28mV in the reverse. These are not a specified pass/fail test
and do not establish a failed MOSFET, a hard short, or normal U2 operation.
Idle heating and absent physical rotation remain the stronger fault evidence.

Next unpowered checks, only if IC terminals are accessible without force:
U2-to-U3 VIN pin2 continuity, GND pin9 continuity, and each U2 EN pin4/14 to
the common U3 EN net. Local PCB confirms direct shared VBAT_SW/GND2/STBY
connections; expect resistance close to shorted test leads. Touching only a
PCB pad does not prove its joint to the IC. Inspect bridges/poor contacts before
replacement. If these checks reveal no repairable connection problem, removal
of U2 to inspect its underside/board pads followed by a new part is a reasonable
repair path, not a confirmed diagnosis; no removal/replacement is performed
here. Keep power disconnected and do not rerun motors. No firmware change or
live device access following the thermal report.
