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

Fixed-machine measurements and final firmware build are recorded below when tested.
