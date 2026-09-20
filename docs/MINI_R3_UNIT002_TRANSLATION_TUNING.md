# mini_r3 unit002 lifted translation tune, 2026-09-21 JST

Scope: user-authorized lifted/secured **translation only**, 8 V / 2 A bench
supply (2S-equivalent), debugger5V OFF, fan OFF. This is a coarse no-load tune,
not a floor, turn, suction,3S or maze qualification. Encoder direction is trusted
per the user's earlier visual confirmation. No rotation/fan shortcut was used.

## Communication and baseline

UART resumed after the user reseated the connection. No particular connector
or electrical root cause was isolated. STLINK066CFF545771485067013914,
UART `/dev/cu.usbmodem211202` at921600; IMU6B/config74,71,44 PASS and wall ADC
ready03/no saturation. Offsets remain FR82/FL66/R27/L43.

Before any tune, `|,V` saved the previous2544-record trace to
`tools/logging/logs/mini_r3_unit002_translation_reconnect_20260921.log` with
CSV completion marker. At this turn's initial read the map already reported
60 known cells; do not confuse that with the prior turn's empty-map observation.
Tuning intentionally replaces only the normal run-trace log. No identity,
wall/distance calibration or maze save/diagnostic-write commands were issued.

Baseline firmware974b280dirty / t0.5, UART `!` (velocity300 step,900ms),
then `V`: large sustained oscillation, filtered speed clipped at+-1200mm/s,
167/900 active samples at>=99% PWM; VBAT minimum1349 (~5.11V nominal conversion).
Average steady speed alone hides the problem:285mm/s with939mm/s standard
deviation. Test ended automatically; no repeat of this baseline was performed.
User was asked about noise, heat and current limiting; physical feedback was
not yet available when recording the electronic results.

The30ms accelerometer-assisted estimator is unsuitable for representing wheel
acceleration on a fixed lifted body: the body does not accelerate with the
wheels. Its delay and the inherited r2 gains are plausible contributors.
Estimator and PI were changed together, so this test does **not** isolate their
individual contribution or prove the same oscillation on the floor.

## Selected provisional r3 profile

`params/mini_r3_0/params.h`, `mini-r3-translation-t0.6`:

| Setting | Previous | Selected |
| --- | ---: | ---: |
| Velocity P, fan OFF | 0.8 | 0.08 |
| Velocity I, fan OFF | 0.012 | 0.001 |
| Static translation feedforward, PWM/1000 | 45 | 35 |
| Accelerometer estimate used for velocity feedback | 1 | 0 |
| Distance P, fan OFF | 6 | 2 |
| Distance I, fan OFF | 0.05 | 0 |

D remains0, velocity FF0.035 and acceleration FF0.004 are unchanged. I uses
the existing1ms error-sum convention, not an extra seconds multiplier. Feedback
now uses the existing3ms encoder LPF; the accelerometer-assisted estimate is
still logged. Angle/omega gains, wall values/LUTs, encoder signs, motor
polarity and all r2/F405 parameters are unchanged. The r3 model profile applies
to both registered units; unit001 remains hardware-blocked and was not tested.

For candidate tests only, `f413_control.c` PWM limit was temporarily180/1000.
For rc2 only, velocity set1 was temporarily500mm/s rather than1000; the OP label
still says1000, so use the actual trace reference. Both edits were removed from
the source before the final normal build; never infer1000mm/s qualification.

## Candidate results

All results below are encoder-equivalent motion with the body fixed. The steady
window excludes the first200ms and last100ms of active tuning. Quantized encoder
samples contribute to the remaining velocity standard deviation. End distances
are the firmware snapshots, not externally measured travel or coasting distance.

| Profile / command | Result | Steady velocity mean / SD | Peak absolute PWM |
| --- | --- | --- | --- |
| rc1, `!`,300 step | no sustained oscillation,283mm vs270 reference | 310 /16mm/s | 81/1000 |
| rc1, `"`,300 trapezoid | no sustained oscillation,286mm vs270 reference | 305 /15mm/s | 51/1000 |
| rc2, `(`,500 trapezoid | no sustained oscillation,179mm vs180 reference | 480 /15mm/s | 67/1000 |

Rc1 retains static FF45; rc2 uses35. Candidate firmware727cad8dirty, normal
destructive NVM guard LOCKED throughout. Rc1 minimum VBAT2063 (~7.82V), rc2
500-test minimum2075 (~7.87V); neither touched the18% cap. All trace captures
reported no overflow/drops/compact clipping/NVM errors. Logs:

- `tools/logging/logs/mini_r3_unit002_translation_rc1_20260921.log`
- `tools/logging/logs/mini_r3_unit002_translation_rc2_20260921.log`

Rc2 distance-trapezoid `^` (300mm/s,270mm target) completed, reporting265mm,
but the inherited distance gains produced a speed-reference oscillation (steady
target318/SD142mm/s, actual317/SD91mm/s; peak PWM64/1000). Therefore the final
candidate reduces distance P6->2 and I0.05->0 rather than accepting rc2 for the
cascade. The final normal-image checks below passed the coarse no-load gate.

## Final normal-image verification

- Source `0020011`, boot `0020011 DIRTY=1`, tune `mini-r3-translation-t0.6`.
  The dirty marker includes pre-existing host-tool changes, not a hidden bench
  firmware patch. Temporary PWM cap/set1 changes are fully removed. Common F413
  image RAM274120/Flash367124B; BIN SHA256:
  `b781e9d599547a9232f810e705d7af677c9de72f9a4c2ac66a57c199c70ba7ff`.
- `^`,300mm/s distance trapezoid:2579 complete rows,269mm vs270.002mm target,
  peak absolute PWM54/1000. Steady target velocity345/SD10mm/s and actual317/
  SD26mm/s versus rc2's target SD142 and actual SD91. Residual startup lag is
  still present; this is deliberately conservative and not a precision floor
  tune. No output saturation, minimum VBAT2063, stop completed normally.
- `!`,300mm/s velocity step:2496 complete rows, steady303.73/SD15.43mm/s,
  peak356mm/s (about19% transient overshoot), peak absolute PWM73/1000,
  steady PWM about45/1000, minimum VBAT2067. No sustained oscillation or PWM
  saturation. End distance274mm vs270.001mm is a velocity-only diagnostic,
  not a closed-loop stopping-position result.
- F413/F405 builds, ASan/UBSan machine resolver/runtime profiles including
  both r3 units and unchanged r2 gains, both262144-case PWM configurations,
  both NVM guard configurations and diffcheck PASS. The old host assertion
  requiring all models to useP0.8 was updated to verify model-specific values.
- All three application flashes (rc1, rc2, normal final) used the temporary
  native CubeCLI and verified only sectors0..6; identity sector untouched.
  Full128KiB identity readback equals registration backup. Both256-byte
  sensor/distance prefixes equal the pre-test dump; maze still reports60 known
  cells. No calibration/maze write, global diagnostic unlock or fan command.
- Complete command sequence: `w,i,|,V` baseline backup; `!,V`; rc1 flash/boot,
  `w,!,V,",V`; rc2 flash/boot, `(,V,^,V`; final flash/boot, `^,V,!,V,i,w,|`.
  Every drive command was a separately observed bounded forward-translation
  test; no unattended motor-command queue. Run hooks replace only trace data.
- Final IMU ID/config PASS, wall ready03/all flags0, VBAT2083. HOTPLUG
  CFSR/HFSR0, TIM2CCER/CCR1/CCR3=0, DIR/STBYlow, fanTIM10CCR1=0. Mode0 idle,
  UART closed. No physical noise/temperature/current-meter claim is made;
  the user's feedback question remained unanswered at completion.
- Final evidence `tools/logging/logs/mini_r3_unit002_translation_final_20260921.log`
  SHA256 `42f1a407715f12cbe9ad160a64d193e7d0240d08b0386d8fc4a9cb109043631b`.
  Initial backup/baseline log SHA256
  `c7b6e3d4c26b1db8cd94f2a5a7a57cdeabe81fca9ffe239a4deba7b76f396b11`.

## Remaining limits

- No-load gains are a starting point; floor friction/load, battery sag, heading
  and actual wheel-diameter/stop-distance accuracy still need short low-speed
  floor tests. Left/right wheel matching on a rigid fixture cannot validate
  gyro heading correction or straightness.
- Acceleration feedforward is not tuned by this work.
  The single-loop tune synthesizes velocity without the normal acceleration
  feedforward reference; floor trajectory tests must cover it separately.
- Do not use these results to raise speed, enable suction or switch to3S.
