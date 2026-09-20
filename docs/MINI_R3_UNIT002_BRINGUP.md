# mini_r3_0_unit002 bring-up, 2026-09-20

This is a new assembly, not the unit001 board with the unresolved hot U2.
Current update (2026-09-21): unit002 left motor polarity is now High, matching
its confirmed unit001-style wiring; four-direction encoder HIL passes. Unit001's
post-shield wall offsets have also been transferred at the user's request after
transplanting the same optical hardware; see the final section. Earlier
Low/pending-direction statements below are historical bring-up observations.
User confirms 8 V / 2 A bench power, initially no noticeable heating, and IMU
installed. Wall sensors, encoders, motors and buzzer are not installed.
The initial results below are non-motor checks. The 2026-09-21 follow-up at
the end records newly populated hardware tests; no floor-run clearance is implied.

## Registration

- User explicitly approved unit002 registration.
- MCU UID: `001D0038-32345108-36383936`.
- Identity: mini / board `0x00030000` / revision 3.0 / serial 2.
- Firmware: `3ee4d27 DIRTY=1`, normal protected common F413 build, 100 MHz.
- Profile: `0x00030001`, `mini-r3-wall-centre-t0.5`, provisional model defaults.
- Left/right forward IN2: Low/High. Unit001's physical left-lead swap is NOT
  inherited. Actual wiring/direction must be checked after motor installation.
- Unit001 FRAM calibration was not copied. The shared profile's wall LUT and
  run parameters are not calibrated/qualified for unit002.
- Backup: `build/identity/mini_r3_0_unit002_20260920/`. Entire sector15 was
  read and found erased before programming only the 68-byte identity with
  `--skipErase`; full-sector readback verified all other bytes unchanged.
  `identity.bin` SHA256:
  `9a8624642472d824f500d7d073a646177bd1af439bd92ed8d6a1ec1d567893ff`.
  Preserve the backup/manifest separately; these build artifacts are not in Git.

## Non-motor results

| Item | Observed result / limit |
| --- | --- |
| SWD / application Flash / UART | F413 device0x463; app sectors0..6 write/verify PASS, 921600bps boot and repeated software reset PASS; unit002 selected correctly |
| Supply | ST-LINK reports VDD3.24V; VBAT ADC2100..2103, about7.96..7.97V using nominal3.3V reference; user reports idle input74mA at8V after tests (0.592W whole-board input, not a component loss); ripple/load qualification not performed |
| IMU | Repeated WHO_AM_I0x6B, CTRL1_XL/CTRL2_G/CTRL3_C=74/71/44 PASS |
| Stationary gyro | Local diagnostic offset0.38dps; printed corrected samples -0.38..+0.18dps; eight-second final angle0.0deg |
| Stationary acceleration | Mean57.8,-187.8,9906.0mm/s² (about1.01g magnitude); eight-second integrated residual12,5,-8mm/s; no persistent calibration write |
| Wall ADC | Scheduler ready0x03; unpopulated channels0..1 and no-wall flags; acquisition only, no optical sensing test |
| Switch | Released input reads high; physical pressed transition not yet tested |
| LEDs | All-on30s command completed and switched off; user visually confirms all LEDs lit |
| FRAM | Authorized follow-up: four scattered256-byte trace-area samples, five patterns write/readback PASS; original1024B restored and verified. Both256-byte calibration prefixes unchanged. Sample test only, not full-memory or power-cycle retention qualification |
| MCU faults / drive off | CFSR/HFSR0 after diagnostics; TIM2CCER/CCR1/CCR3=0, motor DIR/STBYlow; fanTIM10CCR1=0; buzzerTIM11CCR1=0 |

`maze=OK maze_known=0` is not proof of a valid saved maze: the legacy raw-map
fallback accepts zero data without a modern header. No maze has been measured
or saved on this board. No FRAM calibration, map, trace-format or append writes
were performed; the bounded raw trace-area probe below restored its original
contents. Do not use a broad destructive diagnostic suite for follow-up.

## FRAM write/readback follow-up

- User explicitly authorized FRAM testing on this empty new board. UID and
  unit002 identity were checked before the test; motors/fan were not allowed
  or driven.
- Temporarily added a maintenance-only `}` command, restricted to this UID,
  unit2 and idle mode0 with no control/test/automatic trace activity. Test
  helper and host harness remain local in `build/hil_unit002/`; all temporary
  firmware source edits were removed after testing.
- Backed up four256-byte samples to RAM and UART before any write: physical
  FRAM addresses `0x60000`, `0x7FF00`, `0xBFF00`, `0xFFF00`, entirely within
  the trace area. Original bytes were all zero. Wrote/read back00, FF, 55, AA
  and a position/address-dependent pattern. Every pattern passed across all
  four samples, then original1024B were restored and verified byte-for-byte.
- Host harness with ASan/UBSan passed normal operation, backup read failure
  with no writes, transient read/write failure recovery and address-alias
  detection. Both protected/maintenance NVM-guard host tests passed.
- Built/flashed maintenance application, issued only the custom FRAM probe,
  removed the temporary hook, rebuilt/flashed normal protected application.
  Both app flashes verified and touched only sectors0..6. Build ID for both
  was `001d688 DIRTY=1`; final normal binary SHA256:
  `135ce79fdfc9559256f2c88ddb019e1b9301c3d2479f0e82de9bfc6bac497d89`.
  Maintenance binary SHA256:
  `972808450a3dc439873e4fc49cbe3fb67811b68e7bf0f1246470113979a7a2b2`.
- UART sequence: baseline `|`; maintenance flash/boot; `}`; normal
  flash/boot; `i,w,k,|`. `k` was sent only after normal LOCKED boot and was
  refused without writes. IMU ID/config PASS, ADC ready03 and VBAT2089.
- Full128KiB identity sector matches the registered backup exactly; both
  sensor/distance256-byte calibration prefixes match before/after. CFSR/HFSR0,
  motor PWM/enable/direction and fan duty off. Final mode0, UART closed.
- UART evidence: `tools/logging/logs/mini_r3_unit002_fram_20260920.log`, SHA256
  `99a16b255858e0b2bcbc1d1a79f9103a7901cc420e000074e5166e56e7707aeb`.
- This establishes basic nonzero write/read communication for the sampled
  locations, not integrity of every FRAM byte or retention after power removal.

## Reproduction and remaining checks

- ST-LINK SN `066CFF545771485067013914`, UART `/dev/cu.usbmodem211202`.
- Host tests: machine resolver/runtime boot (including new unit2), both PWM
  polarities, four identity-tool tests, F413/F405 builds and diff check PASS.
- Installed default CubeCLI is x86-only and cannot run on this host. The
  installed ARM-capable api/lib CLI was temporarily copied to
  `/tmp/mini-r3-cube-arm64.IPtRtX/bin/STM32_Programmer_CLI` with links to its
  existing library/framework, Data_Base and FlashLoader resources. Installed
  applications and the pre-existing dirty flash tool were not edited.
- UART sequence: capture unregistered SAFE after app update; provision identity;
  software reset; `i,w,p,|,I,c,l`; stopped-output SWD reads; software reset;
  `i,w,|`; close UART. Normal boot buzzer pattern ran into the unpopulated output;
  no buzzer diagnostic was commanded. No motor/fan/run commands were sent.
- Log: `tools/logging/logs/mini_r3_unit002_nonmotor_20260920.log`.
- Final state: mode0 idle, UART released, normal NVM diagnostic guard LOCKED.
- Pending: pressed-switch check and thermal recheck after IMU activation.
  Current baseline is74mA. C21
  population on the new board and SR resistor values have not been confirmed.
- After missing parts are installed: individual sensor calibration, lifted motor
  and encoder direction checks with fresh permission, then staged loaded tests.

## Populated drive/sensor follow-up, 2026-09-21 JST

- User reports wall sensors, encoders, drive motors and suction motor installed;
  explicitly confirms lifted/secured chassis, bench8V/current-limit2A and
  debugger5V OFF. Buzzer population was not reconfirmed.
- No firmware change/flash. Software reset identifies unit002, normal protected
  `001d688 DIRTY=1`, L/R forward IN2 Low/High, encoder signs+1/-1, PWM PSC0.
  IMU6B/config74,71,44 PASS, STLINK VDD3.24V, switch releasedhigh.
- Initial wall deltas FR98/FL100/R104/L101; later FR98/FL98/R77/L93,
  offsets all0, ready03, no saturation and no wall flags. After user placed
  walls/paper, `w` reports front/right/left wall flags all1 and no saturation;
  read-only `:`512-sample average FR1235/FL837/R1757/L1863, standard deviations
  4.60/3.61/3.67/3.55. After user removes walls, flags all0 and512-sample mean
  FR71/FL58/R35/L48 (SD4.54/3.38/3.09/2.83). All four optical channels respond
  and return to no-wall. No offsets were saved; distance/side-base calibration
  remains pending. Existing unit001
  distance LUT and side-control baselines remain provisional for this unit.
- UART single-side tests `6,7,8,9`, each12%/500ms plus300ms disabled coast:

  | Command | Intended direction | Left count | Right count |
  | --- | --- | ---: | ---: |
  | 6 | Left forward | -2192 | 0 |
  | 7 | Right forward | 0 | +2535 |
  | 8 | Left reverse | +2581 | 0 |
  | 9 | Right reverse | 0 | -2210 |

  Both motors generate encoder motion in both directions with no inactive-side
  counts. Left sign is opposite to intended direction: user visual direction
  confirmation is required before deciding motor-polarity versus encoder-sign
  correction. No closed-loop, sweep, floor run or automatic polarity edit.
- Navigated UART `P` x9, `E`, `P` x4, `E`, verifying each state, to mode9
  case4: fan20/50/80%, each1200ms, then duty0 and PWM stopped; firmware reports
  completion. Physical suction/rotation, abnormal noise/heat/current-limit
  observations are pending user reply (no fan tachometer).
- Reset to mode0 after fan test. SWD HOTPLUG shows CFSR/HFSR0,
  TIM2CCER/CCR1/CCR3=0, DIR/STBYlow, TIM10CCR1=0. Calibration prefixes remain
  allzero; trace status already400 records at initial read and unchanged after
  tests. No NVM writes/format/calibration or identity operations were requested
  or performed. Log: `tools/logging/logs/mini_r3_unit002_full_hw_20260921.log`.
  UART released while waiting for user direction/fan observations; all outputs
  remain stopped in mode0. Wall removal follow-up log:
  `tools/logging/logs/mini_r3_unit002_wall_clear_20260921.log`.

### Direction repeat and fan confirmation

- User confirms suction operation OK and requests another direction test,
  reporting that wiring should match unit001. This suggests the left motor
  polarity override may be needed, but physical direction is still unconfirmed.
- Repeated `p,6,7,8,9` with spoken/text direction announcements, a5s lead-in
  and about5s gaps. Same12%/500ms drive and300ms disabled coast. Counts:
  left forward-2195, right forward+2570, left reverse+2568,
  right reverse-2225; inactive encoder0 each time. Left sign mismatch repeats.
- No polarity/encoder setting change, build, flash, reset, fan rerun or NVM
  write. Final HOTPLUG CFSR/HFSR0, motorCCER/CCR1/CCR3=0, DIR/STBYlow,
  fanCCR1=0; UART closed, mode0. Log:
  `tools/logging/logs/mini_r3_unit002_direction_repeat_20260921.log`.

### Left motor polarity applied, 2026-09-21 JST

- User visually confirms only the left wheel ran opposite to commands and
  authorizes the matching setting. User also accepts the encoders as the
  direction reference for subsequent checks; no encoder sign change is needed.
- Source `18c5660`: added unit002-specific `MINI_R3_HW(true)` override. Right
  polarity, L/R encoder signs+1/-1, model defaults, unit001/r2, run parameters,
  IMU settings and NVM formats remain unchanged. Resolver/runtime-boot tests
  cover unit2 forward/reverse PWM polarity and retained encoder signs; ASan/
  UBSan machine suite and both262144-case PWM configurations PASS. Both F413
  and F405 Debug builds and diff check PASS.
- Rebuilt source commit and flashed/verified application sectors0..6 only.
  Boot `18c5660 DIRTY=1` selects unit002 with L/R forward IN2=1/1,
  enc=1/-1, PSC0 and normal NVM guard LOCKED. Binary SHA256:
  `d8185c3d84602e69ab6417fdd947878794e37e2742bc27d2639d9ab5c72b0e02`.
- Same authorized lifted/secured8V2A/debugger5Voff conditions. UART sequence:
  baseline `|`, app flash/reset, `w,p,6,7,8,9,i,|`. Each motor test12%/500ms
  plus300ms disabled coast. Left forward+2577/reverse-2233, right
  forward+2569/reverse-2236, inactive side0 throughout. All signs now correct.
  No closed-loop/floor/search/turn or fan retest was performed.
- IMU ID/config PASS, wall ADC ready03/no saturation. Full128KiB identity
  sector equals registration backup and both256B calibration prefixes are
  unchanged; trace metadata remains400 records. No FRAM or identity writes.
  Final CFSR/HFSR0, TIM2CCER/CCR1/CCR3=0, DIR/STBYlow, fanTIM10CCR1=0,
  mode0/UART closed. Direction mismatch is resolved; floor gains, metric
  odometry and individual wall calibration are not qualified by this test.
- Log: `tools/logging/logs/mini_r3_unit002_left_polarity_20260921.log`.

## Unit001 wall calibration transfer, 2026-09-21 JST

- User transplanted the same wall sensors and explicitly requested provisional
  reuse of unit001 calibration, with remeasurement if maze performance differs.
  Source is the post-shield read-only dump in
  `tools/logging/logs/mini_r3_left_motor_recheck_20260912.log`, not the obsolete
  pre-shield offsets. Sensor offsets are FR82/FL66/R27/L43; stored side/front
  bases and gyro offset are zero. The shared `mini-r3-wall-centre-t0.5` LUT was
  already selected and was not changed (body-centre front40..80mm, side23..80mm).
- Unit001's distance FRAM blob is an invalid old diagnostic fixture, not usable
  calibration; it was deliberately NOT copied. Unit002 distance data remains
  zero/MISS. Side-wall control still uses provisional fallback L1941/R1989:
  the transfer does not provide measured corridor-centre baselines.
- Backed up unit002's sensor/distance256-byte prefixes through UART; both were
  zero. A temporary pre-timer startup hook checked exact UID, unit/model/profile,
  NVM layout, expected68-byte format and empty256-byte destination before saving
  only the68-byte sensor blob. It verified load and full-prefix readback, with
  rollback on failure and safe halt on rejection. Already-matching data was
  idempotent. ASan/UBSan host checks passed identity/layout/data rejection,
  backup read failure, partial write failure/rollback, verification failure/
  rollback and repeat invocation; existing NVM params and both NVM guard tests
  also passed. Local helpers remain ignored under `build/hil_unit002/`.
- Built/flashed temporary application with destructive diagnostics OFF, observed
  exact transfer PASS, removed all temporary `main.c` changes, built/flashed
  normal protected application. Both flashes verified app sectors0..6 only.
  Final boot `974b280 DIRTY=1`, unit2, L/R forwardIN2=1/1, encoder1/-1,
  profile0x30001/t0.5, normal guard LOCKED. No persistent source change this turn.
  Normal build RAM274120/Flash367124B; binary SHA256:
  `d77a433046116b552edc3dba6a236b1bca8c900c7e9058edf1b5c35a9f93f8c3`.
- UART921600 sequence: baseline `|,w`; temporary flash/boot and `|`; normal
  flash/boot and `w,:,|`. Normal firmware loads offsets from NVM. Final corrected
  512-sample means FR17/FL6/R35/L63, SD3.78/3.10/3.32/2.67, all wall flags off,
  ready03 and no saturation in the current scene (not a calibrated fixture).
  Source and both post-transfer sensor256-byte prefixes match exactly, SHA256:
  `ef65dd1b727fee87a65d1ab159834bcf6ba640e77124df4011651d40bd773cde`.
  Bytes68..255 and distance prefixes are unchanged; full128KiB identity matches
  registration backup. Trace count remains400. No maze/trace/identity writes.
- No motors/fan/run commands were authorized or issued this turn. Final SWD
  CFSR/HFSR0, motorTIM2CCER/CCR1/CCR3=0, DIR/STBYlow, fanTIM10CCR1=0;
  mode0, UART closed. Persistence was checked across firmware reset/reflash,
  not a physical power-cycle or floor-run qualification.
- Log: `tools/logging/logs/mini_r3_unit002_sensor_transfer_20260921.log`, SHA256
  `4ecd6813e5b27f0a9287119b0c544078f4b5442b49bede32a284c294c1d3e543`.

### Remaining tuning order

1. Keep2S/8V and fanOFF initially. Check front45mm and corridor-centre side
   values; establish side baselines and check wall thresholds/end detection.
   Reuse transferred LUT initially; remeasure only if alignment is inaccurate.
2. Short straight floor runs: actual distance/stopping point, effective tyre
   diameter if needed, then larger-motor velocity feedforward/PID and heading.
3. Low-speed90-degree/U turns, front alignment and wall-end timing, then basic
   maze exploration/map persistence/return and conservative shortest runs.
4. Implement normal run fan start/stabilization/stop/abort integration before
   fan-on tuning. Currently `f413_ctrl_use_fan_on_gains()` always returns false;
   diagnostic fan operation does not qualify fan-on normal running. Tune gains,
   turn traction and verify current/temperature with suction separately.
5. Before3S, implement/verify cell-specific battery warnings/cutoff and motor/
   fan voltage/duty limits;2S results do not qualify3S.
6. Higher-speed/large/diagonal turns and the precomputed KERI route table need
   separate qualification. The r3 profile deliberately reports precomputed
   table incompatibility; normal r2-table preview/selection is blocked.

Body geometry, half-cell distances, encoder configuration and the r3 IMU
orientation/rearward2.5mm position are already represented in machine settings.
This does not establish dynamic equivalence to r2: larger motors, suction and
supply changes require tuning, and fan/3S/route integration still needs software.
