# mini_r3_0_unit002 bring-up, 2026-09-20

This is a new assembly, not the unit001 board with the unresolved hot U2.
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
