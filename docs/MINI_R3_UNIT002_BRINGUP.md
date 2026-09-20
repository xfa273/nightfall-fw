# mini_r3_0_unit002 bring-up, 2026-09-20

This is a new assembly, not the unit001 board with the unresolved hot U2.
User confirms 8 V / 2 A bench power, initially no noticeable heating, and IMU
installed. Wall sensors, encoders, motors and buzzer are not installed.
No motor/fan test or floor-run clearance is implied by these results.

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
| Supply | ST-LINK reports VDD3.24V; VBAT ADC2100..2103, about7.96..7.97V using nominal3.3V reference; ripple/load qualification not performed |
| IMU | Repeated WHO_AM_I0x6B, CTRL1_XL/CTRL2_G/CTRL3_C=74/71/44 PASS |
| Stationary gyro | Local diagnostic offset0.38dps; printed corrected samples -0.38..+0.18dps; eight-second final angle0.0deg |
| Stationary acceleration | Mean57.8,-187.8,9906.0mm/s² (about1.01g magnitude); eight-second integrated residual12,5,-8mm/s; no persistent calibration write |
| Wall ADC | Scheduler ready0x03; unpopulated channels0..1 and no-wall flags; acquisition only, no optical sensing test |
| Switch | Released input reads high; physical pressed transition not yet tested |
| LEDs | All-on30s command completed and switched off; emitted PASS is software completion, visual confirmation still required |
| FRAM | Read calls return status0 and both256-byte calibration prefixes are all zero and unchanged after reset. Sensor/distance/trace absent. This does NOT prove physical read/write integrity; write/readback permission requested and pending |
| MCU faults / drive off | CFSR/HFSR0 after diagnostics; TIM2CCER/CCR1/CCR3=0, motor DIR/STBYlow; fanTIM10CCR1=0; buzzerTIM11CCR1=0 |

`maze=OK maze_known=0` is not proof of a valid saved maze: the legacy raw-map
fallback accepts zero data without a modern header. No maze has been measured
or saved on this board. No FRAM calibration, map, trace-format or append writes
were performed. Do not use a broad destructive diagnostic suite for follow-up.

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
- Pending: authorized bounded FRAM write/readback with backup, LED visual and
  pressed-switch checks, current/thermal baseline after IMU activation. C21
  population on the new board and SR resistor values have not been confirmed.
- After missing parts are installed: individual sensor calibration, lifted motor
  and encoder direction checks with fresh permission, then staged loaded tests.
