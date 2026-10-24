# F413 manual-operation LED indications

`ENABLE_AUTO_VIDEO_CAPTURE=0` enables status indications in the common F413
firmware for every registered model/unit (including mini_r2 and mini_r3).
The setting comes from the runtime-selected machine profile; there is no
model-specific LED gate. `1` retains the existing optical START/STOP signaling
and suppresses the additional wall/failure indications.

| State with automatic capture OFF | Indication |
| --- | --- |
| OP run-start delay | Front LED toggles every 250 ms during the existing 2-second delay |
| Guarded running | Front/right/left wall detection on LED1/rear-right LED2/rear-left LED3, updated by the existing 20 ms guard sample |
| Wall check, mode9 case3 or case6 | Same wall indication at the monitor's 500 ms sample interval; PUSH exits |
| Missing/saturated wall sample | Wall status LEDs off; the existing run guard still rejects the sample |
| Shortest route derivation failure, empty/invalid/preflight-rejected path, or turn-profile preparation failure | Front LED flashes ten times, 70 ms on / 70 ms off |
| Search route/save failure or search/path run abort | Same ten flashes after control has stopped (and path fan/trace cleanup has finished) |
| OP search/shortest run ended | Rear LEDs alternate every 400 ms until reset; input remains locked |

Selection LEDs, enter confirmation, calibration busy indication and photo mode
retain their existing behavior. Case6 becomes a continuous wall check only with
automatic capture OFF; capture ON retains its single snapshot action. The
failure sequence is skipped if capture is ON, control is still active, or the
fan is still active, so it cannot add a blocking wait to powered control.

The retained run-end lock already existed in F413 before optical capture.
History did not show a removal of all these indications at the optical-capture
introduction: current run loops and wall diagnostics had no wall LED output,
and path failures were text-only. This change reconnects the requested manual
status feedback without reverting control, sensor thresholds or NVM behavior.

Host validation:

- `python3 tools/hil/run_f413_status_led_tests.py`: production wall guard and
  legacy/KERI planner failure hooks; invalid samples retain their abort result.
- `sh tools/hil/run_f413_suction_tests.sh`: actual GPIO patterns, all wall
  combinations, capture ON/OFF, powered-control/fan suppression, pre-run timing,
  optical token waveforms, stopped-only path failure notification and the
  production OP run-end lock/400 ms alternating LEDs.
- `sh tools/hil/run_f413_search_distance_tests.sh`: both F413 run profiles.
- F413 and F405 builds, `git diff --check`.

No live hardware, motor/fan operation, firmware flash or NVM write was performed.
Physical LED visibility and floor/maze behavior remain unverified.
