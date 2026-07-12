# F413 HIL Safety Runbook

This file defines what Codex may do with an STM32F413 machine connected through ST-LINK/UART.

## Default Rule

Builds and host-only tools are always allowed. Live hardware actions are allowed only within the class below.

If a command can move motors, spin the fan, erase/write persistent NVM, overwrite identity/calibration/map data, or run search/shortest motion, stop and make sure the current user request explicitly calls for that class of operation.

## Green: Host-Only

Allowed freely:

```sh
cmake --build --preset Debug-stm32f413
cmake --build --preset Debug-stm32f405
tools/solver_host/run_solver_host.sh --explore-sim --max-steps 512
python3 -m py_compile tools/flashing/flash_stlink tools/flashing/flash_uart tools/flashing/make_identity_block.py
python3 -m py_compile tools/logging/*.py
python3 tools/logging/analyze_trace_csv.py tools/logging/logs
python3 tools/logging/analyze_turn_csv.py tools/logging/logs --target-angle 90
git diff --check
```

## Green: Connected, Non-Motor, Non-Destructive

Allowed when an F413 is connected and the task is HIL/debug related:

```sh
python3 tools/flashing/flash_stlink --list
python3 tools/flashing/flash_stlink --reset-only
python3 tools/logging/serial_terminal.py --list
```

UART commands that are normally non-motor:

- `p`: switch raw check
- `w`: wall sensor snapshot
- `n`: wall sensor distance-conversion snapshot
- `:`: detailed wall sensor distance-conversion snapshot
- `W`: wall-end monitor
- `i`: IMU WHO_AM_I
- `I`: manual IMU angle observation
- `c`: IMU acceleration observation
- `[`: search state / FRAM map consistency display
- `@`: FRAM search map dump
- `v`: bounded trace CSV dump
- `V`: full trace CSV dump; use only when the capture waits for the firmware dump-completion marker
- OP UI `mode9 case7`: non-destructive NVM status

Use the current baud from CMake unless a preset overrides it. At the time of this file, `Debug-stm32f413` uses USART1 at `921600 8N1`.

## Yellow: Flash / Reset / Trace Destructive

Allowed when the task calls for firmware validation or log handling:

```sh
python3 tools/flashing/flash_stlink --build
python3 tools/flashing/flash_stlink --image build/Debug/nightfall_stm32f413.elf
```

Rules:

- Build first unless intentionally flashing an already selected artifact.
- Do not use `flash_uart --erase all`.
- Do not write protected sectors unless the task is explicitly identity/NVM provisioning.
- Application flashing via ST-LINK is acceptable for firmware validation because identity/calibration FRAM data is not intentionally erased.

Trace-log formatting commands:

- `q`: formats the trace log area
- `k`: trace selftest
- `r`: sample append

These do not move the robot, but they overwrite or alter trace-log contents. Use only when losing the previous trace log is acceptable.

## Orange: NVM / Identity / Calibration / Maze Writes

Require explicit task intent. These can overwrite data needed for real runs.

Examples:

- `tools/flashing/make_identity_block.py`
- `tools/flashing/flash_uart --allow-protected`
- `nvm_identity_write()`
- UART `a`, `A`, `t`, `T` diagnostic save/load paths when they write test blobs
- UART `O`, `G`, `N`, `B`, `]` if they save or reset maze/search state
- `nvm_params_distance_save`
- `nvm_params_sensor_save`
- OP UI `mode9 case8`: side wall baseline sampling and sensor parameter save
- OP UI `mode9 case9`: wall sensor offset sampling and sensor parameter save
- `nvm_maze_save_map`
- `nvm_trace_log_format`

Before running, record what will be overwritten and how to recover or regenerate it.

## Red: Motor / Fan / Motion

Require explicit confirmation that the machine is lifted and secured, or that the user has explicitly authorized floor/maze motion for the specific step.

Motor/fan/motion commands include:

- `o` / `0`: motor driver pulse
- `6`, `7`, `8`, `9`: single-side motor and encoder checks
- `y`: motor trace session
- `1`, `2`, `3`, `4`, `5`: closed-loop motion tests
- `F`: armed button-run test
- `z`: search entry / solver path
- `j`: shortest entry / solver path
- `N`: stateful search step, may move straight or turn
- OP UI selections that map to motion, fan PWM, search, shortest, path tests, or tuning
- fan PWM check

Extra caution:

- `j` and shortest-safe can command left/right opposite motor directions.
- `z` and `j` may enter solver path when `NIGHTFALL_F413_REAL_RUN_PATH_ENABLED=1`.
- Tune shortcuts such as `!`, `"`, `#`, `$`, `%`, `^`, `&`, `*`, `(`, `)` can drive control-loop references.

## Recommended Safe HIL Loop

For a firmware change that should not move the robot:

1. Build:

   ```sh
   cmake --build --preset Debug-stm32f413
   cmake --build --preset Debug-stm32f405
   ```

2. Flash and reset:

   ```sh
   python3 tools/flashing/flash_stlink --build
   ```

3. Capture non-motor status:

   ```sh
   python3 tools/logging/serial_capture_csv.py --show-noncsv --send i,w,v --send-interval-ms 1500 tools/logging/logs /dev/cu.usbmodemXXXX 921600
   ```

4. Check logs/analyzers only if CSV was produced.

The same flow is wrapped by:

```sh
python3 tools/hil/f413_safe_hil.py flash-nonmotor-smoke --port /dev/cu.usbmodemXXXX
```

For a motor-required check, first state the exact command sequence and require lifted/secured confirmation.

## Safe Helper Scripts

`tools/hil/f413_safe_hil.py` provides only non-motor workflows:

- `list`
- `reset-capture`
- `nonmotor-smoke`
- `dump-trace`
- `flash-nonmotor-smoke`

Do not add motor/fan/search/shortest commands to this helper. Create a separate script with an explicit `--requires-lifted` gate if such automation becomes necessary.

## What To Record

In `docs/ai/WORKLOG.md`, record:

- firmware commit or `git describe --dirty`
- build command
- flash command
- UART port and baud
- command sequence
- whether motors/fan were allowed
- result and any CSV path
