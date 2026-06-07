# F405 To F413 Porting State

This file is the current map for Codex working on the F405-to-F413 migration.

## Goal

Bring the F413 `mini_r2_0` machine to F405-equivalent micromouse behavior:

1. reliable hardware and NVM bring-up
2. reusable trace/log capture
3. closed-loop straight and turn motion
4. wall sensing, wall-end, and wall control
5. exploration map update and persistence
6. shortest path generation from saved map
7. low-speed real maze exploration and shortest run
8. later: speed-up, large turns, diagonal motion, refined tuning

## Confirmed Complete Or Mostly Complete

- F413 target exists as `nightfall_stm32f413`.
- F413 CubeMX platform is under `platform/stm32f413/HM_Nightfall_f413_preorder/`.
- Identity read/write foundation exists. F413 identity lives in internal Flash sector 15 at `0x08160000`.
- F413 data NVM is currently FRAM-backed for distance params, sensor params, maze map, and trace log.
- F413 FRAM SPI2 access is protected against TIM5 1kHz IMU SPI2 conflict by masking TIM5 around FRAM transactions.
- Trace log schema is v3: `NVM_TRACE_LOG_SCHEMA_VERSION = 0x00030000`.
- F413 reuses F405 solver/path/maze logic through `f413_solver_bridge.c`.
- `tools/solver_host` can run solver and exploration simulation on the host.
- F413 OP UI has F405-style mode/case/sub selection and UART `P`/`E` wrappers.
- F413 run-session helpers and safe trace sessions have been split into `f413_run_session.c`.
- Recent refactors split F413 helpers/diagnostics/UI/run-session code out of `main.c`.
- Recent commits verified both `Debug-stm32f413` and `Debug-stm32f405` builds.
- Recent HIL checks verified safe boot state with motors disabled after flashing/reset.

## Active / Not Yet Finished

- Practical F413 gain tuning is still the next major work.
- Real maze exploration is not finished as a trusted competition flow.
- Wall control and wall-end behavior need more floor/maze data.
- Shortest-run UI flow from a real explored FRAM map needs careful validation.
- F413 `main.c` is still large and still owns important application routing.
- `board/mini_r2_0` does not exist yet; board separation is incomplete.
- Official name migration from `f413_preorder` to `mini_r2_0` is incomplete.
- F405 common-binary goal is not fully realized; current F405 build still produces mini/classic binaries.

## Key F413 Code

- `platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/main.c`
  - Still owns boot, peripheral init, app routing, many UART commands, search-step state, trace context callbacks.
- `platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_control.c`
  - TIM5 1kHz control loop, encoder distance/velocity, IMU omega/accel, cascaded control, motor output.
- `platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_wall_sensor.c`
  - Async wall ADC scheduler, IR emitters, offsets, wall snapshot.
- `platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_run_session.c`
  - Guarded waits, run abort flags, idle/motor/search-safe/shortest-safe sessions.
- `platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_op_ui.c`
  - OP UI names, state/input handling, action routing decision tree.
- `platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_trace_log.c`
  - F413 run-hook trace capture service.
- `platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_trace_diag.c`
  - Trace dump/selftest/CSV/bin diagnostic output.
- `nvm/nvm.c`
  - F405 internal Flash backend and F413 internal Flash/FRAM backend dispatch.

## F405 Reused By F413

F413 currently compiles these F405-origin sources:

- `platform/stm32f405/Core/Src/sensor_distance.c`
- `platform/stm32f405/Core/Src/solver.c`
- `platform/stm32f405/Core/Src/solver_params.c`
- `platform/stm32f405/Core/Src/path.c`
- `platform/stm32f405/Core/Src/maze_grid.c`

F413 provides missing globals and NVM bridge behavior in `f413_solver_bridge.c`.

## HIL Progress Boundary

Recent WORKLOG entries show:

- F413 hardware, encoder signs, motor mapping, FRAM logs, OP UI, and straight-only lifted tests have been verified in prior work.
- Recent module-split commits flashed F413 and checked safe boot state.
- Recent run-session split commits intentionally did not execute `z`, `j`, or turn-equivalent sessions after refactor.

Treat motor, fan, turn, search, shortest, and NVM-destructive operations as gated by `docs/ai/HIL_SAFETY.md`.

## Good Next Technical Steps

1. Continue reducing F413 `main.c` carefully:
   - Move search preview/step state into a module.
   - Move wall-end/wall-control glue into a module.
   - Move UART command dispatch into a module only after OP UI routing is stable.
2. Add safe HIL wrapper scripts:
   - reset-only + boot capture
   - non-motor status smoke
   - trace dump + CSV analysis
   - motor-required scripts with explicit `--requires-lifted`
3. Tune F413 basic loops using FRAM v3 CSV:
   - velocity inner loop
   - omega inner loop
   - distance outer loop
   - angle outer loop
4. Validate exploration without motion first:
   - `tools/solver_host/run_solver_host.sh --explore-sim`
   - UART `@` dump rendering
   - solver from search dump
5. Only after basic tuning: floor low-speed one-step exploration, then short maze exploration.
