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
- Trace log schema is v6: `NVM_TRACE_LOG_SCHEMA_VERSION = 0x00060000`.
- F413 reuses F405 solver/path/maze logic through `f413_solver_bridge.c`.
- `tools/solver_host` can run solver and exploration simulation on the host.
- The fixed-memory KERI time planner is connected to mode2 case6--9.  Its turn
  geometry, sampled poses, turn durations, and connector/approach durations are
  generated on the PC into a const Flash table, so the MCU does not repeat the
  expensive turn integration.  It uses #1--#5 normally and admits calibrated
  small90 only when nominal run-up or a stoppable terminal is unavailable.
  Run generation requires a successfully loaded FRAM maze and compiled
  `GOAL1..9`, then transactionally converts the route into the existing
  `path[]` runner grammar.  Read-only UART `+` HIL on the saved `G(1,0)` maze
  produced executable diagonal paths for cases 6--9 in 4.3--7.2 seconds
  (formerly about 115 seconds and no path).  UART `K` remains a read-only case8
  diagnostic with a centre 2x2 goal and a pinned 16MM2014CX fallback.
- F413 OP UI has F405-style mode/case/sub selection and UART `P`/`E` wrappers.
- F413 run-session helpers and safe trace sessions have been split into `f413_run_session.c`.
- Recent refactors split F413 helpers/diagnostics/UI/run-session code out of `main.c`.
- Recent commits verified both `Debug-stm32f413` and `Debug-stm32f405` builds.
- Recent HIL checks verified safe boot state with motors disabled after flashing/reset.
- F413 shortest-run path execution now covers all F405 path codes, mode/case feature
  mappings, per-mode wall-end thresholds, large-turn chaining correction, and the
  calibrated front-wall distance correction shared with exploration.
- The F413 maze/path limits build for both the current 16x16 configuration and a
  32x32 configuration by changing `MAZE_SIZE`; the path execution limit is 1024 codes.

## Active / Not Yet Finished

- Practical F413 gain tuning is still the next major work.
- Real maze exploration is not finished as a trusted competition flow.
- Wall control and wall-end behavior need more floor/maze data.
- Shortest-run UI flow from a real explored FRAM map needs careful validation.
- Mode 2 individual turns were endpoint-calibrated, but full-body swept
  clearance with the user-confirmed 70 x 39 mm envelope (35 mm front/rear and
  19.5 mm left/right from the coincident blue-label/turn centre) exposed the
  current R135-in entry as unsafe in a walled shortest path.  The new case6--9
  diagonal shortest flow therefore needs the clearance-aware simulator ->
  absolute-scene repeated-video retune before staged case6--8 floor validation.
  The historical non-zero lag artifact is diagnostic only: its endpoint-only
  `K=0.03303`, trajectory-start-corrected endpoint `K=0.0261`, and full-path
  `K=0.0178` (2.89 mm RMSE) disagree, so new absolute-scene data are required
  before any non-zero model may qualify a clearance recommendation.  The
  zero-slip firmware model is likewise limited to rough-design diagnostics
  until a measured swept-path artifact validates it.  The tracking reference
  is now fully measured: the blue centre is 10 mm and the red front label is
  2 mm above the maze floor, with a 24 mm horizontal baseline.  Because those
  planes differ, absolute video acceptance additionally requires a qualified
  stationary camera/label-plane calibration and a hash-bound corrected-CSV
  sidecar; uncorrected video remains diagnostic only.  The current 3D-printed
  ArUco top surface is user-confirmed 2 mm above the maze floor and is retained
  only for image registration.  The dense metric reference is the flush maze
  floor at 0 mm, so red 2 mm and blue 10 mm are both plane-corrected.  Absolute qualification
  requires HFR Recorder 0.5.7 or newer with AF off at fixed 1.05-diopter focus,
  stationary per-frame lens metadata, same-run report/video/Camera2-sidecar
  SHA/integrity, and matching calibration/run camera-setup fingerprints.
  Recorder 0.5.6 introduced geometry metadata but does not meet that final
  contract.  The first fixed-rig five-pose attempt exposed a more fundamental
  board-coordinate problem before label-plane fitting: four ruler-placed
  corner positions were observed 26--30 mm inward.  The permanent 90 mm
  orthogonal grid then proved that IDs 6/4/5/7 coincide with its outer
  intersections and span 720 mm, whereas the old layout declared 780 mm.
  After correcting that definition, the same Pixel clip measures mean pitch
  89.791/89.796 mm in X/Y and at most 1.014/0.922 mm non-uniform residual.
  The 30 mm-class failure was therefore a software layout error, not Pixel
  optical distortion.  Four outer markers remain registration anchors, not
  metric qualification.  Absolute work is still blocked on a distributed flush
  board lattice (32+ points with 8+ held out), a qualified dense metric map
  with <=1.0 mm held-out p95 and <=1.5 mm maximum error, and only then a new
  label-plane five-pose fit.  The failed five-pose set remains diagnostic and
  must not change turn parameters.
  Modes 3-7 remain parameter baselines and are constrained by explicit F413
  straight/diagonal/turn speed caps.
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
  - F413 run-hook trace capture service and guarded idle-scratch lease used by
    foreground-only route preview.
- `platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_route_preview.c`
  - Fixed-memory KERI time planner shared by the UART preview and mode2
    case6--9 path generation.  It uses #1--#5 plus an execution-gated small90
    fallback.  Run generation requires the FRAM maze and compiled goals;
    preview alone may use the built-in diagnostic fixture.  This module builds
    but never starts motor/fan execution or writes NVM.
- `platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_route_motion_table.c`
  - PC-generated const geometry, pose, turn-time, connector-time, and
    wall-end-approach tables for mode2 case6--9.
- `tools/route_precompute/`
  - Deterministic table generator, stale-input check, and numeric verifier.
- `tools/tuning/turn_clearance.py`
  - Host-only full-body wall/post clearance evaluator and candidate search.
    It uses the actual shortest-run turn profile, keeps uncertainty separate
    from free margin, and applies the same envelope gate to video trajectories.
    Video acceptance also requires a complete angle/out-offset, bounded tracking
    gaps, pre/post-roll, and endpoint/heading closure; incomplete captures fail
    analysis rather than being treated as clear.  Normalized video registration
    is limited to shape/repeatability; only board-coordinate absolute registration
    with a matching measured scene and verified blue/red height-correction
    sidecar can qualify physical body-to-maze clearance.
- `tools/vision/label_plane_geometry.py`, `fit_label_plane_camera.py`, and
  `apply_label_plane_geometry.py`
  - Fit a held-out-qualified stationary camera geometry from the 10 mm blue and
    2 mm red label planes, correct retained trajectory label centres without
    re-decoding video, and bind each result to its board/tracking/camera inputs.
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
5. Validate diagonal shortest running in stages: inspect the generated path,
   then run the low profile mode2 case6 before case7 and case8.  Current
   orthogonal/diagonal speed and acceleration ladders are
   `1000/800 @ 1000`, `1250/900 @ 3000`, and `1500/1000 @ 4000`
   (mm/s and mm/s^2).  Keep case9 as a later comparison profile.
6. Only after basic tuning: floor low-speed one-step exploration, then short maze exploration.
