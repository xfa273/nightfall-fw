# Codex Onboarding

This is the practical entry point for Codex working on `nightfall-fw`.

## Current Mission

`nightfall-fw` is firmware for STM32 micromouse machines. The active development line is moving proven STM32F405 machine behavior to the new STM32F413 `mini_r2_0` machine while keeping the existing F405 machines usable.

The repository is in a transition state:

- F405 stable machines: `mini_r1_0`, `classic_r1_0`
- F413 active machines: `mini_r2_0_unit001` and `mini_r3_0_unit001`
- F413 uses one binary with runtime model/unit selection from NVM; see `docs/F413_MACHINE_CONFIG.md`
- Current F413 state: buildable, hardware bring-up done, FRAM logging present, closed-loop/test entries present, tuning and real maze validation still active

## Start Here Each Session

1. Check the worktree:

   ```sh
   git status --short --branch
   git log --oneline --decorate -5
   ```

2. Read current state:

   ```sh
   sed -n '1,220p' docs/ai/STATE.md
   sed -n '1,260p' docs/ai/F413_PORTING_STATE.md
   sed -n '1,260p' docs/ai/HIL_SAFETY.md
   sed -n '1,220p' docs/ai/GIT_GITHUB_POLICY.md
   ```

3. For code changes, run the closest build/check:

   ```sh
   cmake --build --preset Debug-stm32f413
   cmake --build --preset Debug-stm32f405
   git diff --check
   ```

4. For solver/search work, run host simulation:

   ```sh
   tools/solver_host/run_solver_host.sh --explore-sim --max-steps 512
   ```

## Directory Map

- `platform/stm32f405/`: existing F405 platform and much of the original app/control/search code
- `platform/stm32f413/HM_Nightfall_f413_preorder/`: active F413 platform and migration target
- `params/`: machine/profile tuning constants and path-speed tables
- `nvm/`: identity, params, maze, and trace-log persistence abstraction
- `tools/flashing/`: ST-LINK and UART flashing tools
- `tools/logging/`: UART capture, FRAM trace CSV analysis, visualizer, PlotJuggler support
- `tools/solver_host/`: host-side solver/path/search simulation
- `hardware/mini_r2_0/`: F413 hardware references, KiCad/Eagle files, pin list
- `stable/`: known stable F405 machine configurations
- `docs/ai/`: AI operating state, HIL safety, Codex migration notes
  and Git/GitHub policy

## Important Current Inconsistencies

These are known and should be cleaned up opportunistically:

- Some older docs still say `Cascade`; current primary agent is Codex.
- Some older docs mention trace schema v1/v2/v3/v4/v5; current `nvm_trace_log.h` defines schema `0x00060000` and CSV `nightfall_trace_csv_v6`.
- Some names still use `f413_preorder`; official machine naming is `mini_r2_0`.
- `board/f413/` owns model/unit settings and runtime parameter selection. Platform pin initialization is still the shared mini r2/r3 layout; future differing layouts must be explicitly implemented.
- F405 is not yet a true single MCU-family runtime-selected binary; `nightfall_stm32f405` is currently an aggregate target for `nightfall_mini_r1_0` and `nightfall_classic_r1_0`.

## What To Preserve

- Existing F405 behavior is stable and must not be casually broken.
- F413 safety gates matter more than speed.
- Keep trace/log metadata tied to firmware build and machine identity.
- Keep NVM access behind `nvm/` APIs; do not spread raw Flash sectors or FRAM offsets into app code.
- Keep motor/fan actions behind explicit HIL safety checks.

## Recommended Next Work

The most useful next Codex work is usually one of:

- Continue splitting F413 `main.c` into small modules while preserving behavior.
- Use and extend safe F413 HIL scripts around reset, capture, non-motor smoke, trace dump, and analysis.
- Fix docs/tooling inconsistencies around UART baud, current trace schema, and Codex ownership.
- Improve F413 control tuning workflow using FRAM trace CSV and host analyzers.
- Move F413 from `f413_preorder` naming toward `mini_r2_0` when ready.
