# AGENTS.md

This file applies to the whole `nightfall-fw` repository.

## 1. First Read

Before changing code, read these files in this order:

1. `docs/ai/STATE.md`
2. `docs/ai/CODEX_ONBOARDING.md`
3. `docs/ai/F413_PORTING_STATE.md`
4. `docs/ai/HIL_SAFETY.md`
5. `docs/ai/GIT_GITHUB_POLICY.md`
6. `docs/NIGHTFALL_FW_DEV_POLICY.md`
7. `docs/NIGHTFALL_FW_2026_DEVELOPMENT_PLAN.md`

If these documents conflict, prefer `docs/NIGHTFALL_FW_DEV_POLICY.md` for project policy and `docs/ai/HIL_SAFETY.md` for live hardware safety.

## 2. Current AI Owner

Codex is now the primary development agent for this repository.

Historical Windsurf/Cascade/Codex-delegation assets are preserved under `docs/ai/archive/` and `.windsurf/`. Treat them as background unless a current document explicitly points to them.

## 3. Development Focus

The main active work is the migration from STM32F405 machines to the STM32F413 `mini_r2_0` machine.

Key facts:

- F405 machines are the existing stable `mini_r1_0` and `classic_r1_0` targets.
- F413 is built as `nightfall_stm32f413` and currently uses the `f413_preorder` params profile.
- F413 reuses selected F405 logic such as solver/path/maze code, but has its own platform, NVM/FRAM, operation UI, control loop, trace logging, and hardware helpers.
- The target architecture is one firmware binary per MCU family, with per-machine identity and runtime data read from NVM.

## 4. Build And Host Checks

Use these checks freely when they are relevant:

```sh
cmake --build --preset Debug-stm32f405
cmake --build --preset Debug-stm32f413
tools/solver_host/run_solver_host.sh --explore-sim --max-steps 512
python3 -m py_compile tools/flashing/flash_stlink tools/flashing/flash_uart tools/flashing/make_identity_block.py
python3 -m py_compile tools/logging/*.py
git diff --check
```

Do not assume a firmware change is safe just because it builds. For platform, NVM, logging, control, and OP UI changes, choose the closest host or HIL check from `docs/ai/HIL_SAFETY.md`.

## 5. Hardware Safety

Never run motor, fan, drive, turn, search, shortest, or Flash/NVM-destructive commands by accident.

Allowed without a fresh user prompt when an F413 machine is already connected and the task calls for HIL:

- ST-LINK probe listing
- ST-LINK software reset
- UART port listing
- boot log capture
- non-motor UART status/diagnostic commands listed as green in `docs/ai/HIL_SAFETY.md`
- ST-LINK flashing of the application image, if the build succeeded and the command does not erase protected NVM/identity sectors

Require explicit user confirmation that the machine is lifted and secured before any command that may move motors, spin the fan, or execute a run/session.

Require explicit task intent before writing identity, calibration, maze, trace format, or other NVM areas. Never use `flash_uart --erase all`.

## 6. Git And Records

The worktree may be detached. Check `git status --short --branch` before starting.

Commits and GitHub operations are allowed when useful, but keep them intentional:

- Use focused commits.
- Do not mix firmware platform changes and host tooling changes unless the task genuinely requires both.
- Record important AI/HIL events in `docs/ai/WORKLOG.md`.
- For F413 HIL, record build id, command sequence, whether motors were allowed, and result.
- Use `docs/ai/GIT_GITHUB_POLICY.md` for branch, commit, PR, and backup handling.

## 7. Code Boundaries

Avoid broad refactors unless they directly support the migration.

High-risk areas:

- `platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/main.c`
- `platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_control.c`
- `platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_wall_sensor.c`
- `platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_run_session.c`
- `nvm/`
- `tools/flashing/`
- `params/*/params.h`
- linker scripts and CubeMX generated files

When changing CubeMX-controlled code, keep edits inside USER CODE blocks or move logic to separate modules where possible. Do not casually edit generated init code or `.ioc` without documenting the regeneration intent.

## Review guidelines

- Treat unintended F405 behavior changes as P1 unless the PR explicitly states and verifies the migration impact.
- Treat F413 motor, fan, search, shortest-run, Flash/NVM, identity, calibration, maze, or trace-format risk as P1 when the PR lacks a matching HIL or safety note.
- For F413 control-loop changes, check that the PR describes whether tests were fixed-machine, straight-only, turn-capable, or floor/maze runs.
- For logging/tooling changes, check that CSV/binary trace metadata, row splitting, and `fw_git_sha`/dirty reporting remain usable for later HIL analysis.
- For CubeMX-generated files, check whether edits are inside USER CODE blocks or are explicitly documented as regeneration-intended.
