# classic r1 branch worklog

This branch starts from the 2025 competition baseline `v1.0.0` (`b426b1f`).
F413 development records remain in the original worktree and are not imported.

## 2026-10-07 F405 orthogonal planner preparation

- User reports the original 2025 nationals ROM still ran correctly in a maze a few days earlier. Robot and maze are currently unavailable; no live HIL is authorized or performed in this task.
- Created and backed up `feature/classic-r1/competition-2026` at exactly `b426b1f`; clean worktree `../nightfall-fw-classic-baseline`. Release classic build PASS, boot metadata SHA b426b1f/dirty0, GCC15.2.1. Current ROM/Flash backup and baseline floor comparison remain pending.
- Created separate `feature/f405/orthogonal-time-planner` / `../nightfall-fw-classic-r1`. Imported only pure motion calculations and host reference from `2677cae` in commit `89f67bf`; added compact 16x16 small90/large90/large180 planner, conservative F405 map adapter and opt-in linked preview. Default OFF, no motor/session dispatch. Original control/drive/run/search/OP/params/NVM/solver files unchanged.
- Implementation and host validation are in `512966b`. The clean competition worktree remains at `b426b1f`; planner work is not merged into that baseline.
- Fixed arrays use 36,876 B; connector cost cache is 1,536 B foreground stack. Optional Release ARM link includes old solver and logs: RAM 120,408 / 131,072 B, CCM 60,024 / 65,536 B, extra RAM 10,664 B, Flash 188,984 B. The post-link gate requires another 8 KiB of RAM and the application ending before calibration sector 9. Static stack frames: planner 2,376 B, wrapper 272 B, excluding callees and interrupts.
- Motion tests: 1,195 checks PASS. Compact tests: 6,137 checks PASS. Both use ASan/UBSan. Compact/general reference comparisons agree for 242 synthetic cases and 378 historical nominal-angle cases. Nine historical 16x16 mazes × 54 configurations: 467 paths, 19 no-path results. All 19 are reference-confirmed conditions in 16MM2015CX. Independent path replay checks walls, first goal, final heading/position, half-cell ownership and orthogonal grammar. Default mini/classic Release builds PASS.
- Prediction uses the nominal motion model, not full F405 per-code execution equivalence. Old runner time/velocity/stop mapping, body clearance, F405 latency/stack high-water and staged real motion remain gates before enabling a run. 32x32 unsupported and rejected; no automatic tuning/fallback.
- No ST-LINK/UART probe/reset/flash, motors/fan, NVM reads/writes or floor/maze runs. Original worktree's dirty F413 params, WORKLOG, flashing tools and other user files preserved.
- Reproduction and return steps: `docs/CLASSIC_R1_2026_PREPARATION.md`, `tools/f405_time_planner/README.md`.
