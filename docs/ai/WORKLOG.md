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

### Follow-up: diagnose the 19 no-path configurations

- All 19 stop at the initial state: 16MM2015CX forces a right turn after one northward cell, but the model's 43 + 90 mm acceleration distance cannot reach the configured small-turn velocity. Required distances are 245 mm (mode2), 180 mm (mode3/7), and 137.8 mm (mode4).
- An ASan/UBSan diagnostic copy under ignored `build/f405_time_planner/` relaxes only the first connector's acceleration limit; all 19 then yield paths. Restricting the original model to small turns alone does not resolve any of them. No production code or params were changed.
- The old runner directly changes the speed reference in the short wall-end buffer, and `driveA` derives acceleration from requested boundary speeds and distance. The shared nominal model instead treats profile acceleration as a hard bound. Reference agreement therefore does not establish old-runner equivalence. Qualify the F405 start/short-connector cost model before changing tuning or enabling motion.
- No hardware access. Added the cause, numerical evidence and response order to the preparation document.

## 2026-10-10 Classic baseline flash and NVM preservation

- User connected classic r1 by USB-UART and explicitly requested firmware flashing. Motors/fan/motion were not authorized for assistant execution and no such commands were sent. No ST-LINK operation or application GO/reset was issued.
- Selected the clean baseline worktree at `b426b1f729ea9071c88a6de8cad7948a0dfe1a96`, branch `feature/classic-r1/competition-2026`. Release classic build succeeded; metadata SHA `b426b1f`, dirty 0, t1.0, embedded build time `2026-10-07T07:23:27Z`. Image 169,504 B, SHA-256 `82f42732326c4a19919b040cdda0b460b9d654819e92eb657c5ee3423e37bd7d`. Baseline source and branch remain unchanged.
- Port `/dev/cu.usbserial-DP04SWHO`, bootloader version 0x31, device ID 0x0413 (STM32F40xxx/41xxx). Initial handshake failed until the user selected BOOT/write and reset. A 115200-baud read then failed at `0x0801e300`; reconnection also failed. No erase/write had occurred. After a second user reset, 57600 baud completed all operations.
- Backup directory outside worktrees: `../classic-r1-backups/20261010-140208/`. Full pre-flash files `flash-before-57600-1.bin` and `flash-before-57600-2.bin` are each 1,048,576 B and byte-identical, SHA-256 `a8101bef1d1b6803653cffcfb29eb873039f05951356e1864b2579d8c1056564`. Earlier failed `flash-before-1.bin` is partial; do not restore it. Full backups, individual NVM sectors, candidate image, logs, manifest and hashes remain local.
- Successful command sequence (paths abbreviated to backup-directory filenames):
  1. `stm32flash -b 57600 -S 0x08000000:0x100000 -r flash-before-57600-1.bin PORT`
  2. `stm32flash -c -b 57600 -S 0x08000000:0x100000 -r flash-before-57600-2.bin PORT`; compare all bytes before erase.
  3. `stm32flash -c -b 57600 -s 0 -e 6 -w candidate-b426b1f.bin -v PORT`; sector 0..5 erase, all 169,504 application bytes written and readback-verified. An earlier `-S` plus `-e` invocation was rejected by host option validation before device access; the successful command uses page start/count.
  4. `stm32flash -c -b 57600 -S 0x080A0000:0x60000 -r sectors9-11-after.bin PORT`; all 393,216 B match pre-flash sectors 9..11.
- Distance/sensor/maze sector SHA-256 values are unchanged and recorded in the backup manifest. Firmware source, planner activation, control and tuning were not changed. Port closed; device left in bootloader. User must return BOOT to normal and reset; normal FW UART is 115200 baud. Boot log, sensor checks and floor/maze equivalence remain pending.
- Original F413 worktree's dirty files were not edited. These records live on the planner branch so the baseline branch stays exactly at the competition source.
