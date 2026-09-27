# mini_r2 August firmware / goal (0,1) HIL — 2026-09-27

User reports that the motion-policy restoration in PR47 did not fix physical
exploration. At the user's explicit request, flashed the complete pre-r3
August implementation with goal (0,1), not the newer binary's compatibility mode.

## User follow-up / paused

2026-09-27 16:40 JST: the user reports partial improvement with the August
firmware, but continuing instability. Hardware is a possible contributor, not
an established diagnosis. At the user's direction, pause r2 investigation,
changes and selective adoption of r3 features; prioritize mini_r3. Retain the
last-flashed August goal(0,1) image and all comparison artifacts. No additional
hardware commands or firmware changes were performed for this status update.

## Exact image

- Base: `57a9c3be207e368f0a4f57fd1688ab59e6f367b6`, 2026-08-29 17:31 JST.
- Image commit: `99e44d9`, branch `exp/f413/20260927-r2-august-goal01` (pushed).
- Separate worktree: `build/r2_august_goal01`; current development checkout and
  all existing user edits retained.
- Source delta: `params/f413_preorder/params.h` GOAL_X 1→0, GOAL_Y 0→1.
  Route-table regeneration changes input fingerprints only; no numeric table or
  control/search/IMU/NVM implementation changes. All other goals remain unused.
- Boot: `GIT=99e44d9 DIRTY=0`, `nightfall_stm32f413`, Debug.
- Build time: 2026-09-27T07:24:46Z; RAM270224 B, Flash343548 B.
- ELF SHA256: `4c8d8b2d8601dedfe9f82d79c20df4c1b8a2cc4f29b955d91822de6e7deb215d`.
- BIN SHA256: `eb55acfb150f27aaf7af3eb0b6d946d41d7c10f7e5d82132eb32b23984b7b87a`.
- Artifact: `build/r2_august_goal01/build/Debug/nightfall_stm32f413.elf`.

## Target and command sequence

Target identity read before writing: mini_r2_0 unit001, board0x00020000,
MCU UID00250029-31335117-34313932, STM32F413 device0x463.
The legacy identity stores zero UID; its complete131072-byte sector was backed up.
ST-LINK V3MINIE SN003B00273234511537333934 / V3J16M8, VDD3.23–3.25V.
UART `/dev/cu.usbmodem2124202`, 9216008N1.

1. Current tools: probe listing, identity inspect; read-only UART `|` captures
   NVM status and distance/sensor blob prefixes. No provisioning or erase command.
2. HOTPLUG1000kHz uploads: MCU UID12B, identity sector128KiB, current app1MiB.
3. Isolated worktree `cmake --preset Debug`, `cmake --build --preset Debug-stm32f413`;
   host exploration explicitly reports start(0,0), goal(0,1), success in1step.
   Route generator `--check` and `git diff --check` PASS.
4. After committing goal change, rebuild clean artifact and capture UART while flashing:

   ```sh
   python3 tools/flashing/flash_stlink \
     --image build/r2_august_goal01/build/Debug/nightfall_stm32f413.elf \
     --sn 003B00273234511537333934 --freq 1000 --mode UR --reset-mode HWrst
   ```

   07:25:26UTC: one write, application sectors0–6 only; vendor verify and reset PASS.
5. Boot identifies mini_r2_0_unit001 and expected clean commit, ADC scheduler and
   control initialization succeed, `[OP-UI] ready mode=0 idle`.
6. HOTPLUG1000kHz: all343548 app bytes match BIN after reset; full identity sector
   byte-identical before/after, SHA256
   `d4b9cbd0f7646b860bade79a5206911142722efffe52b9f77d9d095917c6ff83`.
7. Read-only UART `w`, `n`: wall acquisition PASS, no wall/no saturation.
   Stored sensor offsets R11/L17/FR6/FL11 and bases L670/R757/F23 match pre-flash
   values. Close UART. No further reset/run command.

Motor/fan/floor/maze motion was neither authorized nor commanded. Flash/reset
was explicitly requested. No identity/calibration/maze/trace-format write, mass
erase, or destructive UART diagnostic. Physical running remains for user comparison.
The August UART lacks newer destructive-command protection; do not run modern
NVM-guard smoke sequences against it.

## Persistent-data observation for follow-up

Before flash, modern firmware reported distance=MISS, sensor=OK, maze=OK(known107),
trace=OK(total3253, schema0x00060000). The distance prefix contains the historical
bring-up test anchors (230/420/680,180/360/540,etc.). Modern NVM loading explicitly
rejects that fixture; August code accepts a valid version/length/checksum and
applies its warp. This is an additional source-level difference outside the
previous motion comparison. Its actual effect on this run is not established;
no calibration data was altered. Current saved data has not been rolled back to
an August snapshot.

Local raw evidence, command/manifest and backups:
`build/r2_august_flash_20260927/`; vendor log
`build/flashing_logs/stlink_20260927T072526Z_mpop5cim.log`.
