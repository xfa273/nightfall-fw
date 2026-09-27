# ST-LINK erase failure investigation, 2026-09-27

**Recovered at 04:00 UTC using STLINK-V3MINIE and its dedicated cable.** The
selected application was erased/programmed/verified successfully once, booted to
OP mode0 idle, and full post-reset readback matched the build. Identity is
unchanged; non-motor IMU/wall checks pass. Two WeAct Mini Debugger units sharing
one adapter/SWD cable had inconsistent ROM reads. The faulty component within
the WeAct path is not isolated; probe firmware/common compatibility also remains
possible. Use the working V3MINIE path until the old adapter/cable can be tested.

## Scope and hardware

User requested live investigation while `tools/flashing/flash_stlink` was failing.
Application flash/reset and read-only diagnostics were used; motors/fan/run were
not authorized or commanded. No mass erase, option-byte/protection change,
identity write or calibration/maze/trace mutation was requested. The recovered
application performs its normal boot-time FRAM loads.

- Probe: ST-LINK V2-1, USB `0483:3752`, SN `066CFF545771485067013914`, FW `V2J43M28`.
- Target: STM32F413/F423, device ID `0x463`, revision A, reported VDD 3.23–3.24 V.
- Identity: `mini_r3_0`, unit 2, UID `001D0038-32345108-36383936`.
- UART: `/dev/cu.usbmodem2124202`, 921600 baud; no boot output during failure.
- Built with `cmake --build --preset Debug-stm32f413`: PASS, RAM 274264 B,
  application Flash 377052 B. Build ID `dfbfc9e`, dirty 1,
  build time `2026-09-27T03:13:00Z`.
- ELF: `build/Debug/nightfall_stm32f413.elf`, SHA256
  `c87754eb5d7b9df65689a6fb912c867e2f900c84a8b2e2c2463b23f46c2433c2`.
  File-backed load segments occupy `0x08000000..0x0805c0db`, sectors 0–6.
  Protected sectors 12–15 are outside this image.

All raw logs/dumps below are local, ignored files under `build/flashing_logs/`.
Source firmware, parameters and the user's existing worktree changes were not
changed for this investigation.

## Observations and attempted recovery

Times below are UTC. CubeProgrammer 2.21 normally uses the existing native arm64
runtime helper on this Mac; the direct x86_64 CLI also launched successfully in
this session and was tested separately.

| Time | Operation | Result / artifact |
| --- | --- | --- |
| 03:11:44 | User's original NORMAL/SWrst write, requested 1000 kHz (actual 950) | Loader initialization succeeds, then `failed to erase memory`, sectors 0–6. `stlink_20260927T031144Z_cb3ls4q7.log`. |
| 03:14:06 | HOTPLUG read of status, UID and full identity sector | Valid identity saved as `identity_before_20260927.bin`; SR read as zero and option bytes show RDP AA. `stlink_20260927T031406Z_kckd5wwz.log`. |
| 03:14–03:15 | Software reset/boot capture, then app-vector read | No UART boot; first 32 bytes at `0x08000000` all FF **before the agent's first write attempt**. `f413_boot_20260927_121445.log`, `stlink_20260927T031503Z_38tf6tkf.log`. |
| 03:15:24 | Built ELF, UR/HWrst, 1000 kHz requested | Erase failure. `stlink_20260927T031524Z_g6vk7_6r.log`. |
| 03:15:58 | Same ELF, UR/HWrst, 400 kHz requested (actual 240), UART closed | Erase failure. `stlink_20260927T031558Z_8g84un7b.log`. |
| 03:16:28 | Same write after one exact-SN libusb reset/re-enumeration of 0483:3752 | Erase failure. `stlink_20260927T031628Z_tqsbaig9.log`. This was a diagnostic invocation, not a persistent change to V3-only auto recovery. |
| 03:17:39 | Same write after user physically replugged USB | Erase failure. `stlink_20260927T031739Z_c11b19yr.log`. |
| 03:18–03:19 | Halt/core registers and repeated reads of RAM loader | Some dumps differ from host loader by shifted/missing 32-bit words; another dump exactly matches its 2232-byte code section. Small direct read can match while an upload differs. This does **not** prove RAM contents are corrupt. |
| 03:21:14 | Direct Intel CLI, same ELF/UR/HWrst/400 kHz | Erase failure; Intel RAM reads also differ. `stlink_20260927T032114Z_ejcn2rpt.log`. Identity read is identical to initial backup. |
| 03:24–03:25 | Installed ST OpenOCD 0.12.0+dev-00623-g0ba753ca7, `stlink-dap.cfg`/`dapdirect_swd`, 400 kHz; `program ... verify reset exit` | Connects/detects target but fails reading `0x40023c10` while erasing sectors 0–6. `openocd_program_20260927.log`. OpenOCD RAM read also differs. No successful programming or verification. |
| 03:26:16 | Cube HOTPLUG/CPU halt, 50 kHz; same 4096-byte RAM region read three times | Reads claim success but dumps differ by 1834 and 966 bytes from first. `ram_50_{a,b,c}.bin`, `stlink_20260927T032616Z__1udxr7m.log`. |
| 03:27:10 | After user cycled machine power: UR/HWrst/halt at requested 400 kHz, three RAM reads plus full identity | RAM differs by 1237 and 1209 bytes from first. Identity remains identical. `ram_powercycle_{a,b,c}.bin`, `identity_powercycle.bin`, `stlink_20260927T032710Z_huf4hrkk.log`. |
| 03:28:58 | After user disconnected both power sources and reseated SWD: same stopped RAM reads | Still differs by 1206 and 1572 bytes. `ram_reseat_{a,b,c}.bin`, `stlink_20260927T032858Z_xel5z9ky.log`. |
| 03:30:32 | User moved ST-LINK directly to Mac USB controller (topology confirmed); same stopped RAM reads | Still differs by 1884 and 442 bytes. `ram_direct_{a,b,c}.bin`, `stlink_20260927T033032Z_h4vbr55k.log`. |

The initial, Intel, post-power-cycle and final full 128 KiB identity reads have SHA256
`ef710246e2824452ccde43cf5318fcd6b6127a345a30be7ccf1ad5dad8eafb79`.
Identical reads support identity preservation; they do not establish general
readback reliability (most of the sector is FF).

The failed loader's saved exception frame has R0=1 and PC=`0x20000000`, its
completion breakpoint. Loss of a debug completion event is one possible
interpretation, but the inconsistent readbacks prevent a firm root-cause claim.
Independent host tools failing and stopped-memory read instability mean the
issue is not isolated to the Python wrapper or native arm64 CLI. SWD wiring,
probe, USB path and target electrical state still need separation. Reported VDD
alone does not measure rail transients.

At the first pause (03:41 UTC), recovery was incomplete and no successful
application write/boot had been observed. Original Mac `ioreg -p IOUSB -w0` topology showed
three hub levels (`USB2.0 Hub` → `USB2.1 Hub` → `USB2.1 Hub`); direct connection
was confirmed at `STM32 STLink@01100000` but did not cure read instability. Writes
are paused. A spare probe is unavailable. Next: compare replacement SWD/USB
cables or another probe, and require stable immutable-ROM reads before another
application write. Then verify the selected image, compare identity and capture
non-motor boot.

## Additional host diagnostics

The user has no spare ST-LINK for comparison. Bundled libusb is 1.0.27 and
Homebrew libusb is 1.0.30. A private copied Cube runtime with the latter library
was rejected at launch by macOS library signature validation; no device operation
occurred and no signature/security settings were changed
(`stlink_20260927T033508Z_28r1hpza.log`).

Installed Homebrew `stlink` 1.8.0 for an independent native/libusb 1.0.30 read test.
No Homebrew firmware write has been performed. The install also ran Homebrew's
normal auto-update/portable-Ruby update.

The bundled official `STLinkUpgrade.jar -displayLastJtagVer` reports J46 for V2;
`-sn 066CFF545771485067013914 -checkVer` failed with JNI error 0x1002. **No
`-update` command was run.** After this version query, the probe appeared as
0483:3748 with the same SN, and both Cube and stlink reported USB communication
errors. One exact-SN software USB reset did not restore target access; a physical
USB replug was requested to restore normal probe operation. Do not attempt a
firmware update while the updater cannot reliably communicate with the probe.

After the user's physical USB replug, Homebrew `st-flash` connected normally to
STM32F413/F423 again. RAM reads at requested 400 kHz (actual 480 for this driver)
still differed by 475 and 1213 bytes (`stflash_reads_replug_20260927.log`).

To distinguish read-path instability from RAM changing, repeated **immutable
system-ROM** reads at `0x1fff0000`, 4096 B, three separate `st-flash --hot-plug`
invocations per frequency. Results against the first read in each group:

| Requested SWD kHz | Second read: differing bytes | Third read: differing bytes |
| --- | --- | --- |
| 4000 | 1518 | 1312 |
| 1800 | 2130 | 1602 |
| 125 | 1742 | 772 |
| 50 | 2155 | 1924 |

Exact command/output: `stflash_rom_frequency_20260927.log`; dumps:
`rom_stflash_<frequency>_{a,b,c}.bin`. All reads reported success. This confirms
that successful tool exit status is not sufficient evidence of reliable readback
in this incident. It does not identify whether probe hardware/firmware, SWD cable,
USB path or target electrical/debug hardware is responsible. New libusb and the
third independent driver did not resolve it. Further erase attempts are stopped.

## State at 03:41 UTC

Final Cube UR/HWrst read (`stlink_20260927T034102Z_v53o3hsb.log`) connects normally
again with original ST-LINK FW V2J43M28 and VDD 3.24 V. The entire identity sector
is byte-identical to the initial backup (`identity_final_20260927.bin`). App vectors
remain all FF; **application recovery and boot are not complete**. The core was
halted for this final read; no run/motor/fan command was sent. No additional erase
is justified while immutable-ROM readback is inconsistent. Asked for the actual
ST-LINK product/model and availability of replacement SWD/USB cables. A different
probe/physical link comparison is still required to isolate the fault.

## Reference commands

Application attempt (build succeeded first):

```sh
python3 tools/flashing/flash_stlink --image build/Debug/nightfall_stm32f413.elf \
  --sn 066CFF545771485067013914 --freq 400 --mode UR --reset-mode HWrst
```

Diagnostic CLI argument sequence, with the same exact probe serial:

```text
-c port=SWD freq=400 mode=UR reset=HWrst sn=066CFF545771485067013914
-halt
-u 0x20000000 4096 <ram-a.bin>
-u 0x20000000 4096 <ram-b.bin>
-u 0x20000000 4096 <ram-c.bin>
-u 0x08160000 131072 <identity.bin>
```

OpenOCD uses the installed CubeIDE executable and matching ST scripts. Exact
paths and command are preserved in `openocd_program_20260927.log`. No OpenOCD
fallback is added to `flash_stlink`: it has not recovered this fault.

The 1 MHz default and diagnostic logging change in draft PR
[46](https://github.com/xfa273/nightfall-fw/pull/46) remains useful for capturing
failures, but did not cure this live incident. Do not present it as a verified fix.

## Resumed at 03:55 UTC: replacement WeAct probe

User identified both V2-1 probes as WeAct Mini Debuggers. The earlier Mac-direct
trial used a C-to-C cable, which the user reports has historically failed for this
debugger; therefore it was not a controlled hub-only comparison. User subsequently
changed to a different hub, different A-to-C cable and another WeAct unit, keeping
the **same WeAct adapter PCB and target-side SWD cable**.

- New probe SN `066BFF545771485067014053`, same FW `V2J43M28`, USB0483:3752.
- New topology: one USB2.1 hub under a separate Mac controller; saved in
  `new_probe_usb_topology.log`.
- UR/HWrst at requested1000 kHz (actual950), CPU halt, target UID unchanged,
  three4096-byte system-ROM reads differ by1177 and775 bytes from the first.
  `stlink_20260927T035513Z_ux339xq1.log`, `rom_new_probe_{a,b,c}.bin`.
- `identity_new_probe_before.bin` again matches the full initial identity backup.
- Smaller transfers at requested400 kHz can agree: three reads each of64/256/512 B
  from0x1fff0400 agree, while one1024 B read differs by59 B and4096 B reads differ
  by1451/78 B. `rom_bulk_size_20260927.log`. This is not proof of a stable write
  path or permission to retry erases. A separate512 B r8/r32 comparison also
  agrees (`read_width_compare_20260927.log`).
- No erase/write or UART/motor/fan operation in this resumed phase. Existing built
  ELF remains byte-identical to the artifact recorded above.

User has no replacement SWD cable for the WeAct setup, but has an STLINK-V3MINIE
and its dedicated cable. The user switched to that complete debug path for the successful recovery below.
This comparison changes the probe family **and** adapter/cable path, so success
alone does not isolate one failed component of the old setup.

## Recovery at 04:00 UTC: STLINK-V3MINIE

- Probe SN `003B00273234511537333934`, FW `V3J16M8`, target VDD3.24 V,
  same target UID and identity. UART now `/dev/cu.usbmodem11102` at921600 baud.
- 03:59:15: UR/HWrst, requested/actual1000 kHz; three4096-byte system-ROM reads
  agree exactly. `stlink_20260927T035915Z_v8n2i8uy.log`, `rom_v3_{a,b,c}.bin`.
  All have SHA256 `7067109ca608556be1e742b9a1bd8e1401815b0c01ce4c00dfc2239afa8c2065`.
  Full identity matches the initial backup (`identity_v3_before.bin`).
- 04:00:22: one application programming attempt with the same previously built
  and SHA256-checked ELF succeeds: erase only sectors0–6, download, verify, reset.
  `stlink_20260927T040022Z_n5ydszlb.log`, `v3_recovery_flash_console.log`.
  No repeat programming was needed on the V3MINIE path.
- UART boot captured across the write/reset in `v3_recovery_boot.log`:
  `GIT=dfbfc9e DIRTY=1`, machine `mini_r3_0_unit002`, tune
  `mini-r3-wall-case3-t0.28`, `[NVM-GUARD] LOCKED`, wall ADC-DMA started,
  control initialized, `[OP-UI] ready mode=0 idle`.
- 04:00:59: HOTPLUG readback of the entire377052-byte app after reset exactly
  matches `arm-none-eabi-objcopy -O binary` of the selected ELF. Both binaries
  have SHA256 `ef21da10e0e9bd0ddc695c5ce67ac3d3abfb01bc03cd491ee4b7a6f22234cbde`.
  Full128 KiB identity still matches the initial backup. Three more system-ROM
  reads agree with all three before programming (six identical reads total).
  `stlink_20260927T040059Z_89rzhgzu.log`, `app_v3_after.bin`,
  `app_recovery_expected.bin`, `identity_v3_after.bin`, `rom_v3_after_{a,b,c}.bin`.
- Final UART `i,w` only: WHO_AM_I0x6B and IMU control-register checks PASS,
  wall measurement PASS with ready0x03. `v3_recovery_status.log`.
  Application is left running; all capture/programmer processes are closed.
  No motors/fan/run were authorized or commanded. No calibration/NVM test writes,
  trace formatting, option-byte updates or protected-sector erase/write occurred.

Successful recovery command (the ELF above was already successfully built):

```sh
python3 tools/flashing/flash_stlink --image build/Debug/nightfall_stm32f413.elf \
  --sn 003B00273234511537333934 --freq 1000 --mode UR --reset-mode HWrst
```

For subsequent source changes, use `--build` instead of `--image ...` with the
same V3MINIE connection/options. This outcome verifies one complete flash/boot
and repeated readback; it does not measure the future failure rate, nor prove
that the WeAct adapter alone is faulty. No WeAct or V3MINIE firmware was updated.
