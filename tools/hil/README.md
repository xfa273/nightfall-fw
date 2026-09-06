# tools/hil

Safe hardware-in-the-loop helpers for Codex-driven F413 work.

These scripts intentionally avoid motor/fan/run commands. For motion checks, follow `docs/ai/HIL_SAFETY.md` and run the underlying UART commands only after the machine is lifted and secured.

## F413 safe helper

List connected tools:

```sh
python3 tools/hil/f413_safe_hil.py list
```

Capture a boot log around ST-LINK software reset:

```sh
python3 tools/hil/f413_safe_hil.py reset-capture --port /dev/cu.usbmodem112202
```

`reset-capture` uses the non-interactive CSV capture backend and writes the
combined UART output to `tools/logging/logs/f413_boot_*.log`. The capture
subprocess runs with stdin detached so terminal input cannot be forwarded to
the robot as UART commands.

Run a non-motor smoke sequence:

```sh
python3 tools/hil/f413_safe_hil.py nonmotor-smoke --port /dev/cu.usbmodem112202
```

Dump the latest bounded trace CSV:

```sh
python3 tools/hil/f413_safe_hil.py dump-trace --port /dev/cu.usbmodem112202
```

Build, flash through ST-LINK, then run non-motor smoke:

```sh
python3 tools/hil/f413_safe_hil.py flash-nonmotor-smoke --port /dev/cu.usbmodem112202 --sn 003B00273234511537333934
```

The default UART baud is `921600`, matching `Debug-stm32f413`.

The safe helper uses lowercase `v` for bounded trace dumps. Use uppercase `V`
only when an explicit task needs a full FRAM trace dump and the capture window
is sized to wait for the firmware dump-completion marker.

## Motor checks

Host-only PWM mapping regression (does not access hardware):

```sh
sh tools/hil/run_f413_motor_pwm_tests.sh
```

F413 now selects wiring from NVM model/unit identity. mini r2 unit001 uses its
original wiring; mini r3 unit001 has the left leads swapped. The old compiler
polarity override is rejected. This regression tests both electrical mappings,
zero duty, and saturation over all 16-bit inputs.

`sh tools/hil/run_f413_machine_tests.sh` checks runtime selection, independent
unit profiles, mini/classic namespace separation, invalid-ID fail-closed behavior,
and immutable boot settings. See `docs/F413_MACHINE_CONFIG.md` for operation.

It also checks the r3 -Y IMU mounting, the rearward-offset acceleration correction,
and each board's battery divider without changing r2 geometry or polarity.

Read-only calibration-loader regression with ASan/UBSan (no hardware access):

```sh
sh tools/hil/run_f413_nvm_params_tests.sh
```

This verifies rejection of the exact historical dummy distance-calibration blob,
preservation of real calibration and stored bytes, and CRC/schema failure paths.
On hardware, `|` dumps both calibration prefixes without writing them; `{` is a
separate lifted-only 7..9V bounded motor sweep, not part of the safe helper.
See `docs/MINI_R3_COMMISSIONING.md` for limits and r3 commissioning results.

Destructive-diagnostic guard regression (ASan/UBSan, host memory only):

```sh
sh tools/hil/run_f413_nvm_guard_tests.sh
```

Normal F413 builds refuse `a/d/s/m/t/q/Q/r/k` at the diagnostic entry points,
including non-UART callers. They cannot replace calibration/maze data with test
fixtures or format/append synthetic trace records. Read-only dumps, intentional
OP calibration, normal maze saves and run logging are unchanged. The test checks
all protected entry points and every mock NVM byte, and verifies ordinary sensor
calibration still saves/loads with the guard locked.

The CMake option `NIGHTFALL_F413_DESTRUCTIVE_NVM_DIAGNOSTICS` defaults to `OFF`.
Only an explicitly authorized **separate maintenance build**, after backing up
all affected NVM, may enable it. There is no one-character UART unlock. Never
leave that build on a calibrated robot; rebuild/reflash with the option `OFF`
and verify the `[NVM-GUARD] LOCKED` boot message. Host tests exercise the opt-in
path in memory; they do not enable it in the firmware build cache.

This safe helper intentionally does not automate motor commands. When the
machine is lifted and secured, use the UART commands from `docs/ai/HIL_SAFETY.md`
directly and record the command sequence plus result in `docs/ai/WORKLOG.md`.
