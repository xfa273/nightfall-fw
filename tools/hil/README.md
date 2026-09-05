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

The current F413 wiring default is mini r3 with the **left motor leads swapped**
(2026-09-06); both sides use IN2 high for forward. Original mini r2 or unswapped
mini r3 wiring requires `NIGHTFALL_F413_MOTOR_LEFT_FORWARD_IN2_HIGH=0` as a compiler
definition for all F413 application sources. This is not automatic NVM machine
selection. The regression tests both mappings, zero duty, and saturation across
all 16-bit duty inputs. Encoder signs and PWM frequency are unchanged.

This safe helper intentionally does not automate motor commands. When the
machine is lifted and secured, use the UART commands from `docs/ai/HIL_SAFETY.md`
directly and record the command sequence plus result in `docs/ai/WORKLOG.md`.
