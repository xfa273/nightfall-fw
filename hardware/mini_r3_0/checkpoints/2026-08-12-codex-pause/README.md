# mini_r3_0 PCB pause checkpoint — 2026-08-12

This directory is a durable handoff of the 4-layer routing work at the point
where Codex usage was paused.  It intentionally contains both the clean common
integration base and several independently DRC-clean candidate deltas.  The
independent candidates have **not** all been merged together and therefore are
not manufacturing outputs.

## Resume from here

Use this board as the only integration base:

`common/HM_Nightfall-mini-3a_v0.control4-led3.kicad_pcb`

- SHA-256: `a1a231fa70815dee5e19d0daeda741c14aa30f56400d8977a5e4750b6d5aa1ac`
- Four copper layers: F.Cu / In1.Cu / In2.Cu / B.Cu.
- Exact KiCad refill DRC has no clearance, short, hole, crossing, edge, width,
  or connection-width error.
- GND and GND2 each form one connected group, all four outer ground-zone
  self-edge counts are zero, and R0 is the sole GND/GND2 bridge.
- In1 is plane-only; there are no signal tracks on In1.
- Unconnected count is 20:
  `+3V3` 11, `/VOL_CHECK` 3, and one each for `/BOOT0`, `/BUZZER_PWM`,
  `/EC_R_B`, `/IMU_CS`, plus `/PUSH_IN_1` 2.
- Already complete here: power/current core, fan supply/return and FAN_PWM,
  motor outputs, all four motor DIR/PWM controls plus STBY, sensors including
  SENSOR_FR, SPI2 SCK/MISO/MOSI, SWCLK, EC_L_A/B, NRST, USART1_RX, and
  LED1/2/3.

See `common/manifest.json` and `common/audit/audit.md` for the machine-readable
details and replay contract.

## Independently clean candidates

These are useful deltas but must be replayed one at a time onto the latest
integration board and exact-refill DRC checked after every merge.

### PUSH

`push/push-complete-d.kicad_pcb`

- Derived from the common base.
- `/PUSH_IN_1` is fully connected with R3 and MODE0 unmoved.
- Exact twice-refilled DRC: physical errors 0, unconnected 18.
- It competes with the current BOOT corridor.  Do not merge the standalone
  BOOT board without rerouting one of the two paths.

### BOOT

`boot/out.kicad_pcb`

- Derived from the common base, without PUSH.
- `/BOOT0` is fully connected using 0.20 mm tracks and three 0.60/0.30 mm
  normal vias.
- Exact refill DRC: physical errors 0, unconnected 19.
- The central In2 corridor conflicts with the PUSH candidate above.

### BUZZER with local MOSI relocation

`buzzer/HM_Nightfall-mini-3a_v0.kicad_pcb`

- Derived from the common base.
- Moves only the local `/SPI2_MOSI` B.Cu spine to In2 while keeping MOSI fully
  connected; the rear MOSI segment remains on B.Cu.
- `/BUZZER_PWM` is fully connected.  The QFN toe uses one 0.40/0.20 mm via;
  the route added by the router uses two normal 0.60/0.30 mm vias.  The
  redundant dangling toe via was removed.
- Exact refill DRC: physical errors 0, unconnected 19.
- Reconstruction helpers are beside the board.  Re-run strict audit after
  integrating PUSH/BOOT because those routes share the central corridor.

### Logic supply and voltage monitor

`supply/3v3-all-clean.kicad_pcb` and
`supply/control4-supply-mlpwm-Broute.kicad_pcb`

- Both have `+3V3` and `/VOL_CHECK` fully connected and physical DRC 0 on
  their respective earlier bases.
- The control4 version also preserves IR_FL and MOTOR_L_PWM connectivity.
- Neither is yet replay-clean on the latest common LED3/NRST/SENSOR_FR base.
  Treat these as routing geometry references, not direct merge sources.
- Supply integration was actively being adapted when work stopped.  R3 must
  remain in place because R3.1 is shared with PUSH.

### EC_R_B

`ec/ec_common_short.kicad_pcb`

- Derived from the current common base.
- `/EC_R_B` is fully connected, physical DRC 0, unconnected 19.
- This is a shortened common-base candidate; inspect
  `ec/ec_short_geometry_spec.json` before replay.
- `/IMU_CS` was not completed.  The last IMU experiment had 4 clearance,
  2 crossing, and 3 short errors and is deliberately not preserved as an
  adoptable board.

### Via-in-pad cleanup

`via_cleanup/`

- Reusable cleanup scripts plus the verified `subset7` and `full12` packages.
- They were validated on an earlier fan-integrated base, not the final merged
  board.  Apply only after all routing is integrated, then run exact DRC and
  the strict final-board audit.

## Final work still required

1. Merge PUSH, supply, EC_R_B, BUZZER and BOOT while resolving the shared
   central In2 corridor.
2. Complete `/IMU_CS` without moving C_MCU1 more than 1 mm and without
   degrading MCU decoupling.  The U5 no-exposed-pad footprint permits an
   inward QFN toe escape.
3. Reach unconnected count 0.
4. Apply the via-in-pad cleanup and reach actual drill-in-pad count 0.
5. Run `finalize_four_layer.py` to add the final split In2 GND/GND2 pours.
6. Run `tools/audit_final_board.sh --strict`; required result is physical DRC
   0, GND/GND2 one group each, zone self-edge 0, R0-only bridge, In1
   plane-only, In2 policy pass, no via-in-pad, and no unconnected items.
7. Copy the audited board into the canonical KiCad project, rerun ERC/DRC,
   verify schematic/PCB net parity and U5 no-EP footprint, then generate the
   4-layer Gerber/drill/job archive and fabrication notes.

## Electrical/firmware state already completed

- The canonical schematic reached KiCad ERC 0.  No physical pin-to-net
  assignment was lost during the ERC cleanup.
- The F413 battery safety implementation is already committed and pushed as
  commit `5c77482`: 2S/3S voltage classification, low-voltage inhibit, stale
  ADC protection, host tests (63 checks), and F405/F413 build verification.
- No HIL, motor, fan, Flash/NVM-destructive, or motion operation was performed
  during this PCB session.

## Verification and integrity

`SHA256SUMS` contains a digest for every preserved file.  Validate it from
this directory with:

```sh
shasum -a 256 -c SHA256SUMS
```

The legacy footprint/library and text warnings recorded in the candidate DRC
JSON files are not physical copper errors, but they still need normalization
before manufacturing release.
