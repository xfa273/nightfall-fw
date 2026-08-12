# FAN_PWM route handoff

This directory contains the scoped `/FAN_PWM` completion applied to
`/private/tmp/mini3-irfl-route-handoff/HM_Nightfall-mini-3a_v0.kicad_pcb`.
It connects `U5.45`, `R32.1`, and `Q3.1`, keeps the gate pulldown at R32,
and preserves the existing schematic/net names. No series gate resistor or
schematic change was required.

## Result

- `/FAN_PWM` unconnected items: **0**
- Exact-project physical DRC errors: **0**
- GND / GND2 connected groups: **1 / 1**
- `LOGIC_GND_F`, `LOGIC_GND_B`, `POWER_GND2_F`, and `POWER_GND2_B`
  zone self-edges: **0 / 0 / 0 / 0**
- Zone cross-incidents on the same four zones: **0 / 0 / 0 / 0**
- R0 single-point contract: **PASS**
- New via-in-pad overlaps: **0**; the global count remains the source baseline
  of 12.
- Changed copper nets: exactly `/FAN_PWM` and `GND`.
- Moved footprints: exactly `R32`, rotated in place from 0 degrees to -90
  degrees.
- Every other copper net is geometry-signature identical to the source. This
  includes `/FAN_NEG_INTERNAL`, VBAT, motor, current-sense, IR, sensor, SPI,
  and GND2 copper.

The global audit release flag remains false only because the integration base
still has 32 unrelated unfinished items and 12 pre-existing via-in-pad
overlaps. The source had 34 unfinished items; completing the three-terminal
FAN net removes its two missing edges.

## Routing notes

- The route crosses the logic/power split on B.Cu beside R0. Its nearest
  crossing vertex is `(145.85, 121.90)` mm, 0.206 mm from the R0.2 center.
- The 0.8 mm `/FAN_NEG_INTERNAL` and VBAT high-current geometry is unchanged.
- 23 FAN segments use 0.20 mm. The three-segment, 1.54 mm dense escape between
  the `MOTOR_R_PWM` via and the FAN_NEG bus uses 0.16 mm; 0.20 mm cannot retain
  the exact 0.16 mm clearance there.
- FAN layer changes use three 0.40/0.20 mm U5 escape vias and one normal
  0.60/0.30 mm gate-side via.
- GND is restored to one group with two 0.60/0.30 mm stitches and one
  0.50/0.30 mm stitch. The latter, at `(156.40, 113.65)` mm, is the largest
  exact-DRC-clean bridge in the +3V3//FRAM_CS slot; a 0.60/0.30 mm via misses
  clearance by 0.028-0.033 mm.

## Files

- `HM_Nightfall-mini-3a_v0.kicad_pcb`: exact-project-refilled deliverable.
- `HM_Nightfall-mini-3a_v0.pre-refill.kicad_pcb`: direct replay geometry
  before zone refill.
- `HM_Nightfall-mini-3a_v0.kicad_pro`, `.kicad_dru`, and `.kicad_sch`: exact
  project/rule/schematic companions used by the audit.
- `apply_fan_pwm.py`: fail-closed, self-contained replay against the included
  source base.
- `verify_replay.py`: geometry-signature and scope verifier.
- `replay-report.json` / `signature-audit.json`: replay equality and scope
  proof.
- `audit.json`, `audit.md`, and `drc.json`: exact-project audit artifacts.
- `audit/replay_exact_summary.json`: independent replay-audit summary.
- `manifest.json`: checksums, route geometry, exception rationale, and
  acceptance metrics.
- `reference/source_base.kicad_pcb`: exact source copy expected by the replay.

## Replay

Use KiCad's bundled Python because the script imports `pcbnew`. Generate into a
scratch directory so KiCad's Python save helper cannot touch the supplied
project file:

```sh
HANDOFF=/private/tmp/mini3-fan-pwm-handoff
KI_PY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3.9
REPLAY_DIR=$(mktemp -d /private/tmp/mini3-fan-replay.XXXXXX)

"$KI_PY" "$HANDOFF/apply_fan_pwm.py" \
  --output "$REPLAY_DIR/HM_Nightfall-mini-3a_v0.kicad_pcb"

"$KI_PY" "$HANDOFF/verify_replay.py" \
  "$REPLAY_DIR/HM_Nightfall-mini-3a_v0.kicad_pcb"
```

The checked replay report proves exact track/via and footprint-placement
signature equality with the supplied pre-refill reference. For the repository
exact audit:

```sh
hardware/mini_r3_0/tools/audit_final_board.sh \
  --candidate "$REPLAY_DIR/HM_Nightfall-mini-3a_v0.kicad_pcb" \
  --output-root "$REPLAY_DIR/audit"
```

For direct `kicad-cli pcb drc`, copy the supplied `.kicad_pro`, `.kicad_dru`,
and `.kicad_sch` into `REPLAY_DIR` after running the replay, then use
`--severity-all --all-track-errors --refill-zones`.
