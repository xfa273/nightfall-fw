# Full 12-item drill-in-pad cleanup (fan handoff reference)

Reference input only:

`/private/tmp/mini3-fan-pwm-handoff/HM_Nightfall-mini-3a_v0.kicad_pcb`

Replay:

```sh
/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3.9 \
  apply_via_cleanup.py INPUT/HM_Nightfall-mini-3a_v0.kicad_pcb \
  OUTPUT/HM_Nightfall-mini-3a_v0.kicad_pcb
```

The output directory must contain the matching `.kicad_pro`, `.kicad_dru`,
and library sidecars before exact DRC/audit.

Cleaned items: UBUZ0.2, C_MCU1.2, C_MCU2.2, C_MCU-M0.2, U4.8,
Q3.2, C7.1, R1FR0.1, R2R0.2, C8.2, Q3.3, and R33.2.

Verified on the fan handoff:

- exact KiCad physical DRC errors: 0
- drill-in-pad overlaps: 0
- KiCad unconnected count: 32 (unchanged from input)
- GND groups: 1; GND2 groups: 1
- zone self/cross-edge incidents: 0

The release gate remains false solely because the reference input contains 32
unrelated unfinished connections.  `audit.json`, `audit.md`,
`drc-exact.json`, and `audit-summary.json` contain the evidence.

Integration note: C_MCU1/C_MCU2/C_MCU-M0 and U4 are supply-routing owned;
R33 is close to the CS/supply placement area. Re-run this replay against the
integrated base and resolve topology guards locally rather than copying the
reference board.
