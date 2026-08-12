# Integration-safe 7-item drill-in-pad cleanup subset

This subset intentionally excludes the supply/CS-sensitive C_MCU1.2,
C_MCU2.2, C_MCU-M0.2, U4.8, and R33.2 items.

Cleaned items: UBUZ0.2, Q3.2, C7.1, R1FR0.1, R2R0.2, C8.2, Q3.3.

Verified on the fan handoff:

- exact KiCad physical DRC errors: 0
- drill-in-pad overlaps: 5, exactly the five intentionally deferred items
- KiCad unconnected count: 32 (unchanged from input)
- GND groups: 1; GND2 groups: 1
- zone self/cross-edge incidents: 0

Replay with `apply_independent_cleanup.py` using canonical-named project
copies, then run exact refill/DRC/audit again on the integrated board.
