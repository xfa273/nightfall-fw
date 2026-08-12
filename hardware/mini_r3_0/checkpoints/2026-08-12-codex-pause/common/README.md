# control4 + LED3 rear-I/O handoff

This exact project is based on the reviewed `control4-led2` board and adds a
clean `/LED_3` route plus conflict-free `/NRST` and `/SENSOR_FR` reroutes.
The six control nets and the split inner-plane geometry are preserved.

Primary board:

`HM_Nightfall-mini-3a_v0.control4-led3.kicad_pcb`

Replay with KiCad's bundled Python:

```sh
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
"$KPY" replay.py \
  /private/tmp/mini3-control4-led2-handoff-20260812/HM_Nightfall-mini-3a_v0.control4-led2.kicad_pcb \
  replayed.kicad_pcb delta.json
```

The delta uses integer nanometer coordinates.  A replay-to-candidate geometry
comparison reports zero removed and zero added copper objects.

Verification:

- exact physical DRC errors: 0
- unconnected items: 20
- GND groups: 1; GND2 groups: 1
- named zone self-edges: 0
- In1 signal tracks: 0
- via-in-pad: 22, unchanged from the `control4-led2` source
- R0 ground bridge contract: pass

Completed rear nets are `/SWCLK`, `/EC_L_A`, `/EC_L_B`, `/NRST`,
`/USART1_RX`, `/LED_1`, `/LED_2`, and `/LED_3`. `/SENSOR_FR` remains fully
connected. Remaining non-supply nets are `/BOOT0`, `/BUZZER_PWM`, `/EC_R_B`,
`/IMU_CS`, and `/PUSH_IN_1`.

`/PUSH_IN_1` was not included because U5.5, MODE0.1, and R3.1 lie in separate
local routing pockets on the integrated board. A small local reroute or the
permitted <=1 mm R3 move is needed; no speculative copper remains in this
handoff.
