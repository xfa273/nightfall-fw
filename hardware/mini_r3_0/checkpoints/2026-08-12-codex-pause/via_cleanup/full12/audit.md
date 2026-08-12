# mini_r3 final PCB audit

- Candidate: `/private/tmp/mini3-via-cleanup.WaS5Gg/full12/HM_Nightfall-mini-3a_v0.kicad_pcb`
- Candidate SHA-256: `f987c929750164a1e7e76291d9aae1d1efa411d33208b54a1874c2d484a07b89`
- KiCad: `10.0.3`
- Temporary audit directory: `/private/tmp/mini3-via-cleanup.WaS5Gg/full12/audit-2/mini3-final-audit.vw1j_55m`
- Overall release gate: **FAIL**

## Release checks

| Check | Result |
|---|---:|
| `candidate_and_canonical_sources_unchanged` | PASS |
| `exact_project_rules` | PASS |
| `physical_drc_errors_zero` | PASS |
| `unconnected_items_zero` | FAIL |
| `gnd_and_gnd2_single_group_no_zone_self_edges` | PASS |
| `r0_single_point_contract` | PASS |
| `via_in_pad_zero` | PASS |
| `two_copper_layers_only` | PASS |
| `u2_left_u3_right_parity` | PASS |
| `u5_no_ep49_no_central_paste` | PASS |
| `jlc_2oz_track_width` | PASS |
| `jlc_2oz_spacing` | PASS |
| `jlc_routed_edge_clearance` | PASS |
| `jlc_via_hard_capability` | PASS |
| `project_via_0p40_0p20_floor` | PASS |

## KiCad DRC after exact-project zone refill

- Physical violations: 105 (errors: 0)
- Unconnected items: 32
- Schematic-parity warnings: 55 (U5 pad49 expected exception: 1)
- Violation types: `{"lib_footprint_issues": 83, "lib_footprint_mismatch": 8, "text_height": 1, "text_thickness": 11, "track_dangling": 1, "via_dangling": 1}`

## Unconnected nets

| Net | DRC missing edges |
|---|---:|
| `+3V3` | 11 |
| `+5V` | 1 |
| `/BOOT0` | 1 |
| `/BUZZER_PWM` | 1 |
| `/EC_L_A` | 1 |
| `/EC_L_B` | 1 |
| `/EC_R_B` | 1 |
| `/IMU_CS` | 1 |
| `/LED_1` | 1 |
| `/LED_2` | 1 |
| `/LED_3` | 1 |
| `/NRST` | 4 |
| `/PUSH_IN_1` | 2 |
| `/SWCLK` | 1 |
| `/USART1_RX` | 1 |
| `/VOL_CHECK` | 3 |

## Split ground

- `GND`: 49 pads, 1 connected group(s)
- `GND2`: 23 pads, 1 connected group(s)
- Zone self-edge unconnected: `{"LOGIC_GND_B": 0, "LOGIC_GND_F": 0, "POWER_GND2_B": 0, "POWER_GND2_F": 0}`
- All zone-related unconnected incidents: `{"LOGIC_GND_B": 0, "LOGIC_GND_F": 0, "POWER_GND2_B": 0, "POWER_GND2_F": 0}`
- R0 contract: **PASS**
- GND/GND2 bridge footprints: `["R0"]`

## Vias

- Total vias: 155
- Drill-in-pad overlaps: 0
- 0.40/0.20 mm toe vias: 32
- Small-via order-option items: 32

## Track widths by net

- Global min/max: 0.16 / 1.2 mm
- Below JLC 2 oz hard minimum (0.16 mm): 0
- 0.16–<0.20 mm local escapes: 43

| Net | Segments | Length (mm) | Min (mm) | Max (mm) | Width histogram |
|---|---:|---:|---:|---:|---|
| `+3V3` | 182 | 194.945 | 0.160 | 0.400 | `{"0.1600": 1, "0.2000": 173, "0.4000": 8}` |
| `+5V` | 14 | 29.253 | 0.500 | 0.600 | `{"0.5000": 13, "0.6000": 1}` |
| `/BOOT0` | 7 | 8.632 | 0.200 | 0.200 | `{"0.2000": 7}` |
| `/BUZZER_PWM` | 4 | 1.440 | 0.200 | 0.200 | `{"0.2000": 4}` |
| `/EC_R_A` | 11 | 21.177 | 0.200 | 0.200 | `{"0.2000": 11}` |
| `/FAN_NEG_INTERNAL` | 22 | 73.033 | 0.300 | 0.800 | `{"0.3000": 3, "0.8000": 19}` |
| `/FAN_PWM` | 26 | 31.139 | 0.160 | 0.200 | `{"0.1600": 3, "0.2000": 23}` |
| `/FRAM_CS` | 15 | 24.795 | 0.200 | 0.200 | `{"0.2000": 15}` |
| `/IMU_CS` | 6 | 12.630 | 0.200 | 0.200 | `{"0.2000": 6}` |
| `/IR_FL` | 44 | 47.869 | 0.200 | 0.200 | `{"0.2000": 44}` |
| `/IR_FR` | 13 | 34.500 | 0.160 | 0.200 | `{"0.1600": 1, "0.2000": 12}` |
| `/IR_L` | 19 | 48.611 | 0.200 | 0.200 | `{"0.2000": 19}` |
| `/IR_R` | 16 | 52.723 | 0.200 | 0.200 | `{"0.2000": 16}` |
| `/MOTOR_L_DIR` | 8 | 5.846 | 0.160 | 0.200 | `{"0.1600": 2, "0.2000": 6}` |
| `/MOTOR_L_OUT1` | 3 | 2.983 | 0.300 | 0.800 | `{"0.3000": 2, "0.8000": 1}` |
| `/MOTOR_L_OUT2` | 3 | 2.935 | 0.300 | 0.800 | `{"0.3000": 2, "0.8000": 1}` |
| `/MOTOR_L_PWM` | 14 | 6.421 | 0.160 | 0.200 | `{"0.1600": 4, "0.2000": 10}` |
| `/MOTOR_L_SR` | 4 | 2.995 | 0.200 | 0.200 | `{"0.2000": 4}` |
| `/MOTOR_R_DIR` | 22 | 11.236 | 0.160 | 0.200 | `{"0.1600": 2, "0.2000": 20}` |
| `/MOTOR_R_OUT1` | 3 | 3.059 | 0.300 | 0.800 | `{"0.3000": 2, "0.8000": 1}` |
| `/MOTOR_R_OUT2` | 3 | 3.000 | 0.300 | 0.800 | `{"0.3000": 2, "0.8000": 1}` |
| `/MOTOR_R_PWM` | 12 | 8.927 | 0.160 | 0.200 | `{"0.1600": 2, "0.2000": 10}` |
| `/MOTOR_R_SR` | 2 | 1.274 | 0.200 | 0.200 | `{"0.2000": 2}` |
| `/MOTOR_STBY` | 34 | 24.374 | 0.160 | 0.200 | `{"0.1600": 5, "0.2000": 29}` |
| `/MPM_EN_INTERNAL` | 3 | 2.690 | 0.200 | 0.200 | `{"0.2000": 3}` |
| `/MPM_FB_INTERNAL` | 3 | 2.813 | 0.200 | 0.200 | `{"0.2000": 3}` |
| `/PUSH_IN_1` | 1 | 4.100 | 0.200 | 0.200 | `{"0.2000": 1}` |
| `/PWR_GATE_INTERNAL` | 7 | 10.966 | 0.200 | 0.250 | `{"0.2000": 6, "0.2500": 1}` |
| `/PWR_SWITCH_RETURN` | 12 | 43.679 | 0.200 | 0.200 | `{"0.2000": 12}` |
| `/SENSOR_FL` | 14 | 41.276 | 0.200 | 0.200 | `{"0.2000": 14}` |
| `/SENSOR_FR` | 22 | 49.297 | 0.200 | 0.200 | `{"0.2000": 22}` |
| `/SENSOR_L` | 58 | 70.620 | 0.200 | 0.200 | `{"0.2000": 58}` |
| `/SENSOR_R` | 40 | 63.022 | 0.200 | 0.200 | `{"0.2000": 40}` |
| `/SPI2_MISO` | 17 | 29.110 | 0.160 | 0.200 | `{"0.1600": 2, "0.2000": 15}` |
| `/SPI2_MOSI` | 19 | 20.971 | 0.160 | 0.200 | `{"0.1600": 1, "0.2000": 18}` |
| `/SPI2_SCK` | 57 | 55.947 | 0.160 | 0.200 | `{"0.1600": 2, "0.2000": 55}` |
| `/SWDIO` | 7 | 21.653 | 0.200 | 0.200 | `{"0.2000": 7}` |
| `/SWO` | 7 | 15.194 | 0.200 | 0.200 | `{"0.2000": 7}` |
| `/USART1_TX` | 5 | 15.605 | 0.200 | 0.200 | `{"0.2000": 5}` |
| `GND` | 74 | 67.880 | 0.200 | 0.300 | `{"0.2000": 73, "0.3000": 1}` |
| `GND2` | 10 | 13.939 | 0.300 | 0.800 | `{"0.3000": 7, "0.8000": 3}` |
| `Net-(D3-A)` | 7 | 10.649 | 0.200 | 0.300 | `{"0.2000": 4, "0.3000": 3}` |
| `Net-(IR_LED_FL0-C)` | 11 | 14.915 | 0.200 | 0.200 | `{"0.2000": 11}` |
| `Net-(IR_LED_FL0-PadA)` | 17 | 15.896 | 0.160 | 0.160 | `{"0.1600": 17}` |
| `Net-(IR_LED_FR0-C)` | 10 | 15.907 | 0.200 | 0.200 | `{"0.2000": 10}` |
| `Net-(IR_LED_FR0-PadA)` | 9 | 15.045 | 0.200 | 0.200 | `{"0.2000": 9}` |
| `Net-(IR_LED_L0-C)` | 7 | 22.692 | 0.300 | 0.300 | `{"0.3000": 7}` |
| `Net-(IR_LED_L0-PadA)` | 3 | 3.896 | 0.200 | 0.200 | `{"0.2000": 3}` |
| `Net-(IR_LED_R0-C)` | 10 | 21.064 | 0.300 | 0.300 | `{"0.3000": 10}` |
| `Net-(IR_LED_R0-PadA)` | 3 | 4.283 | 0.200 | 0.200 | `{"0.2000": 3}` |
| `Net-(LED1-PadA)` | 3 | 1.904 | 0.200 | 0.200 | `{"0.2000": 3}` |
| `Net-(LED2-PadA)` | 1 | 1.625 | 0.200 | 0.200 | `{"0.2000": 1}` |
| `Net-(LED3-PadA)` | 1 | 1.625 | 0.200 | 0.200 | `{"0.2000": 1}` |
| `Net-(P-LED0-PadA)` | 2 | 0.793 | 0.200 | 0.200 | `{"0.2000": 2}` |
| `Net-(U5B-VCAP_1)` | 1 | 1.055 | 0.160 | 0.160 | `{"0.1600": 1}` |
| `Net-(U6-~{WP})` | 6 | 8.225 | 0.200 | 0.200 | `{"0.2000": 6}` |
| `VBAT_RAW` | 9 | 13.924 | 0.300 | 1.200 | `{"0.3000": 7, "1.2000": 2}` |
| `VBAT_SW` | 62 | 126.731 | 0.300 | 1.000 | `{"0.3000": 10, "0.4000": 2, "0.6000": 14, "0.8000": 28, "1.0000": 8}` |

## Copper-to-edge

- Exact KiCad DRC threshold: 0.2 mm
- Violations: 0
- At/above JLC routed-edge capability: **PASS**

## Logical driver and MCU footprint audits

- U2 logical LEFT / U3 logical RIGHT: **PASS**
- U5 no exposed pad 49 / no central paste windows: **PASS**

## JLCPCB 2-layer / 2 oz fabrication guardrails

- Configured copper layers: 2
- Inner-copper items: 0
- 2 oz copper weight is an **order setting**, not encoded in this board file.
- Select **2 layers, 2 oz copper**, and the small-via option when 0.40/0.20 mm vias remain.
- Official capability references:
  - https://jlcpcb.com/help/article/jlcpcb-copper-weight
  - https://jlcpcb.com/capabilities/pcb-capabilities/

## Artifacts

- `audit.json`: complete machine-readable audit
- `drc.json`: KiCad's exact refilled DRC
- `schematic-netlist.xml`: exported schematic truth
- `HM_Nightfall-mini-3a_v0.kicad_pcb`: refilled temporary copy
