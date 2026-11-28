# HM_Nightfall-mini-3a v0 fabrication notes

## Order settings

- PCB type: rigid FR-4
- Layers: 2
- Finished thickness: 1.6 mm
- Finished copper: 2 oz outer copper
- Solder mask: both sides (color is optional; green matches the preview)
- Silkscreen: top side only (white is recommended)
- Surface finish: ENIG recommended for the fine-pitch UFQFPN/QFN devices; lead-free HASL is also manufacturable if cost is prioritized
- Castellated holes / edge plating / impedance control: not required

## Included manufacturing layers

- Front copper
- Back copper
- Front solder mask
- Back solder mask
- Front manufacturing silkscreen
- Board profile
- Separate plated and non-plated Excellon drill files

`User.5` (`Footprint Silk (No Print)`) is intentionally excluded. The top silkscreen contains only the imported mechanical outlines and the board name.

## Verification summary

- KiCad 10.0.3 ERC: 0 violations
- KiCad 10.0.3 DRC: 0 unconnected items
- No clearance, track-width, via-size, thermal, or dangling-track violations remain
- 90 remaining DRC entries are footprint-library mismatch warnings caused by local/placed footprint edits
- 19 schematic-parity entries are intentional no-connect pins
- U5 schematic pin 49 (exposed GND pad) has no physical footprint pad; this is an accepted prototype risk because the selected hand-solder footprint intentionally omits the exposed pad

The upload ZIP contains only files intended for the PCB manufacturer's parser. This notes file and the DRC/ERC reports are intentionally outside that ZIP.
