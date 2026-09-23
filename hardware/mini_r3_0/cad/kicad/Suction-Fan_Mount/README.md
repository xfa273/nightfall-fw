# Suction Fan Mount mechanical PCB

This project converts `source/Suction-Fan_Mount.DXF` into a mechanical-only
1.6 mm FR-4 PCB. It contains no circuit, copper features, components, nets, or
routing.

## Geometry mapping

- DXF `LINE` and `ARC` entities become KiCad `Edge.Cuts`.
- DXF `CIRCLE` entities become round NPTH drills for unambiguous fabrication.
- Source dimensions: 38.0 mm x 22.0 mm.
- Round holes: 2 x 1.6 mm and 1 x 2.3 mm.
- Source SHA-256: `c101fbe827477468e041bf8332646104333e9f8e1a050116760072fa72abea9c`.

## Regeneration

```sh
python3 tools/dxf_to_kicad_edgecuts.py \
  source/Suction-Fan_Mount.DXF \
  Suction-Fan_Mount.kicad_pcb
```

The converter rejects unsupported geometry and verifies that every line/arc
endpoint belongs to a closed contour before writing the board.
