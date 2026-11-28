# Suction-Fan_Mount fabrication notes

## Order settings

- Product: rigid FR-4 PCB
- Board size: 38.0 mm x 22.0 mm
- Layers: 2
- Finished thickness: 1.6 mm
- Finished copper: 1 oz (the copper artwork is intentionally empty)
- Solder mask: both sides; color optional
- Silkscreen: none
- Surface finish: standard/default; there is no exposed copper artwork
- Electrical test: not needed for function, because this is a mechanical part
- Castellated holes / edge plating / impedance control: not required

## Order remark

Paste this into the manufacturer's order remarks:

> Mechanical FR-4 part only. There is intentionally no copper or circuitry. Please CNC-route the external profile and all internal closed cutouts exactly as shown, and make all Excellon holes non-plated (NPTH).

## Expected geometry

- External size: 38.0 mm x 22.0 mm
- CNC-routed closed contours: 4 total, consisting of one external profile and three internal cutouts
- NPTH drill holes: 2 x 1.6 mm and 1 x 2.3 mm

Confirm all three internal cutouts and all three NPTH holes in the fabricator's online Gerber/DFM preview before payment.
