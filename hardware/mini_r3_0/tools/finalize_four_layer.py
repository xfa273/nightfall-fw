#!/usr/bin/env python3
"""Add the final split GND/GND2 pours to mini_r3 In2.Cu.

Routing checkpoints deliberately omit In2 pours so that candidate tracks can
be reviewed without stale-fill geometry.  This finalizer clones the *current*
In1 split outlines after all return-path edits are complete, puts the clones
on In2.Cu, and leaves zone filling to the subsequent KiCad CLI DRC command.
The input and output must be different files.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import tempfile

import pcbnew


SOURCE_ZONES = {
    "LOGIC_GND_IN1": "LOGIC_GND_IN2",
    "POWER_GND2_IN1": "POWER_GND2_IN2",
}


def save_atomic(board: object, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(
        prefix=f".{output.stem}.", suffix=".kicad_pcb", dir=str(output.parent)
    )
    os.close(fd)
    temporary = Path(temporary_name)
    try:
        if not pcbnew.SaveBoard(str(temporary), board):
            raise RuntimeError(f"failed to save temporary board: {temporary}")
        if pcbnew.LoadBoard(str(temporary)) is None:
            raise RuntimeError(f"failed to reload temporary board: {temporary}")
        os.replace(temporary, output)
    finally:
        if temporary.exists():
            temporary.unlink()
        for suffix in (".kicad_pro", ".kicad_prl"):
            sidecar = temporary.with_suffix(suffix)
            if sidecar.exists():
                sidecar.unlink()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    source = args.input.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if source == output:
        raise RuntimeError("input and output must differ")
    if not source.is_file():
        raise RuntimeError(f"missing input: {source}")
    if output.exists():
        raise RuntimeError(f"refusing to replace output: {output}")

    board = pcbnew.LoadBoard(str(source))
    if board is None:
        raise RuntimeError(f"cannot load {source}")
    if int(board.GetCopperLayerCount()) != 4:
        raise RuntimeError(
            f"expected four copper layers, found {board.GetCopperLayerCount()}"
        )

    by_name = {str(zone.GetZoneName()): zone for zone in board.Zones()}
    missing = sorted(set(SOURCE_ZONES) - set(by_name))
    if missing:
        raise RuntimeError("missing final In1 zones: " + ", ".join(missing))

    # Replace any provisional In2 copies so their split boundary exactly
    # follows the final, noise-reviewed In1 reference plane.
    for destination_name in SOURCE_ZONES.values():
        existing = [
            zone
            for zone in list(board.Zones())
            if str(zone.GetZoneName()) == destination_name
        ]
        if len(existing) > 1:
            raise RuntimeError(f"duplicate zone {destination_name}")
        for zone in existing:
            board.Remove(zone)

    for source_name, destination_name in SOURCE_ZONES.items():
        clone = pcbnew.ZONE(by_name[source_name])
        clone.SetLayer(pcbnew.In2_Cu)
        clone.SetZoneName(destination_name)
        clone.SetAssignedPriority(0)
        # In2 is primarily a routing layer.  Direct connections avoid narrow
        # thermal-spoke warnings at through pads while the 0.20 mm zone
        # clearance still creates proper antipads for other nets.
        clone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        clone.UnFill()
        board.Add(clone)

    board.BuildConnectivity()
    save_atomic(board, output)

    check = pcbnew.LoadBoard(str(output))
    if check is None:
        raise RuntimeError("saved board did not reload")
    found = {
        str(zone.GetZoneName()): str(zone.GetNetname()) for zone in check.Zones()
    }
    expected = {"LOGIC_GND_IN2": "GND", "POWER_GND2_IN2": "GND2"}
    if any(found.get(name) != net for name, net in expected.items()):
        raise RuntimeError(f"final In2 zone contract failed: {found}")

    print(f"saved={output}")
    print("In2.Cu=LOGIC_GND_IN2(GND)+POWER_GND2_IN2(GND2)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
