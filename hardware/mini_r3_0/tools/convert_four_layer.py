#!/usr/bin/env python3
"""Convert the reviewed mini_r3 board from two to four copper layers.

The outer-layer routing and zones are preserved byte-for-byte at the item
level.  In1.Cu receives clones of the reviewed logic-GND and power-GND2
outlines as the primary reference plane.  In2.Cu is initially free for
low-current routing; ``finalize_four_layer.py`` later copies the final split
outlines there as low-priority pours.  The input and output must be different
files.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pcbnew


SOURCE_ZONES = {
    "LOGIC_GND_F": ("LOGIC_GND_IN1", "LOGIC_GND_IN2"),
    "POWER_GND2_F": ("POWER_GND2_IN1", "POWER_GND2_IN2"),
}


def main() -> int:
    parser = argparse.ArgumentParser()
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
    if board.GetCopperLayerCount() != 2:
        raise RuntimeError(
            f"expected reviewed two-layer source, found {board.GetCopperLayerCount()} layers"
        )
    if board.HasItemsOnLayer(pcbnew.In1_Cu) or board.HasItemsOnLayer(pcbnew.In2_Cu):
        raise RuntimeError("source unexpectedly already contains inner-layer items")

    zones_by_name = {zone.GetZoneName(): zone for zone in board.Zones()}
    missing = sorted(set(SOURCE_ZONES) - set(zones_by_name))
    if missing:
        raise RuntimeError("missing reviewed source zones: " + ", ".join(missing))

    board.SetCopperLayerCount(4)
    destination_names = []
    for source_name, names in SOURCE_ZONES.items():
        # ``Clone()`` is exposed as the EDA_ITEM base class by KiCad's SWIG
        # bindings, which hides the zone-only setters.  The copy constructor
        # preserves the polygon/settings while retaining the concrete type.
        # Only the uninterrupted In1 reference plane is present during route
        # development.  ``finalize_four_layer.py`` adds the In2 low-priority
        # pours from these final outlines after routing is complete.
        for layer, destination_name in ((pcbnew.In1_Cu, names[0]),):
            clone = pcbnew.ZONE(zones_by_name[source_name])
            clone.SetLayer(layer)
            clone.SetZoneName(destination_name)
            # Inner-layer pours are a reference/copper-balance aid, never an
            # obstacle that should win over deliberately routed logic tracks.
            clone.SetAssignedPriority(0)
            board.Add(clone)
            destination_names.append(destination_name)

    pcbnew.ZONE_FILLER(board).Fill(list(board.Zones()))
    board.BuildConnectivity()
    if not pcbnew.SaveBoard(str(output), board):
        raise RuntimeError(f"failed to save {output}")

    check = pcbnew.LoadBoard(str(output))
    if check is None or check.GetCopperLayerCount() != 4:
        raise RuntimeError("saved board did not reload as four layers")
    found = {zone.GetZoneName() for zone in check.Zones()}
    if not set(destination_names) <= found:
        raise RuntimeError("inner ground zones were not preserved after reload")

    # KiCad's stackup editor and JLCPCB ordering are the source of truth for
    # dielectric/core selection.  The board deliberately records only the
    # 1.6 mm total thickness here; fabrication notes require 2 oz outer and
    # 1 oz inner copper and a JLC-confirmed symmetric four-layer stackup.

    print(f"saved={output}")
    print("layers=F.Cu,In1.Cu,In2.Cu,B.Cu")
    print("In1.Cu=split GND/GND2 reference plane")
    print("In2.Cu=signal/logic-power; finalizer adds split GND/GND2 pours")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
