#!/usr/bin/env python3
"""Synchronize the MP6551 slew-rate resistors before routing.

The imported MP6551 footprint previously assigned pin 6 to GND2.  Pin 6 is
actually SR, so this helper creates the two schematic SR nets, reassigns the
driver pads, and adds the matching 220 kOhm 0402 footprints.  It is intended
for an unrouted board, after the reviewed placement has been applied.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pcbnew


DATASHEET = (
    "https://www.monolithicpower.com/en/documentview/productdocument/"
    "index/version/2/document_type/Datasheet/lang/en/sku/MP6551GQB/"
)

RESISTORS = {
    "R34": {
        "path": "/03805902-9cd2-4f42-a8d3-3c149ecc12cb",
        "description": (
            "MP6551 left-motor slew-rate programming resistor; 220 kOhm "
            "initial value balances EMI and switching loss at 100 kHz PWM."
        ),
        "net": "/MOTOR_L_SR",
        "driver": "U3",
        "position": (141.800, 125.850, 90.0),
    },
    "R35": {
        "path": "/3c3b90da-8621-47c4-bb13-faf7c8e5d137",
        "description": (
            "MP6551 right-motor slew-rate programming resistor; 220 kOhm "
            "initial value balances EMI and switching loss at 100 kHz PWM."
        ),
        "net": "/MOTOR_R_SR",
        "driver": "U2",
        "position": (140.800, 120.244, 90.0),
    },
}


def mm(value: float) -> int:
    return int(pcbnew.FromMM(value))


def find_pad(footprint: pcbnew.FOOTPRINT, number: str) -> pcbnew.PAD:
    for pad in footprint.Pads():
        if pad.GetNumber() == number:
            return pad
    raise RuntimeError(f"{footprint.GetReference()} has no pad {number}")


def get_or_add_net(board: pcbnew.BOARD, name: str) -> pcbnew.NETINFO_ITEM:
    net = board.GetNetInfo().GetNetItem(name)
    if net is None:
        net = pcbnew.NETINFO_ITEM(board, name)
        board.Add(net)
    return net


def load_resistor() -> pcbnew.FOOTPRINT:
    library = Path(
        "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints/"
        "Resistor_SMD.pretty"
    )
    footprint = pcbnew.FootprintLoad(str(library), "R_0402_1005Metric")
    if footprint is None:
        raise RuntimeError(f"could not load 0402 footprint from {library}")
    return footprint


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    board = pcbnew.LoadBoard(str(args.input))
    if board is None:
        raise RuntimeError(f"could not load {args.input}")
    if list(board.GetTracks()) or board.GetAreaCount():
        raise RuntimeError("add_sr_resistors.py must run before routing/zones")

    gnd2 = board.GetNetInfo().GetNetItem("GND2")
    if gnd2 is None:
        raise RuntimeError("board has no GND2 net")

    for reference, config in RESISTORS.items():
        if board.FindFootprintByReference(reference) is not None:
            raise RuntimeError(f"footprint already exists: {reference}")

        sr_net = get_or_add_net(board, str(config["net"]))
        driver = board.FindFootprintByReference(str(config["driver"]))
        if driver is None:
            raise RuntimeError(f"missing driver: {config['driver']}")
        find_pad(driver, "6").SetNet(sr_net)

        resistor = load_resistor()
        resistor.SetFPIDAsString("Resistor_SMD:R_0402_1005Metric")
        resistor.SetReference(reference)
        resistor.SetValue("220k")
        resistor.SetPath(pcbnew.KIID_PATH(str(config["path"])))
        x, y, angle = config["position"]
        resistor.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        resistor.SetOrientationDegrees(angle)
        resistor.Reference().SetVisible(False)
        resistor.Value().SetVisible(False)
        resistor.SetField("Datasheet", DATASHEET)
        resistor.SetField("Description", str(config["description"]))
        find_pad(resistor, "1").SetNet(gnd2)
        find_pad(resistor, "2").SetNet(sr_net)
        board.Add(resistor)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not pcbnew.SaveBoard(str(args.output), board):
        raise RuntimeError(f"could not save {args.output}")
    for reference, config in RESISTORS.items():
        x, y, angle = config["position"]
        print(
            f"added {reference} ({config['net']}) at "
            f"({x:.3f}, {y:.3f}, {angle:.1f} deg)"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
