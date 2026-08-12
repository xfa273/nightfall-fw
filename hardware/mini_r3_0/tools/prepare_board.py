#!/usr/bin/env python3
"""Apply the reviewed pre-routing placement to the mini_r3 main PCB.

Run this only after PCB Editor's "Update PCB from Schematic" operation.  The
script is intentionally limited to deterministic footprint moves and the IMU
land-pattern manufacturing adjustment; it does not add copper.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pcbnew


PLACEMENT = {
    # MPM3610 input/output, feedback, enable and ground-star neighbourhood.
    "C22": (146.250, 125.700, 0.0),
    "C23": (149.300, 130.950, 90.0),
    "R29": (142.700, 130.100, -90.0),
    "R30": (142.700, 128.200, 90.0),
    "R31": (143.400, 126.200, 180.0),
    # Put the sole GND2↔GND bridge beside the motor-control boundary so PWM
    # return current does not take a long detour around the split plane.
    "R0": (142.800, 120.200, 0.0),
    # MP6551 local 100 nF first, 10 uF / 25 V 0805 immediately behind it.
    "C8": (139.350, 118.100, 90.0),
    "C9": (136.800, 119.200, -90.0),
    "C6": (139.350, 123.688, 90.0),
    "C7": (136.800, 123.688, -90.0),
    # Battery monitor referenced to logic ground and adjacent reset collateral.
    "R4": (142.500, 114.900, 180.0),
    "R5": (142.500, 113.700, 0.0),
    "C21": (143.600, 112.200, 180.0),
    "C3": (139.250, 111.500, 0.0),
    # 3S-safe P-channel MOSFET gate protection.
    "R12": (131.800, 112.200, 0.0),
    "R33": (134.000, 112.600, 180.0),
    "D5": (129.950, 115.500, -90.0),
    # Logic regulator support parts.
    "C14": (151.450, 130.200, -90.0),
    "C15": (154.500, 133.800, 180.0),
    # Buzzer MOSFET gate pull-down; keep it beside UBUZ0, not the LDO.
    "R8": (132.500, 124.300, 0.0),
    # Compact fan switching / flyback / HF bypass loop.
    "Q3": (160.000, 122.000, 0.0),
    "D4": (158.000, 125.800, 180.0),
    "C24": (156.600, 122.600, 0.0),
    "R32": (156.900, 121.050, 0.0),
}


EXPECTED_FOOTPRINTS = {
    "Q2": "Package_TO_SOT_SMD:SOT-23-6",
    "D4": "Diode_SMD:D_SMA",
    "D5": "Diode_SMD:D_SOD-323",
    "C7": "Capacitor_SMD:C_0805_2012Metric",
    "C9": "Capacitor_SMD:C_0805_2012Metric",
}


def mm(value: float) -> int:
    return int(pcbnew.FromMM(value))


def move(board: pcbnew.BOARD, reference: str, x: float, y: float, angle: float) -> None:
    footprint = board.FindFootprintByReference(reference)
    if footprint is None:
        raise RuntimeError(f"missing footprint: {reference}")
    footprint.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
    footprint.SetOrientationDegrees(angle)


def adjust_imu_lands(board: pcbnew.BOARD) -> None:
    """Give the 0.5 mm-pitch LGA a real 0.20 mm copper gap for 2 oz."""
    imu = board.FindFootprintByReference("U4")
    if imu is None:
        raise RuntimeError("missing footprint: U4")
    for pad in imu.Pads():
        size = pad.GetSize()
        if size.x > size.y:
            pad.SetSize(pcbnew.VECTOR2I(size.x, mm(0.30)))
        else:
            pad.SetSize(pcbnew.VECTOR2I(mm(0.30), size.y))


def adjust_constrained_through_hole_lands(board: pcbnew.BOARD) -> None:
    """Keep fixed mechanical hole positions while restoring 0.20 mm copper gaps.

    K1 uses a 1.27 mm pitch and the front-left sensor pair is intentionally
    packed tightly for the optical geometry.  Only their copper diameter is
    reduced; drill locations and drill sizes remain unchanged.
    """
    programming_header = board.FindFootprintByReference("K1")
    if programming_header is None:
        raise RuntimeError("missing footprint: K1")
    for pad in programming_header.Pads():
        pad.SetSize(pcbnew.VECTOR2I(mm(1.00), mm(1.00)))

    for reference, number in (("IR_LED_L0", "K"), ("PT_L0", "K")):
        footprint = board.FindFootprintByReference(reference)
        if footprint is None:
            raise RuntimeError(f"missing footprint: {reference}")
        pad = footprint.FindPadByNumber(number)
        if pad is None:
            raise RuntimeError(f"missing pad: {reference}.{number}")
        pad.SetSize(pcbnew.VECTOR2I(mm(1.2708), mm(1.2708)))


def repair_imported_pad_orientations(board: pcbnew.BOARD) -> None:
    """Repair two imported 90-degree SOT footprints whose pads stayed at 0°."""
    for reference in ("UL0", "UFR0"):
        footprint = board.FindFootprintByReference(reference)
        if footprint is None:
            raise RuntimeError(f"missing footprint: {reference}")
        for pad in footprint.Pads():
            pad.SetOrientation(footprint.GetOrientation())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    board = pcbnew.LoadBoard(str(args.input))
    if board is None:
        raise RuntimeError(f"could not load {args.input}")
    if list(board.GetTracks()) or board.GetAreaCount():
        raise RuntimeError("prepare_board.py must run before routing/zones are added")

    for reference, expected in EXPECTED_FOOTPRINTS.items():
        footprint = board.FindFootprintByReference(reference)
        actual = footprint.GetFPIDAsString() if footprint else "<missing>"
        if actual != expected:
            raise RuntimeError(f"{reference}: expected {expected}, found {actual}")

    for reference, placement in PLACEMENT.items():
        move(board, reference, *placement)
    adjust_imu_lands(board)
    adjust_constrained_through_hole_lands(board)
    repair_imported_pad_orientations(board)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not pcbnew.SaveBoard(str(args.output), board):
        raise RuntimeError(f"could not save {args.output}")
    print(f"prepared {len(PLACEMENT)} footprints -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
