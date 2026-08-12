#!/usr/bin/env python3
"""Route the mini_r3 motor/fan control bundle through the MCU escape area.

This is a board-specific, deterministic finishing helper.  It is intended for
the reviewed output of ``finish_protected.py`` after the STM32 exposed pad has
been removed for hand-solder assembly.  The helper keeps the 0.8 mm power
trunks intact while moving two short B.Cu doglegs far enough apart to make a
small-signal escape channel under U5.  Control routes are added only after the
power-channel move has passed project DRC on a temporary project copy.

The output remains a review artifact: refill zones and run the exact project
DRC before accepting it.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import tempfile

import pcbnew


def iu(value_mm: float) -> int:
    return int(pcbnew.FromMM(value_mm))


def point(x_mm: float, y_mm: float) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I(iu(x_mm), iu(y_mm))


def point_mm(value: object) -> tuple[float, float]:
    return float(pcbnew.ToMM(value.x)), float(pcbnew.ToMM(value.y))


def close(left: object, right: tuple[float, float], tolerance_mm: float = 0.002) -> bool:
    x_mm, y_mm = point_mm(left)
    return math.hypot(x_mm - right[0], y_mm - right[1]) <= tolerance_mm


def net(board: object, name: str) -> object:
    result = board.FindNet(name)
    if result is None:
        raise RuntimeError(f"missing net: {name}")
    return result


def remove_track_groups(
    board: object,
    groups: list[
        tuple[
            str,
            int,
            float,
            list[tuple[tuple[float, float], tuple[float, float]]],
        ]
    ],
    vias: list[tuple[str, tuple[float, float], float, float]] | None = None,
) -> None:
    # KiCad 9's SWIG wrapper can invalidate the TRACKS proxy after the first
    # BOARD.Remove().  Resolve every segment against one stable snapshot, then
    # remove the resolved items as a batch.
    snapshot = list(board.GetTracks())
    matches: list[object] = []
    for net_name, layer, width_mm, segments in groups:
        for start, end in segments:
            segment_matches = []
            for item in snapshot:
                if isinstance(item, pcbnew.PCB_VIA):
                    continue
                if item.GetNetname() != net_name or item.GetLayer() != layer:
                    continue
                if abs(float(pcbnew.ToMM(item.GetWidth())) - width_mm) > 0.002:
                    continue
                direct = close(item.GetStart(), start) and close(item.GetEnd(), end)
                reverse = close(item.GetStart(), end) and close(item.GetEnd(), start)
                if direct or reverse:
                    segment_matches.append(item)
            if len(segment_matches) != 1:
                raise RuntimeError(
                    f"expected one {net_name} track {start}->{end}, "
                    f"found {len(segment_matches)}"
                )
            matches.extend(segment_matches)
    for net_name, position, diameter_mm, drill_mm in vias or []:
        via_matches = []
        for item in snapshot:
            if not isinstance(item, pcbnew.PCB_VIA):
                continue
            if item.GetNetname() != net_name or not close(item.GetPosition(), position):
                continue
            if abs(float(pcbnew.ToMM(item.GetWidth(pcbnew.F_Cu))) - diameter_mm) > 0.002:
                continue
            if abs(float(pcbnew.ToMM(item.GetDrillValue())) - drill_mm) > 0.002:
                continue
            via_matches.append(item)
        if len(via_matches) != 1:
            raise RuntimeError(
                f"expected one {net_name} via at {position}, found {len(via_matches)}"
            )
        matches.extend(via_matches)
    for item in matches:
        board.Remove(item)


def add_track(
    board: object,
    net_name: str,
    layer: int,
    width_mm: float,
    start: tuple[float, float],
    end: tuple[float, float],
) -> None:
    item = pcbnew.PCB_TRACK(board)
    item.SetNet(net(board, net_name))
    item.SetLayer(layer)
    item.SetWidth(iu(width_mm))
    item.SetStart(point(*start))
    item.SetEnd(point(*end))
    board.Add(item)


def add_polyline(
    board: object,
    net_name: str,
    layer: int,
    width_mm: float,
    vertices: list[tuple[float, float]],
) -> None:
    for start, end in zip(vertices, vertices[1:]):
        add_track(board, net_name, layer, width_mm, start, end)


def add_via(
    board: object,
    net_name: str,
    position: tuple[float, float],
    diameter_mm: float = 0.40,
    drill_mm: float = 0.20,
) -> None:
    item = pcbnew.PCB_VIA(board)
    item.SetNet(net(board, net_name))
    item.SetPosition(point(*position))
    item.SetWidth(iu(diameter_mm))
    item.SetDrill(iu(drill_mm))
    item.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    board.Add(item)


def move_power_channel(board: object) -> None:
    """Open a B.Cu signal corridor without narrowing either current path."""

    # Keep both endpoint via banks and F.Cu fanouts unchanged.  Only replace
    # the reviewed 0.8 mm B.Cu doglegs between them.
    remove_track_groups(
        board,
        [
            (
                "/MOTOR_R_OUT1",
                pcbnew.B_Cu,
                0.8,
                [
                    ((139.0, 116.3), (139.0, 115.5)),
                    ((139.0, 115.5), (146.2, 115.5)),
                    ((146.2, 115.5), (146.2, 116.8)),
                    ((146.2, 116.8), (146.2, 120.8)),
                    ((146.2, 120.8), (145.9, 120.8)),
                ],
            ),
            (
                "/FAN_NEG_INTERNAL",
                pcbnew.B_Cu,
                0.8,
                [
                    ((145.0, 113.5), (147.6, 114.5)),
                    ((147.6, 114.5), (147.6, 120.0)),
                    ((147.6, 120.0), (158.5, 120.0)),
                ],
            ),
        ],
    )
    add_polyline(
        board,
        "/MOTOR_R_OUT1",
        pcbnew.B_Cu,
        0.8,
        [
            (139.0, 116.3),
            (139.0, 114.7),
            (148.5, 114.7),
            (148.5, 121.5),
            (146.6, 121.5),
            (145.9, 120.8),
        ],
    )
    add_polyline(
        board,
        "/FAN_NEG_INTERNAL",
        pcbnew.B_Cu,
        0.8,
        [
            (145.0, 113.5),
            (151.5, 113.5),
            (151.5, 119.2),
            (152.3, 120.0),
            (158.5, 120.0),
        ],
    )

def save_atomic(board: object, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary_name = tempfile.mkstemp(
        prefix=f".{output.stem}.", suffix=".kicad_pcb", dir=output.parent
    )
    os.close(handle)
    temporary = Path(temporary_name)
    try:
        if not pcbnew.SaveBoard(str(temporary), board):
            raise RuntimeError(f"could not save temporary board: {temporary}")
        if pcbnew.LoadBoard(str(temporary)) is None:
            raise RuntimeError(f"could not reload temporary board: {temporary}")
        os.replace(temporary, output)
    finally:
        if temporary.exists():
            temporary.unlink()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--move-power-channel", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_path = args.input.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    board = pcbnew.LoadBoard(str(input_path))
    if board is None:
        raise RuntimeError(f"could not load board: {input_path}")
    if args.move_power_channel:
        move_power_channel(board)
    board.BuildConnectivity()
    save_atomic(board, output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
