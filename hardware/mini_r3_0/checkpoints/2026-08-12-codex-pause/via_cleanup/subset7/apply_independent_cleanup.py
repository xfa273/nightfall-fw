#!/usr/bin/env python3
"""Remove the 12 reviewed drill-in-pad overlaps from the fan-route handoff.

The input and output must be different canonical-named project copies.  The
script is deliberately topology-specific and aborts if the expected source
geometry is not present.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import tempfile

import wx

_APP = wx.GetApp() or wx.App(False)
_APP.SetAssertMode(wx.APP_ASSERT_EXCEPTION)
wx.Log.SetLogLevel(wx.LOG_Error)

import pcbnew

F = pcbnew.F_Cu
B = pcbnew.B_Cu


def iu(value: float) -> int:
    return int(pcbnew.FromMM(value))


def pt(x: float, y: float) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I(iu(x), iu(y))


def xy(value: object) -> tuple[float, float]:
    return (pcbnew.ToMM(value.x), pcbnew.ToMM(value.y))


def close(value: object, expected: tuple[float, float], tol: float = 0.003) -> bool:
    x, y = xy(value)
    return abs(x - expected[0]) < tol and abs(y - expected[1]) < tol


def require_one(items: list[object], label: str) -> object:
    if len(items) != 1:
        raise RuntimeError(f"{label}: expected 1 item, found {len(items)}")
    return items[0]


def via_at(board: object, net: str, at: tuple[float, float], dia: float, drill: float) -> list[object]:
    return [
        item
        for item in board.GetTracks()
        if isinstance(item, pcbnew.PCB_VIA)
        and item.GetNetname() == net
        and close(item.GetPosition(), at)
        and abs(pcbnew.ToMM(item.GetWidth(F)) - dia) < 0.003
        and abs(pcbnew.ToMM(item.GetDrillValue()) - drill) < 0.003
    ]


def same_edge(item: object, a: tuple[float, float], b: tuple[float, float]) -> bool:
    return (close(item.GetStart(), a) and close(item.GetEnd(), b)) or (
        close(item.GetStart(), b) and close(item.GetEnd(), a)
    )


def track_at(
    board: object,
    net: str,
    layer: int,
    a: tuple[float, float],
    b: tuple[float, float],
    width: float,
) -> list[object]:
    return [
        item
        for item in board.GetTracks()
        if not isinstance(item, pcbnew.PCB_VIA)
        and item.GetNetname() == net
        and item.GetLayer() == layer
        and same_edge(item, a, b)
        and abs(pcbnew.ToMM(item.GetWidth()) - width) < 0.003
    ]


def move_via(
    board: object,
    net: str,
    old: tuple[float, float],
    new: tuple[float, float],
    dia: float,
    drill: float,
) -> None:
    via = require_one(via_at(board, net, old, dia, drill), f"{net} via at {old}")
    via.SetPosition(pt(*new))


def remove_via(
    board: object,
    net: str,
    at: tuple[float, float],
    dia: float,
    drill: float,
) -> None:
    board.Delete(require_one(via_at(board, net, at, dia, drill), f"{net} via at {at}"))


def move_endpoint(
    board: object,
    net: str,
    layer: int,
    old: tuple[float, float],
    new: tuple[float, float],
    endpoint: str,
) -> None:
    found = []
    for item in board.GetTracks():
        if isinstance(item, pcbnew.PCB_VIA) or item.GetNetname() != net or item.GetLayer() != layer:
            continue
        position = item.GetStart() if endpoint == "start" else item.GetEnd()
        if close(position, old):
            found.append(item)
    item = require_one(found, f"{net} {board.GetLayerName(layer)} {endpoint} at {old}")
    if endpoint == "start":
        item.SetStart(pt(*new))
    else:
        item.SetEnd(pt(*new))


def delete_track(
    board: object,
    net: str,
    layer: int,
    a: tuple[float, float],
    b: tuple[float, float],
    width: float,
) -> None:
    board.Delete(require_one(track_at(board, net, layer, a, b, width), f"{net} track {a}->{b}"))


def add_track(
    board: object,
    net_name: str,
    layer: int,
    a: tuple[float, float],
    b: tuple[float, float],
    width: float,
) -> None:
    net = board.FindNet(net_name)
    if net is None:
        raise RuntimeError(f"missing net {net_name}")
    item = pcbnew.PCB_TRACK(board)
    item.SetStart(pt(*a))
    item.SetEnd(pt(*b))
    item.SetWidth(iu(width))
    item.SetLayer(layer)
    item.SetNet(net)
    board.Add(item)


def add_via(
    board: object,
    net_name: str,
    at: tuple[float, float],
    diameter: float = 0.6,
    drill: float = 0.3,
) -> None:
    net = board.FindNet(net_name)
    if net is None:
        raise RuntimeError(f"missing net {net_name}")
    item = pcbnew.PCB_VIA(board)
    item.SetPosition(pt(*at))
    item.SetWidth(iu(diameter))
    item.SetDrill(iu(drill))
    item.SetLayerPair(F, B)
    item.SetNet(net)
    board.Add(item)


def relocate_zone_stitch(
    board: object,
    net: str,
    old: tuple[float, float],
    new: tuple[float, float],
    dia: float,
    drill: float,
    pad: tuple[float, float] | None = None,
    toe_width: float = 0.20,
) -> None:
    move_via(board, net, old, new, dia, drill)
    if pad is not None:
        add_track(board, net, F, pad, new, toe_width)


def cleanup_ground_stitches(board: object) -> None:
    # Preserve every stitch count.  The new annuli remain adjacent to their
    # original ground pads, but the plated drills no longer pierce solderable
    # copper.  Short F.Cu toes are only needed where the annulus does not touch.
    # The buzzer return is boxed in by the board edge and D3/SPK copper.  A
    # 0.40 mm component nudge preserves its ground via at the reviewed point
    # while moving the solderable pad clear of the drill.
    ubuz = board.FindFootprintByReference("UBUZ0")
    if ubuz is None or not close(ubuz.GetPosition(), (132.10, 126.00)):
        raise RuntimeError("unexpected UBUZ0 placement")
    ubuz.SetPosition(pt(132.10, 125.60))
    move_endpoint(board, "/BUZZER_PWM", F, (131.2872, 125.492), (131.2872, 125.092), "start")
    move_endpoint(board, "Net-(D3-A)", F, (132.9128, 126.00), (132.9128, 125.60), "end")
    move_endpoint(board, "Net-(D3-A)", F, (132.9128, 126.00), (132.9128, 125.60), "start")

    relocate_zone_stitch(board, "GND2", (159.40, 123.00), (159.70, 123.00), 0.8, 0.4)
    relocate_zone_stitch(board, "GND2", (137.00, 123.00), (136.80, 121.95), 0.8, 0.4)


def cleanup_imu_supply(board: object) -> None:
    net = "+3V3"
    move_via(board, net, (149.90, 109.00), (150.10, 109.15), 0.6, 0.3)
    move_endpoint(board, net, F, (149.90, 109.00), (150.10, 109.15), "start")
    move_endpoint(board, net, B, (149.90, 109.00), (150.10, 109.15), "end")


def cleanup_front_sensor_toes(board: object) -> None:
    # Keep normal 0.60/0.30 mm vias: there is enough room outside both 0402
    # pads, so no dense-toe exception is needed here.
    net = "Net-(IR_LED_FR0-PadA)"
    move_via(board, net, (165.20, 82.15), (165.31, 82.305), 0.6, 0.3)
    move_endpoint(board, net, F, (165.20, 82.15), (165.31, 82.305), "end")
    move_endpoint(board, net, B, (165.20, 82.15), (165.31, 82.305), "start")

    net = "/SENSOR_R"
    move_via(board, net, (162.40, 86.95), (162.40, 86.795), 0.6, 0.3)
    move_endpoint(board, net, F, (162.40, 86.95), (162.40, 86.795), "end")
    move_endpoint(board, net, B, (162.40, 86.95), (162.40, 86.795), "start")


def cleanup_c8(board: object) -> None:
    # Move C9 0.50 mm to make room for an off-pad 0.80/0.40 transition while
    # retaining the existing wide B.Cu VBAT_SW trunk and second bank via.
    c9 = board.FindFootprintByReference("C9")
    if c9 is None or not close(c9.GetPosition(), (136.8, 119.2)):
        raise RuntimeError("unexpected C9 placement")
    c9.SetPosition(pt(136.8, 119.7))
    for other in ((135.4, 120.15), (137.8, 121.2)):
        delete_track(board, "VBAT_SW", F, (136.8, 120.15), other, 0.8)
    add_track(board, "VBAT_SW", F, (135.4, 120.15), (136.8, 120.65), 0.8)
    add_track(board, "VBAT_SW", F, (136.8, 120.65), (137.8, 121.2), 0.8)

    move_via(board, "VBAT_SW", (138.30, 117.45), (137.70, 117.45), 1.0, 0.4)
    # Reduce only this relocated via's annulus to the reviewed 0.80 mm size.
    require_one(via_at(board, "VBAT_SW", (137.70, 117.45), 1.0, 0.4), "moved C8 via").SetWidth(iu(0.8))
    delete_track(board, "VBAT_SW", B, (134.4, 117.8), (138.3, 117.45), 0.8)
    delete_track(board, "VBAT_SW", B, (138.3, 117.45), (139.35, 117.45), 0.8)
    add_track(board, "VBAT_SW", F, (137.70, 117.45), (138.50, 117.45), 0.6)
    add_track(board, "VBAT_SW", B, (134.40, 117.80), (137.70, 117.45), 0.8)
    add_track(board, "VBAT_SW", B, (137.70, 117.45), (139.35, 117.45), 0.8)


def cleanup_fan_drain(board: object) -> None:
    # Relocate, rather than delete, the Q3 drain transition so the 2 A fan
    # branch retains both 0.80/0.40 mm vias.  The new point is the existing
    # B.Cu Y-junction and is connected to pad 3 by a short 0.80 mm F.Cu toe.
    move_via(board, "/FAN_NEG_INTERNAL", (160.9375, 122.0), (160.40, 121.00), 0.8, 0.4)
    delete_track(board, "/FAN_NEG_INTERNAL", B, (160.40, 121.00), (160.9375, 122.0), 0.8)
    add_track(board, "/FAN_NEG_INTERNAL", F, (160.9375, 122.0), (160.40, 121.00), 0.8)


def cleanup_power_switch_return(board: object) -> None:
    # All four sides around the original 0402 pad are occupied by Q2, R33.1,
    # and the two protection-network vias.  Moving R33 0.50 mm upward keeps the
    # normal via in place and moves its pad completely off the plated drill.
    r33 = board.FindFootprintByReference("R33")
    if r33 is None or not close(r33.GetPosition(), (134.00, 112.60)):
        raise RuntimeError("unexpected R33 placement")
    r33.SetPosition(pt(134.00, 112.10))
    move_endpoint(board, "/PWR_SWITCH_RETURN", F, (133.50, 112.60), (133.50, 112.10), "start")
    move_endpoint(board, "/PWR_GATE_INTERNAL", F, (134.50, 112.60), (134.50, 112.10), "start")


def refill(board: object) -> None:
    zones = list(board.Zones())
    if zones:
        pcbnew.ZONE_FILLER(board).Fill(zones)
    board.BuildConnectivity()


def save_atomic(board: object, output: Path) -> None:
    fd, name = tempfile.mkstemp(prefix=".via-clean-", suffix=".kicad_pcb", dir=str(output.parent))
    os.close(fd)
    temp = Path(name)
    try:
        if not pcbnew.SaveBoard(str(temp), board):
            raise RuntimeError("SaveBoard failed")
        if pcbnew.LoadBoard(str(temp)) is None:
            raise RuntimeError("reload failed")
        os.replace(temp, output)
    finally:
        if temp.exists():
            temp.unlink()
        for suffix in (".kicad_pro", ".kicad_prl"):
            sidecar = temp.with_suffix(suffix)
            if sidecar.exists():
                sidecar.unlink()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    source = args.input.resolve()
    output = args.output.resolve()
    if source == output or output.exists():
        raise RuntimeError("input/output must differ and output must not exist")
    board = pcbnew.LoadBoard(str(source))
    if board is None:
        raise RuntimeError(f"cannot load {source}")

    cleanup_ground_stitches(board)
    cleanup_front_sensor_toes(board)
    cleanup_c8(board)
    cleanup_fan_drain(board)
    refill(board)
    save_atomic(board, output)
    print(f"saved={output} unconnected={board.GetConnectivity().GetUnconnectedCount(False)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
