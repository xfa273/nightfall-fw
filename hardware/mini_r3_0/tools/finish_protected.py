#!/usr/bin/env python3
"""Conservatively finish approved mini_r3 protected nets after route_power.

This board-specific helper is the bridge between ``route_power.py`` and the
general signal-only ``route_board.py``.  It replaces four reviewed legacy
motor-driver doglegs with direct under-package duplicate-output links, adds a
few reviewed fixed connections, then reuses the latter's deterministic
obstacle-aware A* implementation only for the explicit low-current profiles
below.  It never generically routes GND/GND2 or the MPM3610 switch node, never
moves footprints, and limits GND/GND2 changes to reviewed MP6551 returns,
zone outlines, and fixed F/B stitching points.

The output is routing assistance, not a release artifact.  Run it on a whole
project copy, refill zones, compare DRC with the route_power checkpoint, and
inspect the high-current fan pair plus GND/GND2 return geometry manually.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import pcbnew

import route_board


@dataclasses.dataclass(frozen=True)
class NetSpec:
    name: str
    width_mm: float
    via_diameter_mm: float
    via_drill_mm: float
    max_vias: int
    max_expanded: int = 300_000
    max_net_expanded: int = 1_200_000
    candidate_pairs: int = 10
    local_escape: bool = False


# Order is intentional: compact control loops first, optical pulse-current
# branches next, and finally the global rails.  Existing complete nets are
# detected and left untouched.  High-current trunks and the remote fan pair
# are deliberately not eligible for this generic router.
NET_SPECS: Tuple[NetSpec, ...] = (
    NetSpec("/MPM_FB_INTERNAL", 0.20, 0.60, 0.30, 0, 120_000, 360_000, 6),
    NetSpec("/MPM_EN_INTERNAL", 0.20, 0.60, 0.30, 0, 120_000, 240_000, 4),
    NetSpec("/PWR_GATE_INTERNAL", 0.20, 0.60, 0.30, 0),
    NetSpec("Net-(D3-A)", 0.40, 0.60, 0.30, 0),
    NetSpec("Net-(IR_LED_FL0-C)", 0.30, 0.60, 0.30, 0),
    NetSpec("Net-(IR_LED_FL0-PadA)", 0.30, 0.60, 0.30, 0),
    NetSpec("Net-(IR_LED_FR0-C)", 0.30, 0.60, 0.30, 0),
    NetSpec("Net-(IR_LED_FR0-PadA)", 0.30, 0.60, 0.30, 0),
    NetSpec("Net-(IR_LED_L0-C)", 0.30, 0.60, 0.30, 0),
    NetSpec("Net-(IR_LED_L0-PadA)", 0.30, 0.60, 0.30, 0),
    NetSpec("Net-(IR_LED_R0-C)", 0.30, 0.60, 0.30, 0),
    NetSpec("Net-(IR_LED_R0-PadA)", 0.30, 0.60, 0.30, 0),
    NetSpec("+5V", 0.50, 0.80, 0.40, 0, 350_000, 3_000_000, 14),
    NetSpec("+3V3", 0.30, 0.60, 0.30, 0, 350_000, 6_000_000, 16),
)

# These routes were individually checked against the route_power checkpoint
# with filled-zone project DRC.  Their exact endpoints and intermediate
# vertices are board-placement anchors, not suggestions for a generic router.
EXACT_ROUTE_NAMES = (
    "/MOTOR_R_OUT1",
    "/MOTOR_R_OUT2",
    "/MOTOR_L_OUT1",
    "/MOTOR_L_OUT2",
    "/MOTOR_R_SR",
    "/MOTOR_L_SR",
    "Net-(U5B-VCAP_1)",
    "VBAT_SW",
    "/FAN_NEG_INTERNAL",
    "GND",
    "GND2",
)

# These are intentionally never sent to the generic A* router.  The fan pair
# uses a reviewed paired corridor and parallel vias; all MP6551 duplicate
# outputs use reviewed direct under-package links.
MANUAL_NETS = (
    "VBAT_RAW",
    "VBAT_SW",
    "/FAN_NEG_INTERNAL",
    "/PWR_SWITCH_RETURN",
    "/MOTOR_L_OUT1",
    "/MOTOR_L_OUT2",
    "/MOTOR_R_OUT1",
    "/MOTOR_R_OUT2",
    "GND2",
)


EXPECTED_ZONE_NAMES = {
    "LOGIC_GND_F",
    "LOGIC_GND_B",
    "POWER_GND2_F",
    "POWER_GND2_B",
}

EXPECTED_ZONES = {
    "LOGIC_GND_F": ("GND", pcbnew.F_Cu),
    "LOGIC_GND_B": ("GND", pcbnew.B_Cu),
    "POWER_GND2_F": ("GND2", pcbnew.F_Cu),
    "POWER_GND2_B": ("GND2", pcbnew.B_Cu),
}


def count_unconnected(board: object) -> int:
    board.BuildConnectivity()
    return int(board.GetConnectivity().GetUnconnectedCount(False))


def pads_by_net(board: object) -> Dict[int, List[object]]:
    result: Dict[int, List[object]] = {}
    for pad in board.GetPads():
        if pad.GetNetCode() > 0 and not pad.IsNoConnectPad():
            result.setdefault(int(pad.GetNetCode()), []).append(pad)
    return result


def iu(value_mm: float) -> int:
    return int(pcbnew.FromMM(value_mm))


def point(x_mm: float, y_mm: float) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I(iu(x_mm), iu(y_mm))


def pad(board: object, reference: str, number: str) -> object:
    footprint = board.FindFootprintByReference(reference)
    if footprint is None:
        raise RuntimeError(f"missing footprint: {reference}")
    result = footprint.FindPadByNumber(str(number))
    if result is None:
        raise RuntimeError(f"missing pad: {reference}.{number}")
    return result


def expect_pad_position(
    board: object,
    reference: str,
    number: str,
    x_mm: float,
    y_mm: float,
    tolerance_mm: float = 0.01,
) -> None:
    actual = pad(board, reference, number).GetPosition()
    actual_x = float(pcbnew.ToMM(actual.x))
    actual_y = float(pcbnew.ToMM(actual.y))
    if abs(actual_x - x_mm) > tolerance_mm or abs(actual_y - y_mm) > tolerance_mm:
        raise RuntimeError(
            f"stale placement: expected {reference}.{number} at "
            f"({x_mm:.3f}, {y_mm:.3f}) mm, found "
            f"({actual_x:.3f}, {actual_y:.3f}) mm"
        )


def verify_exact_route_anchors(board: object) -> None:
    for reference, number, x_mm, y_mm in (
        ("U5", "22", 147.262247, 118.413725),
        ("C_MCU-C0", "1", 146.360589, 118.960589),
        ("R33", "2", 133.500000, 112.600000),
        ("POWER0", "2", 149.251098, 135.153598),
        ("R4", "2", 142.000000, 114.900000),
        ("C8", "2", 139.350000, 117.450000),
        ("J2", "1", 137.092934, 85.042649),
        ("J2", "2", 138.507066, 83.357351),
        ("D4", "1", 160.000000, 125.800000),
        ("R0", "2", 143.300000, 120.200000),
        ("U2", "1", 140.693098, 116.949473),
        ("U2", "10", 142.543098, 116.949473),
        ("U2", "3", 140.743098, 117.949723),
        ("U2", "8", 142.543098, 117.949723),
        ("U2", "6", 141.893160, 118.849598),
        ("U2", "9", 142.543098, 117.449598),
        ("U3", "1", 140.693098, 122.537473),
        ("U3", "10", 142.543098, 122.537473),
        ("U3", "3", 140.743098, 123.537723),
        ("U3", "8", 142.543098, 123.537723),
        ("U3", "6", 141.893160, 124.437598),
        ("U3", "9", 142.543098, 123.037598),
        ("R34", "1", 141.800000, 126.360000),
        ("R34", "2", 141.800000, 125.340000),
        ("R35", "1", 140.800000, 120.754000),
        ("R35", "2", 140.800000, 119.734000),
        ("TP1", "TP", 144.691098, 122.300000),
        ("TP2", "TP", 144.691098, 123.800000),
        ("TP3", "TP", 144.691098, 120.800000),
        ("TP4", "TP", 144.564098, 119.100598),
    ):
        expect_pad_position(board, reference, number, x_mm, y_mm)
    if board.FindFootprintByReference("C25") is not None:
        expect_pad_position(board, "C25", "1", 145.239411, 117.839411)
        expect_pad_position(board, "C25", "2", 144.560589, 117.160589)


def same_point(left: object, right: object) -> bool:
    return left.x == right.x and left.y == right.y


def track_matches(
    item: object,
    net_code: int,
    layer: int,
    width: int,
    start: object,
    end: object,
) -> bool:
    if isinstance(item, pcbnew.PCB_VIA):
        return False
    if (
        int(item.GetNetCode()) != net_code
        or item.GetLayer() != layer
        or int(item.GetWidth()) != width
    ):
        return False
    item_start = item.GetStart()
    item_end = item.GetEnd()
    return (same_point(item_start, start) and same_point(item_end, end)) or (
        same_point(item_start, end) and same_point(item_end, start)
    )


def add_track_if_missing(
    board: object,
    net_name: str,
    width_mm: float,
    layer: int,
    start: object,
    end: object,
) -> int:
    if same_point(start, end):
        return 0
    net_info = board.GetNetInfo().NetsByName()[net_name]
    width = iu(width_mm)
    if any(
        track_matches(
            item, int(net_info.GetNetCode()), layer, width, start, end
        )
        for item in board.GetTracks()
    ):
        return 0
    track = pcbnew.PCB_TRACK(board)
    track.SetLayer(layer)
    track.SetStart(start)
    track.SetEnd(end)
    track.SetWidth(width)
    track.SetNet(net_info)
    board.Add(track)
    return 1


def add_polyline_if_missing(
    board: object,
    net_name: str,
    width_mm: float,
    layer: int,
    vertices: Sequence[object],
) -> int:
    return sum(
        add_track_if_missing(board, net_name, width_mm, layer, start, end)
        for start, end in zip(vertices, vertices[1:])
    )


def add_via_if_missing(
    board: object,
    net_name: str,
    at: object,
    diameter_mm: float,
    drill_mm: float,
) -> int:
    net_info = board.GetNetInfo().NetsByName()[net_name]
    diameter = iu(diameter_mm)
    drill = iu(drill_mm)
    for item in board.GetTracks():
        if not isinstance(item, pcbnew.PCB_VIA) or not same_point(
            item.GetPosition(), at
        ):
            continue
        if int(item.GetNetCode()) != int(net_info.GetNetCode()):
            raise RuntimeError(
                f"{net_name}: via location already occupied by {item.GetNetname()}"
            )
        if (
            int(item.GetWidth(pcbnew.F_Cu)) != diameter
            or int(item.GetDrillValue()) != drill
        ):
            raise RuntimeError(
                f"{net_name}: existing via at reviewed position has wrong size"
            )
        return 0
    via = pcbnew.PCB_VIA(board)
    via.SetPosition(at)
    via.SetWidth(diameter)
    via.SetDrill(drill)
    via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    via.SetNet(net_info)
    board.Add(via)
    return 1


def matching_track_items(
    board: object,
    net_name: str,
    width_mm: float,
    layer: int,
    start: object,
    end: object,
) -> List[object]:
    net_info = board.GetNetInfo().NetsByName()[net_name]
    width = iu(width_mm)
    return [
        item
        for item in board.GetTracks()
        if track_matches(
            item, int(net_info.GetNetCode()), layer, width, start, end
        )
    ]


def matching_via_items(
    board: object,
    net_name: str,
    at: object,
    diameter_mm: float,
    drill_mm: float,
) -> List[object]:
    net_info = board.GetNetInfo().NetsByName()[net_name]
    return [
        item
        for item in board.GetTracks()
        if isinstance(item, pcbnew.PCB_VIA)
        and same_point(item.GetPosition(), at)
        and int(item.GetNetCode()) == int(net_info.GetNetCode())
        and int(item.GetWidth(pcbnew.F_Cu)) == iu(diameter_mm)
        and int(item.GetDrillValue()) == iu(drill_mm)
    ]


def find_via_at(board: object, net_name: str, at: object) -> object:
    net_info = board.GetNetInfo().NetsByName()[net_name]
    for item in board.GetTracks():
        if (
            isinstance(item, pcbnew.PCB_VIA)
            and same_point(item.GetPosition(), at)
            and int(item.GetNetCode()) == int(net_info.GetNetCode())
        ):
            return item
    raise RuntimeError(
        f"{net_name}: missing route_power anchor via at "
        f"({pcbnew.ToMM(at.x):.3f}, {pcbnew.ToMM(at.y):.3f}) mm"
    )


def pads_connected(board: object, left: object, right: object) -> bool:
    if int(left.GetNetCode()) != int(right.GetNetCode()):
        raise RuntimeError("connectivity query uses pads on different nets")
    pads = pads_by_net(board).get(int(left.GetNetCode()), [])
    by_uuid = {route_board.point_key(item): index for index, item in enumerate(pads)}
    left_index = by_uuid[route_board.point_key(left)]
    right_index = by_uuid[route_board.point_key(right)]
    dsu = route_board.initial_connectivity(board, pads)
    return dsu.find(left_index) == dsu.find(right_index)


def run_exact_route(
    board: object,
    net_name: str,
    left: object,
    right: object,
    add_material: object,
    description: str,
) -> route_board.NetReport:
    net_info = board.GetNetInfo().NetsByName()[net_name]
    pads = pads_by_net(board).get(int(net_info.GetNetCode()), [])
    initial = component_count(board, pads)
    if pads_connected(board, left, right):
        return route_board.NetReport(
            name=net_name,
            pad_count=len(pads),
            initial_components=initial,
            remaining_components=initial,
            connections_added=0,
            tracks_added=0,
            vias_added=0,
            expanded_nodes=0,
            note=f"already connected: {description}",
        )
    previous_uuids = {route_board.point_key(item) for item in board.GetTracks()}
    tracks_added, vias_added = add_material()
    board.BuildConnectivity()
    if not pads_connected(board, left, right):
        removed = rollback_new_tracks(board, previous_uuids)
        raise RuntimeError(
            f"{net_name}: reviewed route did not connect its endpoints; "
            f"rolled back {removed} items"
        )
    remaining = component_count(board, pads)
    return route_board.NetReport(
        name=net_name,
        pad_count=len(pads),
        initial_components=initial,
        remaining_components=remaining,
        connections_added=max(0, initial - remaining),
        tracks_added=tracks_added,
        vias_added=vias_added,
        expanded_nodes=0,
        note=description,
    )


MOTOR_DIRECT_LINKS = {
    "/MOTOR_R_OUT1": ("U2", "1", "10"),
    "/MOTOR_R_OUT2": ("U2", "3", "8"),
    "/MOTOR_L_OUT1": ("U3", "1", "10"),
    "/MOTOR_L_OUT2": ("U3", "3", "8"),
}


def motor_legacy_items(board: object, net_name: str) -> List[object]:
    """Return the exact route_power dogleg items replaced for one output."""
    track_specs: Dict[str, Tuple[Tuple[float, int, object, object], ...]] = {
        "/MOTOR_R_OUT1": (
            (0.30, pcbnew.F_Cu, pad(board, "U2", "10").GetPosition(), point(143.80, 116.80)),
            (0.80, pcbnew.B_Cu, point(139.00, 116.30), point(143.80, 116.80)),
            (0.80, pcbnew.B_Cu, point(143.80, 116.80), point(146.20, 116.80)),
        ),
        "/MOTOR_R_OUT2": (
            (0.30, pcbnew.F_Cu, pad(board, "U2", "3").GetPosition(), point(140.25, 117.95)),
            (0.30, pcbnew.F_Cu, point(140.25, 117.95), point(140.25, 120.10)),
            (0.30, pcbnew.F_Cu, point(140.25, 120.10), point(139.20, 120.10)),
            (0.80, pcbnew.B_Cu, point(139.20, 120.10), point(144.00, 119.10)),
            (0.80, pcbnew.B_Cu, point(144.00, 119.10), point(144.80, 119.10)),
            (0.30, pcbnew.F_Cu, point(144.80, 119.10), point(143.80, 117.95)),
            (0.30, pcbnew.F_Cu, pad(board, "U2", "8").GetPosition(), point(143.80, 117.95)),
            (0.80, pcbnew.F_Cu, point(143.80, 117.95), pad(board, "TP4", "TP").GetPosition()),
        ),
        "/MOTOR_L_OUT1": (
            (0.30, pcbnew.F_Cu, pad(board, "U3", "1").GetPosition(), point(140.10, 122.54)),
            (0.30, pcbnew.F_Cu, point(140.10, 122.54), point(140.10, 120.90)),
            (0.30, pcbnew.F_Cu, point(140.10, 120.90), point(143.20, 120.90)),
            (0.30, pcbnew.F_Cu, point(143.20, 120.90), point(143.20, 122.54)),
            (0.30, pcbnew.F_Cu, point(143.20, 122.54), pad(board, "U3", "10").GetPosition()),
            (0.30, pcbnew.F_Cu, pad(board, "U3", "10").GetPosition(), point(143.80, 122.54)),
            (0.80, pcbnew.F_Cu, point(143.80, 122.54), pad(board, "TP1", "TP").GetPosition()),
        ),
        "/MOTOR_L_OUT2": (
            (0.30, pcbnew.F_Cu, pad(board, "U3", "8").GetPosition(), point(143.80, 123.54)),
            (0.80, pcbnew.F_Cu, point(143.80, 123.54), pad(board, "TP2", "TP").GetPosition()),
        ),
    }
    via_specs: Dict[str, Tuple[Tuple[object, float, float], ...]] = {
        "/MOTOR_R_OUT1": ((point(143.80, 116.80), 0.80, 0.40),),
        "/MOTOR_R_OUT2": (
            (point(139.20, 120.10), 0.80, 0.40),
            (point(144.80, 119.10), 0.80, 0.40),
        ),
        "/MOTOR_L_OUT1": (),
        "/MOTOR_L_OUT2": (),
    }
    found: List[object] = []
    for width, layer, start, end in track_specs[net_name]:
        matches = matching_track_items(
            board, net_name, width, layer, start, end
        )
        if len(matches) > 1:
            raise RuntimeError(f"{net_name}: duplicate legacy track anchor")
        found.extend(matches)
    for at, diameter, drill in via_specs[net_name]:
        matches = matching_via_items(
            board, net_name, at, diameter, drill
        )
        if len(matches) > 1:
            raise RuntimeError(f"{net_name}: duplicate legacy via anchor")
        found.extend(matches)
    return found


def add_motor_escape_material(board: object, net_name: str) -> Tuple[int, int]:
    tracks = 0
    vias = 0
    if net_name == "/MOTOR_R_OUT1":
        left = pad(board, "U2", "1").GetPosition()
        left_via = point(139.00, 116.30)
        tracks += add_polyline_if_missing(
            board,
            net_name,
            0.30,
            pcbnew.F_Cu,
            (
                left,
                point(140.10, 116.95),
                point(140.10, 116.60),
                point(139.00, 116.60),
                left_via,
            ),
        )
        vias += add_via_if_missing(board, net_name, left_via, 0.80, 0.40)
        tracks += add_polyline_if_missing(
            board,
            net_name,
            0.80,
            pcbnew.B_Cu,
            (
                left_via,
                point(139.00, 115.50),
                point(146.20, 115.50),
                point(146.20, 116.80),
            ),
        )
    elif net_name == "/MOTOR_R_OUT2":
        tracks += add_polyline_if_missing(
            board,
            net_name,
            0.30,
            pcbnew.F_Cu,
            (
                pad(board, "U2", "8").GetPosition(),
                point(143.15, 118.549598),
                pad(board, "TP4", "TP").GetPosition(),
            ),
        )
    elif net_name == "/MOTOR_L_OUT1":
        tracks += add_polyline_if_missing(
            board,
            net_name,
            0.30,
            pcbnew.F_Cu,
            (
                pad(board, "U3", "10").GetPosition(),
                point(143.15, 121.937598),
                pad(board, "TP1", "TP").GetPosition(),
            ),
        )
    elif net_name == "/MOTOR_L_OUT2":
        tracks += add_polyline_if_missing(
            board,
            net_name,
            0.30,
            pcbnew.F_Cu,
            (
                pad(board, "U3", "8").GetPosition(),
                point(143.15, 124.137598),
                pad(board, "TP2", "TP").GetPosition(),
            ),
        )
    return tracks, vias


def simplify_motor_output(
    board: object, net_name: str
) -> route_board.NetReport:
    reference, left_number, right_number = MOTOR_DIRECT_LINKS[net_name]
    left = pad(board, reference, left_number)
    right = pad(board, reference, right_number)
    net_info = board.GetNetInfo().NetsByName()[net_name]
    pads = pads_by_net(board).get(int(net_info.GetNetCode()), [])
    initial = component_count(board, pads)
    legacy = motor_legacy_items(board, net_name)

    tracks_added = add_track_if_missing(
        board,
        net_name,
        0.30,
        pcbnew.F_Cu,
        left.GetPosition(),
        right.GetPosition(),
    )
    escape_tracks, vias_added = add_motor_escape_material(board, net_name)
    tracks_added += escape_tracks
    removed_vias = sum(isinstance(item, pcbnew.PCB_VIA) for item in legacy)
    for item in legacy:
        # This is an intentional permanent replacement, so let KiCad delete
        # the detached C++ item.  BOARD.Remove() transfers ownership to the
        # SWIG wrapper and is unsafe when several legacy items are discarded
        # before subsequent pcbnew calls on macOS.
        board.Delete(item)
    board.BuildConnectivity()
    remaining = component_count(board, pads)
    if remaining > initial or not pads_connected(board, left, right):
        raise RuntimeError(
            f"{net_name}: direct duplicate-output replacement broke connectivity"
        )
    return route_board.NetReport(
        name=net_name,
        pad_count=len(pads),
        initial_components=initial,
        remaining_components=remaining,
        connections_added=max(0, initial - remaining),
        tracks_added=tracks_added,
        vias_added=vias_added,
        expanded_nodes=0,
        note=(
            f"direct 0.30 mm under-{reference} link plus reviewed fanout; "
            f"removed {len(legacy) - removed_vias} tracks/{removed_vias} vias"
        ),
    )


def route_vcap(board: object) -> route_board.NetReport:
    name = "Net-(U5B-VCAP_1)"
    left = pad(board, "U5", "22")
    right = pad(board, "C_MCU-C0", "1")

    def add_material() -> Tuple[int, int]:
        return (
            add_track_if_missing(
                board,
                name,
                0.16,
                pcbnew.F_Cu,
                left.GetPosition(),
                right.GetPosition(),
            ),
            0,
        )

    return run_exact_route(
        board, name, left, right, add_material, "reviewed 0.16 mm VCAP lead"
    )


def route_power_switch_return(board: object) -> route_board.NetReport:
    name = "/PWR_SWITCH_RETURN"
    left = pad(board, "R33", "2")
    right = pad(board, "POWER0", "2")

    def add_material() -> Tuple[int, int]:
        first_via = point(136.50, 111.50)
        second_via = point(148.40, 134.00)
        tracks = add_polyline_if_missing(
            board,
            name,
            0.16,
            pcbnew.F_Cu,
            (left.GetPosition(), point(132.96, 112.60), point(132.96, 111.50), first_via),
        )
        vias = add_via_if_missing(board, name, first_via, 0.60, 0.30)
        tracks += add_polyline_if_missing(
            board,
            name,
            0.20,
            pcbnew.B_Cu,
            (
                first_via,
                point(154.80, 111.50),
                point(158.00, 115.50),
                point(158.00, 131.80),
                second_via,
            ),
        )
        vias += add_via_if_missing(board, name, second_via, 0.60, 0.30)
        tracks += add_track_if_missing(
            board,
            name,
            0.20,
            pcbnew.F_Cu,
            second_via,
            right.GetPosition(),
        )
        return tracks, vias

    return run_exact_route(
        board,
        name,
        left,
        right,
        add_material,
        "reviewed low-current switch-return dogleg",
    )


def route_vbat_monitor(board: object) -> route_board.NetReport:
    name = "VBAT_SW"
    left = pad(board, "R4", "2")
    right = pad(board, "C8", "2")
    net_info = board.GetNetInfo().NetsByName()[name]
    pads = pads_by_net(board).get(int(net_info.GetNetCode()), [])
    initial = component_count(board, pads)
    previous_uuids = {route_board.point_key(item) for item in board.GetTracks()}
    tracks = 0
    vias = 0
    try:
        # Every listed anchor is an existing route_power layer transition.
        # The Q2 transition receives three additional 1.00/0.40 mm vias (four
        # total), while the five branch transitions receive one additional
        # via each (two total).  Overlapping copper on both layers prevents a
        # narrow single-via neck from remaining between nominally parallel
        # vias.  These coordinates were jointly checked with filled-zone DRC.
        q2_anchor = point(134.40, 117.80)
        for anchor in (
            q2_anchor,
            point(135.40, 120.15),
            point(135.60, 124.638),
            point(138.30, 117.45),
            point(146.00, 124.50),
            point(154.50, 124.00),
        ):
            find_via_at(board, name, anchor)

        q2_new = (
            point(134.40, 118.80),
            point(135.40, 117.80),
            point(135.40, 118.80),
        )
        for at in q2_new:
            vias += add_via_if_missing(board, name, at, 1.00, 0.40)
        q2_edges = (
            (point(134.40, 117.80), point(135.40, 117.80)),
            (point(134.40, 118.80), point(135.40, 118.80)),
            (point(134.40, 117.80), point(134.40, 118.80)),
            (point(135.40, 117.80), point(135.40, 118.80)),
        )
        for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
            for start, end in q2_edges:
                tracks += add_track_if_missing(
                    board, name, 1.00, layer, start, end
                )
        # A second F.Cu feed starts at the already tied Q2 drain rail instead
        # of forcing all aggregate load current through the original dogleg.
        tracks += add_track_if_missing(
            board,
            name,
            0.60,
            pcbnew.F_Cu,
            point(134.60, 116.64),
            point(135.40, 117.80),
        )

        branch_banks = (
            (point(135.40, 120.15), point(134.40, 120.15)),
            (point(135.60, 124.638), point(134.60, 124.638)),
            # C8.2 is the only DRC-clean second transition in this congested
            # branch.  It is intentionally a bottom-tented via-in-pad.
            (point(138.30, 117.45), right.GetPosition()),
            (point(146.00, 124.50), point(146.00, 123.50)),
            (point(154.50, 124.00), point(154.50, 123.00)),
        )
        for anchor, at in branch_banks:
            vias += add_via_if_missing(board, name, at, 1.00, 0.40)
            for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
                tracks += add_track_if_missing(
                    board, name, 0.80, layer, anchor, at
                )

        if not pads_connected(board, left, right):
            tracks += add_polyline_if_missing(
                board,
                name,
                0.30,
                pcbnew.F_Cu,
                (
                    left.GetPosition(),
                    point(137.80, 114.90),
                    point(137.80, 117.45),
                    point(138.30, 117.45),
                ),
            )

        # Keep the remote fan supply and return close together until the
        # return changes layers at y=97.6 mm.  The return-side geometry is in
        # route_fan_return(); this forward path remains entirely on F.Cu.
        tracks += add_polyline_if_missing(
            board,
            name,
            0.80,
            pcbnew.F_Cu,
            (
                pad(board, "J2", "2").GetPosition(),
                point(139.60, 85.00),
                point(139.60, 98.90),
                point(136.20, 98.90),
                point(136.20, 108.50),
                point(136.50, 110.00),
                point(136.50, 114.36),
                point(134.60, 114.36),
            ),
        )
        board.BuildConnectivity()
        if component_count(board, pads) != 1:
            raise RuntimeError("reviewed VBAT_SW branches did not fully connect")
    except Exception:
        rollback_new_tracks(board, previous_uuids)
        raise

    remaining = component_count(board, pads)
    return route_board.NetReport(
        name=name,
        pad_count=len(pads),
        initial_components=initial,
        remaining_components=remaining,
        connections_added=max(0, initial - remaining),
        tracks_added=tracks,
        vias_added=vias,
        expanded_nodes=0,
        note=(
            "reviewed monitor/J2 branches plus 4-via Q2 and 2-via branch "
            "banks; C8.2 via-in-pad requires bottom tenting"
        ),
    )


def route_fan_return(board: object) -> route_board.NetReport:
    name = "/FAN_NEG_INTERNAL"
    left = pad(board, "J2", "1")
    right = pad(board, "D4", "1")

    def add_material() -> Tuple[int, int]:
        first_left = point(135.80, 97.60)
        first_right = point(136.90, 97.60)
        # The local transition lands on the already-routed F.Cu fan rail.
        # Keeping it at y=122 avoids the added VBAT_SW banks at y=123..124.5.
        second_right = point(162.00, 122.00)
        tracks = add_polyline_if_missing(
            board,
            name,
            0.80,
            pcbnew.F_Cu,
            (
                left.GetPosition(),
                point(138.20, 86.20),
                point(138.20, 96.30),
                first_right,
                first_left,
            ),
        )
        vias = 0
        for at in (first_left, first_right, second_right):
            vias += add_via_if_missing(board, name, at, 0.80, 0.40)
        tracks += add_polyline_if_missing(
            board,
            name,
            0.80,
            pcbnew.B_Cu,
            (
                first_left,
                point(136.35, 98.70),
                point(139.00, 100.00),
                point(141.00, 103.00),
                point(141.00, 111.50),
                point(145.00, 113.50),
                point(147.60, 114.50),
                point(147.60, 120.00),
                point(158.50, 120.00),
                point(160.40, 121.00),
                second_right,
            ),
        )
        tracks += add_track_if_missing(
            board,
            name,
            0.80,
            pcbnew.B_Cu,
            first_right,
            point(136.35, 98.70),
        )
        return tracks, vias

    return run_exact_route(
        board,
        name,
        left,
        right,
        add_material,
        "reviewed 0.80 mm paired fan return; two vias at J2 and one "
        "off-pad via at the switching stage",
    )


def route_r0_logic_ground(board: object) -> route_board.NetReport:
    name = "GND"
    stitch_pad = pad(board, "R0", "2")
    net_info = board.GetNetInfo().NetsByName()[name]
    pads = pads_by_net(board).get(int(net_info.GetNetCode()), [])
    initial = component_count(board, pads)
    vias = add_via_if_missing(
        board, name, stitch_pad.GetPosition(), 0.60, 0.30
    )
    if board.FindFootprintByReference("C25") is not None:
        # C25.2 sits in a small F.Cu logic-GND island beside the motor power
        # pour.  This reviewed point is also inside the B.Cu logic-GND zone,
        # joining the island without putting a via in the 0402 pad itself.
        vias += add_via_if_missing(
            board, name, point(144.00, 116.50), 0.60, 0.30
        )
    # Reviewed spare stitch in the logic region.  It is already useful on the
    # protected core and prevents the audited signal subset from separating
    # the large front/back logic-GND pours later in the workflow.
    vias += add_via_if_missing(
        board, name, point(140.00, 109.00), 0.60, 0.30
    )
    board.BuildConnectivity()
    remaining = component_count(board, pads)
    return route_board.NetReport(
        name=name,
        pad_count=len(pads),
        initial_components=initial,
        remaining_components=remaining,
        connections_added=max(0, initial - remaining),
        tracks_added=0,
        vias_added=vias,
        expanded_nodes=0,
        note=(
            "reviewed R0.2 via-in-pad plus C25/main logic-GND F/B stitches; "
            "tent R0.2 from bottom"
        ),
    )


def route_motor_sr(board: object, net_name: str) -> route_board.NetReport:
    """Connect each MP6551 SR pin to its new 220 kOhm pull-down."""
    anchors = {
        "/MOTOR_R_SR": (("U2", "6"), ("R35", "2")),
        "/MOTOR_L_SR": (("U3", "6"), ("R34", "2")),
    }
    (left_ref, left_number), (right_ref, right_number) = anchors[net_name]
    left = pad(board, left_ref, left_number)
    right = pad(board, right_ref, right_number)

    def add_material() -> Tuple[int, int]:
        if net_name == "/MOTOR_R_SR":
            escape = point(141.893160, 119.45)
            tracks = add_track_if_missing(
                board,
                net_name,
                0.16,
                pcbnew.F_Cu,
                left.GetPosition(),
                escape,
            )
            tracks += add_polyline_if_missing(
                board,
                net_name,
                0.20,
                pcbnew.F_Cu,
                (escape, point(141.50, 119.60), right.GetPosition()),
            )
            return tracks, 0
        return (
            add_track_if_missing(
                board,
                net_name,
                0.20,
                pcbnew.F_Cu,
                left.GetPosition(),
                right.GetPosition(),
            ),
            0,
        )

    return run_exact_route(
        board,
        net_name,
        left,
        right,
        add_material,
        "reviewed MP6551 SR lead; U2 uses a 0.16 mm fine-pitch escape",
    )


POWER_GND2_OUTLINE: Tuple[Tuple[float, float], ...] = (
    (128.00, 116.30),
    (142.75, 116.30),
    (142.75, 121.85),
    (144.25, 121.85),
    (144.25, 124.25),
    (142.75, 124.25),
    (142.75, 125.55),
    (146.30, 125.55),
    (146.30, 124.70),
    (153.00, 124.70),
    (153.00, 120.40),
    (162.50, 120.40),
    (162.50, 136.20),
    (134.80, 136.20),
    (134.80, 123.00),
    (128.00, 123.00),
)


def reviewed_polygon(vertices: Sequence[Tuple[float, float]]) -> object:
    shape = pcbnew.SHAPE_POLY_SET()
    outline = shape.NewOutline()
    for x_mm, y_mm in vertices:
        shape.Append(iu(x_mm), iu(y_mm), outline)
    return shape


def set_power_ground_outline(board: object) -> None:
    """Extend both GND2 pours only far enough to receive U3 pad 9."""
    replacement = reviewed_polygon(POWER_GND2_OUTLINE)
    zones = {
        str(zone.GetZoneName()): zone
        for zone in board.Zones()
        if str(zone.GetZoneName()).startswith("POWER_GND2_")
    }
    for name in ("POWER_GND2_F", "POWER_GND2_B"):
        zone = zones[name]
        zone.Outline().RemoveAllContours()
        zone.Outline().Append(replacement)


def route_power_ground_returns(board: object) -> route_board.NetReport:
    """Add reviewed MP6551 pad-9 returns and SR-resistor GND2 stubs."""
    name = "GND2"
    net_info = board.GetNetInfo().NetsByName()[name]
    pads = pads_by_net(board).get(int(net_info.GetNetCode()), [])
    initial = component_count(board, pads)
    previous_uuids = {route_board.point_key(item) for item in board.GetTracks()}
    tracks = 0
    vias = 0
    try:
        # U2 uses an explicit B.Cu return to the GND2 side of R0.  This keeps
        # the return deterministic even when the narrow F.Cu pour is split by
        # the driver fanout.  HighCurrent rules require 0.80/0.40 mm vias.
        u2_via = point(143.60, 117.449598)
        r0_via = pad(board, "R0", "1").GetPosition()
        tracks += add_polyline_if_missing(
            board,
            name,
            0.30,
            pcbnew.F_Cu,
            (
                pad(board, "U2", "9").GetPosition(),
                u2_via,
            ),
        )
        vias += add_via_if_missing(board, name, u2_via, 0.80, 0.40)
        vias += add_via_if_missing(board, name, r0_via, 0.80, 0.40)
        tracks += add_polyline_if_missing(
            board,
            name,
            0.80,
            pcbnew.B_Cu,
            (u2_via, point(142.30, 119.20), r0_via),
        )

        # U3 returns through the reviewed local protrusion added to both GND2
        # pours.  The via ties the small F.Cu reception area to B.Cu/main GND2.
        u3_via = point(143.65, 123.037598)
        tracks += add_track_if_missing(
            board,
            name,
            0.30,
            pcbnew.F_Cu,
            pad(board, "U3", "9").GetPosition(),
            u3_via,
        )
        vias += add_via_if_missing(board, name, u3_via, 0.80, 0.40)

        # GND2 is HighCurrent, so these short SR ground stubs use the project
        # rule minimum 0.30 mm rather than the 0.20 mm signal-lead width.
        tracks += add_track_if_missing(
            board,
            name,
            0.30,
            pcbnew.F_Cu,
            pad(board, "R35", "1").GetPosition(),
            point(139.80, 120.754),
        )
        tracks += add_track_if_missing(
            board,
            name,
            0.30,
            pcbnew.F_Cu,
            pad(board, "R34", "1").GetPosition(),
            point(140.80, 126.360),
        )

        # The motor escapes split the front pour from the broad back pour.
        # This reviewed free-space stitch is inside the main filled polygon
        # on both layers and makes all GND2 pad islands one component.
        vias += add_via_if_missing(
            board, name, point(140.00, 130.00), 0.80, 0.40
        )
        set_power_ground_outline(board)
    except Exception:
        rollback_new_tracks(board, previous_uuids)
        raise

    # The free-space F/B stitch must see the updated power-zone outline before
    # connectivity is rebuilt; otherwise KiCad may propagate the stale logic-
    # GND fill onto the floating via during the intermediate connectivity pass.
    if not pcbnew.ZONE_FILLER(board).Fill(board.Zones()):
        rollback_new_tracks(board, previous_uuids)
        raise RuntimeError("GND2 local zone refill failed")
    board.BuildConnectivity()
    remaining = component_count(board, pads)
    return route_board.NetReport(
        name=name,
        pad_count=len(pads),
        initial_components=initial,
        remaining_components=remaining,
        connections_added=max(0, initial - remaining),
        tracks_added=tracks,
        vias_added=vias,
        expanded_nodes=0,
        note=(
            "reviewed U2 explicit B.Cu return, U3 dual-pour return, SR "
            "GND2 stubs, and F/B main-pour stitch; filled-zone DRC is "
            "mandatory"
        ),
    )


EXACT_ROUTERS = {
    **{
        name: (lambda board, net_name=name: simplify_motor_output(board, net_name))
        for name in MOTOR_DIRECT_LINKS
    },
    "/MOTOR_R_SR": lambda board: route_motor_sr(board, "/MOTOR_R_SR"),
    "/MOTOR_L_SR": lambda board: route_motor_sr(board, "/MOTOR_L_SR"),
    "GND2": route_power_ground_returns,
    "Net-(U5B-VCAP_1)": route_vcap,
    "VBAT_SW": route_vbat_monitor,
    "/FAN_NEG_INTERNAL": route_fan_return,
    "GND": route_r0_logic_ground,
}


def config_for(spec: NetSpec, netclass: str, verbose: bool) -> object:
    # HighCurrent-to-Sensitive is symmetric.  route_board is signal-only, so
    # for this helper we expand Sensitive obstacles to 0.40 mm when the route
    # itself is HighCurrent.
    sensitive_clearance = 0.40 if netclass == "HighCurrent" else 0.25
    return route_board.RouterConfig(
        grid_mm=0.10,
        trace_width_mm=spec.width_mm,
        clearance_mm=0.20,
        sensitive_clearance_mm=sensitive_clearance,
        highcurrent_sensitive_clearance_mm=0.40,
        switch_clearance_mm=0.50,
        escape_clearance_mm=0.16,
        pad_escape_mm=0.60,
        via_diameter_mm=spec.via_diameter_mm,
        via_drill_mm=spec.via_drill_mm,
        hole_clearance_mm=0.25,
        hole_to_hole_mm=0.20,
        edge_clearance_mm=0.50,
        via_cost_mm=3.0,
        bend_cost_mm=0.30,
        max_vias=spec.max_vias,
        max_expanded=spec.max_expanded,
        max_net_expanded=spec.max_net_expanded,
        candidate_pairs=spec.candidate_pairs,
        front_axis="vertical",
        net_order="fewest-pads",
        fanout_references=(),
        verbose=verbose,
    )


def component_count(board: object, pads: Sequence[object]) -> int:
    return route_board.initial_connectivity(board, pads).component_count()


def net_component_count(board: object, name: str) -> int:
    nets = board.GetNetInfo().NetsByName()
    if name not in nets:
        raise RuntimeError(f"missing net: {name}")
    net_info = nets[name]
    pads = pads_by_net(board).get(int(net_info.GetNetCode()), [])
    return component_count(board, pads) if pads else 0


def verify_split_ground(board: object) -> None:
    zones_by_name = {str(zone.GetZoneName()): zone for zone in board.Zones()}
    missing = sorted(EXPECTED_ZONE_NAMES - set(zones_by_name))
    if missing:
        raise RuntimeError("route_power ground zones are missing: " + ", ".join(missing))
    for name, (expected_net, expected_layer) in EXPECTED_ZONES.items():
        zone = zones_by_name[name]
        if str(zone.GetNetname()) != expected_net or zone.GetLayer() != expected_layer:
            raise RuntimeError(
                f"{name}: expected {expected_net} on "
                f"{board.GetLayerName(expected_layer)}, found "
                f"{zone.GetNetname()} on {board.GetLayerName(zone.GetLayer())}"
            )
    bridge = board.FindFootprintByReference("R0")
    if bridge is None:
        raise RuntimeError("missing GND/GND2 bridge R0")
    expected_pads = {"1": "GND2", "2": "GND"}
    for number, expected_net in expected_pads.items():
        pad = bridge.FindPadByNumber(number)
        if pad is None or str(pad.GetNetname()) != expected_net:
            found = "missing" if pad is None else str(pad.GetNetname())
            raise RuntimeError(
                f"R0.{number}: expected {expected_net}, found {found}"
            )


def rollback_new_tracks(board: object, previous_uuids: set[str]) -> int:
    added = [
        item
        for item in board.GetTracks()
        if route_board.point_key(item) not in previous_uuids
    ]
    for item in added:
        board.Remove(item)
    if added:
        board.BuildConnectivity()
    return len(added)


class ProtectedOccupancyGrid(route_board.OccupancyGrid):
    """Allow a reviewed 0.16 mm exit halo only for an explicit local spec."""

    def __init__(self, board: object, config: object, local_escape: bool) -> None:
        self.local_escape = local_escape
        super().__init__(board, config)

    def own_pad_masks(
        self, pads: Sequence[object]
    ) -> Tuple[object, object]:
        pad_mask, escape_mask = super().own_pad_masks(pads)
        if not self.local_escape:
            return pad_mask, escape_mask
        for pad in pads:
            for layer_index, layer_id in enumerate(route_board.LAYER_IDS):
                if pad.FlashLayer(layer_id):
                    self._raster_shape_bool(
                        pad.GetEffectiveShape(layer_id),
                        layer_index,
                        self.config.pad_escape_mm,
                        escape_mask,
                    )
        return pad_mask, escape_mask


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Finish reviewed protected nets after route_power.py",
        epilog=(
            "Run only on a whole-project copy. GND2 is limited to the fixed "
            "MP6551/SR returns; /MPM_SW_INTERNAL is never routed. Refill "
            "zones and compare KiCad DRC afterwards."
        ),
    )
    parser.add_argument("--input", "-i", required=True, type=Path)
    parser.add_argument("--output", "-o", type=Path)
    parser.add_argument(
        "--in-place",
        action="store_true",
        help="permit atomic replacement when --output equals --input",
    )
    parser.add_argument(
        "--overwrite-output",
        action="store_true",
        help="permit replacement of an existing output distinct from input",
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--net",
        action="append",
        default=[],
        help="route only this approved exact net; repeat as needed",
    )
    parser.add_argument("--report-json", type=Path)
    parser.add_argument("--overwrite-report", action="store_true")
    parser.add_argument("--verbose", "-v", action="store_true")
    return parser.parse_args(argv)


def validate_args(args: argparse.Namespace) -> None:
    args.input = args.input.expanduser().resolve()
    if not args.input.is_file():
        raise SystemExit(f"input board does not exist: {args.input}")
    if not args.dry_run and args.output is None:
        raise SystemExit("--output is required unless --dry-run is used")
    if args.output is not None:
        args.output = args.output.expanduser().resolve()
        if args.output == args.input and not args.in_place:
            raise SystemExit("refusing input replacement without --in-place")
        if args.in_place and args.output != args.input:
            raise SystemExit("--in-place requires --output to equal --input")
        if (
            args.output != args.input
            and args.output.exists()
            and not args.overwrite_output
        ):
            raise SystemExit(
                "refusing existing --output without --overwrite-output"
            )
    elif args.in_place:
        raise SystemExit("--in-place requires --output")

    approved = {spec.name for spec in NET_SPECS} | set(EXACT_ROUTE_NAMES)
    unknown = sorted(set(args.net) - approved)
    if unknown:
        raise SystemExit("nets are not approved by this helper: " + ", ".join(unknown))

    if args.report_json is not None:
        args.report_json = args.report_json.expanduser().resolve()
        occupied = {args.input}
        if args.output is not None:
            occupied.add(args.output)
        if args.report_json in occupied:
            raise SystemExit("--report-json must differ from board paths")
        if args.report_json.exists() and not args.overwrite_report:
            raise SystemExit(
                "refusing existing report without --overwrite-report"
            )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    validate_args(args)
    board = pcbnew.LoadBoard(str(args.input))
    if board is None:
        raise SystemExit(f"KiCad failed to load {args.input}")
    if board.GetCopperLayerCount() != 2:
        raise SystemExit("this helper supports exactly two copper layers")
    verify_split_ground(board)
    verify_exact_route_anchors(board)

    by_name = board.GetNetInfo().NetsByName()
    selected = set(args.net)
    specs = [spec for spec in NET_SPECS if not selected or spec.name in selected]
    exact_names = [
        name for name in EXACT_ROUTE_NAMES if not selected or name in selected
    ]
    missing_nets = [
        name
        for name in [*(spec.name for spec in specs), *exact_names]
        if name not in by_name
    ]
    if missing_nets:
        raise SystemExit("board nets are missing: " + ", ".join(missing_nets))

    # Compare connectivity under identical filled-zone conditions.
    if not pcbnew.ZONE_FILLER(board).Fill(board.Zones()):
        raise RuntimeError("KiCad pre-route zone refill failed")
    before = count_unconnected(board)
    ground_before = {
        name: net_component_count(board, name) for name in ("GND", "GND2")
    }
    reports = []
    for name in exact_names:
        print(f"{name}: reviewed fixed local route", flush=True)
        reports.append(EXACT_ROUTERS[name](board))

    for spec in specs:
        net_info = by_name[spec.name]
        pads = pads_by_net(board).get(int(net_info.GetNetCode()), [])
        if len(pads) < 2:
            continue
        netclass = route_board.effective_netclass(net_info)
        config = config_for(spec, netclass, args.verbose)
        occupancy = ProtectedOccupancyGrid(
            board, config, local_escape=spec.local_escape
        )
        router = route_board.AStarRouter(occupancy, config)
        print(
            f"{spec.name}: {spec.width_mm:.2f} mm, "
            f"via {spec.via_diameter_mm:.2f}/{spec.via_drill_mm:.2f} mm, "
            f"class {netclass}",
            flush=True,
        )
        previous_uuids = {
            route_board.point_key(item) for item in board.GetTracks()
        }
        report = route_board.route_net(
            board, occupancy, router, net_info, pads, config
        )
        if report.remaining_components > 1:
            removed = rollback_new_tracks(board, previous_uuids)
            report.connections_added = 0
            report.tracks_added = 0
            report.vias_added = 0
            report.remaining_components = report.initial_components
            report.note = (
                f"rolled back {removed} partial track/via items; " + report.note
            )
        reports.append(report)

    if not pcbnew.ZONE_FILLER(board).Fill(board.Zones()):
        raise RuntimeError("KiCad zone refill failed")
    after = count_unconnected(board)
    ground_after = {
        name: net_component_count(board, name) for name in ("GND", "GND2")
    }
    degraded_ground = [
        name
        for name in ("GND", "GND2")
        if ground_after[name] > ground_before[name]
    ]
    if degraded_ground:
        details = ", ".join(
            f"{name} {ground_before[name]}->{ground_after[name]}"
            for name in degraded_ground
        )
        raise RuntimeError(
            "refusing output because split-ground connectivity degraded: "
            + details
        )

    remaining = []
    current_pads = pads_by_net(board)
    evaluated_names = (
        {spec.name for spec in specs} | set(exact_names)
        if selected
        else {spec.name for spec in NET_SPECS}
        | set(EXACT_ROUTE_NAMES)
        | set(MANUAL_NETS)
    )
    for name in sorted(evaluated_names):
        if name not in by_name:
            continue
        net_info = by_name[name]
        pads = current_pads.get(int(net_info.GetNetCode()), [])
        if len(pads) >= 2:
            components = component_count(board, pads)
            if components > 1:
                remaining.append((name, components))

    if not args.dry_run:
        assert args.output is not None
        route_board.save_board_atomic(board, args.output)

    connections = sum(item.connections_added for item in reports)
    tracks = sum(item.tracks_added for item in reports)
    vias = sum(item.vias_added for item in reports)
    print(
        f"protected finish: {connections} connections, {tracks} tracks, "
        f"{vias} vias; unconnected {before} -> {after}",
        flush=True,
    )
    if remaining:
        print("remaining approved nets:")
        for name, components in remaining:
            print(f"  {name}: {components} disconnected groups")
    else:
        print("remaining approved nets: none")
    if args.dry_run:
        print("Dry run: no board file was written")
    else:
        print(f"Wrote {args.output}")
    print("MANDATORY: refill zones, inspect both layers, and compare KiCad DRC.")

    if args.report_json is not None:
        payload = {
            "input": str(args.input),
            "output": None if args.dry_run else str(args.output),
            "dry_run": bool(args.dry_run),
            "unconnected_before": before,
            "unconnected_after": after,
            "ground_components_before": ground_before,
            "ground_components_after": ground_after,
            "remaining": [
                {"name": name, "components": components}
                for name, components in remaining
            ],
            "nets": [dataclasses.asdict(item) for item in reports],
        }
        route_board.save_text_atomic(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
            args.report_json,
        )
    return 0 if not remaining else 2


if __name__ == "__main__":
    raise SystemExit(main())
