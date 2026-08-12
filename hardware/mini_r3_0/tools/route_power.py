#!/usr/bin/env python3
"""Route the reviewed mini_r3 power loops and add split ground pours.

This script is deliberately board-specific.  It runs after ``prepare_board.py``
and before ``route_board.py``.  To keep it deterministic and auditable it:

* refuses any input that already contains tracks, vias, or zones;
* never moves or edits a footprint;
* uses named footprint pads as route endpoints;
* routes only the local/high-current nets listed in ``ROUTED_NETS``; and
* adds full-board GND plus a concave, higher-priority GND2 power region on
  both copper layers.

The remote fan connector and rear power-switch return are intentionally left
for a later reviewed routing pass.  Their long paths depend on the final
signal escape choices and are safer to review in context than to hard-code
here.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable, Sequence

# KiCad's zone filler needs a live wx application even in a headless script.
# Creating it before importing pcbnew also avoids the stdpbase traits assert on
# macOS KiCad 10's bundled Python.
import wx

_WX_APP = wx.GetApp() or wx.App(False)

import pcbnew


FRONT = pcbnew.F_Cu
BACK = pcbnew.B_Cu

# These are the only nets for which this script adds tracks or vias.  GND and
# GND2 are also connected by the zones declared below.
ROUTED_NETS = {
    "VBAT_RAW",
    "VBAT_SW",
    "+5V",
    "+3V3",
    "/MOTOR_R_OUT1",
    "/MOTOR_R_OUT2",
    "/MOTOR_L_OUT1",
    "/MOTOR_L_OUT2",
    "/MPM_FB_INTERNAL",
    "/MPM_EN_INTERNAL",
    "/FAN_NEG_INTERNAL",
    "/PWR_GATE_INTERNAL",
    "GND2",
}


def iu(value_mm: float) -> int:
    return int(pcbnew.FromMM(value_mm))


def point(x_mm: float, y_mm: float) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I(iu(x_mm), iu(y_mm))


def pad(board: pcbnew.BOARD, reference: str, number: str) -> object:
    footprint = board.FindFootprintByReference(reference)
    if footprint is None:
        raise RuntimeError(f"missing footprint: {reference}")
    result = footprint.FindPadByNumber(str(number))
    if result is None:
        raise RuntimeError(f"missing pad: {reference}.{number}")
    return result


def pads_on_net(board: pcbnew.BOARD, reference: str, net_name: str) -> list[object]:
    footprint = board.FindFootprintByReference(reference)
    if footprint is None:
        raise RuntimeError(f"missing footprint: {reference}")
    result = [item for item in footprint.Pads() if item.GetNetname() == net_name]
    if not result:
        raise RuntimeError(f"{reference} has no pad on {net_name}")
    return sorted(
        result,
        key=lambda item: (item.GetPosition().y, item.GetPosition().x, item.GetNumber()),
    )


def position(item: object) -> pcbnew.VECTOR2I:
    return item.GetPosition()


def expect_pad_position(
    board: pcbnew.BOARD,
    reference: str,
    number: str,
    x_mm: float,
    y_mm: float,
    tolerance_mm: float = 0.01,
) -> None:
    """Refuse stale placement inputs before adding coordinate-based routes."""
    actual = position(pad(board, reference, number))
    if (
        abs(pcbnew.ToMM(actual.x) - x_mm) > tolerance_mm
        or abs(pcbnew.ToMM(actual.y) - y_mm) > tolerance_mm
    ):
        raise RuntimeError(
            f"stale placement: expected {reference}.{number} at "
            f"({x_mm:.3f}, {y_mm:.3f}) mm, found "
            f"({pcbnew.ToMM(actual.x):.3f}, {pcbnew.ToMM(actual.y):.3f}) mm"
        )


def verify_reviewed_placement(board: pcbnew.BOARD) -> None:
    """Check the anchors that define the split and the dense power routes."""
    d5 = board.FindFootprintByReference("D5")
    if d5 is None or d5.GetFPIDAsString() != "Diode_SMD:D_SOD-323":
        found = "missing" if d5 is None else d5.GetFPIDAsString()
        raise RuntimeError(f"expected reviewed D5 SOD-323 footprint, found {found}")
    for reference, number, x_mm, y_mm in (
        ("D5", "1", 129.950, 114.450),
        ("D5", "2", 129.950, 116.550),
        ("Q2", "3", 131.750, 114.3625),
        ("Q2", "4", 131.750, 116.6375),
        ("R0", "1", 142.300, 120.200),
        ("R0", "2", 143.300, 120.200),
        ("U7", "16", 145.750, 127.750),
    ):
        expect_pad_position(board, reference, number, x_mm, y_mm)


def net(board: pcbnew.BOARD, name: str) -> object:
    nets = board.GetNetInfo().NetsByName()
    if name not in nets:
        raise RuntimeError(f"missing net: {name}")
    return nets[name]


def add_track(
    board: pcbnew.BOARD,
    net_name: str,
    width_mm: float,
    layer: int,
    vertices: Sequence[pcbnew.VECTOR2I],
) -> int:
    if len(vertices) < 2:
        raise ValueError("a track needs at least two vertices")
    net_info = net(board, net_name)
    count = 0
    for start, end in zip(vertices, vertices[1:]):
        if start.x == end.x and start.y == end.y:
            continue
        track = pcbnew.PCB_TRACK(board)
        track.SetLayer(layer)
        track.SetStart(start)
        track.SetEnd(end)
        track.SetWidth(iu(width_mm))
        track.SetNet(net_info)
        board.Add(track)
        count += 1
    return count


def add_via(
    board: pcbnew.BOARD,
    net_name: str,
    at: pcbnew.VECTOR2I,
    diameter_mm: float = 0.80,
    drill_mm: float = 0.40,
) -> object:
    via = pcbnew.PCB_VIA(board)
    via.SetPosition(at)
    via.SetWidth(iu(diameter_mm))
    via.SetDrill(iu(drill_mm))
    via.SetLayerPair(FRONT, BACK)
    via.SetNet(net(board, net_name))
    board.Add(via)
    return via


def add_route(
    board: pcbnew.BOARD,
    net_name: str,
    width_mm: float,
    vertices: Sequence[pcbnew.VECTOR2I],
    layer: int = FRONT,
) -> int:
    """Add one same-layer polyline and return its segment count."""
    return add_track(board, net_name, width_mm, layer, vertices)


def connect_with_vias(
    board: pcbnew.BOARD,
    net_name: str,
    width_mm: float,
    start: pcbnew.VECTOR2I,
    first_via: pcbnew.VECTOR2I,
    back_vertices: Sequence[pcbnew.VECTOR2I],
    second_via: pcbnew.VECTOR2I,
    end: pcbnew.VECTOR2I,
    escape_width_mm: float | None = None,
) -> int:
    """Route F.Cu -> B.Cu -> F.Cu with standard 0.80/0.40 mm vias."""
    escape = width_mm if escape_width_mm is None else escape_width_mm
    count = add_route(board, net_name, escape, (start, first_via), FRONT)
    add_via(board, net_name, first_via)
    count += add_route(
        board,
        net_name,
        width_mm,
        (first_via, *back_vertices, second_via),
        BACK,
    )
    add_via(board, net_name, second_via)
    count += add_route(board, net_name, escape, (second_via, end), FRONT)
    return count


def add_zone(
    board: pcbnew.BOARD,
    net_name: str,
    layer: int,
    outline: pcbnew.SHAPE_POLY_SET,
    priority: int,
    zone_name: str,
) -> None:
    zone = pcbnew.ZONE(board)
    zone.SetLayer(layer)
    zone.SetNet(net(board, net_name))
    # Append copies the polygon into zone-owned storage.  ``SetOutline`` keeps
    # a SWIG-owned object reference and is unsafe when the same source outline
    # is reused for the second copper layer in headless pcbnew.
    zone.Outline().Append(outline)
    zone.SetAssignedPriority(priority)
    zone.SetZoneName(zone_name)
    # The front GND2 pour needs 0.50 mm around the MPM3610 SW pads.  KiCad's
    # zone filler does not apply the track/reference-based SwitchNode custom
    # rule to a zone item, so encode the same functional clearance here.
    local_clearance = 0.50 if net_name == "GND2" and layer == FRONT else 0.20
    zone.SetLocalClearance(iu(local_clearance))
    zone.SetMinThickness(iu(0.30 if net_name == "GND2" else 0.20))
    zone.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    zone.SetThermalReliefGap(iu(0.30))
    zone.SetThermalReliefSpokeWidth(iu(0.30))
    board.Add(zone)


def polygon(vertices_mm: Iterable[tuple[float, float]]) -> pcbnew.SHAPE_POLY_SET:
    shape = pcbnew.SHAPE_POLY_SET()
    outline = shape.NewOutline()
    for x_mm, y_mm in vertices_mm:
        shape.Append(iu(x_mm), iu(y_mm), outline)
    return shape


def add_ground_zones(board: pcbnew.BOARD) -> None:
    board_outline = pcbnew.SHAPE_POLY_SET()
    if not board.GetBoardPolygonOutlines(board_outline, False):
        raise RuntimeError("KiCad could not construct a closed board outline")

    # GND2 follows the actual power-component islands.  The notches keep the
    # battery ADC/reset collateral, buzzer pull-down, MCU/FRAM decoupling,
    # BOOT0, and programming header in the logic-GND pour.  R0 at
    # (142.8, 120.2) straddles the vertical boundary: pad 1 is GND2 and pad 2
    # is GND.  The polygon remains one connected, concave power-return region.
    power_outline = polygon(
        (
            (128.00, 116.30),
            (142.75, 116.30),
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
    )

    for layer, suffix in ((FRONT, "F"), (BACK, "B")):
        add_zone(board, "GND", layer, board_outline, 0, f"LOGIC_GND_{suffix}")
        add_zone(board, "GND2", layer, power_outline, 10, f"POWER_GND2_{suffix}")


def route_gate_and_battery_input(board: pcbnew.BOARD) -> int:
    count = 0

    # Clamp the P-FET source-to-gate voltage locally.  The SOD-323 diode was
    # deliberately moved beside Q2 so these two short routes do not enter the
    # motor-mount opening or enlarge the switched-current loop.
    q2_gate = position(pad(board, "Q2", "3"))
    # Battery input to the protected P-FET source.  The short, high-current
    # section is 1.20 mm on 2 oz copper.
    q2_source = position(pad(board, "Q2", "4"))
    count += add_route(
        board,
        "/PWR_GATE_INTERNAL",
        0.25,
        (position(pad(board, "D5", "1")), q2_gate),
    )
    count += add_route(
        board,
        "VBAT_RAW",
        0.30,
        (position(pad(board, "D5", "2")), q2_source),
    )

    # Gate bias and the remote switch pull-down.  Q2's rotated, elongated pads
    # leave no legal F.Cu channel between R12 and R33.  Two vias take this
    # microampere gate path under the package while the source path stays
    # direct on F.Cu.
    r12_gate = position(pad(board, "R12", "1"))
    r33_gate = position(pad(board, "R33", "1"))
    gate_left_via = point(131.00, 113.00)
    gate_right_via = point(135.30, 112.40)
    count += add_route(board, "/PWR_GATE_INTERNAL", 0.20, (r12_gate, gate_left_via))
    count += add_route(board, "/PWR_GATE_INTERNAL", 0.20, (q2_gate, gate_left_via))
    count += add_route(board, "/PWR_GATE_INTERNAL", 0.20, (r33_gate, gate_right_via))
    add_via(board, "/PWR_GATE_INTERNAL", gate_left_via)
    add_via(board, "/PWR_GATE_INTERNAL", gate_right_via)
    count += add_route(
        board,
        "/PWR_GATE_INTERNAL",
        0.20,
        (
            gate_left_via,
            point(131.00, 111.90),
            point(135.30, 111.90),
            gate_right_via,
        ),
        BACK,
    )

    # R12 only carries the gate-bias current.  A two-via B.Cu dogleg passes to
    # the right of the gate bridge, then returns beside D5.  This keeps the
    # F.Cu source connection direct and avoids a crossing between the two nets.
    r12_raw = position(pad(board, "R12", "2"))
    r12_raw_via = point(132.30, 113.00)
    d5_raw_via = point(130.50, 116.55)
    count += add_route(board, "VBAT_RAW", 0.30, (r12_raw, r12_raw_via))
    add_via(board, "VBAT_RAW", r12_raw_via, diameter_mm=1.00)
    add_via(board, "VBAT_RAW", d5_raw_via, diameter_mm=1.00)
    count += add_route(
        board,
        "VBAT_RAW",
        0.30,
        (r12_raw_via, point(132.30, 115.80), point(130.50, 115.80), d5_raw_via),
        BACK,
    )
    count += add_route(
        board,
        "VBAT_RAW",
        0.30,
        (d5_raw_via, position(pad(board, "D5", "2"))),
    )

    source_escape = point(131.75, 118.20)
    count += add_route(board, "VBAT_RAW", 0.30, (q2_source, source_escape))
    count += add_route(
        board,
        "VBAT_RAW",
        1.20,
        (source_escape, point(132.00, 119.00), position(pad(board, "P1", "2"))),
    )

    return count


def route_switched_battery(board: pcbnew.BOARD) -> int:
    count = 0
    q2 = {item.GetNumber(): item for item in board.FindFootprintByReference("Q2").Pads()}

    # Tie all four drain leads immediately outside the package.
    count += add_route(board, "VBAT_SW", 0.60, (position(q2["1"]), position(q2["2"])))
    count += add_route(board, "VBAT_SW", 0.60, (position(q2["5"]), position(q2["6"])))
    count += add_route(
        board,
        "VBAT_SW",
        0.60,
        (position(q2["1"]), point(134.60, 114.36), point(134.60, 116.64), position(q2["6"])),
    )

    c8_vbat = position(pad(board, "C8", "2"))
    c9_vbat = position(pad(board, "C9", "2"))
    c6_vbat = position(pad(board, "C6", "2"))
    c7_vbat = position(pad(board, "C7", "2"))

    # Short P-FET-to-driver trunk and the two local bulk/HF capacitor pairs.
    q2_via = point(134.40, 117.80)
    c8_via = point(138.30, 117.45)
    c9_via = point(135.40, 120.15)
    count += add_route(board, "VBAT_SW", 0.60, (position(q2["6"]), q2_via))
    add_via(board, "VBAT_SW", q2_via)
    # A 1.00/0.40 mm via gives the 0.30 mm annular connection width required
    # by the HighCurrent rule where the diagonal 0.80 mm trunk enters C8.
    add_via(board, "VBAT_SW", c8_via, diameter_mm=1.00)
    add_via(board, "VBAT_SW", c9_via)
    count += add_route(board, "VBAT_SW", 0.80, (q2_via, c8_via), BACK)
    count += add_route(board, "VBAT_SW", 0.80, (q2_via, c9_via), BACK)
    count += add_route(board, "VBAT_SW", 0.60, (c8_via, c8_vbat))
    count += add_route(board, "VBAT_SW", 0.30, (c8_vbat, position(pad(board, "U2", "2"))))
    count += add_route(board, "VBAT_SW", 0.80, (c9_via, c9_vbat))
    count += add_route(
        board,
        "VBAT_SW",
        0.80,
        (c9_vbat, point(137.80, 121.20), c6_vbat),
    )
    c7_branch_via = pcbnew.VECTOR2I(iu(135.60), c7_vbat.y)
    add_via(board, "VBAT_SW", c7_branch_via)
    count += add_route(board, "VBAT_SW", 0.80, (c9_via, c7_branch_via), BACK)
    count += add_route(board, "VBAT_SW", 0.80, (c7_branch_via, c7_vbat))
    count += add_route(board, "VBAT_SW", 0.30, (c6_vbat, position(pad(board, "U3", "2"))))

    # Cross the congested R0/driver boundary on B.Cu.  This is a short branch,
    # not the motor-return path, and leaves both ground pours connected around
    # its ends.
    left_via = point(136.80, 126.30)
    mpm_via = point(146.00, 124.50)
    fan_via = point(154.50, 124.00)
    count += connect_with_vias(
        board,
        "VBAT_SW",
        0.80,
        c7_vbat,
        left_via,
        (),
        mpm_via,
        position(pad(board, "C22", "1")),
        escape_width_mm=0.60,
    )
    add_via(board, "VBAT_SW", fan_via)
    count += add_route(board, "VBAT_SW", 0.80, (mpm_via, fan_via), BACK)
    count += add_route(
        board,
        "VBAT_SW",
        0.60,
        (fan_via, position(pad(board, "D4", "2"))),
        FRONT,
    )

    # Local MPM input/enable and fan HF bypass/flyback supply branches.
    count += add_route(
        board,
        "VBAT_SW",
        0.40,
        (
            position(pad(board, "C22", "1")),
            point(145.75, 126.30),
            position(pad(board, "U7", "16")),
        ),
    )
    count += add_route(
        board,
        "VBAT_SW",
        0.30,
        (
            position(pad(board, "R31", "2")),
            point(142.89, 124.90),
            point(145.30, 124.90),
            point(146.00, 124.50),
        ),
    )
    count += add_route(
        board,
        "VBAT_SW",
        0.30,
        (fan_via, position(pad(board, "C24", "1"))),
    )
    return count


def route_motor_outputs(board: pcbnew.BOARD) -> int:
    count = 0

    # The 0.5 mm-pitch MP6551 power leads escape at the DRC-enforced 0.30 mm
    # minimum, then widen to 0.80 mm only after clearing the adjacent lead.
    u3_out1_escape = point(143.80, 122.54)
    u3_out2_escape = point(143.80, 123.54)
    count += add_route(
        board,
        "/MOTOR_L_OUT1",
        0.30,
        (position(pad(board, "U3", "10")), u3_out1_escape),
    )
    count += add_route(
        board,
        "/MOTOR_L_OUT1",
        0.80,
        (u3_out1_escape, position(pad(board, "TP1", "TP"))),
    )
    count += add_route(
        board,
        "/MOTOR_L_OUT2",
        0.30,
        (position(pad(board, "U3", "8")), u3_out2_escape),
    )
    count += add_route(
        board,
        "/MOTOR_L_OUT2",
        0.80,
        (u3_out2_escape, position(pad(board, "TP2", "TP"))),
    )

    # Join the duplicate left leads around the package perimeter.  The chosen
    # y coordinates leave at least 0.16 mm to R0 and R31 while avoiding vias.
    count += add_route(
        board,
        "/MOTOR_L_OUT1",
        0.30,
        (
            position(pad(board, "U3", "1")),
            point(140.10, 122.54),
            point(140.10, 120.90),
            point(143.20, 120.90),
            point(143.20, 122.54),
            position(pad(board, "U3", "10")),
        ),
    )
    # U3.3 is deliberately left as a duplicate output lead for the manual
    # finish.  The reviewed placement has no 0.30/0.16 mm path around R31 and
    # the MPM input via without cutting the power-return plane.

    # Right motor OUT2 is direct after its escape.  OUT1 uses one short B.Cu
    # crossover to avoid TP4.
    u2_out2_escape = point(143.80, 117.95)
    count += add_route(
        board,
        "/MOTOR_R_OUT2",
        0.30,
        (position(pad(board, "U2", "8")), u2_out2_escape),
    )
    count += add_route(
        board,
        "/MOTOR_R_OUT2",
        0.80,
        (u2_out2_escape, position(pad(board, "TP4", "TP"))),
    )
    count += connect_with_vias(
        board,
        "/MOTOR_R_OUT1",
        0.80,
        position(pad(board, "U2", "10")),
        point(143.80, 116.80),
        (point(146.20, 116.80), point(146.20, 120.80)),
        point(145.90, 120.80),
        position(pad(board, "TP3", "TP")),
        escape_width_mm=0.30,
    )
    u2_out1_left_via = point(139.00, 116.30)
    count += add_route(
        board,
        "/MOTOR_R_OUT1",
        0.30,
        (
            position(pad(board, "U2", "1")),
            point(140.10, 116.95),
            point(140.10, 116.60),
            point(139.00, 116.60),
            u2_out1_left_via,
        ),
    )
    add_via(board, "/MOTOR_R_OUT1", u2_out1_left_via)
    count += add_route(
        board,
        "/MOTOR_R_OUT1",
        0.80,
        (u2_out1_left_via, point(143.80, 116.80)),
        BACK,
    )

    u2_out2_left_via = point(139.20, 120.10)
    u2_out2_right_via = point(144.80, 119.10)
    count += add_route(
        board,
        "/MOTOR_R_OUT2",
        0.30,
        (
            position(pad(board, "U2", "3")),
            point(140.25, 117.95),
            point(140.25, 120.10),
            u2_out2_left_via,
        ),
    )
    add_via(board, "/MOTOR_R_OUT2", u2_out2_left_via)
    add_via(board, "/MOTOR_R_OUT2", u2_out2_right_via)
    count += add_route(
        board,
        "/MOTOR_R_OUT2",
        0.80,
        (u2_out2_left_via, point(144.00, 119.10), u2_out2_right_via),
        BACK,
    )
    count += add_route(
        board,
        "/MOTOR_R_OUT2",
        0.30,
        (u2_out2_right_via, u2_out2_escape),
    )
    return count


def route_mpm_and_regulator(board: pcbnew.BOARD) -> int:
    count = 0

    # MPM3610 feedback divider and enable pull-up stay away from SW copper.
    fb = position(pad(board, "U7", "1"))
    r29_fb = position(pad(board, "R29", "1"))
    r30_fb = position(pad(board, "R30", "1"))
    count += add_route(board, "/MPM_FB_INTERNAL", 0.20, (fb, point(143.70, 128.45), r30_fb))
    count += add_route(board, "/MPM_FB_INTERNAL", 0.20, (r30_fb, r29_fb))
    count += add_route(
        board,
        "/MPM_EN_INTERNAL",
        0.20,
        (
            position(pad(board, "R31", "1")),
            point(143.91, 126.80),
            point(145.25, 126.80),
            position(pad(board, "U7", "17")),
        ),
    )

    # The SW and +5 V perimeter pads already touch their respective exposed
    # copper lands in the footprint.  Do not enlarge the high-dv/dt SW island
    # with explicit tracks; only identify the large anchors for the external
    # output connection below.
    sw_pads = pads_on_net(board, "U7", "/MPM_SW_INTERNAL")
    sw_anchor = max(sw_pads, key=lambda item: item.GetSize().x * item.GetSize().y)
    del sw_anchor

    # Widen the touching +5 V output anchor immediately to the 10 uF capacitor.
    output_pads = pads_on_net(board, "U7", "+5V")
    output_anchor = max(output_pads, key=lambda item: item.GetSize().x * item.GetSize().y)

    c23_out = position(pad(board, "C23", "1"))
    c14_in = position(pad(board, "C14", "1"))
    count += add_route(board, "+5V", 0.60, (position(output_anchor), c23_out))
    count += add_route(
        board,
        "+5V",
        0.50,
        (c23_out, point(150.30, 131.90), point(150.30, 129.55), c14_in),
    )
    count += add_route(board, "+5V", 0.50, (c14_in, position(pad(board, "U1", "3"))))
    count += add_route(
        board,
        "+3V3",
        0.40,
        (position(pad(board, "U1", "2")), position(pad(board, "C15", "1"))),
    )
    return count


def route_fan_local_loop(board: pcbnew.BOARD) -> int:
    count = 0
    fan_drain = position(pad(board, "Q3", "3"))
    flyback_fan = position(pad(board, "D4", "1"))
    bypass_fan = position(pad(board, "C24", "2"))

    # Keep the MOSFET/flyback commutation loop compact and entirely on F.Cu.
    count += add_route(
        board,
        "/FAN_NEG_INTERNAL",
        0.80,
        (fan_drain, point(162.00, 122.00), point(162.00, 125.80), flyback_fan),
    )
    count += add_route(
        board,
        "/FAN_NEG_INTERNAL",
        0.30,
        (bypass_fan, point(157.80, 123.60), point(159.00, 124.50), flyback_fan),
    )
    return count


def count_unconnected(board: pcbnew.BOARD) -> int:
    board.BuildConnectivity()
    return int(board.GetConnectivity().GetUnconnectedCount(False))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    if args.input.resolve() == args.output.resolve():
        raise RuntimeError("refusing in-place write; use a separate output path")
    if args.output.exists():
        raise RuntimeError(f"refusing to overwrite existing output: {args.output}")

    board = pcbnew.LoadBoard(str(args.input))
    if board is None:
        raise RuntimeError(f"could not load {args.input}")
    if list(board.GetTracks()) or board.GetAreaCount():
        raise RuntimeError("route_power.py requires a prepared board with no copper")
    if board.GetCopperLayerCount() != 2:
        raise RuntimeError(f"expected a 2-layer board, found {board.GetCopperLayerCount()}")
    verify_reviewed_placement(board)

    before = count_unconnected(board)
    segment_count = 0
    segment_count += route_gate_and_battery_input(board)
    segment_count += route_switched_battery(board)
    segment_count += route_motor_outputs(board)
    segment_count += route_mpm_and_regulator(board)
    segment_count += route_fan_local_loop(board)
    add_ground_zones(board)
    if not pcbnew.ZONE_FILLER(board).Fill(board.Zones()):
        raise RuntimeError("KiCad zone fill failed")
    after = count_unconnected(board)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not pcbnew.SaveBoard(str(args.output), board):
        raise RuntimeError(f"could not save {args.output}")
    reloaded = pcbnew.LoadBoard(str(args.output))
    if reloaded is None:
        raise RuntimeError(f"saved board could not be reloaded: {args.output}")

    vias = sum(isinstance(item, pcbnew.PCB_VIA) for item in reloaded.GetTracks())
    tracks = sum(
        isinstance(item, pcbnew.PCB_TRACK)
        and not isinstance(item, pcbnew.PCB_VIA)
        for item in reloaded.GetTracks()
    )
    print(
        f"power route: {segment_count} segments, {tracks} tracks, {vias} vias, "
        f"4 filled zones; unconnected {before} -> {after}; output {args.output}"
    )
    print("routed nets: " + ", ".join(sorted(ROUTED_NETS)))
    print(
        "left for later routing: J2.1/J2.2 remote fan pair, "
        "R33.2-POWER0.2 switch return, VBAT_SW battery-divider branch, "
        "global +5V/+3V3 distribution, and duplicate IC power/output pads disclosed in README"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
