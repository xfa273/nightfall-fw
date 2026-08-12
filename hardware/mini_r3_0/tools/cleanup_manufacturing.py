#!/usr/bin/env python3
"""Apply reviewed manufacturing cleanups to a routed mini_r3 board copy.

The transformations are idempotent and deliberately board-specific.  The tool
refuses in-place writes and existing outputs, saves through a non-project
temporary basename, reloads the result, and atomically replaces the requested
output.  Run KiCad DRC on the complete copied project after every application.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import tempfile

try:
    import wx
except ImportError as exc:  # pragma: no cover - KiCad runtime dependency
    raise SystemExit(
        "wx is unavailable. Run this tool with KiCad's bundled Python."
    ) from exc

# Zone fill requires a live wx application.  Explicit exception-only assertion
# handling keeps headless runs from opening a modal assertion dialog.
_WX_APP = wx.GetApp() or wx.App(False)
_WX_APP.SetAssertMode(wx.APP_ASSERT_EXCEPTION)
wx.Log.SetLogLevel(wx.LOG_Error)

try:
    import pcbnew
except ImportError as exc:  # pragma: no cover - KiCad runtime dependency
    raise SystemExit(
        "pcbnew is unavailable. Run this tool with KiCad's bundled Python."
    ) from exc


F = pcbnew.F_Cu
B = pcbnew.B_Cu


def iu(x: float) -> int:
    return int(pcbnew.FromMM(x))


def pt(x: float, y: float) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I(iu(x), iu(y))


def close(a: object, b: tuple[float, float], tol: float = 0.003) -> bool:
    return (
        abs(pcbnew.ToMM(a.x) - b[0]) < tol
        and abs(pcbnew.ToMM(a.y) - b[1]) < tol
    )


def net(board, name: str):
    n = board.FindNet(name)
    if n is None:
        raise RuntimeError(f"missing net {name}")
    return n


def remove_via(
    board: object,
    net_name: str,
    at: tuple[float, float],
    diameter: float,
    drill: float,
) -> None:
    found = find_vias(board, net_name, at, diameter, drill)
    if len(found) != 1:
        raise RuntimeError(f"expected one via {net_name}@{at}, found {len(found)}")
    board.Delete(found[0])


def find_vias(
    board: object,
    net_name: str,
    at: tuple[float, float],
    diameter: float,
    drill: float,
) -> list[object]:
    found = []
    for item in board.GetTracks():
        if not isinstance(item, pcbnew.PCB_VIA) or item.GetNetname() != net_name:
            continue
        if not close(item.GetPosition(), at):
            continue
        if abs(pcbnew.ToMM(item.GetWidth(F))-diameter) > 0.003:
            continue
        if abs(pcbnew.ToMM(item.GetDrillValue())-drill) > 0.003:
            continue
        found.append(item)
    return found


def same_edge(
    item: object, a: tuple[float, float], b: tuple[float, float]
) -> bool:
    return (
        close(item.GetStart(), a)
        and close(item.GetEnd(), b)
    ) or (
        close(item.GetStart(), b)
        and close(item.GetEnd(), a)
    )


def remove_track(
    board: object,
    net_name: str,
    layer: int,
    a: tuple[float, float],
    b: tuple[float, float],
    width: float | None = None,
) -> None:
    found = find_tracks(board, net_name, layer, a, b, width)
    if len(found) != 1:
        raise RuntimeError(f"expected one track {net_name} {a}->{b} w={width}, found {len(found)}")
    board.Delete(found[0])


def find_tracks(
    board: object,
    net_name: str,
    layer: int,
    a: tuple[float, float],
    b: tuple[float, float],
    width: float | None = None,
) -> list[object]:
    found = []
    for item in board.GetTracks():
        if isinstance(item, pcbnew.PCB_VIA) or item.GetNetname() != net_name or item.GetLayer() != layer:
            continue
        if not same_edge(item,a,b):
            continue
        if width is not None and abs(pcbnew.ToMM(item.GetWidth())-width) > 0.003:
            continue
        found.append(item)
    return found


def require_count(items: list[object], count: int, label: str) -> None:
    if len(items) != count:
        raise RuntimeError(f"{label}: expected {count}, found {len(items)}")


def add_track(
    board: object,
    net_name: str,
    layer: int,
    a: tuple[float, float],
    b: tuple[float, float],
    width: float,
) -> None:
    t = pcbnew.PCB_TRACK(board)
    t.SetStart(pt(*a))
    t.SetEnd(pt(*b))
    t.SetWidth(iu(width))
    t.SetLayer(layer)
    t.SetNet(net(board, net_name))
    board.Add(t)


def add_via(
    board: object,
    net_name: str,
    at: tuple[float, float],
    diameter: float = 1.0,
    drill: float = 0.4,
) -> None:
    v = pcbnew.PCB_VIA(board)
    v.SetPosition(pt(*at))
    v.SetWidth(iu(diameter))
    v.SetDrill(iu(drill))
    v.SetLayerPair(F, B)
    v.SetNet(net(board, net_name))
    board.Add(v)


def move_track_endpoint(
    board: object,
    net_name: str,
    layer: int,
    old: tuple[float, float],
    new: tuple[float, float],
    endpoint: str,
) -> None:
    """Move exactly one start or end point while preserving the track UUID."""
    if endpoint not in {"start", "end"}:
        raise ValueError(f"invalid endpoint {endpoint}")
    found = []
    for item in board.GetTracks():
        if (
            isinstance(item, pcbnew.PCB_VIA)
            or item.GetNetname() != net_name
            or item.GetLayer() != layer
        ):
            continue
        position = item.GetStart() if endpoint == "start" else item.GetEnd()
        if close(position, old):
            found.append(item)
    require_count(found, 1, f"{net_name} {layer} {endpoint} at {old}")
    setter = found[0].SetStart if endpoint == "start" else found[0].SetEnd
    setter(pt(*new))


def q3_toe(board: object) -> None:
    if not find_vias(board, "/FAN_NEG_INTERNAL", (160.9375, 122.0), 0.8, 0.4):
        require_count(
            find_vias(board, "/FAN_NEG_INTERNAL", (162.0, 122.0), 0.8, 0.4),
            1,
            "Q3 toe via",
        )
        return
    remove_via(board, "/FAN_NEG_INTERNAL", (160.9375, 122.0), 0.8, 0.4)
    remove_track(
        board,
        "/FAN_NEG_INTERNAL",
        B,
        (160.4, 121.0),
        (160.9375, 122.0),
        0.8,
    )


def c8_offpad(board: object) -> None:
    """Move the C8 transition completely off its 0805 pad.

    The first proposed cleanup put another power via too close to C8/C9.  The
    reviewed geometry instead moves C9 by 0.50 mm, uses one 0.80/0.40 mm via at
    (137.70, 117.45), and retains the existing (139.35, 117.45) bank via.  This
    avoids solder wicking without narrowing the 0.80 mm bottom-layer trunk.
    """
    final_vias = find_vias(board, "VBAT_SW", (137.70, 117.45), 0.8, 0.4)
    c9 = board.FindFootprintByReference("C9")
    if c9 is None:
        raise RuntimeError("missing footprint C9")
    if final_vias:
        require_count(final_vias, 1, "C8 off-pad via")
        if not close(c9.GetPosition(), (136.8, 119.7)):
            raise RuntimeError("C8 cleanup is partial: C9 is not at final position")
        for layer, a, b, width, label in (
            (F, (137.70, 117.45), (138.50, 117.45), 0.6, "C8 F toe"),
            (B, (134.40, 117.80), (137.70, 117.45), 0.8, "C8 B inlet"),
            (B, (137.70, 117.45), (139.35, 117.45), 0.8, "C8 B outlet"),
        ):
            require_count(find_tracks(board, "VBAT_SW", layer, a, b, width), 1, label)
        return

    require_count(
        find_vias(board, "VBAT_SW", (138.30, 117.45), 1.0, 0.4),
        1,
        "C8 legacy in-pad via",
    )
    if not close(c9.GetPosition(), (136.8, 119.2)):
        raise RuntimeError("unexpected C9 position before C8 cleanup")

    # Move C9 and follow its VBAT_SW pad with the two incident F.Cu tracks.
    c9.SetPosition(pt(136.8, 119.7))
    for other in ((135.4, 120.15), (137.8, 121.2)):
        remove_track(board, "VBAT_SW", F, (136.8, 120.15), other, 0.8)
    add_track(board, "VBAT_SW", F, (135.4, 120.15), (136.8, 120.65), 0.8)
    add_track(board, "VBAT_SW", F, (136.8, 120.65), (137.8, 121.2), 0.8)

    remove_via(board, "VBAT_SW", (138.30, 117.45), 1.0, 0.4)
    remove_track(board, "VBAT_SW", B, (134.4, 117.8), (138.3, 117.45), 0.8)
    remove_track(board, "VBAT_SW", B, (138.3, 117.45), (139.35, 117.45), 0.8)
    add_via(board, "VBAT_SW", (137.70, 117.45), 0.8, 0.4)
    add_track(board, "VBAT_SW", F, (137.70, 117.45), (138.50, 117.45), 0.6)
    add_track(board, "VBAT_SW", B, (134.40, 117.80), (137.70, 117.45), 0.8)
    add_track(board, "VBAT_SW", B, (137.70, 117.45), (139.35, 117.45), 0.8)


def front_sensor_toes(board: object) -> None:
    """Move two front-sensor holes out of their 0402 pads.

    These are short 0.40/0.20 mm toe vias; all following signal vias remain the
    normal 0.60/0.30 mm size.  The small moves preserve the reviewed sensor
    routing and the mechanical sensor placement.
    """
    moves = (
        (
            "Net-(IR_LED_FR0-PadA)",
            (165.20, 82.15),
            (165.30, 82.35),
            F,
            "end",
            B,
            "start",
        ),
        (
            "/SENSOR_R",
            (162.40, 86.95),
            (162.40, 86.80),
            F,
            "end",
            B,
            "start",
        ),
    )
    for net_name, old, new, layer1, end1, layer2, end2 in moves:
        final = find_vias(board, net_name, new, 0.4, 0.2)
        if final:
            require_count(final, 1, f"{net_name} final toe via")
            continue
        legacy = find_vias(board, net_name, old, 0.6, 0.3)
        require_count(legacy, 1, f"{net_name} legacy in-pad via")
        legacy[0].SetPosition(pt(*new))
        legacy[0].SetWidth(iu(0.4))
        legacy[0].SetDrill(iu(0.2))
        move_track_endpoint(board, net_name, layer1, old, new, end1)
        move_track_endpoint(board, net_name, layer2, old, new, end2)

    # This LED pulse-current branch was originally autorouted for its entire
    # 15.9 mm length at the 2 oz fabrication minimum (0.16 mm).  Moving the
    # intermediate polyline nodes 0.10 mm toward the board centre leaves the
    # mechanical LED/resistor pads fixed and provides enough edge margin to
    # widen every segment to the nominal 0.20 mm signal width.
    led_net = "Net-(IR_LED_FL0-PadA)"
    led_tracks = [
        item
        for item in board.GetTracks()
        if not isinstance(item, pcbnew.PCB_VIA)
        and item.GetNetname() == led_net
    ]
    require_count(led_tracks, 17, f"{led_net} edge polyline")
    legacy = [item for item in led_tracks if abs(pcbnew.ToMM(item.GetWidth()) - 0.16) < 0.003]
    final = [item for item in led_tracks if abs(pcbnew.ToMM(item.GetWidth()) - 0.20) < 0.003]
    if final and not legacy:
        require_count(final, 17, f"{led_net} widened polyline")
        return
    require_count(legacy, 17, f"{led_net} legacy narrow polyline")
    fixed_pad_endpoints = ((132.0, 81.8), (131.111563, 95.854774))

    def is_fixed_pad_endpoint(position: object) -> bool:
        return any(close(position, endpoint) for endpoint in fixed_pad_endpoints)

    offset = pt(0.10, 0.0)
    for item in led_tracks:
        start = item.GetStart()
        end = item.GetEnd()
        if not is_fixed_pad_endpoint(start):
            item.SetStart(start + offset)
        if not is_fixed_pad_endpoint(end):
            item.SetEnd(end + offset)
        item.SetWidth(iu(0.20))


def q2_source(board: object) -> None:
    # The old 0.30 mm source escape lasted 1.5625 mm.  Match the 0.60 mm pad
    # immediately, then widen to 0.80 and 1.20 mm as soon as lead clearance
    # permits; the existing 1.20 mm trunk from (131.75,118.20) to P1 remains.
    old = find_tracks(
        board, "VBAT_RAW", F, (131.75, 116.6375), (131.75, 118.20), 0.3
    )
    if not old:
        for start, end, width, label in (
            ((131.75, 116.6375), (131.75, 117.60), 0.6, "Q2 0.6 escape"),
            ((131.75, 117.60), (131.75, 117.85), 0.8, "Q2 0.8 escape"),
            ((131.75, 117.85), (131.75, 118.20), 1.2, "Q2 1.2 escape"),
        ):
            require_count(
                find_tracks(board, "VBAT_RAW", F, start, end, width), 1, label
            )
        return
    require_count(old, 1, "Q2 legacy source neck")
    board.Delete(old[0])
    add_track(board, "VBAT_RAW", F, (131.75, 116.6375), (131.75, 117.60), 0.6)
    add_track(board, "VBAT_RAW", F, (131.75, 117.60), (131.75, 117.85), 0.8)
    add_track(board, "VBAT_RAW", F, (131.75, 117.85), (131.75, 118.20), 1.2)


def refill(board: object) -> None:
    zones = list(board.Zones())
    if zones:
        pcbnew.ZONE_FILLER(board).Fill(zones)
    board.BuildConnectivity()


def save_atomic(board: object, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(
        prefix=".mfg-", suffix=".kicad_pcb", dir=str(output.parent)
    )
    os.close(fd)
    tmp = Path(name)
    # KiCad may create a project sidecar matching the temporary PCB basename.
    # It must not be left beside the user's real .kicad_pro/.kicad_prl files.
    temporary_sidecars = tuple(
        tmp.with_suffix(suffix) for suffix in (".kicad_pro", ".kicad_prl")
    )
    try:
        if not pcbnew.SaveBoard(str(tmp), board):
            raise RuntimeError("SaveBoard failed")
        if pcbnew.LoadBoard(str(tmp)) is None:
            raise RuntimeError("reload failed")
        os.replace(tmp, output)
    finally:
        if tmp.exists():
            tmp.unlink()
        for sidecar in temporary_sidecars:
            if sidecar.exists():
                sidecar.unlink()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="reviewed routed input board")
    parser.add_argument("output", type=Path, help="new output board; must not exist")
    parser.add_argument(
        "--actions",
        default="q3,c8,q2,front",
        help="comma-separated subset of q3,c8,q2,front (default: all)",
    )
    return parser.parse_args()


def require_project_context(board_path: Path, role: str) -> None:
    missing = [
        board_path.with_suffix(suffix)
        for suffix in (".kicad_pro", ".kicad_dru")
        if not board_path.with_suffix(suffix).is_file()
    ]
    if missing:
        formatted = ", ".join(str(path) for path in missing)
        raise RuntimeError(
            f"{role} board must have matching project/rule sidecars: {formatted}"
        )


def main() -> int:
    args = parse_args()
    input_path = args.input.resolve()
    output_path = args.output.resolve()
    if not input_path.is_file():
        raise RuntimeError(f"input board does not exist: {input_path}")
    if input_path == output_path:
        raise RuntimeError("refusing in-place write; use a separate output path")
    if output_path.exists():
        raise RuntimeError(f"refusing to overwrite existing output: {output_path}")
    # Zone refill and netclass behavior are project-dependent.  Requiring the
    # canonical basename sidecars also prevents a second run from interpreting
    # a board under different rules and losing filled-zone connectivity.
    require_project_context(input_path, "input")
    require_project_context(output_path, "output")

    board = pcbnew.LoadBoard(str(input_path))
    if board is None:
        raise RuntimeError(f"could not load board: {input_path}")

    actions = {item.strip() for item in args.actions.split(",") if item.strip()}
    unknown = actions - {"q3", "c8", "q2", "front"}
    if unknown:
        raise RuntimeError(f"unknown actions: {sorted(unknown)}")
    if "q3" in actions:
        q3_toe(board)
    if "c8" in actions:
        c8_offpad(board)
    if "q2" in actions:
        q2_source(board)
    if "front" in actions:
        front_sensor_toes(board)
    refill(board)
    save_atomic(board, output_path)
    print(
        f"saved={output_path} "
        f"unconnected={board.GetConnectivity().GetUnconnectedCount(False)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
