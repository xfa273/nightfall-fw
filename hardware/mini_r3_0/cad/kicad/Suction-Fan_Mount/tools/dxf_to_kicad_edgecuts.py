#!/usr/bin/env python3
"""Convert the supplied simple DXF into a mechanical KiCad PCB.

LINE and ARC entities become Edge.Cuts. CIRCLE entities become round NPTH
drills because PCB fabricators handle round holes more accurately in Excellon.
"""

from __future__ import annotations

import argparse
import hashlib
import math
import uuid
from collections import Counter
from pathlib import Path


SUPPORTED_ENTITIES = {"LINE", "ARC", "CIRCLE"}
IGNORED_ENTITIES = {"MTEXT"}  # SolidWorks Maker license annotation outside the part geometry.
UUID_NAMESPACE = uuid.UUID("c5a36e75-7f7e-4a1f-b57b-6db2fbbe97d1")


def fmt(value: float) -> str:
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return "0" if text in {"", "-0"} else text


def stable_uuid(label: str) -> str:
    return str(uuid.uuid5(UUID_NAMESPACE, label))


def read_pairs(path: Path) -> list[tuple[int, str]]:
    raw = path.read_text(encoding="cp932", errors="strict").splitlines()
    if len(raw) % 2:
        raise ValueError("DXF has an incomplete group-code pair")
    pairs: list[tuple[int, str]] = []
    for index in range(0, len(raw), 2):
        pairs.append((int(raw[index].strip()), raw[index + 1].strip()))
    return pairs


def header_value(pairs: list[tuple[int, str]], name: str, code: int) -> str:
    for index, pair in enumerate(pairs):
        if pair == (9, name):
            for candidate_code, value in pairs[index + 1 : index + 8]:
                if candidate_code == code:
                    return value
                if candidate_code == 9:
                    break
    raise ValueError(f"DXF header variable {name} is missing")


def parse_entities(pairs: list[tuple[int, str]]) -> list[dict]:
    start = next(i for i, pair in enumerate(pairs) if pair == (2, "ENTITIES")) + 1
    entities: list[dict] = []
    current: dict | None = None
    for code, value in pairs[start:]:
        if code == 0 and value == "ENDSEC":
            if current is not None:
                entities.append(current)
            break
        if code == 0:
            if current is not None:
                entities.append(current)
            if value in IGNORED_ENTITIES:
                current = None
                continue
            if value not in SUPPORTED_ENTITIES:
                raise ValueError(f"Unsupported DXF entity in ENTITIES section: {value}")
            current = {"type": value, "groups": {}}
            continue
        if current is not None:
            current["groups"].setdefault(code, []).append(value)
    return entities


def group_float(entity: dict, code: int) -> float:
    return float(entity["groups"][code][0])


def arc_point(entity: dict, angle_degrees: float) -> tuple[float, float]:
    angle = math.radians(angle_degrees)
    cx = group_float(entity, 10)
    cy = group_float(entity, 20)
    radius = group_float(entity, 40)
    return cx + radius * math.cos(angle), cy + radius * math.sin(angle)


def segment_endpoints(entity: dict) -> tuple[tuple[float, float], tuple[float, float]]:
    if entity["type"] == "LINE":
        return (
            (group_float(entity, 10), group_float(entity, 20)),
            (group_float(entity, 11), group_float(entity, 21)),
        )
    if entity["type"] == "ARC":
        return arc_point(entity, group_float(entity, 50)), arc_point(entity, group_float(entity, 51))
    raise ValueError("Circles do not have segment endpoints")


def validate_closed_contours(entities: list[dict], tolerance: float = 1e-5) -> int:
    segments = [entity for entity in entities if entity["type"] != "CIRCLE"]
    clusters: list[tuple[tuple[float, float], list[int]]] = []
    for index, entity in enumerate(segments):
        for point in segment_endpoints(entity):
            for cluster_point, members in clusters:
                if math.dist(point, cluster_point) <= tolerance:
                    members.append(index)
                    break
            else:
                clusters.append((point, [index]))

    open_nodes = [(point, members) for point, members in clusters if len(members) != 2]
    if open_nodes:
        raise ValueError(f"DXF contains open or branched contour endpoints: {open_nodes}")

    adjacency = [set() for _ in segments]
    for _, members in clusters:
        first, second = members
        adjacency[first].add(second)
        adjacency[second].add(first)

    visited: set[int] = set()
    components = 0
    for start in range(len(segments)):
        if start in visited:
            continue
        components += 1
        stack = [start]
        visited.add(start)
        while stack:
            current = stack.pop()
            for neighbor in adjacency[current]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    stack.append(neighbor)
    return components


def entity_extents(entity: dict) -> tuple[float, float, float, float]:
    if entity["type"] == "LINE":
        xs = [group_float(entity, 10), group_float(entity, 11)]
        ys = [group_float(entity, 20), group_float(entity, 21)]
        return min(xs), min(ys), max(xs), max(ys)
    if entity["type"] == "CIRCLE":
        cx, cy, radius = group_float(entity, 10), group_float(entity, 20), group_float(entity, 40)
        return cx - radius, cy - radius, cx + radius, cy + radius

    start = group_float(entity, 50) % 360
    end = group_float(entity, 51) % 360
    sweep = (end - start) % 360
    angles = [start, end]
    for candidate in (0.0, 90.0, 180.0, 270.0):
        if (candidate - start) % 360 <= sweep + 1e-9:
            angles.append(candidate)
    points = [arc_point(entity, angle) for angle in angles]
    return (
        min(point[0] for point in points),
        min(point[1] for point in points),
        max(point[0] for point in points),
        max(point[1] for point in points),
    )


def transform(point: tuple[float, float], tx: float, ty: float) -> tuple[float, float]:
    return point[0] + tx, ty - point[1]


def footprint_for_hole(index: int, entity: dict, tx: float, ty: float) -> str:
    cx, cy = transform((group_float(entity, 10), group_float(entity, 20)), tx, ty)
    diameter = 2 * group_float(entity, 40)
    ref = f"H{index}"
    value = f"DXF_NPTH_{fmt(diameter)}mm"
    return f'''\t(footprint "Suction-Fan-Mount:{value}"
\t\t(layer "F.Cu")
\t\t(uuid "{stable_uuid(f'hole-{index}-footprint')}")
\t\t(at {fmt(cx)} {fmt(cy)})
\t\t(descr "Round {fmt(diameter)} mm non-plated hole converted from Suction-Fan_Mount.DXF")
\t\t(property "Reference" "{ref}"
\t\t\t(at 0 {fmt(-(diameter / 2 + 1.0))} 0)
\t\t\t(layer "F.Fab")
\t\t\t(hide yes)
\t\t\t(uuid "{stable_uuid(f'hole-{index}-reference')}")
\t\t\t(effects (font (size 1 1) (thickness 0.15)))
\t\t)
\t\t(property "Value" "{value}"
\t\t\t(at 0 {fmt(diameter / 2 + 1.0)} 0)
\t\t\t(layer "F.Fab")
\t\t\t(hide yes)
\t\t\t(uuid "{stable_uuid(f'hole-{index}-value')}")
\t\t\t(effects (font (size 1 1) (thickness 0.15)))
\t\t)
\t\t(property "Datasheet" ""
\t\t\t(at 0 0 0)
\t\t\t(layer "F.Fab")
\t\t\t(hide yes)
\t\t\t(uuid "{stable_uuid(f'hole-{index}-datasheet')}")
\t\t\t(effects (font (size 1 1) (thickness 0.15)))
\t\t)
\t\t(property "Description" "Round NPTH from Suction-Fan_Mount.DXF"
\t\t\t(at 0 0 0)
\t\t\t(layer "F.Fab")
\t\t\t(hide yes)
\t\t\t(uuid "{stable_uuid(f'hole-{index}-description')}")
\t\t\t(effects (font (size 1 1) (thickness 0.15)))
\t\t)
\t\t(attr exclude_from_pos_files exclude_from_bom)
\t\t(pad "" np_thru_hole circle
\t\t\t(at 0 0)
\t\t\t(size {fmt(diameter)} {fmt(diameter)})
\t\t\t(drill {fmt(diameter)})
\t\t\t(layers "*.Cu" "*.Mask")
\t\t\t(uuid "{stable_uuid(f'hole-{index}-pad')}")
\t\t)
\t\t(embedded_fonts no)
\t)'''


def edge_graphic(index: int, entity: dict, tx: float, ty: float) -> str:
    graphic_uuid = stable_uuid(f"edge-{index}")
    if entity["type"] == "LINE":
        start, end = segment_endpoints(entity)
        start = transform(start, tx, ty)
        end = transform(end, tx, ty)
        return f'''\t(gr_line
\t\t(start {fmt(start[0])} {fmt(start[1])})
\t\t(end {fmt(end[0])} {fmt(end[1])})
\t\t(stroke (width 0.05) (type default))
\t\t(layer "Edge.Cuts")
\t\t(uuid "{graphic_uuid}")
\t)'''

    if entity["type"] == "ARC":
        start_angle = group_float(entity, 50)
        end_angle = group_float(entity, 51)
        sweep = (end_angle - start_angle) % 360
        start = transform(arc_point(entity, start_angle), tx, ty)
        mid = transform(arc_point(entity, start_angle + sweep / 2), tx, ty)
        end = transform(arc_point(entity, end_angle), tx, ty)
        return f'''\t(gr_arc
\t\t(start {fmt(start[0])} {fmt(start[1])})
\t\t(mid {fmt(mid[0])} {fmt(mid[1])})
\t\t(end {fmt(end[0])} {fmt(end[1])})
\t\t(stroke (width 0.05) (type default))
\t\t(layer "Edge.Cuts")
\t\t(uuid "{graphic_uuid}")
\t)'''
    raise ValueError(f"Unexpected Edge.Cuts entity: {entity['type']}")


def board_header() -> str:
    return '''(kicad_pcb
\t(version 20260206)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general
\t\t(thickness 1.6)
\t\t(legacy_teardrops no)
\t)
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(9 "F.Adhes" user "F.Adhesive")
\t\t(11 "B.Adhes" user "B.Adhesive")
\t\t(13 "F.Paste" user)
\t\t(15 "B.Paste" user)
\t\t(5 "F.SilkS" user "F.Silkscreen")
\t\t(7 "B.SilkS" user "B.Silkscreen")
\t\t(1 "F.Mask" user)
\t\t(3 "B.Mask" user)
\t\t(17 "Dwgs.User" user "User.Drawings")
\t\t(19 "Cmts.User" user "User.Comments")
\t\t(21 "Eco1.User" user "User.Eco1")
\t\t(23 "Eco2.User" user "User.Eco2")
\t\t(25 "Edge.Cuts" user)
\t\t(27 "Margin" user)
\t\t(31 "F.CrtYd" user "F.Courtyard")
\t\t(29 "B.CrtYd" user "B.Courtyard")
\t\t(35 "F.Fab" user)
\t\t(33 "B.Fab" user)
\t)
\t(setup
\t\t(stackup
\t\t\t(layer "F.SilkS" (type "Top Silk Screen"))
\t\t\t(layer "F.Paste" (type "Top Solder Paste"))
\t\t\t(layer "F.Mask" (type "Top Solder Mask") (thickness 0.01))
\t\t\t(layer "F.Cu" (type "copper") (thickness 0.035))
\t\t\t(layer "dielectric 1"
\t\t\t\t(type "core")
\t\t\t\t(thickness 1.51)
\t\t\t\t(material "FR4")
\t\t\t\t(epsilon_r 4.5)
\t\t\t\t(loss_tangent 0.02)
\t\t\t)
\t\t\t(layer "B.Cu" (type "copper") (thickness 0.035))
\t\t\t(layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.01))
\t\t\t(layer "B.Paste" (type "Bottom Solder Paste"))
\t\t\t(layer "B.SilkS" (type "Bottom Silk Screen"))
\t\t\t(copper_finish "None")
\t\t\t(dielectric_constraints no)
\t\t)
\t\t(pad_to_mask_clearance 0)
\t\t(allow_soldermask_bridges_in_footprints no)
\t\t(tenting (front yes) (back yes))
\t\t(covering (front no) (back no))
\t\t(plugging (front no) (back no))
\t\t(capping no)
\t\t(filling no)
\t\t(pcbplotparams
\t\t\t(layerselection 0x00000000_00000000_00000000_00000003)
\t\t\t(plot_on_all_layers_selection 0x00000000_00000000_00000000_00000000)
\t\t\t(disableapertmacros no)
\t\t\t(usegerberextensions no)
\t\t\t(usegerberattributes yes)
\t\t\t(usegerberadvancedattributes yes)
\t\t\t(creategerberjobfile yes)
\t\t\t(svgprecision 4)
\t\t\t(plotframeref no)
\t\t\t(mode 1)
\t\t\t(useauxorigin no)
\t\t\t(plot_black_and_white yes)
\t\t\t(subtractmaskfromsilk no)
\t\t\t(outputformat 1)
\t\t\t(mirror no)
\t\t\t(drillshape 1)
\t\t\t(scaleselection 1)
\t\t\t(outputdirectory "")
\t\t)
\t)'''


def generate_board(source: Path) -> tuple[str, dict]:
    pairs = read_pairs(source)
    if int(header_value(pairs, "$INSUNITS", 70)) != 4:
        raise ValueError("DXF is not declared in millimetres ($INSUNITS must be 4)")
    entities = parse_entities(pairs)
    counts = Counter(entity["type"] for entity in entities)
    contour_count = validate_closed_contours(entities)

    extents = [entity_extents(entity) for entity in entities]
    min_x = min(extent[0] for extent in extents)
    min_y = min(extent[1] for extent in extents)
    max_x = max(extent[2] for extent in extents)
    max_y = max(extent[3] for extent in extents)
    tx = 100.0 - (min_x + max_x) / 2
    ty = 100.0 + (min_y + max_y) / 2

    holes = [entity for entity in entities if entity["type"] == "CIRCLE"]
    edges = [entity for entity in entities if entity["type"] != "CIRCLE"]
    body = [board_header()]
    body.extend(footprint_for_hole(index, hole, tx, ty) for index, hole in enumerate(holes, 1))
    body.extend(edge_graphic(index, edge, tx, ty) for index, edge in enumerate(edges, 1))
    body.append("\t(embedded_fonts no)\n)")

    report = {
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "entity_counts": dict(counts),
        "closed_edge_contours": contour_count,
        "npth_holes": len(holes),
        "width_mm": max_x - min_x,
        "height_mm": max_y - min_y,
        "source_extents_mm": [min_x, min_y, max_x, max_y],
    }
    return "\n".join(body) + "\n", report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    board, report = generate_board(args.source)
    args.output.write_text(board, encoding="utf-8")
    for key, value in report.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
