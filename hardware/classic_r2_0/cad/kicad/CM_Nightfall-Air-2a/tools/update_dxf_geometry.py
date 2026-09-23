#!/usr/bin/env python3
"""Refresh only DXF-owned board graphics and mechanical holes on an edited PCB.

Run with KiCad's bundled Python.  Unlike prepare_classic_project.py this keeps
footprints (other than the four DXF holes), tracks, vias, zones and annotations.
"""

from __future__ import annotations

import argparse
import os
import re
import tempfile
from pathlib import Path

from prepare_classic_project import (
    format_mm,
    graphic_sexpr,
    group_float,
    parse_dxf_entities,
    sexpr_block_end,
    stable_uuid,
    transform,
    validate_closed_contours,
)


def board_graphics(outline: list[dict], silk: list[dict]) -> dict[str, str]:
    graphics = {}
    edge_index = 0
    for entity in outline:
        if entity["type"] == "CIRCLE" and 2 * group_float(entity, 40) <= 3:
            continue
        edge_index += 1
        uuid = stable_uuid(f'Edge.Cuts-{edge_index}-{entity["type"]}')
        graphics[uuid] = graphic_sexpr(edge_index, entity, "Edge.Cuts", 0.05)
    for index, entity in enumerate(silk, 1):
        uuid = stable_uuid(f'F.SilkS-{index}-{entity["type"]}')
        graphics[uuid] = graphic_sexpr(index, entity, "F.SilkS", 0.15)
    return graphics


def mechanical_holes(outline: list[dict]) -> list[tuple[float, float]]:
    return [
        transform((group_float(entity, 10), group_float(entity, 20)))
        for entity in outline
        if entity["type"] == "CIRCLE" and 2 * group_float(entity, 40) <= 3
    ]


def signature(block: str) -> tuple:
    shape = re.match(r"\((gr_\w+)", block.lstrip()).group(1)
    coords = tuple(
        (name, tuple(float(value) for value in pair.split()))
        for name, pair in re.findall(r"\((start|mid|end|center) ([^)]+)\)", block)
    )
    width = float(re.search(r"\(width ([^)]+)\)", block).group(1))
    layer = re.search(r'\(layer "([^"]+)"\)', block).group(1)
    return shape, coords, width, layer


def same_graphic(actual: str, expected: str) -> bool:
    actual_shape, actual_coords, actual_width, actual_layer = signature(actual)
    expected_shape, expected_coords, expected_width, expected_layer = signature(expected)
    if (actual_shape, actual_width, actual_layer) != (expected_shape, expected_width, expected_layer):
        return False
    a, e = dict(actual_coords), dict(expected_coords)
    near = lambda p, q: all(abs(x - y) <= 0.0001 for x, y in zip(p, q))
    if actual_shape in {"gr_line", "gr_arc"}:
        ends_match = (near(a["start"], e["start"]) and near(a["end"], e["end"])) or (
            near(a["start"], e["end"]) and near(a["end"], e["start"]))
        return ends_match and (actual_shape == "gr_line" or near(a["mid"], e["mid"]))
    return a.keys() == e.keys() and all(near(a[key], e[key]) for key in a)


def update(text: str, old_outline: list[dict], old_silk: list[dict],
           new_outline: list[dict], new_silk: list[dict]) -> str:
    old_graphics = board_graphics(old_outline, old_silk)
    new_graphics = board_graphics(new_outline, new_silk)
    old_holes = mechanical_holes(old_outline)
    new_holes = mechanical_holes(new_outline)
    if len(old_holes) != 4 or len(new_holes) != 4:
        raise ValueError("Expected exactly four DXF mechanical holes")
    if validate_closed_contours(new_outline) != validate_closed_contours(old_outline):
        raise ValueError("Outline contour count changed; inspect manually")

    seen_graphics: set[str] = set()
    seen_holes: set[str] = set()
    cursor = 0
    parts: list[str] = []
    while (start := text.find("\n\t(", cursor)) >= 0:
        start += 1
        parts.append(text[cursor:start])
        end = sexpr_block_end(text, start)
        block = text[start:end]
        token = re.match(r"\(([^\s()]+)", block.lstrip()).group(1)
        if token.startswith("gr_"):
            uuid_match = re.search(r'\(uuid "([^"]+)"\)', block)
            uuid = uuid_match.group(1) if uuid_match else None
            if uuid in old_graphics:
                if not same_graphic(block, old_graphics[uuid]):
                    raise ValueError(f"DXF graphic {uuid} was manually changed")
                seen_graphics.add(uuid)
                cursor = end
                continue
        if token == "footprint":
            ref_match = re.search(r'\(property "Reference" "(H[1-4])"', block)
            if ref_match:
                ref = ref_match.group(1)
                index = int(ref[1:]) - 1
                if '"Nightfall-Mechanical:DXF_NPTH_1.5mm"' not in block.splitlines()[0]:
                    raise ValueError(f"{ref} is no longer a DXF mechanical hole")
                at = re.search(r"\n\t\t\(at ([^()]+)\)", block)
                if not at:
                    raise ValueError(f"{ref}: top-level footprint position missing")
                xy = tuple(float(x) for x in at.group(1).split()[:2])
                if any(abs(a - b) > 1e-6 for a, b in zip(xy, old_holes[index])):
                    raise ValueError(f"{ref} moved manually from old DXF position: {xy}")
                replacement = f"\n\t\t(at {format_mm(new_holes[index][0])} {format_mm(new_holes[index][1])})"
                block = block[:at.start()] + replacement + block[at.end():]
                seen_holes.add(ref)
        parts.append(block)
        cursor = end
    parts.append(text[cursor:])

    if seen_graphics != set(old_graphics):
        raise ValueError(f"Missing DXF graphics: {sorted(set(old_graphics) - seen_graphics)}")
    if seen_holes != {f"H{i}" for i in range(1, 5)}:
        raise ValueError(f"Missing mechanical holes: {sorted(seen_holes)}")
    updated = "".join(parts)
    tail = "\t(embedded_fonts no)\n)"
    index = updated.rfind(tail)
    if index < 0:
        raise ValueError("PCB root tail not found")
    return updated[:index] + "\n".join(new_graphics.values()) + "\n" + updated[index:]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--board", required=True, type=Path)
    parser.add_argument("--old-outline", required=True, type=Path)
    parser.add_argument("--old-silk", required=True, type=Path)
    parser.add_argument("--new-outline", required=True, type=Path)
    parser.add_argument("--new-silk", required=True, type=Path)
    parser.add_argument("--apply", action="store_true", help="write PCB after validation")
    args = parser.parse_args()
    old_outline = parse_dxf_entities(args.old_outline)
    old_silk = parse_dxf_entities(args.old_silk)
    new_outline = parse_dxf_entities(args.new_outline)
    new_silk = parse_dxf_entities(args.new_silk)
    original = args.board.read_text(encoding="utf-8")
    updated = update(original, old_outline, old_silk, new_outline, new_silk)
    print(f"Validated: {len(board_graphics(old_outline, old_silk))} old graphics -> "
          f"{len(board_graphics(new_outline, new_silk))} new graphics; 4 mechanical holes")
    if not args.apply:
        print("Dry run; use --apply to update the PCB")
        return
    mode = args.board.stat().st_mode
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=args.board.parent,
                                     prefix=".dxf-update-", suffix=".kicad_pcb", delete=False) as out:
        temporary = Path(out.name)
        out.write(updated)
    try:
        os.chmod(temporary, mode)
        os.replace(temporary, args.board)
    finally:
        temporary.unlink(missing_ok=True)
    print(f"Updated {args.board}")


if __name__ == "__main__":
    main()
