#!/usr/bin/env python3
"""Prepare the Nightfall Air 2 KiCad artwork starting point.

Run this script with KiCad's bundled Python.  It deliberately rebuilds the
initial PCB from the mini-3a board, so do not run it after placement/routing
work has started unless that work is backed up.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import uuid
from collections import Counter
from pathlib import Path

import pcbnew


PROJECT_NAME = "CM_Nightfall-Air-2a_v0"
SOURCE_PROJECT_NAME = "HM_Nightfall-mini-3a_v0"
BOARD_CENTER_MM = (100.0, 100.0)
SUPPORTED_DXF_ENTITIES = {"LINE", "ARC", "CIRCLE"}
IGNORED_DXF_ENTITIES = {"MTEXT"}
UUID_NAMESPACE = uuid.UUID("a3165e44-02ab-42eb-8885-7d2c0db0e25d")
PHOTOTRANSISTORS = {
    "PT_FL0": "/SENSOR_FL",
    "PT_FR0": "/SENSOR_FR",
    "PT_L0": "/SENSOR_L",
    "PT_R0": "/SENSOR_R",
}
IR_EMITTERS = {
    "IR_LED_FL0": ("/Net-(IR_LED_FL0-C)", "/Net-(IR_LED_FL0-PadA)"),
    "IR_LED_FR0": ("/Net-(IR_LED_FR0-C)", "/Net-(IR_LED_FR0-PadA)"),
    "IR_LED_L0": ("/Net-(IR_LED_L0-C)", "/Net-(IR_LED_L0-PadA)"),
    "IR_LED_R0": ("/Net-(IR_LED_R0-C)", "/Net-(IR_LED_R0-PadA)"),
}


def stable_uuid(label: str) -> str:
    return str(uuid.uuid5(UUID_NAMESPACE, label))


def format_mm(value: float) -> str:
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return "0" if text in {"", "-0"} else text


def sexpr_block_end(text: str, start: int) -> int:
    depth = 0
    quoted = False
    escaped = False
    for index in range(start, len(text)):
        char = text[index]
        if quoted:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                quoted = False
        else:
            if char == '"':
                quoted = True
            elif char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if depth == 0:
                    return index + 1
    raise ValueError("Unterminated S-expression")


def find_symbol_instance(text: str, reference: str) -> tuple[int, int, str]:
    cursor = 0
    marker = "\n\t(symbol"
    while True:
        start = text.find(marker, cursor)
        if start < 0:
            raise ValueError(f"Schematic symbol instance {reference} not found")
        start += 1
        end = sexpr_block_end(text, start)
        block = text[start:end]
        if f'(property "Reference" "{reference}"' in block:
            return start, end, block
        cursor = end


def replace_symbol_property(block: str, property_name: str, new_value: str) -> str:
    pattern = rf'(\(property "{re.escape(property_name)}" ")[^"]*(")'
    updated, count = re.subn(pattern, rf"\g<1>{new_value}\g<2>", block, count=1)
    if count != 1:
        raise ValueError(f"Property {property_name} not found in symbol instance")
    return updated


def instance_pin_entries(reference: str, pin_numbers: list[str]) -> str:
    return "\n".join(
        f'\t\t(pin "{number}"\n\t\t\t(uuid "{stable_uuid(f"{PROJECT_NAME}-{reference}-pin-{number}")}")\n\t\t)'
        for number in pin_numbers
    )


def update_symbol_instance(
    text: str,
    reference: str,
    lib_id: str,
    value: str,
    footprint: str,
    datasheet: str,
    description: str,
    pin_numbers: list[str],
) -> str:
    start, end, block = find_symbol_instance(text, reference)
    block, count = re.subn(r'(\(lib_id ")[^"]*(")', rf"\g<1>{lib_id}\g<2>", block, count=1)
    if count != 1:
        raise ValueError(f"{reference}: lib_id not found")
    block = replace_symbol_property(block, "Value", value)
    block = replace_symbol_property(block, "Footprint", footprint)
    block = replace_symbol_property(block, "Datasheet", datasheet)
    block = replace_symbol_property(block, "Description", description)
    pin_start = block.find("\n\t\t(pin ")
    instance_start = block.find("\n\t\t(instances", pin_start)
    if pin_start < 0 or instance_start < 0:
        raise ValueError(f"{reference}: pin list not found")
    block = block[: pin_start + 1] + instance_pin_entries(reference, pin_numbers) + block[instance_start:]
    return text[:start] + block + text[end:]


def pin_effects(indent: str, visible: bool) -> str:
    size = "1.27 1.27" if visible else "0 0"
    return f'''{indent}(effects
{indent}\t(font
{indent}\t\t(size {size})
{indent}\t)
{indent})'''


def symbol_pin(
    indent: str,
    electrical_type: str,
    at: str,
    length: str,
    name: str,
    number: str,
    visible: bool,
) -> str:
    effects_indent = indent + "\t\t"
    return f'''{indent}(pin {electrical_type} line
{indent}\t(at {at})
{indent}\t(length {length})
{indent}\t(name "{name}"
{pin_effects(effects_indent, visible)}
{indent}\t)
{indent}\t(number "{number}"
{pin_effects(effects_indent, visible)}
{indent}\t)
{indent})'''


def transform_irlml_symbol_to_pmpb(block: str, embedded: bool) -> str:
    block = block.replace("Nightfall-Power:IRLML6344TRPBF", "Nightfall-Power:PMPB13XNE_115")
    block = block.replace("IRLML6344TRPBF", "PMPB13XNE_115")
    block = block.replace('property "Value" "PMPB13XNE_115"', 'property "Value" "PMPB13XNE,115"')
    block = re.sub(
        r'\(property "Footprint" "[^"]*"',
        '(property "Footprint" "Nightfall-Power:DFN2020MD-6_SOT1220_NEX"',
        block,
        count=1,
    )
    block = re.sub(
        r'\(property "Datasheet" "[^"]*"',
        '(property "Datasheet" "https://assets.nexperia.com/documents/data-sheet/PMPB13XNE.pdf"',
        block,
        count=1,
    )
    block = re.sub(
        r'\(property "Description" "[^"]*"',
        '(property "Description" "30 V N-channel MOSFET from CM_Nightfall-Air_v1; fan low-side PWM switch. D=1/2/5/6/7, G=3, S=4/8."',
        block,
        count=1,
    )

    child_marker = '(symbol "PMPB13XNE_115_1_1"'
    child_start = block.find(child_marker)
    if child_start < 0:
        raise ValueError("PMPB symbol pin unit not found")
    child_end = sexpr_block_end(block, child_start)
    child = block[child_start:child_end]
    pin_indent = "\t\t\t\t" if embedded else "\t\t\t"
    first_pin = child.find(f"\n{pin_indent}(pin ")
    if first_pin < 0:
        raise ValueError("PMPB source pin list not found")
    pins = [
        symbol_pin(pin_indent, "input", "-7.62 0 0", "2.54", "G", "3", True),
        symbol_pin(pin_indent, "passive", "0 -7.62 270", "5.08", "S", "4", True),
        symbol_pin(pin_indent, "passive", "0 -7.62 270", "5.08", "S", "8", False),
        symbol_pin(pin_indent, "passive", "0 7.62 90", "5.08", "D", "1", True),
        symbol_pin(pin_indent, "passive", "0 7.62 90", "5.08", "D", "2", False),
        symbol_pin(pin_indent, "passive", "0 7.62 90", "5.08", "D", "5", False),
        symbol_pin(pin_indent, "passive", "0 7.62 90", "5.08", "D", "6", False),
        symbol_pin(pin_indent, "passive", "0 7.62 90", "5.08", "D", "7", False),
    ]
    child = child[: first_pin + 1] + "\n".join(pins) + f"\n{pin_indent[:-1]})"
    return block[:child_start] + child + block[child_end:]


def insert_pmpb_lib_symbol(text: str, embedded: bool) -> str:
    new_name = "Nightfall-Power:PMPB13XNE_115" if embedded else "PMPB13XNE_115"
    if f'(symbol "{new_name}"' in text:
        return text
    old_name = "Nightfall-Power:IRLML6344TRPBF" if embedded else "IRLML6344TRPBF"
    indent = "\t\t" if embedded else "\t"
    marker = f'\n{indent}(symbol "{old_name}"'
    start = text.find(marker)
    if start < 0:
        raise ValueError(f"Source symbol {old_name} not found")
    start += 1
    end = sexpr_block_end(text, start)
    new_block = transform_irlml_symbol_to_pmpb(text[start:end], embedded)
    return text[:start] + new_block + "\n" + text[start:]


def insert_project_symbol(
    text: str,
    library_text: str,
    symbol_name: str,
    library_name: str,
) -> str:
    embedded_name = f"{library_name}:{symbol_name}"
    if f'(symbol "{embedded_name}"' in text:
        return text

    marker = f'\n\t(symbol "{symbol_name}"'
    source_start = library_text.find(marker)
    if source_start < 0:
        raise ValueError(f"Project library symbol {library_name}:{symbol_name} not found")
    source_start += 1
    source_end = sexpr_block_end(library_text, source_start)
    block = library_text[source_start:source_end]
    block = block.replace(f'(symbol "{symbol_name}"', f'(symbol "{embedded_name}"', 1)
    block = "\n".join("\t" + line for line in block.splitlines())

    lib_start = text.find("\n\t(lib_symbols")
    if lib_start < 0:
        raise ValueError("Embedded schematic library not found")
    lib_start += 1
    lib_end = sexpr_block_end(text, lib_start)
    return text[: lib_end - 1] + "\n" + block + "\n\t" + text[lib_end - 1 :]


def prepare_schematic(schematic: Path, symbol_library: Path, sensor_symbol_library: Path) -> None:
    text = schematic.read_text(encoding="utf-8")
    text = text.replace(SOURCE_PROJECT_NAME, PROJECT_NAME)
    text = re.sub(r'\(title "[^"]*"\)', '(title "Nightfall Air 2 classic / mini-3a circuit")', text, count=1)
    text = re.sub(r'\(date "[^"]*"\)', '(date "2026-08-23")', text, count=1)
    text = re.sub(
        r'\(comment 1 "[^"]*"\)',
        '(comment 1 "mini-3a circuit; Q2/Q3 and wall sensors/emitters from CM_Nightfall-Air_v1")',
        text,
        count=1,
    )
    text = text.replace("Package_SO:PowerPAK_SO-8_Single", "Nightfall-Power:PowerPAK_SO-8_FullPins")
    text = insert_pmpb_lib_symbol(text, embedded=True)
    library_text = symbol_library.read_text(encoding="utf-8")
    text = insert_project_symbol(text, library_text, "NJM2884U1-33", "Nightfall-Power")
    text = update_symbol_instance(
        text,
        "Q2",
        "Nightfall-Power:SI7135DP-T1-GE3",
        "SI7135DP-T1-GE3",
        "Nightfall-Power:PowerPAK_SO-8_FullPins",
        "https://www.vishay.com/docs/73197/si7135dp.pdf",
        "30 V P-channel PowerPAK MOSFET from CM_Nightfall-Air_v1; main power switch. S=1/2/3, G=4, D=5/6/7/8.",
        ["1", "2", "3", "4", "5", "6", "7", "8"],
    )
    text = update_symbol_instance(
        text,
        "Q3",
        "Nightfall-Power:PMPB13XNE_115",
        "PMPB13XNE,115",
        "Nightfall-Power:DFN2020MD-6_SOT1220_NEX",
        "https://assets.nexperia.com/documents/data-sheet/PMPB13XNE.pdf",
        "30 V N-channel MOSFET from CM_Nightfall-Air_v1; fan low-side PWM switch. D=1/2/5/6/7, G=3, S=4/8.",
        ["1", "2", "3", "4", "5", "6", "7", "8"],
    )
    text = update_symbol_instance(
        text,
        "U1",
        "Nightfall-Power:NJM2884U1-33",
        "NJM2884U1-33",
        "Nightfall-Power:NJM2884U1-33_SOT-89-5_Air_v1",
        "https://www.nisshinbo-microdevices.co.jp/en/pdf/datasheet/NJM2884_2884A_E.pdf",
        "NJM2884U1-33 3.3 V / 500 mA LDO in SOT-89-5. Pin 1=CONTROL, 2=GND, 3=NC, 4=VOUT, 5=VIN.",
        ["1", "2", "3", "4", "5"],
    )

    sensor_library_text = sensor_symbol_library.read_text(encoding="utf-8")
    text = insert_project_symbol(text, sensor_library_text, "ST-1KL3A", "Nightfall-Sensor")
    text = insert_project_symbol(text, sensor_library_text, "SFH4550", "Nightfall-Sensor")
    for reference in PHOTOTRANSISTORS:
        text = update_symbol_instance(
            text,
            reference,
            "Nightfall-Sensor:ST-1KL3A",
            "ST-1KL3A",
            "Nightfall-Sensor:ST-1KL3A_TO-18_2Pin_P2.54mm",
            "https://www.kodenshi.co.jp/top/wp-content/uploads/2022/03/ST-1KL3A.pdf",
            "ST-1KL3A two-lead NPN phototransistor in TO-18 metal can. Pin 1=Emitter, pin 2=Collector.",
            ["1", "2"],
        )
    for reference in IR_EMITTERS:
        text = update_symbol_instance(
            text,
            reference,
            "Nightfall-Sensor:SFH4550",
            "SFH4550",
            "Nightfall-Sensor:SFH4550_T1-3-4_P2.54mm",
            "https://look.ams-osram.com/m/7d214b223a9adb85/original/SFH-4550.pdf",
            "SFH 4550 high-power 850 nm infrared emitter, radial T1 3/4. Pin 1=K, pin 2=A.",
            ["1", "2"],
        )
    schematic.write_text(text, encoding="utf-8")

    library_text = library_text.replace("Package_SO:PowerPAK_SO-8_Single", "Nightfall-Power:PowerPAK_SO-8_FullPins")
    library_text = insert_pmpb_lib_symbol(library_text, embedded=False)
    symbol_library.write_text(library_text, encoding="utf-8")


def read_dxf_pairs(path: Path) -> list[tuple[int, str]]:
    raw = path.read_text(encoding="cp932", errors="strict").splitlines()
    if len(raw) % 2:
        raise ValueError(f"{path.name}: incomplete DXF group-code pair")
    return [(int(raw[index].strip()), raw[index + 1].strip()) for index in range(0, len(raw), 2)]


def dxf_header_value(pairs: list[tuple[int, str]], name: str, code: int) -> str:
    for index, pair in enumerate(pairs):
        if pair != (9, name):
            continue
        for candidate_code, value in pairs[index + 1 : index + 8]:
            if candidate_code == code:
                return value
            if candidate_code == 9:
                break
    raise ValueError(f"DXF header variable {name} is missing")


def parse_dxf_entities(path: Path) -> list[dict]:
    pairs = read_dxf_pairs(path)
    if int(dxf_header_value(pairs, "$INSUNITS", 70)) != 4:
        raise ValueError(f"{path.name}: $INSUNITS must declare millimetres")

    start = next(index for index, pair in enumerate(pairs) if pair == (2, "ENTITIES")) + 1
    entities: list[dict] = []
    current: dict | None = None
    for code, value in pairs[start:]:
        if (code, value) == (0, "ENDSEC"):
            if current is not None:
                entities.append(current)
            break
        if code == 0:
            if current is not None:
                entities.append(current)
            if value in IGNORED_DXF_ENTITIES:
                current = None
            elif value in SUPPORTED_DXF_ENTITIES:
                current = {"type": value, "groups": {}}
            else:
                raise ValueError(f"{path.name}: unsupported DXF entity {value}")
        elif current is not None:
            current["groups"].setdefault(code, []).append(value)
    return entities


def group_float(entity: dict, code: int) -> float:
    return float(entity["groups"][code][0])


def arc_point(entity: dict, angle_degrees: float) -> tuple[float, float]:
    angle = math.radians(angle_degrees)
    radius = group_float(entity, 40)
    return (
        group_float(entity, 10) + radius * math.cos(angle),
        group_float(entity, 20) + radius * math.sin(angle),
    )


def segment_endpoints(entity: dict) -> tuple[tuple[float, float], tuple[float, float]]:
    if entity["type"] == "LINE":
        return (
            (group_float(entity, 10), group_float(entity, 20)),
            (group_float(entity, 11), group_float(entity, 21)),
        )
    if entity["type"] == "ARC":
        return arc_point(entity, group_float(entity, 50)), arc_point(entity, group_float(entity, 51))
    raise ValueError("Circle has no segment endpoints")


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


def transform(point: tuple[float, float]) -> tuple[float, float]:
    return BOARD_CENTER_MM[0] + point[0], BOARD_CENTER_MM[1] - point[1]


def vector_mm(point: tuple[float, float]) -> pcbnew.VECTOR2I:
    return pcbnew.VECTOR2I(pcbnew.FromMM(point[0]), pcbnew.FromMM(point[1]))


def add_line(board: pcbnew.BOARD, start: tuple[float, float], end: tuple[float, float], layer: int, width: float) -> None:
    shape = pcbnew.PCB_SHAPE(board)
    shape.SetShape(pcbnew.SHAPE_T_SEGMENT)
    shape.SetStart(vector_mm(transform(start)))
    shape.SetEnd(vector_mm(transform(end)))
    shape.SetLayer(layer)
    shape.SetWidth(pcbnew.FromMM(width))
    board.Add(shape)


def add_arc(board: pcbnew.BOARD, entity: dict, layer: int, width: float) -> None:
    start_angle = group_float(entity, 50)
    end_angle = group_float(entity, 51)
    sweep = (end_angle - start_angle) % 360.0
    start = vector_mm(transform(arc_point(entity, start_angle)))
    mid = vector_mm(transform(arc_point(entity, start_angle + sweep / 2.0)))
    end = vector_mm(transform(arc_point(entity, end_angle)))
    shape = pcbnew.PCB_SHAPE(board)
    shape.SetShape(pcbnew.SHAPE_T_ARC)
    shape.SetArcGeometry(start, mid, end)
    shape.SetLayer(layer)
    shape.SetWidth(pcbnew.FromMM(width))
    board.Add(shape)


def add_circle(board: pcbnew.BOARD, center: tuple[float, float], radius: float, layer: int, width: float) -> None:
    transformed_center = transform(center)
    shape = pcbnew.PCB_SHAPE(board)
    shape.SetShape(pcbnew.SHAPE_T_CIRCLE)
    shape.SetCenter(vector_mm(transformed_center))
    shape.SetEnd(vector_mm((transformed_center[0] + radius, transformed_center[1])))
    shape.SetLayer(layer)
    shape.SetWidth(pcbnew.FromMM(width))
    board.Add(shape)


def add_dxf_graphics(
    board: pcbnew.BOARD,
    outline: list[dict],
    silk: list[dict],
    hole_footprints: list[pcbnew.FOOTPRINT],
) -> dict:
    contour_count = validate_closed_contours(outline)
    npth_holes = 0
    cutout_circles = 0

    for entity in outline:
        if entity["type"] == "LINE":
            add_line(board, *segment_endpoints(entity), pcbnew.Edge_Cuts, 0.05)
        elif entity["type"] == "ARC":
            add_arc(board, entity, pcbnew.Edge_Cuts, 0.05)
        else:
            diameter = 2.0 * group_float(entity, 40)
            center = (group_float(entity, 10), group_float(entity, 20))
            if diameter <= 3.0:
                npth_holes += 1
                footprint = hole_footprints[npth_holes - 1]
                footprint.SetReference(f"H{npth_holes}")
                footprint.SetValue("DXF_NPTH_1.5mm")
                footprint.SetBoardOnly(True)
                footprint.SetPosition(vector_mm(transform(center)))
                board.Add(footprint)
            else:
                cutout_circles += 1
                add_circle(board, center, diameter / 2.0, pcbnew.Edge_Cuts, 0.05)

    for entity in silk:
        if entity["type"] == "LINE":
            add_line(board, *segment_endpoints(entity), pcbnew.F_SilkS, 0.15)
        elif entity["type"] == "ARC":
            add_arc(board, entity, pcbnew.F_SilkS, 0.15)
        else:
            add_circle(
                board,
                (group_float(entity, 10), group_float(entity, 20)),
                group_float(entity, 40),
                pcbnew.F_SilkS,
                0.15,
            )

    return {
        "outline_entities": len(outline),
        "outline_entity_types": dict(Counter(entity["type"] for entity in outline)),
        "closed_segment_contours": contour_count,
        "edge_cutout_circles": cutout_circles,
        "npth_holes": npth_holes,
        "silk_entities": len(silk),
        "silk_entity_types": dict(Counter(entity["type"] for entity in silk)),
    }


def graphic_sexpr(index: int, entity: dict, layer: str, width: float) -> str:
    graphic_uuid = stable_uuid(f"{layer}-{index}-{entity['type']}")
    if entity["type"] == "LINE":
        start, end = segment_endpoints(entity)
        start = transform(start)
        end = transform(end)
        return f'''\t(gr_line
\t\t(start {format_mm(start[0])} {format_mm(start[1])})
\t\t(end {format_mm(end[0])} {format_mm(end[1])})
\t\t(stroke (width {format_mm(width)}) (type default))
\t\t(layer "{layer}")
\t\t(uuid "{graphic_uuid}")
\t)'''
    if entity["type"] == "ARC":
        start_angle = group_float(entity, 50)
        end_angle = group_float(entity, 51)
        sweep = (end_angle - start_angle) % 360.0
        start = transform(arc_point(entity, start_angle))
        mid = transform(arc_point(entity, start_angle + sweep / 2.0))
        end = transform(arc_point(entity, end_angle))
        return f'''\t(gr_arc
\t\t(start {format_mm(start[0])} {format_mm(start[1])})
\t\t(mid {format_mm(mid[0])} {format_mm(mid[1])})
\t\t(end {format_mm(end[0])} {format_mm(end[1])})
\t\t(stroke (width {format_mm(width)}) (type default))
\t\t(layer "{layer}")
\t\t(uuid "{graphic_uuid}")
\t)'''
    center = transform((group_float(entity, 10), group_float(entity, 20)))
    radius = group_float(entity, 40)
    return f'''\t(gr_circle
\t\t(center {format_mm(center[0])} {format_mm(center[1])})
\t\t(end {format_mm(center[0] + radius)} {format_mm(center[1])})
\t\t(stroke (width {format_mm(width)}) (type default))
\t\t(fill none)
\t\t(layer "{layer}")
\t\t(uuid "{graphic_uuid}")
\t)'''


def rewrite_board_layout(board_path: Path, outline: list[dict], silk: list[dict]) -> None:
    text = board_path.read_text(encoding="utf-8")
    removable = {
        "arc",
        "dimension",
        "gr_arc",
        "gr_bbox",
        "gr_circle",
        "gr_curve",
        "gr_line",
        "gr_poly",
        "gr_rect",
        "gr_text",
        "group",
        "segment",
        "target",
        "via",
        "zone",
    }
    cursor = 0
    parts: list[str] = []
    marker = "\n\t("
    while True:
        start = text.find(marker, cursor)
        if start < 0:
            parts.append(text[cursor:])
            break
        start += 1
        parts.append(text[cursor:start])
        end = sexpr_block_end(text, start)
        block = text[start:end]
        match = re.match(r"\(([^\s()]+)", block.lstrip())
        token = match.group(1) if match is not None else ""
        expected_values = {
            "U1": "NJM2884U1-33",
            "Q2": "SI7135DP-T1-GE3",
            "Q3": "PMPB13XNE,115",
            **{reference: "ST-1KL3A" for reference in PHOTOTRANSISTORS},
            **{reference: "SFH4550" for reference in IR_EMITTERS},
        }
        obsolete_replacement = token == "footprint" and any(
            f'(property "Reference" "{reference}"' in block
            and f'(property "Value" "{value}"' not in block
            for reference, value in expected_values.items()
        )
        if token not in removable and not obsolete_replacement:
            parts.append(block)
        cursor = end
    text = "".join(parts)

    graphics: list[str] = []
    edge_index = 0
    for entity in outline:
        if entity["type"] == "CIRCLE" and 2.0 * group_float(entity, 40) <= 3.0:
            continue
        edge_index += 1
        graphics.append(graphic_sexpr(edge_index, entity, "Edge.Cuts", 0.05))
    for index, entity in enumerate(silk, 1):
        graphics.append(graphic_sexpr(index, entity, "F.SilkS", 0.15))

    insertion = "\n".join(graphics) + "\n"
    root_tail = "\t(embedded_fonts no)\n)"
    tail_index = text.rfind(root_tail)
    if tail_index < 0:
        raise ValueError("Board root tail not found")
    board_path.write_text(text[:tail_index] + insertion + text[tail_index:], encoding="utf-8")


def natural_reference_key(reference: str) -> tuple:
    parts = re.split(r"(\d+)", reference)
    return tuple(int(part) if part.isdigit() else part for part in parts)


def remove_layout(board: pcbnew.BOARD) -> None:
    tracks = list(board.GetTracks())
    zones = list(board.Zones())
    drawings = list(board.GetDrawings())
    groups = list(board.Groups())
    for item in tracks:
        board.Remove(item)
    for zone in zones:
        board.Remove(zone)
    for drawing in drawings:
        board.Remove(drawing)
    for group in groups:
        board.Remove(group)


def set_pad_nets(
    board: pcbnew.BOARD,
    footprint: pcbnew.FOOTPRINT,
    mapping: dict[str, str | None],
) -> None:
    for pad in footprint.Pads():
        if pad.GetNumber() not in mapping:
            raise ValueError(f"{footprint.GetReference()}: no net mapping for pad {pad.GetNumber()}")
        net_name = mapping[pad.GetNumber()]
        if net_name is None:
            pad.SetNetCode(0)
            continue
        net = board.FindNet(net_name)
        if net is None:
            raise ValueError(f"{footprint.GetReference()}: board net not found: {net_name}")
        pad.SetNet(net)


def configure_replacement_footprint(
    board: pcbnew.BOARD,
    reference: str,
    new: pcbnew.FOOTPRINT,
    value: str,
    pad_nets: dict[str, str | None],
) -> pcbnew.FOOTPRINT:
    old = board.FindFootprintByReference(reference)
    if old is None:
        raise ValueError(f"Footprint {reference} not found")
    new.SetReference(reference)
    new.SetValue(value)
    new.SetPath(old.GetPath())
    new.SetPosition(old.GetPosition())
    new.SetOrientation(old.GetOrientation())
    new.SetLayer(old.GetLayer())
    new.SetSheetname("/")
    new.SetSheetfile(f"{PROJECT_NAME}.kicad_sch")
    set_pad_nets(board, new, pad_nets)
    return old


def track_signature(board: pcbnew.BOARD) -> list[tuple]:
    """Return the routed copper state in a form that survives save/reload."""
    signature = []
    for item in board.GetTracks():
        start = item.GetStart()
        end = item.GetEnd()
        width = item.GetWidth(pcbnew.F_Cu) if isinstance(item, pcbnew.PCB_VIA) else item.GetWidth()
        row = (
            item.__class__.__name__,
            start.x,
            start.y,
            end.x,
            end.y,
            item.GetLayer(),
            width,
            item.GetNetCode(),
        )
        if isinstance(item, pcbnew.PCB_VIA):
            row += (item.GetDrillValue(), item.TopLayer(), item.BottomLayer())
        signature.append(row)
    return sorted(signature)


def replace_u1_in_routed_board(board_path: Path, footprint_library: Path) -> dict:
    """Replace only U1, preserving all routed copper and placement metadata."""
    board = pcbnew.LoadBoard(str(board_path))
    old = board.FindFootprintByReference("U1")
    if old is None:
        raise ValueError("Footprint U1 not found")

    track_state = track_signature(board)
    footprint_count = len(list(board.GetFootprints()))
    position = old.GetPosition()
    orientation = old.GetOrientation()
    layer = old.GetLayer()
    locked = old.IsLocked()
    path = old.GetPath()
    sheetname = old.GetSheetname()
    sheetfile = old.GetSheetfile()

    new = pcbnew.FootprintLoad(
        str(footprint_library), "NJM2884U1-33_SOT-89-5_Air_v1"
    )
    if new is None:
        raise RuntimeError("Unable to load NJM2884 U1 footprint")
    new.SetFPID(
        pcbnew.LIB_ID("Nightfall-Power", "NJM2884U1-33_SOT-89-5_Air_v1")
    )
    new.SetReference("U1")
    new.SetValue("NJM2884U1-33")
    new.SetPath(path)
    new.SetPosition(position)
    new.SetOrientation(orientation)
    new.SetLayer(layer)
    new.SetLocked(locked)
    new.SetSheetname(sheetname)
    new.SetSheetfile(sheetfile)
    set_pad_nets(
        board,
        new,
        {"1": "+5V", "2": "GND", "3": None, "4": "+3V3", "5": "+5V"},
    )

    board.Remove(old)
    board.Add(new)
    if len(list(board.GetFootprints())) != footprint_count:
        raise RuntimeError("Footprint count changed while replacing U1")
    if track_signature(board) != track_state:
        raise RuntimeError("Routed copper changed while replacing U1")

    temporary = board_path.with_name(f".{board_path.stem}.njm.tmp.kicad_pcb")
    pcbnew.SaveBoard(str(temporary), board)
    check = pcbnew.LoadBoard(str(temporary))
    check_u1 = check.FindFootprintByReference("U1")
    expected_nets = {"1": "+5V", "2": "GND", "3": "", "4": "+3V3", "5": "+5V"}
    actual_nets = {pad.GetNumber(): pad.GetNetname() for pad in check_u1.Pads()}
    if actual_nets != expected_nets:
        raise RuntimeError(f"U1 pad nets after reload: {actual_nets}")
    if (
        check_u1.GetPosition().x != position.x
        or check_u1.GetPosition().y != position.y
        or check_u1.GetOrientationDegrees() != orientation.AsDegrees()
    ):
        raise RuntimeError("U1 placement changed while replacing its footprint")
    if track_signature(check) != track_state:
        raise RuntimeError("Routed copper changed after board save/reload")
    os.replace(temporary, board_path)
    return {
        "footprints": footprint_count,
        "tracks": len(track_state),
        "position_mm": (pcbnew.ToMM(position.x), pcbnew.ToMM(position.y)),
        "orientation_deg": orientation.AsDegrees(),
        "pad_nets": actual_nets,
    }


def stage_footprints(board: pcbnew.BOARD) -> None:
    footprints = sorted(list(board.GetFootprints()), key=lambda item: natural_reference_key(item.GetReference()))
    columns = 9
    start_x, start_y = 140.0, 18.0
    pitch_x, pitch_y = 16.0, 16.0
    nonprint_silk_layer = board.GetLayerID("User.5")
    for index, footprint in enumerate(footprints):
        if footprint.GetReference().startswith("H") and footprint.IsBoardOnly():
            continue
        footprint.SetPosition(vector_mm((start_x + pitch_x * (index % columns), start_y + pitch_y * (index // columns))))
        footprint.SetOrientationDegrees(0.0)
        footprint.SetLayer(pcbnew.F_Cu)
        footprint.SetLocked(False)
        footprint.SetSheetname("/")
        footprint.SetSheetfile(f"{PROJECT_NAME}.kicad_sch")
        for graphic in footprint.GraphicalItems():
            if graphic.GetLayer() == pcbnew.F_SilkS:
                graphic.SetLayer(nonprint_silk_layer)
        for field in footprint.GetFields():
            if field.GetLayer() == pcbnew.F_SilkS:
                field.SetLayer(nonprint_silk_layer)


def prepare_board(base_board: Path, output_board: Path, outline_dxf: Path, silk_dxf: Path) -> dict:
    board = pcbnew.LoadBoard(str(base_board))
    project_dir = output_board.parent
    outline = parse_dxf_entities(outline_dxf)
    silk = parse_dxf_entities(silk_dxf)
    power_library = project_dir / "Nightfall-Power.pretty"
    mechanical_library = project_dir / "Nightfall-Mechanical.pretty"
    sensor_library = project_dir / "Nightfall-Sensor.pretty"
    q2_footprint = pcbnew.FootprintLoad(str(power_library), "PowerPAK_SO-8_FullPins")
    q3_footprint = pcbnew.FootprintLoad(str(power_library), "DFN2020MD-6_SOT1220_NEX")
    u1_footprint = pcbnew.FootprintLoad(
        str(power_library), "NJM2884U1-33_SOT-89-5_Air_v1"
    )
    sensor_footprints = {
        **{
            reference: pcbnew.FootprintLoad(
                str(sensor_library), "ST-1KL3A_TO-18_2Pin_P2.54mm"
            )
            for reference in PHOTOTRANSISTORS
        },
        **{
            reference: pcbnew.FootprintLoad(
                str(sensor_library), "SFH4550_T1-3-4_P2.54mm"
            )
            for reference in IR_EMITTERS
        },
    }
    hole_count = sum(
        1
        for entity in outline
        if entity["type"] == "CIRCLE" and 2.0 * group_float(entity, 40) <= 3.0
    )
    hole_footprints = [
        pcbnew.FootprintLoad(str(mechanical_library), "DXF_NPTH_1.5mm") for _ in range(hole_count)
    ]
    if (
        u1_footprint is None
        or q2_footprint is None
        or q3_footprint is None
        or any(item is None for item in sensor_footprints.values())
        or any(item is None for item in hole_footprints)
    ):
        raise RuntimeError("Unable to preload one or more project-local footprints")
    q2_footprint.SetFPID(pcbnew.LIB_ID("Nightfall-Power", "PowerPAK_SO-8_FullPins"))
    q3_footprint.SetFPID(pcbnew.LIB_ID("Nightfall-Power", "DFN2020MD-6_SOT1220_NEX"))
    u1_footprint.SetFPID(
        pcbnew.LIB_ID("Nightfall-Power", "NJM2884U1-33_SOT-89-5_Air_v1")
    )
    for reference, footprint in sensor_footprints.items():
        library_name = (
            "ST-1KL3A_TO-18_2Pin_P2.54mm"
            if reference in PHOTOTRANSISTORS
            else "SFH4550_T1-3-4_P2.54mm"
        )
        footprint.SetFPID(pcbnew.LIB_ID("Nightfall-Sensor", library_name))
    for footprint in hole_footprints:
        footprint.SetFPID(pcbnew.LIB_ID("Nightfall-Mechanical", "DXF_NPTH_1.5mm"))

    stage_footprints(board)

    configure_replacement_footprint(
        board,
        "U1",
        u1_footprint,
        "NJM2884U1-33",
        {"1": "+5V", "2": "GND", "3": None, "4": "+3V3", "5": "+5V"},
    )
    old_q2 = configure_replacement_footprint(
        board,
        "Q2",
        q2_footprint,
        "SI7135DP-T1-GE3",
        {
            "1": "VBAT_RAW",
            "2": "VBAT_RAW",
            "3": "VBAT_RAW",
            "4": "/PWR_GATE_INTERNAL",
            "5": "VBAT_SW",
            "6": "VBAT_SW",
            "7": "VBAT_SW",
            "8": "VBAT_SW",
        },
    )
    old_q3 = configure_replacement_footprint(
        board,
        "Q3",
        q3_footprint,
        "PMPB13XNE,115",
        {
            "1": "/FAN_NEG_INTERNAL",
            "2": "/FAN_NEG_INTERNAL",
            "3": "/FAN_PWM",
            "4": "GND2",
            "5": "/FAN_NEG_INTERNAL",
            "6": "/FAN_NEG_INTERNAL",
            "7": "/FAN_NEG_INTERNAL",
            "8": "GND2",
        },
    )
    for reference, sensor_net in PHOTOTRANSISTORS.items():
        configure_replacement_footprint(
            board,
            reference,
            sensor_footprints[reference],
            "ST-1KL3A",
            {"1": sensor_net, "2": "+3V3"},
        )
    for reference, (cathode_net, anode_net) in IR_EMITTERS.items():
        configure_replacement_footprint(
            board,
            reference,
            sensor_footprints[reference],
            "SFH4550",
            {"1": cathode_net, "2": anode_net},
        )
    npth_index = 0
    for entity in outline:
        if entity["type"] != "CIRCLE" or 2.0 * group_float(entity, 40) > 3.0:
            continue
        footprint = hole_footprints[npth_index]
        npth_index += 1
        footprint.SetReference(f"H{npth_index}")
        footprint.SetValue("DXF_NPTH_1.5mm")
        footprint.SetBoardOnly(True)
        footprint.SetPosition(vector_mm(transform((group_float(entity, 10), group_float(entity, 20)))))
        board.Add(footprint)
    board.Add(q2_footprint)
    board.Add(q3_footprint)
    board.Add(u1_footprint)
    for footprint in sensor_footprints.values():
        board.Add(footprint)
    pcbnew.SaveBoard(str(output_board), board)
    rewrite_board_layout(output_board, outline, silk)
    report = {
        "outline_entities": len(outline),
        "outline_entity_types": dict(Counter(entity["type"] for entity in outline)),
        "closed_segment_contours": validate_closed_contours(outline),
        "edge_cutout_circles": sum(
            1 for entity in outline if entity["type"] == "CIRCLE" and 2.0 * group_float(entity, 40) > 3.0
        ),
        "npth_holes": hole_count,
        "silk_entities": len(silk),
        "silk_entity_types": dict(Counter(entity["type"] for entity in silk)),
    }
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-board", type=Path, required=True)
    parser.add_argument("--schematic", type=Path, required=True)
    parser.add_argument("--symbol-library", type=Path, required=True)
    parser.add_argument("--sensor-symbol-library", type=Path, required=True)
    parser.add_argument("--outline", type=Path, required=True)
    parser.add_argument("--silk", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    prepare_schematic(args.schematic, args.symbol_library, args.sensor_symbol_library)
    report = prepare_board(args.base_board, args.output, args.outline, args.silk)
    for key, value in report.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
