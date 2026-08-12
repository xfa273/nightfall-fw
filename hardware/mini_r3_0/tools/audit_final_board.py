#!/usr/bin/env python3
"""Read-only release audit for the Nightfall mini_r3 four-layer PCB.

The candidate board is never edited.  A temporary project is assembled from
the candidate PCB plus the canonical .kicad_pro/.kicad_dru/.kicad_sch files,
then KiCad CLI refills and saves only that temporary copy.  The resulting DRC,
schematic netlist, connectivity, geometry, and fabrication checks are written
to audit.json and audit.md.

Run this script with KiCad's bundled Python (the shell wrapper does that).
"""

from __future__ import annotations

import argparse
import collections
import datetime as dt
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import pcbnew


PROJECT_STEM = "HM_Nightfall-mini-3a_v0"
KICAD_CLI = Path("/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli")
F_CU = int(pcbnew.F_Cu)
B_CU = int(pcbnew.B_Cu)
IN1_CU = int(pcbnew.In1_Cu)
IN2_CU = int(pcbnew.In2_Cu)
REQUIRED_COPPER_LAYER_NAMES = ("F.Cu", "In1.Cu", "In2.Cu", "B.Cu")

ZONE_NAMES = (
    "LOGIC_GND_F",
    "LOGIC_GND_B",
    "POWER_GND2_F",
    "POWER_GND2_B",
    "LOGIC_GND_IN1",
    "POWER_GND2_IN1",
    "LOGIC_GND_IN2",
    "POWER_GND2_IN2",
)

IN1_ALLOWED_ZONE_NETS = frozenset(("GND", "GND2"))
IN2_ALWAYS_ALLOWED_NETS = frozenset(("+3V3", "GND", "GND2"))
IN2_FORBIDDEN_NETS = frozenset(("+5V",))
IN2_FORBIDDEN_NETCLASSES = frozenset(("HighCurrent", "SwitchNode"))

# This is the reviewed post-schematic-swap contract.  A final board may move
# the footprints, but it must not change these logical assignments.
DRIVER_CONTRACT: Mapping[Tuple[str, str], str] = {
    ("U2", "1"): "/MOTOR_L_OUT1",
    ("U2", "10"): "/MOTOR_L_OUT1",
    ("U2", "3"): "/MOTOR_L_OUT2",
    ("U2", "8"): "/MOTOR_L_OUT2",
    ("U2", "5"): "/MOTOR_L_DIR",
    ("U2", "6"): "/MOTOR_L_SR",
    ("U2", "13"): "/MOTOR_L_PWM",
    ("U3", "1"): "/MOTOR_R_OUT1",
    ("U3", "10"): "/MOTOR_R_OUT1",
    ("U3", "3"): "/MOTOR_R_OUT2",
    ("U3", "8"): "/MOTOR_R_OUT2",
    ("U3", "5"): "/MOTOR_R_DIR",
    ("U3", "6"): "/MOTOR_R_SR",
    ("U3", "13"): "/MOTOR_R_PWM",
    ("R35", "2"): "/MOTOR_L_SR",
    ("R34", "2"): "/MOTOR_R_SR",
    ("TP3", "TP"): "/MOTOR_L_OUT1",
    ("TP4", "TP"): "/MOTOR_L_OUT2",
    ("TP1", "TP"): "/MOTOR_R_OUT1",
    ("TP2", "TP"): "/MOTOR_R_OUT2",
    ("U5", "14"): "/MOTOR_L_DIR",
    ("U5", "15"): "/MOTOR_L_PWM",
    ("U5", "19"): "/MOTOR_R_DIR",
    ("U5", "21"): "/MOTOR_R_PWM",
}

R0_CONTRACT: Mapping[Tuple[str, str], str] = {
    ("R0", "1"): "GND2",
    ("R0", "2"): "GND",
}

# JLCPCB public capability limits used by this audit.  The local project is
# intentionally more conservative for the normal route/via choices.
JLC = {
    "two_oz_trace_width_min_mm": 0.16,
    "two_oz_spacing_min_mm": 0.16,
    "routed_edge_copper_clearance_min_mm": 0.20,
    "via_hard_drill_min_mm": 0.15,
    "via_hard_diameter_min_mm": 0.25,
    "via_preferred_drill_min_mm": 0.20,
    "project_toe_via_diameter_mm": 0.40,
    "project_toe_via_drill_mm": 0.20,
    "project_normal_width_floor_mm": 0.20,
    "inner_layer_copper_coverage_min_percent": 25.0,
    "sources": [
        "https://jlcpcb.com/help/article/jlcpcb-copper-weight",
        "https://jlcpcb.com/capabilities/pcb-capabilities/",
        "https://jlcpcb.com/help/article/inner-layer-copper-coverage-pcb",
    ],
}


class DSU:
    def __init__(self, count: int) -> None:
        self.parent = list(range(count))
        self.rank = [0] * count

    def find(self, item: int) -> int:
        root = item
        while self.parent[root] != root:
            root = self.parent[root]
        while self.parent[item] != item:
            parent = self.parent[item]
            self.parent[item] = root
            item = parent
        return root

    def union(self, left: int, right: int) -> None:
        left_root = self.find(left)
        right_root = self.find(right)
        if left_root == right_root:
            return
        if self.rank[left_root] < self.rank[right_root]:
            left_root, right_root = right_root, left_root
        self.parent[right_root] = left_root
        if self.rank[left_root] == self.rank[right_root]:
            self.rank[left_root] += 1


def mm(value_iu: int) -> float:
    return float(pcbnew.ToMM(value_iu))


def rounded(value: float, digits: int = 6) -> float:
    return round(float(value), digits)


def point_mm(item: Any) -> List[float]:
    point = item.GetPosition()
    return [rounded(mm(point.x)), rounded(mm(point.y))]


def enabled_copper_layers(board: Any) -> List[int]:
    layers = [
        int(layer)
        for layer in board.GetEnabledLayers().Seq()
        if str(board.GetLayerName(int(layer))).endswith(".Cu")
    ]
    order = {name: index for index, name in enumerate(REQUIRED_COPPER_LAYER_NAMES)}
    return sorted(
        layers,
        key=lambda layer: (
            order.get(str(board.GetLayerName(layer)), len(order)),
            str(board.GetLayerName(layer)),
        ),
    )


def enabled_copper_layer_names(board: Any) -> List[str]:
    names = [str(board.GetLayerName(layer)) for layer in enabled_copper_layers(board)]
    order = {name: index for index, name in enumerate(REQUIRED_COPPER_LAYER_NAMES)}
    return sorted(names, key=lambda name: (order.get(name, len(order)), name))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def item_uuid(item: Any) -> str:
    return str(item.m_Uuid.AsString())


def pad_label(pad: Any) -> str:
    parent = pad.GetParentFootprint()
    return "{}.{}".format(parent.GetReference(), pad.GetNumber())


def copy_exact_project(project_dir: Path, candidate: Path, work_dir: Path) -> Dict[str, Any]:
    required = [
        project_dir / (PROJECT_STEM + ".kicad_pro"),
        project_dir / (PROJECT_STEM + ".kicad_dru"),
        project_dir / (PROJECT_STEM + ".kicad_sch"),
    ]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise RuntimeError("missing canonical project files: " + ", ".join(missing))

    copied: Dict[str, Dict[str, str]] = {}
    for source in required:
        destination = work_dir / source.name
        shutil.copy2(str(source), str(destination))
        copied[source.name] = {
            "source": str(source),
            "source_sha256": sha256(source),
            "copy_sha256": sha256(destination),
        }

    for filename in (
        "fp-lib-table",
        "sym-lib-table",
        PROJECT_STEM + "-power.kicad_sym",
    ):
        source = project_dir / filename
        if source.is_file():
            shutil.copy2(str(source), str(work_dir / filename))

    for directory in project_dir.glob("*.pretty"):
        if directory.is_dir():
            shutil.copytree(str(directory), str(work_dir / directory.name))

    board_copy = work_dir / (PROJECT_STEM + ".kicad_pcb")
    shutil.copy2(str(candidate), str(board_copy))
    return {
        "canonical_files": copied,
        "candidate": str(candidate),
        "candidate_sha256_before_refill": sha256(candidate),
        "temporary_board": str(board_copy),
        "temporary_board_sha256_before_refill": sha256(board_copy),
    }


def run_command(command: Sequence[str], cwd: Path, stdout_path: Path, stderr_path: Path) -> int:
    with stdout_path.open("w", encoding="utf-8") as stdout, stderr_path.open(
        "w", encoding="utf-8"
    ) as stderr:
        completed = subprocess.run(
            list(command),
            cwd=str(cwd),
            stdout=stdout,
            stderr=stderr,
            check=False,
            text=True,
        )
    return int(completed.returncode)


def run_kicad(work_dir: Path) -> Dict[str, Any]:
    if not KICAD_CLI.is_file():
        raise RuntimeError("KiCad CLI not found: {}".format(KICAD_CLI))

    board = work_dir / (PROJECT_STEM + ".kicad_pcb")
    schematic = work_dir / (PROJECT_STEM + ".kicad_sch")
    drc_json = work_dir / "drc.json"
    netlist_xml = work_dir / "schematic-netlist.xml"

    drc_command = [
        str(KICAD_CLI),
        "pcb",
        "drc",
        "--all-track-errors",
        "--schematic-parity",
        "--severity-all",
        "--refill-zones",
        "--save-board",
        "--units",
        "mm",
        "--format",
        "json",
        "--output",
        str(drc_json),
        str(board),
    ]
    drc_rc = run_command(
        drc_command,
        work_dir,
        work_dir / "drc.stdout.log",
        work_dir / "drc.stderr.log",
    )

    netlist_command = [
        str(KICAD_CLI),
        "sch",
        "export",
        "netlist",
        "--format",
        "kicadxml",
        "--output",
        str(netlist_xml),
        str(schematic),
    ]
    netlist_rc = run_command(
        netlist_command,
        work_dir,
        work_dir / "netlist.stdout.log",
        work_dir / "netlist.stderr.log",
    )

    if drc_rc != 0 or not drc_json.is_file():
        raise RuntimeError(
            "KiCad DRC failed (exit {}); inspect {}".format(
                drc_rc, work_dir / "drc.stderr.log"
            )
        )
    if netlist_rc != 0 or not netlist_xml.is_file():
        raise RuntimeError(
            "KiCad netlist export failed (exit {}); inspect {}".format(
                netlist_rc, work_dir / "netlist.stderr.log"
            )
        )

    with drc_json.open("r", encoding="utf-8") as handle:
        drc = json.load(handle)
    return {
        "kicad_cli": str(KICAD_CLI),
        "kicad_version": subprocess.check_output(
            [str(KICAD_CLI), "--version"], text=True
        ).strip(),
        "drc_exit_code": drc_rc,
        "netlist_exit_code": netlist_rc,
        "drc_command": drc_command,
        "netlist_command": netlist_command,
        "drc": drc,
        "drc_json": str(drc_json),
        "netlist_xml": str(netlist_xml),
        "refilled_board_sha256": sha256(board),
    }


def board_pad_map(board: Any) -> Dict[Tuple[str, str], str]:
    result: Dict[Tuple[str, str], str] = {}
    for footprint in board.GetFootprints():
        reference = str(footprint.GetReference())
        for pad in footprint.Pads():
            result[(reference, str(pad.GetNumber()))] = str(pad.GetNetname())
    return result


def schematic_pad_map(netlist_xml: Path) -> Dict[Tuple[str, str], str]:
    root = ET.parse(str(netlist_xml)).getroot()
    result: Dict[Tuple[str, str], str] = {}
    nets = root.find("nets")
    if nets is None:
        raise RuntimeError("schematic netlist has no <nets> element")
    for net in nets.findall("net"):
        name = str(net.attrib.get("name", ""))
        for node in net.findall("node"):
            result[(str(node.attrib["ref"]), str(node.attrib["pin"]))] = name
    return result


def check_contract(
    contract: Mapping[Tuple[str, str], str],
    board_map: Mapping[Tuple[str, str], str],
    schematic_map: Mapping[Tuple[str, str], str],
) -> Dict[str, Any]:
    rows: List[Dict[str, Any]] = []
    for key, expected in contract.items():
        board_net = board_map.get(key)
        schematic_net = schematic_map.get(key)
        rows.append(
            {
                "reference": key[0],
                "pad": key[1],
                "expected": expected,
                "board": board_net,
                "schematic": schematic_net,
                "board_ok": board_net == expected,
                "schematic_ok": schematic_net == expected,
                "parity_ok": board_net == schematic_net == expected,
            }
        )
    return {
        "pass": all(row["parity_ok"] for row in rows),
        "rows": rows,
        "mismatches": [row for row in rows if not row["parity_ok"]],
    }


def ground_bridge_audit(board: Any) -> Dict[str, Any]:
    bridges: List[Dict[str, Any]] = []
    for footprint in board.GetFootprints():
        pad_nets = {
            str(pad.GetNumber()): str(pad.GetNetname())
            for pad in footprint.Pads()
            if pad.GetNetname()
        }
        if "GND" in pad_nets.values() and "GND2" in pad_nets.values():
            bridges.append(
                {
                    "reference": str(footprint.GetReference()),
                    "value": str(footprint.GetValue()),
                    "pad_nets": dict(sorted(pad_nets.items())),
                }
            )
    return {
        "bridges": bridges,
        "pass": len(bridges) == 1 and bridges[0]["reference"] == "R0",
    }


def pads_by_net(board: Any) -> Dict[str, List[Any]]:
    result: Dict[str, List[Any]] = collections.defaultdict(list)
    for pad in board.GetPads():
        name = str(pad.GetNetname())
        if name and pad.GetNetCode() > 0 and not pad.IsNoConnectPad():
            result[name].append(pad)
    return dict(result)


def connected_pad_groups(board: Any, pads: Sequence[Any]) -> List[List[Any]]:
    if not pads:
        return []
    board.BuildConnectivity()
    connectivity = board.GetConnectivity()
    dsu = DSU(len(pads))
    index_by_uuid = {item_uuid(pad): index for index, pad in enumerate(pads)}

    for index, pad in enumerate(pads):
        try:
            connected = connectivity.GetConnectedItems(pad)
        except Exception:
            connected = []
        for item in connected:
            if not isinstance(item, pcbnew.PAD):
                continue
            other = index_by_uuid.get(item_uuid(item))
            if other is not None:
                dsu.union(index, other)

    # KiCad occasionally omits coincident jumper pads from the SWIG result.
    for left in range(len(pads)):
        for right in range(left + 1, len(pads)):
            if pads[left].GetPosition() != pads[right].GetPosition():
                continue
            if any(
                pads[left].FlashLayer(layer)
                and pads[right].FlashLayer(layer)
                and pads[left]
                .GetEffectiveShape(layer)
                .Collide(pads[right].GetEffectiveShape(layer), 0)
                for layer in enabled_copper_layers(board)
            ):
                dsu.union(left, right)

    grouped: Dict[int, List[Any]] = collections.defaultdict(list)
    for index, pad in enumerate(pads):
        grouped[dsu.find(index)].append(pad)
    return list(grouped.values())


def describe_groups(groups: Sequence[Sequence[Any]]) -> List[List[Dict[str, Any]]]:
    described: List[List[Dict[str, Any]]] = []
    for group in groups:
        described.append(
            sorted(
                [
                    {
                        "pad": pad_label(pad),
                        "position_mm": point_mm(pad),
                        "layer_set": str(pad.GetLayerSet().FmtHex()),
                    }
                    for pad in group
                ],
                key=lambda row: row["pad"],
            )
        )
    return sorted(described, key=lambda group: (len(group), group[0]["pad"] if group else ""))


def extract_unconnected_nets(drc: Mapping[str, Any], known_nets: Iterable[str]) -> Dict[str, Any]:
    known = set(known_nets)
    counts: Dict[str, int] = collections.Counter()
    rows: List[Dict[str, Any]] = []
    bracket = re.compile(r"\[([^\]]+)\]")
    for item in drc.get("unconnected_items", []):
        descriptions = [str(entry.get("description", "")) for entry in item.get("items", [])]
        found: List[str] = []
        for description in descriptions:
            for match in bracket.findall(description):
                if match in known:
                    found.append(match)
        net = found[0] if found else "<unknown>"
        counts[net] += 1
        rows.append(
            {
                "net": net,
                "severity": item.get("severity"),
                "descriptions": descriptions,
            }
        )
    return {
        "count": len(rows),
        "by_net": dict(sorted(counts.items())),
        "items": rows,
    }


def zone_unconnected_audit(
    drc: Mapping[str, Any], zone_names: Iterable[str]
) -> Dict[str, Any]:
    names = tuple(sorted(set(zone_names)))
    incidents = {name: 0 for name in names}
    self_edges = {name: 0 for name in names}
    cross_zone_items: List[Dict[str, Any]] = []
    for item in drc.get("unconnected_items", []):
        descriptions = [str(entry.get("description", "")) for entry in item.get("items", [])]
        names_in_item: List[str] = []
        for name in names:
            hits = sum(name in description for description in descriptions)
            if hits:
                incidents[name] += 1
                names_in_item.append(name)
            if hits >= 2:
                self_edges[name] += 1
        if len(names_in_item) >= 2:
            cross_zone_items.append(
                {
                    "zones": sorted(names_in_item),
                    "descriptions": descriptions,
                }
            )
    return {
        "audited_zone_names": list(names),
        "incidents": incidents,
        "self_edges": self_edges,
        "cross_zone_items": cross_zone_items,
        "pass": all(value == 0 for value in incidents.values()),
    }


def connectivity_audit(board: Any, drc: Mapping[str, Any]) -> Dict[str, Any]:
    by_net = pads_by_net(board)
    disconnected: Dict[str, Any] = {}
    ground: Dict[str, Any] = {}
    for net, pads in sorted(by_net.items()):
        groups = connected_pad_groups(board, pads)
        if len(groups) > 1:
            disconnected[net] = {
                "pad_count": len(pads),
                "group_count": len(groups),
                "groups": describe_groups(groups),
            }
        if net in ("GND", "GND2"):
            ground[net] = {
                "pad_count": len(pads),
                "group_count": len(groups),
                "groups": describe_groups(groups),
            }

    ground_zone_names = set(ZONE_NAMES)
    ground_zone_names.update(
        str(zone.GetZoneName())
        for zone in board.Zones()
        if str(zone.GetNetname()) in ("GND", "GND2") and zone.GetZoneName()
    )
    zones = zone_unconnected_audit(drc, ground_zone_names)
    return {
        "audited_copper_layers": enabled_copper_layer_names(board),
        "kicad_unconnected_count": int(
            board.GetConnectivity().GetUnconnectedCount(False)
        ),
        "disconnected_pad_nets": disconnected,
        "ground": ground,
        "zone_unconnected": zones,
        "zone_self_edge_unconnected": zones["self_edges"],
        "ground_pass": all(
            ground.get(net, {}).get("group_count") == 1 for net in ("GND", "GND2")
        )
        and bool(zones["pass"]),
    }


def via_dimensions(via: Any) -> Tuple[float, float]:
    diameter = mm(via.GetWidth(F_CU))
    drill = mm(via.GetDrillValue())
    return diameter, drill


def via_in_pad_audit(board: Any) -> Dict[str, Any]:
    pads = list(board.GetPads())
    rows: List[Dict[str, Any]] = []
    audited_layers = enabled_copper_layers(board)
    vias = [item for item in board.GetTracks() if isinstance(item, pcbnew.PCB_VIA)]
    for via in vias:
        # Only the drilled hole creates the solder-wicking/manufacturing risk
        # meant by "via in pad" here.  Treat tangency as clear: an off-pad toe
        # via whose drill circle only touches the pad boundary does not pierce
        # solderable copper.  Shrinking by one IU avoids integer-geometry
        # tangency being reported as an overlap while leaving real overlap
        # detection unchanged at any manufacturable scale.
        drill_shape = pcbnew.SHAPE_CIRCLE()
        drill_shape.SetCenter(via.GetPosition())
        drill_shape.SetRadius(max(0, via.GetDrillValue() // 2 - 1))
        overlaps: Dict[str, Dict[str, Any]] = {}
        for layer in audited_layers:
            if not via.FlashLayer(layer):
                continue
            for pad in pads:
                if not pad.FlashLayer(layer):
                    continue
                if pad.GetEffectiveShape(layer).Collide(drill_shape, 0):
                    label = pad_label(pad)
                    overlap = overlaps.setdefault(
                        label,
                        {
                            "pad": label,
                            "pad_net": str(pad.GetNetname()),
                            "layers": [],
                        },
                    )
                    overlap["layers"].append(board.GetLayerName(layer))
        if overlaps:
            diameter, drill = via_dimensions(via)
            rows.append(
                {
                    "position_mm": point_mm(via),
                    "net": str(via.GetNetname()),
                    "diameter_mm": rounded(diameter),
                    "drill_mm": rounded(drill),
                    "pads": sorted(overlaps.values(), key=lambda row: row["pad"]),
                }
            )
    return {
        "audited_copper_layers": [board.GetLayerName(layer) for layer in audited_layers],
        "via_count": len(vias),
        "via_in_pad_count": len(rows),
        "pass": len(rows) == 0,
        "items": rows,
    }


def track_width_audit(board: Any) -> Dict[str, Any]:
    by_net: Dict[str, Dict[str, Any]] = {}
    by_layer: Dict[str, Dict[str, Any]] = {
        name: {
            "segment_count": 0,
            "total_length_mm": 0.0,
            "widths_mm": collections.Counter(),
            "net_segment_counts": collections.Counter(),
        }
        for name in enabled_copper_layer_names(board)
    }
    all_widths: List[float] = []
    below_hard: List[Dict[str, Any]] = []
    below_nominal: List[Dict[str, Any]] = []
    segment_count = 0
    for item in board.GetTracks():
        if isinstance(item, pcbnew.PCB_VIA):
            continue
        segment_count += 1
        width = mm(item.GetWidth())
        all_widths.append(width)
        net = str(item.GetNetname()) or "<no-net>"
        layer_name = str(board.GetLayerName(item.GetLayer()))
        layer_row = by_layer.setdefault(
            layer_name,
            {
                "segment_count": 0,
                "total_length_mm": 0.0,
                "widths_mm": collections.Counter(),
                "net_segment_counts": collections.Counter(),
            },
        )
        layer_row["segment_count"] += 1
        layer_row["total_length_mm"] += mm(item.GetLength())
        layer_row["widths_mm"][rounded(width, 4)] += 1
        layer_row["net_segment_counts"][net] += 1
        row = by_net.setdefault(
            net,
            {
                "segment_count": 0,
                "total_length_mm": 0.0,
                "widths_mm": collections.Counter(),
            },
        )
        row["segment_count"] += 1
        row["total_length_mm"] += mm(item.GetLength())
        row["widths_mm"][rounded(width, 4)] += 1
        location = {
            "net": net,
            "position_mm": point_mm(item),
            "layer": board.GetLayerName(item.GetLayer()),
            "width_mm": rounded(width),
            "length_mm": rounded(mm(item.GetLength())),
        }
        if width < JLC["two_oz_trace_width_min_mm"] - 1e-6:
            below_hard.append(location)
        elif width < JLC["project_normal_width_floor_mm"] - 1e-6:
            below_nominal.append(location)

    rendered: Dict[str, Any] = {}
    for net, row in sorted(by_net.items()):
        widths = sorted(float(value) for value in row["widths_mm"].keys())
        rendered[net] = {
            "segment_count": row["segment_count"],
            "total_length_mm": rounded(row["total_length_mm"]),
            "min_width_mm": rounded(min(widths)),
            "max_width_mm": rounded(max(widths)),
            "width_histogram": {
                "{:.4f}".format(width): row["widths_mm"][rounded(width, 4)]
                for width in widths
            },
        }
    rendered_layers: Dict[str, Any] = {}
    for layer_name, row in by_layer.items():
        widths = sorted(float(value) for value in row["widths_mm"].keys())
        rendered_layers[layer_name] = {
            "segment_count": row["segment_count"],
            "total_length_mm": rounded(row["total_length_mm"]),
            "min_width_mm": rounded(min(widths)) if widths else None,
            "max_width_mm": rounded(max(widths)) if widths else None,
            "width_histogram": {
                "{:.4f}".format(width): row["widths_mm"][rounded(width, 4)]
                for width in widths
            },
            "net_segment_counts": dict(sorted(row["net_segment_counts"].items())),
        }
    return {
        "audited_copper_layers": enabled_copper_layer_names(board),
        "segment_count": segment_count,
        "global_min_width_mm": rounded(min(all_widths)) if all_widths else None,
        "global_max_width_mm": rounded(max(all_widths)) if all_widths else None,
        "jlc_two_oz_hard_min_mm": JLC["two_oz_trace_width_min_mm"],
        "project_normal_floor_mm": JLC["project_normal_width_floor_mm"],
        "below_jlc_hard_min": below_hard,
        "below_project_normal_floor": below_nominal,
        "jlc_width_pass": len(below_hard) == 0,
        "by_layer": rendered_layers,
        "by_net": rendered,
    }


def via_audit(board: Any, via_in_pad: Mapping[str, Any]) -> Dict[str, Any]:
    rows: List[Dict[str, Any]] = []
    toe_rows: List[Dict[str, Any]] = []
    invalid_hard: List[Dict[str, Any]] = []
    invalid_project: List[Dict[str, Any]] = []
    special_option: List[Dict[str, Any]] = []
    overlap_positions = {
        tuple(item["position_mm"]) for item in via_in_pad.get("items", [])
    }
    for via in board.GetTracks():
        if not isinstance(via, pcbnew.PCB_VIA):
            continue
        diameter, drill = via_dimensions(via)
        top, bottom = int(via.TopLayer()), int(via.BottomLayer())
        row = {
            "position_mm": point_mm(via),
            "net": str(via.GetNetname()),
            "diameter_mm": rounded(diameter),
            "drill_mm": rounded(drill),
            "annular_ring_mm": rounded((diameter - drill) / 2.0),
            "layer_pair": [board.GetLayerName(top), board.GetLayerName(bottom)],
            "through_via": int(top) == F_CU and int(bottom) == B_CU,
            "via_in_pad": tuple(point_mm(via)) in overlap_positions,
        }
        rows.append(row)
        if (
            abs(diameter - JLC["project_toe_via_diameter_mm"]) <= 1e-6
            and abs(drill - JLC["project_toe_via_drill_mm"]) <= 1e-6
        ):
            toe_rows.append(row)
        if (
            drill < JLC["via_hard_drill_min_mm"] - 1e-6
            or diameter < JLC["via_hard_diameter_min_mm"] - 1e-6
            or not row["through_via"]
        ):
            invalid_hard.append(row)
        if drill < 0.20 - 1e-6 or diameter < 0.40 - 1e-6:
            invalid_project.append(row)
        # JLC's capability table says 0.20/0.25 mm drills with via diameter
        # below 0.45 mm require selecting the corresponding small-via option.
        if drill <= 0.25 + 1e-6 and diameter < 0.45 - 1e-6:
            special_option.append(row)
    return {
        "count": len(rows),
        "items": rows,
        "toe_0p40_0p20": toe_rows,
        "toe_0p40_0p20_count": len(toe_rows),
        "jlc_hard_invalid": invalid_hard,
        "project_0p40_0p20_invalid": invalid_project,
        "small_via_order_option_items": special_option,
        "jlc_via_pass": len(invalid_hard) == 0,
        "project_via_pass": len(invalid_project) == 0,
    }


def netclass_name(item: Any) -> str:
    try:
        return str(item.GetNetClassName())
    except Exception:
        return "<unknown>"


def track_description(board: Any, item: Any) -> Dict[str, Any]:
    return {
        "net": str(item.GetNetname()) or "<no-net>",
        "netclass": netclass_name(item),
        "layer": str(board.GetLayerName(item.GetLayer())),
        "position_mm": point_mm(item),
        "width_mm": rounded(mm(item.GetWidth())),
        "length_mm": rounded(mm(item.GetLength())),
    }


def copper_graphics_on_layer(board: Any, layer: int) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for drawing in board.GetDrawings():
        if int(drawing.GetLayer()) == layer:
            rows.append(
                {
                    "kind": "board_graphic",
                    "position_mm": point_mm(drawing),
                }
            )
    for footprint in board.GetFootprints():
        for graphic in footprint.GraphicalItems():
            if int(graphic.GetLayer()) == layer:
                rows.append(
                    {
                        "kind": "footprint_graphic",
                        "reference": str(footprint.GetReference()),
                        "position_mm": point_mm(graphic),
                    }
                )
    return rows


def in1_plane_policy_audit(board: Any) -> Dict[str, Any]:
    configured = "In1.Cu" in enabled_copper_layer_names(board)
    if not configured:
        return {
            "applicable": False,
            "pass": True,
            "note": "In1.Cu is absent; the four-layer-count gate reports this separately.",
            "zones": [],
            "track_segments": [],
            "copper_graphics": [],
            "ground_stitch_vias": [],
            "signal_transit_through_vias": [],
            "invalid_vias": [],
        }

    zones: List[Dict[str, Any]] = []
    for zone in board.Zones():
        if not zone.GetLayerSet().Contains(IN1_CU):
            continue
        net = str(zone.GetNetname()) or "<no-net>"
        zones.append(
            {
                "name": str(zone.GetZoneName()),
                "net": net,
                "netclass": netclass_name(zone),
                "layers": [
                    board.GetLayerName(layer) for layer in zone.GetLayerSet().Seq()
                ],
                "position_mm": point_mm(zone),
                "allowed": net in IN1_ALLOWED_ZONE_NETS,
            }
        )

    tracks = [
        track_description(board, item)
        for item in board.GetTracks()
        if not isinstance(item, pcbnew.PCB_VIA)
        and int(item.GetLayer()) == IN1_CU
    ]
    graphics = copper_graphics_on_layer(board, IN1_CU)

    ground_stitches: List[Dict[str, Any]] = []
    signal_transits: List[Dict[str, Any]] = []
    invalid_vias: List[Dict[str, Any]] = []
    for via in board.GetTracks():
        if not isinstance(via, pcbnew.PCB_VIA):
            continue
        if not via.GetLayerSet().Contains(IN1_CU):
            continue
        top = int(via.TopLayer())
        bottom = int(via.BottomLayer())
        net = str(via.GetNetname()) or "<no-net>"
        row = {
            "net": net,
            "position_mm": point_mm(via),
            "layer_pair": [board.GetLayerName(top), board.GetLayerName(bottom)],
            "through_via": top == F_CU and bottom == B_CU,
        }
        if not row["through_via"] or via.GetNetCode() <= 0:
            invalid_vias.append(row)
        elif net in IN1_ALLOWED_ZONE_NETS:
            ground_stitches.append(row)
        else:
            signal_transits.append(row)

    zone_nets = {row["net"] for row in zones if row["allowed"]}
    required_zone_nets_present = all(
        net in zone_nets for net in IN1_ALLOWED_ZONE_NETS
    )
    return {
        "applicable": True,
        "required_zone_nets": sorted(IN1_ALLOWED_ZONE_NETS),
        "required_zone_nets_present": required_zone_nets_present,
        "zones": zones,
        "invalid_zones": [row for row in zones if not row["allowed"]],
        "track_segments": tracks,
        "copper_graphics": graphics,
        "ground_stitch_vias": ground_stitches,
        "signal_transit_through_vias": signal_transits,
        "invalid_vias": invalid_vias,
        "pass": required_zone_nets_present
        and all(row["allowed"] for row in zones)
        and not tracks
        and not graphics
        and not invalid_vias,
        "note": (
            "In1.Cu is plane-only: GND/GND2 zones and assigned F.Cu-B.Cu "
            "through-vias may cross it; track segments and copper graphics may not."
        ),
    }


def in2_routing_policy_audit(board: Any) -> Dict[str, Any]:
    configured = "In2.Cu" in enabled_copper_layer_names(board)
    if not configured:
        return {
            "applicable": False,
            "pass": True,
            "note": "In2.Cu is absent; the four-layer-count gate reports this separately.",
            "track_segments": [],
            "zones": [],
            "copper_graphics": [],
            "forbidden_items": [],
        }

    def allowed(net: str, netclass: str) -> bool:
        if net in IN2_ALWAYS_ALLOWED_NETS:
            return True
        return (
            bool(net)
            and net != "<no-net>"
            and net not in IN2_FORBIDDEN_NETS
            and netclass not in IN2_FORBIDDEN_NETCLASSES
        )

    tracks: List[Dict[str, Any]] = []
    for item in board.GetTracks():
        if isinstance(item, pcbnew.PCB_VIA) or int(item.GetLayer()) != IN2_CU:
            continue
        row = track_description(board, item)
        row["allowed"] = allowed(row["net"], row["netclass"])
        tracks.append(row)

    zones: List[Dict[str, Any]] = []
    for zone in board.Zones():
        if not zone.GetLayerSet().Contains(IN2_CU):
            continue
        net = str(zone.GetNetname()) or "<no-net>"
        netclass = netclass_name(zone)
        zones.append(
            {
                "name": str(zone.GetZoneName()),
                "net": net,
                "netclass": netclass,
                "position_mm": point_mm(zone),
                "allowed": allowed(net, netclass),
            }
        )

    graphics = copper_graphics_on_layer(board, IN2_CU)
    forbidden = [row for row in tracks + zones if not row["allowed"]]
    ground_zone_nets = {
        row["net"] for row in zones if row["net"] in IN1_ALLOWED_ZONE_NETS
    }
    required_ground_zones_present = all(
        net in ground_zone_nets for net in IN1_ALLOWED_ZONE_NETS
    )
    return {
        "applicable": True,
        "required_ground_zone_nets": sorted(IN1_ALLOWED_ZONE_NETS),
        "required_ground_zones_present": required_ground_zones_present,
        "track_segments": tracks,
        "zones": zones,
        "copper_graphics": graphics,
        "forbidden_items": forbidden,
        "pass": required_ground_zones_present and not forbidden and not graphics,
        "note": (
            "In2.Cu requires split GND/GND2 pours and permits +3V3 and "
            "logic/sensor signal tracks through those low-priority pours. +5V, "
            "other HighCurrent, and SwitchNode tracks/zones are forbidden."
        ),
    }


def copper_layer_audit(board: Any) -> Dict[str, Any]:
    used: Dict[str, int] = collections.Counter()
    kinds_by_layer: Dict[str, Dict[str, int]] = collections.defaultdict(
        collections.Counter
    )

    def record(kind: str, layers: Iterable[int]) -> None:
        for layer in layers:
            name = str(board.GetLayerName(int(layer)))
            if not name.endswith(".Cu"):
                continue
            used[name] += 1
            kinds_by_layer[name][kind] += 1

    for item in board.GetTracks():
        if isinstance(item, pcbnew.PCB_VIA):
            record("via", item.GetLayerSet().Seq())
        else:
            record("track", [int(item.GetLayer())])
    for zone in board.Zones():
        record("zone", zone.GetLayerSet().Seq())
    for drawing in board.GetDrawings():
        record("board_graphic", [int(drawing.GetLayer())])
    for footprint in board.GetFootprints():
        for graphic in footprint.GraphicalItems():
            record("footprint_graphic", [int(graphic.GetLayer())])

    configured_names = enabled_copper_layer_names(board)
    missing = [
        name for name in REQUIRED_COPPER_LAYER_NAMES if name not in configured_names
    ]
    unexpected = [
        name for name in configured_names if name not in REQUIRED_COPPER_LAYER_NAMES
    ]
    four_layer_pass = (
        int(board.GetCopperLayerCount()) == 4 and not missing and not unexpected
    )
    return {
        "configured_copper_layer_count": int(board.GetCopperLayerCount()),
        "configured_copper_layers": configured_names,
        "required_copper_layers": list(REQUIRED_COPPER_LAYER_NAMES),
        "missing_required_copper_layers": missing,
        "unexpected_copper_layers": unexpected,
        "used_copper_layers": dict(sorted(used.items())),
        "item_kinds_by_layer": {
            layer: dict(sorted(counts.items()))
            for layer, counts in sorted(kinds_by_layer.items())
        },
        "four_layer_pass": four_layer_pass,
        "pass": four_layer_pass,
    }


def inner_ground_copper_coverage_audit(board: Any) -> Dict[str, Any]:
    """Conservatively measure filled split-ground coverage on inner layers.

    JLCPCB asks designers to add copper pour when an inner layer has less than
    25 percent copper coverage.  Signal tracks and via annuli also add copper,
    but counting only the refilled GND/GND2 zones gives a stable lower bound.
    """

    outline = pcbnew.SHAPE_POLY_SET()
    outline_ok = bool(board.GetBoardPolygonOutlines(outline, False))
    board_area_raw = abs(float(outline.Area())) if outline_ok else 0.0
    rows: Dict[str, Any] = {}
    for layer in (IN1_CU, IN2_CU):
        layer_name = str(board.GetLayerName(layer))
        zones: List[Dict[str, Any]] = []
        filled_area_raw = 0.0
        for zone in board.Zones():
            if not zone.GetLayerSet().Contains(layer):
                continue
            net = str(zone.GetNetname())
            if net not in IN1_ALLOWED_ZONE_NETS:
                continue
            polys = zone.GetFilledPolysList(layer)
            area_raw = abs(float(polys.Area()))
            filled_area_raw += area_raw
            zones.append(
                {
                    "name": str(zone.GetZoneName()),
                    "net": net,
                    "filled_area_mm2": rounded(area_raw / 1.0e12),
                    "outline_count": int(polys.OutlineCount()),
                }
            )
        coverage = (
            100.0 * filled_area_raw / board_area_raw if board_area_raw > 0.0 else 0.0
        )
        rows[layer_name] = {
            "ground_zone_filled_area_mm2": rounded(filled_area_raw / 1.0e12),
            "coverage_percent_lower_bound": rounded(coverage, 3),
            "zones": zones,
            "pass": coverage
            >= JLC["inner_layer_copper_coverage_min_percent"] - 1.0e-6,
        }
    return {
        "board_outline_valid": outline_ok,
        "board_area_mm2": rounded(board_area_raw / 1.0e12),
        "minimum_percent": JLC["inner_layer_copper_coverage_min_percent"],
        "layers": rows,
        "pass": outline_ok
        and all(rows.get(name, {}).get("pass", False) for name in ("In1.Cu", "In2.Cu")),
        "note": (
            "Coverage counts only refilled GND/GND2 zones, so it is a "
            "conservative lower bound that excludes signal tracks and via annuli."
        ),
    }


def u5_audit(board: Any) -> Dict[str, Any]:
    footprint = board.FindFootprintByReference("U5")
    if footprint is None:
        return {"pass": False, "error": "U5 footprint missing"}
    pad49 = footprint.FindPadByNumber("49")
    pad49_info: Dict[str, Any] = {"present": pad49 is not None}
    if pad49 is not None:
        pad49_info.update(
            {
                "net": str(pad49.GetNetname()),
                "position_mm": point_mm(pad49),
                "size_mm": [rounded(mm(pad49.GetSizeX())), rounded(mm(pad49.GetSizeY()))],
                "layers": [
                    board.GetLayerName(layer)
                    for layer in pad49.GetLayerSet().Seq()
                ],
                "has_f_paste": bool(pad49.GetLayerSet().Contains(pcbnew.F_Paste)),
                "has_b_paste": bool(pad49.GetLayerSet().Contains(pcbnew.B_Paste)),
            }
        )

    paste_graphics: List[Dict[str, Any]] = []
    for graphic in footprint.GraphicalItems():
        if int(graphic.GetLayer()) not in (int(pcbnew.F_Paste), int(pcbnew.B_Paste)):
            continue
        bbox = graphic.GetBoundingBox()
        paste_graphics.append(
            {
                "layer": board.GetLayerName(graphic.GetLayer()),
                "bbox_mm": [
                    rounded(mm(bbox.GetX())),
                    rounded(mm(bbox.GetY())),
                    rounded(mm(bbox.GetWidth())),
                    rounded(mm(bbox.GetHeight())),
                ],
            }
        )

    paste_pads = []
    for pad in footprint.Pads():
        if pad.GetLayerSet().Contains(pcbnew.F_Paste) or pad.GetLayerSet().Contains(
            pcbnew.B_Paste
        ):
            paste_pads.append(str(pad.GetNumber()))
    return {
        "footprint": "{}:{}".format(
            footprint.GetFPID().GetLibNickname(),
            footprint.GetFPID().GetLibItemName(),
        ),
        "position_mm": point_mm(footprint),
        "pad49": pad49_info,
        "paste_graphic_count": len(paste_graphics),
        "paste_graphics": paste_graphics,
        "paste_pad_numbers": sorted(paste_pads),
        "hand_solder_no_ep_pass": pad49 is None and len(paste_graphics) == 0,
        "pass": pad49 is None and len(paste_graphics) == 0,
        "note": (
            "Lead-pad F.Paste is expected; pass requires no exposed pad 49 and "
            "no separate central paste-window graphics."
        ),
    }


def project_rules(project_pro: Path, project_dru: Path) -> Dict[str, Any]:
    with project_pro.open("r", encoding="utf-8") as handle:
        project = json.load(handle)
    settings = project.get("board", {}).get("design_settings", {})
    rules = settings.get("rules", {})
    defaults = settings.get("defaults", {})
    text = project_dru.read_text(encoding="utf-8")
    checks = {
        "min_track_width_equals_0p16": rules.get("min_track_width") is not None
        and abs(float(rules["min_track_width"]) - 0.16) <= 1e-9,
        "min_clearance_at_least_0p20": rules.get("min_clearance") is not None
        and float(rules["min_clearance"]) >= 0.20 - 1e-9,
        "min_copper_edge_at_least_0p20": rules.get("min_copper_edge_clearance")
        is not None
        and float(rules["min_copper_edge_clearance"]) >= 0.20 - 1e-9,
        "min_via_diameter_at_least_0p40": rules.get("min_via_diameter") is not None
        and float(rules["min_via_diameter"]) >= 0.40 - 1e-9,
        "dru_has_two_oz_0p16_width": bool(
            re.search(r"track_width\s+\(min\s+0\.16mm\)", text)
        ),
        "dru_has_two_oz_0p16_clearance": bool(
            re.search(r"clearance\s+\(min\s+0\.16mm\)", text)
        ),
    }
    return {
        "pro_sha256": sha256(project_pro),
        "dru_sha256": sha256(project_dru),
        "rules": rules,
        "zone_defaults": defaults.get("zones", {}),
        "checks": checks,
        "pass": all(checks.values()),
    }


def drc_audit(drc: Mapping[str, Any]) -> Dict[str, Any]:
    by_type: Dict[str, int] = collections.Counter()
    by_severity: Dict[str, int] = collections.Counter()
    error_rows: List[Dict[str, Any]] = []
    edge_rows: List[Dict[str, Any]] = []
    for item in drc.get("violations", []):
        kind = str(item.get("type", "<unknown>"))
        severity = str(item.get("severity", "<unknown>"))
        by_type[kind] += 1
        by_severity[severity] += 1
        if severity == "error":
            error_rows.append(item)
        if kind == "copper_edge_clearance":
            description = str(item.get("description", ""))
            values = [float(value) for value in re.findall(r"([0-9]+(?:\.[0-9]+)?)\s*mm", description)]
            edge_rows.append(
                {
                    "description": description,
                    "actual_mm_inferred": values[-1] if values else None,
                    "items": item.get("items", []),
                }
            )
    parity = list(drc.get("schematic_parity", []))
    parity_u5_49 = [
        item
        for item in parity
        if "49" in str(item.get("description", ""))
        and any("U5" in str(entry.get("description", "")) for entry in item.get("items", []))
    ]
    return {
        "violation_count": len(drc.get("violations", [])),
        "violation_by_type": dict(sorted(by_type.items())),
        "violation_by_severity": dict(sorted(by_severity.items())),
        "error_count": len(error_rows),
        "errors": error_rows,
        "unconnected_item_count": len(drc.get("unconnected_items", [])),
        "schematic_parity_count": len(parity),
        "schematic_parity_u5_pad49_expected_exception_count": len(parity_u5_49),
        "schematic_parity": parity,
        "copper_edge_clearance": {
            "project_threshold_mm": 0.20,
            "violation_count": len(edge_rows),
            "violations": edge_rows,
            "proven_at_or_above_threshold": len(edge_rows) == 0,
        },
        "physical_error_pass": len(error_rows) == 0,
    }


def build_markdown(report: Mapping[str, Any]) -> str:
    lines: List[str] = []
    checks = report["release_checks"]
    lines.extend(
        [
            "# mini_r3 final PCB audit",
            "",
            "- Candidate: `{}`".format(report["input"]["candidate"]),
            "- Candidate SHA-256: `{}`".format(
                report["input"]["candidate_sha256_before_refill"]
            ),
            "- KiCad: `{}`".format(report["kicad"]["kicad_version"]),
            "- Temporary audit directory: `{}`".format(report["artifacts"]["directory"]),
            "- Overall release gate: **{}**".format(
                "PASS" if report["release_gate_pass"] else "FAIL"
            ),
            "",
            "## Release checks",
            "",
            "| Check | Result |",
            "|---|---:|",
        ]
    )
    for name, passed in checks.items():
        lines.append("| `{}` | {} |".format(name, "PASS" if passed else "FAIL"))

    drc = report["drc"]
    lines.extend(
        [
            "",
            "## KiCad DRC after exact-project zone refill",
            "",
            "- Physical violations: {} (errors: {})".format(
                drc["violation_count"], drc["error_count"]
            ),
            "- Unconnected items: {}".format(drc["unconnected_item_count"]),
            "- Schematic-parity warnings: {} (U5 pad49 expected exception: {})".format(
                drc["schematic_parity_count"],
                drc["schematic_parity_u5_pad49_expected_exception_count"],
            ),
            "- Violation types: `{}`".format(
                json.dumps(drc["violation_by_type"], ensure_ascii=False, sort_keys=True)
            ),
            "",
            "## Unconnected nets",
            "",
        ]
    )
    unconnected = report["unconnected"]
    if unconnected["by_net"]:
        lines.extend(["| Net | DRC missing edges |", "|---|---:|"])
        for net, count in unconnected["by_net"].items():
            lines.append("| `{}` | {} |".format(net, count))
    else:
        lines.append("None.")

    connectivity = report["connectivity"]
    lines.extend(["", "## Split ground", ""])
    lines.append(
        "- Connectivity audited on: `{}`".format(
            ", ".join(connectivity["audited_copper_layers"])
        )
    )
    for net in ("GND", "GND2"):
        info = connectivity["ground"].get(net, {})
        lines.append(
            "- `{}`: {} pads, {} connected group(s)".format(
                net, info.get("pad_count", 0), info.get("group_count", 0)
            )
        )
    lines.append(
        "- Zone self-edge unconnected: `{}`".format(
            json.dumps(connectivity["zone_self_edge_unconnected"], sort_keys=True)
        )
    )
    lines.append(
        "- All zone-related unconnected incidents: `{}`".format(
            json.dumps(connectivity["zone_unconnected"]["incidents"], sort_keys=True)
        )
    )
    lines.append(
        "- R0 contract: **{}**".format(
            "PASS" if report["r0_contract"]["pass"] else "FAIL"
        )
    )
    lines.append(
        "- GND/GND2 bridge footprints: `{}`".format(
            json.dumps(
                [item["reference"] for item in report["ground_bridges"]["bridges"]]
            )
        )
    )

    vip = report["via_in_pad"]
    vias = report["vias"]
    lines.extend(
        [
            "",
            "## Vias",
            "",
            "- Copper layers checked for drill-in-pad overlap: `{}`".format(
                ", ".join(vip["audited_copper_layers"])
            ),
            "- Total vias: {}".format(vip["via_count"]),
            "- Drill-in-pad overlaps: {}".format(vip["via_in_pad_count"]),
            "- 0.40/0.20 mm toe vias: {}".format(vias["toe_0p40_0p20_count"]),
            "- Small-via order-option items: {}".format(
                len(vias["small_via_order_option_items"])
            ),
        ]
    )
    for item in vip["items"]:
        lines.append(
            "  - `{}` at {} overlaps {}".format(
                item["net"], item["position_mm"], ", ".join(x["pad"] for x in item["pads"])
            )
        )

    tracks = report["tracks"]
    lines.extend(
        [
            "",
            "## Track widths by net",
            "",
            "- Copper layers audited: `{}`".format(
                ", ".join(tracks["audited_copper_layers"])
            ),
            "- Global min/max: {} / {} mm".format(
                tracks["global_min_width_mm"], tracks["global_max_width_mm"]
            ),
            "- Below JLC 2 oz hard minimum (0.16 mm): {}".format(
                len(tracks["below_jlc_hard_min"])
            ),
            "- 0.16–<0.20 mm local escapes: {}".format(
                len(tracks["below_project_normal_floor"])
            ),
            "",
            "| Layer | Segments | Length (mm) | Min (mm) | Max (mm) |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for layer, row in tracks["by_layer"].items():
        lines.append(
            "| `{}` | {} | {:.3f} | {} | {} |".format(
                layer,
                row["segment_count"],
                row["total_length_mm"],
                "-" if row["min_width_mm"] is None else "{:.3f}".format(row["min_width_mm"]),
                "-" if row["max_width_mm"] is None else "{:.3f}".format(row["max_width_mm"]),
            )
        )
    lines.extend(
        [
            "",
            "| Net | Segments | Length (mm) | Min (mm) | Max (mm) | Width histogram |",
            "|---|---:|---:|---:|---:|---|",
        ]
    )
    for net, row in tracks["by_net"].items():
        lines.append(
            "| `{}` | {} | {:.3f} | {:.3f} | {:.3f} | `{}` |".format(
                net,
                row["segment_count"],
                row["total_length_mm"],
                row["min_width_mm"],
                row["max_width_mm"],
                json.dumps(row["width_histogram"], sort_keys=True),
            )
        )

    edge = drc["copper_edge_clearance"]
    lines.extend(
        [
            "",
            "## Copper-to-edge",
            "",
            "- Exact KiCad DRC threshold: {} mm".format(edge["project_threshold_mm"]),
            "- Violations: {}".format(edge["violation_count"]),
            "- At/above JLC routed-edge capability: **{}**".format(
                "PASS" if edge["proven_at_or_above_threshold"] else "FAIL"
            ),
            "",
            "## Logical driver and MCU footprint audits",
            "",
            "- U2 logical LEFT / U3 logical RIGHT: **{}**".format(
                "PASS" if report["driver_contract"]["pass"] else "FAIL"
            ),
            "- U5 no exposed pad 49 / no central paste windows: **{}**".format(
                "PASS" if report["u5"]["pass"] else "FAIL"
            ),
        ]
    )
    if report["driver_contract"]["mismatches"]:
        lines.append("")
        lines.append("Driver mismatches:")
        for row in report["driver_contract"]["mismatches"]:
            lines.append(
                "- `{}.{}` expected `{}`, board `{}`, schematic `{}`".format(
                    row["reference"],
                    row["pad"],
                    row["expected"],
                    row["board"],
                    row["schematic"],
                )
            )

    layers = report["layers"]
    in1 = report["in1_policy"]
    in2 = report["in2_policy"]
    inner_coverage = report["inner_ground_copper_coverage"]
    lines.extend(
        [
            "",
            "## Four-layer stack and inner-layer policy",
            "",
            "- Configured copper layers: {}".format(
                layers["configured_copper_layer_count"]
            ),
            "- Layer sequence: `{}`".format(
                " / ".join(layers["configured_copper_layers"])
            ),
            "- Required sequence `F.Cu / In1.Cu / In2.Cu / B.Cu`: **{}**".format(
                "PASS" if layers["pass"] else "FAIL"
            ),
            "- In1 GND/GND2 plane-only policy: **{}** (zones {}, tracks {}, copper graphics {}, invalid vias {})".format(
                "PASS" if in1["pass"] else "FAIL",
                len(in1["zones"]),
                len(in1["track_segments"]),
                len(in1["copper_graphics"]),
                len(in1["invalid_vias"]),
            ),
            "- In2 logic/+3V3/local-ground policy: **{}** (tracks {}, zones {}, forbidden {}, copper graphics {})".format(
                "PASS" if in2["pass"] else "FAIL",
                len(in2["track_segments"]),
                len(in2["zones"]),
                len(in2["forbidden_items"]),
                len(in2["copper_graphics"]),
            ),
            "- In2 split GND/GND2 pours present: **{}**".format(
                "PASS" if in2.get("required_ground_zones_present", False) else "FAIL"
            ),
            "- Inner-layer split-ground copper coverage lower bound: In1 `{:.3f}%`, In2 `{:.3f}%` (minimum `{:.1f}%`): **{}**".format(
                inner_coverage["layers"].get("In1.Cu", {}).get(
                    "coverage_percent_lower_bound", 0.0
                ),
                inner_coverage["layers"].get("In2.Cu", {}).get(
                    "coverage_percent_lower_bound", 0.0
                ),
                inner_coverage["minimum_percent"],
                "PASS" if inner_coverage["pass"] else "FAIL",
            ),
            "- Copper weight and stackup are **order settings**, not fully encoded in this board file.",
            "- Select **4 layers, 2 oz outer copper**, confirm the JLCPCB stackup, and select the small-via option when 0.40/0.20 mm vias remain.",
            "- Official capability references:",
        ]
    )
    for source in JLC["sources"]:
        lines.append("  - {}".format(source))
    lines.extend(
        [
            "",
            "## Artifacts",
            "",
            "- `audit.json`: complete machine-readable audit",
            "- `drc.json`: KiCad's exact refilled DRC",
            "- `schematic-netlist.xml`: exported schematic truth",
            "- `{}.kicad_pcb`: refilled temporary copy".format(PROJECT_STEM),
            "",
        ]
    )
    return "\n".join(lines)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--project-dir", required=True, type=Path)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("/private/tmp"),
        help="Parent for a new mini3-final-audit.* directory (default: /private/tmp)",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return exit 2 when the release gate fails (artifacts are still written)",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    candidate = args.candidate.expanduser().resolve()
    project_dir = args.project_dir.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    if not candidate.is_file():
        raise SystemExit("candidate not found: {}".format(candidate))
    if not project_dir.is_dir():
        raise SystemExit("project directory not found: {}".format(project_dir))
    output_root.mkdir(parents=True, exist_ok=True)
    work_dir = Path(
        tempfile.mkdtemp(prefix="mini3-final-audit.", dir=str(output_root))
    ).resolve()

    input_info = copy_exact_project(project_dir, candidate, work_dir)
    kicad = run_kicad(work_dir)
    input_info["candidate_sha256_after_audit"] = sha256(candidate)
    input_info["candidate_unchanged"] = (
        input_info["candidate_sha256_before_refill"]
        == input_info["candidate_sha256_after_audit"]
    )
    canonical_sources_unchanged = True
    for file_info in input_info["canonical_files"].values():
        current_hash = sha256(Path(file_info["source"]))
        file_info["source_sha256_after_audit"] = current_hash
        file_info["source_unchanged"] = current_hash == file_info["source_sha256"]
        canonical_sources_unchanged = (
            canonical_sources_unchanged and file_info["source_unchanged"]
        )
    input_info["canonical_sources_unchanged"] = canonical_sources_unchanged
    board_path = work_dir / (PROJECT_STEM + ".kicad_pcb")
    board = pcbnew.LoadBoard(str(board_path))
    if board is None:
        raise RuntimeError("KiCad Python could not load refilled board")
    board.BuildConnectivity()

    drc = kicad["drc"]
    board_map = board_pad_map(board)
    schematic_map = schematic_pad_map(work_dir / "schematic-netlist.xml")
    drc_result = drc_audit(drc)
    unconnected = extract_unconnected_nets(drc, pads_by_net(board).keys())
    connectivity = connectivity_audit(board, drc)
    via_in_pad = via_in_pad_audit(board)
    vias = via_audit(board, via_in_pad)
    tracks = track_width_audit(board)
    layers = copper_layer_audit(board)
    in1_policy = in1_plane_policy_audit(board)
    in2_policy = in2_routing_policy_audit(board)
    inner_ground_copper_coverage = inner_ground_copper_coverage_audit(board)
    driver_contract = check_contract(DRIVER_CONTRACT, board_map, schematic_map)
    r0_contract = check_contract(R0_CONTRACT, board_map, schematic_map)
    ground_bridges = ground_bridge_audit(board)
    u5 = u5_audit(board)
    rules = project_rules(
        work_dir / (PROJECT_STEM + ".kicad_pro"),
        work_dir / (PROJECT_STEM + ".kicad_dru"),
    )

    release_checks = {
        "candidate_and_canonical_sources_unchanged": bool(
            input_info["candidate_unchanged"]
            and input_info["canonical_sources_unchanged"]
        ),
        "exact_project_rules": bool(rules["pass"]),
        "physical_drc_errors_zero": bool(drc_result["physical_error_pass"]),
        "unconnected_items_zero": unconnected["count"] == 0,
        "gnd_and_gnd2_single_group_no_zone_self_edges": bool(
            connectivity["ground_pass"]
        ),
        "r0_single_point_contract": bool(
            r0_contract["pass"] and ground_bridges["pass"]
        ),
        "via_in_pad_zero": bool(via_in_pad["pass"]),
        "four_copper_layers_f_in1_in2_b": bool(layers["pass"]),
        "in1_gnd_gnd2_plane_only": bool(in1_policy["pass"]),
        "in2_logic_3v3_local_ground_only": bool(in2_policy["pass"]),
        "inner_ground_copper_coverage_at_least_25_percent": bool(
            inner_ground_copper_coverage["pass"]
        ),
        "u2_left_u3_right_parity": bool(driver_contract["pass"]),
        "u5_no_ep49_no_central_paste": bool(u5["pass"]),
        "jlc_2oz_track_width": bool(tracks["jlc_width_pass"]),
        "jlc_2oz_spacing": bool(
            drc_result["violation_by_type"].get("clearance", 0) == 0
            and rules["checks"]["dru_has_two_oz_0p16_clearance"]
        ),
        "jlc_routed_edge_clearance": bool(
            drc_result["copper_edge_clearance"]["proven_at_or_above_threshold"]
        ),
        "jlc_via_hard_capability": bool(vias["jlc_via_pass"]),
        "project_via_0p40_0p20_floor": bool(vias["project_via_pass"]),
    }

    report: Dict[str, Any] = {
        "schema": "nightfall-mini3-final-pcb-audit-v2",
        "generated_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "input": input_info,
        "kicad": {key: value for key, value in kicad.items() if key != "drc"},
        "project_rules": rules,
        "drc": drc_result,
        "unconnected": unconnected,
        "connectivity": connectivity,
        "via_in_pad": via_in_pad,
        "vias": vias,
        "tracks": tracks,
        "layers": layers,
        "in1_policy": in1_policy,
        "in2_policy": in2_policy,
        "inner_ground_copper_coverage": inner_ground_copper_coverage,
        "driver_contract": driver_contract,
        "r0_contract": r0_contract,
        "ground_bridges": ground_bridges,
        "u5": u5,
        "jlcpcb": JLC,
        "release_checks": release_checks,
        "release_gate_pass": all(release_checks.values()),
        "manual_order_checks": [
            "Order as 4-layer FR-4 with 2 oz outer copper and confirm the selected JLCPCB stackup.",
            "If 0.40/0.20 mm vias remain, select JLCPCB's corresponding small-via option.",
            "Review KiCad warning-only footprint/text/solder-mask categories separately.",
            "Review Gerber and drill files in JLCPCB's online viewer before payment.",
        ],
        "artifacts": {
            "directory": str(work_dir),
            "audit_json": str(work_dir / "audit.json"),
            "audit_markdown": str(work_dir / "audit.md"),
            "drc_json": str(work_dir / "drc.json"),
            "schematic_netlist_xml": str(work_dir / "schematic-netlist.xml"),
            "refilled_board": str(board_path),
        },
    }

    audit_json = work_dir / "audit.json"
    audit_md = work_dir / "audit.md"
    audit_json.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    audit_md.write_text(build_markdown(report), encoding="utf-8")

    print("AUDIT_DIR={}".format(work_dir))
    print("AUDIT_JSON={}".format(audit_json))
    print("AUDIT_MD={}".format(audit_md))
    print("RELEASE_GATE={}".format("PASS" if report["release_gate_pass"] else "FAIL"))
    failed = [name for name, passed in release_checks.items() if not passed]
    print("FAILED_CHECKS={}".format(",".join(failed) if failed else "none"))
    if args.strict and not report["release_gate_pass"]:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
