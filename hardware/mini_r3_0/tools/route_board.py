#!/usr/bin/env python3
"""Conservative signal-routing assistant for the mini_r3 PCB.

This is deliberately *not* a power autorouter.  It preserves every existing
board item, never moves a footprint, and refuses to route the protected power,
ground, motor, fan-current, and MPM3610 switching nets listed below.  Its
intended workflow is:

1. Manually place the critical parts and route the power/current loops.
2. Save a checkpoint board.
3. Run this script with KiCad's bundled Python, writing a new board file.
4. Inspect every generated route in PCB Editor and run the full DRC.

The router builds a 0.10 mm occupancy grid from the board outline, copper pads,
existing tracks/vias, and copper keepouts.  For each eligible net it connects
the nearest disconnected pad groups using deterministic, octilinear A* paths
on the available routing layers.  Four-layer boards reserve In1.Cu for the
split GND/GND2 reference plane and route only on F.Cu, In2.Cu, and B.Cu.
New routes use 0.20 mm tracks and 0.60/0.30 mm through vias
by default.  General copper clearance is 0.20 mm; a 0.16 mm JLCPCB 2 oz process
minimum is allowed only within a short escape halo around fine-pitch pads.
Existing copper on the same net remains traversable, while all other copper is
expanded by the requested clearance.  The mini_r3 functional clearances are
also hard routing constraints: Sensitive nets use 0.25 mm, Sensitive-to-
HighCurrent spacing uses 0.40 mm, and every route stays 0.50 mm away from
SwitchNode copper.

This is routing assistance, not sign-off.  Raster routing is necessarily an
approximation of KiCad's exact geometry rules, so the output must always be
reviewed and checked with KiCad DRC before manufacture.
"""

from __future__ import annotations

import argparse
import dataclasses
import heapq
import json
import math
import os
from pathlib import Path
import re
import tempfile
from typing import Dict, List, Optional, Sequence, Set, Tuple

try:
    import numpy as np
    from scipy.ndimage import distance_transform_edt
except ImportError as exc:  # pragma: no cover - depends on KiCad installation
    raise SystemExit(
        "This tool needs NumPy and SciPy. Run it with KiCad's bundled Python "
        "rather than the system Python."
    ) from exc

try:
    import pcbnew
except ImportError as exc:  # pragma: no cover - depends on KiCad installation
    raise SystemExit(
        "pcbnew is unavailable. Example on macOS:\n"
        "  /Applications/KiCad/KiCad.app/Contents/Frameworks/"
        "Python.framework/Versions/3.9/bin/python3 route_board.py ..."
    ) from exc


FREE = -2
HARD = -1
MIXED = -3

FRONT = 0
BACK = 1
LAYER_IDS = (pcbnew.F_Cu, pcbnew.B_Cu)
ROUTING_LAYER_INDEXES = (FRONT, BACK)
MIN_SENSITIVE_CLEARANCE_MM = 0.25
MIN_HIGHCURRENT_SENSITIVE_CLEARANCE_MM = 0.40
MIN_SWITCH_CLEARANCE_MM = 0.50
MIN_HOLE_TO_HOLE_MM = 0.20
# Exact or deliberately narrow patterns.  In particular, /FAN_PWM and motor
# control nets remain eligible; only the corresponding current paths are
# protected.  A protected net cannot be overridden from the command line.
PROTECTED_NET_PATTERNS = tuple(
    re.compile(pattern)
    for pattern in (
        r"^$",
        r"^GND2?$",
        r"^\+(?:3V3|5V)$",
        r"^VBAT(?:_RAW|_SW)?$",
        r"^/FAN_NEG_INTERNAL$",
        r"^/MOTOR_[LR]_OUT[12]$",
        r"^/MPM_",
        r"^/PWR_GATE_INTERNAL$",
        r"^/PWR_SWITCH_RETURN$",
        r"^Net-\(D3-A\)$",
        r"^Net-\(U5B-VCAP_1\)$",
        r"^Net-\(IR_LED_.*-(?:C|PadA)\)$",
        r"^unconnected-",
    )
)

# These narrow fallbacks make the safety rules survive a board-only copy for
# which KiCad cannot find the adjacent .kicad_pro.  A whole-project copy is
# still mandatory for meaningful DRC comparison.
FALLBACK_NETCLASS_PATTERNS = (
    (
        "HighCurrent",
        re.compile(
            r"^(?:GND2|VBAT_RAW|VBAT_SW|/FAN_NEG_INTERNAL|"
            r"/MOTOR_[LR]_OUT[12])$"
        ),
    ),
    ("SwitchNode", re.compile(r"^/MPM_SW_INTERNAL$")),
    (
        "Sensitive",
        re.compile(r"^(?:/SENSOR_.*|/VOL_CHECK|/SPI2_.*|/IMU_CS)$"),
    ),
)


GridNode = Tuple[int, int, int]  # layer-index, y-index, x-index


@dataclasses.dataclass(frozen=True)
class RouterConfig:
    grid_mm: float
    trace_width_mm: float
    clearance_mm: float
    sensitive_clearance_mm: float
    highcurrent_sensitive_clearance_mm: float
    switch_clearance_mm: float
    escape_clearance_mm: float
    pad_escape_mm: float
    via_diameter_mm: float
    via_drill_mm: float
    hole_clearance_mm: float
    hole_to_hole_mm: float
    edge_clearance_mm: float
    via_cost_mm: float
    bend_cost_mm: float
    max_vias: int
    max_expanded: int
    max_net_expanded: int
    candidate_pairs: int
    front_axis: str
    net_order: str
    fanout_references: Tuple[str, ...]
    verbose: bool

    @property
    def trace_radius_mm(self) -> float:
        return self.trace_width_mm / 2.0

    @property
    def via_radius_mm(self) -> float:
        return self.via_diameter_mm / 2.0

    @property
    def raster_guard_mm(self) -> float:
        # Half a grid-cell diagonal prevents a diagonal grid edge from cutting
        # through an obstacle that lies between sampled cell centres.
        return self.grid_mm / math.sqrt(2.0)


@dataclasses.dataclass
class RouteMaterial:
    tracks: List[object]
    vias: List[object]


@dataclasses.dataclass
class NetReport:
    name: str
    pad_count: int
    initial_components: int
    remaining_components: int
    connections_added: int
    tracks_added: int
    vias_added: int
    expanded_nodes: int
    note: str = ""


class DisjointSet:
    def __init__(self, size: int) -> None:
        self.parent = list(range(size))
        self.rank = [0] * size

    def find(self, item: int) -> int:
        root = item
        while self.parent[root] != root:
            root = self.parent[root]
        while self.parent[item] != item:
            parent = self.parent[item]
            self.parent[item] = root
            item = parent
        return root

    def union(self, left: int, right: int) -> bool:
        left_root = self.find(left)
        right_root = self.find(right)
        if left_root == right_root:
            return False
        if self.rank[left_root] < self.rank[right_root]:
            left_root, right_root = right_root, left_root
        self.parent[right_root] = left_root
        if self.rank[left_root] == self.rank[right_root]:
            self.rank[left_root] += 1
        return True

    def component_count(self) -> int:
        return len({self.find(index) for index in range(len(self.parent))})


def mm(value_iu: int) -> float:
    return float(pcbnew.ToMM(value_iu))


def iu(value_mm: float) -> int:
    return int(pcbnew.FromMM(value_mm))


def point_key(item: object) -> str:
    """Return a stable KiCad UUID string for a connected board item."""
    return str(item.m_Uuid.AsString())


def pad_label(pad: object) -> str:
    footprint = pad.GetParentFootprint()
    return f"{footprint.GetReference()}.{pad.GetNumber()}"


def is_protected_net(name: str) -> bool:
    return any(pattern.search(name) for pattern in PROTECTED_NET_PATTERNS)


def effective_netclass(net_info: object) -> str:
    """Return the project netclass, with conservative mini_r3 fallbacks."""
    reported = str(net_info.GetNetClassName())
    if reported in {"HighCurrent", "Sensitive", "SwitchNode"}:
        return reported
    name = str(net_info.GetNetname())
    for netclass, pattern in FALLBACK_NETCLASS_PATTERNS:
        if pattern.search(name):
            return netclass
    return reported


def pad_layers(pad: object) -> Tuple[int, ...]:
    layers = tuple(
        index
        for index, layer_id in enumerate(LAYER_IDS)
        if bool(pad.FlashLayer(layer_id))
    )
    return layers


def configure_routing_layers(board: object) -> None:
    """Select routing layers while keeping In1 as a four-layer plane."""
    global BACK, LAYER_IDS, ROUTING_LAYER_INDEXES
    copper_layers = int(board.GetCopperLayerCount())
    if copper_layers == 2:
        LAYER_IDS = (pcbnew.F_Cu, pcbnew.B_Cu)
    elif copper_layers == 4:
        LAYER_IDS = (pcbnew.F_Cu, pcbnew.In2_Cu, pcbnew.B_Cu)
    else:
        raise SystemExit(
            "this assistant supports two- or four-layer boards; found "
            f"{copper_layers} copper layers"
        )
    BACK = len(LAYER_IDS) - 1
    ROUTING_LAYER_INDEXES = tuple(range(len(LAYER_IDS)))


class OccupancyGrid:
    """Copper ownership and edge-distance grids used by the A* router."""

    def __init__(self, board: object, config: RouterConfig) -> None:
        self.board = board
        self.config = config
        self.netclass_by_code = {
            int(net_info.GetNetCode()): effective_netclass(net_info)
            for _, net_info in board.GetNetInfo().NetsByName().items()
        }

        outline = pcbnew.SHAPE_POLY_SET()
        if not board.GetBoardPolygonOutlines(outline, False):
            raise RuntimeError("KiCad could not construct a closed board outline")
        if outline.OutlineCount() == 0:
            raise RuntimeError("The board has no closed Edge.Cuts outline")
        self.outline = outline

        bbox = outline.BBox()
        margin = max(1.0, config.via_radius_mm + config.edge_clearance_mm)
        self.x0_mm = (
            math.floor((mm(bbox.GetLeft()) - margin) / config.grid_mm)
            * config.grid_mm
        )
        self.y0_mm = (
            math.floor((mm(bbox.GetTop()) - margin) / config.grid_mm)
            * config.grid_mm
        )
        x1_mm = (
            math.ceil((mm(bbox.GetRight()) + margin) / config.grid_mm)
            * config.grid_mm
        )
        y1_mm = (
            math.ceil((mm(bbox.GetBottom()) + margin) / config.grid_mm)
            * config.grid_mm
        )
        self.nx = int(round((x1_mm - self.x0_mm) / config.grid_mm)) + 1
        self.ny = int(round((y1_mm - self.y0_mm) / config.grid_mm)) + 1

        self.inside = np.zeros((self.ny, self.nx), dtype=np.bool_)
        for y_index in range(self.ny):
            y_iu = iu(self.y0_mm + y_index * config.grid_mm)
            for x_index in range(self.nx):
                point = pcbnew.VECTOR2I(
                    iu(self.x0_mm + x_index * config.grid_mm), y_iu
                )
                self.inside[y_index, x_index] = bool(outline.Contains(point))

        distance_mm = distance_transform_edt(self.inside) * config.grid_mm
        trace_edge_margin = (
            config.trace_radius_mm
            + config.edge_clearance_mm
            + config.raster_guard_mm
        )
        via_edge_margin = (
            config.via_radius_mm
            + config.edge_clearance_mm
            + config.raster_guard_mm
        )
        self.trace_inside = self.inside & (distance_mm >= trace_edge_margin)
        self.via_inside = self.inside & (distance_mm >= via_edge_margin)

        layer_count = len(LAYER_IDS)
        grid_shape = (layer_count, self.ny, self.nx)
        self.trace_owner = np.full(grid_shape, FREE, dtype=np.int32)
        self.escape_owner = np.full(grid_shape, FREE, dtype=np.int32)
        self.via_owner = np.full(grid_shape, FREE, dtype=np.int32)
        self.sensitive_trace_owner = np.full(
            grid_shape, FREE, dtype=np.int32
        )
        self.sensitive_via_owner = np.full(
            grid_shape, FREE, dtype=np.int32
        )
        self.sensitive_trace_blocked = np.zeros(
            grid_shape, dtype=np.bool_
        )
        self.sensitive_via_blocked = np.zeros(
            grid_shape, dtype=np.bool_
        )
        self.highcurrent_trace_blocked = np.zeros(
            grid_shape, dtype=np.bool_
        )
        self.highcurrent_via_blocked = np.zeros(
            grid_shape, dtype=np.bool_
        )
        self.switch_trace_blocked = np.zeros(
            grid_shape, dtype=np.bool_
        )
        self.switch_via_blocked = np.zeros(
            grid_shape, dtype=np.bool_
        )
        self.via_pad_blocked = np.zeros(grid_shape, dtype=np.bool_)
        self.via_hole_blocked = np.zeros((self.ny, self.nx), dtype=np.bool_)

        self.unsafe_pad_uuids = self._find_overlapping_pads()
        self._build_copper_obstacles()
        self._build_keepout_obstacles()

    def _find_overlapping_pads(self) -> Set[str]:
        """Find pads already copper-shorted to a different net.

        Starting a new track inside an existing pad-to-pad short makes KiCad
        report additional shorting items even when the route itself follows
        all clearance rules.  Such pads are left for manual repair/routing.
        """
        pads = [pad for pad in self.board.GetPads() if pad.GetNetCode() > 0]
        unsafe: Set[str] = set()
        for left_index, left in enumerate(pads):
            for right in pads[left_index + 1 :]:
                if left.GetNetCode() == right.GetNetCode():
                    continue
                for layer_id in LAYER_IDS:
                    if not left.FlashLayer(layer_id) or not right.FlashLayer(layer_id):
                        continue
                    left_shape = left.GetEffectiveShape(layer_id)
                    right_shape = right.GetEffectiveShape(layer_id)
                    if not left_shape.BBox().Intersects(right_shape.BBox()):
                        continue
                    if left_shape.Collide(right_shape, 0):
                        unsafe.add(point_key(left))
                        unsafe.add(point_key(right))
                        break
        return unsafe

    def point_for(self, x_index: int, y_index: int) -> object:
        return pcbnew.VECTOR2I(
            iu(self.x0_mm + x_index * self.config.grid_mm),
            iu(self.y0_mm + y_index * self.config.grid_mm),
        )

    def index_for(self, position: object) -> Tuple[int, int]:
        x_index = int(round((mm(position.x) - self.x0_mm) / self.config.grid_mm))
        y_index = int(round((mm(position.y) - self.y0_mm) / self.config.grid_mm))
        return x_index, y_index

    def in_bounds(self, x_index: int, y_index: int) -> bool:
        return 0 <= x_index < self.nx and 0 <= y_index < self.ny

    @staticmethod
    def _merge_owner(current: int, owner: int) -> int:
        if owner <= 0:
            return HARD
        if current == FREE or current == owner:
            return owner
        if current == HARD:
            return HARD
        return MIXED

    def _shape_bounds(
        self, shape: object, expansion_mm: float
    ) -> Tuple[int, int, int, int]:
        bbox = shape.BBox()
        x_min = max(
            0,
            int(
                math.floor(
                    (mm(bbox.GetLeft()) - expansion_mm - self.x0_mm)
                    / self.config.grid_mm
                )
            ),
        )
        x_max = min(
            self.nx - 1,
            int(
                math.ceil(
                    (mm(bbox.GetRight()) + expansion_mm - self.x0_mm)
                    / self.config.grid_mm
                )
            ),
        )
        y_min = max(
            0,
            int(
                math.floor(
                    (mm(bbox.GetTop()) - expansion_mm - self.y0_mm)
                    / self.config.grid_mm
                )
            ),
        )
        y_max = min(
            self.ny - 1,
            int(
                math.ceil(
                    (mm(bbox.GetBottom()) + expansion_mm - self.y0_mm)
                    / self.config.grid_mm
                )
            ),
        )
        return x_min, x_max, y_min, y_max

    def _raster_shape_owner(
        self,
        shape: object,
        layer_index: int,
        owner: int,
        expansion_mm: float,
        destination: np.ndarray,
    ) -> None:
        x_min, x_max, y_min, y_max = self._shape_bounds(shape, expansion_mm)
        clearance_iu = iu(expansion_mm)
        layer_grid = destination[layer_index]
        for y_index in range(y_min, y_max + 1):
            for x_index in range(x_min, x_max + 1):
                if shape.Collide(self.point_for(x_index, y_index), clearance_iu):
                    layer_grid[y_index, x_index] = self._merge_owner(
                        int(layer_grid[y_index, x_index]), owner
                    )

    def _raster_shape_bool(
        self,
        shape: object,
        layer_index: int,
        expansion_mm: float,
        destination: np.ndarray,
    ) -> None:
        x_min, x_max, y_min, y_max = self._shape_bounds(shape, expansion_mm)
        clearance_iu = iu(expansion_mm)
        layer_grid = destination[layer_index]
        for y_index in range(y_min, y_max + 1):
            for x_index in range(x_min, x_max + 1):
                if shape.Collide(self.point_for(x_index, y_index), clearance_iu):
                    layer_grid[y_index, x_index] = True

    def _add_copper_shape(
        self, shape: object, layer_index: int, net_code: int
    ) -> None:
        self._raster_shape_owner(
            shape,
            layer_index,
            net_code,
            self.config.clearance_mm
            + self.config.trace_radius_mm
            + self.config.raster_guard_mm,
            self.trace_owner,
        )
        self._raster_shape_owner(
            shape,
            layer_index,
            net_code,
            self.config.escape_clearance_mm
            + self.config.trace_radius_mm
            + self.config.raster_guard_mm,
            self.escape_owner,
        )
        self._raster_shape_owner(
            shape,
            layer_index,
            net_code,
            self.config.clearance_mm
            + self.config.via_radius_mm
            + self.config.raster_guard_mm,
            self.via_owner,
        )

        # Netclass-aware grids enforce the functional .kicad_dru rules during
        # A* search instead of merely discovering violations afterwards.
        self._raster_shape_owner(
            shape,
            layer_index,
            net_code,
            self.config.sensitive_clearance_mm
            + self.config.trace_radius_mm
            + self.config.raster_guard_mm,
            self.sensitive_trace_owner,
        )
        self._raster_shape_owner(
            shape,
            layer_index,
            net_code,
            self.config.sensitive_clearance_mm
            + self.config.via_radius_mm
            + self.config.raster_guard_mm,
            self.sensitive_via_owner,
        )
        netclass = self.netclass_by_code.get(net_code, "Default")
        if netclass == "HighCurrent":
            self._raster_shape_bool(
                shape,
                layer_index,
                self.config.highcurrent_sensitive_clearance_mm
                + self.config.trace_radius_mm
                + self.config.raster_guard_mm,
                self.highcurrent_trace_blocked,
            )
            self._raster_shape_bool(
                shape,
                layer_index,
                self.config.highcurrent_sensitive_clearance_mm
                + self.config.via_radius_mm
                + self.config.raster_guard_mm,
                self.highcurrent_via_blocked,
            )
        elif netclass == "SwitchNode":
            self._raster_shape_bool(
                shape,
                layer_index,
                self.config.switch_clearance_mm
                + self.config.trace_radius_mm
                + self.config.raster_guard_mm,
                self.switch_trace_blocked,
            )
            self._raster_shape_bool(
                shape,
                layer_index,
                self.config.switch_clearance_mm
                + self.config.via_radius_mm
                + self.config.raster_guard_mm,
                self.switch_via_blocked,
            )
        elif netclass == "Sensitive":
            self._raster_shape_bool(
                shape,
                layer_index,
                self.config.sensitive_clearance_mm
                + self.config.trace_radius_mm
                + self.config.raster_guard_mm,
                self.sensitive_trace_blocked,
            )
            self._raster_shape_bool(
                shape,
                layer_index,
                self.config.sensitive_clearance_mm
                + self.config.via_radius_mm
                + self.config.raster_guard_mm,
                self.sensitive_via_blocked,
            )

    def _build_copper_obstacles(self) -> None:
        for pad in self.board.GetPads():
            for layer_index, layer_id in enumerate(LAYER_IDS):
                if not pad.FlashLayer(layer_id):
                    continue
                shape = pad.GetEffectiveShape(layer_id)
                self._add_copper_shape(shape, layer_index, pad.GetNetCode())
                # Via-in-pad is intentionally disallowed, including on the
                # routed net.  It is a poor default for inexpensive assembly.
                self._raster_shape_bool(
                    shape,
                    layer_index,
                    self.config.grid_mm / 2.0,
                    self.via_pad_blocked,
                )
            if pad.HasHole():
                hole_shape = pad.GetEffectiveHoleShape()
                self._raster_hole_for_new_via(hole_shape)
                if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                    self._raster_npth_for_tracks(hole_shape)

        for item in self.board.GetTracks():
            if isinstance(item, pcbnew.PCB_VIA):
                for layer_index, layer_id in enumerate(LAYER_IDS):
                    if item.IsOnLayer(layer_id):
                        self._add_copper_shape(
                            item.GetEffectiveShape(layer_id),
                            layer_index,
                            item.GetNetCode(),
                        )
                self._raster_hole_for_new_via(item.GetEffectiveHoleShape())
            else:
                for layer_index, layer_id in enumerate(LAYER_IDS):
                    if item.IsOnLayer(layer_id):
                        self._add_copper_shape(
                            item.GetEffectiveShape(layer_id),
                            layer_index,
                            item.GetNetCode(),
                        )

    def _raster_hole_for_new_via(self, hole_shape: object) -> None:
        expansion_mm = (
            max(
                self.config.hole_clearance_mm + self.config.via_radius_mm,
                self.config.hole_to_hole_mm
                + self.config.via_drill_mm / 2.0,
            )
            + self.config.raster_guard_mm
        )
        x_min, x_max, y_min, y_max = self._shape_bounds(
            hole_shape, expansion_mm
        )
        clearance_iu = iu(expansion_mm)
        for y_index in range(y_min, y_max + 1):
            for x_index in range(x_min, x_max + 1):
                if hole_shape.Collide(
                    self.point_for(x_index, y_index), clearance_iu
                ):
                    self.via_hole_blocked[y_index, x_index] = True

    def _raster_npth_for_tracks(self, hole_shape: object) -> None:
        expansion_mm = (
            max(self.config.clearance_mm, self.config.hole_clearance_mm)
            + self.config.trace_radius_mm
            + self.config.raster_guard_mm
        )
        for layer_index in ROUTING_LAYER_INDEXES:
            self._raster_shape_owner(
                hole_shape,
                layer_index,
                HARD,
                expansion_mm,
                self.trace_owner,
            )
            self._raster_shape_owner(
                hole_shape,
                layer_index,
                HARD,
                expansion_mm,
                self.escape_owner,
            )

    def _build_keepout_obstacles(self) -> None:
        for zone in self.board.Zones():
            # KiCad 9's Python wrapper reports the keepout flags as set on
            # ordinary copper-pour zones as well.  Only rule-area zones are
            # actual routing keepouts; treating GND/GND2 pours as keepouts
            # makes every legal plane-referenced route appear impossible.
            if not zone.GetIsRuleArea():
                continue
            if not zone.HasKeepoutParametersSet():
                continue
            for layer_index, layer_id in enumerate(LAYER_IDS):
                if not zone.IsOnLayer(layer_id):
                    continue
                shape = zone.GetEffectiveShape(layer_id)
                if zone.GetDoNotAllowTracks():
                    self._raster_shape_owner(
                        shape,
                        layer_index,
                        HARD,
                        self.config.trace_radius_mm
                        + self.config.raster_guard_mm,
                        self.trace_owner,
                    )
                    self._raster_shape_owner(
                        shape,
                        layer_index,
                        HARD,
                        self.config.trace_radius_mm
                        + self.config.raster_guard_mm,
                        self.escape_owner,
                    )
                if zone.GetDoNotAllowVias():
                    self._raster_shape_owner(
                        shape,
                        layer_index,
                        HARD,
                        self.config.via_radius_mm
                        + self.config.raster_guard_mm,
                        self.via_owner,
                    )

    def own_pad_masks(
        self, pads: Sequence[object]
    ) -> Tuple[np.ndarray, np.ndarray]:
        shape = (len(LAYER_IDS), self.ny, self.nx)
        pad_mask = np.zeros(shape, dtype=np.bool_)
        escape_mask = np.zeros(shape, dtype=np.bool_)
        for pad in pads:
            for layer_index, layer_id in enumerate(LAYER_IDS):
                if not pad.FlashLayer(layer_id):
                    continue
                shape = pad.GetEffectiveShape(layer_id)
                self._raster_shape_bool(
                    shape, layer_index, 0.0, pad_mask
                )
                footprint = pad.GetParentFootprint()
                size = pad.GetSize()
                fine_pitch = (
                    pad.GetAttribute() == pcbnew.PAD_ATTRIB_SMD
                    and min(mm(size.x), mm(size.y)) <= 0.40
                )
                if footprint.GetReference() == "K1" or fine_pitch:
                    self._raster_shape_bool(
                        shape,
                        layer_index,
                        self.config.pad_escape_mm,
                        escape_mask,
                    )
        return pad_mask, escape_mask

    def trace_passable(
        self,
        layer: int,
        x_index: int,
        y_index: int,
        net_code: int,
        own_pads: np.ndarray,
        own_escape: np.ndarray,
    ) -> bool:
        if not self.in_bounds(x_index, y_index):
            return False
        if not self.inside[y_index, x_index]:
            return False
        netclass = self.netclass_by_code.get(net_code, "Default")
        if self.switch_trace_blocked[layer, y_index, x_index]:
            return False
        if (
            netclass != "Sensitive"
            and self.sensitive_trace_blocked[layer, y_index, x_index]
        ):
            return False
        if (
            netclass == "Sensitive"
            and self.highcurrent_trace_blocked[layer, y_index, x_index]
        ):
            return False
        if own_pads[layer, y_index, x_index]:
            return True
        if not self.trace_inside[y_index, x_index]:
            return False
        owner = int(self.trace_owner[layer, y_index, x_index])
        normal_clear = owner in (FREE, net_code)
        if normal_clear and netclass == "Sensitive":
            sensitive_owner = int(
                self.sensitive_trace_owner[layer, y_index, x_index]
            )
            normal_clear = sensitive_owner in (FREE, net_code)
        if normal_clear:
            return True
        # Fine-pitch pads and the 1.27 mm programming connector cannot escape
        # with 0.20 mm copper clearance, even though the JLCPCB 2 oz process
        # supports 0.16 mm.  Permit that process minimum only in a short halo
        # around a pad of this net; the rest of every route stays at the normal
        # clearance.
        escape_owner = int(self.escape_owner[layer, y_index, x_index])
        return (
            bool(own_escape[layer, y_index, x_index])
            and escape_owner in (FREE, net_code)
        )

    def via_passable(
        self, x_index: int, y_index: int, net_code: int
    ) -> bool:
        if not self.in_bounds(x_index, y_index):
            return False
        if not self.via_inside[y_index, x_index]:
            return False
        if self.via_hole_blocked[y_index, x_index]:
            return False
        netclass = self.netclass_by_code.get(net_code, "Default")
        for layer in ROUTING_LAYER_INDEXES:
            if self.via_pad_blocked[layer, y_index, x_index]:
                return False
            if self.switch_via_blocked[layer, y_index, x_index]:
                return False
            if (
                netclass != "Sensitive"
                and self.sensitive_via_blocked[layer, y_index, x_index]
            ):
                return False
            if (
                netclass == "Sensitive"
                and self.highcurrent_via_blocked[layer, y_index, x_index]
            ):
                return False
            owner = int(self.via_owner[layer, y_index, x_index])
            if owner not in (FREE, net_code):
                return False
            if netclass == "Sensitive":
                sensitive_owner = int(
                    self.sensitive_via_owner[layer, y_index, x_index]
                )
                if sensitive_owner not in (FREE, net_code):
                    return False
        return True

    def add_material(self, material: RouteMaterial) -> None:
        for track in material.tracks:
            layer_index = LAYER_IDS.index(track.GetLayer())
            self._add_copper_shape(
                track.GetEffectiveShape(track.GetLayer()),
                layer_index,
                track.GetNetCode(),
            )
        for via in material.vias:
            for layer_index, layer_id in enumerate(LAYER_IDS):
                self._add_copper_shape(
                    via.GetEffectiveShape(layer_id),
                    layer_index,
                    via.GetNetCode(),
                )
            self._raster_hole_for_new_via(via.GetEffectiveHoleShape())


class AStarRouter:
    DIRECTIONS = (
        (-1, 0),
        (-1, -1),
        (0, -1),
        (1, -1),
        (1, 0),
        (1, 1),
        (0, 1),
        (-1, 1),
    )

    def __init__(self, occupancy: OccupancyGrid, config: RouterConfig) -> None:
        self.occupancy = occupancy
        self.config = config

    def _heuristic(
        self, node: GridNode, goal_x: int, goal_y: int, goal_layers: Set[int]
    ) -> float:
        layer, y_index, x_index = node
        dx = abs(goal_x - x_index)
        dy = abs(goal_y - y_index)
        diagonal = min(dx, dy)
        straight = max(dx, dy) - diagonal
        distance = diagonal * math.sqrt(2.0) + straight
        if layer not in goal_layers:
            distance += self.config.via_cost_mm / self.config.grid_mm
        return distance

    def _axis_penalty(self, layer: int, dx: int, dy: int) -> float:
        if dx and dy:
            return 0.03
        front_prefers_vertical = self.config.front_axis == "vertical"
        if layer not in (FRONT, BACK):
            return 0.0 if dx != 0 else 0.04
        preferred = (
            (layer == FRONT and ((dy != 0) == front_prefers_vertical))
            or (layer == BACK and ((dx != 0) == front_prefers_vertical))
        )
        return 0.0 if preferred else 0.08

    def route(
        self,
        source: object,
        target: object,
        net_code: int,
        own_pads: np.ndarray,
        own_escape: np.ndarray,
        expansion_limit: int,
    ) -> Tuple[Optional[List[GridNode]], int, str]:
        source_x, source_y = self.occupancy.index_for(source.GetPosition())
        target_x, target_y = self.occupancy.index_for(target.GetPosition())
        source_layers = set(pad_layers(source))
        target_layers = set(pad_layers(target))
        if not source_layers or not target_layers:
            return None, 0, "a pad has no routable outer-layer copper"
        if not self.occupancy.in_bounds(source_x, source_y):
            return None, 0, f"{pad_label(source)} is outside the routing grid"
        if not self.occupancy.in_bounds(target_x, target_y):
            return None, 0, f"{pad_label(target)} is outside the routing grid"
        if not self.occupancy.inside[source_y, source_x]:
            return None, 0, f"{pad_label(source)} centre is outside Edge.Cuts"
        if not self.occupancy.inside[target_y, target_x]:
            return None, 0, f"{pad_label(target)} centre is outside Edge.Cuts"

        # Via count is part of the state.  Otherwise a cheaper path using too
        # many vias can suppress a slightly longer valid path within the limit.
        shape = (
            self.config.max_vias + 1,
            len(LAYER_IDS),
            self.occupancy.ny,
            self.occupancy.nx,
        )
        g_score = np.full(shape, np.inf, dtype=np.float64)
        parent = np.full(shape, -1, dtype=np.int16)
        parent_layer = np.full(shape, -1, dtype=np.int8)
        closed = np.zeros(shape, dtype=np.bool_)
        queue: List[Tuple[float, float, int, int, int, int]] = []
        for layer in sorted(source_layers):
            g_score[0, layer, source_y, source_x] = 0.0
            parent[0, layer, source_y, source_x] = -2
            start = (layer, source_y, source_x)
            heapq.heappush(
                queue,
                (
                    self._heuristic(start, target_x, target_y, target_layers),
                    0.0,
                    0,
                    layer,
                    source_y,
                    source_x,
                ),
            )

        expanded = 0
        goal: Optional[Tuple[int, int, int, int]] = None
        while queue:
            (
                _,
                current_g,
                vias_used,
                layer,
                y_index,
                x_index,
            ) = heapq.heappop(queue)
            if closed[vias_used, layer, y_index, x_index]:
                continue
            if (
                current_g
                > float(g_score[vias_used, layer, y_index, x_index]) + 1e-9
            ):
                continue
            closed[vias_used, layer, y_index, x_index] = True
            expanded += 1
            if expanded > expansion_limit:
                return None, expanded, "A* expansion limit reached"
            if x_index == target_x and y_index == target_y and layer in target_layers:
                goal = (vias_used, layer, y_index, x_index)
                break

            for direction, (dx, dy) in enumerate(self.DIRECTIONS):
                next_x = x_index + dx
                next_y = y_index + dy
                if not self.occupancy.trace_passable(
                    layer, next_x, next_y, net_code, own_pads, own_escape
                ):
                    continue
                if dx and dy:
                    # Do not cut diagonally between two blocked orthogonal
                    # neighbours.  This is stricter than centre-only sampling.
                    if not self.occupancy.trace_passable(
                        layer,
                        x_index + dx,
                        y_index,
                        net_code,
                        own_pads,
                        own_escape,
                    ):
                        continue
                    if not self.occupancy.trace_passable(
                        layer,
                        x_index,
                        y_index + dy,
                        net_code,
                        own_pads,
                        own_escape,
                    ):
                        continue
                step = math.sqrt(2.0) if dx and dy else 1.0
                step += self._axis_penalty(layer, dx, dy)
                previous_direction = int(
                    parent[vias_used, layer, y_index, x_index]
                )
                if 0 <= previous_direction < len(self.DIRECTIONS):
                    if previous_direction != direction:
                        step += self.config.bend_cost_mm / self.config.grid_mm
                candidate_g = current_g + step
                if (
                    candidate_g + 1e-9
                    >= g_score[vias_used, layer, next_y, next_x]
                ):
                    continue
                g_score[vias_used, layer, next_y, next_x] = candidate_g
                parent[vias_used, layer, next_y, next_x] = direction
                node = (layer, next_y, next_x)
                heapq.heappush(
                    queue,
                    (
                        candidate_g
                        + self._heuristic(node, target_x, target_y, target_layers),
                        candidate_g,
                        vias_used,
                        layer,
                        next_y,
                        next_x,
                    ),
                )

            if (
                vias_used < self.config.max_vias
                and self.occupancy.via_passable(x_index, y_index, net_code)
            ):
                next_vias = vias_used + 1
                via_step = self.config.via_cost_mm / self.config.grid_mm
                for other_layer in ROUTING_LAYER_INDEXES:
                    if other_layer == layer:
                        continue
                    candidate_g = current_g + via_step
                    if (
                        candidate_g + 1e-9
                        < g_score[next_vias, other_layer, y_index, x_index]
                    ):
                        g_score[next_vias, other_layer, y_index, x_index] = candidate_g
                        parent[next_vias, other_layer, y_index, x_index] = 8
                        parent_layer[
                            next_vias, other_layer, y_index, x_index
                        ] = layer
                        node = (other_layer, y_index, x_index)
                        heapq.heappush(
                            queue,
                            (
                                candidate_g
                                + self._heuristic(
                                    node, target_x, target_y, target_layers
                                ),
                                candidate_g,
                                next_vias,
                                other_layer,
                                y_index,
                                x_index,
                            ),
                        )

        if goal is None:
            return None, expanded, "no obstacle-clear path"

        path: List[GridNode] = []
        vias_used, layer, y_index, x_index = goal
        while True:
            path.append((layer, y_index, x_index))
            direction = int(parent[vias_used, layer, y_index, x_index])
            if direction == -2:
                break
            if direction == 8:
                previous_layer = int(
                    parent_layer[vias_used, layer, y_index, x_index]
                )
                if previous_layer < 0:
                    return None, expanded, "invalid A* via parent chain"
                layer = previous_layer
                vias_used -= 1
                continue
            if direction < 0 or direction >= len(self.DIRECTIONS):
                return None, expanded, "invalid A* parent chain"
            dx, dy = self.DIRECTIONS[direction]
            x_index -= dx
            y_index -= dy
        path.reverse()
        return path, expanded, ""


def simplify_points(points: Sequence[object]) -> List[object]:
    unique: List[object] = []
    for point in points:
        if unique and point.x == unique[-1].x and point.y == unique[-1].y:
            continue
        unique.append(point)
    if len(unique) < 3:
        return unique
    simplified = [unique[0]]
    for point in unique[1:]:
        if len(simplified) < 2:
            simplified.append(point)
            continue
        first = simplified[-2]
        middle = simplified[-1]
        cross = (middle.x - first.x) * (point.y - middle.y) - (
            middle.y - first.y
        ) * (point.x - middle.x)
        if cross == 0:
            simplified[-1] = point
        else:
            simplified.append(point)
    return simplified


def materialize_path(
    board: object,
    occupancy: OccupancyGrid,
    path: Sequence[GridNode],
    source: object,
    target: object,
    net_info: object,
    config: RouterConfig,
) -> RouteMaterial:
    if not path:
        raise ValueError("cannot materialize an empty route")

    tracks: List[object] = []
    vias: List[object] = []
    run_layer = path[0][0]
    run_points = [source.GetPosition()]
    first = path[0]
    run_points.append(occupancy.point_for(first[2], first[1]))

    def finish_run(points: Sequence[object], layer_index: int) -> None:
        simplified = simplify_points(points)
        for start, end in zip(simplified, simplified[1:]):
            track = pcbnew.PCB_TRACK(board)
            track.SetLayer(LAYER_IDS[layer_index])
            track.SetStart(start)
            track.SetEnd(end)
            track.SetWidth(iu(config.trace_width_mm))
            track.SetNet(net_info)
            board.Add(track)
            tracks.append(track)

    for previous, current in zip(path, path[1:]):
        previous_layer, previous_y, previous_x = previous
        current_layer, current_y, current_x = current
        if previous_layer == current_layer:
            run_points.append(occupancy.point_for(current_x, current_y))
            continue
        finish_run(run_points, run_layer)
        via_position = occupancy.point_for(previous_x, previous_y)
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(via_position)
        via.SetWidth(iu(config.via_diameter_mm))
        via.SetDrill(iu(config.via_drill_mm))
        via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        via.SetNet(net_info)
        board.Add(via)
        vias.append(via)
        run_layer = current_layer
        run_points = [occupancy.point_for(current_x, current_y)]

    run_points.append(target.GetPosition())
    finish_run(run_points, run_layer)
    return RouteMaterial(tracks=tracks, vias=vias)


def initial_connectivity(board: object, pads: Sequence[object]) -> DisjointSet:
    dsu = DisjointSet(len(pads))
    if not pads:
        return dsu
    board.BuildConnectivity()
    connectivity = board.GetConnectivity()
    pad_by_uuid = {point_key(pad): index for index, pad in enumerate(pads)}
    for index, pad in enumerate(pads):
        try:
            connected = connectivity.GetConnectedItems(pad)
        except Exception:
            # Keep the tool useful on a KiCad build with reduced SWIG
            # connectivity bindings.  In that case, only exact coincident pads
            # are pre-unioned below; existing copper still remains an obstacle.
            connected = []
        for item in connected:
            if not isinstance(item, pcbnew.PAD):
                continue
            other = pad_by_uuid.get(point_key(item))
            if other is not None:
                dsu.union(index, other)
    for left in range(len(pads)):
        left_position = pads[left].GetPosition()
        for right in range(left + 1, len(pads)):
            right_position = pads[right].GetPosition()
            if (
                left_position.x == right_position.x
                and left_position.y == right_position.y
            ):
                copper_overlap = any(
                    pads[left].FlashLayer(layer_id)
                    and pads[right].FlashLayer(layer_id)
                    and pads[left]
                    .GetEffectiveShape(layer_id)
                    .Collide(pads[right].GetEffectiveShape(layer_id), 0)
                    for layer_id in LAYER_IDS
                )
                if copper_overlap:
                    dsu.union(left, right)
    return dsu


def candidate_pad_pairs(
    pads: Sequence[object],
    dsu: DisjointSet,
    limit: int,
    forbidden: Set[int],
    fanout_references: Set[str],
) -> List[Tuple[int, int]]:
    component_sizes: Dict[int, int] = {}
    for index in range(len(pads)):
        root = dsu.find(index)
        component_sizes[root] = component_sizes.get(root, 0) + 1
    active_fanout = {
        index
        for index, pad in enumerate(pads)
        if pad.GetParentFootprint().GetReference() in fanout_references
        and component_sizes[dsu.find(index)] == 1
    }

    candidates: List[Tuple[int, int, str, str, int, int]] = []
    for left in range(len(pads)):
        if left in forbidden:
            continue
        left_position = pads[left].GetPosition()
        for right in range(left + 1, len(pads)):
            if right in forbidden:
                continue
            if dsu.find(left) == dsu.find(right):
                continue
            right_position = pads[right].GetPosition()
            distance_squared = (left_position.x - right_position.x) ** 2 + (
                left_position.y - right_position.y
            ) ** 2
            candidates.append(
                (
                    0 if left in active_fanout or right in active_fanout else 1,
                    distance_squared,
                    pad_label(pads[left]),
                    pad_label(pads[right]),
                    left,
                    right,
                )
            )
    candidates.sort()
    return [(item[4], item[5]) for item in candidates[:limit]]


def estimated_tree_length_mm(pads: Sequence[object]) -> float:
    if len(pads) < 2:
        return 0.0
    reached = {0}
    total_iu = 0.0
    while len(reached) < len(pads):
        best: Optional[Tuple[float, int]] = None
        for left in reached:
            left_position = pads[left].GetPosition()
            for right in range(len(pads)):
                if right in reached:
                    continue
                right_position = pads[right].GetPosition()
                distance = math.hypot(
                    left_position.x - right_position.x,
                    left_position.y - right_position.y,
                )
                if best is None or distance < best[0]:
                    best = (distance, right)
        assert best is not None
        total_iu += best[0]
        reached.add(best[1])
    return mm(int(total_iu))


def route_net(
    board: object,
    occupancy: OccupancyGrid,
    router: AStarRouter,
    net_info: object,
    pads: Sequence[object],
    config: RouterConfig,
) -> NetReport:
    name = str(net_info.GetNetname())
    dsu = initial_connectivity(board, pads)
    initial_components = dsu.component_count()
    unsafe_indices = {
        index
        for index, pad in enumerate(pads)
        if point_key(pad) in occupancy.unsafe_pad_uuids
    }
    safe_pads = [
        pad for index, pad in enumerate(pads) if index not in unsafe_indices
    ]
    own_pads, own_escape = occupancy.own_pad_masks(safe_pads)
    connections = 0
    tracks = 0
    vias = 0
    expanded_total = 0
    last_failures: List[str] = []

    while dsu.component_count() > 1:
        routed = False
        failures: List[str] = []
        if expanded_total >= config.max_net_expanded:
            last_failures = [
                f"per-net A* budget {config.max_net_expanded} reached"
            ]
            break
        for left, right in candidate_pad_pairs(
            pads,
            dsu,
            config.candidate_pairs,
            unsafe_indices,
            set(config.fanout_references),
        ):
            remaining_budget = config.max_net_expanded - expanded_total
            if remaining_budget <= 0:
                failures.append(
                    f"per-net A* budget {config.max_net_expanded} reached"
                )
                break
            path, expanded, reason = router.route(
                pads[left],
                pads[right],
                net_info.GetNetCode(),
                own_pads,
                own_escape,
                min(config.max_expanded, remaining_budget),
            )
            expanded_total += expanded
            if path is None:
                failures.append(
                    f"{pad_label(pads[left])}->{pad_label(pads[right])}: {reason}"
                )
                continue
            material = materialize_path(
                board,
                occupancy,
                path,
                pads[left],
                pads[right],
                net_info,
                config,
            )
            occupancy.add_material(material)
            dsu.union(left, right)
            connections += 1
            tracks += len(material.tracks)
            vias += len(material.vias)
            routed = True
            if config.verbose:
                print(
                    f"  {name}: {pad_label(pads[left])} -> "
                    f"{pad_label(pads[right])}, {len(material.tracks)} tracks, "
                    f"{len(material.vias)} vias, {expanded} A* nodes",
                    flush=True,
                )
            break
        if not routed:
            last_failures = failures
            break

    note = ""
    if dsu.component_count() > 1:
        note = f"could not join all groups; {dsu.component_count()} remain"
        if last_failures:
            note += "; " + " | ".join(last_failures[:3])
        if unsafe_indices:
            note += "; skipped pre-shorted pads: " + ", ".join(
                pad_label(pads[index]) for index in sorted(unsafe_indices)
            )
    return NetReport(
        name=name,
        pad_count=len(pads),
        initial_components=initial_components,
        remaining_components=dsu.component_count(),
        connections_added=connections,
        tracks_added=tracks,
        vias_added=vias,
        expanded_nodes=expanded_total,
        note=note,
    )


def save_board_atomic(board: object, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output_path.stem}.route-",
        suffix=output_path.suffix,
        dir=str(output_path.parent),
    )
    os.close(descriptor)
    temporary_path = Path(temporary_name)
    try:
        if not pcbnew.SaveBoard(str(temporary_path), board):
            raise RuntimeError(f"KiCad failed to save {temporary_path}")
        # Reload before replacement so a failed serialization cannot destroy a
        # requested in-place input board.
        verification = pcbnew.LoadBoard(str(temporary_path))
        if verification is None:
            raise RuntimeError(f"KiCad failed to reload {temporary_path}")
        os.replace(temporary_path, output_path)
    finally:
        if temporary_path.exists():
            temporary_path.unlink()


def save_text_atomic(text: str, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output_path.stem}.route-",
        suffix=output_path.suffix,
        dir=str(output_path.parent),
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, output_path)
    finally:
        if temporary_path.exists():
            temporary_path.unlink()


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Obstacle-aware 2/4-layer signal-routing assistant for KiCad",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
SAFETY WORKFLOW
  1. Route critical power/current loops manually and save a checkpoint.
  2. Run this tool on a copied board or with a distinct --output path.
  3. Inspect every generated route in PCB Editor.
  4. Compare KiCad DRC against the checkpoint before accepting the output.

ALWAYS-PROTECTED NETS
  GND, GND2, +3V3, +5V, VBAT_RAW, VBAT_SW,
  /FAN_NEG_INTERNAL, /MOTOR_[LR]_OUT[12], /MPM_*,
  /PWR_GATE_INTERNAL, /PWR_SWITCH_RETURN, Net-(D3-A),
  Net-(U5B-VCAP_1), Net-(IR_LED_*-C/PadA), every unconnected-* net,
  and net 0.

Motor/FAN logic controls such as /MOTOR_L_PWM and /FAN_PWM are signals and
remain eligible. Protected nets cannot be enabled through command-line flags.
Pads already copper-overlapping another net are also skipped until repaired.
""",
    )
    parser.add_argument("--input", "-i", required=True, type=Path)
    parser.add_argument("--output", "-o", type=Path)
    parser.add_argument(
        "--in-place",
        action="store_true",
        help="permit output to replace input atomically (requires --output=input)",
    )
    parser.add_argument(
        "--overwrite-output",
        action="store_true",
        help="permit replacement of an existing output file distinct from input",
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="route in memory but do not save"
    )
    parser.add_argument(
        "--net",
        action="append",
        default=[],
        help="route only this exact net name; repeat as needed",
    )
    parser.add_argument(
        "--exclude-net",
        action="append",
        default=[],
        help="additional regular expression to exclude; repeat as needed",
    )
    parser.add_argument("--grid", type=float, default=0.10, dest="grid_mm")
    parser.add_argument(
        "--trace-width", type=float, default=0.20, dest="trace_width_mm"
    )
    parser.add_argument(
        "--clearance", type=float, default=0.20, dest="clearance_mm"
    )
    parser.add_argument(
        "--escape-clearance",
        type=float,
        default=0.16,
        dest="escape_clearance_mm",
        help="local copper clearance allowed only while escaping a net's pad",
    )
    parser.add_argument(
        "--pad-escape",
        type=float,
        default=0.60,
        dest="pad_escape_mm",
        help="length of the fine-pitch clearance halo around a routed pad",
    )
    parser.add_argument(
        "--via-diameter", type=float, default=0.60, dest="via_diameter_mm"
    )
    parser.add_argument(
        "--via-drill", type=float, default=0.30, dest="via_drill_mm"
    )
    parser.add_argument(
        "--hole-clearance",
        type=float,
        default=0.25,
        dest="hole_clearance_mm",
        help="minimum existing-hole to new-via-copper spacing",
    )
    parser.add_argument(
        "--edge-clearance", type=float, default=0.50, dest="edge_clearance_mm"
    )
    parser.add_argument(
        "--via-cost",
        type=float,
        default=3.0,
        dest="via_cost_mm",
        help="A* length-equivalent penalty for a layer transition",
    )
    parser.add_argument(
        "--bend-cost",
        type=float,
        default=0.30,
        dest="bend_cost_mm",
        help="A* length-equivalent penalty for changing track direction",
    )
    parser.add_argument("--max-vias", type=int, default=2)
    parser.add_argument("--max-expanded", type=int, default=200_000)
    parser.add_argument("--max-net-expanded", type=int, default=600_000)
    parser.add_argument("--candidate-pairs", type=int, default=8)
    parser.add_argument(
        "--front-axis", choices=("vertical", "horizontal"), default="vertical"
    )
    parser.add_argument(
        "--net-order",
        choices=("long-first", "fewest-pads", "short-first", "mcu-first"),
        default="long-first",
        help="deterministic ordering of eligible nets before routing",
    )
    parser.add_argument(
        "--fanout-reference",
        action="append",
        default=[],
        help="prioritize the first connection from this footprint",
    )
    parser.add_argument("--report-json", type=Path)
    parser.add_argument(
        "--overwrite-report",
        action="store_true",
        help="permit replacement of an existing --report-json file",
    )
    parser.add_argument("--verbose", "-v", action="store_true")
    return parser.parse_args(argv)


def validate_args(args: argparse.Namespace) -> RouterConfig:
    input_path = args.input.expanduser().resolve()
    if not input_path.is_file():
        raise SystemExit(f"input board does not exist: {input_path}")
    args.input = input_path

    if not args.dry_run and args.output is None:
        raise SystemExit("--output is required unless --dry-run is used")
    if args.output is not None:
        args.output = args.output.expanduser().resolve()
        if args.output == input_path and not args.in_place:
            raise SystemExit(
                "refusing to overwrite --input without explicit --in-place"
            )
        if args.in_place and args.output != input_path:
            raise SystemExit("--in-place requires --output to equal --input")
        if (
            args.output != input_path
            and args.output.exists()
            and not args.overwrite_output
        ):
            raise SystemExit(
                "refusing to replace an existing --output without "
                "--overwrite-output"
            )
    elif args.in_place:
        raise SystemExit("--in-place requires --output")

    if args.report_json is not None:
        args.report_json = args.report_json.expanduser().resolve()
        protected_paths = {input_path}
        if args.output is not None:
            protected_paths.add(args.output)
        if args.report_json in protected_paths:
            raise SystemExit(
                "--report-json must be distinct from --input and --output"
            )
        if args.report_json.exists() and not args.overwrite_report:
            raise SystemExit(
                "refusing to replace an existing --report-json without "
                "--overwrite-report"
            )

    positive_values = {
        "grid": args.grid_mm,
        "trace width": args.trace_width_mm,
        "clearance": args.clearance_mm,
        "escape clearance": args.escape_clearance_mm,
        "pad escape": args.pad_escape_mm,
        "via diameter": args.via_diameter_mm,
        "via drill": args.via_drill_mm,
        "hole clearance": args.hole_clearance_mm,
        "edge clearance": args.edge_clearance_mm,
        "via cost": args.via_cost_mm,
        "bend cost": args.bend_cost_mm,
    }
    for label, value in positive_values.items():
        if value <= 0:
            raise SystemExit(f"{label} must be positive")
    if args.via_drill_mm >= args.via_diameter_mm:
        raise SystemExit("via drill must be smaller than via diameter")
    if args.escape_clearance_mm > args.clearance_mm:
        raise SystemExit("escape clearance must not exceed normal clearance")
    if (
        args.max_vias < 0
        or args.max_expanded <= 0
        or args.max_net_expanded <= 0
        or args.candidate_pairs <= 0
    ):
        raise SystemExit("routing limits must be positive (max-vias may be zero)")

    requested_protected = [name for name in args.net if is_protected_net(name)]
    if requested_protected:
        raise SystemExit(
            "protected power/current nets cannot be autorouted: "
            + ", ".join(requested_protected)
        )
    try:
        args.exclude_patterns = [re.compile(item) for item in args.exclude_net]
    except re.error as exc:
        raise SystemExit(f"invalid --exclude-net regular expression: {exc}") from exc

    return RouterConfig(
        grid_mm=args.grid_mm,
        trace_width_mm=args.trace_width_mm,
        clearance_mm=args.clearance_mm,
        sensitive_clearance_mm=max(
            args.clearance_mm, MIN_SENSITIVE_CLEARANCE_MM
        ),
        highcurrent_sensitive_clearance_mm=max(
            args.clearance_mm, MIN_HIGHCURRENT_SENSITIVE_CLEARANCE_MM
        ),
        switch_clearance_mm=max(args.clearance_mm, MIN_SWITCH_CLEARANCE_MM),
        escape_clearance_mm=args.escape_clearance_mm,
        pad_escape_mm=args.pad_escape_mm,
        via_diameter_mm=args.via_diameter_mm,
        via_drill_mm=args.via_drill_mm,
        hole_clearance_mm=args.hole_clearance_mm,
        hole_to_hole_mm=MIN_HOLE_TO_HOLE_MM,
        edge_clearance_mm=args.edge_clearance_mm,
        via_cost_mm=args.via_cost_mm,
        bend_cost_mm=args.bend_cost_mm,
        max_vias=args.max_vias,
        max_expanded=args.max_expanded,
        max_net_expanded=args.max_net_expanded,
        candidate_pairs=args.candidate_pairs,
        front_axis=args.front_axis,
        net_order=args.net_order,
        fanout_references=tuple(args.fanout_reference),
        verbose=args.verbose,
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    config = validate_args(args)
    board = pcbnew.LoadBoard(str(args.input))
    if board is None:
        raise SystemExit(f"KiCad failed to load {args.input}")
    configure_routing_layers(board)

    pads_by_net: Dict[int, List[object]] = {}
    for pad in board.GetPads():
        if pad.GetNetCode() <= 0 or pad.IsNoConnectPad():
            continue
        pads_by_net.setdefault(pad.GetNetCode(), []).append(pad)

    requested = set(args.net)
    all_nets = []
    found_requested: Set[str] = set()
    for _, net_info in board.GetNetInfo().NetsByName().items():
        name = str(net_info.GetNetname())
        if requested and name not in requested:
            continue
        if name in requested:
            found_requested.add(name)
        if is_protected_net(name):
            continue
        if any(pattern.search(name) for pattern in args.exclude_patterns):
            continue
        pads = pads_by_net.get(net_info.GetNetCode(), [])
        if len(pads) < 2:
            continue
        all_nets.append((net_info, pads))

    missing = sorted(requested - found_requested)
    if missing:
        raise SystemExit("requested nets not found: " + ", ".join(missing))

    # Names break all ties for full determinism.  Different placements respond
    # differently to ordering, so staged temp-copy experiments can select an
    # order without changing geometry or design rules.
    if config.net_order == "long-first":
        all_nets.sort(
            key=lambda item: (
                -estimated_tree_length_mm(item[1]),
                -len(item[1]),
                str(item[0].GetNetname()),
            )
        )
    elif config.net_order == "fewest-pads":
        all_nets.sort(
            key=lambda item: (
                len(item[1]),
                -estimated_tree_length_mm(item[1]),
                str(item[0].GetNetname()),
            )
        )
    elif config.net_order == "short-first":
        all_nets.sort(
            key=lambda item: (
                estimated_tree_length_mm(item[1]),
                len(item[1]),
                str(item[0].GetNetname()),
            )
        )
    else:
        fanout_references = set(config.fanout_references or ("U5",))
        all_nets.sort(
            key=lambda item: (
                0
                if any(
                    pad.GetParentFootprint().GetReference()
                    in fanout_references
                    for pad in item[1]
                )
                else 1,
                -estimated_tree_length_mm(item[1]),
                len(item[1]),
                str(item[0].GetNetname()),
            )
        )

    print(
        f"Loading occupancy grid for {args.input.name}: "
        f"{len(all_nets)} eligible nets, {config.grid_mm:.2f} mm grid"
    )
    occupancy = OccupancyGrid(board, config)
    router = AStarRouter(occupancy, config)
    reports: List[NetReport] = []
    for net_info, pads in all_nets:
        reports.append(
            route_net(board, occupancy, router, net_info, pads, config)
        )

    if not args.dry_run:
        assert args.output is not None
        save_board_atomic(board, args.output)

    total_connections = sum(item.connections_added for item in reports)
    total_tracks = sum(item.tracks_added for item in reports)
    total_vias = sum(item.vias_added for item in reports)
    complete = sum(item.remaining_components == 1 for item in reports)
    incomplete = [item for item in reports if item.remaining_components > 1]
    print(
        f"Routed {complete}/{len(reports)} eligible nets: "
        f"{total_connections} connections, {total_tracks} tracks, "
        f"{total_vias} vias"
    )
    if incomplete:
        print(f"Incomplete nets ({len(incomplete)}):")
        for report in incomplete:
            print(f"  {report.name}: {report.note}")
    if args.dry_run:
        print("Dry run: no board file was written")
    else:
        print(f"Wrote {args.output}")
    print("MANDATORY: inspect the output in PCB Editor and run KiCad DRC.")

    if args.report_json is not None:
        report_data = {
            "input": str(args.input),
            "output": None if args.dry_run else str(args.output),
            "dry_run": bool(args.dry_run),
            "config": dataclasses.asdict(config),
            "summary": {
                "eligible_nets": len(reports),
                "complete_nets": complete,
                "connections_added": total_connections,
                "tracks_added": total_tracks,
                "vias_added": total_vias,
            },
            "nets": [dataclasses.asdict(item) for item in reports],
        }
        save_text_atomic(
            json.dumps(report_data, indent=2, ensure_ascii=False) + "\n",
            args.report_json,
        )
    return 0 if not incomplete else 2


if __name__ == "__main__":
    raise SystemExit(main())
