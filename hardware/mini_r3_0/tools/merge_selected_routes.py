#!/usr/bin/env python3
"""Copy reviewed track/via subsets between matching KiCad boards.

The script is intentionally small and conservative: it copies only explicitly
named nets, recreates segments and through vias with the destination board's
net objects, preserves all existing destination copper, and refuses ambiguous
overwrites.  It is used to combine net-by-net audited autorouter candidates
with the separately reviewed mini_r3 power-routing checkpoint.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import tempfile
from typing import Iterable, Tuple

import pcbnew


def point_key(point: object) -> Tuple[int, int]:
    return int(point.x), int(point.y)


def item_key(item: object) -> tuple:
    if isinstance(item, pcbnew.PCB_VIA):
        return (
            "via",
            str(item.GetNetname()),
            point_key(item.GetPosition()),
            int(item.GetWidth(pcbnew.F_Cu)),
            int(item.GetDrillValue()),
            int(item.TopLayer()),
            int(item.BottomLayer()),
        )
    start = point_key(item.GetStart())
    end = point_key(item.GetEnd())
    return (
        "track",
        str(item.GetNetname()),
        min(start, end),
        max(start, end),
        int(item.GetLayer()),
        int(item.GetWidth()),
    )


def clone_item(board: object, item: object, net_info: object) -> object:
    if isinstance(item, pcbnew.PCB_VIA):
        result = pcbnew.PCB_VIA(board)
        result.SetPosition(item.GetPosition())
        result.SetWidth(item.GetWidth(pcbnew.F_Cu))
        result.SetDrill(item.GetDrillValue())
        result.SetLayerPair(item.TopLayer(), item.BottomLayer())
    elif isinstance(item, pcbnew.PCB_TRACK):
        result = pcbnew.PCB_TRACK(board)
        result.SetStart(item.GetStart())
        result.SetEnd(item.GetEnd())
        result.SetLayer(item.GetLayer())
        result.SetWidth(item.GetWidth())
    else:
        raise RuntimeError(f"unsupported routing item: {type(item).__name__}")
    result.SetNet(net_info)
    return result


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
    parser.add_argument("--destination", "-d", required=True, type=Path)
    parser.add_argument("--source", "-s", required=True, type=Path)
    parser.add_argument("--output", "-o", required=True, type=Path)
    parser.add_argument("--net", action="append", required=True)
    parser.add_argument("--overwrite-output", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    destination_path = args.destination.expanduser().resolve()
    source_path = args.source.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    if destination_path == source_path:
        raise RuntimeError("source and destination must be different boards")
    if output_path in {destination_path, source_path}:
        raise RuntimeError("output must be a separate board path")
    if output_path.exists() and not args.overwrite_output:
        raise RuntimeError(f"refusing existing output: {output_path}")

    destination = pcbnew.LoadBoard(str(destination_path))
    source = pcbnew.LoadBoard(str(source_path))
    if destination is None or source is None:
        raise RuntimeError("could not load source or destination board")
    destination_nets = destination.GetNetInfo().NetsByName()
    source_nets = source.GetNetInfo().NetsByName()
    selected = set(args.net)
    destination_names = {str(name) for name in destination_nets.keys()}
    source_names = {str(name) for name in source_nets.keys()}
    missing_destination = sorted(selected - destination_names)
    missing_source = sorted(selected - source_names)
    if missing_destination or missing_source:
        raise RuntimeError(
            "missing selected nets; destination="
            + ",".join(missing_destination)
            + "; source="
            + ",".join(missing_source)
        )

    existing = {item_key(item) for item in destination.GetTracks()}
    copied = 0
    skipped = 0
    for item in source.GetTracks():
        name = str(item.GetNetname())
        if name not in selected:
            continue
        key = item_key(item)
        if key in existing:
            skipped += 1
            continue
        clone = clone_item(destination, item, destination_nets[name])
        destination.Add(clone)
        existing.add(key)
        copied += 1

    destination.BuildConnectivity()
    save_atomic(destination, output_path)
    print(
        f"copied {copied} routing items on {len(selected)} selected nets; "
        f"skipped {skipped} duplicates; output {output_path}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
