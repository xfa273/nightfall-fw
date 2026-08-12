#!/usr/bin/env python3
"""Replace one pre-routing footprint while preserving schematic identity/nets.

KiCad's Python plug-in loader is safest when loading one library footprint per
process.  Invoke this helper once per replacement after ``prepare_board.py``
and before adding tracks or zones.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import pcbnew


def mm(value: float) -> int:
    return int(pcbnew.FromMM(value))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--reference", required=True)
    parser.add_argument("--library-path", required=True, type=Path)
    parser.add_argument("--footprint", required=True)
    parser.add_argument("--fpid", required=True)
    parser.add_argument("--x", required=True, type=float)
    parser.add_argument("--y", required=True, type=float)
    parser.add_argument("--rotation", required=True, type=float)
    parser.add_argument("--value")
    args = parser.parse_args()

    board = pcbnew.LoadBoard(str(args.input))
    if board is None:
        raise RuntimeError(f"could not load {args.input}")
    if list(board.GetTracks()) or board.GetAreaCount():
        raise RuntimeError("replace_footprint.py must run before routing/zones")

    old = board.FindFootprintByReference(args.reference)
    if old is None:
        raise RuntimeError(f"missing footprint: {args.reference}")
    replacement = pcbnew.FootprintLoad(str(args.library_path), args.footprint)
    if replacement is None:
        raise RuntimeError(
            f"could not load {args.footprint} from {args.library_path}"
        )

    old_pads = {pad.GetNumber(): pad for pad in old.Pads()}
    new_numbers = {pad.GetNumber() for pad in replacement.Pads()}
    if new_numbers != set(old_pads):
        raise RuntimeError(
            f"pad mismatch for {args.reference}: "
            f"old={sorted(old_pads)}, new={sorted(new_numbers)}"
        )

    replacement.SetFPIDAsString(args.fpid)
    replacement.SetReference(args.reference)
    replacement.SetValue(args.value if args.value is not None else old.GetValue())
    replacement.SetPath(old.GetPath())
    replacement.SetPosition(pcbnew.VECTOR2I(mm(args.x), mm(args.y)))
    replacement.SetOrientationDegrees(args.rotation)
    replacement.Reference().SetVisible(False)
    replacement.Value().SetVisible(False)
    for pad in replacement.Pads():
        pad.SetNet(old_pads[pad.GetNumber()].GetNet())

    board.Remove(old)
    board.Add(replacement)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not pcbnew.SaveBoard(str(args.output), board):
        raise RuntimeError(f"could not save {args.output}")
    print(
        f"replaced {args.reference} with {args.fpid} at "
        f"({args.x:.3f}, {args.y:.3f}, {args.rotation:.1f} deg)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
