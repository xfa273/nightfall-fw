#!/usr/bin/env python3
import argparse
import re
import sys
from pathlib import Path

DUMP_RE = re.compile(r"\[SEARCH-DUMP\]\s+y=(\d+)\s*:\s*([0-9A-Fa-f]+)")
HEADER_RE = re.compile(r"\[SEARCH-DUMP\]\s+source=(\S+)\s+size=(\d+)")
WEST = 0x01
SOUTH = 0x02
EAST = 0x04
NORTH = 0x08


def parse_dump(text):
    rows = {}
    size = None
    source = None
    for line in text.splitlines():
        header = HEADER_RE.search(line)
        if header:
            source = header.group(1)
            size = int(header.group(2))
            continue
        match = DUMP_RE.search(line)
        if not match:
            continue
        y = int(match.group(1))
        hex_cells = match.group(2)
        if len(hex_cells) % 2 != 0:
            raise ValueError(f"row y={y} has odd hex length: {len(hex_cells)}")
        cells = [int(hex_cells[i:i + 2], 16) for i in range(0, len(hex_cells), 2)]
        rows[y] = cells
    if not rows:
        raise ValueError("no [SEARCH-DUMP] rows found")
    inferred_size = max(max(rows.keys()) + 1, max(len(cells) for cells in rows.values()))
    if size is None:
        size = inferred_size
    grid = [[0 for _ in range(size)] for _ in range(size)]
    for y, cells in rows.items():
        if y >= size:
            raise ValueError(f"row y={y} is outside size={size}")
        if len(cells) != size:
            raise ValueError(f"row y={y} has {len(cells)} cells, expected {size}")
        grid[y] = cells
    missing = [y for y in range(size) if y not in rows]
    if missing:
        raise ValueError(f"missing rows: {','.join(str(y) for y in missing)}")
    return source, size, grid


def render_ascii(grid, mark_start=True, mark_goal=True):
    size = len(grid)
    out = []
    for y in range(size - 1, -1, -1):
        top = []
        mid = []
        for x in range(size):
            cell = grid[y][x]
            top.append("+---" if (cell & NORTH) else "+   ")
            if x == 0:
                mid.append("|" if (cell & WEST) else " ")
            ch = " "
            if mark_start and x == 0 and y == 0:
                ch = "S"
            elif mark_goal and (size // 2 - 1) <= x <= (size // 2) and (size // 2 - 1) <= y <= (size // 2):
                ch = "G"
            mid.append(f" {ch} ")
            mid.append("|" if (cell & EAST) else " ")
        top.append("+")
        out.append("".join(top))
        out.append("".join(mid))
    bottom = []
    for x in range(size):
        bottom.append("+---" if (grid[0][x] & SOUTH) else "+   ")
    bottom.append("+")
    out.append("".join(bottom))
    return "\n".join(out)


def summarize(grid, strict_consistency=False):
    size = len(grid)
    wall_cells = 0
    boundary_errors = 0
    mismatches = 0
    for y in range(size):
        for x in range(size):
            cell = grid[y][x]
            if cell & 0x0F:
                wall_cells += 1
            if x == 0 and not (cell & WEST):
                boundary_errors += 1
            if x == size - 1 and not (cell & EAST):
                boundary_errors += 1
            if y == 0 and not (cell & SOUTH):
                boundary_errors += 1
            if y == size - 1 and not (cell & NORTH):
                boundary_errors += 1
            if x + 1 < size and bool(cell & EAST) != bool(grid[y][x + 1] & WEST):
                if not strict_consistency and x == 0 and y == 0 and (cell & EAST) and not (grid[y][x + 1] & WEST):
                    continue
                mismatches += 1
            if y + 1 < size and bool(cell & NORTH) != bool(grid[y + 1][x] & SOUTH):
                mismatches += 1
    return wall_cells, boundary_errors, mismatches


def main():
    parser = argparse.ArgumentParser(description="Render F413 [SEARCH-DUMP] UART map output as an ASCII maze.")
    parser.add_argument("input", nargs="?", help="log file containing [SEARCH-DUMP] lines; stdin is used when omitted")
    parser.add_argument("--no-mark", action="store_true", help="do not mark start/goal cells")
    parser.add_argument("--summary-only", action="store_true", help="print only parsed summary and integrity counters")
    parser.add_argument("--strict-consistency", action="store_true", help="count the F405-compatible start-cell one-way east wall as a mismatch")
    args = parser.parse_args()

    if args.input:
        text = Path(args.input).read_text(encoding="utf-8", errors="replace")
    else:
        text = sys.stdin.read()

    source, size, grid = parse_dump(text)
    wall_cells, boundary_errors, mismatches = summarize(grid, strict_consistency=args.strict_consistency)
    print(f"[SEARCH-DUMP-RENDER] source={source or 'unknown'} size={size} wall_cells={wall_cells} boundary_errors={boundary_errors} mismatches={mismatches}")
    if not args.summary_only:
        print(render_ascii(grid, mark_start=not args.no_mark, mark_goal=not args.no_mark))


if __name__ == "__main__":
    main()
