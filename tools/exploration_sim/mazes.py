"""KeriLab competition maze discovery, parsing and reproducible provenance.

Coordinates are x east / y north, with y=0 at the bottom. Wall bits are
N=0, E=1, S=2, W=3. The ASCII source's S and every G are authoritative;
goals are never inferred from the dimensions. Ground truth must be complete.
Only source data and its MIT license are fetched, into ignored build storage.
"""
from __future__ import annotations

import hashlib
import io
import json
import os
from pathlib import Path
import re
import subprocess
from dataclasses import dataclass, field
from urllib.request import Request, urlopen
import zipfile

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
SOURCE_REPOSITORY = "https://github.com/kerikun11/micromouse-maze-data"
SOURCE_VIEWER = "https://www.kerislab.jp/micromouse-maze-data/"
# Verified upstream revision; downloads stay reproducible even if master moves.
SOURCE_REVISION = "762ed2b68735ea29148c6a1251a90ed0651ff26b"
CACHE_DIRECTORY = REPOSITORY_ROOT / "build/exploration_sim/maze-data"
DIRECTIONS = ((0, 1), (1, 0), (0, -1), (-1, 0))
COMPETITION_NAME = re.compile(r"^(\d+)MM(\d{4})([CH])(.*)$")


class MazeFormatError(ValueError):
    """The maze cannot be used as a complete physical ground truth."""


@dataclass
class Maze:
    width: int
    height: int
    walls: list[list[int]]
    start: tuple[int, int]
    goals: list[tuple[int, int]]
    metadata: dict = field(default_factory=dict)

    def has_wall(self, x: int, y: int, direction: int) -> bool:
        return bool(self.walls[y][x] & (1 << direction))

    def to_dict(self) -> dict:
        return {
            **self.metadata,
            "width": self.width,
            "height": self.height,
            "walls": self.walls,
            "start": list(self.start),
            "start_heading": 0,
            "goals": [list(cell) for cell in self.goals],
        }


def parse_maze(text: str, *, metadata: dict | None = None) -> Maze:
    """Parse the upstream +---/|/S/G format, rejecting ambiguous truth.

    Accept CRLF and a missing terminal newline. Reject missing boundaries,
    invalid glyphs, unknown edges, missing goals, and disconnected goals.
    Rectangular mazes are supported for locally authored fixtures too.
    """
    lines = text.lstrip("\ufeff").splitlines()
    if len(lines) < 3 or len(lines) % 2 != 1:
        raise MazeFormatError("Maze requires 2*height+1 nonempty ASCII rows")
    row_length = len(lines[0])
    if row_length < 5 or (row_length - 1) % 4:
        raise MazeFormatError("Maze row width must be 4*width+1")
    width, height = (row_length - 1) // 4, (len(lines) - 1) // 2
    if width > 64 or height > 64:
        raise MazeFormatError("Maze dimensions exceed 64 cells")
    if any(len(line) != row_length for line in lines):
        raise MazeFormatError("Maze rows have inconsistent widths")
    if "." in text:
        raise MazeFormatError("Source has unknown '.' edges; complete ground truth is required")

    horizontal = []  # top-to-bottom boundaries
    for row in range(0, len(lines), 2):
        line = lines[row]
        if any(line[col] != "+" for col in range(0, row_length, 4)):
            raise MazeFormatError(f"Missing pillar on source row {row + 1}")
        edges = [line[4 * x + 1:4 * x + 4] for x in range(width)]
        if any(edge not in ("---", "   ") for edge in edges):
            raise MazeFormatError(f"Invalid horizontal edge on source row {row + 1}")
        horizontal.append([edge == "---" for edge in edges])

    walls = [[0] * width for _ in range(height)]
    starts, goals = [], []
    for top_y in range(height):
        y, line = height - 1 - top_y, lines[2 * top_y + 1]
        if any(line[col] not in ("|", " ") for col in range(0, row_length, 4)):
            raise MazeFormatError(f"Invalid vertical edge on source row {2 * top_y + 2}")
        for x in range(width):
            marker = line[4 * x + 1:4 * x + 4]
            if marker not in ("   ", " S ", " G "):
                raise MazeFormatError(f"Invalid cell marker at ({x}, {y})")
            if marker == " S ":
                starts.append((x, y))
            elif marker == " G ":
                goals.append((x, y))
            walls[y][x] = (
                int(horizontal[top_y][x])
                | (int(line[4 * x + 4] == "|") << 1)
                | (int(horizontal[top_y + 1][x]) << 2)
                | (int(line[4 * x] == "|") << 3)
            )
    if len(starts) != 1:
        raise MazeFormatError(f"Expected exactly one S marker, found {len(starts)}")
    if not goals:
        raise MazeFormatError("No G markers; set the goal explicitly in the source maze")
    maze = Maze(width, height, walls, starts[0], sorted(goals), metadata or {})
    for y in range(height):
        for x in range(width):
            for direction, (dx, dy) in enumerate(DIRECTIONS):
                nx, ny = x + dx, y + dy
                if not (0 <= nx < width and 0 <= ny < height):
                    if not maze.has_wall(x, y, direction):
                        raise MazeFormatError(f"Open outside boundary at ({x}, {y})")
                elif maze.has_wall(x, y, direction) != maze.has_wall(nx, ny, (direction + 2) % 4):
                    raise MazeFormatError(f"Conflicting shared edge at ({x}, {y})")
    reached, pending = {maze.start}, [maze.start]
    while pending:
        x, y = pending.pop()
        for direction, (dx, dy) in enumerate(DIRECTIONS):
            cell = (x + dx, y + dy)
            if not maze.has_wall(x, y, direction) and cell not in reached:
                reached.add(cell)
                pending.append(cell)
    if not any(goal in reached for goal in maze.goals):
        raise MazeFormatError("No goal cell is reachable from S")
    maze.metadata["reachable_cells"] = len(reached)
    return maze


def _dataset_root(path: Path) -> Path | None:
    path = path.expanduser().resolve()
    if path.is_dir() and any(path.glob("*MM*.maze")):
        return path.parent if path.name == "data" else path
    if (path / "data").is_dir() and any((path / "data").glob("*MM*.maze")):
        return path
    return None


def _data_directory(root: Path) -> Path:
    return root / "data" if (root / "data").is_dir() else root


def discover_dataset(maze_dir: str | Path | None = None, *, download: bool = True) -> Path:
    """Use an explicit/local clone first; download the pinned archive if absent.

    NIGHTFALL_MAZE_DIR accepts the clone root or its data directory. No existing
    clone is modified or pulled. A --maze-dir typo fails instead of silently
    selecting another dataset.
    """
    explicit = maze_dir or os.environ.get("NIGHTFALL_MAZE_DIR")
    if explicit:
        found = _dataset_root(Path(explicit))
        if found is None:
            raise FileNotFoundError(f"No competition .maze files in {explicit}")
        return found
    candidates = [CACHE_DIRECTORY]
    roots = [REPOSITORY_ROOT.parent, Path.home() / "workspace/micromouse"]
    try:
        common = subprocess.check_output(
            ["git", "rev-parse", "--git-common-dir"], cwd=REPOSITORY_ROOT,
            text=True, stderr=subprocess.DEVNULL, timeout=3,
        ).strip()
        common_path = (REPOSITORY_ROOT / common).resolve()
        roots.append(common_path.parent.parent)
    except (OSError, subprocess.SubprocessError):
        pass
    for root in roots:
        candidates.extend((root / "maze-data", root / "micromouse-maze-data"))
    for candidate in candidates:
        found = _dataset_root(candidate)
        if found:
            return found
    if not download:
        raise FileNotFoundError("KeriLab data not found; set NIGHTFALL_MAZE_DIR or use fetch_dataset()")
    return fetch_dataset()


def fetch_dataset(destination: str | Path = CACHE_DIRECTORY) -> Path:
    """Fetch pinned data plus LICENSE; never extract arbitrary archive paths."""
    destination = Path(destination).expanduser().resolve()
    url = f"https://codeload.github.com/kerikun11/micromouse-maze-data/zip/{SOURCE_REVISION}"
    request = Request(url, headers={"User-Agent": "nightfall-exploration-simulator"})
    with urlopen(request, timeout=30) as response:
        payload = response.read(8 * 1024 * 1024 + 1)
    if len(payload) > 8 * 1024 * 1024:
        raise ValueError("KeriLab archive exceeds download size limit")
    prefix = f"micromouse-maze-data-{SOURCE_REVISION}/"
    selected: dict[str, bytes] = {}
    with zipfile.ZipFile(io.BytesIO(payload)) as archive:
        for item in archive.infolist():
            if not item.filename.startswith(prefix) or item.is_dir():
                continue
            relative = item.filename[len(prefix):]
            if relative in ("LICENSE", "README.md") or re.fullmatch(r"data/[A-Za-z0-9_-]+\.maze", relative):
                if item.file_size > 1024 * 1024:
                    raise ValueError("Unexpectedly large KeriLab source file")
                selected[relative] = archive.read(item)
    if "LICENSE" not in selected or not any(name.endswith(".maze") for name in selected):
        raise ValueError("KeriLab archive lacks data or its license")
    for relative, content in selected.items():
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(content)
    (destination / "source.json").write_text(json.dumps({
        "repository": SOURCE_REPOSITORY, "revision": SOURCE_REVISION,
        "archive_sha256": hashlib.sha256(payload).hexdigest(), "license": "MIT",
    }, indent=2) + "\n", encoding="utf-8")
    return destination


def _source_revision(root: Path) -> str:
    manifest = root / "source.json"
    if manifest.is_file():
        try:
            return str(json.loads(manifest.read_text(encoding="utf-8"))["revision"])
        except (ValueError, KeyError):
            pass
    if (root / ".git").exists():
        try:
            return subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=root, text=True,
                stderr=subprocess.DEVNULL, timeout=3,
            ).strip()
        except (OSError, subprocess.SubprocessError):
            pass
    return "local-unversioned"


def _metadata(path: Path, root: Path, revision: str) -> dict:
    match = COMPETITION_NAME.fullmatch(path.stem)
    if not match:
        raise ValueError(f"Not a competition maze identifier: {path.stem}")
    _, year, code, suffix = match.groups()
    family = "half" if code == "H" else "classic"
    family_label = "ハーフ" if family == "half" else "クラシック"
    events = {
        "X": "全日本 決勝", "X_pre": "全日本 予選", "_semi": "全日本 セミファイナル",
        "_student": "学生大会", "_Kansai": "関西地区", "_Chubu": "中部地区",
        "_Kyushu": "九州地区", "_East": "東日本地区", "_Hokuriku": "北陸地区",
        "_Kanazawa": "金沢", "_Tashiro": "田代杯", "_Cheese": "Cheese",
        "_Cheese_cand": "Cheese 候補", "X_Taiwan": "台湾", "F_pre": "初級 予選",
    }
    event = events.get(suffix, suffix.lstrip("_") or "大会")
    return {
        "id": path.stem, "name": f"{year} {family_label} {event}",
        "family": family, "year": int(year), "event": event,
        "is_final": suffix == "X", "source_repository": SOURCE_REPOSITORY,
        "source_url": f"{SOURCE_REPOSITORY}/blob/{revision if revision != 'local-unversioned' else 'master'}/data/{path.name}",
        "source_revision": revision,
        "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "license": "MIT", "license_url": f"{SOURCE_REPOSITORY}/blob/{SOURCE_REVISION}/LICENSE",
        "goal_source": "G markers in source .maze; all marked cells accepted",
        "label_source": "filename convention; original identifier retained",
    }


def catalog(maze_dir: str | Path | None = None, *, download: bool = True) -> list[dict]:
    """List competition maps; unavailable truth is visible with a reason."""
    root = discover_dataset(maze_dir, download=download)
    revision = _source_revision(root)
    entries = []
    for path in sorted(_data_directory(root).glob("*.maze")):
        if not COMPETITION_NAME.fullmatch(path.stem):
            continue
        entry = _metadata(path, root, revision)
        try:
            maze = parse_maze(path.read_text(encoding="utf-8"), metadata=entry)
            entry.update({"width": maze.width, "height": maze.height,
                          "start": list(maze.start), "goals": [list(g) for g in maze.goals],
                          "available": True})
        except MazeFormatError as error:
            entry.update({"available": False, "error": str(error)})
        entries.append(entry)
    return sorted(entries, key=lambda entry: (-entry["year"], not entry["is_final"], entry["id"]))


def load_maze(maze_id: str, maze_dir: str | Path | None = None, *, download: bool = True) -> Maze:
    """Load only a catalog identifier, never a request-supplied filesystem path."""
    identifier = maze_id.removesuffix(".maze")
    if not COMPETITION_NAME.fullmatch(identifier) or "/" in identifier or "\\" in identifier:
        raise ValueError("Invalid competition maze identifier")
    root = discover_dataset(maze_dir, download=download)
    path = _data_directory(root) / f"{identifier}.maze"
    if not path.is_file():
        raise FileNotFoundError(f"Maze {identifier} is not in the selected dataset")
    metadata = _metadata(path, root, _source_revision(root))
    return parse_maze(path.read_text(encoding="utf-8"), metadata=metadata)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--maze-dir")
    parser.add_argument("--fetch", action="store_true", help="Fetch the pinned upstream archive into build/")
    args = parser.parse_args()
    if args.fetch:
        print(fetch_dataset())
    else:
        print(json.dumps(catalog(args.maze_dir), ensure_ascii=False, indent=2))
