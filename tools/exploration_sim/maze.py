"""maze.py - マイクロマウス迷路の読み込みと壁ビット管理.

壁ビットは nightfall-fw のファームと同じ配置を使う。

    bit3 = 0x08 北 (N) / 前方
    bit2 = 0x04 東 (E)
    bit1 = 0x02 南 (S)
    bit0 = 0x01 西 (W)

座標系もファームと同じで、南西隅が (0, 0)、y 増加分が北。
方角コードは 0=北, 1=東, 2=南, 3=西。
"""

from __future__ import annotations

# 方角コード（ファームの mouse.dir と同じ）
DIR_N, DIR_E, DIR_S, DIR_W = 0, 1, 2, 3
# 壁ビット（ファームの map[][] 下位 4bit と同じ）
N, E, S, W = 0x08, 0x04, 0x02, 0x01
DIR_WALL = (N, E, S, W)   # 方角コード -> 壁ビット
DIR_DX = (0, 1, 0, -1)
DIR_DY = (1, 0, -1, 0)
DIR_NAME = ("N", "E", "S", "W")
REL_FWD, REL_RIGHT, REL_BACK, REL_LEFT = 0, 1, 3, 2  # make_route の相対コード


def turn_right(d: int) -> int:
    return (d + 1) & 3


def turn_left(d: int) -> int:
    return (d + 3) & 3


def turn_back(d: int) -> int:
    return (d + 2) & 3


class Maze:
    """16x16 (可変) 迷路。ground truth 側。"""

    def __init__(self, n: int, walls: list[list[int]], start=(0, 0), start_dir=N,
                 goals=None, name="maze"):
        self.n = n
        self.walls = walls          # walls[y][x] = 壁ビット
        self.start = start
        self.start_dir = start_dir
        self.goals = goals or self._default_goals()
        self.name = name

    # ------------------------------------------------------------------ util
    def _default_goals(self):
        m = self.n // 2
        return [(m - 1, m - 1), (m, m - 1), (m - 1, m), (m, m)]

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.n and 0 <= y < self.n

    def wall(self, x: int, y: int, d: int) -> bool:
        """绝对方向 d の壁の有無。外周は常に壁。"""
        if not self.in_bounds(x, y):
            return True
        return bool(self.walls[y][x] & DIR_WALL[d])

    def open(self, x: int, y: int, d: int) -> bool:
        """壁がなく、隣マスが迷路内か、またはルール上の開口か。"""
        if not self.in_bounds(x, y):
            return False
        if self.wall(x, y, d):
            return False
        nx, ny = x + DIR_DX[d], y + DIR_DY[d]
        return self.in_bounds(nx, ny)

    def wall_counts(self):
        return sum(bin(w).count("1") for row in self.walls for w in row) // 2

    def degree(self) -> float:
        """開いている adjacency の総数（探索しやすさの目安）。"""
        return sum(1 for y in range(self.n) for x in range(self.n)
                   for d in range(4) if self.open(x, y, d)) // 2

    # --------------------------------------------------------------- formats
    @classmethod
    def load(cls, path) -> "Maze":
        text = open(path, "r", encoding="utf-8", errors="ignore").read()
        if "+" in text and "|" in text:
            return cls.parse_ascii(text, name=str(path).split("/")[-1].split(".")[0])
        raise ValueError(f"未対応の迷路ファイルです: {path}")

    @classmethod
    def parse_ascii(cls, text: str, name="maze") -> "Maze":
        """KERI さんの micromouse-maze-data 形式（+---+ / | 罫線）を読む。

        図の最上行が北。ファーム座標に合わせるため上下反転して y を決める。
        'S' がスタート、'G' がゴール区画。セル幅は 4 桁形式と 2 桁形式の両方、
        未知壁のプレースホルダ '.' などにも対応する。
        """
        raw = [ln.rstrip("\n") for ln in text.splitlines()]
        raw = [ln for ln in raw if ln.strip()]
        lines = []
        for ln in raw:
            if not set(ln) <= set("+-|. \tSGabcedfXYZ?01"):
                continue
            lines.append(ln)
        if len(lines) < 3:
            raise ValueError("迷路の罫線が見つかりません")
        base = lines[0]
        unit = 4 if (len(base) - 1) % 4 == 0 else (2 if (len(base) - 1) % 2 == 0 else 0)
        if unit == 0:
            raise ValueError("セル幅を判定できません (line width=%d)" % len(base))
        n = (len(base) - 1) // unit
        nr = (len(lines) - 1) // 2
        if len(base) != unit * n + 1:
            raise ValueError("外周の幅が不揃いです")

        def mark(line, c):
            """セル c の本体文字列（記号は空白扱いせずそのまま返す）。"""
            a = unit * c + 1
            return line[a:a + unit] if unit == 4 else line[a:a + unit]

        def hwall(line, c):
            seg = line[unit * c + 1:unit * c + unit]
            return "-" in seg or "+" in seg

        def vwall(line, c):
            i = unit * c
            return len(line) > i and line[i] == "|"

        walls = [[0] * n for _ in range(nr)]
        start, start_dir = None, DIR_N
        goals = []

        for r in range(nr):
            y = nr - 1 - r
            cell_line = lines[2 * r + 1]
            top_line = lines[2 * r]
            bot_line = lines[2 * r + 2] if 2 * r + 2 < len(lines) else ""
            for c in range(n):
                body = mark(cell_line, c)
                w = 0
                if r == 0 or hwall(top_line, c):
                    w |= N
                if r == nr - 1 or (bot_line and hwall(bot_line, c)):
                    w |= S
                if c == n - 1 or vwall(cell_line, c + 1):
                    w |= E
                if c == 0 or vwall(cell_line, c):
                    w |= W
                walls[y][c] = w
                if "S" in body:
                    start = (c, y)
                if "G" in body:
                    goals.append((c, y))

        if nr != n:
            raise ValueError("正方形ではありません (rows=%d cols=%d)" % (nr, n))
        if start is None:
            start = (0, 0)
        return cls(n, walls, start=start, start_dir=start_dir,
                   goals=sorted(set(goals)) or None, name=name)

    def to_ascii(self, overlay=None) -> str:
        """overlay: {(x, y): 'A'} を区画に書き込む。"""
        n = self.n
        out = []
        for r in range(n):
            y = n - 1 - r
            boundary = []
            body = []
            for c in range(n):
                boundary.append("---" if self.walls[y][c] & N else "   ")
                ch = " "
                if overlay and (c, y) in overlay:
                    ch = overlay[(c, y)]
                body.append(" %s" % ch)
            out.append("+" + "+".join(boundary) + "+")
            row = []
            for c in range(n):
                row.append(body[c] + ("|" if self.walls[y][c] & E else " "))
            out.append("|" + "".join(row))
        boundary = []
        for c in range(n):
            boundary.append("---" if self.walls[0][c] & S else "   ")
        out.append("+" + "+".join(boundary) + "+")
        return "\n".join(out)

    def to_c_array(self) -> str:
        rows = []
        for y in range(self.n):
            rows.append("    {" + ",".join(str(self.walls[y][x]) for x in range(self.n)) + "}")
        return "uint8_t map[%d][%d] = {\n%s\n};" % (self.n, self.n, ",\n".join(rows))
