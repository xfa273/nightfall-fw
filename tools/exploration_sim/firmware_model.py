"""firmware_model.py - nightfall-fw の探索ロジックのホスト側再現モデル.

再現対象 (platform/stm32f405/Core/Src/search.c):

  get_wall_info()  センサが F/R/L の壁を読み wall_info に格納（後方センサは無い）
  write_map()      m_temp = (wall_info >> dir) & 0x0f で絶対方位へ変換し、
                   現マスを上書き、4 近傍へ壁あり/なしを両方向に伝搬
  make_smap()      全未探索セル（またはゴール群）を 0 とする多起点 BFS（1セル=1コスト）
  make_route()     進行方向に対し 直進→右→左→後退 の優先順で smap が
                   厳密に小さくなる方へ降下
  conf_route()     1 セル到着ごとに再計算

`sensing=` / `propagate=` / `unknown_walls=` で挙動を切り替えられ、
比較用の別戦略（時間コスト Dijkstra、利得ベース目標選択）も同じ器で扱える。
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field

from maze import DIR_DX, DIR_DY, DIR_WALL, turn_back, turn_left, turn_right

REL_NAME = {0: "F", 1: "R", 2: "B", 3: "L"}
REL_ANGLE = {0: 0, 1: -90, 2: 180, 3: 90}
# make_route の優先順位: 直進(0) → 右(1) → 左(3) → 後退(2)
REL_PRIORITY = (0, 1, 3, 2)


def abs_dir(d: int, rel: int) -> int:
    return (d + rel) & 3


def add(x, y, d):
    return x + DIR_DX[d], y + DIR_DY[d]


@dataclass
class MouseState:
    n: int
    x: int
    y: int
    d: int
    map: list = field(default_factory=list)
    known_cell: list = field(default_factory=list)
    visit_order: dict = field(default_factory=dict)

    @classmethod
    def create(cls, maze, unknown_walls="open", start_dir=0):
        n = maze.n
        st = cls(n=n, x=maze.start[0], y=maze.start[1], d=start_dir)
        fill = 0x00 if unknown_walls == "open" else 0x0F
        st.map = [[fill for _ in range(n)] for _ in range(n)]
        st.known_cell = [[False] * n for _ in range(n)]
        st.visit_order = {}
        sx, sy = maze.start
        st.map[sy][sx] |= 0x07                       # 規則で確定する E/S/W
        if sx + 1 < n:
            st.map[sy][sx + 1] |= 0x01               # 東隣の西壁
        return st

    def known_open(self, x, y, d) -> bool:
        if not (0 <= x < self.n and 0 <= y < self.n):
            return False
        if self.map[y][x] & DIR_WALL[d]:
            return False
        return 0 <= x + DIR_DX[d] < self.n and 0 <= y + DIR_DY[d] < self.n

    def mark_seen(self, x, y):
        if (x, y) not in self.visit_order:
            self.visit_order[(x, y)] = len(self.visit_order) + 1

    def known_ratio(self, maze) -> float:
        return 100.0 * sum(1 for row in self.known_cell for v in row if v) / (self.n * self.n)

    def wrong_wall_count(self, maze) -> int:
        """認識マップが「実在しない壁」を持っている数（誤り検出用）。"""
        bad = 0
        for y in range(self.n):
            for x in range(self.n):
                for d in range(4):
                    bit = DIR_WALL[d]
                    if self.map[y][x] & bit and not maze.wall(x, y, d):
                        bad += 1
        return bad

    def missing_wall_count(self, maze) -> int:
        bad = 0
        for y in range(self.n):
            for x in range(self.n):
                for d in range(4):
                    bit = DIR_WALL[d]
                    if not (self.map[y][x] & bit) and maze.wall(x, y, d):
                        bad += 1
        return bad


# --------------------------------------------------------------- センサモデル
def sense(maze, x, y, d, mode="fw") -> int:
    """観測される壁ビット（絶対方位の N/E/S/W ビット）を返す。"""
    if mode == "ideal4":
        return maze.walls[y][x] & 0x0F
    if mode == "ideal4_ahead":
        w = maze.walls[y][x] & 0x0F
        ax, ay = add(x, y, d)
        if maze.in_bounds(ax, ay):
            for side in (turn_left(d), turn_right(d)):
                if maze.wall(ax, ay, side):
                    w |= DIR_WALL[side]
        return w
    # 相対センサ: F=bit3, R=bit2, B=bit1(未実装), L=bit0
    rel = 0
    if maze.wall(x, y, d):
        rel |= 8
    if maze.wall(x, y, turn_right(d)):
        rel |= 4
    if maze.wall(x, y, turn_left(d)):
        rel |= 1
    if mode == "fw":
        return (rel >> d) & 0x0F            # ファームと同じ右ローテート
    if mode == "fw_fixed":
        out = 0
        for a in range(4):                  # 絶対方位 a を観測できる相対ビットは (a-d)%4
            src = (a - d) & 3
            if src == 2:                    # 後方はセンサ無し
                continue
            if rel & (8 >> src):
                out |= DIR_WALL[a]
        return out
    if mode == "ideal3":                    # 3 センサ完動版（後方のみ不明）
        out = 0
        for a in range(4):
            src = (a - d) & 3
            if src == 2:
                continue
            if rel & (8 >> src):
                out |= DIR_WALL[a]
        return out
    raise ValueError("unknown sensing mode: %s" % mode)


def write_map(state: MouseState, maze, observed, propagate="fw", map_write="overwrite"):
    """write_map() 相当。

    map_write:
      overwrite ... ファーム通り現マスを上書き（観測できない方位の壁は消える）
      accumulate ... 現マスは OR 蓄積（一度知った壁は消さない）
    """
    x, y, d = state.x, state.y, state.d
    st = observed
    if (x, y) == maze.start:
        st |= 0x07
    if map_write == "accumulate":
        state.map[y][x] |= st
    else:
        state.map[y][x] = st                # ファームは現マスを上書き
    state.known_cell[y][x] = True
    state.mark_seen(x, y)
    for rel in range(4):
        ad = abs_dir(d, rel)
        nx, ny = add(x, y, ad)
        if not (0 <= nx < state.n and 0 <= ny < state.n):
            continue
        opp = DIR_WALL[turn_back(ad)]
        if st & DIR_WALL[ad]:
            state.map[ny][nx] |= opp
        elif propagate == "fw":
            state.map[ny][nx] &= ~opp       # 「壁なし」も伝搬する


# --------------------------------------------------------------- 距離場
def bfs_field(state: MouseState, seeds) -> list:
    n = state.n
    dist = [[0xFFFF] * n for _ in range(n)]
    q = []
    for (sx, sy) in seeds:
        if dist[sy][sx] == 0xFFFF:
            dist[sy][sx] = 0
            q.append((sx, sy))
    head = 0
    while head < len(q):
        cx, cy = q[head]
        head += 1
        step = dist[cy][cx]
        for d in range(4):
            if state.map[cy][cx] & DIR_WALL[d]:
                continue
            nx, ny = add(cx, cy, d)
            if not (0 <= nx < n and 0 <= ny < n):
                continue
            if dist[ny][nx] == 0xFFFF:
                dist[ny][nx] = step + 1
                q.append((nx, ny))
    return dist


def dijkstra_field(state: MouseState, seeds, motion, turn_cost=None) -> list:
    """時間コスト（秒）の多起点 Dijkstra。方向をまたぐと旋回コストを足す。"""
    n = state.n
    if turn_cost is None:
        turn_cost = {1: motion.turn_time(90)[0], 2: motion.turn_time(180)[0],
                     3: motion.turn_time(90)[0]}
    t_cell = motion.straight_time(1)[0]
    # dist[dir] = その向きでセルに「向かう途中」のコスト。セル着地後は min を取る
    dist = [[[float("inf")] * 4 for _ in range(n)] for _ in range(n)]
    pq = []
    for (sx, sy) in seeds:
        for d in range(4):
            dist[sy][sx][d] = 0.0
            pq.append((0.0, sx, sy, d))
    heapq.heapify(pq)
    while pq:
        c, cx, cy, cd = heapq.heappop(pq)
        if c > dist[cy][cx][cd] + 1e-9:
            continue
        for rel in (0, 1, 2, 3):
            ad = abs_dir(cd, rel)
            if state.map[cy][cx] & DIR_WALL[ad]:
                continue
            nx, ny = add(cx, cy, ad)
            if not (0 <= nx < n and 0 <= ny < n):
                continue
            nc = c + t_cell + (0.0 if rel == 0 else turn_cost[rel])
            if nc < dist[ny][nx][ad] - 1e-9:
                dist[ny][nx][ad] = nc
                heapq.heappush(pq, (nc, nx, ny, ad))
    return [[min(dist[y][x]) for x in range(n)] for y in range(n)]


def seeds_unvisited(state: MouseState, maze):
    sx, sy = maze.start
    return [(x, y) for y in range(state.n) for x in range(state.n)
            if not state.known_cell[y][x] and (x, y) != (sx, sy)]


def frontier_cells(state: MouseState):
    """既知セルのうち、既知開口で未探索セルに隣接しているもの。"""
    out = []
    for y in range(state.n):
        for x in range(state.n):
            if not state.known_cell[y][x]:
                continue
            for d in range(4):
                nx, ny = add(x, y, d)
                if state.known_open(x, y, d) and not (0 <= nx < state.n and 0 <= ny < state.n):
                    continue
                if state.known_open(x, y, d) and not state.known_cell[ny][nx]:
                    out.append((x, y))
                    break
    return out


def field_at(state: MouseState, fld, x, y):
    v = fld[y][x]
    return float("inf") if v in (0xFFFF, float("inf")) else float(v)


def goal_reachable(state: MouseState, maze) -> bool:
    return bfs_field(state, maze.goals)[state.y][state.x] != 0xFFFF
