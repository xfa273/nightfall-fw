"""viewer_patch.py - 観測マスク対応のセンサ/地図更新（firmware_model の拡張版）.

`firmware_model` の sense/write_map は「ファームと同じ挙動」を素直に写したもの。
ここでは可視化と比較のために、

  * どの方位を実際に観測したか（マスク）を保持する
  * 観測していない方位の「壁なし」を伝搬しない

という改善版を同じ器で表現する。`propagate` の取りうる値:

  fw      ... 現ファーム。観測可否に関係なく「壁なし」も隣へ伝搬する
  aware   ... 観測した方位だけ「壁なし」を伝搬する（改善版）
  safe    ... 「壁あり」だけ伝搬し、「壁なし」は隣へ広げない
"""

from __future__ import annotations

from firmware_model import add, abs_dir
from maze import DIR_WALL, turn_back, turn_left, turn_right

N_BIT, E_BIT, S_BIT, W_BIT = 0x08, 0x04, 0x02, 0x01
OPP_BIT = [0x02, 0x01, 0x08, 0x04]       # 方角コード -> 反対側の壁ビット


def sense_masked(maze, x, y, d, mode="fw"):
    """(観測ビット, 観測マスク) を返す。マスクはその方位を読んだか。"""
    if mode in ("ideal4", "ideal4_ahead"):
        return maze.walls[y][x] & 0x0F, 0x0F
    rel = 0
    if maze.wall(x, y, d):
        rel |= 8                                  # F
    if maze.wall(x, y, turn_right(d)):
        rel |= 4                                  # R
    if maze.wall(x, y, turn_left(d)):
        rel |= 1                                  # L (B はセンサ無し)
    if mode == "fw":                              # 現行: 右ローテート（ビットが流れ落ちる）
        return (rel >> d) & 0x0F, 0x0F            # 現行コードは 4 方位とも「分かった」扱い
    if mode == "fw_masked":                       # 変換は現行のまま、信頼できるスロットだけ信任
        msk = 0
        for a in range(4):
            if (a - d) & 3 == 2:
                continue                          # 後方はセンサ無し
            msk |= DIR_WALL[a]
        ideal = 0
        for a in range(4):                        # 正しい回転での期待値
            src = (a - d) & 3
            if src != 2 and (rel & (8 >> src)):
                ideal |= DIR_WALL[a]
        bits = (rel >> d) & 0x0F                  # 現行コードの実効値
        trust = 0
        for a in range(4):
            b = DIR_WALL[a]
            if (msk & b) and bool(ideal & b) == bool(bits & b):
                trust |= b                        # 期待値と一致するスロットだけ既知扱い
        return bits & trust, trust

    # 改善: 絶対方位へ正しく変換し、後方は unknown のまま
    bits = mask = 0
    for a in range(4):
        src = (a - d) & 3                         # 絶対方位 a を見る相対センサ
        if src == 2:
            continue                              # 後方は読めない
        mask |= DIR_WALL[a]
        if rel & (8 >> src):
            bits |= DIR_WALL[a]
    return bits, mask


def write_map_masked(state, maze, bits, mask, propagate="aware", map_write="accumulate"):
    """write_map() の拡張。

    map_write: overwrite（現行・現マスを上書き）/ accumulate（OR 蓄積）
    propagate: 上記 3 とおり
    """
    x, y, d = state.x, state.y, state.d
    if (x, y) == maze.start:
        bits |= 0x07
        mask |= 0x07
    for a in range(4):
        bit = DIR_WALL[a]
        if mask & bit:
            if bits & bit:
                state.map[y][x] |= bit
            elif map_write == "accumulate":
                state.map[y][x] &= ~bit
            else:
                state.map[y][x] &= ~bit
        elif map_write == "overwrite":
            state.map[y][x] &= ~bit               # 現行コードと同じ「消える」挙動
    state.known_cell[y][x] = True
    state.mark_seen(x, y)
    for a in range(4):
        nx, ny = add(x, y, a)
        if not (0 <= nx < state.n and 0 <= ny < state.n):
            continue
        bit = DIR_WALL[a]
        opp = OPP_BIT[a]
        if (mask & bit) and (bits & bit):
            state.map[ny][nx] |= opp               # 壁ありは必ず伝搬
        elif (mask & bit) and propagate in ("fw", "aware"):
            state.map[ny][nx] &= ~opp              # 観測した「壁なし」だけを伝搬
        elif propagate == "fw" and map_write == "overwrite":
            state.map[ny][nx] &= ~opp              # 現行コードの暴挙（未知を開口扱い）


def map_diff_bits(state, maze):
    """(誤って壁ありと信じている数, 実際は壁なのに未知/開口扱いの数)。"""
    wrong = missing = 0
    for y in range(state.n):
        for x in range(state.n):
            for a in range(4):
                bit = DIR_WALL[a]
                known_wall = bool(state.map[y][x] & bit)
                real_wall = bool(maze.walls[y][x] & bit)
                if known_wall and not real_wall:
                    wrong += 1
                elif real_wall and not known_wall:
                    missing += 1
    return wrong, missing
