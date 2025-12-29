#include "global.h"
#include "diag_solver_test.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// 方向定義（4近傍）
typedef enum {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3,
    DIR_UNKNOWN = 4
} Dir4;

// 2次元座標
typedef struct {
    int x;
    int y;
} Pos2D;

// ノード情報（コストと来歴）
typedef struct {
    float dist;              // スタートからの距離（コスト）
    bool visited;            // 訪問フラグ
    Pos2D prev;              // ひとつ前の座標
    Dir4 from_dir;           // どの方向から来たか
    Dir4 prev_from_dir;      // さらにその前にどの方向から来たか
    int diagonal_count;      // 連続「斜め」パターン回数
    int straight_count;      // 連続直線回数
} DS_Node;

// コスト設定（diagonal_solver_ascii と整合性のある安全値）
#define DS_MOVE_COST_NORMAL     1.0f
#define DS_MOVE_COST_STRAIGHT   0.8f
#define DS_MOVE_COST_DIAGONAL   0.85f
#define DS_MOVE_COST_MIN        0.05f
#define DS_STRAIGHT_DISCOUNT    0.006f
#define DS_DIAGONAL_DISCOUNT    0.005f
#define DS_TURN_PENALTY         0.5f

// 大きなワーキング領域はスタックを避け、静的に確保
static DS_Node g_nodes[MAZE_SIZE][MAZE_SIZE];
static Pos2D  g_path_buf[MAZE_SIZE * MAZE_SIZE * 2];

static inline Dir4 get_dir(Pos2D from, Pos2D to) {
    int dx = to.x - from.x;
    int dy = to.y - from.y;
    // 配列は top-left 原点（y増加で南）
    if (dx == 0 && dy == -1) return DIR_NORTH;
    if (dx == 1 && dy == 0)  return DIR_EAST;
    if (dx == 0 && dy == 1)  return DIR_SOUTH;
    if (dx == -1 && dy == 0) return DIR_WEST;
    return DIR_UNKNOWN;
}

static inline bool in_bounds(int x, int y) {
    return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

// 既存の maze[][] と dijkstra.h の壁ビット（NORTH_WALL=0x8, EAST_WALL=0x4, SOUTH_WALL=0x2, WEST_WALL=0x1）に従い移動可否を判定
static bool can_move_cell(Pos2D cur, Pos2D nxt) {
    if (!in_bounds(nxt.x, nxt.y)) return false;
    int dx = nxt.x - cur.x;
    int dy = nxt.y - cur.y;

    if (dx == 0 && dy == 1) {
        // 南へ（配列は上がy=0なので dy>0 は下方向＝南）
        if ((maze[cur.y][cur.x] & SOUTH_WALL) || (maze[nxt.y][nxt.x] & NORTH_WALL)) return false;
        return true;
    } else if (dx == 1 && dy == 0) {
        // 東へ
        if ((maze[cur.y][cur.x] & EAST_WALL) || (maze[nxt.y][nxt.x] & WEST_WALL)) return false;
        return true;
    } else if (dx == 0 && dy == -1) {
        // 北へ（配列は上がy=0なので dy<0 は上方向＝北）
        if ((maze[cur.y][cur.x] & NORTH_WALL) || (maze[nxt.y][nxt.x] & SOUTH_WALL)) return false;
        return true;
    } else if (dx == -1 && dy == 0) {
        // 西へ
        if ((maze[cur.y][cur.x] & WEST_WALL) || (maze[nxt.y][nxt.x] & EAST_WALL)) return false;
        return true;
    }
    return false;
}

// 斜めパターン判定（N-E-N / E-N-E / N-W-N / W-N-W / S-E-S / E-S-E / S-W-S / W-S-W）
static bool is_diagonal_pattern(Dir4 prev_dir, Dir4 cur_dir, Dir4 next_dir) {
    if ((prev_dir == DIR_NORTH && cur_dir == DIR_EAST && next_dir == DIR_NORTH) ||
        (prev_dir == DIR_EAST  && cur_dir == DIR_NORTH && next_dir == DIR_EAST)  ||
        (prev_dir == DIR_NORTH && cur_dir == DIR_WEST && next_dir == DIR_NORTH) ||
        (prev_dir == DIR_WEST  && cur_dir == DIR_NORTH && next_dir == DIR_WEST) ||
        (prev_dir == DIR_SOUTH && cur_dir == DIR_EAST && next_dir == DIR_SOUTH) ||
        (prev_dir == DIR_EAST  && cur_dir == DIR_SOUTH && next_dir == DIR_EAST)  ||
        (prev_dir == DIR_SOUTH && cur_dir == DIR_WEST && next_dir == DIR_SOUTH) ||
        (prev_dir == DIR_WEST  && cur_dir == DIR_SOUTH && next_dir == DIR_WEST)) {
        return true;
    }
    return false;
}

static float calc_move_cost(const DS_Node *cur_node, Dir4 next_dir) {
    // 初回移動
    if (cur_node->prev.x < 0) {
        return DS_MOVE_COST_NORMAL;
    }

    Dir4 cur_dir = cur_node->from_dir;
    if (cur_dir == next_dir) {
        // 直線継続
        float cost = DS_MOVE_COST_STRAIGHT - DS_STRAIGHT_DISCOUNT * (float)(cur_node->straight_count);
        return (cost < DS_MOVE_COST_MIN) ? DS_MOVE_COST_MIN : cost;
    }

    if (cur_node->diagonal_count > 0) {
        Dir4 prev_dir = cur_node->prev_from_dir;
        if (is_diagonal_pattern(prev_dir, cur_dir, next_dir)) {
            float cost = DS_MOVE_COST_DIAGONAL - DS_DIAGONAL_DISCOUNT * (float)(cur_node->diagonal_count);
            return (cost < DS_MOVE_COST_MIN) ? DS_MOVE_COST_MIN : cost;
        }
    }

    // 通常ターン
    return DS_MOVE_COST_NORMAL + DS_TURN_PENALTY;
}

static int update_diag_count(const DS_Node *cur_node, Dir4 next_dir) {
    if (cur_node->prev.x < 0) return 0;
    Dir4 cur_dir = cur_node->from_dir;
    if (cur_dir == next_dir) return 0; // 直線なら斜め継続ではない
    if (cur_node->diagonal_count > 0) {
        Dir4 prev_dir = cur_node->prev_from_dir;
        if (is_diagonal_pattern(prev_dir, cur_dir, next_dir)) {
            int c = cur_node->diagonal_count + 1;
            return (c > 32) ? 32 : c;
        }
    }
    return 1; // 新しい斜めパターン開始
}

static int update_straight_count(const DS_Node *cur_node, Dir4 next_dir) {
    if (cur_node->prev.x < 0) return 0;
    Dir4 cur_dir = cur_node->from_dir;
    if (cur_dir == next_dir) {
        int c = cur_node->straight_count + 1;
        return (c > 32) ? 32 : c;
    }
    return 0;
}

// ゴール到達後、最後の方向に沿って壁に当たるまで延長（視認性向上用）
static int extend_path_after_goal(Pos2D *path, int length, Dir4 last_dir) {
    if (length <= 0 || last_dir == DIR_UNKNOWN) return length;
    Pos2D cur = path[length - 1];
    int extend = 0;
    while (extend < MAZE_SIZE) {
        Pos2D nxt = cur;
        switch (last_dir) {
            case DIR_NORTH: nxt.y -= 1; break; // 北はy-1
            case DIR_EAST:  nxt.x += 1; break;
            case DIR_SOUTH: nxt.y += 1; break; // 南はy+1
            case DIR_WEST:  nxt.x -= 1; break;
            default: return length;
        }
        if (!can_move_cell(cur, nxt)) break;
        if (length < (MAZE_SIZE * MAZE_SIZE * 2 - 1)) {
            path[length++] = nxt;
        } else {
            break;
        }
        cur = nxt;
        extend++;
    }
    return length;
}

// 斜め優先ダイクストラ（4近傍・斜めはコストモデルで表現）
static int diag_shortest_path(Pos2D start, Pos2D goal, Pos2D *out_path, int out_cap) {
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            g_nodes[y][x].dist = FLT_MAX;
            g_nodes[y][x].visited = false;
            g_nodes[y][x].prev.x = -1;
            g_nodes[y][x].prev.y = -1;
            g_nodes[y][x].from_dir = DIR_UNKNOWN;
            g_nodes[y][x].prev_from_dir = DIR_UNKNOWN;
            g_nodes[y][x].diagonal_count = 0;
            g_nodes[y][x].straight_count = 0;
        }
    }

    g_nodes[start.y][start.x].dist = 0.0f;

    while (1) {
        // 未訪問で最小コストのノードを選択
        float min_d = FLT_MAX;
        Pos2D cur = { -1, -1 };
        for (int y = 0; y < MAZE_SIZE; y++) {
            for (int x = 0; x < MAZE_SIZE; x++) {
                if (!g_nodes[y][x].visited && g_nodes[y][x].dist < min_d) {
                    min_d = g_nodes[y][x].dist;
                    cur.x = x; cur.y = y;
                }
            }
        }
        if (cur.x < 0) break; // 見つからない

        if (cur.x == goal.x && cur.y == goal.y) {
            break; // ゴール
        }

        g_nodes[cur.y][cur.x].visited = true;

        // 4方向へ緩和（Dir4順：N,E,S,W）。top-left原点に合わせて dy を定義
        const int dx[4] = {0, 1, 0, -1};
        const int dy[4] = {-1, 0, 1, 0};
        for (int i = 0; i < 4; i++) {
            Pos2D nxt = { cur.x + dx[i], cur.y + dy[i] };
            if (!in_bounds(nxt.x, nxt.y)) continue;
            if (!can_move_cell(cur, nxt)) continue;

            DS_Node *cn = &g_nodes[cur.y][cur.x];
            float move_cost = calc_move_cost(cn, (Dir4)i);
            float nd = cn->dist + move_cost;
            DS_Node *nn = &g_nodes[nxt.y][nxt.x];
            if (nd < nn->dist) {
                nn->dist = nd;
                nn->prev = cur;
                nn->prev_from_dir = cn->from_dir;
                nn->from_dir = (Dir4)i;
                nn->diagonal_count = update_diag_count(cn, (Dir4)i);
                nn->straight_count = update_straight_count(cn, (Dir4)i);
            }
        }
    }

    // 経路復元
    if (g_nodes[goal.y][goal.x].dist == FLT_MAX) {
        return 0; // 見つからず
    }

    // 長さを数える
    int length = 1;
    Pos2D cur = goal;
    while (!(cur.x == start.x && cur.y == start.y)) {
        Pos2D p = g_nodes[cur.y][cur.x].prev;
        if (p.x < 0) { length = 0; break; }
        length++;
        cur = p;
        if (length > MAZE_SIZE * MAZE_SIZE * 2) { length = 0; break; }
    }
    if (length <= 0) return 0;

    if (length > out_cap) length = out_cap; // 収まりきらない分は切り捨て

    // 経路を前から順に格納
    cur = goal;
    int idx = length - 1;
    out_path[idx] = cur;
    while (idx > 0) {
        idx--;
        cur = g_nodes[cur.y][cur.x].prev;
        out_path[idx] = cur;
    }

    // 最終方向で延長（視認性向上）
    Dir4 last_dir = DIR_UNKNOWN;
    if (length >= 2) {
        last_dir = get_dir(out_path[length - 2], out_path[length - 1]);
        length = extend_path_after_goal(out_path, length, last_dir);
    }

    return length;
}

void run_diagonal_solver_test(void) {
    printf("[DiagSolverTest] Start.\n");

    // EEPROM から map を読み込み、既存のパイプラインで maze を構築
    load_map_from_eeprom();

    uint8_t fixedMap[MAZE_SIZE][MAZE_SIZE];
    initializeMaze(fixedMap);

    // map の下位4bitを抽出し y 反転して maze に適用
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            fixedMap[y][x] = (map[y][x] >> 4) & 0x0F; // NESW の各壁ビット
        }
    }

    reverseArrayYAxis(fixedMap);
    setMazeWalls(fixedMap);
    correctWallInconsistencies();

    // 経路マーキング配列をクリア
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            path_cell[y][x] = false;
        }
    }

    // スタート/ゴール（bottom-left座標 -> top-left座標に変換）
    Pos2D start = { START_X, START_Y };
    Pos2D goal  = { GOAL_X,  GOAL_Y  };
    Pos2D start_tl = { start.x, MAZE_SIZE - 1 - start.y };
    Pos2D goal_tl  = { goal.x,  MAZE_SIZE - 1 - goal.y  };

    // 経路探索
    int path_len = diag_shortest_path(start_tl, goal_tl, g_path_buf, (int)(sizeof(g_path_buf) / sizeof(g_path_buf[0])));

    if (path_len <= 0) {
        printf("[DiagSolverTest] 経路が見つかりませんでした。\n");
        printMaze();
        return;
    }

    // 経路を path_cell にマーキング（printMaze で '+' 表示。'*' の代替として流用）
    for (int i = 0; i < path_len; i++) {
        int x = g_path_buf[i].x;
        int y = g_path_buf[i].y;
        if (in_bounds(x, y)) {
            // path_cell は bottom-left 座標系で保持するため、Y を反転してマーキング
            int y_bl = MAZE_SIZE - 1 - y;
            path_cell[y_bl][x] = true;
        }
    }

    // 情報出力
    printf("[DiagSolverTest] 経路長: %d\n", path_len);
    printf("[DiagSolverTest] 経路座標: ");
    for (int i = 0; i < path_len; i++) {
        printf("(%d,%d)%s", g_path_buf[i].x, g_path_buf[i].y, (i + 1 < path_len) ? " -> " : "\n");
    }

    // 迷路表示（既存の printMaze を流用）
    printMaze();
}
