#include "global.h"
#include "maze_grid.h"
#include "solver.h"
#include "solver_params.h"
#include "path.h"
#include "shortest_run_params.h"

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
} NodeCost;

// 大きなワーキング領域はスタックを避け、静的に確保
static NodeCost g_nodes[MAZE_SIZE][MAZE_SIZE];
static Pos2D    g_path_buf[MAZE_SIZE * MAZE_SIZE * 2];

// 旧dijkstra.h への依存を避けるためのフォールバック定義
#ifndef MOVE_NORTH
#define MOVE_NORTH 100
#endif
#ifndef MOVE_EAST
#define MOVE_EAST 101
#endif
#ifndef MOVE_SOUTH
#define MOVE_SOUTH 102
#endif
#ifndef MOVE_WEST
#define MOVE_WEST 103
#endif
#ifndef NORTH
#define NORTH 0
#endif
#ifndef EAST
#define EAST 1
#endif
#ifndef SOUTH
#define SOUTH 2
#endif
#ifndef WEST
#define WEST 3
#endif
#ifndef STRAIGHT
#define STRAIGHT 200
#endif
#ifndef TURN_R
#define TURN_R 300
#endif
#ifndef TURN_L
#define TURN_L 400
#endif

// 先行宣言
static inline bool in_bounds(int x, int y);
static bool can_move_cell(Pos2D cur, Pos2D nxt);
static int shortest_path(Pos2D start, Pos2D goal, Pos2D *out_path, int out_cap, const SolverCaseParams_t* sp);

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

// dijkstra.c の convertPathCellToRun と同等の処理をローカルに実装
static void convertDirectionTokensToRun(void) {
    int8_t Direction = NORTH;
    for (int i = 0; i < 256; i++) {

        if (path[i] - 100 == Direction) {
            // 直進
            path[i] = STRAIGHT;
        } else if (path[i] == MOVE_EAST) {
            if (Direction == NORTH) {
                path[i] = TURN_R;
            } else if (Direction == SOUTH) {
                path[i] = TURN_L;
            }
            Direction = EAST;
        } else if (path[i] == MOVE_SOUTH) {
            if (Direction == EAST) {
                path[i] = TURN_R;
            } else if (Direction == WEST) {
                path[i] = TURN_L;
            }
            Direction = SOUTH;
        } else if (path[i] == MOVE_WEST) {
            if (Direction == SOUTH) {
                path[i] = TURN_R;
            } else if (Direction == NORTH) {
                path[i] = TURN_L;
            }
            Direction = WEST;
        } else if (path[i] == MOVE_NORTH) {
            if (Direction == WEST) {
                path[i] = TURN_R;
            } else if (Direction == EAST) {
                path[i] = TURN_L;
            }
            Direction = NORTH;
        } else {
            // 0 or others
        }
    }
}

// ゴール進入時の直進区画数を計算（経路の末尾から同じ方向が続く区画数）
static int calc_goal_approach_straight(Pos2D *path_buf, int path_len) {
    if (path_len < 2) return 0;
    
    int straight_count = 1;
    // 最後の移動方向を取得
    int last_dx = path_buf[path_len - 1].x - path_buf[path_len - 2].x;
    int last_dy = path_buf[path_len - 1].y - path_buf[path_len - 2].y;
    
    // 末尾から同じ方向が続く区画数をカウント
    for (int i = path_len - 2; i >= 1; i--) {
        int dx = path_buf[i].x - path_buf[i - 1].x;
        int dy = path_buf[i].y - path_buf[i - 1].y;
        if (dx == last_dx && dy == last_dy) {
            straight_count++;
        } else {
            break;
        }
    }
    return straight_count;
}

void solver_build_path(uint8_t mode, uint8_t case_index) {
    // 最短走行パラメータからソルバプロファイルを設定
    const ShortestRunCaseParams_t* cp = shortest_get_case_params(mode, case_index);
    solver_set_profile(cp->solver_profile);

    // 壁・迷路を構築
    load_map_from_eeprom();
    uint8_t fixedMap[MAZE_SIZE][MAZE_SIZE];
    initializeMaze(fixedMap);
    for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
            fixedMap[y][x] = (map[y][x] >> 4) & 0x0F;
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

    // スタート座標（bottom-left -> top-left）
    Pos2D start_bl = { START_X, START_Y };
    Pos2D start_tl = { start_bl.x, MAZE_SIZE - 1 - start_bl.y };

    // ケースパラメータ取得
    const SolverCaseParams_t* sp = solver_get_case_params(mode, case_index);

    // 複数ゴール座標を定義
    const uint8_t goals[9][2] = {
        {GOAL1_X, GOAL1_Y}, {GOAL2_X, GOAL2_Y}, {GOAL3_X, GOAL3_Y},
        {GOAL4_X, GOAL4_Y}, {GOAL5_X, GOAL5_Y}, {GOAL6_X, GOAL6_Y},
        {GOAL7_X, GOAL7_Y}, {GOAL8_X, GOAL8_Y}, {GOAL9_X, GOAL9_Y},
    };

    // 全ゴール座標を探索し、最適な経路を選択
    float best_cost = FLT_MAX;
    int best_straight = 0;
    int best_path_len = 0;
    static Pos2D best_path_buf[MAZE_SIZE * MAZE_SIZE];
    
    for (int g = 0; g < 9; g++) {
        uint8_t gx = goals[g][0];
        uint8_t gy = goals[g][1];
        
        // (0,0) は未使用スロットとして無視
        if (gx == 0 && gy == 0) continue;
        if (gx >= MAZE_SIZE || gy >= MAZE_SIZE) continue;
        
        // ゴール座標を変換（bottom-left -> top-left）
        Pos2D goal_tl = { gx, MAZE_SIZE - 1 - gy };
        
        // 経路探索
        int path_len = shortest_path(start_tl, goal_tl, g_path_buf, 
                                     (int)(sizeof(g_path_buf) / sizeof(g_path_buf[0])), sp);
        
        if (path_len <= 0) continue;
        
        // このゴールへの経路コストを取得
        float cost = g_nodes[goal_tl.y][goal_tl.x].dist;
        
        // ゴール進入時の直進区画数を計算
        int approach_straight = calc_goal_approach_straight(g_path_buf, path_len);
        
        // 最適経路の選択：コストが小さい、または同じコストで直進距離が長い
        bool is_better = false;
        if (cost < best_cost - 0.001f) {
            is_better = true;
        } else if (cost < best_cost + 0.001f && approach_straight > best_straight) {
            is_better = true;
        }
        
        if (is_better) {
            best_cost = cost;
            best_straight = approach_straight;
            best_path_len = path_len;
            for (int i = 0; i < path_len; i++) {
                best_path_buf[i] = g_path_buf[i];
            }
        }
    }

    // 初期化
    for (int i = 0; i < 256; i++) path[i] = 0;

    if (best_path_len <= 0) {
        // 経路無し
        return;
    }
    
    // 最適経路をg_path_bufにコピー
    for (int i = 0; i < best_path_len; i++) {
        g_path_buf[i] = best_path_buf[i];
    }
    int path_len = best_path_len;

    // path_cell マーキング（bottom-leftで保持）
    for (int i = 0; i < path_len; i++) {
        int x_tl = g_path_buf[i].x;
        int y_tl = g_path_buf[i].y;
        if (in_bounds(x_tl, y_tl)) {
            int y_bl = MAZE_SIZE - 1 - y_tl;
            path_cell[y_bl][x_tl] = true;
        }
    }

    // 方向トークンを path[] に格納（MOVE_*）
    uint16_t pc = 0;
    for (int i = 1; i < path_len; i++) {
        int dx = g_path_buf[i].x - g_path_buf[i-1].x;
        int dy = g_path_buf[i].y - g_path_buf[i-1].y; // top-left基準
        if (dx == 1 && dy == 0)      path[pc++] = MOVE_EAST;
        else if (dx == -1 && dy == 0)path[pc++] = MOVE_WEST;
        else if (dx == 0 && dy == -1)path[pc++] = MOVE_NORTH; // 上へ
        else if (dx == 0 && dy == 1) path[pc++] = MOVE_SOUTH; // 下へ
    }

    // 方向→走行パスへ変換
    convertDirectionTokensToRun();

    // makePath と同様の後段処理
    // モード共通パラメータから path_type を決定
    const ShortestRunModeParams_t *pm = shortest_get_mode_params(mode);
    // case1 は小回り（makepath_type_case3）、case3 は大回り（makepath_type_case47）
    int path_type = (case_index == 1) ? pm->makepath_type_case3 : pm->makepath_type_case47;
    // case8/9 は斜め走行を有効化
    if (case_index >= 8) {
        path_type = 2;
    }

    simplifyPath();
    if (path_type > 0) {
        convertLTurn();
    }

    // デバッグ表示（必要に応じて）
    for (int i = 0; i < 256 && path[i] != 0; i++) {
        printf("%d ", path[i]);
    }
    printf("\n");

    if (path_type > 1) {
        convertDiagonal();
    }

    printMaze();

    for (int i = 0; i < 256 && path[i] != 0; i++) {
        // printf("%d ", path[i]);
        if (path[i] < 300) {
            uint8_t str_sec;
            str_sec = path[i] - 200;
            printf("S%d", str_sec);
        } else if (path[i] == 300) {
            printf("S-R90");
        } else if (path[i] == 400) {
            printf("S-L90");
        } else if (path[i] == 501) {
            printf("L-R90");
        } else if (path[i] == 502) {
            printf("L-R180");
        } else if (path[i] == 601) {
            printf("L-L90");
        } else if (path[i] == 602) {
            printf("L-L180");
        } else if (path[i] == 701) {
            printf("R45-in");
        } else if (path[i] == 702) {
            printf("L45-in");
        } else if (path[i] == 703) {
            printf("R45-out");
        } else if (path[i] == 704) {
            printf("L45-out");
        } else if (path[i] == 801) {
            printf("R-V90");
        } else if (path[i] == 802) {
            printf("L-V90");
        } else if (path[i] == 901) {
            printf("R135-in");
        } else if (path[i] == 902) {
            printf("L135-in");
        } else if (path[i] == 903) {
            printf("R135-out");
        } else if (path[i] == 904) {
            printf("L135-out");
        } else if (path[i] > 1000) {
            uint8_t diag_sec;
            diag_sec = path[i] - 1000;
            printf("D-S%d", diag_sec);
        }
        printf(", ");
    }
    printf("Goal\n");
}

// 既存の maze[][] の壁ビット（NORTH_WALL/EAST_WALL/SOUTH_WALL/WEST_WALL）に従い移動可否を判定
static bool can_move_cell(Pos2D cur, Pos2D nxt) {
    if (!in_bounds(nxt.x, nxt.y)) return false;
    int dx = nxt.x - cur.x;
    int dy = nxt.y - cur.y;

    if (dx == 0 && dy == 1) {
        // 南（dy>0）
        if ((maze[cur.y][cur.x] & SOUTH_WALL) || (maze[nxt.y][nxt.x] & NORTH_WALL)) return false;
        return true;
    } else if (dx == 1 && dy == 0) {
        // 東
        if ((maze[cur.y][cur.x] & EAST_WALL) || (maze[nxt.y][nxt.x] & WEST_WALL)) return false;
        return true;
    } else if (dx == 0 && dy == -1) {
        // 北（dy<0）
        if ((maze[cur.y][cur.x] & NORTH_WALL) || (maze[nxt.y][nxt.x] & SOUTH_WALL)) return false;
        return true;
    } else if (dx == -1 && dy == 0) {
        // 西
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

static float calc_move_cost(const NodeCost *cur_node, Dir4 next_dir, const SolverCaseParams_t* sp) {
    // 初回移動
    if (cur_node->prev.x < 0) {
        return sp->move_cost_normal;
    }

    Dir4 cur_dir = cur_node->from_dir;
    if (cur_dir == next_dir) {
        // 直線継続
        float cost = sp->move_cost_straight - sp->straight_discount * (float)(cur_node->straight_count);
        return (cost < sp->move_cost_min) ? sp->move_cost_min : cost;
    }

    if (cur_node->diagonal_count > 0) {
        Dir4 prev_dir = cur_node->prev_from_dir;
        if (is_diagonal_pattern(prev_dir, cur_dir, next_dir)) {
            float cost = sp->move_cost_diagonal - sp->diagonal_discount * (float)(cur_node->diagonal_count);
            return (cost < sp->move_cost_min) ? sp->move_cost_min : cost;
        }
    }

    // 通常ターン
    return sp->move_cost_normal + sp->turn_penalty;
}

static int update_diag_count(const NodeCost *cur_node, Dir4 next_dir) {
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

static int update_straight_count(const NodeCost *cur_node, Dir4 next_dir) {
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
static int shortest_path(Pos2D start, Pos2D goal, Pos2D *out_path, int out_cap, const SolverCaseParams_t* sp) {
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

            NodeCost *cn = &g_nodes[cur.y][cur.x];
            float move_cost = calc_move_cost(cn, (Dir4)i, sp);
            float nd = cn->dist + move_cost;
            NodeCost *nn = &g_nodes[nxt.y][nxt.x];
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

void solver_run(uint8_t mode, uint8_t case_index) {
    printf("[Solver] Start (mode=%u, case=%u)\n", (unsigned)mode, (unsigned)case_index);

    // EEPROM から map を読み込み、既存のパイプラインで maze を構築
    load_map_from_eeprom();

    uint8_t fixedMap[MAZE_SIZE][MAZE_SIZE];
    initializeMaze(fixedMap);

    // map の下位4bitを抽出し y 反転して maze に適用（既存の makePath と同じ）
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

    // ケースパラメータ取得
    const SolverCaseParams_t* sp = solver_get_case_params(mode, case_index);

    // 経路探索
    int path_len = shortest_path(start_tl, goal_tl, g_path_buf, (int)(sizeof(g_path_buf) / sizeof(g_path_buf[0])), sp);

    if (path_len <= 0) {
        printf("[Solver] 経路が見つかりませんでした。\n");
        printMaze();
        return;
    }

    // 経路を path_cell にマーキング（printMaze で '+' 表示）
    for (int i = 0; i < path_len; i++) {
        int x = g_path_buf[i].x;
        int y = g_path_buf[i].y;
        if (in_bounds(x, y)) {
            // bottom-left 座標で保持
            int y_bl = MAZE_SIZE - 1 - y;
            path_cell[y_bl][x] = true;
        }
    }

    // 情報出力
    printf("[Solver] 経路長: %d\n", path_len);
    printf("[Solver] 経路座標: ");
    for (int i = 0; i < path_len; i++) {
        printf("(%d,%d)%s", g_path_buf[i].x, g_path_buf[i].y, (i + 1 < path_len) ? " -> " : "\n");
    }

    // 迷路表示
    printMaze();
}
