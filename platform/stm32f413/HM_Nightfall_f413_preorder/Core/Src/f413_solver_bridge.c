/**
 * @file f413_solver_bridge.c
 * @brief F413 ビルド向けにソルバ関連グローバル変数を実体化し、
 *        load_map_from_eeprom() を NVM API へブリッジする。
 *
 * global.h の MAIN_C_ パターンを利用して MF / g_ctrl_dt / g_debug /
 * map[][] / path[] / route[] 等を定義する。
 * solver.c / path.c / maze_grid.c はこれらのシンボルを extern 参照する。
 */

#define MAIN_C_
#include "global.h"

#include "nvm.h"
#include "nvm_params.h"
#include <string.h>

/* maze[][] / path_cell[][] は maze_grid.c で定義されるため、ここでは不要 */

/* search.h 内の bg_plan_ready（ENABLE_BG_REPLAN=0 時はマクロ置換されるが
   ヘッダでは extern volatile bool bg_plan_ready と宣言されている）。
   リンクエラー防止用に実体を置く。 */
volatile bool bg_plan_ready;

/**
 * @brief solver.c が呼ぶ load_map_from_eeprom() を NVM 経由で実装。
 *
 * F405 では内蔵 Flash エミュレーションだが、F413 では FRAM 経由の
 * nvm_maze_load_map() を使う。読み出したセルデータ（上位4bit=壁確定,
 * 下位4bit=探索中壁）を map[y][x] へ展開する。
 */
void load_map_from_eeprom(void)
{
    static uint16_t cells[MAZE_SIZE * MAZE_SIZE];

    memset(map, 0, sizeof(map));

    if (!nvm_maze_load_map(cells, (uint16_t)(MAZE_SIZE * MAZE_SIZE)))
    {
        return;
    }

    for (int y = 0; y < MAZE_SIZE; y++)
    {
        for (int x = 0; x < MAZE_SIZE; x++)
        {
            map[y][x] = cells[(uint32_t)y * MAZE_SIZE + (uint32_t)x];
        }
    }
}

/**
 * @brief store_map_in_eeprom() — F413 では未実装のスタブ。
 *
 * solver.c は直接呼ばないが search.h で宣言されているためリンク時に
 * 必要になる可能性がある。
 */
void store_map_in_eeprom(void) { /* stub */ }
