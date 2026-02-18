/*
 * search.h
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#ifndef INC_SEARCH_H_
#define INC_SEARCH_H_

// 経路配列の最大長（迷路全マス数に追従）
#ifndef ROUTE_MAX_LEN
#define ROUTE_MAX_LEN (MAZE_SIZE * MAZE_SIZE)
#endif

/*============================================================
    各種定数・変数宣言
============================================================*/
/**********
迷路の絶対座標について，スタート地点が左下になるような位置から見たとき，
上方向を北，右方向を東，下方向を南，左方向を西として定義している。
m_dirの場合，北は0x00，東は0x01，南は0x02，西は0x03で表される。（turn_dir関数参照）
また，マップ格納配列map[][]について，要素は上位4nitと下位4bitに分かれていて，
北壁は3bit目，東壁は2bit目，南壁は1bit目，西壁は0bit目に，
壁がある場合は1，ない場合は0が格納されている。
下位4bitは1次走行用(未探索壁は無しと判定)で上位4bitは2次走行用（未探索壁は有りと判定）
を表している。（write_map関数およびmap_Init関数参照）
最後に，最短経路格納配列route[]について，進行順に移動が記録されている。
各要素に機体が前進する際は0x88が，右折する際は0x44が，Uターンする際は0x22が，左折する場合は0x11が，
それ以外の場合には0x00が格納される（make_route関数参照）。なお，進行の経過はr_cntで管理されている。
**********/

//----現在地・方角格納構造体----
struct coordinate_and_direction {
    uint8_t x;
    uint8_t y;
    uint8_t dir;
};

#ifdef MAIN_C_ // main.cからこのファイルが呼ばれている場合
/*グローバル変数の定義*/
volatile struct coordinate_and_direction mouse;
#else // main.c以外からこのファイルが呼ばれている場合
/*グローバル変数の宣言*/
extern volatile struct coordinate_and_direction mouse;
#endif

//----方向転換用定数----
#define DIR_TURN_R90 0x01 // 右90度回転
#define DIR_TURN_L90 0xff // 左90度回転
#define DIR_TURN_180 0x02 // 180度回転

// 探索モードの定義（0: 全面探索, 1: ゴール到達で終了）
typedef enum {
    SEARCH_MODE_FULL = 0,
    SEARCH_MODE_GOAL = 1,
} search_mode_t;

//====変数====
#ifdef MAIN_C_ // main.cからこのファイルが呼ばれている場合
/*グローバル変数の定義*/
uint16_t map[MAZE_SIZE][MAZE_SIZE];         // マップ格納配列
uint16_t smap[MAZE_SIZE][MAZE_SIZE];        // 歩数マップ格納配列
bool visited[MAZE_SIZE][MAZE_SIZE];         // 探索済区画の配列
uint16_t closest_unvisited_x; // 最近の未探索区画のX座標
uint16_t closest_unvisited_y; // 最近の未探索区画のY座標
bool search_end;              // 全面探索の終了
uint16_t wall_info;           // 壁情報格納変数
uint16_t goal_x, goal_y;      // ゴール座標
uint16_t route[ROUTE_MAX_LEN];          // 最短経路格納配列
uint16_t path[ROUTE_MAX_LEN];           // 最短経路パス配列
uint16_t r_cnt;               // 経路カウンタ

/*直線加速用*/
float strait_count; // 直線区画数
float accel;
float acceled_count; // 直線加速

/*既知区間加速用*/
bool known_straight;
bool acceled;

/*壁判定用のセンサ補正係数*/
float sensor_kx;
float fwall_kx;

/* 探索モード */
search_mode_t g_search_mode;

/* GOALモード時の到達判定にスタート座標を用いるフラグ（trueでSTART_X/START_Yを到達判定に含める） */
bool g_goal_is_start;

/* 第2フェーズ探索フラグ（ゴール区画からの再開始を示す） */
bool g_second_phase_search;

/* conf_route() 計算中に進んだ距離[mm]（次の走行で1回だけ相殺する） */
float g_search_coast_mm;

#else // main.c以外からこのファイルが呼ばれている場合
/*グローバル変数の宣言*/
extern uint16_t map[MAZE_SIZE][MAZE_SIZE];         // マップ格納配列
extern uint16_t smap[MAZE_SIZE][MAZE_SIZE];        // 歩数マップ格納配列
extern bool visited[MAZE_SIZE][MAZE_SIZE];         // 探索済区画の配列
extern uint16_t closest_unvisited_x; // 最近の未探索区画のX座標
extern uint16_t closest_unvisited_y; // 最近の未探索区画のY座標
extern bool search_end;              // 全面探索の終了
extern uint16_t wall_info;           // 壁情報格納変数
extern uint16_t goal_x, goal_y;      // ゴール座標
extern uint16_t route[ROUTE_MAX_LEN];          // 最短経路格納配列
extern uint16_t path[ROUTE_MAX_LEN];           // 最短経路パス配列
extern uint16_t r_cnt;               // 経路カウンタ

/*直線加速用*/
extern float strait_count; // 直線区画数
extern float accel;
extern float acceled_count; // 直線加速

/*既知区間加速用*/
extern bool known_straight;
extern bool acceled;

/*壁判定用のセンサ補正係数*/
extern float sensor_kx;
extern float fwall_kx;

/* 探索モード */
extern search_mode_t g_search_mode;

/* GOALモード時に開始座標を到達判定に含めるフラグ */
extern bool g_goal_is_start;

/* 第2フェーズ探索フラグ（ゴール区画からの再開始を示す） */
extern bool g_second_phase_search;

/* conf_route() 計算中に進んだ距離[mm]（次の走行で1回だけ相殺する） */
extern float g_search_coast_mm;

#endif

/*============================================================
    関数プロトタイプ宣言
============================================================*/
//====探索系====

void search_init(void);

void searchA();         // 1区画停止型探索走行
void searchB(uint16_t); // 連続探索走行
void adachi(uint16_t);  // 全面探索のための足立法

void adv_pos();                     // マウスの位置情報を前進
void conf_route();                  // 次ルートの確認
void map_Init();                    // マップデータ初期化
void write_map();                   // マップ書き込み
void turn_dir(uint8_t);             // 自機方向情報変更
int make_smap(uint8_t, uint8_t);    // 歩数マップ作成
void make_route();                  // 最短経路検索
void markVisited(uint8_t, uint8_t); // 探索済区画の管理
void findClosestUnvisitedCell(uint8_t, uint8_t); // 最近の未探索区画を選択

void store_map_in_eeprom(void);
void load_map_from_eeprom(void);

// 経路が存在する場合のみFlashへ保存する。保存したらtrue、保存しなければfalse。
bool try_store_map_safely(void);

// 探索モード設定
void set_search_mode(search_mode_t mode);

//==== 背景再計算（走行中に実行）====
// 現在の移動内容（routeの要素: 0x88直進, 0x44右, 0x22U, 0x11左）から、
// 次区画の予測座標をもとに make_smap/make_route を先行実行する
void arm_background_replan(uint16_t move_opcode);
void background_replan_tick(void);
extern volatile bool bg_plan_ready; // 先行計算が完了し、route[] が次区画起点で準備済み

// 背景再計算の有効/無効を切り替えるマクロ（0: 無効, 1: 有効）
#ifndef ENABLE_BG_REPLAN
#define ENABLE_BG_REPLAN 0
#endif

#if !ENABLE_BG_REPLAN
// 無効化時はノーオペに展開
#define background_replan_tick() ((void)0)
#define arm_background_replan(move_opcode) ((void)(move_opcode))
#endif

#endif /* INC_SEARCH_H_ */
