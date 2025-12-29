/*
 * global.h
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

#include "main.h"
#include "params.h"
#include <stdbool.h>

/*------------------------------------------------------------
    共用・構造体の定義
------------------------------------------------------------*/
/**********
共用・構造体とは，共用体と構造体を組み合わせたもので，
内部の一括操作も，メンバ単位での操作も可能なものである。
例えば，以下のmouse_flags共用・構造体のMFでは，
MF.FLAGS = 0;と全体を一括変更できるほか，
MF.FLAG.DECL = 1;とメンバを指定して単体で変更することも出来る。
**********/

//----フラグ共用・構造体----
typedef union { // 共用体の宣言
    uint64_t FLAGS;
    struct ms_flags {             // 構造体の宣言
        uint64_t SCND : 1;        // B0: 二次走行フラグ
        uint64_t RETURN : 1;      // B1: 帰り走行フラグ
        uint64_t CTRL : 1;        // B2: 壁制御フラグ
        uint64_t OVERRIDE : 1;    // B3: 割り込み制御無視フラグ
        uint64_t GET_LOG_1 : 1;   // B4: ログ取得フラグ(1)
        uint64_t GET_LOG_2 : 1;   // B5: ログ取得フラグ(2)
        uint64_t SPEED_x2 : 1;    // B6: 2倍加速状態フラグ
        uint64_t SPEED_x3 : 1;    // B7: 3倍加速状態フラグ
        uint64_t SPARE_16 : 1;    // B8:
        uint64_t TURN_1ST : 1;    // B9: 初回ターンフラグ
        uint64_t SLALOM_R : 1;    // B10: 右スラロームフラグ
        uint64_t SLALOM_L : 1;    // B11: 左スラロームフラグ
        uint64_t F_WALL : 1;      // B12: 前壁有りフラグ
        uint64_t F_WALL_STOP : 1; // B13: 停止の前壁補正フラグ
        uint64_t GOALED : 1;      // B14: 探索中のゴール済フラグ
        uint64_t SUCTION : 1; // B15: 吸引フラグ（ブザーとの干渉回避）
        uint64_t R_WALL : 1;        // B16: 右壁有りフラグ
        uint64_t L_WALL : 1;        // B17: 左壁有りフラグ
        uint64_t R_WALL_END : 1;    // B18: 右壁切れフラグ
        uint64_t L_WALL_END : 1;    // B19: 左壁切れフラグ
        uint64_t WALL_END : 1;      // B20: 壁切れ探しフラグ
        uint64_t CTRL_DIAGONAL : 1; // B21: 斜め制御フラグ
        uint64_t WALL_ALIGN : 1;    // B22: 壁揃えフラグ
        uint64_t SEARCH_HALF_RATE : 1;      // B23: 探索時のみ制御/エンコーダを0.5kHzに
        uint64_t SPARE_09 : 1;      // B24:
        uint64_t SPARE_10 : 1;      // B25:
        uint64_t SPARE_11 : 1;      // B26:
        uint64_t SPARE_12 : 1;      // B27:
        uint64_t SPARE_13 : 1;      // B28:
        uint64_t SPARE_14 : 1;      // B29:
        uint64_t RUNNING : 1;       // B31: 走行中フラグ
        uint64_t FAILED : 1;        // B31: フェイルセーフ発動フラグ
    } FLAG;
} mouse_flags;

typedef struct {
    bool test_mode_run;
    bool disable_wall_end_correction;
    bool disable_front_wall_correction;
    bool sensor_log_enabled;
} DebugFlags_t;

#ifdef MAIN_C_ // main.cからこのファイルが呼ばれている場合
/*グローバル変数の定義*/
volatile mouse_flags MF;
volatile float g_ctrl_dt;

/* テスト動作フラグ: trueの場合、センサ補正（壁切れ、前壁、横壁）を無効化 */
/* 壁切れ補正無効化フラグ: trueの場合、壁切れ検出を行わず距離ベースで走行 */
/* 前壁補正無効化フラグ: trueの場合、小回りターンの前壁補正(F_WALL)を無効化 */
volatile DebugFlags_t g_debug;
#else // main.c以外からこのファイルが呼ばれている場合
/*グローバル変数の宣言*/
extern volatile mouse_flags MF;
extern volatile float g_ctrl_dt;

/* テスト動作フラグ: trueの場合、センサ補正（壁切れ、前壁、横壁）を無効化 */
/* 壁切れ補正無効化フラグ: trueの場合、壁切れ検出を行わず距離ベースで走行 */
/* 前壁補正無効化フラグ: trueの場合、小回りターンの前壁補正(F_WALL)を無効化 */
extern volatile DebugFlags_t g_debug;
#endif

#define g_test_mode_run (g_debug.test_mode_run)
#define g_disable_wall_end_correction (g_debug.disable_wall_end_correction)
#define g_disable_front_wall_correction (g_debug.disable_front_wall_correction)
#define g_sensor_log_enabled (g_debug.sensor_log_enabled)

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "auxiliary.h"
#include "control.h"
#include "drive.h"
#include "eeprom.h"
#include "interrupt.h"
#include "mode1.h"
#include "mode2.h"
#include "mode3.h"
#include "mode4.h"
#include "mode5.h"
#include "mode6.h"
#include "mode7.h"
#include "path.h"
#include "run.h"
#include "search.h"
#include "sensor.h"
#include "test_mode.h"

#endif /* INC_GLOBAL_H_ */
