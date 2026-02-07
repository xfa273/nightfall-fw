/*
 * drive.c
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#include "global.h"
#include "main.h"
#include <params.h>
#include "drive.h"
#include "sensor.h"
#include "interrupt.h"
#include "logging.h"
#include <math.h>
#include <stdlib.h>

static inline float consume_search_coast_mm(float planned_mm) {
    float dist = planned_mm;
    if (planned_mm <= 0.0f) return 0.0f;
    if (g_search_coast_mm > 0.0f) {
        dist = planned_mm - g_search_coast_mm;
        if (dist < 1e-3f) dist = 1e-3f;
        g_search_coast_mm = 0.0f;
    }
    return dist;
}

// ==== Motor control params (sign-magnitude) ====
// 周波数は TIM 設定に従う（本実装では変更しない）
// ARR はランタイムで読み出して使用する（__HAL_TIM_GET_AUTORELOAD）
#ifndef PWM_INVERT_DIR_LEVEL
#define PWM_INVERT_DIR_LEVEL 1  // DIR がこのレベルのとき PWM を反転（0:Lowで反転, 1:Highで反転）
#endif
#ifndef MOTOR_MIN_DUTY_PCT
#define MOTOR_MIN_DUTY_PCT 0    // デッドゾーン除去用の最低デューティ[%]
#endif
#ifndef MOTOR_BOOST_DUTY_PCT
#define MOTOR_BOOST_DUTY_PCT 0  // 起動ブーストのデューティ[%]
#endif
#ifndef MOTOR_BOOST_TIME_MS
#define MOTOR_BOOST_TIME_MS 0   // 起動ブースト時間[ms]
#endif

// 起動検出・ブースト管理用のローカル状態
static uint16_t s_prev_cmd_l = 0;
static uint16_t s_prev_cmd_r = 0;
static uint32_t s_boost_until_ms_l = 0;
static uint32_t s_boost_until_ms_r = 0;

// 現在のDIRピンレベル（High/Low）を保持
static uint8_t s_dir_pin_high_l = 0; // 0:Low, 1:High
static uint8_t s_dir_pin_high_r = 0; // 0:Low, 1:High

// CCR更新抑止フラグ：有効化/無効化シーケンス中の微小回転を防止
static volatile uint8_t s_outputs_locked = 0;

// モータドライバの有効/無効状態（STBY+PWM）
static volatile uint8_t s_motor_enabled = 0;

// 最後にファンを停止した時刻（ms）: ブザー起動のクールダウン制御用
volatile uint32_t fan_last_off_ms = 0;

static uint8_t s_fan_running = 0;

static volatile uint8_t s_fail_turn_angle_enabled = 0;
static volatile float s_fail_turn_angle_start_deg = 0.0f;
static volatile float s_fail_turn_angle_limit_deg = 0.0f;
static volatile uint16_t s_fail_turn_angle_count = 0;
static volatile uint8_t s_fail_turn_angle_armed_for_start = 0;

static inline void failsafe_turn_angle_begin(float cmd_angle_deg) {
    s_fail_turn_angle_enabled = 1;
    s_fail_turn_angle_start_deg = IMU_angle;
    s_fail_turn_angle_limit_deg = fabsf(cmd_angle_deg) + (float)FAIL_TURN_ANGLE_MARGIN_DEG;
    s_fail_turn_angle_count = 0;
    s_fail_turn_angle_armed_for_start = 1;
}

/*==========================================================
    走行系 上位関数
==========================================================*/
/*
    マウスフラグ(MF)
      6Bit:デフォルトインターバルフラグ
      5Bit:減速フラグ
      4Bit:加速フラグ
      3Bit:制御フラグ
      1Bit:二次走行フラグ
*/

//+++++++++++++++++++++++++++++++++++++++++++++++
// first_sectionA
// 半区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void first_sectionA(void) {
    float speed_out;

    speed_out =
        sqrt(speed_now * speed_now + 2 * acceleration_straight * DIST_FIRST_SEC);

    MF.FLAG.CTRL = 1;
    driveA(DIST_FIRST_SEC, speed_now, speed_out, 0);
    MF.FLAG.CTRL = 0;
    speed_now = speed_out;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// half_sectionA
// 半区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionA(uint16_t val) {
    float speed_out;

    if (val > 1) {
        speed_out = val;
    } else {
        if (MF.FLAG.SCND && !val) {
            // 最短走行時: 速度に応じて加速度を切り替え
            float accel;
            if (accel_switch_velocity > 0.0f && speed_now >= accel_switch_velocity) {
                accel = acceleration_straight_dash;  // 高速域
            } else {
                accel = acceleration_straight;       // 低速域
            }
            speed_out = sqrt(speed_now * speed_now + 2 * accel * DIST_HALF_SEC);
        } else {
            speed_out = sqrt(speed_now * speed_now +
                             2 * acceleration_straight * DIST_HALF_SEC);
        }
    }

    MF.FLAG.CTRL = 1;
    driveA(DIST_HALF_SEC, speed_now, speed_out, 0);
    MF.FLAG.CTRL = 0;
    speed_now = speed_out;

    if (val == 1) {
        get_wall_info();
    }
    // ロック中にDIRが変わった場合でも、IN1==IN2 を維持
    if (s_outputs_locked) {
        const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, s_dir_pin_high_l ? arr : 0u);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, s_dir_pin_high_r ? arr : 0u);
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// half_sectionAD
// 斜め半区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionAD(uint16_t val) {
    float speed_out;

    if (val > 1) {
        speed_out = val;
    } else {
        if (MF.FLAG.SCND && acceleration_straight_dash > 0 && !val) {
            speed_out = sqrt(speed_now * speed_now +
                             2 * acceleration_straight_dash * DIST_D_HALF_SEC);
        } else {
            speed_out = sqrt(speed_now * speed_now +
                             2 * acceleration_straight * DIST_D_HALF_SEC);
        }
    }

    MF.FLAG.CTRL = 1;
    driveA(DIST_D_HALF_SEC, speed_now, speed_out, 0);
    MF.FLAG.CTRL = 0;
    speed_now = speed_out;

    if (val == 1) {
        get_wall_info();
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// half_sectionD
// 半区画分減速しながら走行し停止する
// 引数：val（0=補正なし、1=前壁センサによる補正あり）
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionD(uint16_t val) {
    (void)val;           // ← 未使用を明示（コンパイラ警告回避）
    
    float speed_out = 0;  // 減速停止

    // 従来通りの動作（補正なし）
    MF.FLAG.CTRL = 1;
    float dist = consume_search_coast_mm(DIST_HALF_SEC);
    driveA(dist, speed_now, speed_out, 0);
    MF.FLAG.CTRL = 0;
    speed_now = speed_out;

    return;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// half_sectionDD
// 斜め半区画分減速しながら走行し停止する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionDD(uint16_t val) {
    float speed_out;

    if (val > 1) {
        speed_out = val;
    } else {
        if (MF.FLAG.SCND && acceleration_straight_dash > 0 && val) {
            speed_out = sqrt(speed_now * speed_now -
                             2 * acceleration_straight_dash * DIST_D_HALF_SEC);
        } else {
            speed_out = 0;
        }
    }

    MF.FLAG.CTRL = 1;

    driveA(DIST_D_HALF_SEC, speed_now, speed_out, 0);

    MF.FLAG.CTRL = 0;
    speed_now = speed_out;

    if (!val) {
        velocity_interrupt = 0;
    }

    // get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// one_sectionA
// 1区画分加速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionA(void) {
    float speed_out;
    /*
    if (MF.FLAG.SCND || known_straight) {
        // 最短走行時: 速度に応じて加速度を切り替え
        float accel;
        if (accel_switch_velocity > 0.0f && speed_now >= accel_switch_velocity) {
            accel = acceleration_straight_dash;  // 高速域
        } else {
            accel = acceleration_straight;       // 低速域
        }
        speed_out = sqrt(speed_now * speed_now + 2 * accel * DIST_HALF_SEC * 2);
    } else {
        speed_out = sqrt(speed_now * speed_now +
                         2 * acceleration_straight * DIST_HALF_SEC * 2);
    }
    */

    speed_out = sqrt(speed_now * speed_now +
                         2 * acceleration_straight * DIST_HALF_SEC * 2);
    

    MF.FLAG.CTRL = 1;
    // kp_wall = kp_wall * 2;
    driveA(DIST_HALF_SEC * 2, speed_now, speed_out, 0);
    MF.FLAG.CTRL = 0;
    // kp_wall = kp_wall * 1 / 2;
    speed_now = speed_out;

    get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// one_sectionD
//  1区間減速しながら走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionD(void) {
    // 探索向け: 単一の連続走行で減速し、必要なら壁切れ追従（半区画+バッファ）を動的に行う
    float v0 = speed_now;
    float accel_lin;
    /*
    if (MF.FLAG.SCND || acceled) {
        // 最短走行時: 速度に応じて加速度を切り替え
        if (accel_switch_velocity > 0.0f && v0 >= accel_switch_velocity) {
            accel_lin = acceleration_straight_dash;  // 高速域
        } else {
            accel_lin = acceleration_straight;       // 低速域
        }
    } else {
        accel_lin = acceleration_straight;
    }
    */

    accel_lin = acceleration_straight;

    float speed_out = sqrtf(fmaxf(0.0f, v0 * v0 - 2.0f * accel_lin * (DIST_HALF_SEC * 2.0f)));

    MF.FLAG.CTRL = 1;

    // 前壁停止は維持
    if (ad_fl > WALL_BASE_FL || ad_fr > WALL_BASE_FR) {
        MF.FLAG.F_WALL_STOP = 1;
    }

    // 壁切れ検出をリセット（ターン後の状態を初期化）
    wall_end_reset();

    // driveA 内で壁切れ追従（探索のみ一時SCND=1でアーム）。追従距離は DIST_HALF_SEC + dist_wall_end。
    driveA(DIST_HALF_SEC * 2.0f, speed_now, speed_out, dist_wall_end);
    // 減速後の現在速度を反映
    speed_now = speed_out;

    MF.FLAG.F_WALL_STOP = 0;
    MF.FLAG.CTRL = 0;
    get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// one_section
// 1区画分進んで停止する。1区画走行用
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_section(void) {}

//+++++++++++++++++++++++++++++++++++++++++++++++
// one_sectionU
// 等速で1区画分進む（壁切れ補正付き）
// 壁切れ検出時: 検出位置から45mm追加直進して終了
// 壁切れ未検出時: 90mm（1区画）で終了
// 引数：section（未使用）, spd_out（未使用）
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void one_sectionU(float section, float spd_out) {
    (void)section;
    (void)spd_out;
    
    const float v_const = speed_now;
    float dist_max = DIST_HALF_SEC * 2.0f;  // 90mm（1区画）
    dist_max = consume_search_coast_mm(dist_max);
    // 壁切れ後の追加直進: 小回りターン用なので45mm + dist_wall_end
    // const float follow_dist = DIST_HALF_SEC + dist_wall_end;
    const float follow_dist = dist_wall_end;
    
    MF.FLAG.CTRL = 1;
    
    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
    target_distance = 0;  // 距離制御の目標値もリセット
    
    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;
    
    // 等速走行
    acceleration_interrupt = 0.0f;
    velocity_interrupt = v_const;
    
    // 壁切れ検出をアーム
    wall_end_reset();
    MF.FLAG.WALL_END = 1;
    
    drive_start();
    
    bool wall_end_detected = false;
    
    // 壁切れ検出または最大距離到達まで走行
    while (real_distance < dist_max && !MF.FLAG.FAILED) {
        if (wall_end_detected_r || wall_end_detected_l) {
            wall_end_detected = true;
            break;
        }
        background_replan_tick();
    }
    
    // 壁切れ検出をディスアーム
    MF.FLAG.WALL_END = 0;
    
    // 壁切れ検出時は45mm追加直進（壁の端から次の区画中心への位置補正）
    if (wall_end_detected) {
        // 走行距離カウントをリセット
        real_distance = 0;
        encoder_distance_r = 0;
        encoder_distance_l = 0;
        target_distance = 0;
        
        // 45mm追加直進
        while (real_distance < follow_dist && !MF.FLAG.FAILED) {
            background_replan_tick();
        }
    }
    
    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
    target_distance = 0;
    
    MF.FLAG.CTRL = 0;
    speed_now = v_const;
    get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// half_sectionU
// 等速で1区画分進む
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void half_sectionU(void) {
    MF.FLAG.CTRL = 1;
    driveA(DIST_HALF_SEC, speed_now, speed_now, 0);
    MF.FLAG.CTRL = 0;
    speed_now = speed_now;

    get_wall_info();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// run_straight
// 指定区画を指定速度で走行する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void run_straight(float section, float spd_out, float dist_wallend) {
    (void)dist_wallend;

    MF.FLAG.CTRL = 1;
    driveA(DIST_HALF_SEC * section, speed_now, spd_out, 0);

    MF.FLAG.CTRL = 0;
    speed_now = spd_out;

    if (!spd_out) {
        velocity_interrupt = 0;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// rotate_R90
// 右に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_R90(void) {
    driveR(ANGLE_ROTATE_90_R);
    // drive_stop();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// rotate_L90
// 左に90度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_L90(void) {
    driveR(-ANGLE_ROTATE_90_L);
    // drive_stop();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// reverse_distance
// 指定距離だけ後退する
// 引数：後退距離[mm]
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void reverse_distance(float distance_mm) {
    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 設定距離だけ後進
    velocity_interrupt = -60;
    drive_start();
    while(real_distance > - distance_mm){}
    drive_stop();

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    speed_now = 0;  // 後退後は停止状態
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// rotate_180
// 180度回転する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void rotate_180(void) { driveR(ANGLE_ROTATE_90_R * 2); }

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_R90
// スラロームで右に90度旋回前進する
// 引数：前壁補正のON/OFF
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R90(uint8_t fwall) {
    MF.FLAG.SLALOM_R = 1;
    MF.FLAG.CTRL = 1;

    float dist_in = consume_search_coast_mm(dist_offset_in);

    // テスト動作フラグが立っていいる場合は前壁補正を無効化
    if (fwall && !g_test_mode_run) {
        if (MF.FLAG.F_WALL) {
            driveFWall(dist_in, speed_now, velocity_turn90);
        } else {
            driveA(dist_in, speed_now, velocity_turn90, 0);
        }
    } else {
        driveA(dist_in, speed_now, velocity_turn90, 0);
    }

    MF.FLAG.CTRL = 0;

    driveSR(angle_turn_90, alpha_turn90);
    MF.FLAG.CTRL = 1;
    driveA(dist_offset_out, velocity_turn90, speed_now, 0);
    MF.FLAG.CTRL = 0;
    get_wall_info();
    MF.FLAG.SLALOM_R = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_L90
// スラロームで左に90度旋回前進する
// 引数：引数：前壁補正のON/OFF
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L90(uint8_t fwall) {
    MF.FLAG.SLALOM_L = 1;
    MF.FLAG.CTRL = 1;

    float dist_in = consume_search_coast_mm(dist_offset_in);

    // テスト動作フラグが立っていいる場合は前壁補正を無効化
    if (fwall && !g_test_mode_run) {
        if (MF.FLAG.F_WALL) {
            driveFWall(dist_in, speed_now, velocity_turn90);
        } else {
            driveA(dist_in, speed_now, velocity_turn90, 0);
        }
    } else {
        driveA(dist_in, speed_now, velocity_turn90, 0);
    }

    MF.FLAG.CTRL = 0;

    driveSL(angle_turn_90, alpha_turn90);
    MF.FLAG.CTRL = 1;
    driveA(dist_offset_out, velocity_turn90, speed_now, 0);
    MF.FLAG.CTRL = 0;
    get_wall_info();
    MF.FLAG.SLALOM_L = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// l_turn_R90
// 90度大回り右旋回
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void l_turn_R90(bool next_is_large) {

    // スラローム前は壁制御OFF（元の挙動を維持）
    MF.FLAG.CTRL = 0;

    // 前距離（入口オフセット）
    if (dist_l_turn_in_90 > 0.0f) {
        MF.FLAG.CTRL = 1;
        driveA(dist_l_turn_in_90, speed_now, velocity_l_turn_90, 0);
        MF.FLAG.CTRL = 0;
    }

    MF.FLAG.SLALOM_R = 1;
    driveSR(angle_l_turn_90, alpha_l_turn_90);
    MF.FLAG.SLALOM_R = 0;  // ターン終了後にフラグを落とす（壁切れ検出を有効化）
    MF.FLAG.CTRL = 1;
    
    // 出オフセット（次が大回りターンの場合のみ壁切れ補正）
    if (next_is_large && !g_disable_wall_end_correction && dist_l_turn_out_90 > 0.0f) {
        bool found = driveC_wallend(dist_l_turn_out_90, velocity_l_turn_90);
        if (found) {
            // 壁切れ検出時: dist_wall_end分追加直進
            if (dist_wall_end > 0.0f) {
                driveA(dist_wall_end, speed_now, velocity_l_turn_90, 0);
            }
        }
    } else {
        // 次が大回りでない or 壁切れ補正無効時: 距離ベース
        driveA(dist_l_turn_out_90, speed_now, velocity_l_turn_90, 0);
    }
    
    MF.FLAG.CTRL = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// l_turn_L90
// 90度大回り左旋回
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void l_turn_L90(bool next_is_large) {

    // スラローム前は壁制御OFF（元の挙動を維持）
    MF.FLAG.CTRL = 0;

    // 前距離（入口オフセット）
    if (dist_l_turn_in_90 > 0.0f) {
        MF.FLAG.CTRL = 1;
        driveA(dist_l_turn_in_90, speed_now, velocity_l_turn_90, 0);
        MF.FLAG.CTRL = 0;
    }

    MF.FLAG.SLALOM_L = 1;
    driveSL(angle_l_turn_90, alpha_l_turn_90);
    MF.FLAG.SLALOM_L = 0;  // ターン終了後にフラグを落とす（壁切れ検出を有効化）
    MF.FLAG.CTRL = 1;
    
    // 出オフセット（次が大回りターンの場合のみ壁切れ補正）
    if (next_is_large && !g_disable_wall_end_correction && dist_l_turn_out_90 > 0.0f) {
        bool found = driveC_wallend(dist_l_turn_out_90, velocity_l_turn_90);
        if (found) {
            // 壁切れ検出時: dist_wall_end分追加直進
            if (dist_wall_end > 0.0f) {
                driveA(dist_wall_end, speed_now, velocity_l_turn_90, 0);
            }
        }
    } else {
        // 次が大回りでない or 壁切れ補正無効時: 距離ベース
        driveA(dist_l_turn_out_90, speed_now, velocity_l_turn_90, 0);
    }

    MF.FLAG.CTRL = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// l_turn_R180
// 180度大回り右旋回
// 引数：前壁補正のON/OFF
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void l_turn_R180(bool next_is_large) {
    // スラローム前は壁制御OFF（元の挙動を維持）
    MF.FLAG.CTRL = 0;

    // 前距離（入口オフセット）
    if (dist_l_turn_in_180 > 0.0f) {
        MF.FLAG.CTRL = 1;
        driveA(dist_l_turn_in_180, speed_now, velocity_l_turn_180, 0);
        MF.FLAG.CTRL = 0;
    }

    MF.FLAG.SLALOM_R = 1;
    driveSR(angle_l_turn_180, alpha_l_turn_180);
    MF.FLAG.SLALOM_R = 0;  // ターン終了後にフラグを落とす（壁切れ検出を有効化）
    MF.FLAG.CTRL = 1;
    
    // 出オフセット（次が大回りターンの場合のみ壁切れ補正）
    if (next_is_large && !g_disable_wall_end_correction && dist_l_turn_out_180 > 0.0f) {
        bool found = driveC_wallend(dist_l_turn_out_180, velocity_l_turn_180);
        if (found) {
            // 壁切れ検出時: dist_wall_end分追加直進
            if (dist_wall_end > 0.0f) {
                driveA(dist_wall_end, speed_now, velocity_l_turn_180, 0);
            }
        }
    } else {
        // 次が大回りでない or 壁切れ補正無効時: 距離ベース
        driveA(dist_l_turn_out_180, speed_now, velocity_l_turn_180, 0);
    }
    MF.FLAG.CTRL = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// l_turn_L180
// 180度大回り左旋回
// 引数：前壁補正のON/OFF
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void l_turn_L180(bool next_is_large) {
    // スラローム前は壁制御OFF（元の挙動を維持）
    MF.FLAG.CTRL = 0;

    // 前距離（入口オフセット）
    if (dist_l_turn_in_180 > 0.0f) {
        MF.FLAG.CTRL = 1;
        driveA(dist_l_turn_in_180, speed_now, velocity_l_turn_180, 0);
        MF.FLAG.CTRL = 0;
    }

    MF.FLAG.SLALOM_L = 1;
    driveSL(angle_l_turn_180, alpha_l_turn_180);
    MF.FLAG.SLALOM_L = 0;  // ターン終了後にフラグを落とす（壁切れ検出を有効化）
    MF.FLAG.CTRL = 1;
    
    // 出オフセット（次が大回りターンの場合のみ壁切れ補正）
    if (next_is_large && !g_disable_wall_end_correction && dist_l_turn_out_180 > 0.0f) {
        bool found = driveC_wallend(dist_l_turn_out_180, velocity_l_turn_180);
        if (found) {
            // 壁切れ検出時: dist_wall_end分追加直進
            if (dist_wall_end > 0.0f) {
                driveA(dist_wall_end, speed_now, velocity_l_turn_180, 0);
            }
        }
    } else {
        // 次が大回りでない or 壁切れ補正無効時: 距離ベース
        driveA(dist_l_turn_out_180, speed_now, velocity_l_turn_180, 0);
    }
    MF.FLAG.CTRL = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_R45_In
// 右45度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R45_In(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    // 前直進（必要量）
    driveA(dist_turn45in_in, speed_now, velocity_turn45in, 0);
    // 旋回
    driveSR(angle_turn45in, alpha_turn45in);
    // 出オフセット
    driveA(dist_turn45in_out, speed_now, velocity_turn45in, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_R45_Out
// 右45度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R45_Out(void) {

    MF.FLAG.CTRL = 0;
    driveA(dist_turn45out_in, speed_now, velocity_turn45out, 0);
    driveSR(angle_turn45out, alpha_turn45out);
    driveA(dist_turn45out_out, speed_now, velocity_turn45out, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_L45_In
// 右45度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L45_In(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    // 前直進（必要量）
    driveA(dist_turn45in_in, speed_now, velocity_turn45in, 0);
    // 旋回
    driveSL(angle_turn45in, alpha_turn45in);
    // 出オフセット
    driveA(dist_turn45in_out, speed_now, velocity_turn45in, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_L45_Out
// 右45度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L45_Out(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    driveA(dist_turn45out_in, speed_now, velocity_turn45out, 0);
    driveSL(angle_turn45out, alpha_turn45out);
    driveA(dist_turn45out_out, speed_now, velocity_turn45out, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_RV90
// 右V90度ターン
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_RV90(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    driveA(dist_turnV90_in, speed_now, velocity_turnV90, 0);
    driveSR(angle_turnV90, alpha_turnV90);
    driveA(dist_turnV90_out, speed_now, velocity_turnV90, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_LV90
// 右V90度ターン
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_LV90(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    driveA(dist_turnV90_in, speed_now, velocity_turnV90, 0);
    driveSL(angle_turnV90, alpha_turnV90);
    driveA(dist_turnV90_out, speed_now, velocity_turnV90, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_R135_In
// 右135度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R135_In(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    driveA(dist_turn135in_in, speed_now, velocity_turn135in, 0);
    driveSR(angle_turn135in, alpha_turn135in);
    driveA(dist_turn135in_out, speed_now, velocity_turn135in, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_R135_Out
// 右135度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_R135_Out(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    driveA(dist_turn135out_in, speed_now, velocity_turn135out, 0);
    driveSR(angle_turn135out, alpha_turn135out);
    driveA(dist_turn135out_out, speed_now, velocity_turn135out, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_L135_In
// 右135度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L135_In(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    driveA(dist_turn135in_in, speed_now, velocity_turn135in, 0);
    driveSL(angle_turn135in, alpha_turn135in);
    driveA(dist_turn135in_out, speed_now, velocity_turn135in, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_L135_Out
// 右135度ターン入り
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_L135_Out(void) {

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    driveA(dist_turn135out_in, speed_now, velocity_turn135out, 0);
    driveSL(angle_turn135out, alpha_turn135out);
    driveA(dist_turn135out_out, speed_now, velocity_turn135out, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// run_diagonal
// 斜め直進
// 引数：区画数，終端速度
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void run_diagonal(float section, float spd_out) {
    MF.FLAG.CTRL = 0;
    if(spd_out>1){
        MF.FLAG.CTRL_DIAGONAL = 1;
    }
    driveA(DIST_D_HALF_SEC * section, speed_now, spd_out, 0);

    MF.FLAG.CTRL = 0;
    MF.FLAG.CTRL_DIAGONAL = 0;
    speed_now = spd_out;

    if (!spd_out) {
        velocity_interrupt = 0;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_setposition_R90
// スラロームで右に90度旋回前進する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_setposition_R90(void) {

    half_sectionD(1);
    drive_wait();

    rotate_R90();
    drive_wait();

    set_position();

    half_sectionA(1);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// turn_setposition_L90
// スラロームで左に90度旋回前進する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void turn_setposition_L90(void) {

    half_sectionD(1);
    drive_wait();

    rotate_L90();
    drive_wait();

    set_position();

    half_sectionA(1);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// set_position
// 機体の尻を壁に当てて場所を区画中央に合わせる
// 引数：sw …… 0以外ならget_base()する
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void set_position(void) {

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 設定距離だけ後進
    velocity_interrupt = -130;
    drive_start();
    HAL_Delay(400);
    drive_stop();

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 設定距離だけ前進
    driveA(DIST_SET_POSITION * 0.5, 0, 200, 0);
    driveA(DIST_SET_POSITION * 0.5, 200, 0, 0);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// match_position
// 壁との距離を基準値に合わせる
// 引数1: センサ値の基準値
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void match_position(uint16_t target_value) {
    (void)target_value; // 未使用（パラメータは params.h の定数を使用）

    // 前壁が見えていなければ何もしない
    /*
    if (ad_fr < F_ALIGN_DETECT_THR || ad_fl < F_ALIGN_DETECT_THR) {
        printf("match_position: front wall not detected (FR=%u, FL=%u)\r\n",
               (unsigned)ad_fr, (unsigned)ad_fl);
        buzzer_beep(2500);
        return;
    }
    */

    // 側壁の自動壁制御は無効化（前壁のみで位置・角度を合わせる）
    uint8_t ctrl_prev = MF.FLAG.CTRL;
    float kp_wall_prev = kp_wall;
    MF.FLAG.CTRL = 0;
    kp_wall = 0.0f;

    // 走行制御の状態を初期化して開始
    drive_variable_reset();
    acceleration_interrupt = 0;
    alpha_interrupt = 0;
    // 以降、target_distance は手動更新、omega_interrupt は直接指示
    // （calculate_translation/calculate_rotation は 0 加速度なので増分は本関数で与える）
    velocity_interrupt = 0;
    omega_interrupt = 0;
    drive_start();

    int stable_count = 0;
    uint32_t count = 0;

    while (count < 2000 && ad_fr > WALL_BASE_FR * 1.5 && ad_fl > WALL_BASE_FL * 1.5) {

        // センサ生値[ADcount]ベースの誤差算出（+は目標より遠い/右が遠い）
        int e_fr_cnt = (int)ad_fr - (int)F_ALIGN_TARGET_FR; // [ADcount]
        int e_fl_cnt = (int)ad_fl - (int)F_ALIGN_TARGET_FL; // [ADcount]
        float e_pos = 0.5f * (float)(e_fr_cnt + e_fl_cnt);  // 並進：平均（正: 遠い→前進）
        float e_ang = (float)(e_fr_cnt - e_fl_cnt);         // 角度：差分（正: 右が遠い）

        // 収束判定（両センサが目標±MATCH_POS_TOL 内に連続して入ったら終了）
        if ((float)abs(e_fr_cnt) <= (float)MATCH_POS_TOL && (float)abs(e_fl_cnt) <= (float)MATCH_POS_TOL) {
            stable_count++;
        } else {
            stable_count = 0;
        }
        if (stable_count >= MATCH_POS_STABLE_COUNT) {
            printf("match_position: converged (stable_count=%d)\r\n", stable_count);
            break;
        }

        // 並進は速度FBへ直接指示（velocity_interrupt を更新）
        // 距離が遠い(+e_pos)ときは前進(+)させる
        float v_cmd = MATCH_POS_KP_TRANS * e_pos; // [mm/s] / [ADcount]
        if (v_cmd >  MATCH_POS_VEL_MAX) v_cmd =  MATCH_POS_VEL_MAX;
        if (v_cmd < -MATCH_POS_VEL_MAX) v_cmd = -MATCH_POS_VEL_MAX;
        velocity_interrupt = v_cmd;

        // 角度は omega_interrupt を直接与えて omega_PID を活用
        float w_cmd = MATCH_POS_KP_ROT * e_ang; // [deg/s] / [ADcount]
        if (w_cmd >  MATCH_POS_OMEGA_MAX) w_cmd =  MATCH_POS_OMEGA_MAX;
        if (w_cmd < -MATCH_POS_OMEGA_MAX) w_cmd = -MATCH_POS_OMEGA_MAX;
        omega_interrupt = w_cmd;

        HAL_Delay(1); // 1ms周期で更新（ISRは1kHz）
        count++;
    }

    // 停止（収束 or 安全離脱）
    omega_interrupt = 0;
    velocity_interrupt = 0;
    drive_variable_reset();

    // 復帰
    MF.FLAG.CTRL = ctrl_prev;
    kp_wall = kp_wall_prev;
}

/*==========================================================
    走行系 基幹関数
==========================================================*/

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveA
// 指定パルス分加速走行する
// 引数1：dist …… 走行する距離[mm]
// 引数2: spd_in …… 初速度[mm/sec]
// 引数3: spd_out …… 到達速度[mm/sec]
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveA(float dist, float spd_in, float spd_out, float dist_wallend) {

    // printf("driveA: %.2f, %.2f, %.2f, dwe=%.2f\n", dist, spd_in, spd_out, dist_wallend);

    // 目標終端距離・速度（動的に更新する可能性あり）
    float dist_end = dist;
    float v_target = spd_out;

    velocity_interrupt = spd_in;
    target_velocity = spd_in;
    velocity_profile_target = v_target;
    velocity_profile_clamp_enabled = 1;

    // 探索時のみ: 壁切れ追従をアーム（detect_wall_end のゲート条件に合わせ、WALL_END=1, SCND を一時的にON）
    const bool arm_wallend_init = (dist_wallend > 0.0f);
    bool arm_wallend = arm_wallend_init;
    uint8_t prev_scnd = MF.FLAG.SCND;
    uint8_t prev_wallend = MF.FLAG.WALL_END;
    if (arm_wallend) {
        MF.FLAG.WALL_END = 1;
        MF.FLAG.R_WALL_END = 0;
        MF.FLAG.L_WALL_END = 0;
        if (!MF.FLAG.SCND) {
            // 探索中に限り一時的にSCND=1を立て、検知ゲートを通す
            MF.FLAG.SCND = 1;
        }
    }

    // 加速度を設定（以後、壁切れで終端距離を動的変更した場合は都度更新）
    acceleration_interrupt = (v_target * v_target - spd_in * spd_in) / (2 * dist_end);

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;

    failsafe_turn_angle_begin(0.0f);

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    drive_start();

    // 実際の距離が目標距離になるまで走行（途中で壁切れを検知したら、終端距離・加速度を滑らかに張り替える）
    if (acceleration_interrupt > 0) {

        while (real_distance < dist_end && !MF.FLAG.FAILED) {
            // 動的壁切れ追従
            if (arm_wallend && (MF.FLAG.R_WALL_END || MF.FLAG.L_WALL_END)) {
                // 消費
                MF.FLAG.R_WALL_END = 0;
                MF.FLAG.L_WALL_END = 0;

                // 追従距離: 半区画 + dist_wallend（per-case）。クリップ無し。
                float follow_mm = (float)DIST_HALF_SEC + dist_wallend;
                if (follow_mm < 0.0f) follow_mm = 0.0f;

                dist_end = real_distance + follow_mm;

                // 現在の目標速度から v_target まで等加速度で接続
                float v_now = velocity_interrupt;
                float remain = dist_end - real_distance;
                if (remain > 1e-3f) {
                    acceleration_interrupt = (v_target * v_target - v_now * v_now) / (2.0f * remain);
                } else {
                    acceleration_interrupt = 0.0f;
                }

                // 1回検知で十分。以降は解除
                arm_wallend = false;
                MF.FLAG.WALL_END = prev_wallend; // 元に戻す（通常0）
                MF.FLAG.SCND = prev_scnd;        // SCND復帰
            }

            background_replan_tick();
        }

    } else { // 等速または減速

        if (MF.FLAG.F_WALL_STOP) {
            while (real_distance < dist_end && velocity_interrupt > 0 &&
                   (ad_fl + ad_fr) < thr_f_wall && !MF.FLAG.FAILED) {
                // 動的壁切れ追従
                if (arm_wallend && (MF.FLAG.R_WALL_END || MF.FLAG.L_WALL_END)) {
                    MF.FLAG.R_WALL_END = 0;
                    MF.FLAG.L_WALL_END = 0;

                    float follow_mm = (float)DIST_HALF_SEC + dist_wallend;
                    if (follow_mm < 0.0f) follow_mm = 0.0f;

                    dist_end = real_distance + follow_mm;

                    float v_now = velocity_interrupt;
                    float remain = dist_end - real_distance;
                    if (remain > 1e-3f) {
                        acceleration_interrupt = (v_target * v_target - v_now * v_now) / (2.0f * remain);
                    } else {
                        acceleration_interrupt = 0.0f;
                    }

                    arm_wallend = false;
                    MF.FLAG.WALL_END = prev_wallend;
                    MF.FLAG.SCND = prev_scnd;
                }

                background_replan_tick();
            }
        } else {
            while (real_distance < dist_end && velocity_interrupt > 0 && !MF.FLAG.FAILED) {
                if (arm_wallend && (MF.FLAG.R_WALL_END || MF.FLAG.L_WALL_END)) {
                    MF.FLAG.R_WALL_END = 0;
                    MF.FLAG.L_WALL_END = 0;

                    float follow_mm = (float)DIST_HALF_SEC + dist_wallend;
                    if (follow_mm < 0.0f) follow_mm = 0.0f;

                    dist_end = real_distance + follow_mm;

                    float v_now = velocity_interrupt;
                    float remain = dist_end - real_distance;
                    if (remain > 1e-3f) {
                        acceleration_interrupt = (v_target * v_target - v_now * v_now) / (2.0f * remain);
                    } else {
                        acceleration_interrupt = 0.0f;
                    }

                    arm_wallend = false;
                    MF.FLAG.WALL_END = prev_wallend;
                    MF.FLAG.SCND = prev_scnd;
                }
                background_replan_tick();
            }
        }
    }

    // 終了時にフラグを復帰（未検知でarm_wallendが残っている場合）
    if (arm_wallend_init) {
        MF.FLAG.WALL_END = prev_wallend;
        MF.FLAG.SCND = prev_scnd;
        MF.FLAG.R_WALL_END = 0;
        MF.FLAG.L_WALL_END = 0;
    }

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveR
// 指定パルス分等速走行して停止する
// 引数1：dist …… 走行する距離[mm]
// 引数2：spd …… 速度[mm/s]
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveR(float angle) {

    // 角加速度を設定
    if (angle >= 0) {
        alpha_interrupt = ALPHA_ROTATE_90;
    } else {
        alpha_interrupt = -ALPHA_ROTATE_90;
    }

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
    velocity_interrupt = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;

    failsafe_turn_angle_begin(angle);

    drive_start();

    // 実際の角度が目標角度（30°）になるまで角加速走行
    if (angle >= 0) {
        while (real_angle > -angle * 0.333 && !MF.FLAG.FAILED) {
            background_replan_tick();
        }
    } else {
        while (real_angle < -angle * 0.333 && !MF.FLAG.FAILED) {
            background_replan_tick();
        }
    }

    // 実際の角度が目標角度（30°）になるまで等角速度走行
    alpha_interrupt = 0;
    if (angle >= 0) {
        while (real_angle > -angle * 0.666 && !MF.FLAG.FAILED) {
            background_replan_tick();
        }
    } else {
        while (real_angle < -angle * 0.666 && !MF.FLAG.FAILED) {
            background_replan_tick();
        }
    }

    // 実際の角度が目標角度（30°）になるまで角減速走行
    if (angle >= 0) {
        alpha_interrupt = -ALPHA_ROTATE_90;
    } else {
        alpha_interrupt = +ALPHA_ROTATE_90;
    };
    if (angle >= 0) {
        while (real_angle > -angle && !MF.FLAG.FAILED) {
            background_replan_tick();
        }
    } else {
        while (real_angle < -angle && !MF.FLAG.FAILED) {
            background_replan_tick();
        }
    }

    alpha_interrupt = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;

    // drive_stop();

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveC
// 指定パルス分デフォルトインターバルで走行して停止する
// 引数1：dist …… 走行するパルス
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveC(int dist) {
    (void)dist;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveM
// センサ値が基準値になるまで前進or後進する
// 引数1: センサ値の基準値
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveM(uint16_t sens_tgt) {
    (void)sens_tgt;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveSR
// 指定速度でスラロームで右旋回して停止する
// 引数1：spd_turn
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveSR(float angle_turn, float alpha_turn) {

    // printf("%f\n", alpha_interrupt);

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;

    failsafe_turn_angle_begin(angle_turn);

    drive_start();

    // 角加速度と並進加速度を設定
    alpha_interrupt = alpha_turn;
    acceleration_interrupt = -acceleration_turn;

    // 実際の角度が目標角度（30°）になるまで角加速走行
    while (real_angle > -angle_turn * 0.333 && !MF.FLAG.FAILED) {
        background_replan_tick();
    }

    // 実際の角度が目標角度（30°）になるまで等角速度走行
    alpha_interrupt = 0;
    acceleration_interrupt = 0;

    while (real_angle > -angle_turn * 0.666 && !MF.FLAG.FAILED) {
        background_replan_tick();
    }

    // 実際の角度が目標角度（30°）になるまで角減速走行
    alpha_interrupt = -alpha_turn;
    acceleration_interrupt = acceleration_turn;

    while (real_angle > -angle_turn && !MF.FLAG.FAILED) {
        background_replan_tick();
    }

    alpha_interrupt = 0;
    velocity_interrupt = speed_now;

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveSL
// 指定速度でスラロームで左旋回して停止する
// 引数1：spd_turn
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void driveSL(float angle_turn, float alpha_turn) {

    // printf("%f\n", alpha_interrupt);
    // printf("%f\n", alpha_interrupt);

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;

    failsafe_turn_angle_begin(angle_turn);

    // 角加速度と並進加速度を設定
    alpha_interrupt = -alpha_turn;
    acceleration_interrupt = -acceleration_turn;

    drive_start();

    // 実際の角度が目標角度（30°）になるまで角加速走行
    while (real_angle < angle_turn * 0.333 && !MF.FLAG.FAILED) {
        background_replan_tick();
    }

    // 実際の角度が目標角度（30°）になるまで等角速度走行
    alpha_interrupt = 0;
    acceleration_interrupt = 0;
    while (real_angle < angle_turn * 0.666 && !MF.FLAG.FAILED) {
        background_replan_tick();
    }

    // 実際の角度が目標角度（30°）になるまで角減速走行
    alpha_interrupt = alpha_turn;
    acceleration_interrupt = acceleration_turn;

    while (real_angle < angle_turn && !MF.FLAG.FAILED) {
        background_replan_tick();
    }

    alpha_interrupt = 0;
    velocity_interrupt = speed_now;

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
}

void driveFWall(float dist, float spd_in, float spd_out) {

    // printf("driveA: %.2f, %.2f, %.2f\n", dist, spd_in, spd_out);

    // 加速度を設定
    acceleration_interrupt = (spd_out * spd_out - spd_in * spd_in) / (2 * dist);

    // printf("acceleration_interrupt: %.2f\n", acceleration_interrupt);

    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;

    failsafe_turn_angle_begin(0.0f);

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    drive_start();

    // 実際の距離が目標距離になるか前壁しきい値に達した時点で抜ける。
    // 未検知の場合は、WALL_END_EXTEND_MAX_MM の範囲で距離を延長して検出を待つ。
    bool reached = false;
    if (MF.FLAG.SLALOM_R) {
        while (real_distance < dist) {
            background_replan_tick();
            if (MF.FLAG.F_WALL && (ad_fr + ad_fl) >= val_offset_in) {
                reached = true;
                break;
            }
        }
    } else if (MF.FLAG.SLALOM_L) {
        while (real_distance < dist) {
            background_replan_tick();
            if (MF.FLAG.F_WALL && (ad_fr + ad_fl) >= val_offset_in) {
                reached = true;
                break;
            }
        }
    }

    // 前壁しきい値に達していなければ延長して待つ（上限: WALL_END_EXTEND_MAX_MM）
    if (!reached) {
        const float extend_limit_mm = WALL_END_EXTEND_MAX_MM;
        if (extend_limit_mm > 0.0f) {
            const float dist_end = dist + extend_limit_mm;
            // 延長区間は等速で追従
            acceleration_interrupt = 0;
            velocity_interrupt = spd_out;
            while (real_distance < dist_end) {
                background_replan_tick();
                if (MF.FLAG.F_WALL && (ad_fr + ad_fl) >= val_offset_in) {
                    reached = true;
                    break;
                }
                if (MF.FLAG.FAILED) {
                    break;
                }
            }
        }
    }

    velocity_interrupt = spd_out;

    // 割込み内の変数をリセット
    drive_variable_reset();

    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
}

/*==========================================================
    初期化関数・設定関数・その他関数
==========================================================*/
//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_init
// 走行系の変数の初期化，モータードライバ関係のGPIO設定とPWM出力に使うタイマの設定をする
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_init(void) {
    // エンコーダの読み取り開始
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    wall_end_count = 0;

    //====走行系の変数の初期化====

    //====マウスフラグの初期化===
    MF.FLAGS = 0; // フラグクリア
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_variable_reset
// 割込み内の走行系の変数の初期化
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_variable_reset(void) {

    // 速度
    acceleration_interrupt = 0;
    target_distance = 0;
    velocity_profile_target = 0;
    velocity_profile_clamp_enabled = 0;

    // 角度
    alpha_interrupt = 0;
    omega_interrupt = 0;
    target_angle = 0;

    // PIDの積算項（距離）
    distance_error = 0;
    previous_distance_error = 0;
    distance_error_error = 0;
    distance_integral = 0;

    // PIDの積算項（速度）
    velocity_error = 0;
    previous_velocity_error = 0;
    velocity_error_error = 0;
    velocity_integral = 0;

    // PIDの積算項（角度）
    angle_error = 0;
    previous_angle_error = 0;
    angle_error_error = 0;
    angle_integral = 0;

    // PIDの積算項（角速度）
    target_omega = 0;
    omega_error = 0;
    previous_omega_error = 0;
    omega_error_error = 0;
    omega_integral = 0;
}

void drive_reset_before_run(void) {
    drive_variable_reset();

    s_fail_turn_angle_enabled = 0;
    s_fail_turn_angle_start_deg = 0.0f;
    s_fail_turn_angle_limit_deg = 0.0f;
    s_fail_turn_angle_count = 0;
    s_fail_turn_angle_armed_for_start = 0;

    MF.FLAG.OVERRIDE = 0;
    MF.FLAG.FAILED = 0;
    MF.FLAG.CTRL = 0;
    MF.FLAG.SLALOM_R = 0;
    MF.FLAG.SLALOM_L = 0;
    MF.FLAG.F_WALL = 0;

    speed_now = 0;
    omega_now = 0;

    acceleration_interrupt = 0;
    velocity_interrupt = 0;
    velocity_profile_target = 0;
    velocity_profile_clamp_enabled = 0;
    target_distance = 0;

    alpha_interrupt = 0;
    omega_interrupt = 0;
    target_angle = 0;

    target_velocity = 0;
    target_omega = 0;

    out_r = 0;
    out_l = 0;
    out_translation = 0;
    out_rotate = 0;

    wall_control = 0;
    diagonal_control = 0;
    latest_wall_error = 0;

    previous_ad_r = ad_r;
    previous_ad_l = ad_l;

    real_distance = 0;
    real_velocity = 0;
    encoder_speed_r = 0;
    encoder_speed_l = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;

    TIM8->CNT = 30000;
    TIM4->CNT = 30000;
    encoder_count_r = 30000;
    encoder_count_l = 30000;
    previous_encoder_count_r = 30000;
    previous_encoder_count_l = 30000;

    IMU_angle = 0;
    real_angle = 0;
    real_omega = 0;
    IMU_acceleration = 0;

    wall_end_reset();
    MF.FLAG.WALL_END = 0;
    MF.FLAG.R_WALL_END = 0;
    MF.FLAG.L_WALL_END = 0;
    wall_end_count = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_enable_motor
// モータドライバの電源ON
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_enable_motor(void) {
    // すでに有効なら何もしない（冪等）
    if (s_motor_enabled) {
        s_outputs_locked = 0;
        return;
    }

    // 現在のDIRピンレベルを取得
    GPIO_PinState dir_l = HAL_GPIO_ReadPin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin);
    GPIO_PinState dir_r = HAL_GPIO_ReadPin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin);
    s_dir_pin_high_l = (dir_l == GPIO_PIN_SET) ? 1 : 0;
    s_dir_pin_high_r = (dir_r == GPIO_PIN_SET) ? 1 : 0;

    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);

    // 1) CCR更新をロックし、まず IN1==IN2 となるアイドル（短絡ブレーキ相当）を作る
    s_outputs_locked = 1;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, s_dir_pin_high_l ? arr : 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, s_dir_pin_high_r ? arr : 0u);
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    // 2) PWM を開始（安全CCRで IN1==IN2 を提示）
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    tim1_wait_us(50);

    // 3) STBY を有効化（出力段を有効化）
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
    tim1_wait_us(100);

    // 4) 有効化完了
    s_motor_enabled = 1;
    s_outputs_locked = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_disable_motor
// モータードライバの電源OFF
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_disable_motor(void) {
    // すでに無効なら何もしない（冪等）
    if (!s_motor_enabled) {
        HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
        return;
    }

    // 停止時は IN1==IN2 となるアイドルを明示してから停止
    s_outputs_locked = 1;
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, s_dir_pin_high_l ? arr : 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, s_dir_pin_high_r ? arr : 0u);

    // PWM 停止 → STBY 無効化
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);

    s_motor_enabled = 0;
    s_outputs_locked = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_brake
// 短絡ブレーキのON/OFF（STBY/PWM状態は変更しない）
// 引数：enable …… trueでブレーキ、falseで解除
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_brake(bool enable) {
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);

    if (enable) {
        // DIRピンから現在レベルを取得して IN1==IN2 を生成
        GPIO_PinState dir_l = HAL_GPIO_ReadPin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin);
        GPIO_PinState dir_r = HAL_GPIO_ReadPin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin);
        s_dir_pin_high_l = (dir_l == GPIO_PIN_SET) ? 1 : 0;
        s_dir_pin_high_r = (dir_r == GPIO_PIN_SET) ? 1 : 0;

        s_outputs_locked = 1;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, s_dir_pin_high_l ? arr : 0u);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, s_dir_pin_high_r ? arr : 0u);
    } else {
        // 通常更新に戻す
        s_outputs_locked = 0;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_start
// 走行を開始する
// （pulse_l,pulse_rを0にリセットしてタイマを有効にする）
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_start(void) {
    // 統合：STBY と PWM を安全に同時運用へ
    if (!s_fail_turn_angle_armed_for_start) {
        failsafe_turn_angle_begin(0.0f);
    }
    s_fail_turn_angle_armed_for_start = 0;
    drive_enable_motor();
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_stop
// 走行を終了する
// （タイマを止めてタイマカウント値を0にリセットする）
// 引数1：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_stop(void) {
    // 互換：従来の drive_stop は「ブレーキ」に読み替え
    drive_brake(true);
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_set_dir
// 進行方向を設定する
// 引数1：d_dir …… どの方向に進行するか  0桁目で左，1桁目で右の方向設定
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_set_dir(uint8_t d_dir) {
    // printf("DIR\n");

    //====左モータ====
    switch (d_dir & 0x0f) { // 0~3ビット目を取り出す
    //----正回転----
    case 0x00: // 0x00の場合
        HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, DIR_FWD_L);
        s_dir_pin_high_l = (DIR_FWD_L == GPIO_PIN_SET) ? 1 : 0;
        // 左を前進方向に設定
        break;
    //----逆回転----
    case 0x01: // 0x01の場合
        HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, DIR_BACK_L);
        s_dir_pin_high_l = (DIR_BACK_L == GPIO_PIN_SET) ? 1 : 0;
        // 左を後進方向に設定
        break;
    }
    //====右モータ====
    switch (d_dir & 0xf0) { // 4~7ビット目を取り出す
    case 0x00:              // 0x00の場合
        HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, DIR_FWD_R);
        s_dir_pin_high_r = (DIR_FWD_R == GPIO_PIN_SET) ? 1 : 0;
        // 右を前進方向に設定
        break;
    //----逆回転----
    case 0x10: // 0x01の場合
        HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, DIR_BACK_R);
        s_dir_pin_high_r = (DIR_BACK_R == GPIO_PIN_SET) ? 1 : 0;
        // 右を後進方向に設定
        break;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_motor
// モータを回す
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_motor(void) {
    // 有効化/停止シーケンス中はCCR更新を抑止し、IN1==IN2のアイドルを維持
    if (s_outputs_locked) {
        const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, s_dir_pin_high_l ? arr : 0u);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, s_dir_pin_high_r ? arr : 0u);
        return;
    }

    // 並進と回転の出力を合算
    out_r = out_translation - out_rotate;
    out_l = out_translation + out_rotate;

    // 回転のフェイルセーフ
    if (((out_r - out_l > FAIL_LR_ERROR || out_r - out_l < -FAIL_LR_ERROR)) &&
        MF.FLAG.RUNNING) {
        fail_count_lr++;
    } else {
        fail_count_lr = 0;
    }

    if (fail_count_lr > FAIL_COUNT_LR) {
        MF.FLAG.FAILED = 1;
        fail_count_lr = 0;
    }

    // 並進衝突のフェイルセーフ
    if ((IMU_acceleration < -FAIL_ACC || IMU_acceleration > FAIL_ACC) &&
        MF.FLAG.RUNNING) {
        fail_count_acc++;
    } else {
        fail_count_acc = 0;
    }

    if (fail_count_acc > FAIL_COUNT_ACC && MF.FLAG.RUNNING) {
        MF.FLAG.FAILED = 1;
        fail_count_acc = 0;
    }

    if (s_fail_turn_angle_enabled && MF.FLAG.RUNNING && !MF.FLAG.OVERRIDE) {
        float delta = fabsf(IMU_angle - s_fail_turn_angle_start_deg);
        if (delta > s_fail_turn_angle_limit_deg) {
            s_fail_turn_angle_count++;
        } else {
            s_fail_turn_angle_count = 0;
        }

        if (s_fail_turn_angle_count > FAIL_TURN_ANGLE_COUNT) {
            MF.FLAG.FAILED = 1;
            s_fail_turn_angle_count = 0;
        }
    } else {
        s_fail_turn_angle_count = 0;
    }

    // 左右モータの回転方向の指定（sign-magnitude）
    // DIR ピンで方向を出し、PWM は常に High アクティブのまま使用する
    if (out_r >= 0 && out_l >= 0) {
        drive_set_dir(0x00);
    } else if (out_r >= 0 && out_l < 0) {
        drive_set_dir(0x01);
        out_l = -out_l; // duty 計算用に絶対値へ
    } else if (out_r < 0 && out_l >= 0) {
        drive_set_dir(0x10);
        out_r = -out_r;
    } else if (out_r < 0 && out_l < 0) {
        drive_set_dir(0x11);
        out_r = -out_r;
        out_l = -out_l;
    }

    // PWM出力
    if (MF.FLAG.FAILED) {
        // フェイルセーフ発動の場合，強制停止（PWM=0 ＋ STBY=Low）
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
        drive_disable_motor();
        drive_fan(0);

        buzzer_beep(1200);
        led_write(0,0,0);

        while(1){
            led_write(1,1,1);
            HAL_Delay(500);
            led_write(0,0,0);
            HAL_Delay(500);
        }

    } else {
        // TIM2 の ARR を取得（0..ARR のカウント幅）
        const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
        const uint16_t min_counts = (uint16_t)((arr * MOTOR_MIN_DUTY_PCT) / 100u);
        const uint16_t boost_counts = (uint16_t)((arr * MOTOR_BOOST_DUTY_PCT) / 100u);

        // コマンド（0..ARR）へクリップ
        uint16_t cmd_l = (uint16_t)fminf(fabsf(out_l), (float)arr);
        uint16_t cmd_r = (uint16_t)fminf(fabsf(out_r), (float)arr);

        // 最小デューティ補償（非ゼロの場合のみ）
        if (cmd_l > 0 && cmd_l < min_counts) cmd_l = min_counts;
        if (cmd_r > 0 && cmd_r < min_counts) cmd_r = min_counts;

        // 起動ブースト（停止 -> 非ゼロの立上りで一定時間ブースト）
        const uint32_t now = HAL_GetTick();
        if (s_prev_cmd_l == 0 && cmd_l > 0) {
            s_boost_until_ms_l = now + MOTOR_BOOST_TIME_MS;
        }
        if (s_prev_cmd_r == 0 && cmd_r > 0) {
            s_boost_until_ms_r = now + MOTOR_BOOST_TIME_MS;
        }
        if (cmd_l > 0 && now < s_boost_until_ms_l && boost_counts > cmd_l) {
            cmd_l = boost_counts;
        }
        if (cmd_r > 0 && now < s_boost_until_ms_r && boost_counts > cmd_r) {
            cmd_r = boost_counts;
        }

        // デューティ適用（DIRが反転対象レベルのとき PWM反転）
        uint32_t ccr_l;
        uint32_t ccr_r;
        const uint32_t arr_apply = arr;
        uint8_t invert_active_l = ((s_dir_pin_high_l ? 1 : 0) == PWM_INVERT_DIR_LEVEL);
        uint8_t invert_active_r = ((s_dir_pin_high_r ? 1 : 0) == PWM_INVERT_DIR_LEVEL);
        if (cmd_l == 0) {
            ccr_l = s_dir_pin_high_l ? arr_apply : 0u; // アイドルはIN1==IN2
        } else if (invert_active_l) {
            ccr_l = arr_apply - cmd_l;
        } else {
            ccr_l = cmd_l;
        }

        if (cmd_r == 0) {
            ccr_r = s_dir_pin_high_r ? arr_apply : 0u; // アイドルはIN1==IN2
        } else if (invert_active_r) {
            ccr_r = arr_apply - cmd_r;
        } else {
            ccr_r = cmd_r;
        }

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_l);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ccr_r);

        // 次回用の履歴
        s_prev_cmd_l = cmd_l;
        s_prev_cmd_r = cmd_r;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// drive_fan
// ファンを回す
// 引数：fan_power (0~1000)
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void drive_fan(uint16_t fan_power) {

    const uint8_t was_running = s_fan_running;

    if (fan_power > 0) {
        MF.FLAG.SUCTION = 1;
    } else {
        MF.FLAG.SUCTION = 0;
    }

    if (fan_power) {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

        if (!was_running) {
            for (uint16_t i = 0; i < fan_power; i++) {
                __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);
                HAL_Delay(1);
            }

            HAL_Delay(SUCTION_FAN_STABILIZE_DELAY_MS);
        } else {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, fan_power);
        }

        s_fan_running = 1;
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
        // 停止時刻を記録（同一TIM3を使うブザーとの干渉を避けるためのクールダウン判定用）
        fan_last_off_ms = HAL_GetTick();

        s_fan_running = 0;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// test_run
// テスト走行モード
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void test_run(void) {
    int mode = 0;
    // drive_enable_motor();

    led_flash(8);

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0:
            //----尻当て----
            printf("Mode 4-0 Set Position.\n");

            // 直線
            acceleration_straight = 2777.778;
            acceleration_straight_dash = 3000; // 5000
            velocity_straight = 500;
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 12800;
            acceleration_turn = 0;
            dist_offset_in = 14;
            dist_offset_out = 18; // 32
            val_offset_in = 2000;
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 500;
            alpha_l_turn_90 = 4100;
            angle_l_turn_90 = 89.0;
            dist_l_turn_out_90 = 10;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            set_position();

            led_flash(5);
            drive_stop();

            break;
        case 1:
            // ログ確認用 
            printf("Mode 6-1 Get Log.\n");

            // 直線
            acceleration_straight = 2777.778;
            acceleration_straight_dash = 3000; // 5000
            velocity_straight = 500;
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 12800;
            acceleration_turn = 0;
            dist_offset_in = 14;
            dist_offset_out = 18; // 32
            val_offset_in = 2000;
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 500;
            alpha_l_turn_90 = 4100;
            angle_l_turn_90 = 89.0;
            dist_l_turn_out_90 = 10;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            drive_variable_reset();
            IMU_GetOffset();

            MF.FLAG.CTRL = 1;

            // ロギング開始 - 新しいAPIを使用
            log_init(); // 必要に応じて初期化
            log_set_profile(LOG_PROFILE_VELOCITY);
            // log_set_profile(LOG_PROFILE_OMEGA);
            log_start(HAL_GetTick());
            
            half_sectionA(0);
            one_sectionU(1.0f, speed_now);
            half_sectionD(0);
            log_stop();

            drive_stop();

            break;
        case 2:
            // ログ確認用
            printf("Mode 6-2 Get Log.\n");

            // 直線
            acceleration_straight = 2777.778;
            acceleration_straight_dash = 3000; // 5000
            velocity_straight = 500;
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 12800;
            acceleration_turn = 0;
            dist_offset_in = 14;
            dist_offset_out = 18; // 32
            val_offset_in = 2000;
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 500;
            alpha_l_turn_90 = 4100;
            angle_l_turn_90 = 89.0;
            dist_l_turn_out_90 = 10;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            drive_variable_reset();
            IMU_GetOffset();

            MF.FLAG.CTRL = 1;

            // ロギング開始 - 新しいAPIを使用
            log_init(); // 必要に応じて初期化
            log_set_profile(LOG_PROFILE_DISTANCE);
            // log_set_profile(LOG_PROFILE_OMEGA);
            log_start(HAL_GetTick());
            
            half_sectionA(0);
            one_sectionU(1.0f, speed_now);
            one_sectionU(1.0f, speed_now);
            one_sectionU(1.0f, speed_now);
            half_sectionD(0);
            log_stop();

            // drive_stop();

            break;
        case 3:
            // ログ確認用
            printf("Mode 6-2 Get Log.\n");

            // 直線
            acceleration_straight = 2777.778;
            acceleration_straight_dash = 3000; // 5000
            velocity_straight = 500;
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 12800;
            acceleration_turn = 0;
            dist_offset_in = 14;
            dist_offset_out = 18; // 32
            val_offset_in = 2000;
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 500;
            alpha_l_turn_90 = 4100;
            angle_l_turn_90 = 89.0;
            dist_l_turn_out_90 = 10;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            drive_variable_reset();
            IMU_GetOffset();

            MF.FLAG.CTRL = 1;

            // ロギング開始 - 新しいAPIを使用
            log_init(); // 必要に応じて初期化
            log_set_profile(LOG_PROFILE_OMEGA);
            log_start(HAL_GetTick());
            
            half_sectionA(0);

            turn_R90(0);
            // rotate_180();

            half_sectionD(0);
            log_stop();

            drive_stop();

            break;
        case 4:
            //----直進----
            printf("Mode 4-4 straight 2 Sections.\n");

            // 直線
            acceleration_straight = 5444.44;
            acceleration_straight_dash = 8000; // 5000
            velocity_straight = 700;
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 43900;
            acceleration_turn = 0;
            dist_offset_in = 5;
            dist_offset_out = 8.47; // 32
            val_offset_in = 2000;
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 500;
            alpha_l_turn_90 = 4100;
            angle_l_turn_90 = 89.5;
            dist_l_turn_out_90 = 10;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            drive_variable_reset();
            IMU_GetOffset();

            MF.FLAG.CTRL = 1;

            led_flash(3);
            drive_fan(1000);
            led_flash(3);

            // ロギング開始 - 新しいAPIを使用
            log_init(); // 必要に応じて初期化
            log_set_profile(LOG_PROFILE_OMEGA);
            log_start(HAL_GetTick());
            
            half_sectionA(700);

            turn_R90(0);

            half_sectionD(0);
            log_stop();

            drive_stop();

            led_flash(3);
            drive_fan(0);
            led_flash(3);

            break;

        case 5:
            //----右旋回----
            printf("Mode 4-5 Turn R90.\n");

            // 直線
            acceleration_straight = 11111.11;
            acceleration_straight_dash = 10000; // 5000
            velocity_straight = 1000;
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 43900;
            acceleration_turn = 0;
            dist_offset_in = 5;
            dist_offset_out = 8.47; // 32
            val_offset_in = 2000;
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 1000;
            alpha_l_turn_90 = 20400;
            angle_l_turn_90 = 89.5;
            dist_l_turn_out_90 = 8.25;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            drive_variable_reset();
            IMU_GetOffset();

            MF.FLAG.CTRL = 1;

            led_flash(3);
            drive_fan(1000);
            led_flash(3);

            // ロギング開始 - 新しいAPIを使用
            log_init(); // 必要に応じて初期化
            log_set_profile(LOG_PROFILE_OMEGA);
            log_start(HAL_GetTick());
            
            half_sectionA(0);

            l_turn_R90(false);

            half_sectionD(0);
            log_stop();

            drive_stop();

            led_flash(3);
            drive_fan(0);
            led_flash(3);

            break;
        case 6:
            //--------
            printf("Mode 4-6 Rotate R90.\n");

            // 直線
            acceleration_straight = 2777.778;
            acceleration_straight_dash = 3000; // 5000
            velocity_straight = 500;
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 12800;
            acceleration_turn = 0;
            dist_offset_in = 14;
            dist_offset_out = 18; // 32
            val_offset_in = 2000;
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 500;
            alpha_l_turn_90 = 4100;
            angle_l_turn_90 = 89.0;
            dist_l_turn_out_90 = 10;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            get_base();

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            first_sectionA();
            for (uint8_t i = 0; i < 7; i++) {
                half_sectionU();
            }
            turn_R90(1);
            half_sectionD(0);

            led_flash(5);
            drive_stop();

            break;
        case 7:

            printf("Mode 4-7.\n");

            // 直線
            acceleration_straight = 2777.778;
            acceleration_straight_dash = 3000; // 5000
            velocity_straight = 500;
            // ターン
            velocity_turn90 = 300;
            alpha_turn90 = 12800;
            acceleration_turn = 0;
            dist_offset_in = 14;
            dist_offset_out = 18; // 32
            val_offset_in = 2000;
            angle_turn_90 = 89.5;
            // 90°大回りターン
            velocity_l_turn_90 = 500;
            alpha_l_turn_90 = 4100;
            angle_l_turn_90 = 89.0;
            dist_l_turn_out_90 = 10;
            // 180°大回りターン
            velocity_l_turn_180 = 450;
            alpha_l_turn_180 = 3600;
            angle_l_turn_180 = 180;
            dist_l_turn_out_180 = 17;
            // 壁切れ後の距離
            dist_wall_end = 0;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            half_sectionA(velocity_l_turn_90);
            l_turn_R90(false);
            half_sectionD(0);

            led_flash(5);
            drive_stop();

            break;

        case 8:

            printf("Mode 4-8.\n");

            // 前壁センサを使った非接触中央合わせ
            led_flash(5);
            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();
            match_position(0);
            buzzer_beep(1200);
            drive_stop();

            break;
        
            case 9:

            // 新しいロギングシステムでログを出力
            printf("Mode 4-2 - ロギングデータの出力\n");
            log_print_all();
        }
    }
    drive_disable_motor();
}

void reset_failed(void) {

    drive_stop();
    drive_variable_reset();

    for (int i = 0; i < 5; i++) {
        buzzer_beep(900);
    }

    MF.FLAG.FAILED = 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++
// driveC_wallend
// 等速走行し、壁切れ検出時に即座に終了する
// 引数1：dist_max …… 最大走行距離[mm]
// 引数2: spd …… 走行速度[mm/sec]
// 戻り値：壁切れを検出した場合true、未検出でdist_max走行した場合false
//+++++++++++++++++++++++++++++++++++++++++++++++
bool driveC_wallend(float dist_max, float spd) {
    
    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
    
    // 回転角度カウントをリセット
    real_angle = 0;
    IMU_angle = 0;
    target_angle = 0;
    
    // 等速走行（加速度0）
    acceleration_interrupt = 0.0f;
    velocity_interrupt = spd;
    
    // 壁切れ検出をアーム
    wall_end_reset();
    MF.FLAG.WALL_END = 1;
    
    failsafe_turn_angle_begin(0.0f);
    
    drive_start();
    
    bool wall_end_detected = false;
    
    // 壁切れ検出または最大距離到達まで走行
    while (real_distance < dist_max && !MF.FLAG.FAILED) {
        // 壁切れ検出チェック
        if (wall_end_detected_r || wall_end_detected_l) {
            wall_end_detected = true;
            break;  // 即座に終了
        }
        background_replan_tick();
    }
    
    // 壁切れ検出をディスアーム
    MF.FLAG.WALL_END = 0;
    
    // 割込み内の変数をリセット（速度は維持）
    // drive_variable_reset(); // 速度を維持するためリセットしない
    
    // 走行距離カウントをリセット
    real_distance = 0;
    encoder_distance_r = 0;
    encoder_distance_l = 0;
    
    return wall_end_detected;
}
