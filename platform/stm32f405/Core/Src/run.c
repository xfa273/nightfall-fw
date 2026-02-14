/*
 * run.c
 *
 *  Created on: Mar 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include <math.h>
#include "../Inc/shortest_run_params.h"
#include "../Inc/path.h"
#include "../Inc/solver.h"

void run(void) {

    drive_reset_before_run();
    drive_start();
    get_base();

    first_sectionA();

    // センサログ有効時: 走行開始時からログ取得開始
    if (g_sensor_log_enabled) {
        sensor_log_start();
    }

    for (uint8_t path_count = 0; path[path_count] != 0; path_count++) {
        if (200 < path[path_count] && path[path_count] < 300) {
            // 直進（距離ベース）

            float straight_mm = (path[path_count] - 200) * DIST_HALF_SEC; // [mm]

            // 次動作に応じたターン入口速度を決定（0なら終端）
            uint16_t next_code = path[path_count + 1];
            
            // 壁切れ検出用バッファ距離（直進の最後の部分）
            const float WALL_END_BUFFER = (float)DIST_HALF_SEC;  // 45mm
            
            float v_next = 0.0f; // [mm/s]
            if (next_code >= 300 && next_code < 400) {
                v_next = velocity_turn90;
            } else if (next_code >= 400 && next_code < 500) {
                v_next = velocity_turn90;
            } else if (next_code >= 500 && next_code < 600) {
                uint8_t l_turn_sections = next_code - 500;
                v_next = (l_turn_sections == 2) ? velocity_l_turn_180 : velocity_l_turn_90;
            } else if (next_code >= 600 && next_code < 700) {
                uint8_t l_turn_sections = next_code - 600;
                v_next = (l_turn_sections == 2) ? velocity_l_turn_180 : velocity_l_turn_90;
            } else if (next_code >= 701 && next_code < 800) {
                v_next = velocity_turn45in;
            } else if (next_code >= 801 && next_code < 900) {
                v_next = velocity_turnV90;
            } else if (next_code >= 901 && next_code < 1000) {
                v_next = velocity_turn135in;
            } else {
                v_next = 0.0f; // 次なし（終端）
            }

            // ゴール停止時（次コード=0）は、最後の half_sectionD(0) で
            // 通常の直線減速(acceleration_straight)と同等の減速度になるよう終端速度を設定
            if (next_code == 0) {
                float goal_entry_speed = sqrtf(fmaxf(0.0f, 2.0f * acceleration_straight * (float)DIST_HALF_SEC));
                if (goal_entry_speed > velocity_straight) {
                    goal_entry_speed = velocity_straight;
                }
                v_next = goal_entry_speed;
            }

            // 直線の加減速区画を計算（二段階加速対応）
            // v_start=0 から加速を開始すると仮定
            float v_start = 0.0f;
            float v_switch = accel_switch_velocity;  // 切り替え速度
            float v_max = velocity_straight;         // 目標最高速度
            float accel_low = acceleration_straight;       // 低速域加速度
            float accel_high = acceleration_straight_dash; // 高速域加速度
            
            float d_acc = 0.0f;      // 総加速距離 [mm]
            float d_constant = 0.0f; // 等速距離 [mm]
            float max_reached_speed = v_max;
            
            // 二段階加速が有効かどうか
            bool two_stage = (v_switch > 0.0f && v_switch < v_max && accel_low > 0.0f && accel_high > 0.0f);
            
            if (two_stage) {
                // 二段階加速の計算
                // Phase1: v_start → v_switch (低速域加速度)
                float d1 = (v_switch * v_switch - v_start * v_start) / (2.0f * accel_low);
                // Phase2: v_switch → v_max (高速域加速度)
                float d2 = (v_max * v_max - v_switch * v_switch) / (2.0f * accel_high);
                
                float d_acc_full = d1 + d2;  // 最高速度到達までの総加速距離
                float d_total_acc_dec = 2.0f * d_acc_full;  // 加速+減速距離
                
                if (d_total_acc_dec <= straight_mm) {
                    // 最高速度に到達する場合（台形）
                    d_acc = d_acc_full;
                    d_constant = straight_mm - d_total_acc_dec;
                    max_reached_speed = v_max;
                } else {
                    // 最高速度に到達しない場合
                    // まず、切り替え速度に到達するか確認
                    float d_to_switch_and_back = 2.0f * d1;  // 切り替え速度まで加速+減速
                    
                    if (d_to_switch_and_back <= straight_mm) {
                        // 切り替え速度には到達するが、最高速度には到達しない
                        // 残り距離で高速域加速度を使って到達可能な速度を計算
                        float remain_half = (straight_mm - d_to_switch_and_back) / 2.0f;
                        // v_switch² + 2*accel_high*remain_half = v_reached²
                        float v_reached_sq = v_switch * v_switch + 2.0f * accel_high * remain_half;
                        max_reached_speed = sqrtf(v_reached_sq);
                        d_acc = d1 + remain_half;
                        d_constant = 0.0f;
                    } else {
                        // 切り替え速度にも到達しない（低速域のみ）
                        d_acc = straight_mm / 2.0f;
                        d_constant = 0.0f;
                        max_reached_speed = sqrtf(v_start * v_start + 2.0f * accel_low * d_acc);
                    }
                }
            } else {
                // 従来の1段階加速（accel_switch_velocity無効時）
                float accel = (accel_high > 0.0f) ? accel_high : accel_low;
                float t_acc_time = v_max / accel;
                float d_acc_full = 0.5f * accel * t_acc_time * t_acc_time;
                float d_total_acc_dec = 2.0f * d_acc_full;
                
                if (d_total_acc_dec > straight_mm) {
                    d_acc = straight_mm / 2.0f;
                    d_constant = 0.0f;
                    max_reached_speed = sqrtf(2.0f * accel * d_acc);
                } else {
                    d_acc = d_acc_full;
                    d_constant = straight_mm - d_total_acc_dec;
                }
            }

            // 各距離を区画数に変換
            float d_acc_blocks = d_acc / DIST_HALF_SEC;
            float d_constant_blocks = d_constant / DIST_HALF_SEC;
            float d_dec_blocks = d_acc / DIST_HALF_SEC;  // 減速区間 = 加速区間と同じ

            // 次がターンかどうか、小回りか大回りかを判定
            bool next_is_small_turn = (next_code >= 300 && next_code < 500);  // 小回り90度
            bool next_is_large_turn = (next_code >= 500 && next_code < 700);  // 大回り90/180度
            
            // 前のコードを取得（例外処理用）
            uint16_t prev_code = (path_count > 0) ? path[path_count - 1] : 0;
            bool prev_is_small_turn = (prev_code >= 300 && prev_code < 500);
            bool prev_is_large_turn = (prev_code >= 500 && prev_code < 700);
            
            // 例外1: 大回りターン→半区画直進(S1)→小回りターン
            // 理由: 大回りターン出口で既に壁が途切れているため、壁切れを検出できない
            // 例外2: 小回りターン→半区画直進(S1)→大回りターン
            // 理由: 45mmしかなく、ターン速度が異なるため等速バッファ区間を取れない
            // 例外3: g_disable_wall_end_correctionが立っている場合（デバッグ用）
            bool skip_wallend = (prev_is_large_turn && path[path_count] == 201 && next_is_small_turn) ||
                                (prev_is_small_turn && path[path_count] == 201 && next_is_large_turn) ||
                                g_disable_wall_end_correction;
            
            // 壁切れ補正を適用するかどうか
            // 小回りターンと大回りターンの前で適用（例外パターンを除く）
            if ((next_is_small_turn || next_is_large_turn) && !skip_wallend) {
                // 壁切れ補正付き直進
                // メイン部分: ターン開始位置の45mm手前までターン速度まで減速
                // バッファ部分: 最大90mmの間等速で壁切れを探しながら走行
                
                const float BUFFER_MAX = (float)DIST_HALF_SEC;  // バッファ区間の最大距離[mm]
                
                // メイン部分の距離（ターン開始位置の45mm手前まで）
                float main_mm = straight_mm - WALL_END_BUFFER;
                if (main_mm < 0.0f) main_mm = 0.0f;
                
                // 短い直線の判定: メイン部分で十分な加減速ができるか
                // 最低でも1区画(90mm)のメイン部分がないと加減速が困難
                const float MIN_MAIN_FOR_ACCEL = (float)DIST_HALF_SEC * 2.0f;  // 90mm
                
                if (main_mm >= MIN_MAIN_FOR_ACCEL) {
                    // 通常の加減速処理（メイン部分が十分長い場合）
                    // 加速区間
                    if (d_acc > 0.0f) {
                        float acc_run = (d_acc < main_mm) ? d_acc : main_mm;
                        run_straight(acc_run / DIST_HALF_SEC, max_reached_speed, 0);
                    }
                    // 等速区間
                    float const_run = main_mm - d_acc;
                    if (const_run > 0.0f && d_constant > 0.0f) {
                        float run_dist = (const_run < d_constant) ? const_run : d_constant;
                        run_straight(run_dist / DIST_HALF_SEC, max_reached_speed, 0);
                    }
                    // 減速区間（メイン部分の残り）
                    float dec_in_main = main_mm - d_acc - d_constant;
                    if (dec_in_main > 0.0f) {
                        run_straight(dec_in_main / DIST_HALF_SEC, v_next, 0);
                    }
                } else if (main_mm > 0.0f) {
                    // 短い直線: 前のターン速度から次のターン速度へ直接遷移
                    // speed_now（現在速度）からv_next（次のターン速度）への加減速
                    run_straight(main_mm / DIST_HALF_SEC, v_next, 0);
                }
                
                // バッファ部分の走行（壁切れ検出付き、等速でターン速度を維持）
                // 最大90mm走行し、壁切れ検出で即座に終了
                bool wall_end_found = driveC_wallend(BUFFER_MAX, v_next);
                
                // 壁切れ検出後の処理
                if (wall_end_found) {
                    // 小回りターンの場合: 45mm + dist_wall_end 追加直進
                    // 大回りターンの場合: dist_wall_end 追加直進
                    float follow_dist = next_is_small_turn 
                        ? (WALL_END_BUFFER + dist_wall_end) 
                        : dist_wall_end;
                    if (follow_dist > 0.0f) {
                        run_straight(follow_dist / DIST_HALF_SEC, v_next, 0);
                    }
                }
                // 壁切れ未検出の場合（90mm走行完了）、そのままターン開始
                
            } else {
                // 次がターンでない場合、または壁切れ補正無効時の処理
                if (d_acc_blocks > 0.0f) {
                    run_straight(d_acc_blocks, max_reached_speed, 0);
                }
                if (d_constant_blocks > 0.0f) {
                    run_straight(d_constant_blocks, max_reached_speed, 0);
                }
                if (d_dec_blocks > 0.0f) {
                    run_straight(d_dec_blocks, v_next, 0);
                }
                
                // センサログ有効時: バッファ区間でセンサ値を記録（壁切れ補正は無効のまま）
                if (g_sensor_log_enabled && (next_is_small_turn || next_is_large_turn)) {
                    const float BUFFER_MAX = (float)DIST_HALF_SEC;
                    driveC_wallend(BUFFER_MAX, v_next);  // 壁切れ検出しても補正は行わない
                }
            }

        } else if (path[path_count] < 400) {
            // 右旋回

            turn_R90(1);

            turn_dir(DIR_TURN_R90); // マイクロマウス内部位置情報でも右回転処理

        } else if (path[path_count] < 500) {
            // 左旋回

            turn_L90(1);

            turn_dir(DIR_TURN_L90); // マイクロマウス内部位置情報でも右回転処理

        } else if (path[path_count] < 600) {
            // 右大回り旋回
            uint8_t l_turn_sections = path[path_count] - 500;
            // 次が大回りターンかどうか判定（500-699が大回り）
            uint16_t next_code = path[path_count + 1];
            bool next_is_large = (next_code >= 500 && next_code < 700);

            if (l_turn_sections == 2) {
                l_turn_R180(next_is_large);
            } else {
                l_turn_R90(next_is_large);
            }
        } else if (path[path_count] < 700) {
            // 左大回り旋回
            uint8_t l_turn_sections = path[path_count] - 600;
            // 次が大回りターンかどうか判定（500-699が大回り）
            uint16_t next_code = path[path_count + 1];
            bool next_is_large = (next_code >= 500 && next_code < 700);

            if (l_turn_sections == 2) {
                l_turn_L180(next_is_large);
            } else {
                l_turn_L90(next_is_large);
            }
        } else if (path[path_count] == 701) {
            // 45degターン右入り

            turn_R45_In();

        } else if (path[path_count] == 702) {
            // 45degターン左入り

            turn_L45_In();

        } else if (path[path_count] == 703) {
            // 45degターン右出

            turn_R45_Out();

        } else if (path[path_count] == 704) {
            // 45degターン左出

            turn_L45_Out();

        } else if (path[path_count] == 801) {
            // V90degターン右

            turn_RV90();

        } else if (path[path_count] == 802) {
            // V90degターン左

            turn_LV90();

        } else if (path[path_count] == 901) {
            // 135degターン右入り

            turn_R135_In();

        } else if (path[path_count] == 902) {
            // 135degターン左入り

            turn_L135_In();

        } else if (path[path_count] == 903) {
            // 135degターン右出

            turn_R135_Out();

        } else if (path[path_count] == 904) {
            // 135degターン左出

            turn_L135_Out();

        } else if (1000 < path[path_count] && path[path_count] < 1100) {
            // 斜め直進

            float straight_sections =
                (path[path_count] - 1000) * DIST_D_HALF_SEC;

            // 直線の加減速区画を計算

            // 加速時間、加速距離、減速距離の計算
            float t_acc = velocity_d_straight /
                          acceleration_d_straight_dash; // 加速時間 [s]
            float d_acc = 0.5f * acceleration_d_straight_dash * t_acc *
                          t_acc; // 加速距離 [mm]

            // 最高到達速度が走行距離内で到達できるかのチェック
            float d_total_acc_dec = 2 * d_acc; // 加速距離と減速距離の合計 [mm]
            float d_constant = 0.0f; // 等速距離 [mm]
            float max_reached_speed =
                velocity_d_straight; // 最高到達速度 [mm/s]

            if (d_total_acc_dec > straight_sections) {
                // 最高速度に達しない場合
                d_acc = straight_sections / 2; // 加速距離と減速距離は等しい
                d_constant = 0.0f;
                t_acc = sqrtf(2 * d_acc / acceleration_d_straight_dash);
                max_reached_speed = acceleration_d_straight_dash * t_acc;
            } else {
                // 最高速度に達する場合
                d_constant = straight_sections - d_total_acc_dec;
            }

            // 各距離を区画数に変換
            float d_acc_blocks = d_acc / DIST_D_HALF_SEC;
            float d_constant_blocks = d_constant / DIST_D_HALF_SEC;
            float d_dec_blocks = d_acc_blocks; // 減速距離は加速距離と等しい

            // 加速区間
            run_diagonal(d_acc_blocks, max_reached_speed);

            // 等速区間
            run_diagonal(d_constant_blocks, max_reached_speed);

            // 減速区間
            if (path[path_count + 1] < 800) {
                // 次が45degターン
                run_diagonal(d_dec_blocks, velocity_turn45out);
            } else if (path[path_count + 1] < 900) {
                // 次がV90degターン
                run_diagonal(d_dec_blocks, velocity_turnV90);
            } else if (path[path_count + 1] < 1000) {
                // 次が135degターン
                run_diagonal(d_dec_blocks, velocity_turn135out);
            } else {
                // 次が終了
                run_diagonal(d_dec_blocks, 0);
            }
        }
    }

    half_sectionD(0);

    // センサログ停止
    if (g_sensor_log_enabled) {
        sensor_log_stop();
    }

    // ゴール演出（LED/Buzzer）は run_shortest() 側で必要に応じて実施する。
    // ここでは直後にファン停止やLED消灯を行うため、一時的な再点灯/再起動を避ける目的で呼ばない。
    drive_stop();
}

void run_shortest(uint8_t mode, uint8_t case_index) {
    // case_index: 1..9 -> idx 0..8（mode2..7 は9要素、他は従来通り）
    uint8_t idx = 0;
    if (case_index >= 1 && case_index <= 9) {
        idx = (uint8_t)(case_index - 1);
    } else {
        // フォールバック: 0 を使用
        idx = 0;
    }

    const ShortestRunModeParams_t *pm = NULL;
    const ShortestRunCaseParams_t *pcases = NULL;
    switch (mode) {
        case 2: pm = &shortestRunModeParams2; pcases = &shortestRunCaseParamsMode2[0]; break;
        case 3: pm = &shortestRunModeParams3; pcases = &shortestRunCaseParamsMode3[0]; break;
        case 4: pm = &shortestRunModeParams4; pcases = &shortestRunCaseParamsMode4[0]; break;
        case 5: pm = &shortestRunModeParams5; pcases = &shortestRunCaseParamsMode5[0]; break;
        case 6: pm = &shortestRunModeParams6; pcases = &shortestRunCaseParamsMode6[0]; break;
        case 7: pm = &shortestRunModeParams7; pcases = &shortestRunCaseParamsMode7[0]; break;
        default: pm = &shortestRunModeParams2; pcases = &shortestRunCaseParamsMode2[0]; break;
    }

    // モードごとのケース数でクランプ（mode2..7:9要素=idx0..8）
    uint8_t max_idx = 4;
    if (mode >= 2 && mode <= 7) {
        max_idx = 8;
    }
    if (idx > max_idx) idx = max_idx;

    const ShortestRunCaseParams_t *p = &pcases[idx];

    printf("Mode %d-%d Shortest Run.\n", mode, case_index);

    // 経路作成（新ソルバを使用）
    solver_build_path(mode, case_index);

    // 走行フラグ
    MF.FLAG.RUNNING = 1;

    // パラメータ適用
    // 直線（caseごと）
    acceleration_straight      = p->acceleration_straight;
    acceleration_straight_dash = p->acceleration_straight_dash;
    velocity_straight          = p->velocity_straight;
    // 斜め直線（caseごと）
    acceleration_d_straight      = p->acceleration_d_straight;
    acceleration_d_straight_dash = p->acceleration_d_straight_dash;
    velocity_d_straight          = p->velocity_d_straight;
    // ターン（mode共通）
    velocity_turn90            = pm->velocity_turn90;
    alpha_turn90               = pm->alpha_turn90;
    acceleration_turn          = pm->acceleration_turn;
    dist_offset_in             = pm->dist_offset_in;
    dist_offset_out            = pm->dist_offset_out;
    val_offset_in              = pm->val_offset_in;
    angle_turn_90              = pm->angle_turn_90;
    velocity_l_turn_90         = pm->velocity_l_turn_90;
    alpha_l_turn_90            = pm->alpha_l_turn_90;
    angle_l_turn_90            = pm->angle_l_turn_90;
    dist_l_turn_in_90          = pm->dist_l_turn_in_90;
    dist_l_turn_out_90         = pm->dist_l_turn_out_90;
    velocity_l_turn_180        = pm->velocity_l_turn_180;
    alpha_l_turn_180           = pm->alpha_l_turn_180;
    angle_l_turn_180           = pm->angle_l_turn_180;
    dist_l_turn_in_180         = pm->dist_l_turn_in_180;
    dist_l_turn_out_180        = pm->dist_l_turn_out_180;
    // 斜めターン（mode共通）
    velocity_turn45in          = pm->velocity_turn45in;
    alpha_turn45in             = pm->alpha_turn45in;
    angle_turn45in             = pm->angle_turn45in;
    dist_turn45in_in           = pm->dist_turn45in_in;
    dist_turn45in_out          = pm->dist_turn45in_out;
    velocity_turn45out         = pm->velocity_turn45out;
    alpha_turn45out            = pm->alpha_turn45out;
    angle_turn45out            = pm->angle_turn45out;
    dist_turn45out_in          = pm->dist_turn45out_in;
    dist_turn45out_out         = pm->dist_turn45out_out;
    velocity_turnV90           = pm->velocity_turnV90;
    alpha_turnV90              = pm->alpha_turnV90;
    angle_turnV90              = pm->angle_turnV90;
    dist_turnV90_in            = pm->dist_turnV90_in;
    dist_turnV90_out           = pm->dist_turnV90_out;
    velocity_turn135in         = pm->velocity_turn135in;
    alpha_turn135in            = pm->alpha_turn135in;
    angle_turn135in            = pm->angle_turn135in;
    dist_turn135in_in          = pm->dist_turn135in_in;
    dist_turn135in_out         = pm->dist_turn135in_out;
    velocity_turn135out        = pm->velocity_turn135out;
    alpha_turn135out           = pm->alpha_turn135out;
    angle_turn135out           = pm->angle_turn135out;
    dist_turn135out_in         = pm->dist_turn135out_in;
    dist_turn135out_out        = pm->dist_turn135out_out;
    // 壁制御（caseごと）
    kp_wall                    = p->kp_wall;

    // 壁切れ後の距離・ケツ当て
    dist_wall_end = pm->dist_wall_end;
    duty_setposition = 40;

    // 壁切れ検出しきい値（モードごと、ヒステリシス付き）
    wall_end_thr_r_high = pm->wall_end_thr_r_high;
    wall_end_thr_r_low = pm->wall_end_thr_r_low;
    wall_end_thr_l_high = pm->wall_end_thr_l_high;
    wall_end_thr_l_low = pm->wall_end_thr_l_low;

    // 加速度切り替え速度（モードごと）
    accel_switch_velocity = pm->accel_switch_velocity;

    velocity_interrupt = 0;

    // センサ・モータ初期化
    led_flash(4);
    drive_variable_reset();
    IMU_GetOffset();
    drive_enable_motor();

    MF.FLAG.SCND = 1;
    MF.FLAG.RETURN = 0;

    led_flash(2);
    get_base();

    led_write(1,1,1);

    // ファン出力（mode共通）
    drive_fan(pm->fan_power);

    // 実行
    run();

    // 後処理
    drive_fan(0);
    MF.FLAG.RUNNING = 0;

    led_write(0,0,0);
    led_wait();
}
