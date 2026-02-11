/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "sensor_distance.h"
#include "../Inc/logging.h"
#include "../Inc/search_run_params.h"



//============================================================
// 探索走行パラメータ適用ヘルパー関数
//============================================================
static void apply_search_params(int case_index)
{
    if (case_index < 0 || case_index >= 2) {
        printf("Error: Invalid case_index %d\n", case_index);
        return;
    }

    const SearchRunParams_t *params = &searchRunParams[case_index];

    // 直線パラメータ
    acceleration_straight = params->acceleration_straight;
    acceleration_straight_dash = params->acceleration_straight_dash;

    // ターンパラメータ
    velocity_turn90 = params->velocity_turn90;
    alpha_turn90 = params->alpha_turn90;
    acceleration_turn = params->acceleration_turn;
    dist_offset_in = params->dist_offset_in;
    dist_offset_out = params->dist_offset_out;
    val_offset_in = params->val_offset_in;
    angle_turn_90 = params->angle_turn_90;

    // 壁切れ後の追従距離
    dist_wall_end = params->dist_wall_end;

    // 壁制御パラメータ
    kp_wall = params->kp_wall;
    duty_setposition = params->duty_setposition;

    // センサパラメータ
    sensor_kx = params->sensor_kx;
    fwall_kx = params->fwall_kx;

    // 壁切れ検出しきい値（ヒステリシス付き）
    wall_end_thr_r_high = params->wall_end_thr_r_high;
    wall_end_thr_r_low = params->wall_end_thr_r_low;
    wall_end_thr_l_high = params->wall_end_thr_l_high;
    wall_end_thr_l_low = params->wall_end_thr_l_low;

    // フラグ
    MF.FLAG.WALL_ALIGN = params->wall_align_enable;

    printf("Applied search params case %d (velocity: %.0f mm/s)\n", 
           case_index + 1, params->velocity_turn90);
}

//============================================================
// 前壁追従（match_position連続実行）テスト
//  with_print!=0 でFR/FLのAD値と距離[mm]を定期表示
//  PUSHボタン押下で終了
//============================================================
static void front_follow_continuous(int with_print)
{
    printf("[FrontFollow] match_position continuous. Press PUSH to exit.\n");
    led_flash(3);

    // 側壁制御を無効化して前壁のみで合わせる
    MF.FLAG.CTRL = 0;
    kp_wall = 0.0f;

    // 走行開始準備
    drive_variable_reset();
    IMU_GetOffset();
    drive_enable_motor();
    drive_start();

    uint32_t last_print = HAL_GetTick();

    while (1) {
        // 抜け条件：PUSHボタン
        if (HAL_GPIO_ReadPin(PUSH_IN_1_GPIO_Port, PUSH_IN_1_Pin) == 0) {
            buzzer_enter(900);
            break;
        }

        // 前壁が見えているときに位置合わせを実行
        if (ad_fr > F_ALIGN_DETECT_THR && ad_fl > F_ALIGN_DETECT_THR) {
            match_position(0);
        } else {
            // 待機（見えていない間は停止）
            velocity_interrupt = 0;
            omega_interrupt = 0;
            HAL_Delay(50);
        }

        // 任意の表示
        if (with_print) {
            uint32_t now = HAL_GetTick();
            if (now - last_print >= 200) {
                float d_fr = sensor_distance_from_fr(ad_fr);
                float d_fl = sensor_distance_from_fl(ad_fl);
                printf("FR=%u (%.1fmm), FL=%u (%.1fmm)\n",
                       (unsigned)ad_fr, d_fr, (unsigned)ad_fl, d_fl);
                last_print = now;
            }
        }

        HAL_Delay(10);
    }

    // 停止処理
    velocity_interrupt = 0;
    omega_interrupt = 0;
    drive_variable_reset();
    drive_stop();
    led_flash(2);
}

void mode1() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: { // 探索走行テスト調整モード

            printf("Mode 1-0 Search Test (sub 0..3).\n");

            led_flash(5);

            int sub = 0;
            sub = select_mode(sub);

            switch (sub) {
            case 0:
                printf("[Test] Reserved (not implemented).\n");
                break;

            case 1: { // 標準速度 小回りターン
                printf("Mode 1-0-1: Standard speed turn test.\n");
                
                // 標準速度パラメータ適用
                apply_search_params(0);
                
                // 壁制御・補正無効化
                float kp_wall_backup = kp_wall;
                float dist_wall_end_backup = dist_wall_end;
                kp_wall = 0.0f;           // 横壁制御無効
                dist_wall_end = 0.0f;     // 壁切れ補正無効
                MF.FLAG.CTRL = 0;         // 制御系無効化
                
                // モーター・センサ初期化
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();
                
                // ログ開始
                log_init();
                log_set_profile(LOG_PROFILE_OMEGA);
                log_start(HAL_GetTick());
                
                // 探索走行シーケンス実行
                drive_start();
                half_sectionA(1);
                turn_R90(0);       
                half_sectionD(0);      
                drive_stop();
                
                // ログ停止
                log_stop();
                
                // パラメータ復元
                kp_wall = kp_wall_backup;
                dist_wall_end = dist_wall_end_backup;
                
                // センサ入力待ち
                printf("[mode1-case0-sub1] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                       (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                while (1) {
                    if (ad_fr > WALL_BASE_FR) {
                        log_print_omega_all();
                        break;
                    } else if (ad_fl > WALL_BASE_FL) {
                        log_print_angle_all();
                        break;
                    }
                    HAL_Delay(50);
                }
                break;
            }
            
            case 2: { // 低速 小回りターン
                printf("Mode 1-0-2: Low speed turn test.\n");
                
                // 低速パラメータ適用
                apply_search_params(1);
                
                // 壁制御・補正無効化
                float kp_wall_backup = kp_wall;
                float dist_wall_end_backup = dist_wall_end;
                kp_wall = 0.0f;           // 横壁制御無効
                dist_wall_end = 0.0f;     // 壁切れ補正無効
                MF.FLAG.CTRL = 0;         // 制御系無効化
                
                // モーター・センサ初期化
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();
                
                // ログ開始
                log_init();
                log_set_profile(LOG_PROFILE_OMEGA);
                log_start(HAL_GetTick());
                
                // 探索走行シーケンス実行
                drive_start();
                half_sectionA(1);
                turn_R90(0);       
                half_sectionD(0);      
                drive_stop();
                
                // ログ停止
                log_stop();
                
                // パラメータ復元
                kp_wall = kp_wall_backup;
                dist_wall_end = dist_wall_end_backup;
                
                // センサ入力待ち
                printf("[mode1-case0-sub2] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                       (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                while (1) {
                    if (ad_fr > WALL_BASE_FR) {
                        log_print_omega_all();
                        break;
                    } else if (ad_fl > WALL_BASE_FL) {
                        log_print_angle_all();
                        break;
                    }
                    HAL_Delay(50);
                }
                break;
            }
            
            case 3: { // 直進3区画
                printf("Mode 1-0-3: Straight 3-section test.\n");
                
                // 標準速度パラメータ適用
                apply_search_params(0);
                
                // 壁制御・補正無効化
                float kp_wall_backup = kp_wall;
                float dist_wall_end_backup = dist_wall_end;
                kp_wall = 0.0f;           // 横壁制御無効
                dist_wall_end = 0.0f;     // 壁切れ補正無効
                MF.FLAG.CTRL = 0;         // 制御系無効化
                
                // モーター・センサ初期化
                drive_variable_reset();
                IMU_GetOffset();
                drive_enable_motor();
                
                // ログ開始
                log_init();
                log_set_profile(LOG_PROFILE_VELOCITY);
                log_start(HAL_GetTick());
                
                // 探索走行シーケンス実行
                drive_start();
                first_sectionA();     // 最初の加速区画
                one_sectionU(1.0f, speed_now);        // 2区画目
                one_sectionU(1.0f, speed_now);        // 3区画目
                drive_stop();
                
                // ログ停止
                log_stop();
                
                // パラメータ復元
                kp_wall = kp_wall_backup;
                dist_wall_end = dist_wall_end_backup;
                
                // センサ入力待ち（直進なのでVELOCITYログのみ）
                printf("[mode1-case0-sub3] Press RIGHT FRONT for VELOCITY log (FR>%u) ...\n",
                       (unsigned)WALL_BASE_FR);
                while (1) {
                    if (ad_fr > WALL_BASE_FR) {
                        log_print_all();
                        break;
                    }
                    HAL_Delay(50);
                }
                break;
            }
            
            default:
                printf("Invalid sub mode.\n");
                break;
            }

            break;
        }


        case 1: { // 標準速度で ゴール探索→全面探索

            printf("Mode 1-1: Standard speed (Goal->Full).\n");
            
            // パラメータ適用（標準速度）
            apply_search_params(0);

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            // ===== 第1フェーズ: ゴール到達で終了 =====
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            g_defer_save_until_end = true;  // 全面探索終了まで保存を延期
            search_end = false;
            adachi(searchRunParams[0].fan_duty);

            // ゴール到達時は保存せず、全面探索終了後に保存

            // ===== 第2フェーズ: 全面探索 =====
            led_flash(2);
            drive_variable_reset();
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_FULL);
            g_suppress_first_stop_save = true;
            g_second_phase_search = true;  // 第2フェーズフラグ設定
            search_end = false;
            adachi(searchRunParams[0].fan_duty);

            // 全面探索終了後に保存
            g_defer_save_until_end = false;
            store_map_in_eeprom();

            led_wait();

            break;
        }

        case 2: // 標準速度で 最初から全面探索

            printf("Mode 1-2: Standard speed (Full from start).\n");

            // パラメータ適用（標準速度）
            apply_search_params(0);

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();
            
            // 最初から全面探索
            set_search_mode(SEARCH_MODE_FULL);
            search_end = false;
            adachi(searchRunParams[0].fan_duty);

            led_wait();

            break;

        case 3: // 標準速度で ゴール探索→スタートへ帰り探索

            printf("Mode 1-3: Standard speed (Goal->Return to Start).\n");

            // パラメータ適用（標準速度）
            apply_search_params(0);

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            // ===== 第1フェーズ: ゴール到達で終了 =====
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            g_goal_is_start = false;
            goal_x = GOAL_X; goal_y = GOAL_Y;
            search_end = false;
            adachi(searchRunParams[0].fan_duty);

            // ゴール到達後に一度だけ安全に保存
            if (save_count == 0) {
                if (try_store_map_safely()) {
                    save_count = 1;
                }
            }

            // ===== 第2フェーズ: スタートへ復帰（スタート到達で終了） =====
            led_flash(2);
            drive_variable_reset();
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            MF.FLAG.GOALED = 0;
            g_goal_is_start = true;
            goal_x = START_X; goal_y = START_Y;
            search_end = false;
            g_second_phase_search = true;  // 第2フェーズフラグ設定
            adachi(searchRunParams[0].fan_duty);

            // 後処理
            g_goal_is_start = false;

            led_wait();

            break;

        case 4: // 標準速度で ゴール探索→ゴール到達で終了
            printf("Mode 1-4: Standard speed (Goal only).\n");

            // パラメータ適用（標準速度）
            apply_search_params(0);

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();
            
            // ゴール到達で終了モード
            set_search_mode(SEARCH_MODE_GOAL);
            search_end = false;
            adachi(searchRunParams[0].fan_duty);

            led_wait();

            break;

        case 5: // 低速で ゴール探索→全面探索

            printf("Mode 1-5: Low speed (Goal->Full).\n");

            MF.FLAG.RUNNING = 1;

            // パラメータ適用（低速）
            apply_search_params(1);

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            // ===== 第1フェーズ: ゴール到達で終了 =====
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            g_defer_save_until_end = true;  // 全面探索終了まで保存を延期
            search_end = false;
            adachi(searchRunParams[1].fan_duty);

            // ゴール到達時は保存せず、全面探索終了後に保存

            // ===== 第2フェーズ: 全面探索 =====
            led_flash(2);
            drive_variable_reset();
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_FULL);
            g_suppress_first_stop_save = true;
            g_second_phase_search = true;  // 第2フェーズフラグ設定
            search_end = false;
            adachi(searchRunParams[1].fan_duty);

            // 全面探索終了後に保存
            g_defer_save_until_end = false;
            store_map_in_eeprom();

            led_wait();

            break;

        case 6: // 低速で 最初から全面探索

            printf("Mode 1-6: Low speed (Full from start).\n");

            MF.FLAG.RUNNING = 1;

            // パラメータ適用（低速）
            apply_search_params(1);

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();
            
            // 最初から全面探索
            set_search_mode(SEARCH_MODE_FULL);
            search_end = false;
            adachi(searchRunParams[1].fan_duty);

            led_wait();

            break;

        case 7: // 低速で ゴール探索→スタートへ帰り探索

            printf("Mode 1-7: Low speed (Goal->Return to Start).\n");

            MF.FLAG.RUNNING = 1;

            // パラメータ適用（低速）
            apply_search_params(1);

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            // ===== 第1フェーズ: ゴール到達で終了 =====
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            g_goal_is_start = false;
            goal_x = GOAL_X; goal_y = GOAL_Y;
            search_end = false;
            adachi(searchRunParams[1].fan_duty);

            // ゴール到達後に一度だけ安全に保存
            if (save_count == 0) {
                if (try_store_map_safely()) {
                    save_count = 1;
                }
            }

            // ===== 第2フェーズ: スタートへ復帰（スタート到達で終了） =====
            led_flash(2);
            drive_variable_reset();
            get_base();
            drive_start();
            set_search_mode(SEARCH_MODE_GOAL);
            MF.FLAG.GOALED = 0;
            g_goal_is_start = true;
            goal_x = START_X; goal_y = START_Y;
            search_end = false;
            g_second_phase_search = true;  // 第2フェーズフラグ設定
            adachi(searchRunParams[1].fan_duty);

            // 後処理
            g_goal_is_start = false;

            led_wait();

            break;

        case 8: // 低速で ゴール探索→ゴール到達で終了
            printf("Mode 1-8: Low speed (Goal only).\n");

            MF.FLAG.RUNNING = 1;

            // パラメータ適用（低速）
            apply_search_params(1);

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(2);

            get_base();

            drive_start();
            
            // ゴール到達で終了モード
            set_search_mode(SEARCH_MODE_GOAL);
            search_end = false;
            adachi(searchRunParams[1].fan_duty);

            led_wait();

            break;
        }
    }
}
