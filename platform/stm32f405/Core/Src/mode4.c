/*
 * mode5.c
 *
 *  Created on: Jan 16, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/shortest_run_params.h"
#include "../Inc/run.h"
#include "../Inc/logging.h"

// Helper loaders: apply case/mode parameters to runtime globals (mode4)
static void apply_case_params_mode4_idx(int idx) {
    const ShortestRunCaseParams_t *c = &shortestRunCaseParamsMode4[idx];
    acceleration_straight = c->acceleration_straight;
    acceleration_straight_dash = c->acceleration_straight_dash;
    velocity_straight = c->velocity_straight;
    acceleration_d_straight = c->acceleration_d_straight;
    acceleration_d_straight_dash = c->acceleration_d_straight_dash;
    velocity_d_straight = c->velocity_d_straight;
    kp_wall = c->kp_wall;
    kp_diagonal = c->kp_diagonal;
}

static void apply_turn_normal_mode4(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_turn90 = m->velocity_turn90;
    alpha_turn90 = m->alpha_turn90;
    acceleration_turn = m->acceleration_turn;
    dist_offset_in = m->dist_offset_in;
    dist_offset_out = m->dist_offset_out;
    val_offset_in = m->val_offset_in;
    angle_turn_90 = m->angle_turn_90;
    dist_wall_end = m->dist_wall_end;
}

static void apply_turn_large90_mode4(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_l_turn_90 = m->velocity_l_turn_90;
    alpha_l_turn_90 = m->alpha_l_turn_90;
    angle_l_turn_90 = m->angle_l_turn_90;
    dist_l_turn_in_90 = m->dist_l_turn_in_90;
    dist_l_turn_out_90 = m->dist_l_turn_out_90;
}

static void apply_turn_large180_mode4(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_l_turn_180 = m->velocity_l_turn_180;
    alpha_l_turn_180 = m->alpha_l_turn_180;
    angle_l_turn_180 = m->angle_l_turn_180;
    dist_l_turn_in_180 = m->dist_l_turn_in_180;
    dist_l_turn_out_180 = m->dist_l_turn_out_180;
}

static void apply_turn_d45in_mode4(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_turn45in = m->velocity_turn45in;
    alpha_turn45in = m->alpha_turn45in;
    angle_turn45in = m->angle_turn45in;
    dist_turn45in_in = m->dist_turn45in_in;
    dist_turn45in_out = m->dist_turn45in_out;
}

static void apply_turn_d45out_mode4(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_turn45out = m->velocity_turn45out;
    alpha_turn45out = m->alpha_turn45out;
    angle_turn45out = m->angle_turn45out;
    dist_turn45out_in = m->dist_turn45out_in;
    dist_turn45out_out = m->dist_turn45out_out;
}

static void apply_turn_v90_mode4(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_turnV90 = m->velocity_turnV90;
    alpha_turnV90 = m->alpha_turnV90;
    angle_turnV90 = m->angle_turnV90;
    dist_turnV90_in = m->dist_turnV90_in;
    dist_turnV90_out = m->dist_turnV90_out;
}

static void apply_turn_d135in_mode4(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_turn135in = m->velocity_turn135in;
    alpha_turn135in = m->alpha_turn135in;
    angle_turn135in = m->angle_turn135in;
    dist_turn135in_in = m->dist_turn135in_in;
    dist_turn135in_out = m->dist_turn135in_out;
}

static void apply_turn_d135out_mode4(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams4;
    velocity_turn135out = m->velocity_turn135out;
    alpha_turn135out = m->alpha_turn135out;
    angle_turn135out = m->angle_turn135out;
    dist_turn135out_in = m->dist_turn135out_in;
    dist_turn135out_out = m->dist_turn135out_out;
}

void mode4() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: { // 調整モード選択（0..9）

            printf("Mode 4-0 Turn/Diagonal/Straight Test (sub 0..9).\n");

            led_flash(5);

            int sub = 0;
            sub = select_mode(sub);

            // 直線パラメータは、通常=case3(index 2)、斜め系=case8(index 7) を使用（case1/2 追加に伴う再マップ）
            const int idx_normal = 2; // case3
            const int idx_diag   = 7; // case8

            // テスト動作フラグを設定（センサ補正を無効化）
            g_test_mode_run = true;

            switch (sub) {
            case 0: // 通常ターン
                apply_case_params_mode4_idx(idx_normal);
                apply_turn_normal_mode4();
                printf("Loaded params: normal turn (mode4).\n");
                {
                    const ShortestRunModeParams_t *pm0 = &shortestRunModeParams4;
                    drive_fan(pm0->fan_power);
                    log_init();
                    log_set_profile(LOG_PROFILE_OMEGA);
                    log_start(HAL_GetTick());
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3; // S3
                    path[1] = 300; // 右小回り
                    path[2] = 0;
                    run();
                    drive_fan(0);
                    log_stop();
                    printf("[mode4-case0-sub0] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                           (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                    while (1) {
                        if (ad_fr > WALL_BASE_FR) { log_print_omega_all(); break; }
                        else if (ad_fl > WALL_BASE_FL) { log_print_angle_all(); break; }
                        HAL_Delay(50);
                    }
                }
                break;
            case 8: { // Straight test using case1 params (index0)
                // 直進テスト: mode4 の case1（index0）の直線パラメータを使用
                apply_case_params_mode4_idx(0);
                kp_wall = 0.0f; // テスト時は壁制御を無効化
                printf("Loaded params: straight test (mode4, case8 -> case1 params).\n");

                // fan_power を反映（代替案A）
                drive_fan(shortestRunModeParams4.fan_power);

                // ログ開始（距離プロファイル）
                log_init();
                log_set_profile(LOG_PROFILE_DISTANCE);
                log_start(HAL_GetTick());

                // path を上書きして run()
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                // 直進：初期 half_sectionA(S2) + S3 + 最後の half_sectionD(S1) = 合計S6
                path[0] = 200 + 5; // S3 (半区画×3)
                path[1] = 0;

                // 実行
                run();

                // fan停止
                drive_fan(0);

                // ログ停止
                log_stop();

                // センサEnter待ち（右前=速度ログ, 左前=距離ログ）
                printf("[mode4-case8] Press RIGHT FRONT for VELOCITY (FR>%u), LEFT FRONT for DISTANCE (FL>%u) ...\n",
                       (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                while (1) {
                    if (ad_fr > WALL_BASE_FR) {
                        log_print_velocity_all();
                        break;
                    } else if (ad_fl > WALL_BASE_FL) {
                        log_print_distance_all();
                        break;
                    }
                    HAL_Delay(50);
                }

                led_flash(5);
                break;
            }
            case 9: { // Straight test using caseX params (fast)
                // 直進テスト: mode4 の高速ケース（ここでは case7 相当として index6 がないため index4）
                apply_case_params_mode4_idx(4);
                kp_wall = 0.0f; // テスト時は壁制御を無効化
                printf("Loaded params: straight test (mode4, case9 -> case5 params).\n");

                // fan_power を反映（代替案A）
                drive_fan(shortestRunModeParams4.fan_power);

                // ログ開始（速度プロファイル）
                log_init();
                log_set_profile(LOG_PROFILE_VELOCITY);
                log_start(HAL_GetTick());

                // path を上書きして run()
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                // 直進：初期 half_sectionA(S2) + S3 + 最後の half_sectionD(S1) = 合計S6
                path[0] = 200 + 5; // S3
                path[1] = 0;

                // 実行
                run();

                // fan停止
                drive_fan(0);

                // ログ停止
                log_stop();

                // センサEnter待ち（右前=速度ログ, 左前=距離ログ）
                printf("[mode4-case9] Press RIGHT FRONT for VELOCITY (FR>%u), LEFT FRONT for DISTANCE (FL>%u) ...\n",
                       (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                while (1) {
                    if (ad_fr > WALL_BASE_FR) {
                        log_print_velocity_all();
                        break;
                    } else if (ad_fl > WALL_BASE_FL) {
                        log_print_distance_all();
                        break;
                    }
                    HAL_Delay(50);
                }

                led_flash(5);
                break;
            }
            case 1: // 90deg大回り
                apply_case_params_mode4_idx(idx_normal);
                apply_turn_large90_mode4();
                printf("Loaded params: large 90deg (mode4).\n");
                {
                    const ShortestRunModeParams_t *pm1 = &shortestRunModeParams4;
                    drive_fan(pm1->fan_power);
                    log_init();
                    log_set_profile(LOG_PROFILE_OMEGA);
                    log_start(HAL_GetTick());
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3; // S3
                    path[1] = 501; // L-R90
                    run();
                    drive_fan(0);
                    log_stop();
                    printf("[mode4-case0-sub1] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                           (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                    while (1) {
                        if (ad_fr > WALL_BASE_FR) { log_print_omega_all(); break; }
                        else if (ad_fl > WALL_BASE_FL) { log_print_angle_all(); break; }
                        HAL_Delay(50);
                    }
                }
                break;
            case 2: // 180deg大回り
                apply_case_params_mode4_idx(idx_normal);
                apply_turn_large180_mode4();
                printf("Loaded params: large 180deg (mode4).\n");
                {
                    const ShortestRunModeParams_t *pm2 = &shortestRunModeParams4;
                    drive_fan(pm2->fan_power);
                    log_init();
                    log_set_profile(LOG_PROFILE_OMEGA);
                    log_start(HAL_GetTick());
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3; // S3
                    path[1] = 502; // L-R180
                    run();
                    drive_fan(0);
                    log_stop();
                    printf("[mode4-case0-sub2] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                           (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                    while (1) {
                        if (ad_fr > WALL_BASE_FR) { log_print_omega_all(); break; }
                        else if (ad_fl > WALL_BASE_FL) { log_print_angle_all(); break; }
                        HAL_Delay(50);
                    }
                }
                break;
            case 3: // 45deg 入り
                apply_case_params_mode4_idx(idx_diag);
                apply_turn_d45in_mode4();
                printf("Loaded params: diag 45-in (mode4).\n");
                {
                    const ShortestRunModeParams_t *pm3 = &shortestRunModeParams4;
                    drive_fan(pm3->fan_power);
                    log_init();
                    log_set_profile(LOG_PROFILE_OMEGA);
                    log_start(HAL_GetTick());
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3; // S3
                    path[1] = 901; // 右45°入
                    path[2] = 1000 + 1; // 斜めS1
                    run();
                    drive_fan(0);
                    log_stop();
                    printf("[mode4-case0-sub3] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                           (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                    while (1) {
                        if (ad_fr > WALL_BASE_FR) { log_print_omega_all(); break; }
                        else if (ad_fl > WALL_BASE_FL) { log_print_angle_all(); break; }
                        HAL_Delay(50);
                    }
                }
                break;
            case 4: // 45deg 出
                apply_case_params_mode4_idx(idx_diag);
                apply_turn_d45out_mode4();
                printf("Loaded params: diag 45-out (mode4).\n");
                {
                    const ShortestRunModeParams_t *pm4 = &shortestRunModeParams4;
                    drive_fan(pm4->fan_power);
                    log_init();
                    log_set_profile(LOG_PROFILE_OMEGA);
                    log_start(HAL_GetTick());
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3; // S3
                    path[1] = 1000 + 1; // 斜めS1
                    path[2] = 904; // 左45°出
                    path[3] = 1000 + 1; // 斜めS1
                    run();
                    drive_fan(0);
                    log_stop();
                    printf("[mode4-case0-sub4] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                           (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                    while (1) {
                        if (ad_fr > WALL_BASE_FR) { log_print_omega_all(); break; }
                        else if (ad_fl > WALL_BASE_FL) { log_print_angle_all(); break; }
                        HAL_Delay(50);
                    }
                }
                break;
            case 5: // V90
                apply_case_params_mode4_idx(idx_diag);
                apply_turn_v90_mode4();
                printf("Loaded params: diag V90 (mode4).\n");
                {
                    const ShortestRunModeParams_t *pm5 = &shortestRunModeParams4;
                    drive_fan(pm5->fan_power);
                    log_init();
                    log_set_profile(LOG_PROFILE_OMEGA);
                    log_start(HAL_GetTick());
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3; // S3
                    path[1] = 1000 + 1; // 斜めS1
                    path[2] = 701; // 右V90
                    path[3] = 1000 + 1; // 斜めS1
                    run();
                    drive_fan(0);
                    log_stop();
                    printf("[mode4-case0-sub5] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                           (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                    while (1) {
                        if (ad_fr > WALL_BASE_FR) { log_print_omega_all(); break; }
                        else if (ad_fl > WALL_BASE_FL) { log_print_angle_all(); break; }
                        HAL_Delay(50);
                    }
                }
                break;
            case 6: // 135deg 入り
                apply_case_params_mode4_idx(idx_diag);
                apply_turn_d135in_mode4();
                printf("Loaded params: diag 135-in (mode4).\n");
                {
                    const ShortestRunModeParams_t *pm6 = &shortestRunModeParams4;
                    drive_fan(pm6->fan_power);
                    log_init();
                    log_set_profile(LOG_PROFILE_OMEGA);
                    log_start(HAL_GetTick());
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3; // S3
                    path[1] = 903; // 右135°入
                    path[2] = 1000 + 1; // 斜めS1
                    run();
                    drive_fan(0);
                    log_stop();
                    printf("[mode4-case0-sub6] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                           (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                    while (1) {
                        if (ad_fr > WALL_BASE_FR) { log_print_omega_all(); break; }
                        else if (ad_fl > WALL_BASE_FL) { log_print_angle_all(); break; }
                        HAL_Delay(50);
                    }
                }
                break;
            case 7: // 135deg 出
                apply_case_params_mode4_idx(idx_diag);
                apply_turn_d135out_mode4();
                printf("Loaded params: diag 135-out (mode4).\n");
                {
                    const ShortestRunModeParams_t *pm7 = &shortestRunModeParams4;
                    drive_fan(pm7->fan_power);
                    log_init();
                    log_set_profile(LOG_PROFILE_OMEGA);
                    log_start(HAL_GetTick());
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3; // S3
                    path[1] = 1000 + 1; // 斜めS1
                    path[2] = 904; // 左135°出
                    path[3] = 1000 + 1; // 斜めS1
                    run();
                    drive_fan(0);
                    log_stop();
                    printf("[mode4-case0-sub7] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                           (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                    while (1) {
                        if (ad_fr > WALL_BASE_FR) { log_print_omega_all(); break; }
                        else if (ad_fl > WALL_BASE_FL) { log_print_angle_all(); break; }
                        HAL_Delay(50);
                    }
                }
                break;
            default:
                printf("No sub-mode selected.\n");
                break;
            }

            // テスト動作フラグをリセット
            g_test_mode_run = false;

            // 動作内容はユーザー側で実装予定のため、ここでは読み込みのみ
            break;
        }

        case 1:
            // 最短走行（case1）
            run_shortest(4, 1);
            break;

        case 2:
            // 最短走行（case2）- 前壁補正無効
            g_disable_front_wall_correction = true;
            sensor_log_init();
            g_sensor_log_enabled = true;
            g_disable_front_wall_correction = true;
            run_shortest(4, 2);
            g_sensor_log_enabled = false;
            g_disable_front_wall_correction = false;
            printf("Sensor log recorded: %d entries\n", sensor_log_buffer.count);
            printf("Press button for sensor log output...\n");
            while (HAL_GPIO_ReadPin(PUSH_IN_1_GPIO_Port, PUSH_IN_1_Pin) != 0) { HAL_Delay(50); }
            sensor_log_print();
            led_flash(3);
            break;

        case 3:
            g_disable_front_wall_correction = true;
            run_shortest(4, 3);
            g_disable_front_wall_correction = false;
            break;

        case 4:
            g_disable_front_wall_correction = true;
            run_shortest(4, 4);
            g_disable_front_wall_correction = false;
            break;

        case 5:
            g_disable_front_wall_correction = true;
            run_shortest(4, 5);
            g_disable_front_wall_correction = false;
            break;

        case 6:
            g_disable_front_wall_correction = true;
            run_shortest(4, 6);
            g_disable_front_wall_correction = false;
            break;

        case 7:
            g_disable_front_wall_correction = true;
            run_shortest(4, 7);
            g_disable_front_wall_correction = false;
            break;
        case 8:
            g_disable_front_wall_correction = true;
            run_shortest(4, 8);
            g_disable_front_wall_correction = false;
            break;
        case 9:
            g_disable_front_wall_correction = true;
            run_shortest(4, 9);
            g_disable_front_wall_correction = false;
            break;




        }
    }
}
