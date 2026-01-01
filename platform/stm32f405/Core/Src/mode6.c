/*
 * mode6.c
 *
 *  Created on: Aug 17, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/shortest_run_params.h"
#include "../Inc/run.h"
#include "../Inc/logging.h"

// mode6用パラメータ適用関数
static void apply_case_params_mode6_idx(int idx) {
    const ShortestRunCaseParams_t *pc = &shortestRunCaseParamsMode6[idx];
    const ShortestRunModeParams_t *pm = shortest_get_mode_params(6);
    // 直線（caseごと）
    acceleration_straight      = pc->acceleration_straight;
    acceleration_straight_dash = pc->acceleration_straight_dash;
    velocity_straight          = pc->velocity_straight;
    kp_wall                    = pc->kp_wall;
    // 斜め直線（caseごと）
    acceleration_d_straight      = pc->acceleration_d_straight;
    acceleration_d_straight_dash = pc->acceleration_d_straight_dash;
    velocity_d_straight          = pc->velocity_d_straight;
    kp_diagonal                  = pc->kp_diagonal;
    // ターン（mode共通）
    velocity_turn90            = pm->velocity_turn90;
    alpha_turn90               = pm->alpha_turn90;
    acceleration_turn          = pm->acceleration_turn;
    dist_offset_in             = pm->dist_offset_in;
    dist_offset_out            = pm->dist_offset_out;
    val_offset_in              = pm->val_offset_in;
    angle_turn_90              = pm->angle_turn_90;
    dist_wall_end              = pm->dist_wall_end;
    // 大回り90
    velocity_l_turn_90         = pm->velocity_l_turn_90;
    alpha_l_turn_90            = pm->alpha_l_turn_90;
    angle_l_turn_90            = pm->angle_l_turn_90;
    dist_l_turn_in_90          = pm->dist_l_turn_in_90;
    dist_l_turn_out_90         = pm->dist_l_turn_out_90;
    // 大回り180
    velocity_l_turn_180        = pm->velocity_l_turn_180;
    alpha_l_turn_180           = pm->alpha_l_turn_180;
    angle_l_turn_180           = pm->angle_l_turn_180;
    dist_l_turn_in_180         = pm->dist_l_turn_in_180;
    dist_l_turn_out_180        = pm->dist_l_turn_out_180;
}

static void apply_turn_normal_mode6(void) {
    const ShortestRunModeParams_t *pm = shortest_get_mode_params(6);
    velocity_turn90    = pm->velocity_turn90;
    alpha_turn90       = pm->alpha_turn90;
    acceleration_turn  = pm->acceleration_turn;
    dist_offset_in     = pm->dist_offset_in;
    dist_offset_out    = pm->dist_offset_out;
    val_offset_in      = pm->val_offset_in;
    angle_turn_90      = pm->angle_turn_90;
}

static void apply_turn_large90_mode6(void) {
    const ShortestRunModeParams_t *pm = shortest_get_mode_params(6);
    velocity_l_turn_90  = pm->velocity_l_turn_90;
    alpha_l_turn_90     = pm->alpha_l_turn_90;
    angle_l_turn_90     = pm->angle_l_turn_90;
    dist_l_turn_in_90   = pm->dist_l_turn_in_90;
    dist_l_turn_out_90  = pm->dist_l_turn_out_90;
}

static void apply_turn_large180_mode6(void) {
    const ShortestRunModeParams_t *pm = shortest_get_mode_params(6);
    velocity_l_turn_180  = pm->velocity_l_turn_180;
    alpha_l_turn_180     = pm->alpha_l_turn_180;
    angle_l_turn_180     = pm->angle_l_turn_180;
    dist_l_turn_in_180   = pm->dist_l_turn_in_180;
    dist_l_turn_out_180  = pm->dist_l_turn_out_180;
}

void mode6() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: {
            printf("Mode 6-0: Turn test.\n");

            // テスト動作フラグをセット
            g_test_mode_run = true;

            // サブモード選択
            int sub = select_mode(0);
            // 通常ターン用のインデックス（case1相当）
            const int idx_normal = 0;
            // 斜め用のインデックス（case8相当）
            const int idx_diag = 7;

            switch (sub) {
            case 0: // 通常ターン
                apply_case_params_mode6_idx(idx_normal);
                apply_turn_normal_mode6();
                printf("Loaded params: normal turn (mode6).\n");
                {
                    const ShortestRunModeParams_t *pm0 = shortest_get_mode_params(6);
                    drive_fan(pm0->fan_power);
                    log_init();
                    log_set_profile(LOG_PROFILE_OMEGA);
                    log_start(HAL_GetTick());
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3; // S3
                    path[1] = 300;     // 右小回り
                    run();
                    drive_fan(0);
                    log_stop();
                    printf("[mode6-case0-sub0] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                           (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                    while (1) {
                        if (ad_fr > WALL_BASE_FR) { log_print_omega_all(); break; }
                        else if (ad_fl > WALL_BASE_FL) { log_print_angle_all(); break; }
                        HAL_Delay(50);
                    }
                }
                break;
            case 1: // 90deg大回り
                apply_case_params_mode6_idx(idx_normal);
                apply_turn_large90_mode6();
                printf("Loaded params: large 90deg (mode6).\n");
                {
                    const ShortestRunModeParams_t *pm1 = shortest_get_mode_params(6);
                    drive_fan(pm1->fan_power);
                    log_init();
                    log_set_profile(LOG_PROFILE_OMEGA);
                    log_start(HAL_GetTick());
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3; // S3
                    path[1] = 501;     // L-R90
                    run();
                    drive_fan(0);
                    log_stop();
                    printf("[mode6-case0-sub1] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                           (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                    while (1) {
                        if (ad_fr > WALL_BASE_FR) { log_print_omega_all(); break; }
                        else if (ad_fl > WALL_BASE_FL) { log_print_angle_all(); break; }
                        HAL_Delay(50);
                    }
                }
                break;
            case 2: // 180deg大回り
                apply_case_params_mode6_idx(idx_normal);
                apply_turn_large180_mode6();
                printf("Loaded params: large 180deg (mode6).\n");
                {
                    const ShortestRunModeParams_t *pm2 = shortest_get_mode_params(6);
                    drive_fan(pm2->fan_power);
                    log_init();
                    log_set_profile(LOG_PROFILE_OMEGA);
                    log_start(HAL_GetTick());
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3; // S3
                    path[1] = 502;     // L-R180
                    run();
                    drive_fan(0);
                    log_stop();
                    printf("[mode6-case0-sub2] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                           (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                    while (1) {
                        if (ad_fr > WALL_BASE_FR) { log_print_omega_all(); break; }
                        else if (ad_fl > WALL_BASE_FL) { log_print_angle_all(); break; }
                        HAL_Delay(50);
                    }
                }
                break;
            case 3: // 45deg 入り
                apply_case_params_mode6_idx(idx_diag);
                printf("Loaded params: diag 45-in (mode6).\n");
                {
                    const ShortestRunModeParams_t *pm3 = shortest_get_mode_params(6);
                    drive_fan(pm3->fan_power);
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3; // S3
                    path[1] = 701;     // 右45°入り
                    path[2] = 1000+1;  // 斜めS1
                    run();
                    drive_fan(0);
                }
                break;
            case 4: // 45deg 出
                apply_case_params_mode6_idx(idx_diag);
                printf("Loaded params: diag 45-out (mode6).\n");
                {
                    const ShortestRunModeParams_t *pm4 = shortest_get_mode_params(6);
                    drive_fan(pm4->fan_power);
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3;  // S3
                    path[1] = 1000+1;   // 斜めS1
                    path[2] = 704;      // 左45°出
                    path[3] = 1000+1;   // 斜めS1
                    run();
                    drive_fan(0);
                }
                break;
            case 5: // V90
                apply_case_params_mode6_idx(idx_diag);
                printf("Loaded params: diag V90 (mode6).\n");
                {
                    const ShortestRunModeParams_t *pm5 = shortest_get_mode_params(6);
                    drive_fan(pm5->fan_power);
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3;  // S3
                    path[1] = 1000+1;   // 斜めS1
                    path[2] = 802;      // 左V90
                    path[3] = 1000+1;   // 斜めS1
                    run();
                    drive_fan(0);
                }
                break;
            case 6: // 135deg 入り
                apply_case_params_mode6_idx(idx_diag);
                printf("Loaded params: diag 135-in (mode6).\n");
                {
                    const ShortestRunModeParams_t *pm6 = shortest_get_mode_params(6);
                    drive_fan(pm6->fan_power);
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3;  // S3
                    path[1] = 901;      // 右135°入り
                    path[2] = 1000+1;   // 斜めS1
                    run();
                    drive_fan(0);
                }
                break;
            case 7: // 135deg 出
                apply_case_params_mode6_idx(idx_diag);
                printf("Loaded params: diag 135-out (mode6).\n");
                {
                    const ShortestRunModeParams_t *pm7 = shortest_get_mode_params(6);
                    drive_fan(pm7->fan_power);
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 3;  // S3
                    path[1] = 1000+1;   // 斜めS1
                    path[2] = 904;      // 左135°出
                    path[3] = 1000+1;   // 斜めS1
                    run();
                    drive_fan(0);
                }
                break;
            case 8: { // Straight test (slow)
                apply_case_params_mode6_idx(0);
                kp_wall = 0.0f;
                printf("Loaded params: straight test (mode6, case8 -> case1 params).\n");
                drive_fan(shortest_get_mode_params(6)->fan_power);
                log_init();
                log_set_profile(LOG_PROFILE_DISTANCE);
                log_start(HAL_GetTick());
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                path[0] = 200 + 3; // S3
                path[1] = 0;
                run();
                drive_fan(0);
                log_stop();
                printf("[mode6-case0-sub8] Press RIGHT FRONT for VELOCITY (FR>%u), LEFT FRONT for DISTANCE (FL>%u) ...\n",
                       (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                while (1) {
                    if (ad_fr > WALL_BASE_FR) { log_print_velocity_all(); break; }
                    else if (ad_fl > WALL_BASE_FL) { log_print_distance_all(); break; }
                    HAL_Delay(50);
                }
                led_flash(5);
                break; }
            case 9: { // Straight test (fast)
                apply_case_params_mode6_idx(4);
                kp_wall = 0.0f;
                printf("Loaded params: straight test (mode6, case9 -> case5 params).\n");
                drive_fan(shortest_get_mode_params(6)->fan_power);
                log_init();
                log_set_profile(LOG_PROFILE_VELOCITY);
                log_start(HAL_GetTick());
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                path[0] = 200 + 3; // S3
                path[1] = 0;
                run();
                drive_fan(0);
                log_stop();
                printf("[mode6-case0-sub9] Press RIGHT FRONT for VELOCITY (FR>%u), LEFT FRONT for DISTANCE (FL>%u) ...\n",
                       (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                while (1) {
                    if (ad_fr > WALL_BASE_FR) { log_print_velocity_all(); break; }
                    else if (ad_fl > WALL_BASE_FL) { log_print_distance_all(); break; }
                    HAL_Delay(50);
                }
                led_flash(5);
                break; }
            default:
                printf("No sub-mode selected.\n");
                break;
            }

            // テスト動作フラグをリセット
            g_test_mode_run = false;
            break;
        }

        case 1:
            run_shortest(6, 1);
            break;

        case 2:
            g_disable_front_wall_correction = true;  // 前壁補正無効（距離ベース走行）
            sensor_log_init();
            log_init();
            log_set_profile(LOG_PROFILE_WALL_END_DERIV);
            log_start(HAL_GetTick());
            g_sensor_log_enabled = true;
            g_disable_front_wall_correction = true;
            run_shortest(6, 2);
            g_sensor_log_enabled = false;
            log_stop();
            g_disable_front_wall_correction = false;
            printf("Wall-end deriv log recorded: %d entries\n", log_buffer2.count);
            printf("Press button for wall-end deriv log output...\n");
            while (HAL_GPIO_ReadPin(PUSH_IN_1_GPIO_Port, PUSH_IN_1_Pin) != 0) { HAL_Delay(50); }
            log_print_wall_end_deriv_all();
            while (HAL_GPIO_ReadPin(PUSH_IN_1_GPIO_Port, PUSH_IN_1_Pin) == 0) { HAL_Delay(30); }
            printf("Sensor log recorded: %d entries\n", sensor_log_buffer.count);
            printf("Press button for sensor log output...\n");
            while (HAL_GPIO_ReadPin(PUSH_IN_1_GPIO_Port, PUSH_IN_1_Pin) != 0) { HAL_Delay(50); }
            sensor_log_print();
            led_flash(3);
            break;

        case 3:
            g_disable_front_wall_correction = true;
            run_shortest(6, 3);
            g_disable_front_wall_correction = false;
            break;

        case 4:
            g_disable_front_wall_correction = true;
            run_shortest(6, 4);
            g_disable_front_wall_correction = false;
            break;

        case 5:
            g_disable_front_wall_correction = true;
            run_shortest(6, 5);
            g_disable_front_wall_correction = false;
            break;

        case 6:
            g_disable_front_wall_correction = true;
            run_shortest(6, 6);
            g_disable_front_wall_correction = false;
            break;

        case 7:
            g_disable_front_wall_correction = true;
            run_shortest(6, 7);
            g_disable_front_wall_correction = false;
            break;

        case 8:
            g_disable_front_wall_correction = true;
            run_shortest(6, 8);
            g_disable_front_wall_correction = false;
            break;

        case 9:
            g_disable_front_wall_correction = true;
            run_shortest(6, 9);
            g_disable_front_wall_correction = false;
            break;
        }
    }
}
