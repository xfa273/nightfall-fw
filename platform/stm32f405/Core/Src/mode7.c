/*
 * mode7.c
 *
 *  Created on: Aug 17, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/shortest_run_params.h"
#include "../Inc/run.h"
#include "../Inc/logging.h"

static void apply_case_params_mode7_idx(int idx) {
    if (idx < 0) idx = 0;
    if (idx > 8) idx = 8;

    const ShortestRunCaseParams_t *pc = &shortestRunCaseParamsMode7[idx];
    const ShortestRunModeParams_t *pm = &shortestRunModeParams7;

    acceleration_straight      = pc->acceleration_straight;
    acceleration_straight_dash = pc->acceleration_straight_dash;
    velocity_straight          = pc->velocity_straight;
    kp_wall                    = pc->kp_wall;

    acceleration_d_straight      = pc->acceleration_d_straight;
    acceleration_d_straight_dash = pc->acceleration_d_straight_dash;
    velocity_d_straight          = pc->velocity_d_straight;
    kp_diagonal                  = pc->kp_diagonal;

    velocity_turn90    = pm->velocity_turn90;
    alpha_turn90       = pm->alpha_turn90;
    acceleration_turn  = pm->acceleration_turn;
    dist_offset_in     = pm->dist_offset_in;
    dist_offset_out    = pm->dist_offset_out;
    val_offset_in      = pm->val_offset_in;
    angle_turn_90      = pm->angle_turn_90;
    dist_wall_end      = pm->dist_wall_end;

    velocity_l_turn_90   = pm->velocity_l_turn_90;
    alpha_l_turn_90      = pm->alpha_l_turn_90;
    angle_l_turn_90      = pm->angle_l_turn_90;
    dist_l_turn_in_90    = pm->dist_l_turn_in_90;
    dist_l_turn_out_90   = pm->dist_l_turn_out_90;
    velocity_l_turn_180  = pm->velocity_l_turn_180;
    alpha_l_turn_180     = pm->alpha_l_turn_180;
    angle_l_turn_180     = pm->angle_l_turn_180;
    dist_l_turn_in_180   = pm->dist_l_turn_in_180;
    dist_l_turn_out_180  = pm->dist_l_turn_out_180;

    velocity_turn45in    = pm->velocity_turn45in;
    alpha_turn45in       = pm->alpha_turn45in;
    angle_turn45in       = pm->angle_turn45in;
    dist_turn45in_in     = pm->dist_turn45in_in;
    dist_turn45in_out    = pm->dist_turn45in_out;
    velocity_turn45out   = pm->velocity_turn45out;
    alpha_turn45out      = pm->alpha_turn45out;
    angle_turn45out      = pm->angle_turn45out;
    dist_turn45out_in    = pm->dist_turn45out_in;
    dist_turn45out_out   = pm->dist_turn45out_out;
    velocity_turnV90     = pm->velocity_turnV90;
    alpha_turnV90        = pm->alpha_turnV90;
    angle_turnV90        = pm->angle_turnV90;
    dist_turnV90_in      = pm->dist_turnV90_in;
    dist_turnV90_out     = pm->dist_turnV90_out;
    velocity_turn135in   = pm->velocity_turn135in;
    alpha_turn135in      = pm->alpha_turn135in;
    angle_turn135in      = pm->angle_turn135in;
    dist_turn135in_in    = pm->dist_turn135in_in;
    dist_turn135in_out   = pm->dist_turn135in_out;
    velocity_turn135out  = pm->velocity_turn135out;
    alpha_turn135out     = pm->alpha_turn135out;
    angle_turn135out     = pm->angle_turn135out;
    dist_turn135out_in   = pm->dist_turn135out_in;
    dist_turn135out_out  = pm->dist_turn135out_out;
}

static void apply_turn_normal_mode7(void) {
    const ShortestRunModeParams_t *pm = &shortestRunModeParams7;
    velocity_turn90    = pm->velocity_turn90;
    alpha_turn90       = pm->alpha_turn90;
    acceleration_turn  = pm->acceleration_turn;
    dist_offset_in     = pm->dist_offset_in;
    dist_offset_out    = pm->dist_offset_out;
    val_offset_in      = pm->val_offset_in;
    angle_turn_90      = pm->angle_turn_90;
}

static void apply_turn_large90_mode7(void) {
    const ShortestRunModeParams_t *pm = &shortestRunModeParams7;
    velocity_l_turn_90  = pm->velocity_l_turn_90;
    alpha_l_turn_90     = pm->alpha_l_turn_90;
    angle_l_turn_90     = pm->angle_l_turn_90;
    dist_l_turn_in_90   = pm->dist_l_turn_in_90;
    dist_l_turn_out_90  = pm->dist_l_turn_out_90;
}

static void apply_turn_large180_mode7(void) {
    const ShortestRunModeParams_t *pm = &shortestRunModeParams7;
    velocity_l_turn_180  = pm->velocity_l_turn_180;
    alpha_l_turn_180     = pm->alpha_l_turn_180;
    angle_l_turn_180     = pm->angle_l_turn_180;
    dist_l_turn_in_180   = pm->dist_l_turn_in_180;
    dist_l_turn_out_180  = pm->dist_l_turn_out_180;
}

static void apply_turn_d45in_mode7(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams7;
    velocity_turn45in = m->velocity_turn45in;
    alpha_turn45in = m->alpha_turn45in;
    angle_turn45in = m->angle_turn45in;
    dist_turn45in_in = m->dist_turn45in_in;
    dist_turn45in_out = m->dist_turn45in_out;
}

static void apply_turn_d45out_mode7(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams7;
    velocity_turn45out = m->velocity_turn45out;
    alpha_turn45out = m->alpha_turn45out;
    angle_turn45out = m->angle_turn45out;
    dist_turn45out_in = m->dist_turn45out_in;
    dist_turn45out_out = m->dist_turn45out_out;
}

static void apply_turn_v90_mode7(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams7;
    velocity_turnV90 = m->velocity_turnV90;
    alpha_turnV90 = m->alpha_turnV90;
    angle_turnV90 = m->angle_turnV90;
    dist_turnV90_in = m->dist_turnV90_in;
    dist_turnV90_out = m->dist_turnV90_out;
}

static void apply_turn_d135in_mode7(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams7;
    velocity_turn135in = m->velocity_turn135in;
    alpha_turn135in = m->alpha_turn135in;
    angle_turn135in = m->angle_turn135in;
    dist_turn135in_in = m->dist_turn135in_in;
    dist_turn135in_out = m->dist_turn135in_out;
}

static void apply_turn_d135out_mode7(void) {
    const ShortestRunModeParams_t *m = &shortestRunModeParams7;
    velocity_turn135out = m->velocity_turn135out;
    alpha_turn135out = m->alpha_turn135out;
    angle_turn135out = m->angle_turn135out;
    dist_turn135out_in = m->dist_turn135out_in;
    dist_turn135out_out = m->dist_turn135out_out;
}

void mode7() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: //
#if 0

            printf("Mode 7-0 Normal Turn.\n");

            // 直線
            acceleration_straight = 14222.2;
            acceleration_straight_dash = 25000;
            velocity_straight = 2300;

            // ターン
            velocity_turn90 = 1400;
            alpha_turn90 = 60000;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 35;
            val_offset_in = 1100;
            angle_turn_90 = 88;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            drive_fan(600);

            led_flash(20);

            half_sectionA(1400);
            turn_R90(0);
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;
#else
            printf("Mode 7-0: Turn/Straight Test.\n");
            g_test_mode_run = true;

            int sub = select_mode(0);

            const int idx_normal = 0;
            const int idx_diag = 7;

            switch (sub) {
            case 0:
                apply_case_params_mode7_idx(idx_normal);
                apply_turn_normal_mode7();
                {
                    const ShortestRunModeParams_t *pm0 = &shortestRunModeParams7;
                    drive_fan(pm0->fan_power);
                    log_init();
                    log_set_profile(LOG_PROFILE_OMEGA);
                    log_start(HAL_GetTick());
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 5;
                    path[1] = 300;
                    path[2] = 0;
                    run();
                    drive_fan(0);
                    log_stop();
                    printf("[mode7-case0-sub0] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                           (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                    while (1) {
                        if (ad_fr > WALL_BASE_FR) { log_print_omega_all(); break; }
                        else if (ad_fl > WALL_BASE_FL) { log_print_angle_all(); break; }
                        HAL_Delay(50);
                    }
                }
                break;
            case 3:
                apply_case_params_mode7_idx(idx_diag);
                printf("Loaded params: diag 45-in (mode7).\n");
                {
                    const ShortestRunModeParams_t *pm3 = &shortestRunModeParams7;
                    drive_fan(pm3->fan_power);
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 4;
                    path[1] = 701;
                    path[2] = 1000 + 1;
                    run();
                    drive_fan(0);
                }
                break;
            case 4:
                apply_case_params_mode7_idx(idx_diag);
                printf("Loaded params: diag 45-out (mode7).\n");
                {
                    const ShortestRunModeParams_t *pm4 = &shortestRunModeParams7;
                    drive_fan(pm4->fan_power);
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 4;
                    path[1] = 1000 + 1;
                    path[2] = 704;
                    path[3] = 1000 + 1;
                    run();
                    drive_fan(0);
                }
                break;
            case 5:
                apply_case_params_mode7_idx(idx_diag);
                printf("Loaded params: diag V90 (mode7).\n");
                {
                    const ShortestRunModeParams_t *pm5 = &shortestRunModeParams7;
                    drive_fan(pm5->fan_power);
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 4;
                    path[1] = 1000 + 1;
                    path[2] = 802;
                    path[3] = 1000 + 1;
                    run();
                    drive_fan(0);
                }
                break;
            case 6:
                apply_case_params_mode7_idx(idx_diag);
                printf("Loaded params: diag 135-in (mode7).\n");
                {
                    const ShortestRunModeParams_t *pm6 = &shortestRunModeParams7;
                    drive_fan(pm6->fan_power);
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 4;
                    path[1] = 901;
                    path[2] = 1000 + 1;
                    run();
                    drive_fan(0);
                }
                break;
            case 7:
                apply_case_params_mode7_idx(idx_diag);
                printf("Loaded params: diag 135-out (mode7).\n");
                {
                    const ShortestRunModeParams_t *pm7 = &shortestRunModeParams7;
                    drive_fan(pm7->fan_power);
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 4;
                    path[1] = 1000 + 1;
                    path[2] = 904;
                    path[3] = 1000 + 1;
                    run();
                    drive_fan(0);
                }
                break;
            case 1:
                apply_case_params_mode7_idx(idx_normal);
                apply_turn_large90_mode7();
                {
                    const ShortestRunModeParams_t *pm1 = &shortestRunModeParams7;
                    drive_fan(pm1->fan_power);
                    log_init();
                    log_set_profile(LOG_PROFILE_OMEGA);
                    log_start(HAL_GetTick());
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 4;
                    path[1] = 501;
                    path[2] = 200 + 1;
                    run();
                    drive_fan(0);
                    log_stop();
                    printf("[mode7-case0-sub1] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                           (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                    while (1) {
                        if (ad_fr > WALL_BASE_FR) { log_print_omega_all(); break; }
                        else if (ad_fl > WALL_BASE_FL) { log_print_angle_all(); break; }
                        HAL_Delay(50);
                    }
                }
                break;
            case 2:
                apply_case_params_mode7_idx(idx_normal);
                apply_turn_large180_mode7();
                {
                    const ShortestRunModeParams_t *pm2 = &shortestRunModeParams7;
                    drive_fan(pm2->fan_power);
                    log_init();
                    log_set_profile(LOG_PROFILE_OMEGA);
                    log_start(HAL_GetTick());
                    for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                    path[0] = 200 + 4;
                    path[1] = 502;
                    path[2] = 200 + 1;
                    run();
                    drive_fan(0);
                    log_stop();
                    printf("[mode7-case0-sub2] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
                           (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                    while (1) {
                        if (ad_fr > WALL_BASE_FR) { log_print_omega_all(); break; }
                        else if (ad_fl > WALL_BASE_FL) { log_print_angle_all(); break; }
                        HAL_Delay(50);
                    }
                }
                break;
            case 8:
                apply_case_params_mode7_idx(0);
                kp_wall = 0.0f;
                drive_fan(shortestRunModeParams7.fan_power);
                log_init();
                log_set_profile(LOG_PROFILE_DISTANCE);
                log_start(HAL_GetTick());
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                path[0] = 200 + 3;
                path[1] = 0;
                run();
                drive_fan(0);
                log_stop();
                printf("[mode7-case0-sub8] Press RIGHT FRONT for VELOCITY (FR>%u), LEFT FRONT for DISTANCE (FL>%u) ...\n",
                       (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                while (1) {
                    if (ad_fr > WALL_BASE_FR) { log_print_velocity_all(); break; }
                    else if (ad_fl > WALL_BASE_FL) { log_print_distance_all(); break; }
                    HAL_Delay(50);
                }
                led_flash(5);
                break;
            case 9:
                apply_case_params_mode7_idx(4);
                kp_wall = 0.0f;
                drive_fan(shortestRunModeParams7.fan_power);
                log_init();
                log_set_profile(LOG_PROFILE_VELOCITY);
                log_start(HAL_GetTick());
                for (int i = 0; i < ROUTE_MAX_LEN; i++) path[i] = 0;
                path[0] = 200 + 3;
                path[1] = 0;
                run();
                drive_fan(0);
                log_stop();
                printf("[mode7-case0-sub9] Press RIGHT FRONT for VELOCITY (FR>%u), LEFT FRONT for DISTANCE (FL>%u) ...\n",
                       (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
                while (1) {
                    if (ad_fr > WALL_BASE_FR) { log_print_velocity_all(); break; }
                    else if (ad_fl > WALL_BASE_FL) { log_print_distance_all(); break; }
                    HAL_Delay(50);
                }
                led_flash(5);
                break;
            default:
                printf("No sub-mode selected.\n");
                break;
            }

            g_test_mode_run = false;
            break;
#endif

        case 1: //
#if 0

            printf("Mode 7-1 Large Turn 90deg.\n");

            // 直線
            acceleration_straight = 14222.2;
            acceleration_straight_dash = 25000;
            velocity_straight = 2300;

            // 90°大回りターン
            velocity_l_turn_90 = 2200;
            alpha_l_turn_90 = 27500;
            angle_l_turn_90 = 85.0;
            dist_l_turn_out_90 = 38;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            drive_fan(600);

            led_flash(20);

            half_sectionA(2200);
            l_turn_R90(false);
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;
#else
            g_disable_front_wall_correction = true;
            g_disable_wall_end_correction = true;
            run_shortest(7, 1);
            g_disable_front_wall_correction = false;
            g_disable_wall_end_correction = false;
            break;
#endif

        case 2:
#if 0

            printf("Mode 7-2 Large Turn 180deg.\n");

            // 直線
            acceleration_straight = 14222.2;
            acceleration_straight_dash = 25000;
            velocity_straight = 2300;

            // 180°大回りターン
            velocity_l_turn_180 = 2200;
            alpha_l_turn_180 = 23000;
            angle_l_turn_180 = 176.8;
            dist_l_turn_out_180 = 50;

            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();

            led_flash(5);

            drive_fan(600);

            led_flash(20);

            half_sectionA(2200);
            l_turn_R180(0);
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;
#else

            // g_disable_front_wall_correction = true;
            // g_disable_wall_end_correction = true;
            run_shortest(7, 2);
            // g_disable_front_wall_correction = false;
            // g_disable_wall_end_correction = false;
            break;
#endif

        case 3:
            run_shortest(7, 3);
            break;

        case 4:
            run_shortest(7, 4);
            break;

        case 5:
            run_shortest(7, 5);
            break;

        case 6:
            run_shortest(7, 6);
            break;

        case 7:
            run_shortest(7, 7);
            break;

        case 8:
            run_shortest(7, 8);
            break;

        case 9:
            run_shortest(7, 9);
            break;

        default:
            break;
        }
    }
}
