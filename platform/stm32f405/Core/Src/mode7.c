/*
 * mode7.c
 *
 *  Created on: Aug 17, 2024
 *      Author: yuho-
 */

#include "global.h"
#include "../Inc/shortest_run_params.h"
#include "../Inc/run.h"

void mode7() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0: //

            printf("Mode 6-0 Normal Turn.\n");

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
            half_sectionU();
            turn_R90(0);
            half_sectionD(0);

            led_flash(5);

            drive_fan(0);
            led_flash(5);

            drive_stop();

            break;

        case 1: //

            printf("Mode 6-1 Large Turn 90deg.\n");

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

        case 2:

            printf("Mode 6-2 Large Turn 180deg.\n");

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

        case 3:
            printf("Mode 6-3.\n");
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
        }
    }
}
