/*
 * test_mode.c
 *
 *  Created on: Dec 23, 2023
 *      Author: yuho-
 */

#include "global.h"
#include "solver.h"
#include "sensor_distance.h"
#include "distance_params.h"
#include "logging.h"
#include "build_info.h"

// drive.c と同じ条件でPWM反転するための定義（DIR==Lowで反転が既定）
#ifndef PWM_INVERT_DIR_LEVEL
#define PWM_INVERT_DIR_LEVEL 0
#endif

#define IMU_DIAG_CAPTURE_MS 5000u
#define IMU_DIAG_SAMPLE_INTERVAL_MS 10u
#define IMU_DIAG_MAX_SAMPLES (IMU_DIAG_CAPTURE_MS / IMU_DIAG_SAMPLE_INTERVAL_MS + 1u)

typedef struct {
    uint32_t t_ms;
    float omega_z_raw;
    float omega_z_true;
    float real_omega;
    float imu_angle;
    float real_angle;
    float accel_y_true;
} ImuDiagSample_t;

static ImuDiagSample_t s_imu_diag_samples[IMU_DIAG_MAX_SAMPLES];

static void imu_diag_capture_and_print_csv(uint16_t fan_duty) {
    printf("[IMU_DIAG] start: fan_duty=%u, capture=%lums, dt=%lums\n",
           (unsigned)fan_duty,
           (unsigned long)IMU_DIAG_CAPTURE_MS,
           (unsigned long)IMU_DIAG_SAMPLE_INTERVAL_MS);

    drive_fan(0);
    drive_stop();
    drive_disable_motor();
    drive_reset_before_run();

    if (fan_duty > 0) {
        drive_fan(fan_duty);
        HAL_Delay(2000);
    } else {
        HAL_Delay(500);
    }

    IMU_GetOffset();
    IMU_angle = 0.0f;
    real_angle = 0.0f;
    target_angle = 0.0f;

    uint32_t t0 = HAL_GetTick();
    uint16_t n = 0;
    while ((HAL_GetTick() - t0) < IMU_DIAG_CAPTURE_MS && n < IMU_DIAG_MAX_SAMPLES) {
        ImuDiagSample_t *s = &s_imu_diag_samples[n++];
        s->t_ms = HAL_GetTick() - t0;
        s->omega_z_raw = omega_z_raw;
        s->omega_z_true = omega_z_true;
        s->real_omega = real_omega;
        s->imu_angle = IMU_angle;
        s->real_angle = real_angle;
        s->accel_y_true = accel_y_true;
        HAL_Delay(IMU_DIAG_SAMPLE_INTERVAL_MS);
    }

    drive_fan(0);

    printf("=== IMU DIAG LOG (CSV) ===\n");
    printf("#fw_target=%s\n", FW_TARGET);
    printf("#fw_build_type=%s\n", FW_BUILD_TYPE);
    printf("#fw_build_time_utc=%s\n", FW_BUILD_TIME_UTC);
    printf("#fw_git_describe=%s\n", FW_GIT_DESCRIBE);
    printf("#fw_git_sha=%s\n", FW_GIT_SHA);
    printf("#fw_git_branch=%s\n", FW_GIT_BRANCH);
    printf("#fw_git_dirty=%d\n", (int)FW_GIT_DIRTY);
    printf("#mm_columns=t_ms,omega_z_raw,omega_z_true,real_omega,imu_angle,real_angle,accel_y_true\n");
    for (uint16_t i = 0; i < n; i++) {
        const ImuDiagSample_t *s = &s_imu_diag_samples[i];
        printf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
               (unsigned long)s->t_ms,
               s->omega_z_raw,
               s->omega_z_true,
               s->real_omega,
               s->imu_angle,
               s->real_angle,
               s->accel_y_true);
    }
    printf("=== END IMU DIAG LOG ===\n");
}

//============================================================
// Front sensors averaging helper (blocking)
// samples: number of samples, interval_ms: delay between samples
// out: average FR/FL (uint16)
//============================================================
static void sample_front_avg(int samples, int interval_ms, uint16_t *avg_fr, uint16_t *avg_fl)
{
    uint32_t sum_fr = 0, sum_fl = 0;
    for (int i = 0; i < samples; i++) {
        sum_fr += (uint32_t)ad_fr;
        sum_fl += (uint32_t)ad_fl;
        HAL_Delay(interval_ms);
    }
    if (avg_fr) *avg_fr = (uint16_t)(sum_fr / (uint32_t)samples);
    if (avg_fl) *avg_fl = (uint16_t)(sum_fl / (uint32_t)samples);
}

// Capture helper for case7 (C version)
static void capture_anchor_point(int idx, const char *label,
                                 const float y_true[3],
                                 float x_est_fl[3], float x_est_fr[3], float x_est_fsum[3],
                                 int samples, int interval_ms)
{
    printf("Place front at %s anchor (%.1fmm). Press PUSH to start...\n",
           label, (double)y_true[idx]);
    // wait for push (active-low)
    while (HAL_GPIO_ReadPin(PUSH_IN_1_GPIO_Port, PUSH_IN_1_Pin) != 0) {
        HAL_Delay(50);
    }
    buzzer_enter(900);
    uint16_t afr = 0, afl = 0;
    sample_front_avg(samples, interval_ms, &afr, &afl);
    uint32_t sum = (uint32_t)afr + (uint32_t)afl;
    if (sum > 0xFFFFu) sum = 0xFFFFu;
    float est_fl = sensor_distance_from_fl_unwarped(afl);
    float est_fr = sensor_distance_from_fr_unwarped(afr);
    float est_fsum = sensor_distance_from_fsum_unwarped((uint16_t)sum);
    x_est_fl[idx] = est_fl;
    x_est_fr[idx] = est_fr;
    x_est_fsum[idx] = est_fsum;
    printf("[Captured %s] FR=%u -> %.2fmm(est), FL=%u -> %.2fmm(est), SUM=%u -> %.2fmm(est)\n",
           label, (unsigned)afr, (double)est_fr, (unsigned)afl, (double)est_fl, (unsigned)sum, (double)est_fsum);
    led_flash(2);
    // wait for release to avoid immediate next trigger
    while (HAL_GPIO_ReadPin(PUSH_IN_1_GPIO_Port, PUSH_IN_1_Pin) == 0) {
        HAL_Delay(30);
    }
}

void test_mode() {

    int mode = 0;

    while (1) {
        mode = select_mode(mode);

        switch (mode) {
        case 0:

            printf("Test Mode 0: Inner loop tune (velocity/omega).\n");

            {
                int axis = 0;
                int set = 0;
                int pattern = 0;

                printf("Select axis: 0=velocity, 1=omega\n");
                axis = select_mode(axis);
                if (axis != 0 && axis != 1) {
                    axis = 0;
                }

                printf("Select set: 0=500, 1=1000, 2=2000 (fan ON for 1/2)\n");
                set = select_mode(set);
                if (set < 0) set = 0;
                if (set > 2) set = 2;

                printf("Select pattern: 0=step, 1=ramp(triangle), 2=trapezoid\n");
                pattern = select_mode(pattern);
                if (pattern < 0) pattern = 0;
                if (pattern > 2) pattern = 2;

                // 初期化
                drive_reset_before_run();
                IMU_GetOffset();

                g_test_mode_run = true;
                g_disable_front_wall_correction = false;

                // モータ有効化
                drive_enable_motor();
                MF.FLAG.RUNNING = 1;

                // ファン（set 0 は非吸引、それ以外は100%）
                if (set == 0) {
                    drive_fan(0);
                } else {
                    drive_fan(300);
                    HAL_Delay(1500);
                }

                // ログ
                log_init();
                if (axis == 0) {
                    log_set_profile(LOG_PROFILE_VELOCITY);
                } else {
                    log_set_profile(LOG_PROFILE_OMEGA);
                }
                log_start(HAL_GetTick());

                inner_tune_test_clear_done();
                inner_tune_test_start((uint8_t)axis, (uint8_t)set, (uint8_t)pattern);

                // 完了待ち（割り込み側で800ms or 安全停止）
                while (!inner_tune_test_is_done()) {
                    HAL_Delay(10);
                }

                // 停止処理
                log_stop();
                drive_fan(0);
                MF.FLAG.RUNNING = 0;
                drive_stop();
                drive_disable_motor();
                led_flash(30);

                // ログ出力
                if (axis == 0) {
                    printf("[test_mode-case0] Press RIGHT FRONT for VELOCITY (FR>%u), LEFT FRONT for DISTANCE (FL>%u) ...\n",
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
                } else {
                    printf("[test_mode-case0] Press RIGHT FRONT for OMEGA (FR>%u), LEFT FRONT for ANGLE (FL>%u) ...\n",
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
                }

                g_test_mode_run = false;
                led_flash(5);
            }

            break;

        case 1:

            printf("Test Mode 1 IMU Check.\n");

            {
                int sub = 0;
                printf("Select sub: 0=live monitor, 1=CSV drift(fan OFF), 2=CSV drift(fan ON)\n");
                sub = select_mode(sub);
                if (sub < 0 || sub > 2) {
                    sub = 0;
                }

                if (sub == 0) {
                    drive_variable_reset();
                    IMU_GetOffset();

                    drive_fan(250);

                    while (1) {

                        printf("omega: %.3f [deg/s] ,  ", real_omega);

                        printf("angle: %.3f [deg] ,  ", real_angle);

                        printf("accl_x: %.3f ,  ", accel_x_true);

                        printf("accl_y: %.3f\n", accel_y_true);

                        HAL_Delay(100);
                    }
                } else {
                    uint16_t fan_duty = (sub == 2) ? 500u : 0u;
                    imu_diag_capture_and_print_csv(fan_duty);
                    led_flash(5);
                }
            }

            break;

        case 2:
            printf("Test Mode 2 Encoder Check.\n");

            drive_variable_reset();

            while (1) {

                printf("speed L R: %.3f %.3f [mm/s] ,  ", encoder_speed_l,
                       encoder_speed_r);
                printf("distance: %.3f [mm]\n",
                       (encoder_distance_l + encoder_distance_r) * 0.5);

                HAL_Delay(100);
            }

            break;
        case 3:
            printf("Test Mode 3 Sensor AD Value Check (+ front distance).\n");

            while (1) {
                float d_fr = sensor_distance_from_fr(ad_fr);
                float d_fl = sensor_distance_from_fl(ad_fl);
                printf("R: %d, L: %d, FR: %d (%.1fmm), FL: %d (%.1fmm), BAT: %d\n",
                       ad_r, ad_l, ad_fr, d_fr, ad_fl, d_fl, ad_bat);

                HAL_Delay(300);
            }
            break;

        case 4:
            printf("Test Mode 4: Fan noise check (IMU/Encoder).\n");

            {
                int sub = 0;
                printf("Select sub: 0=keep fan running, 1=capture noise log\n");
                sub = select_mode(sub);
                if (sub != 0 && sub != 1) {
                    sub = 0;
                }

                if (sub == 0) {
                    led_flash(10);

                    IMU_GetOffset();
                    drive_enable_motor();

                    drive_variable_reset();

                    real_angle = 0;
                    IMU_angle = 0;
                    target_angle = 0;

                    real_distance = 0;
                    encoder_distance_r = 0;
                    encoder_distance_l = 0;

                    led_write(1, 1,0);

                    drive_start();

                    drive_fan(500);

                } else {
                    const uint16_t fan_power = 700;
                    const uint32_t settle_ms = 2000;
                    const uint32_t capture_ms = 3000;

                    drive_fan(0);
                    drive_stop();
                    drive_disable_motor();

                    drive_reset_before_run();
                    IMU_GetOffset();

                    g_test_mode_run = true;
                    MF.FLAG.OVERRIDE = 1;

                    drive_variable_reset();
                    real_angle = 0;
                    IMU_angle = 0;
                    target_angle = 0;
                    real_distance = 0;
                    encoder_distance_r = 0;
                    encoder_distance_l = 0;

                    drive_fan(fan_power);
                    HAL_Delay(settle_ms);

                    log_init();
                    log_set_profile(LOG_PROFILE_CUSTOM);
                    log_start(HAL_GetTick());

                    uint32_t t0 = HAL_GetTick();
                    while ((HAL_GetTick() - t0) < capture_ms) {
                        HAL_Delay(10);
                    }

                    log_stop();
                    drive_fan(0);

                    printf("[test_mode-case4] Press RIGHT FRONT for NOISE (FR>%u) ...\n",
                           (unsigned)WALL_BASE_FR);
                    while (1) {
                        if (ad_fr > WALL_BASE_FR) {
                            log_print_noise_all();
                            break;
                        }
                        HAL_Delay(50);
                    }

                    g_test_mode_run = false;
                    MF.FLAG.OVERRIDE = 0;
                    led_flash(5);
                }
            }

            break;
        case 5:
            printf("Test Mode 5: Motor Sequence (DIR x DUTY, BOTH wheels, 2s each).\n");

            // 割り込み内の速度制御・drive_motor()呼び出しを無視
            MF.FLAG.OVERRIDE = 1;

            // モータドライバを有効化（STBYを上げ、PWMを開始）
            drive_enable_motor();

            // シーケンス条件（両輪のみ）
            const uint8_t dirs[2] = {FORWARD, BACK};
            const uint8_t duty_pct_list[3] = {10, 30, 60};

            while (1) {
                for (uint8_t di = 0; di < 2; di++) {
                    uint8_t dir_code = dirs[di];

                    // 安全のため一旦アイドルへ（IN1==IN2）
                    uint32_t arr_idle = __HAL_TIM_GET_AUTORELOAD(&htim2);
                    GPIO_PinState cur_dir_l = HAL_GPIO_ReadPin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin);
                    GPIO_PinState cur_dir_r = HAL_GPIO_ReadPin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin);
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (cur_dir_l == GPIO_PIN_SET) ? arr_idle : 0u);
                    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (cur_dir_r == GPIO_PIN_SET) ? arr_idle : 0u);
                    HAL_Delay(20);

                    // 進行方向を設定
                    drive_set_dir(dir_code);
                    HAL_Delay(20);

                    for (uint8_t pi = 0; pi < 3; pi++) {
                        uint8_t duty_pct = duty_pct_list[pi];

                            const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
                            uint32_t duty_counts = (arr * (uint32_t)duty_pct) / 100u;
                            if (duty_counts == 0) duty_counts = 1;
                            if (duty_counts >= arr) duty_counts = arr - 1u;

                            // DIR実レベル読み取り（アイドル設定に使用）
                            GPIO_PinState dir_l = HAL_GPIO_ReadPin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin);
                            GPIO_PinState dir_r = HAL_GPIO_ReadPin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin);
                            uint8_t dir_high_l = (dir_l == GPIO_PIN_SET) ? 1 : 0;
                            uint8_t dir_high_r = (dir_r == GPIO_PIN_SET) ? 1 : 0;

                            // 実機挙動: DIRピンがHighのときは実効Dutyが「逆数」になる
                            // → DIR==High の側は CCR = ARR - duty_counts（Low期間がduty%）
                            // → DIR==Low  の側は CCR = duty_counts
                            uint32_t ccr_l = 0, ccr_r = 0;
                            ccr_l = dir_high_l ? (arr - duty_counts) : duty_counts;
                            ccr_r = dir_high_r ? (arr - duty_counts) : duty_counts;

                            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_l);
                            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ccr_r);

                            // 進行状況を出力
                            const char *dir_str = (dir_code == FORWARD) ? "FWD" : "BACK";
                            const char *side_str = "BOTH";
                            printf("[DIR=%s][SIDE=%s][DUTY=%u%%] CCR_L=%lu, CCR_R=%lu (ARR=%lu)\n",
                                   dir_str, side_str, (unsigned)duty_pct,
                                   (unsigned long)ccr_l, (unsigned long)ccr_r, (unsigned long)arr);

                            // 2秒維持
                            HAL_Delay(2000);
                    }
                }
            }

            // breakには到達しない
            break;
        case 6:

            printf("Test Mode 6 Circuit.\n");

            // 直線
            acceleration_straight = 3555.6;
            acceleration_straight_dash = 10000; // 5000
            velocity_straight = 3000;
            // ターン
            velocity_turn90 = 700;
            alpha_turn90 = 10900;
            acceleration_turn = 0;
            dist_offset_in = 10;
            dist_offset_out = 32;
            val_offset_in = 700;
            angle_turn_90 = 88.5;
            // 90°大回りターン
            velocity_l_turn_90 = 850;
            alpha_l_turn_90 = 3000;
            angle_l_turn_90 = 89;
            dist_l_turn_out_90 = 26;
            // 壁制御とケツ当て
            kp_wall = 0.05;
            duty_setposition = 40;

            velocity_interrupt = 0;

            led_flash(10);

            drive_variable_reset();
            IMU_GetOffset();
            drive_enable_motor();
            led_flash(5);
            get_base();
            led_flash(5);

            first_sectionA();

            // 1回目の直線
            run_straight(10, velocity_straight, 0);
            run_straight(8, velocity_straight, 0);
            run_straight(10, velocity_l_turn_90, 0);

            // 1回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(4, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 1回目のターン
            l_turn_R90(false);

            // 2回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 2回目のターン
            l_turn_R90(false);

            // 3回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 3回目のターン
            l_turn_R90(false);

            // 4回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 4回目のターン
            l_turn_R90(false);

            // 5回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 5回目のターン
            l_turn_R90(false);

            // 6回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 6回目のターン
            l_turn_R90(false);

            // 7回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 7回目のターン
            l_turn_R90(false);

            // 8回目の直線
            run_straight(12, velocity_straight, 0);
            run_straight(2, velocity_straight, 0);
            run_straight(12, velocity_l_turn_90, 0);

            // 8回目のターン
            l_turn_R90(false);

            half_sectionD(0);

            led_flash(5);
            drive_stop();

            break;

        case 7:
            printf("Test Mode 7: Front distance 3-point warp calibration.\n");
            printf("Anchors (mm): %.1f, %.1f, %.1f\n",
                   (double)SENSOR_WARP_ANCHOR0_MM,
                   (double)SENSOR_WARP_ANCHOR1_MM,
                   (double)SENSOR_WARP_ANCHOR2_MM);

            // Safety: stop fan and any motion
            drive_fan(0);
            velocity_interrupt = 0; omega_interrupt = 0;
            MF.FLAG.CTRL = 0; // disable wall control

            const float y_true[3] = {
                SENSOR_WARP_ANCHOR0_MM,
                SENSOR_WARP_ANCHOR1_MM,
                SENSOR_WARP_ANCHOR2_MM
            };
            float x_est_fl[3] = {0}, x_est_fr[3] = {0}, x_est_fsum[3] = {0};
            const int SAMPLES = 400;     // ~2s at 5ms
            const int INTERVAL = 5;      // ms

            // Capture near/mid/far
            capture_anchor_point(0, "NEAR", y_true, x_est_fl, x_est_fr, x_est_fsum, SAMPLES, INTERVAL);
            capture_anchor_point(1, "MID",  y_true, x_est_fl, x_est_fr, x_est_fsum, SAMPLES, INTERVAL);
            capture_anchor_point(2, "FAR",  y_true, x_est_fl, x_est_fr, x_est_fsum, SAMPLES, INTERVAL);

            // Apply warps
            sensor_distance_set_warp_fl_3pt(x_est_fl, y_true);
            sensor_distance_set_warp_fr_3pt(x_est_fr, y_true);
            sensor_distance_set_warp_front_sum_3pt(x_est_fsum, y_true);
            printf("Applied 3-point warp for FL/FR/FSUM.\n");
            buzzer_beep(1200);

            // Persist to Flash
            HAL_StatusTypeDef stw = distance_params_save(x_est_fl, y_true,
                                                         x_est_fr, y_true,
                                                         x_est_fsum, y_true);
            if (stw == HAL_OK) {
                printf("Saved distance warp params to Flash (Sector 9).\n");
                buzzer_beep(1800);
            } else {
                printf("Failed to save distance warp params. HAL=%d\n", stw);
                buzzer_beep(3000);
            }

            // Quick verification readout loop (optional): press FR to print 10 lines, FL to exit
            printf("Verification: Press RIGHT FRONT (FR>%u) to print 10 samples, LEFT FRONT (FL>%u) to exit.\n",
                   (unsigned)WALL_BASE_FR, (unsigned)WALL_BASE_FL);
            while (1) {
                if (ad_fr > WALL_BASE_FR) {
                    for (int i=0;i<10;i++) {
                        float dfr = sensor_distance_from_fr(ad_fr);
                        float dfl = sensor_distance_from_fl(ad_fl);
                        uint32_t s = (uint32_t)ad_fr + (uint32_t)ad_fl;
                        if (s > 0xFFFFu) s = 0xFFFFu;
                        float df = sensor_distance_from_fsum((uint16_t)s);
                        printf("[Now] FR=%u(%.2fmm), FL=%u(%.2fmm), SUM=%u(%.2fmm)\n",
                               (unsigned)ad_fr, (double)dfr, (unsigned)ad_fl, (double)dfl, (unsigned)s, (double)df);
                        HAL_Delay(200);
                    }
                } else if (ad_fl > WALL_BASE_FL) {
                    break;
                }
                HAL_Delay(50);
            }

            led_flash(3);
            break;

        case 8:

            printf("Test Mode 8: Side Wall Baseline Auto-Calibration (L/R).\n");
            printf("Place the robot at cell center with walls on BOTH sides.\n");
            printf("Fan will be stopped to reduce noise. Sampling will start shortly...\n");

            // 安定化のため少し待つ
            drive_fan(0);
            HAL_Delay(300);

            // サンプリング条件
            const int SAMPLE_COUNT = 600; // 約3秒 @5ms
            const int SAMPLE_INTERVAL_MS = 5;

            uint32_t sum_l = 0;
            uint32_t sum_r = 0;

            // 初期値を表示
            printf("Initial AD: L=%u, R=%u (thresholds L=%u, R=%u)\n",
                   (unsigned)ad_l, (unsigned)ad_r,
                   (unsigned)WALL_BASE_L, (unsigned)WALL_BASE_R);

            // 計測
            for (int i = 0; i < SAMPLE_COUNT; i++) {
                sum_l += (uint32_t)ad_l;
                sum_r += (uint32_t)ad_r;
                HAL_Delay(SAMPLE_INTERVAL_MS);
            }

            uint16_t avg_l = (uint16_t)(sum_l / (uint32_t)SAMPLE_COUNT);
            uint16_t avg_r = (uint16_t)(sum_r / (uint32_t)SAMPLE_COUNT);

            // ランタイム基準値に適用
            base_l = avg_l;
            base_r = avg_r;

            printf("Measured side baselines -> L=%u, R=%u\n",
                   (unsigned)base_l, (unsigned)base_r);

            // フラッシュへ保存
            HAL_StatusTypeDef st2 = sensor_params_save_to_flash();
            if (st2 == HAL_OK) {
                printf("Saved to Flash successfully.\n");
                buzzer_beep(1200);
            } else {
                printf("Failed to save to Flash. HAL status=%d\n", st2);
                buzzer_beep(3000);
            }

            // 完了合図
            led_flash(5);
            break;

        case 9:

            printf("Test Mode 9 Circuit.\n");

            HAL_StatusTypeDef st = sensor_recalibrate_and_save();
                if (st == HAL_OK) {
                        printf("Sensor parameters saved to Flash successfully.\n");
                        buzzer_beep(1200);
                    } else {
                        printf("Failed to save sensor parameters. HAL status=%d\n", st);
                        buzzer_beep(3000);
                    }
            break;
        }
    }
}
