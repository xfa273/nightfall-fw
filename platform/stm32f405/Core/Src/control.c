/*
 * control.c
 *
 *  Created on: 2023/07/29
 *      Author: yuho-
 */

#include "global.h"
#include <math.h>

/*エンコーダから速度と位置を取得する*/
void read_encoder(void) {
     static float s_real_velocity_f = 0.0f;
     static uint8_t s_real_velocity_f_inited = 0;

    // エンコーダのパルスカウンタを取得
    encoder_count_r = TIM8->CNT;
    encoder_count_l = TIM4->CNT;

    // パルスカウンタの増加量を速度として取得
    encoder_speed_r = encoder_count_r - 30000;
    encoder_speed_l = encoder_count_l - 30000;

    // マイナス速度用にパルスカウントを30000にセット
    TIM8->CNT = 30000;
    TIM4->CNT = 30000;

    encoder_speed_r = -encoder_speed_r; // 右の速度の符号を補正

    // 速度の換算 → [mm/s]
    // 新機体はエンコーダがホイール側搭載のため減速比は不要
    // Cpr_wheel = 400[PPR] × 4[逓倍] = 1600[count/rev]
    // rev/s = (counts / g_ctrl_dt) * (1 / Cpr_wheel)
    // mm/s  = rev/s * (π * D_TIRE)
    const float Cpr_wheel = 1600.0f;
    const float scale = (D_TIRE * 3.1415f) / (Cpr_wheel * g_ctrl_dt);
    encoder_speed_r = encoder_speed_r * scale;
    encoder_speed_l = encoder_speed_l * scale;

    // 回転方向を補正
    encoder_speed_r = DIR_ENC_R * encoder_speed_r;
    encoder_speed_l = DIR_ENC_L * encoder_speed_l;

    // 走行距離カウンタを加算（可変制御周期）
    encoder_distance_r += encoder_speed_r * g_ctrl_dt;
    encoder_distance_l += encoder_speed_l * g_ctrl_dt;

    // 並進のPID制御用に格納
    const float real_velocity_raw = (encoder_speed_r + encoder_speed_l) * 0.5f;
    if (!s_real_velocity_f_inited) {
        s_real_velocity_f = real_velocity_raw;
        s_real_velocity_f_inited = 1;
    } else {
        float alpha = g_ctrl_dt / (VELOCITY_LPF_TAU + g_ctrl_dt);
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;
        s_real_velocity_f = s_real_velocity_f + alpha * (real_velocity_raw - s_real_velocity_f);
    }
    real_velocity = s_real_velocity_f;
    real_distance = (encoder_distance_r + encoder_distance_l) * 0.5;

    // パルスカウントを保存
    previous_encoder_count_r = encoder_count_r;
    previous_encoder_count_l = encoder_count_l;
}

/*IMUから角速度と角度を取得する*/
void read_IMU(void) {
    static float s_real_omega_f = 0.0f;
    static uint8_t s_real_omega_f_inited = 0;

    // 時計回りが正
    IMU_DataUpdate();
    const float real_omega_raw = -omega_z_true * KP_IMU;
    if (!s_real_omega_f_inited) {
        s_real_omega_f = real_omega_raw;
        s_real_omega_f_inited = 1;
    } else {
        float alpha = g_ctrl_dt / (OMEGA_LPF_TAU + g_ctrl_dt);
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;
        s_real_omega_f = s_real_omega_f + alpha * (real_omega_raw - s_real_omega_f);
    }
    real_omega = s_real_omega_f;
    IMU_angle += omega_z_true * g_ctrl_dt;
    real_angle = IMU_angle;
    IMU_acceleration = accel_y_true * 1000;
}

/*並進の積算計算*/
void calculate_translation(void) {
    // 設定された加速度から並進速度を計算
    velocity_interrupt += acceleration_interrupt * g_ctrl_dt;

    if (velocity_profile_clamp_enabled && acceleration_interrupt != 0.0f) {
        if (acceleration_interrupt > 0.0f) {
            if (velocity_interrupt > velocity_profile_target) {
                velocity_interrupt = velocity_profile_target;
            }
        } else {
            if (velocity_interrupt < velocity_profile_target) {
                velocity_interrupt = velocity_profile_target;
            }
        }
    }

    // 並進速度から目標位置を計算
    target_distance += velocity_interrupt * g_ctrl_dt;
}

/*回転の積算計算*/
void calculate_rotation(void) {
    // 設定された角加速度から角速度を計算
    omega_interrupt += alpha_interrupt * g_ctrl_dt;

    /*
    omega_interrupt += wall_control * 0.01;
    if (wall_control != 0) {
        omega_integral = 0;
    }
    */

    // 角速度から角度を計算
    target_angle += omega_interrupt * g_ctrl_dt;
}

/*並進速度のPID制御*/
void velocity_PID(void) {
    // 速度フィードバック: 目標速度（distance_PIDで算出） - 実測速度
    // target_velocity は distance_PID() 内で更新される
    velocity_error = target_velocity - real_velocity;

    if (velocity_error > 10000 || velocity_error < -10000) {
        MF.FLAG.FAILED = 1;
    }

    // I項（単純積分）
    float v_i_next = velocity_integral + velocity_error;

    // D項
    velocity_error_error = velocity_error - previous_velocity_error;

    // 吸引ON/OFFでゲイン切替（v1最終の値を params.h に定義）
    const float kp_v = MF.FLAG.SUCTION ? KP_VELOCITY_FAN_ON : KP_VELOCITY_FAN_OFF;
    const float ki_v = MF.FLAG.SUCTION ? KI_VELOCITY_FAN_ON : KI_VELOCITY_FAN_OFF;
    const float kd_v = MF.FLAG.SUCTION ? KD_VELOCITY_FAN_ON : KD_VELOCITY_FAN_OFF;

    // モータ制御量を計算（純PID）
    if (CTRL_ENABLE_ANTI_WINDUP) {
        const float out_candidate = kp_v * velocity_error + ki_v * v_i_next + kd_v * velocity_error_error;
        const float lim = CTRL_OUTPUT_MAX;
        const int would_saturate = (fabsf(out_candidate) > lim) ? 1 : 0;
        const int drives_further = ((out_candidate > 0.0f && velocity_error > 0.0f) ||
                                   (out_candidate < 0.0f && velocity_error < 0.0f)) ? 1 : 0;
        if (!(would_saturate && drives_further)) {
            velocity_integral = v_i_next;
        }
    } else {
        velocity_integral = v_i_next;
    }

    out_translation =   kp_v * velocity_error
                      + ki_v * velocity_integral
                      + kd_v * velocity_error_error;

    // 並進速度の偏差を保存
    previous_velocity_error = velocity_error;
}

/*並進距離のPID制御*/
void distance_PID(void) {
    // 位置PID（距離誤差から目標速度を生成）
    static uint16_t s_div = 0;
    static float s_v_fb = 0.0f;

    // 誤差
    distance_error = target_distance - real_distance;

    // 目標速度（プロファイル速度 + 位置PIDの補正）: 吸引ON/OFFでゲイン切替
    const float kp_d = MF.FLAG.SUCTION ? KP_DISTANCE_FAN_ON : KP_DISTANCE_FAN_OFF;
    const float ki_d = MF.FLAG.SUCTION ? KI_DISTANCE_FAN_ON : KI_DISTANCE_FAN_OFF;
    const float kd_d = MF.FLAG.SUCTION ? KD_DISTANCE_FAN_ON : KD_DISTANCE_FAN_OFF;

    s_div++;
    if (s_div >= (uint16_t)CTRL_DISTANCE_OUTER_DIV) {
        s_div = 0;
        // I項
        distance_integral += distance_error;
        // D項
        distance_error_error = distance_error - previous_distance_error;
        // 誤差履歴更新
        previous_distance_error = distance_error;

        s_v_fb = (kp_d * distance_error)
               + (ki_d * distance_integral)
               + (kd_d * distance_error_error);
    }

    target_velocity = velocity_interrupt + s_v_fb;
}

/*角速度のPID制御*/
void omega_PID(void) {
    const int angle_outer_enabled = (((KP_ANGLE != 0.0f) || (KI_ANGLE != 0.0f) || (KD_ANGLE != 0.0f)) ? 1 : 0);
    const float omega_outer = (angle_outer_enabled ? target_omega : 0.0f);
    const float omega_ref = omega_interrupt + omega_outer;

    omega_error = omega_ref - real_omega + wall_control + diagonal_control + kushi_control;

    float o_i_next = omega_integral + omega_error;

    // D項（角速度誤差の差分）
    omega_error_error = omega_error - previous_omega_error;

    // 吸引ON/OFFでゲイン切替
    const float kp_o = MF.FLAG.SUCTION ? KP_OMEGA_FAN_ON : KP_OMEGA_FAN_OFF;
    const float ki_o = MF.FLAG.SUCTION ? KI_OMEGA_FAN_ON : KI_OMEGA_FAN_OFF;
    const float kd_o = MF.FLAG.SUCTION ? KD_OMEGA_FAN_ON : KD_OMEGA_FAN_OFF;

    // モータ制御量を計算
    if (CTRL_ENABLE_ANTI_WINDUP) {
        const float out_candidate = kp_o * omega_error + ki_o * o_i_next + kd_o * omega_error_error;
        const float lim = CTRL_OUTPUT_MAX;
        const int would_saturate = (fabsf(out_candidate) > lim) ? 1 : 0;
        const int drives_further = ((out_candidate > 0.0f && omega_error > 0.0f) ||
                                   (out_candidate < 0.0f && omega_error < 0.0f)) ? 1 : 0;
        if (!(would_saturate && drives_further)) {
            omega_integral = o_i_next;
        }
    } else {
        omega_integral = o_i_next;
    }

    out_rotate = kp_o * omega_error + ki_o * omega_integral + kd_o * omega_error_error;

    // 角速度の偏差を保存
    previous_omega_error = omega_error;
}

/*角度のPID制御*/
void angle_PID(void) {
    static uint16_t s_div = 0;

    // ゲインが全て0なら外側角度ループを無効化（従来の角速度制御に戻す）
    if ((KP_ANGLE == 0.0f) && (KI_ANGLE == 0.0f) && (KD_ANGLE == 0.0f)) {
        target_omega = 0.0f;
        s_div = 0;
        angle_error = 0.0f;
        previous_angle_error = 0.0f;
        angle_error_error = 0.0f;
        angle_integral = 0.0f;
        return;
    }

    // P項
    angle_error = (-target_angle) - real_angle;

    s_div++;
    if (s_div >= (uint16_t)CTRL_ANGLE_OUTER_DIV) {
        s_div = 0;

        // I項
        angle_integral += angle_error;

        // D項
        angle_error_error = angle_error - previous_angle_error;

        // 角度の偏差を保存
        previous_angle_error = angle_error;

        // 目標角速度を計算
        target_omega = -(KP_ANGLE * angle_error + KI_ANGLE * angle_integral + KD_ANGLE * angle_error_error);
    }
}

/*壁のPID制御*/
void wall_PID(void) {

    // Priority-2: low-pass filter state for wall error
    static uint8_t s_prev_ctrl = 0;
    static float wall_err_f = 0.0f;
    // Priority-3: slew-rate limit state for wall control
    static float wall_ctrl_prev = 0.0f;

    // 制御フラグがあれば制御
    if (MF.FLAG.CTRL) {

        // Reset filter on rising edge of CTRL enable
        if (!s_prev_ctrl) {
            wall_err_f = 0.0f;
            wall_ctrl_prev = 0.0f;
            previous_ad_r = ad_r;
            previous_ad_l = ad_l;
        }

        float wall_error = 0;
        uint16_t wall_thr_r;
        uint16_t wall_thr_l;
        float sense_diff_r;
        float sense_diff_l;

        sense_diff_r = ad_r - previous_ad_r;
        sense_diff_l = ad_l - previous_ad_l;

        if (fabsf(sense_diff_r) > WALL_DIFF_THR) {
            wall_thr_r = WALL_BASE_R + 30; //30
        } else {
            wall_thr_r = WALL_BASE_R;
        }
        if (fabsf(sense_diff_l) > WALL_DIFF_THR) {
            wall_thr_l = WALL_BASE_L + 30;
        } else {
            wall_thr_l = WALL_BASE_L;
        }

        if (ad_r > wall_thr_r && ad_l > wall_thr_l) {
            // 左右壁が両方ある場合
            wall_error = (ad_l - base_l) - (ad_r - base_r);
            latest_wall_error = wall_error;
        } else if (ad_r < wall_thr_r && ad_l < wall_thr_l) {
            // 左右壁が両方ない場合
            wall_error = 0;
            latest_wall_error = wall_error;
        } else if (ad_r > wall_thr_r && ad_l < wall_thr_l) {
            // 右壁のみある場合
            wall_error = -2 * (ad_r - base_r);
            latest_wall_error = wall_error*0.5;
        } else if (ad_r < wall_thr_r && ad_l > wall_thr_l) {
            // 左壁のみある場合
            wall_error = 2 * (ad_l - base_l);
            latest_wall_error = wall_error*0.5;
        }

        // Priority-2: first-order low-pass filter for wall_error
        float alpha = WALL_LPF_ALPHA;
        if (alpha < 0.0f) alpha = 0.0f;
        if (alpha > 1.0f) alpha = 1.0f;
        wall_err_f = wall_err_f + alpha * (wall_error - wall_err_f);

        // Priority-1: deadband + proper saturation (no minimum-force injection)
        float wc = wall_err_f * kp_wall;

        // テスト動作フラグが立っている場合は横壁制御を無効化
        if (g_test_mode_run) {
            wc = 0.0f;
        }

        // Deadband around zero
        if (fabsf(wc) < WALL_CTRL_MIN) {
            wc = 0.0f;
        }

        // Proper clamp to ±WALL_CTRL_MAX
        if (wc >  WALL_CTRL_MAX) wc =  WALL_CTRL_MAX;
        if (wc < -WALL_CTRL_MAX) wc = -WALL_CTRL_MAX;

        // Disable at near-stop output (no slew, hard reset)
        int near_stop = (fabsf(out_l) < 50 && fabsf(out_r) < 50) ? 1 : 0;
        if (near_stop) {
            wc = 0.0f;
            wall_ctrl_prev = 0.0f;
            wall_control = 0.0f;
        } else {
            // Priority-3: apply slew rate limiting per variable step
            float delta = wc - wall_ctrl_prev;
            float lim = WALL_CTRL_SLEW_MAX * (g_ctrl_dt / 0.001f);
            if (delta >  lim) delta =  lim;
            if (delta < -lim) delta = -lim;
            float wc_slewed = wall_ctrl_prev + delta;
            wall_control = wc_slewed;
            wall_ctrl_prev = wc_slewed;
        }

        previous_ad_r = ad_r;
        previous_ad_l = ad_l;

    } else {
        // 制御フラグがなければ制御値0
        wall_control = 0;
        wall_err_f = 0.0f;
        wall_ctrl_prev = 0.0f;
    }

    // Update CTRL state
    s_prev_ctrl = MF.FLAG.CTRL ? 1 : 0;
}

/*斜めの制御*/
void diagonal_CTRL(void) {

    // 制御フラグがあれば制御
    if (MF.FLAG.CTRL_DIAGONAL) {

        if (ad_fr > diagonal_control_thr && ad_fl > diagonal_control_thr) {
            // 左右センサが閾値以上なら，差分で制御
            if (ad_fr > ad_fl) {
                diagonal_control = -kp_diagonal * (ad_fr - ad_fl);
            } else {
                diagonal_control = kp_diagonal * (ad_fl - ad_fr);
            }

        } else if (ad_fr > diagonal_control_thr) {
            // 右センサのみHigh
            diagonal_control = -kp_diagonal * ad_fr;
        } else if (ad_fl > diagonal_control_thr) {
            // 左センサのみHigh
            diagonal_control = kp_diagonal * ad_fl;
        }
    } else {
        diagonal_control = 0;
    }
}

void kushi_front_asym_CTRL(void) {
    static int8_t s_dir = 0;
    static uint8_t s_cnt = 0;

    // デフォルトOFF（安全側）
    kushi_control = 0.0f;

    if ((KUSHI_FRONT_ASYM_OMEGA == 0.0F) || g_test_mode_run) {
        s_dir = 0;
        s_cnt = 0;
        return;
    }

    // 直進中のみ
    const bool is_straight = (!MF.FLAG.SLALOM_R && !MF.FLAG.SLALOM_L);
    if (!is_straight) {
        s_dir = 0;
        s_cnt = 0;
        return;
    }

    // CTRLがONのときのみ（ターン前後の壁制御OFF区間では動かさない）
    if (!MF.FLAG.CTRL) {
        s_dir = 0;
        s_cnt = 0;
        return;
    }

    // 横壁なし判定（櫛区間の前提）
    float kx = sensor_kx;
    if (kx < 0.3f || kx > 2.0f) {
        kx = 1.0f;
    }
    const bool no_side_walls =
        (ad_r < (uint16_t)(WALL_BASE_R * kx)) &&
        (ad_l < (uint16_t)(WALL_BASE_L * kx));
    if (!no_side_walls) {
        s_dir = 0;
        s_cnt = 0;
        return;
    }

    // 前壁左右の片側のみ反応（閾値は前壁補正と同じスケール）
    float kf = fwall_kx;
    if (kf < 0.3f || kf > 2.0f) {
        kf = 1.1f;
    }
    const bool fr_on = (ad_fr > (uint16_t)(WALL_BASE_FR * kf));
    const bool fl_on = (ad_fl > (uint16_t)(WALL_BASE_FL * kf));

    int8_t dir = 0;
    if (fr_on && !fl_on) {
        dir = -1; // 右側が当たりそう -> 左へ
    } else if (fl_on && !fr_on) {
        dir = +1; // 左側が当たりそう -> 右へ
    } else {
        dir = 0;
    }

    if (dir == 0) {
        s_dir = 0;
        s_cnt = 0;
        return;
    }

    if (dir != s_dir) {
        s_dir = dir;
        s_cnt = 1;
        return;
    }

    if (s_cnt < 255u) {
        s_cnt++;
    }

    // 2ms以上連続で同じ片側反応なら有効化
    if (s_cnt >= 2u) {
        kushi_control = (float)s_dir * (float)KUSHI_FRONT_ASYM_OMEGA;
    }
}
