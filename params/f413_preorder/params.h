/*
 * params.h  –  F413 preorder (mini class)
 *
 * mini_r1_0/params.h をベースに F413 preorder 固有値を反映。
 * ハードウェア依存定数（モーター方向、センサ閾値 等）は実機調整で修正する。
 */

#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_

#define PARAMS_TUNE_VERSION "f413pre-t0.1"

/*============================================================
    各種定数（パラメータ）設定
============================================================*/
/*------------------------------------------------------------
    走行系
------------------------------------------------------------*/
#define D_TIRE            14.37
#define DIST_HALF_SEC     45.0
#define DIST_D_HALF_SEC   67.279
#define DIST_FIRST_SEC    5
#define DIST_SET_POSITION 5

/*------------------------------------------------------------
    フィルタ設定

    *_TAU は一次LPFの時定数 [s]。0以下にすると入力をそのまま使う。
------------------------------------------------------------*/
#ifndef F413_VELOCITY_LPF_TAU
#define F413_VELOCITY_LPF_TAU 0.003F
#endif

#ifndef F413_IMU_GYRO_Z_LPF_TAU
#define F413_IMU_GYRO_Z_LPF_TAU 0.0F
#endif

#ifndef F413_IMU_GYRO_Z_SCALE
#define F413_IMU_GYRO_Z_SCALE 1.0F
#endif

#ifndef F413_IMU_ACCEL_FORWARD_LPF_TAU
#define F413_IMU_ACCEL_FORWARD_LPF_TAU 0.010F
#endif

#ifndef F413_WALL_CTRL_LPF_ALPHA
#define F413_WALL_CTRL_LPF_ALPHA 0.1F
#endif

#ifndef VELOCITY_ACCEL_COMP_WINDOW_MS
#define VELOCITY_ACCEL_COMP_WINDOW_MS 30U
#endif

#ifndef VELOCITY_ACCEL_COMP_GAIN
#define VELOCITY_ACCEL_COMP_GAIN 1.0F
#endif

#ifndef VELOCITY_ACCEL_COMP_ENABLE_CONTROL
#define VELOCITY_ACCEL_COMP_ENABLE_CONTROL 1U
#endif

#ifndef SEARCH_STEP_MM
#define SEARCH_STEP_MM 10.0F
#endif

#ifndef SEARCH_ANGLE_RESET_DUAL_WALL_STREAK_CELLS
#define SEARCH_ANGLE_RESET_DUAL_WALL_STREAK_CELLS 3u
#endif
#ifndef SEARCH_ANGLE_RESET_SINGLE_WALL_STREAK_CELLS
#define SEARCH_ANGLE_RESET_SINGLE_WALL_STREAK_CELLS 5u
#endif

#ifndef SEARCH_POST_GOAL_SAVE_NEW_CELL_THRESHOLD
#define SEARCH_POST_GOAL_SAVE_NEW_CELL_THRESHOLD 128u
#endif

#define ALPHA_ROTATE_90   3000
#define ANGLE_ROTATE_90_R 90.0F
#define ANGLE_ROTATE_90_L 90.0F

#define DIFF_SETPOSITION 1500

/*PIDパラメータ*/
#define KP_DISTANCE 1.5F
#define KI_DISTANCE 0.03F
#define KD_DISTANCE 0.0F

#define KP_VELOCITY 0.03F
#define KI_VELOCITY 0.30F
#define KD_VELOCITY 0.0F

#ifndef KP_VELOCITY_FAN_ON
#define KP_VELOCITY_FAN_ON  2.5F
#endif
#ifndef KI_VELOCITY_FAN_ON
#define KI_VELOCITY_FAN_ON  0.03F
#endif
#ifndef KD_VELOCITY_FAN_ON
#define KD_VELOCITY_FAN_ON  0.0
#endif
#ifndef FF_TRANSLATION_STATIC_PWM_FAN_ON
#define FF_TRANSLATION_STATIC_PWM_FAN_ON 0.0F
#endif
#ifndef FF_TRANSLATION_VELOCITY_PWM_FAN_ON
#define FF_TRANSLATION_VELOCITY_PWM_FAN_ON 0.0F
#endif
#ifndef FF_TRANSLATION_ACCEL_PWM_FAN_ON
#define FF_TRANSLATION_ACCEL_PWM_FAN_ON 0.0F
#endif

#ifndef KP_VELOCITY_FAN_OFF
#define KP_VELOCITY_FAN_OFF 0.8F
#endif
#ifndef KI_VELOCITY_FAN_OFF
#define KI_VELOCITY_FAN_OFF 0.012F
#endif
#ifndef KD_VELOCITY_FAN_OFF
#define KD_VELOCITY_FAN_OFF 0.0F
#endif
#ifndef FF_TRANSLATION_STATIC_PWM_FAN_OFF
#define FF_TRANSLATION_STATIC_PWM_FAN_OFF 45.0F
#endif
#ifndef FF_TRANSLATION_VELOCITY_PWM_FAN_OFF
#define FF_TRANSLATION_VELOCITY_PWM_FAN_OFF 0.035F
#endif
#ifndef FF_TRANSLATION_ACCEL_PWM_FAN_OFF
#define FF_TRANSLATION_ACCEL_PWM_FAN_OFF 0.0040F
#endif

#ifndef KP_DISTANCE_FAN_ON
#define KP_DISTANCE_FAN_ON  0.10F
#endif
#ifndef KI_DISTANCE_FAN_ON
#define KI_DISTANCE_FAN_ON  0.0F
#endif
#ifndef KD_DISTANCE_FAN_ON
#define KD_DISTANCE_FAN_ON  0.0F
#endif
#ifndef FF_DISTANCE_FAN_ON
#define FF_DISTANCE_FAN_ON  1.0F
#endif

#ifndef KP_DISTANCE_FAN_OFF
#define KP_DISTANCE_FAN_OFF 6.00F
#endif
#ifndef KI_DISTANCE_FAN_OFF
#define KI_DISTANCE_FAN_OFF 0.05F
#endif
#ifndef KD_DISTANCE_FAN_OFF
#define KD_DISTANCE_FAN_OFF 0.0F
#endif
#ifndef FF_DISTANCE_FAN_OFF
#define FF_DISTANCE_FAN_OFF 1.0F
#endif

#define SUCTION_FAN_STABILIZE_DELAY_MS 100

#ifndef SUCTION_GAIN_ON_THRESHOLD_PERCENT
#define SUCTION_GAIN_ON_THRESHOLD_PERCENT 50U
#endif

#ifndef KP_ANGLE_FAN_ON
#define KP_ANGLE_FAN_ON 80.0F
#endif
#ifndef KI_ANGLE_FAN_ON
#define KI_ANGLE_FAN_ON 8.0F
#endif
#ifndef KD_ANGLE_FAN_ON
#define KD_ANGLE_FAN_ON 0.0F
#endif
#ifndef FF_ANGLE_FAN_ON
#define FF_ANGLE_FAN_ON 1.0F
#endif

#ifndef KP_ANGLE_FAN_OFF
#define KP_ANGLE_FAN_OFF 10.0F
#endif
#ifndef KI_ANGLE_FAN_OFF
#define KI_ANGLE_FAN_OFF 0.1F
#endif
#ifndef KD_ANGLE_FAN_OFF
#define KD_ANGLE_FAN_OFF 0.0F
#endif
#ifndef FF_ANGLE_FAN_OFF
#define FF_ANGLE_FAN_OFF 1.0F
#endif

#ifndef TURN_OMEGA_PROFILE_ROUNDING_SCALE
#define TURN_OMEGA_PROFILE_ROUNDING_SCALE 1.2F
#endif

#ifndef KP_OMEGA_FAN_ON
#define KP_OMEGA_FAN_ON  0.9F
#endif
#ifndef KI_OMEGA_FAN_ON
#define KI_OMEGA_FAN_ON  0.035F
#endif
#ifndef KD_OMEGA_FAN_ON
#define KD_OMEGA_FAN_ON  1.0F
#endif
#ifndef FF_OMEGA_PWM_FAN_ON
#define FF_OMEGA_PWM_FAN_ON 0.0F
#endif
#ifndef FF_OMEGA_ACCEL_PWM_FAN_ON
#define FF_OMEGA_ACCEL_PWM_FAN_ON 0.0F
#endif

#ifndef KP_OMEGA_FAN_OFF
#define KP_OMEGA_FAN_OFF 1.35F
#endif
#ifndef KI_OMEGA_FAN_OFF
#define KI_OMEGA_FAN_OFF 0.006F
#endif
#ifndef KD_OMEGA_FAN_OFF
#define KD_OMEGA_FAN_OFF 0.0F
#endif
#ifndef FF_OMEGA_PWM_FAN_OFF
#define FF_OMEGA_PWM_FAN_OFF 0.0F
#endif
#ifndef FF_OMEGA_ACCEL_PWM_FAN_OFF
#define FF_OMEGA_ACCEL_PWM_FAN_OFF 0.0F
#endif

#ifndef FF_OMEGA_LEAD_TIME_S
#define FF_OMEGA_LEAD_TIME_S 0.004F
#endif
#ifndef FF_OMEGA_LEAD_MAX_DPS
#define FF_OMEGA_LEAD_MAX_DPS 120.0F
#endif

#define KP_IMU 1.0F

#define FAIL_COUNT_LR  20
#define FAIL_LR_ERROR  4000
#define FAIL_COUNT_ACC 20
#define FAIL_ACC       17000

#define FAIL_TURN_ANGLE_MARGIN_DEG 90
#define FAIL_TURN_ANGLE_COUNT 2

/*動作方向関連 — F413 preorder は mini_r1_0 と同一（実機で要確認）*/
#define DIR_FWD_L  GPIO_PIN_RESET
#define DIR_BACK_L GPIO_PIN_SET
#define DIR_FWD_R  GPIO_PIN_SET
#define DIR_BACK_R GPIO_PIN_RESET
#define DIR_ENC_R  -1
#define DIR_ENC_L  -1

/*------------------------------------------------------------
    センサ系
------------------------------------------------------------*/
#define WALL_BASE_FR  160
#define WALL_BASE_FL  160
#define WALL_BASE_R   160
#define WALL_BASE_L   160
#define K_SENSOR      1.00F

#ifndef SENSOR_DIST_GAIN
#define SENSOR_DIST_GAIN 1.0F
#endif

#ifndef WALL_END_THR_R_HIGH
#define WALL_END_THR_R_HIGH  280
#endif
#ifndef WALL_END_THR_R_LOW
#define WALL_END_THR_R_LOW   200
#endif
#ifndef WALL_END_THR_L_HIGH
#define WALL_END_THR_L_HIGH  280
#endif
#ifndef WALL_END_THR_L_LOW
#define WALL_END_THR_L_LOW   200
#endif

#define WALL_END_DERIV_FALL_THR 200
#define WALL_CTRL_DERIV_FALL_THR 120

#ifndef WALL_END_EXTEND_MAX_MM
#define WALL_END_EXTEND_MAX_MM  10.0F
#endif

#define WALL_CTRL_BASE_L 1941
#define WALL_CTRL_BASE_R 1989

#ifndef BAT_WARN_ADC_THR
#define BAT_WARN_ADC_THR 2150
#endif

/*制御閾値*/
#define CTRL_BASE_L   1
#define CTRL_BASE_R   1
#define WALL_CTRL_MAX 100
#ifndef WALL_CTRL_MIN
#define WALL_CTRL_MIN 0.2F
#endif
#ifndef WALL_CTRL_SLEW_MAX
#define WALL_CTRL_SLEW_MAX 5.0F
#endif
#define KP_DEFAULT    0.1F
#define KP_TURN_AP    0.3F

#define IR_WAIT_US 30

#define WALL_ALIGN_ERR_THR  700

#define F_ALIGN_TARGET_FR    1420
#define F_ALIGN_TARGET_FL    1400

#define F_ALIGN_DETECT_THR   500

#define MATCH_POS_KP_TRANS   -0.4F
#define MATCH_POS_KP_ROT     0.2F

#define MATCH_POS_VEL_MAX     200.0F
#define MATCH_POS_OMEGA_MAX   300.0F
#define MATCH_POS_TOL         100
#define MATCH_POS_TOL_ANGLE   40
#define MATCH_POS_TIMEOUT_MS  20
#define MATCH_POS_STABLE_COUNT 100

#ifndef SENSOR_WARP_ANCHOR0_MM
#define SENSOR_WARP_ANCHOR0_MM  0.0f
#endif
#ifndef SENSOR_WARP_ANCHOR1_MM
#define SENSOR_WARP_ANCHOR1_MM  26.0f
#endif
#ifndef SENSOR_WARP_ANCHOR2_MM
#define SENSOR_WARP_ANCHOR2_MM  113.0f
#endif

/*------------------------------------------------------------
    探索系
------------------------------------------------------------*/
#define GOAL_X   1
#define GOAL_Y   0
#define MAZE_SIZE 16
#define START_X   0
#define START_Y   0

// 複数ゴール設定（3x3 = 9 セル）
#ifndef GOAL1_X
#define GOAL1_X GOAL_X
#define GOAL1_Y GOAL_Y
#endif

#ifndef GOAL2_X
#define GOAL2_X 0
#define GOAL2_Y 0
#endif

#ifndef GOAL3_X
#define GOAL3_X 0
#define GOAL3_Y 0
#endif

#ifndef GOAL4_X
#define GOAL4_X 0
#define GOAL4_Y 0
#endif

#ifndef GOAL5_X
#define GOAL5_X 0
#define GOAL5_Y 0
#endif

#ifndef GOAL6_X
#define GOAL6_X 0
#define GOAL6_Y 0
#endif

#ifndef GOAL7_X
#define GOAL7_X 0
#define GOAL7_Y 0
#endif

#ifndef GOAL8_X
#define GOAL8_X 0
#define GOAL8_Y 0
#endif

#ifndef GOAL9_X
#define GOAL9_X 0
#define GOAL9_Y 0
#endif

#endif /* INC_PARAMS_H_ */
