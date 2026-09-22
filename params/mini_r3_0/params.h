/*
 * params.h  –  mini_r3_0 initial tuning profile
 *
 * mini_r1_0/params.h をベースに F413 preorder 固有値を反映。
 * ハードウェア依存定数（モーター方向、センサ閾値 等）は実機調整で修正する。
 */

#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_

#define PARAMS_TUNE_VERSION "mini-r3-manual-tuning-t0.23"

/*============================================================
    各種定数（パラメータ）設定
============================================================*/
/* 自動撮影: 0=OFF（手動調整）、1=ON（開始/終了LED信号と撮影用待機）。 */
#define ENABLE_AUTO_VIDEO_CAPTURE 0U

/*------------------------------------------------------------
    走行系
------------------------------------------------------------*/
#define D_TIRE            14.13
#define DIST_HALF_SEC     45.0
#define DIST_D_HALF_SEC   67.279
#define ROBOT_REAR_OVERHANG 35.0
/* Rear-wall contact to the first 45 mm centre line. */
#define DIST_FIRST_SEC    (DIST_HALF_SEC - ROBOT_REAR_OVERHANG)
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
/* Encoder + IMU acceleration feedback for floor use. User accepted the
 * unit002 2S/fan-off tune on 2026-09-21; suction and 3S remain unqualified. */
#define VELOCITY_ACCEL_COMP_ENABLE_CONTROL 1U
#endif

/*
 * Preserve the existing omega-profile guard: use the short-delay encoder LPF
 * during turns until IMU-axis cross-coupling is separately floor-qualified.
 * This exception does not disable IMU-assisted straight/translation tuning.
 */
#ifndef VELOCITY_ACCEL_COMP_ENABLE_DURING_OMEGA_PROFILE
#define VELOCITY_ACCEL_COMP_ENABLE_DURING_OMEGA_PROFILE 0U
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

/* Suction gains are independent of the accepted FAN_OFF tune. */
#ifndef KP_VELOCITY_FAN_ON
/* 10:36 R180: additional load drops speed during the turn without saturation. */
#define KP_VELOCITY_FAN_ON  0.45F
#endif
#ifndef KI_VELOCITY_FAN_ON
#define KI_VELOCITY_FAN_ON  0.001F
#endif
#ifndef KD_VELOCITY_FAN_ON
#define KD_VELOCITY_FAN_ON  0.0F
#endif
#ifndef FF_TRANSLATION_STATIC_PWM_FAN_ON
/* Initial fit to 2026-09-22 09:52 mode6 trace, current fan=100%.
 * PWM is per-mille; velocity/acceleration references are mm/s and mm/s^2.
 * u_FF[%] = 6.5*sign(v) + 11.0*v[m/s] + 1.0*a[m/s^2].
 * Shared by all suction modes; lower fan duty/low-speed behavior unverified.
 * See docs/MINI_R3_SUCTION_FF_TUNING.md for fit and next-run checks. */
#define FF_TRANSLATION_STATIC_PWM_FAN_ON 65.0F
#endif
#ifndef FF_TRANSLATION_VELOCITY_PWM_FAN_ON
#define FF_TRANSLATION_VELOCITY_PWM_FAN_ON 0.110F
#endif
#ifndef FF_TRANSLATION_ACCEL_PWM_FAN_ON
#define FF_TRANSLATION_ACCEL_PWM_FAN_ON 0.0100F
#endif

#ifndef KP_VELOCITY_FAN_OFF
/* 2S, fan OFF, unit002 user-accepted floor tune, 2026-09-21.
 * I is accumulated per 1 ms control tick, without an extra dt multiplier. */
#define KP_VELOCITY_FAN_OFF 0.24F
#endif
#ifndef KI_VELOCITY_FAN_OFF
#define KI_VELOCITY_FAN_OFF 0.001F
#endif
#ifndef KD_VELOCITY_FAN_OFF
#define KD_VELOCITY_FAN_OFF 0.0F
#endif
#ifndef FF_TRANSLATION_STATIC_PWM_FAN_OFF
#define FF_TRANSLATION_STATIC_PWM_FAN_OFF 35.0F
#endif
#ifndef FF_TRANSLATION_VELOCITY_PWM_FAN_OFF
#define FF_TRANSLATION_VELOCITY_PWM_FAN_OFF 0.035F
#endif
#ifndef FF_TRANSLATION_ACCEL_PWM_FAN_OFF
#define FF_TRANSLATION_ACCEL_PWM_FAN_OFF 0.0040F
#endif

#ifndef KP_DISTANCE_FAN_ON
#define KP_DISTANCE_FAN_ON  2.00F
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
#define KP_DISTANCE_FAN_OFF 2.00F
#endif
#ifndef KI_DISTANCE_FAN_OFF
#define KI_DISTANCE_FAN_OFF 0.0F
#endif
#ifndef KD_DISTANCE_FAN_OFF
#define KD_DISTANCE_FAN_OFF 0.0F
#endif
#ifndef FF_DISTANCE_FAN_OFF
#define FF_DISTANCE_FAN_OFF 1.0F
#endif

#define SUCTION_FAN_STABILIZE_DELAY_MS 300

#ifndef SUCTION_GAIN_ON_THRESHOLD_PERCENT
#define SUCTION_GAIN_ON_THRESHOLD_PERCENT 50U
#endif

#ifndef KP_ANGLE_FAN_ON
/* t0.19: reduce the measured startup/turn heading error (10:17 R90 trace). */
#define KP_ANGLE_FAN_ON 15.0F
#endif
#ifndef KI_ANGLE_FAN_ON
/* The inner omega PI already rejects steady torque. Outer angle integration
 * accumulated the wrong-way rate command during suction holding (10:34/10:36). */
#define KI_ANGLE_FAN_ON 0.0F
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
#define KP_OMEGA_FAN_ON  0.8F
#endif
#ifndef KI_OMEGA_FAN_ON
#define KI_OMEGA_FAN_ON  0.006F
#endif
#ifndef KD_OMEGA_FAN_ON
#define KD_OMEGA_FAN_ON  0.0F
#endif
#ifndef FF_OMEGA_PWM_FAN_ON
/* PWM per-mille per deg/s or deg/s^2. These are initial suction values.
 * Acceleration FF supplements the existing 4 ms / 120 deg/s capped lead;
 * it is deliberately smaller than the fitted full inertia coefficient.
 * See docs/MINI_R3_SUCTION_GAIN_REVIEW.md. */
#define FF_OMEGA_PWM_FAN_ON 0.040F
#endif
#ifndef FF_OMEGA_ACCEL_PWM_FAN_ON
#define FF_OMEGA_ACCEL_PWM_FAN_ON 0.0008F
#endif

#ifndef KP_OMEGA_FAN_OFF
#define KP_OMEGA_FAN_OFF 0.45F
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
#define WALL_BASE_R   300
#define WALL_BASE_L   300
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

/* Front LUT is body-centre-to-wall mm, not the legacy sensor reference.
 * Retain the 2.5mm target/backoff margin; floor control still needs validation. */
#define F_ALIGN_TARGET_MM           45.0F
#define F_ALIGN_TOO_CLOSE_MM        42.5F

#define MATCH_POS_KP_TRANS_MM       10.0F
#define MATCH_POS_KP_ROT_MM         20.0F

#define MATCH_POS_VEL_MAX      60.0F
#define MATCH_POS_OMEGA_MAX   300.0F
#define MATCH_POS_TRANS_TOL_MM        1.5F
#define MATCH_POS_YAW_TOL_MM          0.8F
#define MATCH_POS_TRANS_RESTART_MM    2.5F
#define MATCH_POS_YAW_RESTART_MM      1.2F
#define MATCH_POS_SETTLE_GAP_MS         3U
#define MATCH_POS_YAW_SETTLE_MS         25U
#define MATCH_POS_FINAL_SETTLE_MS       30U
#define MATCH_POS_POST_COMPLETE_DELAY_MS 50U
#define MATCH_POS_RELAXED_AFTER_MS     600U
#define MATCH_POS_RELAXED_SETTLE_MS     10U
#define MATCH_POS_RELAXED_TRANS_TOL_MM 2.5F
#define MATCH_POS_RELAXED_YAW_TOL_MM   1.2F
#define MATCH_POS_REACQUIRE_TRANS_MM   3.0F
#define MATCH_POS_REACQUIRE_YAW_MM     1.5F
#define MATCH_POS_REACQUIRE_MS        200U
#define MATCH_POS_RECOVERY_VALID_MS    20U
#define MATCH_POS_TOO_CLOSE_RECOVERY_VEL_MM_S 30.0F
#define MATCH_POS_TOO_CLOSE_RECOVERY_MAX_MM   12.0F
#define MATCH_POS_TOO_CLOSE_RECOVERY_MAX_MS 1500U
#define MATCH_POS_MAX_DURATION_MS    1000U
#define MATCH_POS_SENSOR_LPF_ALPHA   0.5F
#define MATCH_POS_TRACE_PERIOD_MS      10U
#define MATCH_POS_TRACE_IDLE_PERIOD_MS 50U

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
#ifndef GOAL_X
#define GOAL_X   1
#endif
#ifndef GOAL_Y
#define GOAL_Y   0
#endif
#ifndef MAZE_SIZE
#define MAZE_SIZE 16
#endif
#ifndef START_X
#define START_X   0
#endif
#ifndef START_Y
#define START_Y   0
#endif

#if ((MAZE_SIZE != 16) && (MAZE_SIZE != 32))
#error "F413 MAZE_SIZE must be 16 or 32"
#endif
#if ((START_X >= MAZE_SIZE) || (START_Y >= MAZE_SIZE))
#error "F413 start cell is outside MAZE_SIZE"
#endif
#if ((GOAL_X >= MAZE_SIZE) || (GOAL_Y >= MAZE_SIZE))
#error "F413 goal cell is outside MAZE_SIZE"
#endif

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
