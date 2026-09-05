#ifndef F413_RUNTIME_ALIASES_H
#define F413_RUNTIME_ALIASES_H
#include "f413_machine.h"
/* Compatibility facade: algorithm code reads the immutable boot-selected values. */
#undef D_TIRE
#define D_TIRE (f413_machine_params()->scalar.v_D_TIRE)
#undef DIST_HALF_SEC
#define DIST_HALF_SEC (f413_machine_params()->scalar.v_DIST_HALF_SEC)
#undef DIST_D_HALF_SEC
#define DIST_D_HALF_SEC (f413_machine_params()->scalar.v_DIST_D_HALF_SEC)
#undef ROBOT_REAR_OVERHANG
#define ROBOT_REAR_OVERHANG (f413_machine_params()->scalar.v_ROBOT_REAR_OVERHANG)
#undef DIST_FIRST_SEC
#define DIST_FIRST_SEC (f413_machine_params()->scalar.v_DIST_FIRST_SEC)
#undef DIST_SET_POSITION
#define DIST_SET_POSITION (f413_machine_params()->scalar.v_DIST_SET_POSITION)
#undef F413_VELOCITY_LPF_TAU
#define F413_VELOCITY_LPF_TAU (f413_machine_params()->scalar.v_F413_VELOCITY_LPF_TAU)
#undef F413_IMU_GYRO_Z_LPF_TAU
#define F413_IMU_GYRO_Z_LPF_TAU (f413_machine_params()->scalar.v_F413_IMU_GYRO_Z_LPF_TAU)
#undef F413_IMU_GYRO_Z_SCALE
#define F413_IMU_GYRO_Z_SCALE (f413_machine_params()->scalar.v_F413_IMU_GYRO_Z_SCALE)
#undef F413_IMU_ACCEL_FORWARD_LPF_TAU
#define F413_IMU_ACCEL_FORWARD_LPF_TAU (f413_machine_params()->scalar.v_F413_IMU_ACCEL_FORWARD_LPF_TAU)
#undef F413_WALL_CTRL_LPF_ALPHA
#define F413_WALL_CTRL_LPF_ALPHA (f413_machine_params()->scalar.v_F413_WALL_CTRL_LPF_ALPHA)
#undef VELOCITY_ACCEL_COMP_WINDOW_MS
#define VELOCITY_ACCEL_COMP_WINDOW_MS (f413_machine_params()->scalar.v_VELOCITY_ACCEL_COMP_WINDOW_MS)
#undef VELOCITY_ACCEL_COMP_GAIN
#define VELOCITY_ACCEL_COMP_GAIN (f413_machine_params()->scalar.v_VELOCITY_ACCEL_COMP_GAIN)
#undef VELOCITY_ACCEL_COMP_ENABLE_CONTROL
#define VELOCITY_ACCEL_COMP_ENABLE_CONTROL (f413_machine_params()->scalar.v_VELOCITY_ACCEL_COMP_ENABLE_CONTROL)
#undef VELOCITY_ACCEL_COMP_ENABLE_DURING_OMEGA_PROFILE
#define VELOCITY_ACCEL_COMP_ENABLE_DURING_OMEGA_PROFILE (f413_machine_params()->scalar.v_VELOCITY_ACCEL_COMP_ENABLE_DURING_OMEGA_PROFILE)
#undef SEARCH_STEP_MM
#define SEARCH_STEP_MM (f413_machine_params()->scalar.v_SEARCH_STEP_MM)
#undef SEARCH_ANGLE_RESET_DUAL_WALL_STREAK_CELLS
#define SEARCH_ANGLE_RESET_DUAL_WALL_STREAK_CELLS (f413_machine_params()->scalar.v_SEARCH_ANGLE_RESET_DUAL_WALL_STREAK_CELLS)
#undef SEARCH_ANGLE_RESET_SINGLE_WALL_STREAK_CELLS
#define SEARCH_ANGLE_RESET_SINGLE_WALL_STREAK_CELLS (f413_machine_params()->scalar.v_SEARCH_ANGLE_RESET_SINGLE_WALL_STREAK_CELLS)
#undef SEARCH_POST_GOAL_SAVE_NEW_CELL_THRESHOLD
#define SEARCH_POST_GOAL_SAVE_NEW_CELL_THRESHOLD (f413_machine_params()->scalar.v_SEARCH_POST_GOAL_SAVE_NEW_CELL_THRESHOLD)
#undef ALPHA_ROTATE_90
#define ALPHA_ROTATE_90 (f413_machine_params()->scalar.v_ALPHA_ROTATE_90)
#undef ANGLE_ROTATE_90_R
#define ANGLE_ROTATE_90_R (f413_machine_params()->scalar.v_ANGLE_ROTATE_90_R)
#undef ANGLE_ROTATE_90_L
#define ANGLE_ROTATE_90_L (f413_machine_params()->scalar.v_ANGLE_ROTATE_90_L)
#undef DIFF_SETPOSITION
#define DIFF_SETPOSITION (f413_machine_params()->scalar.v_DIFF_SETPOSITION)
#undef KP_DISTANCE
#define KP_DISTANCE (f413_machine_params()->scalar.v_KP_DISTANCE)
#undef KI_DISTANCE
#define KI_DISTANCE (f413_machine_params()->scalar.v_KI_DISTANCE)
#undef KD_DISTANCE
#define KD_DISTANCE (f413_machine_params()->scalar.v_KD_DISTANCE)
#undef KP_VELOCITY
#define KP_VELOCITY (f413_machine_params()->scalar.v_KP_VELOCITY)
#undef KI_VELOCITY
#define KI_VELOCITY (f413_machine_params()->scalar.v_KI_VELOCITY)
#undef KD_VELOCITY
#define KD_VELOCITY (f413_machine_params()->scalar.v_KD_VELOCITY)
#undef KP_VELOCITY_FAN_ON
#define KP_VELOCITY_FAN_ON (f413_machine_params()->scalar.v_KP_VELOCITY_FAN_ON)
#undef KI_VELOCITY_FAN_ON
#define KI_VELOCITY_FAN_ON (f413_machine_params()->scalar.v_KI_VELOCITY_FAN_ON)
#undef KD_VELOCITY_FAN_ON
#define KD_VELOCITY_FAN_ON (f413_machine_params()->scalar.v_KD_VELOCITY_FAN_ON)
#undef FF_TRANSLATION_STATIC_PWM_FAN_ON
#define FF_TRANSLATION_STATIC_PWM_FAN_ON (f413_machine_params()->scalar.v_FF_TRANSLATION_STATIC_PWM_FAN_ON)
#undef FF_TRANSLATION_VELOCITY_PWM_FAN_ON
#define FF_TRANSLATION_VELOCITY_PWM_FAN_ON (f413_machine_params()->scalar.v_FF_TRANSLATION_VELOCITY_PWM_FAN_ON)
#undef FF_TRANSLATION_ACCEL_PWM_FAN_ON
#define FF_TRANSLATION_ACCEL_PWM_FAN_ON (f413_machine_params()->scalar.v_FF_TRANSLATION_ACCEL_PWM_FAN_ON)
#undef KP_VELOCITY_FAN_OFF
#define KP_VELOCITY_FAN_OFF (f413_machine_params()->scalar.v_KP_VELOCITY_FAN_OFF)
#undef KI_VELOCITY_FAN_OFF
#define KI_VELOCITY_FAN_OFF (f413_machine_params()->scalar.v_KI_VELOCITY_FAN_OFF)
#undef KD_VELOCITY_FAN_OFF
#define KD_VELOCITY_FAN_OFF (f413_machine_params()->scalar.v_KD_VELOCITY_FAN_OFF)
#undef FF_TRANSLATION_STATIC_PWM_FAN_OFF
#define FF_TRANSLATION_STATIC_PWM_FAN_OFF (f413_machine_params()->scalar.v_FF_TRANSLATION_STATIC_PWM_FAN_OFF)
#undef FF_TRANSLATION_VELOCITY_PWM_FAN_OFF
#define FF_TRANSLATION_VELOCITY_PWM_FAN_OFF (f413_machine_params()->scalar.v_FF_TRANSLATION_VELOCITY_PWM_FAN_OFF)
#undef FF_TRANSLATION_ACCEL_PWM_FAN_OFF
#define FF_TRANSLATION_ACCEL_PWM_FAN_OFF (f413_machine_params()->scalar.v_FF_TRANSLATION_ACCEL_PWM_FAN_OFF)
#undef KP_DISTANCE_FAN_ON
#define KP_DISTANCE_FAN_ON (f413_machine_params()->scalar.v_KP_DISTANCE_FAN_ON)
#undef KI_DISTANCE_FAN_ON
#define KI_DISTANCE_FAN_ON (f413_machine_params()->scalar.v_KI_DISTANCE_FAN_ON)
#undef KD_DISTANCE_FAN_ON
#define KD_DISTANCE_FAN_ON (f413_machine_params()->scalar.v_KD_DISTANCE_FAN_ON)
#undef FF_DISTANCE_FAN_ON
#define FF_DISTANCE_FAN_ON (f413_machine_params()->scalar.v_FF_DISTANCE_FAN_ON)
#undef KP_DISTANCE_FAN_OFF
#define KP_DISTANCE_FAN_OFF (f413_machine_params()->scalar.v_KP_DISTANCE_FAN_OFF)
#undef KI_DISTANCE_FAN_OFF
#define KI_DISTANCE_FAN_OFF (f413_machine_params()->scalar.v_KI_DISTANCE_FAN_OFF)
#undef KD_DISTANCE_FAN_OFF
#define KD_DISTANCE_FAN_OFF (f413_machine_params()->scalar.v_KD_DISTANCE_FAN_OFF)
#undef FF_DISTANCE_FAN_OFF
#define FF_DISTANCE_FAN_OFF (f413_machine_params()->scalar.v_FF_DISTANCE_FAN_OFF)
#undef SUCTION_FAN_STABILIZE_DELAY_MS
#define SUCTION_FAN_STABILIZE_DELAY_MS (f413_machine_params()->scalar.v_SUCTION_FAN_STABILIZE_DELAY_MS)
#undef SUCTION_GAIN_ON_THRESHOLD_PERCENT
#define SUCTION_GAIN_ON_THRESHOLD_PERCENT (f413_machine_params()->scalar.v_SUCTION_GAIN_ON_THRESHOLD_PERCENT)
#undef KP_ANGLE_FAN_ON
#define KP_ANGLE_FAN_ON (f413_machine_params()->scalar.v_KP_ANGLE_FAN_ON)
#undef KI_ANGLE_FAN_ON
#define KI_ANGLE_FAN_ON (f413_machine_params()->scalar.v_KI_ANGLE_FAN_ON)
#undef KD_ANGLE_FAN_ON
#define KD_ANGLE_FAN_ON (f413_machine_params()->scalar.v_KD_ANGLE_FAN_ON)
#undef FF_ANGLE_FAN_ON
#define FF_ANGLE_FAN_ON (f413_machine_params()->scalar.v_FF_ANGLE_FAN_ON)
#undef KP_ANGLE_FAN_OFF
#define KP_ANGLE_FAN_OFF (f413_machine_params()->scalar.v_KP_ANGLE_FAN_OFF)
#undef KI_ANGLE_FAN_OFF
#define KI_ANGLE_FAN_OFF (f413_machine_params()->scalar.v_KI_ANGLE_FAN_OFF)
#undef KD_ANGLE_FAN_OFF
#define KD_ANGLE_FAN_OFF (f413_machine_params()->scalar.v_KD_ANGLE_FAN_OFF)
#undef FF_ANGLE_FAN_OFF
#define FF_ANGLE_FAN_OFF (f413_machine_params()->scalar.v_FF_ANGLE_FAN_OFF)
#undef TURN_OMEGA_PROFILE_ROUNDING_SCALE
#define TURN_OMEGA_PROFILE_ROUNDING_SCALE (f413_machine_params()->scalar.v_TURN_OMEGA_PROFILE_ROUNDING_SCALE)
#undef KP_OMEGA_FAN_ON
#define KP_OMEGA_FAN_ON (f413_machine_params()->scalar.v_KP_OMEGA_FAN_ON)
#undef KI_OMEGA_FAN_ON
#define KI_OMEGA_FAN_ON (f413_machine_params()->scalar.v_KI_OMEGA_FAN_ON)
#undef KD_OMEGA_FAN_ON
#define KD_OMEGA_FAN_ON (f413_machine_params()->scalar.v_KD_OMEGA_FAN_ON)
#undef FF_OMEGA_PWM_FAN_ON
#define FF_OMEGA_PWM_FAN_ON (f413_machine_params()->scalar.v_FF_OMEGA_PWM_FAN_ON)
#undef FF_OMEGA_ACCEL_PWM_FAN_ON
#define FF_OMEGA_ACCEL_PWM_FAN_ON (f413_machine_params()->scalar.v_FF_OMEGA_ACCEL_PWM_FAN_ON)
#undef KP_OMEGA_FAN_OFF
#define KP_OMEGA_FAN_OFF (f413_machine_params()->scalar.v_KP_OMEGA_FAN_OFF)
#undef KI_OMEGA_FAN_OFF
#define KI_OMEGA_FAN_OFF (f413_machine_params()->scalar.v_KI_OMEGA_FAN_OFF)
#undef KD_OMEGA_FAN_OFF
#define KD_OMEGA_FAN_OFF (f413_machine_params()->scalar.v_KD_OMEGA_FAN_OFF)
#undef FF_OMEGA_PWM_FAN_OFF
#define FF_OMEGA_PWM_FAN_OFF (f413_machine_params()->scalar.v_FF_OMEGA_PWM_FAN_OFF)
#undef FF_OMEGA_ACCEL_PWM_FAN_OFF
#define FF_OMEGA_ACCEL_PWM_FAN_OFF (f413_machine_params()->scalar.v_FF_OMEGA_ACCEL_PWM_FAN_OFF)
#undef FF_OMEGA_LEAD_TIME_S
#define FF_OMEGA_LEAD_TIME_S (f413_machine_params()->scalar.v_FF_OMEGA_LEAD_TIME_S)
#undef FF_OMEGA_LEAD_MAX_DPS
#define FF_OMEGA_LEAD_MAX_DPS (f413_machine_params()->scalar.v_FF_OMEGA_LEAD_MAX_DPS)
#undef KP_IMU
#define KP_IMU (f413_machine_params()->scalar.v_KP_IMU)
#undef FAIL_COUNT_LR
#define FAIL_COUNT_LR (f413_machine_params()->scalar.v_FAIL_COUNT_LR)
#undef FAIL_LR_ERROR
#define FAIL_LR_ERROR (f413_machine_params()->scalar.v_FAIL_LR_ERROR)
#undef FAIL_COUNT_ACC
#define FAIL_COUNT_ACC (f413_machine_params()->scalar.v_FAIL_COUNT_ACC)
#undef FAIL_ACC
#define FAIL_ACC (f413_machine_params()->scalar.v_FAIL_ACC)
#undef FAIL_TURN_ANGLE_MARGIN_DEG
#define FAIL_TURN_ANGLE_MARGIN_DEG (f413_machine_params()->scalar.v_FAIL_TURN_ANGLE_MARGIN_DEG)
#undef FAIL_TURN_ANGLE_COUNT
#define FAIL_TURN_ANGLE_COUNT (f413_machine_params()->scalar.v_FAIL_TURN_ANGLE_COUNT)
#undef WALL_BASE_FR
#define WALL_BASE_FR (f413_machine_params()->scalar.v_WALL_BASE_FR)
#undef WALL_BASE_FL
#define WALL_BASE_FL (f413_machine_params()->scalar.v_WALL_BASE_FL)
#undef WALL_BASE_R
#define WALL_BASE_R (f413_machine_params()->scalar.v_WALL_BASE_R)
#undef WALL_BASE_L
#define WALL_BASE_L (f413_machine_params()->scalar.v_WALL_BASE_L)
#undef K_SENSOR
#define K_SENSOR (f413_machine_params()->scalar.v_K_SENSOR)
#undef SENSOR_DIST_GAIN
#define SENSOR_DIST_GAIN (f413_machine_params()->scalar.v_SENSOR_DIST_GAIN)
#undef WALL_END_THR_R_HIGH
#define WALL_END_THR_R_HIGH (f413_machine_params()->scalar.v_WALL_END_THR_R_HIGH)
#undef WALL_END_THR_R_LOW
#define WALL_END_THR_R_LOW (f413_machine_params()->scalar.v_WALL_END_THR_R_LOW)
#undef WALL_END_THR_L_HIGH
#define WALL_END_THR_L_HIGH (f413_machine_params()->scalar.v_WALL_END_THR_L_HIGH)
#undef WALL_END_THR_L_LOW
#define WALL_END_THR_L_LOW (f413_machine_params()->scalar.v_WALL_END_THR_L_LOW)
#undef WALL_END_DERIV_FALL_THR
#define WALL_END_DERIV_FALL_THR (f413_machine_params()->scalar.v_WALL_END_DERIV_FALL_THR)
#undef WALL_CTRL_DERIV_FALL_THR
#define WALL_CTRL_DERIV_FALL_THR (f413_machine_params()->scalar.v_WALL_CTRL_DERIV_FALL_THR)
#undef WALL_END_EXTEND_MAX_MM
#define WALL_END_EXTEND_MAX_MM (f413_machine_params()->scalar.v_WALL_END_EXTEND_MAX_MM)
#undef WALL_CTRL_BASE_L
#define WALL_CTRL_BASE_L (f413_machine_params()->scalar.v_WALL_CTRL_BASE_L)
#undef WALL_CTRL_BASE_R
#define WALL_CTRL_BASE_R (f413_machine_params()->scalar.v_WALL_CTRL_BASE_R)
#undef BAT_WARN_ADC_THR
#define BAT_WARN_ADC_THR (f413_machine_params()->scalar.v_BAT_WARN_ADC_THR)
#undef CTRL_BASE_L
#define CTRL_BASE_L (f413_machine_params()->scalar.v_CTRL_BASE_L)
#undef CTRL_BASE_R
#define CTRL_BASE_R (f413_machine_params()->scalar.v_CTRL_BASE_R)
#undef WALL_CTRL_MAX
#define WALL_CTRL_MAX (f413_machine_params()->scalar.v_WALL_CTRL_MAX)
#undef WALL_CTRL_MIN
#define WALL_CTRL_MIN (f413_machine_params()->scalar.v_WALL_CTRL_MIN)
#undef WALL_CTRL_SLEW_MAX
#define WALL_CTRL_SLEW_MAX (f413_machine_params()->scalar.v_WALL_CTRL_SLEW_MAX)
#undef KP_DEFAULT
#define KP_DEFAULT (f413_machine_params()->scalar.v_KP_DEFAULT)
#undef KP_TURN_AP
#define KP_TURN_AP (f413_machine_params()->scalar.v_KP_TURN_AP)
#undef IR_WAIT_US
#define IR_WAIT_US (f413_machine_params()->scalar.v_IR_WAIT_US)
#undef WALL_ALIGN_ERR_THR
#define WALL_ALIGN_ERR_THR (f413_machine_params()->scalar.v_WALL_ALIGN_ERR_THR)
#undef F_ALIGN_TARGET_MM
#define F_ALIGN_TARGET_MM (f413_machine_params()->scalar.v_F_ALIGN_TARGET_MM)
#undef F_ALIGN_TOO_CLOSE_MM
#define F_ALIGN_TOO_CLOSE_MM (f413_machine_params()->scalar.v_F_ALIGN_TOO_CLOSE_MM)
#undef MATCH_POS_KP_TRANS_MM
#define MATCH_POS_KP_TRANS_MM (f413_machine_params()->scalar.v_MATCH_POS_KP_TRANS_MM)
#undef MATCH_POS_KP_ROT_MM
#define MATCH_POS_KP_ROT_MM (f413_machine_params()->scalar.v_MATCH_POS_KP_ROT_MM)
#undef MATCH_POS_VEL_MAX
#define MATCH_POS_VEL_MAX (f413_machine_params()->scalar.v_MATCH_POS_VEL_MAX)
#undef MATCH_POS_OMEGA_MAX
#define MATCH_POS_OMEGA_MAX (f413_machine_params()->scalar.v_MATCH_POS_OMEGA_MAX)
#undef MATCH_POS_TRANS_TOL_MM
#define MATCH_POS_TRANS_TOL_MM (f413_machine_params()->scalar.v_MATCH_POS_TRANS_TOL_MM)
#undef MATCH_POS_YAW_TOL_MM
#define MATCH_POS_YAW_TOL_MM (f413_machine_params()->scalar.v_MATCH_POS_YAW_TOL_MM)
#undef MATCH_POS_TRANS_RESTART_MM
#define MATCH_POS_TRANS_RESTART_MM (f413_machine_params()->scalar.v_MATCH_POS_TRANS_RESTART_MM)
#undef MATCH_POS_YAW_RESTART_MM
#define MATCH_POS_YAW_RESTART_MM (f413_machine_params()->scalar.v_MATCH_POS_YAW_RESTART_MM)
#undef MATCH_POS_SETTLE_GAP_MS
#define MATCH_POS_SETTLE_GAP_MS (f413_machine_params()->scalar.v_MATCH_POS_SETTLE_GAP_MS)
#undef MATCH_POS_YAW_SETTLE_MS
#define MATCH_POS_YAW_SETTLE_MS (f413_machine_params()->scalar.v_MATCH_POS_YAW_SETTLE_MS)
#undef MATCH_POS_FINAL_SETTLE_MS
#define MATCH_POS_FINAL_SETTLE_MS (f413_machine_params()->scalar.v_MATCH_POS_FINAL_SETTLE_MS)
#undef MATCH_POS_POST_COMPLETE_DELAY_MS
#define MATCH_POS_POST_COMPLETE_DELAY_MS (f413_machine_params()->scalar.v_MATCH_POS_POST_COMPLETE_DELAY_MS)
#undef MATCH_POS_RELAXED_AFTER_MS
#define MATCH_POS_RELAXED_AFTER_MS (f413_machine_params()->scalar.v_MATCH_POS_RELAXED_AFTER_MS)
#undef MATCH_POS_RELAXED_SETTLE_MS
#define MATCH_POS_RELAXED_SETTLE_MS (f413_machine_params()->scalar.v_MATCH_POS_RELAXED_SETTLE_MS)
#undef MATCH_POS_RELAXED_TRANS_TOL_MM
#define MATCH_POS_RELAXED_TRANS_TOL_MM (f413_machine_params()->scalar.v_MATCH_POS_RELAXED_TRANS_TOL_MM)
#undef MATCH_POS_RELAXED_YAW_TOL_MM
#define MATCH_POS_RELAXED_YAW_TOL_MM (f413_machine_params()->scalar.v_MATCH_POS_RELAXED_YAW_TOL_MM)
#undef MATCH_POS_REACQUIRE_TRANS_MM
#define MATCH_POS_REACQUIRE_TRANS_MM (f413_machine_params()->scalar.v_MATCH_POS_REACQUIRE_TRANS_MM)
#undef MATCH_POS_REACQUIRE_YAW_MM
#define MATCH_POS_REACQUIRE_YAW_MM (f413_machine_params()->scalar.v_MATCH_POS_REACQUIRE_YAW_MM)
#undef MATCH_POS_REACQUIRE_MS
#define MATCH_POS_REACQUIRE_MS (f413_machine_params()->scalar.v_MATCH_POS_REACQUIRE_MS)
#undef MATCH_POS_RECOVERY_VALID_MS
#define MATCH_POS_RECOVERY_VALID_MS (f413_machine_params()->scalar.v_MATCH_POS_RECOVERY_VALID_MS)
#undef MATCH_POS_TOO_CLOSE_RECOVERY_VEL_MM_S
#define MATCH_POS_TOO_CLOSE_RECOVERY_VEL_MM_S (f413_machine_params()->scalar.v_MATCH_POS_TOO_CLOSE_RECOVERY_VEL_MM_S)
#undef MATCH_POS_TOO_CLOSE_RECOVERY_MAX_MM
#define MATCH_POS_TOO_CLOSE_RECOVERY_MAX_MM (f413_machine_params()->scalar.v_MATCH_POS_TOO_CLOSE_RECOVERY_MAX_MM)
#undef MATCH_POS_TOO_CLOSE_RECOVERY_MAX_MS
#define MATCH_POS_TOO_CLOSE_RECOVERY_MAX_MS (f413_machine_params()->scalar.v_MATCH_POS_TOO_CLOSE_RECOVERY_MAX_MS)
#undef MATCH_POS_MAX_DURATION_MS
#define MATCH_POS_MAX_DURATION_MS (f413_machine_params()->scalar.v_MATCH_POS_MAX_DURATION_MS)
#undef MATCH_POS_SENSOR_LPF_ALPHA
#define MATCH_POS_SENSOR_LPF_ALPHA (f413_machine_params()->scalar.v_MATCH_POS_SENSOR_LPF_ALPHA)
#undef MATCH_POS_TRACE_PERIOD_MS
#define MATCH_POS_TRACE_PERIOD_MS (f413_machine_params()->scalar.v_MATCH_POS_TRACE_PERIOD_MS)
#undef MATCH_POS_TRACE_IDLE_PERIOD_MS
#define MATCH_POS_TRACE_IDLE_PERIOD_MS (f413_machine_params()->scalar.v_MATCH_POS_TRACE_IDLE_PERIOD_MS)
#undef SENSOR_WARP_ANCHOR0_MM
#define SENSOR_WARP_ANCHOR0_MM (f413_machine_params()->scalar.v_SENSOR_WARP_ANCHOR0_MM)
#undef SENSOR_WARP_ANCHOR1_MM
#define SENSOR_WARP_ANCHOR1_MM (f413_machine_params()->scalar.v_SENSOR_WARP_ANCHOR1_MM)
#undef SENSOR_WARP_ANCHOR2_MM
#define SENSOR_WARP_ANCHOR2_MM (f413_machine_params()->scalar.v_SENSOR_WARP_ANCHOR2_MM)
#undef PARAMS_TUNE_VERSION
#define PARAMS_TUNE_VERSION (f413_machine_profile_name())
#define searchRunParams (f413_machine_params()->search)
#define shortestRunModeParams2 (f413_machine_params()->modes[0])
#define shortestRunCaseParamsMode2 (f413_machine_params()->cases[0])
#define shortestRunModeParams3 (f413_machine_params()->modes[1])
#define shortestRunCaseParamsMode3 (f413_machine_params()->cases[1])
#define shortestRunModeParams4 (f413_machine_params()->modes[2])
#define shortestRunCaseParamsMode4 (f413_machine_params()->cases[2])
#define shortestRunModeParams5 (f413_machine_params()->modes[3])
#define shortestRunCaseParamsMode5 (f413_machine_params()->cases[3])
#define shortestRunModeParams6 (f413_machine_params()->modes[4])
#define shortestRunCaseParamsMode6 (f413_machine_params()->cases[4])
#define shortestRunModeParams7 (f413_machine_params()->modes[5])
#define shortestRunCaseParamsMode7 (f413_machine_params()->cases[5])
#endif
