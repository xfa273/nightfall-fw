/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>

#include "build_info.h"
#include "nvm.h"
#include "nvm_identity.h"
#include "nvm_params.h"
#include "nvm_trace_log.h"
#include "f413_control.h"
#include "f413_diag.h"
#include "f413_hw.h"
#include "f413_hw_diag.h"
#include "f413_op_ui.h"
#include "f413_run_session.h"
#include "f413_search_step.h"
#include "f413_test_run.h"
#include "f413_trace_log.h"
#include "f413_trace_diag.h"
#include "f413_wall_sensor.h"
#include "params.h"
#include "shortest_run_params.h"
#include "solver.h"
#include "search.h"
#include "trace.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

static nvm_status_t g_boot_identity_status = NVM_STATUS_UNSUPPORTED;
static nvm_identity_block_t g_boot_identity;
static volatile uint8_t g_trace_log_context_mode = 0xFFU;
static volatile uint8_t g_trace_log_context_case = 0xFFU;
static volatile uint8_t g_trace_log_context_sub = 0xFFU;
static volatile uint8_t g_trace_log_context_test_id = 0U;
static volatile uint16_t g_trace_log_adc_fr = 0U;
static volatile uint16_t g_trace_log_adc_r = 0U;
static volatile uint16_t g_trace_log_adc_fl = 0U;
static volatile uint16_t g_trace_log_adc_l = 0U;
static volatile uint16_t g_trace_log_adc_vbat = 0U;
static volatile int32_t g_trace_log_reserved_i32_0 = 0;
static volatile int32_t g_trace_log_reserved_i32_1 = 0;
static volatile int32_t g_trace_log_reserved_i32_2 = 0;
static volatile int32_t g_trace_log_reserved_i32_3 = 0;
static volatile uint16_t g_trace_log_reserved_u16_0 = 0U;
static volatile uint16_t g_trace_log_reserved_u16_1 = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define NIGHTFALL_F413_ENCODER_WINDOW_MS (3000U)
#define NIGHTFALL_F413_LED_ON_WINDOW_MS (30000U)
#define NIGHTFALL_F413_TRACE_DUMP_MAX_RECORDS (8U)
#define NIGHTFALL_F413_TRACE_CSV_MAX_RECORDS (256U)
#define NIGHTFALL_F413_TRACE_SELFTEST_RECORDS (16U)
#define NIGHTFALL_F413_TRACE_SWITCH_FLAG (0x0001U)
#define NIGHTFALL_F413_TRACE_MODE_IDLE_FLAG (0x0002U)
#define NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG (0x0004U)
#define NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG (0x0008U)
#define NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG (0x0010U)
#define NIGHTFALL_F413_TRACE_MODE_SMOKE_FLAG (0x0020U)
#define NIGHTFALL_F413_TRACE_MODE_SEARCH_SAFE_FLAG (0x0040U)
#define NIGHTFALL_F413_TRACE_MODE_SHORTEST_SAFE_FLAG (0x0080U)
#define NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG (0x1000U)
#define NIGHTFALL_F413_TRACE_ANGLE_TARGET_FLAG (0x2000U)
#define NIGHTFALL_F413_TRACE_MODE_TUNE_FLAG (0x4000U)
#define NIGHTFALL_F413_RUN_SESSION_IDLE_MS (1000U)
#define NIGHTFALL_F413_RUN_SESSION_MOTOR_DUTY (120U)
#define NIGHTFALL_F413_RUN_SESSION_MOTOR_PULSE_MS (300U)
#define NIGHTFALL_F413_RUN_SESSION_MOTOR_COAST_MS (120U)
#define NIGHTFALL_F413_RUN_SESSION_SAFE_DUTY (70U)
#define NIGHTFALL_F413_RUN_SESSION_SAFE_FORWARD_MS (160U)
#define NIGHTFALL_F413_RUN_SESSION_SAFE_TURN_MS (120U)
#define NIGHTFALL_F413_RUN_SESSION_SAFE_COAST_MS (80U)
#define NIGHTFALL_F413_RUN_SESSION_SAFE_EXPLORE_STEPS (4U)
#define NIGHTFALL_F413_IMU_WHO_AM_I_REG (0x0FU)
#define NIGHTFALL_F413_IMU_WHO_AM_I_EXPECTED (0x6BU)
#define NIGHTFALL_F413_IMU_CTRL1_XL (0x10U)
#define NIGHTFALL_F413_IMU_CTRL2_G (0x11U)
#define NIGHTFALL_F413_IMU_CTRL3_C (0x12U)
#define NIGHTFALL_F413_IMU_OUTZ_G_L (0x26U)
#define NIGHTFALL_F413_IMU_OUTX_XL_L (0x28U)
#define NIGHTFALL_F413_IMU_OUTY_XL_L (0x2AU)
#define NIGHTFALL_F413_IMU_OUTZ_XL_L (0x2CU)
#define NIGHTFALL_F413_IMU_GYRO_SENSITIVITY (0.14f)
#define NIGHTFALL_F413_IMU_ACCEL_SENS_MG (0.488f)
#define NIGHTFALL_F413_IMU_GRAVITY_MM_S2 (9.80665f)
#define NIGHTFALL_F413_IMU_MANUAL_OFFSET_SAMPLES (500U)
#define NIGHTFALL_F413_IMU_MANUAL_TEST_MS (8000U)
#define NIGHTFALL_F413_IMU_MANUAL_SAMPLE_MS (10U)
#define NIGHTFALL_F413_IMU_MANUAL_PRINT_MS (200U)
#define NIGHTFALL_F413_MAZE_SIZE (32U)
#define NIGHTFALL_F413_MAZE_CELL_COUNT (NIGHTFALL_F413_MAZE_SIZE * NIGHTFALL_F413_MAZE_SIZE)
#define NIGHTFALL_F413_REAL_GOAL_X (15U)
#define NIGHTFALL_F413_REAL_GOAL_Y (15U)

#ifndef NIGHTFALL_F413_REAL_RUN_PATH_ENABLED
#define NIGHTFALL_F413_REAL_RUN_PATH_ENABLED (1U)
#endif
#ifndef NIGHTFALL_F413_UART_BAUD_RATE
#define NIGHTFALL_F413_UART_BAUD_RATE (115200U)
#endif

#if (NIGHTFALL_F413_REAL_RUN_PATH_ENABLED != 0U)
#define NIGHTFALL_F413_SOLVER_MODE      (2U)
#define NIGHTFALL_F413_SOLVER_CASE      (1U)
#define NIGHTFALL_F413_PATH_PREVIEW_MAX (24U)
#define NIGHTFALL_F413_PATH_VELOCITY      (200.0f)  /* 直進速度 [mm/s] */
#define NIGHTFALL_F413_PATH_OMEGA         (200.0f)  /* 旋回角速度 [deg/s] */
#define NIGHTFALL_F413_PATH_HALF_CELL_MM  (45.0f)   /* 半区画距離 [mm] */
#define NIGHTFALL_F413_PATH_DIAG_HALF_CELL_MM ((float)DIST_D_HALF_SEC)
#define NIGHTFALL_F413_PATH_COAST_MS      (60U)     /* 区間間停止 [ms] */
#define NIGHTFALL_F413_PATH_MAX_CODES     (256U)
#define NIGHTFALL_F413_PATH_TIMEOUT_MS    (5000U)   /* 区間タイムアウト [ms] */
#define NIGHTFALL_F413_PATH_VELOCITY_CAP  (NIGHTFALL_F413_PATH_VELOCITY)
#endif

/* ---------- 調整用テストモード定数 ---------- */
#define NIGHTFALL_F413_OP_MODE_MAX        (9U)
#define NIGHTFALL_F413_OP_ENTER_DELTA_ADC (150)
#define NIGHTFALL_F413_OP_ENTER_RELEASE_ADC (250)
#define NIGHTFALL_F413_OP_START_DELAY_MS  (2000U)
#define NIGHTFALL_F413_OP_TEST_VEL_CAP    (450.0f)
#define NIGHTFALL_F413_OP_TEST_DIAG_SCALE (0.75f)
#define NIGHTFALL_F413_WALL_END_MONITOR_MS (4000U)
#define NIGHTFALL_F413_WALL_END_MONITOR_SAMPLE_MS (50U)
#ifndef NIGHTFALL_F413_DISABLE_WALL_TRACE_OBSERVE
#define NIGHTFALL_F413_DISABLE_WALL_TRACE_OBSERVE (0U)
#endif
#define NIGHTFALL_F413_WALL_TRACE_FRONT_FLAG (0x0001U)
#define NIGHTFALL_F413_WALL_TRACE_RIGHT_FLAG (0x0002U)
#define NIGHTFALL_F413_WALL_TRACE_LEFT_FLAG (0x0004U)
#define NIGHTFALL_F413_WALL_TRACE_SAT_FLAG (0x0008U)
#define NIGHTFALL_F413_WALL_TRACE_END_WALL_R_FLAG (0x0010U)
#define NIGHTFALL_F413_WALL_TRACE_END_WALL_L_FLAG (0x0020U)
#define NIGHTFALL_F413_WALL_TRACE_END_R_FLAG (0x0040U)
#define NIGHTFALL_F413_WALL_TRACE_END_L_FLAG (0x0080U)
#define NIGHTFALL_F413_WALL_TRACE_GATE_FLAG (0x0100U)
#define NIGHTFALL_F413_WALL_TRACE_CTRL_FLAG (0x0200U)
#define NIGHTFALL_F413_WALL_TRACE_ENABLED_FLAG (0x8000U)
#define NIGHTFALL_F413_WALL_TRACE_VERSION (1U)
#ifndef NIGHTFALL_F413_DISABLE_WALL_CONTROL
#define NIGHTFALL_F413_DISABLE_WALL_CONTROL (0U)
#endif
#define NIGHTFALL_F413_WALL_CTRL_KP_DEG_PER_ADC (KP_DEFAULT)
#define NIGHTFALL_F413_WALL_CTRL_MAX_DEG (WALL_CTRL_MAX)
#define NIGHTFALL_F413_WALL_CTRL_LPF_ALPHA (WALL_LPF_ALPHA)
#define NIGHTFALL_F413_WALL_CTRL_SLEW_DEG (WALL_CTRL_SLEW_MAX)
#define NIGHTFALL_F413_WALL_CTRL_MIN_VEL_MM_S (20.0f)
#define NIGHTFALL_F413_MAZE_WALL_W (0x01U)
#define NIGHTFALL_F413_MAZE_WALL_S (0x02U)
#define NIGHTFALL_F413_MAZE_WALL_E (0x04U)
#define NIGHTFALL_F413_MAZE_WALL_N (0x08U)
#define NIGHTFALL_F413_MAZE_WALL_KNOWN_MASK (0x0FU)
#define NIGHTFALL_F413_MAZE_START_FORCED_WALLS (0x07U)
#define NIGHTFALL_F413_SEARCH_MAP_CELL_COUNT ((uint32_t)(MAZE_SIZE * MAZE_SIZE))
#define NIGHTFALL_F413_SEARCH_STEP_VELOCITY_MM_S (150.0f)
#define NIGHTFALL_F413_SEARCH_STEP_TARGET_MM (90.0f)
#define NIGHTFALL_F413_SEARCH_STEP_TURN_DEG (90.0f)
#define NIGHTFALL_F413_TUNE_TIMEOUT_MS (1500U)

#define NIGHTFALL_RUN_ABORT_NONE F413_RUN_SESSION_ABORT_NONE
#define NIGHTFALL_RUN_ABORT_SWITCH F413_RUN_SESSION_ABORT_SWITCH
#define NIGHTFALL_RUN_ABORT_WALL_FAULT F413_RUN_SESSION_ABORT_WALL_FAULT
#define NIGHTFALL_RUN_ABORT_ENCODER_FAULT F413_RUN_SESSION_ABORT_ENCODER_FAULT
#define NIGHTFALL_RUN_ABORT_IMU_FAULT F413_RUN_SESSION_ABORT_IMU_FAULT
typedef f413_run_session_abort_reason_t nightfall_run_abort_reason_t;
typedef f413_run_session_guard_t nightfall_run_guard_t;

typedef f413_wall_sensor_snapshot_t nightfall_wall_sensor_snapshot_t;

typedef struct
{
  bool right_wall;
  bool left_wall;
  bool prev_right_wall;
  bool prev_left_wall;
  bool detected_r;
  bool detected_l;
  float dist_r_mm;
  float dist_l_mm;
  int32_t deriv_r;
  int32_t deriv_l;
  int32_t prev_r_delta;
  int32_t prev_l_delta;
  uint8_t deriv_fall_count_r;
  uint8_t deriv_fall_count_l;
  bool initialized;
} nightfall_wall_end_state_t;

static void nightfall_run_search_safe_trace_session_once(void);
static void nightfall_run_shortest_safe_trace_session_once(void);
static void nightfall_run_search_trace_entry_once(void);
static void nightfall_run_shortest_trace_entry_once(void);
#if (NIGHTFALL_F413_REAL_RUN_PATH_ENABLED != 0U)
static void nightfall_solver_path_print_preview(void);
static void nightfall_run_solver_path_session_once(uint16_t base_trace_flag);
static void nightfall_run_path_code_sequence_once(const char* label, uint8_t mode, uint8_t case_index,
                                                   const uint16_t* codes, uint16_t code_count,
                                                   float straight_velocity, float diagonal_velocity);
static const ShortestRunModeParams_t* nightfall_f413_shortest_mode_params(uint8_t mode);
static const ShortestRunCaseParams_t* nightfall_f413_shortest_case_params(uint8_t mode, uint8_t case_index);
static float nightfall_f413_path_velocity_or_cap(float candidate, float fallback);
static bool nightfall_f413_path_turn_from_code(uint16_t code,
                                               const ShortestRunModeParams_t* params,
                                               float* signed_angle_deg,
                                               float* abs_angle_deg);
#endif
static bool nightfall_run_stop_switch_pressed(void);
static int16_t nightfall_run_session_encoder_l_count(void);
static int16_t nightfall_run_session_encoder_r_count(void);
static void nightfall_run_session_encoder_stop_all(void);
static void nightfall_run_session_encoder_reset_all(void);
static bool nightfall_run_session_encoder_start_l(void);
static bool nightfall_run_session_encoder_start_r(void);
static void nightfall_run_session_encoder_stop_l(void);
static void nightfall_run_session_encoder_stop_r(void);
static bool nightfall_run_session_wall_sensor_ok(void);
static bool nightfall_run_session_imu_ok(void);
static bool nightfall_run_guard_prepare(nightfall_run_guard_t* guard);
static void nightfall_run_guard_cleanup(nightfall_run_guard_t* guard);
static nightfall_run_abort_reason_t nightfall_trace_log_wait_with_auto_step_guarded(uint32_t duration_ms,
                                                                                     nightfall_run_guard_t* guard);
static uint16_t nightfall_run_abort_reason_to_trace_flag(nightfall_run_abort_reason_t reason);
static const char* nightfall_run_abort_reason_to_text(nightfall_run_abort_reason_t reason);
static void nightfall_trace_log_on_run_start(void);
static void nightfall_trace_log_on_run_stop(void);
static void nightfall_trace_log_auto_step(void);
static void nightfall_trace_log_auto_tick_sample(void);
static void nightfall_trace_log_update_observe_cache(void);
static void nightfall_trace_log_set_mode_flags(uint16_t flags);
static void nightfall_trace_log_set_context(uint8_t mode, uint8_t op_case, uint8_t sub, uint8_t test_id);
static void nightfall_trace_diag_get_context(uint8_t* mode, uint8_t* op_case, uint8_t* sub, uint8_t* test_id);
static void nightfall_trace_diag_emit_extra_csv_meta(void);
static int32_t nightfall_trace_log_scale_float(float value, float scale);
static const char* nightfall_op_mode_name(uint8_t mode);
static const char* nightfall_op_level_name(uint8_t level);
static const char* nightfall_op_case_name(uint8_t mode, uint8_t op_case);
static const char* nightfall_op_sub_name(uint8_t mode, uint8_t sub);
static const char* nightfall_tune_axis_name(uint8_t axis);
static const char* nightfall_tune_pattern_name(uint8_t pattern);
static bool nightfall_trace_log_read_adc_raw(uint16_t* fr, uint16_t* r, uint16_t* fl, uint16_t* l, uint16_t* vbat);
static bool nightfall_trace_log_fill_wall_observe(nvm_trace_log_record_t* out);
static bool nightfall_wall_sensor_start_async(void);
static void nightfall_wall_sensor_tim6_tick(void);
static bool nightfall_wall_sensor_read_snapshot(nightfall_wall_sensor_snapshot_t* out);
static void nightfall_wall_end_clear(void);
static uint16_t nightfall_wall_trace_flags_from_snapshot(const nightfall_wall_sensor_snapshot_t* wall,
                                                         bool gate_on);
static void nightfall_wall_control_reset(void);
static void nightfall_wall_control_update(const nightfall_wall_sensor_snapshot_t* wall, bool gate_on);
static void nightfall_wall_control_apply(bool straight_gate);
static void nightfall_run_search_decision_preview_once(void);
static void nightfall_search_step_session_reset(void);
static void nightfall_run_search_step_once(void);
static void nightfall_run_search_map_probe_once(void);
static void nightfall_run_search_status_once(void);
static void nightfall_run_search_map_clear_once(void);
static void nightfall_run_search_map_dump_once(void);
static void nightfall_verify_all_nvm_load_only(void);
static void nightfall_wall_end_reset_from_snapshot(const nightfall_wall_sensor_snapshot_t* wall);
static void nightfall_wall_end_update(const nightfall_wall_sensor_snapshot_t* wall, bool gate_on);
static void nightfall_run_wall_end_monitor_once(void);
static void nightfall_motor_set(bool enable, bool left_forward, bool right_forward,
                                uint16_t left_duty, uint16_t right_duty);
static void nightfall_run_control_tune_once(uint8_t axis, uint8_t set, uint8_t pattern);
static void nightfall_op_run_tune_sub_after_delay(uint8_t sub);
static void nightfall_op_execute_action(f413_op_ui_action_t action, uint8_t mode, uint8_t op_case, uint8_t sub);
static void nightfall_op_ui_step(void);
static void nightfall_run_imu_manual_turn_test_once(void);
static void nightfall_run_imu_accel_test_once(void);
static void nightfall_run_fan_pwm_test_once(void);
static void nightfall_run_encoder_test_once(void);

static uint8_t g_last_test_id = 0U;
static nightfall_run_abort_reason_t g_last_test_abort_reason = NIGHTFALL_RUN_ABORT_NONE;
static float g_last_test_distance_mm = 0.0f;
static float g_last_test_angle_deg = 0.0f;
static nightfall_wall_end_state_t g_wall_end;
static float g_wall_ctrl_angle_deg = 0.0f;
static float g_wall_ctrl_error_lpf = 0.0f;
static bool g_wall_ctrl_active = false;
static bool g_last_tune_valid = false;
static uint8_t g_last_tune_axis = 0U;
static uint8_t g_last_tune_set = 0U;
static uint8_t g_last_tune_pattern = 0U;

static const char* nightfall_identity_family_name(uint32_t family)
{
  switch (family)
  {
    case NVM_FAMILY_MINI:
      return "mini";
    case NVM_FAMILY_CLASSIC:
      return "classic";
    default:
      return "unknown";
  }
}

static void nightfall_run_search_trace_entry_once(void)
{
#if (NIGHTFALL_F413_REAL_RUN_PATH_ENABLED != 0U)
  if (solver_build_path(NIGHTFALL_F413_SOLVER_MODE, NIGHTFALL_F413_SOLVER_CASE))
  {
    trace_printf("[RUN-TEST] search-entry solver-path ready mode=%u case=%u\r\n",
                 (unsigned int)NIGHTFALL_F413_SOLVER_MODE,
                 (unsigned int)NIGHTFALL_F413_SOLVER_CASE);
    nightfall_solver_path_print_preview();
    nightfall_run_solver_path_session_once(NIGHTFALL_F413_TRACE_MODE_SEARCH_SAFE_FLAG);
  }
  else
  {
    trace_printf("[RUN-TEST] search-entry solver-path build failed -> fallback safe\r\n");
    nightfall_run_search_safe_trace_session_once();
  }
#else
  nightfall_run_search_safe_trace_session_once();
#endif
}

static void nightfall_run_shortest_trace_entry_once(void)
{
#if (NIGHTFALL_F413_REAL_RUN_PATH_ENABLED != 0U)
  if (solver_build_path(NIGHTFALL_F413_SOLVER_MODE, NIGHTFALL_F413_SOLVER_CASE))
  {
    trace_printf("[RUN-TEST] shortest-entry solver-path ready mode=%u case=%u\r\n",
                 (unsigned int)NIGHTFALL_F413_SOLVER_MODE,
                 (unsigned int)NIGHTFALL_F413_SOLVER_CASE);
    nightfall_solver_path_print_preview();
    nightfall_run_solver_path_session_once(NIGHTFALL_F413_TRACE_MODE_SHORTEST_SAFE_FLAG);
  }
  else
  {
    trace_printf("[RUN-TEST] shortest-entry solver-path build failed -> fallback safe\r\n");
    nightfall_run_shortest_safe_trace_session_once();
  }
#else
  nightfall_run_shortest_safe_trace_session_once();
#endif
}

#if (NIGHTFALL_F413_REAL_RUN_PATH_ENABLED != 0U)
/* solver_build_path() が生成した path[] 配列を trace 出力する。
   path[] は f413_solver_bridge.c の MAIN_C_ 定義で実体化されたグローバル配列。 */
extern uint16_t path[];

static const ShortestRunModeParams_t* nightfall_f413_shortest_mode_params(uint8_t mode)
{
  switch (mode)
  {
    case 2U: return &shortestRunModeParams2;
    case 3U: return &shortestRunModeParams3;
    case 4U: return &shortestRunModeParams4;
    case 5U: return &shortestRunModeParams5;
    case 6U: return &shortestRunModeParams6;
    case 7U: return &shortestRunModeParams7;
    default: return &shortestRunModeParams2;
  }
}

static const ShortestRunCaseParams_t* nightfall_f413_shortest_case_params(uint8_t mode, uint8_t case_index)
{
  uint8_t idx = 0U;

  if ((case_index >= 1U) && (case_index <= 9U))
  {
    idx = (uint8_t)(case_index - 1U);
  }

  switch (mode)
  {
    case 2U: return &shortestRunCaseParamsMode2[idx];
    case 3U: return &shortestRunCaseParamsMode3[idx];
    case 4U: return &shortestRunCaseParamsMode4[idx];
    case 5U: return &shortestRunCaseParamsMode5[idx];
    case 6U: return &shortestRunCaseParamsMode6[idx];
    case 7U: return &shortestRunCaseParamsMode7[idx];
    default: return &shortestRunCaseParamsMode2[idx];
  }
}

static float nightfall_f413_path_velocity_or_cap(float candidate, float fallback)
{
  float v = candidate;

  if (v <= 0.0f)
  {
    v = fallback;
  }
  if (v <= 0.0f)
  {
    v = NIGHTFALL_F413_PATH_VELOCITY;
  }
  if (v > NIGHTFALL_F413_PATH_VELOCITY_CAP)
  {
    v = NIGHTFALL_F413_PATH_VELOCITY_CAP;
  }

  return v;
}

static float nightfall_f413_op_test_velocity_for_mode(uint8_t mode, bool fast)
{
  float v = 150.0f + ((float)((mode > 2U) ? (mode - 2U) : 0U) * 50.0f);
  if (fast)
  {
    v += 75.0f;
  }
  if (v > NIGHTFALL_F413_OP_TEST_VEL_CAP)
  {
    v = NIGHTFALL_F413_OP_TEST_VEL_CAP;
  }
  return v;
}

static bool nightfall_f413_path_turn_from_code(uint16_t code,
                                               const ShortestRunModeParams_t* params,
                                               float* signed_angle_deg,
                                               float* abs_angle_deg)
{
  float angle = 0.0f;
  bool right = false;

  if ((params == NULL) || (signed_angle_deg == NULL) || (abs_angle_deg == NULL))
  {
    return false;
  }

  switch (code)
  {
    case 300U: angle = params->angle_turn_90; right = true; break;
    case 400U: angle = params->angle_turn_90; right = false; break;
    case 501U: angle = params->angle_l_turn_90; right = true; break;
    case 601U: angle = params->angle_l_turn_90; right = false; break;
    case 502U: angle = params->angle_l_turn_180; right = true; break;
    case 602U: angle = params->angle_l_turn_180; right = false; break;
    case 701U: angle = params->angle_turn45in; right = true; break;
    case 702U: angle = params->angle_turn45in; right = false; break;
    case 703U: angle = params->angle_turn45out; right = true; break;
    case 704U: angle = params->angle_turn45out; right = false; break;
    case 801U: angle = params->angle_turnV90; right = true; break;
    case 802U: angle = params->angle_turnV90; right = false; break;
    case 901U: angle = params->angle_turn135in; right = true; break;
    case 902U: angle = params->angle_turn135in; right = false; break;
    case 903U: angle = params->angle_turn135out; right = true; break;
    case 904U: angle = params->angle_turn135out; right = false; break;
    default: return false;
  }

  if (angle <= 0.0f)
  {
    angle = 90.0f;
  }

  *abs_angle_deg = fabsf(angle);
  *signed_angle_deg = right ? -*abs_angle_deg : *abs_angle_deg;
  return true;
}

static void nightfall_solver_path_print_preview(void)
{
  uint16_t count = 0U;
  uint16_t limit;
  uint16_t i;

  while ((count < 1024U) && (path[count] != 0U))
  {
    count++;
  }

  limit = (count > NIGHTFALL_F413_PATH_PREVIEW_MAX) ? NIGHTFALL_F413_PATH_PREVIEW_MAX : count;
  trace_printf("[RUN-TEST] solver-path codes(%u): ", (unsigned int)count);

  for (i = 0U; i < limit; i++)
  {
    uint16_t code = path[i];
    if ((code > 200U) && (code < 300U))
    {
      trace_printf("S%u", (unsigned int)(code - 200U));
    }
    else if (code == 300U)
    {
      trace_printf("s-R90");
    }
    else if (code == 400U)
    {
      trace_printf("s-L90");
    }
    else if (code == 501U || code == 502U)
    {
      trace_printf("L-R%s", (code == 501U) ? "90" : "180");
    }
    else if (code == 601U || code == 602U)
    {
      trace_printf("L-L%s", (code == 601U) ? "90" : "180");
    }
    else if ((code >= 701U) && (code <= 704U))
    {
      trace_printf("D45-%u", (unsigned int)(code - 700U));
    }
    else if ((code >= 801U) && (code <= 802U))
    {
      trace_printf("V90-%u", (unsigned int)(code - 800U));
    }
    else if ((code >= 901U) && (code <= 904U))
    {
      trace_printf("D135-%u", (unsigned int)(code - 900U));
    }
    else if (code > 1000U)
    {
      trace_printf("DS%u", (unsigned int)(code - 1000U));
    }
    else
    {
      trace_printf("%u", (unsigned int)code);
    }

    if ((i + 1U) < limit)
    {
      trace_printf(",");
    }
  }
  if (count > limit)
  {
    trace_printf(",...");
  }
  trace_printf("\r\n");
}

/* 距離 or 角度が目標に到達するまで待つヘルパー。
   guard チェック + trace auto step 付き。
   戻り値: abort reason (NONE = 正常到達) */
static nightfall_run_abort_reason_t nightfall_wait_ctrl_target(
    float target, bool is_angle,
    nightfall_run_guard_t* guard, uint16_t trace_flags)
{
  nightfall_run_abort_reason_t reason = NIGHTFALL_RUN_ABORT_NONE;
  uint32_t deadline = HAL_GetTick() + NIGHTFALL_F413_PATH_TIMEOUT_MS;

  while (1)
  {
    if (is_angle)
    {
      float current = f413_ctrl_get_angle();
      if (((target >= 0.0f) && (current >= target)) ||
          ((target < 0.0f) && (current <= target)))
      {
        break;
      }
    }
    else
    {
      float current = fabsf(f413_ctrl_get_distance());
      if (current >= fabsf(target))
      {
        break;
      }
    }

    if (HAL_GetTick() >= deadline)
    {
      break;
    }

    nightfall_trace_log_set_mode_flags(trace_flags);
    reason = nightfall_trace_log_wait_with_auto_step_guarded(10U, guard);
    nightfall_wall_control_apply(!is_angle &&
        ((trace_flags & NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG) != 0U));
    if (reason != NIGHTFALL_RUN_ABORT_NONE)
    {
      return reason;
    }
  }
  return NIGHTFALL_RUN_ABORT_NONE;
}

static void nightfall_run_solver_path_session_once(uint16_t base_trace_flag)
{
  nightfall_run_abort_reason_t abort_reason = NIGHTFALL_RUN_ABORT_NONE;
  nightfall_run_guard_t guard = {0};
  const ShortestRunModeParams_t* mode_params =
      nightfall_f413_shortest_mode_params(NIGHTFALL_F413_SOLVER_MODE);
  const ShortestRunCaseParams_t* case_params =
      nightfall_f413_shortest_case_params(NIGHTFALL_F413_SOLVER_MODE, NIGHTFALL_F413_SOLVER_CASE);
  const float straight_velocity =
      nightfall_f413_path_velocity_or_cap(case_params->velocity_straight, NIGHTFALL_F413_PATH_VELOCITY);
  const float diagonal_velocity =
      nightfall_f413_path_velocity_or_cap(case_params->velocity_d_straight, straight_velocity);
  uint16_t pi;
  uint16_t code;

  if (f413_trace_log_auto_is_enabled())
  {
    trace_printf("[RUN-TEST] busy(auto already running)\r\n");
    return;
  }

  if (nightfall_run_stop_switch_pressed())
  {
    trace_printf("[RUN-TEST] solver-path canceled(start switch pressed)\r\n");
    return;
  }

  trace_printf("[RUN-TEST] solver-path session start (mode=%u case=%u v=%.0f diag_v=%.0f cap=%.0f, press switch to abort)\r\n",
               (unsigned int)NIGHTFALL_F413_SOLVER_MODE,
               (unsigned int)NIGHTFALL_F413_SOLVER_CASE,
               (double)straight_velocity,
               (double)diagonal_velocity,
               (double)NIGHTFALL_F413_PATH_VELOCITY_CAP);

  if (!nightfall_run_guard_prepare(&guard))
  {
    trace_printf("[RUN-TEST] solver-path canceled(guard init fail)\r\n");
    return;
  }

  nightfall_trace_log_on_run_start();
  f413_ctrl_start();

  for (pi = 0U; pi < NIGHTFALL_F413_PATH_MAX_CODES; pi++)
  {
    code = path[pi];
    if (code == 0U)
    {
      break;
    }

    f413_ctrl_reset_distance();
    f413_ctrl_reset_angle();

    if ((code > 200U) && (code < 300U))
    {
      /* Straight: code - 200 = half-cell count */
      uint16_t half_cells = (uint16_t)(code - 200U);
      float target_mm = (float)half_cells * NIGHTFALL_F413_PATH_HALF_CELL_MM;

      f413_ctrl_set_velocity(straight_velocity);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);

      abort_reason = nightfall_wait_ctrl_target(target_mm, false, &guard,
          (uint16_t)(base_trace_flag | NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                     NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
    }
    else if (code > 1000U)
    {
      /* Diagonal straight: forward motion along current heading */
      uint16_t diag_half = (uint16_t)(code - 1000U);
      float target_mm = (float)diag_half * NIGHTFALL_F413_PATH_DIAG_HALF_CELL_MM;

      f413_ctrl_set_velocity(diagonal_velocity);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);

      abort_reason = nightfall_wait_ctrl_target(target_mm, false, &guard,
          (uint16_t)(base_trace_flag | NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                     NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
    }
    else
    {
      float signed_angle = 0.0f;
      float abs_angle = 0.0f;

      if (nightfall_f413_path_turn_from_code(code, mode_params, &signed_angle, &abs_angle))
      {
        f413_ctrl_set_velocity(0.0f);
        f413_ctrl_set_omega(0.0f);
        f413_ctrl_set_angle_target(signed_angle);

        abort_reason = nightfall_wait_ctrl_target(signed_angle, true, &guard,
            (uint16_t)(base_trace_flag | NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                       NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG));
      }
      else
      {
        trace_printf("[RUN-TEST] unsupported path code %u at [%u]\r\n",
                     (unsigned int)code, (unsigned int)pi);
        abort_reason = NIGHTFALL_RUN_ABORT_IMU_FAULT;
      }
    }

    if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
    {
      break;
    }

    /* Coast between segments */
    f413_ctrl_clear_angle_target();
    f413_ctrl_set_velocity(0.0f);
    f413_ctrl_set_omega(0.0f);
    nightfall_trace_log_set_mode_flags((uint16_t)(base_trace_flag |
                                                   NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                                                   NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG));
    abort_reason = nightfall_trace_log_wait_with_auto_step_guarded(NIGHTFALL_F413_PATH_COAST_MS, &guard);
    if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
    {
      break;
    }
  }

  f413_ctrl_stop();

  if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
  {
    nightfall_trace_log_set_mode_flags((uint16_t)(base_trace_flag |
                                                   NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG |
                                                   nightfall_run_abort_reason_to_trace_flag(abort_reason)));
    nightfall_trace_log_auto_step();
  }

  nightfall_trace_log_set_mode_flags(0U);
  nightfall_trace_log_on_run_stop();

  nightfall_run_guard_cleanup(&guard);

  if (abort_reason == NIGHTFALL_RUN_ABORT_SWITCH)
  {
    trace_printf("[RUN-TEST] solver-path aborted by switch at code[%u]\r\n", (unsigned int)pi);
  }
  else if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
  {
    trace_printf("[RUN-TEST] solver-path aborted(%s) at code[%u]\r\n",
                 nightfall_run_abort_reason_to_text(abort_reason),
                 (unsigned int)pi);
  }
  else
  {
    trace_printf("[RUN-TEST] solver-path end (%u codes, dist=%.0fmm, angle=%.0fdeg)\r\n",
                 (unsigned int)pi,
                 (double)f413_ctrl_get_distance(),
                 (double)f413_ctrl_get_angle());
  }
}

static void nightfall_run_path_code_sequence_once(const char* label, uint8_t mode, uint8_t case_index,
                                                   const uint16_t* codes, uint16_t code_count,
                                                   float straight_velocity, float diagonal_velocity)
{
  nightfall_run_abort_reason_t abort_reason = NIGHTFALL_RUN_ABORT_NONE;
  nightfall_run_guard_t guard = {0};
  const ShortestRunModeParams_t* mode_params = nightfall_f413_shortest_mode_params(mode);
  uint16_t i;
  uint16_t code = 0U;

  if ((codes == NULL) || (code_count == 0U))
  {
    trace_printf("[OP-UI][PATH-TEST] no sequence\r\n");
    return;
  }
  if (f413_trace_log_auto_is_enabled())
  {
    trace_printf("[OP-UI][PATH-TEST] busy(auto already running)\r\n");
    return;
  }
  if (nightfall_run_stop_switch_pressed())
  {
    trace_printf("[OP-UI][PATH-TEST] canceled(start switch pressed)\r\n");
    return;
  }
  if (!nightfall_run_guard_prepare(&guard))
  {
    trace_printf("[OP-UI][PATH-TEST] canceled(guard init fail)\r\n");
    return;
  }

  trace_printf("[OP-UI][PATH-TEST] start %s mode=%u case=%u v=%.0f diag_v=%.0f codes=",
               label,
               (unsigned int)mode,
               (unsigned int)case_index,
               (double)straight_velocity,
               (double)diagonal_velocity);
  for (i = 0U; i < code_count; i++)
  {
    trace_printf("%u%s", (unsigned int)codes[i], ((i + 1U) < code_count) ? "," : "");
  }
  trace_printf("\r\n");

  nightfall_trace_log_on_run_start();
  f413_ctrl_start();

  for (i = 0U; i < code_count; i++)
  {
    code = codes[i];
    f413_ctrl_reset_distance();
    f413_ctrl_reset_angle();

    if ((code > 200U) && (code < 300U))
    {
      uint16_t half_cells = (uint16_t)(code - 200U);
      float target_mm = (float)half_cells * NIGHTFALL_F413_PATH_HALF_CELL_MM;
      f413_ctrl_set_velocity(straight_velocity);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);
      abort_reason = nightfall_wait_ctrl_target(target_mm, false, &guard,
          (uint16_t)(NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG | NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
    }
    else if (code > 1000U)
    {
      uint16_t diag_half = (uint16_t)(code - 1000U);
      float target_mm = (float)diag_half * NIGHTFALL_F413_PATH_DIAG_HALF_CELL_MM;
      f413_ctrl_set_velocity(diagonal_velocity);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);
      abort_reason = nightfall_wait_ctrl_target(target_mm, false, &guard,
          (uint16_t)(NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG | NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
    }
    else
    {
      float signed_angle = 0.0f;
      float abs_angle = 0.0f;
      if (nightfall_f413_path_turn_from_code(code, mode_params, &signed_angle, &abs_angle))
      {
        f413_ctrl_set_velocity(0.0f);
        f413_ctrl_set_omega(0.0f);
        f413_ctrl_set_angle_target(signed_angle);
        abort_reason = nightfall_wait_ctrl_target(signed_angle, true, &guard,
            (uint16_t)(NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG | NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG));
      }
      else
      {
        abort_reason = NIGHTFALL_RUN_ABORT_IMU_FAULT;
      }
    }

    if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
    {
      break;
    }

    f413_ctrl_clear_angle_target();
    f413_ctrl_set_velocity(0.0f);
    f413_ctrl_set_omega(0.0f);
    nightfall_trace_log_set_mode_flags((uint16_t)(NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG | NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG));
    abort_reason = nightfall_trace_log_wait_with_auto_step_guarded(NIGHTFALL_F413_PATH_COAST_MS, &guard);
    if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
    {
      break;
    }
  }

  f413_ctrl_clear_angle_target();
  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);
  f413_ctrl_stop();
  if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
  {
    nightfall_trace_log_set_mode_flags((uint16_t)(NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG | nightfall_run_abort_reason_to_trace_flag(abort_reason)));
    nightfall_trace_log_auto_step();
  }
  nightfall_trace_log_set_mode_flags(0U);
  nightfall_trace_log_on_run_stop();
  nightfall_run_guard_cleanup(&guard);
  g_last_test_id = (uint8_t)'P';
  g_last_test_abort_reason = abort_reason;
  g_last_test_distance_mm = f413_ctrl_get_distance();
  g_last_test_angle_deg = f413_ctrl_get_angle();
  trace_printf("[OP-UI][PATH-TEST] %s %s at code[%u]=%u dist=%.0fmm angle=%.0fdeg\r\n",
               label,
               (abort_reason == NIGHTFALL_RUN_ABORT_NONE) ? "OK" : nightfall_run_abort_reason_to_text(abort_reason),
               (unsigned int)i,
               (unsigned int)code,
               (double)g_last_test_distance_mm,
               (double)g_last_test_angle_deg);
}
#endif

/* ========================================================== */
/* 調整用テストモード                                          */
/* test_id: '1'=S6直進, '2'=S12直進, '3'=右90°, '4'=左90°,    */
/*          '5'=S6+R90+S6                                      */
/* ========================================================== */

static void nightfall_run_control_tune_once(uint8_t axis, uint8_t set, uint8_t pattern)
{
  nightfall_run_abort_reason_t abort_reason = NIGHTFALL_RUN_ABORT_NONE;
  nightfall_run_guard_t guard = {0};
  uint32_t deadline;
  uint16_t flags = NIGHTFALL_F413_TRACE_MODE_TUNE_FLAG;

  if (f413_trace_log_auto_is_enabled())
  {
    trace_printf("[TUNE] busy(auto already running)\r\n");
    return;
  }
  if (nightfall_run_stop_switch_pressed())
  {
    trace_printf("[TUNE] canceled(start switch pressed)\r\n");
    return;
  }
  if (!nightfall_run_guard_prepare(&guard))
  {
    trace_printf("[TUNE] canceled(guard init fail)\r\n");
    return;
  }

  if ((axis == F413_CTRL_TUNE_AXIS_VELOCITY) ||
      (axis == F413_CTRL_TUNE_AXIS_DISTANCE))
  {
    flags |= NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG;
  }
  else
  {
    flags |= NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG;
  }

  g_last_tune_valid = true;
  g_last_tune_axis = axis;
  g_last_tune_set = set;
  g_last_tune_pattern = pattern;

  trace_printf("[TUNE] start axis=%s set=%u pattern=%s\r\n",
               nightfall_tune_axis_name(axis),
               (unsigned int)set,
               nightfall_tune_pattern_name(pattern));

  nightfall_trace_log_on_run_start();
  f413_ctrl_start();
  f413_ctrl_tune_clear_done();
  f413_ctrl_tune_start(axis, set, pattern);

  deadline = HAL_GetTick() + NIGHTFALL_F413_TUNE_TIMEOUT_MS;
  while (!f413_ctrl_tune_is_done())
  {
    if (HAL_GetTick() >= deadline)
    {
      abort_reason = NIGHTFALL_RUN_ABORT_IMU_FAULT;
      break;
    }
    nightfall_trace_log_set_mode_flags(flags);
    abort_reason = nightfall_trace_log_wait_with_auto_step_guarded(10U, &guard);
    if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
    {
      break;
    }
  }

  f413_ctrl_tune_stop();
  f413_ctrl_stop();
  if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
  {
    nightfall_trace_log_set_mode_flags((uint16_t)(flags |
        nightfall_run_abort_reason_to_trace_flag(abort_reason)));
    nightfall_trace_log_auto_step();
  }
  nightfall_trace_log_set_mode_flags(0U);
  nightfall_trace_log_on_run_stop();
  nightfall_run_guard_cleanup(&guard);

  g_last_test_id = (uint8_t)'0';
  g_last_test_abort_reason = abort_reason;
  g_last_test_distance_mm = f413_ctrl_get_distance();
  g_last_test_angle_deg = f413_ctrl_get_angle();

  trace_printf("[TUNE] %s axis=%s set=%u pattern=%s dist=%.0fmm angle=%.0fdeg\r\n",
               (abort_reason == NIGHTFALL_RUN_ABORT_NONE) ? "OK" :
               nightfall_run_abort_reason_to_text(abort_reason),
               nightfall_tune_axis_name(axis),
               (unsigned int)set,
               nightfall_tune_pattern_name(pattern),
               (double)g_last_test_distance_mm,
               (double)g_last_test_angle_deg);
}
static void nightfall_trace_log_emit_identity_meta(const nvm_identity_block_t* id)
{
  const char* family_name;

  if (id == NULL)
  {
    return;
  }

  family_name = nightfall_identity_family_name(id->family);
  trace_printf("#fw_family=%lu\r\n", (unsigned long)id->family);
  trace_printf("#fw_family_name=%s\r\n", family_name);
  trace_printf("#fw_machine_name=%s_r%u_%u\r\n",
               family_name,
               (unsigned int)id->hw_rev_major,
               (unsigned int)id->hw_rev_minor);
  trace_printf("#fw_machine_unit=%s_r%u_%u_unit%03lu\r\n",
               family_name,
               (unsigned int)id->hw_rev_major,
               (unsigned int)id->hw_rev_minor,
               (unsigned long)id->unit_serial);
  trace_printf("#fw_board_id=%lu\r\n", (unsigned long)id->board_id);
  trace_printf("#fw_board_id_hex=0x%08lX\r\n", (unsigned long)id->board_id);
  trace_printf("#fw_hw_rev=%u.%u\r\n",
               (unsigned int)id->hw_rev_major,
               (unsigned int)id->hw_rev_minor);
  trace_printf("#fw_unit_serial=%lu\r\n", (unsigned long)id->unit_serial);
}

static void nightfall_identity_enter_safe_mode(void)
{
  while (1)
  {
    HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
    HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
    HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
    HAL_Delay(150U);
  }
}

static void nightfall_trace_log_set_context(uint8_t mode, uint8_t op_case, uint8_t sub, uint8_t test_id)
{
  g_trace_log_context_mode = mode;
  g_trace_log_context_case = op_case;
  g_trace_log_context_sub = sub;
  g_trace_log_context_test_id = test_id;
}

static int32_t nightfall_trace_log_scale_float(float value, float scale)
{
  float scaled = value * scale;

  if (scaled > 2147483000.0f)
  {
    return 2147483000L;
  }
  if (scaled < -2147483000.0f)
  {
    return -2147483000L;
  }
  return (int32_t)lrintf(scaled);
}

static bool nightfall_wall_sensor_start_async(void)
{
  return f413_wall_sensor_start_async();
}

static void nightfall_wall_sensor_tim6_tick(void)
{
  f413_wall_sensor_tim6_tick();
}

static bool nightfall_trace_log_read_adc_raw(uint16_t* fr, uint16_t* r, uint16_t* fl, uint16_t* l, uint16_t* vbat)
{
  return f413_wall_sensor_read_adc_raw(fr, r, fl, l, vbat);
}

static void nightfall_wall_end_clear(void)
{
  memset(&g_wall_end, 0, sizeof(g_wall_end));
}

static uint16_t nightfall_wall_trace_flags_from_snapshot(const nightfall_wall_sensor_snapshot_t* wall,
                                                         bool gate_on)
{
  uint16_t flags = NIGHTFALL_F413_WALL_TRACE_ENABLED_FLAG;

  if (wall == NULL)
  {
    return flags;
  }
  if (wall->front_wall)
  {
    flags |= NIGHTFALL_F413_WALL_TRACE_FRONT_FLAG;
  }
  if (wall->right_wall)
  {
    flags |= NIGHTFALL_F413_WALL_TRACE_RIGHT_FLAG;
  }
  if (wall->left_wall)
  {
    flags |= NIGHTFALL_F413_WALL_TRACE_LEFT_FLAG;
  }
  if (wall->saturated)
  {
    flags |= NIGHTFALL_F413_WALL_TRACE_SAT_FLAG;
  }
  if (g_wall_end.right_wall)
  {
    flags |= NIGHTFALL_F413_WALL_TRACE_END_WALL_R_FLAG;
  }
  if (g_wall_end.left_wall)
  {
    flags |= NIGHTFALL_F413_WALL_TRACE_END_WALL_L_FLAG;
  }
  if (g_wall_end.detected_r)
  {
    flags |= NIGHTFALL_F413_WALL_TRACE_END_R_FLAG;
  }
  if (g_wall_end.detected_l)
  {
    flags |= NIGHTFALL_F413_WALL_TRACE_END_L_FLAG;
  }
  if (gate_on)
  {
    flags |= NIGHTFALL_F413_WALL_TRACE_GATE_FLAG;
  }

  if (g_wall_ctrl_active)
  {
    flags |= NIGHTFALL_F413_WALL_TRACE_CTRL_FLAG;
  }
  return flags;
}

static void nightfall_wall_control_reset(void)
{
  g_wall_ctrl_angle_deg = 0.0f;
  g_wall_ctrl_error_lpf = 0.0f;
  g_wall_ctrl_active = false;
  f413_ctrl_set_heading_omega_correction(0.0f);
}

static void nightfall_wall_control_update(const nightfall_wall_sensor_snapshot_t* wall, bool gate_on)
{
  float wall_error = 0.0f;
  float target_deg;
  float delta;

#if (NIGHTFALL_F413_DISABLE_WALL_CONTROL != 0U)
  (void)wall;
  (void)gate_on;
  nightfall_wall_control_reset();
  return;
#else
  if ((wall == NULL) || !gate_on || wall->saturated ||
      (fabsf(f413_ctrl_get_target_velocity()) < NIGHTFALL_F413_WALL_CTRL_MIN_VEL_MM_S))
  {
    nightfall_wall_control_reset();
    return;
  }

  if (wall->right_wall && wall->left_wall)
  {
    wall_error = (float)(wall->l_delta - WALL_CTRL_BASE_L) -
                 (float)(wall->r_delta - WALL_CTRL_BASE_R);
  }
  else if (wall->right_wall && !wall->left_wall)
  {
    wall_error = -2.0f * (float)(wall->r_delta - WALL_BASE_R);
  }
  else if (!wall->right_wall && wall->left_wall)
  {
    wall_error = 2.0f * (float)(wall->l_delta - WALL_BASE_L);
  }
  else
  {
    nightfall_wall_control_reset();
    return;
  }

  g_wall_ctrl_error_lpf += NIGHTFALL_F413_WALL_CTRL_LPF_ALPHA *
                           (wall_error - g_wall_ctrl_error_lpf);
  target_deg = g_wall_ctrl_error_lpf * NIGHTFALL_F413_WALL_CTRL_KP_DEG_PER_ADC;
  if (fabsf(target_deg) < WALL_CTRL_MIN)
  {
    target_deg = 0.0f;
  }
  if (target_deg > NIGHTFALL_F413_WALL_CTRL_MAX_DEG)
  {
    target_deg = NIGHTFALL_F413_WALL_CTRL_MAX_DEG;
  }
  else if (target_deg < -NIGHTFALL_F413_WALL_CTRL_MAX_DEG)
  {
    target_deg = -NIGHTFALL_F413_WALL_CTRL_MAX_DEG;
  }

  delta = target_deg - g_wall_ctrl_angle_deg;
  if (delta > NIGHTFALL_F413_WALL_CTRL_SLEW_DEG)
  {
    delta = NIGHTFALL_F413_WALL_CTRL_SLEW_DEG;
  }
  else if (delta < -NIGHTFALL_F413_WALL_CTRL_SLEW_DEG)
  {
    delta = -NIGHTFALL_F413_WALL_CTRL_SLEW_DEG;
  }

  g_wall_ctrl_angle_deg += delta;
  g_wall_ctrl_active = true;
#endif
}

static void nightfall_wall_control_apply(bool straight_gate)
{
#if (NIGHTFALL_F413_DISABLE_WALL_CONTROL != 0U)
  (void)straight_gate;
#else
  if (straight_gate && g_wall_ctrl_active)
  {
    f413_ctrl_set_heading_omega_correction(-g_wall_ctrl_angle_deg);
  }
  else
  {
    f413_ctrl_set_heading_omega_correction(0.0f);
  }
#endif
}

static bool nightfall_trace_log_fill_wall_observe(nvm_trace_log_record_t* out)
{
  nightfall_wall_sensor_snapshot_t wall;
  bool gate_on;
  int32_t dist_r_q4 = 0;
  int32_t dist_l_q4 = 0;

  if (out == NULL)
  {
    return false;
  }
  if (!nightfall_wall_sensor_read_snapshot(&wall))
  {
    return false;
  }

  gate_on = (f413_trace_log_get_mode_flags() & NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG) != 0U;
  nightfall_wall_end_update(&wall, gate_on);
  nightfall_wall_control_update(&wall, gate_on);

  out->adc_fr = (uint16_t)wall.fr_delta;
  out->adc_r = (uint16_t)wall.r_delta;
  out->adc_fl = (uint16_t)wall.fl_delta;
  out->adc_l = (uint16_t)wall.l_delta;
  out->adc_vbat = wall.vbat_on;
  out->reserved_i32_0 = wall.fr_delta;
  out->reserved_i32_1 = wall.r_delta;
  out->reserved_i32_2 = wall.fl_delta;
  out->reserved_i32_3 = wall.l_delta;
  out->reserved_u16_0 = nightfall_wall_trace_flags_from_snapshot(&wall, gate_on);

  if (g_wall_end.detected_r)
  {
    dist_r_q4 = nightfall_trace_log_scale_float(g_wall_end.dist_r_mm, 0.25f);
  }
  if (g_wall_end.detected_l)
  {
    dist_l_q4 = nightfall_trace_log_scale_float(g_wall_end.dist_l_mm, 0.25f);
  }
  if (dist_r_q4 < 0)
  {
    dist_r_q4 = 0;
  }
  if (dist_l_q4 < 0)
  {
    dist_l_q4 = 0;
  }
  if (dist_r_q4 > 255)
  {
    dist_r_q4 = 255;
  }
  if (dist_l_q4 > 255)
  {
    dist_l_q4 = 255;
  }
  out->reserved_u16_1 = (uint16_t)(((uint16_t)dist_l_q4 << 8U) | (uint16_t)dist_r_q4);

  return true;
}

static void nightfall_trace_log_update_observe_cache(void)
{
  nvm_trace_log_record_t rec;
  uint16_t adc_fr = 0U;
  uint16_t adc_r = 0U;
  uint16_t adc_fl = 0U;
  uint16_t adc_l = 0U;
  uint16_t adc_vbat = 0U;

#if (NIGHTFALL_F413_DISABLE_WALL_TRACE_OBSERVE == 0U)
  memset(&rec, 0, sizeof(rec));
  if (nightfall_trace_log_fill_wall_observe(&rec))
  {
    g_trace_log_adc_fr = rec.adc_fr;
    g_trace_log_adc_r = rec.adc_r;
    g_trace_log_adc_fl = rec.adc_fl;
    g_trace_log_adc_l = rec.adc_l;
    g_trace_log_adc_vbat = rec.adc_vbat;
    g_trace_log_reserved_i32_0 = rec.reserved_i32_0;
    g_trace_log_reserved_i32_1 = rec.reserved_i32_1;
    g_trace_log_reserved_i32_2 = rec.reserved_i32_2;
    g_trace_log_reserved_i32_3 = rec.reserved_i32_3;
    g_trace_log_reserved_u16_0 = rec.reserved_u16_0;
    g_trace_log_reserved_u16_1 = rec.reserved_u16_1;
    return;
  }
#endif

  if (nightfall_trace_log_read_adc_raw(&adc_fr, &adc_r, &adc_fl, &adc_l, &adc_vbat))
  {
    g_trace_log_adc_fr = adc_fr;
    g_trace_log_adc_r = adc_r;
    g_trace_log_adc_fl = adc_fl;
    g_trace_log_adc_l = adc_l;
    g_trace_log_adc_vbat = adc_vbat;
  }
}

static void nightfall_run_trace_log_dump_csv_once(void)
{
  f413_trace_diag_run_dump_csv_once();
}

static void nightfall_run_trace_log_dump_csv_all_once(void)
{
  f413_trace_diag_run_dump_csv_all_once();
}

static void nightfall_run_trace_log_dump_bin_once(void)
{
  f413_trace_diag_run_dump_bin_once();
}

static void nightfall_run_trace_log_dump_bin_all_once(void)
{
  f413_trace_diag_run_dump_bin_all_once();
}

static void nightfall_run_trace_log_selftest_once(void)
{
  f413_trace_diag_run_selftest_once();
}

static void nightfall_print_nvm_cli_help(void)
{
  trace_printf("[NVM-TEST] commands: h=help, a=save+load all, A=load-only all\r\n");
  trace_printf("[NVM-TEST] d/s/m/t=save+load, D/S/M/T=load-only verify\r\n");
  trace_printf("[TRACE-LOG] q=format, r=append sample, R=dump latest, v/V=dump csv(256/all), </>=dump bin(256/all), k=selftest, u=run-start hook, U=run-stop hook\r\n");
  trace_printf("[RUN-TEST]  x=idle-run-session(1000ms), y=motor-run-session(short), z=search-entry(solver/fallback), j=shortest-entry(solver/fallback)\r\n");
  trace_printf("[HW-TEST]  w=wall, W=wall-end, O=search-map, G=search-preview, B=search-reset, N=search-step, [/]/@=state/clear/dump, p=switch, i=imu, I=imu-angle, c=imu-accel, b=buzzer, o/0=motor, e=encoder, l=led30s, g=smoke+trace\r\n");
  trace_printf("[TEST]     1=S3straight, 2=S6straight, 3=R90turn, 4=L90turn, 5=S3+R90+S3, F=arm for button; OP mode9/case0/sub0-9=control tune\r\n");
  trace_printf("[TEST]     OP mode2-7/case0/sub0-9=path-code tests\r\n");
  trace_printf("[TUNE]     !/\"/#/$/%%/^/&/*/(/)=OP mode9 case0 sub0..9 shortcut, then V=dump CSV\r\n");
  trace_printf("[HW-ENC]  6=L-motor-fwd, 7=R-motor-fwd, 8=L-motor-rev, 9=R-motor-rev (open-loop+enc)\r\n");
  trace_printf("[OP-UI]   F405-compatible select: PUSH increments 0..9 at each level, FR wall only=enter, mode9 case0=tune, case5=dump latest full log(bin)\r\n");
  trace_printf("[OP-UART] P=PUSH increment, E=FR enter; reset via ST-LINK software reset\r\n");
}

static void nightfall_trace_diag_get_context(uint8_t* mode, uint8_t* op_case, uint8_t* sub, uint8_t* test_id)
{
  if (mode != NULL)
  {
    *mode = g_trace_log_context_mode;
  }
  if (op_case != NULL)
  {
    *op_case = g_trace_log_context_case;
  }
  if (sub != NULL)
  {
    *sub = g_trace_log_context_sub;
  }
  if (test_id != NULL)
  {
    *test_id = g_trace_log_context_test_id;
  }
}

static void nightfall_trace_diag_emit_extra_csv_meta(void)
{
#if (NIGHTFALL_F413_DISABLE_WALL_TRACE_OBSERVE == 0U)
  trace_printf("#wall_trace_observe=%u\r\n", (unsigned int)NIGHTFALL_F413_WALL_TRACE_VERSION);
  trace_printf("#wall_trace_reserved_i32=delta_fr,delta_r,delta_fl,delta_l\r\n");
  trace_printf("#wall_trace_reserved_u16_0=flags\r\n");
  trace_printf("#wall_trace_reserved_u16_1=dist_q4_lr\r\n");
#else
  trace_printf("#wall_trace_observe=disabled\r\n");
#endif
  if (g_last_tune_valid)
  {
    trace_printf("#tune_axis=%s\r\n", nightfall_tune_axis_name(g_last_tune_axis));
    trace_printf("#tune_set=%u\r\n", (unsigned int)g_last_tune_set);
    trace_printf("#tune_pattern=%s\r\n", nightfall_tune_pattern_name(g_last_tune_pattern));
    trace_printf("#tune_reserved_i32=primary_ref_x1000,axis,target_distance_x1000,target_angle_x1000\r\n");
    trace_printf("#tune_reserved_u16_0=axis_pattern\r\n");
    trace_printf("#tune_reserved_u16_1=set\r\n");
  }
  if (g_boot_identity_status == NVM_STATUS_OK)
  {
    nightfall_trace_log_emit_identity_meta(&g_boot_identity);
  }
  else
  {
    trace_printf("#fw_identity_status=%d\r\n", (int)g_boot_identity_status);
  }
  if (g_last_test_id != 0U)
  {
    trace_printf("#last_test_id=%c\r\n", (char)g_last_test_id);
    trace_printf("#last_test_status=%s\r\n",
                 (g_last_test_abort_reason == NIGHTFALL_RUN_ABORT_NONE) ? "OK" :
                 nightfall_run_abort_reason_to_text(g_last_test_abort_reason));
    trace_printf("#last_test_dist_mm=%.0f\r\n", (double)g_last_test_distance_mm);
    trace_printf("#last_test_angle_deg=%.0f\r\n", (double)g_last_test_angle_deg);
  }
}

static void nightfall_run_trace_log_format_once(void)
{
  f413_trace_diag_run_format_once();
}

static void nightfall_fill_trace_log_sample(nvm_trace_log_record_t* out, uint32_t seq)
{
  uint16_t adc_fr = 0U;
  uint16_t adc_r = 0U;
  uint16_t adc_fl = 0U;
  uint16_t adc_l = 0U;
  uint16_t adc_vbat = 0U;

  if (out == NULL)
  {
    return;
  }

  memset(out, 0, sizeof(*out));
  out->seq = seq;
  out->timestamp_ms = HAL_GetTick();
  out->target_distance_x1000 = nightfall_trace_log_scale_float(f413_ctrl_get_target_distance(), 1000.0f);
  out->distance_mm = nightfall_trace_log_scale_float(f413_ctrl_get_distance(), 1.0f);
  out->angle_mdeg = nightfall_trace_log_scale_float(f413_ctrl_get_log_angle(), 1000.0f);
  out->target_velocity_mm_s = nightfall_trace_log_scale_float(f413_ctrl_get_target_velocity(), 1.0f);
  out->real_velocity_mm_s = nightfall_trace_log_scale_float(f413_ctrl_get_real_velocity(), 1.0f);
  out->accel_velocity_mm_s = nightfall_trace_log_scale_float(f413_ctrl_get_accel_velocity(), 1.0f);
  out->target_omega_mdps = nightfall_trace_log_scale_float(f413_ctrl_get_target_omega(), 1000.0f);
  out->real_omega_mdps = nightfall_trace_log_scale_float(f413_ctrl_get_log_real_omega(), 1000.0f);
  out->target_angle_mdeg = nightfall_trace_log_scale_float(f413_ctrl_get_target_angle(), 1000.0f);
  out->accel_forward_mm_s2 = nightfall_trace_log_scale_float(f413_ctrl_get_accel_forward(), 1.0f);
  if (f413_ctrl_is_running())
  {
    out->encoder_l = f413_ctrl_get_log_encoder_delta_l();
    out->encoder_r = f413_ctrl_get_log_encoder_delta_r();
  }
  else
  {
    out->encoder_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    out->encoder_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  }
  out->motor_out_l = f413_ctrl_get_motor_out_l();
  out->motor_out_r = f413_ctrl_get_motor_out_r();
#if (NIGHTFALL_F413_DISABLE_WALL_TRACE_OBSERVE == 0U)
  if (nightfall_trace_log_fill_wall_observe(out))
  {
    adc_fr = out->adc_fr;
    adc_r = out->adc_r;
    adc_fl = out->adc_fl;
    adc_l = out->adc_l;
    adc_vbat = out->adc_vbat;
  }
  else
#endif
  if (nightfall_trace_log_read_adc_raw(&adc_fr, &adc_r, &adc_fl, &adc_l, &adc_vbat))
  {
    out->adc_fr = adc_fr;
    out->adc_r = adc_r;
    out->adc_fl = adc_fl;
    out->adc_l = adc_l;
    out->adc_vbat = adc_vbat;
  }
  if ((f413_trace_log_get_mode_flags() & NIGHTFALL_F413_TRACE_MODE_TUNE_FLAG) != 0U)
  {
    out->reserved_i32_0 = nightfall_trace_log_scale_float(f413_ctrl_tune_get_reference(), 1000.0f);
    out->reserved_i32_1 = (int32_t)f413_ctrl_tune_get_axis();
    out->reserved_i32_2 = nightfall_trace_log_scale_float(f413_ctrl_get_target_distance(), 1000.0f);
    out->reserved_i32_3 = nightfall_trace_log_scale_float(f413_ctrl_get_target_angle(), 1000.0f);
    out->reserved_u16_0 = (uint16_t)(((uint16_t)f413_ctrl_tune_get_axis() << 8U) |
                                     (uint16_t)f413_ctrl_tune_get_pattern());
    out->reserved_u16_1 = (uint16_t)f413_ctrl_tune_get_set();
  }
  out->flags = f413_hw_stop_switch_pressed()
                   ? NIGHTFALL_F413_TRACE_SWITCH_FLAG
                   : 0U;
  if (f413_ctrl_angle_target_enabled())
  {
    out->flags |= NIGHTFALL_F413_TRACE_ANGLE_TARGET_FLAG;
  }
  out->op_mode = g_trace_log_context_mode;
  out->op_case = g_trace_log_context_case;
  out->op_sub = g_trace_log_context_sub;
  out->test_id = g_trace_log_context_test_id;
}

static void nightfall_fill_trace_log_control_sample(nvm_trace_log_record_t* out,
                                                    uint32_t seq,
                                                    uint32_t timestamp_ms,
                                                    uint16_t mode_flags)
{
  nightfall_wall_sensor_snapshot_t wall;

  if (out == NULL)
  {
    return;
  }

  memset(out, 0, sizeof(*out));
  out->seq = seq;
  out->timestamp_ms = timestamp_ms;
  out->target_distance_x1000 = nightfall_trace_log_scale_float(f413_ctrl_get_target_distance(), 1000.0f);
  out->distance_mm = nightfall_trace_log_scale_float(f413_ctrl_get_distance(), 1.0f);
  out->angle_mdeg = nightfall_trace_log_scale_float(f413_ctrl_get_log_angle(), 1000.0f);
  out->target_velocity_mm_s = nightfall_trace_log_scale_float(f413_ctrl_get_target_velocity(), 1.0f);
  out->real_velocity_mm_s = nightfall_trace_log_scale_float(f413_ctrl_get_real_velocity(), 1.0f);
  out->accel_velocity_mm_s = nightfall_trace_log_scale_float(f413_ctrl_get_accel_velocity(), 1.0f);
  out->target_omega_mdps = nightfall_trace_log_scale_float(f413_ctrl_get_target_omega(), 1000.0f);
  out->real_omega_mdps = nightfall_trace_log_scale_float(f413_ctrl_get_log_real_omega(), 1000.0f);
  out->target_angle_mdeg = nightfall_trace_log_scale_float(f413_ctrl_get_target_angle(), 1000.0f);
  out->accel_forward_mm_s2 = nightfall_trace_log_scale_float(f413_ctrl_get_accel_forward(), 1.0f);
  out->encoder_l = f413_ctrl_get_log_encoder_delta_l();
  out->encoder_r = f413_ctrl_get_log_encoder_delta_r();
  out->motor_out_l = f413_ctrl_get_motor_out_l();
  out->motor_out_r = f413_ctrl_get_motor_out_r();
  out->adc_fr = g_trace_log_adc_fr;
  out->adc_r = g_trace_log_adc_r;
  out->adc_fl = g_trace_log_adc_fl;
  out->adc_l = g_trace_log_adc_l;
  out->adc_vbat = g_trace_log_adc_vbat;
  out->reserved_i32_0 = g_trace_log_reserved_i32_0;
  out->reserved_i32_1 = g_trace_log_reserved_i32_1;
  out->reserved_i32_2 = g_trace_log_reserved_i32_2;
  out->reserved_i32_3 = g_trace_log_reserved_i32_3;
  out->reserved_u16_0 = g_trace_log_reserved_u16_0;
  out->reserved_u16_1 = g_trace_log_reserved_u16_1;
  if (nightfall_wall_sensor_read_snapshot(&wall))
  {
    bool gate_on = (mode_flags & NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG) != 0U;

    out->adc_fr = (uint16_t)wall.fr_delta;
    out->adc_r = (uint16_t)wall.r_delta;
    out->adc_fl = (uint16_t)wall.fl_delta;
    out->adc_l = (uint16_t)wall.l_delta;
    out->adc_vbat = wall.vbat_on;
    out->reserved_i32_0 = wall.fr_delta;
    out->reserved_i32_1 = wall.r_delta;
    out->reserved_i32_2 = wall.fl_delta;
    out->reserved_i32_3 = wall.l_delta;
    out->reserved_u16_0 = nightfall_wall_trace_flags_from_snapshot(&wall, gate_on);
  }
  if ((mode_flags & NIGHTFALL_F413_TRACE_MODE_TUNE_FLAG) != 0U)
  {
    out->reserved_i32_0 = nightfall_trace_log_scale_float(f413_ctrl_tune_get_reference(), 1000.0f);
    out->reserved_i32_1 = (int32_t)f413_ctrl_tune_get_axis();
    out->reserved_i32_2 = nightfall_trace_log_scale_float(f413_ctrl_get_target_distance(), 1000.0f);
    out->reserved_i32_3 = nightfall_trace_log_scale_float(f413_ctrl_get_target_angle(), 1000.0f);
    out->reserved_u16_0 = (uint16_t)(((uint16_t)f413_ctrl_tune_get_axis() << 8U) |
                                     (uint16_t)f413_ctrl_tune_get_pattern());
    out->reserved_u16_1 = (uint16_t)f413_ctrl_tune_get_set();
  }
  out->flags = f413_hw_stop_switch_pressed()
                   ? NIGHTFALL_F413_TRACE_SWITCH_FLAG
                   : 0U;
  if (f413_ctrl_angle_target_enabled())
  {
    out->flags |= NIGHTFALL_F413_TRACE_ANGLE_TARGET_FLAG;
  }
  out->op_mode = g_trace_log_context_mode;
  out->op_case = g_trace_log_context_case;
  out->op_sub = g_trace_log_context_sub;
  out->test_id = g_trace_log_context_test_id;
}

static void nightfall_run_trace_log_append_sample_once(void)
{
  f413_trace_diag_run_append_sample_once();
}

static void nightfall_trace_log_set_mode_flags(uint16_t mode_flags)
{
  f413_trace_log_set_mode_flags(mode_flags);
}

static void nightfall_trace_log_auto_step(void)
{
  f413_trace_log_auto_step();
}

static void nightfall_trace_log_auto_tick_sample(void)
{
  f413_trace_log_auto_tick_sample(HAL_GetTick());
}

static void nightfall_trace_log_on_run_start(void)
{
  trace_printf("[TRACE-LOG] run-hook: start\r\n");
  f413_trace_log_auto_start();
}

static void nightfall_trace_log_on_run_stop(void)
{
  trace_printf("[TRACE-LOG] run-hook: stop\r\n");
  f413_trace_log_auto_stop();
}

static void nightfall_run_trace_log_dump_latest_once(void)
{
  f413_trace_diag_run_dump_latest_once();
}

static void nightfall_op_led_show_mode(uint8_t mode)
{
  f413_hw_show_mode_leds(mode);
}

static void nightfall_boot_buzzer_pattern(void)
{
  f413_hw_boot_buzzer_pattern();
}

static void nightfall_run_led_test_once(void)
{
  f413_hw_diag_run_led_test_once();
}

static bool nightfall_wall_sensor_read_snapshot(nightfall_wall_sensor_snapshot_t* out)
{
  return f413_wall_sensor_read_snapshot(out);
}

static bool nightfall_op_enter_sensor_active(void)
{
  nightfall_wall_sensor_snapshot_t wall;

  if (!nightfall_wall_sensor_read_snapshot(&wall))
  {
    return false;
  }

  return (wall.fr_delta >= NIGHTFALL_F413_OP_ENTER_DELTA_ADC) &&
         (wall.fl_delta <= NIGHTFALL_F413_OP_ENTER_RELEASE_ADC);
}

static void nightfall_run_wall_sensor_test_once(void)
{
  nightfall_wall_sensor_snapshot_t wall;
  uint16_t offset_r = 0U;
  uint16_t offset_l = 0U;
  uint16_t offset_fr = 0U;
  uint16_t offset_fl = 0U;
  uint8_t ready_mask = 0U;
  uint8_t phase = 0U;
  uint8_t inflight = 0U;

  if (!nightfall_wall_sensor_read_snapshot(&wall))
  {
    trace_printf("[HW-TEST][Wall] FAIL(read snapshot)\r\n");
    return;
  }

  trace_printf("[HW-TEST][Wall] off: R=%u L=%u FR=%u FL=%u VB=%u\r\n",
               (unsigned int)wall.r_off,
               (unsigned int)wall.l_off,
               (unsigned int)wall.fr_off,
               (unsigned int)wall.fl_off,
               (unsigned int)wall.vbat_off);
  trace_printf("[HW-TEST][Wall] on : R=%u L=%u FR=%u FL=%u VB=%u\r\n",
               (unsigned int)wall.r_on,
               (unsigned int)wall.l_on,
               (unsigned int)wall.fr_on,
               (unsigned int)wall.fl_on,
               (unsigned int)wall.vbat_on);
  trace_printf("[HW-TEST][Wall] delta: R=%ld L=%ld FR=%ld FL=%ld\r\n",
               (long)wall.r_delta,
               (long)wall.l_delta,
               (long)wall.fr_delta,
               (long)wall.fl_delta);
  f413_wall_sensor_get_debug_state(&offset_r,
                                   &offset_l,
                                   &offset_fr,
                                   &offset_fl,
                                   &ready_mask,
                                   &phase,
                                   &inflight);
  trace_printf("[HW-TEST][Wall] offset: R=%u L=%u FR=%u FL=%u ready=0x%02X phase=%u inflight=%u\r\n",
               (unsigned int)offset_r,
               (unsigned int)offset_l,
               (unsigned int)offset_fr,
               (unsigned int)offset_fl,
               (unsigned int)ready_mask,
               (unsigned int)phase,
               (unsigned int)inflight);
  trace_printf("[HW-TEST][Wall] detect: front=%u right=%u left=%u sat=%u thr FR=%u FL=%u R=%u L=%u\r\n",
               (unsigned int)wall.front_wall,
               (unsigned int)wall.right_wall,
               (unsigned int)wall.left_wall,
               (unsigned int)wall.saturated,
               (unsigned int)WALL_BASE_FR,
               (unsigned int)WALL_BASE_FL,
               (unsigned int)WALL_BASE_R,
               (unsigned int)WALL_BASE_L);
  trace_printf("[HW-TEST][Wall] PASS(measure done)\r\n");
}
static void nightfall_run_search_decision_preview_once(void)
{
  f413_search_step_run_decision_preview_once();
}

static void nightfall_search_step_session_reset(void)
{
  f413_search_step_session_reset();
}

static void nightfall_run_search_step_once(void)
{
  f413_search_step_run_once();
}

static void nightfall_run_search_map_probe_once(void)
{
  f413_search_step_run_map_probe_once();
}

static void nightfall_run_search_status_once(void)
{
  f413_search_step_run_status_once();
}

static void nightfall_run_search_map_clear_once(void)
{
  f413_search_step_run_map_clear_once();
}

static void nightfall_run_search_map_dump_once(void)
{
  f413_search_step_run_map_dump_once();
}

static void nightfall_wall_end_reset_from_snapshot(const nightfall_wall_sensor_snapshot_t* wall)
{
  memset(&g_wall_end, 0, sizeof(g_wall_end));
  if (wall == NULL)
  {
    return;
  }

  g_wall_end.right_wall = wall->r_delta > WALL_END_THR_R_HIGH;
  g_wall_end.left_wall = wall->l_delta > WALL_END_THR_L_HIGH;
  g_wall_end.prev_right_wall = g_wall_end.right_wall;
  g_wall_end.prev_left_wall = g_wall_end.left_wall;
  g_wall_end.prev_r_delta = wall->r_delta;
  g_wall_end.prev_l_delta = wall->l_delta;
  g_wall_end.initialized = true;
}

static void nightfall_wall_end_update(const nightfall_wall_sensor_snapshot_t* wall, bool gate_on)
{
  bool right_wall;
  bool left_wall;

  if (wall == NULL)
  {
    return;
  }
  if (!g_wall_end.initialized)
  {
    nightfall_wall_end_reset_from_snapshot(wall);
  }

  g_wall_end.deriv_r = wall->r_delta - g_wall_end.prev_r_delta;
  g_wall_end.deriv_l = wall->l_delta - g_wall_end.prev_l_delta;
  g_wall_end.prev_r_delta = wall->r_delta;
  g_wall_end.prev_l_delta = wall->l_delta;

  right_wall = g_wall_end.right_wall;
  left_wall = g_wall_end.left_wall;

  if (right_wall)
  {
    if (wall->r_delta < WALL_END_THR_R_LOW)
    {
      right_wall = false;
    }
  }
  else if (wall->r_delta > WALL_END_THR_R_HIGH)
  {
    right_wall = true;
  }

  if (left_wall)
  {
    if (wall->l_delta < WALL_END_THR_L_LOW)
    {
      left_wall = false;
    }
  }
  else if (wall->l_delta > WALL_END_THR_L_HIGH)
  {
    left_wall = true;
  }

  if (right_wall && (g_wall_end.deriv_r < -(int32_t)WALL_END_DERIV_FALL_THR))
  {
    if (g_wall_end.deriv_fall_count_r < 255U)
    {
      g_wall_end.deriv_fall_count_r++;
    }
  }
  else
  {
    g_wall_end.deriv_fall_count_r = 0U;
  }

  if (left_wall && (g_wall_end.deriv_l < -(int32_t)WALL_END_DERIV_FALL_THR))
  {
    if (g_wall_end.deriv_fall_count_l < 255U)
    {
      g_wall_end.deriv_fall_count_l++;
    }
  }
  else
  {
    g_wall_end.deriv_fall_count_l = 0U;
  }

  if (g_wall_end.deriv_fall_count_r >= 2U)
  {
    right_wall = false;
    g_wall_end.deriv_fall_count_r = 0U;
  }
  if (g_wall_end.deriv_fall_count_l >= 2U)
  {
    left_wall = false;
    g_wall_end.deriv_fall_count_l = 0U;
  }

  if (g_wall_end.prev_right_wall && !right_wall && gate_on && !g_wall_end.detected_r)
  {
    g_wall_end.detected_r = true;
    g_wall_end.dist_r_mm = f413_ctrl_get_distance();
  }
  if (g_wall_end.prev_left_wall && !left_wall && gate_on && !g_wall_end.detected_l)
  {
    g_wall_end.detected_l = true;
    g_wall_end.dist_l_mm = f413_ctrl_get_distance();
  }

  g_wall_end.right_wall = right_wall;
  g_wall_end.left_wall = left_wall;
  g_wall_end.prev_right_wall = right_wall;
  g_wall_end.prev_left_wall = left_wall;
}

static void nightfall_run_wall_end_monitor_once(void)
{
  nightfall_wall_sensor_snapshot_t wall;
  uint32_t start_ms;
  uint32_t now;

  if (!nightfall_wall_sensor_read_snapshot(&wall))
  {
    trace_printf("[HW-TEST][WallEnd] FAIL(read initial)\r\n");
    return;
  }

  nightfall_wall_end_reset_from_snapshot(&wall);
  trace_printf("[HW-TEST][WallEnd] start %lums sample=%lums R=%ld L=%ld wallR=%u wallL=%u\r\n",
               (unsigned long)NIGHTFALL_F413_WALL_END_MONITOR_MS,
               (unsigned long)NIGHTFALL_F413_WALL_END_MONITOR_SAMPLE_MS,
               (long)wall.r_delta,
               (long)wall.l_delta,
               (unsigned int)g_wall_end.right_wall,
               (unsigned int)g_wall_end.left_wall);

  start_ms = HAL_GetTick();
  do
  {
    HAL_Delay(NIGHTFALL_F413_WALL_END_MONITOR_SAMPLE_MS);
    now = HAL_GetTick();
    if (!nightfall_wall_sensor_read_snapshot(&wall))
    {
      trace_printf("[HW-TEST][WallEnd] FAIL(read sample)\r\n");
      return;
    }

    nightfall_wall_end_update(&wall, true);
    trace_printf("[HW-TEST][WallEnd] t=%lu R=%ld L=%ld dR=%ld dL=%ld wallR=%u wallL=%u endR=%u endL=%u\r\n",
                 (unsigned long)(now - start_ms),
                 (long)wall.r_delta,
                 (long)wall.l_delta,
                 (long)g_wall_end.deriv_r,
                 (long)g_wall_end.deriv_l,
                 (unsigned int)g_wall_end.right_wall,
                 (unsigned int)g_wall_end.left_wall,
                 (unsigned int)g_wall_end.detected_r,
                 (unsigned int)g_wall_end.detected_l);
  } while ((now - start_ms) < NIGHTFALL_F413_WALL_END_MONITOR_MS);

  trace_printf("[HW-TEST][WallEnd] done endR=%u distR=%.0f endL=%u distL=%.0f\r\n",
               (unsigned int)g_wall_end.detected_r,
               (double)g_wall_end.dist_r_mm,
               (unsigned int)g_wall_end.detected_l,
               (double)g_wall_end.dist_l_mm);
}

static void nightfall_run_switch_test_once(void)
{
  f413_hw_diag_run_switch_test_once();
}

static bool nightfall_imu_read_reg(uint8_t reg, uint8_t* out)
{
  uint8_t tx[2];
  uint8_t rx[2] = {0U, 0U};

  if (out == NULL)
  {
    return false;
  }

  tx[0] = (uint8_t)(reg | 0x80U);
  tx[1] = 0x00U;

  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, 20U) != HAL_OK)
  {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    return false;
  }
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  *out = rx[1];
  return true;
}

static bool nightfall_imu_write_reg(uint8_t reg, uint8_t val)
{
  uint8_t tx[2];

  tx[0] = (uint8_t)(reg & 0x7FU);
  tx[1] = val;

  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_Transmit(&hspi2, tx, 2U, 20U) != HAL_OK)
  {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    return false;
  }
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  return true;
}

static bool nightfall_imu_read_i16_le(uint8_t reg_l, int16_t* out)
{
  uint8_t tx[3];
  uint8_t rx[3] = {0U, 0U, 0U};

  if (out == NULL)
  {
    return false;
  }

  tx[0] = (uint8_t)(reg_l | 0x80U);
  tx[1] = 0x00U;
  tx[2] = 0x00U;

  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  if (HAL_SPI_TransmitReceive(&hspi2, tx, rx, 3U, 20U) != HAL_OK)
  {
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    return false;
  }
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  *out = (int16_t)((uint16_t)rx[2] << 8U | (uint16_t)rx[1]);
  return true;
}

static bool nightfall_imu_config_for_gyro(void)
{
  uint8_t who = 0U;

  if (!nightfall_imu_read_reg(NIGHTFALL_F413_IMU_WHO_AM_I_REG, &who) ||
      (who != NIGHTFALL_F413_IMU_WHO_AM_I_EXPECTED))
  {
    trace_printf("[HW-TEST][IMU-ANGLE] FAIL(who=0x%02X expected=0x%02X)\r\n",
                 (unsigned int)who,
                 (unsigned int)NIGHTFALL_F413_IMU_WHO_AM_I_EXPECTED);
    return false;
  }

  if (!nightfall_imu_write_reg(NIGHTFALL_F413_IMU_CTRL3_C, 0x44U))
  {
    trace_printf("[HW-TEST][IMU-ANGLE] FAIL(write CTRL3_C)\r\n");
    return false;
  }
  HAL_Delay(10U);

  if (!nightfall_imu_write_reg(NIGHTFALL_F413_IMU_CTRL2_G, 0x71U))
  {
    trace_printf("[HW-TEST][IMU-ANGLE] FAIL(write CTRL2_G)\r\n");
    return false;
  }
  HAL_Delay(10U);

  if (!nightfall_imu_write_reg(NIGHTFALL_F413_IMU_CTRL1_XL, 0x7CU))
  {
    trace_printf("[HW-TEST][IMU-ANGLE] FAIL(write CTRL1_XL)\r\n");
    return false;
  }
  HAL_Delay(10U);

  return true;
}

static bool nightfall_imu_read_gyro_z_dps(float* out)
{
  int16_t raw_z = 0;

  if (out == NULL)
  {
    return false;
  }

  if (!nightfall_imu_read_i16_le(NIGHTFALL_F413_IMU_OUTZ_G_L, &raw_z))
  {
    return false;
  }

  *out = (float)raw_z * NIGHTFALL_F413_IMU_GYRO_SENSITIVITY;
  return true;
}

static bool nightfall_imu_read_accel_xyz_mm_s2(float* ax, float* ay, float* az)
{
  int16_t raw_x = 0;
  int16_t raw_y = 0;
  int16_t raw_z = 0;

  if ((ax == NULL) || (ay == NULL) || (az == NULL))
  {
    return false;
  }

  if (!nightfall_imu_read_i16_le(NIGHTFALL_F413_IMU_OUTX_XL_L, &raw_x) ||
      !nightfall_imu_read_i16_le(NIGHTFALL_F413_IMU_OUTY_XL_L, &raw_y) ||
      !nightfall_imu_read_i16_le(NIGHTFALL_F413_IMU_OUTZ_XL_L, &raw_z))
  {
    return false;
  }

  *ax = (float)raw_x * NIGHTFALL_F413_IMU_ACCEL_SENS_MG * NIGHTFALL_F413_IMU_GRAVITY_MM_S2;
  *ay = (float)raw_y * NIGHTFALL_F413_IMU_ACCEL_SENS_MG * NIGHTFALL_F413_IMU_GRAVITY_MM_S2;
  *az = (float)raw_z * NIGHTFALL_F413_IMU_ACCEL_SENS_MG * NIGHTFALL_F413_IMU_GRAVITY_MM_S2;
  return true;
}

static void nightfall_run_imu_accel_test_once(void)
{
  float off_x = 0.0f;
  float off_y = 0.0f;
  float off_z = 0.0f;
  float vel_x = 0.0f;
  float vel_y = 0.0f;
  float vel_z = 0.0f;
  uint32_t i;
  uint32_t start_ms;
  uint32_t last_ms;
  uint32_t next_print_ms;

  if (f413_ctrl_is_running())
  {
    trace_printf("[HW-TEST][IMU-ACCEL] FAIL(control running)\r\n");
    return;
  }

  if (!nightfall_imu_config_for_gyro())
  {
    return;
  }

  trace_printf("[HW-TEST][IMU-ACCEL] keep still: offset sampling\r\n");
  HAL_Delay(200U);

  for (i = 0U; i < NIGHTFALL_F413_IMU_MANUAL_OFFSET_SAMPLES; i++)
  {
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    if (!nightfall_imu_read_accel_xyz_mm_s2(&ax, &ay, &az))
    {
      trace_printf("[HW-TEST][IMU-ACCEL] FAIL(read offset)\r\n");
      return;
    }
    off_x += ax;
    off_y += ay;
    off_z += az;
    HAL_Delay(1U);
  }
  off_x /= (float)NIGHTFALL_F413_IMU_MANUAL_OFFSET_SAMPLES;
  off_y /= (float)NIGHTFALL_F413_IMU_MANUAL_OFFSET_SAMPLES;
  off_z /= (float)NIGHTFALL_F413_IMU_MANUAL_OFFSET_SAMPLES;

  trace_printf("[HW-TEST][IMU-ACCEL] offset ax=%.1f ay=%.1f az=%.1f mm/s2\r\n",
               (double)off_x, (double)off_y, (double)off_z);
  trace_printf("[HW-TEST][IMU-ACCEL] start: move forward/backward; control forward axis is Y sign +\r\n");

  start_ms = HAL_GetTick();
  last_ms = start_ms;
  next_print_ms = start_ms;

  while ((uint32_t)(HAL_GetTick() - start_ms) < NIGHTFALL_F413_IMU_MANUAL_TEST_MS)
  {
    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    uint32_t now_ms;
    float dt;

    if (!nightfall_imu_read_accel_xyz_mm_s2(&ax, &ay, &az))
    {
      trace_printf("[HW-TEST][IMU-ACCEL] FAIL(read sample)\r\n");
      return;
    }

    now_ms = HAL_GetTick();
    dt = (float)(uint32_t)(now_ms - last_ms) * 0.001f;
    last_ms = now_ms;
    ax -= off_x;
    ay -= off_y;
    az -= off_z;
    vel_x += ax * dt;
    vel_y += ay * dt;
    vel_z += az * dt;

    if ((uint32_t)(now_ms - next_print_ms) >= NIGHTFALL_F413_IMU_MANUAL_PRINT_MS)
    {
      trace_printf("[HW-TEST][IMU-ACCEL] t=%lums ax=%.0f ay=%.0f az=%.0f vx=%.0f vy=%.0f vz=%.0f\r\n",
                   (unsigned long)(uint32_t)(now_ms - start_ms),
                   (double)ax, (double)ay, (double)az,
                   (double)vel_x, (double)vel_y, (double)vel_z);
      next_print_ms = now_ms;
    }

    HAL_Delay(NIGHTFALL_F413_IMU_MANUAL_SAMPLE_MS);
  }

  trace_printf("[HW-TEST][IMU-ACCEL] done vx=%.0f vy=%.0f vz=%.0f mm/s\r\n",
               (double)vel_x, (double)vel_y, (double)vel_z);
}

static void nightfall_run_imu_manual_turn_test_once(void)
{
  float offset = 0.0f;
  float angle = 0.0f;
  uint32_t i;
  uint32_t start_ms;
  uint32_t last_ms;
  uint32_t next_print_ms;

  if (f413_ctrl_is_running())
  {
    trace_printf("[HW-TEST][IMU-ANGLE] FAIL(control running)\r\n");
    return;
  }

  if (!nightfall_imu_config_for_gyro())
  {
    return;
  }

  trace_printf("[HW-TEST][IMU-ANGLE] keep still: offset sampling\r\n");
  HAL_Delay(200U);

  for (i = 0U; i < NIGHTFALL_F413_IMU_MANUAL_OFFSET_SAMPLES; i++)
  {
    float omega = 0.0f;
    if (!nightfall_imu_read_gyro_z_dps(&omega))
    {
      trace_printf("[HW-TEST][IMU-ANGLE] FAIL(read offset)\r\n");
      return;
    }
    offset += omega;
    HAL_Delay(1U);
  }
  offset /= (float)NIGHTFALL_F413_IMU_MANUAL_OFFSET_SAMPLES;

  trace_printf("[HW-TEST][IMU-ANGLE] offset=%.2fdps\r\n", (double)offset);
  trace_printf("[HW-TEST][IMU-ANGLE] start: CCW/left turn should be positive, CW/right negative\r\n");

  start_ms = HAL_GetTick();
  last_ms = start_ms;
  next_print_ms = start_ms;

  while ((uint32_t)(HAL_GetTick() - start_ms) < NIGHTFALL_F413_IMU_MANUAL_TEST_MS)
  {
    float omega = 0.0f;
    uint32_t now_ms;
    float dt;

    if (!nightfall_imu_read_gyro_z_dps(&omega))
    {
      trace_printf("[HW-TEST][IMU-ANGLE] FAIL(read sample)\r\n");
      return;
    }

    now_ms = HAL_GetTick();
    dt = (float)(uint32_t)(now_ms - last_ms) * 0.001f;
    last_ms = now_ms;
    omega -= offset;
    angle += omega * dt;

    if ((uint32_t)(now_ms - next_print_ms) >= NIGHTFALL_F413_IMU_MANUAL_PRINT_MS)
    {
      trace_printf("[HW-TEST][IMU-ANGLE] t=%lums omega_z=%.2fdps angle=%.1fdeg\r\n",
                   (unsigned long)(uint32_t)(now_ms - start_ms),
                   (double)omega,
                   (double)angle);
      next_print_ms = now_ms;
    }

    HAL_Delay(NIGHTFALL_F413_IMU_MANUAL_SAMPLE_MS);
  }

  trace_printf("[HW-TEST][IMU-ANGLE] done angle=%.1fdeg\r\n", (double)angle);
}

static void nightfall_run_imu_test_once(void)
{
  uint8_t who = 0U;
  if (!nightfall_imu_read_reg(NIGHTFALL_F413_IMU_WHO_AM_I_REG, &who))
  {
    trace_printf("[HW-TEST][IMU] FAIL(spi)\r\n");
    return;
  }

  trace_printf("[HW-TEST][IMU] WHO_AM_I=0x%02X expected=0x%02X => %s\r\n",
               (unsigned int)who,
               (unsigned int)NIGHTFALL_F413_IMU_WHO_AM_I_EXPECTED,
               (who == NIGHTFALL_F413_IMU_WHO_AM_I_EXPECTED) ? "PASS" : "FAIL");
}

static void nightfall_run_buzzer_test_once(void)
{
  f413_hw_diag_run_buzzer_test_once();
}

static const char* nightfall_op_mode_name(uint8_t mode)
{
  return f413_op_ui_mode_name(mode);
}

static const char* nightfall_op_level_name(uint8_t level)
{
  return f413_op_ui_level_name(level);
}

static const char* nightfall_op_case_name(uint8_t mode, uint8_t op_case)
{
  return f413_op_ui_case_name(mode, op_case);
}

static const char* nightfall_op_sub_name(uint8_t mode, uint8_t sub)
{
  return f413_op_ui_sub_name(mode, sub);
}

static const char* nightfall_tune_axis_name(uint8_t axis)
{
  switch (axis)
  {
    case F413_CTRL_TUNE_AXIS_VELOCITY: return "velocity";
    case F413_CTRL_TUNE_AXIS_OMEGA: return "omega";
    case F413_CTRL_TUNE_AXIS_DISTANCE: return "distance";
    case F413_CTRL_TUNE_AXIS_ANGLE: return "angle";
    default: return "unknown";
  }
}

static const char* nightfall_tune_pattern_name(uint8_t pattern)
{
  switch (pattern)
  {
    case F413_CTRL_TUNE_PATTERN_STEP: return "step";
    case F413_CTRL_TUNE_PATTERN_TRIANGLE: return "triangle";
    case F413_CTRL_TUNE_PATTERN_TRAPEZOID: return "trapezoid";
    default: return "unknown";
  }
}
static void nightfall_run_identity_status_once(void)
{
  if (g_boot_identity_status == NVM_STATUS_OK)
  {
    const char* family_name = nightfall_identity_family_name(g_boot_identity.family);
    trace_printf("[IDENTITY] status=OK family=%s(%lu) board=0x%08lX rev=%u.%u unit=%lu cap=0x%08lX\r\n",
                 family_name,
                 (unsigned long)g_boot_identity.family,
                 (unsigned long)g_boot_identity.board_id,
                 (unsigned int)g_boot_identity.hw_rev_major,
                 (unsigned int)g_boot_identity.hw_rev_minor,
                 (unsigned long)g_boot_identity.unit_serial,
                 (unsigned long)g_boot_identity.capability_flags);
    trace_printf("[IDENTITY] uid=%08lX-%08lX-%08lX\r\n",
                 (unsigned long)HAL_GetUIDw0(),
                 (unsigned long)HAL_GetUIDw1(),
                 (unsigned long)HAL_GetUIDw2());
  }
  else
  {
    trace_printf("[IDENTITY] status=%d uid=%08lX-%08lX-%08lX\r\n",
                 (int)g_boot_identity_status,
                 (unsigned long)HAL_GetUIDw0(),
                 (unsigned long)HAL_GetUIDw1(),
                 (unsigned long)HAL_GetUIDw2());
  }
}

static void nightfall_run_sensor_params_status_once(void)
{
  nvm_sensor_params_t params;
  bool loaded;

  memset(&params, 0, sizeof(params));
  loaded = nvm_params_sensor_load(&params);
  if (!loaded)
  {
    nvm_params_sensor_defaults(&params);
  }

  trace_printf("[SENSOR-PARAM] source=%s base_l=%u base_r=%u base_f=%u off_r=%u off_l=%u off_fr=%u off_fl=%u imu_z=%.3f\r\n",
               loaded ? "NVM" : "default",
               (unsigned int)params.base_l,
               (unsigned int)params.base_r,
               (unsigned int)params.base_f,
               (unsigned int)params.wall_offset_r,
               (unsigned int)params.wall_offset_l,
               (unsigned int)params.wall_offset_fr,
               (unsigned int)params.wall_offset_fl,
               (double)params.imu_offset_z);
  trace_printf("[SENSOR-PARAM] save is intentionally not performed by OP mode9 case9\r\n");
}


static void nightfall_run_nvm_status_once(void)
{
  static uint16_t cells[NIGHTFALL_F413_SEARCH_MAP_CELL_COUNT];
  nvm_trace_log_header_t header;
  uint32_t known_count = 0U;
  uint32_t i;
  bool distance_ok;
  bool sensor_ok;
  bool maze_ok;
  nvm_status_t trace_st;
  nvm_sensor_params_t sensor_params;

  distance_ok = nvm_params_distance_load_and_apply();
  sensor_ok = nvm_params_sensor_load(&sensor_params);
  maze_ok = nvm_maze_load_map(cells, NIGHTFALL_F413_SEARCH_MAP_CELL_COUNT);
  trace_st = nvm_trace_log_get_header(&header);

  if (maze_ok)
  {
    for (i = 0U; i < NIGHTFALL_F413_SEARCH_MAP_CELL_COUNT; i++)
    {
      if ((cells[i] & NIGHTFALL_F413_MAZE_WALL_KNOWN_MASK) != 0U)
      {
        known_count++;
      }
    }
  }

  trace_printf("[NVM-STATUS] distance=%s sensor=%s maze=%s maze_known=%lu trace=%s(%d)\r\n",
               distance_ok ? "OK" : "MISS",
               sensor_ok ? "OK" : "MISS",
               maze_ok ? "OK" : "MISS",
               (unsigned long)known_count,
               (trace_st == NVM_STATUS_OK) ? "OK" : "MISS",
               (int)trace_st);
  if (trace_st == NVM_STATUS_OK)
  {
    trace_printf("[NVM-STATUS] trace ver=0x%08lX rec_size=%lu cap=%lu write=%lu total=%lu\r\n",
                 (unsigned long)header.version,
                 (unsigned long)header.record_size,
                 (unsigned long)header.record_capacity,
                 (unsigned long)header.write_index,
                 (unsigned long)header.total_records);
  }
}

static uint8_t nightfall_op_selected_value(void)
{
  return f413_op_ui_selected_value();
}

static bool nightfall_op_can_accept_input(void)
{
  if (f413_trace_log_auto_is_enabled() || f413_ctrl_is_running())
  {
    return false;
  }
  if (f413_test_run_is_armed())
  {
    return false;
  }
  return true;
}

static void nightfall_op_run_test_after_delay(uint8_t test_id)
{
  f413_test_run_clear_arm();
  nightfall_trace_log_set_context(8U, f413_op_ui_get_case(), 0xFFU, test_id);
  HAL_Delay(NIGHTFALL_F413_OP_START_DELAY_MS);
  f413_test_run_run_now(test_id);
}

static void nightfall_op_run_tune_sub_after_delay(uint8_t sub)
{
  uint8_t axis = F413_CTRL_TUNE_AXIS_VELOCITY;
  uint8_t set = 0U;
  uint8_t pattern = F413_CTRL_TUNE_PATTERN_STEP;

  switch (sub)
  {
    case 0U:
      axis = F413_CTRL_TUNE_AXIS_VELOCITY;
      pattern = F413_CTRL_TUNE_PATTERN_STEP;
      break;
    case 1U:
      axis = F413_CTRL_TUNE_AXIS_VELOCITY;
      pattern = F413_CTRL_TUNE_PATTERN_TRAPEZOID;
      break;
    case 2U:
      axis = F413_CTRL_TUNE_AXIS_OMEGA;
      pattern = F413_CTRL_TUNE_PATTERN_STEP;
      break;
    case 3U:
      axis = F413_CTRL_TUNE_AXIS_OMEGA;
      pattern = F413_CTRL_TUNE_PATTERN_TRAPEZOID;
      break;
    case 4U:
      axis = F413_CTRL_TUNE_AXIS_DISTANCE;
      pattern = F413_CTRL_TUNE_PATTERN_STEP;
      break;
    case 5U:
      axis = F413_CTRL_TUNE_AXIS_DISTANCE;
      pattern = F413_CTRL_TUNE_PATTERN_TRAPEZOID;
      break;
    case 6U:
      axis = F413_CTRL_TUNE_AXIS_ANGLE;
      pattern = F413_CTRL_TUNE_PATTERN_STEP;
      break;
    case 7U:
      axis = F413_CTRL_TUNE_AXIS_ANGLE;
      pattern = F413_CTRL_TUNE_PATTERN_TRAPEZOID;
      break;
    case 8U:
      axis = F413_CTRL_TUNE_AXIS_VELOCITY;
      set = 1U;
      pattern = F413_CTRL_TUNE_PATTERN_TRAPEZOID;
      break;
    case 9U:
      axis = F413_CTRL_TUNE_AXIS_OMEGA;
      set = 1U;
      pattern = F413_CTRL_TUNE_PATTERN_TRAPEZOID;
      break;
    default:
      break;
  }

  nightfall_trace_log_set_context(9U, 0U, sub, (uint8_t)'0');
  HAL_Delay(NIGHTFALL_F413_OP_START_DELAY_MS);
  nightfall_run_control_tune_once(axis, set, pattern);
}
static void nightfall_op_run_case0_sub_after_delay(uint8_t mode, uint8_t sub)
{
#if (NIGHTFALL_F413_REAL_RUN_PATH_ENABLED != 0U)
  uint16_t codes[4] = {0U, 0U, 0U, 0U};
  uint16_t code_count = 0U;
  uint8_t case_index = 1U;
  float straight_velocity = nightfall_f413_op_test_velocity_for_mode(mode, (sub == 9U));
  float diagonal_velocity = straight_velocity * NIGHTFALL_F413_OP_TEST_DIAG_SCALE;
  const char* label = nightfall_op_sub_name(mode, sub);
  uint16_t lead = (mode >= 6U) ? 204U : 203U;

  if (sub <= 2U)
  {
    case_index = (mode <= 5U) ? 3U : 1U;
  }
  else if ((sub >= 3U) && (sub <= 7U))
  {
    case_index = 8U;
  }
  else if (sub == 8U)
  {
    case_index = 1U;
  }
  else
  {
    case_index = 5U;
  }

  switch (sub)
  {
    case 0U:
      codes[0] = lead;
      codes[1] = 300U;
      code_count = 2U;
      break;
    case 1U:
      codes[0] = lead;
      codes[1] = 501U;
      code_count = 2U;
      break;
    case 2U:
      codes[0] = lead;
      codes[1] = 502U;
      code_count = 2U;
      break;
    case 3U:
      codes[0] = lead;
      codes[1] = 701U;
      codes[2] = 1001U;
      code_count = 3U;
      break;
    case 4U:
      codes[0] = lead;
      codes[1] = 1001U;
      codes[2] = 704U;
      codes[3] = 1001U;
      code_count = 4U;
      break;
    case 5U:
      codes[0] = lead;
      codes[1] = 1001U;
      codes[2] = 802U;
      codes[3] = 1001U;
      code_count = 4U;
      break;
    case 6U:
      codes[0] = lead;
      codes[1] = 901U;
      codes[2] = 1001U;
      code_count = 3U;
      break;
    case 7U:
      codes[0] = lead;
      codes[1] = 1001U;
      codes[2] = 904U;
      codes[3] = 1001U;
      code_count = 4U;
      break;
    case 8U:
    case 9U:
      codes[0] = 206U;
      code_count = 1U;
      break;
    default:
      break;
  }

  nightfall_trace_log_set_context(mode, 0U, sub, (uint8_t)'P');
  HAL_Delay(NIGHTFALL_F413_OP_START_DELAY_MS);
  nightfall_run_path_code_sequence_once(label, mode, case_index, codes, code_count,
                                        straight_velocity, diagonal_velocity);
#else
  trace_printf("[OP-UI] no-op: F413 path-code test runner is disabled in this build\r\n");
#endif
}

static void nightfall_op_execute_action(f413_op_ui_action_t action, uint8_t mode, uint8_t op_case, uint8_t sub)
{
  switch (action)
  {
    case F413_OP_UI_ACTION_SEARCH_TRACE_ENTRY:
      nightfall_trace_log_set_context(mode, op_case, 0xFFU, 0U);
      HAL_Delay(NIGHTFALL_F413_OP_START_DELAY_MS);
      nightfall_run_search_trace_entry_once();
      break;
    case F413_OP_UI_ACTION_SHORTEST_TRACE_ENTRY:
      nightfall_trace_log_set_context(mode, op_case, 0xFFU, 0U);
      HAL_Delay(NIGHTFALL_F413_OP_START_DELAY_MS);
      nightfall_run_shortest_trace_entry_once();
      break;
    case F413_OP_UI_ACTION_TEST_RUN_1:
      nightfall_op_run_test_after_delay('1');
      break;
    case F413_OP_UI_ACTION_TEST_RUN_2:
      nightfall_op_run_test_after_delay('2');
      break;
    case F413_OP_UI_ACTION_TEST_RUN_3:
      nightfall_op_run_test_after_delay('3');
      break;
    case F413_OP_UI_ACTION_TEST_RUN_5:
      nightfall_op_run_test_after_delay('5');
      break;
    case F413_OP_UI_ACTION_IMU_TEST:
      nightfall_run_imu_test_once();
      break;
    case F413_OP_UI_ACTION_ENCODER_TEST:
      nightfall_run_encoder_test_once();
      break;
    case F413_OP_UI_ACTION_WALL_SENSOR_TEST:
      nightfall_run_wall_sensor_test_once();
      break;
    case F413_OP_UI_ACTION_FAN_PWM_TEST:
      nightfall_run_fan_pwm_test_once();
      break;
    case F413_OP_UI_ACTION_TRACE_DUMP_BIN_ALL:
      nightfall_run_trace_log_dump_bin_all_once();
      break;
    case F413_OP_UI_ACTION_NVM_STATUS:
      nightfall_run_nvm_status_once();
      break;
    case F413_OP_UI_ACTION_IDENTITY_STATUS:
      nightfall_run_identity_status_once();
      break;
    case F413_OP_UI_ACTION_SENSOR_PARAMS_STATUS:
      nightfall_run_sensor_params_status_once();
      break;
    case F413_OP_UI_ACTION_CONTROL_TUNE_SUB:
      nightfall_op_run_tune_sub_after_delay(sub);
      break;
    case F413_OP_UI_ACTION_PATH_CASE0_SUB:
      nightfall_op_run_case0_sub_after_delay(mode, sub);
      break;
    case F413_OP_UI_ACTION_NONE:
    default:
      break;
  }
}

static void nightfall_op_ui_step(void)
{
  f413_op_ui_step(HAL_GetTick());
}

static void nightfall_op_uart_push_once(void)
{
  f413_op_ui_uart_push_once();
}

static void nightfall_op_uart_enter_once(void)
{
  f413_op_ui_uart_enter_once();
}

static void nightfall_motor_set(bool enable,
                                bool left_forward,
                                bool right_forward,
                                uint16_t left_duty,
                                uint16_t right_duty)
{
  f413_hw_motor_set(enable, left_forward, right_forward, left_duty, right_duty);
}

static void nightfall_run_fan_pwm_test_once(void)
{
  f413_hw_diag_run_fan_pwm_test_once();
}

static void nightfall_run_motor_driver_test_once(void)
{
  f413_hw_diag_run_motor_driver_test_once();
}

static void nightfall_run_encoder_test_once(void)
{
  f413_hw_diag_run_encoder_test_once();
}

static void nightfall_run_hardware_smoke_tests(void)
{
  trace_printf("[HW-TEST] smoke start\r\n");
  nightfall_run_switch_test_once();
  nightfall_trace_log_auto_step();
  nightfall_run_wall_sensor_test_once();
  nightfall_trace_log_auto_step();
  nightfall_run_imu_test_once();
  nightfall_trace_log_auto_step();
  nightfall_run_buzzer_test_once();
  nightfall_trace_log_auto_step();
  trace_printf("[HW-TEST] smoke end (motor/encoder are manual commands: o/e)\r\n");
}

static void nightfall_run_hardware_smoke_tests_with_trace_session(void)
{
  nightfall_trace_log_on_run_start();
  nightfall_trace_log_set_mode_flags(NIGHTFALL_F413_TRACE_MODE_SMOKE_FLAG);
  nightfall_run_hardware_smoke_tests();
  nightfall_trace_log_auto_step();
  nightfall_trace_log_set_mode_flags(0U);
  nightfall_trace_log_on_run_stop();
}

static bool nightfall_run_stop_switch_pressed(void)
{
  return f413_hw_stop_switch_pressed();
}

static int16_t nightfall_run_session_encoder_l_count(void)
{
  return (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

static int16_t nightfall_run_session_encoder_r_count(void)
{
  return (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

static void nightfall_run_session_encoder_stop_all(void)
{
  (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
}

static void nightfall_run_session_encoder_reset_all(void)
{
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
}

static void nightfall_test_run_encoder_center_all(void)
{
  __HAL_TIM_SET_COUNTER(&htim3, 30000U);
  __HAL_TIM_SET_COUNTER(&htim4, 30000U);
}

static bool nightfall_run_session_encoder_start_l(void)
{
  return HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) == HAL_OK;
}

static bool nightfall_run_session_encoder_start_r(void)
{
  return HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) == HAL_OK;
}

static void nightfall_run_session_encoder_stop_l(void)
{
  (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
}

static void nightfall_run_session_encoder_stop_r(void)
{
  (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
}

static void nightfall_wall_control_apply_straight(void)
{
  nightfall_wall_control_apply(true);
}

static void nightfall_test_run_record_result(uint8_t test_id,
                                             f413_run_session_abort_reason_t abort_reason,
                                             float distance_mm,
                                             float angle_deg)
{
  g_last_test_id = test_id;
  g_last_test_abort_reason = abort_reason;
  g_last_test_distance_mm = distance_mm;
  g_last_test_angle_deg = angle_deg;
}

static bool nightfall_run_session_wall_sensor_ok(void)
{
  nightfall_wall_sensor_snapshot_t wall;

  if (!nightfall_wall_sensor_read_snapshot(&wall))
  {
    return false;
  }
  return !wall.saturated;
}

static bool nightfall_run_session_imu_ok(void)
{
  uint8_t who = 0U;

  return nightfall_imu_read_reg(NIGHTFALL_F413_IMU_WHO_AM_I_REG, &who) &&
         (who == NIGHTFALL_F413_IMU_WHO_AM_I_EXPECTED);
}

static uint16_t nightfall_run_abort_reason_to_trace_flag(nightfall_run_abort_reason_t reason)
{
  return f413_run_session_abort_reason_to_trace_flag(reason);
}

static const char* nightfall_run_abort_reason_to_text(nightfall_run_abort_reason_t reason)
{
  return f413_run_session_abort_reason_to_text(reason);
}

static bool nightfall_run_guard_prepare(nightfall_run_guard_t* guard)
{
  return f413_run_session_guard_prepare(guard);
}

static void nightfall_run_guard_cleanup(nightfall_run_guard_t* guard)
{
  f413_run_session_guard_cleanup(guard);
}

static nightfall_run_abort_reason_t nightfall_trace_log_wait_with_auto_step_guarded(uint32_t duration_ms,
                                                                                     nightfall_run_guard_t* guard)
{
  return f413_run_session_wait_with_auto_step_guarded(duration_ms, guard);
}

static void nightfall_run_idle_trace_session_once(void)
{
  f413_run_session_run_idle_trace_once(NIGHTFALL_F413_RUN_SESSION_IDLE_MS,
                                       NIGHTFALL_F413_TRACE_MODE_IDLE_FLAG);
}

static void nightfall_run_motor_trace_session_once(void)
{
  f413_run_session_run_motor_trace_once(NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG,
                                        NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG,
                                        NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG,
                                        NIGHTFALL_F413_RUN_SESSION_MOTOR_DUTY,
                                        NIGHTFALL_F413_RUN_SESSION_MOTOR_PULSE_MS,
                                        NIGHTFALL_F413_RUN_SESSION_MOTOR_COAST_MS);
}

static void nightfall_run_search_safe_trace_session_once(void)
{
  f413_run_session_run_search_safe_trace_once(NIGHTFALL_F413_TRACE_MODE_SEARCH_SAFE_FLAG,
                                              NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG,
                                              NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG,
                                              NIGHTFALL_F413_RUN_SESSION_SAFE_DUTY,
                                              NIGHTFALL_F413_RUN_SESSION_SAFE_FORWARD_MS,
                                              NIGHTFALL_F413_RUN_SESSION_SAFE_COAST_MS,
                                              NIGHTFALL_F413_RUN_SESSION_SAFE_EXPLORE_STEPS);
}

static void nightfall_run_shortest_safe_trace_session_once(void)
{
  f413_run_session_run_shortest_safe_trace_once(NIGHTFALL_F413_TRACE_MODE_SHORTEST_SAFE_FLAG,
                                                NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG,
                                                NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG,
                                                NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG,
                                                NIGHTFALL_F413_RUN_SESSION_SAFE_DUTY,
                                                NIGHTFALL_F413_RUN_SESSION_SAFE_FORWARD_MS,
                                                NIGHTFALL_F413_RUN_SESSION_SAFE_TURN_MS,
                                                NIGHTFALL_F413_RUN_SESSION_SAFE_COAST_MS);
}

static bool nightfall_run_distance_nvm_test(void)
{
  return f413_diag_run_distance_nvm_test();
}

static bool nightfall_run_sensor_nvm_test(void)
{
  return f413_diag_run_sensor_nvm_test();
}

static bool nightfall_run_maze_nvm_test(void)
{
  return f413_diag_run_maze_nvm_test();
}

static bool nightfall_verify_distance_nvm_load_only(void)
{
  return f413_diag_verify_distance_nvm_load_only();
}

static bool nightfall_verify_sensor_nvm_load_only(void)
{
  return f413_diag_verify_sensor_nvm_load_only();
}

static bool nightfall_verify_maze_nvm_load_only(void)
{
  return f413_diag_verify_maze_nvm_load_only();
}

static bool nightfall_run_trace_log_nvm_test(void)
{
  return f413_diag_run_trace_log_nvm_test();
}

static bool nightfall_verify_trace_log_nvm_load_only(void)
{
  return f413_diag_verify_trace_log_nvm_load_only();
}

static void nightfall_run_all_nvm_tests(void)
{
  f413_diag_run_all_nvm_tests();
}

static void nightfall_verify_all_nvm_load_only(void)
{
  f413_diag_verify_all_nvm_load_only();
}

static void nightfall_handle_uart_command(uint8_t cmd)
{
  switch (cmd)
  {
    case 'h':
    case 'H':
    case '?':
      nightfall_print_nvm_cli_help();
      break;

    case 'a':
      trace_printf("[NVM-TEST] run all\r\n");
      nightfall_run_all_nvm_tests();
      break;

    case 'A':
      trace_printf("[NVM-TEST] verify all (load_only)\r\n");
      nightfall_verify_all_nvm_load_only();
      break;

    case 'd':
      trace_printf("[NVM-TEST] run distance\r\n");
      (void)nightfall_run_distance_nvm_test();
      break;

    case 'D':
      trace_printf("[NVM-TEST] verify distance (load_only)\r\n");
      (void)nightfall_verify_distance_nvm_load_only();
      break;

    case 's':
      trace_printf("[NVM-TEST] run sensor\r\n");
      (void)nightfall_run_sensor_nvm_test();
      break;

    case 'S':
      trace_printf("[NVM-TEST] verify sensor (load_only)\r\n");
      (void)nightfall_verify_sensor_nvm_load_only();
      break;

    case 'm':
      trace_printf("[NVM-TEST] run maze\r\n");
      (void)nightfall_run_maze_nvm_test();
      break;

    case 't':
      trace_printf("[NVM-TEST] run trace\r\n");
      (void)nightfall_run_trace_log_nvm_test();
      break;

    case 'M':
      trace_printf("[NVM-TEST] verify maze (load_only)\r\n");
      (void)nightfall_verify_maze_nvm_load_only();
      break;

    case 'T':
      trace_printf("[NVM-TEST] verify trace (load_only)\r\n");
      (void)nightfall_verify_trace_log_nvm_load_only();
      break;

    case 'w':
      nightfall_run_wall_sensor_test_once();
      break;

    case 'W':
      nightfall_run_wall_end_monitor_once();
      break;

    case 'p':
      nightfall_run_switch_test_once();
      break;

    case 'P':
      nightfall_op_uart_push_once();
      break;

    case 'i':
      nightfall_run_imu_test_once();
      break;

    case 'I':
      nightfall_run_imu_manual_turn_test_once();
      break;

    case 'c':
    case 'C':
      nightfall_run_imu_accel_test_once();
      break;

    case 'b':
      nightfall_run_buzzer_test_once();
      break;

    case 'o':
    case '0':
      nightfall_run_motor_driver_test_once();
      break;

    case 'e':
      nightfall_run_encoder_test_once();
      break;

    case 'E':
      nightfall_op_uart_enter_once();
      break;


    case '!':
      nightfall_op_run_tune_sub_after_delay(0U);
      break;

    case '"':
      nightfall_op_run_tune_sub_after_delay(1U);
      break;

    case '#':
      nightfall_op_run_tune_sub_after_delay(2U);
      break;

    case '$':
      nightfall_op_run_tune_sub_after_delay(3U);
      break;

    case '%':
      nightfall_op_run_tune_sub_after_delay(4U);
      break;

    case '^':
      nightfall_op_run_tune_sub_after_delay(5U);
      break;

    case '&':
      nightfall_op_run_tune_sub_after_delay(6U);
      break;

    case '*':
      nightfall_op_run_tune_sub_after_delay(7U);
      break;

    case '(':
      nightfall_op_run_tune_sub_after_delay(8U);
      break;

    case ')':
      nightfall_op_run_tune_sub_after_delay(9U);
      break;


    case 'G':
      nightfall_run_search_decision_preview_once();
      break;

    case 'B':
      nightfall_search_step_session_reset();
      trace_printf("[SEARCH-STEP] session reset\r\n");
      break;

    case 'N':
      nightfall_run_search_step_once();
      break;

    case '[':
      nightfall_run_search_status_once();
      break;

    case ']':
      nightfall_run_search_map_clear_once();
      break;

    case '@':
      nightfall_run_search_map_dump_once();
      break;

    case 'O':
      nightfall_run_search_map_probe_once();
      break;

    case 'q':
    case 'Q':
      nightfall_run_trace_log_format_once();
      break;

    case 'r':
      nightfall_run_trace_log_append_sample_once();
      break;

    case 'R':
      nightfall_run_trace_log_dump_latest_once();
      break;

    case 'v':
      nightfall_run_trace_log_dump_csv_once();
      break;

    case 'V':
      nightfall_run_trace_log_dump_csv_all_once();
      break;

    case '<':
      nightfall_run_trace_log_dump_bin_once();
      break;

    case '>':
      nightfall_run_trace_log_dump_bin_all_once();
      break;

    case 'k':
    case 'K':
      nightfall_run_trace_log_selftest_once();
      break;

    case 'u':
      nightfall_trace_log_on_run_start();
      break;

    case 'U':
      nightfall_trace_log_on_run_stop();
      break;

    case 'l':
    case 'L':
      nightfall_run_led_test_once();
      break;

    case 'g':
      nightfall_run_hardware_smoke_tests_with_trace_session();
      break;

    case 'x':
    case 'X':
      nightfall_trace_log_set_context(0U, 0xFFU, 0xFFU, (uint8_t)'x');
      nightfall_run_idle_trace_session_once();
      break;

    case 'y':
    case 'Y':
      nightfall_trace_log_set_context(0U, 0xFFU, 0xFFU, (uint8_t)'y');
      nightfall_run_motor_trace_session_once();
      break;

    case 'z':
    case 'Z':
      nightfall_trace_log_set_context(1U, 4U, 0xFFU, (uint8_t)'z');
      nightfall_run_search_trace_entry_once();
      break;

    case 'j':
    case 'J':
      nightfall_trace_log_set_context(2U, 1U, 0xFFU, (uint8_t)'j');
      nightfall_run_shortest_trace_entry_once();
      break;

    case '6':
    case '7':
    case '8':
    case '9':
      nightfall_trace_log_set_context(8U, 0xFFU, 0xFFU, cmd);
      f413_test_run_run_now(cmd);
      break;

    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
      f413_test_run_arm_for_button(cmd);
      break;

    case 'f':
    case 'F':
      /* 直前に '1'-'5' を送っていなければデフォルト '1' をアーム */
      {
        uint8_t arm_id = f413_test_run_is_armed() ? f413_test_run_armed_id() : (uint8_t)'1';
        f413_test_run_arm_for_button(arm_id);
      }
      break;

    case '\r':
    case '\n':
      break;

    default:
      trace_printf("[NVM-TEST] unknown command '%c' (0x%02X)\r\n", (char)cmd, (unsigned int)cmd);
      nightfall_print_nvm_cli_help();
      break;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  if (nvm_init() == NVM_STATUS_OK)
  {
    g_boot_identity_status = nvm_identity_read(&g_boot_identity);
  }
  else
  {
    g_boot_identity_status = NVM_STATUS_HW_ERROR;
  }

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  trace_init();
  f413_trace_log_config(nightfall_fill_trace_log_control_sample,
                        nightfall_trace_log_update_observe_cache,
                        nightfall_wall_end_clear);
  {
    const f413_trace_diag_config_t trace_diag_config = {
      nightfall_fill_trace_log_sample,
      nightfall_trace_diag_get_context,
      nightfall_op_mode_name,
      nightfall_op_case_name,
      nightfall_op_sub_name,
      nightfall_trace_diag_emit_extra_csv_meta
    };
    f413_trace_diag_config(&trace_diag_config);
  }
  {
    const f413_op_ui_config_t op_ui_config = {
      nightfall_op_can_accept_input,
      nightfall_run_stop_switch_pressed,
      nightfall_op_enter_sensor_active,
      nightfall_op_execute_action
    };
    f413_op_ui_config(&op_ui_config);
  }
  {
    const f413_run_session_config_t run_session_config = {
      nightfall_run_stop_switch_pressed,
      nightfall_run_session_encoder_l_count,
      nightfall_run_session_encoder_r_count,
      nightfall_run_session_wall_sensor_ok,
      nightfall_run_session_imu_ok,
      nightfall_trace_log_auto_step,
      f413_trace_log_auto_is_enabled,
      nightfall_trace_log_on_run_start,
      nightfall_trace_log_on_run_stop,
      nightfall_trace_log_set_mode_flags,
      nightfall_run_session_encoder_stop_all,
      nightfall_run_session_encoder_reset_all,
      nightfall_run_session_encoder_start_l,
      nightfall_run_session_encoder_start_r,
      nightfall_run_session_encoder_stop_l,
      nightfall_run_session_encoder_stop_r,
      nightfall_motor_set
    };
    f413_run_session_config(&run_session_config);
  }
  {
    const f413_search_step_config_t search_step_config = {
      nightfall_run_stop_switch_pressed,
      HAL_GetTick,
      nightfall_wall_sensor_read_snapshot,
      f413_trace_log_auto_is_enabled,
      nightfall_trace_log_set_context,
      nightfall_trace_log_on_run_start,
      nightfall_trace_log_on_run_stop,
      nightfall_trace_log_set_mode_flags,
      nightfall_trace_log_auto_step,
      nightfall_wall_control_apply_straight,
      NIGHTFALL_F413_PATH_TIMEOUT_MS,
      NIGHTFALL_F413_PATH_COAST_MS,
      NIGHTFALL_F413_SEARCH_STEP_VELOCITY_MM_S,
      NIGHTFALL_F413_SEARCH_STEP_TARGET_MM,
      NIGHTFALL_F413_SEARCH_STEP_TURN_DEG,
      NIGHTFALL_F413_TRACE_MODE_SEARCH_SAFE_FLAG,
      NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG,
      NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG,
      NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG
    };
    f413_search_step_config(&search_step_config);
  }
  {
    const f413_test_run_config_t test_run_config = {
      nightfall_run_stop_switch_pressed,
      HAL_Delay,
      HAL_GetTick,
      nightfall_trace_log_set_context,
      nightfall_trace_log_on_run_start,
      nightfall_trace_log_on_run_stop,
      nightfall_trace_log_set_mode_flags,
      nightfall_trace_log_auto_step,
      nightfall_wall_control_apply_straight,
      nightfall_test_run_record_result,
      nightfall_run_session_encoder_stop_all,
      nightfall_run_session_encoder_reset_all,
      nightfall_test_run_encoder_center_all,
      nightfall_run_session_encoder_start_l,
      nightfall_run_session_encoder_start_r,
      nightfall_run_session_encoder_l_count,
      nightfall_run_session_encoder_r_count,
      nightfall_motor_set,
      NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG,
      NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG,
      NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG,
      NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG
    };
    f413_test_run_config(&test_run_config);
  }
  trace_printf("\r\n[NIGHTFALL] STM32F413 bring-up\r\n");
  trace_printf("FW=%s TARGET=%s BUILD=%s\r\n", FW_VERSION, FW_TARGET, FW_BUILD_TYPE);
  trace_printf("GIT=%s DIRTY=%d\r\n", FW_GIT_SHA, FW_GIT_DIRTY);
  nightfall_boot_buzzer_pattern();

  if (g_boot_identity_status == NVM_STATUS_OK)
  {
    const char* family_name = nightfall_identity_family_name(g_boot_identity.family);
    trace_printf("ID family=%s(%lu) board=0x%08lX rev=%u.%u unit=%lu cap=0x%08lX\r\n",
                 family_name,
                 (unsigned long)g_boot_identity.family,
                 (unsigned long)g_boot_identity.board_id,
                 (unsigned int)g_boot_identity.hw_rev_major,
                 (unsigned int)g_boot_identity.hw_rev_minor,
                 (unsigned long)g_boot_identity.unit_serial,
                 (unsigned long)g_boot_identity.capability_flags);
    trace_printf("ID machine=%s_r%u_%u unit_name=%s_r%u_%u_unit%03lu\r\n",
                 family_name,
                 (unsigned int)g_boot_identity.hw_rev_major,
                 (unsigned int)g_boot_identity.hw_rev_minor,
                 family_name,
                 (unsigned int)g_boot_identity.hw_rev_major,
                 (unsigned int)g_boot_identity.hw_rev_minor,
                 (unsigned long)g_boot_identity.unit_serial);
  }
  else if ((g_boot_identity_status == NVM_STATUS_NOT_FOUND) ||
           (g_boot_identity_status == NVM_STATUS_UNSUPPORTED))
  {
    trace_printf("ID status=%d (continue boot)\r\n", (int)g_boot_identity_status);
  }
  else
  {
    trace_printf("[SAFE] identity invalid status=%d uid=%08lX-%08lX-%08lX\r\n",
                 (int)g_boot_identity_status,
                 (unsigned long)HAL_GetUIDw0(),
                 (unsigned long)HAL_GetUIDw1(),
                 (unsigned long)HAL_GetUIDw2());
    nightfall_identity_enter_safe_mode();
  }

  trace_printf("[NVM-TEST] UART command mode ready\r\n");
  nightfall_print_nvm_cli_help();

  if (nightfall_wall_sensor_start_async())
  {
    trace_printf("[WALL] async ADC DMA sensor scheduler started\r\n");
  }
  else
  {
    trace_printf("[WALL] FAIL(start async ADC DMA sensor scheduler)\r\n");
  }

  f413_ctrl_init();
  trace_printf("[CTRL] 1kHz velocity control initialized\r\n");
  nightfall_op_led_show_mode(f413_op_ui_get_mode());
  trace_printf("[OP-UI] ready %s=%u %s\r\n",
               nightfall_op_level_name(f413_op_ui_get_level()),
               (unsigned int)nightfall_op_selected_value(),
               nightfall_op_mode_name(f413_op_ui_get_mode()));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t cmd = 0U;
    nightfall_trace_log_auto_step();
    nightfall_op_ui_step();

    f413_test_run_step_button_armed();

    if (HAL_UART_Receive(&huart1, &cmd, 1U, 1U) == HAL_OK)
    {
      nightfall_handle_uart_command(cmd);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 60000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 99;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 99;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 20;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 99;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = NIGHTFALL_F413_UART_BAUD_RATE;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IR_FL_Pin|IR_L_Pin|IR_FR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOTOR_L_DIR_Pin|FRAM_CS_Pin|LED_2_Pin|LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_R_DIR_Pin|MOTOR_STBY_Pin|IMU_CS_Pin|IR_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IR_FL_Pin IR_L_Pin IR_FR_Pin */
  GPIO_InitStruct.Pin = IR_FL_Pin|IR_L_Pin|IR_FR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PUSH_IN_1_Pin */
  GPIO_InitStruct.Pin = PUSH_IN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PUSH_IN_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_3_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_L_DIR_Pin FRAM_CS_Pin LED_2_Pin LED_1_Pin */
  GPIO_InitStruct.Pin = MOTOR_L_DIR_Pin|FRAM_CS_Pin|LED_2_Pin|LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_R_DIR_Pin MOTOR_STBY_Pin IMU_CS_Pin IR_R_Pin */
  GPIO_InitStruct.Pin = MOTOR_R_DIR_Pin|MOTOR_STBY_Pin|IMU_CS_Pin|IR_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
/* ---------- TIM 割り込みコールバック ---------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5)
  {
    f413_ctrl_tick();
    nightfall_trace_log_auto_tick_sample();
  }
  else if (htim->Instance == TIM6)
  {
    nightfall_wall_sensor_tim6_tick();
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  f413_wall_sensor_adc_complete(hadc);
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
