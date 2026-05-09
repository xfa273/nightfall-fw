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
#include "params.h"
#include "shortest_run_params.h"
#include "solver.h"
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
static uint8_t g_trace_log_auto_enabled = 0U;
static uint32_t g_trace_log_auto_period_ms = 10U;
static uint32_t g_trace_log_auto_next_tick_ms = 0U;
static uint32_t g_trace_log_auto_seq = 0U;
static uint16_t g_trace_log_auto_mode_flags = 0U;
static uint8_t g_trace_log_context_mode = 0xFFU;
static uint8_t g_trace_log_context_case = 0xFFU;
static uint8_t g_trace_log_context_sub = 0xFFU;
static uint8_t g_trace_log_context_test_id = 0U;

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

#define NIGHTFALL_F413_TEST_MAZE_CELLS (32U)
#define NIGHTFALL_F413_TEST_TRACE_BYTES (32U)
#define NIGHTFALL_F413_TEST_TRACE_OFFSET (0x100U)
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
#define NIGHTFALL_F413_TRACE_ABORT_SWITCH_FLAG (0x0100U)
#define NIGHTFALL_F413_TRACE_ABORT_WALL_FAULT_FLAG (0x0200U)
#define NIGHTFALL_F413_TRACE_ABORT_ENCODER_FAULT_FLAG (0x0400U)
#define NIGHTFALL_F413_TRACE_ABORT_IMU_FAULT_FLAG (0x0800U)
#define NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG (0x1000U)
#define NIGHTFALL_F413_TRACE_ANGLE_TARGET_FLAG (0x2000U)
#define NIGHTFALL_F413_TRACE_AUTO_FLAG (0x8000U)
#define NIGHTFALL_F413_RUN_SESSION_IDLE_MS (1000U)
#define NIGHTFALL_F413_RUN_SESSION_MOTOR_DUTY (120U)
#define NIGHTFALL_F413_RUN_SESSION_MOTOR_PULSE_MS (300U)
#define NIGHTFALL_F413_RUN_SESSION_MOTOR_COAST_MS (120U)
#define NIGHTFALL_F413_RUN_SESSION_SAFE_DUTY (70U)
#define NIGHTFALL_F413_RUN_SESSION_SAFE_FORWARD_MS (160U)
#define NIGHTFALL_F413_RUN_SESSION_SAFE_TURN_MS (120U)
#define NIGHTFALL_F413_RUN_SESSION_SAFE_COAST_MS (80U)
#define NIGHTFALL_F413_RUN_SESSION_SAFE_EXPLORE_STEPS (4U)
#define NIGHTFALL_F413_RUN_GUARD_WALL_CHECK_MS (20U)
#define NIGHTFALL_F413_RUN_GUARD_IMU_CHECK_MS (100U)
#define NIGHTFALL_F413_RUN_GUARD_ENCODER_DELTA_MAX (6000)
#define NIGHTFALL_F413_RUN_GUARD_WALL_SAT_ADC (4090U)
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
#define NIGHTFALL_F413_REAL_RUN_PATH_ENABLED (0U)
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
#define NIGHTFALL_F413_TEST_VEL           (200.0f)  /* テスト直進速度 [mm/s] */
#define NIGHTFALL_F413_TEST_OMEGA         (200.0f)  /* テスト旋回角速度 [deg/s] */
#define NIGHTFALL_F413_TEST_HALF_CELL_MM  (45.0f)   /* 半区画 [mm] */
#define NIGHTFALL_F413_TEST_COAST_MS      (200U)    /* 区間間停止 [ms] */
#define NIGHTFALL_F413_TEST_TIMEOUT_MS    (5000U)   /* タイムアウト [ms] */
#define NIGHTFALL_F413_TEST_ARMED_NONE    (0U)
#define NIGHTFALL_F413_TEST_BUTTON_WAIT_MS (3000U)  /* ボタン待ち上限 [ms] */
#define NIGHTFALL_F413_ENCODER_WRAP_COUNT (60000L)
#define NIGHTFALL_F413_ENCODER_WRAP_HALF  (NIGHTFALL_F413_ENCODER_WRAP_COUNT / 2L)
#define NIGHTFALL_F413_ENCODER_SIGN_L     (1L)
#define NIGHTFALL_F413_ENCODER_SIGN_R     (-1L)
#define NIGHTFALL_F413_MOTOR_PWM_MAX      (1000U)
#define NIGHTFALL_F413_OP_MODE_MAX        (9U)
#define NIGHTFALL_F413_OP_SELECT_MAX      (9U)
#define NIGHTFALL_F413_OP_LEVEL_TOP      (0U)
#define NIGHTFALL_F413_OP_LEVEL_CASE     (1U)
#define NIGHTFALL_F413_OP_LEVEL_SUB      (2U)
#define NIGHTFALL_F413_OP_UI_POLL_MS      (40U)
#define NIGHTFALL_F413_OP_BUTTON_LOCK_MS  (250U)
#define NIGHTFALL_F413_OP_ENTER_DELTA_ADC (150)
#define NIGHTFALL_F413_OP_ENTER_RELEASE_ADC (250)
#define NIGHTFALL_F413_OP_ENTER_STREAK    (3U)
#define NIGHTFALL_F413_OP_START_DELAY_MS  (2000U)
#define NIGHTFALL_F413_OP_TEST_VEL_CAP    (450.0f)
#define NIGHTFALL_F413_OP_TEST_DIAG_SCALE (0.75f)

typedef enum
{
  NIGHTFALL_RUN_ABORT_NONE = 0,
  NIGHTFALL_RUN_ABORT_SWITCH,
  NIGHTFALL_RUN_ABORT_WALL_FAULT,
  NIGHTFALL_RUN_ABORT_ENCODER_FAULT,
  NIGHTFALL_RUN_ABORT_IMU_FAULT,
} nightfall_run_abort_reason_t;

typedef struct
{
  int16_t prev_encoder_l;
  int16_t prev_encoder_r;
  uint32_t next_wall_check_ms;
  uint32_t next_imu_check_ms;
  uint8_t adc_prepared;
} nightfall_run_guard_t;

static void nightfall_fill_trace_log_selftest_record(nvm_trace_log_record_t* out, uint32_t seq);
static uint8_t nightfall_trace_log_record_equals(const nvm_trace_log_record_t* lhs,
                                                 const nvm_trace_log_record_t* rhs);
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
static bool nightfall_run_guard_prepare(nightfall_run_guard_t* guard);
static void nightfall_run_guard_cleanup(nightfall_run_guard_t* guard);
static nightfall_run_abort_reason_t nightfall_trace_log_wait_with_auto_step_guarded(uint32_t duration_ms,
                                                                                     nightfall_run_guard_t* guard);
static uint16_t nightfall_run_abort_reason_to_trace_flag(nightfall_run_abort_reason_t reason);
static const char* nightfall_run_abort_reason_to_text(nightfall_run_abort_reason_t reason);
static void nightfall_trace_log_on_run_start(void);
static void nightfall_trace_log_on_run_stop(void);
static void nightfall_trace_log_auto_step(void);
static void nightfall_trace_log_set_mode_flags(uint16_t flags);
static void nightfall_trace_log_set_context(uint8_t mode, uint8_t op_case, uint8_t sub, uint8_t test_id);
static int32_t nightfall_trace_log_scale_float(float value, float scale);
static bool nightfall_trace_log_read_adc_raw(uint16_t* fr, uint16_t* r, uint16_t* fl, uint16_t* l, uint16_t* vbat);
static bool nightfall_adc_prepare_single_conversion(void);
static bool nightfall_adc_read_single_channel(uint32_t channel, uint16_t* out);
static void nightfall_motor_set(bool enable, bool left_forward, bool right_forward,
                                uint16_t left_duty, uint16_t right_duty);
static void nightfall_test_run(uint8_t test_id);
static void nightfall_test_arm_for_button(uint8_t test_id);
static void nightfall_op_ui_step(void);
static int32_t nightfall_encoder_delta_signed(uint32_t now, uint32_t prev);
static void nightfall_run_imu_manual_turn_test_once(void);
static void nightfall_run_imu_accel_test_once(void);
static void nightfall_run_fan_pwm_test_once(void);
static void nightfall_run_encoder_test_once(void);

static volatile uint8_t g_test_armed_id = 0U; /* 0=none, '1'-'5'=armed test */
static uint8_t g_last_test_id = 0U;
static nightfall_run_abort_reason_t g_last_test_abort_reason = NIGHTFALL_RUN_ABORT_NONE;
static float g_last_test_distance_mm = 0.0f;
static float g_last_test_angle_deg = 0.0f;
static uint8_t g_op_mode = 0U;
static uint8_t g_op_case = 0U;
static uint8_t g_op_sub = 0U;
static uint8_t g_op_level = NIGHTFALL_F413_OP_LEVEL_TOP;
static uint32_t g_op_next_ui_poll_ms = 0U;
static uint32_t g_op_button_lock_until_ms = 0U;
static uint8_t g_op_enter_streak = 0U;
static bool g_op_button_prev_pressed = false;
static bool g_op_enter_latched = false;

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

  if (g_trace_log_auto_enabled != 0U)
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
  if (g_trace_log_auto_enabled != 0U)
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

static nightfall_run_abort_reason_t nightfall_test_wait_target(
    float target_abs, bool is_angle, nightfall_run_guard_t* guard, uint16_t flags)
{
  nightfall_run_abort_reason_t reason = NIGHTFALL_RUN_ABORT_NONE;
  uint32_t deadline = HAL_GetTick() + NIGHTFALL_F413_TEST_TIMEOUT_MS;
  while (1)
  {
    float cur = is_angle ? fabsf(f413_ctrl_get_angle())
                         : fabsf(f413_ctrl_get_distance());
    if (cur >= target_abs) break;
    if (HAL_GetTick() >= deadline) break;
    nightfall_trace_log_set_mode_flags(flags);
    reason = nightfall_trace_log_wait_with_auto_step_guarded(10U, guard);
    if (reason != NIGHTFALL_RUN_ABORT_NONE) return reason;
  }
  return NIGHTFALL_RUN_ABORT_NONE;
}

static nightfall_run_abort_reason_t nightfall_test_wait_angle_signed(
    float target_deg, nightfall_run_guard_t* guard, uint16_t flags)
{
  nightfall_run_abort_reason_t reason = NIGHTFALL_RUN_ABORT_NONE;
  uint32_t deadline = HAL_GetTick() + NIGHTFALL_F413_TEST_TIMEOUT_MS;
  while (1)
  {
    float cur = f413_ctrl_get_angle();
    if (((target_deg >= 0.0f) && (cur >= target_deg)) ||
        ((target_deg < 0.0f) && (cur <= target_deg)))
    {
      break;
    }
    if (HAL_GetTick() >= deadline) break;
    nightfall_trace_log_set_mode_flags(flags);
    reason = nightfall_trace_log_wait_with_auto_step_guarded(10U, guard);
    if (reason != NIGHTFALL_RUN_ABORT_NONE) return reason;
  }
  return NIGHTFALL_RUN_ABORT_NONE;
}

static void nightfall_test_run(uint8_t test_id)
{
  nightfall_run_abort_reason_t abort_reason = NIGHTFALL_RUN_ABORT_NONE;
  nightfall_run_guard_t guard = {0};
  const uint16_t tf = NIGHTFALL_F413_TRACE_MODE_SOLVER_PATH_FLAG; /* reuse for test */

  const char* test_name = "?";
  switch (test_id)
  {
    case '1': test_name = "straight 3 cells heading hold (270mm)"; break;
    case '2': test_name = "straight 6 cells heading hold (540mm)"; break;
    case '3': test_name = "right 90 deg turn"; break;
    case '4': test_name = "left 90 deg turn"; break;
    case '5': test_name = "S3 + R90 + S3 heading hold"; break;
    case '6': test_name = "HW: L-motor fwd only"; break;
    case '7': test_name = "HW: R-motor fwd only"; break;
    case '8': test_name = "HW: L-motor rev only"; break;
    case '9': test_name = "HW: R-motor rev only"; break;
    default:
      trace_printf("[TEST] unknown test_id '%c'\r\n", (char)test_id);
      return;
  }

  trace_printf("[TEST] === %s === start\r\n", test_name);

  if (test_id >= '6' && test_id <= '9')
  {
    bool use_left  = (test_id == '6' || test_id == '8');
    bool forward   = (test_id == '6' || test_id == '7');
    uint16_t duty  = 120U;
    uint32_t run_ms = 500U;
    uint32_t coast_ms = 300U;

    (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    trace_printf("[TEST] motor=%s dir=%s duty=%u ms=%lu\r\n",
                 use_left ? "LEFT" : "RIGHT",
                 forward ? "FWD" : "REV",
                 (unsigned int)duty, (unsigned long)run_ms);
    trace_printf("[TEST] PWM: CH1(TIM2)=%s, CH3(TIM2)=%s\r\n",
                 use_left ? (forward ? "duty" : "inv-duty") : "0",
                 use_left ? "0" : (forward ? "inv-duty" : "duty"));
    trace_printf("[TEST] IN2: L=%s, R=%s\r\n",
                 (use_left && forward) ? "RESET" :
                 (use_left && !forward) ? "SET" : "RESET",
                 (!use_left && forward) ? "SET" :
                 (!use_left && !forward) ? "RESET" : "RESET");

    if (use_left)
    {
      nightfall_motor_set(true, forward, true, duty, 0U);
    }
    else
    {
      nightfall_motor_set(true, true, forward, 0U, duty);
    }

    HAL_Delay(run_ms);
    nightfall_motor_set(false, true, true, 0U, 0U);
    HAL_Delay(coast_ms);

    uint32_t final_enc_l_raw = __HAL_TIM_GET_COUNTER(&htim3);
    uint32_t final_enc_r_raw = __HAL_TIM_GET_COUNTER(&htim4);
    int32_t final_enc_l = NIGHTFALL_F413_ENCODER_SIGN_L *
        nightfall_encoder_delta_signed(final_enc_l_raw, 0U);
    int32_t final_enc_r = NIGHTFALL_F413_ENCODER_SIGN_R *
        nightfall_encoder_delta_signed(final_enc_r_raw, 0U);

    (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 30000U);
    __HAL_TIM_SET_COUNTER(&htim4, 30000U);
    (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    trace_printf("[TEST] === %s === OK enc_l=%ld enc_r=%ld\r\n",
                 test_name,
                 (long)final_enc_l, (long)final_enc_r);
    return;
  }

  if (!nightfall_run_guard_prepare(&guard))
  {
    trace_printf("[TEST] guard init fail\r\n");
    return;
  }

  nightfall_trace_log_on_run_start();

  /* テスト6-9はオープンループ（制御ループ不使用） */
  if (test_id >= '1' && test_id <= '5')
  {
    f413_ctrl_start();
  }

  switch (test_id)
  {
    case '1': /* straight 6 half-cells = 270mm */
    {
      f413_ctrl_reset_distance();
      f413_ctrl_reset_angle();
      f413_ctrl_set_velocity(NIGHTFALL_F413_TEST_VEL);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);
      abort_reason = nightfall_test_wait_target(
          6.0f * NIGHTFALL_F413_TEST_HALF_CELL_MM, false, &guard,
          (uint16_t)(tf | NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
      break;
    }
    case '2': /* straight 12 half-cells = 540mm */
    {
      f413_ctrl_reset_distance();
      f413_ctrl_reset_angle();
      f413_ctrl_set_velocity(NIGHTFALL_F413_TEST_VEL);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);
      abort_reason = nightfall_test_wait_target(
          12.0f * NIGHTFALL_F413_TEST_HALF_CELL_MM, false, &guard,
          (uint16_t)(tf | NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
      break;
    }
    case '3': /* right 90 */
    {
      f413_ctrl_reset_angle();
      f413_ctrl_set_velocity(0.0f);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(-90.0f);
      abort_reason = nightfall_test_wait_angle_signed(
          -90.0f, &guard,
          (uint16_t)(tf | NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG));
      break;
    }
    case '4': /* left 90 */
    {
      f413_ctrl_reset_angle();
      f413_ctrl_set_velocity(0.0f);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(90.0f);
      abort_reason = nightfall_test_wait_angle_signed(
          90.0f, &guard,
          (uint16_t)(tf | NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG));
      break;
    }
    case '5': /* S6 + R90 + S6 */
    {
      /* segment 1: straight 6 half-cells */
      f413_ctrl_reset_distance();
      f413_ctrl_reset_angle();
      f413_ctrl_set_velocity(NIGHTFALL_F413_TEST_VEL);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);
      abort_reason = nightfall_test_wait_target(
          6.0f * NIGHTFALL_F413_TEST_HALF_CELL_MM, false, &guard,
          (uint16_t)(tf | NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
      if (abort_reason != NIGHTFALL_RUN_ABORT_NONE) break;

      /* coast */
      f413_ctrl_clear_angle_target();
      f413_ctrl_set_velocity(0.0f);
      f413_ctrl_set_omega(0.0f);
      nightfall_trace_log_set_mode_flags((uint16_t)(tf | NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG));
      abort_reason = nightfall_trace_log_wait_with_auto_step_guarded(
          NIGHTFALL_F413_TEST_COAST_MS, &guard);
      if (abort_reason != NIGHTFALL_RUN_ABORT_NONE) break;

      /* segment 2: right 90 */
      f413_ctrl_reset_angle();
      f413_ctrl_set_velocity(0.0f);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(-90.0f);
      abort_reason = nightfall_test_wait_angle_signed(
          -90.0f, &guard,
          (uint16_t)(tf | NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG));
      if (abort_reason != NIGHTFALL_RUN_ABORT_NONE) break;

      /* coast */
      f413_ctrl_clear_angle_target();
      f413_ctrl_set_velocity(0.0f);
      f413_ctrl_set_omega(0.0f);
      nightfall_trace_log_set_mode_flags((uint16_t)(tf | NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG));
      abort_reason = nightfall_trace_log_wait_with_auto_step_guarded(
          NIGHTFALL_F413_TEST_COAST_MS, &guard);
      if (abort_reason != NIGHTFALL_RUN_ABORT_NONE) break;

      /* segment 3: straight 6 half-cells */
      f413_ctrl_reset_distance();
      f413_ctrl_reset_angle();
      f413_ctrl_set_velocity(NIGHTFALL_F413_TEST_VEL);
      f413_ctrl_set_omega(0.0f);
      f413_ctrl_set_angle_target(0.0f);
      abort_reason = nightfall_test_wait_target(
          6.0f * NIGHTFALL_F413_TEST_HALF_CELL_MM, false, &guard,
          (uint16_t)(tf | NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG));
      break;
    }
    default:
      break;
  }

  /* 停止 (テスト1-5用) */
  f413_ctrl_clear_angle_target();
  f413_ctrl_set_velocity(0.0f);
  f413_ctrl_set_omega(0.0f);

  /* 減速待ち */
  nightfall_trace_log_set_mode_flags((uint16_t)(tf | NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG));
  (void)nightfall_trace_log_wait_with_auto_step_guarded(NIGHTFALL_F413_TEST_COAST_MS, &guard);

  f413_ctrl_stop();

  if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
  {
    nightfall_trace_log_set_mode_flags((uint16_t)(tf |
        nightfall_run_abort_reason_to_trace_flag(abort_reason)));
    nightfall_trace_log_auto_step();
  }
  nightfall_trace_log_set_mode_flags(0U);
  nightfall_trace_log_on_run_stop();
  nightfall_run_guard_cleanup(&guard);

  g_last_test_id = test_id;
  g_last_test_abort_reason = abort_reason;
  g_last_test_distance_mm = f413_ctrl_get_distance();
  g_last_test_angle_deg = f413_ctrl_get_angle();

  trace_printf("[TEST] === %s === %s dist=%.0fmm angle=%.0fdeg\r\n",
               test_name,
               (abort_reason == NIGHTFALL_RUN_ABORT_NONE) ? "OK" :
               nightfall_run_abort_reason_to_text(abort_reason),
               (double)g_last_test_distance_mm,
               (double)g_last_test_angle_deg);
}

static void nightfall_test_arm_for_button(uint8_t test_id)
{
  g_test_armed_id = test_id;
  nightfall_trace_log_set_context(8U, 0xFFU, 0xFFU, test_id);

  const char* test_name = "?";
  switch (test_id)
  {
    case '1': test_name = "straight 3 cells heading hold"; break;
    case '2': test_name = "straight 6 cells heading hold"; break;
    case '3': test_name = "right 90 deg"; break;
    case '4': test_name = "left 90 deg"; break;
    case '5': test_name = "S3+R90+S3 heading hold"; break;
    default: test_name = "unknown"; break;
  }

  trace_printf("[TEST] armed for button: '%c' (%s)\r\n", (char)test_id, test_name);
  trace_printf("[TEST] disconnect UART, press PUSH button to start, reconnect and 'V' to get CSV\r\n");
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

static bool nightfall_trace_log_read_adc_raw(uint16_t* fr, uint16_t* r, uint16_t* fl, uint16_t* l, uint16_t* vbat)
{
  if ((fr == NULL) || (r == NULL) || (fl == NULL) || (l == NULL) || (vbat == NULL))
  {
    return false;
  }

  *fr = 0U;
  *r = 0U;
  *fl = 0U;
  *l = 0U;
  *vbat = 0U;

  if (!nightfall_adc_prepare_single_conversion())
  {
    return false;
  }

  return nightfall_adc_read_single_channel(ADC_CHANNEL_0, fr) &&
         nightfall_adc_read_single_channel(ADC_CHANNEL_1, r) &&
         nightfall_adc_read_single_channel(ADC_CHANNEL_2, fl) &&
         nightfall_adc_read_single_channel(ADC_CHANNEL_3, l) &&
         nightfall_adc_read_single_channel(ADC_CHANNEL_8, vbat);
}

static void nightfall_run_trace_log_dump_csv_impl(uint32_t max_records)
{
  nvm_trace_log_header_t header;
  nvm_status_t st;
  uint32_t available;
  uint32_t dump_count;
  uint32_t i;

  st = nvm_trace_log_get_header(&header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] csv: FAIL(read header NVM=%d, run q first)\r\n", (int)st);
    return;
  }

  available = header.total_records;
  if (available > header.record_capacity)
  {
    available = header.record_capacity;
  }
  if (available == 0U)
  {
    trace_printf("[TRACE-LOG] csv: no records\r\n");
    return;
  }

  dump_count = available;
  if ((max_records > 0U) && (dump_count > max_records))
  {
    dump_count = max_records;
  }

  trace_printf("[TRACE-LOG] csv latest %lu/%lu (oldest->newest)\r\n",
               (unsigned long)dump_count,
               (unsigned long)available);
  trace_printf("#log_format=nightfall_trace_csv_v2\r\n");
  trace_printf("#fw_target=%s\r\n", FW_TARGET);
  trace_printf("#fw_version=%s\r\n", FW_VERSION);
  trace_printf("#fw_build_type=%s\r\n", FW_BUILD_TYPE);
  trace_printf("#fw_git_sha=%s\r\n", FW_GIT_SHA);
  trace_printf("#fw_git_dirty=%d\r\n", FW_GIT_DIRTY);
  trace_printf("#fw_log_schema=0x%08lX\r\n", (unsigned long)NVM_TRACE_LOG_SCHEMA_VERSION);
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
  trace_printf("#mm_columns=timestamp_ms,seq,op_mode,op_case,op_sub,test_id,");
  trace_printf("distance_mm,angle_mdeg,target_velocity_mm_s,real_velocity_mm_s,accel_velocity_mm_s,");
  trace_printf("target_omega_mdps,real_omega_mdps,target_angle_mdeg,accel_forward_mm_s2,");
  trace_printf("encoder_l,encoder_r,motor_out_l,motor_out_r,adc_fr,adc_r,adc_fl,adc_l,adc_vbat,");
  trace_printf("flags,reserved_i32_0,reserved_i32_1,reserved_i32_2,reserved_i32_3,reserved_u16_0,reserved_u16_1\r\n");

  for (i = dump_count; i > 0U; i--)
  {
    nvm_trace_log_record_t rec;

    st = nvm_trace_log_read_latest(i - 1U, &rec);
    if (st != NVM_STATUS_OK)
    {
      trace_printf("[TRACE-LOG] csv: FAIL(read idx=%lu NVM=%d)\r\n",
                   (unsigned long)(i - 1U),
                   (int)st);
      return;
    }

    trace_printf("%lu,%lu,%u,%u,%u,%u,%ld,%ld,%ld,%ld,",
                 (unsigned long)rec.timestamp_ms,
                 (unsigned long)rec.seq,
                 (unsigned int)rec.op_mode,
                 (unsigned int)rec.op_case,
                 (unsigned int)rec.op_sub,
                 (unsigned int)rec.test_id,
                 (long)rec.distance_mm,
                 (long)rec.angle_mdeg,
                 (long)rec.target_velocity_mm_s,
                 (long)rec.real_velocity_mm_s);
    trace_printf("%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%u,",
                 (long)rec.accel_velocity_mm_s,
                 (long)rec.target_omega_mdps,
                 (long)rec.real_omega_mdps,
                 (long)rec.target_angle_mdeg,
                 (long)rec.accel_forward_mm_s2,
                 (int)rec.encoder_l,
                 (int)rec.encoder_r,
                 (int)rec.motor_out_l,
                 (int)rec.motor_out_r,
                 (unsigned int)rec.adc_fr);
    trace_printf("%u,%u,%u,%u,%u,%ld,%ld,%ld,%ld,%u,%u\r\n",
                 (unsigned int)rec.adc_r,
                 (unsigned int)rec.adc_fl,
                 (unsigned int)rec.adc_l,
                 (unsigned int)rec.adc_vbat,
                 (unsigned int)rec.flags,
                 (long)rec.reserved_i32_0,
                 (long)rec.reserved_i32_1,
                 (long)rec.reserved_i32_2,
                 (long)rec.reserved_i32_3,
                 (unsigned int)rec.reserved_u16_0,
                 (unsigned int)rec.reserved_u16_1);
  }

  trace_printf("[TRACE-LOG] csv: done\r\n");
}

static void nightfall_run_trace_log_dump_csv_once(void)
{
  nightfall_run_trace_log_dump_csv_impl(NIGHTFALL_F413_TRACE_CSV_MAX_RECORDS);
}

static void nightfall_run_trace_log_dump_csv_all_once(void)
{
  nightfall_run_trace_log_dump_csv_impl(0U);
}

static void nightfall_run_trace_log_selftest_once(void)
{
  nvm_trace_log_header_t header;
  nvm_status_t st;
  uint32_t i;

  g_trace_log_auto_enabled = 0U;
  g_trace_log_auto_mode_flags = 0U;

  st = nvm_trace_log_format();
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG][SELFTEST] FAIL(format NVM=%d)\r\n", (int)st);
    return;
  }

  for (i = 0U; i < NIGHTFALL_F413_TRACE_SELFTEST_RECORDS; i++)
  {
    nvm_trace_log_record_t rec;
    nightfall_fill_trace_log_selftest_record(&rec, i);

    st = nvm_trace_log_append(&rec);
    if (st != NVM_STATUS_OK)
    {
      trace_printf("[TRACE-LOG][SELFTEST] FAIL(append i=%lu NVM=%d)\r\n",
                   (unsigned long)i,
                   (int)st);
      return;
    }
  }

  st = nvm_trace_log_get_header(&header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG][SELFTEST] FAIL(read header NVM=%d)\r\n", (int)st);
    return;
  }

  if (header.total_records < NIGHTFALL_F413_TRACE_SELFTEST_RECORDS)
  {
    trace_printf("[TRACE-LOG][SELFTEST] FAIL(total=%lu)\r\n", (unsigned long)header.total_records);
    return;
  }

  for (i = 0U; i < NIGHTFALL_F413_TRACE_SELFTEST_RECORDS; i++)
  {
    nvm_trace_log_record_t got;
    nvm_trace_log_record_t expected;
    uint32_t expected_seq = NIGHTFALL_F413_TRACE_SELFTEST_RECORDS - 1U - i;

    st = nvm_trace_log_read_latest(i, &got);
    if (st != NVM_STATUS_OK)
    {
      trace_printf("[TRACE-LOG][SELFTEST] FAIL(read latest i=%lu NVM=%d)\r\n",
                   (unsigned long)i,
                   (int)st);
      return;
    }

    nightfall_fill_trace_log_selftest_record(&expected, expected_seq);
    if (nightfall_trace_log_record_equals(&got, &expected) == 0U)
    {
      trace_printf("[TRACE-LOG][SELFTEST] FAIL(mismatch i=%lu got_seq=%lu exp_seq=%lu)\r\n",
                   (unsigned long)i,
                   (unsigned long)got.seq,
                   (unsigned long)expected.seq);
      return;
    }
  }

  trace_printf("[TRACE-LOG][SELFTEST] PASS records=%lu\r\n",
               (unsigned long)NIGHTFALL_F413_TRACE_SELFTEST_RECORDS);
}

static void nightfall_fill_expected_trace_bytes(uint8_t* out, uint32_t count)
{
  uint32_t i;

  if ((out == NULL) && (count > 0U))
  {
    return;
  }

  for (i = 0U; i < count; i++)
  {
    out[i] = (uint8_t)(0xA0U + (i * 7U));
  }
}

static void nightfall_print_nvm_cli_help(void)
{
  trace_printf("[NVM-TEST] commands: h=help, a=save+load all, A=load-only all\r\n");
  trace_printf("[NVM-TEST] d/s/m/t=save+load, D/S/M/T=load-only verify\r\n");
  trace_printf("[TRACE-LOG] q=format, r=append sample, R=dump latest, v=dump csv(256), V=dump csv(all/full), k=selftest, u=run-start hook, U=run-stop hook\r\n");
  trace_printf("[RUN-TEST]  x=idle-run-session(1000ms), y=motor-run-session(short), z=search-entry(solver/fallback), j=shortest-entry(solver/fallback)\r\n");
  trace_printf("[HW-TEST]  w=wall, p=switch, i=imu, I=imu-angle, c=imu-accel, b=buzzer, o/0=motor, e=encoder, l=led30s, g=smoke+trace\r\n");
  trace_printf("[TEST]     1=S3straight, 2=S6straight, 3=R90turn, 4=L90turn, 5=S3+R90+S3, F=arm for button; OP mode2-7/case0/sub0-9=path-code tests\r\n");
  trace_printf("[HW-ENC]  6=L-motor-fwd, 7=R-motor-fwd, 8=L-motor-rev, 9=R-motor-rev (open-loop+enc)\r\n");
  trace_printf("[OP-UI]   F405-compatible select: PUSH increments 0..9 at each level, FR wall only=enter, mode9 case5=dump latest full log\r\n");
  trace_printf("[OP-UART] P=PUSH increment, E=FR enter; reset via ST-LINK software reset\r\n");
}

static void nightfall_print_trace_log_header(const nvm_trace_log_header_t* header)
{
  uint32_t stored;

  if (header == NULL)
  {
    return;
  }

  stored = header->total_records;
  if (stored > header->record_capacity)
  {
    stored = header->record_capacity;
  }

  trace_printf("[TRACE-LOG] header ver=0x%08lX rec_size=%lu cap=%lu write=%lu total=%lu stored=%lu\r\n",
               (unsigned long)header->version,
               (unsigned long)header->record_size,
               (unsigned long)header->record_capacity,
               (unsigned long)header->write_index,
               (unsigned long)header->total_records,
               (unsigned long)stored);
}

static void nightfall_run_trace_log_format_once(void)
{
  nvm_trace_log_header_t header;
  nvm_status_t st;

  g_trace_log_auto_enabled = 0U;
  g_trace_log_auto_mode_flags = 0U;

  st = nvm_trace_log_format();
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] format: FAIL (NVM=%d)\r\n", (int)st);
    return;
  }

  st = nvm_trace_log_get_header(&header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] format: FAIL(read header NVM=%d)\r\n", (int)st);
    return;
  }

  trace_printf("[TRACE-LOG] format: PASS\r\n");
  nightfall_print_trace_log_header(&header);
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
  out->distance_mm = nightfall_trace_log_scale_float(f413_ctrl_get_distance(), 1.0f);
  out->angle_mdeg = nightfall_trace_log_scale_float(f413_ctrl_get_angle(), 1000.0f);
  out->target_velocity_mm_s = nightfall_trace_log_scale_float(f413_ctrl_get_target_velocity(), 1.0f);
  out->real_velocity_mm_s = nightfall_trace_log_scale_float(f413_ctrl_get_real_velocity(), 1.0f);
  out->accel_velocity_mm_s = nightfall_trace_log_scale_float(f413_ctrl_get_accel_velocity(), 1.0f);
  out->target_omega_mdps = nightfall_trace_log_scale_float(f413_ctrl_get_target_omega(), 1000.0f);
  out->real_omega_mdps = nightfall_trace_log_scale_float(f413_ctrl_get_real_omega(), 1000.0f);
  out->target_angle_mdeg = nightfall_trace_log_scale_float(f413_ctrl_get_target_angle(), 1000.0f);
  out->accel_forward_mm_s2 = nightfall_trace_log_scale_float(f413_ctrl_get_accel_forward(), 1.0f);
  out->encoder_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  out->encoder_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  out->motor_out_l = f413_ctrl_get_motor_out_l();
  out->motor_out_r = f413_ctrl_get_motor_out_r();
  if (nightfall_trace_log_read_adc_raw(&adc_fr, &adc_r, &adc_fl, &adc_l, &adc_vbat))
  {
    out->adc_fr = adc_fr;
    out->adc_r = adc_r;
    out->adc_fl = adc_fl;
    out->adc_l = adc_l;
    out->adc_vbat = adc_vbat;
  }
  out->flags = (HAL_GPIO_ReadPin(PUSH_IN_1_GPIO_Port, PUSH_IN_1_Pin) == GPIO_PIN_RESET)
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

static void nightfall_fill_trace_log_selftest_record(nvm_trace_log_record_t* out, uint32_t seq)
{
  if (out == NULL)
  {
    return;
  }

  memset(out, 0, sizeof(*out));
  out->seq = seq;
  out->timestamp_ms = 1000U + seq * 10U;
  out->distance_mm = (int32_t)(10 + (int32_t)seq);
  out->angle_mdeg = (int32_t)(-1000 - (int32_t)seq);
  out->target_velocity_mm_s = (int32_t)(200 + (int32_t)seq);
  out->real_velocity_mm_s = (int32_t)(190 + (int32_t)seq);
  out->accel_velocity_mm_s = (int32_t)(185 + (int32_t)seq);
  out->target_omega_mdps = (int32_t)(3000 + (int32_t)seq);
  out->real_omega_mdps = (int32_t)(2900 + (int32_t)seq);
  out->target_angle_mdeg = (int32_t)(90000 + (int32_t)seq);
  out->accel_forward_mm_s2 = (int32_t)(50 + (int32_t)seq);
  out->reserved_i32_0 = (int32_t)(1000 + (int32_t)seq);
  out->reserved_i32_1 = (int32_t)(2000 + (int32_t)seq);
  out->reserved_i32_2 = (int32_t)(3000 + (int32_t)seq);
  out->reserved_i32_3 = (int32_t)(4000 + (int32_t)seq);
  out->encoder_l = (int16_t)(100 + (int32_t)seq);
  out->encoder_r = (int16_t)(-100 - (int32_t)seq);
  out->motor_out_l = (int16_t)(200 + (int32_t)(seq * 2U));
  out->motor_out_r = (int16_t)(-200 - (int32_t)(seq * 2U));
  out->adc_fr = (uint16_t)(1000U + seq);
  out->adc_r = (uint16_t)(1100U + seq);
  out->adc_fl = (uint16_t)(1200U + seq);
  out->adc_l = (uint16_t)(1300U + seq);
  out->adc_vbat = (uint16_t)(1400U + seq);
  out->flags = (uint16_t)(0xA500U | (seq & 0x00FFU));
  out->op_mode = 9U;
  out->op_case = 5U;
  out->op_sub = (uint8_t)(seq & 0xFFU);
  out->test_id = (uint8_t)'T';
  out->reserved_u16_0 = (uint16_t)(0x5500U | (seq & 0x00FFU));
  out->reserved_u16_1 = (uint16_t)(0x6600U | (seq & 0x00FFU));
}

static uint8_t nightfall_trace_log_record_equals(const nvm_trace_log_record_t* lhs,
                                                 const nvm_trace_log_record_t* rhs)
{
  if ((lhs == NULL) || (rhs == NULL))
  {
    return 0U;
  }

  return (memcmp(lhs, rhs, sizeof(*lhs)) == 0) ? 1U : 0U;
}

static void nightfall_run_trace_log_append_sample_once(void)
{
  nvm_trace_log_header_t header;
  nvm_trace_log_record_t rec;
  nvm_status_t st;

  st = nvm_trace_log_get_header(&header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] append: FAIL(read header NVM=%d, run q first)\r\n", (int)st);
    return;
  }

  nightfall_fill_trace_log_sample(&rec, header.total_records);

  st = nvm_trace_log_append(&rec);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] append: FAIL(write NVM=%d)\r\n", (int)st);
    return;
  }

  trace_printf("[TRACE-LOG] append: PASS seq=%lu ts=%lu dist=%ld angle=%ld enc=(%d,%d) motor=(%d,%d) flags=0x%04X\r\n",
               (unsigned long)rec.seq,
               (unsigned long)rec.timestamp_ms,
               (long)rec.distance_mm,
               (long)rec.angle_mdeg,
               (int)rec.encoder_l,
               (int)rec.encoder_r,
               (int)rec.motor_out_l,
               (int)rec.motor_out_r,
               (unsigned int)rec.flags);
}

static void nightfall_trace_log_auto_start(void)
{
  nvm_status_t st;

  if (g_trace_log_auto_enabled != 0U)
  {
    trace_printf("[TRACE-LOG] auto: already running\r\n");
    return;
  }

  st = nvm_trace_log_format();
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] auto: FAIL(format NVM=%d)\r\n", (int)st);
    return;
  }

  g_trace_log_auto_enabled = 1U;
  g_trace_log_auto_seq = 0U;
  g_trace_log_auto_mode_flags = 0U;
  g_trace_log_auto_next_tick_ms = HAL_GetTick() + g_trace_log_auto_period_ms;
  trace_printf("[TRACE-LOG] auto: START period=%lu ms (fresh format)\r\n",
               (unsigned long)g_trace_log_auto_period_ms);
}

static void nightfall_trace_log_auto_stop(void)
{
  nvm_trace_log_header_t header;
  nvm_status_t st;

  if (g_trace_log_auto_enabled == 0U)
  {
    trace_printf("[TRACE-LOG] auto: already stopped\r\n");
    return;
  }

  g_trace_log_auto_enabled = 0U;
  g_trace_log_auto_mode_flags = 0U;

  st = nvm_trace_log_get_header(&header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] auto: STOP (header NVM=%d)\r\n", (int)st);
    return;
  }

  trace_printf("[TRACE-LOG] auto: STOP total=%lu stored=%lu\r\n",
               (unsigned long)header.total_records,
               (unsigned long)((header.total_records > header.record_capacity) ? header.record_capacity : header.total_records));
}

static void nightfall_trace_log_set_mode_flags(uint16_t mode_flags)
{
  g_trace_log_auto_mode_flags = mode_flags;
}

static void nightfall_trace_log_auto_step(void)
{
  nvm_trace_log_record_t rec;
  nvm_status_t st;
  uint32_t now;

  if (g_trace_log_auto_enabled == 0U)
  {
    return;
  }

  now = HAL_GetTick();
  if ((int32_t)(now - g_trace_log_auto_next_tick_ms) < 0)
  {
    return;
  }

  nightfall_fill_trace_log_sample(&rec, g_trace_log_auto_seq);
  rec.flags |= (uint16_t)(NIGHTFALL_F413_TRACE_AUTO_FLAG | g_trace_log_auto_mode_flags);

  st = nvm_trace_log_append(&rec);
  if (st != NVM_STATUS_OK)
  {
    g_trace_log_auto_enabled = 0U;
    g_trace_log_auto_mode_flags = 0U;
    trace_printf("[TRACE-LOG] auto: FAIL(append NVM=%d)\r\n", (int)st);
    return;
  }

  g_trace_log_auto_seq += 1U;
  g_trace_log_auto_next_tick_ms = now + g_trace_log_auto_period_ms;
}

static void nightfall_trace_log_on_run_start(void)
{
  trace_printf("[TRACE-LOG] run-hook: start\r\n");
  nightfall_trace_log_auto_start();
}

static void nightfall_trace_log_on_run_stop(void)
{
  trace_printf("[TRACE-LOG] run-hook: stop\r\n");
  nightfall_trace_log_auto_stop();
}

static void nightfall_run_trace_log_dump_latest_once(void)
{
  nvm_trace_log_header_t header;
  nvm_status_t st;
  uint32_t available;
  uint32_t dump_count;
  uint32_t i;

  st = nvm_trace_log_get_header(&header);
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[TRACE-LOG] dump: FAIL(read header NVM=%d, run q first)\r\n", (int)st);
    return;
  }

  nightfall_print_trace_log_header(&header);

  available = header.total_records;
  if (available > header.record_capacity)
  {
    available = header.record_capacity;
  }
  if (available == 0U)
  {
    trace_printf("[TRACE-LOG] dump: no records\r\n");
    return;
  }

  dump_count = available;
  if (dump_count > NIGHTFALL_F413_TRACE_DUMP_MAX_RECORDS)
  {
    dump_count = NIGHTFALL_F413_TRACE_DUMP_MAX_RECORDS;
  }

  trace_printf("[TRACE-LOG] dump latest %lu/%lu\r\n",
               (unsigned long)dump_count,
               (unsigned long)available);

  for (i = 0U; i < dump_count; i++)
  {
    nvm_trace_log_record_t rec;

    st = nvm_trace_log_read_latest(i, &rec);
    if (st != NVM_STATUS_OK)
    {
      trace_printf("[TRACE-LOG] dump: FAIL(read idx=%lu NVM=%d)\r\n",
                   (unsigned long)i,
                   (int)st);
      return;
    }

    trace_printf("[TRACE-LOG] rec[%lu] seq=%lu ts=%lu mode=%u case=%u sub=%u test=%u dist=%ld angle=%ld v=(%ld/%ld) omega=(%ld/%ld) enc=(%d,%d) motor=(%d,%d) flags=0x%04X\r\n",
                 (unsigned long)i,
                 (unsigned long)rec.seq,
                 (unsigned long)rec.timestamp_ms,
                 (unsigned int)rec.op_mode,
                 (unsigned int)rec.op_case,
                 (unsigned int)rec.op_sub,
                 (unsigned int)rec.test_id,
                 (long)rec.distance_mm,
                 (long)rec.angle_mdeg,
                 (long)rec.target_velocity_mm_s,
                 (long)rec.real_velocity_mm_s,
                 (long)rec.target_omega_mdps,
                 (long)rec.real_omega_mdps,
                 (int)rec.encoder_l,
                 (int)rec.encoder_r,
                 (int)rec.motor_out_l,
                 (int)rec.motor_out_r,
                 (unsigned int)rec.flags);
  }
}

static void nightfall_set_all_leds(GPIO_PinState state)
{
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, state);
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, state);
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, state);
}

static void nightfall_op_led_show_mode(uint8_t mode)
{
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, (mode & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, (mode & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, (mode & 0x04U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void nightfall_buzzer_beep_ms(uint16_t period, uint16_t ms)
{
  if (HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1) != HAL_OK)
  {
    return;
  }

  __HAL_TIM_SET_AUTORELOAD(&htim11, period);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, (period * 6U) / 10U);
  HAL_Delay(ms);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0U);
  (void)HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
}

static void nightfall_op_beep_mode(uint8_t mode)
{
  uint16_t period = (uint16_t)((11U - (uint16_t)mode) * 400U);
  nightfall_buzzer_beep_ms(period, 200U);
}

static void nightfall_op_beep_enter(void)
{
  if (HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1) != HAL_OK)
  {
    return;
  }

  __HAL_TIM_SET_AUTORELOAD(&htim11, 900U);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 630U);
  HAL_Delay(100U);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0U);
  HAL_Delay(50U);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 630U);
  HAL_Delay(100U);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0U);
  (void)HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
}

static void nightfall_boot_buzzer_pattern(void)
{
  nightfall_buzzer_beep_ms(1800U, 70U);
  HAL_Delay(35U);
  nightfall_buzzer_beep_ms(1300U, 70U);
  HAL_Delay(35U);
  nightfall_buzzer_beep_ms(850U, 90U);
  HAL_Delay(120U);
  nightfall_buzzer_beep_ms(1450U, 70U);
  HAL_Delay(35U);
  nightfall_buzzer_beep_ms(1050U, 110U);
}

static void nightfall_run_led_test_once(void)
{
  trace_printf("[HW-TEST][LED] all on for %lu ms\r\n",
               (unsigned long)NIGHTFALL_F413_LED_ON_WINDOW_MS);

  nightfall_set_all_leds(GPIO_PIN_SET);
  HAL_Delay(NIGHTFALL_F413_LED_ON_WINDOW_MS);
  nightfall_set_all_leds(GPIO_PIN_RESET);

  trace_printf("[HW-TEST][LED] PASS(all LEDs were on)\r\n");
}

static bool nightfall_adc_prepare_single_conversion(void)
{
  (void)HAL_ADC_Stop(&hadc1);

  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    return false;
  }

  return true;
}

static bool nightfall_adc_read_single_channel(uint32_t channel, uint16_t* out)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  if (out == NULL)
  {
    return false;
  }

  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    return false;
  }

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return false;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 20U) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc1);
    return false;
  }
  *out = (uint16_t)HAL_ADC_GetValue(&hadc1);

  (void)HAL_ADC_Stop(&hadc1);
  return true;
}

static void nightfall_ir_emitters_set(GPIO_PinState fr,
                                      GPIO_PinState r,
                                      GPIO_PinState fl,
                                      GPIO_PinState l)
{
  HAL_GPIO_WritePin(IR_FR_GPIO_Port, IR_FR_Pin, fr);
  HAL_GPIO_WritePin(IR_R_GPIO_Port, IR_R_Pin, r);
  HAL_GPIO_WritePin(IR_FL_GPIO_Port, IR_FL_Pin, fl);
  HAL_GPIO_WritePin(IR_L_GPIO_Port, IR_L_Pin, l);
}

static bool nightfall_op_read_front_sensor_delta(int32_t* fr_delta, int32_t* fl_delta)
{
  uint16_t fr_off = 0U;
  uint16_t fl_off = 0U;
  uint16_t fr_on = 0U;
  uint16_t fl_on = 0U;
  bool ok = false;

  if ((fr_delta == NULL) || (fl_delta == NULL))
  {
    return false;
  }

  *fr_delta = 0;
  *fl_delta = 0;

  if (!nightfall_adc_prepare_single_conversion())
  {
    return false;
  }

  nightfall_ir_emitters_set(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  HAL_Delay(1U);
  if (!nightfall_adc_read_single_channel(ADC_CHANNEL_0, &fr_off) ||
      !nightfall_adc_read_single_channel(ADC_CHANNEL_2, &fl_off))
  {
    goto op_sensor_cleanup;
  }

  nightfall_ir_emitters_set(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
  HAL_Delay(1U);
  if (!nightfall_adc_read_single_channel(ADC_CHANNEL_0, &fr_on) ||
      !nightfall_adc_read_single_channel(ADC_CHANNEL_2, &fl_on))
  {
    goto op_sensor_cleanup;
  }

  *fr_delta = (int32_t)fr_on - (int32_t)fr_off;
  *fl_delta = (int32_t)fl_on - (int32_t)fl_off;
  ok = true;

op_sensor_cleanup:
  nightfall_ir_emitters_set(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  MX_ADC1_Init();
  return ok;
}

static bool nightfall_op_enter_sensor_active(void)
{
  int32_t fr_delta = 0;
  int32_t fl_delta = 0;

  if (!nightfall_op_read_front_sensor_delta(&fr_delta, &fl_delta))
  {
    return false;
  }

  return (fr_delta >= NIGHTFALL_F413_OP_ENTER_DELTA_ADC) &&
         (fl_delta <= NIGHTFALL_F413_OP_ENTER_RELEASE_ADC);
}

static void nightfall_run_wall_sensor_test_once(void)
{
  uint16_t r_off = 0U;
  uint16_t l_off = 0U;
  uint16_t fr_off = 0U;
  uint16_t fl_off = 0U;
  uint16_t vb_off = 0U;
  uint16_t r_on = 0U;
  uint16_t l_on = 0U;
  uint16_t fr_on = 0U;
  uint16_t fl_on = 0U;
  uint16_t vb_on = 0U;
  bool ok = false;

  if (!nightfall_adc_prepare_single_conversion())
  {
    trace_printf("[HW-TEST][Wall] FAIL(adc prepare)\r\n");
    return;
  }

  nightfall_ir_emitters_set(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  HAL_Delay(2U);
  if (!nightfall_adc_read_single_channel(ADC_CHANNEL_1, &r_off) ||
      !nightfall_adc_read_single_channel(ADC_CHANNEL_3, &l_off) ||
      !nightfall_adc_read_single_channel(ADC_CHANNEL_0, &fr_off) ||
      !nightfall_adc_read_single_channel(ADC_CHANNEL_2, &fl_off) ||
      !nightfall_adc_read_single_channel(ADC_CHANNEL_8, &vb_off))
  {
    trace_printf("[HW-TEST][Wall] FAIL(read off)\r\n");
    goto wall_test_cleanup;
  }

  nightfall_ir_emitters_set(GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET);
  HAL_Delay(2U);
  if (!nightfall_adc_read_single_channel(ADC_CHANNEL_1, &r_on) ||
      !nightfall_adc_read_single_channel(ADC_CHANNEL_3, &l_on) ||
      !nightfall_adc_read_single_channel(ADC_CHANNEL_0, &fr_on) ||
      !nightfall_adc_read_single_channel(ADC_CHANNEL_2, &fl_on) ||
      !nightfall_adc_read_single_channel(ADC_CHANNEL_8, &vb_on))
  {
    trace_printf("[HW-TEST][Wall] FAIL(read on)\r\n");
    goto wall_test_cleanup;
  }

  ok = true;

  trace_printf("[HW-TEST][Wall] off: R=%u L=%u FR=%u FL=%u VB=%u\r\n",
               (unsigned int)r_off,
               (unsigned int)l_off,
               (unsigned int)fr_off,
               (unsigned int)fl_off,
               (unsigned int)vb_off);
  trace_printf("[HW-TEST][Wall] on : R=%u L=%u FR=%u FL=%u VB=%u\r\n",
               (unsigned int)r_on,
               (unsigned int)l_on,
               (unsigned int)fr_on,
               (unsigned int)fl_on,
               (unsigned int)vb_on);

wall_test_cleanup:
  nightfall_ir_emitters_set(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET);
  MX_ADC1_Init();

  if (ok)
  {
    trace_printf("[HW-TEST][Wall] PASS(measure done)\r\n");
  }
}

static void nightfall_run_switch_test_once(void)
{
  GPIO_PinState raw = HAL_GPIO_ReadPin(PUSH_IN_1_GPIO_Port, PUSH_IN_1_Pin);
  trace_printf("[HW-TEST][Switch] raw=%u (%s)\r\n",
               (unsigned int)raw,
               (raw == GPIO_PIN_RESET) ? "pressed or low" : "released or high");
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
  if (HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1) != HAL_OK)
  {
    trace_printf("[HW-TEST][Buzzer] FAIL(start pwm)\r\n");
    return;
  }

  __HAL_TIM_SET_AUTORELOAD(&htim11, 1000U);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 500U);
  HAL_Delay(150U);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0U);
  (void)HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);

  trace_printf("[HW-TEST][Buzzer] PASS(beep)\r\n");
}

static const char* nightfall_op_mode_name(uint8_t mode)
{
  switch (mode)
  {
    case 0U: return "idle";
    case 1U: return "search";
    case 2U: return "shortest-mode2";
    case 3U: return "shortest-mode3";
    case 4U: return "shortest-mode4";
    case 5U: return "shortest-mode5";
    case 6U: return "shortest-mode6";
    case 7U: return "shortest-mode7";
    case 8U: return "test-run";
    case 9U: return "test-mode";
    default: return "unknown";
  }
}

static const char* nightfall_op_level_name(uint8_t level)
{
  switch (level)
  {
    case NIGHTFALL_F413_OP_LEVEL_TOP: return "mode";
    case NIGHTFALL_F413_OP_LEVEL_CASE: return "case";
    case NIGHTFALL_F413_OP_LEVEL_SUB: return "sub";
    default: return "unknown";
  }
}

static const char* nightfall_op_case_name(uint8_t mode, uint8_t op_case)
{
  if (mode == 1U)
  {
    switch (op_case)
    {
      case 0U: return "search test tune";
      case 1U: return "standard goal-full search";
      case 2U: return "standard full search";
      case 3U: return "standard goal-return search";
      case 4U: return "standard goal-only search";
      case 5U: return "low goal-full search";
      case 6U: return "low full search";
      case 7U: return "low goal-return search";
      case 8U: return "low goal-only search";
      default: return "not assigned";
    }
  }
  if ((mode >= 2U) && (mode <= 7U))
  {
    if (op_case == 0U)
    {
      return "turn/diagonal/straight test";
    }
    return "shortest case";
  }
  if (mode == 8U)
  {
    switch (op_case)
    {
      case 0U: return "set position";
      case 1U: return "get log short straight";
      case 2U: return "get log long straight";
      case 3U: return "right 90 turn log";
      case 4U: return "straight plus turn";
      case 5U: return "large right 90";
      case 6U: return "rotate right 90";
      case 7U: return "large turn check";
      case 8U: return "front match position";
      case 9U: return "print log";
      default: return "unknown";
    }
  }
  if (mode == 9U)
  {
    switch (op_case)
    {
      case 0U: return "inner loop tune";
      case 1U: return "IMU check";
      case 2U: return "encoder check";
      case 3U: return "sensor AD check";
      case 4U: return "fan noise check";
      case 5U: return "dump latest full log";
      case 6U: return "wall threshold check";
      case 7U: return "NVM check";
      case 8U: return "identity check";
      case 9U: return "sensor parameter save";
      default: return "unknown";
    }
  }
  return "unknown";
}

static const char* nightfall_op_sub_name(uint8_t mode, uint8_t sub)
{
  if (mode == 1U)
  {
    switch (sub)
    {
      case 0U: return "reserved";
      case 1U: return "standard speed turn test";
      case 2U: return "low speed turn test";
      case 3U: return "straight 3-section test";
      default: return "not assigned";
    }
  }
  if ((mode >= 2U) && (mode <= 7U))
  {
    switch (sub)
    {
      case 0U: return "normal R90 turn";
      case 1U: return "large 90 turn";
      case 2U: return "large 180 turn";
      case 3U: return "diagonal 45-in";
      case 4U: return "diagonal 45-out";
      case 5U: return "diagonal V90";
      case 6U: return "diagonal 135-in";
      case 7U: return "diagonal 135-out";
      case 8U: return "straight slow";
      case 9U: return "straight fast";
      default: return "unknown";
    }
  }
  return "unknown";
}

static uint8_t nightfall_op_selected_value(void)
{
  if (g_op_level == NIGHTFALL_F413_OP_LEVEL_CASE)
  {
    return g_op_case;
  }
  if (g_op_level == NIGHTFALL_F413_OP_LEVEL_SUB)
  {
    return g_op_sub;
  }
  return g_op_mode;
}

static void nightfall_op_set_selected_value(uint8_t value)
{
  if (g_op_level == NIGHTFALL_F413_OP_LEVEL_CASE)
  {
    g_op_case = value;
  }
  else if (g_op_level == NIGHTFALL_F413_OP_LEVEL_SUB)
  {
    g_op_sub = value;
  }
  else
  {
    g_op_mode = value;
  }
}

static void nightfall_op_print_selection(void)
{
  if (g_op_level == NIGHTFALL_F413_OP_LEVEL_CASE)
  {
    trace_printf("[OP-UI] mode=%u %s case=%u %s\r\n",
                 (unsigned int)g_op_mode,
                 nightfall_op_mode_name(g_op_mode),
                 (unsigned int)g_op_case,
                 nightfall_op_case_name(g_op_mode, g_op_case));
  }
  else if (g_op_level == NIGHTFALL_F413_OP_LEVEL_SUB)
  {
    trace_printf("[OP-UI] mode=%u %s case=0 sub=%u %s\r\n",
                 (unsigned int)g_op_mode,
                 nightfall_op_mode_name(g_op_mode),
                 (unsigned int)g_op_sub,
                 nightfall_op_sub_name(g_op_mode, g_op_sub));
  }
  else
  {
    trace_printf("[OP-UI] mode=%u %s\r\n",
                 (unsigned int)g_op_mode,
                 nightfall_op_mode_name(g_op_mode));
  }
}

static bool nightfall_op_can_accept_input(void)
{
  if (g_trace_log_auto_enabled != 0U || f413_ctrl_is_running())
  {
    return false;
  }
  if ((g_test_armed_id >= '1') && (g_test_armed_id <= '5'))
  {
    return false;
  }
  return true;
}

static void nightfall_op_increment_selection(void)
{
  uint8_t selected = nightfall_op_selected_value();
  selected++;
  if (selected > NIGHTFALL_F413_OP_SELECT_MAX)
  {
    selected = 0U;
  }
  nightfall_op_set_selected_value(selected);
  g_op_button_lock_until_ms = HAL_GetTick() + NIGHTFALL_F413_OP_BUTTON_LOCK_MS;
  g_op_enter_streak = 0U;
  nightfall_op_led_show_mode(selected);
  nightfall_op_beep_mode(selected);
  nightfall_op_print_selection();
}

static void nightfall_op_after_execute(void)
{
  g_op_enter_streak = 0U;
  g_op_button_lock_until_ms = HAL_GetTick() + NIGHTFALL_F413_OP_BUTTON_LOCK_MS;
  nightfall_op_led_show_mode(nightfall_op_selected_value());
}

static void nightfall_op_run_test_after_delay(uint8_t test_id)
{
  g_test_armed_id = NIGHTFALL_F413_TEST_ARMED_NONE;
  nightfall_trace_log_set_context(8U, g_op_case, 0xFFU, test_id);
  HAL_Delay(NIGHTFALL_F413_OP_START_DELAY_MS);
  nightfall_test_run(test_id);
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

static void nightfall_op_execute_case(uint8_t mode, uint8_t op_case)
{
  trace_printf("[OP-UI] execute mode=%u %s case=%u %s\r\n",
               (unsigned int)mode,
               nightfall_op_mode_name(mode),
               (unsigned int)op_case,
               nightfall_op_case_name(mode, op_case));

  switch (mode)
  {
    case 1U:
      if (op_case == 4U)
      {
        nightfall_trace_log_set_context(mode, op_case, 0xFFU, 0U);
        HAL_Delay(NIGHTFALL_F413_OP_START_DELAY_MS);
        nightfall_run_search_trace_entry_once();
      }
      else
      {
        trace_printf("[OP-UI] no-op: F405 mode1 case%u is not fully ported on F413 yet\r\n",
                     (unsigned int)op_case);
      }
      break;
    case 2U:
      if (op_case == 1U)
      {
        nightfall_trace_log_set_context(mode, op_case, 0xFFU, 0U);
        HAL_Delay(NIGHTFALL_F413_OP_START_DELAY_MS);
        nightfall_run_shortest_trace_entry_once();
      }
      else
      {
        trace_printf("[OP-UI] no-op: F413 shortest runner is currently wired to mode2 case1 only\r\n");
      }
      break;
    case 8U:
      if (op_case == 1U)
      {
        nightfall_op_run_test_after_delay('1');
      }
      else if (op_case == 2U)
      {
        nightfall_op_run_test_after_delay('2');
      }
      else if (op_case == 3U)
      {
        nightfall_op_run_test_after_delay('3');
      }
      else if (op_case == 4U)
      {
        nightfall_op_run_test_after_delay('5');
      }
      else
      {
        trace_printf("[OP-UI] no-op: F405 test_run case%u is not fully ported on F413 yet\r\n",
                     (unsigned int)op_case);
      }
      break;
    case 9U:
      if (op_case == 1U)
      {
        nightfall_run_imu_test_once();
      }
      else if (op_case == 2U)
      {
        nightfall_run_encoder_test_once();
      }
      else if (op_case == 3U)
      {
        nightfall_run_wall_sensor_test_once();
      }
      else if (op_case == 4U)
      {
        nightfall_run_fan_pwm_test_once();
      }
      else if (op_case == 5U)
      {
        nightfall_run_trace_log_dump_csv_all_once();
      }
      else
      {
        trace_printf("[OP-UI] no-op: F405 test_mode case%u is not fully ported on F413 yet\r\n",
                     (unsigned int)op_case);
      }
      break;
    default:
      trace_printf("[OP-UI] no-op: F405 mode%u case%u is not fully ported on F413 yet\r\n",
                   (unsigned int)mode,
                   (unsigned int)op_case);
      break;
  }

  nightfall_op_after_execute();
}

static void nightfall_op_execute_sub(uint8_t mode, uint8_t sub)
{
  trace_printf("[OP-UI] execute mode=%u %s case=0 sub=%u %s\r\n",
               (unsigned int)mode,
               nightfall_op_mode_name(mode),
               (unsigned int)sub,
               nightfall_op_sub_name(mode, sub));

  if ((mode >= 2U) && (mode <= 7U))
  {
    nightfall_op_run_case0_sub_after_delay(mode, sub);
  }
  else if (mode == 1U)
  {
    if (sub == 3U)
    {
      nightfall_op_run_test_after_delay('1');
    }
    else
    {
      trace_printf("[OP-UI] no-op: F405 mode1 case0 sub%u is not fully ported on F413 yet\r\n",
                   (unsigned int)sub);
    }
  }
  else
  {
    trace_printf("[OP-UI] no-op: unsupported sub selection\r\n");
  }

  g_op_level = NIGHTFALL_F413_OP_LEVEL_CASE;
  nightfall_op_after_execute();
}

static void nightfall_op_enter_selection(void)
{
  uint8_t selected = nightfall_op_selected_value();

  nightfall_op_beep_enter();
  nightfall_set_all_leds(GPIO_PIN_SET);
  HAL_Delay(120U);

  if (g_op_level == NIGHTFALL_F413_OP_LEVEL_TOP)
  {
    if (g_op_mode == 0U)
    {
      trace_printf("[OP-UI] execute mode=0 idle\r\n");
      nightfall_op_after_execute();
      return;
    }

    g_op_level = NIGHTFALL_F413_OP_LEVEL_CASE;
    g_op_case = 0U;
    trace_printf("[OP-UI] enter mode=%u %s; select case 0..9\r\n",
                 (unsigned int)g_op_mode,
                 nightfall_op_mode_name(g_op_mode));
    nightfall_op_print_selection();
    nightfall_op_after_execute();
    return;
  }

  if (g_op_level == NIGHTFALL_F413_OP_LEVEL_CASE)
  {
    if ((g_op_case == 0U) && (g_op_mode >= 1U) && (g_op_mode <= 7U))
    {
      g_op_level = NIGHTFALL_F413_OP_LEVEL_SUB;
      g_op_sub = 0U;
      trace_printf("[OP-UI] enter mode=%u case=0; select sub 0..9\r\n",
                   (unsigned int)g_op_mode);
      nightfall_op_print_selection();
      nightfall_op_after_execute();
      return;
    }
    nightfall_op_execute_case(g_op_mode, selected);
    return;
  }

  nightfall_op_execute_sub(g_op_mode, selected);
}

static void nightfall_op_ui_step(void)
{
  uint32_t now = HAL_GetTick();
  bool button_pressed;
  bool enter_active;

  if ((int32_t)(now - g_op_next_ui_poll_ms) < 0)
  {
    return;
  }
  g_op_next_ui_poll_ms = now + NIGHTFALL_F413_OP_UI_POLL_MS;

  if (!nightfall_op_can_accept_input())
  {
    return;
  }

  button_pressed = nightfall_run_stop_switch_pressed();
  if (button_pressed)
  {
    if (!g_op_button_prev_pressed && ((int32_t)(now - g_op_button_lock_until_ms) >= 0))
    {
      nightfall_op_increment_selection();
    }
    g_op_button_prev_pressed = true;
    return;
  }
  g_op_button_prev_pressed = false;

  enter_active = nightfall_op_enter_sensor_active();
  if (enter_active)
  {
    if (!g_op_enter_latched)
    {
      if (g_op_enter_streak < NIGHTFALL_F413_OP_ENTER_STREAK)
      {
        g_op_enter_streak++;
      }
      if (g_op_enter_streak >= NIGHTFALL_F413_OP_ENTER_STREAK)
      {
        g_op_enter_latched = true;
        nightfall_op_enter_selection();
      }
    }
  }
  else
  {
    g_op_enter_streak = 0U;
    g_op_enter_latched = false;
  }
}

static void nightfall_op_uart_push_once(void)
{
  if (!nightfall_op_can_accept_input())
  {
    trace_printf("[OP-UART] ignored: operation UI is busy\r\n");
    return;
  }
  g_op_button_prev_pressed = false;
  nightfall_op_increment_selection();
}

static void nightfall_op_uart_enter_once(void)
{
  if (!nightfall_op_can_accept_input())
  {
    trace_printf("[OP-UART] ignored: operation UI is busy\r\n");
    return;
  }
  g_op_enter_streak = 0U;
  g_op_enter_latched = true;
  nightfall_op_enter_selection();
}

static void nightfall_motor_set(bool enable,
                                bool left_forward,
                                bool right_forward,
                                uint16_t left_duty,
                                uint16_t right_duty)
{
  uint16_t l_duty = (left_duty > NIGHTFALL_F413_MOTOR_PWM_MAX) ? NIGHTFALL_F413_MOTOR_PWM_MAX : left_duty;
  uint16_t r_duty = (right_duty > NIGHTFALL_F413_MOTOR_PWM_MAX) ? NIGHTFALL_F413_MOTOR_PWM_MAX : right_duty;
  uint16_t l_compare = 0U;
  uint16_t r_compare = 0U;
  GPIO_PinState l_in2 = GPIO_PIN_RESET;
  GPIO_PinState r_in2 = GPIO_PIN_RESET;

  if (!enable)
  {
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
    (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    return;
  }

  if (l_duty != 0U)
  {
    if (left_forward)
    {
      l_compare = l_duty;
      l_in2 = GPIO_PIN_RESET;
    }
    else
    {
      l_compare = (uint16_t)(NIGHTFALL_F413_MOTOR_PWM_MAX - l_duty);
      l_in2 = GPIO_PIN_SET;
    }
  }

  if (r_duty != 0U)
  {
    if (right_forward)
    {
      r_compare = (uint16_t)(NIGHTFALL_F413_MOTOR_PWM_MAX - r_duty);
      r_in2 = GPIO_PIN_SET;
    }
    else
    {
      r_compare = r_duty;
      r_in2 = GPIO_PIN_RESET;
    }
  }

  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  if (l_in2 == GPIO_PIN_SET)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, l_compare);
    HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, l_compare);
  }

  if (r_in2 == GPIO_PIN_SET)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, r_compare);
    HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, r_compare);
  }

  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
}

static void nightfall_run_fan_pwm_test_once(void)
{
  const uint16_t duties[3] = {200U, 500U, 800U};
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim10);
  uint8_t i;

  if (HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) != HAL_OK)
  {
    trace_printf("[HW-TEST][Fan] PWM start failed\r\n");
    return;
  }

  trace_printf("[HW-TEST][Fan] start ARR=%lu\r\n", (unsigned long)arr);
  for (i = 0U; i < 3U; i++)
  {
    uint32_t compare = ((uint32_t)duties[i] * arr) / 1000U;
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, compare);
    trace_printf("[HW-TEST][Fan] duty=%u/1000 compare=%lu\r\n",
                 (unsigned int)duties[i],
                 (unsigned long)compare);
    HAL_Delay(1200U);
  }
  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0U);
  (void)HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
  trace_printf("[HW-TEST][Fan] done\r\n");
}

static void nightfall_run_motor_driver_test_once(void)
{
  trace_printf("[HW-TEST][Motor] start short drive (lift robot before test)\r\n");

  nightfall_motor_set(true, true, true, 120U, 120U);
  HAL_Delay(300U);
  nightfall_motor_set(false, true, true, 0U, 0U);
  HAL_Delay(120U);

  nightfall_motor_set(true, false, false, 120U, 120U);
  HAL_Delay(300U);
  nightfall_motor_set(false, false, false, 0U, 0U);

  trace_printf("[HW-TEST][Motor] PASS(pulse done)\r\n");
}

static void nightfall_run_encoder_test_once(void)
{
  uint32_t l0;
  uint32_t r0;
  uint32_t l1;
  uint32_t r1;
  int32_t dl;
  int32_t dr;

  (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);

  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    trace_printf("[HW-TEST][Encoder] FAIL(start L)\r\n");
    return;
  }
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
  {
    trace_printf("[HW-TEST][Encoder] FAIL(start R)\r\n");
    return;
  }

  l0 = __HAL_TIM_GET_COUNTER(&htim3);
  r0 = __HAL_TIM_GET_COUNTER(&htim4);
  trace_printf("[HW-TEST][Encoder] measuring %lu ms (rotate wheels now)\r\n",
               (unsigned long)NIGHTFALL_F413_ENCODER_WINDOW_MS);
  HAL_Delay(NIGHTFALL_F413_ENCODER_WINDOW_MS);
  l1 = __HAL_TIM_GET_COUNTER(&htim3);
  r1 = __HAL_TIM_GET_COUNTER(&htim4);
  dl = NIGHTFALL_F413_ENCODER_SIGN_L * nightfall_encoder_delta_signed(l1, l0);
  dr = NIGHTFALL_F413_ENCODER_SIGN_R * nightfall_encoder_delta_signed(r1, r0);

  (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);

  trace_printf("[HW-TEST][Encoder] L:%lu->%lu d=%ld, R:%lu->%lu d=%ld\r\n",
               (unsigned long)l0,
               (unsigned long)l1,
               (long)dl,
               (unsigned long)r0,
               (unsigned long)r1,
               (long)dr);
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

static void nightfall_trace_log_wait_with_auto_step(uint32_t duration_ms)
{
  uint32_t deadline = HAL_GetTick() + duration_ms;

  while ((int32_t)(HAL_GetTick() - deadline) < 0)
  {
    nightfall_trace_log_auto_step();
    HAL_Delay(1U);
  }
  nightfall_trace_log_auto_step();
}

static bool nightfall_run_stop_switch_pressed(void)
{
  return (HAL_GPIO_ReadPin(PUSH_IN_1_GPIO_Port, PUSH_IN_1_Pin) == GPIO_PIN_RESET);
}

static int32_t nightfall_abs_i32(int32_t v)
{
  return (v < 0) ? -v : v;
}

static int32_t nightfall_encoder_delta_signed(uint32_t now, uint32_t prev)
{
  int32_t delta = (int32_t)now - (int32_t)prev;
  if (delta > NIGHTFALL_F413_ENCODER_WRAP_HALF)
  {
    delta -= NIGHTFALL_F413_ENCODER_WRAP_COUNT;
  }
  else if (delta < -NIGHTFALL_F413_ENCODER_WRAP_HALF)
  {
    delta += NIGHTFALL_F413_ENCODER_WRAP_COUNT;
  }
  return delta;
}

static uint16_t nightfall_run_abort_reason_to_trace_flag(nightfall_run_abort_reason_t reason)
{
  switch (reason)
  {
    case NIGHTFALL_RUN_ABORT_SWITCH:
      return NIGHTFALL_F413_TRACE_ABORT_SWITCH_FLAG;
    case NIGHTFALL_RUN_ABORT_WALL_FAULT:
      return NIGHTFALL_F413_TRACE_ABORT_WALL_FAULT_FLAG;
    case NIGHTFALL_RUN_ABORT_ENCODER_FAULT:
      return NIGHTFALL_F413_TRACE_ABORT_ENCODER_FAULT_FLAG;
    case NIGHTFALL_RUN_ABORT_IMU_FAULT:
      return NIGHTFALL_F413_TRACE_ABORT_IMU_FAULT_FLAG;
    case NIGHTFALL_RUN_ABORT_NONE:
    default:
      return 0U;
  }
}

static const char* nightfall_run_abort_reason_to_text(nightfall_run_abort_reason_t reason)
{
  switch (reason)
  {
    case NIGHTFALL_RUN_ABORT_SWITCH:
      return "switch pressed";
    case NIGHTFALL_RUN_ABORT_WALL_FAULT:
      return "wall sensor fault";
    case NIGHTFALL_RUN_ABORT_ENCODER_FAULT:
      return "encoder jump fault";
    case NIGHTFALL_RUN_ABORT_IMU_FAULT:
      return "imu fault";
    case NIGHTFALL_RUN_ABORT_NONE:
    default:
      return "none";
  }
}

static bool nightfall_run_guard_prepare(nightfall_run_guard_t* guard)
{
  if (guard == NULL)
  {
    return false;
  }

  memset(guard, 0, sizeof(*guard));
  if (!nightfall_adc_prepare_single_conversion())
  {
    return false;
  }

  guard->adc_prepared = 1U;
  guard->prev_encoder_l = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  guard->prev_encoder_r = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  guard->next_wall_check_ms = HAL_GetTick();
  guard->next_imu_check_ms = HAL_GetTick();
  return true;
}

static void nightfall_run_guard_cleanup(nightfall_run_guard_t* guard)
{
  if ((guard != NULL) && (guard->adc_prepared != 0U))
  {
    MX_ADC1_Init();
    guard->adc_prepared = 0U;
  }
}

static nightfall_run_abort_reason_t nightfall_run_guard_check(nightfall_run_guard_t* guard)
{
  int16_t enc_l_now;
  int16_t enc_r_now;
  int32_t d_l;
  int32_t d_r;
  uint32_t now;

  if (guard == NULL)
  {
    return NIGHTFALL_RUN_ABORT_IMU_FAULT;
  }

  if (nightfall_run_stop_switch_pressed())
  {
    return NIGHTFALL_RUN_ABORT_SWITCH;
  }

  enc_l_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  enc_r_now = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
  d_l = (int32_t)enc_l_now - (int32_t)guard->prev_encoder_l;
  d_r = (int32_t)enc_r_now - (int32_t)guard->prev_encoder_r;
  guard->prev_encoder_l = enc_l_now;
  guard->prev_encoder_r = enc_r_now;

  if ((nightfall_abs_i32(d_l) > NIGHTFALL_F413_RUN_GUARD_ENCODER_DELTA_MAX) ||
      (nightfall_abs_i32(d_r) > NIGHTFALL_F413_RUN_GUARD_ENCODER_DELTA_MAX))
  {
    return NIGHTFALL_RUN_ABORT_ENCODER_FAULT;
  }

  now = HAL_GetTick();
  if ((int32_t)(now - guard->next_wall_check_ms) >= 0)
  {
    uint16_t ad_fr = 0U;
    uint16_t ad_fl = 0U;

    guard->next_wall_check_ms = now + NIGHTFALL_F413_RUN_GUARD_WALL_CHECK_MS;
    if (!nightfall_adc_read_single_channel(ADC_CHANNEL_0, &ad_fr) ||
        !nightfall_adc_read_single_channel(ADC_CHANNEL_2, &ad_fl))
    {
      return NIGHTFALL_RUN_ABORT_WALL_FAULT;
    }
    if ((ad_fr >= NIGHTFALL_F413_RUN_GUARD_WALL_SAT_ADC) ||
        (ad_fl >= NIGHTFALL_F413_RUN_GUARD_WALL_SAT_ADC))
    {
      return NIGHTFALL_RUN_ABORT_WALL_FAULT;
    }
  }

  if ((int32_t)(now - guard->next_imu_check_ms) >= 0)
  {
    uint8_t who = 0U;

    guard->next_imu_check_ms = now + NIGHTFALL_F413_RUN_GUARD_IMU_CHECK_MS;
    if (!nightfall_imu_read_reg(NIGHTFALL_F413_IMU_WHO_AM_I_REG, &who) ||
        (who != NIGHTFALL_F413_IMU_WHO_AM_I_EXPECTED))
    {
      return NIGHTFALL_RUN_ABORT_IMU_FAULT;
    }
  }

  return NIGHTFALL_RUN_ABORT_NONE;
}

static nightfall_run_abort_reason_t nightfall_trace_log_wait_with_auto_step_guarded(uint32_t duration_ms,
                                                                                     nightfall_run_guard_t* guard)
{
  uint32_t deadline = HAL_GetTick() + duration_ms;

  while ((int32_t)(HAL_GetTick() - deadline) < 0)
  {
    nightfall_run_abort_reason_t reason = nightfall_run_guard_check(guard);
    if (reason != NIGHTFALL_RUN_ABORT_NONE)
    {
      return reason;
    }
    nightfall_trace_log_auto_step();
    HAL_Delay(1U);
  }

  nightfall_trace_log_auto_step();
  return nightfall_run_guard_check(guard);
}

static void nightfall_run_idle_trace_session_once(void)
{
  if (g_trace_log_auto_enabled != 0U)
  {
    trace_printf("[RUN-TEST] busy(auto already running)\r\n");
    return;
  }

  trace_printf("[RUN-TEST] idle trace session start %lu ms\r\n",
               (unsigned long)NIGHTFALL_F413_RUN_SESSION_IDLE_MS);

  nightfall_trace_log_on_run_start();
  nightfall_trace_log_set_mode_flags(NIGHTFALL_F413_TRACE_MODE_IDLE_FLAG);
  nightfall_trace_log_wait_with_auto_step(NIGHTFALL_F413_RUN_SESSION_IDLE_MS);
  nightfall_trace_log_set_mode_flags(0U);
  nightfall_trace_log_on_run_stop();

  trace_printf("[RUN-TEST] idle trace session end\r\n");
}

static void nightfall_run_motor_trace_session_once(void)
{
  bool enc_started_l = false;
  bool enc_started_r = false;

  if (g_trace_log_auto_enabled != 0U)
  {
    trace_printf("[RUN-TEST] busy(auto already running)\r\n");
    return;
  }

  trace_printf("[RUN-TEST] motor trace session start (lift robot before test)\r\n");

  (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);

  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) == HAL_OK)
  {
    enc_started_l = true;
  }
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) == HAL_OK)
  {
    enc_started_r = true;
  }

  nightfall_trace_log_on_run_start();
  nightfall_trace_log_set_mode_flags(NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG);
  nightfall_motor_set(true,
                     true,
                     true,
                     NIGHTFALL_F413_RUN_SESSION_MOTOR_DUTY,
                     NIGHTFALL_F413_RUN_SESSION_MOTOR_DUTY);
  nightfall_trace_log_wait_with_auto_step(NIGHTFALL_F413_RUN_SESSION_MOTOR_PULSE_MS);

  nightfall_trace_log_set_mode_flags(NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG);
  nightfall_motor_set(false, true, true, 0U, 0U);
  nightfall_trace_log_wait_with_auto_step(NIGHTFALL_F413_RUN_SESSION_MOTOR_COAST_MS);

  nightfall_trace_log_set_mode_flags(NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG);
  nightfall_motor_set(true,
                     false,
                     false,
                     NIGHTFALL_F413_RUN_SESSION_MOTOR_DUTY,
                     NIGHTFALL_F413_RUN_SESSION_MOTOR_DUTY);
  nightfall_trace_log_wait_with_auto_step(NIGHTFALL_F413_RUN_SESSION_MOTOR_PULSE_MS);

  nightfall_trace_log_set_mode_flags(NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG);
  nightfall_motor_set(false, false, false, 0U, 0U);
  nightfall_trace_log_wait_with_auto_step(NIGHTFALL_F413_RUN_SESSION_MOTOR_COAST_MS);

  nightfall_trace_log_set_mode_flags(0U);
  nightfall_trace_log_on_run_stop();

  if (enc_started_l)
  {
    (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
  }
  if (enc_started_r)
  {
    (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
  }

  trace_printf("[RUN-TEST] motor trace session end\r\n");
}

static void nightfall_run_search_safe_trace_session_once(void)
{
  bool enc_started_l = false;
  bool enc_started_r = false;
  nightfall_run_abort_reason_t abort_reason = NIGHTFALL_RUN_ABORT_NONE;
  nightfall_run_guard_t guard = {0};
  uint32_t step;

  if (g_trace_log_auto_enabled != 0U)
  {
    trace_printf("[RUN-TEST] busy(auto already running)\r\n");
    return;
  }

  if (nightfall_run_stop_switch_pressed())
  {
    trace_printf("[RUN-TEST] search-safe canceled(start switch pressed)\r\n");
    return;
  }

  trace_printf("[RUN-TEST] search-safe start (low speed, press switch to abort)\r\n");

  (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);

  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) == HAL_OK)
  {
    enc_started_l = true;
  }
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) == HAL_OK)
  {
    enc_started_r = true;
  }

  if (!nightfall_run_guard_prepare(&guard))
  {
    if (enc_started_l)
    {
      (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
    }
    if (enc_started_r)
    {
      (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
    }
    trace_printf("[RUN-TEST] search-safe canceled(guard init fail)\r\n");
    return;
  }

  nightfall_trace_log_on_run_start();

  for (step = 0U; step < NIGHTFALL_F413_RUN_SESSION_SAFE_EXPLORE_STEPS; step++)
  {
    nightfall_trace_log_set_mode_flags(NIGHTFALL_F413_TRACE_MODE_SEARCH_SAFE_FLAG |
                                       NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG);
    nightfall_motor_set(true,
                       true,
                       true,
                       NIGHTFALL_F413_RUN_SESSION_SAFE_DUTY,
                       NIGHTFALL_F413_RUN_SESSION_SAFE_DUTY);
    abort_reason = nightfall_trace_log_wait_with_auto_step_guarded(NIGHTFALL_F413_RUN_SESSION_SAFE_FORWARD_MS,
                                                                   &guard);
    if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
    {
      break;
    }

    nightfall_trace_log_set_mode_flags(NIGHTFALL_F413_TRACE_MODE_SEARCH_SAFE_FLAG |
                                       NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG);
    nightfall_motor_set(false, true, true, 0U, 0U);
    abort_reason = nightfall_trace_log_wait_with_auto_step_guarded(NIGHTFALL_F413_RUN_SESSION_SAFE_COAST_MS,
                                                                   &guard);
    if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
    {
      break;
    }
  }

  nightfall_motor_set(false, true, true, 0U, 0U);
  if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
  {
    nightfall_trace_log_set_mode_flags((uint16_t)(NIGHTFALL_F413_TRACE_MODE_SEARCH_SAFE_FLAG |
                                                  nightfall_run_abort_reason_to_trace_flag(abort_reason)));
    nightfall_trace_log_auto_step();
  }
  nightfall_trace_log_set_mode_flags(0U);
  nightfall_trace_log_on_run_stop();

  if (enc_started_l)
  {
    (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
  }
  if (enc_started_r)
  {
    (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
  }

  nightfall_run_guard_cleanup(&guard);

  if (abort_reason == NIGHTFALL_RUN_ABORT_SWITCH)
  {
    trace_printf("[RUN-TEST] search-safe aborted by switch\r\n");
  }
  else if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
  {
    trace_printf("[RUN-TEST] search-safe aborted(%s)\r\n",
                 nightfall_run_abort_reason_to_text(abort_reason));
  }
  else
  {
    trace_printf("[RUN-TEST] search-safe end\r\n");
  }
}

static void nightfall_run_shortest_safe_trace_session_once(void)
{
  bool enc_started_l = false;
  bool enc_started_r = false;
  nightfall_run_abort_reason_t abort_reason = NIGHTFALL_RUN_ABORT_NONE;
  nightfall_run_guard_t guard = {0};

  if (g_trace_log_auto_enabled != 0U)
  {
    trace_printf("[RUN-TEST] busy(auto already running)\r\n");
    return;
  }

  if (nightfall_run_stop_switch_pressed())
  {
    trace_printf("[RUN-TEST] shortest-safe canceled(start switch pressed)\r\n");
    return;
  }

  trace_printf("[RUN-TEST] shortest-safe start (low speed, press switch to abort)\r\n");

  (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);

  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) == HAL_OK)
  {
    enc_started_l = true;
  }
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) == HAL_OK)
  {
    enc_started_r = true;
  }

  if (!nightfall_run_guard_prepare(&guard))
  {
    if (enc_started_l)
    {
      (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
    }
    if (enc_started_r)
    {
      (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
    }
    trace_printf("[RUN-TEST] shortest-safe canceled(guard init fail)\r\n");
    return;
  }

  nightfall_trace_log_on_run_start();

  nightfall_trace_log_set_mode_flags(NIGHTFALL_F413_TRACE_MODE_SHORTEST_SAFE_FLAG |
                                     NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG);
  nightfall_motor_set(true,
                     true,
                     true,
                     NIGHTFALL_F413_RUN_SESSION_SAFE_DUTY,
                     NIGHTFALL_F413_RUN_SESSION_SAFE_DUTY);
  abort_reason = nightfall_trace_log_wait_with_auto_step_guarded(NIGHTFALL_F413_RUN_SESSION_SAFE_FORWARD_MS,
                                                                 &guard);

  if (abort_reason == NIGHTFALL_RUN_ABORT_NONE)
  {
    nightfall_trace_log_set_mode_flags(NIGHTFALL_F413_TRACE_MODE_SHORTEST_SAFE_FLAG |
                                       NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG);
    nightfall_motor_set(false, true, true, 0U, 0U);
    abort_reason = nightfall_trace_log_wait_with_auto_step_guarded(NIGHTFALL_F413_RUN_SESSION_SAFE_COAST_MS,
                                                                   &guard);
  }

  if (abort_reason == NIGHTFALL_RUN_ABORT_NONE)
  {
    nightfall_trace_log_set_mode_flags(NIGHTFALL_F413_TRACE_MODE_SHORTEST_SAFE_FLAG |
                                       NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG);
    nightfall_motor_set(true,
                       true,
                       false,
                       NIGHTFALL_F413_RUN_SESSION_SAFE_DUTY,
                       NIGHTFALL_F413_RUN_SESSION_SAFE_DUTY);
    abort_reason = nightfall_trace_log_wait_with_auto_step_guarded(NIGHTFALL_F413_RUN_SESSION_SAFE_TURN_MS,
                                                                   &guard);
  }

  if (abort_reason == NIGHTFALL_RUN_ABORT_NONE)
  {
    nightfall_trace_log_set_mode_flags(NIGHTFALL_F413_TRACE_MODE_SHORTEST_SAFE_FLAG |
                                       NIGHTFALL_F413_TRACE_MODE_MOTOR_COAST_FLAG);
    nightfall_motor_set(false, true, true, 0U, 0U);
    abort_reason = nightfall_trace_log_wait_with_auto_step_guarded(NIGHTFALL_F413_RUN_SESSION_SAFE_COAST_MS,
                                                                   &guard);
  }

  if (abort_reason == NIGHTFALL_RUN_ABORT_NONE)
  {
    nightfall_trace_log_set_mode_flags(NIGHTFALL_F413_TRACE_MODE_SHORTEST_SAFE_FLAG |
                                       NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG);
    nightfall_motor_set(true,
                       true,
                       true,
                       NIGHTFALL_F413_RUN_SESSION_SAFE_DUTY,
                       NIGHTFALL_F413_RUN_SESSION_SAFE_DUTY);
    abort_reason = nightfall_trace_log_wait_with_auto_step_guarded(NIGHTFALL_F413_RUN_SESSION_SAFE_FORWARD_MS,
                                                                   &guard);
  }

  nightfall_motor_set(false, true, true, 0U, 0U);
  if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
  {
    nightfall_trace_log_set_mode_flags((uint16_t)(NIGHTFALL_F413_TRACE_MODE_SHORTEST_SAFE_FLAG |
                                                  nightfall_run_abort_reason_to_trace_flag(abort_reason)));
    nightfall_trace_log_auto_step();
  }
  nightfall_trace_log_set_mode_flags(0U);
  nightfall_trace_log_on_run_stop();

  if (enc_started_l)
  {
    (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
  }
  if (enc_started_r)
  {
    (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
  }

  nightfall_run_guard_cleanup(&guard);

  if (abort_reason == NIGHTFALL_RUN_ABORT_SWITCH)
  {
    trace_printf("[RUN-TEST] shortest-safe aborted by switch\r\n");
  }
  else if (abort_reason != NIGHTFALL_RUN_ABORT_NONE)
  {
    trace_printf("[RUN-TEST] shortest-safe aborted(%s)\r\n",
                 nightfall_run_abort_reason_to_text(abort_reason));
  }
  else
  {
    trace_printf("[RUN-TEST] shortest-safe end\r\n");
  }
}

static void nightfall_fill_expected_sensor_params(nvm_sensor_params_t* out)
{
  if (out == NULL)
  {
    return;
  }

  nvm_params_sensor_defaults(out);
  out->base_l = 1111U;
  out->base_r = 1222U;
  out->base_f = 1333U;
  out->wall_offset_r = 200U;
  out->wall_offset_l = 210U;
  out->wall_offset_fr = 220U;
  out->wall_offset_fl = 230U;
  out->imu_offset_z = 1.25f;
}

static void nightfall_fill_expected_maze_cells(uint16_t* out, uint32_t count)
{
  uint32_t i;

  if ((out == NULL) && (count > 0U))
  {
    return;
  }

  for (i = 0U; i < count; i++)
  {
    out[i] = (uint16_t)(0x1000U + i * 3U);
  }
}

static bool nightfall_run_distance_nvm_test(void)
{
  static const float x_fl[3] = {230.0f, 420.0f, 680.0f};
  static const float y_fl[3] = {180.0f, 360.0f, 540.0f};
  static const float x_fr[3] = {235.0f, 425.0f, 685.0f};
  static const float y_fr[3] = {180.0f, 360.0f, 540.0f};
  static const float x_fsum[3] = {465.0f, 845.0f, 1365.0f};
  static const float y_fsum[3] = {180.0f, 360.0f, 540.0f};

  HAL_StatusTypeDef st = nvm_params_distance_save(x_fl, y_fl, x_fr, y_fr, x_fsum, y_fsum);
  if (st != HAL_OK)
  {
    trace_printf("[NVM-TEST][Distance] save: FAIL (HAL=%d)\r\n", (int)st);
    return false;
  }

  if (!nvm_params_distance_load_and_apply())
  {
    trace_printf("[NVM-TEST][Distance] load_and_apply: FAIL\r\n");
    return false;
  }

  trace_printf("[NVM-TEST][Distance] save/load_and_apply: PASS\r\n");
  return true;
}

static bool nightfall_run_sensor_nvm_test(void)
{
  nvm_sensor_params_t save_blob;
  nvm_sensor_params_t load_blob;
  HAL_StatusTypeDef st;

  nightfall_fill_expected_sensor_params(&save_blob);

  st = nvm_params_sensor_save(&save_blob);
  if (st != HAL_OK)
  {
    trace_printf("[NVM-TEST][Sensor] save: FAIL (HAL=%d)\r\n", (int)st);
    return false;
  }

  memset(&load_blob, 0, sizeof(load_blob));
  if (!nvm_params_sensor_load(&load_blob))
  {
    trace_printf("[NVM-TEST][Sensor] load: FAIL\r\n");
    return false;
  }

  if ((load_blob.base_l != save_blob.base_l) ||
      (load_blob.base_r != save_blob.base_r) ||
      (load_blob.base_f != save_blob.base_f) ||
      (load_blob.wall_offset_r != save_blob.wall_offset_r) ||
      (load_blob.wall_offset_l != save_blob.wall_offset_l) ||
      (load_blob.wall_offset_fr != save_blob.wall_offset_fr) ||
      (load_blob.wall_offset_fl != save_blob.wall_offset_fl) ||
      (load_blob.imu_offset_z != save_blob.imu_offset_z))
  {
    trace_printf("[NVM-TEST][Sensor] compare: FAIL\r\n");
    return false;
  }

  trace_printf("[NVM-TEST][Sensor] save/load compare: PASS\r\n");
  return true;
}

static bool nightfall_run_maze_nvm_test(void)
{
  uint16_t save_cells[NIGHTFALL_F413_TEST_MAZE_CELLS];
  uint16_t load_cells[NIGHTFALL_F413_TEST_MAZE_CELLS];
  HAL_StatusTypeDef st;
  uint32_t i;

  nightfall_fill_expected_maze_cells(save_cells, NIGHTFALL_F413_TEST_MAZE_CELLS);

  st = nvm_maze_save_map(save_cells, NIGHTFALL_F413_TEST_MAZE_CELLS);
  if (st != HAL_OK)
  {
    trace_printf("[NVM-TEST][Maze] save: FAIL (HAL=%d)\r\n", (int)st);
    return false;
  }

  memset(load_cells, 0, sizeof(load_cells));
  if (!nvm_maze_load_map(load_cells, NIGHTFALL_F413_TEST_MAZE_CELLS))
  {
    trace_printf("[NVM-TEST][Maze] load: FAIL\r\n");
    return false;
  }

  for (i = 0U; i < NIGHTFALL_F413_TEST_MAZE_CELLS; i++)
  {
    if (load_cells[i] != save_cells[i])
    {
      trace_printf("[NVM-TEST][Maze] compare: FAIL idx=%lu saved=0x%04X loaded=0x%04X\r\n",
                   (unsigned long)i,
                   (unsigned int)save_cells[i],
                   (unsigned int)load_cells[i]);
      return false;
    }
  }

  trace_printf("[NVM-TEST][Maze] save/load compare: PASS\r\n");
  return true;
}

static bool nightfall_verify_distance_nvm_load_only(void)
{
  if (!nvm_params_distance_load_and_apply())
  {
    trace_printf("[NVM-TEST][Distance] load_only: FAIL\r\n");
    return false;
  }

  trace_printf("[NVM-TEST][Distance] load_only: PASS\r\n");
  return true;
}

static bool nightfall_verify_sensor_nvm_load_only(void)
{
  nvm_sensor_params_t expected;
  nvm_sensor_params_t load_blob;

  nightfall_fill_expected_sensor_params(&expected);
  memset(&load_blob, 0, sizeof(load_blob));

  if (!nvm_params_sensor_load(&load_blob))
  {
    trace_printf("[NVM-TEST][Sensor] load_only: FAIL(load)\r\n");
    return false;
  }

  if ((load_blob.base_l != expected.base_l) ||
      (load_blob.base_r != expected.base_r) ||
      (load_blob.base_f != expected.base_f) ||
      (load_blob.wall_offset_r != expected.wall_offset_r) ||
      (load_blob.wall_offset_l != expected.wall_offset_l) ||
      (load_blob.wall_offset_fr != expected.wall_offset_fr) ||
      (load_blob.wall_offset_fl != expected.wall_offset_fl) ||
      (load_blob.imu_offset_z != expected.imu_offset_z))
  {
    trace_printf("[NVM-TEST][Sensor] load_only: FAIL(compare)\r\n");
    return false;
  }

  trace_printf("[NVM-TEST][Sensor] load_only: PASS\r\n");
  return true;
}

static bool nightfall_verify_maze_nvm_load_only(void)
{
  uint16_t expected[NIGHTFALL_F413_TEST_MAZE_CELLS];
  uint16_t loaded[NIGHTFALL_F413_TEST_MAZE_CELLS];
  uint32_t i;

  nightfall_fill_expected_maze_cells(expected, NIGHTFALL_F413_TEST_MAZE_CELLS);
  memset(loaded, 0, sizeof(loaded));

  if (!nvm_maze_load_map(loaded, NIGHTFALL_F413_TEST_MAZE_CELLS))
  {
    trace_printf("[NVM-TEST][Maze] load_only: FAIL(load)\r\n");
    return false;
  }

  for (i = 0U; i < NIGHTFALL_F413_TEST_MAZE_CELLS; i++)
  {
    if (loaded[i] != expected[i])
    {
      trace_printf("[NVM-TEST][Maze] load_only: FAIL(compare idx=%lu)\r\n", (unsigned long)i);
      return false;
    }
  }

  trace_printf("[NVM-TEST][Maze] load_only: PASS\r\n");
  return true;
}

static bool nightfall_run_trace_log_nvm_test(void)
{
  uint8_t expected[NIGHTFALL_F413_TEST_TRACE_BYTES];
  uint8_t loaded[NIGHTFALL_F413_TEST_TRACE_BYTES];
  nvm_status_t st;

  nightfall_fill_expected_trace_bytes(expected, NIGHTFALL_F413_TEST_TRACE_BYTES);
  memset(loaded, 0, sizeof(loaded));

  st = nvm_write(NVM_AREA_TRACE_LOG,
                 NIGHTFALL_F413_TEST_TRACE_OFFSET,
                 expected,
                 sizeof(expected));
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[NVM-TEST][Trace] save: FAIL (NVM=%d)\r\n", (int)st);
    return false;
  }

  st = nvm_read(NVM_AREA_TRACE_LOG,
                NIGHTFALL_F413_TEST_TRACE_OFFSET,
                loaded,
                sizeof(loaded));
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[NVM-TEST][Trace] load: FAIL (NVM=%d)\r\n", (int)st);
    return false;
  }

  if (memcmp(expected, loaded, sizeof(expected)) != 0)
  {
    trace_printf("[NVM-TEST][Trace] compare: FAIL\r\n");
    return false;
  }

  trace_printf("[NVM-TEST][Trace] save/load compare: PASS\r\n");
  return true;
}

static bool nightfall_verify_trace_log_nvm_load_only(void)
{
  uint8_t expected[NIGHTFALL_F413_TEST_TRACE_BYTES];
  uint8_t loaded[NIGHTFALL_F413_TEST_TRACE_BYTES];
  nvm_status_t st;

  nightfall_fill_expected_trace_bytes(expected, NIGHTFALL_F413_TEST_TRACE_BYTES);
  memset(loaded, 0, sizeof(loaded));

  st = nvm_read(NVM_AREA_TRACE_LOG,
                NIGHTFALL_F413_TEST_TRACE_OFFSET,
                loaded,
                sizeof(loaded));
  if (st != NVM_STATUS_OK)
  {
    trace_printf("[NVM-TEST][Trace] load_only: FAIL(load NVM=%d)\r\n", (int)st);
    return false;
  }

  if (memcmp(expected, loaded, sizeof(expected)) != 0)
  {
    trace_printf("[NVM-TEST][Trace] load_only: FAIL(compare)\r\n");
    return false;
  }

  trace_printf("[NVM-TEST][Trace] load_only: PASS\r\n");
  return true;
}

static void nightfall_run_all_nvm_tests(void)
{
  bool distance_ok = nightfall_run_distance_nvm_test();
  bool sensor_ok = nightfall_run_sensor_nvm_test();
  bool maze_ok = nightfall_run_maze_nvm_test();
  bool trace_ok = nightfall_run_trace_log_nvm_test();

  trace_printf("[NVM-TEST][Overall] %s\r\n", (distance_ok && sensor_ok && maze_ok && trace_ok) ? "PASS" : "FAIL");
}

static void nightfall_verify_all_nvm_load_only(void)
{
  bool distance_ok = nightfall_verify_distance_nvm_load_only();
  bool sensor_ok = nightfall_verify_sensor_nvm_load_only();
  bool maze_ok = nightfall_verify_maze_nvm_load_only();
  bool trace_ok = nightfall_verify_trace_log_nvm_load_only();

  trace_printf("[NVM-TEST][Overall][LoadOnly] %s\r\n", (distance_ok && sensor_ok && maze_ok && trace_ok) ? "PASS" : "FAIL");
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
      g_test_armed_id = cmd;
      nightfall_trace_log_set_context(8U, 0xFFU, 0xFFU, cmd);
      nightfall_test_run(cmd);
      break;

    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
      nightfall_test_arm_for_button(cmd);
      break;

    case 'f':
    case 'F':
      /* 直前に '1'-'5' を送っていなければデフォルト '1' をアーム */
      {
        uint8_t arm_id = (g_test_armed_id >= '1' && g_test_armed_id <= '5')
                         ? g_test_armed_id : '1';
        nightfall_test_arm_for_button(arm_id);
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

  f413_ctrl_init();
  trace_printf("[CTRL] 1kHz velocity control initialized\r\n");
  nightfall_op_led_show_mode(g_op_mode);
  trace_printf("[OP-UI] ready %s=%u %s\r\n",
               nightfall_op_level_name(g_op_level),
               (unsigned int)nightfall_op_selected_value(),
               nightfall_op_mode_name(g_op_mode));

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

    /* ボタンアーム実行: ボタン押下でアーム済みテストを開始 */
    if ((g_test_armed_id >= '1') && (g_test_armed_id <= '5'))
    {
      if (nightfall_run_stop_switch_pressed())
      {
        /* チャタリング待ち + リリース待ち */
        HAL_Delay(50U);
        while (nightfall_run_stop_switch_pressed()) { HAL_Delay(10U); }
        HAL_Delay(50U);

        /* 2秒待ってから実行（手を離してIMUオフセットを安定させる猶予） */
        HAL_Delay(2000U);

        uint8_t tid = g_test_armed_id;
        g_test_armed_id = NIGHTFALL_F413_TEST_ARMED_NONE;
        nightfall_test_run(tid);
      }
    }

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
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  huart1.Init.BaudRate = 115200;
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
  }
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
