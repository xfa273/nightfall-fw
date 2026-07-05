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
#include "build_info.h"
#include "nvm.h"
#include "nvm_identity.h"
#include "f413_control.h"
#include "f413_control_tune_run.h"
#include "f413_hw.h"
#include "f413_hw_diag.h"
#include "f413_imu_diag.h"
#include "f413_mode1.h"
#include "f413_mode2.h"
#include "f413_mode3.h"
#include "f413_mode4.h"
#include "f413_mode5.h"
#include "f413_mode6.h"
#include "f413_mode7.h"
#include "f413_mode_shortest.h"
#include "f413_nvm_diag.h"
#include "f413_op_ui.h"
#include "f413_path_run.h"
#include "f413_run_features.h"
#include "f413_run_session.h"
#include "f413_search_step.h"
#include "f413_test_run.h"
#include "f413_trace_flags.h"
#include "f413_trace_log.h"
#include "f413_trace_diag.h"
#include "f413_trace_sample.h"
#include "f413_uart_cli.h"
#include "f413_wall_distance.h"
#include "f413_wall_runtime.h"
#include "f413_wall_sensor.h"
#include "params.h"
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
#define NIGHTFALL_F413_RUN_SESSION_IDLE_MS (1000U)
#define NIGHTFALL_F413_RUN_SESSION_MOTOR_DUTY (120U)
#define NIGHTFALL_F413_RUN_SESSION_MOTOR_PULSE_MS (300U)
#define NIGHTFALL_F413_RUN_SESSION_MOTOR_COAST_MS (120U)
#define NIGHTFALL_F413_RUN_SESSION_SAFE_DUTY (70U)
#define NIGHTFALL_F413_RUN_SESSION_SAFE_FORWARD_MS (160U)
#define NIGHTFALL_F413_RUN_SESSION_SAFE_TURN_MS (120U)
#define NIGHTFALL_F413_RUN_SESSION_SAFE_COAST_MS (80U)
#define NIGHTFALL_F413_RUN_SESSION_SAFE_EXPLORE_STEPS (4U)
#define NIGHTFALL_F413_REAL_GOAL_X (15U)
#define NIGHTFALL_F413_REAL_GOAL_Y (15U)

#ifndef NIGHTFALL_F413_UART_BAUD_RATE
#define NIGHTFALL_F413_UART_BAUD_RATE (115200U)
#endif

/* ---------- 調整用テストモード定数 ---------- */
#define NIGHTFALL_F413_OP_MODE_MAX        (9U)
#define NIGHTFALL_F413_OP_ENTER_DELTA_ADC (150)
#define NIGHTFALL_F413_OP_ENTER_RELEASE_ADC (250)
#define NIGHTFALL_F413_OP_START_DELAY_MS  (2000U)
#define NIGHTFALL_F413_SEARCH_STEP_VELOCITY_MM_S (150.0f)
#define NIGHTFALL_F413_SEARCH_STEP_TARGET_MM (90.0f)
#define NIGHTFALL_F413_SEARCH_STEP_TURN_DEG (90.0f)
#define NIGHTFALL_F413_TUNE_TIMEOUT_MS (1500U)
#define NIGHTFALL_F413_SENSOR_CAL_SETTLE_MS (1500U)
#define NIGHTFALL_F413_WALL_SENSOR_MONITOR_PERIOD_MS (500U)

typedef f413_wall_sensor_snapshot_t nightfall_wall_sensor_snapshot_t;

static void nightfall_run_search_safe_trace_session_once(void);
static void nightfall_run_shortest_safe_trace_session_once(void);
static void nightfall_run_search_trace_entry_once(void);
static void nightfall_run_shortest_trace_entry_once(uint8_t mode, uint8_t op_case);
static void nightfall_run_shortest_trace_entry_default_once(void);
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
static void nightfall_trace_log_on_run_start(void);
static void nightfall_trace_log_on_run_stop(void);
static void nightfall_trace_log_auto_step(void);
static void nightfall_trace_log_auto_tick_sample(void);
static void nightfall_trace_log_set_mode_flags(uint16_t flags);
static const char* nightfall_op_mode_name(uint8_t mode);
static const char* nightfall_op_level_name(uint8_t level);
static const char* nightfall_op_case_name(uint8_t mode, uint8_t op_case);
static const char* nightfall_op_sub_name(uint8_t mode, uint8_t sub);
static bool nightfall_wall_sensor_start_async(void);
static void nightfall_wall_sensor_tim6_tick(void);
static bool nightfall_wall_sensor_read_snapshot(nightfall_wall_sensor_snapshot_t* out);
static void nightfall_motor_set(bool enable, bool left_forward, bool right_forward,
                                uint16_t left_duty, uint16_t right_duty);
static void nightfall_op_run_tune_sub_after_delay(uint8_t sub);
static void nightfall_op_execute_action(f413_op_ui_action_t action, uint8_t mode, uint8_t op_case, uint8_t sub);
static void nightfall_op_ui_step(void);
static void nightfall_run_fan_pwm_test_once(void);
static void nightfall_run_encoder_test_once(void);
static void nightfall_op_busy_delay_ms(uint32_t duration_ms);

static void nightfall_run_search_trace_entry_once(void)
{
#if (NIGHTFALL_F413_REAL_RUN_PATH_ENABLED != 0U)
  if (solver_build_path(NIGHTFALL_F413_SOLVER_MODE, NIGHTFALL_F413_SOLVER_CASE))
  {
    trace_printf("[RUN-TEST] search-entry solver-path ready mode=%u case=%u\r\n",
                 (unsigned int)NIGHTFALL_F413_SOLVER_MODE,
                 (unsigned int)NIGHTFALL_F413_SOLVER_CASE);
    f413_path_run_print_preview();
    f413_run_features_reset();
    f413_path_run_solver_session_once(NIGHTFALL_F413_TRACE_MODE_SEARCH_SAFE_FLAG);
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

static void nightfall_run_shortest_trace_entry_once(uint8_t mode, uint8_t op_case)
{
  switch (mode)
  {
    case 2U: f413_mode2_run_case(op_case); break;
    case 3U: f413_mode3_run_case(op_case); break;
    case 4U: f413_mode4_run_case(op_case); break;
    case 5U: f413_mode5_run_case(op_case); break;
    case 6U: f413_mode6_run_case(op_case); break;
    case 7U: f413_mode7_run_case(op_case); break;
    default: f413_mode_shortest_run_case(mode, op_case); break;
  }
}

static void nightfall_run_shortest_trace_entry_default_once(void)
{
  nightfall_run_shortest_trace_entry_once(NIGHTFALL_F413_SOLVER_MODE,
                                          NIGHTFALL_F413_SOLVER_CASE);
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

static bool nightfall_wall_sensor_start_async(void)
{
  return f413_wall_sensor_start_async();
}

static void nightfall_wall_sensor_tim6_tick(void)
{
  f413_wall_sensor_tim6_tick();
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
  trace_printf("[TRACE-LOG] run-hook: stop tail=%u ms\r\n",
               (unsigned int)F413_TRACE_LOG_STOP_TAIL_MS_DEFAULT);
  f413_trace_log_auto_stop_after_tail(F413_TRACE_LOG_STOP_TAIL_MS_DEFAULT);
}

static void nightfall_op_led_show_mode(uint8_t mode)
{
  f413_hw_show_mode_leds(mode);
}

static void nightfall_boot_buzzer_pattern(void)
{
  f413_hw_boot_buzzer_pattern();
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

static void nightfall_op_busy_delay_ms(uint32_t duration_ms)
{
  f413_hw_delay_with_led_blink(F413_HW_LED_REAR_RIGHT_MASK,
                               duration_ms,
                               F413_HW_LED_BLINK_TOGGLE_MS);
}

static void nightfall_run_wall_sensor_test_once(void)
{
  nightfall_wall_sensor_snapshot_t wall;
  uint16_t base_l = 0U;
  uint16_t base_r = 0U;
  uint16_t base_f = 0U;
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
  f413_wall_sensor_get_control_base(&base_l, &base_r, &base_f);
  trace_printf("[HW-TEST][Wall] ctrl-base: L=%u R=%u F=%u fallback L=%u R=%u\r\n",
               (unsigned int)base_l,
               (unsigned int)base_r,
               (unsigned int)base_f,
               (unsigned int)WALL_CTRL_BASE_L,
               (unsigned int)WALL_CTRL_BASE_R);
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

static void nightfall_run_wall_distance_test_once(void)
{
  f413_wall_distance_snapshot_t dist;

  if (!f413_wall_distance_read_snapshot(&dist))
  {
    trace_printf("[HW-TEST][WallDist] FAIL(read snapshot)\r\n");
    return;
  }

  trace_printf("[HW-TEST][WallDist] delta: FR=%ld R=%ld FL=%ld L=%ld fsum=%lu\r\n",
               (long)dist.adc.fr_delta,
               (long)dist.adc.r_delta,
               (long)dist.adc.fl_delta,
               (long)dist.adc.l_delta,
               (unsigned long)((uint32_t)dist.adc.fr_delta + (uint32_t)dist.adc.fl_delta));
  trace_printf("[HW-TEST][WallDist] mm: FR=%.2f R=%.2f FL=%.2f L=%.2f FSUM=%.2f\r\n",
               (double)dist.fr_mm,
               (double)dist.r_mm,
               (double)dist.fl_mm,
               (double)dist.l_mm,
               (double)dist.front_sum_mm);
  trace_printf("[HW-TEST][WallDist] unwarped: FR=%.2f FL=%.2f FSUM=%.2f params=%u\r\n",
               (double)dist.fr_mm_unwarped,
               (double)dist.fl_mm_unwarped,
               (double)dist.front_sum_mm_unwarped,
               (unsigned int)dist.distance_params_loaded);
  trace_printf("[HW-TEST][WallDist] mask: valid=0x%04X extrap=0x%04X sat=0x%04X low=0x%04X front_valid=%u right_valid=%u left_valid=%u\r\n",
               (unsigned int)dist.valid_mask,
               (unsigned int)dist.extrapolated_mask,
               (unsigned int)dist.saturated_mask,
               (unsigned int)dist.below_signal_mask,
               (unsigned int)dist.front_valid,
               (unsigned int)dist.right_valid,
               (unsigned int)dist.left_valid);
  trace_printf("#sensor_distance_columns=record,t_ms,fr_delta,r_delta,fl_delta,l_delta,fr_mm,r_mm,fl_mm,l_mm,front_sum_mm,valid_mask,extrapolated_mask,saturated_mask,below_signal_mask\r\n");
  trace_printf("sensor_distance,%lu,%ld,%ld,%ld,%ld,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%u\r\n",
               (unsigned long)HAL_GetTick(),
               (long)dist.adc.fr_delta,
               (long)dist.adc.r_delta,
               (long)dist.adc.fl_delta,
               (long)dist.adc.l_delta,
               (double)dist.fr_mm,
               (double)dist.r_mm,
               (double)dist.fl_mm,
               (double)dist.l_mm,
               (double)dist.front_sum_mm,
               (unsigned int)dist.valid_mask,
               (unsigned int)dist.extrapolated_mask,
               (unsigned int)dist.saturated_mask,
               (unsigned int)dist.below_signal_mask);
}

static void nightfall_run_wall_sensor_monitor_once(void)
{
  uint16_t base_l = 0U;
  uint16_t base_r = 0U;
  uint16_t base_f = 0U;
  uint16_t offset_r = 0U;
  uint16_t offset_l = 0U;
  uint16_t offset_fr = 0U;
  uint16_t offset_fl = 0U;
  uint8_t ready_mask = 0U;
  uint8_t phase = 0U;
  uint8_t inflight = 0U;

  f413_wall_sensor_get_debug_state(&offset_r,
                                   &offset_l,
                                   &offset_fr,
                                   &offset_fl,
                                   &ready_mask,
                                   &phase,
                                   &inflight);
  f413_wall_sensor_get_control_base(&base_l, &base_r, &base_f);
  trace_printf("[HW-TEST][Wall] monitor start period=%lums; press PUSH to stop\r\n",
               (unsigned long)NIGHTFALL_F413_WALL_SENSOR_MONITOR_PERIOD_MS);
  trace_printf("[HW-TEST][Wall] offset R=%u L=%u FR=%u FL=%u ready=0x%02X phase=%u inflight=%u\r\n",
               (unsigned int)offset_r,
               (unsigned int)offset_l,
               (unsigned int)offset_fr,
               (unsigned int)offset_fl,
               (unsigned int)ready_mask,
               (unsigned int)phase,
               (unsigned int)inflight);
  trace_printf("[HW-TEST][Wall] ctrl-base L=%u R=%u F=%u fallback L=%u R=%u thr FR=%u FL=%u R=%u L=%u\r\n",
               (unsigned int)base_l,
               (unsigned int)base_r,
               (unsigned int)base_f,
               (unsigned int)WALL_CTRL_BASE_L,
               (unsigned int)WALL_CTRL_BASE_R,
               (unsigned int)WALL_BASE_FR,
               (unsigned int)WALL_BASE_FL,
               (unsigned int)WALL_BASE_R,
               (unsigned int)WALL_BASE_L);

  while (!nightfall_run_stop_switch_pressed())
  {
    nightfall_wall_sensor_snapshot_t wall;

    if (nightfall_wall_sensor_read_snapshot(&wall))
    {
      trace_printf("[HW-TEST][Wall] t=%lu off R=%u L=%u FR=%u FL=%u on R=%u L=%u FR=%u FL=%u d R=%ld L=%ld FR=%ld FL=%ld det F=%u R=%u L=%u sat=%u VB=%u\r\n",
                   (unsigned long)HAL_GetTick(),
                   (unsigned int)wall.r_off,
                   (unsigned int)wall.l_off,
                   (unsigned int)wall.fr_off,
                   (unsigned int)wall.fl_off,
                   (unsigned int)wall.r_on,
                   (unsigned int)wall.l_on,
                   (unsigned int)wall.fr_on,
                   (unsigned int)wall.fl_on,
                   (long)wall.r_delta,
                   (long)wall.l_delta,
                   (long)wall.fr_delta,
                   (long)wall.fl_delta,
                   (unsigned int)wall.front_wall,
                   (unsigned int)wall.right_wall,
                   (unsigned int)wall.left_wall,
                   (unsigned int)wall.saturated,
                   (unsigned int)wall.vbat_on);
    }
    else
    {
      trace_printf("[HW-TEST][Wall] FAIL(read snapshot)\r\n");
    }

    HAL_Delay(NIGHTFALL_F413_WALL_SENSOR_MONITOR_PERIOD_MS);
  }

  trace_printf("[HW-TEST][Wall] monitor stop\r\n");
}

static void nightfall_run_sensor_side_base_save_once(void)
{
  nvm_sensor_params_t saved;
  HAL_StatusTypeDef st;

  trace_printf("[SENSOR-CAL] side baseline: place at cell center with walls on both sides\r\n");
  trace_printf("[SENSOR-CAL] wait %lu ms, then sampling 600 x 5ms; motors/fan are not used\r\n",
               (unsigned long)NIGHTFALL_F413_SENSOR_CAL_SETTLE_MS);
  nightfall_op_busy_delay_ms(NIGHTFALL_F413_SENSOR_CAL_SETTLE_MS);
  st = f413_wall_sensor_calibrate_side_base_and_save(600U, 5U, &saved);
  f413_hw_show_led_mask(0U);
  if (st == HAL_OK)
  {
    trace_printf("[SENSOR-CAL] side baseline saved: base_l=%u base_r=%u base_f=%u off_r=%u off_l=%u off_fr=%u off_fl=%u\r\n",
                 (unsigned int)saved.base_l,
                 (unsigned int)saved.base_r,
                 (unsigned int)saved.base_f,
                 (unsigned int)saved.wall_offset_r,
                 (unsigned int)saved.wall_offset_l,
                 (unsigned int)saved.wall_offset_fr,
                 (unsigned int)saved.wall_offset_fl);
    f413_hw_buzzer_beep_ms(1200U, 160U);
  }
  else
  {
    trace_printf("[SENSOR-CAL] side baseline save failed: HAL status=%d\r\n", (int)st);
    f413_hw_buzzer_beep_ms(3000U, 160U);
  }
}

static void nightfall_run_sensor_offset_save_once(void)
{
  nvm_sensor_params_t saved;
  HAL_StatusTypeDef st;

  trace_printf("[SENSOR-CAL] offset: run with no nearby walls / ambient condition\r\n");
  trace_printf("[SENSOR-CAL] wait %lu ms, then sampling 10 x 10ms; motors/fan are not used\r\n",
               (unsigned long)NIGHTFALL_F413_SENSOR_CAL_SETTLE_MS);
  nightfall_op_busy_delay_ms(NIGHTFALL_F413_SENSOR_CAL_SETTLE_MS);
  st = f413_wall_sensor_calibrate_offsets_and_save(10U, 10U, &saved);
  f413_hw_show_led_mask(0U);
  if (st == HAL_OK)
  {
    trace_printf("[SENSOR-CAL] offset saved: off_r=%u off_l=%u off_fr=%u off_fl=%u base_l=%u base_r=%u base_f=%u\r\n",
                 (unsigned int)saved.wall_offset_r,
                 (unsigned int)saved.wall_offset_l,
                 (unsigned int)saved.wall_offset_fr,
                 (unsigned int)saved.wall_offset_fl,
                 (unsigned int)saved.base_l,
                 (unsigned int)saved.base_r,
                 (unsigned int)saved.base_f);
    f413_hw_buzzer_beep_ms(1200U, 160U);
  }
  else
  {
    trace_printf("[SENSOR-CAL] offset save failed: HAL status=%d\r\n", (int)st);
    f413_hw_buzzer_beep_ms(3000U, 160U);
  }
}
static void nightfall_run_imu_test_once(void)
{
  f413_imu_diag_run_whoami_test_once();
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
  f413_trace_sample_set_context(8U, f413_op_ui_get_case(), 0xFFU, test_id);
  nightfall_op_busy_delay_ms(NIGHTFALL_F413_OP_START_DELAY_MS);
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

  f413_trace_sample_set_context(9U, 0U, sub, (uint8_t)'0');
  nightfall_op_busy_delay_ms(NIGHTFALL_F413_OP_START_DELAY_MS);
  f413_control_tune_run_once(axis, set, pattern);
}
static void nightfall_op_run_case0_sub_after_delay(uint8_t mode, uint8_t sub)
{
  f413_trace_sample_set_context(mode, 0U, sub, (uint8_t)'P');
  nightfall_op_busy_delay_ms(NIGHTFALL_F413_OP_START_DELAY_MS);

  switch (mode)
  {
    case 2U:
      f413_mode2_run_case0_sub(sub);
      break;
    case 3U:
      f413_mode3_run_case0_sub(sub);
      break;
    case 4U:
      f413_mode4_run_case0_sub(sub);
      break;
    case 5U:
      f413_mode5_run_case0_sub(sub);
      break;
    case 6U:
      f413_mode6_run_case0_sub(sub);
      break;
    case 7U:
      f413_mode7_run_case0_sub(sub);
      break;
    default:
      trace_printf("[OP-UI] no-op: mode%u case0 sub%u is not assigned\r\n",
                   (unsigned int)mode,
                   (unsigned int)sub);
      break;
  }
}

static void nightfall_op_execute_action(f413_op_ui_action_t action, uint8_t mode, uint8_t op_case, uint8_t sub)
{
  switch (action)
  {
    case F413_OP_UI_ACTION_SEARCH_TRACE_ENTRY:
      f413_trace_sample_set_context(mode, op_case, 0xFFU, 0U);
      nightfall_op_busy_delay_ms(NIGHTFALL_F413_OP_START_DELAY_MS);
      nightfall_run_search_trace_entry_once();
      f413_op_ui_lock_after_run_end();
      break;
    case F413_OP_UI_ACTION_SHORTEST_TRACE_ENTRY:
      f413_trace_sample_set_context(mode, op_case, 0xFFU, 0U);
      nightfall_op_busy_delay_ms(NIGHTFALL_F413_OP_START_DELAY_MS);
      nightfall_run_shortest_trace_entry_once(mode, op_case);
      f413_op_ui_lock_after_run_end();
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
      if ((mode == 9U) && (op_case == 3U))
      {
        nightfall_run_wall_sensor_monitor_once();
      }
      else
      {
        nightfall_run_wall_sensor_test_once();
      }
      break;
    case F413_OP_UI_ACTION_FAN_PWM_TEST:
      nightfall_run_fan_pwm_test_once();
      break;
    case F413_OP_UI_ACTION_TRACE_DUMP_BIN_ALL:
      f413_trace_diag_run_dump_bin_all_once();
      break;
    case F413_OP_UI_ACTION_NVM_STATUS:
      f413_nvm_diag_run_nvm_status_once();
      break;
    case F413_OP_UI_ACTION_IDENTITY_STATUS:
      f413_nvm_diag_run_identity_status_once(g_boot_identity_status, &g_boot_identity);
      break;
    case F413_OP_UI_ACTION_SENSOR_PARAMS_STATUS:
      f413_nvm_diag_run_sensor_params_status_once();
      break;
    case F413_OP_UI_ACTION_SENSOR_SIDE_BASE_SAVE:
      nightfall_run_sensor_side_base_save_once();
      break;
    case F413_OP_UI_ACTION_SENSOR_OFFSET_SAVE:
      nightfall_run_sensor_offset_save_once();
      break;
    case F413_OP_UI_ACTION_CONTROL_TUNE_SUB:
      nightfall_op_run_tune_sub_after_delay(sub);
      break;
    case F413_OP_UI_ACTION_PATH_CASE0_SUB:
      nightfall_op_run_case0_sub_after_delay(mode, sub);
      break;
    case F413_OP_UI_ACTION_SEARCH_CASE0_SUB:
      f413_trace_sample_set_context(mode, 0U, sub, (uint8_t)'T');
      nightfall_op_busy_delay_ms(NIGHTFALL_F413_OP_START_DELAY_MS);
      f413_mode1_run_case0_sub(sub);
      break;
    case F413_OP_UI_ACTION_SEARCH_RUN_CASE:
      f413_trace_sample_set_context(mode, op_case, 0xFFU, 0U);
      nightfall_op_busy_delay_ms(NIGHTFALL_F413_OP_START_DELAY_MS);
      f413_mode1_run_case(op_case);
      f413_op_ui_lock_after_run_end();
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

static void nightfall_run_encoder_test_once(void)
{
  f413_hw_diag_run_encoder_test_once();
}

static void nightfall_run_hardware_smoke_tests(void)
{
  trace_printf("[HW-TEST] smoke start\r\n");
  f413_hw_diag_run_switch_test_once();
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
  (void)f413_wall_runtime_poll_wall_end(true);
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
  return f413_imu_diag_whoami_ok();
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

static __attribute__((unused)) void nightfall_run_shortest_safe_trace_session_once(void)
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
  f413_run_features_reset();
  f413_wall_distance_init();
  {
    const f413_trace_sample_config_t trace_sample_config = {
      HAL_GetTick,
      nightfall_run_session_encoder_l_count,
      nightfall_run_session_encoder_r_count,
      f413_wall_sensor_read_adc_raw
    };
    f413_trace_sample_config(&trace_sample_config);
  }
  f413_trace_sample_set_identity(g_boot_identity_status, &g_boot_identity);
  f413_trace_log_config(f413_trace_sample_fill_control,
                        f413_trace_sample_update_observe_cache,
                        f413_wall_runtime_end_clear);
  {
    const f413_wall_runtime_config_t wall_runtime_config = {
      nightfall_wall_sensor_read_snapshot,
      HAL_Delay,
      HAL_GetTick,
      0U,
      0U,
      NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG
    };
    f413_wall_runtime_config(&wall_runtime_config);
  }
  {
    const f413_trace_diag_config_t trace_diag_config = {
      f413_trace_sample_fill,
      f413_trace_sample_get_context,
      nightfall_op_mode_name,
      nightfall_op_case_name,
      nightfall_op_sub_name,
      f413_trace_sample_emit_extra_csv_meta
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
    const f413_control_tune_run_config_t control_tune_config = {
      nightfall_run_stop_switch_pressed,
      HAL_GetTick,
      f413_trace_log_auto_is_enabled,
      nightfall_trace_log_on_run_start,
      nightfall_trace_log_on_run_stop,
      nightfall_trace_log_set_mode_flags,
      nightfall_trace_log_auto_step,
      f413_trace_sample_record_result,
      NIGHTFALL_F413_TUNE_TIMEOUT_MS,
      NIGHTFALL_F413_TRACE_MODE_TUNE_FLAG,
      NIGHTFALL_F413_TRACE_MODE_MOTOR_FWD_FLAG,
      NIGHTFALL_F413_TRACE_MODE_MOTOR_REV_FLAG
    };
    f413_control_tune_run_config(&control_tune_config);
  }
  {
    const f413_search_step_config_t search_step_config = {
      nightfall_run_stop_switch_pressed,
      HAL_GetTick,
      nightfall_wall_sensor_read_snapshot,
      f413_trace_log_auto_is_enabled,
      f413_trace_sample_set_context,
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
      f413_trace_sample_set_context,
      nightfall_trace_log_on_run_start,
      nightfall_trace_log_on_run_stop,
      nightfall_trace_log_set_mode_flags,
      nightfall_trace_log_auto_step,
      nightfall_wall_control_apply_straight,
      f413_trace_sample_record_result,
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
  {
    const f413_uart_cli_config_t uart_cli_config = {
      nightfall_run_wall_sensor_test_once,
      nightfall_run_wall_distance_test_once,
      nightfall_op_run_tune_sub_after_delay,
      nightfall_trace_log_on_run_start,
      nightfall_trace_log_on_run_stop,
      nightfall_run_hardware_smoke_tests_with_trace_session,
      nightfall_run_idle_trace_session_once,
      nightfall_run_motor_trace_session_once,
      nightfall_run_search_trace_entry_once,
      nightfall_run_shortest_trace_entry_default_once
    };
    f413_uart_cli_config(&uart_cli_config);
  }
  trace_printf("\r\n[NIGHTFALL] STM32F413 bring-up\r\n");
  trace_printf("FW=%s TARGET=%s BUILD=%s\r\n", FW_VERSION, FW_TARGET, FW_BUILD_TYPE);
  trace_printf("GIT=%s DIRTY=%d\r\n", FW_GIT_SHA, FW_GIT_DIRTY);
  nightfall_boot_buzzer_pattern();

  if (g_boot_identity_status == NVM_STATUS_OK)
  {
    const char* family_name = f413_nvm_diag_identity_family_name(g_boot_identity.family);
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
  f413_uart_cli_print_help();

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
      f413_uart_cli_handle_command(cmd);
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
