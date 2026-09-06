#include "f413_hw_diag.h"

#include <stdint.h>

#include "f413_hw.h"
#include "f413_machine.h"
#include "f413_measurements.h"
#include "f413_control.h"
#include "f413_test_run.h"
#include "f413_trace_log.h"
#include "f413_wall_sensor.h"
#include "stm32f4xx_hal.h"
#include "trace.h"

#define F413_HW_DIAG_ENCODER_WINDOW_MS (10000U)
#define F413_HW_DIAG_LED_ON_WINDOW_MS (30000U)
#define F413_HW_DIAG_MOTOR_BREAK_IN_DUTY (500U)
#define F413_HW_DIAG_ENCODER_SIGN_L (f413_machine_hardware()->encoder_sign_l)
#define F413_HW_DIAG_ENCODER_SIGN_R (f413_machine_hardware()->encoder_sign_r)

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

/* No feedback controller or run session: a secured chassis cannot follow a
   yaw reference. All motion times are MCU-bounded even if UART disconnects. */
static bool lifted_wait(uint32_t duration_ms, uint16_t* minimum_adc)
{
  uint32_t start = HAL_GetTick(), fresh = start, sequence = 0U;
  while ((uint32_t)(HAL_GetTick() - start) < duration_ms)
  {
    f413_wall_sensor_snapshot_t wall;
    if (f413_hw_stop_switch_pressed() || !f413_wall_sensor_read_snapshot(&wall)) return false;
    if (wall.sample_sequence != sequence) { sequence = wall.sample_sequence; fresh = HAL_GetTick(); }
    if ((uint32_t)(HAL_GetTick() - fresh) > 30U) return false;
    const float voltage = f413_battery_voltage(wall.vbat_on, 3.3f,
        f413_machine_hardware()->battery_divider_ratio);
    if (voltage < 7.0f || voltage > 9.0f) return false;
    if (wall.vbat_on < *minimum_adc) *minimum_adc = wall.vbat_on;
    HAL_Delay(2U);
  }
  return true;
}

void f413_hw_diag_run_lifted_sweep_once(void)
{
  const uint16_t duties[] = {60U, 120U, 180U};
  uint16_t minimum_adc = 4095U;
  bool ok = false;
  if (!f413_machine_has(F413_CAP_DRIVE | F413_CAP_WALL) ||
      f413_ctrl_is_running() || f413_test_run_is_armed() ||
      f413_trace_log_auto_is_enabled() || __HAL_TIM_GET_COMPARE(&htim10, TIM_CHANNEL_1) != 0U)
  {
    trace_printf("[LIFTED-SWEEP] REFUSED identity/control/armed/trace/fan\r\n");
    return;
  }
  f413_hw_motor_set(false, true, true, 0U, 0U);
  if (!lifted_wait(50U, &minimum_adc)) goto stopped;
  /* ctrl_init already starts these timers. HAL correctly rejects a second
     Start on a busy channel; preserve the running encoder counters. */
  if (((htim3.Instance->CR1 & TIM_CR1_CEN) == 0U &&
       HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) ||
      ((htim4.Instance->CR1 & TIM_CR1_CEN) == 0U &&
       HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK))
  {
    trace_printf("[LIFTED-SWEEP] REFUSED encoder-start\r\n");
    goto stopped;
  }
  trace_printf("[LIFTED-SWEEP] START secured-only nominal_vref=3.3 drive=300ms coast=300ms PSC=%u\r\n",
      f413_machine_hardware()->motor_pwm_prescaler);
  for (unsigned step = 0U; step < 14U; ++step)
  {
    const bool both = step >= 12U;
    const bool left = both || (step % 2U == 0U);
    const bool right = both || !left;
    const bool forward = both ? step == 12U : (step % 4U < 2U);
    const uint16_t duty = both ? 120U : duties[step / 4U];
    const uint32_t l0 = __HAL_TIM_GET_COUNTER(&htim3), r0 = __HAL_TIM_GET_COUNTER(&htim4);
    minimum_adc = 4095U;
    f413_hw_motor_set(true, forward, forward, left ? duty : 0U, right ? duty : 0U);
    ok = lifted_wait(300U, &minimum_adc);
    f413_hw_motor_set(false, true, true, 0U, 0U);
    const int32_t dl = F413_HW_DIAG_ENCODER_SIGN_L * f413_hw_encoder_delta_signed(__HAL_TIM_GET_COUNTER(&htim3), l0);
    const int32_t dr = F413_HW_DIAG_ENCODER_SIGN_R * f413_hw_encoder_delta_signed(__HAL_TIM_GET_COUNTER(&htim4), r0);
    trace_printf("[LIFTED-SWEEP] step=%u side=%s dir=%s duty=%u drive_counts=%ld,%ld vbat_min_adc=%u vbat_min_nominal=%.3f ok=%u\r\n",
        step, both ? "LR" : (left ? "L" : "R"), forward ? "FWD" : "REV", duty,
        (long)dl, (long)dr, minimum_adc,
        (double)f413_battery_voltage(minimum_adc, 3.3f, f413_machine_hardware()->battery_divider_ratio), ok);
    if (!ok || !lifted_wait(300U, &minimum_adc)) { ok = false; goto stopped; }
    /* Reject opposite/stalled driven encoder and unexpected unpowered motion.
       Low duty may fail here; do not automatically escalate a stalled motor. */
    if ((left && (forward ? dl <= 0 : dl >= 0)) ||
        (right && (forward ? dr <= 0 : dr >= 0)) ||
        (!left && (dl < -4 || dl > 4)) || (!right && (dr < -4 || dr > 4)))
    { ok = false; goto stopped; }
  }
stopped:
  f413_hw_motor_set(false, true, true, 0U, 0U);
  trace_printf("[LIFTED-SWEEP] END %s motors=off fan=off nvm=unchanged\r\n", ok ? "PASS" : "ABORT");
}

void f413_hw_diag_run_led_test_once(void)
{
  trace_printf("[HW-TEST][LED] all on for %lu ms\r\n",
               (unsigned long)F413_HW_DIAG_LED_ON_WINDOW_MS);

  f413_hw_set_all_leds(GPIO_PIN_SET);
  HAL_Delay(F413_HW_DIAG_LED_ON_WINDOW_MS);
  f413_hw_set_all_leds(GPIO_PIN_RESET);

  trace_printf("[HW-TEST][LED] PASS(all LEDs were on)\r\n");
}

void f413_hw_diag_run_switch_test_once(void)
{
  GPIO_PinState raw = f413_hw_stop_switch_raw();
  trace_printf("[HW-TEST][Switch] raw=%u (%s)\r\n",
               (unsigned int)raw,
               (raw == GPIO_PIN_RESET) ? "pressed or low" : "released or high");
}

void f413_hw_diag_run_buzzer_test_once(void)
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

void f413_hw_diag_run_fan_pwm_test_once(void)
{
  if (!f413_machine_has(F413_CAP_FAN))
  {
    trace_printf("[HW-TEST][Fan] blocked: identity/capability\r\n");
    return;
  }
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

void f413_hw_diag_run_motor_driver_test_once(void)
{
  trace_printf("[HW-TEST][Motor] start short drive (lift robot before test)\r\n");

  f413_hw_motor_set(true, true, true, 120U, 120U);
  HAL_Delay(300U);
  f413_hw_motor_set(false, true, true, 0U, 0U);
  HAL_Delay(120U);

  f413_hw_motor_set(true, false, false, 120U, 120U);
  HAL_Delay(300U);
  f413_hw_motor_set(false, false, false, 0U, 0U);

  trace_printf("[HW-TEST][Motor] PASS(pulse done)\r\n");
}

void f413_hw_diag_start_motor_break_in_continuous(void)
{
  /* This intentionally has no software stop path: a reset returns the driver
     to its boot-time disabled state.  Use only with the robot lifted/secured. */
  f413_hw_motor_set(true,
                    true,
                    true,
                    F413_HW_DIAG_MOTOR_BREAK_IN_DUTY,
                    F413_HW_DIAG_MOTOR_BREAK_IN_DUTY);
  trace_printf("[HW-TEST][Motor] BREAK-IN active: L/R forward duty=%u/1000; reset to stop\r\n",
               (unsigned int)F413_HW_DIAG_MOTOR_BREAK_IN_DUTY);
}

void f413_hw_diag_run_encoder_test_once(void)
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
               (unsigned long)F413_HW_DIAG_ENCODER_WINDOW_MS);
  HAL_Delay(F413_HW_DIAG_ENCODER_WINDOW_MS);
  l1 = __HAL_TIM_GET_COUNTER(&htim3);
  r1 = __HAL_TIM_GET_COUNTER(&htim4);
  dl = F413_HW_DIAG_ENCODER_SIGN_L * f413_hw_encoder_delta_signed(l1, l0);
  dr = F413_HW_DIAG_ENCODER_SIGN_R * f413_hw_encoder_delta_signed(r1, r0);

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
