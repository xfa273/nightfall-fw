#include "f413_hw_diag.h"

#include <stdint.h>

#include "f413_hw.h"
#include "stm32f4xx_hal.h"
#include "trace.h"

#define F413_HW_DIAG_ENCODER_WINDOW_MS (10000U)
#define F413_HW_DIAG_LED_ON_WINDOW_MS (30000U)
#define F413_HW_DIAG_ENCODER_SIGN_L (1L)
#define F413_HW_DIAG_ENCODER_SIGN_R (-1L)

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

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
