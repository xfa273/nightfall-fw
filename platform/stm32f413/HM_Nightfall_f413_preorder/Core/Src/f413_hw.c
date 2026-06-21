#include "f413_hw.h"

#include "main.h"

#define F413_HW_ENCODER_WRAP_COUNT (60000L)
#define F413_HW_ENCODER_WRAP_HALF (F413_HW_ENCODER_WRAP_COUNT / 2L)
#define F413_HW_MOTOR_PWM_MAX (1000U)

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim11;

void f413_hw_set_all_leds(GPIO_PinState state)
{
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, state);
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, state);
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, state);
}

void f413_hw_show_led_mask(uint8_t mask)
{
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin,
                    ((mask & F413_HW_LED_1_MASK) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin,
                    ((mask & F413_HW_LED_2_MASK) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin,
                    ((mask & F413_HW_LED_3_MASK) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void f413_hw_show_led_blink(uint8_t mask, uint32_t now_ms, uint32_t toggle_ms)
{
  uint8_t blink_mask = mask;

  if (toggle_ms == 0U)
  {
    toggle_ms = F413_HW_LED_BLINK_TOGGLE_MS;
  }
  if (((now_ms / toggle_ms) & 0x01U) == 0U)
  {
    blink_mask = 0U;
  }

  f413_hw_show_led_mask(blink_mask);
}

void f413_hw_delay_with_led_blink(uint8_t mask, uint32_t duration_ms, uint32_t toggle_ms)
{
  uint32_t deadline = HAL_GetTick() + duration_ms;

  while ((int32_t)(HAL_GetTick() - deadline) < 0)
  {
    f413_hw_show_led_blink(mask, HAL_GetTick(), toggle_ms);
    HAL_Delay(10U);
  }
  f413_hw_show_led_mask(0U);
}

void f413_hw_show_mode_leds(uint8_t mode)
{
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, (mode & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, (mode & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, (mode & 0x04U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void f413_hw_buzzer_beep_ms(uint16_t period, uint16_t ms)
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

void f413_hw_op_beep_enter(void)
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

void f413_hw_boot_buzzer_pattern(void)
{
  f413_hw_buzzer_beep_ms(1800U, 70U);
  HAL_Delay(35U);
  f413_hw_buzzer_beep_ms(1300U, 70U);
  HAL_Delay(35U);
  f413_hw_buzzer_beep_ms(850U, 90U);
  HAL_Delay(120U);
  f413_hw_buzzer_beep_ms(1450U, 70U);
  HAL_Delay(35U);
  f413_hw_buzzer_beep_ms(1050U, 110U);
}

bool f413_hw_stop_switch_pressed(void)
{
  return (HAL_GPIO_ReadPin(PUSH_IN_1_GPIO_Port, PUSH_IN_1_Pin) == GPIO_PIN_RESET);
}

GPIO_PinState f413_hw_stop_switch_raw(void)
{
  return HAL_GPIO_ReadPin(PUSH_IN_1_GPIO_Port, PUSH_IN_1_Pin);
}

int32_t f413_hw_encoder_delta_signed(uint32_t now, uint32_t prev)
{
  int32_t delta = (int32_t)now - (int32_t)prev;
  if (delta > F413_HW_ENCODER_WRAP_HALF)
  {
    delta -= F413_HW_ENCODER_WRAP_COUNT;
  }
  else if (delta < -F413_HW_ENCODER_WRAP_HALF)
  {
    delta += F413_HW_ENCODER_WRAP_COUNT;
  }
  return delta;
}

void f413_hw_motor_set(bool enable,
                       bool left_forward,
                       bool right_forward,
                       uint16_t left_duty,
                       uint16_t right_duty)
{
  uint16_t l_duty = (left_duty > F413_HW_MOTOR_PWM_MAX) ? F413_HW_MOTOR_PWM_MAX : left_duty;
  uint16_t r_duty = (right_duty > F413_HW_MOTOR_PWM_MAX) ? F413_HW_MOTOR_PWM_MAX : right_duty;
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
      l_compare = (uint16_t)(F413_HW_MOTOR_PWM_MAX - l_duty);
      l_in2 = GPIO_PIN_SET;
    }
  }

  if (r_duty != 0U)
  {
    if (right_forward)
    {
      r_compare = (uint16_t)(F413_HW_MOTOR_PWM_MAX - r_duty);
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
