#include "f413_hw.h"
#include "f413_motor_pwm.h"

#include "main.h"

#define F413_HW_ENCODER_WRAP_COUNT (60000L)
#define F413_HW_ENCODER_WRAP_HALF (F413_HW_ENCODER_WRAP_COUNT / 2L)
#define F413_HW_VIDEO_SYNC_OFF_PREAMBLE_MS (2500U)
#define F413_HW_VIDEO_SYNC_SYNC_FIRST_ON_MS (75U)
#define F413_HW_VIDEO_SYNC_SYNC_REEDGE_OFF_MS (50U)
#define F413_HW_VIDEO_SYNC_SYNC_SECOND_ON_MS (825U)
#define F413_HW_VIDEO_SYNC_SYNC_GAP_MS (300U)
#define F413_HW_VIDEO_SYNC_PAYLOAD_SLOTS (5U)
#define F413_HW_VIDEO_SYNC_PAYLOAD_SLOT_MS (1100U)
#define F413_HW_VIDEO_SYNC_START_ON_MS (350U)
#define F413_HW_VIDEO_SYNC_STOP_ON_MS (800U)
#define F413_HW_VIDEO_SYNC_FINAL_OFF_MS (500U)

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim11;

static volatile uint16_t g_buzzer_async_remaining_ms = 0U;

static void f413_hw_buzzer_async_stop(void)
{
  g_buzzer_async_remaining_ms = 0U;
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0U);
  (void)HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
}

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

static void f413_hw_emit_video_sync_pattern(uint32_t payload_on_ms)
{
  uint8_t slot;
  uint32_t payload_off_ms = F413_HW_VIDEO_SYNC_PAYLOAD_SLOT_MS - payload_on_ms;

  /*
   * The long OFF preamble separates a run token from OP-UI LED indications.
   * A fixed SYNC pulse then anchors five equal-width payload slots. START uses
   * a short ON interval in every slot; STOP uses a long ON interval. Because
   * both tokens have the same slot count and differ by pulse width, a STOP
   * token can never be accepted as a prefix-compatible START token.
   */
  f413_hw_show_led_mask(0U);
  HAL_Delay(F413_HW_VIDEO_SYNC_OFF_PREAMBLE_MS);

  /*
   * Present two rising edges while remaining one logical 950 ms SYNC pulse.
   * The 50 ms notch is shorter than the Pixel decoder's 175 ms LOW confirm,
   * so a receiver that saw the first edge treats the pulse as continuous.  A
   * receiver that missed it can still acquire the independent 825 ms second
   * edge, which remains inside the accepted 650--1150 ms SYNC width.
   */
  f413_hw_set_all_leds(GPIO_PIN_SET);
  HAL_Delay(F413_HW_VIDEO_SYNC_SYNC_FIRST_ON_MS);
  f413_hw_show_led_mask(0U);
  HAL_Delay(F413_HW_VIDEO_SYNC_SYNC_REEDGE_OFF_MS);
  f413_hw_set_all_leds(GPIO_PIN_SET);
  HAL_Delay(F413_HW_VIDEO_SYNC_SYNC_SECOND_ON_MS);
  f413_hw_show_led_mask(0U);
  HAL_Delay(F413_HW_VIDEO_SYNC_SYNC_GAP_MS);

  for (slot = 0U; slot < F413_HW_VIDEO_SYNC_PAYLOAD_SLOTS; slot++)
  {
    f413_hw_set_all_leds(GPIO_PIN_SET);
    HAL_Delay(payload_on_ms);
    f413_hw_show_led_mask(0U);
    HAL_Delay(payload_off_ms);
  }

  HAL_Delay(F413_HW_VIDEO_SYNC_FINAL_OFF_MS);
}

void f413_hw_emit_video_sync_start_pattern(void)
{
  f413_hw_emit_video_sync_pattern(F413_HW_VIDEO_SYNC_START_ON_MS);
}

void f413_hw_emit_video_sync_stop_pattern(void)
{
  f413_hw_emit_video_sync_pattern(F413_HW_VIDEO_SYNC_STOP_ON_MS);
}

void f413_hw_buzzer_beep_ms(uint16_t period, uint16_t ms)
{
  if (g_buzzer_async_remaining_ms != 0U)
  {
    f413_hw_buzzer_async_stop();
  }
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

void f413_hw_buzzer_beep_async(uint16_t period, uint16_t ms)
{
  if ((period == 0U) || (ms == 0U))
  {
    return;
  }
  if (g_buzzer_async_remaining_ms != 0U)
  {
    f413_hw_buzzer_async_stop();
  }

  __HAL_TIM_SET_AUTORELOAD(&htim11, period);
  __HAL_TIM_SET_COUNTER(&htim11, 0U);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, (period * 6U) / 10U);
  if (HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1) != HAL_OK)
  {
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0U);
    return;
  }
  g_buzzer_async_remaining_ms = ms;
}

void f413_hw_buzzer_tick_1ms(void)
{
  if (g_buzzer_async_remaining_ms == 0U)
  {
    return;
  }
  g_buzzer_async_remaining_ms--;
  if (g_buzzer_async_remaining_ms == 0U)
  {
    f413_hw_buzzer_async_stop();
  }
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
  const f413_motor_pwm_command_t left = f413_motor_pwm_encode(true, left_forward, left_duty);
  const f413_motor_pwm_command_t right = f413_motor_pwm_encode(false, right_forward, right_duty);

  if (!enable || !f413_machine_has(F413_CAP_DRIVE))
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

  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  if (left.in2_high)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left.compare);
    HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_L_DIR_GPIO_Port, MOTOR_L_DIR_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left.compare);
  }

  if (right.in2_high)
  {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right.compare);
    HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_R_DIR_GPIO_Port, MOTOR_R_DIR_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right.compare);
  }

  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
}
