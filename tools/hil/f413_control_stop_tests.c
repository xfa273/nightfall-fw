#include <assert.h>
#include <stdio.h>
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_control.c"

TIM_HandleTypeDef htim2, htim3, htim4, htim5;
SPI_HandleTypeDef hspi2;
static const f413_hardware_config_t hardware = {
  .encoder_cpr = 4096, .encoder_sign_l = 1, .encoder_sign_r = -1, .tread_mm = 36.0f
};
const f413_hardware_config_t* f413_machine_hardware(void) { return &hardware; }
bool f413_machine_has(uint32_t capability) { (void)capability; return true; }
void HAL_GPIO_WritePin(unsigned port, unsigned pin, GPIO_PinState state)
{ (void)port; (void)pin; (void)state; }
void HAL_Delay(uint32_t ms) { (void)ms; }
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef* h, unsigned c)
{ (void)h; (void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef* h)
{ (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef* h, unsigned c)
{ (void)h; (void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef* h, unsigned c)
{ (void)h; (void)c; return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef* h, uint8_t* tx,
                                       uint8_t* rx, uint16_t n, uint32_t timeout)
{ (void)h; (void)tx; (void)timeout; memset(rx, 0, n); return HAL_OK; }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef* h, uint8_t* tx,
                                 uint16_t n, uint32_t timeout)
{ (void)h; (void)tx; (void)n; (void)timeout; return HAL_OK; }

static void setup(void)
{
  f413_ctrl_reset_pid_state();
  f413_ctrl_reset_profile_state();
  f413_ctrl_reset_distance();
  s_running = true;
  s_imu_ok = false;
  s_enc_to_mm = 0.01f;
  htim3.counter = htim4.counter = F413_CTRL_ENCODER_CENTER;
}

static void tick_at_position(float mm)
{
  s_encoder_distance_l = s_encoder_distance_r = mm;
  f413_ctrl_tick();
}

int main(void)
{
  setup();
  f413_ctrl_set_velocity_profile(331.662f, 0, 45);
  assert(test_primask == 0U);
  for (unsigned i = 0; i < 300; ++i) tick_at_position(41.527f);
  assert(f413_ctrl_stop_profile_complete());
  assert(s_acceleration_interrupt == 0.0f && s_velocity_interrupt == 0.0f);
  assert(fabsf(f413_ctrl_get_target_distance() - 45.0f) < 0.001f);
  assert(fabsf(f413_ctrl_get_target_velocity() - 6.946f) < 0.01f);
  /* Feedforward now supports the small forward correction, never stale braking. */
  assert(f413_ctrl_translation_ff_output(s_target_velocity, s_acceleration_interrupt,
                                        35, .035f, .004f) > 35.0f);
  s_velocity_integral = 0.0f;
  tick_at_position(41.527f);
  assert(f413_ctrl_get_motor_out_l() > 0 && f413_ctrl_get_motor_out_r() > 0);
  tick_at_position(0.0f);
  assert(f413_ctrl_get_target_velocity() == F413_STOP_CORRECTION_MAX_MM_S);
  tick_at_position(46.0f);
  assert(f413_ctrl_get_target_velocity() < 0.0f);
  f413_ctrl_set_velocity(0);
  assert(test_primask == 0U);
  tick_at_position(41.527f);
  assert(!f413_ctrl_stop_profile_complete() && f413_ctrl_get_target_velocity() == 0);

  setup();
  tick_at_position(12.0f);
  s_velocity_integral = 100000.0f; /* Prior forward pushing must not defeat retreat. */
  f413_ctrl_set_velocity(-30);
  f413_ctrl_clear_velocity_feedback();
  assert(s_velocity_integral == 0 && f413_ctrl_get_distance() == 12.0f && s_running);
  assert(test_primask == 0U);
  tick_at_position(12.0f);
  assert(f413_ctrl_get_motor_out_l() < 0 && f413_ctrl_get_motor_out_r() < 0);
  setup();
  f413_ctrl_set_velocity_profile(600, 300, 135);
  for (unsigned i = 0; i < 500; ++i) f413_ctrl_tick();
  assert(s_velocity_interrupt == 300 && s_acceleration_interrupt == 0);
  assert(!f413_ctrl_stop_profile_complete());
  assert(f413_ctrl_get_target_velocity() >= 300); /* Preserve deceleration clamp direction. */
  setup();
  f413_ctrl_set_velocity_profile(0, 300, 45);
  for (unsigned i = 0; i < 500; ++i) f413_ctrl_tick();
  assert(s_velocity_interrupt == 300 && s_acceleration_interrupt == 0);
  assert(f413_ctrl_get_target_velocity() <= 300);
  puts("PASS: production control tick, exact stop endpoint, bounded correction, FF sign, nonzero clamps");
  return 0;
}
