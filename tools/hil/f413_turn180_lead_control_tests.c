#define main control_stop_regression_main
#include "f413_control_stop_tests.c"
#undef main

static void check_tick(float lead)
{
  const float previous_ref = s_previous_omega_ref;
  const float previous_i = s_omega_integral;
  const float previous_error = s_previous_omega_error;
  tick_at_position(0);
  const float ref = f413_ctrl_get_target_omega();
  const float alpha = (ref - previous_ref) / F413_CTRL_DT;
  const float advanced = ref + fmaxf(-120, fminf(120, lead * alpha));
  const float error = s_real_omega - advanced;
  const float kp = fan_active ? KP_OMEGA_FAN_ON : KP_OMEGA_FAN_OFF;
  const float ki = fan_active ? KI_OMEGA_FAN_ON : KI_OMEGA_FAN_OFF;
  const float kd = fan_active ? KD_OMEGA_FAN_ON : KD_OMEGA_FAN_OFF;
  const float ff = fan_active ? FF_OMEGA_PWM_FAN_ON : FF_OMEGA_PWM_FAN_OFF;
  const float fa = fan_active ? FF_OMEGA_ACCEL_PWM_FAN_ON : FF_OMEGA_ACCEL_PWM_FAN_OFF;
  const float expected = -(ff * ref + fa * alpha) + kp * error + ki * (previous_i + error) +
      kd * (error - previous_error);
  assert(fabsf(s_omega_ref_lead - advanced) < .002f);
  assert(fabsf(s_omega_ref_accel - alpha) < .01f);
  assert(fabsf(s_omega_integral - (previous_i + error)) < .002f);
  assert(fabsf(s_out_rotate - expected) < .005f);
}

int main(void)
{
  for (unsigned on = 0; on < 2; ++on)
  for (unsigned active = 0; active < 2; ++active)
  for (int sign = -1; sign <= 1; sign += 2)
  {
    setup(); fan_active = on;
    f413_ctrl_set_omega(sign * 55);
    s_omega_integral = 123;
    s_previous_omega_ref = sign * 40;
    s_previous_omega_ref_valid = true;
    f413_ctrl_set_mode4_180_lead(active);
    assert(s_omega_integral == 123 && s_previous_omega_ref == sign * 40);
    assert(s_previous_omega_ref_valid && s_real_angle == 0 && s_target_angle == 0);
    const float lead = on && active && F413_MOTION_ENABLED(F413_MOTION_MODE4_180_LEAD) ?
        FF_OMEGA_LEAD_MODE4_180_TIME_S : FF_OMEGA_LEAD_TIME_S;
    check_tick(lead);
    /* Stopping the omega profile and starting its moving exit keep the context. */
    f413_ctrl_stop_omega_profile();
    f413_ctrl_set_velocity_profile(1400, 1400, 13);
    check_tick(lead);
    const float integral = s_omega_integral, previous = s_previous_omega_ref;
    f413_ctrl_set_mode4_180_lead(false);
    assert(integral == s_omega_integral && previous == s_previous_omega_ref);
    check_tick(FF_OMEGA_LEAD_TIME_S);
  }
  setup(); fan_active = true;
  f413_ctrl_set_mode4_180_lead(true);
  f413_ctrl_stop(); assert(!s_mode4_180_lead);
  f413_ctrl_set_mode4_180_lead(true);
  f413_ctrl_start(); assert(!s_mode4_180_lead);
  f413_ctrl_set_mode4_180_lead(true);
  f413_ctrl_tune_start(F413_CTRL_TUNE_AXIS_OMEGA, 0, F413_CTRL_TUNE_PATTERN_STEP);
  assert(!s_mode4_180_lead);
  printf("PASS: production tick lead/PI/FF in both directions, fan on/off, exit/history and start/stop/tune reset; policy=%#x\n",
      (unsigned)F413_MOTION_FEATURES);
  return 0;
}
