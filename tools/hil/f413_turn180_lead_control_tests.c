#include "params.h"
/* Compile the same real controller with each independent experiment disabled. */
#ifdef TEST_MODE4_POLICY
#undef F413_MOTION_FEATURES
#define F413_MOTION_FEATURES TEST_MODE4_POLICY
#endif
#define main control_stop_regression_main
#include "f413_control_stop_tests.c"
#undef main

static void check_tick(float lead)
{
  const float previous_ref = s_previous_omega_ref;
  const float previous_i = s_omega_integral;
  const float previous_error = s_previous_omega_error;
  const float previous_plan = s_previous_omega_plan;
  const bool plan_valid = s_previous_omega_plan_valid;
  const bool split = fan_active && s_mode4_180_turn &&
      F413_MOTION_ENABLED(F413_MOTION_MODE4_180_TRAJECTORY_FF);
  tick_at_position(0);
  const float ref = f413_ctrl_get_target_omega();
  const float alpha = (ref - previous_ref) / F413_CTRL_DT;
  const float advanced = ref + (split ? 0.0f : fmaxf(-120, fminf(120, lead * alpha)));
  const float plan_alpha = plan_valid ? (s_omega_interrupt - previous_plan) / F413_CTRL_DT : 0.0f;
  const float error = s_real_omega - advanced;
  const float kp = fan_active ? KP_OMEGA_FAN_ON : KP_OMEGA_FAN_OFF;
  const float ki = fan_active ? KI_OMEGA_FAN_ON : KI_OMEGA_FAN_OFF;
  const float kd = fan_active ? KD_OMEGA_FAN_ON : KD_OMEGA_FAN_OFF;
  const float ff = fan_active ? FF_OMEGA_PWM_FAN_ON : FF_OMEGA_PWM_FAN_OFF;
  const float fa = fan_active ? FF_OMEGA_ACCEL_PWM_FAN_ON : FF_OMEGA_ACCEL_PWM_FAN_OFF;
  const float feedforward = split ?
      FF_OMEGA_TRAJECTORY_PWM_MODE4_180 * s_omega_interrupt +
          FF_OMEGA_TRAJECTORY_ACCEL_PWM_MODE4_180 * plan_alpha : ff * ref + fa * alpha;
  const float expected = -feedforward + kp * error + ki * (previous_i + error) +
      kd * (error - previous_error);
  assert(fabsf(s_omega_ref_lead - advanced) < .002f);
  assert(fabsf(s_omega_ref_accel - alpha) < .01f);
  assert(fabsf(s_omega_plan_accel - plan_alpha) < .01f);
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
    f413_ctrl_set_mode4_180_turn(active);
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
    f413_ctrl_set_mode4_180_turn(false);
    assert(integral == s_omega_integral && previous == s_previous_omega_ref);
    check_tick(FF_OMEGA_LEAD_TIME_S);
  }
  /* Feedback and wall-heading steps must not enter trajectory FF or lead. */
  for (int sign = -1; sign <= 1; sign += 2) {
    setup(); fan_active = true;
    f413_ctrl_set_mode4_180_turn(true);
    f413_ctrl_set_omega(0);
    tick_at_position(0);
    s_real_angle = sign * 0.5f;
    f413_ctrl_set_heading_omega_correction(sign * 20);
    const float lead = F413_MOTION_ENABLED(F413_MOTION_MODE4_180_LEAD) ?
        FF_OMEGA_LEAD_MODE4_180_TIME_S : FF_OMEGA_LEAD_TIME_S;
    check_tick(lead);
    assert(s_omega_plan_accel == 0 && s_omega_interrupt == 0);
    if (F413_MOTION_ENABLED(F413_MOTION_MODE4_180_TRAJECTORY_FF))
      assert(s_omega_ref_lead == f413_ctrl_get_target_omega());
  }
  /* Preserve the final planned deceleration across the core->exit boundary. */
  setup(); fan_active = true;
  f413_ctrl_set_mode4_180_turn(true);
  f413_ctrl_set_omega(-1);
  tick_at_position(0);
  const float saved_plan = s_previous_omega_plan;
  f413_ctrl_stop_omega_profile();
  assert(s_previous_omega_plan == saved_plan);
  check_tick(F413_MOTION_ENABLED(F413_MOTION_MODE4_180_LEAD) ?
      FF_OMEGA_LEAD_MODE4_180_TIME_S : FF_OMEGA_LEAD_TIME_S);
  assert(fabsf(s_omega_plan_accel - 1000) < .01f);
  check_tick(F413_MOTION_ENABLED(F413_MOTION_MODE4_180_LEAD) ?
      FF_OMEGA_LEAD_MODE4_180_TIME_S : FF_OMEGA_LEAD_TIME_S);
  assert(s_omega_plan_accel == 0);
  setup(); fan_active = true;
  f413_ctrl_set_mode4_180_turn(true);
  f413_ctrl_stop(); assert(!s_mode4_180_turn);
  f413_ctrl_set_mode4_180_turn(true);
  f413_ctrl_start(); assert(!s_mode4_180_turn);
  f413_ctrl_set_mode4_180_turn(true);
  f413_ctrl_tune_start(F413_CTRL_TUNE_AXIS_OMEGA, 0, F413_CTRL_TUNE_PATTERN_STEP);
  assert(!s_mode4_180_turn);
  printf("PASS: production tick lead/trajectory-FF/PI/D, feedback isolation, exit/history and start/stop/tune reset; policy=%#x\n",
      (unsigned)F413_MOTION_FEATURES);
  return 0;
}
