/* Intentionally distinct ON values prove runtime selection even though the
 * commissioned r3 profile starts with identical ON/OFF values. */
#define KP_VELOCITY_FAN_ON 0.7F
#define KI_VELOCITY_FAN_ON 0.02F
#define FF_TRANSLATION_STATIC_PWM_FAN_ON 40.0F
#define FF_TRANSLATION_VELOCITY_PWM_FAN_ON 0.1F
#define KP_DISTANCE_FAN_ON 5.0F
#define KP_ANGLE_FAN_ON 12.0F
#define KI_ANGLE_FAN_ON 0.2F
#define KP_OMEGA_FAN_ON 0.8F
#define KI_OMEGA_FAN_ON 0.03F
#define main f413_control_stop_regression_main
#include "f413_control_stop_tests.c"
#undef main

int main(void)
{
  for (unsigned on=0; on<2; ++on)
  {
    setup(); fan_active=on;
    assert(f413_ctrl_use_fan_on_gains() == (bool)on);
    f413_ctrl_set_velocity(100);
    tick_at_position(0);
    float expected = on
        ? FF_TRANSLATION_STATIC_PWM_FAN_ON + 100 * (FF_TRANSLATION_VELOCITY_PWM_FAN_ON + KP_VELOCITY_FAN_ON + KI_VELOCITY_FAN_ON)
        : FF_TRANSLATION_STATIC_PWM_FAN_OFF + 100 * (FF_TRANSLATION_VELOCITY_PWM_FAN_OFF + KP_VELOCITY_FAN_OFF + KI_VELOCITY_FAN_OFF);
    assert(f413_ctrl_get_motor_out_l() == lrintf(expected));
    assert(f413_ctrl_get_motor_out_r() == lrintf(expected));
    /* Translation gain tests must use the selected angle gains as well. */
    setup(); fan_active=on;
    f413_ctrl_tune_start(F413_CTRL_TUNE_AXIS_VELOCITY,0,F413_CTRL_TUNE_PATTERN_STEP);
    s_real_angle=2;
    tick_at_position(0);
    expected = -2 * (on ? KP_ANGLE_FAN_ON + KI_ANGLE_FAN_ON : KP_ANGLE_FAN_OFF + KI_ANGLE_FAN_OFF);
    assert(fabsf(f413_ctrl_get_target_omega()-expected) < .001f);
    f413_ctrl_tune_stop();
    setup(); fan_active=on;
    f413_ctrl_set_velocity(100);
    /* Turning the fan off selects OFF again without re-zeroing the robot. */
    fan_active=false;
    assert(!f413_ctrl_use_fan_on_gains() && s_running);
    assert(s_velocity_interrupt == 100);

    setup(); fan_active=on;
    tick_at_position(2);
    expected = -2 * (on ? KP_DISTANCE_FAN_ON : KP_DISTANCE_FAN_OFF);
    assert(fabsf(f413_ctrl_get_target_velocity()-expected) < .001f);

    setup(); fan_active=on;
    f413_ctrl_set_angle_target(0); s_real_angle=2;
    tick_at_position(0);
    expected = -2 * (on ? KP_ANGLE_FAN_ON + KI_ANGLE_FAN_ON : KP_ANGLE_FAN_OFF + KI_ANGLE_FAN_OFF);
    assert(fabsf(f413_ctrl_get_target_omega()-expected) < .001f);
    const float pwm = -expected * (on ? KP_OMEGA_FAN_ON + KI_OMEGA_FAN_ON : KP_OMEGA_FAN_OFF + KI_OMEGA_FAN_OFF);
    assert(f413_ctrl_get_motor_out_l() == lrintf(pwm));
    assert(f413_ctrl_get_motor_out_r() == -lrintf(pwm));
    assert(f413_ctrl_get_angle() == 2 && f413_ctrl_get_target_angle() == 0);
  }
  puts("PASS: fan state selects distinct ON/OFF velocity+FF, distance, angle and omega gains in production tick");
}
