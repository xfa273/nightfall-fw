#define main f413_control_stop_regression_main
#include "f413_control_stop_tests.c"
#undef main

int main(void)
{
  const int velocities[] = {-4500, -2400, -1500, -300, 300, 1500, 2400, 4500};
  for (unsigned fan = 0; fan < 2; ++fan)
    for (unsigned turning = 0; turning < 2; ++turning)
      for (unsigned v = 0; v < sizeof(velocities)/sizeof(velocities[0]); ++v)
      {
        setup();
        fan_active = fan;
        /* Include overspeed: measured feedback must not clip to the target. */
        const float target = velocities[v] > 0 ? 1500.0f : -1500.0f;
        f413_ctrl_set_velocity(target);
        if (turning) f413_ctrl_start_omega_profile(1000, 1, 1);
        for (unsigned tick = 0; tick < 1000; ++tick)
        {
          htim3.counter = F413_CTRL_ENCODER_CENTER + velocities[v] / 10;
          htim4.counter = F413_CTRL_ENCODER_CENTER - velocities[v] / 10;
          f413_ctrl_tick();
        }
        assert(fabsf(s_accel_velocity - velocities[v]) < .01f);
        assert(fabsf(f413_ctrl_get_real_velocity() - velocities[v]) < .01f);
        assert(f413_ctrl_get_target_velocity() == target);

      }
  puts("PASS: production velocity feedback, both signs, overspeed, fan and turn states");
}
