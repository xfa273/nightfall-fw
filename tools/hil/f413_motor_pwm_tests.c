#include "f413_motor_pwm.h"

#include <stdio.h>

#ifndef TEST_EXPECT_LEFT_FORWARD_HIGH
#error "Test must specify the expected left wiring independently"
#endif

int main(void)
{
  unsigned long checked = 0UL;

  if (NIGHTFALL_F413_MOTOR_LEFT_FORWARD_IN2_HIGH != TEST_EXPECT_LEFT_FORWARD_HIGH)
  {
    fputs("Unexpected default/configured left motor wiring\n", stderr);
    return 1;
  }

  for (unsigned int side = 0U; side < 2U; ++side)
  {
    const bool left = (side == 0U);
    for (unsigned int direction = 0U; direction < 2U; ++direction)
    {
      const bool forward = (direction != 0U);
      /* Independent oracle: original mapping, with only left leads reversed. */
      const bool legacy_forward = (left && TEST_EXPECT_LEFT_FORWARD_HIGH) ?
          !forward : forward;
      const bool expected_high = left ? !legacy_forward : legacy_forward;

      for (uint32_t duty = 0U; duty <= UINT16_MAX; ++duty)
      {
        const f413_motor_pwm_command_t got = f413_motor_pwm_encode(
            left, forward, (uint16_t)duty);
        const uint16_t bounded = (uint16_t)((duty > 1000U) ? 1000U : duty);
        const bool high = (duty != 0U) && expected_high;
        const uint16_t compare = high ? (uint16_t)(1000U - bounded) : bounded;

        if ((got.in2_high != high) || (got.compare != compare))
        {
          fprintf(stderr, "FAIL left=%u forward=%u duty=%lu: CCR=%u IN2=%u\n",
                  (unsigned int)left, (unsigned int)forward,
                  (unsigned long)duty, (unsigned int)got.compare,
                  (unsigned int)got.in2_high);
          return 1;
        }
        ++checked;
      }
    }
  }

  printf("PASS: %lu motor PWM cases, left-forward-IN2-high=%d\n",
         checked, NIGHTFALL_F413_MOTOR_LEFT_FORWARD_IN2_HIGH);
  return 0;
}
