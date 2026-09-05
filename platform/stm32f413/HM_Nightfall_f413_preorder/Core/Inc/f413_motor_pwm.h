#ifndef F413_MOTOR_PWM_H_
#define F413_MOTOR_PWM_H_

#include <stdbool.h>
#include <stdint.h>

/* Preserve the existing duty scale and complementary compare convention. */
#define F413_MOTOR_PWM_MAX (1000U)

/*
 * Current bring-up wiring: mini r3, left motor leads swapped on 2026-09-06.
 * Both motors now use IN2 high / complementary IN1 PWM for physical forward.
 * For the original mini r2 / unswapped mini r3 wiring, compile ALL F413
 * application sources with NIGHTFALL_F413_MOTOR_LEFT_FORWARD_IN2_HIGH=0.
 * This is a temporary build-time board setting, not NVM identity selection.
 * Encoder polarity, logical output signs, and right-motor wiring are unchanged.
 */
#ifndef NIGHTFALL_F413_MOTOR_LEFT_FORWARD_IN2_HIGH
#define NIGHTFALL_F413_MOTOR_LEFT_FORWARD_IN2_HIGH 1
#endif

#if (NIGHTFALL_F413_MOTOR_LEFT_FORWARD_IN2_HIGH != 0) && \
    (NIGHTFALL_F413_MOTOR_LEFT_FORWARD_IN2_HIGH != 1)
#error "NIGHTFALL_F413_MOTOR_LEFT_FORWARD_IN2_HIGH must be 0 or 1"
#endif

typedef struct
{
  uint16_t compare;
  bool in2_high;
} f413_motor_pwm_command_t;

/* Pure mapping shared by open-loop diagnostics and the 1kHz controller. */
static inline f413_motor_pwm_command_t f413_motor_pwm_encode(bool left_motor,
                                                            bool forward,
                                                            uint16_t duty)
{
  f413_motor_pwm_command_t command = {0U, false};
  const bool forward_in2_high = left_motor ?
      (NIGHTFALL_F413_MOTOR_LEFT_FORWARD_IN2_HIGH != 0) : true;
  const uint16_t bounded_duty = (duty > F413_MOTOR_PWM_MAX) ?
      F413_MOTOR_PWM_MAX : duty;

  /* Zero always keeps both inputs low, regardless of requested direction. */
  if (bounded_duty != 0U)
  {
    command.in2_high = (forward == forward_in2_high);
    command.compare = command.in2_high ?
        (uint16_t)(F413_MOTOR_PWM_MAX - bounded_duty) : bounded_duty;
  }
  return command;
}

#endif
