#ifndef F413_MOTOR_PWM_H_
#define F413_MOTOR_PWM_H_

#include <stdbool.h>
#include <stdint.h>
#include "f413_machine.h"

/* Preserve the existing duty scale and complementary compare convention. */
#define F413_MOTOR_PWM_MAX (1000U)

/* Direction is selected from model + unit identity, never a build-time override. */
#ifdef NIGHTFALL_F413_MOTOR_LEFT_FORWARD_IN2_HIGH
#error "Obsolete wiring define: use board/f413/f413_registry.c unit settings"
#endif

typedef struct
{
  uint16_t compare;
  bool in2_high;
} f413_motor_pwm_command_t;

/* Pure arithmetic kernel; host tests cover both IN2 forward polarities. */
static inline f413_motor_pwm_command_t f413_motor_pwm_encode_polarity(bool forward_in2_high,
                                                                     bool forward,
                                                                     uint16_t duty)
{
  f413_motor_pwm_command_t command = {0U, false};
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

/* Shared by open-loop diagnostics and the 1kHz controller. */
static inline f413_motor_pwm_command_t f413_motor_pwm_encode(bool left_motor,
                                                            bool forward,
                                                            uint16_t duty)
{
  if (!f413_machine_has(F413_CAP_DRIVE))
  {
    const f413_motor_pwm_command_t stopped = {0U, false};
    return stopped;
  }
  const f413_hardware_config_t *hw = f413_machine_hardware();
  return f413_motor_pwm_encode_polarity(left_motor ? hw->left_forward_in2_high :
      hw->right_forward_in2_high, forward, duty);
}

#endif
