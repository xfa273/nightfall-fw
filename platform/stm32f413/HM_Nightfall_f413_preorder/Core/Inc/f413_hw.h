#ifndef F413_HW_H_
#define F413_HW_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

void f413_hw_set_all_leds(GPIO_PinState state);
void f413_hw_show_mode_leds(uint8_t mode);
void f413_hw_buzzer_beep_ms(uint16_t period, uint16_t ms);
void f413_hw_op_beep_enter(void);
void f413_hw_boot_buzzer_pattern(void);
bool f413_hw_stop_switch_pressed(void);
GPIO_PinState f413_hw_stop_switch_raw(void);
int32_t f413_hw_encoder_delta_signed(uint32_t now, uint32_t prev);
void f413_hw_motor_set(bool enable,
                       bool left_forward,
                       bool right_forward,
                       uint16_t left_duty,
                       uint16_t right_duty);

#endif
