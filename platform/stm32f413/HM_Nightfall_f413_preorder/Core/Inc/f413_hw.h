#ifndef F413_HW_H_
#define F413_HW_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#define F413_HW_LED_1_MASK (0x01U)
#define F413_HW_LED_2_MASK (0x02U)
#define F413_HW_LED_3_MASK (0x04U)
#define F413_HW_LED_REAR_RIGHT_MASK F413_HW_LED_2_MASK
#define F413_HW_LED_REAR_LEFT_MASK  F413_HW_LED_3_MASK
#define F413_HW_LED_BLINK_TOGGLE_MS (250U)

void f413_hw_set_all_leds(GPIO_PinState state);
void f413_hw_show_led_mask(uint8_t mask);
void f413_hw_show_led_blink(uint8_t mask, uint32_t now_ms, uint32_t toggle_ms);
void f413_hw_delay_with_led_blink(uint8_t mask, uint32_t duration_ms, uint32_t toggle_ms);
void f413_hw_show_mode_leds(uint8_t mode);
void f413_hw_buzzer_beep_ms(uint16_t period, uint16_t ms);
void f413_hw_buzzer_beep_async(uint16_t period, uint16_t ms);
void f413_hw_buzzer_tick_1ms(void);
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
