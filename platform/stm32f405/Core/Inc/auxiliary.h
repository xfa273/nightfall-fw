/*
 * auxiliary.h
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#ifndef INC_AUXILIARY_H_
#define INC_AUXILIARY_H_

#include <stdbool.h>
#include <stdint.h>

#define min(A, B) ((A) > (B)) ? (B) : (A)
#define max(A, B) ((A) > (B)) ? (A) : (B)

void led_write(bool, bool, bool);
void led_flash(uint8_t);
void led_wait(void);
void buzzer_beep(uint16_t);
void buzzer_interrupt(uint16_t);
void buzzer_enter(uint16_t);
int select_mode(int);
void __io_putchar(int);

#endif /* INC_AUXILIARY_H_ */
