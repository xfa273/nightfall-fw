/*
 * interrupt.h
 *
 *  Created on: Feb 27, 2022
 *      Author: yuho-
 */

#ifndef INC_INTERRUPT_H_
#define INC_INTERRUPT_H_

#include "main.h"
#include <stdint.h>

#ifndef CCMRAM_ATTR
#define CCMRAM_ATTR __attribute__((section(".ccmram")))
#endif

// New logging system - define max log entries and structure
#define MAX_LOG_ENTRIES 1000

// Log data structure for collecting control and sensor data
typedef struct {
    uint16_t count;          // Sample count/index
    float target_omega;      // Target angular velocity
    float actual_omega;      // Actual angular velocity
    float p_term_omega;      // P-term of angular velocity control
    float i_term_omega;      // I-term of angular velocity control
    float d_term_omega;      // D-term of angular velocity control
    float motor_out_r;       // Right motor output
    float motor_out_l;       // Left motor output
    uint32_t timestamp;      // Timestamp (ms)
} LogEntry;

// Circular buffer for log data
typedef struct {
    LogEntry entries[MAX_LOG_ENTRIES];
    uint16_t head;           // Next position to write
    uint16_t count;          // Number of valid entries
    uint8_t logging_active;  // Flag indicating if logging is active
    uint32_t start_time;     // Timestamp when logging started
} LogBuffer;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void wait_ms(int time);
void wait_ms_non_block(int time);
int check_ms_passed(int time);
void wait_us(int time);
void tim1_wait_us(uint32_t us);

#ifdef MAIN_C_ // main.cからこのファイルが呼ばれている場合
volatile uint8_t ADC_task_counter; // ADCの振り分け用カウンタ
volatile LogBuffer log_buffer;     // 主ログ（速度/角速度 等）
CCMRAM_ATTR volatile LogBuffer log_buffer2;    // 副ログ（距離/角度 等）

#else // main.c以外からこのファイルが呼ばれている場合
extern volatile uint8_t ADC_task_counter; // ADCの振り分け用カウンタ
extern volatile LogBuffer log_buffer;     // 主ログ
extern CCMRAM_ATTR volatile LogBuffer log_buffer2;    // 副ログ

#endif

#endif /* INC_INTERRUPT_H_ */
