#ifndef F413_CONTROL_TEST_MAIN_H
#define F413_CONTROL_TEST_MAIN_H
#include <stdint.h>
#include "stm32f4xx_hal.h"
typedef struct { uint32_t counter, compare[4]; } TIM_HandleTypeDef;
typedef struct { unsigned State; } SPI_HandleTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
static uint32_t test_primask;
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void __disable_irq(void) { test_primask = 1U; }
static inline void __set_PRIMASK(uint32_t mask) { test_primask = mask; }
#define HAL_SPI_STATE_READY 0U
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_3 2U
#define TIM_CHANNEL_ALL 3U
#define FRAM_CS_GPIO_Port 0U
#define FRAM_CS_Pin 0U
#define IMU_CS_GPIO_Port 0U
#define IMU_CS_Pin 1U
#define MOTOR_STBY_GPIO_Port 0U
#define MOTOR_STBY_Pin 2U
#define MOTOR_L_DIR_GPIO_Port 0U
#define MOTOR_L_DIR_Pin 3U
#define MOTOR_R_DIR_GPIO_Port 0U
#define MOTOR_R_DIR_Pin 4U
#define __HAL_TIM_GET_COUNTER(h) ((h)->counter)
#define __HAL_TIM_SET_COUNTER(h, n) ((h)->counter = (n))
#define __HAL_TIM_SET_COMPARE(h, c, n) ((h)->compare[c] = (n))
void HAL_GPIO_WritePin(unsigned port, unsigned pin, GPIO_PinState state);
void HAL_Delay(uint32_t ms);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef* h, unsigned c);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef* h);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef* h, unsigned c);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef* h, unsigned c);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef* h, uint8_t* tx,
                                       uint8_t* rx, uint16_t n, uint32_t timeout);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef* h, uint8_t* tx,
                                 uint16_t n, uint32_t timeout);
#endif
