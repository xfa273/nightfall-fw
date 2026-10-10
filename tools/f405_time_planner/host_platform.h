/* Host-only HAL declarations. Production global/params/run/solver headers
 * remain in use; no HAL, UART, Flash or motor implementation is linked. */
#ifndef F405_HOST_PLATFORM_H
#define F405_HOST_PLATFORM_H
#include <stdint.h>
#define __MAIN_H
#define CCMRAM_ATTR
typedef struct { int unused; } TIM_HandleTypeDef;
typedef struct { int unused; } ADC_HandleTypeDef;
typedef int HAL_StatusTypeDef;
void HAL_Delay(uint32_t ms);
uint32_t HAL_GetTick(void);
#endif
