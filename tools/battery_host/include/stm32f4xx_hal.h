#ifndef STM32F4XX_HAL_H_
#define STM32F4XX_HAL_H_

#include <stdint.h>

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t delay_ms);

#endif
