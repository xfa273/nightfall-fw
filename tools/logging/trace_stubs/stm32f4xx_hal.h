#ifndef F413_TRACE_TEST_HAL_H
#define F413_TRACE_TEST_HAL_H
#include <stdint.h>
static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
#endif
