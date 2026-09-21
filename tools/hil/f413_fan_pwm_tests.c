#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t arr, compare[3], counter; } TIM_HandleTypeDef;
#define GPIOA 0U
#define GPIOB 1U
#define GPIOC 2U
#define GPIOH 7U
#define GPIO_PIN_0 (1U << 0)
#define GPIO_PIN_1 (1U << 1)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_3 (1U << 3)
#define GPIO_PIN_4 (1U << 4)
#define GPIO_PIN_5 (1U << 5)
#define GPIO_PIN_6 (1U << 6)
#define GPIO_PIN_7 (1U << 7)
#define GPIO_PIN_8 (1U << 8)
#define GPIO_PIN_9 (1U << 9)
#define GPIO_PIN_10 (1U << 10)
#define GPIO_PIN_11 (1U << 11)
#define GPIO_PIN_12 (1U << 12)
#define GPIO_PIN_13 (1U << 13)
#define GPIO_PIN_14 (1U << 14)
#define GPIO_PIN_15 (1U << 15)
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_3 2U
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->arr)
#define __HAL_TIM_SET_AUTORELOAD(h,v) ((h)->arr=(v))
#define __HAL_TIM_SET_COMPARE(h,c,v) ((h)->compare[c]=(v))
#define __HAL_TIM_SET_COUNTER(h,v) ((h)->counter=(v))
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
GPIO_PinState HAL_GPIO_ReadPin(unsigned port,unsigned pin);
void HAL_GPIO_WritePin(unsigned port,unsigned pin,GPIO_PinState s);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef* h,unsigned c);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef* h,unsigned c);
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_hw.c"
TIM_HandleTypeDef htim2,htim10,htim11;
static bool capable=true, pressed, fail, active;
static unsigned pwm_starts, pwm_stops;
bool f413_machine_has(uint32_t c) { (void)c; return capable; }
const f413_hardware_config_t* f413_machine_hardware(void)
{ static const f413_hardware_config_t config={0}; return &config; }
uint32_t HAL_GetTick(void) { return 0; }
void HAL_Delay(uint32_t ms) { (void)ms; }
GPIO_PinState HAL_GPIO_ReadPin(unsigned p,unsigned n)
{ (void)p; (void)n; return pressed ? GPIO_PIN_RESET : GPIO_PIN_SET; }
void HAL_GPIO_WritePin(unsigned p,unsigned n,GPIO_PinState s) { (void)p; (void)n; (void)s; }
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef* h,unsigned c)
{ (void)c; assert(h == &htim10); pwm_starts++; active=true; return fail ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef* h,unsigned c)
{ (void)c; assert(h == &htim10); pwm_stops++; active=false; return HAL_OK; }
int main(void)
{
 htim10.arr=999;
 assert(f413_hw_fan_start(500)); assert(active && htim10.compare[0]==500);
 f413_hw_fan_stop(); assert(!active && htim10.compare[0]==0);
 assert(!f413_hw_fan_set_duty(500)); assert(!active);
 assert(f413_hw_fan_start(1));
 unsigned starts=pwm_starts, stops=pwm_stops;
 for (uint16_t duty=1; duty<=500; ++duty)
 {
   assert(f413_hw_fan_set_duty(duty));
   assert(active && htim10.compare[0]==duty);
   assert(pwm_starts==starts && pwm_stops==stops);
 }
 pressed=true; assert(!f413_hw_fan_set_duty(500)); pressed=false;
 assert(!active && htim10.compare[0]==0);
 assert(f413_hw_fan_start(1));
 capable=false; assert(!f413_hw_fan_set_duty(500)); capable=true;
 assert(!active && htim10.compare[0]==0);
 assert(f413_hw_fan_start(1)); assert(!f413_hw_fan_set_duty(1001));
 assert(!active && htim10.compare[0]==0);
 assert(f413_hw_fan_start(1)); assert(!f413_hw_fan_set_duty(0));
 assert(!active && htim10.compare[0]==0);
 capable=false; assert(!f413_hw_fan_start(500)); capable=true;
 pressed=true; assert(!f413_hw_fan_start(500)); pressed=false;
 assert(!f413_hw_fan_start(0)); assert(!f413_hw_fan_start(1001));
 fail=true; assert(!f413_hw_fan_start(500)); assert(!active && htim10.compare[0]==0);
 puts("fan PWM: continuous ramp updates, 50 percent, capability/stop/range refusal and failed-start cleanup PASS");
}
