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
#include "params.h"
static uint32_t test_auto_video_capture;
#undef ENABLE_AUTO_VIDEO_CAPTURE
#define ENABLE_AUTO_VIDEO_CAPTURE test_auto_video_capture
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_hw.c"
#include "../../platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_op_ui.c"
TIM_HandleTypeDef htim2,htim10,htim11;
static bool capable=true, pressed, fail, active, control_running;
static unsigned pwm_starts, pwm_stops;
static uint32_t tick, delays[256];
static uint8_t delay_led_masks[256];
static unsigned delay_count, gpio_writes;
static uint8_t led_mask;
bool f413_machine_has(uint32_t c) { (void)c; return capable; }
int trace_printf(const char* format, ...) { (void)format; return 0; }
bool f413_ctrl_is_running(void) { return control_running; }
const f413_hardware_config_t* f413_machine_hardware(void)
{ static const f413_hardware_config_t config={0}; return &config; }
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms)
{
 assert(delay_count < 256);
 delay_led_masks[delay_count] = led_mask;
 delays[delay_count++] = ms;
 tick += ms;
}
GPIO_PinState HAL_GPIO_ReadPin(unsigned p,unsigned n)
{ (void)p; (void)n; return pressed ? GPIO_PIN_RESET : GPIO_PIN_SET; }
void HAL_GPIO_WritePin(unsigned p,unsigned n,GPIO_PinState s)
{
 uint8_t bit = 0;
 if (p == LED_1_GPIO_Port && n == LED_1_Pin) bit = F413_HW_LED_1_MASK;
 if (p == LED_2_GPIO_Port && n == LED_2_Pin) bit = F413_HW_LED_2_MASK;
 if (p == LED_3_GPIO_Port && n == LED_3_Pin) bit = F413_HW_LED_3_MASK;
 if (s == GPIO_PIN_SET) led_mask |= bit;
 else led_mask &= (uint8_t)~bit;
 gpio_writes++;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef* h,unsigned c)
{ (void)c; assert(h == &htim10); pwm_starts++; active=true; return fail ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef* h,unsigned c)
{ (void)c; assert(h == &htim10); pwm_stops++; active=false; return HAL_OK; }
int main(void)
{
 for (unsigned enabled = 0; enabled <= 1; ++enabled)
 {
   test_auto_video_capture = enabled;
   tick = delay_count = gpio_writes = 0;
   f413_hw_delay_before_run(2000U);
   assert(tick == 2000U && delay_count == 200U && led_mask == 0U);
   for (unsigned i = 0; i < delay_count; ++i)
   {
     assert(delays[i] == 10U);
     assert(delay_led_masks[i] == (((i * 10U / 250U) & 1U)
         ? (enabled ? F413_HW_LED_REAR_RIGHT_MASK : F413_HW_LED_1_MASK) : 0U));
   }
 }
 puts("pre-run LEDs: front blink when capture OFF, original rear blink ON, 2 s unchanged PASS");
 for (unsigned enabled = 0; enabled <= 1; ++enabled)
 for (unsigned drive = 0; drive <= 1; ++drive)
 for (unsigned fan = 0; fan <= 1; ++fan)
 {
   test_auto_video_capture = enabled;
   control_running = drive;
   s_fan_running = fan;
   tick = delay_count = gpio_writes = 0;
   led_mask = 5;
   f413_hw_show_run_failure();
   const bool flash = !enabled && !drive && !fan;
   assert(tick == (flash ? 1400U : 0U));
   assert(delay_count == (flash ? 20U : 0U));
   assert(gpio_writes == (flash ? 60U : 0U));
   assert(led_mask == (flash ? 0U : 5U));
   for (unsigned i = 0; i < delay_count; ++i)
   {
     assert(delays[i] == 70U);
     assert(delay_led_masks[i] == ((i & 1U) ? 0U : F413_HW_LED_1_MASK));
   }
 }
 control_running = s_fan_running = false;
 puts("run failure LEDs: ten flashes; capture ON/drive/fan active cause no writes or waits PASS");
 /* Every wall combination, invalid/saturated samples and both capture modes.
  * The enabled path must preserve even a partially lit optical/UI pattern. */
 for (unsigned enabled = 0; enabled <= 1; ++enabled)
 {
   test_auto_video_capture = enabled;
   for (unsigned valid = 0; valid <= 1; ++valid)
   for (unsigned walls = 0; walls < 8; ++walls)
   {
     led_mask = 5U;
     tick = delay_count = gpio_writes = 0;
     f413_hw_show_wall_status_leds(valid, walls & 1U, walls & 2U, walls & 4U);
     assert(led_mask == (enabled ? 5U : (valid ? walls : 0U)));
     assert(gpio_writes == (enabled ? 0U : 3U));
     assert(tick == 0 && delay_count == 0);
   }
 }
 puts("wall status LEDs: F/R/L mapping, invalid clear, capture ON preservation PASS");
 for (unsigned enabled = 0; enabled <= 1; ++enabled)
 {
   test_auto_video_capture = enabled;
   tick = s_next_ui_poll_ms = 0;
   f413_op_ui_lock_after_run_end();
   assert(led_mask == F413_HW_LED_REAR_RIGHT_MASK);
   for (unsigned phase = 1; phase <= 4; ++phase)
   {
     tick = phase * 400U;
     f413_op_ui_step(tick);
     assert(led_mask == ((phase & 1U) ? F413_HW_LED_REAR_LEFT_MASK
                                    : F413_HW_LED_REAR_RIGHT_MASK));
   }
   f413_op_ui_uart_push_once();
   f413_op_ui_uart_enter_once();
   assert(f413_op_ui_get_mode() == 0 && s_run_end_locked);
 }
 puts("run-end wait: alternating rear LEDs at 400 ms, input locked in both capture modes PASS");
 /* OFF must not alter the current UI LEDs or wait, including explicit UART
  * token tests. ON preserves the decoder's complete START/STOP waveform. */
 for (unsigned enabled = 0; enabled <= 1; ++enabled)
 {
   test_auto_video_capture = enabled;
   for (unsigned stop = 0; stop <= 1; ++stop)
   {
     tick = delay_count = gpio_writes = 0;
     if (stop) f413_hw_emit_video_sync_stop_pattern();
     else f413_hw_emit_video_sync_start_pattern();
     assert(tick == enabled * 9750U);
     assert(gpio_writes == enabled * 45U && delay_count == enabled * 16U);
     if (enabled)
     {
       assert(delays[0] == 2500 && delays[1] == 75 && delays[2] == 50);
       assert(delays[3] == 825 && delays[4] == 300 && delays[15] == 500);
       for (unsigned slot = 0; slot < 5; ++slot)
       {
         assert(delays[5 + 2 * slot] == (stop ? 800U : 350U));
         assert(delays[6 + 2 * slot] == (stop ? 300U : 750U));
       }
     }
   }
 }
 test_auto_video_capture = 0;
 gpio_writes = 0;
 f413_hw_show_mode_leds(3);
 assert(gpio_writes == 3); /* Ordinary UI LEDs are independent of capture. */
 puts("auto video GPIO: OFF no writes/delays, ON fixed-slot START/STOP, normal UI LEDs PASS");
 htim10.arr=999;
 assert(f413_hw_fan_start(500)); assert(active && f413_hw_fan_is_running() && htim10.compare[0]==500);
 f413_hw_fan_stop(); assert(!active && !f413_hw_fan_is_running() && htim10.compare[0]==0);
 assert(!f413_hw_fan_set_duty(500)); assert(!active);
 assert(f413_hw_fan_start(1));
 unsigned starts=pwm_starts, stops=pwm_stops;
 for (uint16_t duty=1; duty<=500; ++duty)
 {
   assert(f413_hw_fan_set_duty(duty));
   assert(active && f413_hw_fan_is_running() && htim10.compare[0]==duty);
   assert(pwm_starts==starts && pwm_stops==stops);
 }
 pressed=true; assert(!f413_hw_fan_set_duty(500)); pressed=false;
 assert(!active && !f413_hw_fan_is_running() && htim10.compare[0]==0);
 assert(f413_hw_fan_start(1));
 capable=false; assert(!f413_hw_fan_set_duty(500)); capable=true;
 assert(!active && !f413_hw_fan_is_running() && htim10.compare[0]==0);
 assert(f413_hw_fan_start(1)); assert(!f413_hw_fan_set_duty(1001));
 assert(!active && !f413_hw_fan_is_running() && htim10.compare[0]==0);
 assert(f413_hw_fan_start(1)); assert(!f413_hw_fan_set_duty(0));
 assert(!active && !f413_hw_fan_is_running() && htim10.compare[0]==0);
 capable=false; assert(!f413_hw_fan_start(500)); capable=true;
 pressed=true; assert(!f413_hw_fan_start(500)); pressed=false;
 assert(!f413_hw_fan_start(0)); assert(!f413_hw_fan_start(1001));
 fail=true; assert(!f413_hw_fan_start(500)); assert(!active && !f413_hw_fan_is_running() && htim10.compare[0]==0);
 puts("fan PWM: continuous ramp updates, 50 percent, capability/stop/range refusal and failed-start cleanup PASS");
}
