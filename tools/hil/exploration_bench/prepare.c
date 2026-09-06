/* First-stage SRAM loader, run before overwriting the application's DMA RAM. */
#include <stdint.h>
#define REG32(a) (*(volatile uint32_t *)(a))
__attribute__((naked, section(".entry"))) void prepare_entry(void) {
  __asm volatile("cpsid i\nldr sp, =0x20050000\nb prepare_main");
}
__attribute__((used, noreturn)) void prepare_main(void) {
  REG32(0xE000E010U) = 0U;
  REG32(0x40020418U) = 1U << 18; /* PB2 motor standby low */
  REG32(0x40023820U) = 0xFFFFFFFFU; /* all APB1 peripherals held in reset */
  REG32(0x40023824U) = 0xFFFFFFFFU; /* all APB2 peripherals held in reset */
  REG32(0x40023810U) = 0x00600000U; /* DMA1/2 reset; GPIOs retained */
  __asm volatile("dsb\nisb");
  REG32(0x40023810U) = 0U;
  REG32(0x40023820U) = 0U;
  REG32(0x40023824U) = 0U;
  REG32(0x2004C008U) = 0x50524550U;
  for (;;) __asm volatile("nop");
}
