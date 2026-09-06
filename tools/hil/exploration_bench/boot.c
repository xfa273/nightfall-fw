/* SRAM-only, interrupts masked, no firmware/HAL/NVM/run dependencies. */
#include "bench.h"
#include <stddef.h>
#define REG32(a) (*(volatile uint32_t *)(a))
extern uint32_t __bss_start__, __bss_end__, __stack_top__;
void bench_entry(void);
void bench_fault(void);
volatile nf_bench_mailbox nf_bench_output __attribute__((section(".mailbox")));
__attribute__((section(".vectors"), used))
const uintptr_t bench_vectors[16] = {
  (uintptr_t)&__stack_top__, (uintptr_t)bench_entry,
  (uintptr_t)bench_fault, (uintptr_t)bench_fault,
  (uintptr_t)bench_fault, (uintptr_t)bench_fault, (uintptr_t)bench_fault
};

void *memset(void *p, int v, size_t n) {
  unsigned char *q = p;
  while (n--) *q++ = (unsigned char)v;
  return p;
}
void *memcpy(void *d, const void *s, size_t n) {
  unsigned char *q = d;
  const unsigned char *r = s;
  while (n--) *q++ = *r++;
  return d;
}
void bench_fault(void) {
  nf_bench_output.cfsr = REG32(0xE000ED28U);
  nf_bench_output.hfsr = REG32(0xE000ED2CU);
  nf_bench_output.status = 0xFFFFFFFFU;
  for (;;) __asm volatile("nop");
}
static void bench_main(void) __attribute__((used, noreturn));
static void bench_main(void) {
  REG32(0xE000E010U) = 0U; /* SysTick off */
  REG32(0xE000ED08U) = (uint32_t)(uintptr_t)bench_vectors;
  /* The loader has already held PB2 motor standby low and reset DMA/timers.
     Assert it again; never enable any PWM or motor driver. */
  REG32(0x40020418U) = 1U << 18;
  REG32(0xE000ED88U) |= 0x00F00000U; /* CP10/11, for linked profile code */
  __asm volatile("dsb\nisb");
  for (uint32_t *p = &__bss_start__; p < &__bss_end__; ++p) *p = 0U;
  memset((void *)&nf_bench_output, 0, sizeof(nf_bench_output));
  nf_bench_output.magic = NF_BENCH_MAGIC;
  nf_bench_output.version = 1U;
  nf_bench_output.status = 1U;
  nf_bench_output.cpu_hz = 100000000U;
  nf_bench_output.rcc_cr = REG32(0x40023800U);
  nf_bench_output.rcc_pllcfgr = REG32(0x40023804U);
  nf_bench_output.rcc_cfgr = REG32(0x40023808U);
  REG32(0xE000EDFCU) |= 1U << 24; /* DWT cycle counter */
  REG32(0xE0001004U) = 0U;
  REG32(0xE0001000U) |= 1U;
  nf_bench_run();
  nf_bench_output.status = 2U;
  for (;;) __asm volatile("nop");
}
__attribute__((naked)) void bench_entry(void) {
  __asm volatile("cpsid i\nldr sp, =__stack_top__\nb bench_main");
}
