#include "trace.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

#include "main.h"

#if defined(STM32F413xx)

extern UART_HandleTypeDef huart1;

#ifndef NIGHTFALL_TRACE_F413_USE_UART
#define NIGHTFALL_TRACE_F413_USE_UART 1
#endif

#if NIGHTFALL_TRACE_F413_USE_UART

static void trace_port_putc(char ch) {
    uint8_t b = (uint8_t)ch;
    HAL_UART_Transmit(&huart1, &b, 1, 1);
}

int __io_putchar(int ch) {
    trace_port_putc((char)ch);
    return ch;
}

void trace_init(void) {}

#else

#define NIGHTFALL_TRACE_SWO_BAUD_HZ 2000000U

static void trace_port_putc(char ch) {
    if (((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) ||
        ((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0U) ||
        ((ITM->TER & 1UL) == 0U)) {
        return;
    }

    for (uint32_t spin = 0U; spin < 100000U; ++spin) {
        if (ITM->PORT[0U].u32 != 0U) {
            ITM->PORT[0U].u8 = (uint8_t)ch;
            return;
        }
    }
}

int __io_putchar(int ch) {
    trace_port_putc((char)ch);
    return ch;
}

void trace_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DBGMCU->CR &= ~DBGMCU_CR_TRACE_MODE;
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN | DBGMCU_CR_TRACE_MODE_0;

    TPI->SPPR = 2U;

    {
        uint32_t hclk = HAL_RCC_GetHCLKFreq();
        if (hclk > NIGHTFALL_TRACE_SWO_BAUD_HZ) {
            TPI->ACPR = (hclk / NIGHTFALL_TRACE_SWO_BAUD_HZ) - 1U;
        } else {
            TPI->ACPR = 0U;
        }
    }

    ITM->TCR = ITM_TCR_ITMENA_Msk;
    ITM->TPR = 0U;
    ITM->TER = 1U;
}

#endif

#elif defined(STM32F405xx)

static void trace_port_putc(char ch) {
    uint8_t b = (uint8_t)ch;
    HAL_UART_Transmit(&huart1, &b, 1, 1);
}

void trace_init(void) {}

#else

static void trace_port_putc(char ch) {
    (void)ch;
}

void trace_init(void) {}

#endif

void trace_write(const char* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        trace_port_putc(data[i]);
    }
}

int trace_printf(const char* fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return n;
    }

    size_t len = (size_t)n;
    if (len >= sizeof(buf)) {
        len = sizeof(buf) - 1U;
    }

    trace_write(buf, len);
    return n;
}
