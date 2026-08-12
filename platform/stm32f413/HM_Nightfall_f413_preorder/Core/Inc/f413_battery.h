#ifndef F413_BATTERY_H_
#define F413_BATTERY_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  F413_BATTERY_STATUS_NOT_READY = 0,
  F413_BATTERY_STATUS_AMBIGUOUS,
  F413_BATTERY_STATUS_OK,
  F413_BATTERY_STATUS_WARNING,
  F413_BATTERY_STATUS_RUN_INHIBITED,
  F413_BATTERY_STATUS_ADC_STALE,
} f413_battery_status_t;

void f413_battery_init(void);
void f413_battery_feed_adc_raw(uint16_t adc_raw);
bool f413_battery_wait_boot_ready(uint32_t timeout_ms);

uint32_t f413_battery_adc_raw_to_mv(uint16_t adc_raw);
uint16_t f413_battery_get_adc_raw(void);
uint32_t f413_battery_get_voltage_mv(void);
uint8_t f413_battery_get_cell_count(void);
f413_battery_status_t f413_battery_get_status(void);
const char* f413_battery_status_text(f413_battery_status_t status);
bool f413_battery_run_allowed(void);

#endif
