#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "f413_battery.h"

static uint32_t s_tick_ms = 0U;
static unsigned int s_checks = 0U;

uint32_t HAL_GetTick(void)
{
  return s_tick_ms;
}

void HAL_Delay(uint32_t delay_ms)
{
  s_tick_ms += delay_ms;
}

static void check(bool condition, const char* expression, int line)
{
  s_checks++;
  if (!condition)
  {
    fprintf(stderr, "FAIL line %d: %s\n", line, expression);
    exit(1);
  }
}

#define CHECK(expr) check((expr), #expr, __LINE__)

static uint16_t adc_raw_for_mv(uint32_t voltage_mv)
{
  const uint64_t numerator = (uint64_t)voltage_mv * 4095ULL * 27000ULL;
  const uint64_t denominator = 3300ULL * 127000ULL;
  return (uint16_t)((numerator + denominator / 2ULL) / denominator);
}

static void feed_raw(uint16_t adc_raw, uint8_t count)
{
  for (uint8_t i = 0U; i < count; i++)
  {
    f413_battery_feed_adc_raw(adc_raw);
    s_tick_ms += 8U;
  }
}

static void feed_mv(uint32_t voltage_mv, uint8_t count)
{
  feed_raw(adc_raw_for_mv(voltage_mv), count);
}

static void reset_monitor(void)
{
  s_tick_ms = 0U;
  f413_battery_init();
}

static void boot_mv(uint32_t voltage_mv)
{
  reset_monitor();
  feed_mv(voltage_mv, 8U);
  CHECK(f413_battery_wait_boot_ready(1U));
}

static void test_adc_conversion(void)
{
  CHECK(f413_battery_adc_raw_to_mv(0U) == 0U);
  CHECK(f413_battery_adc_raw_to_mv(4095U) == 15522U);
  CHECK(f413_battery_adc_raw_to_mv(adc_raw_for_mv(7400U)) >= 7395U);
  CHECK(f413_battery_adc_raw_to_mv(adc_raw_for_mv(7400U)) <= 7405U);
  CHECK(f413_battery_adc_raw_to_mv(adc_raw_for_mv(12600U)) >= 12595U);
  CHECK(f413_battery_adc_raw_to_mv(adc_raw_for_mv(12600U)) <= 12605U);
}

static void test_2s_and_3s_classification(void)
{
  boot_mv(8400U);
  CHECK(f413_battery_get_cell_count() == 2U);
  CHECK(f413_battery_get_status() == F413_BATTERY_STATUS_OK);
  CHECK(f413_battery_run_allowed());

  boot_mv(6900U);
  CHECK(f413_battery_get_cell_count() == 2U);
  CHECK(f413_battery_get_status() == F413_BATTERY_STATUS_WARNING);
  CHECK(f413_battery_run_allowed());

  boot_mv(12600U);
  CHECK(f413_battery_get_cell_count() == 3U);
  CHECK(f413_battery_get_status() == F413_BATTERY_STATUS_OK);
  CHECK(f413_battery_run_allowed());
}

static void test_auto_classification_boundaries(void)
{
  reset_monitor();
  feed_raw(2268U, 8U); /* 8597mV: last representable value <= 8.6V. */
  CHECK(f413_battery_wait_boot_ready(1U));
  CHECK(f413_battery_get_cell_count() == 2U);

  reset_monitor();
  feed_raw(2269U, 8U); /* 8601mV: enters the conservative ambiguous band. */
  CHECK(!f413_battery_wait_boot_ready(1U));
  CHECK(f413_battery_get_status() == F413_BATTERY_STATUS_AMBIGUOUS);
  CHECK(!f413_battery_run_allowed());

  reset_monitor();
  feed_raw(2374U, 8U); /* 8999mV: still ambiguous. */
  CHECK(!f413_battery_wait_boot_ready(1U));
  CHECK(f413_battery_get_status() == F413_BATTERY_STATUS_AMBIGUOUS);

  reset_monitor();
  feed_raw(2375U, 8U); /* 9003mV: first representable value >= 9.0V. */
  CHECK(f413_battery_wait_boot_ready(1U));
  CHECK(f413_battery_get_cell_count() == 3U);
  CHECK(f413_battery_get_status() == F413_BATTERY_STATUS_RUN_INHIBITED);
  CHECK(!f413_battery_run_allowed());
}

static void test_stale_adc(void)
{
  boot_mv(7400U);
  s_tick_ms += 101U;
  CHECK(f413_battery_get_status() == F413_BATTERY_STATUS_ADC_STALE);
  CHECK(!f413_battery_run_allowed());

  feed_mv(7400U, 1U);
  CHECK(f413_battery_get_status() == F413_BATTERY_STATUS_OK);
  CHECK(f413_battery_run_allowed());
}

static void test_tick_wrap_freshness(void)
{
  reset_monitor();
  s_tick_ms = UINT32_MAX - 2U;
  CHECK(!f413_battery_wait_boot_ready(5U));
  CHECK(s_tick_ms == 2U);

  reset_monitor();
  s_tick_ms = UINT32_MAX - 40U;
  feed_mv(7400U, 8U); /* The sample timestamps cross the 32-bit tick wrap. */
  CHECK(f413_battery_wait_boot_ready(1U));
  CHECK(f413_battery_run_allowed());

  /* The last feed is 8ms old here: reach the inclusive 100ms boundary. */
  s_tick_ms += 92U;
  CHECK(f413_battery_run_allowed());
  s_tick_ms += 1U;
  CHECK(f413_battery_get_status() == F413_BATTERY_STATUS_ADC_STALE);
  CHECK(!f413_battery_run_allowed());
}

static void test_low_and_recovery_debounce(void)
{
  boot_mv(7400U);

  feed_mv(6000U, 6U);
  CHECK(f413_battery_run_allowed());
  feed_mv(6000U, 1U);
  CHECK(f413_battery_get_status() == F413_BATTERY_STATUS_RUN_INHIBITED);
  CHECK(!f413_battery_run_allowed());

  feed_mv(7600U, 10U);
  CHECK(!f413_battery_run_allowed());
  feed_mv(7600U, 1U);
  CHECK(f413_battery_get_status() == F413_BATTERY_STATUS_OK);
  CHECK(f413_battery_run_allowed());
}

static void test_invalid_adc_debounce(void)
{
  boot_mv(7400U);
  feed_raw(0U, 2U);
  CHECK(f413_battery_run_allowed());
  feed_raw(0U, 1U);
  CHECK(f413_battery_get_status() == F413_BATTERY_STATUS_RUN_INHIBITED);
  CHECK(!f413_battery_run_allowed());

  feed_mv(7400U, 7U);
  CHECK(!f413_battery_run_allowed());
  feed_mv(7400U, 1U);
  CHECK(f413_battery_get_status() == F413_BATTERY_STATUS_OK);
  CHECK(f413_battery_run_allowed());
}

static void test_cell_count_change_is_fail_safe(void)
{
  boot_mv(7400U);
  feed_mv(12600U, 8U);
  CHECK(f413_battery_get_cell_count() == 2U);
  CHECK(f413_battery_get_status() == F413_BATTERY_STATUS_RUN_INHIBITED);
  CHECK(!f413_battery_run_allowed());

  boot_mv(11100U);
  feed_mv(7400U, 8U);
  CHECK(f413_battery_get_cell_count() == 3U);
  CHECK(f413_battery_get_status() == F413_BATTERY_STATUS_RUN_INHIBITED);
  CHECK(!f413_battery_run_allowed());
}

int main(void)
{
  test_adc_conversion();
  test_2s_and_3s_classification();
  test_auto_classification_boundaries();
  test_stale_adc();
  test_tick_wrap_freshness();
  test_low_and_recovery_debounce();
  test_invalid_adc_debounce();
  test_cell_count_change_is_fail_safe();
  printf("PASS: f413_battery host tests (%u checks)\n", s_checks);
  return 0;
}
