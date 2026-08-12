#include "f413_battery.h"

#include <stddef.h>

#include "params.h"
#include "stm32f4xx_hal.h"

#define F413_BATTERY_ADC_FULL_SCALE (4095ULL)
#define F413_BATTERY_FILTER_SAMPLES (8U)
#define F413_BATTERY_ADC_RAW_MIN_VALID (512U)
#define F413_BATTERY_ADC_RAW_MAX_VALID (4050U)
#define F413_BATTERY_BOOT_MAX_SPREAD_MV (250U)
#define F413_BATTERY_SAMPLE_STALE_MS (100U)

static volatile uint16_t s_last_adc_raw = 0U;
static volatile uint32_t s_last_sample_ms = 0U;
static volatile uint32_t s_sample_mv[F413_BATTERY_FILTER_SAMPLES];
static volatile uint32_t s_sample_sum_mv = 0U;
static volatile uint32_t s_filter_average_mv = 0U;
static volatile uint32_t s_filter_spread_mv = UINT32_MAX;
static volatile uint32_t s_filter_sequence = 0U;
static volatile uint8_t s_sample_index = 0U;
static volatile uint8_t s_sample_count = 0U;
static volatile uint8_t s_cell_count = 0U;
static volatile uint8_t s_low_sample_count = 0U;
static volatile uint8_t s_recover_sample_count = 0U;
static volatile uint8_t s_invalid_sample_count = 0U;
static volatile bool s_boot_classified = false;
static volatile bool s_run_inhibited = true;
static volatile f413_battery_status_t s_status = F413_BATTERY_STATUS_NOT_READY;

static void f413_battery_read_filter_snapshot(uint8_t* count,
                                               uint32_t* average_mv,
                                               uint32_t* spread_mv)
{
  uint32_t sequence_before;
  uint32_t sequence_after;

  for (;;)
  {
    sequence_before = s_filter_sequence;
    if ((sequence_before & 1U) != 0U)
    {
      continue;
    }

    *count = s_sample_count;
    *average_mv = s_filter_average_mv;
    *spread_mv = s_filter_spread_mv;
    sequence_after = s_filter_sequence;
    if ((sequence_before == sequence_after) &&
        ((sequence_after & 1U) == 0U))
    {
      return;
    }
  }
}

static bool f413_battery_sample_is_fresh(void)
{
  if (s_sample_count == 0U)
  {
    return false;
  }
  return (HAL_GetTick() - s_last_sample_ms) <= F413_BATTERY_SAMPLE_STALE_MS;
}

static void f413_battery_update_state(uint32_t voltage_mv)
{
  uint32_t run_min_mv;
  uint32_t recover_mv;
  uint32_t warning_mv;
  uint32_t overvoltage_mv;

  if (!s_boot_classified || ((s_cell_count != 2U) && (s_cell_count != 3U)))
  {
    return;
  }

  run_min_mv = (uint32_t)s_cell_count * NIGHTFALL_F413_BATTERY_RUN_MIN_MV_PER_CELL;
  recover_mv = (uint32_t)s_cell_count * NIGHTFALL_F413_BATTERY_RECOVER_MV_PER_CELL;
  warning_mv = (uint32_t)s_cell_count * NIGHTFALL_F413_BATTERY_WARN_MV_PER_CELL;
  overvoltage_mv = (uint32_t)s_cell_count * NIGHTFALL_F413_BATTERY_MAX_MV_PER_CELL;

  if (voltage_mv > overvoltage_mv)
  {
    s_run_inhibited = true;
    s_low_sample_count = NIGHTFALL_F413_BATTERY_LOW_DEBOUNCE_SAMPLES;
    s_recover_sample_count = 0U;
    s_status = F413_BATTERY_STATUS_RUN_INHIBITED;
    return;
  }

  if (voltage_mv <= run_min_mv)
  {
    if (s_low_sample_count < NIGHTFALL_F413_BATTERY_LOW_DEBOUNCE_SAMPLES)
    {
      s_low_sample_count++;
    }
    s_recover_sample_count = 0U;
    if (s_low_sample_count >= NIGHTFALL_F413_BATTERY_LOW_DEBOUNCE_SAMPLES)
    {
      s_run_inhibited = true;
    }
  }
  else if (voltage_mv >= recover_mv)
  {
    s_low_sample_count = 0U;
    if (s_recover_sample_count < NIGHTFALL_F413_BATTERY_RECOVER_DEBOUNCE_SAMPLES)
    {
      s_recover_sample_count++;
    }
    if (s_recover_sample_count >= NIGHTFALL_F413_BATTERY_RECOVER_DEBOUNCE_SAMPLES)
    {
      s_run_inhibited = false;
    }
  }

  if (s_run_inhibited)
  {
    s_status = F413_BATTERY_STATUS_RUN_INHIBITED;
  }
  else if (voltage_mv <= warning_mv)
  {
    s_status = F413_BATTERY_STATUS_WARNING;
  }
  else
  {
    s_status = F413_BATTERY_STATUS_OK;
  }
}

static uint32_t f413_battery_calculate_window_spread_mv(uint8_t count)
{
  uint32_t minimum = UINT32_MAX;
  uint32_t maximum = 0U;

  for (uint8_t i = 0U; i < count; i++)
  {
    uint32_t value = s_sample_mv[i];
    if (value < minimum)
    {
      minimum = value;
    }
    if (value > maximum)
    {
      maximum = value;
    }
  }

  if ((count == 0U) || (minimum == UINT32_MAX))
  {
    return UINT32_MAX;
  }
  return maximum - minimum;
}

static void f413_battery_classify_boot_voltage(uint32_t voltage_mv)
{
  uint8_t configured_cells = NIGHTFALL_F413_BATTERY_CELL_COUNT;
  uint32_t run_min_mv;
  uint32_t overvoltage_mv;

  s_boot_classified = true;
  s_run_inhibited = true;
  s_low_sample_count = 0U;
  s_recover_sample_count = 0U;

  if ((configured_cells == 2U) || (configured_cells == 3U))
  {
    s_cell_count = configured_cells;
  }
  else if (voltage_mv <= NIGHTFALL_F413_BATTERY_AUTO_2S_MAX_MV)
  {
    s_cell_count = 2U;
  }
  else if (voltage_mv >= NIGHTFALL_F413_BATTERY_AUTO_3S_MIN_MV)
  {
    s_cell_count = 3U;
  }
  else
  {
    s_cell_count = 0U;
    s_status = F413_BATTERY_STATUS_AMBIGUOUS;
    return;
  }

  run_min_mv = (uint32_t)s_cell_count * NIGHTFALL_F413_BATTERY_RUN_MIN_MV_PER_CELL;
  overvoltage_mv = (uint32_t)s_cell_count * NIGHTFALL_F413_BATTERY_MAX_MV_PER_CELL;
  if ((voltage_mv <= run_min_mv) || (voltage_mv > overvoltage_mv))
  {
    s_low_sample_count = NIGHTFALL_F413_BATTERY_LOW_DEBOUNCE_SAMPLES;
    s_status = F413_BATTERY_STATUS_RUN_INHIBITED;
  }
  else
  {
    s_low_sample_count = 0U;
    s_recover_sample_count = NIGHTFALL_F413_BATTERY_RECOVER_DEBOUNCE_SAMPLES;
    s_run_inhibited = false;
    f413_battery_update_state(voltage_mv);
  }
}

void f413_battery_init(void)
{
  s_last_adc_raw = 0U;
  s_last_sample_ms = 0U;
  s_sample_sum_mv = 0U;
  s_filter_average_mv = 0U;
  s_filter_spread_mv = UINT32_MAX;
  s_filter_sequence = 0U;
  s_sample_index = 0U;
  s_sample_count = 0U;
  s_cell_count = 0U;
  s_low_sample_count = 0U;
  s_recover_sample_count = 0U;
  s_invalid_sample_count = 0U;
  s_boot_classified = false;
  s_run_inhibited = true;
  s_status = F413_BATTERY_STATUS_NOT_READY;
  for (uint8_t i = 0U; i < F413_BATTERY_FILTER_SAMPLES; i++)
  {
    s_sample_mv[i] = 0U;
  }
}

uint32_t f413_battery_adc_raw_to_mv(uint16_t adc_raw)
{
  const uint64_t divider_numerator =
      (uint64_t)(NIGHTFALL_F413_BATTERY_DIVIDER_TOP_OHM +
                 NIGHTFALL_F413_BATTERY_DIVIDER_BOTTOM_OHM);
  const uint64_t divider_denominator =
      F413_BATTERY_ADC_FULL_SCALE * (uint64_t)NIGHTFALL_F413_BATTERY_DIVIDER_BOTTOM_OHM;
  uint64_t numerator = (uint64_t)adc_raw *
                       (uint64_t)NIGHTFALL_F413_BATTERY_ADC_REFERENCE_MV *
                       divider_numerator;

  return (uint32_t)((numerator + (divider_denominator / 2ULL)) / divider_denominator);
}

void f413_battery_feed_adc_raw(uint16_t adc_raw)
{
  uint8_t count;
  uint32_t voltage_mv;
  uint32_t average_mv;
  uint32_t spread_mv;
  uint8_t index;

  s_last_adc_raw = adc_raw;
  s_last_sample_ms = HAL_GetTick();

  if ((adc_raw < F413_BATTERY_ADC_RAW_MIN_VALID) ||
      (adc_raw > F413_BATTERY_ADC_RAW_MAX_VALID))
  {
    if (s_invalid_sample_count < NIGHTFALL_F413_BATTERY_LOW_DEBOUNCE_SAMPLES)
    {
      s_invalid_sample_count++;
    }
    if (s_boot_classified &&
        (s_invalid_sample_count >= NIGHTFALL_F413_BATTERY_LOW_DEBOUNCE_SAMPLES))
    {
      s_run_inhibited = true;
      s_low_sample_count = NIGHTFALL_F413_BATTERY_LOW_DEBOUNCE_SAMPLES;
      s_recover_sample_count = 0U;
      s_status = F413_BATTERY_STATUS_RUN_INHIBITED;
    }
    return;
  }
  s_invalid_sample_count = 0U;

  voltage_mv = f413_battery_adc_raw_to_mv(adc_raw);
  index = s_sample_index;
  count = s_sample_count;

  /*
   * The ADC callback writes the filter while boot/main code reads it.  Publish
   * the derived 32-bit values as one sequence-checked snapshot; main code must
   * never observe the sum between the subtract and add operations below.
   */
  s_filter_sequence++;
  if (count < F413_BATTERY_FILTER_SAMPLES)
  {
    count++;
  }
  else
  {
    s_sample_sum_mv -= s_sample_mv[index];
  }
  s_sample_mv[index] = voltage_mv;
  s_sample_sum_mv += voltage_mv;
  s_sample_index = (uint8_t)((index + 1U) % F413_BATTERY_FILTER_SAMPLES);
  average_mv = s_sample_sum_mv / (uint32_t)count;
  spread_mv = f413_battery_calculate_window_spread_mv(count);
  s_filter_average_mv = average_mv;
  s_filter_spread_mv = spread_mv;
  s_sample_count = count;
  s_filter_sequence++;

  if (s_boot_classified)
  {
    f413_battery_update_state(average_mv);
  }
}

bool f413_battery_wait_boot_ready(uint32_t timeout_ms)
{
  uint32_t deadline = HAL_GetTick() + timeout_ms;
  uint32_t average_mv;
  uint32_t spread_mv;
  uint8_t count;

  do
  {
    f413_battery_read_filter_snapshot(&count, &average_mv, &spread_mv);
    if (count >= F413_BATTERY_FILTER_SAMPLES)
    {
      break;
    }
    if ((int32_t)(HAL_GetTick() - deadline) >= 0)
    {
      s_status = F413_BATTERY_STATUS_NOT_READY;
      s_run_inhibited = true;
      return false;
    }
    HAL_Delay(1U);
  }
  while (true);

  if (spread_mv > F413_BATTERY_BOOT_MAX_SPREAD_MV)
  {
    s_status = F413_BATTERY_STATUS_NOT_READY;
    s_run_inhibited = true;
    return false;
  }

  f413_battery_classify_boot_voltage(average_mv);
  return (s_status != F413_BATTERY_STATUS_NOT_READY) &&
         (s_status != F413_BATTERY_STATUS_AMBIGUOUS);
}

uint16_t f413_battery_get_adc_raw(void)
{
  return s_last_adc_raw;
}

uint32_t f413_battery_get_voltage_mv(void)
{
  uint32_t average_mv;
  uint32_t spread_mv;
  uint8_t count;

  f413_battery_read_filter_snapshot(&count, &average_mv, &spread_mv);
  (void)count;
  (void)spread_mv;
  return average_mv;
}

uint8_t f413_battery_get_cell_count(void)
{
  return s_cell_count;
}

f413_battery_status_t f413_battery_get_status(void)
{
  if (s_boot_classified && !f413_battery_sample_is_fresh())
  {
    return F413_BATTERY_STATUS_ADC_STALE;
  }
  return s_status;
}

const char* f413_battery_status_text(f413_battery_status_t status)
{
  switch (status)
  {
    case F413_BATTERY_STATUS_AMBIGUOUS: return "ambiguous 2S/3S";
    case F413_BATTERY_STATUS_OK: return "ok";
    case F413_BATTERY_STATUS_WARNING: return "warning";
    case F413_BATTERY_STATUS_RUN_INHIBITED: return "run inhibited";
    case F413_BATTERY_STATUS_ADC_STALE: return "adc stale";
    case F413_BATTERY_STATUS_NOT_READY:
    default: return "not ready";
  }
}

bool f413_battery_run_allowed(void)
{
  if (!s_boot_classified || s_run_inhibited ||
      ((s_cell_count != 2U) && (s_cell_count != 3U)))
  {
    return false;
  }
  return f413_battery_sample_is_fresh();
}
