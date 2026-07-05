#include "f413_wall_distance.h"

#include <string.h>

#include "nvm_params.h"
#include "params.h"
#include "sensor_distance.h"

#define F413_WALL_DISTANCE_SAT_ADC (4090U)

static bool s_wall_distance_initialized = false;
static bool s_wall_distance_params_loaded = false;

static uint16_t f413_wall_distance_u16_delta(int32_t value)
{
  if (value <= 0)
  {
    return 0U;
  }
  if (value > 65535)
  {
    return 65535U;
  }
  return (uint16_t)value;
}

static bool f413_wall_distance_channel_saturated(const f413_wall_sensor_snapshot_t* adc,
                                                 uint16_t ch_mask)
{
  if (adc == NULL)
  {
    return true;
  }

  switch (ch_mask)
  {
    case F413_WALL_DISTANCE_CH_FR: return adc->fr_on >= F413_WALL_DISTANCE_SAT_ADC;
    case F413_WALL_DISTANCE_CH_R:  return adc->r_on >= F413_WALL_DISTANCE_SAT_ADC;
    case F413_WALL_DISTANCE_CH_FL: return adc->fl_on >= F413_WALL_DISTANCE_SAT_ADC;
    case F413_WALL_DISTANCE_CH_L:  return adc->l_on >= F413_WALL_DISTANCE_SAT_ADC;
    default: return true;
  }
}

static void f413_wall_distance_update_channel(f413_wall_distance_snapshot_t* out,
                                              uint16_t ch_mask,
                                              uint16_t delta,
                                              uint16_t min_signal,
                                              bool in_range,
                                              bool saturated)
{
  if (saturated)
  {
    out->saturated_mask |= ch_mask;
  }
  if (delta <= min_signal)
  {
    out->below_signal_mask |= ch_mask;
  }
  if (!in_range)
  {
    out->extrapolated_mask |= ch_mask;
  }
  if (!saturated && (delta > min_signal) && in_range)
  {
    out->valid_mask |= ch_mask;
  }
}

void f413_wall_distance_init(void)
{
  sensor_distance_init();
  sensor_distance_set_interpolation(SENSOR_DISTANCE_INTERP_PCHIP);
  s_wall_distance_params_loaded = nvm_params_distance_load_and_apply();
  s_wall_distance_initialized = true;
}

bool f413_wall_distance_params_loaded(void)
{
  if (!s_wall_distance_initialized)
  {
    f413_wall_distance_init();
  }
  return s_wall_distance_params_loaded;
}

bool f413_wall_distance_read_snapshot(f413_wall_distance_snapshot_t* out)
{
  uint16_t fr;
  uint16_t r;
  uint16_t fl;
  uint16_t l;
  uint32_t fsum_u32;
  uint16_t fsum;
  bool fr_sat;
  bool r_sat;
  bool fl_sat;
  bool l_sat;

  if (out == NULL)
  {
    return false;
  }
  if (!s_wall_distance_initialized)
  {
    f413_wall_distance_init();
  }
  memset(out, 0, sizeof(*out));
  if (!f413_wall_sensor_read_snapshot(&out->adc))
  {
    return false;
  }

  fr = f413_wall_distance_u16_delta(out->adc.fr_delta);
  r = f413_wall_distance_u16_delta(out->adc.r_delta);
  fl = f413_wall_distance_u16_delta(out->adc.fl_delta);
  l = f413_wall_distance_u16_delta(out->adc.l_delta);
  fsum_u32 = (uint32_t)fr + (uint32_t)fl;
  fsum = (fsum_u32 > 0xFFFFU) ? 0xFFFFU : (uint16_t)fsum_u32;

  out->fr_mm_unwarped = sensor_distance_from_fr_unwarped(fr);
  out->fl_mm_unwarped = sensor_distance_from_fl_unwarped(fl);
  out->front_sum_mm_unwarped = sensor_distance_from_fsum_unwarped(fsum);
  out->fr_mm = sensor_distance_from_fr(fr);
  out->r_mm = sensor_distance_from_r(r);
  out->fl_mm = sensor_distance_from_fl(fl);
  out->l_mm = sensor_distance_from_l(l);
  out->front_sum_mm = sensor_distance_from_fsum(fsum);
  out->distance_params_loaded = s_wall_distance_params_loaded;

  fr_sat = f413_wall_distance_channel_saturated(&out->adc, F413_WALL_DISTANCE_CH_FR);
  r_sat = f413_wall_distance_channel_saturated(&out->adc, F413_WALL_DISTANCE_CH_R);
  fl_sat = f413_wall_distance_channel_saturated(&out->adc, F413_WALL_DISTANCE_CH_FL);
  l_sat = f413_wall_distance_channel_saturated(&out->adc, F413_WALL_DISTANCE_CH_L);

  f413_wall_distance_update_channel(out,
                                    F413_WALL_DISTANCE_CH_FR,
                                    fr,
                                    WALL_BASE_FR,
                                    sensor_distance_ad_in_range_fr(fr),
                                    fr_sat);
  f413_wall_distance_update_channel(out,
                                    F413_WALL_DISTANCE_CH_R,
                                    r,
                                    WALL_BASE_R,
                                    sensor_distance_ad_in_range_r(r),
                                    r_sat);
  f413_wall_distance_update_channel(out,
                                    F413_WALL_DISTANCE_CH_FL,
                                    fl,
                                    WALL_BASE_FL,
                                    sensor_distance_ad_in_range_fl(fl),
                                    fl_sat);
  f413_wall_distance_update_channel(out,
                                    F413_WALL_DISTANCE_CH_L,
                                    l,
                                    WALL_BASE_L,
                                    sensor_distance_ad_in_range_l(l),
                                    l_sat);
  f413_wall_distance_update_channel(out,
                                    F413_WALL_DISTANCE_CH_FSUM,
                                    fsum,
                                    (uint16_t)(WALL_BASE_FR + WALL_BASE_FL),
                                    sensor_distance_ad_in_range_fsum(fsum),
                                    fr_sat || fl_sat);

  out->front_valid = ((out->valid_mask & F413_WALL_DISTANCE_CH_FSUM) != 0U) &&
                     ((out->valid_mask & F413_WALL_DISTANCE_CH_FR) != 0U) &&
                     ((out->valid_mask & F413_WALL_DISTANCE_CH_FL) != 0U);
  out->right_valid = (out->valid_mask & F413_WALL_DISTANCE_CH_R) != 0U;
  out->left_valid = (out->valid_mask & F413_WALL_DISTANCE_CH_L) != 0U;
  return true;
}

bool f413_wall_distance_front_present(const f413_wall_distance_snapshot_t* s)
{
  return (s != NULL) && s->adc.front_wall && s->front_valid;
}

bool f413_wall_distance_side_present(const f413_wall_distance_snapshot_t* s, bool right)
{
  if (s == NULL)
  {
    return false;
  }
  return right ? (s->adc.right_wall && s->right_valid) : (s->adc.left_wall && s->left_valid);
}
