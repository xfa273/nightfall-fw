#include "f413_trace_compact.h"

#include <limits.h>
#include <string.h>

_Static_assert(sizeof(f413_trace_compact_fast_t) == 44U,
               "unexpected fast trace record size");
_Static_assert(sizeof(f413_trace_compact_slow_t) == 42U,
               "unexpected slow trace record size");
_Static_assert((sizeof(f413_trace_compact_fast_t) * F413_TRACE_COMPACT_FAST_RECORDS +
                sizeof(f413_trace_compact_slow_t) * F413_TRACE_COMPACT_SLOW_RECORDS) <
                   (sizeof(nvm_trace_log_record_t) * 4096U),
               "compact staging must use fewer bytes per sample");

static int16_t f413_trace_compact_i16(int32_t value, bool* exact)
{
  if (value > INT16_MAX)
  {
    *exact = false;
    return INT16_MAX;
  }
  if (value < INT16_MIN)
  {
    *exact = false;
    return INT16_MIN;
  }
  return (int16_t)value;
}

static int16_t f413_trace_compact_omega(int32_t mdps, bool* exact)
{
  int32_t quantized;

  if (mdps > (int32_t)INT16_MAX * F413_TRACE_COMPACT_OMEGA_QUANTUM_MDPS +
                 F413_TRACE_COMPACT_OMEGA_QUANTUM_MDPS / 2L)
  {
    *exact = false;
    return INT16_MAX;
  }
  if (mdps < (int32_t)INT16_MIN * F413_TRACE_COMPACT_OMEGA_QUANTUM_MDPS -
                 F413_TRACE_COMPACT_OMEGA_QUANTUM_MDPS / 2L)
  {
    *exact = false;
    return INT16_MIN;
  }
  if (mdps >= 0)
  {
    quantized = (mdps + F413_TRACE_COMPACT_OMEGA_QUANTUM_MDPS / 2L) /
                F413_TRACE_COMPACT_OMEGA_QUANTUM_MDPS;
  }
  else
  {
    quantized = (mdps - F413_TRACE_COMPACT_OMEGA_QUANTUM_MDPS / 2L) /
                F413_TRACE_COMPACT_OMEGA_QUANTUM_MDPS;
  }
  return f413_trace_compact_i16(quantized, exact);
}

bool f413_trace_compact_pack_fast(const nvm_trace_log_record_t* source,
                                  f413_trace_compact_fast_t* destination)
{
  bool exact = true;

  if ((source == NULL) || (destination == NULL))
  {
    return false;
  }

  destination->timestamp_ms = source->timestamp_ms;
  destination->target_distance_x1000 = source->target_distance_x1000;
  destination->distance_mm = source->distance_mm;
  destination->angle_mdeg = source->angle_mdeg;
  destination->target_velocity_mm_s =
      f413_trace_compact_i16(source->target_velocity_mm_s, &exact);
  destination->real_velocity_mm_s =
      f413_trace_compact_i16(source->real_velocity_mm_s, &exact);
  destination->accel_velocity_mm_s =
      f413_trace_compact_i16(source->accel_velocity_mm_s, &exact);
  destination->target_omega_x100mdps =
      f413_trace_compact_omega(source->target_omega_mdps, &exact);
  destination->real_omega_x100mdps =
      f413_trace_compact_omega(source->real_omega_mdps, &exact);
  destination->gyro_z_raw_x100mdps =
      f413_trace_compact_omega(source->gyro_z_raw_mdps, &exact);
  destination->target_angle_mdeg = source->target_angle_mdeg;
  destination->accel_forward_mm_s2 =
      f413_trace_compact_i16(source->accel_forward_mm_s2, &exact);
  destination->encoder_l = source->encoder_l;
  destination->encoder_r = source->encoder_r;
  destination->motor_out_l = source->motor_out_l;
  destination->motor_out_r = source->motor_out_r;
  destination->flags = source->flags;
  return exact;
}

void f413_trace_compact_pack_slow(const nvm_trace_log_record_t* source,
                                  f413_trace_compact_slow_t* destination)
{
  if ((source == NULL) || (destination == NULL))
  {
    return;
  }

  destination->reserved_i32_0 = source->reserved_i32_0;
  destination->reserved_i32_1 = source->reserved_i32_1;
  destination->reserved_i32_2 = source->reserved_i32_2;
  destination->reserved_i32_3 = source->reserved_i32_3;
  destination->adc_fr = source->adc_fr;
  destination->adc_r = source->adc_r;
  destination->adc_fl = source->adc_fl;
  destination->adc_l = source->adc_l;
  destination->adc_vbat = source->adc_vbat;
  destination->wall_read_fr = source->wall_read_fr;
  destination->wall_read_r = source->wall_read_r;
  destination->wall_read_fl = source->wall_read_fl;
  destination->wall_read_l = source->wall_read_l;
  destination->op_mode = source->op_mode;
  destination->op_case = source->op_case;
  destination->op_sub = source->op_sub;
  destination->test_id = source->test_id;
  destination->reserved_u16_0 = source->reserved_u16_0;
  destination->reserved_u16_1 = source->reserved_u16_1;
}

void f413_trace_compact_expand(const f413_trace_compact_fast_t* fast,
                               const f413_trace_compact_slow_t* slow,
                               uint32_t seq,
                               nvm_trace_log_record_t* destination)
{
  if ((fast == NULL) || (slow == NULL) || (destination == NULL))
  {
    return;
  }

  memset(destination, 0, sizeof(*destination));
  destination->seq = seq;
  destination->timestamp_ms = fast->timestamp_ms;
  destination->target_distance_x1000 = fast->target_distance_x1000;
  destination->distance_mm = fast->distance_mm;
  destination->angle_mdeg = fast->angle_mdeg;
  destination->target_velocity_mm_s = fast->target_velocity_mm_s;
  destination->real_velocity_mm_s = fast->real_velocity_mm_s;
  destination->accel_velocity_mm_s = fast->accel_velocity_mm_s;
  destination->target_omega_mdps =
      (int32_t)fast->target_omega_x100mdps * F413_TRACE_COMPACT_OMEGA_QUANTUM_MDPS;
  destination->real_omega_mdps =
      (int32_t)fast->real_omega_x100mdps * F413_TRACE_COMPACT_OMEGA_QUANTUM_MDPS;
  destination->gyro_z_raw_mdps =
      (int32_t)fast->gyro_z_raw_x100mdps * F413_TRACE_COMPACT_OMEGA_QUANTUM_MDPS;
  destination->target_angle_mdeg = fast->target_angle_mdeg;
  destination->accel_forward_mm_s2 = fast->accel_forward_mm_s2;
  destination->encoder_l = fast->encoder_l;
  destination->encoder_r = fast->encoder_r;
  destination->motor_out_l = fast->motor_out_l;
  destination->motor_out_r = fast->motor_out_r;
  destination->flags = fast->flags;

  destination->reserved_i32_0 = slow->reserved_i32_0;
  destination->reserved_i32_1 = slow->reserved_i32_1;
  destination->reserved_i32_2 = slow->reserved_i32_2;
  destination->reserved_i32_3 = slow->reserved_i32_3;
  destination->adc_fr = slow->adc_fr;
  destination->adc_r = slow->adc_r;
  destination->adc_fl = slow->adc_fl;
  destination->adc_l = slow->adc_l;
  destination->adc_vbat = slow->adc_vbat;
  destination->wall_read_fr = slow->wall_read_fr;
  destination->wall_read_r = slow->wall_read_r;
  destination->wall_read_fl = slow->wall_read_fl;
  destination->wall_read_l = slow->wall_read_l;
  destination->op_mode = slow->op_mode;
  destination->op_case = slow->op_case;
  destination->op_sub = slow->op_sub;
  destination->test_id = slow->test_id;
  destination->reserved_u16_0 = slow->reserved_u16_0;
  destination->reserved_u16_1 = slow->reserved_u16_1;
}
