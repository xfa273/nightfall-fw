#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "f413_trace_compact.h"

static int32_t quantized_omega(int32_t value)
{
  if (value >= 0)
  {
    return ((value + 50) / 100) * 100;
  }
  return ((value - 50) / 100) * 100;
}

int main(void)
{
  nvm_trace_log_record_t input;
  nvm_trace_log_record_t output;
  f413_trace_compact_fast_t fast;
  f413_trace_compact_slow_t slow;

  memset(&input, 0, sizeof(input));
  input.seq = 987654U;
  input.timestamp_ms = 123456U;
  input.target_distance_x1000 = 1297345;
  input.distance_mm = 1297;
  input.angle_mdeg = -180363;
  input.target_velocity_mm_s = 1500;
  input.real_velocity_mm_s = -1400;
  input.accel_velocity_mm_s = 1325;
  input.target_omega_mdps = 742349;
  input.real_omega_mdps = -701951;
  input.gyro_z_raw_mdps = 12345;
  input.target_angle_mdeg = -180000;
  input.accel_forward_mm_s2 = 12000;
  input.encoder_l = -321;
  input.encoder_r = 654;
  input.motor_out_l = -999;
  input.motor_out_r = 1000;
  input.adc_fr = 11U;
  input.adc_r = 22U;
  input.adc_fl = 33U;
  input.adc_l = 44U;
  input.adc_vbat = 2456U;
  input.wall_read_fr = 101U;
  input.wall_read_r = 202U;
  input.wall_read_fl = 303U;
  input.wall_read_l = 404U;
  input.flags = 0xF0A5U;
  input.op_mode = 2U;
  input.op_case = 5U;
  input.op_sub = 0xFFU;
  input.test_id = 7U;
  input.reserved_i32_0 = INT32_MIN + 1;
  input.reserved_i32_1 = -1234567;
  input.reserved_i32_2 = 7654321;
  input.reserved_i32_3 = INT32_MAX;
  input.reserved_u16_0 = 0xABCDU;
  input.reserved_u16_1 = 0x1234U;

  assert(f413_trace_compact_pack_fast(&input, &fast));
  f413_trace_compact_pack_slow(&input, &slow);
  f413_trace_compact_expand(&fast, &slow, 42U, &output);

  assert(output.seq == 42U);
  assert(output.timestamp_ms == input.timestamp_ms);
  assert(output.target_distance_x1000 == input.target_distance_x1000);
  assert(output.distance_mm == input.distance_mm);
  assert(output.angle_mdeg == input.angle_mdeg);
  assert(output.target_velocity_mm_s == input.target_velocity_mm_s);
  assert(output.real_velocity_mm_s == input.real_velocity_mm_s);
  assert(output.accel_velocity_mm_s == input.accel_velocity_mm_s);
  assert(output.target_omega_mdps == quantized_omega(input.target_omega_mdps));
  assert(output.real_omega_mdps == quantized_omega(input.real_omega_mdps));
  assert(output.gyro_z_raw_mdps == quantized_omega(input.gyro_z_raw_mdps));
  assert(output.target_angle_mdeg == input.target_angle_mdeg);
  assert(output.accel_forward_mm_s2 == input.accel_forward_mm_s2);
  assert(output.encoder_l == input.encoder_l);
  assert(output.encoder_r == input.encoder_r);
  assert(output.motor_out_l == input.motor_out_l);
  assert(output.motor_out_r == input.motor_out_r);
  assert(output.adc_fr == input.adc_fr);
  assert(output.adc_r == input.adc_r);
  assert(output.adc_fl == input.adc_fl);
  assert(output.adc_l == input.adc_l);
  assert(output.adc_vbat == input.adc_vbat);
  assert(output.wall_read_fr == input.wall_read_fr);
  assert(output.wall_read_r == input.wall_read_r);
  assert(output.wall_read_fl == input.wall_read_fl);
  assert(output.wall_read_l == input.wall_read_l);
  assert(output.flags == input.flags);
  assert(output.op_mode == input.op_mode);
  assert(output.op_case == input.op_case);
  assert(output.op_sub == input.op_sub);
  assert(output.test_id == input.test_id);
  assert(output.reserved_i32_0 == input.reserved_i32_0);
  assert(output.reserved_i32_1 == input.reserved_i32_1);
  assert(output.reserved_i32_2 == input.reserved_i32_2);
  assert(output.reserved_i32_3 == input.reserved_i32_3);
  assert(output.reserved_u16_0 == input.reserved_u16_0);
  assert(output.reserved_u16_1 == input.reserved_u16_1);

  input.target_velocity_mm_s = INT16_MAX + 1L;
  input.gyro_z_raw_mdps = (INT16_MAX + 1L) * 100L;
  assert(!f413_trace_compact_pack_fast(&input, &fast));
  assert(fast.target_velocity_mm_s == INT16_MAX);
  assert(fast.gyro_z_raw_x100mdps == INT16_MAX);

  printf("f413_trace_compact_host_test PASS fast=%zu slow=%zu usable=%u\n",
         sizeof(fast),
         sizeof(slow),
         (unsigned int)F413_TRACE_COMPACT_USABLE_RECORDS);
  return 0;
}
