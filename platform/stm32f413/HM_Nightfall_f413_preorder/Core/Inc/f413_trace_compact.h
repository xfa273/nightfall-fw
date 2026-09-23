#ifndef F413_TRACE_COMPACT_H_
#define F413_TRACE_COMPACT_H_

#include <stdbool.h>
#include <stdint.h>

#include "nvm_trace_log.h"

/*
 * The persistent FRAM schema remains nvm_trace_log_record_t (v6).  These
 * structures are only the deferred-write representation in STM32 RAM.
 * Fast control/pose fields are retained every sample.  Sensor/context and
 * auxiliary observation fields are sampled once per slow group and held when
 * records are expanded for FRAM.
 */
#define F413_TRACE_COMPACT_FAST_RECORDS (4096U)
#define F413_TRACE_COMPACT_SLOW_PERIOD_RECORDS (4U)
#define F413_TRACE_COMPACT_SLOW_RECORDS \
  (F413_TRACE_COMPACT_FAST_RECORDS / F413_TRACE_COMPACT_SLOW_PERIOD_RECORDS)
#define F413_TRACE_COMPACT_USABLE_RECORDS \
  (F413_TRACE_COMPACT_FAST_RECORDS - F413_TRACE_COMPACT_SLOW_PERIOD_RECORDS)
#define F413_TRACE_COMPACT_OMEGA_QUANTUM_MDPS (100L)

typedef struct __attribute__((packed))
{
  uint32_t timestamp_ms;
  int32_t target_distance_x1000;
  int32_t distance_mm;
  int32_t angle_mdeg;
  int16_t target_velocity_mm_s;
  int16_t real_velocity_mm_s;
  int16_t accel_velocity_mm_s;
  int16_t target_omega_x100mdps;
  int16_t real_omega_x100mdps;
  int16_t gyro_z_raw_x100mdps;
  int32_t target_angle_mdeg;
  int16_t accel_forward_mm_s2;
  int16_t encoder_l;
  int16_t encoder_r;
  int16_t motor_out_l;
  int16_t motor_out_r;
  uint16_t flags;
} f413_trace_compact_fast_t;

typedef struct __attribute__((packed))
{
  int32_t reserved_i32_0;
  int32_t reserved_i32_1;
  int32_t reserved_i32_2;
  int32_t reserved_i32_3;
  uint16_t adc_fr;
  uint16_t adc_r;
  uint16_t adc_fl;
  uint16_t adc_l;
  uint16_t adc_vbat;
  uint16_t wall_read_fr;
  uint16_t wall_read_r;
  uint16_t wall_read_fl;
  uint16_t wall_read_l;
  uint8_t op_mode;
  uint8_t op_case;
  uint8_t op_sub;
  uint8_t test_id;
  uint16_t reserved_u16_0;
  uint16_t reserved_u16_1;
} f413_trace_compact_slow_t;

/* Returns false if an int16 field had to be saturated. */
bool f413_trace_compact_pack_fast(const nvm_trace_log_record_t* source,
                                  f413_trace_compact_fast_t* destination);
void f413_trace_compact_pack_slow(const nvm_trace_log_record_t* source,
                                  f413_trace_compact_slow_t* destination);
void f413_trace_compact_expand(const f413_trace_compact_fast_t* fast,
                               const f413_trace_compact_slow_t* slow,
                               uint32_t seq,
                               nvm_trace_log_record_t* destination);

#endif
