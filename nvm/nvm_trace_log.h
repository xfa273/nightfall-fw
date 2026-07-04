#ifndef NIGHTFALL_NVM_TRACE_LOG_H_
#define NIGHTFALL_NVM_TRACE_LOG_H_

#include <stdint.h>

#include "nvm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NVM_TRACE_LOG_MAGIC (0x544C4F47UL)
#define NVM_TRACE_LOG_SCHEMA_VERSION (0x00060000UL)

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t length;
    uint32_t crc;
    uint32_t record_size;
    uint32_t record_capacity;
    uint32_t write_index;
    uint32_t total_records;
} nvm_trace_log_header_t;

typedef struct __attribute__((packed)) {
    uint32_t seq;
    uint32_t timestamp_ms;
    int32_t target_distance_x1000;
    int32_t distance_mm;
    int32_t angle_mdeg;
    int32_t target_velocity_mm_s;
    int32_t real_velocity_mm_s;
    int32_t accel_velocity_mm_s;
    int32_t target_omega_mdps;
    int32_t real_omega_mdps;
    int32_t gyro_z_raw_mdps;
    int32_t target_angle_mdeg;
    int32_t accel_forward_mm_s2;
    int32_t reserved_i32_0;
    int32_t reserved_i32_1;
    int32_t reserved_i32_2;
    int32_t reserved_i32_3;
    int16_t encoder_l;
    int16_t encoder_r;
    int16_t motor_out_l;
    int16_t motor_out_r;
    uint16_t adc_fr;
    uint16_t adc_r;
    uint16_t adc_fl;
    uint16_t adc_l;
    uint16_t adc_vbat;
    uint16_t wall_read_fr;
    uint16_t wall_read_r;
    uint16_t wall_read_fl;
    uint16_t wall_read_l;
    uint16_t flags;
    uint8_t op_mode;
    uint8_t op_case;
    uint8_t op_sub;
    uint8_t test_id;
    uint16_t reserved_u16_0;
    uint16_t reserved_u16_1;
} nvm_trace_log_record_t;

nvm_status_t nvm_trace_log_format(void);
nvm_status_t nvm_trace_log_get_header(nvm_trace_log_header_t* out);
nvm_status_t nvm_trace_log_append(const nvm_trace_log_record_t* record);
nvm_status_t nvm_trace_log_append_cached(nvm_trace_log_header_t* header,
                                         const nvm_trace_log_record_t* record,
                                         uint8_t commit_header);
nvm_status_t nvm_trace_log_commit_header(const nvm_trace_log_header_t* header);
nvm_status_t nvm_trace_log_read_latest(uint32_t newest_index_from_tail,
                                       nvm_trace_log_record_t* out);

#ifdef __cplusplus
}
#endif

#endif
