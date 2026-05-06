#ifndef NIGHTFALL_NVM_TRACE_LOG_H_
#define NIGHTFALL_NVM_TRACE_LOG_H_

#include <stdint.h>

#include "nvm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NVM_TRACE_LOG_MAGIC (0x544C4F47UL)
#define NVM_TRACE_LOG_SCHEMA_VERSION (0x00010000UL)

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
    int16_t encoder_l;
    int16_t encoder_r;
    int16_t motor_out_l;
    int16_t motor_out_r;
    int16_t omega_z_mdps;
    uint16_t flags;
} nvm_trace_log_record_t;

nvm_status_t nvm_trace_log_format(void);
nvm_status_t nvm_trace_log_get_header(nvm_trace_log_header_t* out);
nvm_status_t nvm_trace_log_append(const nvm_trace_log_record_t* record);
nvm_status_t nvm_trace_log_read_latest(uint32_t newest_index_from_tail,
                                       nvm_trace_log_record_t* out);

#ifdef __cplusplus
}
#endif

#endif
