#ifndef F413_TRACE_SAMPLE_H_
#define F413_TRACE_SAMPLE_H_

#include <stdbool.h>
#include <stdint.h>

#include "nvm.h"
#include "nvm_identity.h"
#include "nvm_trace_log.h"
#include "f413_run_session.h"

typedef uint32_t (*f413_trace_sample_tick_fn)(void);
typedef int16_t (*f413_trace_sample_encoder_count_fn)(void);
typedef bool (*f413_trace_sample_read_adc_fn)(uint16_t* fr,
                                              uint16_t* r,
                                              uint16_t* fl,
                                              uint16_t* l,
                                              uint16_t* vbat);

typedef struct {
  f413_trace_sample_tick_fn get_tick_ms;
  f413_trace_sample_encoder_count_fn encoder_l_count;
  f413_trace_sample_encoder_count_fn encoder_r_count;
  f413_trace_sample_read_adc_fn read_adc_raw;
} f413_trace_sample_config_t;

#define F413_TRACE_SAMPLE_FRONT_MATCH_MARKER (0xF400U)
#define F413_TRACE_SAMPLE_FRONT_MATCH_MARKER_MASK (0xFF00U)

void f413_trace_sample_config(const f413_trace_sample_config_t* config);
void f413_trace_sample_set_identity(nvm_status_t status, const nvm_identity_block_t* identity);
void f413_trace_sample_set_context(uint8_t mode, uint8_t op_case, uint8_t sub, uint8_t test_id);
void f413_trace_sample_get_context(uint8_t* mode, uint8_t* op_case, uint8_t* sub, uint8_t* test_id);
void f413_trace_sample_set_front_match(bool active,
                                       uint8_t phase,
                                       float fr_mm,
                                       float fl_mm,
                                       float position_error_mm,
                                       float yaw_error_mm,
                                       uint16_t state_elapsed_ms);
void f413_trace_sample_update_observe_cache(void);
void f413_trace_sample_emit_extra_csv_meta(void);
void f413_trace_sample_fill(nvm_trace_log_record_t* out, uint32_t seq);
void f413_trace_sample_fill_control(nvm_trace_log_record_t* out,
                                    uint32_t seq,
                                    uint32_t timestamp_ms,
                                    uint16_t mode_flags);
void f413_trace_sample_record_result(uint8_t test_id,
                                     f413_run_session_abort_reason_t abort_reason,
                                     float distance_mm,
                                     float angle_deg);

#endif
