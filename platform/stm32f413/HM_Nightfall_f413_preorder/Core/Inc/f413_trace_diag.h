#ifndef F413_TRACE_DIAG_H_
#define F413_TRACE_DIAG_H_

#include <stdint.h>

#include "nvm_trace_log.h"

typedef void (*f413_trace_diag_fill_sample_fn)(nvm_trace_log_record_t* out, uint32_t seq);
typedef void (*f413_trace_diag_get_context_fn)(uint8_t* mode,
                                               uint8_t* op_case,
                                               uint8_t* sub,
                                               uint8_t* test_id);
typedef const char* (*f413_trace_diag_name_fn)(uint8_t value);
typedef const char* (*f413_trace_diag_case_name_fn)(uint8_t mode, uint8_t op_case);
typedef void (*f413_trace_diag_emit_extra_csv_meta_fn)(void);

typedef struct {
  f413_trace_diag_fill_sample_fn fill_sample;
  f413_trace_diag_get_context_fn get_context;
  f413_trace_diag_name_fn op_mode_name;
  f413_trace_diag_case_name_fn op_case_name;
  f413_trace_diag_case_name_fn op_sub_name;
  f413_trace_diag_emit_extra_csv_meta_fn emit_extra_csv_meta;
} f413_trace_diag_config_t;

void f413_trace_diag_config(const f413_trace_diag_config_t* config);
void f413_trace_diag_print_header(const nvm_trace_log_header_t* header);
void f413_trace_diag_run_format_once(void);
void f413_trace_diag_run_append_sample_once(void);
void f413_trace_diag_run_dump_latest_once(void);
void f413_trace_diag_run_dump_csv_once(void);
void f413_trace_diag_run_dump_csv_all_once(void);
void f413_trace_diag_run_dump_bin_once(void);
void f413_trace_diag_run_dump_bin_all_once(void);
void f413_trace_diag_run_selftest_once(void);

#endif
