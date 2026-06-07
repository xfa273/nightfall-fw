#ifndef F413_NVM_DIAG_H_
#define F413_NVM_DIAG_H_

#include <stdbool.h>
#include <stdint.h>

#include "nvm.h"
#include "nvm_identity.h"

const char* f413_nvm_diag_identity_family_name(uint32_t family);
void f413_nvm_diag_emit_identity_meta(const nvm_identity_block_t* id);
void f413_nvm_diag_run_identity_status_once(nvm_status_t status,
                                            const nvm_identity_block_t* identity);
void f413_nvm_diag_run_sensor_params_status_once(void);
void f413_nvm_diag_run_nvm_status_once(void);
bool f413_nvm_diag_run_distance_test(void);
bool f413_nvm_diag_run_sensor_test(void);
bool f413_nvm_diag_run_maze_test(void);
bool f413_nvm_diag_run_trace_log_test(void);
bool f413_nvm_diag_verify_distance_load_only(void);
bool f413_nvm_diag_verify_sensor_load_only(void);
bool f413_nvm_diag_verify_maze_load_only(void);
bool f413_nvm_diag_verify_trace_log_load_only(void);
void f413_nvm_diag_run_all_tests(void);
void f413_nvm_diag_verify_all_load_only(void);

#endif
