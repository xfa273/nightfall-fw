#ifndef F413_DIAG_H_
#define F413_DIAG_H_

#include <stdbool.h>

/* Bring-up fixtures overwrite real calibration/map/trace data. No UART unlock:
 * enabling them requires a separate, explicitly opted-in maintenance build. */
#ifndef NIGHTFALL_F413_DESTRUCTIVE_NVM_DIAGNOSTICS
#define NIGHTFALL_F413_DESTRUCTIVE_NVM_DIAGNOSTICS 0
#endif
#if (NIGHTFALL_F413_DESTRUCTIVE_NVM_DIAGNOSTICS != 0) && \
    (NIGHTFALL_F413_DESTRUCTIVE_NVM_DIAGNOSTICS != 1)
#error "NIGHTFALL_F413_DESTRUCTIVE_NVM_DIAGNOSTICS must be 0 or 1"
#endif

bool f413_diag_require_nvm_writes(void);

bool f413_diag_run_distance_nvm_test(void);
bool f413_diag_run_sensor_nvm_test(void);
bool f413_diag_run_maze_nvm_test(void);
bool f413_diag_run_trace_log_nvm_test(void);
bool f413_diag_verify_distance_nvm_load_only(void);
bool f413_diag_verify_sensor_nvm_load_only(void);
bool f413_diag_verify_maze_nvm_load_only(void);
bool f413_diag_verify_trace_log_nvm_load_only(void);
void f413_diag_run_all_nvm_tests(void);
void f413_diag_verify_all_nvm_load_only(void);

#endif
