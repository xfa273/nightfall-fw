#ifndef F413_DIAG_H_
#define F413_DIAG_H_

#include <stdbool.h>

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
