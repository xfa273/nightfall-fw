#ifndef F413_PATH_RUN_H_
#define F413_PATH_RUN_H_

#include <stdbool.h>
#include <stdint.h>

#ifndef NIGHTFALL_F413_REAL_RUN_PATH_ENABLED
#define NIGHTFALL_F413_REAL_RUN_PATH_ENABLED (1U)
#endif

#define NIGHTFALL_F413_SOLVER_MODE      (2U)
#define NIGHTFALL_F413_SOLVER_CASE      (1U)
#define NIGHTFALL_F413_PATH_PREVIEW_MAX (24U)
#define NIGHTFALL_F413_PATH_VELOCITY      (200.0f)
#define NIGHTFALL_F413_PATH_OMEGA         (200.0f)
#define NIGHTFALL_F413_PATH_HALF_CELL_MM  (45.0f)
#define NIGHTFALL_F413_PATH_COAST_MS      (60U)
#define NIGHTFALL_F413_PATH_MAX_CODES     (256U)
#define NIGHTFALL_F413_PATH_TIMEOUT_MS    (5000U)
#define NIGHTFALL_F413_PATH_VELOCITY_CAP  (NIGHTFALL_F413_PATH_VELOCITY)

#define NIGHTFALL_F413_OP_TEST_VEL_CAP    (450.0f)
#define NIGHTFALL_F413_OP_TEST_DIAG_SCALE (0.75f)

#if (NIGHTFALL_F413_REAL_RUN_PATH_ENABLED != 0U)
float f413_path_run_test_velocity_for_mode(uint8_t mode, bool fast);
void f413_path_run_print_preview(void);
void f413_path_run_solver_session_once(uint16_t base_trace_flag);
void f413_path_run_code_sequence_once(const char* label,
                                      uint8_t mode,
                                      uint8_t case_index,
                                      const uint16_t* codes,
                                      uint16_t code_count,
                                      float straight_velocity,
                                      float diagonal_velocity);
#endif

#endif
