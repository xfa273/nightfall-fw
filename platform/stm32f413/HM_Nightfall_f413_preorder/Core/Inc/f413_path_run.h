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
#define NIGHTFALL_F413_PATH_MAX_CODES     (1024U)
#define NIGHTFALL_F413_PATH_TIMEOUT_MS    (5000U)
#define NIGHTFALL_F413_PATH_VELOCITY_CAP  (1500.0f)
#define NIGHTFALL_F413_PATH_DIAGONAL_VELOCITY_CAP (1000.0f)
#define NIGHTFALL_F413_PATH_TURN_VELOCITY_CAP (500.0f)
#define NIGHTFALL_F413_PATH_OMEGA_CAP     (2200.0f)

#if (NIGHTFALL_F413_REAL_RUN_PATH_ENABLED != 0U)
void f413_path_run_print_preview(void);
void f413_path_run_session_once(uint8_t mode,
                                uint8_t case_index,
                                uint16_t base_trace_flag,
                                const char* label);
void f413_path_run_solver_session_once(uint16_t base_trace_flag);
void f413_path_run_custom_path_session_once(const char* label,
                                            uint8_t mode,
                                            uint8_t case_index,
                                            const uint16_t* codes,
                                            uint16_t code_count,
                                            uint16_t base_trace_flag);
#endif

#endif
