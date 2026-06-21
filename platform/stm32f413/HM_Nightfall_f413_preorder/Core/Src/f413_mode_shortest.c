#include "f413_mode_shortest.h"

#include <stdbool.h>
#include <stddef.h>

#include "f413_path_run.h"
#include "f413_run_features.h"
#include "f413_trace_flags.h"
#include "path.h"
#include "solver.h"
#include "trace.h"

static const f413_run_features_t k_default_shortest_features = {
  .wall_control_enabled = true,
  .wall_end_correction_enabled = true,
  .front_wall_correction_enabled = true,
  .angle_accum_mode = true,
  .test_mode_run = false,
};

void f413_mode_shortest_run_config(const f413_shortest_case_config_t* config)
{
#if (NIGHTFALL_F413_REAL_RUN_PATH_ENABLED != 0U)
  if (config == NULL)
  {
    trace_printf("[RUN-TEST] shortest config missing\r\n");
    return;
  }

  f413_run_features_set(&config->features);
  if (solver_build_path(config->mode, config->op_case))
  {
    trace_printf("[RUN-TEST] shortest path ready mode=%u case=%u\r\n",
                 (unsigned int)config->mode,
                 (unsigned int)config->op_case);
    f413_path_run_print_preview();
    f413_path_run_session_once(config->mode,
                               config->op_case,
                               NIGHTFALL_F413_TRACE_MODE_SHORTEST_SAFE_FLAG,
                               (config->label != NULL) ? config->label : "shortest");
  }
  else
  {
    trace_printf("[RUN-TEST] no path for shortest mode=%u case=%u\r\n",
                 (unsigned int)config->mode,
                 (unsigned int)config->op_case);
  }
#else
  (void)config;
  trace_printf("[RUN-TEST] no-op: F413 real path runner disabled\r\n");
#endif
}

void f413_mode_shortest_run_case(uint8_t mode, uint8_t op_case)
{
  const f413_shortest_case_config_t config = {
    .mode = mode,
    .op_case = op_case,
    .label = "shortest-default",
    .features = k_default_shortest_features,
  };

  f413_mode_shortest_run_config(&config);
}

void f413_mode_shortest_run_case0_path(const char* label,
                                       uint8_t mode,
                                       uint8_t case_index,
                                       const uint16_t* codes,
                                       uint16_t code_count)
{
#if (NIGHTFALL_F413_REAL_RUN_PATH_ENABLED != 0U)
  const f413_run_features_t test_features = {
    .wall_control_enabled = true,
    .wall_end_correction_enabled = false,
    .front_wall_correction_enabled = false,
    .angle_accum_mode = true,
    .test_mode_run = true,
  };

  f413_run_features_set(&test_features);
  f413_path_run_custom_path_session_once(label,
                                         mode,
                                         case_index,
                                         codes,
                                         code_count,
                                         NIGHTFALL_F413_TRACE_MODE_SHORTEST_SAFE_FLAG);
  f413_run_features_reset();
#else
  (void)label;
  (void)mode;
  (void)case_index;
  (void)codes;
  (void)code_count;
  trace_printf("[RUN-TEST] no-op: F413 path-code test runner is disabled\r\n");
#endif
}
