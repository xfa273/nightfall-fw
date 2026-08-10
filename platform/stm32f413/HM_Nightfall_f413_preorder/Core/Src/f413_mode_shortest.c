#include "f413_mode_shortest.h"

#include <stdbool.h>
#include <stddef.h>

#include "f413_path_run.h"
#include "f413_route_preview.h"
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

typedef enum {
  F413_CASE0_PATH_VALID = 0,
  F413_CASE0_PATH_UNSUPPORTED,
  F413_CASE0_PATH_REQUIRES_CARDINAL,
  F413_CASE0_PATH_REQUIRES_DIAGONAL,
} f413_case0_path_validation_t;

static f413_case0_path_validation_t f413_mode_shortest_validate_case0_path(
    const uint16_t* codes,
    uint16_t code_count,
    uint16_t* invalid_index)
{
  bool diagonal = false;
  uint16_t i;

  if ((codes == NULL) || (code_count == 0U))
  {
    return F413_CASE0_PATH_UNSUPPORTED;
  }

  for (i = 0U; i < code_count; i++)
  {
    const uint16_t code = codes[i];
    f413_case0_path_validation_t result = F413_CASE0_PATH_VALID;

    if ((code > 200U) && (code < 300U))
    {
      result = diagonal ? F413_CASE0_PATH_REQUIRES_CARDINAL : F413_CASE0_PATH_VALID;
    }
    else if (code > 1000U)
    {
      result = diagonal ? F413_CASE0_PATH_VALID : F413_CASE0_PATH_REQUIRES_DIAGONAL;
    }
    else
    {
      switch (code)
      {
        case 300U:
        case 400U:
        case 501U:
        case 502U:
        case 601U:
        case 602U:
          result = diagonal ? F413_CASE0_PATH_REQUIRES_CARDINAL : F413_CASE0_PATH_VALID;
          break;
        case 701U:
        case 702U:
        case 901U:
        case 902U:
          if (diagonal)
          {
            result = F413_CASE0_PATH_REQUIRES_CARDINAL;
          }
          else
          {
            diagonal = true;
          }
          break;
        case 703U:
        case 704U:
        case 903U:
        case 904U:
          if (!diagonal)
          {
            result = F413_CASE0_PATH_REQUIRES_DIAGONAL;
          }
          else
          {
            diagonal = false;
          }
          break;
        case 801U:
        case 802U:
          result = diagonal ? F413_CASE0_PATH_VALID : F413_CASE0_PATH_REQUIRES_DIAGONAL;
          break;
        default:
          result = F413_CASE0_PATH_UNSUPPORTED;
          break;
      }
    }

    if (result != F413_CASE0_PATH_VALID)
    {
      if (invalid_index != NULL)
      {
        *invalid_index = i;
      }
      return result;
    }
  }

  return F413_CASE0_PATH_VALID;
}

static const char* f413_mode_shortest_case0_validation_text(
    f413_case0_path_validation_t result)
{
  switch (result)
  {
    case F413_CASE0_PATH_REQUIRES_CARDINAL: return "requires cardinal entry";
    case F413_CASE0_PATH_REQUIRES_DIAGONAL: return "requires diagonal entry";
    case F413_CASE0_PATH_UNSUPPORTED: return "unsupported code";
    default: return "valid";
  }
}

void f413_mode_shortest_run_config(const f413_shortest_case_config_t* config)
{
#if (NIGHTFALL_F413_REAL_RUN_PATH_ENABLED != 0U)
  if (config == NULL)
  {
    trace_printf("[RUN-TEST] shortest config missing\r\n");
    return;
  }

  f413_run_features_set(&config->features);
  if (config->diagonal_time_plan ?
      f413_route_build_mode2_path(config->op_case) :
      solver_build_path(config->mode, config->op_case))
  {
    trace_printf("[RUN-TEST] shortest path ready mode=%u case=%u planner=%s\r\n",
                 (unsigned int)config->mode,
                 (unsigned int)config->op_case,
                 config->diagonal_time_plan ? "KERI-fixed-memory" : "legacy");
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
  f413_run_features_reset();
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
    .diagonal_time_plan = false,
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
  uint16_t invalid_index = 0U;
  const f413_case0_path_validation_t validation =
      f413_mode_shortest_validate_case0_path(codes, code_count, &invalid_index);
  const f413_run_features_t test_features = {
    .wall_control_enabled = true,
    .wall_end_correction_enabled = false,
    .front_wall_correction_enabled = false,
    .angle_accum_mode = true,
    .test_mode_run = true,
  };

  if (validation != F413_CASE0_PATH_VALID)
  {
    const uint16_t invalid_code = ((codes != NULL) && (invalid_index < code_count))
        ? codes[invalid_index]
        : 0U;
    trace_printf("[OP-UI][PATH-TEST] rejected %s code[%u]=%u (%s)\r\n",
                 (label != NULL) ? label : "unnamed",
                 (unsigned int)invalid_index,
                 (unsigned int)invalid_code,
                 f413_mode_shortest_case0_validation_text(validation));
    return;
  }

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
