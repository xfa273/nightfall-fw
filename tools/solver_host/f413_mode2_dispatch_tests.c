#include "f413_mode2.h"
#include "f413_mode_shortest.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

static unsigned int g_checks;
static unsigned int g_failures;
static unsigned int g_config_calls;
static unsigned int g_path_calls;
static f413_shortest_case_config_t g_last_config;
static f413_run_features_t g_last_features;
static uint8_t g_last_mode;
static uint8_t g_last_case;
static uint16_t g_last_codes[16];
static uint16_t g_last_code_count;

#define CHECK(condition)                                                        \
  do                                                                            \
  {                                                                             \
    g_checks++;                                                                 \
    if (!(condition))                                                           \
    {                                                                           \
      g_failures++;                                                             \
      fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);  \
    }                                                                           \
  } while (0)

void f413_mode_shortest_run_case(uint8_t mode, uint8_t op_case)
{
  (void)mode;
  (void)op_case;
}

void f413_mode_shortest_run_config(const f413_shortest_case_config_t* config)
{
  g_config_calls++;
  if (config != NULL)
  {
    g_last_config = *config;
  }
}

void f413_mode_shortest_run_case0_path(const char* label,
                                       uint8_t mode,
                                       uint8_t case_index,
                                       const uint16_t* codes,
                                       uint16_t code_count)
{
  (void)label;
  (void)mode;
  (void)case_index;
  (void)codes;
  (void)code_count;
}

void f413_mode_shortest_run_path_config(const char* label,
                                        uint8_t mode,
                                        uint8_t case_index,
                                        const uint16_t* codes,
                                        uint16_t code_count,
                                        const f413_run_features_t* features)
{
  (void)label;
  g_path_calls++;
  g_last_mode = mode;
  g_last_case = case_index;
  g_last_code_count = code_count;
  if (features != NULL)
  {
    g_last_features = *features;
  }
  for (uint16_t index = 0U;
       (index < code_count) && (index < 16U);
       index++)
  {
    g_last_codes[index] = codes[index];
  }
}

static void check_case6_fixed_path(void)
{
  static const uint16_t expected[] = {
      203U, 701U, 1001U, 802U, 1001U, 703U, 202U};

  f413_mode2_run_case(6U);

  CHECK(g_path_calls == 1U);
  CHECK(g_config_calls == 0U);
  CHECK(g_last_mode == 2U);
  CHECK(g_last_case == 6U);
  CHECK(g_last_code_count ==
        (uint16_t)(sizeof(expected) / sizeof(expected[0])));
  for (size_t index = 0U; index < sizeof(expected) / sizeof(expected[0]); index++)
  {
    CHECK(g_last_codes[index] == expected[index]);
  }
  CHECK(!g_last_features.wall_control_enabled);
  CHECK(!g_last_features.wall_end_correction_enabled);
  CHECK(!g_last_features.front_wall_correction_enabled);
  CHECK(g_last_features.angle_accum_mode);
  CHECK(!g_last_features.test_mode_run);
}

static void check_case7_keeps_saved_maze_planner(void)
{
  g_config_calls = 0U;
  g_path_calls = 0U;

  f413_mode2_run_case(7U);

  CHECK(g_path_calls == 0U);
  CHECK(g_config_calls == 1U);
  CHECK(g_last_config.mode == 2U);
  CHECK(g_last_config.op_case == 7U);
  CHECK(g_last_config.diagonal_time_plan);
  CHECK(g_last_config.features.wall_control_enabled);
  CHECK(g_last_config.features.wall_end_correction_enabled);
  CHECK(g_last_config.features.front_wall_correction_enabled);
  CHECK(g_last_config.features.angle_accum_mode);
  CHECK(!g_last_config.features.test_mode_run);
}

int main(void)
{
  check_case6_fixed_path();
  check_case7_keeps_saved_maze_planner();

  if (g_failures != 0U)
  {
    fprintf(stderr, "f413 mode2 dispatch tests: %u/%u failed\n",
            g_failures, g_checks);
    return 1;
  }
  printf("f413 mode2 dispatch tests: %u checks passed\n", g_checks);
  return 0;
}
