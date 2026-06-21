#include "f413_run_features.h"

#include <stddef.h>

static f413_run_features_t g_features;

void f413_run_features_reset(void)
{
  g_features.wall_control_enabled = true;
  g_features.wall_end_correction_enabled = true;
  g_features.front_wall_correction_enabled = true;
  g_features.angle_accum_mode = true;
  g_features.test_mode_run = false;
}

void f413_run_features_set(const f413_run_features_t* features)
{
  if (features != NULL)
  {
    g_features = *features;
  }
}

f413_run_features_t f413_run_features_get(void)
{
  return g_features;
}

bool f413_run_features_wall_control_enabled(void)
{
  return g_features.wall_control_enabled;
}

bool f413_run_features_wall_end_correction_enabled(void)
{
  return g_features.wall_end_correction_enabled;
}

bool f413_run_features_front_wall_correction_enabled(void)
{
  return g_features.front_wall_correction_enabled;
}

bool f413_run_features_angle_accum_mode(void)
{
  return g_features.angle_accum_mode;
}

bool f413_run_features_test_mode_run(void)
{
  return g_features.test_mode_run;
}
