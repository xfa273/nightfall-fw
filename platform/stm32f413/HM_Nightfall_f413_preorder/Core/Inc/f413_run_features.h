#ifndef F413_RUN_FEATURES_H_
#define F413_RUN_FEATURES_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool wall_control_enabled;
  bool wall_end_correction_enabled;
  bool front_wall_correction_enabled;
  bool angle_accum_mode;
  bool test_mode_run;
} f413_run_features_t;

void f413_run_features_reset(void);
void f413_run_features_set(const f413_run_features_t* features);
f413_run_features_t f413_run_features_get(void);

bool f413_run_features_wall_control_enabled(void);
bool f413_run_features_wall_end_correction_enabled(void);
bool f413_run_features_front_wall_correction_enabled(void);
bool f413_run_features_angle_accum_mode(void);
bool f413_run_features_test_mode_run(void);

#endif
