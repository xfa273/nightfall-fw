#ifndef F413_WALL_DISTANCE_H_
#define F413_WALL_DISTANCE_H_

#include <stdbool.h>
#include <stdint.h>

#include "f413_wall_sensor.h"

#define F413_WALL_DISTANCE_CH_FR    (1U << 0)
#define F413_WALL_DISTANCE_CH_R     (1U << 1)
#define F413_WALL_DISTANCE_CH_FL    (1U << 2)
#define F413_WALL_DISTANCE_CH_L     (1U << 3)
#define F413_WALL_DISTANCE_CH_FSUM  (1U << 4)

typedef struct
{
  f413_wall_sensor_snapshot_t adc;

  float fr_mm;
  float r_mm;
  float fl_mm;
  float l_mm;
  float front_sum_mm;

  float fr_mm_unwarped;
  float fl_mm_unwarped;
  float front_sum_mm_unwarped;

  uint16_t valid_mask;
  uint16_t extrapolated_mask;
  uint16_t saturated_mask;
  uint16_t below_signal_mask;

  bool front_valid;
  bool right_valid;
  bool left_valid;
  bool distance_params_loaded;
} f413_wall_distance_snapshot_t;

void f413_wall_distance_init(void);
bool f413_wall_distance_params_loaded(void);
bool f413_wall_distance_convert_snapshot(const f413_wall_sensor_snapshot_t* adc,
                                         f413_wall_distance_snapshot_t* out);
bool f413_wall_distance_read_snapshot(f413_wall_distance_snapshot_t* out);
bool f413_wall_distance_front_present(const f413_wall_distance_snapshot_t* s);
bool f413_wall_distance_side_present(const f413_wall_distance_snapshot_t* s, bool right);
bool f413_wall_distance_front_unwarped_mm(float* distance_mm);

#endif
