#ifndef F413_WALL_SENSOR_H_
#define F413_WALL_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "nvm_params.h"
#include "stm32f4xx_hal.h"

typedef struct
{
  uint16_t fr_off;
  uint16_t r_off;
  uint16_t fl_off;
  uint16_t l_off;
  uint16_t vbat_off;
  uint16_t fr_on;
  uint16_t r_on;
  uint16_t fl_on;
  uint16_t l_on;
  uint16_t vbat_on;
  int32_t fr_delta;
  int32_t r_delta;
  int32_t fl_delta;
  int32_t l_delta;
  bool front_wall;
  bool right_wall;
  bool left_wall;
  bool saturated;
} f413_wall_sensor_snapshot_t;

bool f413_wall_sensor_start_async(void);
void f413_wall_sensor_tim6_tick(void);
void f413_wall_sensor_adc_complete(ADC_HandleTypeDef* hadc);
bool f413_wall_sensor_read_snapshot(f413_wall_sensor_snapshot_t* out);
bool f413_wall_sensor_read_adc_raw(uint16_t* fr, uint16_t* r, uint16_t* fl, uint16_t* l, uint16_t* vbat);
void f413_wall_sensor_get_control_base(uint16_t* base_l, uint16_t* base_r, uint16_t* base_f);
HAL_StatusTypeDef f413_wall_sensor_calibrate_offsets_and_save(uint16_t sample_count,
                                                              uint32_t sample_interval_ms,
                                                              nvm_sensor_params_t* saved);
HAL_StatusTypeDef f413_wall_sensor_calibrate_side_base_and_save(uint16_t sample_count,
                                                                uint32_t sample_interval_ms,
                                                                nvm_sensor_params_t* saved);
void f413_wall_sensor_get_debug_state(uint16_t* off_r,
                                      uint16_t* off_l,
                                      uint16_t* off_fr,
                                      uint16_t* off_fl,
                                      uint8_t* ready_mask,
                                      uint8_t* phase,
                                      uint8_t* inflight);

#endif
