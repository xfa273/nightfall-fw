/*
 * distance_params.h
 *
 *  Persist 3-point warp calibration for front distance sensors (FL/FR/FSUM)
 */
#ifndef INC_DISTANCE_PARAMS_H_
#define INC_DISTANCE_PARAMS_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Flash layout for distance warp parameters
#define DIST_PARAMS_START_ADDRESS  (0x080A0000UL) // Sector 9
#define DIST_PARAMS_SECTOR         (FLASH_SECTOR_9)

#define DIST_PARAMS_MAGIC   (0x44495354UL)  // 'DIST'
#define DIST_PARAMS_VERSION (0x00010000UL)

// Save 3-point warps for FL/FR/FSUM
HAL_StatusTypeDef distance_params_save(const float x_fl[3], const float y_fl[3],
                                       const float x_fr[3], const float y_fr[3],
                                       const float x_fsum[3], const float y_fsum[3]);

// Load from flash and apply to sensor_distance module.
// Returns true if valid params loaded and applied.
bool distance_params_load_and_apply(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_DISTANCE_PARAMS_H_ */
