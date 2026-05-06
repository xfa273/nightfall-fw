#ifndef NIGHTFALL_NVM_PARAMS_H_
#define NIGHTFALL_NVM_PARAMS_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#if defined(STM32F405xx)
#include "flash_params.h"
typedef flash_params_t nvm_sensor_params_t;
#elif defined(STM32F413xx)
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t length;
    uint32_t crc;

    uint16_t base_l;
    uint16_t base_r;
    uint16_t base_f;

    uint16_t wall_offset_r;
    uint16_t wall_offset_l;
    uint16_t wall_offset_fr;
    uint16_t wall_offset_fl;

    float imu_offset_z;

    uint32_t reserved[8];
} nvm_sensor_params_t;
#endif

#ifdef __cplusplus
extern "C" {
#endif

bool nvm_params_distance_load_and_apply(void);
HAL_StatusTypeDef nvm_params_distance_save(const float x_fl[3], const float y_fl[3],
                                           const float x_fr[3], const float y_fr[3],
                                           const float x_fsum[3], const float y_fsum[3]);

#if defined(STM32F405xx) || defined(STM32F413xx)
bool nvm_params_sensor_load(nvm_sensor_params_t* out);
HAL_StatusTypeDef nvm_params_sensor_save(const nvm_sensor_params_t* in);
void nvm_params_sensor_defaults(nvm_sensor_params_t* out);
#endif

HAL_StatusTypeDef nvm_maze_enable_write(void);
HAL_StatusTypeDef nvm_maze_disable_write(void);
HAL_StatusTypeDef nvm_maze_write_halfword(uint32_t address, uint16_t data);
HAL_StatusTypeDef nvm_maze_write_word(uint32_t address, uint32_t data);
uint16_t nvm_maze_read_halfword(uint32_t address);
uint32_t nvm_maze_read_word(uint32_t address);
HAL_StatusTypeDef nvm_maze_save_map(const uint16_t* cells, uint32_t cell_count);
bool nvm_maze_load_map(uint16_t* cells, uint32_t cell_count);

#ifdef __cplusplus
}
#endif

#endif
