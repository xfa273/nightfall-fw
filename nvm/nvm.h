#ifndef NIGHTFALL_NVM_H_
#define NIGHTFALL_NVM_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NVM_STATUS_OK = 0,
    NVM_STATUS_INVALID_ARG,
    NVM_STATUS_UNSUPPORTED,
    NVM_STATUS_NOT_FOUND,
    NVM_STATUS_INTEGRITY_ERROR,
    NVM_STATUS_HW_ERROR,
} nvm_status_t;

typedef enum {
    NVM_AREA_IDENTITY = 0,
    NVM_AREA_DISTANCE_PARAMS,
    NVM_AREA_FLASH_PARAMS,
    NVM_AREA_MAZE_MAP,
    NVM_AREA_TRACE_LOG,
    NVM_AREA_COUNT,
} nvm_area_t;

typedef struct {
    nvm_area_t area;
    uint32_t base_address;
    uint32_t size_bytes;
    uint32_t schema_version;
} nvm_area_info_t;

nvm_status_t nvm_init(void);
nvm_status_t nvm_get_area_info(nvm_area_t area, nvm_area_info_t* out);
nvm_status_t nvm_read(nvm_area_t area, uint32_t offset, void* out, size_t len);
nvm_status_t nvm_write(nvm_area_t area, uint32_t offset, const void* data, size_t len);
nvm_status_t nvm_erase(nvm_area_t area);

#ifdef __cplusplus
}
#endif

#endif
