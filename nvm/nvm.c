#include "nvm.h"

#include <string.h>

#if defined(STM32F405xx)

#define NVM_STM32F405_IDENTITY_BASE (0x08080000UL)
#define NVM_STM32F405_DISTANCE_PARAMS_BASE (0x080A0000UL)
#define NVM_STM32F405_FLASH_PARAMS_BASE (0x080C0000UL)
#define NVM_STM32F405_MAZE_MAP_BASE (0x080E0000UL)

#define NVM_STM32F405_SECTOR_SIZE_BYTES (128U * 1024U)

static const nvm_area_info_t g_nvm_area_table[NVM_AREA_COUNT] = {
    {NVM_AREA_IDENTITY, NVM_STM32F405_IDENTITY_BASE, NVM_STM32F405_SECTOR_SIZE_BYTES, 0x00010000UL},
    {NVM_AREA_DISTANCE_PARAMS, NVM_STM32F405_DISTANCE_PARAMS_BASE, NVM_STM32F405_SECTOR_SIZE_BYTES, 0x00010000UL},
    {NVM_AREA_FLASH_PARAMS, NVM_STM32F405_FLASH_PARAMS_BASE, NVM_STM32F405_SECTOR_SIZE_BYTES, 0x00010001UL},
    {NVM_AREA_MAZE_MAP, NVM_STM32F405_MAZE_MAP_BASE, NVM_STM32F405_SECTOR_SIZE_BYTES, 0x00000000UL},
    {NVM_AREA_TRACE_LOG, 0x00000000UL, 0U, 0U},
};

#elif defined(STM32F413xx)

static const nvm_area_info_t g_nvm_area_table[NVM_AREA_COUNT] = {
    {NVM_AREA_IDENTITY, 0x00000000UL, 0U, 0U},
    {NVM_AREA_DISTANCE_PARAMS, 0x00000000UL, 0U, 0U},
    {NVM_AREA_FLASH_PARAMS, 0x00000000UL, 0U, 0U},
    {NVM_AREA_MAZE_MAP, 0x00000000UL, 0U, 0U},
    {NVM_AREA_TRACE_LOG, 0x00000000UL, 0U, 0U},
};

#else

static const nvm_area_info_t g_nvm_area_table[NVM_AREA_COUNT] = {
    {NVM_AREA_IDENTITY, 0x00000000UL, 0U, 0U},
    {NVM_AREA_DISTANCE_PARAMS, 0x00000000UL, 0U, 0U},
    {NVM_AREA_FLASH_PARAMS, 0x00000000UL, 0U, 0U},
    {NVM_AREA_MAZE_MAP, 0x00000000UL, 0U, 0U},
    {NVM_AREA_TRACE_LOG, 0x00000000UL, 0U, 0U},
};

#endif

nvm_status_t nvm_init(void) {
    return NVM_STATUS_OK;
}

nvm_status_t nvm_get_area_info(nvm_area_t area, nvm_area_info_t* out) {
    if (out == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }
    if ((uint32_t)area >= (uint32_t)NVM_AREA_COUNT) {
        return NVM_STATUS_INVALID_ARG;
    }

    *out = g_nvm_area_table[(uint32_t)area];
    if (out->base_address == 0UL || out->size_bytes == 0U) {
        return NVM_STATUS_UNSUPPORTED;
    }
    return NVM_STATUS_OK;
}

nvm_status_t nvm_read(nvm_area_t area, uint32_t offset, void* out, size_t len) {
    nvm_area_info_t info;
    nvm_status_t st;

    if (out == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }

    st = nvm_get_area_info(area, &info);
    if (st != NVM_STATUS_OK) {
        return st;
    }

    if ((size_t)offset > (size_t)info.size_bytes) {
        return NVM_STATUS_INVALID_ARG;
    }
    if (len > ((size_t)info.size_bytes - (size_t)offset)) {
        return NVM_STATUS_INVALID_ARG;
    }

    memcpy(out, (const void*)(uintptr_t)(info.base_address + offset), len);
    return NVM_STATUS_OK;
}

nvm_status_t nvm_write(nvm_area_t area, uint32_t offset, const void* data, size_t len) {
    (void)area;
    (void)offset;
    (void)data;
    (void)len;
    return NVM_STATUS_UNSUPPORTED;
}

nvm_status_t nvm_erase(nvm_area_t area) {
    (void)area;
    return NVM_STATUS_UNSUPPORTED;
}
