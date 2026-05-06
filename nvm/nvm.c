#include "nvm.h"
#include "nvm_trace_log.h"

#include <string.h>

#if defined(STM32F405xx) || defined(STM32F413xx)
#include "stm32f4xx_hal.h"
#endif

#if defined(STM32F413xx)
#include "f413_control.h"
/* SPI2 バス排他: 1kHz 割り込みの IMU 読取が終わるまで待機 */
static inline void nvm_wait_spi2_free(void) {
    while (f413_ctrl_spi2_busy()) { /* spin */ }
}
#else
static inline void nvm_wait_spi2_free(void) { }
#endif

typedef enum {
    NVM_BACKEND_NONE = 0,
    NVM_BACKEND_INTERNAL_FLASH,
    NVM_BACKEND_EXTERNAL_FRAM,
} nvm_backend_t;

#if defined(STM32F413xx)
static nvm_status_t nvm_stm32f413_fram_read_area(const nvm_area_info_t* info,
                                                  uint32_t offset,
                                                  void* out,
                                                  size_t len);
static nvm_status_t nvm_stm32f413_fram_write_area(const nvm_area_info_t* info,
                                                   uint32_t offset,
                                                   const void* data,
                                                   size_t len);
static nvm_status_t nvm_stm32f413_fram_erase_area(const nvm_area_info_t* info);
#endif

static nvm_backend_t nvm_get_backend(nvm_area_t area) {
#if defined(STM32F405xx)
    switch (area) {
        case NVM_AREA_IDENTITY:
        case NVM_AREA_DISTANCE_PARAMS:
        case NVM_AREA_FLASH_PARAMS:
        case NVM_AREA_MAZE_MAP:
            return NVM_BACKEND_INTERNAL_FLASH;
        default:
            return NVM_BACKEND_NONE;
    }
#elif defined(STM32F413xx)
    switch (area) {
        case NVM_AREA_IDENTITY:
            return NVM_BACKEND_INTERNAL_FLASH;
        case NVM_AREA_DISTANCE_PARAMS:
        case NVM_AREA_FLASH_PARAMS:
        case NVM_AREA_MAZE_MAP:
        case NVM_AREA_TRACE_LOG:
            return NVM_BACKEND_EXTERNAL_FRAM;
        default:
            return NVM_BACKEND_NONE;
    }
#else
    (void)area;
    return NVM_BACKEND_NONE;
#endif
}

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

#include "main.h"

#define NVM_STM32F413_MAZE_MAP_BASE (0x08100000UL)
#define NVM_STM32F413_DISTANCE_PARAMS_BASE (0x08120000UL)
#define NVM_STM32F413_FLASH_PARAMS_BASE (0x08140000UL)
#define NVM_STM32F413_IDENTITY_BASE (0x08160000UL)
#define NVM_STM32F413_SECTOR_SIZE_BYTES (128U * 1024U)

#define NVM_STM32F413_FRAM_TOTAL_BYTES (1024U * 1024U)
#define NVM_STM32F413_FRAM_AREA_SIZE_BYTES (128U * 1024U)
#define NVM_STM32F413_FRAM_DISTANCE_PARAMS_BASE (0x00000000UL)
#define NVM_STM32F413_FRAM_FLASH_PARAMS_BASE (0x00020000UL)
#define NVM_STM32F413_FRAM_MAZE_MAP_BASE (0x00040000UL)
#define NVM_STM32F413_FRAM_TRACE_LOG_BASE (0x00060000UL)
#define NVM_STM32F413_FRAM_TRACE_LOG_SIZE_BYTES \
    (NVM_STM32F413_FRAM_TOTAL_BYTES - NVM_STM32F413_FRAM_TRACE_LOG_BASE)

#define NVM_STM32F413_FRAM_CMD_WREN (0x06U)
#define NVM_STM32F413_FRAM_CMD_READ (0x03U)
#define NVM_STM32F413_FRAM_CMD_WRITE (0x02U)
#define NVM_STM32F413_FRAM_SPI_TIMEOUT_MS (100U)

extern SPI_HandleTypeDef hspi2;

static const nvm_area_info_t g_nvm_area_table[NVM_AREA_COUNT] = {
    {NVM_AREA_IDENTITY, NVM_STM32F413_IDENTITY_BASE, NVM_STM32F413_SECTOR_SIZE_BYTES, 0x00010000UL},
    {NVM_AREA_DISTANCE_PARAMS, NVM_STM32F413_FRAM_DISTANCE_PARAMS_BASE, NVM_STM32F413_FRAM_AREA_SIZE_BYTES, 0x00010000UL},
    {NVM_AREA_FLASH_PARAMS, NVM_STM32F413_FRAM_FLASH_PARAMS_BASE, NVM_STM32F413_FRAM_AREA_SIZE_BYTES, 0x00010001UL},
    {NVM_AREA_MAZE_MAP, NVM_STM32F413_FRAM_MAZE_MAP_BASE, NVM_STM32F413_FRAM_AREA_SIZE_BYTES, 0x00010000UL},
    {NVM_AREA_TRACE_LOG, NVM_STM32F413_FRAM_TRACE_LOG_BASE, NVM_STM32F413_FRAM_TRACE_LOG_SIZE_BYTES, NVM_TRACE_LOG_SCHEMA_VERSION},
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
    if (out->size_bytes == 0U) {
        return NVM_STATUS_UNSUPPORTED;
    }
    if (nvm_get_backend(area) == NVM_BACKEND_NONE) {
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

    switch (nvm_get_backend(area)) {
        case NVM_BACKEND_INTERNAL_FLASH:
            memcpy(out, (const void*)(uintptr_t)(info.base_address + offset), len);
            return NVM_STATUS_OK;

        case NVM_BACKEND_EXTERNAL_FRAM:
#if defined(STM32F413xx)
            return nvm_stm32f413_fram_read_area(&info, offset, out, len);
#else
            return NVM_STATUS_UNSUPPORTED;
#endif

        default:
            return NVM_STATUS_UNSUPPORTED;
    }
}

#if defined(STM32F405xx)

static nvm_status_t nvm_stm32f405_sector_from_base(uint32_t base_address, uint32_t* out_sector) {
    if (out_sector == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }

    switch (base_address) {
        case NVM_STM32F405_IDENTITY_BASE:
            *out_sector = FLASH_SECTOR_8;
            return NVM_STATUS_OK;
        case NVM_STM32F405_DISTANCE_PARAMS_BASE:
            *out_sector = FLASH_SECTOR_9;
            return NVM_STATUS_OK;
        case NVM_STM32F405_FLASH_PARAMS_BASE:
            *out_sector = FLASH_SECTOR_10;
            return NVM_STATUS_OK;
        case NVM_STM32F405_MAZE_MAP_BASE:
            *out_sector = FLASH_SECTOR_11;
            return NVM_STATUS_OK;
        default:
            return NVM_STATUS_UNSUPPORTED;
    }
}

static nvm_status_t nvm_stm32f405_erase_area(const nvm_area_info_t* info) {
    HAL_StatusTypeDef hal_st;
    FLASH_EraseInitTypeDef erase;
    uint32_t error_sector = 0U;
    uint32_t sector = 0U;
    nvm_status_t st;

    if (info == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }

    st = nvm_stm32f405_sector_from_base(info->base_address, &sector);
    if (st != NVM_STATUS_OK) {
        return st;
    }

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = sector;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    hal_st = HAL_FLASH_Unlock();
    if (hal_st != HAL_OK) {
        return NVM_STATUS_HW_ERROR;
    }

    hal_st = HAL_FLASHEx_Erase(&erase, &error_sector);
    if (hal_st != HAL_OK) {
        (void)HAL_FLASH_Lock();
        return NVM_STATUS_HW_ERROR;
    }

    hal_st = HAL_FLASH_Lock();
    if (hal_st != HAL_OK) {
        return NVM_STATUS_HW_ERROR;
    }
    return NVM_STATUS_OK;
}

static nvm_status_t nvm_stm32f405_write_area(const nvm_area_info_t* info,
                                             uint32_t offset,
                                             const void* data,
                                             size_t len) {
    HAL_StatusTypeDef hal_st;
    uint32_t addr;
    uint32_t start;
    uint32_t end;
    uint32_t write_addr;
    const uint8_t* src;

    if (info == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }
    if ((data == NULL) && (len > 0U)) {
        return NVM_STATUS_INVALID_ARG;
    }
    if (len == 0U) {
        return NVM_STATUS_OK;
    }
    if ((size_t)offset > (size_t)info->size_bytes) {
        return NVM_STATUS_INVALID_ARG;
    }
    if (len > ((size_t)info->size_bytes - (size_t)offset)) {
        return NVM_STATUS_INVALID_ARG;
    }

    src = (const uint8_t*)data;
    start = info->base_address + offset;
    end = start + (uint32_t)len;

    hal_st = HAL_FLASH_Unlock();
    if (hal_st != HAL_OK) {
        return NVM_STATUS_HW_ERROR;
    }

    for (write_addr = (start & ~0x3UL); write_addr < ((end + 3UL) & ~0x3UL); write_addr += 4UL) {
        uint32_t old_word = *(const uint32_t*)(uintptr_t)write_addr;
        uint32_t word = old_word;
        uint8_t* old_word_bytes = (uint8_t*)&old_word;
        uint8_t* word_bytes = (uint8_t*)&word;

        for (addr = 0U; addr < 4U; addr++) {
            uint32_t byte_addr = write_addr + addr;
            if (byte_addr >= start && byte_addr < end) {
                size_t src_index = (size_t)(byte_addr - start);
                uint8_t new_byte = src[src_index];
                if (((uint8_t)(~old_word_bytes[addr]) & new_byte) != 0U) {
                    (void)HAL_FLASH_Lock();
                    return NVM_STATUS_INVALID_ARG;
                }
                word_bytes[addr] = src[src_index];
            }
        }

        hal_st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, write_addr, word);
        if (hal_st != HAL_OK) {
            (void)HAL_FLASH_Lock();
            return NVM_STATUS_HW_ERROR;
        }
    }

    hal_st = HAL_FLASH_Lock();
    if (hal_st != HAL_OK) {
        return NVM_STATUS_HW_ERROR;
    }

    return NVM_STATUS_OK;
}

#elif defined(STM32F413xx)

static HAL_StatusTypeDef nvm_stm32f413_fram_write_enable(void) {
    uint8_t cmd = NVM_STM32F413_FRAM_CMD_WREN;

    if (hspi2.Instance == NULL) {
        return HAL_ERROR;
    }

    nvm_wait_spi2_free();
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(&hspi2, &cmd, 1U, NVM_STM32F413_FRAM_SPI_TIMEOUT_MS) != HAL_OK) {
        HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    return HAL_OK;
}

static HAL_StatusTypeDef nvm_stm32f413_fram_read_raw(uint32_t address, void* out, size_t len) {
    uint8_t cmd[4];

    if ((out == NULL) && (len > 0U)) {
        return HAL_ERROR;
    }
    if ((size_t)address + len > (size_t)NVM_STM32F413_FRAM_TOTAL_BYTES) {
        return HAL_ERROR;
    }
    if (hspi2.Instance == NULL) {
        return HAL_ERROR;
    }
    if (len == 0U) {
        return HAL_OK;
    }

    nvm_wait_spi2_free();
    cmd[0] = NVM_STM32F413_FRAM_CMD_READ;
    cmd[1] = (uint8_t)(address >> 16);
    cmd[2] = (uint8_t)(address >> 8);
    cmd[3] = (uint8_t)address;

    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(&hspi2, cmd, sizeof(cmd), NVM_STM32F413_FRAM_SPI_TIMEOUT_MS) != HAL_OK) {
        HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
        return HAL_ERROR;
    }

    if (HAL_SPI_Receive(&hspi2, (uint8_t*)out, (uint16_t)len, NVM_STM32F413_FRAM_SPI_TIMEOUT_MS) != HAL_OK) {
        HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    return HAL_OK;
}

static HAL_StatusTypeDef nvm_stm32f413_fram_write_raw(uint32_t address, const void* data, size_t len) {
    uint8_t cmd[4];

    if ((data == NULL) && (len > 0U)) {
        return HAL_ERROR;
    }
    if ((size_t)address + len > (size_t)NVM_STM32F413_FRAM_TOTAL_BYTES) {
        return HAL_ERROR;
    }
    if (hspi2.Instance == NULL) {
        return HAL_ERROR;
    }
    if (len == 0U) {
        return HAL_OK;
    }

    if (nvm_stm32f413_fram_write_enable() != HAL_OK) {
        return HAL_ERROR;
    }

    cmd[0] = NVM_STM32F413_FRAM_CMD_WRITE;
    cmd[1] = (uint8_t)(address >> 16);
    cmd[2] = (uint8_t)(address >> 8);
    cmd[3] = (uint8_t)address;

    nvm_wait_spi2_free();
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(&hspi2, cmd, sizeof(cmd), NVM_STM32F413_FRAM_SPI_TIMEOUT_MS) != HAL_OK) {
        HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
        return HAL_ERROR;
    }

    if (HAL_SPI_Transmit(&hspi2, (uint8_t*)data, (uint16_t)len, NVM_STM32F413_FRAM_SPI_TIMEOUT_MS) != HAL_OK) {
        HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    return HAL_OK;
}

static nvm_status_t nvm_stm32f413_fram_read_area(const nvm_area_info_t* info,
                                                  uint32_t offset,
                                                  void* out,
                                                  size_t len) {
    if (info == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }
    if ((out == NULL) && (len > 0U)) {
        return NVM_STATUS_INVALID_ARG;
    }

    if (nvm_stm32f413_fram_read_raw(info->base_address + offset, out, len) != HAL_OK) {
        return NVM_STATUS_HW_ERROR;
    }
    return NVM_STATUS_OK;
}

static nvm_status_t nvm_stm32f413_fram_write_area(const nvm_area_info_t* info,
                                                   uint32_t offset,
                                                   const void* data,
                                                   size_t len) {
    if (info == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }
    if ((data == NULL) && (len > 0U)) {
        return NVM_STATUS_INVALID_ARG;
    }

    if (nvm_stm32f413_fram_write_raw(info->base_address + offset, data, len) != HAL_OK) {
        return NVM_STATUS_HW_ERROR;
    }
    return NVM_STATUS_OK;
}

static nvm_status_t nvm_stm32f413_fram_erase_area(const nvm_area_info_t* info) {
    (void)info;
    return NVM_STATUS_OK;
}

static nvm_status_t nvm_stm32f413_sector_from_base(uint32_t base_address, uint32_t* out_sector) {
    if (out_sector == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }

    switch (base_address) {
        case NVM_STM32F413_MAZE_MAP_BASE:
            *out_sector = FLASH_SECTOR_12;
            return NVM_STATUS_OK;
        case NVM_STM32F413_DISTANCE_PARAMS_BASE:
            *out_sector = FLASH_SECTOR_13;
            return NVM_STATUS_OK;
        case NVM_STM32F413_FLASH_PARAMS_BASE:
            *out_sector = FLASH_SECTOR_14;
            return NVM_STATUS_OK;
        case NVM_STM32F413_IDENTITY_BASE:
            *out_sector = FLASH_SECTOR_15;
            return NVM_STATUS_OK;
        default:
            return NVM_STATUS_UNSUPPORTED;
    }
}

static nvm_status_t nvm_stm32f413_erase_area(const nvm_area_info_t* info) {
    HAL_StatusTypeDef hal_st;
    FLASH_EraseInitTypeDef erase;
    uint32_t error_sector = 0U;
    uint32_t sector = 0U;
    nvm_status_t st;

    if (info == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }

    st = nvm_stm32f413_sector_from_base(info->base_address, &sector);
    if (st != NVM_STATUS_OK) {
        return st;
    }

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = sector;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    hal_st = HAL_FLASH_Unlock();
    if (hal_st != HAL_OK) {
        return NVM_STATUS_HW_ERROR;
    }

    hal_st = HAL_FLASHEx_Erase(&erase, &error_sector);
    if (hal_st != HAL_OK) {
        (void)HAL_FLASH_Lock();
        return NVM_STATUS_HW_ERROR;
    }

    hal_st = HAL_FLASH_Lock();
    if (hal_st != HAL_OK) {
        return NVM_STATUS_HW_ERROR;
    }
    return NVM_STATUS_OK;
}

static nvm_status_t nvm_stm32f413_write_area(const nvm_area_info_t* info,
                                             uint32_t offset,
                                             const void* data,
                                             size_t len) {
    HAL_StatusTypeDef hal_st;
    uint32_t addr;
    uint32_t start;
    uint32_t end;
    uint32_t write_addr;
    const uint8_t* src;

    if (info == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }
    if ((data == NULL) && (len > 0U)) {
        return NVM_STATUS_INVALID_ARG;
    }
    if (len == 0U) {
        return NVM_STATUS_OK;
    }
    if ((size_t)offset > (size_t)info->size_bytes) {
        return NVM_STATUS_INVALID_ARG;
    }
    if (len > ((size_t)info->size_bytes - (size_t)offset)) {
        return NVM_STATUS_INVALID_ARG;
    }

    src = (const uint8_t*)data;
    start = info->base_address + offset;
    end = start + (uint32_t)len;

    hal_st = HAL_FLASH_Unlock();
    if (hal_st != HAL_OK) {
        return NVM_STATUS_HW_ERROR;
    }

    for (write_addr = (start & ~0x3UL); write_addr < ((end + 3UL) & ~0x3UL); write_addr += 4UL) {
        uint32_t old_word = *(const uint32_t*)(uintptr_t)write_addr;
        uint32_t word = old_word;
        uint8_t* old_word_bytes = (uint8_t*)&old_word;
        uint8_t* word_bytes = (uint8_t*)&word;

        for (addr = 0U; addr < 4U; addr++) {
            uint32_t byte_addr = write_addr + addr;
            if (byte_addr >= start && byte_addr < end) {
                size_t src_index = (size_t)(byte_addr - start);
                uint8_t new_byte = src[src_index];
                if (((uint8_t)(~old_word_bytes[addr]) & new_byte) != 0U) {
                    (void)HAL_FLASH_Lock();
                    return NVM_STATUS_INVALID_ARG;
                }
                word_bytes[addr] = new_byte;
            }
        }

        hal_st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, write_addr, word);
        if (hal_st != HAL_OK) {
            (void)HAL_FLASH_Lock();
            return NVM_STATUS_HW_ERROR;
        }
    }

    hal_st = HAL_FLASH_Lock();
    if (hal_st != HAL_OK) {
        return NVM_STATUS_HW_ERROR;
    }

    return NVM_STATUS_OK;
}

#endif

nvm_status_t nvm_write(nvm_area_t area, uint32_t offset, const void* data, size_t len) {
    nvm_area_info_t info;
    nvm_status_t st;

    st = nvm_get_area_info(area, &info);
    if (st != NVM_STATUS_OK) {
        return st;
    }

#if !defined(STM32F405xx) && !defined(STM32F413xx)
    (void)offset;
    (void)data;
    (void)len;
#endif

    switch (nvm_get_backend(area)) {
        case NVM_BACKEND_INTERNAL_FLASH:
#if defined(STM32F405xx)
            return nvm_stm32f405_write_area(&info, offset, data, len);
#elif defined(STM32F413xx)
            return nvm_stm32f413_write_area(&info, offset, data, len);
#else
            return NVM_STATUS_UNSUPPORTED;
#endif

        case NVM_BACKEND_EXTERNAL_FRAM:
#if defined(STM32F413xx)
            return nvm_stm32f413_fram_write_area(&info, offset, data, len);
#else
            return NVM_STATUS_UNSUPPORTED;
#endif

        default:
            return NVM_STATUS_UNSUPPORTED;
    }

    return NVM_STATUS_UNSUPPORTED;
}

nvm_status_t nvm_erase(nvm_area_t area) {
    nvm_area_info_t info;
    nvm_status_t st;

    st = nvm_get_area_info(area, &info);
    if (st != NVM_STATUS_OK) {
        return st;
    }

    switch (nvm_get_backend(area)) {
        case NVM_BACKEND_INTERNAL_FLASH:
#if defined(STM32F405xx)
            return nvm_stm32f405_erase_area(&info);
#elif defined(STM32F413xx)
            return nvm_stm32f413_erase_area(&info);
#else
            return NVM_STATUS_UNSUPPORTED;
#endif

        case NVM_BACKEND_EXTERNAL_FRAM:
#if defined(STM32F413xx)
            return nvm_stm32f413_fram_erase_area(&info);
#else
            return NVM_STATUS_UNSUPPORTED;
#endif

        default:
            return NVM_STATUS_UNSUPPORTED;
    }

    return NVM_STATUS_UNSUPPORTED;
}
