/*
 * flash_params.c
 *
 *  Parameters storage in Flash (separate from maze EEPROM emulation)
 */
#include "flash_params.h"
#include <string.h>

static uint32_t calc_checksum(const uint8_t* data, uint32_t len)
{
    // Simple 32-bit additive checksum
    uint32_t sum = 0;
    for (uint32_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return sum;
}

void flash_params_defaults(flash_params_t* out)
{
    memset(out, 0, sizeof(*out));
    out->magic = FLASH_PARAMS_MAGIC;
    out->version = FLASH_PARAMS_VERSION;
    out->length = sizeof(flash_params_t);
    out->crc = 0; // will be computed when saving
}

static void build_header_and_crc(flash_params_t* p)
{
    p->magic = FLASH_PARAMS_MAGIC;
    p->version = FLASH_PARAMS_VERSION;
    p->length = sizeof(flash_params_t);
    // compute checksum over payload after crc field
    const uint8_t* payload = (const uint8_t*)p + 16; // skip magic, version, length, crc
    const uint32_t payload_len = p->length - 16;
    p->crc = calc_checksum(payload, payload_len);
}

bool flash_params_load(flash_params_t* out)
{
    if (!out) return false;

    const flash_params_t* src = (const flash_params_t*)FLASH_PARAMS_START_ADDRESS;

    if (src->magic != FLASH_PARAMS_MAGIC) {
        return false;
    }
    if (src->length != sizeof(flash_params_t)) {
        // version or layout mismatch
        return false;
    }

    // verify checksum
    const uint8_t* payload = (const uint8_t*)src + 16;
    const uint32_t payload_len = src->length - 16;
    const uint32_t crc_calc = calc_checksum(payload, payload_len);
    if (crc_calc != src->crc) {
        return false;
    }

    // ok, copy out
    memcpy(out, src, sizeof(flash_params_t));
    return true;
}

HAL_StatusTypeDef flash_params_save(const flash_params_t* in)
{
    if (!in) return HAL_ERROR;

    HAL_StatusTypeDef status;

    // Make a local aligned copy to compute header and program
    flash_params_t buf;
    memcpy(&buf, in, sizeof(buf));
    build_header_and_crc(&buf);

    // Erase target sector
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t ErrorSector = 0;
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector      = FLASH_PARAMS_SECTOR;
    EraseInitStruct.NbSectors   = 1;
    EraseInitStruct.VoltageRange= FLASH_VOLTAGE_RANGE_3;

    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_FLASHEx_Erase(&EraseInitStruct, &ErrorSector);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }

    // Program word-by-word (32-bit)
    const uint32_t* p = (const uint32_t*)&buf;
    const uint32_t words = (sizeof(flash_params_t) + 3) / 4;
    uint32_t address = FLASH_PARAMS_START_ADDRESS;
    for (uint32_t i = 0; i < words; ++i) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, p[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        address += 4;
    }

    status = HAL_FLASH_Lock();
    return status;
}
