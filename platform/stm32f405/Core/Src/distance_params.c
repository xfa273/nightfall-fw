/*
 * distance_params.c
 */
#include "distance_params.h"
#include "sensor_distance.h"
#include <string.h>

// On-flash structure (word-aligned)
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t length;
    uint32_t crc;
    float x_fl[3];
    float y_fl[3];
    float x_fr[3];
    float y_fr[3];
    float x_fsum[3];
    float y_fsum[3];
    uint32_t reserved[8];
} dist_params_blob_t;

static uint32_t calc_checksum(const uint8_t* data, uint32_t len)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < len; ++i) sum += data[i];
    return sum;
}

static void build_header(dist_params_blob_t* p)
{
    p->magic = DIST_PARAMS_MAGIC;
    p->version = DIST_PARAMS_VERSION;
    p->length = sizeof(dist_params_blob_t);
    const uint8_t* payload = (const uint8_t*)p + 16;
    const uint32_t payload_len = p->length - 16;
    p->crc = calc_checksum(payload, payload_len);
}

bool distance_params_load_and_apply(void)
{
    const dist_params_blob_t* src = (const dist_params_blob_t*)DIST_PARAMS_START_ADDRESS;
    if (src->magic != DIST_PARAMS_MAGIC) return false;
    if (src->length != sizeof(dist_params_blob_t)) return false;

    const uint8_t* payload = (const uint8_t*)src + 16;
    const uint32_t payload_len = src->length - 16;
    if (calc_checksum(payload, payload_len) != src->crc) return false;

    // Apply warps
    sensor_distance_set_warp_fl_3pt(src->x_fl, src->y_fl);
    sensor_distance_set_warp_fr_3pt(src->x_fr, src->y_fr);
    sensor_distance_set_warp_front_sum_3pt(src->x_fsum, src->y_fsum);
    return true;
}

HAL_StatusTypeDef distance_params_save(const float x_fl[3], const float y_fl[3],
                                       const float x_fr[3], const float y_fr[3],
                                       const float x_fsum[3], const float y_fsum[3])
{
    dist_params_blob_t blob;
    memset(&blob, 0, sizeof(blob));
    memcpy(blob.x_fl,   x_fl,   sizeof(float)*3);
    memcpy(blob.y_fl,   y_fl,   sizeof(float)*3);
    memcpy(blob.x_fr,   x_fr,   sizeof(float)*3);
    memcpy(blob.y_fr,   y_fr,   sizeof(float)*3);
    memcpy(blob.x_fsum, x_fsum, sizeof(float)*3);
    memcpy(blob.y_fsum, y_fsum, sizeof(float)*3);

    build_header(&blob);

    // Erase Sector 9 and program
    HAL_StatusTypeDef st;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t ErrorSector = 0;
    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector       = DIST_PARAMS_SECTOR;
    EraseInitStruct.NbSectors    = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    st = HAL_FLASH_Unlock();
    if (st != HAL_OK) return st;

    st = HAL_FLASHEx_Erase(&EraseInitStruct, &ErrorSector);
    if (st != HAL_OK) {
        HAL_FLASH_Lock();
        return st;
    }

    // Word programming
    const uint32_t* p = (const uint32_t*)&blob;
    const uint32_t words = (sizeof(dist_params_blob_t) + 3) / 4;
    uint32_t addr = DIST_PARAMS_START_ADDRESS;
    for (uint32_t i = 0; i < words; ++i) {
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, p[i]);
        if (st != HAL_OK) {
            HAL_FLASH_Lock();
            return st;
        }
        addr += 4;
    }

    return HAL_FLASH_Lock();
}
