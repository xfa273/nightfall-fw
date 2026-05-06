#include "nvm_params.h"

#include <string.h>

#include "nvm.h"

#if defined(STM32F405xx)

#include "distance_params.h"
#include "eeprom.h"
#include "flash_params.h"

#define NVM_MAZE_BLOB_MAGIC (0x4D5A4531UL)
#define NVM_MAZE_BLOB_VERSION (0x00010000UL)
#define NVM_MAZE_BLOB_HEADER_BYTES (16U)
#define NVM_MAZE_BLOB_HEADER_WORDS (NVM_MAZE_BLOB_HEADER_BYTES / 4U)
#define NVM_MAZE_BLOB_HEADER_HALFWWORDS (NVM_MAZE_BLOB_HEADER_BYTES / 2U)
#define NVM_MAZE_MAX_HALFWWORDS_IN_SECTOR (65536U)

static uint32_t nvm_maze_payload_checksum(const uint16_t* cells, uint32_t cell_count)
{
    uint32_t sum = 0U;
    const uint8_t* payload = (const uint8_t*)cells;
    uint32_t payload_bytes = cell_count * 2U;
    uint32_t i;

    for (i = 0U; i < payload_bytes; i++) {
        sum += payload[i];
    }
    return sum;
}

static bool nvm_maze_is_args_valid(const uint16_t* cells, uint32_t cell_count)
{
    if ((cells == NULL) && (cell_count > 0U)) {
        return false;
    }
    if (cell_count > (NVM_MAZE_MAX_HALFWWORDS_IN_SECTOR - NVM_MAZE_BLOB_HEADER_HALFWWORDS)) {
        return false;
    }
    return true;
}

bool nvm_params_distance_load_and_apply(void)
{
    return distance_params_load_and_apply();
}

HAL_StatusTypeDef nvm_params_distance_save(const float x_fl[3], const float y_fl[3],
                                           const float x_fr[3], const float y_fr[3],
                                           const float x_fsum[3], const float y_fsum[3])
{
    return distance_params_save(x_fl, y_fl, x_fr, y_fr, x_fsum, y_fsum);
}

bool nvm_params_sensor_load(nvm_sensor_params_t* out)
{
    return flash_params_load(out);
}

HAL_StatusTypeDef nvm_params_sensor_save(const nvm_sensor_params_t* in)
{
    return flash_params_save(in);
}

void nvm_params_sensor_defaults(nvm_sensor_params_t* out)
{
    flash_params_defaults(out);
}

HAL_StatusTypeDef nvm_maze_enable_write(void)
{
    return eeprom_enable_write();
}

HAL_StatusTypeDef nvm_maze_disable_write(void)
{
    return eeprom_disable_write();
}

HAL_StatusTypeDef nvm_maze_write_halfword(uint32_t address, uint16_t data)
{
    return eeprom_write_halfword(address, data);
}

HAL_StatusTypeDef nvm_maze_write_word(uint32_t address, uint32_t data)
{
    return eeprom_write_word(address, data);
}

uint16_t nvm_maze_read_halfword(uint32_t address)
{
    return eeprom_read_halfword(address);
}

uint32_t nvm_maze_read_word(uint32_t address)
{
    return eeprom_read_word(address);
}

HAL_StatusTypeDef nvm_maze_save_map(const uint16_t* cells, uint32_t cell_count)
{
    HAL_StatusTypeDef st;
    uint32_t i;
    uint32_t header[NVM_MAZE_BLOB_HEADER_WORDS];

    if (!nvm_maze_is_args_valid(cells, cell_count)) {
        return HAL_ERROR;
    }

    header[0] = NVM_MAZE_BLOB_MAGIC;
    header[1] = NVM_MAZE_BLOB_VERSION;
    header[2] = NVM_MAZE_BLOB_HEADER_BYTES + cell_count * 2U;
    header[3] = nvm_maze_payload_checksum(cells, cell_count);

    st = eeprom_enable_write();
    if (st != HAL_OK) {
        return st;
    }

    for (i = 0U; i < NVM_MAZE_BLOB_HEADER_WORDS; i++) {
        st = eeprom_write_word(i, header[i]);
        if (st != HAL_OK) {
            (void)eeprom_disable_write();
            return st;
        }
    }

    for (i = 0U; i < cell_count; i++) {
        st = eeprom_write_halfword(NVM_MAZE_BLOB_HEADER_HALFWWORDS + i, cells[i]);
        if (st != HAL_OK) {
            (void)eeprom_disable_write();
            return st;
        }
    }

    return eeprom_disable_write();
}

bool nvm_maze_load_map(uint16_t* cells, uint32_t cell_count)
{
    uint32_t i;
    uint32_t magic;
    uint32_t version;
    uint32_t length;
    uint32_t checksum;
    uint32_t expected_length;
    bool all_erased = true;

    if (!nvm_maze_is_args_valid(cells, cell_count)) {
        return false;
    }

    magic = eeprom_read_word(0U);
    if (magic == NVM_MAZE_BLOB_MAGIC) {
        version = eeprom_read_word(1U);
        length = eeprom_read_word(2U);
        checksum = eeprom_read_word(3U);
        expected_length = NVM_MAZE_BLOB_HEADER_BYTES + cell_count * 2U;

        if (version != NVM_MAZE_BLOB_VERSION || length != expected_length) {
            return false;
        }

        for (i = 0U; i < cell_count; i++) {
            cells[i] = eeprom_read_halfword(NVM_MAZE_BLOB_HEADER_HALFWWORDS + i);
        }
        return nvm_maze_payload_checksum(cells, cell_count) == checksum;
    }

    for (i = 0U; i < cell_count; i++) {
        uint16_t v = eeprom_read_halfword(i);
        cells[i] = v;
        if (v != 0xFFFFU) {
            all_erased = false;
        }
    }

    if (all_erased) {
        for (i = 0U; i < cell_count; i++) {
            cells[i] = 0U;
        }
        return false;
    }

    return true;
}

#elif defined(STM32F413xx)

#include "sensor_distance.h"

#define NVM_MAZE_BLOB_MAGIC (0x4D5A4531UL)
#define NVM_MAZE_BLOB_VERSION (0x00010000UL)
#define NVM_MAZE_BLOB_HEADER_BYTES (16U)
#define NVM_MAZE_BLOB_HEADER_WORDS (NVM_MAZE_BLOB_HEADER_BYTES / 4U)
#define NVM_MAZE_BLOB_HEADER_HALFWWORDS (NVM_MAZE_BLOB_HEADER_BYTES / 2U)
#define NVM_MAZE_MAX_HALFWWORDS_IN_SECTOR (65536U)

#define NVM_DISTANCE_BLOB_MAGIC (0x44495354UL)
#define NVM_DISTANCE_BLOB_VERSION (0x00010000UL)
#define NVM_SENSOR_BLOB_MAGIC (0x50415231UL)
#define NVM_SENSOR_BLOB_VERSION (0x00010001UL)

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
} nvm_distance_blob_t;

static uint32_t nvm_payload_checksum(const uint8_t* payload, uint32_t payload_bytes)
{
    uint32_t sum = 0U;
    uint32_t i;

    for (i = 0U; i < payload_bytes; i++) {
        sum += payload[i];
    }
    return sum;
}

static uint32_t nvm_maze_payload_checksum(const uint16_t* cells, uint32_t cell_count)
{
    return nvm_payload_checksum((const uint8_t*)cells, cell_count * 2U);
}

static bool nvm_maze_is_args_valid(const uint16_t* cells, uint32_t cell_count)
{
    if ((cells == NULL) && (cell_count > 0U)) {
        return false;
    }
    if (cell_count > (NVM_MAZE_MAX_HALFWWORDS_IN_SECTOR - NVM_MAZE_BLOB_HEADER_HALFWWORDS)) {
        return false;
    }
    return true;
}

static HAL_StatusTypeDef nvm_status_to_hal(nvm_status_t st)
{
    return (st == NVM_STATUS_OK) ? HAL_OK : HAL_ERROR;
}

bool nvm_params_distance_load_and_apply(void)
{
    nvm_distance_blob_t blob;
    const uint8_t* payload;
    uint32_t crc;
    nvm_status_t st;
    float x_fl[3];
    float y_fl[3];
    float x_fr[3];
    float y_fr[3];
    float x_fsum[3];
    float y_fsum[3];

    st = nvm_read(NVM_AREA_DISTANCE_PARAMS, 0U, &blob, sizeof(blob));
    if (st != NVM_STATUS_OK) {
        return false;
    }

    if (blob.magic != NVM_DISTANCE_BLOB_MAGIC) {
        return false;
    }
    if (blob.version != NVM_DISTANCE_BLOB_VERSION) {
        return false;
    }
    if (blob.length != sizeof(blob)) {
        return false;
    }

    payload = ((const uint8_t*)&blob) + 16U;
    crc = nvm_payload_checksum(payload, (uint32_t)blob.length - 16U);
    if (crc != blob.crc) {
        return false;
    }

    memcpy(x_fl, blob.x_fl, sizeof(x_fl));
    memcpy(y_fl, blob.y_fl, sizeof(y_fl));
    memcpy(x_fr, blob.x_fr, sizeof(x_fr));
    memcpy(y_fr, blob.y_fr, sizeof(y_fr));
    memcpy(x_fsum, blob.x_fsum, sizeof(x_fsum));
    memcpy(y_fsum, blob.y_fsum, sizeof(y_fsum));

    sensor_distance_set_warp_fl_3pt(x_fl, y_fl);
    sensor_distance_set_warp_fr_3pt(x_fr, y_fr);
    sensor_distance_set_warp_front_sum_3pt(x_fsum, y_fsum);

    return true;
}

HAL_StatusTypeDef nvm_params_distance_save(const float x_fl[3], const float y_fl[3],
                                           const float x_fr[3], const float y_fr[3],
                                           const float x_fsum[3], const float y_fsum[3])
{
    nvm_distance_blob_t blob;
    const uint8_t* payload;
    nvm_status_t st;

    memset(&blob, 0, sizeof(blob));
    memcpy(blob.x_fl, x_fl, sizeof(float) * 3U);
    memcpy(blob.y_fl, y_fl, sizeof(float) * 3U);
    memcpy(blob.x_fr, x_fr, sizeof(float) * 3U);
    memcpy(blob.y_fr, y_fr, sizeof(float) * 3U);
    memcpy(blob.x_fsum, x_fsum, sizeof(float) * 3U);
    memcpy(blob.y_fsum, y_fsum, sizeof(float) * 3U);

    blob.magic = NVM_DISTANCE_BLOB_MAGIC;
    blob.version = NVM_DISTANCE_BLOB_VERSION;
    blob.length = sizeof(blob);
    blob.crc = 0U;
    payload = ((const uint8_t*)&blob) + 16U;
    blob.crc = nvm_payload_checksum(payload, (uint32_t)blob.length - 16U);

    st = nvm_erase(NVM_AREA_DISTANCE_PARAMS);
    if (st != NVM_STATUS_OK) {
        return HAL_ERROR;
    }
    st = nvm_write(NVM_AREA_DISTANCE_PARAMS, 0U, &blob, sizeof(blob));
    return nvm_status_to_hal(st);
}

bool nvm_params_sensor_load(nvm_sensor_params_t* out)
{
    nvm_sensor_params_t blob;
    const uint8_t* payload;
    uint32_t crc;
    nvm_status_t st;

    if (out == NULL) {
        return false;
    }

    st = nvm_read(NVM_AREA_FLASH_PARAMS, 0U, &blob, sizeof(blob));
    if (st != NVM_STATUS_OK) {
        return false;
    }

    if (blob.magic != NVM_SENSOR_BLOB_MAGIC) {
        return false;
    }
    if (blob.version != NVM_SENSOR_BLOB_VERSION) {
        return false;
    }
    if (blob.length != sizeof(blob)) {
        return false;
    }

    payload = ((const uint8_t*)&blob) + 16U;
    crc = nvm_payload_checksum(payload, (uint32_t)blob.length - 16U);
    if (crc != blob.crc) {
        return false;
    }

    *out = blob;
    return true;
}

HAL_StatusTypeDef nvm_params_sensor_save(const nvm_sensor_params_t* in)
{
    nvm_sensor_params_t blob;
    const uint8_t* payload;
    nvm_status_t st;

    if (in == NULL) {
        return HAL_ERROR;
    }

    memcpy(&blob, in, sizeof(blob));
    blob.magic = NVM_SENSOR_BLOB_MAGIC;
    blob.version = NVM_SENSOR_BLOB_VERSION;
    blob.length = sizeof(blob);
    blob.crc = 0U;
    payload = ((const uint8_t*)&blob) + 16U;
    blob.crc = nvm_payload_checksum(payload, (uint32_t)blob.length - 16U);

    st = nvm_erase(NVM_AREA_FLASH_PARAMS);
    if (st != NVM_STATUS_OK) {
        return HAL_ERROR;
    }

    st = nvm_write(NVM_AREA_FLASH_PARAMS, 0U, &blob, sizeof(blob));
    return nvm_status_to_hal(st);
}

void nvm_params_sensor_defaults(nvm_sensor_params_t* out)
{
    if (out == NULL) {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->magic = NVM_SENSOR_BLOB_MAGIC;
    out->version = NVM_SENSOR_BLOB_VERSION;
    out->length = sizeof(*out);
    out->crc = 0U;
}

HAL_StatusTypeDef nvm_maze_enable_write(void)
{
    return nvm_status_to_hal(nvm_erase(NVM_AREA_MAZE_MAP));
}

HAL_StatusTypeDef nvm_maze_disable_write(void)
{
    return HAL_OK;
}

HAL_StatusTypeDef nvm_maze_write_halfword(uint32_t address, uint16_t data)
{
    nvm_status_t st = nvm_write(NVM_AREA_MAZE_MAP, address * 2U, &data, sizeof(data));
    return nvm_status_to_hal(st);
}

HAL_StatusTypeDef nvm_maze_write_word(uint32_t address, uint32_t data)
{
    nvm_status_t st = nvm_write(NVM_AREA_MAZE_MAP, address * 4U, &data, sizeof(data));
    return nvm_status_to_hal(st);
}

uint16_t nvm_maze_read_halfword(uint32_t address)
{
    uint16_t value = 0xFFFFU;
    if (nvm_read(NVM_AREA_MAZE_MAP, address * 2U, &value, sizeof(value)) != NVM_STATUS_OK) {
        return 0xFFFFU;
    }
    return value;
}

uint32_t nvm_maze_read_word(uint32_t address)
{
    uint32_t value = 0xFFFFFFFFUL;
    if (nvm_read(NVM_AREA_MAZE_MAP, address * 4U, &value, sizeof(value)) != NVM_STATUS_OK) {
        return 0xFFFFFFFFUL;
    }
    return value;
}

HAL_StatusTypeDef nvm_maze_save_map(const uint16_t* cells, uint32_t cell_count)
{
    HAL_StatusTypeDef st;
    uint32_t i;
    uint32_t header[NVM_MAZE_BLOB_HEADER_WORDS];

    if (!nvm_maze_is_args_valid(cells, cell_count)) {
        return HAL_ERROR;
    }

    header[0] = NVM_MAZE_BLOB_MAGIC;
    header[1] = NVM_MAZE_BLOB_VERSION;
    header[2] = NVM_MAZE_BLOB_HEADER_BYTES + cell_count * 2U;
    header[3] = nvm_maze_payload_checksum(cells, cell_count);

    st = nvm_maze_enable_write();
    if (st != HAL_OK) {
        return st;
    }

    for (i = 0U; i < NVM_MAZE_BLOB_HEADER_WORDS; i++) {
        st = nvm_maze_write_word(i, header[i]);
        if (st != HAL_OK) {
            (void)nvm_maze_disable_write();
            return st;
        }
    }

    for (i = 0U; i < cell_count; i++) {
        st = nvm_maze_write_halfword(NVM_MAZE_BLOB_HEADER_HALFWWORDS + i, cells[i]);
        if (st != HAL_OK) {
            (void)nvm_maze_disable_write();
            return st;
        }
    }

    return nvm_maze_disable_write();
}

bool nvm_maze_load_map(uint16_t* cells, uint32_t cell_count)
{
    uint32_t i;
    uint32_t magic;
    uint32_t version;
    uint32_t length;
    uint32_t checksum;
    uint32_t expected_length;
    bool all_erased = true;

    if (!nvm_maze_is_args_valid(cells, cell_count)) {
        return false;
    }

    magic = nvm_maze_read_word(0U);
    if (magic == NVM_MAZE_BLOB_MAGIC) {
        version = nvm_maze_read_word(1U);
        length = nvm_maze_read_word(2U);
        checksum = nvm_maze_read_word(3U);
        expected_length = NVM_MAZE_BLOB_HEADER_BYTES + cell_count * 2U;

        if (version != NVM_MAZE_BLOB_VERSION || length != expected_length) {
            return false;
        }

        for (i = 0U; i < cell_count; i++) {
            cells[i] = nvm_maze_read_halfword(NVM_MAZE_BLOB_HEADER_HALFWWORDS + i);
        }
        return nvm_maze_payload_checksum(cells, cell_count) == checksum;
    }

    for (i = 0U; i < cell_count; i++) {
        uint16_t v = nvm_maze_read_halfword(i);
        cells[i] = v;
        if (v != 0xFFFFU) {
            all_erased = false;
        }
    }

    if (all_erased) {
        for (i = 0U; i < cell_count; i++) {
            cells[i] = 0U;
        }
        return false;
    }

    return true;
}

#else

bool nvm_params_distance_load_and_apply(void)
{
    return false;
}

HAL_StatusTypeDef nvm_params_distance_save(const float x_fl[3], const float y_fl[3],
                                           const float x_fr[3], const float y_fr[3],
                                           const float x_fsum[3], const float y_fsum[3])
{
    (void)x_fl;
    (void)y_fl;
    (void)x_fr;
    (void)y_fr;
    (void)x_fsum;
    (void)y_fsum;
    return HAL_ERROR;
}

#if defined(STM32F405xx) || defined(STM32F413xx)
bool nvm_params_sensor_load(nvm_sensor_params_t* out)
{
    (void)out;
    return false;
}

HAL_StatusTypeDef nvm_params_sensor_save(const nvm_sensor_params_t* in)
{
    (void)in;
    return HAL_ERROR;
}

void nvm_params_sensor_defaults(nvm_sensor_params_t* out)
{
    (void)out;
}
#endif

HAL_StatusTypeDef nvm_maze_enable_write(void)
{
    return HAL_ERROR;
}

HAL_StatusTypeDef nvm_maze_disable_write(void)
{
    return HAL_ERROR;
}

HAL_StatusTypeDef nvm_maze_write_halfword(uint32_t address, uint16_t data)
{
    (void)address;
    (void)data;
    return HAL_ERROR;
}

HAL_StatusTypeDef nvm_maze_write_word(uint32_t address, uint32_t data)
{
    (void)address;
    (void)data;
    return HAL_ERROR;
}

uint16_t nvm_maze_read_halfword(uint32_t address)
{
    (void)address;
    return 0U;
}

uint32_t nvm_maze_read_word(uint32_t address)
{
    (void)address;
    return 0UL;
}

HAL_StatusTypeDef nvm_maze_save_map(const uint16_t* cells, uint32_t cell_count)
{
    (void)cells;
    (void)cell_count;
    return HAL_ERROR;
}

bool nvm_maze_load_map(uint16_t* cells, uint32_t cell_count)
{
    uint32_t i;

    if ((cells == NULL) && (cell_count > 0U)) {
        return false;
    }

    for (i = 0U; i < cell_count; i++) {
        cells[i] = 0U;
    }
    return false;
}

#endif
