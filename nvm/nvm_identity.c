#include "nvm_identity.h"

#include <string.h>

static uint32_t nvm_identity_checksum(const uint8_t* data, uint32_t len) {
    uint32_t sum = 0;
    for (uint32_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return sum;
}

nvm_status_t nvm_identity_validate(const nvm_identity_block_t* block) {
    const uint8_t* payload;
    uint32_t payload_len;
    uint32_t calc;
    const uint32_t header_size = 16U;

    if (block == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }
    if (block->magic != NVM_IDENTITY_MAGIC) {
        return NVM_STATUS_NOT_FOUND;
    }
    if (block->schema_version != NVM_IDENTITY_SCHEMA_VERSION) {
        return NVM_STATUS_INTEGRITY_ERROR;
    }
    if (block->length < header_size) {
        return NVM_STATUS_INTEGRITY_ERROR;
    }
    if (block->length != sizeof(nvm_identity_block_t)) {
        return NVM_STATUS_INTEGRITY_ERROR;
    }

    payload = ((const uint8_t*)block) + header_size;
    payload_len = block->length - header_size;
    calc = nvm_identity_checksum(payload, payload_len);
    if (calc != block->crc) {
        return NVM_STATUS_INTEGRITY_ERROR;
    }

    return NVM_STATUS_OK;
}

nvm_status_t nvm_identity_read(nvm_identity_block_t* out) {
    nvm_status_t st;

    if (out == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }

    st = nvm_read(NVM_AREA_IDENTITY, 0U, out, sizeof(*out));
    if (st != NVM_STATUS_OK) {
        return st;
    }

    return nvm_identity_validate(out);
}

nvm_status_t nvm_identity_write(const nvm_identity_block_t* in) {
    nvm_identity_block_t tmp;
    uint8_t* payload;
    uint32_t payload_len;
    const uint32_t header_size = 16U;
    nvm_status_t st;

    if (in == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }

    memcpy(&tmp, in, sizeof(tmp));
    tmp.magic = NVM_IDENTITY_MAGIC;
    tmp.schema_version = NVM_IDENTITY_SCHEMA_VERSION;
    tmp.length = sizeof(nvm_identity_block_t);
    tmp.crc = 0U;

    payload = ((uint8_t*)&tmp) + header_size;
    payload_len = tmp.length - header_size;
    tmp.crc = nvm_identity_checksum(payload, payload_len);

    st = nvm_erase(NVM_AREA_IDENTITY);
    if (st != NVM_STATUS_OK) {
        return st;
    }

    return nvm_write(NVM_AREA_IDENTITY, 0U, &tmp, sizeof(tmp));
}

bool nvm_identity_is_valid_for_boot(const nvm_identity_block_t* block) {
    return nvm_identity_validate(block) == NVM_STATUS_OK;
}
