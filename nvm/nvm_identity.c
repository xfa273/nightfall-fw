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

    if (block == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }
    if (block->magic != NVM_IDENTITY_MAGIC) {
        return NVM_STATUS_NOT_FOUND;
    }
    if (block->schema_version != NVM_IDENTITY_SCHEMA_VERSION) {
        return NVM_STATUS_INTEGRITY_ERROR;
    }
    if (block->length != sizeof(nvm_identity_block_t)) {
        return NVM_STATUS_INTEGRITY_ERROR;
    }

    payload = ((const uint8_t*)block) + 16U;
    payload_len = (uint32_t)block->length - 16U;
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

bool nvm_identity_is_valid_for_boot(const nvm_identity_block_t* block) {
    return nvm_identity_validate(block) == NVM_STATUS_OK;
}
