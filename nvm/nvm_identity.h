#ifndef NIGHTFALL_NVM_IDENTITY_H_
#define NIGHTFALL_NVM_IDENTITY_H_

#include <stdbool.h>
#include <stdint.h>

#include "nvm.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NVM_IDENTITY_MAGIC (0x4E464944UL)
#define NVM_IDENTITY_SCHEMA_VERSION (0x00010000UL)

typedef enum {
    NVM_FAMILY_UNKNOWN = 0,
    NVM_FAMILY_MINI = 1,
    NVM_FAMILY_CLASSIC = 2,
} nvm_family_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t schema_version;
    uint32_t length;
    uint32_t crc;

    uint32_t family;
    uint32_t board_id;
    uint16_t hw_rev_major;
    uint16_t hw_rev_minor;
    uint32_t unit_serial;
    uint32_t default_param_profile;
    uint32_t capability_flags;
    uint32_t mcu_uid[3];

    uint32_t reserved[4];
} nvm_identity_block_t;

nvm_status_t nvm_identity_read(nvm_identity_block_t* out);
nvm_status_t nvm_identity_write(const nvm_identity_block_t* in);
nvm_status_t nvm_identity_validate(const nvm_identity_block_t* block);
bool nvm_identity_is_valid_for_boot(const nvm_identity_block_t* block);

#ifdef __cplusplus
}
#endif

#endif
