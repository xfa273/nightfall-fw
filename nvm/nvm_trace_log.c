#include "nvm_trace_log.h"

#include <string.h>

#define NVM_TRACE_LOG_HEADER_PREFIX_BYTES (16U)

static uint32_t nvm_trace_log_checksum(const uint8_t* data, uint32_t len) {
    uint32_t sum = 0U;
    uint32_t i;

    for (i = 0U; i < len; ++i) {
        sum += data[i];
    }
    return sum;
}

static nvm_status_t nvm_trace_log_get_area(nvm_area_info_t* out) {
    nvm_status_t st;

    if (out == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }

    st = nvm_get_area_info(NVM_AREA_TRACE_LOG, out);
    if (st != NVM_STATUS_OK) {
        return st;
    }
    if (out->size_bytes < sizeof(nvm_trace_log_header_t)) {
        return NVM_STATUS_UNSUPPORTED;
    }
    return NVM_STATUS_OK;
}

static nvm_status_t nvm_trace_log_validate_header(const nvm_trace_log_header_t* header,
                                                  const nvm_area_info_t* area,
                                                  uint32_t* out_capacity) {
    const uint8_t* payload;
    uint32_t payload_len;
    uint32_t calc_crc;
    uint32_t expected_capacity;

    if ((header == NULL) || (area == NULL)) {
        return NVM_STATUS_INVALID_ARG;
    }

    if (header->magic != NVM_TRACE_LOG_MAGIC) {
        return NVM_STATUS_NOT_FOUND;
    }
    if (header->version != NVM_TRACE_LOG_SCHEMA_VERSION) {
        return NVM_STATUS_INTEGRITY_ERROR;
    }
    if (header->length != sizeof(nvm_trace_log_header_t)) {
        return NVM_STATUS_INTEGRITY_ERROR;
    }
    if (header->record_size != sizeof(nvm_trace_log_record_t)) {
        return NVM_STATUS_INTEGRITY_ERROR;
    }

    expected_capacity =
        (area->size_bytes - (uint32_t)sizeof(nvm_trace_log_header_t)) / (uint32_t)sizeof(nvm_trace_log_record_t);
    if (expected_capacity == 0U) {
        return NVM_STATUS_UNSUPPORTED;
    }
    if (header->record_capacity != expected_capacity) {
        return NVM_STATUS_INTEGRITY_ERROR;
    }
    if (header->write_index >= header->record_capacity) {
        return NVM_STATUS_INTEGRITY_ERROR;
    }

    payload = ((const uint8_t*)header) + NVM_TRACE_LOG_HEADER_PREFIX_BYTES;
    payload_len = header->length - NVM_TRACE_LOG_HEADER_PREFIX_BYTES;
    calc_crc = nvm_trace_log_checksum(payload, payload_len);
    if (calc_crc != header->crc) {
        return NVM_STATUS_INTEGRITY_ERROR;
    }

    if (out_capacity != NULL) {
        *out_capacity = expected_capacity;
    }
    return NVM_STATUS_OK;
}

static void nvm_trace_log_finalize_header(nvm_trace_log_header_t* header) {
    const uint8_t* payload;
    uint32_t payload_len;

    header->crc = 0U;
    payload = ((const uint8_t*)header) + NVM_TRACE_LOG_HEADER_PREFIX_BYTES;
    payload_len = header->length - NVM_TRACE_LOG_HEADER_PREFIX_BYTES;
    header->crc = nvm_trace_log_checksum(payload, payload_len);
}

nvm_status_t nvm_trace_log_format(void) {
    nvm_area_info_t area;
    nvm_trace_log_header_t header;
    nvm_status_t st;

    st = nvm_trace_log_get_area(&area);
    if (st != NVM_STATUS_OK) {
        return st;
    }

    memset(&header, 0, sizeof(header));
    header.magic = NVM_TRACE_LOG_MAGIC;
    header.version = NVM_TRACE_LOG_SCHEMA_VERSION;
    header.length = sizeof(header);
    header.record_size = sizeof(nvm_trace_log_record_t);
    header.record_capacity =
        (area.size_bytes - (uint32_t)sizeof(nvm_trace_log_header_t)) / (uint32_t)sizeof(nvm_trace_log_record_t);
    header.write_index = 0U;
    header.total_records = 0U;
    nvm_trace_log_finalize_header(&header);

    st = nvm_erase(NVM_AREA_TRACE_LOG);
    if (st != NVM_STATUS_OK) {
        return st;
    }

    return nvm_write(NVM_AREA_TRACE_LOG, 0U, &header, sizeof(header));
}

nvm_status_t nvm_trace_log_get_header(nvm_trace_log_header_t* out) {
    nvm_area_info_t area;
    nvm_status_t st;

    if (out == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }

    st = nvm_trace_log_get_area(&area);
    if (st != NVM_STATUS_OK) {
        return st;
    }

    st = nvm_read(NVM_AREA_TRACE_LOG, 0U, out, sizeof(*out));
    if (st != NVM_STATUS_OK) {
        return st;
    }

    return nvm_trace_log_validate_header(out, &area, NULL);
}

nvm_status_t nvm_trace_log_append(const nvm_trace_log_record_t* record) {
    nvm_trace_log_header_t header;
    nvm_status_t st;

    if (record == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }

    st = nvm_trace_log_get_header(&header);
    if (st != NVM_STATUS_OK) {
        return st;
    }

    return nvm_trace_log_append_cached(&header, record, 1U);
}

nvm_status_t nvm_trace_log_commit_header(const nvm_trace_log_header_t* header) {
    nvm_area_info_t area;
    nvm_trace_log_header_t next;
    nvm_status_t st;

    if (header == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }

    st = nvm_trace_log_get_area(&area);
    if (st != NVM_STATUS_OK) {
        return st;
    }
    st = nvm_trace_log_validate_header(header, &area, NULL);
    if (st != NVM_STATUS_OK) {
        return st;
    }

    next = *header;
    nvm_trace_log_finalize_header(&next);
    return nvm_write(NVM_AREA_TRACE_LOG, 0U, &next, sizeof(next));
}

nvm_status_t nvm_trace_log_append_cached(nvm_trace_log_header_t* header,
                                         const nvm_trace_log_record_t* record,
                                         uint8_t commit_header) {
    nvm_area_info_t area;
    nvm_trace_log_header_t next;
    uint32_t write_pos;
    uint32_t record_offset;
    nvm_status_t st;

    if ((header == NULL) || (record == NULL)) {
        return NVM_STATUS_INVALID_ARG;
    }

    st = nvm_trace_log_get_area(&area);
    if (st != NVM_STATUS_OK) {
        return st;
    }

    st = nvm_trace_log_validate_header(header, &area, NULL);
    if (st != NVM_STATUS_OK) {
        return st;
    }

    write_pos = header->write_index;
    record_offset = (uint32_t)sizeof(nvm_trace_log_header_t) + write_pos * (uint32_t)sizeof(nvm_trace_log_record_t);

    st = nvm_write(NVM_AREA_TRACE_LOG, record_offset, record, sizeof(*record));
    if (st != NVM_STATUS_OK) {
        return st;
    }

    next = *header;
    next.write_index = (write_pos + 1U) % next.record_capacity;
    next.total_records += 1U;
    nvm_trace_log_finalize_header(&next);

    if (commit_header != 0U) {
        st = nvm_write(NVM_AREA_TRACE_LOG, 0U, &next, sizeof(next));
        if (st != NVM_STATUS_OK) {
            return st;
        }
    }

    *header = next;
    return NVM_STATUS_OK;
}

nvm_status_t nvm_trace_log_read_latest(uint32_t newest_index_from_tail,
                                       nvm_trace_log_record_t* out) {
    nvm_area_info_t area;
    nvm_trace_log_header_t header;
    uint32_t available;
    uint32_t logical_index;
    uint32_t record_offset;
    nvm_status_t st;

    if (out == NULL) {
        return NVM_STATUS_INVALID_ARG;
    }

    st = nvm_trace_log_get_area(&area);
    if (st != NVM_STATUS_OK) {
        return st;
    }

    st = nvm_read(NVM_AREA_TRACE_LOG, 0U, &header, sizeof(header));
    if (st != NVM_STATUS_OK) {
        return st;
    }

    st = nvm_trace_log_validate_header(&header, &area, NULL);
    if (st != NVM_STATUS_OK) {
        return st;
    }

    available = header.total_records;
    if (available > header.record_capacity) {
        available = header.record_capacity;
    }
    if (newest_index_from_tail >= available) {
        return NVM_STATUS_NOT_FOUND;
    }

    logical_index = (header.write_index + header.record_capacity - 1U - newest_index_from_tail) % header.record_capacity;
    record_offset =
        (uint32_t)sizeof(nvm_trace_log_header_t) + logical_index * (uint32_t)sizeof(nvm_trace_log_record_t);

    return nvm_read(NVM_AREA_TRACE_LOG, record_offset, out, sizeof(*out));
}
