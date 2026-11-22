// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef LIBPATCH_COMMON_AA_NAV_STATUS_H
#define LIBPATCH_COMMON_AA_NAV_STATUS_H

#include <stdint.h>

// Read NavigationStatus field 1 from a full AA frame (two-byte message ID).
// Validate the complete protobuf before publishing the last status value.
inline bool aa_nav_read_status(const uint8_t *raw, int size, int *status_out)
{
    if (status_out) *status_out = -1;
    if (!raw || size < 2 || !status_out) return false;
    const uint8_t *p = raw + 2;
    const uint8_t *end = raw + size;
    auto varint = [&p, end](uint64_t &value) {
        value = 0;
        for (int shift = 0; p < end && shift < 64; shift += 7) {
            const uint8_t b = *p++;
            if (shift == 63 && b > 1) return false;
            value |= uint64_t(b & 0x7f) << shift;
            if (!(b & 0x80)) return true;
        }
        return false;
    };
    int status = -1;
    bool found = false;
    while (p < end) {
        uint64_t tag, value;
        if (!varint(tag) || tag < 8 || (tag >> 3) > 0x1fffffff) return false;
        switch (tag & 7) {
        case 0:
            if (!varint(value)) return false;
            if ((tag >> 3) == 1) {
                status = static_cast<int>(static_cast<uint32_t>(value));
                found = true;
            }
            break;
        case 1:
            if (end - p < 8) return false;
            p += 8;
            break;
        case 2:
            if (!varint(value) || value > static_cast<uint64_t>(end - p)) return false;
            p += value;
            break;
        case 5:
            if (end - p < 4) return false;
            p += 4;
            break;
        default:
            return false;
        }
    }
    if (found) *status_out = status;
    return found;
}

#endif // LIBPATCH_COMMON_AA_NAV_STATUS_H
