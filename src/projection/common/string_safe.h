// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from oem-aa-mod; see NOTICE.md.

#ifndef LIBPATCH_COMMON_STRING_SAFE_H
#define LIBPATCH_COMMON_STRING_SAFE_H

#include <cstddef>
#include <cstring>

namespace libpatch {

// Copy only complete UTF-8 characters within both bounds. Stop at NUL or
// malformed input, and zero-pad the destination for snapshot comparisons.
inline void copy_utf8_truncated(char *dst, size_t cap,
                                const char *src, size_t src_len)
{
    if (!dst || cap == 0) return;
    const size_t limit = cap - 1;
    size_t used = 0;
    while (src && used < limit && used < src_len) {
        const unsigned char ch = static_cast<unsigned char>(src[used]);
        if (ch == 0) break;
        size_t width;
        if (ch < 0x80) width = 1;
        else if (ch >= 0xc2 && ch <= 0xdf) width = 2;
        else if (ch >= 0xe0 && ch <= 0xef) width = 3;
        else if (ch >= 0xf0 && ch <= 0xf4) width = 4;
        else break;
        if (width > limit - used || width > src_len - used) break;

        bool valid = true;
        for (size_t i = 1; i < width; ++i) {
            const unsigned char b = static_cast<unsigned char>(src[used + i]);
            if ((b & 0xc0) != 0x80) { valid = false; break; }
        }
        if (!valid) break;
        if (width >= 3) {
            const unsigned char second = static_cast<unsigned char>(src[used + 1]);
            // Exclude overlong encodings, UTF-16 surrogates and > U+10FFFF.
            if ((ch == 0xe0 && second < 0xa0) ||
                (ch == 0xed && second >= 0xa0) ||
                (ch == 0xf0 && second < 0x90) ||
                (ch == 0xf4 && second >= 0x90)) break;
        }
        used += width;
    }
    if (used != 0) std::memcpy(dst, src, used);
    std::memset(dst + used, 0, cap - used);
}

} // namespace libpatch

#endif
