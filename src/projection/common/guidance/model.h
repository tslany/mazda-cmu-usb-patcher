// SPDX-License-Identifier: AGPL-3.0-or-later

#ifndef MAZDA_CMU_GUIDANCE_MODEL_H
#define MAZDA_CMU_GUIDANCE_MODEL_H

#include <stdint.h>

namespace mazda_cmu {
namespace guidance {

enum Source : uint8_t {
    SOURCE_NONE = 0,
    SOURCE_ANDROID_AUTO = 1,
    SOURCE_CARPLAY = 2,
};

enum ReleaseReason : uint16_t {
    RELEASE_ROUTE_STOPPED = 1,
    RELEASE_UNSUPPORTED_MANEUVER = 2,
    RELEASE_SESSION_ENDED = 3,
    RELEASE_SOURCE_DISCONNECTED = 4,
    RELEASE_LEASE_EXPIRED = 5,
};

enum {
    GUIDANCE_LANE_COUNT = 8,
    GUIDANCE_STREET_BYTES = 256,
};

// A complete, source-neutral Mazda HUD frame. Protocol adapters own decoding
// and source-specific angle conventions; the common runtime only validates,
// arbitrates, and publishes this Mazda-domain snapshot.
struct Snapshot {
    uint32_t glyph;          // Mazda HUD glyph 1..60. Zero is not publishable.
    int32_t distance_x10;    // Displayed distance * 10; clamped by the sink.
    uint8_t distance_unit;   // Mazda HUD unit: 0=none, 1=m, 2=mi, 3=km, 4=yd, 5=ft.
    uint8_t lanes[GUIDANCE_LANE_COUNT]; // OEM lane codes 0..70, left to right.
    char street[GUIDANCE_STREET_BYTES]; // NUL-terminated UTF-8.
    uint8_t reserved[7];     // Always zero; makes SnapshotMessage gap-free.
};

static_assert(sizeof(Snapshot) == 280,
              "guidance Snapshot wire layout must remain gap-free");

} // namespace guidance
} // namespace mazda_cmu

#endif // MAZDA_CMU_GUIDANCE_MODEL_H
