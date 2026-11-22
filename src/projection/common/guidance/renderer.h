// SPDX-License-Identifier: AGPL-3.0-or-later

#ifndef MAZDA_CMU_GUIDANCE_RENDERER_H
#define MAZDA_CMU_GUIDANCE_RENDERER_H

#include "model.h"

#include <string.h>

namespace mazda_cmu {
namespace guidance {

// Shared final-domain helpers only. Source-specific maneuver decoding and
// roundabout-angle normalization deliberately remain in the role adapters.
inline bool snapshot_is_valid(const Snapshot &snapshot)
{
    if (snapshot.glyph == 0 || snapshot.glyph > 60) return false;
    if (snapshot.distance_x10 < 0 || snapshot.distance_x10 > 0xffff)
        return false;
    if (snapshot.distance_unit > 5) return false;
    for (unsigned i = 0; i < GUIDANCE_LANE_COUNT; ++i) {
        if (snapshot.lanes[i] > 70) return false;
    }
    if (memchr(snapshot.street, '\0', sizeof(snapshot.street)) == nullptr)
        return false;
    for (unsigned i = 0; i < sizeof(snapshot.reserved); ++i) {
        if (snapshot.reserved[i] != 0) return false;
    }
    return true;
}

inline void sanitize_snapshot(Snapshot &snapshot)
{
    if (snapshot.glyph > 60) snapshot.glyph = 0;
    if (snapshot.distance_x10 < 0) snapshot.distance_x10 = 0;
    if (snapshot.distance_x10 > 0xffff) snapshot.distance_x10 = 0xffff;
    if (snapshot.distance_unit > 5) snapshot.distance_unit = 0;
    for (unsigned i = 0; i < GUIDANCE_LANE_COUNT; ++i) {
        if (snapshot.lanes[i] > 70) snapshot.lanes[i] = 0;
    }
    snapshot.street[sizeof(snapshot.street) - 1] = '\0';
    memset(snapshot.reserved, 0, sizeof(snapshot.reserved));
}

} // namespace guidance
} // namespace mazda_cmu

#endif // MAZDA_CMU_GUIDANCE_RENDERER_H
