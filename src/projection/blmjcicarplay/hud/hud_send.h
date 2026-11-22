// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Thin CarPlay guidance adapter. Apple/FW74 decoding stays in nav.cpp; this
// module converts the decoded maneuver into one complete Mazda-domain snapshot
// and hands it to the process-long common guidance client.

#ifndef LIBPATCH_BLMJCICARPLAY_HUD_SEND_H
#define LIBPATCH_BLMJCICARPLAY_HUD_SEND_H

#include "common/guidance/model.h"

#include <stdint.h>

// Start the process-long CarPlay IPC client. This is called once from the
// jciCARPLAY preload constructor, never once per phone session.
bool hud_send_start(void);

// Bracket one armed CarPlay device session. The returned nonzero epoch is the
// token every producer/lifecycle call must carry; stale epochs are ignored.
uint64_t hud_session_begin(void);
void hud_session_claim(uint64_t epoch);
void hud_session_release(
    uint64_t epoch, mazda_cmu::guidance::ReleaseReason reason);
void hud_session_end(uint64_t epoch);

// Decoded Apple navigation producer interface. The functions stage a complete
// latest snapshot even before TBT is confirmed, but publish only while the
// current epoch has been authoritatively claimed.
void hud_on_next_turn(uint64_t    epoch,
                      const char *road_name,
                      uint32_t    turn_side,
                      uint32_t    turn_event,
                      int32_t     turn_angle,
                      int32_t     turn_number);

void hud_on_distance(uint64_t epoch,
                     int32_t  distance_meters,
                     int32_t  time_until_seconds,
                     int32_t  display_distance,
                     uint32_t display_distance_unit);

#endif // LIBPATCH_BLMJCICARPLAY_HUD_SEND_H
