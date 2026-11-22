// SPDX-License-Identifier: AGPL-3.0-or-later
//
// GAL 1.6 nav receiver — public API of nav16_rx.cpp.
//
// The aap_service shim (a SEPARATE process) swallows the Android Auto 1.6
// navigation frames and relays them RAW over an abstract AF_UNIX datagram
// socket. nav16_rx.cpp binds that socket on a dedicated thread, validates each
// datagram, and hands the raw frame to hud_nav16_feed() (declared in
// hud_nav16.h) for decode + sink dispatch. The functions below are that
// receiver's lifecycle + status surface, called from hud.cpp.

#ifndef LIBPATCH_BLMJCIAAPA_HUD_NAV16_RX_H
#define LIBPATCH_BLMJCIAAPA_HUD_NAV16_RX_H

// Re-export the decoded-frame data contract + callback types (and the AA->Mazda
// mapping helpers/formatters) so a HUD consumer can depend on this receiver
// header alone: the receiver's whole reason to exist is to decode 1.6 frames
// into these structs and hand them to the callbacks.
#include "hud_nav16.h"

// Start/stop the receiver thread + socket. Started after a successful
// aap_start_session when use_protocol_v1_6 is set and stopped after
// aap_stop_session (or before destroy as a fallback). Both are idempotent.
//
// start() takes the callbacks the decoder delivers decoded 1.6 frames to: they
// are registered before the thread spawns (so the first frame already has a
// destination) and detached by stop().
void hud_nav16_rx_start(HudNav16GuidanceFn on_guidance,
                        HudNav16PositionFn on_position,
                        HudNav16StatusFn   on_status);
void hud_nav16_rx_stop(void);

// TRUE once an admitted 1.6 sink callback marks this device session as seen.
// rx_start resets the latch before the receiver exists; hud.cpp marks it only
// while holding its producer lock, so a stopped session cannot suppress the
// next session's legacy fallback.
bool hud_nav16_rx_seen(void);
void hud_nav16_rx_mark_seen(void);

#endif // LIBPATCH_BLMJCIAAPA_HUD_NAV16_RX_H
