// HUD hooks used by lifecycle.cpp around aap_create_session. Android Auto is a
// session-scoped producer; the common guidance broker/output sink is
// process-long so it can also serve CarPlay while AA is detached.

#ifndef LIBPATCH_BLMJCIAAPA_HUD_HUD_H
#define LIBPATCH_BLMJCIAAPA_HUD_HUD_H

#include <stdint.h>

// Defined in hud.cpp. Called from the aap_create_session PLT shim
// BEFORE chaining to the real SDK entry point, to substitute the
// (NULL on OEM) navigation callback slot in cb_list with one that
// dumps received 0x500 / 0x501 / 0x502 events to the LOGD sink.
// Safe to call with cb_list==NULL (becomes a no-op).
void hud_pre_aap_create_session(void *cb_list);

// Defined in hud.cpp. Called after a successful real aap_create_session to keep
// HUD producers inactive until a device session actually starts.
void hud_post_aap_create_session(void);

// Defined in hud.cpp. Called after a successful real aap_start_session to reset
// retained state and admit navigation callbacks for the new device session.
void hud_post_aap_start_session(void);

// Defined in hud.cpp. Called after the real aap_stop_session to close the AA
// epoch, join the GAL receiver, and release only AA's guidance ownership.
void hud_post_aap_stop_session(void);

// Defined in hud.cpp. Called before the real aap_destroy_session as an
// idempotent final close of the AA source epoch.
void hud_pre_aap_destroy_session(void);

// === Android Auto GAL 1.6 path ================================
//
// The aap_service shim (a SEPARATE process) swallows the 1.6 navigation frames
// and relays them RAW over an abstract AF_UNIX datagram socket. The receiver
// thread (nav16_rx.cpp) hands each raw frame to hud_nav16_feed() (hud_nav16.h),
// which decodes the AA protocol and invokes the sink callbacks hud.cpp
// registers; those map it to the same complete Mazda-domain snapshot used by
// the 1.5 path and submit it to the process-long common broker.

// Defined in hud.cpp. Reset the GAL 1.6 HUD accumulator + change-gate.
// Called from hud_nav16_rx_start() while the receiver thread is NOT running, so
// a session that ended abnormally — cable pull, no INACTIVE/STOP received —
// can't leave a stale gate that suppresses the next session's first frame.
void hud_feed_nav16_reset(void);

#endif // LIBPATCH_BLMJCIAAPA_HUD_HUD_H
