// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Fixed release behavior shared by the projection shims. The USB product is
// intentionally installed as one coherent feature, so end users do not need a
// runtime configuration file. The sole runtime switch is a persistent marker
// owned by this product: its presence disables pause-on-mute.

#ifndef LIBPATCH_COMMON_CONFIG_H
#define LIBPATCH_COMMON_CONFIG_H

#include "log.h"

#include <unistd.h>

#pragma push_macro("LOG_TAG")
#undef LOG_TAG
#define LOG_TAG "CONFIG"

namespace libpatch_config {

constexpr const char *kDisableMutePauseMarker =
    "/data_persist/mazda-cmu-usb-patcher/disable-mute-pause";

struct Settings {
    bool mute_pauses_phone;
    bool loaded;

    Settings() : mute_pauses_phone(true), loaded(false) {}
};

inline Settings &settings()
{
    static Settings s;
    return s;
}

// `sym_in_self` is retained in the API so existing callers keep the same
// initialization shape. There is no sibling configuration file to locate.
inline void load(const void *sym_in_self)
{
    (void)sym_in_self;
    Settings &s = settings();
    if (s.loaded) return;
    s.loaded = true;
    s.mute_pauses_phone = access(kDisableMutePauseMarker, F_OK) != 0;
    LOGD("release settings: touch=true hud=true guidance_broker=jciAAPA "
         "guidance_output=svcnavi "
         "force_street_name=true hud_fold_latin=true protocol=1.6 "
         "block_headunit_media_play=true mute_pauses_phone=%s",
         s.mute_pauses_phone ? "true" : "false");
}

inline bool touch_enabled() { return true; }
inline bool hud_enabled() { return true; }
inline bool force_street_name() { return true; }
inline bool hud_fold_latin() { return true; }
inline bool use_protocol_v1_6() { return true; }
inline bool mute_pauses_phone() { return settings().mute_pauses_phone; }
inline bool unmute_starts_playback() { return false; }
inline bool block_headunit_media_play() { return true; }
inline bool bt_pairing_bypass_all_devices() { return false; }

} // namespace libpatch_config

#pragma pop_macro("LOG_TAG")

#endif // LIBPATCH_COMMON_CONFIG_H
