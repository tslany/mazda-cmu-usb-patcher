// SPDX-License-Identifier: AGPL-3.0-or-later
//
// CarPlay protocol adapter for the common projection-guidance broker.
//
// This file intentionally contains no HUD transport, D-Bus connection, sender
// thread, refresh ticker, or cross-source ownership workaround. nav.cpp keeps
// the lossless Apple/FW74 route-window decoder in the jciCARPLAY process; this
// adapter only converts its selected maneuver to one complete Mazda-domain
// Snapshot and updates the process-long IPC client.

#include "../patch.h"
#include "hud_send.h"
#include "common/guidance/client.h"

#include <pthread.h>
#include <stdint.h>
#include <string.h>

namespace {

using mazda_cmu::guidance::CLIENT_IPC;
using mazda_cmu::guidance::GUIDANCE_STREET_BYTES;
using mazda_cmu::guidance::RELEASE_UNSUPPORTED_MANEUVER;
using mazda_cmu::guidance::SOURCE_CARPLAY;
using mazda_cmu::guidance::Snapshot;
using mazda_cmu::guidance::client_claim;
using mazda_cmu::guidance::client_publish;
using mazda_cmu::guidance::client_release;
using mazda_cmu::guidance::client_session_begin;
using mazda_cmu::guidance::client_session_end;
using mazda_cmu::guidance::client_start;

pthread_mutex_t g_state_mu = PTHREAD_MUTEX_INITIALIZER;
uint64_t g_epoch = 0;
bool g_tbt_authorized = false;
bool g_have_supported = false;
Snapshot g_snapshot; // static storage: zero-initialized before the constructor

enum MazdaIcon : uint8_t {
    HUD_STRAIGHT          = 1,
    HUD_LEFT              = 2,
    HUD_RIGHT             = 3,
    HUD_SLIGHT_LEFT       = 4,
    HUD_SLIGHT_RIGHT      = 5,
    HUD_OFF_RAMP_RIGHT    = 7,
    HUD_DESTINATION       = 8,
    HUD_SHARP_RIGHT       = 9,
    HUD_U_TURN_RIGHT      = 10,
    HUD_SHARP_LEFT        = 11,
    HUD_FLAG              = 12,
    HUD_U_TURN_LEFT       = 13,
    HUD_FORK_RIGHT        = 14,
    HUD_FORK_LEFT         = 15,
    HUD_MERGE_LEFT        = 16,
    HUD_MERGE_RIGHT       = 17,
    HUD_OFF_RAMP_LEFT     = 30,
    HUD_DESTINATION_LEFT  = 33,
    HUD_DESTINATION_RIGHT = 34,
    HUD_FLAG_LEFT         = 35,
    HUD_FLAG_RIGHT        = 36,
};

constexpr uint8_t kTurnIcons[20][3] = {
    /* UNKNOWN          */ {0, 0, 0},
    /* DEPART           */ {HUD_FLAG_LEFT, HUD_FLAG_RIGHT, HUD_FLAG},
    /* NAME_CHANGE      */ {HUD_STRAIGHT, HUD_STRAIGHT, HUD_STRAIGHT},
    /* SLIGHT_TURN      */ {HUD_SLIGHT_LEFT, HUD_SLIGHT_RIGHT, HUD_STRAIGHT},
    /* TURN             */ {HUD_LEFT, HUD_RIGHT, HUD_STRAIGHT},
    /* SHARP_TURN       */ {HUD_SHARP_LEFT, HUD_SHARP_RIGHT, HUD_STRAIGHT},
    /* U_TURN           */ {HUD_U_TURN_LEFT, HUD_U_TURN_RIGHT, HUD_STRAIGHT},
    /* ON_RAMP          */ {HUD_LEFT, HUD_RIGHT, HUD_STRAIGHT},
    /* OFF_RAMP         */ {HUD_OFF_RAMP_LEFT, HUD_OFF_RAMP_RIGHT, HUD_STRAIGHT},
    /* FORK             */ {HUD_FORK_LEFT, HUD_FORK_RIGHT, HUD_STRAIGHT},
    /* MERGE            */ {HUD_MERGE_LEFT, HUD_MERGE_RIGHT, HUD_STRAIGHT},
    /* ROUNDABOUT_ENTER */ {0, 0, 0},
    /* ROUNDABOUT_EXIT  */ {0, 0, 0},
    /* ROUNDABOUT BOTH  */ {0, 0, 0},
    /* STRAIGHT         */ {HUD_STRAIGHT, HUD_STRAIGHT, HUD_STRAIGHT},
    /* unassigned       */ {0, 0, 0},
    /* FERRY_BOAT       */ {0, 0, 0},
    /* FERRY_TRAIN      */ {0, 0, 0},
    /* unassigned       */ {0, 0, 0},
    /* DESTINATION      */ {HUD_DESTINATION_LEFT, HUD_DESTINATION_RIGHT,
                            HUD_DESTINATION},
};

// iAP2's exit bearing is 0=straight, positive=right/clockwise. Mazda's
// roundabout atlas uses a downward zero and counter-clockwise sector order, so
// reflect and rotate before rounding to one of its twelve 30-degree glyphs.
uint8_t carplay_roundabout_icon(int32_t bearing, bool left_hand_traffic)
{
    int32_t sector_degrees = (180 - bearing) % 360;
    if (sector_degrees < 0) sector_degrees += 360;
    const uint8_t sector =
        static_cast<uint8_t>(((sector_degrees + 15) / 30) % 12);
    return static_cast<uint8_t>(sector + (left_hand_traffic ? 49 : 37));
}

uint32_t resolve_maneuver_icon(uint32_t event, uint32_t side, int32_t angle)
{
    if (event == 13) {
        return carplay_roundabout_icon(angle, side == 1);
    }
    if (event >= 20) return 0;
    int side_index = static_cast<int>(side) - 1;
    if (side_index < 0 || side_index > 2) side_index = 2;
    return kTurnIcons[event][side_index];
}

uint8_t map_distance_unit(uint32_t unit)
{
    switch (unit) {
    case 1: return 1; // meters
    case 2: return 3; // 10 km -> km
    case 3: return 3; // km
    case 4: return 2; // 10 mi -> mi
    case 5: return 2; // mi
    case 6: return 5; // feet
    default: return 0;
    }
}

// Preserve the proven CarPlay text behavior: the HUD can draw one- and
// two-byte Latin code points but not Vietnamese precomposed U+1Exx glyphs.
// Fold those to their renderable base; replace other long glyphs with '?'.
uint32_t fold_vietnamese(uint32_t cp)
{
    switch (cp) {
    case 0x1EA0: case 0x1EA2: return 'A';
    case 0x1EA4: case 0x1EA6: case 0x1EA8: case 0x1EAA: case 0x1EAC: return 0x00C2;
    case 0x1EAE: case 0x1EB0: case 0x1EB2: case 0x1EB4: case 0x1EB6: return 0x0102;
    case 0x1EB8: case 0x1EBA: case 0x1EBC: return 'E';
    case 0x1EBE: case 0x1EC0: case 0x1EC2: case 0x1EC4: case 0x1EC6: return 0x00CA;
    case 0x1EC8: case 0x1ECA: return 'I';
    case 0x1ECC: case 0x1ECE: return 'O';
    case 0x1ED0: case 0x1ED2: case 0x1ED4: case 0x1ED6: case 0x1ED8: return 0x00D4;
    case 0x1EDA: case 0x1EDC: case 0x1EDE: case 0x1EE0: case 0x1EE2: return 0x01A0;
    case 0x1EE4: case 0x1EE6: return 'U';
    case 0x1EE8: case 0x1EEA: case 0x1EEC: case 0x1EEE: case 0x1EF0: return 0x01AF;
    case 0x1EF2: case 0x1EF4: case 0x1EF6: case 0x1EF8: return 'Y';
    case 0x1EA1: case 0x1EA3: return 'a';
    case 0x1EA5: case 0x1EA7: case 0x1EA9: case 0x1EAB: case 0x1EAD: return 0x00E2;
    case 0x1EAF: case 0x1EB1: case 0x1EB3: case 0x1EB5: case 0x1EB7: return 0x0103;
    case 0x1EB9: case 0x1EBB: case 0x1EBD: return 'e';
    case 0x1EBF: case 0x1EC1: case 0x1EC3: case 0x1EC5: case 0x1EC7: return 0x00EA;
    case 0x1EC9: case 0x1ECB: return 'i';
    case 0x1ECD: case 0x1ECF: return 'o';
    case 0x1ED1: case 0x1ED3: case 0x1ED5: case 0x1ED7: case 0x1ED9: return 0x00F4;
    case 0x1EDB: case 0x1EDD: case 0x1EDF: case 0x1EE1: case 0x1EE3: return 0x01A1;
    case 0x1EE5: case 0x1EE7: return 'u';
    case 0x1EE9: case 0x1EEB: case 0x1EED: case 0x1EEF: case 0x1EF1: return 0x01B0;
    case 0x1EF3: case 0x1EF5: case 0x1EF7: case 0x1EF9: return 'y';
    default: return 0;
    }
}

void normalize_street(const char *source, char *dest, size_t dest_size)
{
    if (!dest || dest_size == 0) return;
    if (!source) {
        dest[0] = '\0';
        return;
    }

    const unsigned char *read =
        reinterpret_cast<const unsigned char *>(source);
    size_t written = 0;
    const size_t limit = dest_size - 1;

    while (*read && written < limit) {
        const unsigned char lead = read[0];
        if ((lead & 0x80) == 0) {
            dest[written++] = static_cast<char>(*read++);
            continue;
        }
        if (lead >= 0xC2 && lead <= 0xDF && read[1] &&
            (read[1] & 0xC0) == 0x80) {
            if (written + 2 > limit) break;
            dest[written++] = static_cast<char>(read[0]);
            dest[written++] = static_cast<char>(read[1]);
            read += 2;
            continue;
        }

        uint32_t code_point = 0;
        size_t input_bytes = 1;
        if ((lead & 0xF0) == 0xE0 && read[1] && read[2] &&
            (read[1] & 0xC0) == 0x80 && (read[2] & 0xC0) == 0x80) {
            code_point = ((lead & 0x0f) << 12) |
                         ((read[1] & 0x3f) << 6) | (read[2] & 0x3f);
            input_bytes = 3;
        } else if ((lead & 0xF8) == 0xF0 && read[1] && read[2] && read[3] &&
                   (read[1] & 0xC0) == 0x80 &&
                   (read[2] & 0xC0) == 0x80 &&
                   (read[3] & 0xC0) == 0x80) {
            input_bytes = 4;
        } else {
            ++read; // malformed byte; never forward invalid UTF-8 to D-Bus
            continue;
        }

        const uint32_t folded =
            input_bytes == 3 ? fold_vietnamese(code_point) : 0;
        if (folded != 0 && folded < 0x80) {
            dest[written++] = static_cast<char>(folded);
        } else if (folded >= 0x80 && folded < 0x800) {
            if (written + 2 > limit) break;
            dest[written++] = static_cast<char>(0xc0 | (folded >> 6));
            dest[written++] = static_cast<char>(0x80 | (folded & 0x3f));
        } else {
            dest[written++] = '?';
        }
        read += input_bytes;
    }
    dest[written] = '\0';
}

bool epoch_is_current_locked(uint64_t epoch)
{
    return epoch != 0 && epoch == g_epoch;
}

void reset_snapshot_locked()
{
    memset(&g_snapshot, 0, sizeof(g_snapshot));
    g_have_supported = false;
}

} // namespace

bool hud_send_start(void)
{
    return client_start(SOURCE_CARPLAY, CLIENT_IPC);
}

uint64_t hud_session_begin(void)
{
    const uint64_t epoch = client_session_begin();
    pthread_mutex_lock(&g_state_mu);
    reset_snapshot_locked();
    g_tbt_authorized = false;
    g_epoch = epoch;
    pthread_mutex_unlock(&g_state_mu);
    return epoch;
}

void hud_session_claim(uint64_t epoch)
{
    Snapshot snapshot;
    bool current = false;
    bool publish = false;
    pthread_mutex_lock(&g_state_mu);
    if (epoch_is_current_locked(epoch)) {
        current = true;
        g_tbt_authorized = true;
        publish = g_have_supported;
        if (publish) snapshot = g_snapshot;
    }
    pthread_mutex_unlock(&g_state_mu);
    if (!current) return;

    client_claim(epoch);
    if (publish) client_publish(epoch, snapshot);
}

void hud_session_release(uint64_t epoch,
                         mazda_cmu::guidance::ReleaseReason reason)
{
    bool current = false;
    pthread_mutex_lock(&g_state_mu);
    if (epoch_is_current_locked(epoch)) {
        current = true;
        g_tbt_authorized = false;
        reset_snapshot_locked();
    }
    pthread_mutex_unlock(&g_state_mu);
    if (current) client_release(epoch, reason);
}

void hud_session_end(uint64_t epoch)
{
    bool current = false;
    pthread_mutex_lock(&g_state_mu);
    if (epoch_is_current_locked(epoch)) {
        current = true;
        g_tbt_authorized = false;
        reset_snapshot_locked();
        g_epoch = 0;
    }
    pthread_mutex_unlock(&g_state_mu);
    if (current) client_session_end(epoch);
}

void hud_on_next_turn(uint64_t epoch, const char *road_name,
                      uint32_t turn_side, uint32_t turn_event,
                      int32_t turn_angle, int32_t turn_number)
{
    (void)turn_number; // the Mazda HUD frame has no maneuver-number field
    const uint32_t glyph =
        resolve_maneuver_icon(turn_event, turn_side, turn_angle);
    char street[GUIDANCE_STREET_BYTES];
    normalize_street(road_name, street, sizeof(street));
    if (street[0] == '\0') {
        street[0] = ' ';
        street[1] = '\0';
    }

    Snapshot snapshot;
    bool publish = false;
    bool release_unsupported = false;
    pthread_mutex_lock(&g_state_mu);
    if (epoch_is_current_locked(epoch)) {
        if (glyph == 0) {
            reset_snapshot_locked();
            release_unsupported = g_tbt_authorized;
        } else {
            const bool changed = !g_have_supported ||
                                 g_snapshot.glyph != glyph ||
                                 strcmp(g_snapshot.street, street) != 0;
            g_snapshot.glyph = glyph;
            strncpy(g_snapshot.street, street,
                    sizeof(g_snapshot.street) - 1);
            g_snapshot.street[sizeof(g_snapshot.street) - 1] = '\0';
            if (changed) {
                // Distance belongs to the previous maneuver until the following
                // 0x8058 guidance record supplies the new value. Preserve the
                // proven CarPlay fallback of displaying zero metres meanwhile.
                g_snapshot.distance_x10 = 0;
                g_snapshot.distance_unit = 1;
            }
            g_have_supported = true;
            publish = g_tbt_authorized;
            if (publish) snapshot = g_snapshot;
        }
    }
    pthread_mutex_unlock(&g_state_mu);

    if (release_unsupported) {
        client_release(epoch, RELEASE_UNSUPPORTED_MANEUVER);
    } else if (publish) {
        client_publish(epoch, snapshot);
    }
}

void hud_on_distance(uint64_t epoch, int32_t distance_meters,
                     int32_t time_until_seconds, int32_t display_distance,
                     uint32_t display_distance_unit)
{
    (void)distance_meters;
    (void)time_until_seconds;

    int32_t distance_x10 = display_distance / 100;
    if (distance_x10 < 0) distance_x10 = 0;
    if (distance_x10 > 0xffff) distance_x10 = 0xffff;
    const uint8_t mapped_unit = map_distance_unit(display_distance_unit);
    // The old sender normalized an absent/unknown Apple unit to metres before
    // entering Mazda's final domain. Keep that source-specific behavior here.
    const uint8_t distance_unit = mapped_unit ? mapped_unit : 1;

    Snapshot snapshot;
    bool publish = false;
    pthread_mutex_lock(&g_state_mu);
    if (epoch_is_current_locked(epoch)) {
        g_snapshot.distance_x10 = distance_x10;
        g_snapshot.distance_unit = distance_unit;
        publish = g_tbt_authorized && g_have_supported;
        if (publish) snapshot = g_snapshot;
    }
    pthread_mutex_unlock(&g_state_mu);
    if (publish) client_publish(epoch, snapshot);
}
