// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Adapted from headunit (https://github.com/Trevelopment/headunit),
// licensed under GNU AGPL v3.
// See src/projection/NOTICE.md for attribution.

#define LOG_TAG "HUD"
#include "../log.h"
#include "hud.h"
#include "nav16_rx.h"        // receiver lifecycle/seen + re-exported hud_nav16 API
                             // (HudNav16Sink, AaGuidance/Position/Status, AaLane, mappers)
#include "common/config.h"
#include "common/guidance/client.h"
#include "common/guidance/model.h"
#include "common/string_safe.h"
#include "projection.h"
#include "translit.h"   // hud_translit::fold() — precomposed-Latin street-name fold
#include "hud_nav.h"    // compute_turn_icon() — AA turn fields -> Mazda HUD glyph
#include "hud_lane.h"   // oem_lane_code_for_aa — AA lanes -> OEM lane codes

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <mutex>

namespace {

// Encode the decoded 1.6 lanes to 8 OEM lane CODES (0..70), LEFT to RIGHT; an
// absent slot is HIDDEN (canonical 0 == OEM_LANE_NONE).
// hud_lane.h owns the full AA-shape -> OEM-code mapping: single arrows, the
// multi-direction combos, and the recommended-direction detail codes that pick
// out the highlighted arrow within a combo.
void nav16_encode_lane_codes(const AaLane *lanes, int n, uint8_t out[HUD_NAV16_MAX_LANES])
{
    for (int i = 0; i < HUD_NAV16_MAX_LANES; ++i) {
        out[i] = (lanes && i < n)
                     ? (uint8_t)oem_lane_code_for_aa(lanes[i].present_mask,
                                                     lanes[i].highlight_mask)
                     : (uint8_t)OEM_LANE_NONE;   // 0 = hidden
    }
}

// Legacy SDK callbacks and GAL 1.6 receiver callbacks can arrive on different
// threads. Serialize their one complete source snapshot and session boundary.
std::mutex g_nav_writer_mu;
bool g_aap_session_active = false; // guarded by g_nav_writer_mu

namespace guidance = mazda_cmu::guidance;

// One complete Mazda-domain Android Auto frame. The AA callback and GAL 1.6
// receiver paths both mutate this under g_nav_writer_mu, then submit one atomic
// snapshot to the common process broker. Output transport and cross-protocol
// ownership no longer live in this adapter.
uint64_t            g_guidance_epoch = 0;
// Static storage is zero-initialized before the preload constructor runs.
guidance::Snapshot  g_guidance;
guidance::Snapshot  g_last_published;
bool                g_have_last_published = false;

void guidance_reset()
{
    memset(&g_guidance, 0, sizeof(g_guidance));
    memset(&g_last_published, 0, sizeof(g_last_published));
    g_have_last_published = false;
}

void guidance_publish_if_changed()
{
    if (g_guidance_epoch == 0 || g_guidance.glyph == 0) return;
    if (g_have_last_published &&
        memcmp(&g_guidance, &g_last_published, sizeof(g_guidance)) == 0) {
        return;
    }
    guidance::client_publish(g_guidance_epoch, g_guidance);
    g_last_published = g_guidance;
    g_have_last_published = true;
}

void guidance_release(guidance::ReleaseReason reason)
{
    if (g_guidance_epoch != 0) {
        guidance::client_release(g_guidance_epoch, reason);
    }
    guidance_reset();
}

void guidance_claim()
{
    if (g_guidance_epoch != 0) guidance::client_claim(g_guidance_epoch);
}

void copy_street_bounded(const char *src, size_t src_len,
                         char *dst, size_t dst_cap)
{
    if (!dst || dst_cap == 0) return;
    libpatch::copy_utf8_truncated(dst, dst_cap, src, src_len);
    if (libpatch_config::hud_fold_latin()) {
        hud_translit::fold(dst);
        const size_t n = strnlen(dst, dst_cap);
        memset(dst + n, 0, dst_cap - n);
    }
}

void guidance_status(uint32_t status)
{
    if (status == 1u) {
        guidance_claim();
    } else {
        guidance_release(guidance::RELEASE_ROUTE_STOPPED);
    }
}

void guidance_next_turn(const char *road, size_t road_len, uint32_t side,
                        uint32_t event, int32_t angle)
{
    uint32_t icon = compute_turn_icon(event, side, angle);
    if (icon == 0 || icon > 60) {
        guidance_release(guidance::RELEASE_UNSUPPORTED_MANEUVER);
        return;
    }

    char street[sizeof(g_guidance.street)];
    copy_street_bounded(road, road_len, street, sizeof(street));
    const bool changed = g_guidance.glyph != icon ||
                         strcmp(g_guidance.street, street) != 0;
    g_guidance.glyph = icon;
    memcpy(g_guidance.street, street, sizeof(g_guidance.street));
    memset(g_guidance.lanes, 0, sizeof(g_guidance.lanes));
    if (changed) {
        g_guidance.distance_x10 = 0;
        g_guidance.distance_unit = 0;
    }
    guidance_publish_if_changed();
}

void guidance_distance(int32_t display_distance, uint32_t display_unit)
{
    if (g_guidance.glyph == 0) return;
    int32_t value = display_distance / 100;
    if (value < 0) value = 0;
    if (value > 0xffff) value = 0xffff;
    g_guidance.distance_x10 = value;
    g_guidance.distance_unit = map_distance_unit(display_unit);
    guidance_publish_if_changed();
}

namespace sdk_layout = fw74_00_324a::aap_sdk;

// Tag values that prefix every 36-byte event header. Pinned by the
// producer in aap_service.c (navigationStatusCallback emits 0x500
// from a NAVMessagesStatus proto; navigationNextTurnCallback emits
// 0x501 from a NAVTurnMessage; navigationNextTurnDistanceCallback
// emits 0x502 from a NAVDistanceMessage).
constexpr uint32_t kTagStatus    = 0x500;   // NAVMessagesStatus
constexpr uint32_t kTagNextTurn  = 0x501;   // NAVTurnMessage
constexpr uint32_t kTagDistance  = 0x502;   // NAVDistanceMessage

// NAVMessagesStatus.STATUS — proto enum, hu.proto:
//   START = 1; STOP = 2;
// Any other value seen on the wire is an unspecified / unknown
// state; the producer in aap_service is the only thing in front of
// us that could synthesise one, and it forwards verbatim.
enum NavStatusEnum : uint32_t {
    NAV_STATUS_START = 1,
    NAV_STATUS_STOP  = 2,
};

// NAVTurnMessage.TURN_SIDE — proto enum, hu.proto:
//   TURN_LEFT = 1; TURN_RIGHT = 2; TURN_UNSPECIFIED = 3;
enum NavTurnSideEnum : uint32_t {
    NAV_TURN_LEFT        = 1,
    NAV_TURN_RIGHT       = 2,
    NAV_TURN_UNSPECIFIED = 3,
};

// NAVTurnMessage.TURN_EVENT — proto enum, hu.proto (sparse — 15
// and 18 are intentionally unassigned in the .proto, so 0x11=17
// and 0x13=19 are the only valid values in that gap):
//   0  TURN_UNKNOWN              10 TURN_MERGE
//   1  TURN_DEPART               11 TURN_ROUNDABOUT_ENTER
//   2  TURN_NAME_CHANGE          12 TURN_ROUNDABOUT_EXIT
//   3  TURN_SLIGHT_TURN          13 TURN_ROUNDABOUT_ENTER_AND_EXIT
//   4  TURN_TURN                 14 TURN_STRAIGHT
//   5  TURN_SHARP_TURN           16 TURN_FERRY_BOAT
//   6  TURN_U_TURN               17 TURN_FERRY_TRAIN
//   7  TURN_ON_RAMP              19 TURN_DESTINATION
//   8  TURN_OFF_RAMP
//   9  TURN_FORK
enum NavTurnEventEnum : uint32_t {
    NAV_TURN_EVENT_UNKNOWN                  =  0,
    NAV_TURN_EVENT_DEPART                   =  1,
    NAV_TURN_EVENT_NAME_CHANGE              =  2,
    NAV_TURN_EVENT_SLIGHT_TURN              =  3,
    NAV_TURN_EVENT_TURN                     =  4,
    NAV_TURN_EVENT_SHARP_TURN               =  5,
    NAV_TURN_EVENT_U_TURN                   =  6,
    NAV_TURN_EVENT_ON_RAMP                  =  7,
    NAV_TURN_EVENT_OFF_RAMP                 =  8,
    NAV_TURN_EVENT_FORK                     =  9,
    NAV_TURN_EVENT_MERGE                    = 10,
    NAV_TURN_EVENT_ROUNDABOUT_ENTER         = 11,
    NAV_TURN_EVENT_ROUNDABOUT_EXIT          = 12,
    NAV_TURN_EVENT_ROUNDABOUT_ENTER_AND_EXIT= 13,
    NAV_TURN_EVENT_STRAIGHT                 = 14,
    NAV_TURN_EVENT_FERRY_BOAT               = 16,
    NAV_TURN_EVENT_FERRY_TRAIN              = 17,
    NAV_TURN_EVENT_DESTINATION              = 19,
};

// The value at +0x10 is NOT the raw proto turn_event — the producer
// re-bases it first. Before forwarding, the producer compacts the
// sparse proto enum (which skips 15 and 18) into a dense 0..17 enum,
// i.e. it deletes the two gaps. Its mapping is, equivalently:
//
//     proto 0..14            -> unchanged   (0..14)
//     proto 16 FERRY_BOAT    -> 15
//     proto 17 FERRY_TRAIN   -> 16
//     proto 19 DESTINATION   -> 17
//     anything else          -> garbage (producer logs a warning and
//                               forwards an uninitialised local)
//
// So the byte we receive for the destination maneuver is 17, which —
// if read as a proto value — collides with FERRY_TRAIN and resolves
// to a blank glyph. Undo the compaction here so the rest of the HUD
// path (NavTurnEventEnum, nav_turn_event_name, kTurnIcons) can keep
// indexing by the genuine proto values it was built and validated
// against. Only the top three events move; 0..14 are identity.
uint32_t decode_turn_event(uint32_t compacted)
{
    switch (compacted) {
    case 15: return NAV_TURN_EVENT_FERRY_BOAT;   // 15 -> 16
    case 16: return NAV_TURN_EVENT_FERRY_TRAIN;  // 16 -> 17
    case 17: return NAV_TURN_EVENT_DESTINATION;  // 17 -> 19
    default: return compacted;                   // 0..14 pass through;
                                                 // 18/19/… never appear
                                                 // (producer can't emit
                                                 // them) but stay benign.
    }
}

// NAVDistanceMessage.DISPLAY_DISTANCE_UNIT — proto enum, hu.proto:
//   METERS       = 1  // meters    * 1000 (display < 1000 m)
//   KILOMETERS10 = 2  // km        * 1000 (display > 10 km)
//   KILOMETERS   = 3  // km        * 1000 (display 1..10 km)
//   MILES10      = 4  // miles     * 1000 (display > 10 mi)
//   MILES        = 5  // miles     * 1000 (display 1..10 mi)
//   FEET         = 6  // feet      * 1000
enum NavDistanceUnitEnum : uint32_t {
    NAV_DISTUNIT_METERS       = 1,
    NAV_DISTUNIT_KILOMETERS10 = 2,
    NAV_DISTUNIT_KILOMETERS   = 3,
    NAV_DISTUNIT_MILES10      = 4,
    NAV_DISTUNIT_MILES        = 5,
    NAV_DISTUNIT_FEET         = 6,
};

// 0x501 header field offsets — sized for IMAGE_CODES_ONLY cluster
// mode but the layout itself is identical in CUSTOM_IMAGES_SUPPORTED
// mode (icon pointer/length are just NULL/0 when the receiver-lib
// advertises IMAGE_CODES_ONLY). All offsets are byte offsets into
// the 36-byte buffer the SDK hands us.
//
// Layout (verified against §5.2 of the impl doc and the dispatcher
// prep at libaap_interface.so:0x14f48):
//   +0x00  uint32   tag                  (= 0x501)
//   +0x04  char *   road_name            (heap, freed by SDK on return;
//                                         proto calls this `event_name`)
//   +0x08  uint32   road_name_len
//   +0x0C  uint32   turn_side            (NAVTurnMessage.TURN_SIDE)
//   +0x10  uint32   turn_event           (producer-COMPACTED TURN_EVENT,
//                                         dense 0..17 — NOT the raw proto
//                                         value; decode_turn_event() maps it
//                                         back to the sparse proto enum)
//   +0x14  void *   image                (PNG bytes when IMAGE mode;
//                                         NULL in IMAGE_CODES_ONLY mode)
//   +0x18  uint32   image_len            (0 in IMAGE_CODES_ONLY mode)
//   +0x1C  int32    turn_angle           (degrees, signed)
//   +0x20  int32    turn_number          (maneuver/exit number)
struct NextTurnHdr {
    uint32_t    tag;
    const char *road_name;
    uint32_t    road_name_len;
    uint32_t    turn_side;
    uint32_t    turn_event;   // producer-compacted; run through decode_turn_event()
    const void *image;
    uint32_t    image_len;
    int32_t     turn_angle;
    int32_t     turn_number;
};
static_assert(sizeof(NextTurnHdr) == sdk_layout::LegacyNavEventBytes,
              "NextTurnHdr size does not match the FW74 SDK");
static_assert(offsetof(NextTurnHdr, tag) == sdk_layout::NextTurnTagOffset,
              "NextTurnHdr.tag offset mismatch");
static_assert(offsetof(NextTurnHdr, road_name) == sdk_layout::NextTurnRoadNameOffset,
              "NextTurnHdr.road_name offset mismatch");
static_assert(offsetof(NextTurnHdr, road_name_len) ==
                  sdk_layout::NextTurnRoadNameLengthOffset,
              "NextTurnHdr.road_name_len offset mismatch");
static_assert(offsetof(NextTurnHdr, turn_side) == sdk_layout::NextTurnSideOffset,
              "NextTurnHdr.turn_side offset mismatch");
static_assert(offsetof(NextTurnHdr, turn_event) == sdk_layout::NextTurnEventOffset,
              "NextTurnHdr.turn_event offset mismatch");
static_assert(offsetof(NextTurnHdr, image) == sdk_layout::NextTurnImageOffset,
              "NextTurnHdr.image offset mismatch");
static_assert(offsetof(NextTurnHdr, image_len) ==
                  sdk_layout::NextTurnImageLengthOffset,
              "NextTurnHdr.image_len offset mismatch");
static_assert(offsetof(NextTurnHdr, turn_angle) == sdk_layout::NextTurnAngleOffset,
              "NextTurnHdr.turn_angle offset mismatch");
static_assert(offsetof(NextTurnHdr, turn_number) == sdk_layout::NextTurnNumberOffset,
              "NextTurnHdr.turn_number offset mismatch");

// 0x500 header — only status is meaningful, the remaining 28 bytes
// are zeroed by the dispatcher's memset(stack_buf, 0, 36) at
// libaap_interface.so:0x14cb0. The wire proto only defines
// START=1 / STOP=2; the producer forwards verbatim, so an
// out-of-range value here means a malformed or version-skewed phone.
struct StatusHdr {
    uint32_t tag;       // = 0x500
    uint32_t status;    // NAVMessagesStatus.STATUS — 1=START, 2=STOP
    uint32_t reserved[7];
};
static_assert(sizeof(StatusHdr) == sdk_layout::LegacyNavEventBytes,
              "StatusHdr size does not match the FW74 SDK");
static_assert(offsetof(StatusHdr, tag) == sdk_layout::StatusTagOffset,
              "StatusHdr.tag offset mismatch");
static_assert(offsetof(StatusHdr, status) == sdk_layout::StatusValueOffset,
              "StatusHdr.status offset mismatch");
static_assert(offsetof(StatusHdr, reserved) == sdk_layout::StatusReservedOffset,
              "StatusHdr.reserved offset mismatch");

// 0x502 header — distance / ETA. Last 16 bytes reserved.
//
// `display_distance` is `uint64` on the proto wire but the SDK's
// 36-byte buffer only allocates 4 bytes for it (the next field —
// `display_distance_unit` — sits immediately after at +0x10), so
// the producer truncates to the low 32 bits. In practice values are
// always small (millimetre-precision distance up to ~10 mi/km, well
// under 2^31), so the truncation is loss-free.
struct DistanceHdr {
    uint32_t tag;                      // = 0x502
    int32_t  distance;                 // proto `distance` — meters to next maneuver
    int32_t  time_until;               // proto `time_until` — seconds to next maneuver
    int32_t  display_distance;         // proto `display_distance` low 32 bits
                                       // (raw unit value * 1000; see enum below)
    uint32_t display_distance_unit;    // NAVDistanceMessage.DISPLAY_DISTANCE_UNIT, 1..6
    uint32_t reserved[4];
};
static_assert(sizeof(DistanceHdr) == sdk_layout::LegacyNavEventBytes,
              "DistanceHdr size does not match the FW74 SDK");
static_assert(offsetof(DistanceHdr, tag) == sdk_layout::DistanceTagOffset,
              "DistanceHdr.tag offset mismatch");
static_assert(offsetof(DistanceHdr, distance) == sdk_layout::DistanceMetersOffset,
              "DistanceHdr.distance offset mismatch");
static_assert(offsetof(DistanceHdr, time_until) == sdk_layout::DistanceTimeOffset,
              "DistanceHdr.time_until offset mismatch");
static_assert(offsetof(DistanceHdr, display_distance) ==
                  sdk_layout::DistanceDisplayOffset,
              "DistanceHdr.display_distance offset mismatch");
static_assert(offsetof(DistanceHdr, display_distance_unit) ==
                  sdk_layout::DistanceDisplayUnitOffset,
              "DistanceHdr.display_distance_unit offset mismatch");
static_assert(offsetof(DistanceHdr, reserved) == sdk_layout::DistanceReservedOffset,
              "DistanceHdr.reserved offset mismatch");

const char *nav_status_name(uint32_t v)
{
    switch (v) {
    case NAV_STATUS_START: return "START";
    case NAV_STATUS_STOP:  return "STOP";
    default:               return "?";
    }
}

const char *nav_turn_side_name(uint32_t v)
{
    switch (v) {
    case NAV_TURN_LEFT:        return "LEFT";
    case NAV_TURN_RIGHT:       return "RIGHT";
    case NAV_TURN_UNSPECIFIED: return "UNSPECIFIED";
    default:                   return "?";
    }
}

const char *nav_turn_event_name(uint32_t v)
{
    switch (v) {
    case NAV_TURN_EVENT_UNKNOWN:                   return "UNKNOWN";
    case NAV_TURN_EVENT_DEPART:                    return "DEPART";
    case NAV_TURN_EVENT_NAME_CHANGE:               return "NAME_CHANGE";
    case NAV_TURN_EVENT_SLIGHT_TURN:               return "SLIGHT_TURN";
    case NAV_TURN_EVENT_TURN:                      return "TURN";
    case NAV_TURN_EVENT_SHARP_TURN:                return "SHARP_TURN";
    case NAV_TURN_EVENT_U_TURN:                    return "U_TURN";
    case NAV_TURN_EVENT_ON_RAMP:                   return "ON_RAMP";
    case NAV_TURN_EVENT_OFF_RAMP:                  return "OFF_RAMP";
    case NAV_TURN_EVENT_FORK:                      return "FORK";
    case NAV_TURN_EVENT_MERGE:                     return "MERGE";
    case NAV_TURN_EVENT_ROUNDABOUT_ENTER:          return "ROUNDABOUT_ENTER";
    case NAV_TURN_EVENT_ROUNDABOUT_EXIT:           return "ROUNDABOUT_EXIT";
    case NAV_TURN_EVENT_ROUNDABOUT_ENTER_AND_EXIT: return "ROUNDABOUT_ENTER_AND_EXIT";
    case NAV_TURN_EVENT_STRAIGHT:                  return "STRAIGHT";
    case NAV_TURN_EVENT_FERRY_BOAT:                return "FERRY_BOAT";
    case NAV_TURN_EVENT_FERRY_TRAIN:               return "FERRY_TRAIN";
    case NAV_TURN_EVENT_DESTINATION:               return "DESTINATION";
    default:                                       return "?";
    }
}

const char *nav_distance_unit_name(uint32_t v)
{
    switch (v) {
    case NAV_DISTUNIT_METERS:       return "METERS";
    case NAV_DISTUNIT_KILOMETERS10: return "KILOMETERS10";
    case NAV_DISTUNIT_KILOMETERS:   return "KILOMETERS";
    case NAV_DISTUNIT_MILES10:      return "MILES10";
    case NAV_DISTUNIT_MILES:        return "MILES";
    case NAV_DISTUNIT_FEET:         return "FEET";
    default:                        return "?";
    }
}

// Forward declaration so substitute_nav_cb() below can take its
// address before the body is seen.
void our_nav_cb(void *user_ctx, void *hdr36);

void dump_status(const StatusHdr *h)
{
    LOGD("nav 0x500 NAVMessagesStatus: status=%u(%s)",
         static_cast<unsigned>(h->status),
         nav_status_name(h->status));
}

void dump_next_turn(const NextTurnHdr *h, uint32_t turn_event)
{
    // road_name is a heap pointer the SDK frees the moment we
    // return; for the log we copy at most a sane cap into a stack
    // buffer so the format-string read is bounded even if the SDK
    // ever forgets to NUL-terminate. road_name_len is the
    // authoritative length (excludes the trailing NUL).
    //
    // (The proto field name is `event_name` but observed payloads
    // and the SDK's internal naming both treat this as the road
    // name string. Keeping `road_name` everywhere for clarity.)
    constexpr size_t kRoadCap = 255;
    char road[kRoadCap + 1];
    libpatch::copy_utf8_truncated(road, sizeof(road), h->road_name,
                                  h->road_name_len);

    // h->turn_event is the producer's compacted value; `turn_event`
    // (passed in) has already been mapped back to the proto enum, so
    // the name lookup matches the proto table. Log both so the field
    // semantics stay traceable: raw= what arrived, = proto value.
    LOGD("nav 0x501 NAVTurnMessage: road=\"%s\" road_len=%u "
         "turn_side=%u(%s) turn_event=%u(%s) raw_event=%u "
         "turn_angle=%d turn_number=%d image=%p image_len=%u",
         road,
         static_cast<unsigned>(h->road_name_len),
         static_cast<unsigned>(h->turn_side),
         nav_turn_side_name(h->turn_side),
         static_cast<unsigned>(turn_event),
         nav_turn_event_name(turn_event),
         static_cast<unsigned>(h->turn_event),
         static_cast<int>(h->turn_angle),
         static_cast<int>(h->turn_number),
         h->image,
         static_cast<unsigned>(h->image_len));

    // Sanity check: in IMAGE_CODES_ONLY cluster mode we expect
    // image_len==0 and image==NULL. Surface a warning if the XML
    // wasn't flipped to ENUM (or if the phone ignores the advertised
    // cluster_type and sends a PNG anyway).
    if (h->image_len != 0 || h->image != nullptr) {
        LOGW("nav 0x501: unexpected image payload in IMAGE_CODES_ONLY "
             "mode (image=%p image_len=%u) — check "
             "<cluster_type>ENUM</> in aap_system_attributes*.xml "
             "(image bytes are PNG-encoded per hu.proto)",
             h->image, static_cast<unsigned>(h->image_len));
    }
}

void dump_distance(const DistanceHdr *h)
{
    LOGD("nav 0x502 NAVDistanceMessage: distance=%dm time_until=%ds "
         "display_distance=%d display_distance_unit=%u(%s)",
         static_cast<int>(h->distance),
         static_cast<int>(h->time_until),
         static_cast<int>(h->display_distance),
         static_cast<unsigned>(h->display_distance_unit),
         nav_distance_unit_name(h->display_distance_unit));
}

void our_nav_cb(void *user_ctx, void *hdr36)
{
    (void)user_ctx;  // We don't set cb_list[18], so this is NULL.

    if (!hdr36) {
        LOGW("nav cb: NULL header — SDK contract violation, ignoring");
        return;
    }

    std::lock_guard<std::mutex> writer_lk(g_nav_writer_mu);

    // Single-writer guard + 1.5 fallback: drop the legacy 0x500/0x501/0x502
    // callbacks only once 1.6 frames are actually arriving on the rx socket —
    // NOT on the config flag. aap_service (which advertises 1.6) and this
    // process share only the config key, not the negotiated session, so with
    // the flag set but the 1.6 chain broken (shim not preloaded, byte-verify
    // aborted, phone declined) the phone keeps speaking 1.5 and these
    // callbacks are the only guidance there is — dropping them would leave
    // the HUD dark. g_nav_writer_mu serializes the transition where the first
    // admitted 1.6 frame latches this legacy path off.
    if (!g_aap_session_active || hud_nav16_rx_seen()) return;

    const uint32_t tag = *static_cast<const uint32_t *>(hdr36);
    switch (tag) {
    case kTagStatus: {
        const StatusHdr *s = static_cast<const StatusHdr *>(hdr36);
        dump_status(s);
        guidance_status(s->status);
        break;
    }
    case kTagNextTurn: {
        const NextTurnHdr *t = static_cast<const NextTurnHdr *>(hdr36);
        const uint32_t turn_event = decode_turn_event(t->turn_event);
        dump_next_turn(t, turn_event);
        guidance_next_turn(t->road_name, t->road_name_len, t->turn_side,
                           turn_event, t->turn_angle);
        break;
    }
    case kTagDistance: {
        const DistanceHdr *d = static_cast<const DistanceHdr *>(hdr36);
        dump_distance(d);
        guidance_distance(d->display_distance, d->display_distance_unit);
        break;
    }
    default:
        LOGW("nav cb: unknown tag 0x%x — first 16 bytes: "
             "%02x %02x %02x %02x  %02x %02x %02x %02x  "
             "%02x %02x %02x %02x  %02x %02x %02x %02x",
             tag,
             static_cast<const uint8_t *>(hdr36)[0],
             static_cast<const uint8_t *>(hdr36)[1],
             static_cast<const uint8_t *>(hdr36)[2],
             static_cast<const uint8_t *>(hdr36)[3],
             static_cast<const uint8_t *>(hdr36)[4],
             static_cast<const uint8_t *>(hdr36)[5],
             static_cast<const uint8_t *>(hdr36)[6],
             static_cast<const uint8_t *>(hdr36)[7],
             static_cast<const uint8_t *>(hdr36)[8],
             static_cast<const uint8_t *>(hdr36)[9],
             static_cast<const uint8_t *>(hdr36)[10],
             static_cast<const uint8_t *>(hdr36)[11],
             static_cast<const uint8_t *>(hdr36)[12],
             static_cast<const uint8_t *>(hdr36)[13],
             static_cast<const uint8_t *>(hdr36)[14],
             static_cast<const uint8_t *>(hdr36)[15]);
        break;
    }
}

} // namespace

// Called from the aap_create_session PLT shim BEFORE chaining
// through. Modifying cb_list[10] in place is safe because the SDK
// copies the full 76-byte table into the session handle inside
// aap_create_session; once that copy is done, the caller's stack
// buffer is dead memory.
//
// We leave cb_list[18] (the user-context slot) untouched — the OEM
// never sets it, and we have no per-session state to thread through.
void hud_pre_aap_create_session(void *cb_list)
{
    if (!cb_list) {
        LOGW("hud_pre_aap_create_session: NULL cb_list, skipping "
             "(real aap_create_session will reject it anyway)");
        return;
    }

    void **slots = static_cast<void **>(cb_list);
    void  *prev  = slots[sdk_layout::NavigationCallbackSlot];
    slots[sdk_layout::NavigationCallbackSlot] =
        reinterpret_cast<void *>(&our_nav_cb);

    LOGD("hud_pre_aap_create_session: cb_list=%p slot[%d] %p -> %p (our_nav_cb)",
         cb_list, static_cast<int>(sdk_layout::NavigationCallbackSlot), prev,
         reinterpret_cast<void *>(&our_nav_cb));

    // Compile-time-ish guard against a future firmware that bumps
    // the table size. If the profiled user-context slot ever drifts past the end
    // of what we model, the touch path and the rest of the lib
    // also need rewiring — re-harvest the cb_list layout from
    // libaap_interface.so / blmjciaapa.so before adjusting.
    static_assert(sdk_layout::UserContextSlot < sdk_layout::CallbackTableWords,
                  "user-context slot out of cb_list bounds");
    static_assert(sdk_layout::CallbackTableWords * sizeof(void *) ==
                      sdk_layout::CallbackTableBytes,
                  "callback table size does not match the ARM32 profile");
}

// NavigationStatus (0x8003) enum (aasdk): UNAVAILABLE=0, ACTIVE=1, INACTIVE=2,
// REROUTING=3. INACTIVE/UNAVAILABLE -> blank the HUD; ACTIVE/REROUTING -> keep.
enum { NAV_UNAVAILABLE = 0, NAV_ACTIVE = 1, NAV_INACTIVE = 2, NAV_REROUTING = 3 };

// Quantize the display distance (value*10) to its natural on-HUD granularity, so
// a frame is only re-sent when the SHOWN figure would change — not on every
// position tick. mi/km are already 0.1-granular; m/yd/ft step by magnitude.
int32_t quantize_dist_x10(int32_t v, uint8_t mazda_unit)
{
    if (v < 0) return 0;
    if (mazda_unit == 2 || mazda_unit == 3) return v;        // mi/km: 0.1 already
    if (v >= 5000) return (v + 250) / 500 * 500;             // far  -> 50-unit steps
    if (v >= 1000) return (v +  50) / 100 * 100;             // mid  -> 10-unit steps
    if (v >=  200) return (v +  25) /  50 *  50;             // near ->  5-unit steps
    return v;                                                // final approach -> as-is
}

// Accumulated Mazda-domain guidance: maneuver/road/lanes come from 0x8006,
// distance from 0x8007, so the two streams are merged here before display. The
// memcmp change-gate below is only a change *detector*, not a correctness
// mechanism: it returns non-zero whenever a real field differs, so a stray
// padding byte could at worst cause one extra, harmless HUD re-emit — never a
// missed change. (In practice it won't even do that: both instances are
// zero-init statics and nothing writes padding.) So exact/gap-free layout is
// not required — no manual padding, no size assert.
struct AaNav16HudState {
    uint32_t glyph;                       // Mazda HUD maneuver glyph (MazdaIcon; 0=blank)
    int32_t  dist_dec;                    // display distance * 10
    uint8_t  dist_unit;                   // Mazda HUD unit 0..5
    uint8_t  n_lanes;                     // 0..HUD_NAV16_MAX_LANES
    char     road[64];                    // street name (UTF-8, folded at 0x8006 ingest)
    AaLane   lanes[HUD_NAV16_MAX_LANES];
};

static AaNav16HudState g_nav16_acc;   // merged guidance (zero-init: gap-free memcmp)
static AaNav16HudState g_nav16_last;  // last frame fed to the transport (change-gate)
static bool g_nav16_have_last = false;

// Reset the accumulator + change-gate. The initial call happens before the rx
// thread exists; later session-boundary calls hold g_nav_writer_mu, the same
// lock used by every sink callback. Without this, a session that ends
// abnormally (cable pull — the phone never sends INACTIVE/STOP) leaves a stale
// change-gate that can suppress the next session's first frame against an
// already-blanked HUD.
void hud_feed_nav16_reset(void)
{
    memset(&g_nav16_acc, 0, sizeof(g_nav16_acc));
    g_nav16_have_last = false;
    LOGV("nav: accumulator + change-gate reset");
}

// Change-gate emit: publish one complete frame only when the displayed state
// changed. This prevents a transient maneuver/distance/lane mixture and leaves
// arbitration, D-Bus output, reconnect, and refresh entirely to the broker.
static void nav16_emit_if_changed()
{
    AaNav16HudState &acc = g_nav16_acc;
    if (g_nav16_have_last && memcmp(&acc, &g_nav16_last, sizeof(acc)) == 0) return;

    if (acc.glyph == 0 || acc.glyph > 60) {
        guidance_release(guidance::RELEASE_UNSUPPORTED_MANEUVER);
        g_nav16_last = acc;
        g_nav16_have_last = true;
        return;
    }

    memset(&g_guidance, 0, sizeof(g_guidance));
    g_guidance.glyph = acc.glyph;
    g_guidance.distance_x10 = acc.dist_dec;
    g_guidance.distance_unit = acc.dist_unit;
    copy_street_bounded(acc.road, strnlen(acc.road, sizeof(acc.road)),
                        g_guidance.street, sizeof(g_guidance.street));
    nav16_encode_lane_codes(acc.lanes, acc.n_lanes, g_guidance.lanes);
    guidance_publish_if_changed();
    g_nav16_last      = acc;
    g_nav16_have_last = true;
}

// hud_nav16 sink callbacks (run on the rx thread). hud_nav16 has already decoded
// the AA protocol; these only map the decoded structs to the Mazda HUD domain
// (glyph, unit, fold, change-gate) and drive the shared transport.

// 0x8006 NavigationState -> maneuver glyph + road + lanes.
static void nav16_on_guidance(const AaGuidance *g)
{
#if LOG_LEVEL <= LOG_LEVEL_VERBOSE
    char line[320]; hud_nav16_format_guidance(g, line, sizeof(line)); LOGV("%s", line);
#endif
    std::lock_guard<std::mutex> writer_lk(g_nav_writer_mu);
    if (!g_aap_session_active) return;

    // Only a useful 1.6-exclusive frame may latch the legacy callback path off.
    // NavigationStatus is shared with GAL 1.5, and a malformed/empty 0x8006 must
    // not make the real 1.5 0x501/0x502 callbacks disappear for the session.
    if (!g || !g->have_maneuver) return;
    if (!hud_nav16_rx_seen()) {
        // Discard any partial legacy 1.5 snapshot on the exact edge where a
        // useful 1.6-exclusive frame proves the richer protocol is live.
        guidance_reset();
        hud_feed_nav16_reset();
    }
    hud_nav16_rx_mark_seen();

    AaNav16HudState &acc = g_nav16_acc;

    uint32_t glyph = hud_nav16_glyph(g);
    if (glyph > 60) glyph = 0;                          // clamp untrusted glyph

    // Fold once here, at road ingest, not in the per-emit forwarder — distance
    // ticks re-emit the road far more often than 0x8006 changes it. Fold a local
    // copy (g is const, decoder-owned); strncpy into acc.road zero-pads the tail
    // so a shorter road leaves no stale bytes for the memcmp change-gate to trip
    // on (at worst a harmless extra emit, but free to avoid).
    char road[sizeof(acc.road)];
    libpatch::copy_utf8_truncated(road, sizeof(road), g->road, sizeof(g->road));
    if (libpatch_config::hud_fold_latin()) hud_translit::fold(road);

    // A different maneuver means the held distance belongs to the PREVIOUS step
    // (distance only ever arrives on 0x8007) — invalidate it so the new arrow is
    // never shown against the old step's ticking-down figure. 0 + unit 0 ("none")
    // is the same not-yet-any-distance state every session starts in; the real
    // figure lands with the next 0x8007 (~1 s).
    if (glyph != acc.glyph || strcmp(road, acc.road) != 0) {
        acc.dist_dec  = 0;
        acc.dist_unit = 0;
    }
    acc.glyph = glyph;
    strncpy(acc.road, road, sizeof(acc.road) - 1);
    acc.road[sizeof(acc.road) - 1] = '\0';
    int nl = g->n_lanes;
    if (nl < 0) nl = 0;
    if (nl > HUD_NAV16_MAX_LANES) nl = HUD_NAV16_MAX_LANES;
    acc.n_lanes = (uint8_t)nl;
    for (int i = 0; i < HUD_NAV16_MAX_LANES; ++i) {
        acc.lanes[i].present_mask   = (i < nl) ? g->lanes[i].present_mask   : 0;
        acc.lanes[i].highlight_mask = (i < nl) ? g->lanes[i].highlight_mask : 0;
    }
    nav16_emit_if_changed();
}

// 0x8007 NavigationCurrentPosition -> distance to next maneuver.
static void nav16_on_position(const AaPosition *p)
{
#if LOG_LEVEL <= LOG_LEVEL_VERBOSE
    char line[320]; hud_nav16_format_position(p, line, sizeof(line)); LOGV("%s", line);
#endif
    std::lock_guard<std::mutex> writer_lk(g_nav_writer_mu);
    if (!g_aap_session_active) return;
    if (!p || !p->have_step) return;
    if (!hud_nav16_rx_seen()) {
        guidance_reset();
        hud_feed_nav16_reset();
    }
    hud_nav16_rx_mark_seen();
    uint8_t unit = aa_to_mazda_unit(p->step_units);
    if (unit > 5) unit = 0;                             // clamp untrusted unit
    g_nav16_acc.dist_unit = unit;
    g_nav16_acc.dist_dec  = quantize_dist_x10(parse_dist_x10(p->step_display), unit);
    nav16_emit_if_changed();
}

// 0x8003 NavigationStatus / 0x8002 cluster STOP -> lifecycle: blank or keep.
static void nav16_on_status(const AaStatus *s)
{
    std::lock_guard<std::mutex> writer_lk(g_nav_writer_mu);
    if (!g_aap_session_active) return;
    if (!s) return;
    // 0x8003 NavigationStatus is shared by GAL 1.5 and 1.6, so it cannot
    // identify the negotiated generation. The seen latch is set only by an
    // admitted 0x8006/0x8007 callback above.
    if (s->cluster_stop) {
        guidance_release(guidance::RELEASE_ROUTE_STOPPED);
        hud_feed_nav16_reset();
        LOGD("nav: cluster STOP (0x8002) -> HUD cleared");
        return;
    }
    if (s->nav_status == NAV_INACTIVE || s->nav_status == NAV_UNAVAILABLE) {
        guidance_release(guidance::RELEASE_ROUTE_STOPPED);
        hud_feed_nav16_reset();
        LOGD("nav: NavigationStatus=%d -> HUD cleared", s->nav_status);
    } else if (s->nav_status == NAV_ACTIVE || s->nav_status == NAV_REROUTING) {
        guidance_claim();
        LOGV("nav: NavigationStatus=%d (active/rerouting) -> HUD kept", s->nav_status);
    } else {
        LOGW("nav: NavigationStatus=%d unknown -> admission unchanged", s->nav_status);
    }
}

void hud_post_aap_create_session(void)
{
    std::lock_guard<std::mutex> writer_lk(g_nav_writer_mu);
    // Creating the reusable SDK object does not establish AA ownership. Keep
    // the per-device producers closed until start succeeds; the shared broker
    // itself remains process-long so it can serve CarPlay while AA is detached.
    g_aap_session_active = false;
}

static void end_aap_guidance_session()
{
    uint64_t ending_epoch = 0;
    {
        std::lock_guard<std::mutex> writer_lk(g_nav_writer_mu);
        g_aap_session_active = false;
        ending_epoch = g_guidance_epoch;
    }

    if (libpatch_config::use_protocol_v1_6()) {
        // Do not join while holding g_nav_writer_mu: a receiver callback already
        // queued on that mutex must observe inactive and return before it exits.
        hud_nav16_rx_stop();
    }

    {
        std::lock_guard<std::mutex> writer_lk(g_nav_writer_mu);
        // A duplicate/stale lifecycle edge must not erase a newer epoch. The
        // OEM serializes these hooks, but retain the identity check explicitly.
        if (g_guidance_epoch == ending_epoch) {
            g_guidance_epoch = 0;
            guidance_reset();
            hud_feed_nav16_reset();
        }
    }

    // End only after the raw GAL receiver is joined, so no delayed AA publish
    // can follow this release and clear or steal a later CarPlay owner.
    if (ending_epoch != 0) guidance::client_session_end(ending_epoch);
}

void hud_post_aap_start_session(void)
{
    // Normally stop precedes a subsequent start. Closing here as well makes a
    // duplicate successful start converge to a fresh epoch instead of reusing
    // a receiver/change-gate from the prior device.
    end_aap_guidance_session();

    std::lock_guard<std::mutex> writer_lk(g_nav_writer_mu);
    guidance_reset();
    hud_feed_nav16_reset();
    g_guidance_epoch = guidance::client_session_begin();
    if (libpatch_config::use_protocol_v1_6()) {
        // rx_start resets the decoder/change-gate and seen latch before spawning
        // the receiver. Its callbacks take g_nav_writer_mu, so the new thread
        // cannot publish until this hook opens admission below.
        hud_nav16_rx_start(&nav16_on_guidance, &nav16_on_position, &nav16_on_status);
    }
    g_aap_session_active = true;
}

void hud_post_aap_stop_session(void)
{
    end_aap_guidance_session();
}

void hud_pre_aap_destroy_session(void)
{
    end_aap_guidance_session();
}
