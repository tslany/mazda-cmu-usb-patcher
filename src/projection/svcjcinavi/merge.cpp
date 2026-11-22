// SPDX-License-Identifier: AGPL-3.0-or-later
//
// svcjcinavi HUD merge hook.
//
// Interposes the two OEM HUD setters that svcjcinavi calls on its
// sender thread (thUpdateGuidanceChangeToHUD):
//
//   VBS_NAVI_SetHUDDisplayMsgReq      — the (uqyqyy) maneuver frame
//   VBS_NAVI_TMC_SetHUD_Display_Msg2  — the street-name strip
//
// Both are UND (PLT imports from libjcivbsnaviclient.so) in
// svcjcinavi.so, so an LD_PRELOAD definition at the front of the
// process's global scope binds svcjcinavi's calls to ours. We chain
// through to the real implementations after rewriting.
//
// === The problem this solves =================================
//
// With the nav SD card inserted, the OEM nav engine (NNG) emits
// GuidanceChangedForHUD frames even with no active route — its TSR
// (speed-limit) frames carry a real speed but a BLANK maneuver
// (dirIcon=0). The shared AA/CarPlay broker emits its own
// GuidanceChangedForHUD frames
// carrying a real maneuver and the reserved sentinel speed
// (kProjectionSpeedSentinel = 0xFFFF). Both
// streams converge on svcjcinavi's single sender thread and alternate
// at the HUD: projection maneuver, then NNG's blank-maneuver speed frame,
// then projection maneuver, ... — the maneuver arrow blinks (flicker).
//
// === What we do ==============================================
//
// We make every frame the HUD receives carry the SAME content
// (projection maneuver + OEM speed), so re-asserting it at the union of both
// cadences is flicker-free (re-rendering identical content is
// invisible). We discriminate the two streams by the sentinel speed
// value and keep a little state (all touched only from the single
// sender thread, so no locking):
//
//   * Projection-origin frame (speed == sentinel): if it carries a real
//     maneuver, remember its maneuver block and (re)arm the projection-active
//     window; if it is EMPTY (maneuver == 0, the broker's blanking frame),
//     clear projection ownership so OEM frames resume passing through
//     natively. Either way splice the
//     remembered OEM speed in and forward.
//   * OEM-origin frame (speed != sentinel): remember its speed, then —
//     while projection is active — overwrite its (blank) maneuver block with
//     the remembered projection maneuver and forward. NNG's frame now carries
//     the projected turn + its own speed, identical to the broker's frames.
//     We do
//     NOT touch text_ID3: svcjcinavi computes the sync bit for every
//     frame and stamps the same value into both the maneuver frame and
//     its paired street strip, so the value we see is already correct.
//   * Street strip (Msg2): svcjcinavi emits it back-to-back right after
//     the maneuver frame (same critical section, same generation), so
//     the strip's origin is whatever the maneuver frame just before it
//     was. We mirror the maneuver handling: on a projection-origin generation
//     we CAPTURE the street into our own buffer (read from the OEM's
//     un-blanked current_StreetName, so a market that blanks the
//     outbound strip — e.g. EU — still yields the real street; a null/
//     empty street is stored as the empty string) and re-point the strip
//     at our copy; on an OEM-origin generation while projection owns the
//     display we REPLACE the (blank NNG) street with the captured projected
//     street and pass it, so the street is re-asserted identically at both
//     cadences — flicker-free, same as the maneuver. Projection-idle strips
//     pass untouched. We only ever rewrite the street STRING, never the
//     strip's syncBit: svcjcinavi stamps the same per-generation sync
//     into both the maneuver frame's text_ID3 and this strip's syncBit
//     in one critical section, and that pairing is what tells the HUD
//     the two updates go together. We leave both untouched, so the
//     spliced street rides on the current generation's sync and still
//     matches the (also-spliced) maneuver frame.
//
// On FW74 svcjcinavi itself is started even when the navigation SD card is
// absent; only the NNG producer stops. Projection frames therefore still pass
// through this process cardless. If projection is not active, any OEM frames
// that do arrive pass through untouched (native behaviour).

#define LOG_TAG "MERGE"
#include "log.h"
#include "../common/config.h"
#include "../common/preload.h"
#include "../common/string_safe.h"
#include "../common/oem/vbs_navi_hud.h"
#include "projection.h"

#include <dlfcn.h>
#include <time.h>
#include <string.h>
#include <stdint.h>
#include <sys/syscall.h>
#include <unistd.h>

// Forward declarations of our own exported PLT shadows, so ensure_gate()
// (below) can take their addresses for the resolve_real_symbol self-loop
// guard. Defined at file scope further down.
extern "C" int VBS_NAVI_SetHUDDisplayMsgReq(void *, VbsNaviHudDisplay *,
                                            void *, void *, void *);
extern "C" int VBS_NAVI_TMC_SetHUD_Display_Msg2(void *, VbsNaviHudMsg2 *,
                                                void *, void *, void *);

namespace {

// Self-gate: only act if we are actually inside the navigation
// service PID. dlopen(RTLD_NOLOAD) returns non-NULL iff svcjcinavi.so
// is already mapped in this process.
constexpr const char *kSvcjcinaviSo = "/jci/navi/svcjcinavi.so";

// Where the real HUD setters live (libjcivbsnaviclient.so is a
// DT_NEEDED of svcjcinavi.so).
constexpr const char *kVbsClientSoname  = "libjcivbsnaviclient.so";
constexpr const char *kVbsClientAbspath = "/jci/lib/libjcivbsnaviclient.so";

// --- OEM internal reached by anchor + fixed offset ----------------
//
// svcjcinavi blanks the OUTBOUND street strip in some markets: for HUD
// types 1/2 in certain region/language combinations (notably EU) it
// repoints the strip to a single space instead of the real street. The
// street it actually received still lives in the OEM's own
// current_StreetName buffer — only the outbound pointer is blanked. We
// read that buffer so the projected street survives the blanking.
//
// current_StreetName is NOT a dynamic symbol (dlsym can't see it), but
// svcjcinavi.so ships with its full .symtab, so we compute its address
// from the one dynamically-exported anchor (GetServiceInterfaces) plus a
// fixed file offset, exactly like the blmjciaapa anchor-and-offset
// thunks. File offsets are FW 74.00.324A, harvested with nm; the
// svcjcinavi.so binary is byte-identical across this version's NA and EU
// builds (verified by sha256), so one set of offsets serves both
// markets. Re-harvest with `nm -a` after any OEM update.
namespace addr = fw74_00_324a::svcjcinavi;

// Projected guidance is event-driven, so we do NOT key activity off a feed
// cadence. Ownership is set explicitly: a real broker frame arms it and an
// empty broker frame clears it. This timeout is only a safety net for the case
// where the broker dies without sending its blanking frame — long
// enough never to expire mid-drive, short enough to eventually hand
// the HUD back to OEM nav. Use elapsed time, not the GPS/user-adjustable clock.
constexpr time_t kProjectionTimeoutSec = 5 * 60;

time_t monotonic_seconds()
{
    struct timespec now;
    if (syscall(SYS_clock_gettime, CLOCK_MONOTONIC, &now) != 0) return 0;
    return now.tv_sec;
}

typedef int (*SetFn)(void *, VbsNaviHudDisplay *, void *, void *, void *);
typedef int (*Msg2Fn)(void *, VbsNaviHudMsg2 *, void *, void *, void *);

bool   g_gate_done = false;
bool   g_enabled   = false;

void  *g_vbs_handle = nullptr;
SetFn  g_real_set   = nullptr;
Msg2Fn g_real_msg2  = nullptr;

// The OEM's un-blanked street buffer (current_StreetName), resolved once
// via the anchor in ensure_gate(). nullptr if we are not in the
// svcjcinavi PID or the anchor failed to resolve, in which case the
// Msg2 hook falls back to the (possibly blanked) strip pointer.
const char *g_cur_street = nullptr;

// Merge state — all accessed only from svcjcinavi's single sender
// thread (thUpdateGuidanceChangeToHUD), so plain variables are safe.
bool     g_have_projection  = false;
uint32_t g_projection_man   = 0;
uint16_t g_projection_dist  = 0;
uint8_t  g_projection_dunit = 0;
time_t   g_projection_last  = 0;

// What the street-strip hook should do with the strip svcjcinavi emits
// back-to-back right after each maneuver frame. Set by the maneuver
// hook (which knows the generation's origin), consumed by the Msg2
// hook. Re-set on every maneuver frame, so never stale.
//   CAPTURE — projection-origin strip: copy its street and pass.
//   REPLACE — OEM-origin strip while projection is active: overwrite its
//             street with the captured projected street and pass.
//   PASSTHROUGH — projection idle (or a broker blank): pass untouched.
enum StreetAction { STREET_PASSTHROUGH, STREET_CAPTURE, STREET_REPLACE };
StreetAction g_street_action = STREET_PASSTHROUGH;

// Captured projected street name (from the projection-origin Msg2). The real setter
// copies the string synchronously, so a plain buffer pointed at for the
// duration of the forwarded call is safe. A null projected street (its "no
// street" request) is stored as the empty string, which the OEM setter
// marshals as a blank street line.
char g_projection_street[128] = { 0 };

// Last OEM speed seen. Initialised to 0 — the value OEM nav itself
// sends when there is no speed limit — so projection frames carry a "no speed
// limit" until a real OEM speed has arrived. Safer than the sentinel:
// 0 is a genuine OEM value, whereas the sentinel is reserved for the
// projection-origin discriminator and must never appear on a forwarded frame.
uint16_t g_oem_speed  = 0;
uint8_t  g_oem_sunit  = 0;

void ensure_gate()
{
    if (g_gate_done) {
        return;
    }
    g_gate_done = true;

    // Load the fixed release settings before resolving the unblanked street.
    libpatch_config::load(reinterpret_cast<const void *>(&ensure_gate));

    void *h = dlopen(kSvcjcinaviSo, RTLD_NOW | RTLD_NOLOAD);
    g_enabled = (h != nullptr);
    if (g_enabled) {
        // Resolve the OEM's un-blanked street buffer (current_StreetName)
        // ONLY when force_street_name is set. Left null, the Msg2 hook
        // captures from the strip pointer (native OEM behaviour: markets
        // that blank the street keep blanking it). Load bias =
        // runtime(anchor) - file_offset(anchor); the library stays mapped
        // for the PID's lifetime, so the dlclose below only drops our
        // extra NOLOAD refcount — the computed address stays valid.
        if (libpatch_config::force_street_name()) {
            void *anchor = dlsym(h, addr::AnchorSymbol);
            if (anchor != nullptr) {
                uintptr_t base =
                    reinterpret_cast<uintptr_t>(anchor) - addr::Anchor;
                g_cur_street = reinterpret_cast<const char *>(
                    base + addr::CurrentStreetName);
                LOGD("force_street_name: resolved current_StreetName=%p "
                     "(base=%p)",
                     reinterpret_cast<const void *>(g_cur_street),
                     reinterpret_cast<void *>(base));
            } else {
                LOGW("force_street_name: anchor %s unresolved — falling "
                     "back to the (possibly blanked) Msg2 strip",
                     addr::AnchorSymbol);
            }
        } else {
            LOGD("force_street_name=false — leaving OEM street handling "
                 "intact (strip)");
        }
        // We only need the boolean "is it mapped" plus the anchor; drop
        // the extra refcount RTLD_NOLOAD took so we don't leak it.
        dlclose(h);
        LOGD("self-gate: enabled (svcjcinavi.so mapped)");
    } else {
        LOGW("self-gate: svcjcinavi.so not mapped in this pid — "
             "merge disabled, transparent passthrough");
    }

    // Resolve both real HUD setters UNCONDITIONALLY — even in the wrong
    // process. g_enabled gates only the rewriting; resolution must not
    // be gated, because the disabled path still forwards to the real
    // implementation (transparent passthrough). If we skipped resolution
    // when disabled, a wrong-process call would have no real impl to
    // forward to and would be dropped instead of passed through.
    g_real_set = reinterpret_cast<SetFn>(resolve_real_symbol(
        "VBS_NAVI_SetHUDDisplayMsgReq", kVbsClientSoname, kVbsClientAbspath,
        reinterpret_cast<void *>(&VBS_NAVI_SetHUDDisplayMsgReq),
        &g_vbs_handle));
    if (g_real_set == nullptr) {
        LOGC("could not resolve real VBS_NAVI_SetHUDDisplayMsgReq — "
             "frames will be dropped this session");
    }

    g_real_msg2 = reinterpret_cast<Msg2Fn>(resolve_real_symbol(
        "VBS_NAVI_TMC_SetHUD_Display_Msg2", kVbsClientSoname, kVbsClientAbspath,
        reinterpret_cast<void *>(&VBS_NAVI_TMC_SetHUD_Display_Msg2),
        &g_vbs_handle));
    if (g_real_msg2 == nullptr) {
        LOGC("could not resolve real VBS_NAVI_TMC_SetHUD_Display_Msg2");
    }
}

bool projection_active()
{
    return g_have_projection &&
           (monotonic_seconds() - g_projection_last) <= kProjectionTimeoutSec;
}

} // namespace

// Exported PLT shadows. Default visibility so the loader binds
// svcjcinavi.so's imports to these.

extern "C" PRELOAD_EXPORT
int VBS_NAVI_SetHUDDisplayMsgReq(void *conn, VbsNaviHudDisplay *disp,
                                 void *unused, void *cb, void *user)
{
    ensure_gate();

    // Merge disabled (wrong process), unresolved real impl, or NULL
    // frame: transparent passthrough — forward untouched to the real
    // setter (or return 0 only if it genuinely could not be resolved).
    // Tag the next strip PASSTHROUGH so it can't act on a prior
    // generation's action if we bail before classifying this one.
    if (!g_enabled || g_real_set == nullptr || disp == nullptr) {
        g_street_action = STREET_PASSTHROUGH;
        return g_real_set ? g_real_set(conn, disp, unused, cb, user) : 0;
    }

    // FW74 passes its cached frame, not a disposable copy. Preserve its origin
    // sentinel and native maneuver so retries cannot feed our merged output back in.
    VbsNaviHudDisplay merged = *disp;
    disp = &merged;

    if (disp->displaySpeedLimit == kProjectionSpeedSentinel) {
        if (disp->nextManeuverInfo == 0) {
            // Empty projection frame: guidance has stopped and wants its
            // maneuver cleared. Relinquish ownership so subsequent OEM
            // frames pass through natively again. Let this blank frame
            // (and its blank street strip) through so the HUD clears.
            g_have_projection  = false;
            g_street_action    = STREET_PASSTHROUGH;
            LOGV("projection empty frame (man=0): releasing ownership");
        } else {
            // Real projected guidance: capture the maneuver block and (re)arm
            // the activity window.
            g_projection_man   = disp->nextManeuverInfo;
            g_projection_dist  = disp->distanceValue;
            g_projection_dunit = disp->distanceUnit;
            g_have_projection  = true;
            g_projection_last  = monotonic_seconds();

            // The strip that follows is the projected street: capture it.
            g_street_action = STREET_CAPTURE;

            LOGV("projection frame: man=%u dist=%u  splice speed=0x%x "
                 "unit=%u",
                 static_cast<unsigned>(g_projection_man),
                 static_cast<unsigned>(g_projection_dist),
                 static_cast<unsigned>(g_oem_speed),
                 static_cast<unsigned>(g_oem_sunit));
        }

        // Splice the remembered OEM speed into the projection frame either
        // way.
        disp->displaySpeedLimit = g_oem_speed;
        disp->displaySpeedUnit  = g_oem_sunit;
    } else {
        // OEM-origin frame: remember its speed; while projection is active,
        // overwrite its (blank) maneuver with the projected one so it stops
        // blanking and becomes an identical re-assertion. text_ID3 is
        // left as svcjcinavi computed it.
        g_oem_speed = disp->displaySpeedLimit;
        g_oem_sunit = disp->displaySpeedUnit;

        if (projection_active()) {
            disp->nextManeuverInfo = g_projection_man;
            disp->distanceValue    = g_projection_dist;
            disp->distanceUnit     = g_projection_dunit;
            // Overwrite the OEM strip that follows with the projected street.
            g_street_action = STREET_REPLACE;
            LOGV("OEM frame: speed=0x%x unit=%u  spliced projection man=%u "
                 "dist=%u",
                 static_cast<unsigned>(g_oem_speed),
                 static_cast<unsigned>(g_oem_sunit),
                 static_cast<unsigned>(g_projection_man),
                 static_cast<unsigned>(g_projection_dist));
        } else {
            g_street_action = STREET_PASSTHROUGH;
            LOGV("OEM frame (projection idle): speed=0x%x unit=%u — passthrough",
                 static_cast<unsigned>(g_oem_speed), static_cast<unsigned>(g_oem_sunit));
        }
    }

    return g_real_set(conn, disp, unused, cb, user);
}

extern "C" PRELOAD_EXPORT
int VBS_NAVI_TMC_SetHUD_Display_Msg2(void *conn, VbsNaviHudMsg2 *msg2,
                                     void *unused, void *cb, void *user)
{
    ensure_gate();

    // Merge disabled (wrong process), unresolved real impl, or NULL
    // frame: transparent passthrough — forward untouched to the real
    // setter (or return 0 only if it genuinely could not be resolved).
    if (!g_enabled || g_real_msg2 == nullptr || msg2 == nullptr) {
        return g_real_msg2 ? g_real_msg2(conn, msg2, unused, cb, user) : 0;
    }

    VbsNaviHudMsg2 merged = *msg2;
    msg2 = &merged;

    // svcjcinavi emits the street strip back-to-back right after its
    // maneuver frame, so the maneuver hook has tagged this strip's
    // origin. Mirror the maneuver handling: capture the projected street, or
    // overwrite the OEM strip with it. We rewrite only the street
    // STRING — never syncBit: it is the current generation's sync,
    // matching the maneuver frame's text_ID3, and that pairing is how
    // the HUD knows the maneuver and street belong together.
    if (g_street_action == STREET_CAPTURE) {
        // Record the projected street. Prefer the OEM's un-blanked
        // current_StreetName (g_cur_street): in markets that blank the
        // outbound strip (e.g. EU, HUD type 1/2) the strip pointer here
        // is already a single space, but current_StreetName still holds
        // the real street svcjcinavi received. This hook runs on the
        // sender thread while it holds the guidance mutex that also
        // guards current_StreetName, so the read is consistent with the
        // frame being sent. Fall back to the strip pointer if the anchor
        // didn't resolve. A null/empty street is stored as "".
        const char *src =
            g_cur_street ? g_cur_street : msg2->guidancePointName;
        libpatch::copy_utf8_truncated(g_projection_street,
            sizeof(g_projection_street), src,
            src ? strnlen(src, sizeof(g_projection_street) - 1) : 0);
        // Re-point this projection-origin strip at our captured copy so the
        // frame itself shows the real street rather than the market
        // blank — identical to the OEM-cadence REPLACE below, so the two
        // cadences stay flicker-free.
        msg2->guidancePointName = g_projection_street;
        LOGV("Msg2 capture: projection street \"%s\"%s", g_projection_street,
             g_cur_street ? " (current_StreetName)" : " (strip)");
    } else if (g_street_action == STREET_REPLACE) {
        msg2->guidancePointName = g_projection_street;
        LOGV("Msg2 replace: OEM strip -> projection street \"%s\"",
             g_projection_street);
    }

    return g_real_msg2(conn, msg2, unused, cb, user);
}
