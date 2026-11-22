// Shared com.jci.vbs.navi HUD wire structs + the projection-frame sentinel.
//
// These are the on-wire layouts the OEM navigation HUD path uses
// (libjcivbsnaviclient marshals them onto com.jci.vbs.navi). Both the
// The common guidance sink and the svcjcinavi merge patch need the exact same
// definitions, so they live here.
//
// All offsets hand-derived from the Ghidra decompile
// (libjcivbsnaviclient / libjcimod_navigation). Re-verify after any
// OEM firmware update.

#ifndef LIBPATCH_COMMON_OEM_VBS_NAVI_HUD_H
#define LIBPATCH_COMMON_OEM_VBS_NAVI_HUD_H

#include "projection.h"

#include <stddef.h>
#include <stdint.h>

// 12-byte com.jci.vbs.navi HUD frame, signature (uqyqyy). Field
// offsets verified against VBS_NAVI_HUD_Display_s_t_pack:
// u32@0, u16@4, u8@6, u16@8, u8@10, u8@11 (sizeof == 12 with the
// natural pad byte at offset 7).
struct VbsNaviHudDisplay {
    uint32_t nextManeuverInfo;
    uint16_t distanceValue;
    uint8_t  distanceUnit;
    uint16_t displaySpeedLimit;
    uint8_t  displaySpeedUnit;
    uint8_t  text_ID3;
};

// 8-byte com.jci.vbs.navi.tmc street-name struct, signature (sy):
// char*@0, u8@4. The library transcodes UTF-8 -> UCS-2 and pages it
// internally; we just hand it a NUL-terminated C string.
struct VbsNaviHudMsg2 {
    const char *guidancePointName;
    uint8_t     syncBit;
};

static_assert(sizeof(VbsNaviHudDisplay) ==
                  fw74_00_324a::vbs_navi::HudDisplayBytes,
              "VbsNaviHudDisplay size does not match FW74");
static_assert(offsetof(VbsNaviHudDisplay, nextManeuverInfo) ==
                  fw74_00_324a::vbs_navi::HudManeuverOffset,
              "VbsNaviHudDisplay maneuver offset mismatch");
static_assert(offsetof(VbsNaviHudDisplay, distanceValue) ==
                  fw74_00_324a::vbs_navi::HudDistanceOffset,
              "VbsNaviHudDisplay distance offset mismatch");
static_assert(offsetof(VbsNaviHudDisplay, distanceUnit) ==
                  fw74_00_324a::vbs_navi::HudDistanceUnitOffset,
              "VbsNaviHudDisplay distance-unit offset mismatch");
static_assert(offsetof(VbsNaviHudDisplay, displaySpeedLimit) ==
                  fw74_00_324a::vbs_navi::HudSpeedOffset,
              "VbsNaviHudDisplay speed offset mismatch");
static_assert(offsetof(VbsNaviHudDisplay, displaySpeedUnit) ==
                  fw74_00_324a::vbs_navi::HudSpeedUnitOffset,
              "VbsNaviHudDisplay speed-unit offset mismatch");
static_assert(offsetof(VbsNaviHudDisplay, text_ID3) ==
                  fw74_00_324a::vbs_navi::HudTextIdOffset,
              "VbsNaviHudDisplay text ID offset mismatch");
static_assert(sizeof(VbsNaviHudMsg2) == fw74_00_324a::vbs_navi::HudStreetBytes,
              "VbsNaviHudMsg2 size does not match FW74");
static_assert(offsetof(VbsNaviHudMsg2, guidancePointName) ==
                  fw74_00_324a::vbs_navi::HudStreetPointerOffset,
              "VbsNaviHudMsg2 street pointer offset mismatch");
static_assert(offsetof(VbsNaviHudMsg2, syncBit) ==
                  fw74_00_324a::vbs_navi::HudStreetSyncOffset,
              "VbsNaviHudMsg2 sync offset mismatch");

// Reserved speedLimit value marking a HUD frame as projection-originated.
//
// The common broker sink stamps this into GuidanceChangedForHUD; svcjcinavi's
// merge patch keys on it to distinguish projection frames from the OEM nav
// engine's. 0xFFFF is the max
// uint16 and the HUD firmware's own "no speed limit" sentinel, so a
// stock (un-merged) svcjcinavi renders it as a blank speed rather
// than a bogus number — confirmed on-device.
//
// On the GuidanceChangedForHUD signal wire the speedLimit arg is
// int32; 65535 round-trips into this uint16 field unchanged.
static const uint16_t kProjectionSpeedSentinel = 0xFFFF;

#endif  // LIBPATCH_COMMON_OEM_VBS_NAVI_HUD_H
