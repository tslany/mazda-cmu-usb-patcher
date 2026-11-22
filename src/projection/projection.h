// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Firmware-bound runtime addresses used by the projection shims.
// All values target Mazda Connect 74.00.324A. Re-harvest and validate
// this profile before enabling another firmware version.

#ifndef MAZDA_CMU_PROFILE_FW74_00_324A_PROJECTION_H
#define MAZDA_CMU_PROFILE_FW74_00_324A_PROJECTION_H

#include <stddef.h>
#include <stdint.h>

namespace fw74_00_324a {

namespace abi {
constexpr size_t WordBytes = 4;
constexpr size_t PointerBytes = 4;
} // namespace abi

static_assert(sizeof(uintptr_t) == abi::WordBytes,
              "FW74 projection profile requires 32-bit uintptr_t");
static_assert(sizeof(void *) == abi::PointerBytes,
              "FW74 projection profile requires 32-bit pointers");
static_assert(sizeof(long) == abi::WordBytes,
              "FW74 projection profile requires the ARM32 long ABI");

// libaap_interface / blmjciaapa callback and event-record layouts.
namespace aap_sdk {
constexpr size_t CallbackTableWords = 19;
constexpr size_t CallbackTableBytes = CallbackTableWords * abi::WordBytes;
constexpr size_t ProjectionStatusCallbackSlot = 3;
constexpr size_t NavigationCallbackSlot = 10;
constexpr size_t UserContextSlot = 18;

constexpr size_t TouchMaxFingers = 10;
constexpr size_t TouchEventBytes = 0x88;
constexpr size_t TouchDeviceTypeOffset = 0x00;
constexpr size_t TouchFingerCountOffset = 0x04;
constexpr size_t TouchXOffset = 0x08;
constexpr size_t TouchYOffset = 0x30;
constexpr size_t TouchPointerIdOffset = 0x58;
constexpr size_t TouchActionOffset = 0x80;
constexpr size_t TouchActionIndexOffset = 0x84;
constexpr uint32_t TouchDisplayWidth = 800;
constexpr uint32_t TouchDisplayHeight = 480;
constexpr uint32_t TouchRawCoordinateDivisor = 4099;

constexpr size_t KeyEventBytes = 0x0c;
constexpr size_t KeyEventTypeOffset = 0x00;
constexpr size_t KeyEventCodeOffset = 0x04;
constexpr size_t KeyEventDownOffset = 0x08;

constexpr size_t LegacyNavEventBytes = 36;
constexpr size_t NextTurnTagOffset = 0x00;
constexpr size_t NextTurnRoadNameOffset = 0x04;
constexpr size_t NextTurnRoadNameLengthOffset = 0x08;
constexpr size_t NextTurnSideOffset = 0x0c;
constexpr size_t NextTurnEventOffset = 0x10;
constexpr size_t NextTurnImageOffset = 0x14;
constexpr size_t NextTurnImageLengthOffset = 0x18;
constexpr size_t NextTurnAngleOffset = 0x1c;
constexpr size_t NextTurnNumberOffset = 0x20;

constexpr size_t StatusTagOffset = 0x00;
constexpr size_t StatusValueOffset = 0x04;
constexpr size_t StatusReservedOffset = 0x08;

constexpr size_t DistanceTagOffset = 0x00;
constexpr size_t DistanceMetersOffset = 0x04;
constexpr size_t DistanceTimeOffset = 0x08;
constexpr size_t DistanceDisplayOffset = 0x0c;
constexpr size_t DistanceDisplayUnitOffset = 0x10;
constexpr size_t DistanceReservedOffset = 0x14;

// SerialID_AA startup restoration table on FW74.
constexpr size_t SavedDeviceRecords = 10;
} // namespace aap_sdk

namespace blmjciaapa {
constexpr uintptr_t GetServiceInterfaces = 0x00061768;
constexpr uintptr_t Singleton_AapProc_GetInstance = 0x00062670;
constexpr uintptr_t AapProc_GetVideoManager = 0x0007f664;
constexpr uintptr_t AapProc_GetRaceAap = 0x0007f614;
constexpr uintptr_t AapProc_GetAudioManager = 0x0007f63c;
constexpr uintptr_t AapProc_GetAapConnectionManager = 0x000872fc;
constexpr uintptr_t VideoManager_IsAAVideoInFocus = 0x000b3af4;
constexpr uintptr_t AudioManager_IsAAMediaInFocus = 0x000ac3b0;
constexpr uintptr_t AudioManager_IsAAMediaInPlaying = 0x000ac3f8;
constexpr uintptr_t RaceAap_SendTouchInput = 0x0008e3a8;
constexpr uintptr_t RaceAap_SendKeyInput = 0x0008e534;
constexpr uintptr_t AapConnectionManager_NotifyBtPairingResult = 0x0007c820;
constexpr uintptr_t AapConnectionManager_ActivateAapSession = 0x00078db4;

// AapConnectionManager instance fields, empirically verified on 74.00.324A.
constexpr uintptr_t AapConnectionManager_DevName = 0x54;
constexpr uintptr_t AapConnectionManager_ConnectMode = 0xdc;
} // namespace blmjciaapa

namespace aap_service {
constexpr uintptr_t VersionInsn = 0x00173eb4;
constexpr uint32_t VersionInsnOld = 0xe3a02005;
constexpr uint32_t VersionInsnNew = 0xe3a02006;
constexpr uintptr_t VtableSlot = 0x001f7d2c;
constexpr uintptr_t RouteMessage = 0x0017be10;
constexpr uintptr_t SharedPtrDeref = 0x0016f810;
constexpr uintptr_t IoBufferRaw = 0x0016f2f8;
constexpr uintptr_t IoBufferSize = 0x0016f330;
constexpr uintptr_t NavigationCallbacksDeref = 0x0017c5b0;
constexpr size_t NavigationCallbacksOffset = 0x28;
constexpr size_t NavigationStatusCallbackSlot = 2;
} // namespace aap_service

namespace svcjcinavi {
constexpr const char *AnchorSymbol = "GetServiceInterfaces";
constexpr uintptr_t Anchor = 0x00019008;
constexpr uintptr_t CurrentStreetName = 0x000aab98;
} // namespace svcjcinavi

// libjcivbsnaviclient's ARM32 records.
namespace vbs_navi {
constexpr size_t HudDisplayBytes = 12;
constexpr size_t HudManeuverOffset = 0;
constexpr size_t HudDistanceOffset = 4;
constexpr size_t HudDistanceUnitOffset = 6;
constexpr size_t HudSpeedOffset = 8;
constexpr size_t HudSpeedUnitOffset = 10;
constexpr size_t HudTextIdOffset = 11;

constexpr size_t HudStreetBytes = 8;
constexpr size_t HudStreetPointerOffset = 0;
constexpr size_t HudStreetSyncOffset = 4;

constexpr size_t LaneMappingEntries = 71;
} // namespace vbs_navi

// Decoded ipoddev records delivered to jciCARPLAY through devmgr on FW74.
namespace carplay_nav {
constexpr uint32_t ManeuverMessageType = 0x8059;
constexpr uint32_t GuidanceMessageType = 0x8058;
constexpr uint32_t MessageTypeMask = 0xffff;
constexpr size_t MessageTypeOffset = 0x00;

constexpr size_t ManeuverRecordBytes = 840;   // 0x348
constexpr size_t ValidInfoOffset = 0x1c;
constexpr size_t BurstIndexOffset = 0x20;
constexpr size_t ManeuverTextOffset = 0x24;
constexpr size_t ManeuverTextBytes = 256;
constexpr size_t ManeuverTypeOffset = 0x128;
constexpr size_t RoadNameOffset = 0x12c;
constexpr size_t RoadNameBytes = 256;
constexpr size_t DriveSideOffset = 0x33c;
constexpr size_t JunctionTypeOffset = 0x340;
constexpr size_t JunctionExitAngleOffset = 0x346;

constexpr size_t GuidanceRecordBytes = 1628; // 0x65c
constexpr size_t GuidanceDistanceOffset = 0x34c;
constexpr size_t GuidanceManeuverListOffset = 0x458;
constexpr size_t GuidanceManeuverCountOffset = 0x656;

static_assert(JunctionExitAngleOffset + sizeof(int16_t) <= ManeuverRecordBytes,
              "CarPlay maneuver offsets exceed the FW74 record");
static_assert(ManeuverTextOffset + ManeuverTextBytes <= ManeuverRecordBytes,
              "CarPlay maneuver text exceeds the FW74 record");
static_assert(RoadNameOffset + RoadNameBytes <= ManeuverRecordBytes,
              "CarPlay road-name field exceeds the FW74 record");
static_assert(GuidanceManeuverCountOffset + sizeof(uint16_t) <=
                  GuidanceRecordBytes,
              "CarPlay guidance offsets exceed the FW74 record");
} // namespace carplay_nav

} // namespace fw74_00_324a

#endif // MAZDA_CMU_PROFILE_FW74_00_324A_PROJECTION_H
