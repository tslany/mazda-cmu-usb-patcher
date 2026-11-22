// SPDX-License-Identifier: AGPL-3.0-or-later

#ifndef MAZDA_CMU_GUIDANCE_WIRE_H
#define MAZDA_CMU_GUIDANCE_WIRE_H

#include "model.h"

#include <stddef.h>
#include <stdint.h>

namespace mazda_cmu {
namespace guidance {
namespace wire {

static const char kSocketName[] = "mazda-cmu-guidance-v1";
static const uint32_t kMagic = 0x4d434750u; // "MCGP"
static const uint16_t kVersion = 1;

enum MessageType : uint8_t {
    MESSAGE_HELLO = 1,
    MESSAGE_SESSION_BEGIN = 2,
    MESSAGE_CLAIM = 3,
    MESSAGE_SNAPSHOT = 4,
    MESSAGE_RELEASE = 5,
    MESSAGE_SESSION_END = 6,
    MESSAGE_HEARTBEAT = 7,
};

// Same-host ARM32 ABI. Every field is explicit-width and every reserved byte
// must be zero. Both ends also require an exact message size for the type.
struct Header {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint8_t type;
    uint8_t source;
    uint16_t code;       // ReleaseReason for RELEASE; zero otherwise.
    uint32_t sequence;
    uint64_t instance;   // Per-process client incarnation.
    uint64_t epoch;      // Per-device/session generation; zero only for HELLO.
};

struct ControlMessage {
    Header header;
};

struct SnapshotMessage {
    Header header;
    Snapshot snapshot;
};

static_assert(offsetof(Header, instance) == 16,
              "guidance wire Header layout drifted");
static_assert(sizeof(Header) == 32, "guidance wire Header must be 32 bytes");
static_assert(sizeof(ControlMessage) == sizeof(Header),
              "guidance wire control message has padding");
static_assert(offsetof(SnapshotMessage, snapshot) == sizeof(Header),
              "guidance wire snapshot offset drifted");
static_assert(sizeof(SnapshotMessage) ==
                  sizeof(Header) + sizeof(Snapshot),
              "guidance wire snapshot message has padding");
static_assert(sizeof(SnapshotMessage) == 312,
              "guidance wire snapshot message must be 312 bytes");

} // namespace wire
} // namespace guidance
} // namespace mazda_cmu

#endif // MAZDA_CMU_GUIDANCE_WIRE_H
