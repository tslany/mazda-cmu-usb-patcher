// SPDX-License-Identifier: AGPL-3.0-or-later

#ifndef MAZDA_CMU_GUIDANCE_CLIENT_H
#define MAZDA_CMU_GUIDANCE_CLIENT_H

#include "model.h"

#include <stdint.h>

namespace mazda_cmu {
namespace guidance {

enum ClientTransport : uint8_t {
    CLIENT_LOCAL_BROKER = 1, // Android Auto in the broker's jciAAPA process.
    CLIENT_IPC = 2,          // CarPlay in its separate launcher process.
};

// One client singleton is compiled into each role DSO. start() is nonblocking;
// all role-facing state calls only update a latest-wins mailbox and wake its
// worker. The worker owns IPC/direct-broker delivery, reconnect, replay, and
// heartbeat behavior.
bool client_start(Source source, ClientTransport transport);
void client_stop();

// Opens a fresh source session and returns its nonzero epoch. BEGIN itself
// never claims or clears HUD ownership.
uint64_t client_session_begin();

// All calls with a stale or zero epoch are ignored locally. A supported full
// snapshot implicitly claims; claim() exists for authoritative TBT activation
// before the first renderable maneuver arrives.
void client_claim(uint64_t epoch);
void client_publish(uint64_t epoch, const Snapshot &snapshot);
void client_release(uint64_t epoch, ReleaseReason reason);
void client_session_end(uint64_t epoch);

} // namespace guidance
} // namespace mazda_cmu

#endif // MAZDA_CMU_GUIDANCE_CLIENT_H
