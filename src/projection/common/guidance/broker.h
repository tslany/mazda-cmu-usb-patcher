// SPDX-License-Identifier: AGPL-3.0-or-later

#ifndef MAZDA_CMU_GUIDANCE_BROKER_H
#define MAZDA_CMU_GUIDANCE_BROKER_H

#include "model.h"
#include "wire.h"

namespace mazda_cmu {
namespace guidance {

// Process-long broker hosted by the jciAAPA preload. start() is nonblocking:
// it only creates the worker; socket and service-bus setup happen there.
bool broker_start();
void broker_stop();

// In-process ingress used by the Android Auto client worker. It applies the
// exact same validation, epoch, sequence, ownership, and lease rules as IPC.
// The message is copied before return.
// The AA role defines MAZDA_CMU_GUIDANCE_LOCAL_BROKER and gets an ordinary
// required reference. The CarPlay role deliberately omits the broker and uses
// weak references so the same client source remains linkable there.
#if defined(MAZDA_CMU_GUIDANCE_LOCAL_BROKER)
#define MAZDA_CMU_LOCAL_BROKER_REF
#else
#define MAZDA_CMU_LOCAL_BROKER_REF __attribute__((weak))
#endif
bool broker_submit_local(const wire::ControlMessage &message)
    MAZDA_CMU_LOCAL_BROKER_REF;
bool broker_submit_local(const wire::SnapshotMessage &message)
    MAZDA_CMU_LOCAL_BROKER_REF;
#undef MAZDA_CMU_LOCAL_BROKER_REF

} // namespace guidance
} // namespace mazda_cmu

#endif // MAZDA_CMU_GUIDANCE_BROKER_H
