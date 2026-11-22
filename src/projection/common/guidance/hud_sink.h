// SPDX-License-Identifier: AGPL-3.0-or-later

#ifndef MAZDA_CMU_GUIDANCE_HUD_SINK_H
#define MAZDA_CMU_GUIDANCE_HUD_SINK_H

#include "model.h"

namespace mazda_cmu {
namespace guidance {

// Single-threaded service-bus sink. Every method is called only by the broker
// worker. Failed sends leave the desired broker state pending for reconnect.
class HudSink {
public:
    HudSink();
    ~HudSink();

    bool connect();
    bool connected() const;
    bool publish(const Snapshot &snapshot);
    bool clear();
    void disconnect();

private:
    HudSink(const HudSink &);
    HudSink &operator=(const HudSink &);
    void *connection_;
};

} // namespace guidance
} // namespace mazda_cmu

#endif // MAZDA_CMU_GUIDANCE_HUD_SINK_H
