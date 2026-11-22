// SPDX-License-Identifier: AGPL-3.0-or-later

#define LIBPATCH_NAME "blmjciaapa"
#define LOG_TAG "GUIDANCE-SINK"
#include "common/log.h"

#include "hud_sink.h"
#include "common/oem/vbs_navi_hud.h"
#include "blmjciaapa/oem/libdbus.h"

#include <stdlib.h>

namespace mazda_cmu {
namespace guidance {
namespace {

static const char kServiceBusFallback[] = "unix:path=/tmp/dbus_service_socket";
static const char kSignalPath[] = "/com/NNG/Api/Server";
static const char kSignalInterface[] = "com.NNG.Api.Server.Guidance";
static const char kSignalMember[] = "GuidanceChangedForHUD";

bool append_i32(DBusMessageIter &iter, int32_t value)
{
    return dbus_message_iter_append_basic(&iter, DBUS_TYPE_INT32, &value) != 0;
}

bool emit(void *connection, uint32_t glyph, int32_t distance,
          uint8_t distance_unit, const char *street, const uint8_t *lanes)
{
    if (connection == nullptr ||
        !dbus_connection_get_is_connected(connection)) {
        return false;
    }

    void *message = dbus_message_new_signal(
        kSignalPath, kSignalInterface, kSignalMember);
    if (message == nullptr) return false;

    DBusMessageIter iter;
    dbus_message_iter_init_append(message, &iter);
    const char *text = street ? street : "";

    bool ok = append_i32(iter, static_cast<int32_t>(glyph));
    ok = ok && append_i32(iter, distance);
    ok = ok && append_i32(iter, static_cast<int32_t>(distance_unit));
    ok = ok && dbus_message_iter_append_basic(
                   &iter, DBUS_TYPE_STRING, &text) != 0;
    ok = ok && append_i32(
                   iter, static_cast<int32_t>(kProjectionSpeedSentinel));
    ok = ok && append_i32(iter, 0); // svcjcinavi merge supplies OEM speed/unit.
    for (unsigned i = 0; i < GUIDANCE_LANE_COUNT && ok; ++i) {
        ok = append_i32(iter, lanes ? static_cast<int32_t>(lanes[i]) : 0);
    }

    if (ok) ok = dbus_connection_send(connection, message, nullptr) != 0;
    if (ok) {
        dbus_connection_flush(connection);
        ok = dbus_connection_get_is_connected(connection) != 0;
    }
    dbus_message_unref(message);
    return ok;
}

} // namespace

HudSink::HudSink() : connection_(nullptr)
{
}

HudSink::~HudSink()
{
    disconnect();
}

bool HudSink::connect()
{
    if (connected()) return true;
    disconnect();

    const char *address = getenv("JCI_SERVICE_BUS");
    if (address == nullptr || address[0] == '\0') address = kServiceBusFallback;

    connection_ = dbus_connection_open_private(address, nullptr);
    if (connection_ == nullptr) {
        LOGE("could not open service bus %s", address);
        return false;
    }

    dbus_connection_set_exit_on_disconnect(connection_, 0);
    if (dbus_bus_register(connection_, nullptr) == 0) {
        LOGE("could not register private service-bus connection");
        disconnect();
        return false;
    }
    LOGD("connected to svcjcinavi service bus");
    return true;
}

bool HudSink::connected() const
{
    return connection_ != nullptr &&
           dbus_connection_get_is_connected(connection_) != 0;
}

bool HudSink::publish(const Snapshot &snapshot)
{
    const char *street = snapshot.street[0] ? snapshot.street : " ";
    return emit(connection_, snapshot.glyph, snapshot.distance_x10,
                snapshot.distance_unit, street, snapshot.lanes);
}

bool HudSink::clear()
{
    return emit(connection_, 0, 0, 0, "", nullptr);
}

void HudSink::disconnect()
{
    if (connection_ == nullptr) return;
    dbus_connection_close(connection_);
    dbus_connection_unref(connection_);
    connection_ = nullptr;
}

} // namespace guidance
} // namespace mazda_cmu
