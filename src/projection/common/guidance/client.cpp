// SPDX-License-Identifier: AGPL-3.0-or-later

#define LIBPATCH_NAME "guidance-client"
#define LOG_TAG "CLIENT"
#include "common/log.h"

#include "client.h"
#include "broker.h"
#include "renderer.h"
#include "wire.h"
#include "common/thread_util.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stddef.h>
#include <string.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

namespace mazda_cmu {
namespace guidance {
namespace {

static const int64_t kHeartbeatIntervalMs = 15 * 1000;
static const int64_t kReconnectDelayMs = 1000;

enum DesiredPhase : uint8_t {
    PHASE_BEGUN = 1,
    PHASE_CLAIMED = 2,
    PHASE_SNAPSHOT = 3,
    PHASE_RELEASED = 4,
};

struct DesiredState {
    bool session_open;
    uint64_t epoch;
    DesiredPhase phase;
    ReleaseReason release_reason;
    Snapshot snapshot;
    uint64_t revision;

    DesiredState()
        : session_open(false), epoch(0), phase(PHASE_BEGUN),
          release_reason(RELEASE_ROUTE_STOPPED), revision(0)
    {
        memset(&snapshot, 0, sizeof(snapshot));
    }
};

struct ClientRuntime {
    pthread_mutex_t lifecycle_mu;
    pthread_mutex_t state_mu;
    bool started;
    bool stopping;
    pthread_t thread;
    int wake_fd;
    Source source;
    ClientTransport transport;
    uint64_t instance;
    uint64_t epoch_counter;
    DesiredState desired;

    ClientRuntime()
        : started(false), stopping(false), thread(0), wake_fd(-1),
          source(SOURCE_NONE), transport(CLIENT_IPC), instance(0),
          epoch_counter(0)
    {
        pthread_mutex_init(&lifecycle_mu, nullptr);
        pthread_mutex_init(&state_mu, nullptr);
    }
};

ClientRuntime &runtime()
{
    static ClientRuntime value;
    return value;
}

int64_t monotonic_ms()
{
    struct timespec now;
#ifdef SYS_clock_gettime
    if (syscall(SYS_clock_gettime, CLOCK_MONOTONIC, &now) != 0) return 0;
#else
#error "guidance client requires the Linux clock_gettime syscall number"
#endif
    return static_cast<int64_t>(now.tv_sec) * 1000 + now.tv_nsec / 1000000;
}

uint64_t monotonic_ns()
{
    struct timespec now;
#ifdef SYS_clock_gettime
    if (syscall(SYS_clock_gettime, CLOCK_MONOTONIC, &now) != 0) return 0;
#else
#error "guidance client requires the Linux clock_gettime syscall number"
#endif
    return static_cast<uint64_t>(now.tv_sec) * 1000000000ull +
           static_cast<uint64_t>(now.tv_nsec);
}

uint64_t make_instance_id()
{
    uint64_t value = monotonic_ns();
    value ^= static_cast<uint64_t>(static_cast<uint32_t>(getpid())) << 32;
    value ^= reinterpret_cast<uintptr_t>(&runtime);
    return value ? value : 1;
}

void wake_worker(int fd)
{
    if (fd < 0) return;
    uint64_t one = 1;
    while (write(fd, &one, sizeof(one)) < 0 && errno == EINTR) {}
}

void drain_eventfd(int fd)
{
    uint64_t value;
    while (fd >= 0 && read(fd, &value, sizeof(value)) > 0) {}
}

bool copy_desired(DesiredState &out, bool &stopping)
{
    ClientRuntime &rt = runtime();
    pthread_mutex_lock(&rt.state_mu);
    out = rt.desired;
    stopping = rt.stopping;
    pthread_mutex_unlock(&rt.state_mu);
    return !stopping;
}

void fill_socket_address(struct sockaddr_un &address, socklen_t &length)
{
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    address.sun_path[0] = '\0';
    const size_t name_size = sizeof(wire::kSocketName) - 1;
    memcpy(address.sun_path + 1, wire::kSocketName, name_size);
    length = static_cast<socklen_t>(
        offsetof(struct sockaddr_un, sun_path) + 1 + name_size);
}

void set_nonblocking_cloexec(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    flags = fcntl(fd, F_GETFD, 0);
    if (flags >= 0) fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
}

int connect_ipc()
{
    int fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (fd < 0) return -1;
    set_nonblocking_cloexec(fd);

    struct sockaddr_un address;
    socklen_t length;
    fill_socket_address(address, length);
    int rc = connect(fd, reinterpret_cast<struct sockaddr *>(&address), length);
    if (rc != 0 && errno != EINPROGRESS) {
        close(fd);
        return -1;
    }
    if (rc != 0) {
        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLOUT;
        pfd.revents = 0;
        rc = poll(&pfd, 1, 250);
        int error = 0;
        socklen_t error_size = sizeof(error);
        if (rc <= 0 || getsockopt(fd, SOL_SOCKET, SO_ERROR, &error,
                                  &error_size) != 0 || error != 0) {
            close(fd);
            return -1;
        }
    }
    return fd;
}

typedef bool (*LocalControlFn)(const wire::ControlMessage &);
typedef bool (*LocalSnapshotFn)(const wire::SnapshotMessage &);

bool send_local(const wire::ControlMessage &message)
{
    LocalControlFn function = static_cast<LocalControlFn>(broker_submit_local);
    return function != nullptr && function(message);
}

bool send_local(const wire::SnapshotMessage &message)
{
    LocalSnapshotFn function = static_cast<LocalSnapshotFn>(broker_submit_local);
    return function != nullptr && function(message);
}

template <typename Message>
bool send_ipc(int fd, const Message &message)
{
    ssize_t count;
    do {
        count = send(fd, &message, sizeof(message),
                     MSG_DONTWAIT | MSG_NOSIGNAL);
    } while (count < 0 && errno == EINTR);
    return count == static_cast<ssize_t>(sizeof(message));
}

wire::Header make_header(wire::MessageType type, uint16_t size,
                         uint16_t code, uint32_t sequence, uint64_t epoch)
{
    ClientRuntime &rt = runtime();
    wire::Header header;
    memset(&header, 0, sizeof(header));
    header.magic = wire::kMagic;
    header.version = wire::kVersion;
    header.size = size;
    header.type = static_cast<uint8_t>(type);
    header.source = static_cast<uint8_t>(rt.source);
    header.code = code;
    header.sequence = sequence;
    header.instance = rt.instance;
    header.epoch = epoch;
    return header;
}

struct Delivery {
    int fd;
    bool hello_sent;
    bool session_open;
    uint64_t epoch;
    uint64_t revision;
    uint32_t sequence;
    int64_t heartbeat_due_ms;
    int64_t reconnect_after_ms;

    Delivery()
        : fd(-1), hello_sent(false), session_open(false), epoch(0),
          revision(0), sequence(0), heartbeat_due_ms(0),
          reconnect_after_ms(0)
    {}
};

void reset_delivery(Delivery &delivery, bool local)
{
    if (delivery.fd >= 0) close(delivery.fd);
    delivery.fd = -1;
    delivery.hello_sent = false;
    delivery.session_open = false;
    delivery.epoch = 0;
    delivery.revision = 0;
    // A local broker may have accepted earlier queued messages. Preserve its
    // monotonically increasing sequence across a replay. A fresh IPC connection
    // gets a fresh peer state, so restarting at one is correct there.
    if (!local) delivery.sequence = 0;
    delivery.heartbeat_due_ms = 0;
    delivery.reconnect_after_ms = monotonic_ms() + kReconnectDelayMs;
}

bool transport_send(const wire::ControlMessage &message, Delivery &delivery,
                    bool local)
{
    return local ? send_local(message) : send_ipc(delivery.fd, message);
}

bool transport_send(const wire::SnapshotMessage &message, Delivery &delivery,
                    bool local)
{
    return local ? send_local(message) : send_ipc(delivery.fd, message);
}

bool send_control(Delivery &delivery, bool local, wire::MessageType type,
                  uint64_t epoch, uint16_t code = 0)
{
    wire::ControlMessage message;
    memset(&message, 0, sizeof(message));
    const uint32_t sequence = type == wire::MESSAGE_HELLO
                                  ? 0
                                  : ++delivery.sequence;
    message.header = make_header(type, sizeof(message), code, sequence, epoch);
    return transport_send(message, delivery, local);
}

bool send_snapshot(Delivery &delivery, bool local, uint64_t epoch,
                   const Snapshot &snapshot)
{
    wire::SnapshotMessage message;
    memset(&message, 0, sizeof(message));
    message.header = make_header(wire::MESSAGE_SNAPSHOT, sizeof(message), 0,
                                 ++delivery.sequence, epoch);
    message.snapshot = snapshot;
    return transport_send(message, delivery, local);
}

bool ensure_transport(Delivery &delivery, bool local, int64_t now_ms)
{
    if (local) return true;
    if (delivery.fd >= 0) return true;
    if (now_ms < delivery.reconnect_after_ms) return false;
    delivery.fd = connect_ipc();
    if (delivery.fd < 0) {
        delivery.reconnect_after_ms = now_ms + kReconnectDelayMs;
        return false;
    }
    delivery.reconnect_after_ms = 0;
    return true;
}

bool replay_desired(Delivery &delivery, const DesiredState &desired,
                    bool local, int64_t now_ms)
{
    if (!ensure_transport(delivery, local, now_ms)) return false;

    if (!delivery.hello_sent) {
        if (!send_control(delivery, local, wire::MESSAGE_HELLO, 0)) return false;
        delivery.hello_sent = true;
    }

    if (delivery.session_open &&
        (!desired.session_open || desired.epoch != delivery.epoch)) {
        if (!send_control(delivery, local, wire::MESSAGE_SESSION_END,
                          delivery.epoch)) return false;
        delivery.session_open = false;
        delivery.epoch = 0;
        delivery.revision = 0;
    }

    if (!desired.session_open) return true;
    if (!delivery.session_open) {
        if (!send_control(delivery, local, wire::MESSAGE_SESSION_BEGIN,
                          desired.epoch)) return false;
        delivery.session_open = true;
        delivery.epoch = desired.epoch;
        delivery.revision = 0;
        delivery.heartbeat_due_ms = now_ms + kHeartbeatIntervalMs;
    }

    if (delivery.revision == desired.revision) return true;

    // Serialize the final revision check with role-facing mailbox mutations.
    // A worker that copied an older snapshot may deliver it only if that send
    // linearizes before the newer release/end callback updates the mailbox.
    // This keeps callbacks nonblocking apart from one nonblocking packet send.
    ClientRuntime &rt = runtime();
    pthread_mutex_lock(&rt.state_mu);
    const bool current = !rt.stopping &&
                         rt.desired.revision == desired.revision;
    bool delivered = true;
    if (current) {
        switch (desired.phase) {
        case PHASE_BEGUN:
            break;
        case PHASE_CLAIMED:
            delivered = send_control(delivery, local, wire::MESSAGE_CLAIM,
                                     desired.epoch);
            break;
        case PHASE_SNAPSHOT:
            delivered = send_snapshot(delivery, local, desired.epoch,
                                      desired.snapshot);
            break;
        case PHASE_RELEASED:
            delivered = send_control(
                delivery, local, wire::MESSAGE_RELEASE, desired.epoch,
                static_cast<uint16_t>(desired.release_reason));
            break;
        }
    }
    pthread_mutex_unlock(&rt.state_mu);
    if (!current) return true;
    if (!delivered) return false;
    delivery.revision = desired.revision;
    return true;
}

void *client_worker(void *)
{
    ClientRuntime &rt = runtime();
    const bool local = rt.transport == CLIENT_LOCAL_BROKER;
    Delivery delivery;

    for (;;) {
        DesiredState desired;
        bool stopping = false;
        copy_desired(desired, stopping);
        if (stopping) break;

        const int64_t now_ms = monotonic_ms();
        if (!replay_desired(delivery, desired, local, now_ms)) {
            reset_delivery(delivery, local);
        } else if (delivery.session_open &&
                   now_ms >= delivery.heartbeat_due_ms) {
            if (!send_control(delivery, local, wire::MESSAGE_HEARTBEAT,
                              delivery.epoch)) {
                reset_delivery(delivery, local);
            } else {
                delivery.heartbeat_due_ms = now_ms + kHeartbeatIntervalMs;
            }
        }

        int timeout_ms = 1000;
        if (delivery.session_open && delivery.heartbeat_due_ms > now_ms) {
            int64_t remaining = delivery.heartbeat_due_ms - now_ms;
            if (remaining < timeout_ms) timeout_ms = static_cast<int>(remaining);
        }
        if (!local && delivery.fd < 0 &&
            delivery.reconnect_after_ms > now_ms) {
            int64_t remaining = delivery.reconnect_after_ms - now_ms;
            if (remaining < timeout_ms) timeout_ms = static_cast<int>(remaining);
        }
        if (timeout_ms < 0) timeout_ms = 0;

        struct pollfd pollfds[2];
        pollfds[0].fd = rt.wake_fd;
        pollfds[0].events = POLLIN;
        pollfds[0].revents = 0;
        pollfds[1].fd = local ? -1 : delivery.fd;
        pollfds[1].events = delivery.fd >= 0 ? POLLIN : 0;
        pollfds[1].revents = 0;
        int rc = poll(pollfds, 2, timeout_ms);
        if (rc < 0 && errno != EINTR) {
            reset_delivery(delivery, local);
            continue;
        }
        if (pollfds[0].revents & POLLIN) drain_eventfd(rt.wake_fd);
        if (!local && delivery.fd >= 0 &&
            (pollfds[1].revents & (POLLIN | POLLERR | POLLHUP | POLLNVAL))) {
            // The broker never sends application payloads. Readable data is a
            // protocol violation or EOF; either way reconnect and replay.
            char byte;
            (void)recv(delivery.fd, &byte, sizeof(byte), MSG_DONTWAIT);
            reset_delivery(delivery, false);
        }
    }

    if (delivery.session_open) {
        (void)send_control(delivery, local, wire::MESSAGE_SESSION_END,
                           delivery.epoch);
    }
    if (delivery.fd >= 0) close(delivery.fd);
    return nullptr;
}

void mutate_wake_locked(ClientRuntime &rt)
{
    ++rt.desired.revision;
    wake_worker(rt.wake_fd);
}

} // namespace

bool client_start(Source source, ClientTransport transport)
{
    if ((source != SOURCE_ANDROID_AUTO && source != SOURCE_CARPLAY) ||
        (transport != CLIENT_LOCAL_BROKER && transport != CLIENT_IPC)) {
        return false;
    }
    if ((source == SOURCE_ANDROID_AUTO) !=
        (transport == CLIENT_LOCAL_BROKER)) {
        return false;
    }
    if (transport == CLIENT_LOCAL_BROKER) {
        LocalControlFn local_function =
            static_cast<LocalControlFn>(broker_submit_local);
        if (local_function == nullptr) return false;
    }

    ClientRuntime &rt = runtime();
    pthread_mutex_lock(&rt.lifecycle_mu);
    if (rt.started) {
        bool same = rt.source == source && rt.transport == transport;
        pthread_mutex_unlock(&rt.lifecycle_mu);
        return same;
    }

    int wake_fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (wake_fd < 0) {
        pthread_mutex_unlock(&rt.lifecycle_mu);
        return false;
    }

    pthread_mutex_lock(&rt.state_mu);
    rt.source = source;
    rt.transport = transport;
    rt.instance = make_instance_id();
    rt.epoch_counter = monotonic_ns();
    if (rt.epoch_counter == 0) rt.epoch_counter = 1;
    rt.desired = DesiredState();
    rt.stopping = false;
    rt.wake_fd = wake_fd;
    pthread_mutex_unlock(&rt.state_mu);

    if (preload_thread_create(&rt.thread, client_worker, nullptr) != 0) {
        pthread_mutex_lock(&rt.state_mu);
        rt.wake_fd = -1;
        pthread_mutex_unlock(&rt.state_mu);
        close(wake_fd);
        rt.thread = 0;
        pthread_mutex_unlock(&rt.lifecycle_mu);
        return false;
    }
    pthread_mutex_lock(&rt.state_mu);
    rt.started = true;
    pthread_mutex_unlock(&rt.state_mu);
    pthread_mutex_unlock(&rt.lifecycle_mu);
    return true;
}

void client_stop()
{
    ClientRuntime &rt = runtime();
    pthread_mutex_lock(&rt.lifecycle_mu);
    if (!rt.started) {
        pthread_mutex_unlock(&rt.lifecycle_mu);
        return;
    }
    pthread_mutex_lock(&rt.state_mu);
    rt.stopping = true;
    int wake_fd = rt.wake_fd;
    pthread_mutex_unlock(&rt.state_mu);
    wake_worker(wake_fd);
    pthread_join(rt.thread, nullptr);

    pthread_mutex_lock(&rt.state_mu);
    if (rt.wake_fd >= 0) close(rt.wake_fd);
    rt.wake_fd = -1;
    rt.thread = 0;
    rt.started = false;
    rt.stopping = false;
    rt.source = SOURCE_NONE;
    rt.instance = 0;
    rt.epoch_counter = 0;
    rt.desired = DesiredState();
    pthread_mutex_unlock(&rt.state_mu);
    pthread_mutex_unlock(&rt.lifecycle_mu);
}

uint64_t client_session_begin()
{
    ClientRuntime &rt = runtime();
    pthread_mutex_lock(&rt.state_mu);
    if (!rt.started || rt.stopping) {
        pthread_mutex_unlock(&rt.state_mu);
        return 0;
    }
    uint64_t epoch = monotonic_ns();
    if (epoch <= rt.epoch_counter) epoch = rt.epoch_counter + 1;
    if (epoch == 0) epoch = 1;
    rt.epoch_counter = epoch;
    rt.desired.session_open = true;
    rt.desired.epoch = epoch;
    rt.desired.phase = PHASE_BEGUN;
    rt.desired.release_reason = RELEASE_ROUTE_STOPPED;
    memset(&rt.desired.snapshot, 0, sizeof(rt.desired.snapshot));
    mutate_wake_locked(rt);
    pthread_mutex_unlock(&rt.state_mu);
    return epoch;
}

void client_claim(uint64_t epoch)
{
    ClientRuntime &rt = runtime();
    pthread_mutex_lock(&rt.state_mu);
    if (epoch != 0 && rt.started && !rt.stopping &&
        rt.desired.session_open && rt.desired.epoch == epoch) {
        // An authoritative status callback may arrive after the first live nav
        // frame. Keep a retained full snapshot so the claim can immediately
        // reassert it after the other projection source took ownership.
        if (rt.desired.phase != PHASE_SNAPSHOT) {
            rt.desired.phase = PHASE_CLAIMED;
        }
        mutate_wake_locked(rt);
    }
    pthread_mutex_unlock(&rt.state_mu);
}

void client_publish(uint64_t epoch, const Snapshot &snapshot)
{
    ClientRuntime &rt = runtime();
    pthread_mutex_lock(&rt.state_mu);
    if (epoch != 0 && rt.started && !rt.stopping &&
        rt.desired.session_open && rt.desired.epoch == epoch) {
        Snapshot clean = snapshot;
        sanitize_snapshot(clean);
        if (clean.glyph == 0) {
            rt.desired.phase = PHASE_RELEASED;
            rt.desired.release_reason = RELEASE_UNSUPPORTED_MANEUVER;
            memset(&rt.desired.snapshot, 0, sizeof(rt.desired.snapshot));
        } else {
            rt.desired.phase = PHASE_SNAPSHOT;
            rt.desired.snapshot = clean;
        }
        mutate_wake_locked(rt);
    }
    pthread_mutex_unlock(&rt.state_mu);
}

void client_release(uint64_t epoch, ReleaseReason reason)
{
    if (reason < RELEASE_ROUTE_STOPPED || reason > RELEASE_LEASE_EXPIRED)
        reason = RELEASE_ROUTE_STOPPED;
    ClientRuntime &rt = runtime();
    pthread_mutex_lock(&rt.state_mu);
    if (epoch != 0 && rt.started && !rt.stopping &&
        rt.desired.session_open && rt.desired.epoch == epoch) {
        rt.desired.phase = PHASE_RELEASED;
        rt.desired.release_reason = reason;
        memset(&rt.desired.snapshot, 0, sizeof(rt.desired.snapshot));
        mutate_wake_locked(rt);
    }
    pthread_mutex_unlock(&rt.state_mu);
}

void client_session_end(uint64_t epoch)
{
    ClientRuntime &rt = runtime();
    pthread_mutex_lock(&rt.state_mu);
    if (epoch != 0 && rt.started && !rt.stopping &&
        rt.desired.session_open && rt.desired.epoch == epoch) {
        rt.desired.session_open = false;
        rt.desired.phase = PHASE_RELEASED;
        rt.desired.release_reason = RELEASE_SESSION_ENDED;
        memset(&rt.desired.snapshot, 0, sizeof(rt.desired.snapshot));
        mutate_wake_locked(rt);
    }
    pthread_mutex_unlock(&rt.state_mu);
}

} // namespace guidance
} // namespace mazda_cmu
