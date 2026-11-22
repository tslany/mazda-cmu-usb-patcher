// SPDX-License-Identifier: AGPL-3.0-or-later

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#define LIBPATCH_NAME "blmjciaapa"
#define LOG_TAG "GUIDANCE-BROKER"
#include "common/log.h"

#include "broker.h"
#include "hud_sink.h"
#include "renderer.h"
#include "common/thread_util.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stddef.h>
#include <stdio.h>
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

static const int64_t kHeartbeatLeaseMs = 45 * 1000;
static const int64_t kRefreshIntervalMs = 60 * 1000;
static const int64_t kReconnectDelayMs = 1000;
static const int64_t kUngreetedPeerTimeoutMs = 10 * 1000;
static const int64_t kStartupReconcileDelayMs = 5 * 1000;
static const unsigned kQueueCapacity = 64;
static const unsigned kPeerCapacity = 4;

int64_t monotonic_ms()
{
    struct timespec now;
#ifdef SYS_clock_gettime
    if (syscall(SYS_clock_gettime, CLOCK_MONOTONIC, &now) != 0) return 0;
#else
#error "guidance broker requires the Linux clock_gettime syscall number"
#endif
    return static_cast<int64_t>(now.tv_sec) * 1000 + now.tv_nsec / 1000000;
}

bool valid_source(uint8_t value)
{
    return value == SOURCE_ANDROID_AUTO || value == SOURCE_CARPLAY;
}

const char *source_name(Source source)
{
    switch (source) {
    case SOURCE_ANDROID_AUTO: return "Android Auto";
    case SOURCE_CARPLAY: return "CarPlay";
    default: return "none";
    }
}

struct QueuedPacket {
    size_t size;
    uint8_t bytes[sizeof(wire::SnapshotMessage)];
};

struct BrokerRuntime {
    pthread_mutex_t lifecycle_mu;
    pthread_mutex_t queue_mu;
    bool started;
    bool accepting;
    bool stopping;
    pthread_t thread;
    int wake_fd;
    QueuedPacket queue[kQueueCapacity];
    unsigned queue_head;
    unsigned queue_count;

    BrokerRuntime()
        : started(false), accepting(false), stopping(false), thread(0),
          wake_fd(-1), queue_head(0), queue_count(0)
    {
        pthread_mutex_init(&lifecycle_mu, nullptr);
        pthread_mutex_init(&queue_mu, nullptr);
        memset(queue, 0, sizeof(queue));
    }
};

BrokerRuntime &runtime()
{
    static BrokerRuntime value;
    return value;
}

void wake_worker(int fd)
{
    if (fd < 0) return;
    uint64_t one = 1;
    while (write(fd, &one, sizeof(one)) < 0 && errno == EINTR) {}
}

bool enqueue_local(const void *message, size_t size)
{
    if (message == nullptr || size > sizeof(wire::SnapshotMessage)) return false;
    BrokerRuntime &rt = runtime();

    pthread_mutex_lock(&rt.queue_mu);
    if (!rt.accepting || rt.queue_count == kQueueCapacity) {
        pthread_mutex_unlock(&rt.queue_mu);
        return false;
    }
    unsigned tail = (rt.queue_head + rt.queue_count) % kQueueCapacity;
    rt.queue[tail].size = size;
    memcpy(rt.queue[tail].bytes, message, size);
    ++rt.queue_count;
    wake_worker(rt.wake_fd);
    pthread_mutex_unlock(&rt.queue_mu);
    return true;
}

bool pop_local(QueuedPacket &packet)
{
    BrokerRuntime &rt = runtime();
    pthread_mutex_lock(&rt.queue_mu);
    if (rt.queue_count == 0) {
        pthread_mutex_unlock(&rt.queue_mu);
        return false;
    }
    packet = rt.queue[rt.queue_head];
    rt.queue_head = (rt.queue_head + 1) % kQueueCapacity;
    --rt.queue_count;
    pthread_mutex_unlock(&rt.queue_mu);
    return true;
}

bool stop_requested()
{
    BrokerRuntime &rt = runtime();
    pthread_mutex_lock(&rt.queue_mu);
    bool value = rt.stopping;
    pthread_mutex_unlock(&rt.queue_mu);
    return value;
}

void drain_eventfd(int fd)
{
    if (fd < 0) return;
    uint64_t value;
    while (read(fd, &value, sizeof(value)) > 0) {}
}

struct Peer {
    int fd;
    uint64_t id;
    bool local;
    bool greeted;
    Source source;
    uint64_t instance;
    pid_t pid;
    int64_t accepted_ms;

    Peer()
        : fd(-1), id(0), local(false), greeted(false), source(SOURCE_NONE),
          instance(0), pid(-1), accepted_ms(0)
    {}
};

struct SourceState {
    uint64_t peer_id;
    uint64_t instance;
    uint64_t epoch;
    uint64_t highest_epoch;
    uint32_t last_sequence;
    bool session_open;
    bool claimed;
    bool snapshot_valid;
    bool local;
    Snapshot snapshot;
    uint64_t authority_serial;
    int64_t last_activity_ms;

    SourceState()
        : peer_id(0), instance(0), epoch(0), highest_epoch(0),
          last_sequence(0), session_open(false), claimed(false),
          snapshot_valid(false), local(false), authority_serial(0),
          last_activity_ms(0)
    {
        memset(&snapshot, 0, sizeof(snapshot));
    }
};

struct OutputState {
    Source owner;
    uint64_t instance;
    uint64_t epoch;
    bool have_snapshot;
    bool dirty;
    bool pending_clear;
    bool output_claimed;
    Snapshot snapshot;
    int64_t refresh_due_ms;
    int64_t reconnect_after_ms;

    OutputState()
        : owner(SOURCE_NONE), instance(0), epoch(0), have_snapshot(false),
          dirty(false), pending_clear(false), output_claimed(false),
          refresh_due_ms(0), reconnect_after_ms(0)
    {
        memset(&snapshot, 0, sizeof(snapshot));
    }
};

struct Core {
    SourceState sources[3];
    OutputState output;
    HudSink sink;
    bool source_activity_seen;
    bool startup_reconcile_scheduled;
    uint64_t next_authority_serial;

    Core()
        : source_activity_seen(false), startup_reconcile_scheduled(false),
          next_authority_serial(0)
    {}

    bool owner_is(Source source, const SourceState &state) const
    {
        return output.owner == source && output.instance == state.instance &&
               output.epoch == state.epoch;
    }

    void clear_owner()
    {
        output.owner = SOURCE_NONE;
        output.instance = 0;
        output.epoch = 0;
        output.have_snapshot = false;
        output.dirty = false;
        memset(&output.snapshot, 0, sizeof(output.snapshot));
        // A failed publish or a broker restart means the external retained HUD
        // state is unknown even when this process never recorded a successful
        // output. Releasing an owner therefore always converges through one
        // explicit sentinel blank unless a retained source is promoted.
        output.pending_clear = true;
    }

    uint64_t allocate_authority_serial()
    {
        ++next_authority_serial;
        if (next_authority_serial == 0) ++next_authority_serial;
        return next_authority_serial;
    }

    bool promote_retained(Source released_source)
    {
        Source promoted = SOURCE_NONE;
        SourceState *best = nullptr;
        for (unsigned value = SOURCE_ANDROID_AUTO;
             value <= SOURCE_CARPLAY; ++value) {
            Source source = static_cast<Source>(value);
            SourceState &candidate = sources[value];
            if (source == released_source || !candidate.session_open ||
                !candidate.claimed || !candidate.snapshot_valid) {
                continue;
            }
            if (best == nullptr ||
                candidate.authority_serial > best->authority_serial) {
                promoted = source;
                best = &candidate;
            }
        }
        if (best == nullptr) return false;

        output.owner = promoted;
        output.instance = best->instance;
        output.epoch = best->epoch;
        output.have_snapshot = true;
        output.snapshot = best->snapshot;
        output.pending_clear = false;
        output.dirty = true;
        LOGD("restored retained %s owner (epoch=%llu serial=%llu)",
             source_name(promoted),
             static_cast<unsigned long long>(best->epoch),
             static_cast<unsigned long long>(best->authority_serial));
        return true;
    }

    void release(Source source, SourceState &state, ReleaseReason reason)
    {
        const bool released_owner = owner_is(source, state);
        state.claimed = false;
        state.snapshot_valid = false;
        state.authority_serial = 0;
        memset(&state.snapshot, 0, sizeof(state.snapshot));
        if (released_owner) {
            LOGD("%s released owner (epoch=%llu reason=%u)",
                 source_name(source),
                 static_cast<unsigned long long>(state.epoch),
                 static_cast<unsigned>(reason));
            if (!promote_retained(source)) clear_owner();
        }
    }

    void bind_peer(Source source, const Peer &peer)
    {
        SourceState &state = sources[static_cast<unsigned>(source)];
        if (state.peer_id == peer.id && state.instance == peer.instance) return;
        if (state.peer_id != 0) {
            release(source, state, RELEASE_SOURCE_DISCONNECTED);
        }
        state = SourceState();
        state.peer_id = peer.id;
        state.instance = peer.instance;
        state.local = peer.local;
    }

    void disconnect_peer(const Peer &peer)
    {
        if (!peer.greeted || !valid_source(peer.source)) return;
        SourceState &state = sources[static_cast<unsigned>(peer.source)];
        if (state.peer_id != peer.id || state.instance != peer.instance) return;
        release(peer.source, state, RELEASE_SOURCE_DISCONNECTED);
        state = SourceState();
    }

    void claim(Source source, SourceState &state)
    {
        state.claimed = true;
        state.snapshot_valid = false;
        memset(&state.snapshot, 0, sizeof(state.snapshot));
        state.authority_serial = allocate_authority_serial();
        source_activity_seen = true;
        if (owner_is(source, state)) return;

        output.owner = source;
        output.instance = state.instance;
        output.epoch = state.epoch;
        output.have_snapshot = false;
        output.dirty = false;
        // CLAIM is authoritative source activity without a picture. Clear once
        // even when this freshly restarted broker did not itself publish the
        // stale projection still retained by svcjcinavi.
        output.pending_clear = true;
        memset(&output.snapshot, 0, sizeof(output.snapshot));
        LOGD("%s claimed owner without a snapshot (epoch=%llu)",
             source_name(source),
             static_cast<unsigned long long>(state.epoch));
    }

    void publish(Source source, SourceState &state, const Snapshot &snapshot)
    {
        // Repeated frames still renew ownership, but need no extra bus output.
        // Keep pending sends and let drive_output retain its refresh deadline.
        const bool needs_publish = output.dirty ||
            !owner_is(source, state) || !output.have_snapshot || !sink.connected() ||
            memcmp(&output.snapshot, &snapshot, sizeof(snapshot)) != 0;
        state.claimed = true;
        source_activity_seen = true;
        state.snapshot_valid = true;
        state.snapshot = snapshot;
        state.authority_serial = allocate_authority_serial();

        output.owner = source;
        output.instance = state.instance;
        output.epoch = state.epoch;
        output.have_snapshot = true;
        output.snapshot = snapshot;
        output.pending_clear = false; // Direct replacement avoids a blank flash.
        output.dirty = needs_publish;
        LOGV("%s published glyph=%u epoch=%llu",
             source_name(source), static_cast<unsigned>(snapshot.glyph),
             static_cast<unsigned long long>(state.epoch));
    }

    uint64_t expire_leases(int64_t now_ms)
    {
        uint64_t expired_peer_id = 0;
        for (unsigned value = SOURCE_ANDROID_AUTO;
             value <= SOURCE_CARPLAY; ++value) {
            SourceState &state = sources[value];
            // The local AA producer and broker share one process. There is no
            // transport to close/replay after a local lease expiry, while a
            // dead jciAAPA process tears down both sides together. The lease is
            // therefore meaningful only for the remote CarPlay peer.
            if (state.local || !state.session_open ||
                state.last_activity_ms <= 0) continue;
            if (now_ms - state.last_activity_ms <= kHeartbeatLeaseMs) continue;
            Source source = static_cast<Source>(value);
            LOGW("%s guidance lease expired (epoch=%llu)",
                 source_name(source),
                 static_cast<unsigned long long>(state.epoch));
            release(source, state, RELEASE_LEASE_EXPIRED);
            expired_peer_id = state.peer_id;
            state.session_open = false;
            state.epoch = 0;
        }
        return expired_peer_id;
    }

    void schedule_startup_reconcile(int64_t now_ms, int64_t due_ms)
    {
        if (startup_reconcile_scheduled || source_activity_seen ||
            now_ms < due_ms) return;

        // svcjcinavi retains the last sentinel-owned projection if jciAAPA
        // crashes before sending a release. After a short reconnect window,
        // emit one fail-safe blank when no source came back to claim/publish.
        // On a live native route this can be one blank frame; the next native
        // generation immediately restores it. Bus failures remain pending and
        // retry, but this edge is scheduled only once per broker lifetime.
        startup_reconcile_scheduled = true;
        output.pending_clear = true;
        LOGD("scheduled one startup reconciliation clear");
    }

    void drive_output(int64_t now_ms)
    {
        if (output.owner != SOURCE_NONE && output.have_snapshot &&
            !output.dirty && now_ms >= output.refresh_due_ms) {
            output.dirty = true;
        }

        const bool need_publish = output.owner != SOURCE_NONE &&
                                  output.have_snapshot && output.dirty;
        const bool need_clear = !need_publish && output.pending_clear;
        if (!need_publish && !need_clear) return;
        if (now_ms < output.reconnect_after_ms) return;

        if (!sink.connected() && !sink.connect()) {
            output.reconnect_after_ms = now_ms + kReconnectDelayMs;
            return;
        }

        bool ok = need_publish ? sink.publish(output.snapshot) : sink.clear();
        if (!ok) {
            LOGE("service-bus %s failed; retaining desired state for replay",
                 need_publish ? "publish" : "clear");
            sink.disconnect();
            output.reconnect_after_ms = now_ms + kReconnectDelayMs;
            return;
        }

        output.reconnect_after_ms = 0;
        if (need_publish) {
            output.dirty = false;
            output.output_claimed = true;
            output.refresh_due_ms = now_ms + kRefreshIntervalMs;
        } else {
            output.pending_clear = false;
            output.output_claimed = false;
            output.refresh_due_ms = 0;
        }
    }
};

bool header_shape_valid(const wire::Header &header, size_t received)
{
    if (header.magic != wire::kMagic || header.version != wire::kVersion ||
        header.size != received || !valid_source(header.source)) {
        return false;
    }
    const bool snapshot = header.type == wire::MESSAGE_SNAPSHOT;
    const size_t expected = snapshot ? sizeof(wire::SnapshotMessage)
                                     : sizeof(wire::ControlMessage);
    if (received != expected) return false;
    if (header.type < wire::MESSAGE_HELLO ||
        header.type > wire::MESSAGE_HEARTBEAT) return false;
    if (header.type != wire::MESSAGE_RELEASE && header.code != 0) return false;
    if (header.type == wire::MESSAGE_RELEASE &&
        (header.code < RELEASE_ROUTE_STOPPED ||
         header.code > RELEASE_LEASE_EXPIRED)) return false;
    if (header.type == wire::MESSAGE_HELLO) {
        return header.epoch == 0 && header.sequence == 0 &&
               header.instance != 0;
    }
    return header.epoch != 0 && header.sequence != 0 &&
           header.instance != 0;
}

bool peer_cmdline_allows(pid_t pid, Source source)
{
    // The local AA ingress never reaches this function. IPC is fail-closed and
    // accepts only the real CarPlay service launch shape. Requiring all three
    // fields prevents an inherited watchdog command such as `killall
    // jciCARPLAY` from replacing the actual producer peer.
    if (pid <= 0 || source != SOURCE_CARPLAY) return false;
    char path[64];
    snprintf(path, sizeof(path), "/proc/%ld/cmdline", static_cast<long>(pid));
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return false;
    char text[512];
    ssize_t count = read(fd, text, sizeof(text));
    close(fd);
    if (count <= 0) return false;

    bool launcher = false;
    bool service = false;
    bool module = false;
    size_t argument_index = 0;
    size_t offset = 0;
    while (offset < static_cast<size_t>(count)) {
        const char *argument = text + offset;
        size_t remaining = static_cast<size_t>(count) - offset;
        const void *terminator = memchr(argument, '\0', remaining);
        size_t length = terminator
                            ? static_cast<const char *>(terminator) - argument
                            : remaining;
        if (length == sizeof("jciCARPLAY") - 1 &&
            memcmp(argument, "jciCARPLAY", length) == 0) {
            service = true;
        }
        const char *base = argument;
        for (size_t i = 0; i < length; ++i) {
            if (argument[i] == '/') base = argument + i + 1;
        }
        size_t base_length = length - static_cast<size_t>(base - argument);
        if (argument_index == 0 &&
            base_length == sizeof("sm_svclauncher") - 1 &&
            memcmp(base, "sm_svclauncher", base_length) == 0) {
            launcher = true;
        }
        if (length == sizeof("/jci/carplay/blmjcicarplay.so") - 1 &&
            memcmp(argument, "/jci/carplay/blmjcicarplay.so", length) == 0) {
            module = true;
        }
        if (!terminator) break;
        offset += length + 1;
        ++argument_index;
    }
    return launcher && service && module;
}

bool process_message(Core &core, Peer &peer, const void *data, size_t size,
                     int64_t now_ms)
{
    if (data == nullptr || size < sizeof(wire::Header)) return false;
    wire::Header header;
    memcpy(&header, data, sizeof(header));
    if (!header_shape_valid(header, size)) return false;
    Source source = static_cast<Source>(header.source);

    if (header.type == wire::MESSAGE_HELLO) {
        if ((peer.local && source != SOURCE_ANDROID_AUTO) ||
            (!peer.local && !peer_cmdline_allows(peer.pid, source))) {
            LOGW("rejected HELLO source=%u pid=%ld local=%d",
                 static_cast<unsigned>(source), static_cast<long>(peer.pid),
                 peer.local ? 1 : 0);
            return false;
        }
        if (peer.greeted &&
            (peer.source != source || peer.instance != header.instance)) {
            return false;
        }
        peer.greeted = true;
        peer.source = source;
        peer.instance = header.instance;
        core.bind_peer(source, peer);
        LOGD("accepted %s client instance=%llu pid=%ld",
             source_name(source),
             static_cast<unsigned long long>(peer.instance),
             static_cast<long>(peer.pid));
        return true;
    }

    if (!peer.greeted || peer.source != source ||
        peer.instance != header.instance) return false;
    SourceState &state = core.sources[static_cast<unsigned>(source)];
    if (state.peer_id != peer.id || state.instance != header.instance)
        return false;

    if (header.type == wire::MESSAGE_SESSION_BEGIN) {
        if (header.sequence <= state.last_sequence) return true;
        if (state.session_open && header.epoch == state.epoch) {
            state.last_sequence = header.sequence;
            state.last_activity_ms = now_ms;
            return true;
        }
        if (header.epoch <= state.highest_epoch) return true;
        if (state.session_open) {
            core.release(source, state, RELEASE_SESSION_ENDED);
        }
        state.epoch = header.epoch;
        state.highest_epoch = header.epoch;
        state.last_sequence = header.sequence;
        state.session_open = true;
        state.claimed = false;
        state.snapshot_valid = false;
        state.last_activity_ms = now_ms;
        memset(&state.snapshot, 0, sizeof(state.snapshot));
        LOGD("%s session begin epoch=%llu", source_name(source),
             static_cast<unsigned long long>(state.epoch));
        return true;
    }

    // All remaining messages belong to the currently open exact epoch and are
    // strictly ordered. Old callbacks are benign drops, not protocol failures.
    if (!state.session_open || header.epoch != state.epoch ||
        header.sequence <= state.last_sequence) return true;
    state.last_sequence = header.sequence;
    state.last_activity_ms = now_ms;

    switch (header.type) {
    case wire::MESSAGE_CLAIM:
        core.claim(source, state);
        return true;
    case wire::MESSAGE_SNAPSHOT: {
        wire::SnapshotMessage message;
        memcpy(&message, data, sizeof(message));
        if (!snapshot_is_valid(message.snapshot)) {
            LOGW("dropped invalid %s snapshot", source_name(source));
            return true;
        }
        core.publish(source, state, message.snapshot);
        return true;
    }
    case wire::MESSAGE_RELEASE:
        core.release(source, state,
                     static_cast<ReleaseReason>(header.code));
        return true;
    case wire::MESSAGE_SESSION_END:
        core.release(source, state, RELEASE_SESSION_ENDED);
        state.session_open = false;
        state.epoch = 0;
        LOGD("%s session ended", source_name(source));
        return true;
    case wire::MESSAGE_HEARTBEAT:
        return true;
    default:
        return false;
    }
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

int open_listener()
{
    int fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (fd < 0) {
        LOGE("SOCK_SEQPACKET listener socket failed: %s", strerror(errno));
        return -1;
    }
    set_nonblocking_cloexec(fd);

    struct sockaddr_un address;
    socklen_t length;
    fill_socket_address(address, length);
    if (bind(fd, reinterpret_cast<struct sockaddr *>(&address), length) != 0) {
        LOGE("bind(@%s) failed: %s", wire::kSocketName, strerror(errno));
        close(fd);
        return -1;
    }
    if (listen(fd, static_cast<int>(kPeerCapacity)) != 0) {
        LOGE("listen(@%s) failed: %s", wire::kSocketName, strerror(errno));
        close(fd);
        return -1;
    }
    LOGD("listening on abstract socket @%s", wire::kSocketName);
    return fd;
}

void close_peer(Core &core, Peer &peer)
{
    if (peer.fd >= 0) close(peer.fd);
    core.disconnect_peer(peer);
    peer = Peer();
}

void accept_peers(int listener, Peer *peers, uint64_t &next_peer_id,
                  int64_t now_ms)
{
    for (;;) {
        int fd = accept(listener, nullptr, nullptr);
        if (fd < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
                LOGE("accept failed: %s", strerror(errno));
            return;
        }
        set_nonblocking_cloexec(fd);

        unsigned slot = kPeerCapacity;
        for (unsigned i = 0; i < kPeerCapacity; ++i) {
            if (peers[i].fd < 0) { slot = i; break; }
        }
        if (slot == kPeerCapacity) {
            close(fd);
            continue;
        }

        struct ucred credentials;
        socklen_t credentials_size = sizeof(credentials);
        memset(&credentials, 0, sizeof(credentials));
        if (getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &credentials,
                       &credentials_size) != 0) {
            credentials.pid = -1;
        }
        peers[slot].fd = fd;
        peers[slot].id = next_peer_id++;
        peers[slot].pid = credentials.pid;
        peers[slot].accepted_ms = now_ms;
    }
}

void receive_peer(Core &core, Peer &peer, int64_t now_ms)
{
    uint8_t buffer[sizeof(wire::SnapshotMessage)];
    ssize_t count = recv(peer.fd, buffer, sizeof(buffer),
                         MSG_DONTWAIT | MSG_TRUNC);
    if (count == 0) {
        close_peer(core, peer);
        return;
    }
    if (count < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
            close_peer(core, peer);
        return;
    }
    if (static_cast<size_t>(count) > sizeof(buffer) ||
        !process_message(core, peer, buffer, static_cast<size_t>(count),
                         now_ms)) {
        close_peer(core, peer);
    }
}

void *broker_worker(void *)
{
    BrokerRuntime &rt = runtime();
    Core core;
    Peer local_peer;
    local_peer.local = true;
    local_peer.id = 1;
    local_peer.pid = getpid();
    local_peer.accepted_ms = monotonic_ms();

    Peer peers[kPeerCapacity];
    uint64_t next_peer_id = 2;
    int listener = -1;
    int64_t listener_retry_ms = 0;
    const int64_t startup_reconcile_due_ms =
        monotonic_ms() + kStartupReconcileDelayMs;

    while (!stop_requested()) {
        const int64_t before_poll = monotonic_ms();
        if (listener < 0 && before_poll >= listener_retry_ms) {
            listener = open_listener();
            if (listener < 0)
                listener_retry_ms = before_poll + kReconnectDelayMs;
        }

        struct pollfd pollfds[2 + kPeerCapacity];
        unsigned count = 0;
        pollfds[count].fd = rt.wake_fd;
        pollfds[count].events = POLLIN;
        pollfds[count].revents = 0;
        ++count;
        pollfds[count].fd = listener;
        pollfds[count].events = listener >= 0 ? POLLIN : 0;
        pollfds[count].revents = 0;
        ++count;
        for (unsigned i = 0; i < kPeerCapacity; ++i) {
            pollfds[count].fd = peers[i].fd;
            pollfds[count].events = peers[i].fd >= 0 ? POLLIN : 0;
            pollfds[count].revents = 0;
            ++count;
        }

        int rc = poll(pollfds, count, 1000);
        const int64_t now_ms = monotonic_ms();
        if (rc < 0 && errno != EINTR) LOGE("poll failed: %s", strerror(errno));

        if (pollfds[0].revents & POLLIN) {
            drain_eventfd(rt.wake_fd);
            QueuedPacket packet;
            while (pop_local(packet)) {
                if (!process_message(core, local_peer, packet.bytes,
                                     packet.size, now_ms)) {
                    core.disconnect_peer(local_peer);
                    local_peer.greeted = false;
                    local_peer.source = SOURCE_NONE;
                    local_peer.instance = 0;
                }
            }
        }
        if (listener >= 0 && (pollfds[1].revents & POLLIN)) {
            accept_peers(listener, peers, next_peer_id, now_ms);
        }
        for (unsigned i = 0; i < kPeerCapacity; ++i) {
            const short events = pollfds[2 + i].revents;
            if (peers[i].fd < 0) continue;
            if (events & POLLIN) receive_peer(core, peers[i], now_ms);
            if (peers[i].fd >= 0 &&
                (events & (POLLERR | POLLHUP | POLLNVAL))) {
                close_peer(core, peers[i]);
            }
            if (peers[i].fd >= 0 && !peers[i].greeted &&
                now_ms - peers[i].accepted_ms > kUngreetedPeerTimeoutMs) {
                close_peer(core, peers[i]);
            }
        }

        uint64_t expired_peer_id = core.expire_leases(now_ms);
        if (expired_peer_id != 0) {
            for (unsigned i = 0; i < kPeerCapacity; ++i) {
                if (peers[i].id == expired_peer_id) {
                    close_peer(core, peers[i]);
                    break;
                }
            }
        }
        core.schedule_startup_reconcile(now_ms, startup_reconcile_due_ms);
        core.drive_output(now_ms);
    }

    for (unsigned i = 0; i < kPeerCapacity; ++i) close_peer(core, peers[i]);
    core.disconnect_peer(local_peer);
    if (core.output.output_claimed && core.sink.connected()) {
        core.sink.clear();
    }
    core.sink.disconnect();
    if (listener >= 0) close(listener);
    return nullptr;
}

} // namespace

bool broker_start()
{
    BrokerRuntime &rt = runtime();
    pthread_mutex_lock(&rt.lifecycle_mu);
    if (rt.started) {
        pthread_mutex_unlock(&rt.lifecycle_mu);
        return true;
    }

    int wake_fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (wake_fd < 0) {
        LOGE("eventfd failed: %s", strerror(errno));
        pthread_mutex_unlock(&rt.lifecycle_mu);
        return false;
    }

    pthread_mutex_lock(&rt.queue_mu);
    rt.wake_fd = wake_fd;
    rt.queue_head = 0;
    rt.queue_count = 0;
    rt.stopping = false;
    rt.accepting = true;
    pthread_mutex_unlock(&rt.queue_mu);

    if (preload_thread_create(&rt.thread, broker_worker, nullptr) != 0) {
        pthread_mutex_lock(&rt.queue_mu);
        rt.accepting = false;
        rt.wake_fd = -1;
        pthread_mutex_unlock(&rt.queue_mu);
        close(wake_fd);
        rt.thread = 0;
        pthread_mutex_unlock(&rt.lifecycle_mu);
        return false;
    }
    rt.started = true;
    pthread_mutex_unlock(&rt.lifecycle_mu);
    return true;
}

void broker_stop()
{
    BrokerRuntime &rt = runtime();
    pthread_mutex_lock(&rt.lifecycle_mu);
    if (!rt.started) {
        pthread_mutex_unlock(&rt.lifecycle_mu);
        return;
    }

    pthread_mutex_lock(&rt.queue_mu);
    rt.accepting = false;
    rt.stopping = true;
    int wake_fd = rt.wake_fd;
    pthread_mutex_unlock(&rt.queue_mu);
    wake_worker(wake_fd);
    pthread_join(rt.thread, nullptr);

    pthread_mutex_lock(&rt.queue_mu);
    if (rt.wake_fd >= 0) close(rt.wake_fd);
    rt.wake_fd = -1;
    rt.thread = 0;
    rt.queue_head = 0;
    rt.queue_count = 0;
    rt.stopping = false;
    rt.started = false;
    pthread_mutex_unlock(&rt.queue_mu);
    pthread_mutex_unlock(&rt.lifecycle_mu);
}

bool broker_submit_local(const wire::ControlMessage &message)
{
    return enqueue_local(&message, sizeof(message));
}

bool broker_submit_local(const wire::SnapshotMessage &message)
{
    return enqueue_local(&message, sizeof(message));
}

} // namespace guidance
} // namespace mazda_cmu
