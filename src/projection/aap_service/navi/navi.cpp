// SPDX-License-Identifier: AGPL-3.0-or-later
// GAL 1.6 navigation negotiation and raw-frame relay for aap_service.

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#define LOG_TAG "NAVI"
#include "../log.h"
#include "navi.h"
#include "common/aa_nav16_msg.h"
#include "common/aa_nav_status.h"
#include "projection.h"

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cerrno>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

// Firmware 74.00.324A /usr/bin/aap_service is a non-PIE executable.
// Keep every firmware-bound address in the selected profile.
namespace addr = fw74_00_324a::aap_service;

typedef int      (*route_fn)(void *, uint32_t, uint32_t, const void *);
typedef void *   (*sp_deref_fn)(const void *);
typedef uint8_t *(*iob_raw_fn)(void *);
typedef int      (*iob_size_fn)(void *);
typedef void     (*status_cb_fn)(void *, int);

// FW74 handleNavigationStatus only extracts the enum and calls this observer.
// Use that same callback for REROUTING -> ACTIVE without changing the shared
// incoming IoBuffer or constructing a firmware-owned protobuf object.
void notify_oem_active(void *self)
{
    const void *ref = static_cast<const uint8_t *>(self) +
                      addr::NavigationCallbacksOffset;
    void *callbacks = reinterpret_cast<sp_deref_fn>(addr::NavigationCallbacksDeref)(ref);
    void **vtable = *static_cast<void ***>(callbacks);
    reinterpret_cast<status_cb_fn>(vtable[addr::NavigationStatusCallbackSlot])(callbacks, 1);
}

bool set_prot(uintptr_t addr, size_t len, int prot)
{
    const uintptr_t ps = (uintptr_t)sysconf(_SC_PAGESIZE);
    const uintptr_t page = addr & ~(ps - 1);
    return mprotect(reinterpret_cast<void *>(page), (addr + len) - page, prot) == 0;
}

const uint8_t *frame_bytes(const void *iobuf_ref, int *size_out)
{
    *size_out = 0;
    if (!iobuf_ref) return nullptr;
    void *iob = reinterpret_cast<sp_deref_fn>(addr::SharedPtrDeref)(iobuf_ref);
    if (!iob) return nullptr;
    const uint8_t *raw = reinterpret_cast<iob_raw_fn>(addr::IoBufferRaw)(iob);
    int size = reinterpret_cast<iob_size_fn>(addr::IoBufferSize)(iob);
    if (!raw || size <= 0) return nullptr;
    *size_out = size;
    return raw;
}

int g_tx_fd = -1;

void nav16_send_raw(const void *buf, size_t len)
{
    if (g_tx_fd < 0) {
        g_tx_fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (g_tx_fd < 0) {
            LOGE("nav16: socket() failed: %s", strerror(errno));
            return;
        }
    }
    struct sockaddr_un addr;
    socklen_t addrlen;
    aa_nav16_fill_addr(addr, addrlen);
    if (sendto(g_tx_fd, buf, len, MSG_DONTWAIT,
               reinterpret_cast<struct sockaddr *>(&addr), addrlen) < 0)
        LOGV("nav16: sendto dropped: %s", strerror(errno));
}

void relay_frame(const void *iobuf_ref)
{
    int size = 0;
    const uint8_t *raw = frame_bytes(iobuf_ref, &size);
    if (!raw || size < 2) {
        LOGV("nav16: relay skipped - no/short frame (size=%d)", size);
        return;
    }
    if (size > AA_NAV16_MAX_FRAME) {
        LOGW("nav16: oversized frame (%d bytes) dropped", size);
        return;
    }

    char buf[sizeof(AaNav16Hdr) + AA_NAV16_MAX_FRAME];
    AaNav16Hdr hdr = {AA_NAV16_MAGIC, AA_NAV16_VERSION, 0, (uint16_t)size};
    std::memcpy(buf, &hdr, sizeof(hdr));
    std::memcpy(buf + sizeof(hdr), raw, (size_t)size);
    nav16_send_raw(buf, sizeof(hdr) + (size_t)size);
}

int hook_routeMessage(void *self, uint32_t chan, uint32_t msgId,
                      const void *iobuf_ref)
{
    const uint32_t id = msgId & 0xffff;
    // GAL 1.6 adds cluster lifecycle/state/position messages that the stock
    // GAL 1.5 endpoint does not understand. Relay those to jciAAPA and consume
    // them here so the stock -253 reply cannot tear the whole AA session down.
    //
    // NavigationStatus (0x8003) and the legacy next-turn messages
    // (0x8004/0x8005) ARE understood by the stock endpoint. In particular,
    // 0x8004/0x8005 are what produce jciAAPA's 0x501/0x502 callback records when
    // the phone negotiates GAL 1.5. Swallowing the entire contiguous range made
    // the advertised 1.5 fallback impossible, because the raw 1.6 decoder quite
    // deliberately ignores those legacy messages.
    if (id == AA_NAV16_MSG_CLUSTER_START ||
        id == AA_NAV16_MSG_CLUSTER_STOP ||
        id == AA_NAV16_MSG_NAV_STATE ||
        id == AA_NAV16_MSG_POSITION) {
        LOGV("nav16: relay+swallow msgId=0x%04x chan=%u", id, chan);
        relay_frame(iobuf_ref);
        return 0;
    }

    // Preserve the original status for the HUD. The OEM parser accepts only
    // 0..2: enum 3 becomes an unknown field, leaving its required status unset
    // and tearing down AA. Rerouting still means navigation is active, so
    // deliver ACTIVE through the same observer the OEM route would call.
    // All other statuses and legacy turn/distance messages stay on that route.
    if (id == AA_NAV16_MSG_STATUS) {
        LOGV("nav16: relay+chain shared status msgId=0x%04x chan=%u", id, chan);
        relay_frame(iobuf_ref);
        int size = 0;
        const uint8_t *raw = frame_bytes(iobuf_ref, &size);
        int status;
        if (aa_nav_read_status(raw, size, &status) && status == 3) {
            notify_oem_active(self);
            return 0;
        }
    }

    return reinterpret_cast<route_fn>(addr::RouteMessage)(self, chan, msgId,
                                                           iobuf_ref);
}

bool bump_version()
{
    volatile uint32_t *p = reinterpret_cast<volatile uint32_t *>(addr::VersionInsn);
    if (*p != addr::VersionInsnOld) {
        LOGE("version: ABORT - 0x%08x at 0x%08lx, expected 0x%08x",
             *p, (unsigned long)addr::VersionInsn, addr::VersionInsnOld);
        return false;
    }
    if (!set_prot(addr::VersionInsn, sizeof(uint32_t),
                  PROT_READ | PROT_WRITE)) {
        LOGE("version: ABORT - mprotect RW failed");
        return false;
    }
    *p = addr::VersionInsnNew;
    if (!set_prot(addr::VersionInsn, sizeof(uint32_t), PROT_READ | PROT_EXEC))
        LOGW("version: mprotect RX restore failed (version IS bumped; continuing)");
    __builtin___clear_cache(reinterpret_cast<char *>(addr::VersionInsn),
                            reinterpret_cast<char *>(
                                addr::VersionInsn + sizeof(uint32_t)));
    LOGD("version: advertising GAL 1.6 (mov r2,#5 -> #6 @0x%08lx)",
         (unsigned long)addr::VersionInsn);
    return true;
}

bool install_nav_hook()
{
    volatile uintptr_t *slot = reinterpret_cast<volatile uintptr_t *>(addr::VtableSlot);
    if (*slot != addr::RouteMessage) {
        LOGE("hook: ABORT - slot7 @0x%08lx = 0x%08lx, expected 0x%08lx",
             (unsigned long)addr::VtableSlot, (unsigned long)*slot,
             (unsigned long)addr::RouteMessage);
        return false;
    }
    if (!set_prot(addr::VtableSlot, sizeof(uintptr_t), PROT_READ | PROT_WRITE)) {
        LOGE("hook: ABORT - mprotect RW failed on vtable");
        return false;
    }
    *slot = reinterpret_cast<uintptr_t>(&hook_routeMessage);
    LOGD("hook: nav routeMessage installed (slot7 @0x%08lx)",
         (unsigned long)addr::VtableSlot);
    if (!set_prot(addr::VtableSlot, sizeof(uintptr_t), PROT_READ))
        LOGW("hook: mprotect RO restore failed (hook IS installed; continuing)");
    return true;
}

bool restore_nav_hook()
{
    volatile uintptr_t *slot = reinterpret_cast<volatile uintptr_t *>(addr::VtableSlot);
    if (*slot != reinterpret_cast<uintptr_t>(&hook_routeMessage)) {
        LOGW("hook: rollback skipped - slot7 is no longer ours (0x%08lx)",
             (unsigned long)*slot);
        return false;
    }
    if (!set_prot(addr::VtableSlot, sizeof(uintptr_t), PROT_READ | PROT_WRITE)) {
        LOGE("hook: rollback failed - mprotect RW failed; hook remains installed");
        return false;
    }
    *slot = addr::RouteMessage;
    if (!set_prot(addr::VtableSlot, sizeof(uintptr_t), PROT_READ))
        LOGW("hook: mprotect RO restore failed (hook IS removed; vtable writable)");
    return true;
}

} // namespace

namespace aap_service_navi {

void init()
{
    if (!install_nav_hook()) {
        LOGE("GAL 1.6 path DISABLED - hook failed; staying stock GAL 1.5");
        return;
    }
    if (!bump_version()) {
        if (restore_nav_hook())
            LOGE("GAL 1.6 path ROLLED BACK; staying stock GAL 1.5");
        else
            LOGE("GAL 1.6 path DISABLED - navigation hook rollback failed");
        return;
    }
    LOGD("GAL 1.6 nav path active");
}

} // namespace aap_service_navi
