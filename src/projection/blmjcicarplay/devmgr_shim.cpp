// SPDX-License-Identifier: AGPL-3.0-or-later
//
// CarPlay -> HUD bridge for Mazda Connect (CMU150).
// Copyright (C) 2026 KID MIXER-MODER.
//
// Part of an AGPL-3.0 work - see NOTICE.md for full attribution.
//
// PLT shadows for the devmgr client API that jciCARPLAY (blmjcicarplay.so)
// imports from /usr/lib/libdevmgr_interface.so, PLUS a libc `msgrcv` interposer
// that passively taps the device-manager receive path.
//
// Why msgrcv: RE of eDevMgrRegisterProcess + the dispatch loop (FUN_000402c8 in
// libdevmgr_interface.so) shows the process handle stores cbTable@+0x6c, ctx@+0x70,
// msgq@+0x14, and the receive thread (vDevMgrProcessRcvThread, running INSIDE the
// jciCARPLAY process) pulls every event with race_msg_queue_receivemessage ->
// libc `msgrcv` (imported via PLT, confirmed). ipoddev's viAP2NavRouteManeuverUpdate
// pushes a 0x348-byte maneuver message (guidance = 0x65c). So interposing msgrcv lets us
// SEE every message ipoddev delivers — including maneuvers — without touching the
// OEM vtable/dispatch (fully passive: we always chain the real call and never
// alter the data). The decoder turns those records into the common broker's
// Mazda-domain snapshot.
//
// ARM EABI: devmgr fns take args in r0..r3; we declare 4 void* and forward all.

#include "patch.h"
#include "hud/hud_send.h"
#include "oem/libjcidbus.h"
#include "common/thread_util.h"
#include "projection.h"

#include <dlfcn.h>
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <atomic>

namespace {

namespace cp_layout = fw74_00_324a::carplay_nav;

typedef int (*pfn4)(void *, void *, void *, void *);

pfn4  g_real_register  = nullptr;
pfn4  g_real_callstate = nullptr;
pfn4  g_real_commupd   = nullptr;
pfn4  g_real_sendloc   = nullptr;
pfn4  g_real_startnavi = nullptr;

void *g_devmgr_handle = nullptr;
pthread_mutex_t g_mu        = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t g_decode_mu = PTHREAD_MUTEX_INITIALIZER;
// Serializes the final live-session check and each OEM StartNavi call against
// session deactivation.  The call gate is deliberately outside g_decode_mu:
// the OEM call may synchronously need the devmgr receive/decode path.
pthread_mutex_t g_startnavi_call_mu = PTHREAD_MUTEX_INITIALIZER;
bool            g_navi_armed = false;
std::atomic<bool> g_armed{false};   // gates the passive msgrcv tap
std::atomic<bool> g_tbt_confirmed{false};
bool              g_tbt_signal_seen = false; // guarded by g_decode_mu
std::atomic<uint32_t> g_nav_epoch{0};
uint64_t g_guidance_epoch = 0; // common-client epoch; guarded by g_decode_mu
void *g_procH = nullptr;
void *g_devH  = nullptr;

// Caller holds g_decode_mu, which orders this ownership transition with both
// decoded navigation messages and deactivation. The confirmed latch suppresses
// duplicate broker claims when the HMI signal and nav stream both prove the
// same activation.
void confirm_tbt_locked(const char *source)
{
    if (!g_tbt_confirmed.exchange(true, std::memory_order_acq_rel)) {
        LOGD("carplay TBT confirmed by %s", source);
        hud_session_claim(g_guidance_epoch);
    }
}

extern "C" int eDevMgrRegisterProcess(void *, void *, void *, void *);
extern "C" int eDevMgriPodStartCallStateUpdates(void *, void *, void *, void *);
extern "C" int eDevMgriPodStartCommunicationUpdates(void *, void *, void *, void *);
extern "C" int eDevMgriPodSendLocationInformation(void *, void *, void *, void *);

bool addr_is_own(void *a)
{
    return a == (void *)&eDevMgrRegisterProcess ||
           a == (void *)&eDevMgriPodStartCallStateUpdates ||
           a == (void *)&eDevMgriPodStartCommunicationUpdates ||
           a == (void *)&eDevMgriPodSendLocationInformation;
}

void *resolve_oem(const char *name)
{
    void *p = dlsym(RTLD_NEXT, name);
    if (p && !addr_is_own(p)) return p;
    if (!g_devmgr_handle) {
        g_devmgr_handle = dlopen("libdevmgr_interface.so", RTLD_NOW | RTLD_NOLOAD);
        if (!g_devmgr_handle)
            g_devmgr_handle = dlopen("/usr/lib/libdevmgr_interface.so", RTLD_NOW | RTLD_NOLOAD);
        if (!g_devmgr_handle) { LOGE("resolve_oem: dlopen NOLOAD failed: %s", dlerror()); return nullptr; }
    }
    p = dlsym(g_devmgr_handle, name);
    if (!p) { LOGE("resolve_oem(%s): dlsym failed: %s", name, dlerror()); return nullptr; }
    if (addr_is_own(p)) { LOGC("resolve_oem(%s): interposition trap", name); return nullptr; }
    return p;
}

void resolve_all_once(void)
{
    if (!g_real_register)  g_real_register  = (pfn4)resolve_oem("eDevMgrRegisterProcess");
    if (!g_real_callstate) g_real_callstate = (pfn4)resolve_oem("eDevMgriPodStartCallStateUpdates");
    if (!g_real_commupd)   g_real_commupd   = (pfn4)resolve_oem("eDevMgriPodStartCommunicationUpdates");
    if (!g_real_sendloc)   g_real_sendloc   = (pfn4)resolve_oem("eDevMgriPodSendLocationInformation");
    if (!g_real_startnavi) g_real_startnavi = (pfn4)resolve_oem("eDevMgriPodStartNaviRouteGuidenceUpdates");
}

// Admit an OEM StartNavi call only while `epoch` still owns the published
// process/device handles.  Keep the call gate held through the OEM call so
// deactivate_navigation() either invalidates the session first (and this call
// is rejected), or waits until an already-admitted call has returned.  g_mu is
// released before entering OEM code: holding it there could deadlock with a
// concurrent arm path that owns g_decode_mu.
bool call_startnavi_if_current(uint32_t epoch, int *rc_out,
                               void **proc_out, void **dev_out)
{
    pthread_mutex_lock(&g_startnavi_call_mu);
    pthread_mutex_lock(&g_mu);
    const bool current =
        g_armed.load(std::memory_order_acquire) &&
        g_navi_armed &&
        g_nav_epoch.load(std::memory_order_acquire) == epoch &&
        g_real_startnavi != nullptr && g_procH != nullptr;
    void *procH = current ? g_procH : nullptr;
    void *devH  = current ? g_devH  : nullptr;
    pfn4 startnavi = current ? g_real_startnavi : nullptr;
    pthread_mutex_unlock(&g_mu);

    int rc = 0;
    if (current)
        rc = startnavi(procH, devH, nullptr, nullptr);

    pthread_mutex_unlock(&g_startnavi_call_mu);
    if (rc_out)   *rc_out   = rc;
    if (proc_out) *proc_out = procH;
    if (dev_out)  *dev_out  = devH;
    return current;
}

void *navi_retry_thread(void *arg)
{
    const uint32_t epoch = static_cast<uint32_t>(
        reinterpret_cast<uintptr_t>(arg));

    for (int n = 1; n <= 3; ++n) {
        sleep(5);

        int rc = 0;
        void *procH = nullptr;
        void *devH = nullptr;
        if (!call_startnavi_if_current(epoch, &rc, &procH, &devH))
            return nullptr;
        LOGD("retry#%d StartNaviRouteGuidence(procH=%p devH=%p) rc=%d",
             n, procH, devH, rc);

        pthread_mutex_lock(&g_mu);
        const bool current = g_armed.load(std::memory_order_acquire) &&
                             g_navi_armed &&
                             g_nav_epoch.load(std::memory_order_acquire) == epoch;
        pthread_mutex_unlock(&g_mu);
        if (!current || rc == 1) return nullptr;
    }

    pthread_mutex_lock(&g_mu);
    if (g_nav_epoch.load(std::memory_order_acquire) == epoch)
        g_navi_armed = false;
    pthread_mutex_unlock(&g_mu);
    LOGW("StartNaviRouteGuidence failed after three retries");
    return nullptr;
}

// ===== [NAV-END via OEM CarPlay TBT status] =================================
// The OEM blmjcicarplay.so (this very process) broadcasts the explicit CarPlay
// nav-status on the HMI bus's com.jci.carplay interface — the same event AA gets
// decoded from the AAP SDK. We SUBSCRIBE to it using the OEM's own
// generated client stubs (libjcicarplay_client.so), so the OEM does the match +
// decode for us. Signals of interest:
//   TurnByTurnEntitySignal(entityType u): 0=NONE 1=CARPLAY 2=NATIVE — who owns TBT.
//   SessionDeactiveSignal: CarPlay session ended (e.g. phone unplugged).
// cb ABI (RE-confirmed byte-exact: client dispatcher signal_carplay_* @0xb2d0 +
// consumer svcjcinavi CP_RoutingStatusChanged_cb @0x2e0c4): the worker calls
//   cb(r0=conn, r1=<field0 ptr>, r2=<field1/aux ptr>, r3=userdata)
// with each decoded field delivered BY POINTER (deref r1 for the u32), userdata in
// r3, and r1/r2 freed right after the callback returns (copy out, don't retain).
typedef int (*pfn_cp_enable)(void *conn, int enable, void *cb, void *userdata);
pfn_cp_enable g_cp_tbt_enable      = nullptr;
pfn_cp_enable g_cp_deactive_enable = nullptr;
void *g_cp_hmi_conn   = nullptr;
void *g_cp_client_lib = nullptr;
pthread_mutex_t g_cp_setup_mu = PTHREAD_MUTEX_INITIALIZER;

enum CpStatusState {
    CP_STATUS_IDLE = 0,
    CP_STATUS_CONNECTING,
    CP_STATUS_SUBSCRIBED,
    CP_STATUS_FAILED,
};

std::atomic<unsigned> g_cp_status{CP_STATUS_IDLE};
std::atomic<void *>   g_cp_active_conn{nullptr};

void deactivate_navigation(void *conn = nullptr, bool connection_failed = false)
{
    // Order teardown after any decoder already admitted by msgrcv, then post
    // the release before another decode can enter.  Take the StartNavi gate
    // first: an admitted OEM call may synchronously need g_decode_mu, so waiting
    // for it while already holding g_decode_mu would deadlock.
    pthread_mutex_lock(&g_startnavi_call_mu);
    pthread_mutex_lock(&g_decode_mu);
    // A queued callback must not tear down a session armed on a replacement
    // connection while it waited for these locks. Internal teardown has no conn.
    if (conn && g_cp_active_conn.load(std::memory_order_acquire) != conn) {
        pthread_mutex_unlock(&g_decode_mu);
        pthread_mutex_unlock(&g_startnavi_call_mu);
        return;
    }
    if (connection_failed) {
        void *expected = conn;
        if (!g_cp_active_conn.compare_exchange_strong(
                expected, nullptr, std::memory_order_acq_rel)) {
            pthread_mutex_unlock(&g_decode_mu);
            pthread_mutex_unlock(&g_startnavi_call_mu);
            return;
        }
        g_cp_status.store(CP_STATUS_FAILED, std::memory_order_release);
        LOGW("carplay_status: HMI bus disconnected; releasing HUD guidance");
    }
    pthread_mutex_lock(&g_mu);
    g_armed.store(false, std::memory_order_release);
    g_tbt_confirmed.store(false, std::memory_order_release);
    g_tbt_signal_seen = false;
    g_navi_armed = false;
    g_procH = nullptr;
    g_devH = nullptr;
    const uint64_t guidance_epoch = g_guidance_epoch;
    g_guidance_epoch = 0;
    g_nav_epoch.fetch_add(1, std::memory_order_acq_rel);
    pthread_mutex_unlock(&g_mu);
    nav_request_reset();
    hud_session_end(guidance_epoch);
    pthread_mutex_unlock(&g_decode_mu);
    pthread_mutex_unlock(&g_startnavi_call_mu);
}

void cp_hmi_err(void *conn, void *closure)
{
    (void)closure;
    if (conn) deactivate_navigation(conn, true);
}

// TurnByTurnEntitySignal handler. r1 = uint32_t* entityType (deref).
void cp_tbt_entity_cb(void *conn, void *p_entity, void *p_aux, void *userdata)
{
    (void)p_aux; (void)userdata;
    try {
        unsigned e = p_entity ? *reinterpret_cast<const uint32_t *>(p_entity) : 0xffffffffu;
        LOGD("carplay TBT entity=%u (%s)",
             e, e == 1 ? "CARPLAY" : e == 2 ? "NATIVE" : e == 0 ? "NONE" : "?");
        // entityType 1=CARPLAY (CarPlay owns turn-by-turn = nav active). Anything else — 0=NONE
        // (nav switched off) or 2=NATIVE (the OEM nav took the HUD) — means CarPlay is no longer
        // doing TBT, so release our HUD. Verified live: turning nav off
        // fires entity 1->0.
        pthread_mutex_lock(&g_decode_mu);
        if (!conn || g_cp_active_conn.load(std::memory_order_acquire) != conn) {
            pthread_mutex_unlock(&g_decode_mu);
            return;
        }
        if (!g_armed.load(std::memory_order_acquire)) {
            LOGD("carplay TBT entity=%u ignored (navigation not armed)", e);
            pthread_mutex_unlock(&g_decode_mu);
            return;
        }
        g_tbt_signal_seen = true;
        if (e == 1) {
            confirm_tbt_locked("TurnByTurnEntitySignal");
        } else {
            g_tbt_confirmed.store(false, std::memory_order_release);
            nav_request_reset();
            hud_session_release(
                g_guidance_epoch,
                mazda_cmu::guidance::RELEASE_ROUTE_STOPPED);
        }
        pthread_mutex_unlock(&g_decode_mu);
    } catch (...) { LOGE("cp_tbt_entity_cb exception swallowed (OEM dbus thread)"); }
}

// SessionDeactiveSignal handler. Multi-field payload (clientId,type,reason,...) — we don't
// read the fields; arrival = session gone. DEBUG: LOGD only.
void cp_deactive_cb(void *conn, void *a1, void *a2, void *a3)
{
    (void)a1; (void)a2; (void)a3;
    LOGD("carplay SessionDeactive (session ended)");
    if (conn) deactivate_navigation(conn);
}

void *cp_resolve(const char *name)
{
    void *p = g_cp_client_lib ? dlsym(g_cp_client_lib, name) : nullptr;
    if (!p) p = dlsym(RTLD_DEFAULT, name);
    return p;
}

// Subscribe to the OEM CarPlay TBT-status signals on the HMI bus. Idempotent
// and best-effort.
bool carplay_status_start(void)
{
    pthread_mutex_lock(&g_cp_setup_mu);
    if (g_cp_status.load(std::memory_order_acquire) == CP_STATUS_SUBSCRIBED &&
        g_cp_active_conn.load(std::memory_order_acquire) != nullptr) {
        pthread_mutex_unlock(&g_cp_setup_mu);
        return true;
    }

    // A disconnected worker is cleaned up here, from the caller thread rather
    // than from its own error callback. This also makes a later session retryable.
    if (g_cp_hmi_conn) {
        void *stale = g_cp_hmi_conn;
        g_cp_hmi_conn = nullptr;
        void *expected = stale;
        g_cp_active_conn.compare_exchange_strong(
            expected, nullptr, std::memory_order_acq_rel);
        JCIDBUS_worker_stop(stale);
        JCIDBUS_conn_disconnect(stale);
        JCIDBUS_conn_free(stale);
    }
    g_cp_status.store(CP_STATUS_IDLE, std::memory_order_release);

    // The CARPLAY_*Signal_enable stubs live in the CONSUMER lib libjcicarplay_client.so, which
    // jciCARPLAY (the producer) does not normally load — dlopen it GLOBAL so its JCIDBUS imports
    // resolve against the already-loaded provider.
    if (!g_cp_client_lib) {
        g_cp_client_lib = dlopen("libjcicarplay_client.so", RTLD_NOW | RTLD_GLOBAL);
        if (!g_cp_client_lib)
            g_cp_client_lib = dlopen("/jci/lib/libjcicarplay_client.so", RTLD_NOW | RTLD_GLOBAL);
    }
    if (!g_cp_client_lib) {
        LOGW("carplay_status: dlopen libjcicarplay_client.so failed: %s", dlerror());
        pthread_mutex_unlock(&g_cp_setup_mu);
        return false;
    }
    g_cp_tbt_enable      = (pfn_cp_enable)cp_resolve("CARPLAY_TurnByTurnEntitySignal_enable");
    g_cp_deactive_enable = (pfn_cp_enable)cp_resolve("CARPLAY_SessionDeactiveSignal_enable");
    if (!g_cp_tbt_enable || !g_cp_deactive_enable) {
        LOGW("carplay_status: required signal enable unresolved");
        pthread_mutex_unlock(&g_cp_setup_mu);
        return false;
    }

    void *conn = JCIDBUS_conn_create(reinterpret_cast<void *>(&cp_hmi_err), 0);
    if (!conn) {
        LOGW("carplay_status: HMI conn_create failed");
        pthread_mutex_unlock(&g_cp_setup_mu);
        return false;
    }
    g_cp_hmi_conn = conn;
    g_cp_status.store(CP_STATUS_CONNECTING, std::memory_order_release);
    g_cp_active_conn.store(conn, std::memory_order_release);

    if (JCIDBUS_conn_connect(conn, "com.jci.carplay.hudstatus", kJciHmiBus, nullptr) == 0) {
        LOGW("carplay_status: HMI conn_connect failed");
        void *expected = conn;
        g_cp_active_conn.compare_exchange_strong(
            expected, nullptr, std::memory_order_acq_rel);
        JCIDBUS_conn_free(conn);
        g_cp_hmi_conn = nullptr;
        g_cp_status.store(CP_STATUS_IDLE, std::memory_order_release);
        pthread_mutex_unlock(&g_cp_setup_mu);
        return false;
    }
    int rw = JCIDBUS_worker_start(conn);
    if (rw != 0) {
        LOGW("carplay_status: HMI worker start failed rc=%d", rw);
        void *expected = conn;
        g_cp_active_conn.compare_exchange_strong(
            expected, nullptr, std::memory_order_acq_rel);
        JCIDBUS_conn_disconnect(conn);
        JCIDBUS_conn_free(conn);
        g_cp_hmi_conn = nullptr;
        g_cp_status.store(CP_STATUS_IDLE, std::memory_order_release);
        pthread_mutex_unlock(&g_cp_setup_mu);
        return false;
    }
    int re = g_cp_tbt_enable(conn, 1, reinterpret_cast<void *>(&cp_tbt_entity_cb), nullptr);
    int rd = g_cp_deactive_enable(
        conn, 1, reinterpret_cast<void *>(&cp_deactive_cb), nullptr);
    unsigned expected_state = CP_STATUS_CONNECTING;
    bool subscribed = re == 0 && rd == 0 &&
                      g_cp_active_conn.load(std::memory_order_acquire) == conn &&
                      g_cp_status.compare_exchange_strong(
                          expected_state, CP_STATUS_SUBSCRIBED,
                          std::memory_order_acq_rel);
    // The error callback clears active_conn before changing the status. It may
    // win that pointer transition between our first check and status CAS.
    if (subscribed &&
        g_cp_active_conn.load(std::memory_order_acquire) != conn)
        subscribed = false;
    if (subscribed) {
        LOGD("carplay_status: subscribed HMI TurnByTurnEntity + SessionDeactive");
        pthread_mutex_unlock(&g_cp_setup_mu);
        return true;
    } else {
        LOGW("carplay_status: subscription failed/disconnected tbt=%d deactive=%d", re, rd);
        void *expected = conn;
        g_cp_active_conn.compare_exchange_strong(
            expected, nullptr, std::memory_order_acq_rel);
        JCIDBUS_worker_stop(conn);
        JCIDBUS_conn_disconnect(conn);
        JCIDBUS_conn_free(conn);
        g_cp_hmi_conn = nullptr;
        g_cp_status.store(CP_STATUS_IDLE, std::memory_order_release);
        pthread_mutex_unlock(&g_cp_setup_mu);
        return false;
    }
}

void arm_navi(const char *from, void *procH, void *devH)
{
    // Only run inside the real jciCARPLAY launcher (main.cpp's constructor gate set g_enabled).
    if (!g_enabled) { LOGW("carplay: not enabled (%s)", from); return; }

    bool start = false;
    uint32_t epoch = 0;
    // Match deactivate_navigation's lock order. Publishing g_armed and resetting
    // confirmation must be atomic with respect to both the HMI callback and the
    // msgrcv fallback, which also run under g_decode_mu.
    pthread_mutex_lock(&g_decode_mu);
    pthread_mutex_lock(&g_mu);
    g_armed.store(true, std::memory_order_release);
    g_procH = procH;
    g_devH  = devH;
    if (!g_navi_armed) {
        g_navi_armed = true;
        // Decode may stage maneuver data immediately, but do not publish it
        // until the HMI signal or first admitted nav frame confirms CarPlay.
        g_tbt_confirmed.store(false, std::memory_order_release);
        g_tbt_signal_seen = false;
        epoch = g_nav_epoch.fetch_add(1, std::memory_order_acq_rel) + 1;
        g_guidance_epoch = hud_session_begin();
        start = true;
    }
    pthread_mutex_unlock(&g_mu);
    pthread_mutex_unlock(&g_decode_mu);

    if (!start) return;

    LOGD("arm_navi from %s: StartNaviRouteGuidence(procH=%p devH=%p 0)",
         from, procH, devH);
    if (!carplay_status_start()) {
        LOGW("arm_navi: CarPlay status subscription unavailable; leaving HUD inactive");
        deactivate_navigation();
        return;
    }

    if (!g_real_startnavi) {
        LOGE("arm_navi: StartNavi unresolved");
        pthread_mutex_lock(&g_mu);
        if (g_nav_epoch.load(std::memory_order_acquire) == epoch)
            g_navi_armed = false;
        pthread_mutex_unlock(&g_mu);
        return;
    }

    int rc = 0;
    if (!call_startnavi_if_current(epoch, &rc, nullptr, nullptr)) {
        LOGD("StartNaviRouteGuidence canceled: session epoch %u is no longer current",
             static_cast<unsigned>(epoch));
        return;
    }
    LOGD("StartNaviRouteGuidence rc=%d", rc);
    if (rc == 1) return;

    pthread_mutex_lock(&g_mu);
    bool current = g_armed.load(std::memory_order_acquire) &&
                   g_navi_armed &&
                   g_nav_epoch.load(std::memory_order_acquire) == epoch;
    pthread_mutex_unlock(&g_mu);
    if (!current) return;

    pthread_t th;
    void *arg = reinterpret_cast<void *>(static_cast<uintptr_t>(epoch));
    if (preload_thread_create(&th, navi_retry_thread, arg) == 0) {
        pthread_detach(th);
    } else {
        LOGE("arm_navi: retry thread creation failed");
        pthread_mutex_lock(&g_mu);
        if (g_nav_epoch.load(std::memory_order_acquire) == epoch)
            g_navi_armed = false;
        pthread_mutex_unlock(&g_mu);
    }
}

// ---- msgrcv tap (passive) --------------------------------------------------
typedef long (*pfn_msgrcv)(int, void *, unsigned, long, int);
pfn_msgrcv      g_real_msgrcv = nullptr;

// [DIAGNOSTIC] Defining CARPLAY_NAV_DIAG compiles in a full-payload dump of every
// maneuver(0x8059)+guidance(0x8058) to /data_persist/nav_diag.log — used to RE the
// distance / turn-type / road-name field offsets on a real drive (privacy: the nav
// payload is location data). SHIP build leaves it UNDEFINED (diag off). NOTE the block
// below gates on `#ifdef` = *defined-or-not*, NOT on a 0/1 value — so a
// `#define CARPLAY_NAV_DIAG 0` would WRONGLY keep it ON. To enable for RE, add a bare
// `#define CARPLAY_NAV_DIAG` here (no value); for ship leave it absent.
// [SHIP=OFF, BUILD-19] CARPLAY_NAV_DIAG is NOT defined here so ship builds write NO nav_diag.log.
// For a bench RE build, pass -DCARPLAY_NAV_DIAG on the make line (CXXFLAGS) instead of editing this file,
// so diag can never accidentally ship on. Ship gate: `strings <the .so> | grep nav_diag` must be EMPTY.

// [NAV-END RE] Temporary: log EVERY received message's (time,type,size) + a payload peek for
// the small (<0x300) messages the HUD path skips — to find the iAP2 "navigation stopped"
// signal. AA gets it decoded as a Status event (kTagStatus); the CarPlay raw
// tap filters it out by size. Drive,
// END NAVIGATION (+ unplug), then look at what type appears as the 0x8058/0x8059 stream stops.
// (Superseded by the OEM com.jci.carplay TBT-status subscribe below — no longer needed.)
// SHIP build leaves it UNDEFINED (diag off). Same `#ifdef` trap as CARPLAY_NAV_DIAG above:
// `#define CARPLAY_MSGTYPE_DIAG 0` would WRONGLY keep it ON. To enable for RE, add a bare
// `#define CARPLAY_MSGTYPE_DIAG` here; for ship leave it absent.

void msg_tap(int msqid, const void *msgp, long n)
{
    (void)msqid;
#ifdef CARPLAY_MSGTYPE_DIAG
    if (msgp && n >= 4) {
        static FILE *tf = nullptr;
        if (!tf) { tf = fopen("/data_persist/msgtype_diag.log", "a"); if (tf) setvbuf(tf, nullptr, _IOLBF, 0); }
        if (tf) {
            uint32_t mt = (*(const uint32_t *)msgp) & cp_layout::MessageTypeMask;
            fprintf(tf, "t=%ld mt=0x%04x n=0x%lx", (long)time(nullptr), (unsigned)mt, n);
            if (mt != cp_layout::GuidanceMessageType &&
                mt != cp_layout::ManeuverMessageType) {
                const unsigned char *b = (const unsigned char *)msgp;
                long dn = (n < 40) ? n : 40;
                for (long i = 0; i < dn; i++) fprintf(tf, " %02x", b[i]);
            }
            fprintf(tf, "\n");
        }
    }
#endif
    if (!msgp || n < static_cast<long>(sizeof(uint32_t))) return;
    const uint32_t mt = (*reinterpret_cast<const uint32_t *>(msgp)) &
                        cp_layout::MessageTypeMask;
    const bool is_guidance = mt == cp_layout::GuidanceMessageType;
    const bool is_maneuver = mt == cp_layout::ManeuverMessageType;
    if (!is_guidance && !is_maneuver) return;

    const size_t expected = is_guidance
                                ? cp_layout::GuidanceRecordBytes
                                : cp_layout::ManeuverRecordBytes;
    if (n < static_cast<long>(expected)) {
        LOGW("devmgr: short FW74 CarPlay nav record type=0x%04x "
             "(%ld < %lu), dropped",
             static_cast<unsigned>(mt), n,
             static_cast<unsigned long>(expected));
        return;
    }

    if (!g_tbt_signal_seen) {
        // We are called only after msgrcv admitted this live session under
        // g_decode_mu. The first real nav frame is a fallback ownership proof
        // for firmware that does not replay the current TBT signal to a new
        // subscriber.
        confirm_tbt_locked(is_guidance ? "guidance 0x8058"
                                       : "maneuver 0x8059");
    }
    // Decode maneuver -> HUD (passive read of the received buffer). The shipped
    // build does NO disk logging here: the BUILD-2b probe that appended every nav
    // payload to /data_persist/nav_msgrcv.log + nav_probe_MANEUVER_SEEN.txt was
    // removed (the user's maneuver text is location data; also avoids flash wear).
    nav_on_devmgr_msg(msgp, n, g_guidance_epoch);
#ifdef CARPLAY_NAV_DIAG
    {
        long mt = (*(const long *)msgp) & cp_layout::MessageTypeMask;
        if (mt == static_cast<long>(cp_layout::ManeuverMessageType) ||
            mt == static_cast<long>(cp_layout::GuidanceMessageType)) {
            static FILE *df = nullptr;
            if (!df) { df = fopen("/data_persist/nav_diag.log", "a"); if (df) setvbuf(df, nullptr, _IOLBF, 0); }
            if (df) {
                const unsigned char *b = (const unsigned char *)msgp;
                fprintf(df, "msgrcv n=0x%lx mtype=%ld :", n, *(const long *)msgp);
                for (long i = 0; i < n; i++) fprintf(df, " %02x", b[i]);
                fprintf(df, "\n");
            }
        }
    }
#endif
}

} // namespace

#define PRELOAD_EXPORT __attribute__((visibility("default")))

extern "C" PRELOAD_EXPORT
long msgrcv(int msqid, void *msgp, unsigned msgsz, long msgtyp, int msgflg)
{
    if (!g_real_msgrcv) g_real_msgrcv = (pfn_msgrcv)dlsym(RTLD_NEXT, "msgrcv");
    long n = g_real_msgrcv ? g_real_msgrcv(msqid, msgp, msgsz, msgtyp, msgflg) : -1;
    // Only tap once nav is armed by a live CarPlay session (g_armed set in arm_navi).
    if (g_enabled && n > 0) {
        pthread_mutex_lock(&g_decode_mu);
        if (g_armed.load(std::memory_order_acquire))
            msg_tap(msqid, msgp, n);
        pthread_mutex_unlock(&g_decode_mu);
    }
    return n;
}

extern "C" PRELOAD_EXPORT
int eDevMgrRegisterProcess(void *procStruct, void *role, void *cbTable, void *ctx)
{
    resolve_all_once();
    LOGD("eDevMgrRegisterProcess: procStruct=%p role=%ld cbTable=%p ctx=%p",
         procStruct, (long)role, cbTable, ctx);
    if (!g_real_register) { LOGC("RegisterProcess: real unresolved"); return 0x103; }
    int rc = g_real_register(procStruct, role, cbTable, ctx);
    LOGD("eDevMgrRegisterProcess: rc=%d", rc);
    return rc;
}

extern "C" PRELOAD_EXPORT
int eDevMgriPodStartCallStateUpdates(void *a0, void *a1, void *a2, void *a3)
{
    resolve_all_once();
    LOGD("StartCallStateUpdates: a0=%p a1=%p a2=%p a3=%p", a0, a1, a2, a3);
    int rc = g_real_callstate ? g_real_callstate(a0, a1, a2, a3) : 0x103;
    arm_navi("CallState", a0, a1);
    return rc;
}

extern "C" PRELOAD_EXPORT
int eDevMgriPodStartCommunicationUpdates(void *a0, void *a1, void *a2, void *a3)
{
    resolve_all_once();
    int rc = g_real_commupd ? g_real_commupd(a0, a1, a2, a3) : 0x103;
    arm_navi("CommUpdates", a0, a1);
    return rc;
}

extern "C" PRELOAD_EXPORT
int eDevMgriPodSendLocationInformation(void *a0, void *a1, void *a2, void *a3)
{
    resolve_all_once();
    int rc = g_real_sendloc ? g_real_sendloc(a0, a1, a2, a3) : 0x103;
    arm_navi("SendLocation", a0, a1);
    return rc;
}
