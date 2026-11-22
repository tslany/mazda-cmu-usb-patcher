// SPDX-License-Identifier: AGPL-3.0-or-later
//
// CarPlay -> HUD bridge for Mazda Connect (CMU150).
// Copyright (C) 2026 KID MIXER-MODER.
//
// Part of an AGPL-3.0 work - see NOTICE.md for full attribution.
//
// Shared globals, logging, and cross-TU decls for the CarPlay->HUD bridge.
//
// This patch taps the iAP2 device-manager client API used by jciCARPLAY. Its
// PLT shims capture the live process/device handles and request Apple navigation
// updates; a passive msgrcv interposer decodes the delivered FW74 records. The
// selected maneuver becomes one complete Mazda-domain snapshot and is relayed
// to the process-long central broker in jciAAPA.

#ifndef LIBPATCH_BLMJCICARPLAY_PATCH_H
#define LIBPATCH_BLMJCICARPLAY_PATCH_H

#include <stdint.h>
#include <cstdio>

// Master enable. Set by main.cpp once the self-gate confirms we are in the
// {L_jciCARPLAY} launcher PID.
extern bool g_enabled;

// Bring-up log sink. jciCARPLAY's stderr is swallowed by syslog-ng (routed to
// console/dropped, not a file we can read over SSH), so main.cpp opens this
// file in /tmp on load and every LOGx goes here. NULL -> fall back to stderr.
extern std::FILE *g_logf;

// ---- logging (stderr -> captured by sm into the device log) ----------------
#define LOG_LEVEL_VERBOSE  0
#define LOG_LEVEL_DEBUG    1
#define LOG_LEVEL_WARN     2
#define LOG_LEVEL_ERROR    3
#define LOG_LEVEL_CRITICAL 4
#define LOG_LEVEL_SILENT   5

#ifndef LOG_LEVEL
#  ifdef NDEBUG
#    define LOG_LEVEL LOG_LEVEL_ERROR   /* release: only LOGE/LOGC; no maneuver-text/location chatter to /tmp */
#  else
#    define LOG_LEVEL LOG_LEVEL_VERBOSE
#  endif
#endif

#define LOG_EMIT(tag, fmt, ...)                                                  \
    do {                                                                         \
        std::FILE *_lf = g_logf ? g_logf : stderr;                               \
        std::fprintf(_lf, "[libpatch-blmjcicarplay][" tag "] " fmt "\n",         \
                     ##__VA_ARGS__);                                             \
        std::fflush(_lf);                                                         \
    } while (0)

#define LOG_DROP(fmt, ...)                                                       \
    do { if (false) { std::fprintf(stderr, "[libpatch-blmjcicarplay] " fmt "\n", \
                                   ##__VA_ARGS__); } } while (0)

#if LOG_LEVEL <= LOG_LEVEL_VERBOSE
#  define LOGV(fmt, ...) LOG_EMIT("V", fmt, ##__VA_ARGS__)
#else
#  define LOGV(fmt, ...) LOG_DROP(fmt, ##__VA_ARGS__)
#endif
#if LOG_LEVEL <= LOG_LEVEL_DEBUG
#  define LOGD(fmt, ...) LOG_EMIT("D", fmt, ##__VA_ARGS__)
#else
#  define LOGD(fmt, ...) LOG_DROP(fmt, ##__VA_ARGS__)
#endif
#if LOG_LEVEL <= LOG_LEVEL_WARN
#  define LOGW(fmt, ...) LOG_EMIT("W", fmt, ##__VA_ARGS__)
#else
#  define LOGW(fmt, ...) LOG_DROP(fmt, ##__VA_ARGS__)
#endif
#if LOG_LEVEL <= LOG_LEVEL_ERROR
#  define LOGE(fmt, ...) LOG_EMIT("E", fmt, ##__VA_ARGS__)
#else
#  define LOGE(fmt, ...) LOG_DROP(fmt, ##__VA_ARGS__)
#endif
#if LOG_LEVEL <= LOG_LEVEL_CRITICAL
#  define LOGC(fmt, ...) LOG_EMIT("C", fmt, ##__VA_ARGS__)
#else
#  define LOGC(fmt, ...) LOG_DROP(fmt, ##__VA_ARGS__)
#endif

// ---- nav message decoder (nav.cpp) -----------------------------------------
// Called from the msgrcv tap for every devmgr message. Decodes iAP2 nav
// maneuver messages (mtype 0x8059) into the complete Mazda-domain snapshot for
// the current common-client epoch. Passive: only reads the already-received
// OEM buffer.
extern "C" void nav_on_devmgr_msg(const void *msgp, long n,
                                  uint64_t guidance_epoch);

// Reset the route-window selection/dedup state at the next admitted record.
// All callers serialize this edge with the decoder through g_decode_mu.
extern "C" void nav_request_reset(void);

// ---- self-gate (main.cpp) --------------------------------------------------
// True when /proc/self/cmdline shows we are the jciCARPLAY launcher.
bool in_carplay_launcher(void);

#endif // LIBPATCH_BLMJCICARPLAY_PATCH_H
