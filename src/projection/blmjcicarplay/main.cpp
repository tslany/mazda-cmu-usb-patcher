// SPDX-License-Identifier: AGPL-3.0-or-later
//
// CarPlay -> HUD bridge for Mazda Connect (CMU150).
// Copyright (C) 2026 KID MIXER-MODER.
//
// Part of an AGPL-3.0 work - see NOTICE.md for full attribution.
//
// Patch entry point for the CarPlay->HUD bridge. Runs at LD_PRELOAD time,
// before sm_svclauncher dlopen's /jci/carplay/blmjcicarplay.so.
//
// Self-gate: unlike blmjciaapa.so (which exports GetServiceInterfaces and can
// be probed with dlopen+dlsym), blmjcicarplay.so exports no usable anchor, so
// we gate on /proc/self/cmdline instead — the launcher is invoked as
//   {L_jciCARPLAY} /jci/sm/sm_svclauncher -l jciCARPLAY /jci/carplay/blmjcicarplay.so 0 -a
// If we are not in that PID we stay completely inert (the PLT shims in
// devmgr_shim.cpp still chain straight through to the real libdevmgr_interface
// impls, so any other process that happens to inherit LD_PRELOAD is unaffected).

#include "patch.h"
#include "hud/hud_send.h"
#define LIBPATCH_NAME "blmjcicarplay"
#include "common/preload_guard.h"

#include <dlfcn.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

bool g_enabled = false;
std::FILE *g_logf = nullptr;

namespace {

static int read_cmdline(char *out, size_t outsz)
{
    if (!out || outsz == 0) return 0;
    out[0] = '\0';
    int fd = open("/proc/self/cmdline", O_RDONLY | O_CLOEXEC);
    if (fd < 0) return 0;
    ssize_t n = read(fd, out, outsz - 1);
    close(fd);
    if (n <= 0) { out[0] = '\0'; return 0; }
    for (ssize_t i = 0; i < n; ++i) if (out[i] == '\0') out[i] = ' ';
    out[n] = '\0';
    return (int)n;
}

bool cmdline_has_exact_token(const char *cmdline, const char *wanted)
{
    if (!cmdline || !wanted) return false;
    const size_t wanted_len = strlen(wanted);
    const char *p = cmdline;
    while (*p) {
        while (*p == ' ') ++p;
        const char *end = strchr(p, ' ');
        const size_t len = end ? static_cast<size_t>(end - p) : strlen(p);
        if (len == wanted_len && memcmp(p, wanted, len) == 0) return true;
        if (!end) break;
        p = end + 1;
    }
    return false;
}

bool cmdline_is_sm_launcher(const char *cmdline)
{
    if (!cmdline || !*cmdline) return false;
    const char *end = strchr(cmdline, ' ');
    const size_t length = end ? static_cast<size_t>(end - cmdline)
                              : strlen(cmdline);
    const char *base = cmdline;
    for (size_t i = 0; i < length; ++i) {
        if (cmdline[i] == '/') base = cmdline + i + 1;
    }
    const size_t base_length =
        length - static_cast<size_t>(base - cmdline);
    static const char expected[] = "sm_svclauncher";
    return base_length == sizeof(expected) - 1 &&
           memcmp(base, expected, sizeof(expected) - 1) == 0;
}

} // namespace

// Exposed for devmgr_shim.cpp's lazy gate.
bool in_carplay_launcher(void)
{
    char cmdline[512];
    read_cmdline(cmdline, sizeof(cmdline));
    // Require the real service launcher plus both exact identity arguments.
    // The prior substring/OR gate also matched inherited watchdog commands such
    // as `killall jciCARPLAY`, which could start a second process-long client.
    return cmdline_is_sm_launcher(cmdline) &&
           cmdline_has_exact_token(cmdline, "jciCARPLAY") &&
           cmdline_has_exact_token(cmdline,
                                   "/jci/carplay/blmjcicarplay.so");
}

__attribute__((constructor))
static void on_load(void)
{
    if (!in_carplay_launcher()) {
        // Not our PID (could be the once-a-second killall watchdog child that
        // inherits LD_PRELOAD). Stay silent + inert.
        return;
    }
    g_enabled = true;
    // Open the bring-up log file (stderr is swallowed by syslog-ng). Append so
    // multiple boots accumulate; line-buffered so tail -f sees lines live.
    g_logf = std::fopen("/tmp/carplay_bridge.log", "a");
    if (g_logf) std::setvbuf(g_logf, nullptr, _IOLBF, 0);
    preload_install_fatal_handler();
    if (!hud_send_start()) {
        // The OEM CarPlay service remains fully functional. The common client
        // is fail-open and can only affect optional projected HUD guidance.
        LOGE("guidance IPC client failed to start; CarPlay HUD disabled");
    }
    LOGD("=== loaded into jciCARPLAY launcher pid=%d ppid=%d (CarPlay->HUD bridge, FW 74.00.324) ===",
         (int)getpid(), (int)getppid());
}
