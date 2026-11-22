// SPDX-License-Identifier: AGPL-3.0-or-later
//
// Session-lifecycle entry points for the mute -> phone play/pause bridge.

#ifndef LIBPATCH_BLMJCIAAPA_PLAYPAUSE_PLAYPAUSE_H
#define LIBPATCH_BLMJCIAAPA_PLAYPAUSE_PLAYPAUSE_H

// Defined in playpause.cpp. Spawns / joins the com.jci.vbs.am mute watcher
// thread that pauses the phone's Android Auto media on the user mute
// button and resumes it on unmute. Start after successful aap_start_session;
// stop before SDK stop/destroy so a pause latch cannot cross phone sessions.
void playpause_start(void);
void playpause_stop(void);

#endif // LIBPATCH_BLMJCIAAPA_PLAYPAUSE_PLAYPAUSE_H
