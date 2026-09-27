/*
 * nsf_frontend.c - Frontend helpers for NSF and NSFe track controls
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "nsf_frontend.h"
#include "../rom/rom.h"

bool nsf_frontend_step_track(int direction,
                             NsfFrontendAudioGuard lock_audio,
                             NsfFrontendAudioGuard unlock_audio,
                             void *audio_context,
                             unsigned *selected_track) {
    if ((direction != -1 && direction != 1) || !rom_is_nsf()) return false;
    const NsfMetadata *music = rom_nsf_metadata();
    if (!music || !music->total_songs) return false;

    unsigned track = rom_nsf_current_track();
    if (direction > 0) track = (track + 1u) % music->total_songs;
    else track = track ? track - 1u : music->total_songs - 1u;

    if (lock_audio) lock_audio(audio_context);
    bool changed = rom_nsf_select_track(track);
    if (unlock_audio) unlock_audio(audio_context);
    if (changed && selected_track) *selected_track = track;
    return changed;
}
