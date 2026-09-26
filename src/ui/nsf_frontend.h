/*
 * nsf_frontend.h - Frontend helpers for NSF and NSFe track controls
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef NSF_FRONTEND_H
#define NSF_FRONTEND_H

#include <stdbool.h>

typedef void (*NsfFrontendAudioGuard)(void *context);

bool nsf_frontend_step_track(int direction,
                             NsfFrontendAudioGuard lock_audio,
                             NsfFrontendAudioGuard unlock_audio,
                             void *audio_context,
                             unsigned *selected_track);

#endif
