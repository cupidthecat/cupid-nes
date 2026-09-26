/*
 * audio_runtime.h - Desktop host audio device configuration
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_AUDIO_RUNTIME_H
#define FRONTEND_AUDIO_RUNTIME_H

#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stddef.h>
#include "frontend_execution.h"
#include "settings.h"

typedef struct {
    SDL_AudioDeviceID *device;
    SDL_AudioSpec *have;
    FrontendExecutionRuntime *execution;
    char device_name[256];
    unsigned sample_rate;
    unsigned buffer_samples;
    char backend[32];
} FrontendAudioRuntime;

typedef struct {
    SDL_AudioDeviceID device;
    SDL_AudioSpec have;
    char device_name[256];
    unsigned sample_rate;
    unsigned buffer_samples;
    bool replacement;
} FrontendAudioPrepared;

void frontend_audio_runtime_bind(FrontendAudioRuntime *runtime,
                                 SDL_AudioDeviceID *device, SDL_AudioSpec *have);
void frontend_audio_runtime_set_execution(FrontendAudioRuntime *runtime,
                                          FrontendExecutionRuntime *execution);
bool frontend_audio_runtime_apply(FrontendAudioRuntime *runtime,
                                  const FrontendSettings *settings, bool force,
                                  char *error, size_t error_size);
bool frontend_audio_runtime_prepare(FrontendAudioRuntime *runtime,
                                    const FrontendSettings *settings,
                                    FrontendAudioPrepared *prepared,
                                    char *error, size_t error_size);
void frontend_audio_runtime_cancel(FrontendAudioPrepared *prepared);
void frontend_audio_runtime_commit(FrontendAudioRuntime *runtime,
                                   FrontendAudioPrepared *prepared);
void frontend_audio_runtime_shutdown(FrontendAudioRuntime *runtime);

/* SDL's native output drivers consume the same final stereo mix. Backend
 * changes reopen the global SDL audio subsystem; call on the host thread. */
bool frontend_audio_backend_available(const char *backend);
const char *frontend_audio_runtime_backend(const FrontendAudioRuntime *runtime);
bool frontend_audio_runtime_select_backend(FrontendAudioRuntime *runtime,
    const FrontendSettings *settings, const char *backend, char *error, size_t error_size);
bool frontend_audio_runtime_handle_device_event(FrontendAudioRuntime *runtime,
    const FrontendSettings *settings, const SDL_AudioDeviceEvent *event,
    char *error, size_t error_size);

#endif
