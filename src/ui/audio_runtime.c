/*
 * audio_runtime.c - Desktop host audio device configuration
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "audio_runtime.h"
#include "../system/vs_system.h"
#include <stdio.h>
#include <string.h>

static bool fail(char *error, size_t error_size, const char *message) {
    if (error && error_size) snprintf(error, error_size, "%s", message ? message : "Audio output failed");
    return false;
}

static bool should_pause(const FrontendAudioRuntime *runtime) {
    return runtime && runtime->execution
        && (frontend_execution_paused(runtime->execution) || runtime->execution->rewind_held || runtime->execution->muted);
}

void frontend_audio_runtime_bind(FrontendAudioRuntime *runtime,
                                 SDL_AudioDeviceID *device, SDL_AudioSpec *have) {
    if (!runtime) return;
    memset(runtime, 0, sizeof(*runtime));
    runtime->device = device;
    runtime->have = have;
    snprintf(runtime->backend, sizeof(runtime->backend), "default");
}

void frontend_audio_runtime_set_execution(FrontendAudioRuntime *runtime,
                                          FrontendExecutionRuntime *execution) {
    if (!runtime) return;
    runtime->execution = execution;
    if (execution && runtime->have && runtime->device && *runtime->device)
        execution->audio_output_rate = runtime->have->freq;
}

static bool prepare_audio(FrontendAudioRuntime *runtime, const FrontendSettings *settings,
                          FrontendAudioPrepared *prepared, bool force,
                          char *error, size_t error_size) {
    if (!runtime || !runtime->device || !runtime->have || !settings)
        return fail(error, error_size, "Audio runtime is incomplete");
    if (settings->audio_sample_rate < 8000 || settings->audio_sample_rate > 192000
        || settings->audio_buffer_samples < 64 || settings->audio_buffer_samples > 8192)
        return fail(error, error_size, "Audio rate or buffer size is outside the supported range");

    if (!prepared) return fail(error, error_size, "Audio replacement is missing");
    memset(prepared, 0, sizeof(*prepared));
    const char *requested_name = settings->audio_device[0] ? settings->audio_device : NULL;
    if (!force && *runtime->device && runtime->sample_rate == settings->audio_sample_rate
        && runtime->buffer_samples == settings->audio_buffer_samples
        && strcmp(runtime->device_name, requested_name ? requested_name : "") == 0) {
        if (error && error_size) error[0] = '\0';
        return true;
    }

    SDL_AudioDeviceID old_device = *runtime->device;
    if (old_device) SDL_PauseAudioDevice(old_device, 1);
    SDL_AudioSpec want = {0};
    want.freq = (int)settings->audio_sample_rate;
    want.format = AUDIO_F32;
    want.channels = 2;
    want.samples = (Uint16)settings->audio_buffer_samples;
    want.callback = vs_audio_stereo_callback;
    prepared->device = SDL_OpenAudioDevice(requested_name, 0, &want, &prepared->have, 0);
    if (old_device && !should_pause(runtime)) SDL_PauseAudioDevice(old_device, 0);
    if (!prepared->device) {
        char message[320];
        snprintf(message, sizeof(message), "Could not open audio device: %s", SDL_GetError());
        return fail(error, error_size, message);
    }
    prepared->replacement = true;
    snprintf(prepared->device_name, sizeof(prepared->device_name), "%s",
             requested_name ? requested_name : "");
    prepared->sample_rate = settings->audio_sample_rate;
    prepared->buffer_samples = settings->audio_buffer_samples;
    if (error && error_size) error[0] = '\0';
    return true;
}

bool frontend_audio_runtime_apply(FrontendAudioRuntime *runtime,
                                  const FrontendSettings *settings, bool force,
                                  char *error, size_t error_size) {
    FrontendAudioPrepared prepared = {0};
    if (!prepare_audio(runtime, settings, &prepared, force, error, error_size)) return false;
    frontend_audio_runtime_commit(runtime, &prepared);
    return true;
}

bool frontend_audio_runtime_prepare(FrontendAudioRuntime *runtime,
                                    const FrontendSettings *settings,
                                    FrontendAudioPrepared *prepared,
                                    char *error, size_t error_size) {
    return prepare_audio(runtime, settings, prepared, false, error, error_size);
}

void frontend_audio_runtime_cancel(FrontendAudioPrepared *prepared) {
    if (!prepared) return;
    if (prepared->replacement && prepared->device) SDL_CloseAudioDevice(prepared->device);
    memset(prepared, 0, sizeof(*prepared));
}

void frontend_audio_runtime_commit(FrontendAudioRuntime *runtime,
                                   FrontendAudioPrepared *prepared) {
    if (!runtime || !prepared || !prepared->replacement) return;
    SDL_AudioDeviceID old_device = runtime->device ? *runtime->device : 0;
    if (old_device) {
        SDL_PauseAudioDevice(old_device, 1);
        SDL_CloseAudioDevice(old_device);
    }
    *runtime->device = prepared->device;
    *runtime->have = prepared->have;
    snprintf(runtime->device_name, sizeof(runtime->device_name), "%s", prepared->device_name);
    runtime->sample_rate = prepared->sample_rate;
    runtime->buffer_samples = prepared->buffer_samples;
    prepared->device = 0;
    prepared->replacement = false;
    if (runtime->execution) runtime->execution->audio_output_rate = runtime->have->freq;
    if (runtime->execution) frontend_execution_refresh_audio(runtime->execution);
    else vs_audio_init(runtime->have->freq);
    if (!should_pause(runtime)) SDL_PauseAudioDevice(*runtime->device, 0);
}

void frontend_audio_runtime_shutdown(FrontendAudioRuntime *runtime) {
    if (!runtime) return;
    if (runtime->device && *runtime->device) {
        SDL_CloseAudioDevice(*runtime->device);
        *runtime->device = 0;
    }
    if (runtime->have) memset(runtime->have, 0, sizeof(*runtime->have));
    runtime->execution = NULL;
}

bool frontend_audio_backend_available(const char *backend) {
    if (!backend) return false;
    if (!strcmp(backend, "default")) return true;
    if (strcmp(backend, "wasapi") && strcmp(backend, "directsound")) return false;
    for (int i = 0; i < SDL_GetNumAudioDrivers(); ++i) {
        const char *driver = SDL_GetAudioDriver(i);
        if (driver && !strcmp(driver, backend)) return true;
    }
    return false;
}

const char *frontend_audio_runtime_backend(const FrontendAudioRuntime *runtime) {
    return runtime && runtime->backend[0] ? runtime->backend : "default";
}

static bool restart_driver(FrontendAudioRuntime *runtime, const FrontendSettings *settings,
                           const char *driver, char *error, size_t error_size) {
    if (*runtime->device) {
        SDL_PauseAudioDevice(*runtime->device, 1);
        SDL_CloseAudioDevice(*runtime->device);
        *runtime->device = 0;
    }
    SDL_AudioQuit();
    memset(runtime->have, 0, sizeof(*runtime->have));
    if (SDL_AudioInit(driver && strcmp(driver, "default") ? driver : NULL) != 0)
        return fail(error, error_size, SDL_GetError());
    return frontend_audio_runtime_apply(runtime, settings, true, error, error_size);
}

bool frontend_audio_runtime_select_backend(FrontendAudioRuntime *runtime,
    const FrontendSettings *settings, const char *backend, char *error, size_t error_size) {
    if (!runtime || !runtime->device || !runtime->have || !settings)
        return fail(error, error_size, "Audio runtime is incomplete");
    if (!frontend_audio_backend_available(backend))
        return fail(error, error_size, "Requested native audio driver is not included in this SDL build");
    if (settings->audio_sample_rate < 8000 || settings->audio_sample_rate > 192000 ||
        settings->audio_buffer_samples < 64 || settings->audio_buffer_samples > 8192)
        return fail(error, error_size, "Audio rate or buffer size is outside the supported range");
    if (!strcmp(frontend_audio_runtime_backend(runtime), backend) && *runtime->device)
        return frontend_audio_runtime_apply(runtime, settings, false, error, error_size);

    FrontendSettings previous = *settings;
    if (runtime->sample_rate) previous.audio_sample_rate = runtime->sample_rate;
    if (runtime->buffer_samples) previous.audio_buffer_samples = runtime->buffer_samples;
    snprintf(previous.audio_device, sizeof(previous.audio_device), "%s", runtime->device_name);
    char old_backend[32], old_driver[32], failure[256];
    snprintf(old_backend, sizeof(old_backend), "%s", frontend_audio_runtime_backend(runtime));
    const char *active = SDL_GetCurrentAudioDriver();
    snprintf(old_driver, sizeof(old_driver), "%s", active ? active : "default");
    if (restart_driver(runtime, settings, backend, failure, sizeof(failure))) {
        snprintf(runtime->backend, sizeof(runtime->backend), "%s", backend);
        if (error && error_size) error[0] = 0;
        return true;
    }
    char recovery[256];
    bool restored = restart_driver(runtime, &previous, old_driver, recovery, sizeof(recovery));
    if (!restored) {
        previous.audio_device[0] = 0;
        restored = restart_driver(runtime, &previous, "default", recovery, sizeof(recovery));
        snprintf(runtime->backend, sizeof(runtime->backend), "default");
    } else {
        snprintf(runtime->backend, sizeof(runtime->backend), "%s", old_backend);
    }
    char message[640];
    snprintf(message, sizeof(message), "%s. %s", failure,
             restored ? "Previous or default audio output restored" : "Audio unavailable; select another output to retry");
    return fail(error, error_size, message);
}

bool frontend_audio_runtime_handle_device_event(FrontendAudioRuntime *runtime,
    const FrontendSettings *settings, const SDL_AudioDeviceEvent *event,
    char *error, size_t error_size) {
    if (!runtime || !runtime->device || !settings || !event)
        return fail(error, error_size, "Audio device event is incomplete");
    if (event->iscapture || event->type != SDL_AUDIODEVICEREMOVED || event->which != *runtime->device)
        return true;
    SDL_CloseAudioDevice(*runtime->device);
    *runtime->device = 0;
    FrontendSettings fallback = *settings;
    fallback.audio_device[0] = 0;
    if (frontend_audio_runtime_apply(runtime, &fallback, true, error, error_size)) return true;
    return frontend_audio_runtime_select_backend(runtime, &fallback, "default", error, error_size);
}
