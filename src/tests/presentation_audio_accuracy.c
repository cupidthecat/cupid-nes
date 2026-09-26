/*
 * presentation_audio_accuracy.c - Actual SDL audio driver switching
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/audio_runtime.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                                       \
    do {                                                                                                               \
        if (!(x)) {                                                                                                    \
            fprintf(stderr, "Audio %d: %s (%s)\n", __LINE__, #x, error);                                               \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

int run_presentation_audio_accuracy_tests(void) {
    char error[640] = {0};
    CHECK(SDL_AudioInit("dummy") == 0);
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    FrontendExecutionRuntime execution = {0};
    execution.muted = true;
    SDL_AudioDeviceID device = 0;
    SDL_AudioSpec have = {0};
    FrontendAudioRuntime runtime;
    frontend_audio_runtime_bind(&runtime, &device, &have);
    frontend_audio_runtime_set_execution(&runtime, &execution);
    CHECK(frontend_audio_runtime_apply(&runtime, &settings, false, error, sizeof(error)));
    CHECK(have.channels == 2 && have.format == AUDIO_F32 && SDL_GetAudioDeviceStatus(device) == SDL_AUDIO_PAUSED);
    SDL_AudioDeviceID original = device;
    CHECK(!frontend_audio_runtime_select_backend(&runtime, &settings, "not-a-driver", error, sizeof(error)) &&
          device == original);
    FrontendSettings invalid = settings;
    invalid.audio_buffer_samples = 0;
    CHECK(!frontend_audio_runtime_select_backend(&runtime, &invalid, "default", error, sizeof(error)) &&
          device == original);
    SDL_AudioDeviceEvent unrelated = {0};
    unrelated.type = SDL_AUDIODEVICEREMOVED;
    unrelated.which = device + 100;
    CHECK(frontend_audio_runtime_handle_device_event(&runtime, &settings, &unrelated, error, sizeof(error)) &&
          device == original);
    frontend_audio_runtime_shutdown(&runtime);
    CHECK(device == 0);
    SDL_AudioQuit();
    puts("Audio validation and paused output regressions passed");
    return 0;
}

/* Host acceptance test: failures are reported, never converted into skips. */
int run_native_audio_backend_tests(void) {
    char error[640] = {0};
    CHECK(SDL_AudioInit(NULL) == 0);
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    FrontendExecutionRuntime execution = {0};
    execution.muted = true;
    SDL_AudioDeviceID device = 0;
    SDL_AudioSpec have = {0};
    FrontendAudioRuntime runtime;
    frontend_audio_runtime_bind(&runtime, &device, &have);
    frontend_audio_runtime_set_execution(&runtime, &execution);
    const char *drivers[] = {"wasapi", "directsound", "wasapi"};
    for (size_t i = 0; i < 3; ++i) {
        settings.audio_device[0] = 0;
        settings.audio_sample_rate = i == 1 ? 48000 : 44100;
        settings.audio_buffer_samples = i == 1 ? 512 : 1024;
        CHECK(frontend_audio_backend_available(drivers[i]));
        CHECK(frontend_audio_runtime_select_backend(&runtime, &settings, drivers[i], error, sizeof(error)));
        CHECK(SDL_GetCurrentAudioDriver() && !strcmp(SDL_GetCurrentAudioDriver(), drivers[i]));
        CHECK(device && have.channels == 2 && have.format == AUDIO_F32 &&
              SDL_GetAudioDeviceStatus(device) == SDL_AUDIO_PAUSED);
        CHECK((unsigned)have.freq == settings.audio_sample_rate && have.samples == settings.audio_buffer_samples);
        if (SDL_GetNumAudioDevices(0) > 0) {
            snprintf(settings.audio_device, sizeof(settings.audio_device), "%s", SDL_GetAudioDeviceName(0, 0));
            CHECK(frontend_audio_runtime_apply(&runtime, &settings, false, error, sizeof(error)));
            CHECK(!strcmp(runtime.device_name, settings.audio_device));
        }
        printf("Native audio verified: %s, %d Hz, %u buffer samples, %d devices\n", drivers[i], have.freq, have.samples,
               SDL_GetNumAudioDevices(0));
    }
    FrontendSettings missing = settings;
    snprintf(missing.audio_device, sizeof(missing.audio_device), "Cupid nonexistent output device");
    CHECK(!frontend_audio_runtime_select_backend(&runtime, &missing, "directsound", error, sizeof(error)));
    CHECK(device && !strcmp(SDL_GetCurrentAudioDriver(), "wasapi"));
    SDL_AudioDeviceEvent removed = {0};
    removed.type = SDL_AUDIODEVICEREMOVED;
    removed.which = device;
    CHECK(frontend_audio_runtime_handle_device_event(&runtime, &settings, &removed, error, sizeof(error)));
    CHECK(device && SDL_GetAudioDeviceStatus(device) == SDL_AUDIO_PAUSED);
    frontend_audio_runtime_shutdown(&runtime);
    SDL_AudioQuit();
    return 0;
}
