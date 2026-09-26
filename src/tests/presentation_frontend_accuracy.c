/*
 * presentation_frontend_accuracy.c - Idle tools, settings bridge and unload lifecycle
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/presentation_host.h"
#include "../ui/frontend_panels.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                                       \
    do {                                                                                                               \
        if (!(x)) {                                                                                                    \
            fprintf(stderr, "Presentation frontend %d: %s (%s)\n", __LINE__, #x, error);                               \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

int run_presentation_frontend_accuracy_tests(void) {
    char error[1024] = {0};
    CHECK(SDL_setenv("SDL_VIDEODRIVER", "dummy", 1) == 0);
    CHECK(SDL_InitSubSystem(SDL_INIT_VIDEO) == 0 && SDL_AudioInit("dummy") == 0);
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    SDL_Surface *surface = SDL_CreateRGBSurfaceWithFormat(0, 256, 240, 32, SDL_PIXELFORMAT_ARGB8888);
    CHECK(surface);
    SDL_Renderer *renderer = SDL_CreateSoftwareRenderer(surface);
    CHECK(renderer);
    FrontendVideoRuntime video;
    CHECK(frontend_video_runtime_init(&video, renderer, &settings, false, error, sizeof(error)));
    CHECK(video.builder && video.shader && !nes_shader_enabled(video.shader));
    SDL_AudioDeviceID device = 0;
    SDL_AudioSpec have = {0};
    FrontendAudioRuntime audio;
    frontend_audio_runtime_bind(&audio, &device, &have);
    FrontendExecutionRuntime execution = {0};
    execution.muted = true;
    frontend_audio_runtime_set_execution(&audio, &execution);
    frontend_panels_reset();
    FrontendPresentationHost host;
    CHECK(frontend_presentation_host_init(&host, &video, &audio, &execution, NULL, NULL));
    FrontendPanelInfo info;
    CHECK(frontend_panel_count() == 5 && frontend_panel_get(0x2740, &info) && !info.enabled);
    CHECK(frontend_panel_get(0x2780, &info) && info.enabled);
    FrontendPanelControl controls[16];
    FrontendPanelModel model = {.controls = controls, .capacity = 16};
    CHECK(frontend_panel_snapshot(0x2780, &model, error, sizeof(error)) && model.count == 8);
    CHECK(frontend_panel_snapshot(0x2720, &model, error, sizeof(error)) && model.count == 2);
    CHECK(frontend_presentation_restore_settings(&video, &audio, &settings, error, sizeof(error)));
    snprintf(settings.shader_path, sizeof(settings.shader_path), "build/does-not-exist.glslp");
    settings.shader_enabled = true;
    CHECK(!frontend_presentation_restore_settings(&video, &audio, &settings, error, sizeof(error)));
    CHECK(error[0] && !nes_shader_enabled(video.shader));
    CHECK(!frontend_panel_action(0x2780, 0x2785, NULL, 1, error, sizeof(error)) && !nes_shader_enabled(video.shader));
    CHECK(!frontend_panel_action(0x2780, 0x2784, NULL, 0, error, sizeof(error)) && !nes_shader_enabled(video.shader));
    CHECK(frontend_presentation_capture_settings(&video, &audio, &settings));
    CHECK(!settings.shader_enabled && !strcmp(settings.shader_path, "build/does-not-exist.glslp"));
    settings.shader_path[0] = 0;
    CHECK(frontend_presentation_restore_settings(&video, &audio, &settings, error, sizeof(error)));
    host.frequency = 1000;
    CHECK(frontend_presentation_host_record(&host, 100, 110, 115, 116, 120));
    NesFrameTimingSummary summary;
    CHECK(nes_frame_timing_summary(&host.timing.timing, &summary) && summary.fps == 50 &&
          summary.metrics[NES_HOST_EMULATION].average == 10 && summary.metrics[NES_HOST_PRESENT].average == 1);
    frontend_presentation_host_reset(&host);
    CHECK(host.previous_end == 0 && host.timing.timing.count == 0);
    NesShaderPreset *shader = video.shader;
    frontend_video_runtime_unload_image(&video);
    CHECK(video.shader == shader && video.builder && !video.texture && !video.source_pixels && !video.builder_preview);
    frontend_presentation_host_tick(&host);
    frontend_presentation_host_shutdown(&host);
    CHECK(frontend_panel_count() == 0);
    frontend_audio_runtime_shutdown(&audio);
    frontend_video_runtime_shutdown(&video);
    SDL_DestroyRenderer(renderer);
    SDL_FreeSurface(surface);
    SDL_AudioQuit();
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
    puts("Presentation idle panels, settings failure fallback, host timing and unload passed");
    return 0;
}
