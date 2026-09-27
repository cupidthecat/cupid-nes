/*
 * gtk_benchmark.c - Measure live game presentation at desktop window sizes
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/gtk_internal.h"
#include "../ui/gtk_desktop.h"
#include "../ui/frontend_commands.h"
#include "../ui/debug_frontend.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../apu/apu.h"
#include "../rom/rom.h"
#include "../system/vs_system.h"
#include "../debugger/debugger.h"
#include "../debugger/ppu_inspector.h"
#include "../video/frame_timing.h"
#include <stdio.h>
#include <stdlib.h>

static void settle(FrontendDesktopUi *ui) {
    for (unsigned i = 0; i < 30; ++i) {
        frontend_desktop_render(ui, 256, 240, "Presentation benchmark", "NTSC", "Running");
        SDL_Delay(10);
    }
}

int benchmark_gtk(const char *path) {
    uint8_t *image = NULL;
    size_t size = 0;
    if (nes_file_read_all(path, 64u * 1024u * 1024u, &image, &size) != NES_FILE_OK) {
        return 2;
    }
    int loaded = load_rom_memory(image, size);
    free(image);
    if (loaded || SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_AUDIO) != 0) {
        return 2;
    }
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    if (!cpu_power_on(&cpu)) {
        return 2;
    }
    debugger_init();
    frontend_commands_reset();
    frontend_panels_reset();
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    settings.pause_on_focus_loss = false;
    settings.show_fps = true;
    SDL_AudioSpec want = {0}, have;
    want.freq = 44100;
    want.format = AUDIO_F32SYS;
    want.channels = 2;
    want.samples = 1024;
    want.callback = vs_audio_stereo_callback;
    SDL_AudioDeviceID audio = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    if (!audio) {
        return 2;
    }
    vs_audio_init(have.freq);
    FrontendExecutionRuntime execution;
    frontend_execution_init(&execution, &audio, have.freq, NULL, NULL, NULL, NULL);
    frontend_execution_set_rewind_seconds(&execution, settings.rewind_seconds);
    SDL_Window *window = SDL_CreateWindow("Presentation benchmark", 0, 0, 900, 700, SDL_WINDOW_HIDDEN);
    SDL_Renderer *renderer = window ? SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE) : NULL;
    if (!renderer) {
        return 2;
    }
    FrontendVideoRuntime video;
    char error[256] = {0};
    if (!frontend_video_runtime_init(&video, renderer, &settings, false, error, sizeof(error))) {
        return 2;
    }
    FrontendDesktopUi ui;
    frontend_desktop_init(&ui, window, renderer, &settings, &execution, NULL, NULL);
    if (!ui.gtk) {
        return 2;
    }
    ui.video = &video;
    frontend_panel_set_session_active(true);
    frontend_command_set_session_active(true);
    DebugFrontend *debug = debug_frontend_create(&execution);
    if (!debug || !debug_frontend_register_ui(debug)) {
        return 2;
    }
    SDL_PauseAudioDevice(audio, 0);
    const char *names[] = {"window", "maximized", "fullscreen", "maximized-ppu"};
    double frequency = (double)SDL_GetPerformanceFrequency();
    int result = 0;
    for (unsigned mode = 0; mode < G_N_ELEMENTS(names); ++mode) {
        settings.fullscreen = mode == 2;
        settle(&ui);
        if (mode == 1 || mode == 3) {
            gtk_window_maximize(GTK_WINDOW(ui.gtk->window));
        }
        if (mode == 3) {
            cupid_gtk_open(&ui, 1, DEBUG_PPU_PATTERNS);
        }
        settle(&ui);
        uint64_t begin = 0;
        double core = 0, presentation = 0;
        unsigned frames = 300;
        for (unsigned i = 0; i < frames + 60; ++i) {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
            }
            if (i == 60) {
                begin = SDL_GetPerformanceCounter();
            }
            uint64_t start = SDL_GetPerformanceCounter();
            if (!frontend_execution_run_frame(&execution)) {
                result = 1;
                break;
            }
            uint64_t emulated = SDL_GetPerformanceCounter();
            if (!frontend_video_runtime_refresh(&video, error, sizeof(error))) {
                result = 1;
                break;
            }
            frontend_desktop_render(&ui, 256, 240, "Presentation benchmark", "NTSC", "Running");
            uint64_t end = SDL_GetPerformanceCounter();
            if (i >= 60) {
                core += (emulated - start) / frequency;
                presentation += (end - emulated) / frequency;
            }
        }
        if (result) {
            break;
        }
        double elapsed = (SDL_GetPerformanceCounter() - begin) / frequency;
        GskRenderer *gsk = gtk_native_get_renderer(GTK_NATIVE(ui.gtk->window));
        printf("GTK benchmark: %s %dx%d, %s, %u frames, %.1f fps, core %.2f ms, presentation %.2f ms\n", names[mode],
               gtk_widget_get_width(ui.gtk->picture), gtk_widget_get_height(ui.gtk->picture), G_OBJECT_TYPE_NAME(gsk),
               frames, frames / elapsed, core * 1000 / frames, presentation * 1000 / frames);
        fflush(stdout);
        /* Repeat with the production deadline calculation. Throughput above
         * 60 does not by itself establish stable normal-speed playback. */
        begin = SDL_GetPerformanceCounter();
        double deadline = (double)begin;
        double last_present = 0;
        unsigned presented = 0;
        for (unsigned i = 0; i < frames; ++i) {
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
            }
            uint64_t cycles = cpu_total_cycles;
            if (!frontend_execution_run_frame(&execution) ||
                !frontend_video_runtime_refresh(&video, error, sizeof(error))) {
                result = 1;
                break;
            }
            deadline += (cpu_total_cycles - cycles) * frequency / nes_timing()->cpu_hz;
            double now = (double)SDL_GetPerformanceCounter();
            if (nes_frame_timing_present(now, deadline, last_present, frequency)) {
                frontend_desktop_render(&ui, 256, 240, "Presentation benchmark", "NTSC", "Running");
                last_present = now;
                ++presented;
            }
            double remaining = deadline - SDL_GetPerformanceCounter();
            if (remaining > 0) {
                SDL_Delay((Uint32)(remaining * 1000 / frequency));
            }
        }
        elapsed = (SDL_GetPerformanceCounter() - begin) / frequency;
        printf("GTK paced: %s, %.2f fps (target %.2f), %u/%u presentations\n", names[mode], frames / elapsed, nes_timing()->fps, presented, frames);
        fflush(stdout);
        if (result) {
            break;
        }
    }
    SDL_PauseAudioDevice(audio, 1);
    frontend_desktop_shutdown(&ui);
    debug_frontend_destroy(debug);
    frontend_execution_shutdown(&execution);
    frontend_video_runtime_shutdown(&video);
    SDL_CloseAudioDevice(audio);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    unload_rom();
    SDL_Quit();
    if (result) {
        fprintf(stderr, "Benchmark failed: %s\n", error);
    }
    return result;
}
