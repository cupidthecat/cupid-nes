/* Production frontend throughput diagnostic. SPDX-License-Identifier: GPL-3.0-or-later */
#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../debugger/debugger.h"
#include "../debugger/ppu_inspector.h"
#include "../capture/capture_writer.h"
#include "../ppu/ppu.h"
#include "../rom/rom.h"
#include "../system/vs_system.h"
#include "../ui/desktop_ui.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_execution.h"
#include "../ui/frontend_panels.h"
#include "../ui/settings.h"
#include <stdio.h>
#include <stdlib.h>

int benchmark_frontend(unsigned frames, const char *path) {
    if (!frames || frames > 100000) {
        return 2;
    }
    uint8_t *image = NULL;
    size_t size = 0;
    if (nes_file_read_all(path, 64u * 1024u * 1024u, &image, &size) != NES_FILE_OK) {
        return 2;
    }
    SDL_setenv("SDL_AUDIODRIVER", "dummy", 1);
    SDL_setenv("SDL_VIDEODRIVER", "dummy", 1);
    if (SDL_InitSubSystem(SDL_INIT_AUDIO | SDL_INIT_TIMER | SDL_INIT_VIDEO) != 0) {
        free(image);
        return 2;
    }
    SDL_AudioSpec want = {0}, have;
    want.freq = 44100;
    want.format = AUDIO_F32SYS;
    want.channels = 2;
    want.samples = 1024;
    want.callback = vs_audio_stereo_callback;
    SDL_AudioDeviceID device = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    if (!device) {
        free(image);
        SDL_Quit();
        return 2;
    }
    int result = 0;
    for (unsigned seconds = 0; seconds <= 10; seconds += 10) {
        if (load_rom_memory(image, size)) {
            result = 2;
            break;
        }
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        cpu_total_cycles = 0;
        if (!cpu_power_on(&cpu)) {
            result = 2;
            break;
        }
        debugger_init();
        frontend_commands_reset();
        frontend_panels_reset();
        FrontendExecutionRuntime runtime;
        frontend_execution_init(&runtime, &device, have.freq, NULL, NULL, NULL, NULL);
        frontend_execution_set_rewind_seconds(&runtime, seconds);
        vs_audio_init(have.freq);
        SDL_PauseAudioDevice(device, 0);
        uint64_t begin = SDL_GetPerformanceCounter();
        for (unsigned i = 0; i < frames; ++i) {
            if (!frontend_execution_run_frame(&runtime)) {
                result = 1;
                break;
            }
        }
        double elapsed = (double)(SDL_GetPerformanceCounter() - begin) / SDL_GetPerformanceFrequency();
        printf("Frontend benchmark: rewind=%us, %u frames, %.3fs, %.2f ms/frame, %.1f fps, history=%zu bytes\n",
               seconds, frames, elapsed, elapsed * 1000 / frames, frames / elapsed, nes_rewind_bytes(&runtime.rewind));
        frontend_execution_shutdown(&runtime);
        SDL_PauseAudioDevice(device, 1);
        if (seconds < 10) {
            unload_rom();
        }
        if (result) {
            break;
        }
    }
    SDL_Window *window = SDL_CreateWindow("Frontend benchmark", 0, 0, 768, 720, SDL_WINDOW_HIDDEN);
    SDL_Renderer *renderer = window ? SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE) : NULL;
    if (renderer) {
        FrontendSettings settings;
        frontend_settings_defaults(&settings);
        FrontendDesktopUi ui;
        frontend_commands_reset();
        frontend_desktop_init(&ui, window, renderer, &settings, NULL, NULL, NULL);
        frontend_desktop_register_commands(&ui);
        frontend_desktop_render(&ui, 256, 240, "Benchmark", "NTSC", "Running");
        SDL_RenderPresent(renderer);
        uint64_t begin = SDL_GetPerformanceCounter();
        for (unsigned i = 0; i < frames; ++i) {
            frontend_desktop_render(&ui, 256, 240, "Benchmark", "NTSC", "Running");
            SDL_RenderPresent(renderer);
        }
        double elapsed = (double)(SDL_GetPerformanceCounter() - begin) / SDL_GetPerformanceFrequency();
        printf("Desktop renderer (software): %.3f ms/frame over %u frames\n", elapsed * 1000 / frames, frames);
        DebugPpuImage *ppu_image = malloc(sizeof(*ppu_image));
        uint32_t *graphics = malloc(768 * 720 * sizeof(*graphics));
        if (ppu_image && graphics && !result) {
            begin = SDL_GetPerformanceCounter();
            for (unsigned i = 0; i < frames; ++i) {
                debug_ppu_capture(ppu_image, true);
                debug_ppu_nametables(ppu_image, false, graphics);
            }
            elapsed = (double)(SDL_GetPerformanceCounter() - begin) / SDL_GetPerformanceFrequency();
            printf("PPU snapshot and four nametables: %.3f ms/update over %u updates\n", elapsed * 1000 / frames,
                   frames);
            frontend_panel_set_session_active(true);
            ui.panel_open = true;
            for (unsigned id = DEBUG_PPU_PATTERNS; id <= DEBUG_PPU_PALETTE; ++id) {
                ui.panel_id = id;
                SDL_SetRenderDrawColor(renderer, 24, 28, 39, 255);
                SDL_RenderClear(renderer);
                frontend_desktop_render(&ui, 256, 240, "Benchmark", "NTSC", "Running");
                if (!SDL_RenderReadPixels(renderer, NULL, SDL_PIXELFORMAT_ARGB8888, graphics, 768 * 4)) {
                    char output[128];
                    snprintf(output, sizeof(output), "build/ppu-benchmark-%u.png", id - DEBUG_PPU_PATTERNS);
                    NesCaptureFrame frame = {graphics, 768, 720, 768};
                    if (nes_capture_png(output, &frame) != NES_FILE_OK) {
                        result = 2;
                    }
                } else {
                    result = 2;
                }
            }
        } else {
            result = 2;
        }
        free(ppu_image);
        free(graphics);
        frontend_desktop_shutdown(&ui);
        SDL_DestroyRenderer(renderer);
    } else {
        result = 2;
    }
    SDL_DestroyWindow(window);
    unload_rom();
    free(image);
    SDL_CloseAudioDevice(device);
    apu_audio_shutdown_state(&apu);
    debugger_shutdown();
    SDL_Quit();
    return result;
}
