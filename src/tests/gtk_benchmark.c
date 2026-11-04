/*
 * gtk_benchmark.c - Measure live game presentation at desktop window sizes
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/gtk_internal.h"
#include "../ui/gtk_desktop.h"
#include "../ui/frontend_commands.h"
#include "../ui/debug_frontend.h"
#include "../ui/tas_frontend.h"
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
#include <math.h>

static void settle(FrontendDesktopUi *ui) {
    for (unsigned i = 0; i < 30; ++i) {
        frontend_desktop_render(ui, 256, 240, "Presentation benchmark", "NTSC", "Running");
        SDL_Delay(10);
    }
    uint64_t until = SDL_GetTicks64() + 10000;
    while ((!gtk_widget_get_mapped(ui->gtk->picture) || gtk_widget_get_width(ui->gtk->picture) <= 0 ||
            gtk_widget_get_height(ui->gtk->picture) <= 0) && SDL_GetTicks64() < until) {
        frontend_desktop_render(ui, 256, 240, "Presentation benchmark", "NTSC", "Running");
        SDL_Delay(10);
    }
}

static GtkWidget *find_type(GtkWidget *root, GType type) {
    if (G_TYPE_CHECK_INSTANCE_TYPE(root, type)) {
        return root;
    }
    for (GtkWidget *child = gtk_widget_get_first_child(root); child; child = gtk_widget_get_next_sibling(child)) {
        GtkWidget *found = find_type(child, type);
        if (found) {
            return found;
        }
    }
    return NULL;
}

typedef struct {
    uint64_t start, count, total, maximum;
} PaintStats;

static void paint_start(GdkFrameClock *clock, GtkWidget *window) {
    (void)clock;
    PaintStats *stats = g_object_get_data(G_OBJECT(window), "benchmark-paint");
    stats->start = SDL_GetPerformanceCounter();
}

static void paint_end(GdkFrameClock *clock, GtkWidget *window) {
    (void)clock;
    PaintStats *stats = g_object_get_data(G_OBJECT(window), "benchmark-paint");
    uint64_t elapsed = SDL_GetPerformanceCounter() - stats->start;
    ++stats->count;
    stats->total += elapsed;
    if (elapsed > stats->maximum) {
        stats->maximum = elapsed;
    }
}

static void track_paints(GtkWidget *window) {
    if (g_object_get_data(G_OBJECT(window), "benchmark-paint")) {
        return;
    }
    GdkFrameClock *clock = gtk_widget_get_frame_clock(window);
    if (!clock) {
        return;
    }
    g_object_set_data_full(G_OBJECT(window), "benchmark-paint", g_new0(PaintStats, 1), g_free);
    g_signal_connect_object(clock, "before-paint", G_CALLBACK(paint_start), window, 0);
    g_signal_connect_object(clock, "after-paint", G_CALLBACK(paint_end), window, 0);
}

static void interact(FrontendDesktopUi *ui, unsigned mode, unsigned frame) {
    if (frame % 15) {
        return;
    }
    if (mode == 0) {
        GtkWidget *popover = find_type(ui->gtk->menubar, GTK_TYPE_POPOVER);
        if (popover) {
            if ((frame / 15) & 1) {
                gtk_popover_popdown(GTK_POPOVER(popover));
            } else {
                gtk_popover_popup(GTK_POPOVER(popover));
            }
        }
    } else if (mode == 1) {
        cupid_gtk_open(ui, 0, 0);
        for (CupidGtkTool *tool = ui->gtk->tools; tool; tool = tool->next) {
            if (tool->kind != 0) {
                continue;
            }
            GtkWidget *categories = find_type(tool->content, GTK_TYPE_LIST_BOX);
            gtk_list_box_select_row(GTK_LIST_BOX(categories),
                                    gtk_list_box_get_row_at_index(GTK_LIST_BOX(categories), (frame / 15) % 8));
        }
    } else if (mode == 2) {
        FrontendPanelInfo info;
        size_t count = frontend_panel_count();
        if (count && frontend_panel_at((frame / 15) % count, &info) && info.enabled) {
            cupid_gtk_open(ui, 1, info.id);
        }
    } else if (mode == 3) {
        gtk_window_set_default_size(GTK_WINDOW(ui->gtk->window), ((frame / 15) & 1) ? 900 : 1100,
                                    ((frame / 15) & 1) ? 700 : 800);
    } else if (mode == 4) {
        if (!frame) {
            for (CupidGtkTool *tool = ui->gtk->tools; tool; tool = tool->next) {
                if (tool->kind == 1) {
                    gtk_widget_set_visible(tool->window, TRUE);
                }
            }
        }
    } else {
        cupid_gtk_open(ui, 0, 0);
        for (CupidGtkTool *tool = ui->gtk->tools; tool; tool = tool->next) {
            if (tool->kind != 0) {
                continue;
            }
            GtkWidget *categories = find_type(tool->content, GTK_TYPE_LIST_BOX);
            gtk_list_box_select_row(GTK_LIST_BOX(categories),
                                    gtk_list_box_get_row_at_index(GTK_LIST_BOX(categories), 2));
            GtkWidget *stack = find_type(tool->content, GTK_TYPE_STACK);
            GtkWidget *choice = find_type(gtk_stack_get_visible_child(GTK_STACK(stack)), GTK_TYPE_DROP_DOWN);
            if (choice) {
                guint count = g_list_model_get_n_items(gtk_drop_down_get_model(GTK_DROP_DOWN(choice)));
                if (count) {
                    gtk_drop_down_set_selected(GTK_DROP_DOWN(choice),
                                               (gtk_drop_down_get_selected(GTK_DROP_DOWN(choice)) + 1) % count);
                }
            }
        }
    }
}

int benchmark_gtk(const char *path, const char *movie) {
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
    settings.pause_on_ui = false;
    settings.show_fps = true;
    SDL_AudioSpec have = {0};
    SDL_AudioDeviceID audio = 0;
    FrontendAudioRuntime audio_runtime;
    frontend_audio_runtime_bind(&audio_runtime, &audio, &have);
    char error[256] = {0};
    if (!frontend_audio_runtime_apply(&audio_runtime, &settings, false, error, sizeof(error))) return 2;
    if (!audio) {
        return 2;
    }
    vs_audio_init(have.freq);
    FrontendExecutionRuntime execution;
    frontend_execution_init(&execution, &audio, have.freq, NULL, NULL, NULL, NULL);
    frontend_audio_runtime_set_execution(&audio_runtime, &execution);
    frontend_execution_set_rewind_seconds(&execution, settings.rewind_seconds);
    SDL_Window *window = SDL_CreateWindow("Presentation benchmark", 0, 0, 900, 700, SDL_WINDOW_HIDDEN);
    SDL_Renderer *renderer = frontend_desktop_create_renderer(window, SDL_RENDERER_SOFTWARE);
    if (!renderer) {
        return 2;
    }
    FrontendVideoRuntime video;
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
    const char *names[] = {"window", "maximized", "fullscreen", "maximized-ppu", "movie", "tas-editor"};
    const char *ui_names[] = {"menus", "settings", "tools", "resize", "retained-tools", "choices"};
    bool interaction = g_getenv("CUPID_BENCHMARK_UI") != NULL;
    if (interaction) {
        movie = NULL;
    }
    double frequency = (double)SDL_GetPerformanceFrequency();
    bool work_jitter = g_getenv("CUPID_BENCHMARK_WORK_JITTER") != NULL;
    int result = 0;
    for (unsigned mode = 0; mode < (interaction ? G_N_ELEMENTS(ui_names) : movie ? G_N_ELEMENTS(names) : 4); ++mode) {
        for (CupidGtkTool *tool = ui.gtk->tools; tool; tool = tool->next) {
            gtk_widget_set_visible(tool->window, FALSE);
        }
        if (movie && mode == 4) {
            for (CupidGtkTool *tool = ui.gtk->tools; tool; tool = tool->next) {
                gtk_widget_set_visible(tool->window, FALSE);
            }
            if (!frontend_execution_movie_set_path(&execution, movie, error, sizeof(error)) ||
                !frontend_execution_movie_play(&execution, error, sizeof(error))) {
                result = 1;
                break;
            }
        }
        if (movie && mode == 5) {
            cupid_gtk_open(&ui, 1, TAS_PANEL);
        }
        settings.fullscreen = !interaction && mode == 2;
        settle(&ui);
        if (!interaction && (mode == 1 || mode == 3)) {
            gtk_window_maximize(GTK_WINDOW(ui.gtk->window));
        }
        if (!interaction && mode == 3) {
            cupid_gtk_open(&ui, 1, DEBUG_PPU_PATTERNS);
        }
        settle(&ui);
        if (interaction) {
            gtk_window_unmaximize(GTK_WINDOW(ui.gtk->window));
        }
        if (!gtk_widget_get_mapped(ui.gtk->picture) || gtk_widget_get_width(ui.gtk->picture) <= 0 ||
            gtk_widget_get_height(ui.gtk->picture) <= 0) {
            fprintf(stderr, "Benchmark viewport is not allocated; keep the game window visible\n");
            result = 1;
            break;
        }
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
            frontend_audio_runtime_submit(&audio_runtime);
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
        const char *game_renderer = cupid_gtk_native_video_name(ui.gtk);
        printf("GTK benchmark: %s %dx%d, %s, %u frames, %.1f fps, core %.2f ms, presentation %.2f ms\n",
               interaction ? ui_names[mode] : names[mode], gtk_widget_get_width(ui.gtk->picture),
               gtk_widget_get_height(ui.gtk->picture), game_renderer ? game_renderer : G_OBJECT_TYPE_NAME(gsk), frames,
               frames / elapsed, core * 1000 / frames, presentation * 1000 / frames);
        fflush(stdout);
        /* Repeat with the production deadline calculation. Throughput above
         * 60 does not by itself establish stable normal-speed playback. */
        begin = SDL_GetPerformanceCounter();
        double deadline = (double)begin;
        double last_present = 0, present_cost = 0;
        unsigned presented = 0;
        NesFrameTiming pacing;
        nes_frame_timing_reset(&pacing);
        nes_frame_timing_reset(&ui.gtk->draw_timing);
        ui.gtk->drawn_frames = 0;
        ui.gtk->last_draw = 0;
        uint64_t previous = begin;
        for (unsigned i = 0; i < frames; ++i) {
            if (interaction) {
                interact(&ui, mode, i);
                track_paints(ui.gtk->window);
                for (CupidGtkTool *tool = ui.gtk->tools; tool; tool = tool->next) {
                    if (gtk_widget_get_visible(tool->window)) {
                        track_paints(tool->window);
                    }
                }
            }
            SDL_Event event;
            while (SDL_PollEvent(&event)) {
            }
            uint64_t cycles = cpu_total_cycles;
            if (!frontend_execution_run_frame(&execution)) {
                result = 1;
                break;
            }
            frontend_audio_runtime_submit(&audio_runtime);
            deadline += (cpu_total_cycles - cycles) * frequency / nes_timing()->cpu_hz;
            /* Alternate host workload without changing emulated timing. */
            if (work_jitter && (i & 1)) {
                SDL_Delay(4);
            }
            double now = (double)SDL_GetPerformanceCounter();
            if (nes_frame_timing_present_cost(now, deadline, last_present, frequency, present_cost)) {
                if (!frontend_video_runtime_refresh(&video, error, sizeof(error))) {
                    result = 1;
                    break;
                }
                uint64_t rendered = SDL_GetPerformanceCounter();
                frontend_desktop_wait(&ui, deadline, frequency);
                uint64_t present_start = SDL_GetPerformanceCounter();
                frontend_desktop_render(&ui, 256, 240, "Presentation benchmark", "NTSC", "Running");
                present_cost =
                    fmax(present_cost * 0.8, rendered - now + (double)SDL_GetPerformanceCounter() - present_start);
                last_present = (double)present_start;
                ++presented;
            } else {
                nes_shader_skip_frame(video.shader);
                frontend_desktop_wait(&ui, deadline, frequency);
            }
            uint64_t end = SDL_GetPerformanceCounter();
            NesHostFrameSample sample = {0};
            sample.milliseconds[NES_HOST_INTERVAL] = nes_frame_timing_elapsed(previous, end, (uint64_t)frequency);
            (void)nes_frame_timing_push(&pacing, &sample);
            previous = end;
        }
        elapsed = (SDL_GetPerformanceCounter() - begin) / frequency;
        if (!ui.gtk->drawn_frames) {
            fprintf(stderr, "Benchmark viewport did not draw; keep the game window visible\n");
            result = 1;
            break;
        }
        const char *label = interaction ? ui_names[mode] : names[mode];
        printf("GTK paced: %s, %.2f fps (target %.2f), %u/%u presentations\n", label, frames / elapsed,
               nes_timing()->fps, presented, frames);
        NesFrameTimingSummary paced, drawn;
        (void)nes_frame_timing_summary(&pacing, &paced);
        (void)nes_frame_timing_summary(&ui.gtk->draw_timing, &drawn);
        printf("GTK cadence: %s, host jitter %.3f ms, p95 %.3f ms, max %.3f ms; %llu unique draws, draw jitter %.3f "
               "ms, p95 %.3f ms, max %.3f ms\n",
               label, paced.interval_jitter_ms, paced.metrics[NES_HOST_INTERVAL].p95,
               paced.metrics[NES_HOST_INTERVAL].maximum, (unsigned long long)ui.gtk->drawn_frames,
               drawn.interval_jitter_ms, drawn.metrics[NES_HOST_INTERVAL].p95,
               drawn.metrics[NES_HOST_INTERVAL].maximum);
        fflush(stdout);
        if (interaction) {
            for (CupidGtkTool *tool = ui.gtk->tools; tool; tool = tool->next) {
                PaintStats *stats = g_object_get_data(G_OBJECT(tool->window), "benchmark-paint");
                if (stats && stats->count) {
                    printf("GTK paints: %s, %llu paints, avg %.3f ms, max %.3f ms\n",
                           gtk_window_get_title(GTK_WINDOW(tool->window)), (unsigned long long)stats->count,
                           stats->total * 1000.0 / frequency / stats->count, stats->maximum * 1000.0 / frequency);
                    *stats = (PaintStats){0};
                }
            }
            fflush(stdout);
        }
        if (result) {
            break;
        }
    }
    SDL_PauseAudioDevice(audio, 1);
    frontend_desktop_shutdown(&ui);
    debug_frontend_destroy(debug);
    frontend_execution_shutdown(&execution);
    frontend_video_runtime_shutdown(&video);
    frontend_audio_runtime_shutdown(&audio_runtime);
    frontend_desktop_destroy_renderer(window, renderer);
    SDL_DestroyWindow(window);
    unload_rom();
    SDL_Quit();
    if (result) {
        fprintf(stderr, "Benchmark failed: %s\n", error);
    }
    return result;
}
