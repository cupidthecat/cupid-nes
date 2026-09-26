/*
 * video_runtime_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Desktop texture sampling and presentation integration. SPDX-License-Identifier: GPL-3.0-or-later */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../state/state.h"
#include "../system/vs_system.h"
#include "../ui/desktop_internal.h"
#include "../ui/video_runtime.h"
#include "../video/frame_snapshot.h"
#include "../util/file_io.h"
#include "../../include/globals.h"
#include <string.h>

static int failures;

#define CHECK(condition)                                                                                               \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            fprintf(stderr, "Video runtime check %d failed: %s\n", __LINE__, #condition);                              \
            ++failures;                                                                                                \
        }                                                                                                              \
    } while (0)

static uint32_t pixels[256 * 240];
static uint16_t signal[256 * 240];

static void check_hd(FrontendVideoRuntime *runtime) {
    char error[256];
    runtime->hd = nes_hd_runtime_create("build", error, sizeof(error));
    CHECK(runtime->hd != NULL);
    if (!runtime->hd) {
        return;
    }

    runtime->hd_frontend = nes_hd_frontend_create(runtime->hd, error, sizeof(error));
    CHECK(runtime->hd_frontend != NULL);
    NesHdGameInfo game = {
        .rom_path = "video-runtime.nes", .chr_rom = chr_rom, .chr_size = (size_t)chr_size, .chr_is_rom = true};
    CHECK(nes_hd_runtime_set_game(runtime->hd, &game, error, sizeof(error)));
    CHECK(nes_video_trace_use(NES_VIDEO_TRACE_EXPORT, true));
    for (unsigned y = 0; y < 240; ++y) {
        for (unsigned x = 0; x < 256; ++x) {
            NesVideoPixel *pixel = nes_video_trace_pixel(0, x, y);
            CHECK(pixel != NULL);
            if (!pixel) {
                continue;
            }

            pixel->original_rgb = pixel->backdrop_rgb = 0xFF112233u;
            pixel->background.chr.valid = true;
            memcpy(pixel->background.chr.bytes, chr_rom, 16);
            pixel->background.x = (uint8_t)(x & 7u);
            pixel->background.y = (uint8_t)(y & 7u);
            pixel->background.palette = 0x0F212730u;
            pixel->background.color_index = 1;
            pixel->background.color = 0x21;
            pixel->background.rgb = 0xFF112233u;
        }
    }

    nes_video_trace_complete(0, 1);
    NesHdFrameSource source = {.fallback_pixels = pixels,
                               .fallback_width = 256,
                               .fallback_height = 240,
                               .screens = 1,
                               .trace = {nes_video_trace_frame(0), NULL},
                               .show_background = true,
                               .show_sprites = true};
    const char *pack = "build/video-runtime-hd.zip";
    bool captured = nes_hd_runtime_capture(runtime->hd, &source, pack, error, sizeof(error));
    if (!captured) {
        fprintf(stderr, "Video runtime HD capture: %s\n", error);
    }

    CHECK(captured);
    CHECK(nes_hd_runtime_load(runtime->hd, pack, error, sizeof(error)));
    CHECK(nes_hd_runtime_enable(runtime->hd, true, error, sizeof(error)));
    for (unsigned pass = 0; pass < 4; ++pass) {
        unsigned scale = pass >= 2 ? 2 : 1;
        runtime->settings->pixel_filter.kind = pass >= 2 ? NES_PIXEL_FILTER_LCD : NES_PIXEL_FILTER_NONE;
        runtime->settings->bilinear_interpolation = !(pass & 1);
        CHECK(frontend_video_runtime_refresh(runtime, error, sizeof(error)));
        NesHdRuntimeInfo info;
        CHECK(nes_hd_runtime_info(runtime->hd, &info) && info.enabled && info.trace_ready);
        CHECK(runtime->texture_width == 256 * scale && runtime->texture_height == 240 * scale);
        CHECK(runtime->frame.pixels[0] == 0xFF112233u);
        const uint32_t *capture;
        unsigned width, height, stride;
        CHECK(frontend_video_runtime_pixels(runtime, true, &capture, &width, &height, &stride));
        CHECK(width == 256 * scale && height == 240 * scale && stride == width && capture[0] == 0xFF112233u);
        if (scale == 2) {
            CHECK(capture[1] == 0xFF0E1C2Bu && capture[width] == 0xFF0E1C2Bu);
        }
        SDL_ScaleMode mode;
        CHECK(SDL_GetTextureScaleMode(runtime->texture, &mode) == 0);
        CHECK(mode == (!(pass & 1) ? SDL_ScaleModeLinear : SDL_ScaleModeNearest));
        unsigned side;
        int x, y;
        CHECK(frontend_video_runtime_aim(runtime, (int)width - 1, (int)height - 1, &side, &x, &y));
        CHECK(side == 0 && x == 255 && y == 239);
    }

    for (int kind = NES_PIXEL_FILTER_XBRZ2; kind < NES_PIXEL_FILTER_COUNT; ++kind) {
        runtime->settings->pixel_filter.kind = (NesPixelFilterKind)kind;
        unsigned factor = nes_pixel_filter_scale((NesPixelFilterKind)kind);
        CHECK(frontend_video_runtime_refresh(runtime, error, sizeof(error)));
        CHECK(runtime->texture_width == 256 * factor && runtime->texture_height == 240 * factor);
        const uint32_t *capture;
        unsigned width, height, stride;
        CHECK(frontend_video_runtime_pixels(runtime, true, &capture, &width, &height, &stride));
        CHECK(width == 256 * factor && height == 240 * factor && stride == width);
        CHECK(capture[0] == 0xFF112233u && capture[(size_t)width * height - 1] == 0xFF112233u);
    }

    CHECK(nes_hd_runtime_enable(runtime->hd, false, error, sizeof(error)));
    runtime->settings->pixel_filter.kind = NES_PIXEL_FILTER_NONE;
    CHECK(nes_video_trace_use(NES_VIDEO_TRACE_EXPORT, false));
    CHECK(nes_file_remove(pack) == NES_FILE_OK);
}

static void check_settings(FrontendVideoRuntime *runtime, FrontendSettings *settings) {
    FrontendDesktopUi ui;
    frontend_desktop_init(&ui, NULL, NULL, settings, NULL, NULL, "build/video-settings.ini");
    frontend_desktop_set_runtime(&ui, NULL, runtime);
    desktop_settings_open(&ui, true);
    ui.settings_category = 2;
    CHECK(desktop_setting_rows(&ui) == 36);
    CHECK(desktop_setting_kind(&ui, 19) == SETTING_TOGGLE);
    CHECK(!desktop_setting_commit_number(&ui, 19, "1"));
    CHECK(desktop_setting_kind(&ui, 20) == SETTING_CHOICE);
    CHECK(!desktop_setting_commit_number(&ui, 20, "1"));
    int selected;
    CHECK(desktop_setting_choices(&ui, 20, &selected) == NES_PIXEL_FILTER_COUNT && selected == NES_PIXEL_FILTER_NONE);
    desktop_setting_choose(&ui, 20, NES_PIXEL_FILTER_LCD);
    CHECK(desktop_setting_commit_number(&ui, 21, "50"));
    CHECK(!desktop_setting_commit_number(&ui, 21, "101"));
    CHECK(!desktop_setting_commit_number(&ui, 24, "-1"));
    desktop_activate_setting(&ui, 19);
    CHECK(ui.staged.bilinear_interpolation && !settings->bilinear_interpolation);
    desktop_settings_button(&ui, 2);
    CHECK(!settings->bilinear_interpolation);
    CHECK(settings->pixel_filter.kind == NES_PIXEL_FILTER_NONE && settings->pixel_filter.lcd_brightness[0] == 100);
    desktop_settings_open(&ui, true);
    ui.settings_category = 2;
    desktop_activate_setting(&ui, 19);
    desktop_setting_choose(&ui, 20, NES_PIXEL_FILTER_LCD);
    CHECK(desktop_setting_commit_number(&ui, 21, "50"));
    desktop_settings_button(&ui, 0);
    if (!settings->bilinear_interpolation) {
        fprintf(stderr, "Video settings Apply: category %d, staged %d, applied %d, status: %s\n", ui.settings_category,
                ui.staged.bilinear_interpolation, settings->bilinear_interpolation, ui.status);
    }

    CHECK(settings->bilinear_interpolation && runtime->settings == settings);
    CHECK(settings->pixel_filter.kind == NES_PIXEL_FILTER_LCD && settings->pixel_filter.lcd_brightness[0] == 50);
    CHECK(runtime->frame.width == 512 && runtime->frame.height == 480);
    SDL_ScaleMode mode;
    CHECK(SDL_GetTextureScaleMode(runtime->texture, &mode) == 0 && mode == SDL_ScaleModeLinear);
    FrontendSettings restored;
    FrontendSettingsReport report;
    CHECK(frontend_settings_load(ui.settings_path, &restored, &report));
    CHECK(restored.bilinear_interpolation);
    CHECK(restored.pixel_filter.kind == NES_PIXEL_FILTER_LCD && restored.pixel_filter.lcd_brightness[0] == 50);
    ui.settings_path = "build/video-settings.ini/missing/settings.ini";
    desktop_activate_setting(&ui, 19);
    desktop_setting_choose(&ui, 20, NES_PIXEL_FILTER_NONE);
    desktop_settings_button(&ui, 0);
    CHECK(settings->bilinear_interpolation && runtime->settings == settings);
    CHECK(settings->pixel_filter.kind == NES_PIXEL_FILTER_LCD && runtime->frame.width == 512);
    CHECK(runtime->frame.pixels[2] == 0xFF7F7F7Fu);
    CHECK(SDL_GetTextureScaleMode(runtime->texture, &mode) == 0 && mode == SDL_ScaleModeLinear);
    ui.settings_path = "build/video-settings.ini";
    desktop_settings_button(&ui, 3);
    CHECK(!ui.staged.bilinear_interpolation && settings->bilinear_interpolation);
    CHECK(ui.staged.pixel_filter.kind == NES_PIXEL_FILTER_NONE && ui.staged.pixel_filter.lcd_brightness[0] == 100);
    desktop_settings_button(&ui, 1);
    CHECK(!ui.settings_open && !settings->bilinear_interpolation && runtime->settings == settings);
    CHECK(settings->pixel_filter.kind == NES_PIXEL_FILTER_NONE && runtime->frame.width == 256);
    CHECK(SDL_GetTextureScaleMode(runtime->texture, &mode) == 0 && mode == SDL_ScaleModeNearest);
    CHECK(nes_file_remove(ui.settings_path) == NES_FILE_OK);
    frontend_desktop_shutdown(&ui);
}

static void check_lcd(FrontendVideoRuntime *runtime) {
    FrontendSettings *settings = runtime->settings;
    char error[256];
    frontend_video_runtime_set_composite(runtime, false);
    for (unsigned pass = 0; pass < 3; ++pass) {
        settings->pixel_filter.kind = pass == 1 ? NES_PIXEL_FILTER_NONE : NES_PIXEL_FILTER_LCD;
        unsigned factor = pass == 1 ? 1 : 2;
        CHECK(frontend_video_runtime_refresh(runtime, error, sizeof(error)));
        CHECK(runtime->frame.width == 256 * factor && runtime->frame.height == 240 * factor);
        const uint32_t *capture;
        unsigned width, height, stride, side;
        int x, y;
        CHECK(frontend_video_runtime_pixels(runtime, false, &capture, &width, &height, &stride));
        CHECK(width == 256 && height == 240 && !memcmp(capture, pixels, sizeof(pixels)));
        CHECK(frontend_video_runtime_pixels(runtime, true, &capture, &width, &height, &stride));
        CHECK(width == 256 * factor && height == 240 * factor && stride == width);
        if (factor == 2) {
            CHECK(capture[0] == 0xFF000000u && capture[2] == 0xFFFFFFFFu);
            CHECK(capture[3] == 0xFFD8D8D8u && capture[width + 2] == 0xFFD8D8D8u);
        }
        CHECK(frontend_video_runtime_aim(runtime, (int)width - 1, (int)height - 1, &side, &x, &y));
        CHECK(side == 0 && x == 255 && y == 239);
    }
    settings->presentation.overscan[0] = (NesVideoOverscan){3, 5, 7, 9};
    CHECK(nes_video_presentation_set(&settings->presentation, error, sizeof(error)));
    CHECK(frontend_video_runtime_refresh(runtime, error, sizeof(error)));
    CHECK(runtime->frame.width == 496 && runtime->frame.height == 448);
    unsigned side, width, height;
    int x, y;
    CHECK(frontend_video_runtime_aim(runtime, 0, 0, &side, &x, &y));
    CHECK(side == 0 && x == 3 && y == 7);
    CHECK(frontend_video_runtime_aim(runtime, 495, 447, &side, &x, &y));
    CHECK(side == 0 && x == 250 && y == 230);
    settings->aspect_mode = FRONTEND_ASPECT_4_3;
    frontend_video_runtime_display_size(runtime, &width, &height);
    /* The existing 4:3 layout rounds upward to include the whole display. */
    CHECK(width == 598 && height == 448);
    settings->aspect_mode = FRONTEND_ASPECT_SOURCE;
    nes_video_presentation_defaults(&settings->presentation);
    CHECK(nes_video_presentation_set(&settings->presentation, error, sizeof(error)));
    frontend_video_runtime_set_composite(runtime, true);
    CHECK(frontend_video_runtime_refresh(runtime, error, sizeof(error)));
    CHECK(runtime->frame.width == 1024 && runtime->frame.height == 960 && runtime->frame.filtered);
    CHECK(frontend_video_runtime_aim(runtime, 1023, 959, &side, &x, &y));
    CHECK(side == 0 && x == 255 && y == 239);
    settings->pixel_filter.kind = NES_PIXEL_FILTER_NONE;
}

static void check_lcd_window_sizes(void) {
    /* Game rectangles use renderer pixels and the independently scaled desktop chrome. */
    for (unsigned pass = 0; pass < 3; ++pass) {
        float dpi = pass == 0 ? 1.0f : pass == 1 ? 1.5f : 2.0f;
        int dimension = (int)(1024 * dpi);
        SDL_Surface *surface = SDL_CreateRGBSurfaceWithFormat(0, dimension, dimension, 32, SDL_PIXELFORMAT_ARGB8888);
        SDL_Renderer *renderer = surface ? SDL_CreateSoftwareRenderer(surface) : NULL;
        CHECK(renderer != NULL);
        if (!renderer) {
            SDL_FreeSurface(surface);
            continue;
        }
        FrontendSettings settings;
        frontend_settings_defaults(&settings);
        settings.pixel_filter.kind = NES_PIXEL_FILTER_LCD;
        FrontendVideoRuntime runtime = {0};
        char error[160];
        bool initialized = frontend_video_runtime_init(&runtime, renderer, &settings, false, error, sizeof(error));
        CHECK(initialized);
        if (initialized) {
            FrontendDesktopUi ui = {.ui_scale = dpi};
            SDL_Rect rect;
            frontend_desktop_game_rect(&ui, dimension, dimension, 512, 480, true, &rect);
            CHECK(rect.w > 0 && rect.h > 0 && rect.x >= 0 && rect.y >= 0);
            CHECK(rect.x + rect.w <= dimension && rect.y + rect.h <= dimension);
            CHECK(rect.w % 512 == 0 && rect.h % 480 == 0 && rect.w / 512 == rect.h / 480);
            CHECK(SDL_RenderCopy(renderer, runtime.texture, NULL, &rect) == 0);
            unsigned factor = (unsigned)rect.w / 512;
            const SDL_Rect samples[] = {{rect.x + (int)(2 * factor), rect.y, 1, 1},
                                        {rect.x + (int)(3 * factor), rect.y, 1, 1},
                                        {rect.x + (int)(2 * factor), rect.y + (int)factor, 1, 1}};
            const uint32_t expected[] = {0xFFFFFFFFu, 0xFFD8D8D8u, 0xFFD8D8D8u};
            for (unsigned i = 0; i < 3; ++i) {
                uint32_t pixel = 0;
                CHECK(SDL_RenderReadPixels(renderer, &samples[i], SDL_PIXELFORMAT_ARGB8888, &pixel, 4) == 0);
                CHECK(pixel == expected[i]);
            }
        }
        frontend_video_runtime_shutdown(&runtime);
        SDL_DestroyRenderer(renderer);
        SDL_FreeSurface(surface);
    }
}

static void check_scalers(FrontendVideoRuntime *runtime) {
    FrontendSettings *settings = runtime->settings;
    char error[160];
    for (int kind = NES_PIXEL_FILTER_XBRZ2; kind < NES_PIXEL_FILTER_COUNT; ++kind) {
        settings->pixel_filter.kind = (NesPixelFilterKind)kind;
        unsigned factor = nes_pixel_filter_scale(settings->pixel_filter.kind);
        frontend_video_runtime_set_composite(runtime, false);
        CHECK(frontend_video_runtime_refresh(runtime, error, sizeof(error)));
        CHECK(runtime->texture_width == 256 * factor && runtime->texture_height == 240 * factor);
        const uint32_t *capture;
        unsigned width, height, stride, side;
        int x, y;
        CHECK(frontend_video_runtime_pixels(runtime, true, &capture, &width, &height, &stride));
        CHECK(width == 256 * factor && height == 240 * factor && stride == width);
        CHECK(frontend_video_runtime_aim(runtime, (int)width - 1, (int)height - 1, &side, &x, &y));
        CHECK(side == 0 && x == 255 && y == 239);
        CHECK(!frontend_video_runtime_aim(runtime, (int)width, 0, &side, &x, &y));
        CHECK(!frontend_video_runtime_aim(runtime, -1, 0, &side, &x, &y));
        CHECK(frontend_video_runtime_pixels(runtime, false, &capture, &width, &height, &stride));
        CHECK(width == 256 && height == 240 && stride == width && !memcmp(capture, pixels, sizeof(pixels)));
        settings->presentation.overscan[0] = (NesVideoOverscan){3, 5, 7, 9};
        CHECK(nes_video_presentation_set(&settings->presentation, error, sizeof(error)));
        CHECK(frontend_video_runtime_refresh(runtime, error, sizeof(error)));
        CHECK(runtime->frame.width == 248 * factor && runtime->frame.height == 224 * factor);
        CHECK(frontend_video_runtime_aim(runtime, 0, 0, &side, &x, &y));
        CHECK(side == 0 && x == 3 && y == 7);
        CHECK(frontend_video_runtime_aim(runtime, (int)(248 * factor - 1), (int)(224 * factor - 1), &side, &x, &y));
        CHECK(side == 0 && x == 250 && y == 230);
        nes_video_presentation_defaults(&settings->presentation);
        CHECK(nes_video_presentation_set(&settings->presentation, error, sizeof(error)));
        /* Each family also receives the composite frame before scaling. */
        if (kind == NES_PIXEL_FILTER_XBRZ3 || kind == NES_PIXEL_FILTER_HQ4X || kind == NES_PIXEL_FILTER_SCALE4X ||
            kind == NES_PIXEL_FILTER_SUPEREAGLE || kind == NES_PIXEL_FILTER_PRESCALE10) {
            frontend_video_runtime_set_composite(runtime, true);
            CHECK(frontend_video_runtime_refresh(runtime, error, sizeof(error)));
            CHECK(runtime->frame.width == 512 * factor && runtime->frame.height == 480 * factor &&
                  runtime->frame.filtered);
            CHECK(frontend_video_runtime_aim(runtime, (int)(512 * factor - 1), (int)(480 * factor - 1), &side, &x, &y));
            CHECK(side == 0 && x == 255 && y == 239);
        }
    }

    settings->pixel_filter.kind = NES_PIXEL_FILTER_NONE;
    frontend_video_runtime_set_composite(runtime, false);
    CHECK(frontend_video_runtime_refresh(runtime, error, sizeof(error)));
}

static void check_ntsc_picture(FrontendVideoRuntime *runtime) {
    static uint32_t expected[NTSC_COMPOSITE_WIDTH * NTSC_COMPOSITE_HEIGHT];
    char error[160];
    FrontendSettings *settings = runtime->settings;
    frontend_video_runtime_set_composite(runtime, true);
    for (unsigned preset = 0; preset < NTSC_PRESET_COUNT; ++preset) {
        CHECK(ntsc_composite_preset(&settings->ntsc_picture, preset));
        CHECK(frontend_video_runtime_refresh(runtime, error, sizeof(error)));
        CHECK(ntsc_composite_filter_frame_settings(signal, 0, expected, &settings->ntsc_picture, error, sizeof(error)));
        CHECK(runtime->frame.filtered && runtime->frame.width == 512 && runtime->frame.height == 480);
        CHECK(!memcmp(runtime->frame.pixels, expected, sizeof(expected)));
        const uint32_t *capture;
        unsigned width, height, stride, side;
        int x, y;
        CHECK(frontend_video_runtime_pixels(runtime, true, &capture, &width, &height, &stride));
        CHECK(width == 512 && height == 480 && stride == 512 && !memcmp(capture, expected, sizeof(expected)));
        CHECK(frontend_video_runtime_pixels(runtime, false, &capture, &width, &height, &stride));
        CHECK(width == 256 && height == 240 && stride == 256 && !memcmp(capture, pixels, sizeof(pixels)));
        CHECK(frontend_video_runtime_aim(runtime, 511, 479, &side, &x, &y) && side == 0 && x == 255 && y == 239);
    }
    ntsc_composite_defaults(&settings->ntsc_picture);
    frontend_video_runtime_set_composite(runtime, false);
    CHECK(frontend_video_runtime_refresh(runtime, error, sizeof(error)));
    CHECK(!memcmp(runtime->frame.pixels, pixels, sizeof(pixels)));
}

static void check_ntsc_settings(FrontendVideoRuntime *runtime) {
    FrontendSettings *settings = runtime->settings;
    const char *path = "build/ntsc-runtime-settings.ini";
    FrontendDesktopUi ui;
    frontend_desktop_init(&ui, NULL, NULL, settings, NULL, NULL, path);
    frontend_desktop_set_runtime(&ui, NULL, runtime);
    desktop_settings_open(&ui, true);
    ui.settings_category = 2;
    int selected;
    CHECK(desktop_setting_choices(&ui, 25, &selected) == NTSC_PRESET_COUNT && selected == 0);
    CHECK(desktop_setting_choices(&ui, 35, &selected) == NTSC_FIELDS_COUNT && selected == NTSC_FIELDS_SOURCE);
    for (int row = 26; row < 35; ++row) {
        CHECK(desktop_setting_kind(&ui, row) == SETTING_NUMBER);
        CHECK(!desktop_setting_commit_number(&ui, row, "NaN"));
        CHECK(!desktop_setting_commit_number(&ui, row, "1.5"));
    }
    CHECK(desktop_setting_commit_number(&ui, 26, "-30"));
    CHECK(ntsc_composite_preset_index(&ui.staged.ntsc_picture) == -1);
    desktop_setting_choose(&ui, 25, 0);
    CHECK(ntsc_composite_preset_index(&ui.staged.ntsc_picture) == 0);
    desktop_setting_choose(&ui, 25, 2);
    desktop_activate_setting(&ui, 4);
    CHECK(ui.staged.ntsc_composite && !settings->ntsc_composite);
    desktop_settings_button(&ui, 2);
    CHECK(!ui.settings_open && !settings->ntsc_composite && !runtime->composite);
    CHECK(ntsc_composite_preset_index(&settings->ntsc_picture) == 0);
    desktop_settings_open(&ui, true);
    ui.settings_category = 2;
    desktop_setting_choose(&ui, 25, 2);
    desktop_activate_setting(&ui, 4);
    desktop_settings_button(&ui, 0);
    CHECK(settings->ntsc_composite && runtime->composite && runtime->settings == settings);
    CHECK(ntsc_composite_preset_index(&settings->ntsc_picture) == 2 && runtime->frame.filtered);
    uint32_t retained[32];
    memcpy(retained, runtime->frame.pixels, sizeof(retained));
    FrontendSettings restored;
    FrontendSettingsReport report;
    CHECK(frontend_settings_load(path, &restored, &report));
    CHECK(restored.ntsc_composite && ntsc_composite_preset_index(&restored.ntsc_picture) == 2);
    ui.settings_path = "build/ntsc-runtime-settings.ini/missing/settings.ini";
    desktop_setting_choose(&ui, 25, 3);
    desktop_settings_button(&ui, 0);
    CHECK(runtime->settings == settings && ntsc_composite_preset_index(&settings->ntsc_picture) == 2);
    CHECK(!memcmp(retained, runtime->frame.pixels, sizeof(retained)));
    ui.settings_path = path;
    desktop_settings_button(&ui, 3);
    CHECK(ntsc_composite_preset_index(&ui.staged.ntsc_picture) == 0 && !ui.staged.ntsc_composite);
    CHECK(ntsc_composite_preset_index(&settings->ntsc_picture) == 2);
    desktop_settings_button(&ui, 1);
    CHECK(!ui.settings_open && !settings->ntsc_composite && !runtime->composite && runtime->settings == settings);
    CHECK(ntsc_composite_preset_index(&settings->ntsc_picture) == 0);
    CHECK(nes_file_remove(path) == NES_FILE_OK);
    frontend_desktop_shutdown(&ui);
}

static bool prepare_machine(void) {
    BoardImage image = {0};
    if (!board_image_create(&image, 0, 0x8000, 0x2000, true)) {
        return false;
    }

    memset(image.data + 16, 0xEA, 0x8000);
    memset(image.data + 16 + 0x8000, 0xFF, 8);
    for (unsigned vector = 0x7FFA; vector <= 0x7FFE; vector += 2) {
        image.data[16 + vector] = 0;
        image.data[16 + vector + 1] = 0x80;
    }

    bool loaded = load_rom_memory(image.data, image.size) == 0;
    board_image_free(&image);
    if (!loaded) {
        return false;
    }

    cpu_select_machine(NULL);
    cpu_use_default_startup_alignment();
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    return cpu_power_on(&cpu);
}

static void check_interpolation(SDL_Renderer *renderer) {
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    CHECK(!settings.bilinear_interpolation);
    for (unsigned i = 0; i < 256 * 240; ++i) {
        pixels[i] = i & 1 ? 0xFFFFFFFFu : 0xFF000000u;
        signal[i] = (uint16_t)(i % 64);
    }

    nes_video_snapshot_complete(0, pixels, signal, 0, 1);
    NesStateBlob before = {0}, after = {0};
    CHECK(nes_state_capture(&before) == NES_STATE_OK);
    FrontendVideoRuntime runtime = {0};
    char error[256];
    bool initialized = frontend_video_runtime_init(&runtime, renderer, &settings, false, error, sizeof(error));
    CHECK(initialized);
    if (!initialized) {
        fprintf(stderr, "%s\n", error);
        nes_state_blob_free(&before);
        frontend_video_runtime_shutdown(&runtime);
        return;
    }

    SDL_Texture *inspection = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, 8, 8);
    CHECK(inspection != NULL);
    if (inspection) {
        CHECK(SDL_SetTextureScaleMode(inspection, SDL_ScaleModeNearest) == 0);
    }

    SDL_Texture *original = runtime.texture;
    for (unsigned pass = 0; pass < 3; ++pass) {
        settings.bilinear_interpolation = pass == 1;
        CHECK(frontend_video_runtime_refresh(&runtime, error, sizeof(error)));
        CHECK(runtime.texture == original);
        SDL_ScaleMode mode = SDL_ScaleModeBest;
        CHECK(SDL_GetTextureScaleMode(runtime.texture, &mode) == 0);
        CHECK(mode == (pass == 1 ? SDL_ScaleModeLinear : SDL_ScaleModeNearest));
        if (inspection) {
            CHECK(SDL_GetTextureScaleMode(inspection, &mode) == 0 && mode == SDL_ScaleModeNearest);
        }

        CHECK(SDL_RenderCopy(renderer, runtime.texture, NULL, NULL) == 0);
        uint32_t line[768];
        SDL_Rect first_row = {0, 0, 768, 1};
        CHECK(SDL_RenderReadPixels(renderer, &first_row, SDL_PIXELFORMAT_ARGB8888, line, sizeof(line)) == 0);
        bool blended = false;
        for (unsigned x = 3; x < 765; ++x) {
            unsigned value = line[x] & 0xFF;
            if (value != 0 && value != 255) {
                blended = true;
            }
        }

        CHECK(blended == (pass == 1));
        const uint32_t *capture;
        unsigned width, height, stride, side;
        int x, y;
        CHECK(frontend_video_runtime_pixels(&runtime, true, &capture, &width, &height, &stride));
        CHECK(width == 256 && height == 240 && stride == 256);
        CHECK(!memcmp(capture, pixels, sizeof(pixels)));
        CHECK(frontend_video_runtime_pixels(&runtime, false, &capture, &width, &height, &stride));
        CHECK(width == 256 && height == 240 && stride == 256);
        CHECK(!memcmp(capture, pixels, sizeof(pixels)));
        CHECK(frontend_video_runtime_aim(&runtime, 255, 239, &side, &x, &y));
        CHECK(side == 0 && x == 255 && y == 239);
    }

    settings.bilinear_interpolation = true;
    frontend_video_runtime_set_composite(&runtime, true);
    CHECK(frontend_video_runtime_refresh(&runtime, error, sizeof(error)));
    CHECK(runtime.texture_width == 512 && runtime.texture_height == 480);
    SDL_ScaleMode mode;
    CHECK(SDL_GetTextureScaleMode(runtime.texture, &mode) == 0 && mode == SDL_ScaleModeLinear);
    unsigned side;
    int x, y;
    CHECK(frontend_video_runtime_aim(&runtime, 511, 479, &side, &x, &y));
    CHECK(side == 0 && x == 255 && y == 239);
    check_lcd(&runtime);
    check_scalers(&runtime);
    check_ntsc_picture(&runtime);
    check_hd(&runtime);
    CHECK(nes_state_capture(&after) == NES_STATE_OK);
    CHECK(before.size == after.size && !memcmp(before.data, after.data, before.size));
    nes_state_blob_free(&before);
    nes_state_blob_free(&after);
    settings.bilinear_interpolation = false;
    settings.ntsc_composite = false;
    check_settings(&runtime, &settings);
    check_ntsc_settings(&runtime);
    check_lcd_window_sizes();
    SDL_DestroyTexture(inspection);
    frontend_video_runtime_shutdown(&runtime);
}

int test_video_runtime_accuracy(void) {
    failures = 0;
    NesVideoPresentationSettings previous;
    nes_video_presentation_get(&previous);
    NesRegionMode region = nes_region_mode();
    CHECK(nes_set_region_mode(NES_REGION_MODE_NTSC));
    SDL_setenv("SDL_VIDEODRIVER", "dummy", 1);
    CHECK(SDL_InitSubSystem(SDL_INIT_VIDEO) == 0);
    SDL_Surface *surface = SDL_CreateRGBSurfaceWithFormat(0, 768, 720, 32, SDL_PIXELFORMAT_ARGB8888);
    SDL_Renderer *renderer = surface ? SDL_CreateSoftwareRenderer(surface) : NULL;
    CHECK(renderer != NULL);
    bool ready = prepare_machine();
    CHECK(ready);
    if (renderer && ready) {
        check_interpolation(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_FreeSurface(surface);
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
    CHECK(unload_rom());
    CHECK(nes_video_presentation_set(&previous, NULL, 0));
    CHECK(nes_set_region_mode(region));
    printf("Video runtime: interpolation, pixel filters, capture geometry, DPI and hardware state, %d failures\n",
           failures);
    return failures;
}
