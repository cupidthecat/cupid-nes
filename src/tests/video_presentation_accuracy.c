/*
 * video_presentation_accuracy.c - Cropping, layers, and original sensor output
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../state/state.h"
#include "../system/hardware.h"
#include "../system/vs_system.h"
#include "../video/ntsc_composite.h"
#include "../video/presentation.h"
#include "../../include/globals.h"
#include <string.h>

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static uint32_t raw_pixels[512 * 240];
static uint16_t raw_signal[256 * 240];
static uint32_t filtered[NTSC_COMPOSITE_WIDTH * NTSC_COMPOSITE_HEIGHT];

static int test_regional_dual_screen_crop(void) {
    NesVideoPresentationSettings settings;
    nes_video_presentation_defaults(&settings);
    settings.overscan[0] = (NesVideoOverscan){8, 12, 7, 11};
    settings.overscan[1] = (NesVideoOverscan){2, 3, 4, 5};
    settings.overscan[2] = (NesVideoOverscan){4, 5, 6, 7};
    char error[160];
    CHECK(nes_video_presentation_set(&settings, error, sizeof(error)));
    for (unsigned y = 0; y < 240; ++y)
        for (unsigned x = 0; x < 512; ++x)
            raw_pixels[y * 512 + x] = 0xFF000000u | (y << 10) | x;

    for (NesRegion region = NES_REGION_NTSC; region <= NES_REGION_DENDY; ++region) {
        NesVideoPresentationSource source = {.pixels = raw_pixels, .screens = 2,
                                              .region = region, .vs_system = true};
        NesVideoPresentationFrame frame;
        CHECK(nes_video_presentation_render(&source, &frame, error, sizeof(error)));
        NesVideoOverscan crop = settings.overscan[nes_video_region_index(region)];
        unsigned width = 256 - crop.left - crop.right;
        unsigned height = 240 - crop.top - crop.bottom;
        CHECK(frame.width == width * 2 && frame.height == height && frame.screens == 2);
        CHECK(!frame.filtered && frame.layers_ready);
        for (unsigned y = 0; y < height; ++y) {
            for (unsigned x = 0; x < width * 2; ++x) {
                unsigned input_x = (x / width) * 256 + crop.left + x % width;
                CHECK(frame.pixels[y * width * 2 + x] == raw_pixels[(y + crop.top) * 512 + input_x]);
            }
        }
        unsigned side = 3;
        int x, y;
        CHECK(nes_video_presentation_aim(&frame, (int)width, 0, &side, &x, &y));
        CHECK(side == 1 && x == (int)crop.left && y == (int)crop.top);
        CHECK(nes_video_presentation_aim(&frame, (int)frame.width - 1, (int)frame.height - 1,
                                          &side, &x, &y));
        CHECK(side == 1 && x == (int)(255 - crop.right) && y == (int)(239 - crop.bottom));
        CHECK(!nes_video_presentation_aim(&frame, -1, 0, &side, &x, &y) && x == -1 && y == -1);
        CHECK(!nes_video_presentation_aim(&frame, (int)frame.width, 0, &side, &x, &y));
    }

    NesVideoPresentationSettings invalid = settings, actual;
    invalid.overscan[1].left = 256;
    CHECK(!nes_video_presentation_set(&invalid, error, sizeof(error)) && error[0]);
    nes_video_presentation_get(&actual);
    CHECK(!memcmp(&settings, &actual, sizeof(settings)));
    CHECK(!nes_video_overscan_valid((NesVideoOverscan){128, 128, 0, 0}));
    CHECK(!nes_video_overscan_valid((NesVideoOverscan){0, 0, 120, 120}));
    CHECK(nes_video_overscan_valid((NesVideoOverscan){255, 0, 239, 0}));
    return 0;
}

static int test_composite_crop_coordinates(void) {
    NesVideoPresentationSettings settings;
    nes_video_presentation_defaults(&settings);
    char error[160];
    CHECK(nes_video_presentation_set(&settings, error, sizeof(error)));
    for (unsigned i = 0; i < 256 * 240; ++i) {
        raw_pixels[i] = 0xFF000000u | i;
        raw_signal[i] = (uint16_t)((i / 7) % 64);
    }
    NesVideoPresentationSource source = {
        .pixels = raw_pixels, .signals = {raw_signal, NULL}, .phases = {2, 0},
        .screens = 1, .region = NES_REGION_NTSC, .composite = true
    };
    NesVideoPresentationFrame frame;
    ntsc_composite_filter_frame(raw_signal, 2, filtered);
    CHECK(nes_video_presentation_render(&source, &frame, error, sizeof(error)));
    CHECK(frame.filtered && frame.width == NTSC_COMPOSITE_WIDTH && frame.height == NTSC_COMPOSITE_HEIGHT);
    CHECK(!memcmp(frame.pixels, filtered, sizeof(filtered)));

    settings.overscan[0] = (NesVideoOverscan){8, 12, 7, 11};
    CHECK(nes_video_presentation_set(&settings, error, sizeof(error)));
    CHECK(nes_video_presentation_render(&source, &frame, error, sizeof(error)));
    CHECK(frame.width == 472 && frame.height == 444);
    for (unsigned y = 0; y < frame.height; ++y)
        CHECK(!memcmp(frame.pixels + y * frame.width, filtered + (y + 14) * 512 + 16,
                      frame.width * sizeof(uint32_t)));
    unsigned side;
    int x, y;
    CHECK(nes_video_presentation_aim(&frame, 0, 0, &side, &x, &y));
    CHECK(side == 0 && x == 8 && y == 7);
    CHECK(nes_video_presentation_aim(&frame, 471, 443, &side, &x, &y));
    CHECK(x == 243 && y == 228);

    source.signals[0] = NULL;
    CHECK(!nes_video_presentation_render(&source, &frame, error, sizeof(error)) && error[0]);
    source.region = NES_REGION_PAL;
    CHECK(nes_video_presentation_render(&source, &frame, error, sizeof(error)));
    CHECK(!frame.filtered && frame.width == 256 && frame.height == 240);
    CHECK(!memcmp(frame.pixels, raw_pixels, 256 * 240 * sizeof(uint32_t)));
    source.region = NES_REGION_NTSC;
    source.vs_system = true;
    CHECK(nes_video_presentation_render(&source, &frame, error, sizeof(error)));
    CHECK(!frame.filtered && frame.width == 236 && frame.height == 222);
    return 0;
}

static bool prepare_layer_machine(void) {
    BoardImage image = {0};
    if (!board_image_create(&image, 0, 0x8000, 0x2000, true)) return false;
    uint8_t *prg = image.data + 16;
    memset(prg, 0xEA, 0x8000);
    const uint8_t program[] = {0xA9, 0x1E, 0x8D, 1, 0x20, 0x4C, 5, 0x80};
    memcpy(prg, program, sizeof(program));
    for (unsigned vector = 0x7FFA; vector <= 0x7FFE; vector += 2) {
        prg[vector] = 0;
        prg[vector + 1] = 0x80;
    }
    uint8_t *chr = prg + 0x8000;
    memset(chr, 0, 0x2000);
    memset(chr, 0xFF, 8);
    memset(chr + 24, 0xFF, 8);
    bool loaded = load_rom_memory(image.data, image.size) == 0;
    board_image_free(&image);
    if (!loaded) return false;
    cpu_select_machine(NULL);
    cpu_use_default_startup_alignment();
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    if (!cpu_power_on(&cpu)) return false;
    memset(ppu_vram, 0, NT_RAM_SIZE);
    memset(ppu.oam, 0xFF, sizeof(ppu.oam));
    ppu.oam[0] = 20;
    ppu.oam[1] = 1;
    ppu.oam[2] = 0;
    ppu.oam[3] = 30;
    ppu.oam[4] = 20;
    ppu.oam[5] = 1;
    ppu.oam[6] = 0x20;
    ppu.oam[7] = 50;
    ppu_write(0x3F00, 0x0F);
    ppu_write(0x3F01, 0x21);
    ppu_write(0x3F12, 0x26);
    for (unsigned frame = 0; frame < 2; ++frame) {
        vs_start_frame();
        while (!ppu.frame_complete) if (vs_cpu_step() <= 0) return false;
    }
    return true;
}

static int test_layer_output_keeps_hardware_and_sensor(void) {
    NesVideoPresentationSettings settings;
    nes_video_presentation_defaults(&settings);
    char error[160];
    CHECK(nes_video_presentation_set(&settings, error, sizeof(error)));
    CHECK(nes_video_trace_use(NES_VIDEO_TRACE_EXPORT, true));
    CHECK(prepare_layer_machine());
    NesStateBlob before = {0}, after = {0};
    CHECK(nes_state_capture(&before) == NES_STATE_OK);
    uint16_t brightness = ppu_pixel_brightness(100, 100);
    CHECK(brightness > 0);
    for (unsigned mode = 0; mode < 4; ++mode) {
        settings.show_background = (mode & 1) != 0;
        settings.show_sprites = (mode & 2) != 0;
        CHECK(nes_video_presentation_set(&settings, error, sizeof(error)));
        NesVideoPresentationSource source = {
            .pixels = framebuffer, .signals = {ppu.pixel_signal, NULL},
            .trace = {nes_video_trace_frame(0), NULL}, .phases = {ppu.completed_video_phase, 0},
            .screens = 1, .region = nes_timing()->region
        };
        CHECK(source.trace[0] && source.trace[0]->complete);
        NesVideoPresentationFrame frame;
        CHECK(nes_video_presentation_render(&source, &frame, error, sizeof(error)));
        CHECK(frame.width == 256 && frame.height == 240 && frame.layers_ready);
        const NesVideoPixel *front = source.trace[0]->pixels + 21 * 256 + 31;
        const NesVideoPixel *behind = source.trace[0]->pixels + 21 * 256 + 51;
        CHECK(front->selected_sprite_index && behind->selected_sprite_index);
        CHECK(frame.pixels[21 * 256 + 31] == (settings.show_sprites ? front->selected_sprite_rgb
            : settings.show_background ? front->background.rgb : front->backdrop_rgb));
        CHECK(frame.pixels[21 * 256 + 51] == (settings.show_background ? behind->background.rgb
            : settings.show_sprites ? behind->selected_sprite_rgb : behind->backdrop_rgb));
        CHECK(ppu_pixel_brightness(100, 100) == brightness);
        if (mode == 3) CHECK(!memcmp(frame.pixels, framebuffer, 256 * 240 * sizeof(uint32_t)));
    }
    CHECK(nes_state_capture(&after) == NES_STATE_OK);
    CHECK(before.size == after.size && !memcmp(before.data, after.data, before.size));
    nes_state_blob_free(&before);
    nes_state_blob_free(&after);

    settings.show_background = false;
    CHECK(nes_video_presentation_set(&settings, error, sizeof(error)));
    nes_video_trace_reset_side(0);
    NesVideoPresentationSource unavailable = {.pixels = framebuffer, .screens = 1,
        .trace = {nes_video_trace_frame(0), NULL}, .region = NES_REGION_NTSC};
    NesVideoPresentationFrame fallback;
    CHECK(nes_video_presentation_render(&unavailable, &fallback, error, sizeof(error)));
    CHECK(!fallback.layers_ready && !memcmp(fallback.pixels, framebuffer, 256 * 240 * sizeof(uint32_t)));
    CHECK(nes_video_trace_use(NES_VIDEO_TRACE_EXPORT, false));
    CHECK(unload_rom());
    return 0;
}

int test_video_presentation_accuracy(void) {
    NesVideoPresentationSettings previous;
    nes_video_presentation_get(&previous);
    NesRegionMode region = nes_region_mode();
    CHECK(nes_set_region_mode(NES_REGION_MODE_AUTO));
    int failures = test_regional_dual_screen_crop() + test_composite_crop_coordinates()
                 + test_layer_output_keeps_hardware_and_sensor();
    if (!nes_video_presentation_set(&previous, NULL, 0)) ++failures;
    if (!nes_set_region_mode(region)) ++failures;
    printf("Video presentation: 3 groups, %d failures\n", failures);
    return failures;
}
