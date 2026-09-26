/*
 * ntsc_settings_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Composite controls, persistence, phase handling and rejected input. SPDX-License-Identifier: GPL-3.0-or-later */
#include "../video/ntsc_composite.h"
#include "../video/presentation.h"
#include "../ui/settings.h"
#include "../util/file_io.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

enum { FRAME_PIXELS = NTSC_COMPOSITE_WIDTH * NTSC_COMPOSITE_HEIGHT };

static int failures;
static uint16_t signal_pixels[256 * 240], signal_copy[256 * 240];
static uint32_t original[3][FRAME_PIXELS], result[FRAME_PIXELS + 2], raw[256 * 240];

#define CHECK(condition)                                                                                               \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            fprintf(stderr, "NTSC settings check %d failed: %s\n", __LINE__, #condition);                              \
            ++failures;                                                                                                \
        }                                                                                                              \
    } while (0)

static uint64_t frame_hash(const uint32_t *pixels) {
    uint64_t hash = UINT64_C(1469598103934665603);
    for (size_t i = 0; i < FRAME_PIXELS; ++i) {
        for (unsigned shift = 0; shift < 32; shift += 8) {
            hash = (hash ^ ((pixels[i] >> shift) & 255u)) * UINT64_C(1099511628211);
        }
    }
    return hash;
}

static bool render(const NtscCompositeSettings *settings, uint8_t phase) {
    char error[160];
    result[0] = UINT32_C(0xDEADBEEF);
    result[FRAME_PIXELS + 1] = UINT32_C(0x12345678);
    bool ok = ntsc_composite_filter_frame_settings(signal_pixels, phase, result + 1, settings, error, sizeof(error));
    CHECK(ok);
    if (!ok) {
        fprintf(stderr, "%s\n", error);
    }
    CHECK(result[0] == UINT32_C(0xDEADBEEF) && result[FRAME_PIXELS + 1] == UINT32_C(0x12345678));
    CHECK(!memcmp(signal_pixels, signal_copy, sizeof(signal_pixels)));
    return ok;
}

static void decoder_controls(void) {
    NtscCompositeSettings settings;
    ntsc_composite_defaults(&settings);
    /* The existing independently compiled decoder fixture is unchanged. */
    const uint64_t expected[] = {UINT64_C(0xA645DCD5E139059F), UINT64_C(0xCA5E00539D7CD747),
                                 UINT64_C(0x6CA5D1B39F66106F)};
    for (uint8_t phase = 0; phase < 3; ++phase) {
        if (render(&settings, phase)) {
            memcpy(original[phase], result + 1, sizeof(original[phase]));
            CHECK(frame_hash(original[phase]) == expected[phase]);
            ntsc_composite_filter_frame(signal_pixels, phase, result + 1);
            CHECK(!memcmp(original[phase], result + 1, sizeof(original[phase])));
        }
    }
    const int adjusted[] = {30, 25, 75, 0, 12, 24, 72, 200, 50};
    for (unsigned control = 0; control < NTSC_CONTROL_FIELDS; ++control) {
        ntsc_composite_defaults(&settings);
        *ntsc_composite_control_value(&settings, control) = adjusted[control];
        if (render(&settings, 0)) {
            CHECK(memcmp(original[0], result + 1, sizeof(original[0])) != 0);
        }
    }
    ntsc_composite_defaults(&settings);
    settings.contrast = 0;
    if (render(&settings, 0)) {
        bool black = true;
        for (unsigned i = 0; i < FRAME_PIXELS; ++i) {
            black &= result[i + 1] == UINT32_C(0xFF000000);
        }
        CHECK(black);
    }
    ntsc_composite_defaults(&settings);
    settings.saturation = 0;
    if (render(&settings, 0)) {
        bool gray = true;
        for (unsigned i = 0; i < FRAME_PIXELS; ++i) {
            uint32_t p = result[i + 1];
            gray &= ((p >> 16) & 255u) == ((p >> 8) & 255u) && (p & 255u) == ((p >> 8) & 255u);
        }
        CHECK(gray);
    }
    ntsc_composite_defaults(&settings);
    settings.scanlines = 50;
    if (render(&settings, 0)) {
        bool cells = true;
        for (unsigned y = 0; y < NTSC_COMPOSITE_HEIGHT; y += 2) {
            for (unsigned x = 0; x < NTSC_COMPOSITE_WIDTH; ++x) {
                unsigned i = y * NTSC_COMPOSITE_WIDTH + x;
                uint32_t p = original[0][i];
                uint32_t dim = UINT32_C(0xFF000000) | (((p >> 16) & 255u) / 2 << 16) | (((p >> 8) & 255u) / 2 << 8) |
                               ((p & 255u) / 2);
                cells &= result[i + 1] == p && result[i + NTSC_COMPOSITE_WIDTH + 1] == dim;
            }
        }
        CHECK(cells);
    }
    ntsc_composite_defaults(&settings);
    settings.fields = NTSC_FIELDS_FIXED;
    if (render(&settings, 2)) {
        CHECK(!memcmp(original[0], result + 1, sizeof(original[0])));
    }
    settings.fields = NTSC_FIELDS_BLEND;
    for (uint8_t phase = 0; phase < 3; ++phase) {
        if (!render(&settings, phase)) {
            continue;
        }
        bool averaged = true;
        for (unsigned i = 0; i < FRAME_PIXELS; ++i) {
            uint32_t mean = UINT32_C(0xFF000000);
            for (unsigned shift = 0; shift <= 16; shift += 8) {
                unsigned sum = 0;
                for (unsigned p = 0; p < 3; ++p) {
                    sum += (original[p][i] >> shift) & 255u;
                }
                mean |= (sum / 3) << shift;
            }
            averaged &= result[i + 1] == mean;
        }
        CHECK(averaged);
    }
}

static void rejected_and_extreme_controls(void) {
    NtscCompositeSettings settings;
    char error[160];
    CHECK(!ntsc_composite_validate(NULL, error, sizeof(error)) && error[0]);
    CHECK(ntsc_composite_control_info(NTSC_CONTROL_COUNT) == NULL);
    CHECK(ntsc_composite_control_value(NULL, 0) == NULL);
    CHECK(ntsc_composite_control_value(&settings, NTSC_CONTROL_COUNT) == NULL);
    for (unsigned id = 0; id < NTSC_CONTROL_COUNT; ++id) {
        const NtscCompositeControl *control = ntsc_composite_control_info(id);
        for (unsigned bound = 0; bound < 2; ++bound) {
            ntsc_composite_defaults(&settings);
            int *value = ntsc_composite_control_value(&settings, id);
            *value = bound ? control->maximum + 1 : control->minimum - 1;
            memcpy(result + 1, original[0], sizeof(original[0]));
            CHECK(!ntsc_composite_validate(&settings, error, sizeof(error)) && error[0]);
            CHECK(!ntsc_composite_filter_frame_settings(signal_pixels, 0, result + 1, &settings, error, sizeof(error)));
            CHECK(!memcmp(original[0], result + 1, sizeof(original[0])));
            *value = bound ? control->maximum : control->minimum;
            CHECK(ntsc_composite_validate(&settings, error, sizeof(error)));
            (void)render(&settings, 2);
        }
    }
    /* Exercise narrow luma, wide chroma and high gains together under UBSan. */
    for (unsigned pass = 0; pass < 4; ++pass) {
        ntsc_composite_defaults(&settings);
        settings.contrast = settings.saturation = 200;
        settings.y_width = pass & 1u ? 96 : 1;
        settings.i_width = pass & 2u ? 96 : 12;
        settings.q_width = pass & 2u ? 12 : 96;
        settings.brightness = pass & 1u ? 100 : -100;
        settings.hue = pass & 2u ? 180 : -180;
        (void)render(&settings, 255);
    }
    CHECK(!ntsc_composite_filter_frame_settings(NULL, 0, result, NULL, error, sizeof(error)));
    CHECK(!ntsc_composite_filter_frame_settings(signal_pixels, 0, NULL, NULL, error, sizeof(error)));
}

static void presentation_controls(void) {
    NtscCompositeSettings settings;
    ntsc_composite_defaults(&settings);
    settings.hue = 30;
    settings.fields = NTSC_FIELDS_BLEND;
    settings.scanlines = 40;
    NesVideoPresentationSettings previous, presentation;
    nes_video_presentation_get(&previous);
    nes_video_presentation_defaults(&presentation);
    presentation.overscan[0] = (NesVideoOverscan){3, 5, 7, 9};
    char error[160];
    CHECK(nes_video_presentation_set(&presentation, error, sizeof(error)));
    NesVideoPresentationSource source = {.pixels = raw,
                                         .signals = {signal_pixels, NULL},
                                         .screens = 1,
                                         .region = NES_REGION_NTSC,
                                         .composite = true,
                                         .ntsc_settings = &settings};
    NesVideoPresentationFrame frame;
    if (render(&settings, 0) && nes_video_presentation_render(&source, &frame, error, sizeof(error))) {
        CHECK(frame.width == 496 && frame.height == 448 && frame.filtered);
        bool cropped = true;
        for (unsigned y = 0; y < frame.height; ++y) {
            cropped &= !memcmp(frame.pixels + y * frame.width, result + 1 + (y + 14) * NTSC_COMPOSITE_WIDTH + 6,
                               frame.width * sizeof(uint32_t));
        }
        CHECK(cropped);
        unsigned side;
        int x, y;
        CHECK(nes_video_presentation_aim(&frame, 0, 0, &side, &x, &y) && side == 0 && x == 3 && y == 7);
        CHECK(nes_video_presentation_aim(&frame, 495, 447, &side, &x, &y) && x == 250 && y == 230);
    } else {
        CHECK(false);
    }
    for (unsigned mode = 0; mode < 3; ++mode) {
        source.region = mode == 0 ? NES_REGION_PAL : mode == 1 ? NES_REGION_DENDY : NES_REGION_NTSC;
        source.vs_system = mode == 2;
        CHECK(nes_video_presentation_render(&source, &frame, error, sizeof(error)) && !frame.filtered);
        CHECK(frame.pixels[0] == raw[0]);
    }
    CHECK(nes_video_presentation_set(&previous, error, sizeof(error)));
}

static void settings_and_presets(void) {
    const char *path = "build/ntsc-picture-settings.ini";
    FrontendSettings settings, loaded;
    FrontendSettingsReport report;
    frontend_settings_defaults(&settings);
    for (unsigned preset = 0; preset < NTSC_PRESET_COUNT; ++preset) {
        CHECK(ntsc_composite_preset(&settings.ntsc_picture, preset));
        CHECK(ntsc_composite_preset_index(&settings.ntsc_picture) == (int)preset);
        CHECK(frontend_settings_save(path, &settings, &report));
        CHECK(frontend_settings_load(path, &loaded, &report));
        CHECK(ntsc_composite_preset_index(&loaded.ntsc_picture) == (int)preset);
    }
    settings.ntsc_picture.hue = -45;
    settings.ntsc_picture.brightness = -10;
    settings.ntsc_picture.gamma = 125;
    CHECK(ntsc_composite_preset_index(&settings.ntsc_picture) == -1);
    CHECK(frontend_settings_save(path, &settings, &report));
    CHECK(frontend_settings_load(path, &loaded, &report));
    for (unsigned id = 0; id < NTSC_CONTROL_COUNT; ++id) {
        CHECK(ntsc_composite_control_get(&settings.ntsc_picture, id) ==
              ntsc_composite_control_get(&loaded.ntsc_picture, id));
    }
    const char *invalid[] = {
        "ntsc.hue=181",    "ntsc.brightness=-101", "ntsc.gamma=0",       "ntsc.luma_width=0", "ntsc.i_width=11",
        "ntsc.q_width=97", "ntsc.fields=3",        "ntsc.scanlines=NaN", "ntsc.hue=2.5",      "ntsc.=2"};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        char text[96];
        snprintf(text, sizeof(text), "version=4\n%s\n", invalid[i]);
        CHECK(nes_file_write_atomic(path, text, strlen(text)) == NES_FILE_OK);
        CHECK(!frontend_settings_load(path, &loaded, &report) && report.message[0]);
        CHECK(loaded.ntsc_picture.hue == -45 && loaded.ntsc_picture.gamma == 125);
    }
    const char legacy[] = "version=3\n";
    CHECK(nes_file_write_atomic(path, legacy, sizeof(legacy) - 1) == NES_FILE_OK);
    CHECK(frontend_settings_load(path, &loaded, &report));
    CHECK(ntsc_composite_preset_index(&loaded.ntsc_picture) == 0);
    settings.ntsc_picture.gamma = 0;
    CHECK(!frontend_settings_save(path, &settings, &report));
    CHECK(frontend_settings_load(path, &loaded, &report) && ntsc_composite_preset_index(&loaded.ntsc_picture) == 0);
    CHECK(!ntsc_composite_preset(&loaded.ntsc_picture, NTSC_PRESET_COUNT));
    CHECK(nes_file_remove(path) == NES_FILE_OK);
}

int test_ntsc_settings_accuracy(void) {
    failures = 0;
    for (unsigned y = 0; y < 240; ++y) {
        for (unsigned x = 0; x < 256; ++x) {
            signal_pixels[y * 256 + x] = (uint16_t)((x * 29u + y * 67u + 12u) & 511u);
            raw[y * 256 + x] = UINT32_C(0xFF112233);
        }
    }
    memcpy(signal_copy, signal_pixels, sizeof(signal_pixels));
    decoder_controls();
    rejected_and_extreme_controls();
    presentation_controls();
    settings_and_presets();
    printf("NTSC picture: controls, presets, phase blending, crop/aim, defaults and persistence, %d failures\n",
           failures);
    return failures;
}
