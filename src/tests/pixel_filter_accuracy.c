/*
 * pixel_filter_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Presentation filter vectors and rejected input. SPDX-License-Identifier: GPL-3.0-or-later */
#include "../video/pixel_filter.h"
#include "../ui/settings.h"
#include "../util/file_io.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static int failures;

#define CHECK(condition)                                                                                               \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            fprintf(stderr, "Pixel filter check %d failed: %s\n", __LINE__, #condition);                               \
            ++failures;                                                                                                \
        }                                                                                                              \
    } while (0)

static void lcd_vectors(NesPixelFilter *filter) {
    const uint32_t source[] = {0xFFFFFFFFu, 0x00804020u};
    const uint32_t expected[] = {0xFFFFFFFFu, 0xFFD8D8D8u, 0xFF804020u, 0xFF6C361Bu,
                                 0xFFD8D8D8u, 0xFFD8D8D8u, 0xFF6C361Bu, 0xFF6C361Bu};
    NesVideoPresentationFrame input = {
        .pixels = source, .width = 2, .height = 1, .screens = 2, .overscan = {7, 3, 2, 9}, .layers_ready = true};
    NesVideoPresentationFrame output;
    NesPixelFilterSettings settings;
    nes_pixel_filter_defaults(&settings);
    char error[160];
    CHECK(nes_pixel_filter_render(filter, &settings, &input, &output, error, sizeof(error)));
    CHECK(output.pixels == source && output.width == 2 && output.height == 1);
    settings.kind = NES_PIXEL_FILTER_LCD;
    CHECK(nes_pixel_filter_render(filter, &settings, &input, &output, error, sizeof(error)));
    CHECK(output.width == 4 && output.height == 2 && output.screens == 2 && output.layers_ready);
    CHECK(!memcmp(&output.overscan, &input.overscan, sizeof(input.overscan)));
    CHECK(!memcmp(output.pixels, expected, sizeof(expected)));
    CHECK(source[0] == 0xFFFFFFFFu && source[1] == 0x00804020u);
    settings.lcd_brightness[0] = 0;
    settings.lcd_brightness[1] = 50;
    settings.lcd_brightness[2] = settings.lcd_brightness[3] = 100;
    CHECK(nes_pixel_filter_render(filter, &settings, &input, &output, error, sizeof(error)));
    CHECK(output.pixels[0] == 0xFF000000u && output.pixels[1] == 0xFF7F7F7Fu);
    CHECK(output.pixels[2] == 0xFF000000u && output.pixels[3] == 0xFF3F1F0Fu);
    CHECK(output.pixels[4] == source[0] && output.pixels[7] == (source[1] | 0xFF000000u));

    /* An output frame can share the metadata object, but not the pixel storage. */
    CHECK(!nes_pixel_filter_render(filter, &settings, &output, &output, error, sizeof(error)) && error[0]);
    NesVideoPresentationFrame slice = {.pixels = output.pixels + 1, .width = 1, .height = 1, .screens = 1};
    CHECK(!nes_pixel_filter_render(filter, &settings, &slice, &output, error, sizeof(error)) && error[0]);
    output = input;
    CHECK(nes_pixel_filter_render(filter, &settings, &output, &output, error, sizeof(error)));
    CHECK(output.width == 4 && output.height == 2);
    settings.lcd_brightness[1] = 101;
    NesVideoPresentationFrame retained = output;
    CHECK(!nes_pixel_filter_render(filter, &settings, &input, &output, error, sizeof(error)) && error[0]);
    CHECK(!memcmp(&retained, &output, sizeof(output)));
    nes_pixel_filter_defaults(&settings);
    settings.kind = NES_PIXEL_FILTER_LCD;
    input.width = 8192;
    CHECK(!nes_pixel_filter_render(filter, &settings, &input, &output, error, sizeof(error)) && error[0]);
    input.width = 3;
    CHECK(!nes_pixel_filter_render(filter, &settings, &input, &output, error, sizeof(error)) && error[0]);
    input.width = 2;
    input.pixels = NULL;
    CHECK(!nes_pixel_filter_render(filter, &settings, &input, &output, error, sizeof(error)) && error[0]);
    settings.kind = (NesPixelFilterKind)-1;
    CHECK(!nes_pixel_filter_validate(&settings, error, sizeof(error)));
    CHECK(!nes_pixel_filter_validate(NULL, error, sizeof(error)));
    CHECK(!strcmp(nes_pixel_filter_name(settings.kind), "Invalid"));
    const uint32_t invalid_ids[] = {NES_PIXEL_FILTER_COUNT, NES_PIXEL_FILTER_COUNT + 1u, UINT32_C(0x80000000),
                                    UINT32_MAX};
    for (unsigned i = 0; i < sizeof(invalid_ids) / sizeof(invalid_ids[0]); ++i) {
        settings.kind = invalid_ids[i];
        CHECK(!nes_pixel_filter_validate(&settings, error, sizeof(error)) && error[0]);
        CHECK(!strcmp(nes_pixel_filter_name(settings.kind), "Invalid"));
        CHECK(!strcmp(nes_pixel_filter_key(settings.kind), "invalid"));
        CHECK(nes_pixel_filter_scale(settings.kind) == 0);
    }
    for (unsigned i = 0; i < NES_PIXEL_FILTER_COUNT; ++i) {
        NesPixelFilterKind parsed = NES_PIXEL_FILTER_COUNT;
        CHECK(nes_pixel_filter_from_key(nes_pixel_filter_key((NesPixelFilterKind)i), &parsed));
        CHECK(parsed == (NesPixelFilterKind)i);
    }
    NesPixelFilterKind parsed = NES_PIXEL_FILTER_LCD;
    CHECK(!nes_pixel_filter_from_key("unknown", &parsed) && parsed == NES_PIXEL_FILTER_LCD);
}

static uint64_t pixel_hash(const uint32_t *pixels, size_t count) {
    uint64_t value = UINT64_C(14695981039346656037);
    for (size_t i = 0; i < count; ++i) {
        for (unsigned shift = 0; shift < 32; shift += 8) {
            value = (value ^ ((pixels[i] >> shift) & 255u)) * UINT64_C(1099511628211);
        }
    }

    return value;
}

static const uint32_t vector_pixels[25] = {
    0xFF000000, 0xFFFFFFFF, 0xFF000000, 0xFF00FF00, 0xFF00FF00, 0xFFFFFFFF, 0xFFFFFFFF, 0xFF000000, 0xFF00FF00,
    0xFF0000FF, 0xFF000000, 0xFF000000, 0xFFFFFFFF, 0xFF0000FF, 0xFF0000FF, 0xFF804020, 0xFF804020, 0xFF000000,
    0xFFFFFFFF, 0xFF000000, 0xFF804020, 0xFFFF0000, 0xFF000000, 0xFF000000, 0xFFFFFFFF};

static void scaler_vectors(NesPixelFilter *filter) {
    /* Independent component build at revision 20f497c9620a7f4e55b7752e1d22734f9d7492fa.
     * Hashes cover every output byte, including edges, interpolated colors and alpha.
     * The reference uses the original Scale4x dispatcher rather than our row adapter. */
    static const struct {
        NesPixelFilterKind kind;
        unsigned factor;
        uint64_t expected;
    } vectors[] = {{NES_PIXEL_FILTER_XBRZ2, 2, UINT64_C(0x91F542BC28A5468C)},
                   {NES_PIXEL_FILTER_XBRZ3, 3, UINT64_C(0x2F86A52448275AB5)},
                   {NES_PIXEL_FILTER_XBRZ4, 4, UINT64_C(0xB4F9393D45FE834D)},
                   {NES_PIXEL_FILTER_XBRZ5, 5, UINT64_C(0x1B65E5AA1A46DB04)},
                   {NES_PIXEL_FILTER_XBRZ6, 6, UINT64_C(0xF83C16300A349A55)},
                   {NES_PIXEL_FILTER_HQ2X, 2, UINT64_C(0xCCC1034AEAAD3FB9)},
                   {NES_PIXEL_FILTER_HQ3X, 3, UINT64_C(0x60FCCE20D992A709)},
                   {NES_PIXEL_FILTER_HQ4X, 4, UINT64_C(0x0D4F17A9562B00B4)},
                   {NES_PIXEL_FILTER_SCALE2X, 2, UINT64_C(0x55E23F371650F6AC)},
                   {NES_PIXEL_FILTER_SCALE3X, 3, UINT64_C(0xBB5247FA12D2384C)},
                   {NES_PIXEL_FILTER_SCALE4X, 4, UINT64_C(0x4694375BE6D5FEDC)},
                   {NES_PIXEL_FILTER_2XSAI, 2, UINT64_C(0xD3CE02F3C29E1824)},
                   {NES_PIXEL_FILTER_SUPER2XSAI, 2, UINT64_C(0x6607326C71E3480F)},
                   {NES_PIXEL_FILTER_SUPEREAGLE, 2, UINT64_C(0x70AD979E4F15B28A)}};

    NesVideoPresentationFrame input = {.pixels = vector_pixels, .width = 5, .height = 5, .screens = 1};
    NesPixelFilterSettings settings;
    nes_pixel_filter_defaults(&settings);
    char error[160];
    for (unsigned i = 0; i < sizeof(vectors) / sizeof(vectors[0]); ++i) {
        settings.kind = vectors[i].kind;
        CHECK(nes_pixel_filter_scale(settings.kind) == vectors[i].factor);
        NesVideoPresentationFrame output = {0};
        bool ok = nes_pixel_filter_render(filter, &settings, &input, &output, error, sizeof(error));
        CHECK(ok);
        if (!ok) {
            continue;
        }

        CHECK(output.width == 5 * vectors[i].factor && output.height == output.width);
        uint64_t actual = pixel_hash(output.pixels, (size_t)output.width * output.height);
        if (actual != vectors[i].expected) {
            fprintf(stderr, "%s vector: expected %016" PRIX64 ", got %016" PRIX64 "\n",
                    nes_pixel_filter_name(settings.kind), vectors[i].expected, actual);
        }

        CHECK(actual == vectors[i].expected);
    }

    for (int kind = NES_PIXEL_FILTER_PRESCALE2; kind < NES_PIXEL_FILTER_COUNT; ++kind) {
        settings.kind = (NesPixelFilterKind)kind;
        unsigned factor = nes_pixel_filter_scale(settings.kind);
        NesVideoPresentationFrame output = {0};
        CHECK(nes_pixel_filter_render(filter, &settings, &input, &output, error, sizeof(error)));
        CHECK(output.width == 5 * factor && output.height == 5 * factor);
        for (unsigned y = 0; y < output.height; ++y) {
            for (unsigned x = 0; x < output.width; ++x) {
                CHECK(output.pixels[(size_t)y * output.width + x] == vector_pixels[(y / factor) * 5 + x / factor]);
            }
        }
    }
}

static void scaler_boundaries(NesPixelFilter *filter) {
    NesPixelFilter *single = nes_pixel_filter_create();
    CHECK(single != NULL);
    if (!single) {
        return;
    }

    uint32_t dual[50], sides[2][25];
    for (unsigned i = 0; i < 25; ++i) {
        sides[0][i] = vector_pixels[i];
        sides[1][i] = vector_pixels[24 - i] ^ 0x00FFFFFFu;
        dual[(i / 5) * 10 + i % 5] = sides[0][i];
        dual[(i / 5) * 10 + 5 + i % 5] = sides[1][i];
    }

    const uint64_t retained = pixel_hash(dual, 50);
    char error[160];
    NesPixelFilterSettings settings;
    nes_pixel_filter_defaults(&settings);
    for (int kind = NES_PIXEL_FILTER_XBRZ2; kind < NES_PIXEL_FILTER_COUNT; ++kind) {
        settings.kind = (NesPixelFilterKind)kind;
        unsigned factor = nes_pixel_filter_scale(settings.kind);
        NesVideoPresentationFrame input = {.pixels = dual,
                                           .width = 10,
                                           .height = 5,
                                           .screens = 2,
                                           .overscan = {1, 2, 3, 4},
                                           .filtered = true,
                                           .layers_ready = true};
        NesVideoPresentationFrame output = {0};
        bool ok = nes_pixel_filter_render(filter, &settings, &input, &output, error, sizeof(error));
        CHECK(ok);
        if (!ok) {
            continue;
        }

        CHECK(output.width == 10 * factor && output.height == 5 * factor && output.screens == 2);
        CHECK(output.filtered && output.layers_ready);
        CHECK(!memcmp(&input.overscan, &output.overscan, sizeof(input.overscan)));
        CHECK(pixel_hash(dual, 50) == retained);
        for (unsigned side = 0; side < 2; ++side) {
            NesVideoPresentationFrame one = {.pixels = sides[side], .width = 5, .height = 5, .screens = 1};
            NesVideoPresentationFrame scaled = {0};
            CHECK(nes_pixel_filter_render(single, &settings, &one, &scaled, error, sizeof(error)));
            for (unsigned y = 0; y < scaled.height; ++y) {
                CHECK(!memcmp(output.pixels + (size_t)y * output.width + side * scaled.width,
                              scaled.pixels + (size_t)y * scaled.width, scaled.width * sizeof(uint32_t)));
            }
        }

        NesVideoPresentationFrame unchanged = output;
        input.width = 8192 / factor + 1;
        input.screens = 1;
        CHECK(!nes_pixel_filter_render(filter, &settings, &input, &output, error, sizeof(error)) && error[0]);
        CHECK(!memcmp(&unchanged, &output, sizeof(output)));
        input.width = 5;
        input.height = 8192 / factor + 1;
        CHECK(!nes_pixel_filter_render(filter, &settings, &input, &output, error, sizeof(error)) && error[0]);
        CHECK(!memcmp(&unchanged, &output, sizeof(output)));

        /* A constant crop has no edges to smooth, including a one-pixel crop. */
        const unsigned dimensions[][2] = {{1, 1}, {1, 4}, {4, 1}, {2, 2}};
        const uint32_t white[] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
        for (unsigned size = 0; size < 4; ++size) {
            input = (NesVideoPresentationFrame){
                .pixels = white, .width = dimensions[size][0], .height = dimensions[size][1], .screens = 1};
            CHECK(nes_pixel_filter_render(filter, &settings, &input, &output, error, sizeof(error)));
            CHECK(output.width == input.width * factor && output.height == input.height * factor);
            for (size_t i = 0; i < (size_t)output.width * output.height; ++i) {
                CHECK(output.pixels[i] == 0xFFFFFFFF);
            }
        }
    }

    nes_pixel_filter_destroy(single);
}

static void settings_files(void) {
    const char *path = "build/pixel-filter-settings.ini";
    FrontendSettings settings, loaded;
    FrontendSettingsReport report;
    frontend_settings_defaults(&settings);
    settings.pixel_filter.kind = NES_PIXEL_FILTER_LCD;
    settings.pixel_filter.lcd_brightness[0] = 0;
    settings.pixel_filter.lcd_brightness[1] = 1;
    settings.pixel_filter.lcd_brightness[2] = 50;
    settings.pixel_filter.lcd_brightness[3] = 100;
    CHECK(frontend_settings_save(path, &settings, &report));
    CHECK(frontend_settings_load(path, &loaded, &report));
    CHECK(!memcmp(&settings.pixel_filter, &loaded.pixel_filter, sizeof(settings.pixel_filter)));
    static const char *const rejected[] = {"version=4\npixel_filter=unknown\n", "version=4\nlcd_brightness.0=101\n",
                                           "version=4\nlcd_brightness.1=-1\n",  "version=4\nlcd_brightness.4=50\n",
                                           "version=4\nlcd_brightness.=50\n",   "version=4\nlcd_brightness.00=50\n",
                                           "version=4\nlcd_brightness.3=NaN\n", "version=4\nlcd_brightness.2=50.5\n"};
    for (unsigned i = 0; i < sizeof(rejected) / sizeof(rejected[0]); ++i) {
        CHECK(nes_file_write_atomic(path, rejected[i], strlen(rejected[i])) == NES_FILE_OK);
        CHECK(!frontend_settings_load(path, &loaded, &report) && report.message[0]);
        CHECK(!memcmp(&settings.pixel_filter, &loaded.pixel_filter, sizeof(settings.pixel_filter)));
    }
    const char legacy[] = "version=3\n";
    CHECK(nes_file_write_atomic(path, legacy, sizeof(legacy) - 1) == NES_FILE_OK);
    CHECK(frontend_settings_load(path, &loaded, &report));
    CHECK(loaded.pixel_filter.kind == NES_PIXEL_FILTER_NONE);
    CHECK(loaded.pixel_filter.lcd_brightness[0] == 100 && loaded.pixel_filter.lcd_brightness[3] == 85);
    settings.pixel_filter.lcd_brightness[0] = 101;
    CHECK(!frontend_settings_save(path, &settings, &report));
    CHECK(frontend_settings_load(path, &loaded, &report) && loaded.pixel_filter.kind == NES_PIXEL_FILTER_NONE);
    frontend_settings_defaults(&settings);
    for (int kind = NES_PIXEL_FILTER_XBRZ2; kind < NES_PIXEL_FILTER_COUNT; ++kind) {
        settings.pixel_filter.kind = (NesPixelFilterKind)kind;
        CHECK(frontend_settings_save(path, &settings, &report));
        CHECK(frontend_settings_load(path, &loaded, &report));
        CHECK(loaded.pixel_filter.kind == settings.pixel_filter.kind);
    }

    CHECK(nes_file_remove(path) == NES_FILE_OK);
}

int test_pixel_filter_accuracy(void) {
    failures = 0;
    NesPixelFilter *filter = nes_pixel_filter_create();
    CHECK(filter != NULL);
    if (filter) {
        lcd_vectors(filter);
        scaler_vectors(filter);
        scaler_boundaries(filter);
    }
    nes_pixel_filter_destroy(filter);
    settings_files();
    printf("Pixel filters: reference vectors, LCD cells, dual boundaries, tiny crops, limits and persistence, %d "
           "failures\n",
           failures);
    return failures;
}
