/*
 * pixel_filter.cpp
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Presentation pixel filters. SPDX-License-Identifier: GPL-3.0-or-later */
#include "pixel_filter.h"
#include "pixel_scalers.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <new>
#include <vector>

struct NesPixelFilter {
    std::vector<uint32_t> pixels;
    std::vector<uint32_t> source;
    std::vector<uint32_t> scaled;
    std::vector<uint32_t> scratch;
};

namespace {
struct FilterInfo {
    const char *name;
    const char *key;
    unsigned factor;
};

constexpr FilterInfo filters[] = {{"None", "none", 1},
                                  {"LCD Grid", "lcd-grid", 2},
                                  {"xBRZ 2x", "xbrz-2x", 2},
                                  {"xBRZ 3x", "xbrz-3x", 3},
                                  {"xBRZ 4x", "xbrz-4x", 4},
                                  {"xBRZ 5x", "xbrz-5x", 5},
                                  {"xBRZ 6x", "xbrz-6x", 6},
                                  {"HQ2x", "hq2x", 2},
                                  {"HQ3x", "hq3x", 3},
                                  {"HQ4x", "hq4x", 4},
                                  {"Scale2x", "scale2x", 2},
                                  {"Scale3x", "scale3x", 3},
                                  {"Scale4x", "scale4x", 4},
                                  {"2xSaI", "2xsai", 2},
                                  {"Super2xSaI", "super2xsai", 2},
                                  {"SuperEagle", "supereagle", 2},
                                  {"Prescale 2x", "prescale-2x", 2},
                                  {"Prescale 3x", "prescale-3x", 3},
                                  {"Prescale 4x", "prescale-4x", 4},
                                  {"Prescale 6x", "prescale-6x", 6},
                                  {"Prescale 8x", "prescale-8x", 8},
                                  {"Prescale 10x", "prescale-10x", 10}};

static_assert(sizeof(filters) / sizeof(filters[0]) == NES_PIXEL_FILTER_COUNT);

bool fail(char *error, size_t size, const char *message) {
    if (error && size) {
        std::snprintf(error, size, "%s", message);
    }

    return false;
}

uint32_t brightness(uint32_t pixel, unsigned percent) {
    unsigned level = percent * 255u / 100u;
    unsigned red = ((pixel >> 16) & 255u) * level / 255u;
    unsigned green = ((pixel >> 8) & 255u) * level / 255u;
    unsigned blue = (pixel & 255u) * level / 255u;
    return 0xFF000000u | (red << 16) | (green << 8) | blue;
}
} // namespace

void nes_pixel_filter_defaults(NesPixelFilterSettings *settings) {
    if (settings) {
        *settings = {NES_PIXEL_FILTER_NONE, {100, 85, 85, 85}};
    }
}

bool nes_pixel_filter_validate(const NesPixelFilterSettings *settings, char *error, size_t error_size) {
    if (!settings || static_cast<unsigned>(settings->kind) >= NES_PIXEL_FILTER_COUNT) {
        return fail(error, error_size, "Pixel filter is invalid");
    }

    for (unsigned value : settings->lcd_brightness) {
        if (value > 100) {
            return fail(error, error_size, "LCD cell brightness must be between 0 and 100 percent");
        }
    }

    if (error && error_size) {
        error[0] = 0;
    }

    return true;
}

const char *nes_pixel_filter_name(NesPixelFilterKind kind) {
    return static_cast<unsigned>(kind) < NES_PIXEL_FILTER_COUNT ? filters[kind].name : "Invalid";
}

const char *nes_pixel_filter_key(NesPixelFilterKind kind) {
    return static_cast<unsigned>(kind) < NES_PIXEL_FILTER_COUNT ? filters[kind].key : "invalid";
}

unsigned nes_pixel_filter_scale(NesPixelFilterKind kind) {
    return static_cast<unsigned>(kind) < NES_PIXEL_FILTER_COUNT ? filters[kind].factor : 0;
}

bool nes_pixel_filter_from_key(const char *key, NesPixelFilterKind *kind) {
    if (!key || !kind) {
        return false;
    }

    for (unsigned i = 0; i < NES_PIXEL_FILTER_COUNT; ++i) {
        auto candidate = static_cast<NesPixelFilterKind>(i);
        if (!std::strcmp(key, nes_pixel_filter_key(candidate))) {
            *kind = candidate;
            return true;
        }
    }

    return false;
}

NesPixelFilter *nes_pixel_filter_create(void) {
    return new (std::nothrow) NesPixelFilter;
}

void nes_pixel_filter_destroy(NesPixelFilter *filter) {
    delete filter;
}

bool nes_pixel_filter_render(NesPixelFilter *filter, const NesPixelFilterSettings *settings,
                             const NesVideoPresentationFrame *input, NesVideoPresentationFrame *output, char *error,
                             size_t error_size) {
    if (!filter || !input || !output || !input->pixels || !input->width || !input->height || input->screens < 1 ||
        input->screens > 2 || input->width % input->screens) {
        return fail(error, error_size, "Pixel filter requires a complete presentation frame");
    }

    if (!nes_pixel_filter_validate(settings, error, error_size)) {
        return false;
    }

    if (settings->kind == NES_PIXEL_FILTER_NONE) {
        *output = *input;
        return true;
    }

    unsigned factor = nes_pixel_filter_scale(settings->kind);
    constexpr unsigned max_dimension = 8192;
    constexpr size_t max_pixels = 64u * 1024u * 1024u;
    if (input->width > max_dimension / factor || input->height > max_dimension / factor ||
        static_cast<size_t>(input->width) * input->height > max_pixels / (factor * factor)) {
        return fail(error, error_size, "Filtered output exceeds 8192 pixels per dimension or 256 MiB");
    }

    uintptr_t source_address = reinterpret_cast<uintptr_t>(input->pixels);
    uintptr_t output_address = reinterpret_cast<uintptr_t>(filter->pixels.data());
    if (!filter->pixels.empty() && source_address >= output_address &&
        source_address - output_address < filter->pixels.size() * sizeof(uint32_t)) {
        return fail(error, error_size, "Pixel filter input must be separate from its output storage");
    }

    unsigned width = input->width * factor, height = input->height * factor;
    try {
        filter->pixels.resize(static_cast<size_t>(width) * height);
        if (settings->kind != NES_PIXEL_FILTER_LCD) {
            unsigned side_width = input->width / input->screens;
            unsigned scaled_width = side_width * factor;
            filter->source.resize(static_cast<size_t>(side_width) * input->height);
            if (input->screens == 2) {
                filter->scaled.resize(static_cast<size_t>(scaled_width) * height);
            }

            for (unsigned side = 0; side < input->screens; ++side) {
                for (unsigned y = 0; y < input->height; ++y) {
                    std::copy_n(input->pixels + static_cast<size_t>(y) * input->width + side * side_width, side_width,
                                filter->source.data() + static_cast<size_t>(y) * side_width);
                }

                uint32_t *target = input->screens == 1 ? filter->pixels.data() : filter->scaled.data();
                nes_pixel_scaler_render(settings->kind, factor, filter->source.data(), side_width, input->height,
                                        target, filter->scratch);
                if (input->screens == 2) {
                    for (unsigned y = 0; y < height; ++y) {
                        std::copy_n(target + static_cast<size_t>(y) * scaled_width, scaled_width,
                                    filter->pixels.data() + static_cast<size_t>(y) * width + side * scaled_width);
                    }
                }
            }
        }
    } catch (const std::bad_alloc &) {
        return fail(error, error_size, "Could not allocate pixel filter output");
    }

    for (unsigned y = 0; settings->kind == NES_PIXEL_FILTER_LCD && y < input->height; ++y) {
        for (unsigned x = 0; x < input->width; ++x) {
            uint32_t pixel = input->pixels[static_cast<size_t>(y) * input->width + x];
            size_t offset = static_cast<size_t>(y) * factor * width + x * factor;
            filter->pixels[offset] = brightness(pixel, settings->lcd_brightness[0]);
            filter->pixels[offset + 1] = brightness(pixel, settings->lcd_brightness[1]);
            filter->pixels[offset + width] = brightness(pixel, settings->lcd_brightness[2]);
            filter->pixels[offset + width + 1] = brightness(pixel, settings->lcd_brightness[3]);
        }
    }

    *output = *input;
    output->pixels = filter->pixels.data();
    output->width = width;
    output->height = height;
    return true;
}
