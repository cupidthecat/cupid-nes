/*
 * ntsc_composite.c - Optional NTSC composite video reconstruction
 *
 * Copyright (C) 2014-2026 Sour and contributors
 * Copyright (C) 2026 Francis Hagan
 *
 * NTSC decoding algorithm by Bisqwit, with signal measurements by lidnariq.
 *
 * The decoder models the 12-phase color carrier from the PPU's palette index
 * and emphasis outputs. Signal levels use published terminated-voltage
 * measurements normalized so blank is zero and white is one hundred.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "ntsc_composite.h"
#include "../ppu/ppu.h"
#include "../../include/globals.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

enum { SIGNALS_PER_PIXEL = 8, SIGNAL_WIDTH = SCREEN_WIDTH * SIGNALS_PER_PIXEL, OUTPUT_DIVIDER = 4 };

static int8_t signal_low[0x80];
static int8_t signal_high[0x80];
static const int8_t sine_table[27] = {0, 3, 6, 8, 6, 3,  0,  -3, -6, -8, -6, -4, 0, 4,
                                      6, 8, 6, 3, 0, -3, -6, -8, -6, -4, 0,  4,  6};
static bool initialized;

typedef struct {
    NtscCompositeSettings picture;
    int y_gain, ir_gain, ig_gain, ib_gain, qr_gain, qg_gain, qb_gain;
    int8_t carrier[27];
    uint8_t gamma[256];
} Decoder;

static int clamp_byte(int64_t value) {
    if (value < 0) {
        return 0;
    }
    if (value > 255) {
        return 255;
    }
    return (int)value;
}

static void init_filter(void) {
    if (initialized) {
        return;
    }

    static const double luma_low[2][4] = {{0.228, 0.312, 0.552, 0.880}, {0.192, 0.256, 0.448, 0.712}};
    static const double luma_high[2][4] = {{0.616, 0.840, 1.100, 1.100}, {0.500, 0.676, 0.896, 0.896}};
    const double blank = luma_low[0][1];
    const double white = luma_high[0][3];

    for (unsigned attenuated = 0; attenuated < 2; ++attenuated) {
        for (unsigned color = 0; color < 64; ++color) {
            double low = luma_low[attenuated][color >> 4];
            double high = luma_high[attenuated][color >> 4];
            unsigned hue = color & 0x0F;
            if (hue == 0x0D) {
                high = low;
            } else if (hue == 0) {
                low = high;
            } else if (hue >= 0x0E) {
                low = blank;
                high = blank;
            }
            unsigned index = color | (attenuated ? 0x40u : 0u);
            signal_low[index] = (int8_t)floor(((low - blank) / (white - blank)) * 100.0);
            signal_high[index] = (int8_t)floor(((high - blank) / (white - blank)) * 100.0);
        }
    }

    initialized = true;
}

static void init_decoder(Decoder *decoder, const NtscCompositeSettings *picture) {
    decoder->picture = *picture;
    double contrast_level = picture->contrast / 100.0;
    double saturation_level = picture->saturation / 100.0;
    int contrast = (int)(contrast_level * contrast_level * 167941);
    int saturation = (int)(saturation_level * saturation_level * 144044);
    decoder->y_gain = contrast / picture->y_width;
    decoder->ir_gain = (int)(contrast * 1.994681e-6 * saturation / picture->i_width);
    decoder->qr_gain = (int)(contrast * 9.915742e-7 * saturation / picture->q_width);
    decoder->ig_gain = (int)(contrast * 9.151351e-8 * saturation / picture->i_width);
    decoder->qg_gain = (int)(contrast * -6.334805e-7 * saturation / picture->q_width);
    decoder->ib_gain = (int)(contrast * -1.012984e-6 * saturation / picture->i_width);
    decoder->qb_gain = (int)(contrast * 1.667217e-6 * saturation / picture->q_width);
    if (!picture->hue) {
        /* Retain the existing integer carrier at the neutral setting. */
        memcpy(decoder->carrier, sine_table, sizeof(sine_table));
    } else {
        const double pi = 3.14159265358979323846;
        for (unsigned i = 0; i < 27; ++i) {
            decoder->carrier[i] = (int8_t)(8 * sin(i * 2 * pi / 12 + picture->hue * pi / 180));
        }
    }
    for (unsigned i = 0; i < 256; ++i) {
        decoder->gamma[i] = picture->gamma == 100
                                ? (uint8_t)i
                                : (uint8_t)clamp_byte((int)(255 * pow(i / 255.0, 100.0 / picture->gamma) + 0.5));
    }
}

bool ntsc_composite_supported(NesRegion region, bool vs_hardware) {
    return region == NES_REGION_NTSC && !vs_hardware;
}

static void generate_signal(const uint16_t *pixels, int8_t *signal, unsigned *phase) {
    static const uint16_t emphasis_lut[8] = {0x000, 0x03F, 0x3F0, 0x3FF, 0xF03, 0xF3F, 0xFF3, 0xFFF};

    for (unsigned x = 0; x < SCREEN_WIDTH; ++x) {
        uint16_t ppu_data = pixels[x];
        uint8_t pixel_color = (uint8_t)(ppu_data & 0x3F);
        uint8_t emphasis = (uint8_t)((ppu_data >> 6) & 7);
        uint8_t hue = (uint8_t)(pixel_color & 0x0F);
        uint16_t emphasis_wave = 0;
        if (emphasis) {
            unsigned shift = hue % 12u;
            emphasis_wave = emphasis_lut[emphasis];
            emphasis_wave = (uint16_t)(((uint32_t)emphasis_wave >> shift) | ((uint32_t)emphasis_wave << (12u - shift)));
        }

        int difference = (int)*phase - (int)hue;
        if (difference < 0) {
            difference = -difference;
        }
        unsigned phase_difference = (unsigned)difference % 12u;
        uint16_t phase_mask = (uint16_t)(1u << phase_difference);
        for (unsigned sample = 0; sample < SIGNALS_PER_PIXEL; ++sample) {
            phase_mask <<= 1;
            uint8_t color = (uint8_t)(pixel_color | ((phase_mask & emphasis_wave) ? 0x40u : 0u));
            int8_t voltage = signal_high[color];
            if (phase_mask >= (1u << 12)) {
                phase_mask = 1;
            } else if (phase_mask >= (1u << 6)) {
                voltage = signal_low[color];
            }
            signal[x * SIGNALS_PER_PIXEL + sample] = voltage;
        }
        *phase += SIGNALS_PER_PIXEL;
    }
    *phase += (341u - SCREEN_WIDTH) * SIGNALS_PER_PIXEL;
}

static int read_signal(const int8_t *signal, int position) {
    return position >= 0 && position < SIGNAL_WIDTH ? signal[position] : 0;
}

static int carrier_cos(const Decoder *decoder, int position, int phase0) {
    return decoder->carrier[((position % 12 + 12) % 12) + phase0];
}

static int carrier_sin(const Decoder *decoder, int position, int phase0) {
    return decoder->carrier[((position % 12 + 12) % 12) + 3 + phase0];
}

static void decode_line(const Decoder *decoder, const int8_t *signal, uint32_t *target, int phase0) {
    const NtscCompositeSettings *picture = &decoder->picture;
    int y_sum = picture->brightness * 750 / 100, i_sum = 0, q_sum = 0;
    int max_filter = picture->y_width;
    if (picture->i_width > max_filter) {
        max_filter = picture->i_width;
    }
    if (picture->q_width > max_filter) {
        max_filter = picture->q_width;
    }
    max_filter /= 2;

    for (int position = -max_filter; position < SIGNAL_WIDTH; ++position) {
        int y_position = position + picture->y_width / 2;
        int i_position = position + picture->i_width / 2;
        int q_position = position + picture->q_width / 2;
        y_sum += read_signal(signal, y_position) - read_signal(signal, y_position - picture->y_width);
        i_sum += read_signal(signal, i_position) * carrier_cos(decoder, i_position, phase0) -
                 read_signal(signal, i_position - picture->i_width) *
                     carrier_cos(decoder, i_position - picture->i_width, phase0);
        q_sum += read_signal(signal, q_position) * carrier_sin(decoder, q_position, phase0) -
                 read_signal(signal, q_position - picture->q_width) *
                     carrier_sin(decoder, q_position - picture->q_width, phase0);

        if (position >= 0 && position % OUTPUT_DIVIDER == 0) {
            int red = clamp_byte(((int64_t)y_sum * decoder->y_gain + (int64_t)i_sum * decoder->ir_gain +
                                  (int64_t)q_sum * decoder->qr_gain) /
                                 65536);
            int green = clamp_byte(((int64_t)y_sum * decoder->y_gain + (int64_t)i_sum * decoder->ig_gain +
                                    (int64_t)q_sum * decoder->qg_gain) /
                                   65536);
            int blue = clamp_byte(((int64_t)y_sum * decoder->y_gain + (int64_t)i_sum * decoder->ib_gain +
                                   (int64_t)q_sum * decoder->qb_gain) /
                                  65536);
            *target++ = 0xFF000000u | ((uint32_t)red << 16) | ((uint32_t)green << 8) | (uint32_t)blue;
        }
    }
}

void ntsc_composite_filter_frame(const uint16_t *ppu_signal, uint8_t video_phase, uint32_t *output) {
    (void)ntsc_composite_filter_frame_settings(ppu_signal, video_phase, output, NULL, NULL, 0);
}

bool ntsc_composite_filter_frame_settings(const uint16_t *ppu_signal, uint8_t video_phase, uint32_t *output,
                                          const NtscCompositeSettings *settings, char *error, size_t error_size) {
    if (!ppu_signal || !output) {
        if (error && error_size) {
            snprintf(error, error_size, "A source signal and output buffer are required");
        }
        return false;
    }
    NtscCompositeSettings defaults;
    ntsc_composite_defaults(&defaults);
    if (!settings) {
        settings = &defaults;
    }
    if (!ntsc_composite_validate(settings, error, error_size)) {
        return false;
    }
    init_filter();
    Decoder decoder;
    init_decoder(&decoder, settings);

    int8_t row_signal[SIGNAL_WIDTH];
    unsigned phase[3] = {((unsigned)video_phase % 3u) * 4u, 4, 8};
    if (settings->fields != NTSC_FIELDS_SOURCE) {
        phase[0] = 0;
    }
    unsigned passes = settings->fields == NTSC_FIELDS_BLEND ? 3u : 1u;
    uint32_t lines[3][NTSC_COMPOSITE_WIDTH];
    for (unsigned y = 0; y < SCREEN_HEIGHT; ++y) {
        uint32_t *first = output + (y * 2u) * NTSC_COMPOSITE_WIDTH;
        for (unsigned pass = 0; pass < passes; ++pass) {
            unsigned start_phase = phase[pass] % 12u;
            generate_signal(ppu_signal + y * SCREEN_WIDTH, row_signal, &phase[pass]);
            decode_line(&decoder, row_signal, lines[pass], (int)((start_phase + 7u) % 12u));
        }
        for (unsigned x = 0; x < NTSC_COMPOSITE_WIDTH; ++x) {
            uint32_t full = 0xFF000000u, dim = 0xFF000000u;
            for (unsigned shift = 0; shift <= 16; shift += 8) {
                unsigned level = 0;
                for (unsigned pass = 0; pass < passes; ++pass) {
                    level += (lines[pass][x] >> shift) & 255u;
                }
                level = decoder.gamma[level / passes];
                full |= level << shift;
                dim |= (level * (unsigned)(100 - settings->scanlines) / 100) << shift;
            }
            first[x] = full;
            first[NTSC_COMPOSITE_WIDTH + x] = dim;
        }
    }
    if (error && error_size) {
        error[0] = 0;
    }
    return true;
}
