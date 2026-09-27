/*
 * pixel_filter.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Presentation pixel filters. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_PIXEL_FILTER_H
#define CUPID_PIXEL_FILTER_H

#include "presentation.h"

#ifdef __cplusplus
extern "C" {
#endif

/* C callers can provide arbitrary IDs. Keep every bit pattern valid at the C++
 * boundary so validation can reject it before indexing the filter table. */
typedef uint32_t NesPixelFilterKind;

enum {
    NES_PIXEL_FILTER_NONE,
    NES_PIXEL_FILTER_LCD,
    NES_PIXEL_FILTER_XBRZ2,
    NES_PIXEL_FILTER_XBRZ3,
    NES_PIXEL_FILTER_XBRZ4,
    NES_PIXEL_FILTER_XBRZ5,
    NES_PIXEL_FILTER_XBRZ6,
    NES_PIXEL_FILTER_HQ2X,
    NES_PIXEL_FILTER_HQ3X,
    NES_PIXEL_FILTER_HQ4X,
    NES_PIXEL_FILTER_SCALE2X,
    NES_PIXEL_FILTER_SCALE3X,
    NES_PIXEL_FILTER_SCALE4X,
    NES_PIXEL_FILTER_2XSAI,
    NES_PIXEL_FILTER_SUPER2XSAI,
    NES_PIXEL_FILTER_SUPEREAGLE,
    NES_PIXEL_FILTER_PRESCALE2,
    NES_PIXEL_FILTER_PRESCALE3,
    NES_PIXEL_FILTER_PRESCALE4,
    NES_PIXEL_FILTER_PRESCALE6,
    NES_PIXEL_FILTER_PRESCALE8,
    NES_PIXEL_FILTER_PRESCALE10,
    NES_PIXEL_FILTER_COUNT
};

typedef struct {
    NesPixelFilterKind kind;
    unsigned lcd_brightness[4]; /* Percent: top left, top right, bottom left, bottom right. */
} NesPixelFilterSettings;

typedef struct NesPixelFilter NesPixelFilter;

void nes_pixel_filter_defaults(NesPixelFilterSettings *settings);
bool nes_pixel_filter_validate(const NesPixelFilterSettings *settings, char *error, size_t error_size);
const char *nes_pixel_filter_name(NesPixelFilterKind kind);
const char *nes_pixel_filter_key(NesPixelFilterKind kind);
unsigned nes_pixel_filter_scale(NesPixelFilterKind kind);
bool nes_pixel_filter_from_key(const char *key, NesPixelFilterKind *kind);
NesPixelFilter *nes_pixel_filter_create(void);
void nes_pixel_filter_destroy(NesPixelFilter *filter);

/* Input is a tightly packed presentation frame. Output belongs to the filter until
 * its next render or destruction. Input may not alias a previous filtered output.
 * Geometry metadata follows the source; hardware pixels are never modified. */
bool nes_pixel_filter_render(NesPixelFilter *filter, const NesPixelFilterSettings *settings,
                             const NesVideoPresentationFrame *input, NesVideoPresentationFrame *output, char *error,
                             size_t error_size);

#ifdef __cplusplus
}
#endif
#endif
