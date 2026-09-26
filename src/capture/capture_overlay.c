/* capture_overlay.c - CPU overlay compositor using the bundled display font
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "capture_overlay.h"
#include "../third_party/fonts/font_data.h"
#include "../third_party/miniz/miniz.h"
#include "../third_party/stb/stb_truetype.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    unsigned code;
    unsigned char mask[24 * 24];
    int width, height, x, y, advance;
} Glyph;

typedef struct {
    unsigned char *data;
    stbtt_fontinfo font;
    Glyph glyphs[256];
    unsigned count;
    float factor;
} OverlayFont;

static OverlayFont *create_font(void) {
    OverlayFont *font = calloc(1, sizeof(*font));
    if (!font) {
        return NULL;
    }
    font->data = malloc(CUPID_FONT_BYTES);
    mz_ulong size = CUPID_FONT_BYTES;
    if (!font->data || mz_uncompress(font->data, &size, cupid_font_data, sizeof(cupid_font_data)) != MZ_OK ||
        size != CUPID_FONT_BYTES || !stbtt_InitFont(&font->font, font->data, 0)) {
        free(font->data);
        free(font);
        return NULL;
    }
    font->factor = stbtt_ScaleForPixelHeight(&font->font, 12);
    return font;
}

static Glyph *glyph(OverlayFont *font, unsigned codepoint) {
    if (!stbtt_FindGlyphIndex(&font->font, (int)codepoint)) {
        codepoint = '?';
    }
    for (unsigned i = 0; i < font->count; ++i) {
        if (font->glyphs[i].code == codepoint) {
            return &font->glyphs[i];
        }
    }
    unsigned slot = font->count < 256 ? font->count++ : codepoint % 256u;
    Glyph *g = &font->glyphs[slot];
    memset(g, 0, sizeof(*g));
    g->code = codepoint;
    int x1, y1, advance, bearing;
    stbtt_GetCodepointBitmapBox(&font->font, (int)codepoint, font->factor, font->factor, &g->x, &g->y, &x1, &y1);
    g->width = x1 - g->x;
    g->height = y1 - g->y;
    if (g->width > 24) {
        g->width = 24;
    }
    if (g->height > 24) {
        g->height = 24;
    }
    stbtt_GetCodepointHMetrics(&font->font, (int)codepoint, &advance, &bearing);
    g->advance = (int)(advance * font->factor + 0.5f);
    stbtt_MakeCodepointBitmap(&font->font, g->mask, g->width, g->height, 24, font->factor, font->factor,
                              (int)codepoint);
    return g;
}

static unsigned next_codepoint(const char **text) {
    const unsigned char *p = (const unsigned char *)*text;
    unsigned c = *p++;
    unsigned n = c >= 0xF0 && c <= 0xF4 ? 3 : c >= 0xE0 && c <= 0xEF ? 2 : c >= 0xC2 && c <= 0xDF ? 1 : 0;
    if (c >= 128 && !n) {
        *text = (const char *)p;
        return '?';
    }
    if (n) {
        c &= (1u << (6u - n)) - 1u;
        while (n--) {
            if ((*p & 0xC0) != 0x80) {
                *text = (const char *)p;
                return '?';
            }
            c = c * 64 + (*p++ & 63);
        }
    }
    *text = (const char *)p;
    return c;
}

static void pixel(uint32_t *pixels, unsigned width, unsigned height, int x, int y, uint32_t color) {
    if (x >= 0 && y >= 0 && (unsigned)x < width && (unsigned)y < height) {
        pixels[(size_t)y * width + (unsigned)x] = color;
    }
}

static void text(NesCaptureOverlay *o, unsigned width, unsigned height, int x, int y, const char *string,
                 unsigned scale, uint32_t color, bool wrap) {
    OverlayFont *font = o->implementation;
    int start = x;
    unsigned count = 0;
    while (*string && count++ < NES_OVERLAY_TEXT) {
        Glyph *g = glyph(font, next_codepoint(&string));
        if (wrap && x + (g->advance + 2) * (int)scale >= (int)width) {
            x = start;
            y += 13 * (int)scale;
        }
        for (int row = 0; row < g->height; ++row) {
            for (int col = 0; col < g->width; ++col) {
                if (g->mask[row * 24 + col] < 96) {
                    continue;
                }
                int dx = x + (g->x + col) * (int)scale, dy = y + (10 + g->y + row) * (int)scale;
                for (int oy = -1; oy <= (int)scale; ++oy) {
                    for (int ox = -1; ox <= (int)scale; ++ox) {
                        pixel(o->pixels, width, height, dx + ox, dy + oy, 0xFF000000);
                    }
                }
            }
        }
        for (int row = 0; row < g->height; ++row) {
            for (int col = 0; col < g->width; ++col) {
                if (g->mask[row * 24 + col] >= 96) {
                    for (unsigned sy = 0; sy < scale; ++sy) {
                        for (unsigned sx = 0; sx < scale; ++sx) {
                            pixel(o->pixels, width, height, x + (g->x + col) * (int)scale + (int)sx,
                                  y + (10 + g->y + row) * (int)scale + (int)sy, color);
                        }
                    }
                }
            }
        }
        x += g->advance * (int)scale;
    }
}

NesFileResult nes_capture_overlay_compose(NesCaptureOverlay *o, const NesCaptureFrame *source,
                                          const NesCaptureOverlayState *state, unsigned mask,
                                          NesOverlayPosition position, NesCaptureFrame *output) {
    if (!o || !source || !output || !source->pixels || !source->width || !source->height ||
        source->width > NES_CAPTURE_MAX_DIMENSION || source->height > NES_CAPTURE_MAX_DIMENSION ||
        source->stride < source->width || source->stride > SIZE_MAX / source->height / 4u ||
        (mask & ~NES_OVERLAY_ALL) || (unsigned)position > NES_OVERLAY_BOTTOM_RIGHT) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    if (!mask) {
        *output = *source;
        return NES_FILE_OK;
    }
    if (!state || !memchr(state->subtitle, 0, sizeof(state->subtitle)) ||
        !memchr(state->status, 0, sizeof(state->status))) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    if (!o->implementation) {
        o->implementation = create_font();
    }
    if (!o->implementation) {
        return NES_FILE_OUT_OF_MEMORY;
    }
    size_t count = (size_t)source->width * source->height;
    if (count > o->capacity) {
        uint32_t *pixels = realloc(o->pixels, count * 4u);
        if (!pixels) {
            return NES_FILE_OUT_OF_MEMORY;
        }
        o->pixels = pixels;
        o->capacity = count;
    }
    for (unsigned y = 0; y < source->height; ++y) {
        memcpy(o->pixels + (size_t)y * source->width, source->pixels + (size_t)y * source->stride, source->width * 4u);
    }
    unsigned scale = source->height / 240u;
    if (!scale) {
        scale = 1;
    }
    if (scale > 4) {
        scale = 4;
    }
    unsigned rows = 0;
    if (mask & NES_OVERLAY_INPUT) {
        for (unsigned i = 0; i < 6; ++i) {
            if (state->active_players & (1u << i)) {
                ++rows;
            }
        }
    }
    if (mask & NES_OVERLAY_FRAME) {
        ++rows;
    }
    if ((mask & NES_OVERLAY_STATUS) && state->status[0]) {
        ++rows;
    }
    int x = 4 * (int)scale, y = 4 * (int)scale;
    if (position & 1) {
        x = (int)source->width - 224 * (int)scale;
    }
    if (x < 0) {
        x = 0;
    }
    unsigned subtitle_rows = (mask & NES_OVERLAY_SUBTITLE) && state->movie_active && state->subtitle[0] ? 43u : 0u;
    if (position & 2) {
        y = (int)source->height - (int)(rows * 13u + 4u + subtitle_rows) * (int)scale;
    }
    if (y < 0) {
        y = 0;
    }
    if (mask & NES_OVERLAY_INPUT) {
        static const char *const buttons[] = {"A", "B", "Sel", "Start", "Up", "Dn", "Left", "Right"};
        for (unsigned i = 0; i < 6; ++i) {
            if (!(state->active_players & (1u << i))) {
                continue;
            }
            char label[8];
            snprintf(label, sizeof(label), "P%u", i + 1);
            text(o, source->width, source->height, x, y, label, scale, 0xFFFFFFFF, false);
            for (unsigned b = 0; b < 8; ++b) {
                text(o, source->width, source->height, x + (20 + (int)b * 25) * (int)scale, y, buttons[b], scale,
                     state->pads[i] & (1u << b) ? 0xFFFFFF40 : 0xFF909090, false);
            }
            y += 13 * (int)scale;
        }
    }
    if (mask & NES_OVERLAY_FRAME) {
        char line[64];
        snprintf(line, sizeof(line), "%sframe %" PRIu64, state->movie_active ? "Movie " : "", state->frame);
        text(o, source->width, source->height, x, y, line, scale, 0xFFFFFFFF, false);
        y += 13 * (int)scale;
    }
    if (mask & NES_OVERLAY_STATUS) {
        text(o, source->width, source->height, x, y, state->status, scale, 0xFFFFFFFF, false);
    }
    if ((mask & NES_OVERLAY_SUBTITLE) && state->movie_active) {
        text(o, source->width, source->height, 4 * (int)scale, (int)source->height - 43 * (int)scale, state->subtitle,
             scale, 0xFFFFFFFF, true);
    }
    *output = (NesCaptureFrame){o->pixels, source->width, source->height, source->width};
    return NES_FILE_OK;
}

void nes_capture_overlay_destroy(NesCaptureOverlay *o) {
    if (!o) {
        return;
    }
    OverlayFont *font = o->implementation;
    if (font) {
        free(font->data);
        free(font);
    }
    free(o->pixels);
    memset(o, 0, sizeof(*o));
}
