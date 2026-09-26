/*
 * font_atlas.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Cached TrueType glyph atlas. SPDX-License-Identifier: GPL-3.0-or-later */
#include "font_atlas.h"
#include "../third_party/fonts/font_data.h"
#include "../third_party/miniz/miniz.h"
#include "../third_party/stb/stb_truetype.h"
#include <stdlib.h>
#include <string.h>
#define ATLAS_SIZE 1024
#define GLYPH_LIMIT 1024
#define BAKE_SIZE 32.0f

typedef struct {
    int code, x, y, w, h, ox, oy;
    float advance;
} Glyph;

struct DesktopFont {
    stbtt_fontinfo info;
    unsigned char *data;
    SDL_Texture *texture;
    Glyph glyph[GLYPH_LIMIT];
    int count, x, y, row_height;
    float factor, ascent;
};

static int next_codepoint(const char *text, int length, int *offset) {
    unsigned char a = (unsigned char)text[(*offset)++];
    if (a < 128) {
        return a;
    }
    int count = a >= 0xF0 && a <= 0xF4 ? 3 : a >= 0xE0 && a < 0xF0 ? 2 : a >= 0xC2 && a < 0xE0 ? 1 : 0;
    if (!count || *offset + count > length) {
        return '?';
    }
    unsigned code = a & ((1u << (6 - count)) - 1u);
    for (int i = 0; i < count; ++i) {
        unsigned char b = (unsigned char)text[*offset];
        if ((b & 0xC0) != 0x80) {
            return '?';
        }
        ++*offset;
        code = (code << 6) | (b & 63);
    }
    unsigned minimum = count == 1 ? 0x80 : count == 2 ? 0x800 : 0x10000;
    return code >= minimum && code <= 0x10FFFF && !(code >= 0xD800 && code <= 0xDFFF) ? (int)code : '?';
}

static Glyph *glyph_for(DesktopFont *font, int code) {
    if (!stbtt_FindGlyphIndex(&font->info, code)) {
        code = '?';
    }
    for (int i = 0; i < font->count; ++i) {
        if (font->glyph[i].code == code) {
            return &font->glyph[i];
        }
    }
    if (font->count == GLYPH_LIMIT) {
        return &font->glyph[0];
    }
    int advance, bearing, x0, y0, x1, y1;
    stbtt_GetCodepointHMetrics(&font->info, code, &advance, &bearing);
    stbtt_GetCodepointBitmapBox(&font->info, code, font->factor, font->factor, &x0, &y0, &x1, &y1);
    int width = x1 - x0, height = y1 - y0;
    if (font->x + width + 2 > ATLAS_SIZE) {
        font->x = 1;
        font->y += font->row_height + 2;
        font->row_height = 0;
    }
    if (font->y + height + 2 > ATLAS_SIZE) {
        return &font->glyph[0];
    }
    Glyph *glyph = &font->glyph[font->count++];
    *glyph = (Glyph){code, font->x, font->y, width, height, x0, y0, advance * font->factor};
    if (width && height) {
        unsigned char *mask = calloc((size_t)width * height, 1);
        uint32_t *pixels = malloc((size_t)width * height * 4);
        if (!mask || !pixels) {
            free(mask);
            free(pixels);
            --font->count;
            return &font->glyph[0];
        }
        stbtt_MakeCodepointBitmap(&font->info, mask, width, height, width, font->factor, font->factor, code);
        for (int i = 0; i < width * height; ++i) {
            pixels[i] = ((uint32_t)mask[i] << 24) | 0xFFFFFFu;
        }
        SDL_Rect rect = {glyph->x, glyph->y, width, height};
        SDL_UpdateTexture(font->texture, &rect, pixels, width * 4);
        free(mask);
        free(pixels);
    }
    font->x += width + 2;
    if (height > font->row_height) {
        font->row_height = height;
    }
    return glyph;
}

DesktopFont *desktop_font_create(SDL_Renderer *renderer) {
    DesktopFont *font = calloc(1, sizeof(*font));
    if (!font) {
        return NULL;
    }
    font->data = malloc(CUPID_FONT_BYTES);
    mz_ulong size = CUPID_FONT_BYTES;
    if (!font->data || mz_uncompress(font->data, &size, cupid_font_data, sizeof(cupid_font_data)) != MZ_OK ||
        !stbtt_InitFont(&font->info, font->data, 0)) {
        desktop_font_destroy(font);
        return NULL;
    }
    font->texture =
        SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STATIC, ATLAS_SIZE, ATLAS_SIZE);
    if (!font->texture) {
        desktop_font_destroy(font);
        return NULL;
    }
    uint32_t *clear = calloc(ATLAS_SIZE * ATLAS_SIZE, 4);
    if (!clear) {
        desktop_font_destroy(font);
        return NULL;
    }
    SDL_UpdateTexture(font->texture, NULL, clear, ATLAS_SIZE * 4);
    free(clear);
    SDL_SetTextureBlendMode(font->texture, SDL_BLENDMODE_BLEND);
    SDL_SetTextureScaleMode(font->texture, SDL_ScaleModeLinear);
    int ascent;
    stbtt_GetFontVMetrics(&font->info, &ascent, NULL, NULL);
    font->factor = stbtt_ScaleForMappingEmToPixels(&font->info, BAKE_SIZE);
    font->ascent = ascent * font->factor;
    font->x = font->y = 1;
    (void)glyph_for(font, '?');
    for (int code = 32; code < 127; ++code) {
        (void)glyph_for(font, code);
    }
    return font;
}

void desktop_font_destroy(DesktopFont *font) {
    if (font) {
        SDL_DestroyTexture(font->texture);
        free(font->data);
        free(font);
    }
}

float desktop_font_measure(DesktopFont *font, const char *text, int length, float size) {
    if (!font || !text) {
        return 0;
    }
    float width = 0;
    int offset = 0, previous = 0;
    while (offset < length) {
        int code = next_codepoint(text, length, &offset);
        if (!stbtt_FindGlyphIndex(&font->info, code)) {
            code = '?';
        }
        int advance;
        stbtt_GetCodepointHMetrics(&font->info, code, &advance, NULL);
        width += (advance + stbtt_GetCodepointKernAdvance(&font->info, previous, code)) * font->factor;
        previous = code;
    }
    return width * size / BAKE_SIZE;
}

int desktop_font_fit(DesktopFont *font, const char *text, int length, float size, float width) {
    if (!font || !text || length <= 0 || size <= 0 || width <= 0) {
        return 0;
    }
    int offset = 0, fit = 0;
    while (offset < length) {
        (void)next_codepoint(text, length, &offset);
        if (desktop_font_measure(font, text, offset, size) > width) {
            break;
        }
        fit = offset;
    }
    return fit;
}

void desktop_font_draw(DesktopFont *font, SDL_Renderer *renderer, const char *text, int length, float x, float y,
                       float size, SDL_Color color) {
    if (!font || !text) {
        return;
    }
    SDL_Vertex vertices[6 * 512];
    int count = 0, offset = 0, previous = 0;
    float scale = size / BAKE_SIZE;
    /* Resolve atlas additions before queuing geometry that uses the texture. */
    while (offset < length) {
        int code = next_codepoint(text, length, &offset);
        (void)glyph_for(font, code);
    }
    offset = 0;
    while (offset < length && count < 6 * 512) {
        int code = next_codepoint(text, length, &offset);
        Glyph *glyph = glyph_for(font, code);
        code = glyph->code;
        x += stbtt_GetCodepointKernAdvance(&font->info, previous, code) * font->factor * scale;
        previous = code;
        float left = x + glyph->ox * scale, top = y + (font->ascent + glyph->oy) * scale,
              right = left + glyph->w * scale, bottom = top + glyph->h * scale;
        float u = (float)glyph->x / ATLAS_SIZE, v = (float)glyph->y / ATLAS_SIZE,
              u1 = (float)(glyph->x + glyph->w) / ATLAS_SIZE, v1 = (float)(glyph->y + glyph->h) / ATLAS_SIZE;
        SDL_Vertex quad[6] = {{{left, top}, color, {u, v}},       {{right, top}, color, {u1, v}},
                              {{right, bottom}, color, {u1, v1}}, {{left, top}, color, {u, v}},
                              {{right, bottom}, color, {u1, v1}}, {{left, bottom}, color, {u, v1}}};
        if (glyph->w && glyph->h) {
            memcpy(vertices + count, quad, sizeof(quad));
            count += 6;
        }
        x += glyph->advance * scale;
    }
    if (count) {
        SDL_RenderGeometry(renderer, font->texture, vertices, count, NULL, 0);
    }
}
