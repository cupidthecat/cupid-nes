/*
 * palette_tool.c - Runtime palette editor and color picker
 *
 * Author: @frankischilling
 *
 * This file stores runtime palette colors and reads .pal files, hexadecimal palette data, and
 * optional color emphasis tables used by the PPU renderer.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "palette_tool.h"
#include "../ppu/ppu.h"
#include <SDL2/SDL.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Runtime palette storage includes the base colors and eight emphasis variants.
// Colors are stored as ARGB 0xFFRRGGBB for direct use by the renderer.
uint32_t ppu__active_palette_base[64];
uint32_t ppu__emphasis_palettes[8][64];
bool     ppu__have_emphasis_tables = false;

static bool show_overlay = false;
void palette_tool_init(void) { show_overlay = false; }
void palette_tool_toggle_overlay(void) { show_overlay = !show_overlay; }
void palette_tool_hide_overlay(void) { show_overlay = false; }
bool palette_tool_is_visible(void) { return show_overlay; }

// Palette loading and editing.

static inline uint32_t rgb_bytes_to_argb(uint8_t r, uint8_t g, uint8_t b) {
    return 0xFF000000u | ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

void ppu_palette_reset_default(void) {
    for (int i = 0; i < 64; ++i) ppu__active_palette_base[i] = nes_palette[i];
    ppu__have_emphasis_tables = false;
}

void ppu_palette_get(uint32_t out[64]) {
    if (!out) return;
    for (int i = 0; i < 64; ++i) out[i] = ppu__active_palette_base[i];
}

bool ppu_palette_has_emphasis_tables(void) {
    return ppu__have_emphasis_tables;
}

int ppu_palette_set_color(int index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < 0 || index >= 64) return -1;
    uint32_t argb = rgb_bytes_to_argb(r,g,b);
    ppu__active_palette_base[index] = argb;
    if (ppu__have_emphasis_tables) {
        ppu__emphasis_palettes[0][index] = argb;
    }
    return 0;
}

// Read a palette file into a caller-owned heap buffer.
static int read_entire_file(const char *path, uint8_t **out_data, size_t *out_size) {
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return -2; }
    long sz = ftell(f);
    if (sz < 0) { fclose(f); return -3; }
    if (fseek(f, 0, SEEK_SET) != 0) { fclose(f); return -4; }
    uint8_t *buf = (uint8_t*)malloc((size_t)sz);
    if (!buf) { fclose(f); return -5; }
    size_t rd = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    if (rd != (size_t)sz) { free(buf); return -6; }
    *out_data = buf; *out_size = (size_t)sz; return 0;
}

int ppu_palette_load_pal_file(const char *path) {
    if (!path) return -1;
    uint8_t *data = NULL; size_t size = 0; int rc = read_entire_file(path, &data, &size);
    if (rc != 0) return rc;

    if (size == 192) {
        for (int i = 0; i < 64; ++i) {
            uint8_t r = data[i*3 + 0];
            uint8_t g = data[i*3 + 1];
            uint8_t b = data[i*3 + 2];
            ppu__active_palette_base[i] = rgb_bytes_to_argb(r,g,b);
        }
        ppu__have_emphasis_tables = false;
        free(data);
        return 0;
    } else if (size == 1536) {
        // A 1536-byte file contains eight 64-color emphasis palettes.
        for (int e = 0; e < 8; ++e) {
            for (int i = 0; i < 64; ++i) {
                size_t off = (size_t)e*64*3 + (size_t)i*3;
                uint8_t r = data[off + 0];
                uint8_t g = data[off + 1];
                uint8_t b = data[off + 2];
                uint32_t argb = rgb_bytes_to_argb(r,g,b);
                ppu__emphasis_palettes[e][i] = argb;
                if (e == 0) ppu__active_palette_base[i] = argb;
            }
        }
        ppu__have_emphasis_tables = true;
        free(data);
        return 0;
    } else {
        free(data);
        // The file size does not match either supported palette layout.
        return -7;
    }
}

static int hex_digit(int c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

// Parse 64 RRGGBB tokens; each token may start with #, 0x, or $.
static int try_parse_hex_tokens(const char *s) {
    uint32_t tmp[64];
    int count = 0;
    const char *p = s;
    while (*p && count < 64) {
        while (*p && (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t' || *p == ',' || *p == ';')) p++;
        if (!*p) break;
        // Accept the common prefixes used by palette dumps.
        if (*p == '#') p++;
        if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) p += 2;
        if (*p == '$') p++;
        // Each token must contain exactly six hex digits.
        int h[6];
        for (int i = 0; i < 6; ++i) {
            int d = hex_digit(p[i]);
            if (d < 0) return -1;
            h[i] = d;
        }
        p += 6;
        uint8_t r = (uint8_t)((h[0] << 4) | h[1]);
        uint8_t g = (uint8_t)((h[2] << 4) | h[3]);
        uint8_t b = (uint8_t)((h[4] << 4) | h[5]);
        tmp[count++] = rgb_bytes_to_argb(r,g,b);
        while (*p && ((*p >= '0' && *p <= '9') || (*p >= 'A' && *p <= 'F') || (*p >= 'a' && *p <= 'f'))) {
            // Extra hex digits without a separator mean this is raw byte data.
            return -1;
        }
    }
    if (count == 64) {
        for (int i = 0; i < 64; ++i) ppu__active_palette_base[i] = tmp[i];
        ppu__have_emphasis_tables = false;
        return 0;
    }
    return -1;
}

// Raw byte input ignores non-hex separators and groups the remaining digits into pairs.
static int try_parse_raw_hex_bytes(const char *s) {
    size_t cap = 2048;
    char *digits = (char*)malloc(cap);
    if (!digits) return -1;
    size_t n = 0;
    for (const char *p = s; *p; ++p) {
        if (hex_digit(*p) >= 0) {
            if (n >= cap) { free(digits); return -1; }
            digits[n++] = *p;
        }
    }
    if ((n & 1) != 0) { free(digits); return -1; }
    size_t bytes = n / 2;
    if (!(bytes == 192 || bytes == 1536)) { free(digits); return -1; }
    uint8_t *buf = (uint8_t*)malloc(bytes);
    if (!buf) { free(digits); return -1; }
    for (size_t i = 0; i < bytes; ++i) {
        int hi = hex_digit(digits[i*2]);
        int lo = hex_digit(digits[i*2+1]);
        buf[i] = (uint8_t)((hi << 4) | lo);
    }
    free(digits);

    if (bytes == 192) {
        for (int i = 0; i < 64; ++i) {
            uint8_t r = buf[i*3 + 0];
            uint8_t g = buf[i*3 + 1];
            uint8_t b = buf[i*3 + 2];
            ppu__active_palette_base[i] = rgb_bytes_to_argb(r,g,b);
        }
        ppu__have_emphasis_tables = false;
        free(buf);
        return 0;
    } else if (bytes == 1536) {
        for (int e = 0; e < 8; ++e) {
            for (int i = 0; i < 64; ++i) {
                size_t off = (size_t)e*64*3 + (size_t)i*3;
                uint8_t r = buf[off + 0];
                uint8_t g = buf[off + 1];
                uint8_t b = buf[off + 2];
                uint32_t argb = rgb_bytes_to_argb(r,g,b);
                ppu__emphasis_palettes[e][i] = argb;
                if (e == 0) ppu__active_palette_base[i] = argb;
            }
        }
        ppu__have_emphasis_tables = true;
        free(buf);
        return 0;
    }
    free(buf);
    return -1;
}

int ppu_palette_load_hex_string(const char *hex_text) {
    if (!hex_text) return -1;
    if (try_parse_hex_tokens(hex_text) == 0) return 0;
    if (try_parse_raw_hex_bytes(hex_text) == 0) return 0;
    return -1;
}
