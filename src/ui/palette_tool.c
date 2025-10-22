/*
 * palette_tool.c - Runtime palette tool overlay and picker UI (SDL) implementation for Cupid NES Emulator
 * Author: @frankischilling
 * 
 * This file implements the runtime palette tool overlay and picker UI for the NES emulator.
 * It allows the user to change the PPU palette at runtime without rebuilding the emulator.
 * It also implements the picker UI for the palette tool.
 * 
 * This file is part of Cupid NES Emulator.
 * 
 * This program is free software: you can redistribute it and modify
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

// -------------------------------------------------------------------------
// Runtime palette storage
// Active base palette (no-emphasis) and optional emphasis tables (8 variants)
// Colors are stored as ARGB 0xFFRRGGBB for direct use by the renderer.
uint32_t ppu__active_palette_base[64];
uint32_t ppu__emphasis_palettes[8][64];
bool     ppu__have_emphasis_tables = false;

// -------------------------------------------------------------------------
// UI overlay state
static bool show_overlay = false;
static Uint32 overlay_flash_ms = 0;
static Uint8 overlay_flash_r = 0, overlay_flash_g = 0, overlay_flash_b = 0;

typedef struct {
    int pad, cols, rows, sw, sh, boxw, boxh, x0, y0, grid_x, grid_y;
} PaletteOverlayLayout;

typedef struct {
    bool open;
    int index;
    int x, y;
    int sv_size;
    int hue_w;
    int pad;
    SDL_Rect rect_sv, rect_hue, rect_sugg;
    float h, s, v;
    bool drag_sv, drag_hue;
} PalettePicker;

static PalettePicker picker;

void palette_tool_init(void) {
    memset(&picker, 0, sizeof(picker));
    picker.open = false; picker.index = -1;
}

void palette_tool_toggle_overlay(void) { show_overlay = !show_overlay; if (!show_overlay) picker.open = false; }
void palette_tool_hide_overlay(void) { show_overlay = false; picker.open = false; }
bool palette_tool_is_visible(void) { return show_overlay; }

void palette_tool_flash(bool ok) {
    overlay_flash_ms = 250;
    if (ok) { overlay_flash_r = 0x20; overlay_flash_g = 0xA0; overlay_flash_b = 0x40; }
    else    { overlay_flash_r = 0xC0; overlay_flash_g = 0x30; overlay_flash_b = 0x30; }
}

static void compute_palette_overlay_layout(int ww, int hh, PaletteOverlayLayout *L) {
    L->pad = 8; L->cols = 16; L->rows = 4; L->sw = (ww >= 800 ? 22 : (ww >= 640 ? 18 : 14));
    L->sh = L->sw; L->boxw = L->cols*L->sw + (L->cols+1); L->boxh = L->rows*L->sh + (L->rows+1) + 20;
    L->x0 = L->pad; L->y0 = L->pad;
    if (L->boxw + L->pad*2 > ww) L->x0 = 0;
    if (L->boxh + L->pad*2 > hh) L->y0 = 0;
    L->grid_x = L->x0 + L->pad + 1; L->grid_y = L->y0 + L->pad + 1 + 18;
}

static int overlay_swatch_index_at(const PaletteOverlayLayout *L, int px, int py) {
    int gx = px - L->grid_x, gy = py - L->grid_y; if (gx < 0 || gy < 0) return -1;
    int cellw = L->sw + 1, cellh = L->sh + 1; int c = gx / cellw, r = gy / cellh;
    if (c < 0 || c >= L->cols || r < 0 || r >= L->rows) return -1;
    int lx = gx % cellw, ly = gy % cellh; if (lx >= L->sw || ly >= L->sh) return -1;
    return r * L->cols + c;
}

static void rgb_to_hsv(Uint8 R, Uint8 G, Uint8 B, float *out_h, float *out_s, float *out_v) {
    float r = R / 255.0f, g = G / 255.0f, b = B / 255.0f;
    float max = r; if (g > max) max = g; if (b > max) max = b;
    float min = r; if (g < min) min = g; if (b < min) min = b;
    float v = max; float d = max - min; float s = (max == 0.0f) ? 0.0f : (d / max); float h = 0.0f;
    if (d != 0.0f) { if (max == r) h = (g - b) / d + (g < b ? 6.0f : 0.0f); else if (max == g) h = (b - r) / d + 2.0f; else h = (r - g) / d + 4.0f; h *= 60.0f; }
    if (out_h) *out_h = h;
    if (out_s) *out_s = s;
    if (out_v) *out_v = v;
}

static void hsv_to_rgb(float h, float s, float v, Uint8 *out_R, Uint8 *out_G, Uint8 *out_B) {
    float c = v * s; float hh = h / 60.0f; float x = c * (1.0f - fabsf(fmodf(hh, 2.0f) - 1.0f));
    float r1=0, g1=0, b1=0;
    if (0.0f <= hh && hh < 1.0f) { r1=c; g1=x; b1=0; }
    else if (1.0f <= hh && hh < 2.0f) { r1=x; g1=c; b1=0; }
    else if (2.0f <= hh && hh < 3.0f) { r1=0; g1=c; b1=x; }
    else if (3.0f <= hh && hh < 4.0f) { r1=0; g1=x; b1=c; }
    else if (4.0f <= hh && hh < 5.0f) { r1=x; g1=0; b1=c; }
    else { r1=c; g1=0; b1=x; }
    float m = v - c; float rf = r1 + m, gf = g1 + m, bf = b1 + m;
    int R = (int)(rf * 255.0f + 0.5f); int G = (int)(gf * 255.0f + 0.5f); int B = (int)(bf * 255.0f + 0.5f);
    if (R < 0) R = 0;
    if (R > 255) R = 255;
    if (G < 0) G = 0;
    if (G > 255) G = 255;
    if (B < 0) B = 0;
    if (B > 255) B = 255;
    if (out_R) *out_R = (Uint8)R;
    if (out_G) *out_G = (Uint8)G;
    if (out_B) *out_B = (Uint8)B;
}

static void picker_init(int mouse_x, int mouse_y, uint32_t current_argb, int ww, int hh) {
    picker.open = true; picker.x = mouse_x; picker.y = mouse_y; picker.sv_size = 160; picker.hue_w = 16; picker.pad = 8;
    if (picker.x + picker.sv_size + picker.hue_w + picker.pad*3 > ww) picker.x = ww - (picker.sv_size + picker.hue_w + picker.pad*3);
    if (picker.x < 0) picker.x = 0;
    if (picker.y + picker.sv_size + picker.pad*4 + 24 > hh) picker.y = hh - (picker.sv_size + picker.pad*4 + 24);
    if (picker.y < 0) picker.y = 0;
    picker.rect_sv.x = picker.x + picker.pad; picker.rect_sv.y = picker.y + picker.pad; picker.rect_sv.w = picker.sv_size; picker.rect_sv.h = picker.sv_size;
    picker.rect_hue.x = picker.rect_sv.x + picker.rect_sv.w + picker.pad; picker.rect_hue.y = picker.rect_sv.y; picker.rect_hue.w = picker.hue_w; picker.rect_hue.h = picker.sv_size;
    picker.rect_sugg.x = picker.x + picker.pad; picker.rect_sugg.y = picker.rect_sv.y + picker.rect_sv.h + picker.pad; picker.rect_sugg.w = picker.sv_size + picker.hue_w + picker.pad; picker.rect_sugg.h = 24;
    Uint8 R = (Uint8)((current_argb >> 16) & 0xFF); Uint8 G = (Uint8)((current_argb >> 8) & 0xFF); Uint8 B = (Uint8)( current_argb & 0xFF);
    rgb_to_hsv(R,G,B, &picker.h, &picker.s, &picker.v); picker.drag_sv = false; picker.drag_hue = false;
}

static void draw_palette_overlay(SDL_Renderer *rr, int ww, int hh) {
    uint32_t colors[64]; ppu_palette_get(colors);
    SDL_SetRenderDrawBlendMode(rr, SDL_BLENDMODE_BLEND);
    PaletteOverlayLayout L; compute_palette_overlay_layout(ww, hh, &L);
    Uint8 bgR = 0x00, bgG = 0x00, bgB = 0x00, bgA = 180; if (overlay_flash_ms) { bgR = overlay_flash_r; bgG = overlay_flash_g; bgB = overlay_flash_b; }
    SDL_Rect bg = { L.x0, L.y0, L.boxw + L.pad*2, L.boxh + L.pad*2 };
    SDL_SetRenderDrawColor(rr, bgR, bgG, bgB, bgA); SDL_RenderFillRect(rr, &bg);
    SDL_SetRenderDrawColor(rr, 220, 220, 220, 220); SDL_RenderDrawRect(rr, &bg);
    SDL_SetRenderDrawColor(rr, 255, 255, 255, 40); SDL_Rect hdr = { L.x0 + L.pad, L.y0 + L.pad, L.boxw, 16 }; SDL_RenderFillRect(rr, &hdr);
    int sx0 = L.grid_x, sy0 = L.grid_y; for (int i = 0; i < 64; ++i) {
        int r = i / L.cols, c = i % L.cols; uint32_t argb = colors[i]; Uint8 R = (Uint8)((argb >> 16) & 0xFF); Uint8 G = (Uint8)((argb >> 8) & 0xFF); Uint8 B = (Uint8)( argb & 0xFF);
        SDL_SetRenderDrawColor(rr, R, G, B, 255); SDL_Rect swr = { sx0 + c*L.sw + c, sy0 + r*L.sh + r, L.sw, L.sh }; SDL_RenderFillRect(rr, &swr);
        SDL_SetRenderDrawColor(rr, 0, 0, 0, 160); SDL_RenderDrawRect(rr, &swr);
    }
}

static void draw_picker(SDL_Renderer *rr) {
    SDL_SetRenderDrawBlendMode(rr, SDL_BLENDMODE_BLEND);
    SDL_Rect bg = { picker.x, picker.y, picker.rect_sugg.x + picker.rect_sugg.w - picker.x + picker.pad, picker.rect_sugg.y + picker.rect_sugg.h - picker.y + picker.pad };
    SDL_SetRenderDrawColor(rr, 20, 20, 20, 220); SDL_RenderFillRect(rr, &bg);
    SDL_SetRenderDrawColor(rr, 220, 220, 220, 220); SDL_RenderDrawRect(rr, &bg);
    for (int iy = 0; iy < picker.rect_sv.h; iy+=2) {
        float v = 1.0f - (float)iy / (float)(picker.rect_sv.h - 1);
        for (int ix = 0; ix < picker.rect_sv.w; ix+=2) {
            float s = (float)ix / (float)(picker.rect_sv.w - 1);
            Uint8 R,G,B; hsv_to_rgb(picker.h, s, v, &R,&G,&B);
            SDL_SetRenderDrawColor(rr, R, G, B, 255);
            SDL_Rect r = { picker.rect_sv.x + ix, picker.rect_sv.y + iy, 2, 2 }; SDL_RenderFillRect(rr, &r);
        }
    }
    int cx = picker.rect_sv.x + (int)(picker.s * (picker.rect_sv.w - 1));
    int cy = picker.rect_sv.y + (int)((1.0f - picker.v) * (picker.rect_sv.h - 1));
    SDL_SetRenderDrawColor(rr, 0, 0, 0, 255); SDL_Rect cur = { cx-3, cy-3, 7, 7 }; SDL_RenderDrawRect(rr, &cur);
    SDL_SetRenderDrawColor(rr, 255, 255, 255, 255); SDL_Rect cur2 = { cx-2, cy-2, 5, 5 }; SDL_RenderDrawRect(rr, &cur2);
    for (int iy = 0; iy < picker.rect_hue.h; iy++) {
        float h = (float)iy / (float)(picker.rect_hue.h - 1) * 360.0f;
        Uint8 R,G,B; hsv_to_rgb(h, 1.0f, 1.0f, &R,&G,&B);
        SDL_SetRenderDrawColor(rr, R, G, B, 255);
        SDL_RenderDrawLine(rr, picker.rect_hue.x, picker.rect_hue.y + iy, picker.rect_hue.x + picker.rect_hue.w - 1, picker.rect_hue.y + iy);
    }
    int hy = picker.rect_hue.y + (int)(picker.h / 360.0f * (picker.rect_hue.h - 1));
    SDL_SetRenderDrawColor(rr, 255, 255, 255, 255); SDL_RenderDrawLine(rr, picker.rect_hue.x, hy, picker.rect_hue.x + picker.rect_hue.w - 1, hy);
    int sugg_sw = 22; int sugg_gap = 6; int base_x = picker.rect_sugg.x; int base_y = picker.rect_sugg.y + 2; int mod16 = picker.index & 15;
    uint32_t sugg[4] = { nes_palette[mod16], nes_palette[16+mod16], nes_palette[32+mod16], nes_palette[48+mod16] };
    for (int i = 0; i < 4; i++) { uint32_t c = sugg[i]; Uint8 R = (Uint8)((c >> 16) & 0xFF); Uint8 G = (Uint8)((c >> 8) & 0xFF); Uint8 B = (Uint8)( c & 0xFF);
        SDL_SetRenderDrawColor(rr, R,G,B,255); SDL_Rect sw = { base_x + i*(sugg_sw + sugg_gap), base_y, sugg_sw, picker.rect_sugg.h - 4 }; SDL_RenderFillRect(rr, &sw);
        SDL_SetRenderDrawColor(rr, 0,0,0,160); SDL_RenderDrawRect(rr, &sw); }
}

void palette_tool_draw(SDL_Renderer *renderer, int ww, int hh) {
    if (!show_overlay) return;
    draw_palette_overlay(renderer, ww, hh);
    if (picker.open) draw_picker(renderer);
}

void palette_tool_handle_event(const SDL_Event *e, SDL_Renderer *renderer) {
    (void)renderer;
    if (!show_overlay) return;
    if (e->type == SDL_MOUSEBUTTONDOWN && e->button.button == SDL_BUTTON_LEFT) {
        int mx = e->button.x, my = e->button.y; int ww=0,hh=0; SDL_GetRendererOutputSize(renderer, &ww, &hh);
        if (picker.open) {
            if (mx >= picker.rect_sv.x && mx < picker.rect_sv.x + picker.rect_sv.w && my >= picker.rect_sv.y && my < picker.rect_sv.y + picker.rect_sv.h) {
                picker.drag_sv = true; float s = (float)(mx - picker.rect_sv.x) / (float)(picker.rect_sv.w - 1); float v = 1.0f - (float)(my - picker.rect_sv.y) / (float)(picker.rect_sv.h - 1);
                if (s < 0) s = 0;
                if (s > 1) s = 1;
                if (v < 0) v = 0;
                if (v > 1) v = 1;
                picker.s = s; picker.v = v;
            } else if (mx >= picker.rect_hue.x && mx < picker.rect_hue.x + picker.rect_hue.w && my >= picker.rect_hue.y && my < picker.rect_hue.y + picker.rect_hue.h) {
                picker.drag_hue = true; float h = (float)(my - picker.rect_hue.y) / (float)(picker.rect_hue.h - 1) * 360.0f; if (h < 0) h = 0; if (h > 360) h = 360; picker.h = h;
            } else if (mx >= picker.rect_sugg.x && mx < picker.rect_sugg.x + picker.rect_sugg.w && my >= picker.rect_sugg.y && my < picker.rect_sugg.h) {
                int sugg_sw = 22; int sugg_gap = 6; int idx = (mx - picker.rect_sugg.x) / (sugg_sw + sugg_gap);
                if (idx >= 0 && idx < 4) { int mod16 = picker.index & 15; uint32_t c = nes_palette[idx*16 + mod16]; Uint8 R = (Uint8)((c >> 16) & 0xFF); Uint8 G = (Uint8)((c >> 8) & 0xFF); Uint8 B = (Uint8)( c & 0xFF);
                    ppu_palette_set_color(picker.index, R,G,B); palette_tool_flash(true); }
            } else { picker.open = false; }
        } else {
            PaletteOverlayLayout L; compute_palette_overlay_layout(ww, hh, &L); int idx = overlay_swatch_index_at(&L, mx, my);
            if (idx >= 0) { uint32_t colors[64]; ppu_palette_get(colors); picker.index = idx; picker_init(mx, my, colors[idx], ww, hh); }
        }
    } else if (e->type == SDL_MOUSEBUTTONUP && e->button.button == SDL_BUTTON_LEFT) {
        if (picker.open) { picker.drag_sv = false; picker.drag_hue = false; }
    } else if (e->type == SDL_MOUSEMOTION) {
        if (picker.open && (picker.drag_sv || picker.drag_hue)) {
            int mx = e->motion.x, my = e->motion.y;
            if (picker.drag_sv) { float s = (float)(mx - picker.rect_sv.x) / (float)(picker.rect_sv.w - 1); float v = 1.0f - (float)(my - picker.rect_sv.y) / (float)(picker.rect_sv.h - 1);
                if (s < 0) s = 0;
                if (s > 1) s = 1;
                if (v < 0) v = 0;
                if (v > 1) v = 1;
                picker.s = s; picker.v = v; }
            if (picker.drag_hue) { float h = (float)(my - picker.rect_hue.y) / (float)(picker.rect_hue.h - 1) * 360.0f; if (h < 0) h = 0; if (h > 360) h = 360; picker.h = h; }
            Uint8 R,G,B; hsv_to_rgb(picker.h, picker.s, picker.v, &R,&G,&B); if (picker.index >= 0) ppu_palette_set_color(picker.index, R,G,B);
        }
    }
}

void palette_tool_tick(Uint32 delta_ms) {
    if (overlay_flash_ms) {
        if (delta_ms >= overlay_flash_ms) overlay_flash_ms = 0; else overlay_flash_ms -= delta_ms;
    }
}

// -------------------------------------------------------------------------
// Palette management functions

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

// Read whole file into memory buffer (malloc'd). Returns 0 on success.
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
        // 8 emphasis sets * 64 entries * 3 bytes
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
        return -7; // unsupported size
    }
}

static int hex_digit(int c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

// Try parsing as 64 tokens of RRGGBB (optionally prefixed by # or 0x/$)
static int try_parse_hex_tokens(const char *s) {
    uint32_t tmp[64];
    int count = 0;
    const char *p = s;
    while (*p && count < 64) {
        while (*p && (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t' || *p == ',' || *p == ';')) p++;
        if (!*p) break;
        // skip prefixes
        if (*p == '#') p++;
        if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) p += 2;
        if (*p == '$') p++;
        // need 6 hex digits
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
        // consume trailing token separators if any
        while (*p && ((*p >= '0' && *p <= '9') || (*p >= 'A' && *p <= 'F') || (*p >= 'a' && *p <= 'f'))) {
            // If there are more than 6 consecutive hex digits without separators,
            // this wasn't tokenized; fall back to raw parsing.
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

// Parse raw byte hex: accept any non-hex separators, gather hex pairs
static int try_parse_raw_hex_bytes(const char *s) {
    // Collect hex digits
    size_t cap = 2048; // enough for 1536*2 digits
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