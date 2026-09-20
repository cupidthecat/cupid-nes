/* PPU inspector snapshots and pixel decoding. SPDX-License-Identifier: GPL-3.0-or-later */
#include "ppu_inspector.h"
#include "../ppu/ppu.h"
#include "../system/execution_policy.h"
#include <string.h>

void debug_ppu_capture(DebugPpuImage *image, bool nametables) {
    debugger_get_ppu(&image->state);
    image->session = debugger_session_revision();
    image->mirroring = cart_get_mirroring();
    for (unsigned i = 0; i < 0x4000; ++i) {
        image->memory[i] = debugger_peek_ppu((uint16_t)i);
    }
    for (unsigned i = 0; i < 0x2000; ++i) {
        image->sprites[i] = cart_debug_chr((uint16_t)i, CART_PPU_FETCH_SPRITE);
        image->background[i] = cart_debug_chr((uint16_t)i, CART_PPU_FETCH_BG);
    }
    debugger_copy_oam(image->oam);
    for (unsigned i = 0; i < 64; ++i) {
        image->colors[i] = 0xFF000000u | get_color((uint8_t)i);
    }
    if (!nametables) {
        return;
    }
    for (unsigned table = 0; table < 4; ++table) {
        for (unsigned tile = 0; tile < 960; ++tile) {
            unsigned address = 0x2000 + table * 0x400 + tile;
            unsigned x = tile % 32, y = tile / 32;
            unsigned attr = 0x23C0 + table * 0x400 + (y / 4) * 8 + x / 4;
            uint8_t palette = (image->memory[attr] >> ((x & 2) | ((y & 2) << 1))) & 3;
            unsigned pattern = ((image->state.ctrl & 0x10) ? 0x1000 : 0) + image->memory[address] * 16;
            unsigned index = table * 960 + tile;
            for (unsigned row = 0; row < 8; ++row) {
                cart_debug_bg_row((uint16_t)address, (uint16_t)pattern, row, &image->nt_planes[index][row],
                                  &image->nt_planes[index][row + 8], &palette);
            }
            image->nt_palettes[index] = palette;
        }
    }
}

uint8_t debug_ppu_pixel(const uint8_t *planes, unsigned x, unsigned y) {
    unsigned shift = 7 - (x & 7);
    return (uint8_t)(((planes[y & 7] >> shift) & 1) | (((planes[(y & 7) + 8] >> shift) & 1) << 1));
}

uint32_t debug_ppu_color(const DebugPpuImage *image, unsigned palette, unsigned pixel, bool sprite) {
    if (!pixel && sprite) {
        return 0; /* Transparent pixels are never palette RAM entry 0. */
    }
    unsigned address = pixel ? (palette & 7) * 4 + (pixel & 3) : 0;
    return image->colors[image->memory[0x3F00 + address] & 0x3F];
}

DebugPpuSelection debug_ppu_sprite(const DebugPpuImage *image, unsigned index) {
    const uint8_t *o = image->oam + (index & 63) * 4;
    unsigned height = image->state.ctrl & 0x20 ? 16 : 8;
    unsigned address =
        height == 16 ? ((o[1] & 1) * 0x1000 + (o[1] & 0xFE) * 16) : ((image->state.ctrl & 8 ? 0x1000 : 0) + o[1] * 16);
    return (DebugPpuSelection){.pattern = (uint16_t)address,
                               .palette = 4 + (o[2] & 3),
                               .tile = o[1],
                               .x = o[3],
                               .y = (unsigned)o[0] + 1,
                               .height = height,
                               .flip_x = (o[2] & 0x40) != 0,
                               .flip_y = (o[2] & 0x80) != 0,
                               .behind = (o[2] & 0x20) != 0};
}

DebugPpuSelection debug_ppu_nametable(const DebugPpuImage *image, unsigned x, unsigned y) {
    x %= 512;
    y %= 480;
    unsigned table = x / 256 + 2 * (y / 240), column = (x % 256) / 8, row = (y % 240) / 8;
    unsigned address = 0x2000 + table * 0x400 + row * 32 + column;
    unsigned tile = image->memory[address];
    return (DebugPpuSelection){.nametable = (uint16_t)address,
                               .attribute = (uint16_t)(0x23C0 + table * 0x400 + (row / 4) * 8 + column / 4),
                               .pattern = (uint16_t)((image->state.ctrl & 0x10 ? 0x1000 : 0) + tile * 16),
                               .tile = tile,
                               .palette = image->nt_palettes[table * 960 + row * 32 + column],
                               .x = x,
                               .y = y,
                               .height = 8};
}

void debug_ppu_patterns(const DebugPpuImage *image, const uint8_t *chr, unsigned palette, uint32_t out[256 * 128]) {
    for (unsigned tile = 0; tile < 512; ++tile) {
        unsigned left = (tile / 256) * 128 + (tile % 16) * 8, top = ((tile % 256) / 16) * 8;
        for (unsigned y = 0; y < 8; ++y) {
            for (unsigned x = 0; x < 8; ++x) {
                out[(top + y) * 256 + left + x] =
                    debug_ppu_color(image, palette, debug_ppu_pixel(chr + tile * 16, x, y), false);
            }
        }
    }
}

void debug_ppu_nametables(const DebugPpuImage *image, bool attributes, uint32_t out[512 * 480]) {
    for (unsigned y = 0; y < 480; ++y) {
        for (unsigned x = 0; x < 512; ++x) {
            unsigned table = x / 256 + 2 * (y / 240), column = (x % 256) / 8, row = (y % 240) / 8;
            unsigned tile = table * 960 + row * 32 + column;
            unsigned pixel =
                attributes ? ((x & 4) >> 2) + ((y & 4) >> 1) : debug_ppu_pixel(image->nt_planes[tile], x, y);
            out[y * 512 + x] = debug_ppu_color(image, image->nt_palettes[tile], pixel, false);
        }
    }
}

void debug_ppu_sprites(const DebugPpuImage *image, bool screen, uint32_t out[256 * 240]) {
    for (unsigned y = 0; y < 240; ++y) {
        for (unsigned x = 0; x < 256; ++x) {
            out[y * 256 + x] = ((x / 8 + y / 8) & 1) ? 0xFF202839 : 0xFF30394B;
        }
    }
    /* Lower OAM indices win overlapping opaque sprite pixels. */
    for (int index = 63; index >= 0; --index) {
        DebugPpuSelection s = debug_ppu_sprite(image, (unsigned)index);
        unsigned left = screen ? s.x : (unsigned)(index % 16) * 16 + 4;
        unsigned top = screen ? s.y : (unsigned)(index / 16) * 32 + 8;
        for (unsigned y = 0; y < s.height; ++y) {
            for (unsigned x = 0; x < 8; ++x) {
                unsigned sy = s.flip_y ? s.height - 1 - y : y, sx = s.flip_x ? 7 - x : x;
                const uint8_t *planes = image->sprites + s.pattern + (sy / 8) * 16;
                uint32_t color = debug_ppu_color(image, s.palette, debug_ppu_pixel(planes, sx, sy), true);
                if (color && left + x < 256 && top + y < 240) {
                    out[(top + y) * 256 + left + x] = color;
                }
            }
        }
    }
}

bool debug_ppu_write(bool oam, unsigned address, uint8_t value) {
    if (!debugger_is_paused() || nes_execution_policy() != NES_EXECUTION_LIVE) {
        return false;
    }
    if (oam) {
        if (address > 255) {
            return false;
        }
        ppu.oam[address] = (address & 3) == 2 ? value & 0xE3 : value;
        return true;
    }
    return address < 0x4000 && ppu_debug_write((uint16_t)address, value);
}

bool debug_ppu_paint(unsigned tile_address, unsigned x, unsigned y, unsigned color) {
    if (tile_address > 0x1FF0 || (tile_address & 15) || x > 7 || y > 7 || color > 3 || !debugger_is_paused() ||
        nes_execution_policy() != NES_EXECUTION_LIVE) {
        return false;
    }
    unsigned low_address = tile_address + y, high_address = low_address + 8;
    uint8_t mask = (uint8_t)(0x80 >> x), low = debugger_peek_ppu((uint16_t)low_address),
            high = debugger_peek_ppu((uint16_t)high_address);
    uint8_t new_low = (uint8_t)((low & ~mask) | ((color & 1) ? mask : 0));
    uint8_t new_high = (uint8_t)((high & ~mask) | ((color & 2) ? mask : 0));
    if (!debug_ppu_write(false, low_address, new_low)) {
        return false;
    }
    if (!debug_ppu_write(false, high_address, new_high)) {
        (void)debug_ppu_write(false, low_address, low);
        return false;
    }
    return true;
}
