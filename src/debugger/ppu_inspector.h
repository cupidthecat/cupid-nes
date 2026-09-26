/*
 * ppu_inspector.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* PPU inspector snapshots and pixel decoding. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_PPU_INSPECTOR_H
#define CUPID_PPU_INSPECTOR_H
#include "debugger.h"
#include "../rom/mapper.h"

enum {
    DEBUG_PPU_PATTERNS = 0x1D00,
    DEBUG_PPU_NAMETABLES,
    DEBUG_PPU_SPRITES,
    DEBUG_PPU_REGISTERS,
    DEBUG_PPU_VRAM,
    DEBUG_PPU_TILE,
    DEBUG_PPU_PALETTE
};

typedef struct {
    DebugPpuSnapshot state;
    uint8_t memory[0x4000], sprites[0x2000], background[0x2000], oam[256];
    uint8_t nt_planes[3840][16], nt_palettes[3840];
    uint32_t colors[64];
    Mirroring mirroring;
    uint64_t session;
} DebugPpuImage;

typedef struct {
    uint16_t pattern, nametable, attribute;
    unsigned palette, tile, x, y, height;
    bool flip_x, flip_y, behind;
} DebugPpuSelection;

void debug_ppu_capture(DebugPpuImage *image, bool nametables);
uint8_t debug_ppu_pixel(const uint8_t *planes, unsigned x, unsigned y);
uint32_t debug_ppu_color(const DebugPpuImage *image, unsigned palette, unsigned pixel, bool sprite);
DebugPpuSelection debug_ppu_sprite(const DebugPpuImage *image, unsigned index);
DebugPpuSelection debug_ppu_nametable(const DebugPpuImage *image, unsigned x, unsigned y);
/* ARGB8888 output. Caller provides width * height pixels. */
void debug_ppu_patterns(const DebugPpuImage *image, const uint8_t *chr, unsigned palette, uint32_t out[256 * 128]);
void debug_ppu_nametables(const DebugPpuImage *image, bool attributes, uint32_t out[512 * 480]);
void debug_ppu_sprites(const DebugPpuImage *image, bool screen, uint32_t out[256 * 240]);
/* Edits require a paused, live debugger. ROM and protected pages are rejected. */
bool debug_ppu_write(bool oam, unsigned address, uint8_t value);
bool debug_ppu_paint(unsigned tile_address, unsigned x, unsigned y, unsigned color);
#endif
