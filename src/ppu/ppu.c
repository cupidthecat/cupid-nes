// ppu.c
#include "ppu.h"
#include "../rom/rom.h"
#include "../include/globals.h"
#include <stdbool.h>

// PPU Memory
uint8_t ppu_vram[PPU_VRAM_SIZE];
uint8_t ppu_palette[PPU_PALETTE_SIZE];

// PPU Registers
PPU ppu;

static uint16_t mirror_nametable_addr(uint16_t addr) {
    // addr is already masked to 0x3FFF in ppu_read/ppu_write
    // For addresses in 0x2000-0x3EFF (nametables and mirrors):
    if(addr >= 0x2000 && addr < 0x3F00) {
        uint16_t offset = addr - 0x2000;
        // Determine which nametable (0-3)
        int nametable = offset / 0x400;
        int innerOffset = offset % 0x400;
        // Use mirroring mode from the ROM header:
        if(mirroring_mode == 0) { // HORIZONTAL mirroring
            // NT0 and NT1 are unique; NT2 mirrors NT0, NT3 mirrors NT1.
            if(nametable == 0 || nametable == 2)
                return 0x2000 + innerOffset;
            else
                return 0x2400 + innerOffset;
        } else { // VERTICAL mirroring
            // NT0 and NT2 are unique; NT1 mirrors NT0, NT3 mirrors NT2.
            if(nametable == 0 || nametable == 1)
                return 0x2000 + innerOffset;
            else
                return 0x2800 + innerOffset;
        }
    }
    return addr; // Other areas are not modified
}

// PPU Register Read
uint8_t ppu_read(uint16_t addr) {
    addr &= 0x3FFF; // Mirror every 16KB

    // Handle palette mirroring: addresses 0x3F00-0x3F1F.
    if(addr >= 0x3F00 && addr < 0x3F20) {
        return ppu_palette[addr & 0x1F];
    }
    
    // Nametable read: apply mirroring.
    if(addr >= 0x2000 && addr < 0x3F00) {
        uint16_t effective_addr = mirror_nametable_addr(addr);
        return ppu_vram[effective_addr];
    }
    
    // Pattern tables and others.
    return ppu_vram[addr];
}

// PPU Register Write
void ppu_write(uint16_t addr, uint8_t value) {
    addr &= 0x3FFF; // Mirror every 16KB

    if(addr >= 0x3F00 && addr < 0x3F20) {
        ppu_palette[addr & 0x1F] = value;
        return;
    }
    
    if(addr >= 0x2000 && addr < 0x3F00) {
        uint16_t effective_addr = mirror_nametable_addr(addr);
        ppu_vram[effective_addr] = value;
        return;
    }
    
    ppu_vram[addr] = value;
}

// Reset PPU to initial state
void ppu_reset(PPU* ppu) {
    ppu->ctrl = 0;
    ppu->mask = 0;
    ppu->status = 0xA0; // Status bits 5 and 7 are always set
    ppu->oam_addr = 0;
    ppu->scroll_x = 0;
    ppu->scroll_y = 0;
    ppu->vram_addr = 0;
    ppu->vram_latch = 0;
    
    // Clear OAM
    for (int i = 0; i < PPU_OAM_SIZE; i++) {
        ppu->oam[i] = 0xFF;
    }
}

// Convert a 2-bit pixel value to an ARGB color (using a simple grayscale palette)
uint32_t get_color(uint8_t pixel) {
    // Simple mapping: 0->black, 1->dark gray, 2->light gray, 3->white.
    uint8_t intensity;
    switch(pixel) {
        case 0: intensity = 0x00; break;
        case 1: intensity = 0x55; break;
        case 2: intensity = 0xAA; break;
        case 3: intensity = 0xFF; break;
        default: intensity = 0x00; break;
    }
    // ARGB: alpha=0xFF, then intensity for red, green, blue.
    return (0xFF << 24) | (intensity << 16) | (intensity << 8) | intensity;
}

// Render the first pattern table (first 4KB of CHR-ROM) as a grid of 16x16 8x8 tiles.
// Each tile is decoded from 16 bytes (8 bytes for plane 0 and 8 bytes for plane 1).
// The resulting image is 128x128 pixels and is centered in the 256x240 screen.
void render_tiles() {
    if (!chr_rom) {
        // Fallback to test pattern if no CHR-ROM data
        for (int i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; i++)
            framebuffer[i] = 0xFF0000FF; // solid blue, for example
        return;
    }

    const int tileWidth = 8;
    const int tileHeight = 8;
    const int tilesPerRow = 16;
    const int gridWidth = tilesPerRow * tileWidth; // 128 pixels
    const int gridHeight = tilesPerRow * tileHeight; // 128 pixels (16 rows)

    // Compute top-left corner to center the grid on the screen
    int offsetX = (SCREEN_WIDTH - gridWidth) / 2;
    int offsetY = (SCREEN_HEIGHT - gridHeight) / 2;

    // Clear framebuffer (set to black)
    for (int i = 0; i < SCREEN_WIDTH * SCREEN_HEIGHT; i++) {
        framebuffer[i] = 0xFF000000;
    }

    // For each tile in the first pattern table (4KB of data = 256 tiles)
    for (int tile = 0; tile < 256; tile++) {
        int tileX = (tile % tilesPerRow) * tileWidth;
        int tileY = (tile / tilesPerRow) * tileHeight;
        int tileOffset = tile * 16;  // each tile uses 16 bytes

        // For each row in the tile (0 to 7)
        for (int row = 0; row < tileHeight; row++) {
            // The two bit planes: first 8 bytes and second 8 bytes.
            uint8_t plane0 = chr_rom[tileOffset + row];
            uint8_t plane1 = chr_rom[tileOffset + row + 8];

            // For each pixel in the row (0 to 7)
            for (int col = 0; col < tileWidth; col++) {
                // The bit for this pixel is the (7-col) bit of each plane.
                uint8_t bit0 = (plane0 >> (7 - col)) & 1;
                uint8_t bit1 = (plane1 >> (7 - col)) & 1;
                uint8_t pixelValue = (bit1 << 1) | bit0;

                // Compute the position in the framebuffer:
                int screenX = offsetX + tileX + col;
                int screenY = offsetY + tileY + row;
                if (screenX < 0 || screenX >= SCREEN_WIDTH ||
                    screenY < 0 || screenY >= SCREEN_HEIGHT) {
                    continue;
                }
                framebuffer[screenY * SCREEN_WIDTH + screenX] = get_color(pixelValue);
            }
        }
    }
}