/*
 * ppu.h - Picture Processing Unit (PPU) header
 * 
 * Author: @frankischilling
 * 
 * This header defines the PPU structure, registers, memory layout, and function prototypes
 * for the NES PPU emulator. Includes definitions for nametables, OAM (Object Attribute Memory),
 * palette memory, and the NES color palette.
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

#ifndef PPU_H
#define PPU_H

#include <stdint.h>
#include <stdbool.h>
#include <SDL2/SDL.h>

// PPU Memory Sizes
#define NT_RAM_SIZE 0x1000    // 4KB nametable RAM (supports four-screen; normal carts use 2KB)
#define PPU_OAM_SIZE 0x100    // 256 bytes OAM
#define PPU_PALETTE_SIZE 0x20 // 32 bytes palette RAM

// PPU Register Addresses
#define PPUCTRL   0x2000
#define PPUMASK   0x2001
#define PPUSTATUS 0x2002
#define OAMADDR   0x2003
#define OAMDATA   0x2004
#define PPUSCROLL 0x2005
#define PPUADDR   0x2006
#define PPUDATA   0x2007
#define OAMDMA    0x4014

// PPU Registers
typedef struct {
    uint8_t ctrl;       // PPUCTRL
    uint8_t mask;       // PPUMASK
    uint8_t status;     // PPUSTATUS
    uint8_t oam_addr;   // OAMADDR
    uint8_t scroll_x;   // PPUSCROLL X (fine X is 'x' below)
    uint8_t scroll_y;   // PPUSCROLL Y

    // Loopy registers / VRAM addressing
    uint16_t v;         // current VRAM address (15 bits)
    uint16_t t;         // temporary VRAM address (15 bits)
    uint8_t  x;         // fine X scroll (3 bits)
    uint8_t  w;         // write toggle for $2005/$2006 (0 or 1)

    // PPUDATA read buffer and open bus
    uint8_t ppudata_buffer;
    uint8_t open_bus;

    uint8_t oam[PPU_OAM_SIZE]; // Object Attribute Memory
    uint8_t secondary_oam[32]; // Secondary OAM for sprite evaluation
    uint8_t sprite_count;      // Number of sprites found on current scanline
    uint8_t sprite_positions[8]; // Sprite X positions
    uint8_t sprite_patterns[8];  // Sprite pattern data
    uint8_t sprite_attributes[8]; // Sprite attributes
    bool sprite_zero_hit;  // Sprite Zero Hit flag
    bool nmi_out; 

    bool   have_split;
    int split_x;     // pixel 0..255 where first opaque overlap occurs
    int split_y;     // scanline 0..239
    int split_cpu_cycles; // when to assert hit this frame (CPU cycles since start of visible)

    uint16_t t_pre,  t_post;
    uint8_t  x_pre,  x_post;
    uint8_t  ctrl_pre, ctrl_post;
    bool     post_scroll_valid;
    bool     pre_scroll_valid;
    
    // Track post-hit writes separately
    bool     wrote_2000_post;     // saw $2000 after sprite 0 hit (this frame, before vblank)
    bool     wrote_2005_x_post;   // saw first $2005 after sprite 0 hit
    bool     wrote_2005_y_post;   // saw second $2005 after sprite 0 hit (vertical; for completeness)
} PPU;

// PPU Memory
extern uint8_t ppu_vram[NT_RAM_SIZE];        // Nametable RAM only
extern uint8_t ppu_palette[PPU_PALETTE_SIZE];  // Palette RAM
extern PPU ppu;  // PPU registers global variable

// NES Palette Colors (RGB format)
// More accurate NES palette (NTSC approximations)
static const uint32_t nes_palette[64] = {
    0xFF7C7C7C, 0xFF0000FC, 0xFF0000BC, 0xFF4428BC, 0xFF940084, 0xFFA80020, 0xFFA81000, 0xFF881400,
    0xFF503000, 0xFF007800, 0xFF006800, 0xFF005800, 0xFF004058, 0xFF000000, 0xFF000000, 0xFF000000,
    0xFFBCBCBC, 0xFF0078F8, 0xFF0058F8, 0xFF6844FC, 0xFFD800CC, 0xFFE40058, 0xFFF83800, 0xFFE45C10,
    0xFFAC7C00, 0xFF00B800, 0xFF00A800, 0xFF00A844, 0xFF008888, 0xFF000000, 0xFF000000, 0xFF000000,
    0xFFF8F8F8, 0xFF3CBCFC, 0xFF6888FC, 0xFF9878F8, 0xFFF878F8, 0xFFF85898, 0xFFF87858, 0xFFFCA044,
    0xFFF8B800, 0xFFB8F818, 0xFF58D854, 0xFF58F898, 0xFF00E8D8, 0xFF787878, 0xFF000000, 0xFF000000,
    0xFFFCFCFC, 0xFFA4E4FC, 0xFFB8B8F8, 0xFFD8B8F8, 0xFFF8B8F8, 0xFFF8A4C0, 0xFFF0D0B0, 0xFFFCE0A8,
    0xFFF8D878, 0xFFD8F878, 0xFFB8F8B8, 0xFFB8F8D8, 0xFF00FCFC, 0xFFF8D8F8, 0xFF000000, 0xFF000000
};

// Function prototypes
uint8_t ppu_read(uint16_t addr);
void ppu_write(uint16_t addr, uint8_t value);
void ppu_reset(PPU* ppu);
void render_tiles();
uint32_t get_color(uint8_t pixel);
void render_background(uint32_t *framebuffer);
void start_frame();
void ppu_render_frame(SDL_Renderer *renderer, SDL_Texture *texture);
uint8_t ppu_reg_read(uint16_t reg);
void ppu_reg_write(uint16_t reg, uint8_t value);
void ppu_oam_dma(uint8_t page);
void ppu_begin_vblank(void);
void ppu_end_vblank(void);
void render_sprites(uint32_t *framebuffer);
extern uint8_t bg_opaque[256 * 240];
void ppu_predict_sprite0_split_for_frame(void);
void ppu_latch_pre_for_visible(void);

#endif // PPU_H
