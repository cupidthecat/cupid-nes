/*
 * ppu.c - Picture Processing Unit (PPU) emulation
 * 
 * Author: @frankischilling
 * 
 * This file implements the NES PPU (Picture Processing Unit) which handles graphics rendering.
 * It manages background and sprite rendering, nametable memory, palette memory, VRAM addressing,
 * scrolling, sprite-0 collision detection, and timing (VBlank, NMI generation). Supports
 * various mirroring modes and split-screen scrolling effects.
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

#include "ppu.h"
#include "../rom/rom.h"
#include "../../include/globals.h"
#include "../rom/mapper.h"
#include "../cpu/cpu.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
// PPU Memory
uint8_t ppu_vram[NT_RAM_SIZE];
uint8_t ppu_palette[PPU_PALETTE_SIZE];
uint8_t bg_opaque[256 * 240];

// PPU Registers
PPU ppu;
extern CPU cpu;

// -------------------------------------------------------------------------
// Runtime palette storage
// Active base palette (no-emphasis) and optional emphasis tables (8 variants)
// Colors are stored as ARGB 0xFFRRGGBB for direct use by the renderer.
uint32_t ppu__active_palette_base[64];
uint32_t ppu__emphasis_palettes[8][64];
bool     ppu__have_emphasis_tables = false;

// Helpers ----------------------------------------------------------------
static inline void set_open_bus(uint8_t v) { ppu.open_bus = v; }
static inline uint8_t get_open_bus(void)   { return ppu.open_bus; }

static inline void ppu_eval_nmi(void){
    bool want = (ppu.ctrl & 0x80) && (ppu.status & 0x80);
    if (want && !ppu.nmi_out) cpu_nmi(&cpu);  // fire on rising edge only
    ppu.nmi_out = want;
}

static inline bool in_vblank_now(void) { return (ppu.status & 0x80) != 0; }

static inline bool ppu_show_bg_px(int x) {
    // BG enabled AND (not in left 8 unless left-8-BG mask is on)
    return (ppu.mask & 0x08) && (x >= 8 || (ppu.mask & 0x02));
}

static inline bool ppu_show_sp_px(int x) {
    // Sprites enabled AND (not in left 8 unless left-8-sprite mask is on)
    return (ppu.mask & 0x10) && (x >= 8 || (ppu.mask & 0x04));
}

void ppu_latch_pre_for_visible(void) {
    ppu.t_pre    = ppu.t;
    ppu.x_pre    = ppu.x;
    ppu.ctrl_pre = ppu.ctrl;
}

// $2000–$2007
uint8_t ppu_reg_read(uint16_t reg) {
    switch (reg & 7) {
        case 0: case 1: case 3: case 5: case 6: {
            uint8_t v = get_open_bus(); // PPU's open-bus latch
            return v;
        }

        case 2: { // PPUSTATUS
            uint8_t ob = get_open_bus();
            uint8_t ret = (ppu.status & 0xE0) | (ob & 0x1F);
            set_open_bus(ret);
            ppu.status &= ~0x80;       // clear VBlank
            ppu.w = 0;
            ppu_eval_nmi();            // output may drop
            return ret;
        }
        case 4: { // OAMDATA
            uint8_t v = ppu.oam[ppu.oam_addr];
            set_open_bus(v);
            return v;
        }
        case 7: { // PPUDATA (read)
            uint16_t addr = ppu.v & 0x3FFF;
            uint8_t ret;
            if (addr >= 0x3F00 && addr < 0x4000) {
                // palette reads return palette immediately but also update buffer with mirrored nametable
                ret = ppu_read(addr);
                ppu.ppudata_buffer = ppu_read(addr - 0x1000); // fill buffer with nametable mirror
                set_open_bus(ret); // PPU latch sees palette byte
            } else {
                // CPU gets buffered old byte; bus sees the newly fetched one
                ret = ppu.ppudata_buffer;
                uint8_t fetched = ppu_read(addr);
                ppu.ppudata_buffer = fetched;
                set_open_bus(fetched); // PPU latch sees newly fetched byte
            }
            ppu.v += (ppu.ctrl & 0x04) ? 32 : 1;
            return ret;
        }
        default: {
            // This should never happen since reg & 7 covers all cases 0-7
            // But add it to satisfy the compiler
            uint8_t v = get_open_bus();
            return v;
        }
    }
}

void ppu_reg_write(uint16_t reg, uint8_t value) {
    set_open_bus(value); // any write to PPU updates open bus
    switch (reg & 7) {
        case 0: { // PPUCTRL
            ppu.ctrl = value;
            ppu.t = (ppu.t & ~0x0C00) | ((value & 0x03) << 10);
            
            if (ppu.have_split && !(ppu.status & 0x80)) {
                ppu.ctrl_post = value;
                
                // Update only NT bits in the queued post state
                ppu.t_post = (ppu.t_post & ~0x0C00) | ((value & 0x03) << 10);
                
                ppu.wrote_2000_post = true;
                ppu.post_scroll_valid = true;   // horizontal copy at 257 can proceed
            }
            
            if (in_vblank_now()) ppu_latch_pre_for_visible();   // refresh pre-latch during vblank
            ppu_eval_nmi();
        } break;
        case 1: // PPUMASK
            ppu.mask = value;
            break;
        case 3: // OAMADDR
            ppu.oam_addr = value;
            break;
        case 4: // OAMDATA
            ppu.oam[ppu.oam_addr++] = value;
            break;
        case 5: {
            if (ppu.w == 0) {
                // first write: fine X + coarse X
                ppu.x = value & 0x07;
                ppu.t = (ppu.t & ~0x001F) | (value >> 3);
                ppu.w = 1;
                
                if (ppu.have_split && !(ppu.status & 0x80)) {
                    // queue only horizontal pieces
                    ppu.x_post = ppu.x;
                    ppu.t_post = (ppu.t_post & ~0x001F) | (ppu.t & 0x001F);
                    
                    ppu.wrote_2005_x_post = true;
                    ppu.post_scroll_valid = true;   // enough for the 257 horizontal copy
                }
            } else {
                // second write: fine Y + coarse Y
                ppu.t = (ppu.t & ~0x73E0)
                      | ((value & 0x07) << 12)   // fine Y
                      | ((value & 0xF8) << 2);   // coarse Y
                ppu.w = 0;
                
                if (ppu.have_split && !(ppu.status & 0x80)) {
                    // queue vertical pieces for next pre-render copy (not for 257)
                    ppu.t_post = (ppu.t_post & ~0x73E0) | (ppu.t & 0x73E0);
                    ppu.wrote_2005_y_post = true;
                    // do NOT change post_scroll_valid here; vertical doesn't apply at 257
                }
            }
            if (in_vblank_now()) ppu_latch_pre_for_visible();   // refresh pre-latch during vblank
        } break;
            
        case 6: { // PPUADDR
            if (ppu.w == 0) {
                // high byte (masked to 6 bits)
                ppu.t = (ppu.t & 0x00FF) | ((uint16_t)(value & 0x3F) << 8);
                ppu.w = 1;
            } else {
                // low byte; commit to v
                ppu.t = (ppu.t & 0xFF00) | value;
                ppu.v = ppu.t;
                ppu.w = 0;
            }
            break;
        }
        case 7: { // PPUDATA (write)
            uint16_t addr = ppu.v & 0x3FFF;
            ppu_write(addr, value);            
        
            // Common quirk: palette writes update the internal buffer as if reading $xF00
            if (addr >= 0x3F00 && addr < 0x4000) {
                ppu.ppudata_buffer = ppu_read(addr - 0x1000);
            }
            ppu.v += (ppu.ctrl & 0x04) ? 32 : 1;
            break;                               // <-- no return here
        }
    }
}

// $4014 OAM DMA — copy 256 bytes from page value<<8 in CPU RAM
extern uint8_t read_mem(uint16_t addr); // declare

void ppu_oam_dma(uint8_t page) {
    uint16_t base = ((uint16_t)page) << 8;
    for (int i = 0; i < 256; i++) {
        uint8_t b = read_mem((uint16_t)(base + i)); // must go through CPU bus!
        ppu.oam[(ppu.oam_addr + i) & 0xFF] = b;     // wrap in 256-byte OAM
    }
    // CPU stalls 513 or 514 cycles; OK to approximate for now
}

// begin vblank sets the flag and evaluates NMI
void ppu_begin_vblank(void){ ppu.status |= 0x80; ppu_eval_nmi(); }
void ppu_end_vblank(void){
    ppu.status &= ~0x80;
    ppu_eval_nmi();
    ppu_latch_pre_for_visible();
}
//---------------------------------------------------------------------
// Sample background palettes (4 background palettes, 4 colors each)
// Note: Palette[0][0] is the universal background color.
// For simplicity these are dummy ARGB colors.
uint32_t bg_palettes[4][4] = {
    { 0xFF757575, 0xFF271B8F, 0xFF0000AB, 0xFF47009F },
    { 0xFF757575, 0xFF006400, 0xFF13A1A1, 0xFF000000 },
    { 0xFF757575, 0xFF8B0000, 0xFFB22222, 0xFFFF0000 },
    { 0xFF757575, 0xFF00008B, 0xFF0000CD, 0xFF4169E1 }
};

// Get the base address of the background pattern table based on PPUCTRL
// Currently unused, but kept for potential future use
// static uint16_t get_bg_pattern_table_base() {
//     // Bit 4 of PPUCTRL controls which pattern table is used for backgrounds
//     return (ppu.ctrl & 0x10) ? 0x1000 : 0x0000;
// }

// $2000–$2FFF -> nametable RAM index, obeying mirroring
static inline uint16_t nt_index(uint16_t addr) {
    // collapse 0x2000–0x3EFF to a 4-Nametable window (0..0x0FFF)
    uint16_t off = (addr - 0x2000) & 0x0FFF;  // 4 * 0x400
    uint16_t nt  = (off >> 10) & 3;           // which table 0..3
    uint16_t in  =  off & 0x03FF;             // offset inside table

    switch (cart_get_mirroring()) {
        case MIRROR_HORIZONTAL: // 0,1 share; 2,3 share   (top/bottom)
            return ((nt & 2) ? 0x400 : 0x000) + in;
        case MIRROR_VERTICAL:   // 0,2 share; 1,3 share   (left/right)  <-- SMB
            return ((nt & 1) ? 0x400 : 0x000) + in;
        case MIRROR_SINGLE0:    return in;
        case MIRROR_SINGLE1:    return 0x400 + in;
        case MIRROR_FOUR:       return (nt * 0x400) + in; // needs 4 KiB backing
    }
    return in;
}

// Clear the framebuffer to the universal background color each frame
static inline void clear_framebuffer(uint32_t *fb) {
    uint32_t bg = get_color(ppu_palette[0]); // universal background ($3F00)
    for (int i = 0; i < 256*240; ++i) fb[i] = bg;
}



// (removed old full-frame renderers; replaced by scanline versions)

// ------------------- Scanline-based rendering -------------------
void ppu_begin_frame_render(uint32_t *fb) {
    clear_framebuffer(fb);
    memset(bg_opaque, 0, sizeof(bg_opaque));
}

void ppu_render_scanline_bg(int sy, uint32_t *framebuffer) {
    const int W = 256;
    if (sy < 0 || sy >= 240) return;
    if (!(ppu.mask & 0x08)) return;  // BG disabled

    // Split handling: if we predicted a split and this line is where it begins,
    // apply the queued post-scroll and ctrl for subsequent pixels/lines.
    if (ppu.have_split && ppu.post_scroll_valid && sy == ppu.split_y) {
        ppu.t_pre    = (ppu.t_pre & ~0x041F) | (ppu.t_post & 0x041F);
        ppu.x_pre    = ppu.x_post;
        ppu.ctrl_pre = ppu.ctrl_post;
    }

    uint16_t T         = ppu.t_pre;
    uint8_t  fineX     = ppu.x_pre;
    uint8_t  ctrl_here = ppu.ctrl_pre;

    int coarseX =  T        & 0x1F;
    int coarseY = (T >> 5)  & 0x1F;
    int fineY0  = (T >> 12) & 0x07;
    int ntx0    = (T >> 10) & 1;
    int nty0    = (T >> 11) & 1;

    int scrollX = coarseX * 8 + (fineX & 7);
    int scrollY = coarseY * 8 + fineY0;

    uint16_t bgPT = (ctrl_here & 0x10) ? 0x1000 : 0x0000;

    int wy = scrollY + sy + (nty0 * 240);
    int ty = (wy / 8)   % 30;
    int fy =  wy        & 7;

    for (int sx = 0; sx < W; sx++) {
        if (!(ppu.mask & 0x02) && sx < 8) continue; // left-8 BG mask

        int wx = scrollX + sx + (ntx0 * 256);
        int tx = (wx / 8)   % 32;
        int fx =  wx        & 7;

        int nt_x = (wx / 256) & 1;
        int nt_y = (wy / 240) & 1;
        uint16_t ntBase = 0x2000 + ((nt_y << 1) | nt_x) * 0x400;

        uint8_t  tileIdx = ppu_vram[nt_index(ntBase + ty * 32 + tx)];

        uint16_t tileAddr = bgPT + tileIdx * 16;
        uint8_t  p0 = cart_ppu_read(tileAddr + fy);
        uint8_t  p1 = cart_ppu_read(tileAddr + fy + 8);
        uint8_t  bit = 7 - fx;
        uint8_t  px  = ((p1 >> bit) & 1) << 1 | ((p0 >> bit) & 1);

        // Attribute lookup
        uint16_t atAddr = ntBase + 0x3C0 + ((ty >> 2) * 8) + (tx >> 2);
        uint8_t atByte = ppu_vram[nt_index(atAddr)];
        uint8_t shift  = ((ty & 2) << 1) | (tx & 2);
        uint8_t palSel = (atByte >> shift) & 0x03;

        uint8_t palIndex = (px == 0) ? ppu_palette[0] : ppu_palette[(palSel * 4) + px];
        framebuffer[sy * W + sx] = get_color(palIndex);
        if (px) bg_opaque[sy * 256 + sx] = 1;
    }
}

void ppu_render_scanline_sprites(int sy, uint32_t *fb) {
    if (sy < 0 || sy >= 240) return;
    if (!(ppu.mask & 0x10)) return;   // sprites disabled by PPUMASK

    const bool tall = (ppu.ctrl & 0x20) != 0;
    for (int i = 63; i >= 0; i--) {
        uint8_t y0 = ppu.oam[i*4 + 0];
        if (y0 >= 0xEF) continue;
        int baseY = (int)y0 + 1;
        if (sy < baseY || sy >= baseY + (tall ? 16 : 8)) continue; // not on this line

        uint8_t id = ppu.oam[i*4 + 1];
        uint8_t at = ppu.oam[i*4 + 2];
        uint8_t x0 = ppu.oam[i*4 + 3];

        uint8_t pal   = (at & 0x03);
        bool prio     = (at & 0x20) != 0;
        bool flip_h   = (at & 0x40) != 0;
        bool flip_v   = (at & 0x80) != 0;

        uint16_t base = (ppu.ctrl & 0x08) ? 0x1000 : 0x0000;
        if (tall) { base = (id & 1) ? 0x1000 : 0x0000; id &= 0xFE; }

        int row_in_sprite = sy - baseY;
        for (int half = 0; half < (tall ? 2 : 1); half++) {
            if (row_in_sprite < half*8 || row_in_sprite >= (half+1)*8) continue;
            int row = row_in_sprite - half*8;
            int rr = flip_v ? (7 - row) : row;
            uint16_t tile = id + half;
            uint16_t addr = base + tile * 16;
            uint8_t p0 = cart_ppu_read(addr + rr);
            uint8_t p1 = cart_ppu_read(addr + rr + 8);

            for (int col = 0; col < 8; col++) {
                int bit = flip_h ? col : (7 - col);
                uint8_t px = ((p1 >> bit) & 1) << 1 | ((p0 >> bit) & 1);
                if (!px) continue;

                int sx = x0 + col;
                if (sx < 0 || sx >= 256) continue;
                if (!(ppu.mask & 0x04) && sx < 8) continue;  // left-8 sprite mask
                if (prio && (ppu.mask & 0x08) && bg_opaque[sy*256 + sx]) continue;

                uint8_t idx = ppu_palette[0x10 + pal*4 + px];
                fb[sy*256 + sx] = get_color(idx);
            }
        }
    }
}

// PPU Register Read
uint8_t ppu_read(uint16_t addr) {
    addr &= 0x3FFF;

    // Handle palette mirroring
    if(addr >= 0x3F00 && addr < 0x4000) {
        addr &= 0x1F;
        // Special case: address 0x3F10, 0x3F14, 0x3F18, 0x3F1C mirror 0x3F00
        if((addr & 0x03) == 0) addr &= 0x0F;
        return ppu_palette[addr];
    }

    // Nametables 0x2000–0x3EFF (with mirroring)
    if (addr >= 0x2000 && addr < 0x3F00) {
        return ppu_vram[nt_index(addr)];
    }

   // Pattern tables 0x0000–0x1FFF: cart (CHR-ROM/RAM, banked)
   return cart_ppu_read(addr & 0x1FFF);
}

// PPU Register Write
void ppu_write(uint16_t addr, uint8_t value) {
    addr &= 0x3FFF;

    if(addr >= 0x3F00 && addr < 0x4000) {
        addr &= 0x1F;
        // Special case: address 0x3F10, 0x3F14, 0x3F18, 0x3F1C mirror 0x3F00
        if((addr & 0x03) == 0) addr &= 0x0F;
        ppu_palette[addr] = value;
        return;
    }

    // Pattern tables (CHR). If the cart uses CHR-RAM, writes should stick; with CHR-ROM they
    // are typically ignored. For the test ROMs, allowing writes is fine.
    if (addr < 0x2000) { cart_ppu_write(addr & 0x1FFF, value); return; }

    if(addr >= 0x2000 && addr < 0x3F00) {
        ppu_vram[nt_index(addr)] = value;
        return;
    }

    // 0x3F00–0x3FFF handled above; nothing else is writable here.
}

// Reset PPU to initial state
void ppu_reset(PPU* ppu) {
    ppu->ctrl = 0;
    ppu->mask = 0;
    ppu->status = 0x00;
    ppu->oam_addr = 0;
    ppu->scroll_x = 0;
    ppu->scroll_y = 0;
    ppu->v = 0;
    ppu->t = 0;
    ppu->x = 0;
    ppu->w = 0;
    ppu->ppudata_buffer = 0;
    ppu->open_bus = 0;
    ppu->nmi_out = false;

    // Clear OAM
    for (int i = 0; i < PPU_OAM_SIZE; i++) {
        ppu->oam[i] = 0xFF;
    }

    // Initialize palette RAM with power-up values that match Blargg's NES
    // These are the actual values found in Blargg's test NES at power-up
    static const uint8_t power_up_palette[PPU_PALETTE_SIZE] = {
        0x09, 0x01, 0x00, 0x01, 0x00, 0x02, 0x02, 0x0D,
        0x08, 0x10, 0x08, 0x24, 0x00, 0x00, 0x04, 0x2C,
        0x09, 0x01, 0x34, 0x03, 0x00, 0x04, 0x00, 0x14,
        0x08, 0x3A, 0x00, 0x02, 0x00, 0x20, 0x2C, 0x08
    };
    for (int i = 0; i < PPU_PALETTE_SIZE; i++) {
        ppu_palette[i] = power_up_palette[i];
    }

    // Initialize runtime palette to built-in defaults
    ppu_palette_reset_default();
}

// Convert a pixel value (0-3 or an index into the NES palette) to an ARGB color
uint32_t get_color(uint8_t idx) {
    // Runtime palette system -------------------------------------------------
    // We keep a mutable active palette and optional 1536-byte emphasis tables.
    // Declarations (internal linkage) placed near top of file; define once.
    extern uint32_t ppu__active_palette_base[64];
    extern uint32_t ppu__emphasis_palettes[8][64];
    extern bool     ppu__have_emphasis_tables;

    idx &= 0x3F;

    // PPUMASK bit 0: grayscale
    if (ppu.mask & 0x01) idx &= 0x30;

    // If we have emphasis tables, pick the one matching PPUMASK emphasis bits.
    if (ppu__have_emphasis_tables) {
        int e = ((ppu.mask & 0x20) ? 1 : 0)
              | ((ppu.mask & 0x40) ? 2 : 0)
              | ((ppu.mask & 0x80) ? 4 : 0);
        return ppu__emphasis_palettes[e][idx];
    }

    // Otherwise, use the active base palette and apply software emphasis.
    uint32_t c = ppu__active_palette_base[idx];
    float r = (float)((c >> 16) & 0xFF);
    float g = (float)((c >>  8) & 0xFF);
    float b = (float)((c      ) & 0xFF);

    const float ATTEN = 0.60f;
    if (ppu.mask & 0x20) { g *= ATTEN; b *= ATTEN; }  // emphasize RED
    if (ppu.mask & 0x40) { r *= ATTEN; b *= ATTEN; }  // emphasize GREEN
    if (ppu.mask & 0x80) { r *= ATTEN; g *= ATTEN; }  // emphasize BLUE

    int R = (int)(r < 0 ? 0 : (r > 255 ? 255 : r));
    int G = (int)(g < 0 ? 0 : (g > 255 ? 255 : g));
    int B = (int)(b < 0 ? 0 : (b > 255 ? 255 : b));
    return 0xFF000000u | (R << 16) | (G << 8) | B;
}

// -------------------------------------------------------------------------
// Palette tool APIs

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

    // Print first few bytes of CHR-ROM
    printf("First 16 bytes of CHR-ROM:\n");
    for (int i = 0; i < 16; i++) {
        printf("%02X ", chr_rom[i]);
    }
    printf("\n\n");

    // Print pixel values for first few tiles
    printf("Pixel values for first 3 tiles:\n");
    for (int tile = 0; tile < 3; tile++) {
        printf("Tile %d:\n", tile);
        int tileOffset = tile * 16;
        for (int row = 0; row < 8; row++) {
            uint8_t plane0 = chr_rom[tileOffset + row];
            uint8_t plane1 = chr_rom[tileOffset + row + 8];
            for (int col = 0; col < 8; col++) {
                uint8_t bit0 = (plane0 >> (7 - col)) & 1;
                uint8_t bit1 = (plane1 >> (7 - col)) & 1;
                uint8_t pixelValue = (bit1 << 1) | bit0;
                printf("%d ", pixelValue);
            }
            printf("\n");
        }
        printf("\n");
    }

    // Print palette colors
    printf("NES Palette Colors:\n");
    for (int i = 0; i < 16; i++) {  // Print first 16 colors
        printf("Color %02d: 0x%08X\n", i, nes_palette[i]);
    }
}

void start_frame(void) {
    ppu.v = ppu.t;

    ppu.status &= ~0x40;
    ppu.sprite_zero_hit = false;

    ppu.have_split = false;
    ppu.post_scroll_valid = false;
    ppu.split_y = 240;
    
    // Reset post-hit write tracking
    ppu.wrote_2000_post = false;
    ppu.wrote_2005_x_post = false;
    ppu.wrote_2005_y_post = false;


    ppu.t_post    = ppu.t;
    ppu.x_post    = ppu.x;
    ppu.ctrl_post = ppu.ctrl;
}

// Predict the first scanline after the sprite-0 collision for this frame,
// using the *top-of-frame* scroll/nametable (pre state).
void ppu_predict_sprite0_split_for_frame(void) {
    ppu.status &= ~0x40;
    ppu.sprite_zero_hit = false;
    ppu.have_split = false;
    ppu.split_y = 240;
    ppu.split_x = 0;
    ppu.split_cpu_cycles = -1;
    // Need both BG and sprites visible for a hit to matter
    if (!(ppu.mask & 0x08) || !(ppu.mask & 0x10)) return;

    // Sprite #0 from OAM
    uint8_t y0 = ppu.oam[0];
    if (y0 >= 0xEF) return;                  // offscreen
    int baseY    = (int)y0 + 1;
    uint8_t id   = ppu.oam[1];
    uint8_t at   = ppu.oam[2];
    uint8_t x0   = ppu.oam[3];
    bool flip_h  = (at & 0x40) != 0;
    bool flip_v  = (at & 0x80) != 0;
    bool tall    = (ppu.ctrl & 0x20) != 0;

    // Pre (top) scroll/ctrl for this frame
    uint16_t T   = ppu.t;
    uint8_t  fxX = ppu.x & 7;
    uint8_t  C   = ppu.ctrl;

    int coarseX =  T        & 0x1F;
    int coarseY = (T >> 5)  & 0x1F;
    int fineY0  = (T >> 12) & 0x07;
    int ntx0    = (T >> 10) & 1;
    int nty0    = (T >> 11) & 1;

    int scrollX = coarseX * 8 + fxX;
    int scrollY = coarseY * 8 + fineY0;

    uint16_t bgPT = (C & 0x10) ? 0x1000 : 0x0000;
    uint16_t spBase;
    if (tall) spBase = (id & 1) ? 0x1000 : 0x0000;
    else      spBase = (C & 0x08) ? 0x1000 : 0x0000;

    // Scan sprite 0 coverage and check the BG pixel beneath it
    for (int half = 0; half < (tall ? 2 : 1); ++half) {
        uint16_t tile = tall ? ((id & 0xFE) + half) : id;
        uint16_t addr = spBase + tile * 16;

        for (int row = 0; row < 8; ++row) {
            int sy = baseY + row + (half * 8);
            if (sy < 0 || sy >= 240) continue;

            int rr  = flip_v ? (7 - row) : row;
            uint8_t p0 = cart_ppu_read(addr + rr);
            uint8_t p1 = cart_ppu_read(addr + rr + 8);

            for (int col = 0; col < 8; ++col) {
                int sx = x0 + col;
                if (sx >= 256) break;
                if (!(ppu.mask & 0x04) && sx < 8) continue;   // left-8 sprite mask
                if (!((ppu.mask & 0x10) && (sx >= 8 || (ppu.mask & 0x04)))) continue;

                int bit = flip_h ? col : (7 - col);
                uint8_t sp_px = ((p1 >> bit) & 1) << 1 | ((p0 >> bit) & 1);
                if (!sp_px) continue;

                if (!((ppu.mask & 0x08) && (sx >= 8 || (ppu.mask & 0x02)))) continue; // BG visibility

                // BG pixel at (sx, sy) with *pre* scroll
                int wx = scrollX + sx + (ntx0 * 256);
                int wy = scrollY + sy + (nty0 * 240);
                int tx = (wx / 8) % 32, ty = (wy / 8) % 30;
                int fx = wx & 7,       fy = wy & 7;

                int nt_x = (wx / 256) & 1;
                int nt_y = (wy / 240) & 1;
                uint16_t ntBase = 0x2000 + ((nt_y << 1) | nt_x) * 0x400;

                uint8_t  tileIdx = ppu_vram[nt_index(ntBase + ty * 32 + tx)];

                uint16_t tileAddr = bgPT + tileIdx * 16;
                uint8_t  b0 = cart_ppu_read(tileAddr + fy);
                uint8_t  b1 = cart_ppu_read(tileAddr + fy + 8);
                uint8_t  bg_px = (((b1 >> (7 - fx)) & 1) << 1) | ((b0 >> (7 - fx)) & 1);

                if (bg_px) {
                    ppu.have_split = true;
                    ppu.split_y = sy + 1;
                    ppu.split_x = sx;
                    // schedule time of the hit in CPU cycles
                    const double CPU_PER_SCANLINE = CPU_CYCLES_PER_FRAME / 262.0;   // ~113.667
                    const double CPU_PER_DOT      = CPU_PER_SCANLINE / 341.0;       // ~0.333
                    ppu.split_cpu_cycles = (int)(sy * CPU_PER_SCANLINE + sx * CPU_PER_DOT);
                    // don't set status bit-6 here
                    return;
                }
            }
        }
    }
}
