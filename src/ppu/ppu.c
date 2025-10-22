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
#include "../ui/palette_tool.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// External CPU cycle counter
extern uint64_t cpu_total_cycles;

// 1 second in CPU cycles (NTSC ~1.789773 MHz)
#define OPEN_BUS_DECAY_CPU_CYCLES 1789773ULL

// Per-bit expiry times (in CPU cycles). If now >= expire, that bit has decayed to 0.
static uint64_t ppu_ob_expire[8];
// PPU Memory
uint8_t ppu_vram[NT_RAM_SIZE];
uint8_t ppu_palette[PPU_PALETTE_SIZE];
uint8_t bg_opaque[256 * 240];

// PPU Registers
PPU ppu;
extern CPU cpu;

// Helpers ----------------------------------------------------------------
// Helper to recompute current open-bus value from per-bit timers
static inline uint8_t ppu_open_bus_current(void) {
    uint64_t now = cpu_total_cycles;
    uint8_t out = 0;
    for (int b = 0; b < 8; ++b) {
        if (ppu_ob_expire[b] > now) out |= (uint8_t)(1u << b);
    }
    return out;
}

static inline void set_open_bus(uint8_t v) {
    uint64_t now = cpu_total_cycles;
    // Any bit driven high is refreshed for 1 second; zeros are immediately low.
    for (int b = 0; b < 8; ++b) {
        if (v & (1u << b)) ppu_ob_expire[b] = now + OPEN_BUS_DECAY_CPU_CYCLES;
        else               ppu_ob_expire[b] = 0;
    }
    ppu.open_bus = v; // remember the last driven value (not strictly required, but handy)
}

static inline uint8_t get_open_bus(void) {
    uint8_t cur = ppu_open_bus_current();
    ppu.open_bus = cur;   // keep the cached byte in sync with the decayed value
    return cur;
}

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

// Refresh only the bits actually driven by the PPU; leave others untouched.
static inline void set_open_bus_masked(uint8_t v, uint8_t mask) {
    uint64_t now = cpu_total_cycles;
    for (int b = 0; b < 8; ++b) {
        if (mask & (1u << b)) {
            if (v & (1u << b)) ppu_ob_expire[b] = now + OPEN_BUS_DECAY_CPU_CYCLES; // driven high -> refresh
            else               ppu_ob_expire[b] = 0;                                // driven low  -> immediate 0
        }
    }
    // Recompute current value from timers; undriven bits continue decaying.
    ppu.open_bus = ppu_open_bus_current();
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
            set_open_bus_masked(ret, 0xE0); // only bits 7-5 are driven by PPUSTATUS
            ppu.status &= ~0x80;       // clear VBlank
            ppu.w = 0;
            ppu_eval_nmi();            // output may drop
            return ret;
        }
        case 4: { // OAMDATA
            uint8_t v = ppu.oam[ppu.oam_addr];
            if ((ppu.oam_addr & 3) == 2) {   // attribute byte (Y,ID,ATTR,X -> index 2)
                v &= 0xE3;                   // clear bits 2–4 on read
            }
            set_open_bus(v);                 // OAMDATA drives all 8 bits
            return v;
        }
        case 7: { // PPUDATA (read)
            uint16_t addr = ppu.v & 0x3FFF;
            uint8_t ret;
            if (addr >= 0x3F00 && addr < 0x4000) {
                // Palette reads: PPU drives only D0–D5; D6–D7 come from open-bus decay.
                uint8_t ob  = get_open_bus();
                uint8_t pal = ppu_read(addr) & 0x3F;               // palette is 6-bit wide
                ret = pal | (ob & 0xC0);                           // high 2 bits from decay
                ppu.ppudata_buffer = ppu_read(addr - 0x1000);      // buffer gets nametable mirror
                set_open_bus_masked(ret, 0x3F);                    // refresh only D0–D5
            } else {
                // CPU gets buffered old byte; bus sees the newly fetched one
                ret = ppu.ppudata_buffer;
                uint8_t fetched = ppu_read(addr);
                ppu.ppudata_buffer = fetched;
                set_open_bus(ret); // refresh with the value the CPU actually reads
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
        if (y0 >= 0xF0) continue;
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
    
    // Initialize open bus decay timers
    for (int i = 0; i < 8; ++i) ppu_ob_expire[i] = 0;

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
    if (y0 >= 0xF0) return;                  // offscreen
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
                if (sx >= 256) break;        // off right edge
                if (sx == 255) continue;     // PPU never sets sprite-0 hit on dot 255
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