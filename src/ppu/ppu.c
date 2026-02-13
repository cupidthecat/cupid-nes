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

#ifdef VBL_TIMING_LOG
static uint32_t vbl_log_count = 0;
static const uint32_t vbl_log_limit = 4000;
#define VBL_LOG(fmt, ...) do { \
    if (vbl_log_count < vbl_log_limit) { \
        fprintf(stderr, "[VBL cpu=%llu sl=%d dot=%d] " fmt "\n", \
                (unsigned long long)cpu_total_cycles, ppu.scanline, ppu.dot, ##__VA_ARGS__); \
        vbl_log_count++; \
    } \
} while (0)
#else
#define VBL_LOG(...) do {} while (0)
#endif

#ifdef PPU_DEBUG_LOG
static uint32_t ppu_dbg_log_count = 0;
static const uint32_t ppu_dbg_log_limit = 25000;
static uint64_t ppu_dbg_frame_counter = 0;
static uint32_t ppu_reg_log_count = 0;
static const uint32_t ppu_reg_log_limit = 3000;
#define PPU_LOG(fmt, ...) do { \
    if (ppu_dbg_log_count < ppu_dbg_log_limit) { \
        fprintf(stderr, "[PPU f=%llu cpu=%llu sl=%d dot=%d] " fmt "\n", \
                (unsigned long long)ppu_dbg_frame_counter, \
                (unsigned long long)cpu_total_cycles, ppu.scanline, ppu.dot, ##__VA_ARGS__); \
        ppu_dbg_log_count++; \
    } \
} while (0)
#define PPU_REG_LOG(fmt, ...) do { \
    if (ppu_reg_log_count < ppu_reg_log_limit) { \
        fprintf(stderr, "[PPU-REG f=%llu cpu=%llu sl=%d dot=%d] " fmt "\n", \
                (unsigned long long)ppu_dbg_frame_counter, \
                (unsigned long long)cpu_total_cycles, ppu.scanline, ppu.dot, ##__VA_ARGS__); \
        ppu_reg_log_count++; \
    } \
} while (0)
#else
#define PPU_LOG(...) do {} while (0)
#define PPU_REG_LOG(...) do {} while (0)
#endif

// 1 second in CPU cycles (NTSC ~1.789773 MHz)
#define OPEN_BUS_DECAY_CPU_CYCLES 1789773ULL

// Per-bit expiry times (in CPU cycles). If now >= expire, that bit has decayed to 0.
static uint64_t ppu_ob_expire[8];
static bool ppu_step_active = false;
static int ppu_step_subcycle = 0;
static int ppu_step_total_subcycles = 0;
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

static int ppu_count_opaque_pixels(void) {
    int count = 0;
    for (int i = 0; i < 256 * 240; ++i) {
        if (bg_opaque[i]) count++;
    }
    return count;
}

static inline void ppu_eval_nmi(void){
    bool want = (ppu.ctrl & 0x80) && (ppu.status & 0x80);
    if (want && !ppu.nmi_out) {
        uint8_t defer_boundaries = 0;
        if (ppu_step_active) {
            int remaining = ppu_step_total_subcycles - 1 - ppu_step_subcycle;
            // Late edge sampling: the later the edge in the current instruction,
            // the more likely visibility slips by one additional boundary.
            if (remaining < 3)      defer_boundaries = 2; // extremely late
            else if (remaining < 12) defer_boundaries = 1; // late
        }
        cpu_request_nmi_timed_defer(defer_boundaries);
    }
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
            if (ppu.status & 0x80) VBL_LOG("$2002 read clears VBL");
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
                set_open_bus(fetched); // PPUDATA read drives PPU bus with newly fetched byte
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
            PPU_REG_LOG("write $2000=%02X", value);
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
            PPU_REG_LOG("write $2001=%02X (bg=%d sp=%d)", value,
                        (value & 0x08) ? 1 : 0, (value & 0x10) ? 1 : 0);
            break;
        case 3: // OAMADDR
            ppu.oam_addr = value;
            break;
        case 4: // OAMDATA
            ppu.oam[ppu.oam_addr++] = value;
            break;
        case 5: {
            PPU_REG_LOG("write $2005=%02X w=%d", value, ppu.w);
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
            PPU_REG_LOG("write $2006=%02X w=%d", value, ppu.w);
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
    
    // DMA reads through the CPU bus - this includes RAM, ROM, PPU registers, etc.
    // All reads trigger their normal side effects (e.g., PPU register reads increment PPUADDR)
    for (int i = 0; i < 256; i++) {
        uint8_t b = read_mem((uint16_t)(base + i)); // must go through CPU bus with side effects!
        ppu.oam[(ppu.oam_addr + i) & 0xFF] = b;     // wrap in 256-byte OAM
    }
    // CPU stalls 513 or 514 cycles depending on alignment.
    cpu_schedule_oam_dma_stall();
}

// begin vblank sets the flag and evaluates NMI
void ppu_begin_vblank(void){
    ppu.status |= 0x80;
    VBL_LOG("VBL set");
    ppu_eval_nmi();
}
void ppu_end_vblank(void){
    if (ppu.status & 0x80) VBL_LOG("VBL clear (pre-render)");
    ppu.status &= ~0x80;
    ppu_eval_nmi();
    ppu_latch_pre_for_visible();
}
// Clear the framebuffer to the universal background color each frame
static inline void clear_framebuffer(uint32_t *fb) {
    uint32_t bg = get_color(ppu_palette[0]); // universal background ($3F00)
    for (int i = 0; i < 256*240; ++i) fb[i] = bg;
}

// ------------------- Line-based rendering (called at end of each line) -------------------
void ppu_begin_frame_render(uint32_t *fb) {
    clear_framebuffer(fb);
    memset(bg_opaque, 0, sizeof(bg_opaque));
    PPU_LOG("begin_frame_render ctrl=%02X mask=%02X status=%02X v=%04X t=%04X x=%u mir=%d",
            ppu.ctrl, ppu.mask, ppu.status, ppu.v, ppu.t, ppu.x, (int)cart_get_mirroring());
}

// Cycle-stepped sprite evaluation
static inline void ppu_evaluate_sprites(void) {
	// Build the entire secondary OAM at the start of the scanline (dot 1)
	if (ppu.dot != 1) return;

	ppu.sprite_count = 0;
	ppu.sprite_zero_on_line = false;
	ppu.status &= ~0x20; // Clear sprite overflow flag
	memset(ppu.secondary_oam, 0xFF, 32);

	// Evaluate all 64 sprites now so the set is stable for the whole line
	for (int sprite_idx = 0; sprite_idx < 64; ++sprite_idx) {
		uint8_t y0 = ppu.oam[sprite_idx * 4 + 0];
		int baseY = (int)y0 + 1;
		
		// Check if sprite is on current scanline
		bool tall = (ppu.ctrl & 0x20) != 0;
		int sprite_height = tall ? 16 : 8;
		
		if (ppu.scanline >= baseY && ppu.scanline < baseY + sprite_height) {
			// Copy entire sprite to secondary OAM if we have room
			if (ppu.sprite_count < 8) {
				ppu.secondary_oam[ppu.sprite_count * 4 + 0] = y0; // Y (store original Y coordinate)
				ppu.secondary_oam[ppu.sprite_count * 4 + 1] = ppu.oam[sprite_idx * 4 + 1]; // ID
				ppu.secondary_oam[ppu.sprite_count * 4 + 2] = ppu.oam[sprite_idx * 4 + 2]; // Attr
				ppu.secondary_oam[ppu.sprite_count * 4 + 3] = ppu.oam[sprite_idx * 4 + 3]; // X
				
				// Track if sprite 0 is on this scanline
				if (sprite_idx == 0) {
					ppu.sprite_zero_on_line = true;
				}
				
				ppu.sprite_count++;
			} else {
				// Sprite overflow: set flag when more than 8 sprites are found
				ppu.status |= 0x20; // Set sprite overflow flag
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
        return cart_nt_read(addr, ppu_vram);
    }

    // Pattern tables 0x0000–0x1FFF: cart (CHR-ROM/RAM, banked)
    cart_set_ppu_fetch_source(CART_PPU_FETCH_CPU);
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
    if (addr < 0x2000) {
        cart_set_ppu_fetch_source(CART_PPU_FETCH_CPU);
        cart_ppu_write(addr & 0x1FFF, value);
        return;
    }

    if(addr >= 0x2000 && addr < 0x3F00) {
        cart_nt_write(addr, value, ppu_vram);
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

    // Timing state
    ppu->scanline = 261;    // pre-render
    ppu->dot = 0;
    ppu->odd_frame = false;
    ppu->frame_complete = false;
    
    // Background tile fetch pipeline
    ppu->nt_byte = 0;
    ppu->at_byte = 0;
    ppu->pt_lo = 0;
    ppu->pt_hi = 0;
    ppu->bg_shift_lo = 0;
    ppu->bg_shift_hi = 0;
    ppu->at_shift_lo = 0;
    ppu->at_shift_hi = 0;
    ppu->at_latch_lo = 0;
    ppu->at_latch_hi = 0;
    ppu_dbg_frame_counter = 0;
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
    // Note: Don't copy t→v here! The real PPU does this during the pre-render scanline:
    // - Horizontal bits: copied at dot 257 (via ppu_copy_horizontal)
    // - Vertical bits: copied at dots 280-304 (via ppu_copy_vertical)
    // Both are handled in ppu_step() when rendering is enabled.

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

    // New frame bookkeeping
    ppu.frame_complete = false;
    ppu.odd_frame = !ppu.odd_frame;
        ppu_dbg_frame_counter++;
        PPU_LOG("start_frame odd=%d ctrl=%02X mask=%02X status=%02X v=%04X t=%04X x=%u",
            ppu.odd_frame ? 1 : 0, ppu.ctrl, ppu.mask, ppu.status, ppu.v, ppu.t, ppu.x);

    // Predict split for this frame based on pre state
    ppu_predict_sprite0_split_for_frame();
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
    int baseY = (int)y0 + 1;
    uint8_t id = ppu.oam[1];
    uint8_t at = ppu.oam[2];
    uint8_t x0 = ppu.oam[3];
    bool flip_h = (at & 0x40) != 0;
    bool flip_v = (at & 0x80) != 0;
    bool tall = (ppu.ctrl & 0x20) != 0;

    // Pre (top) scroll/ctrl for this frame
    uint16_t T = ppu.t;
    uint8_t fxX = ppu.x & 7;
    uint8_t C = ppu.ctrl;

    int coarseX = T & 0x1F;
    int coarseY = (T >> 5) & 0x1F;
    int fineY0 = (T >> 12) & 0x07;
    int ntx0 = (T >> 10) & 1;
    int nty0 = (T >> 11) & 1;

    int scrollX = coarseX * 8 + fxX;
    int scrollY = coarseY * 8 + fineY0;

    uint16_t bgPT = (C & 0x10) ? 0x1000 : 0x0000;
    uint16_t spBase;
    if (tall) spBase = (id & 1) ? 0x1000 : 0x0000;
    else spBase = (C & 0x08) ? 0x1000 : 0x0000;

    // Simulate cycle-stepped rendering for sprite 0 prediction
    for (int sy = baseY; sy < baseY + (tall ? 16 : 8) && sy < 240; sy++) {
        // Simulate sprite evaluation for this scanline
        bool sprite_on_line = false;
        for (int half = 0; half < (tall ? 2 : 1); half++) {
            int row_in_sprite = sy - baseY;
            if (row_in_sprite >= half * 8 && row_in_sprite < (half + 1) * 8) {
                // Account for vertical flip and 8x16 selection
                int rrFull = flip_v ? ((tall ? 15 : 7) - row_in_sprite) : row_in_sprite;
                int use_half = tall ? (rrFull >> 3) : 0;
                int rr = rrFull & 7;

                uint16_t tile = tall ? ((id & 0xFE) + use_half) : id;
                uint16_t addr = spBase + tile * 16;
                cart_set_ppu_fetch_source(CART_PPU_FETCH_SPRITE);
                uint8_t p0 = cart_ppu_read(addr + rr);
                uint8_t p1 = cart_ppu_read(addr + rr + 8);

                // Check if sprite has any pixels on this line
                for (int col = 0; col < 8; col++) {
                    int bit = flip_h ? col : (7 - col);
                    uint8_t sp_px = ((p1 >> bit) & 1) << 1 | ((p0 >> bit) & 1);
                    if (sp_px) {
                        sprite_on_line = true;
                        break;
                    }
                }
                if (sprite_on_line) break;
            }
        }
        
        if (!sprite_on_line) continue;
        
        // Simulate cycle-stepped rendering for this scanline
        for (int dot = 1; dot <= 256; dot++) {
            int sx = dot - 1; // pixel X coordinate
            
            // Check if sprite 0 covers this pixel
            bool sprite_pixel = false;
            for (int half = 0; half < (tall ? 2 : 1); half++) {
                int row_in_sprite = sy - baseY;
                if (row_in_sprite >= half * 8 && row_in_sprite < (half + 1) * 8) {
                    int rrFull = flip_v ? ((tall ? 15 : 7) - row_in_sprite) : row_in_sprite;
                    int use_half = tall ? (rrFull >> 3) : 0;
                    int rr = rrFull & 7;

                    uint16_t tile = tall ? ((id & 0xFE) + use_half) : id;
                    uint16_t addr = spBase + tile * 16;
                    cart_set_ppu_fetch_source(CART_PPU_FETCH_SPRITE);
                    uint8_t p0 = cart_ppu_read(addr + rr);
                    uint8_t p1 = cart_ppu_read(addr + rr + 8);

                    if (sx >= x0 && sx < x0 + 8) {
                        int col = sx - x0;
                        int bit = flip_h ? col : (7 - col);
                        uint8_t sp_px = ((p1 >> bit) & 1) << 1 | ((p0 >> bit) & 1);
                        if (sp_px) {
                            sprite_pixel = true;
                            break;
                        }
                    }
                }
            }
            
            if (!sprite_pixel) continue;
            if (sx == 255) continue; // Never hit at X=255
            if (!(ppu.mask & 0x04) && sx < 8) continue; // left-8 sprite mask
            
            // Check background pixel at this position
            int wx = scrollX + sx + (ntx0 * 256);
            int wy = scrollY + sy + (nty0 * 240);
            int tx = (wx / 8) % 32, ty = (wy / 8) % 30;
            int fx = wx & 7, fy = wy & 7;
            
            int nt_x = (wx / 256) & 1;
            int nt_y = (wy / 240) & 1;
            uint16_t ntBase = 0x2000 + ((nt_y << 1) | nt_x) * 0x400;
            
            uint8_t tileIdx = cart_nt_read((uint16_t)(ntBase + ty * 32 + tx), ppu_vram);
            uint16_t tileAddr = bgPT + tileIdx * 16;
            cart_set_ppu_fetch_source((ppu.ctrl & 0x20) ? CART_PPU_FETCH_BG : CART_PPU_FETCH_SPRITE);
            uint8_t b0 = cart_ppu_read(tileAddr + fy);
            uint8_t b1 = cart_ppu_read(tileAddr + fy + 8);
            uint8_t bg_px = (((b1 >> (7 - fx)) & 1) << 1) | ((b0 >> (7 - fx)) & 1);
            
            if (bg_px) {
                ppu.have_split = true;
                ppu.split_y = sy + 1;
                ppu.split_x = sx;
                // schedule time of the hit in CPU cycles
                const double CPU_PER_SCANLINE = CPU_CYCLES_PER_FRAME / 262.0;   // ~113.667
                const double CPU_PER_DOT = CPU_PER_SCANLINE / 341.0;       // ~0.333c
                ppu.split_cpu_cycles = (int)(sy * CPU_PER_SCANLINE + sx * CPU_PER_DOT);
                return;
            }
        }
    }
}

// ------------------- Cycle-stepped PPU -------------------
static inline void ppu_increment_x(void) {
    if ((ppu.v & 0x001F) == 31) {
        ppu.v &= ~0x001F;
        ppu.v ^= 0x0400; // switch horizontal nametable
    } else {
        ppu.v += 1;
    }
}

static inline void ppu_increment_y(void) {
    if ((ppu.v & 0x7000) != 0x7000) {
        ppu.v += 0x1000; // fine Y
    } else {
        ppu.v &= ~0x7000;
        int y = (ppu.v & 0x03E0) >> 5; // coarse Y
        if (y == 29) {
            y = 0;
            ppu.v ^= 0x0800; // switch vertical nametable
        } else if (y == 31) {
            y = 0; // overflow to attribute row
        } else {
            y += 1;
        }
        ppu.v = (ppu.v & ~0x03E0) | (y << 5);
    }
}

static inline void ppu_copy_horizontal(void) { ppu.v = (ppu.v & ~0x041F) | (ppu.t & 0x041F); }
static inline void ppu_copy_vertical(void)   { ppu.v = (ppu.v & ~0x7BE0) | (ppu.t & 0x7BE0); }

// Background pixel fetch and rendering
static inline void ppu_fetch_nt_byte(void) {
    uint16_t addr = 0x2000 | (ppu.v & 0x0FFF);
    ppu.nt_byte = ppu_read(addr);
}

static inline void ppu_fetch_at_byte(void) {
    uint16_t v = ppu.v;
    uint16_t addr = 0x23C0 | (v & 0x0C00) | ((v >> 4) & 0x38) | ((v >> 2) & 0x07);
    uint8_t at = ppu_read(addr);
    uint8_t shift = ((v >> 4) & 0x04) | (v & 0x02);
    ppu.at_byte = (at >> shift) & 0x03;
}

static inline void ppu_fetch_pt_lo(void) {
    uint16_t base = (ppu.ctrl & 0x10) ? 0x1000 : 0x0000;
    uint16_t fine_y = (ppu.v >> 12) & 0x07;
    uint16_t addr = base + ppu.nt_byte * 16 + fine_y;
    cart_set_ppu_fetch_source((ppu.ctrl & 0x20) ? CART_PPU_FETCH_BG : CART_PPU_FETCH_SPRITE);
    ppu.pt_lo = cart_ppu_read(addr);
}

static inline void ppu_fetch_pt_hi(void) {
    uint16_t base = (ppu.ctrl & 0x10) ? 0x1000 : 0x0000;
    uint16_t fine_y = (ppu.v >> 12) & 0x07;
    uint16_t addr = base + ppu.nt_byte * 16 + fine_y + 8;
    cart_set_ppu_fetch_source((ppu.ctrl & 0x20) ? CART_PPU_FETCH_BG : CART_PPU_FETCH_SPRITE);
    ppu.pt_hi = cart_ppu_read(addr);
}

static inline void ppu_reload_shifters(void) {
    // Reload low 8 bits with new tile data
    ppu.bg_shift_lo = (ppu.bg_shift_lo & 0xFF00) | ppu.pt_lo;
    ppu.bg_shift_hi = (ppu.bg_shift_hi & 0xFF00) | ppu.pt_hi;
    
    // Reload attribute latches
    ppu.at_latch_lo = (ppu.at_byte & 1) ? 0xFF : 0x00;
    ppu.at_latch_hi = (ppu.at_byte & 2) ? 0xFF : 0x00;
}

static inline void ppu_shift_bg(void) {
    if (ppu.mask & 0x08) { // BG enabled
        ppu.bg_shift_lo <<= 1;
        ppu.bg_shift_hi <<= 1;
        ppu.at_shift_lo <<= 1;
        ppu.at_shift_hi <<= 1;
    }
}

// Render a single pixel (background + sprites)
static inline void ppu_render_dot(uint32_t *fb, int sl, int dotx) {
    if (sl < 0 || sl >= 240) return;
    if (dotx < 0 || dotx >= 256) return;

    uint32_t color = get_color(ppu_palette[0]); // default to background color
    uint8_t bg_pixel = 0;
    uint8_t bg_palette = 0;
    uint8_t sp_pixel = 0;
    uint8_t sp_palette = 0;
    bool sp_priority = false;

    // Background rendering
    if (ppu.mask & 0x08) { // BG enabled
        if (dotx >= 8 || (ppu.mask & 0x02)) { // not masked by left-8 clip
            // Select bit based on fine X scroll
            uint16_t mux = 0x8000 >> ppu.x;
            uint8_t p0 = (ppu.bg_shift_lo & mux) ? 1 : 0;
            uint8_t p1 = (ppu.bg_shift_hi & mux) ? 1 : 0;
            bg_pixel = (p1 << 1) | p0;
            
            uint8_t a0 = (ppu.at_shift_lo & mux) ? 1 : 0;
            uint8_t a1 = (ppu.at_shift_hi & mux) ? 1 : 0;
            bg_palette = (a1 << 1) | a0;
        }
    }

    // --- Sprite rendering (per-pixel), left-8 clip handled AFTER sampling pixel ---
    if (ppu.mask & 0x10) { // Sprites enabled
        // Compute whether this *pixel* is clipped by left-8 mask
        const bool left_clip_sp = !(ppu.mask & 0x04) && (dotx < 8);

        // Walk secondary OAM in order; first non-transparent pixel wins
        for (int i = 0; i < ppu.sprite_count; i++) {
            uint8_t y0  = ppu.secondary_oam[i * 4 + 0];
            uint8_t id  = ppu.secondary_oam[i * 4 + 1];
            uint8_t at  = ppu.secondary_oam[i * 4 + 2];
            uint8_t x0u = ppu.secondary_oam[i * 4 + 3];

            // Use signed math so we never wrap at 255 and we handle x0 near edges sanely
            int x0 = (int)x0u;

            // Column inside the sprite for this screen pixel
            int col = dotx - x0;
            if (col < 0 || col >= 8) continue; // not over this sprite this dot

            // Vertical row inside the sprite
            bool tall    = (ppu.ctrl & 0x20) != 0;
            bool flip_h  = (at & 0x40) != 0;
            bool flip_v  = (at & 0x80) != 0;
            bool behind  = (at & 0x20) != 0;

            // NES sprites appear 1 scanline after their Y coordinate
            // Y=0 appears on scanline 1, Y=239 appears on scanline 240 (offscreen)
            int baseY = (int)y0 + 1;
            int row_in_sprite = sl - baseY;
            
            // Allow sprites to render partially offscreen at bottom for smooth scrolling
            if (row_in_sprite < 0) continue;
            if (row_in_sprite >= (tall ? 16 : 8)) continue;

            // Resolve row/half & tile address
            int rrFull = flip_v ? ((tall ? 15 : 7) - row_in_sprite) : row_in_sprite;
            int half   = tall ? (rrFull >> 3) : 0;
            int rr     = rrFull & 7;

            uint16_t base = (ppu.ctrl & 0x08) ? 0x1000 : 0x0000; // sprite PT (bit 3)
            uint16_t tile = id;
            if (tall) { base = (id & 1) ? 0x1000 : 0x0000; tile = (id & 0xFE) + half; }

            uint16_t addr = base + tile * 16;
            cart_set_ppu_fetch_source(CART_PPU_FETCH_SPRITE);
            uint8_t p0 = cart_ppu_read(addr + rr);
            uint8_t p1 = cart_ppu_read(addr + rr + 8);

            int bit = flip_h ? col : (7 - col);
            uint8_t px = ((p1 >> bit) & 1) << 1 | ((p0 >> bit) & 1);

            // Left-8 clip masks the PIXEL (not the whole sprite) after we know it's non-zero
            if (left_clip_sp) px = 0;

            if (px) {
                sp_pixel   = px;
                sp_palette = at & 0x03;
                sp_priority = behind;

                // Sprite 0 hit (suppressed in left 8, and never at X=255)
                if (ppu.sprite_zero_on_line && i == 0 && bg_pixel && dotx != 255 && !(ppu.status & 0x80)) {
                    // If left_clip_sp were true, px would be 0 above, so no hit here
                    ppu.status |= 0x40;
                    ppu.sprite_zero_hit = true;
                }
                break; // first sprite with a visible pixel wins
            }
        }
    }

    // Final pixel composition
    if (bg_pixel) {
        bg_opaque[sl * 256 + dotx] = 1;
    } else {
        bg_opaque[sl * 256 + dotx] = 0;
    }

    // Priority: sprite behind background if priority bit is set
    if (sp_pixel && (!sp_priority || !bg_pixel)) {
        uint8_t pal_addr = 0x10 + sp_palette * 4 + sp_pixel;
        color = get_color(ppu_palette[pal_addr]);
    } else if (bg_pixel) {
        uint8_t pal_addr = bg_palette * 4 + bg_pixel;
        color = get_color(ppu_palette[pal_addr]);
    }

    fb[sl * 256 + dotx] = color;
}

void ppu_step(int cpu_cycles) {
    // Advance PPU 3x the CPU cycles
    ppu_step_active = true;
    ppu_step_total_subcycles = cpu_cycles * 3;
    for (int c = 0; c < cpu_cycles * 3; ++c) {
        ppu_step_subcycle = c;
        int sl = ppu.scanline;
        int dot = ppu.dot; // 0..340
        bool rendering = (ppu.mask & 0x18) != 0;
        bool visible_line = (sl >= 0 && sl < 240) || (sl == 261);

        // Odd frame skipped dot (NTSC) at pre-render if rendering enabled
        if (sl == 261 && dot == 0 && ppu.odd_frame && rendering) {
            ppu.dot = 1;
            continue;
        }

        // Sprite evaluation BEFORE rendering (build once at dot 1 on visible scanlines)
        // Build secondary OAM regardless of current sprite enable; rendering still honors PPUMASK
        if (sl >= 0 && sl < 240 && dot == 1) {
            ppu_evaluate_sprites();
            if ((sl % 60) == 0) {
                PPU_LOG("scanline=%d render=%d sprite_count=%d sprite0_line=%d ctrl=%02X mask=%02X v=%04X",
                        sl, rendering ? 1 : 0, ppu.sprite_count,
                        ppu.sprite_zero_on_line ? 1 : 0, ppu.ctrl, ppu.mask, ppu.v);
            }
        }

        // Render visible pixels (dots 1-256 on scanlines 0-239)
        if (sl >= 0 && sl < 240 && dot >= 1 && dot <= 256) {
            ppu_render_dot(framebuffer, sl, dot - 1);
        }

        // Background tile fetching and shifting (visible + pre-render lines)
        if (visible_line && rendering) {
            // Tile fetch cycles (every 8 dots)
            if ((dot >= 1 && dot <= 256) || (dot >= 321 && dot <= 336)) {
                switch ((dot - 1) % 8) {
                    case 1: ppu_fetch_nt_byte(); break;
                    case 3: ppu_fetch_at_byte(); break;
                    case 5: ppu_fetch_pt_lo(); break;
                    case 7: 
                        ppu_fetch_pt_hi();
                        
                        if (dot <= 256) {
                            // ✅ increment coarse X happens at dot 8
                            ppu_increment_x();
                        } else if (dot >= 321) {
                            // Pre-fetch for next scanline: load into HIGH byte first, then LOW byte
                            if (dot == 328) {
                                // First tile: load into high byte
                                ppu.bg_shift_lo = (uint16_t)(ppu.pt_lo << 8);
                                ppu.bg_shift_hi = (uint16_t)(ppu.pt_hi << 8);
                                ppu.at_shift_lo = (uint16_t)((ppu.at_byte & 1) ? 0xFF00 : 0x0000);
                                ppu.at_shift_hi = (uint16_t)((ppu.at_byte & 2) ? 0xFF00 : 0x0000);
                            } else if (dot == 336) {
                                // Second tile: load into low byte
                                ppu.bg_shift_lo |= ppu.pt_lo;
                                ppu.bg_shift_hi |= ppu.pt_hi;
                                ppu.at_shift_lo |= (ppu.at_byte & 1) ? 0xFF : 0x00;
                                ppu.at_shift_hi |= (ppu.at_byte & 2) ? 0xFF : 0x00;
                            }
                        }
                        break;
                }
            }

            // ⬇️ Do the shifter reload one dot later: dots 9,17,25,... (NOT at 1)
            if (dot >= 9 && dot <= 256 && (dot % 8) == 1) {
                ppu_reload_shifters();
                // also refresh attribute shifter low bytes from latches
                ppu.at_shift_lo = (ppu.at_shift_lo & 0xFF00) | ppu.at_latch_lo;
                ppu.at_shift_hi = (ppu.at_shift_hi & 0xFF00) | ppu.at_latch_hi;
            }

            // Shift AFTER rendering each pixel
            if (dot >= 1 && dot <= 256) {
                ppu_shift_bg();
            }

            // Increment Y at dot 256
            if (dot == 256) {
                ppu_increment_y();
            }
            
            // Increment coarse X for next-scanline tiles (after each pre-fetch)
            if (dot == 328 || dot == 336) {
                ppu_increment_x();
            }
        }

        // Dot-specific actions
        if (sl == 241 && dot == 1) {
            cart_notify_vblank_start();
            // Enter VBlank at the canonical PPU timing point.
            ppu_begin_vblank();
            PPU_LOG("enter_vblank status=%02X nmi_out=%d", ppu.status, ppu.nmi_out ? 1 : 0);
        }

        if (sl == 261) { // pre-render line
            if (dot == 1) {
                ppu_end_vblank();                    // clear VBL and reevaluate NMI output
                ppu.status &= ~(0x40 | 0x20);       // clear sprite0, overflow
                ppu.sprite_zero_hit = false;
                PPU_LOG("end_vblank status=%02X nmi_out=%d", ppu.status, ppu.nmi_out ? 1 : 0);
            }
            if (dot == 257 && rendering) {
                ppu_copy_horizontal();
            }
            if (dot >= 280 && dot <= 304 && rendering) {
                ppu_copy_vertical();
            }
        } else if (sl >= 0 && sl <= 240) { // visible + post-render
            if (dot == 3 && rendering && sl < 240) {
                cart_notify_scanline_early();
            }
            if (dot == 257 && rendering && sl < 240) {
                ppu_copy_horizontal();
            }
            if (dot == 260 && rendering) {
                cart_notify_scanline();
            }
        }

        // Increment dot/scanline with odd frame skip
        ppu.dot++;
        if (ppu.dot == 341) {
            ppu.dot = 0;
            ppu.scanline++;


            if (ppu.scanline == 262) {
                // End of pre-render for next frame; signal frame complete
                ppu.scanline = 0;
                ppu.frame_complete = true;
                PPU_LOG("frame_complete opaque_bg=%d sprite0_hit=%d status=%02X ctrl=%02X mask=%02X",
                        ppu_count_opaque_pixels(), ppu.sprite_zero_hit ? 1 : 0,
                        ppu.status, ppu.ctrl, ppu.mask);
            }
        }
    }
    ppu_step_active = false;
} 