/*
 * ppu.c - Picture Processing Unit (PPU) emulation
 *
 * Author: @frankischilling
 *
 * This file implements PPU register behavior, VRAM access, background and sprite fetches,
 * sprite evaluation, scrolling, palette lookup, pixel rendering, vblank timing, NMI edges,
 * and the PPU bus behavior used by cartridge mappers.
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
#include "../system/timing.h"
#include "../ui/palette_tool.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Each data line retains its charge independently across register reads.
static uint64_t ppu_ob_expire[8];

uint8_t ppu_vram[NT_RAM_SIZE];
uint8_t ppu_palette[PPU_PALETTE_SIZE];
uint8_t bg_opaque[256 * 240];
PPU ppu;

static bool rendering_line(void) {
    return ppu.scanline < 240 || ppu.scanline == (int)nes_timing()->scanlines - 1;
}

static bool rendering_active(void) {
    return rendering_line() && ppu.rendering_enabled;
}

static void ppu_set_bus_address(uint16_t address) {
    ppu.bus_address = address & 0x3FFF;
    cart_notify_ppu_address(ppu.bus_address, ppu.total_cycles);
}

static void ppu_bus_address_phase(uint16_t address) {
    address &= 0x3FFF;
    uint8_t low = (uint8_t)address;
    if (ppu.data_read_delay == 1 && rendering_active()) low = ppu.vram_bus_data;
    ppu.vram_address_latch = low;
    ppu_set_bus_address((address & 0x3F00) | low);
    ppu.bus_ale_this_dot = true;
}

static uint8_t ppu_bus_read_phase(uint16_t par_address, CartPpuFetchSource source) {
    uint16_t address = (par_address & 0x3F00) | ppu.vram_address_latch;
    uint8_t value;
    if (address < 0x2000) {
        cart_set_ppu_fetch_source(source);
        value = cart_ppu_read(address);
    } else {
        value = cart_nt_read(address, ppu_vram);
    }
    ppu.vram_bus_data = value;
    ppu.bus_address = (uint16_t)((address & 0x3F00) | value);
    ppu.bus_read_this_dot = true;
    return value;
}

static uint8_t get_open_bus(void) {
    uint8_t value = 0;
    for (int bit = 0; bit < 8; ++bit)
        if (ppu_ob_expire[bit] > cpu_total_cycles) value |= (uint8_t)(1u << bit);
    ppu.open_bus = value;
    return value;
}

static void set_open_bus_masked(uint8_t value, uint8_t mask) {
    for (int bit = 0; bit < 8; ++bit) {
        if (mask & (1u << bit))
            ppu_ob_expire[bit] = (value & (1u << bit))
                ? cpu_total_cycles + (uint64_t)(nes_timing()->cpu_hz * 4.0 / nes_timing()->fps) : 0;
    }
    get_open_bus();
}

static void set_open_bus(uint8_t value) {
    set_open_bus_masked(value, 0xFF);
}

static void ppu_eval_nmi(void) {
    bool output = (ppu.ctrl & 0x80) && (ppu.status & 0x80);
    cpu_set_nmi_line(output);
    ppu.nmi_out = output;
}

static void ppu_increment_x(void) {
    if ((ppu.v & 0x001F) == 31) {
        ppu.v &= ~0x001F;
        ppu.v ^= 0x0400;
    } else {
        ppu.v++;
    }
}

static void ppu_increment_y(void) {
    if ((ppu.v & 0x7000) != 0x7000) {
        ppu.v += 0x1000;
    } else {
        ppu.v &= ~0x7000;
        int y = (ppu.v >> 5) & 31;
        if (y == 29) {
            y = 0;
            ppu.v ^= 0x0800;
        } else if (y == 31) {
            y = 0;
        } else {
            y++;
        }
        ppu.v = (ppu.v & ~0x03E0) | (y << 5);
    }
}

static void ppu_increment_data_address(void) {
    if (rendering_active()) {
        ppu_increment_x();
        ppu_increment_y();
    } else {
        ppu.v = (ppu.v + ((ppu.ctrl & 0x04) ? 32 : 1)) & 0x7FFF;
        ppu_set_bus_address(ppu.v);
    }
}

static void ppu_increment_secondary_oam(void) {
    if (ppu.secondary_oam_overflowed) return;
    ppu.secondary_index = (uint8_t)((ppu.secondary_index + 1) & 0x1F);
    if (!ppu.secondary_index) ppu.secondary_oam_overflowed = true;
}

static void ppu_corrupt_oam_row(uint8_t row) {
    row &= 0x1F;
    if (row) memcpy(&ppu.oam[row << 3], ppu.oam, 8);
    ppu.secondary_oam[row] = ppu.secondary_oam[0];
}

static uint8_t ppu_oam_corruption_row(int dot) {
    uint8_t row = ppu.secondary_index & 0x1F;
    if (dot >= 65 && dot <= 256 && (row & 3)) row = (uint8_t)((row + 3) & 0x1C);
    return row;
}

// Palette RAM is internal. External reads in its address range still reach
// the cartridge's nametable mirror and refill the CPU read buffer.
static uint8_t ppu_bus_read(uint16_t addr, CartPpuFetchSource source) {
    addr &= 0x3FFF;
    ppu_set_bus_address(addr);
    ppu.vram_address_latch = (uint8_t)addr;
    if (addr < 0x2000) {
        cart_set_ppu_fetch_source(source);
        ppu.vram_bus_data = cart_ppu_read(addr);
        return ppu.vram_bus_data;
    }
    ppu.vram_bus_data = cart_nt_read(addr, ppu_vram);
    return ppu.vram_bus_data;
}

uint8_t ppu_read(uint16_t addr) {
    addr &= 0x3FFF;
    if (addr >= 0x3F00) {
        addr &= 0x1F;
        if ((addr & 3) == 0) addr &= 0x0F;
        return ppu_palette[addr];
    }
    return ppu_bus_read(addr, CART_PPU_FETCH_CPU);
}

void ppu_write(uint16_t addr, uint8_t value) {
    addr &= 0x3FFF;
    if (addr >= 0x3F00) {
        addr &= 0x1F;
        value &= 0x3F;
        if ((addr & 3) == 0) {
            addr &= 0x0F;
            ppu_palette[addr | 0x10] = value;
        }
        ppu_palette[addr] = value;
    } else {
        ppu_set_bus_address(addr);
        if (addr < 0x2000) {
            cart_set_ppu_fetch_source(CART_PPU_FETCH_CPU);
            cart_ppu_write(addr, value);
        } else {
            cart_nt_write(addr, value, ppu_vram);
        }
    }
}

uint8_t ppu_reg_read(uint16_t reg) {
    switch (reg & 7) {
        case 2: {
            uint8_t value = (ppu.status & 0xE0) | (get_open_bus() & 0x1F);
            set_open_bus_masked(value, 0xE0);
            // dot identifies the next clock to execute. Reading immediately
            // before the vblank-set clock suppresses that edge for this frame.
            if (ppu.scanline == (int)nes_timing()->vblank_scanline && ppu.dot == 1)
                ppu.suppress_vblank = true;
            ppu.status &= ~0x80;
            ppu.w = 0;
            ppu_eval_nmi();
            return value;
        }
        case 4: {
            uint8_t value;
            if (rendering_active()) {
                if (ppu.dot == 0 || ppu.dot >= 257)
                    ppu.oam_bus = ppu.secondary_oam[ppu.secondary_index & 0x1F];
                value = ppu.oam_bus;
            } else {
                value = ppu.oam[ppu.oam_addr];
                if ((ppu.oam_addr & 3) == 2) value &= 0xE3;
            }
            set_open_bus(value);
            return value;
        }
        case 7: {
            // The register cannot start another external read until its
            // two-CPU-cycle recovery interval has expired.
            if (ppu.data_read_cooldown) return get_open_bus();
            uint16_t addr = ppu.bus_address;
            uint8_t value;
            if (addr >= 0x3F00) {
                uint8_t mask = (ppu.mask & 1) ? 0x30 : 0x3F;
                value = (ppu_read(addr) & mask) | (get_open_bus() & 0xC0);
                set_open_bus_masked(value, 0x3F);
            } else {
                value = ppu.ppudata_buffer;
                set_open_bus(value);
            }
            ppu.data_read_delay = 5;
            ppu.data_read_cooldown = 6;
            return value;
        }
        default:
            return get_open_bus();
    }
}

uint8_t ppu_reg_read_finish(uint16_t reg, uint8_t value) {
    switch (reg & 7) {
        case 2:
            value = (uint8_t)((value & 0x9F) | (ppu.status & 0x60));
            set_open_bus_masked(value, 0x60);
            return value;
        case 4:
            if (rendering_active()) value = ppu.oam_read_latch;
            set_open_bus(value);
            return value;
        default:
            return value;
    }
}

void ppu_reg_write(uint16_t reg, uint8_t value) {
    set_open_bus(value);
    switch (reg & 7) {
        case 0:
            ppu.ctrl = value;
            ppu.t = (ppu.t & ~0x0C00) | ((value & 3) << 10);
            ppu_eval_nmi();
            break;
        case 1:
            ppu.mask = value;
            break;
        case 3:
            ppu.oam_addr = value;
            break;
        case 4:
            if (rendering_active()) {
                ppu.oam_addr = (ppu.oam_addr + 4) & 0xFC;
            } else {
                if ((ppu.oam_addr & 3) == 2) value &= 0xE3;
                ppu.oam[ppu.oam_addr] = value;
                // PAL refresh also clocks this address latch late in vblank.
                int previous_dot = (ppu.dot + 340) % 341;
                bool refresh = nes_timing()->region == NES_REGION_PAL
                            && ppu.scanline >= 265 && !rendering_line();
                if (!refresh || ((previous_dot & 1) && previous_dot != 339))
                    ppu.oam_addr++;
            }
            break;
        case 5:
            if (!ppu.w) {
                ppu.x = value & 7;
                ppu.t = (ppu.t & ~0x001F) | (value >> 3);
            } else {
                ppu.t = (ppu.t & ~0x73E0) | ((value & 7) << 12) | ((value & 0xF8) << 2);
            }
            ppu.w ^= 1;
            break;
        case 6:
            if (!ppu.w) {
                ppu.t = (ppu.t & 0x00FF) | ((uint16_t)(value & 0x3F) << 8);
            } else {
                ppu.t = (ppu.t & 0x7F00) | value;
                ppu.address_write_value = ppu.t;
                ppu.address_write_delay = 3;
            }
            ppu.w ^= 1;
            break;
        case 7:
            ppu.data_write_value = value;
            ppu.data_write_delay = 5;
            break;
        default:
            break;
    }
}

static void ppu_complete_register_accesses(int dot) {
    if (ppu.address_write_delay && --ppu.address_write_delay == 0) {
        uint16_t address = ppu.address_write_value;
        if (rendering_active()) {
            if (dot == 257) {
                address &= ppu.v;
            } else if (dot > 0 && !(dot & 7) && (dot <= 256 || dot > 320)) {
                address = (address & ~0x041F) | (address & ppu.v & 0x041F);
            }
        }
        ppu.v = ppu.t = address;
        if (!rendering_active()) ppu_set_bus_address(ppu.v);
    }
    if (ppu.data_read_cooldown) ppu.data_read_cooldown--;
    if (ppu.data_increment_pending) {
        ppu.data_increment_pending = false;
        ppu_increment_data_address();
    }
    if (ppu.data_read_delay) {
        if (ppu.data_read_delay == 3 && !ppu.bus_ale_this_dot)
            ppu_bus_address_phase(ppu.v);
        if (ppu.data_read_delay == 1) {
            if (ppu.bus_read_this_dot) {
                ppu.ppudata_buffer = ppu.vram_bus_data;
            } else {
                ppu.ppudata_buffer = ppu_bus_read_phase(ppu.bus_address, CART_PPU_FETCH_CPU);
            }
            ppu.data_increment_pending = true;
            ppu.data_read_delay = 0;
        } else {
            ppu.data_read_delay--;
        }
    }
    if (ppu.data_write_delay) {
        if (ppu.data_write_delay == 3 && !ppu.bus_ale_this_dot)
            ppu_bus_address_phase(ppu.v);
        if (ppu.data_write_delay == 1) {
            uint16_t address = (uint16_t)((ppu.bus_address & 0x3F00) | ppu.vram_address_latch);
            uint8_t value = ppu.data_write_value;
            // While rendering, the multiplexed address lines drive the external
            // data bus. Palette RAM remains on the internal six-bit data path.
            if (address < 0x3F00 && rendering_active()) value = (uint8_t)address;
            ppu_write(address, value);
            ppu.data_increment_pending = true;
            ppu.data_write_delay = 0;
        } else {
            ppu.data_write_delay--;
        }
    }
}

void ppu_oam_dma(uint8_t page) {
    write_mem(0x4014, page);
}

void ppu_begin_vblank(void) {
    if (!ppu.suppress_vblank) ppu.status |= 0x80;
    ppu.suppress_vblank = false;
    ppu_eval_nmi();
}

void ppu_end_vblank(void) {
    ppu.status &= ~0x80;
    ppu_eval_nmi();
}

void ppu_begin_frame_render(uint32_t *fb) {
    uint32_t color = get_color(ppu_palette[0]);
    for (int i = 0; i < 256 * 240; ++i) fb[i] = color;
    memset(bg_opaque, 0, sizeof(bg_opaque));
}

void start_frame(void) {
    ppu.frame_complete = false;
}

void ppu_power_on(PPU *state) {
    memset(state, 0, sizeof(*state));
    memset(ppu_ob_expire, 0, sizeof(ppu_ob_expire));
    memset(state->oam, 0xFF, sizeof(state->oam));
    memset(state->secondary_oam, 0xFF, sizeof(state->secondary_oam));
    state->oam_bus = 0xFF;
    state->oam_read_latch = 0xFF;
    state->scanline = (int)nes_timing()->scanlines - 1;
    memset(ppu_vram, 0, sizeof(ppu_vram));
    cpu_set_nmi_line(false);
    static const uint8_t power_up_palette[PPU_PALETTE_SIZE] = {
        0x09,0x01,0x00,0x01,0x00,0x02,0x02,0x0D,
        0x08,0x10,0x08,0x24,0x00,0x00,0x04,0x2C,
        0x09,0x01,0x34,0x03,0x00,0x04,0x00,0x14,
        0x08,0x3A,0x00,0x02,0x00,0x20,0x2C,0x08
    };
    memcpy(ppu_palette, power_up_palette, sizeof(ppu_palette));
    ppu_palette_reset_default();
}

void ppu_reset(PPU *state) {
    ppu_power_on(state);
}

void ppu_soft_reset(PPU *state) {
    uint8_t oam[PPU_OAM_SIZE];
    uint8_t secondary_oam[32];
    memcpy(oam, state->oam, sizeof(oam));
    memcpy(secondary_oam, state->secondary_oam, sizeof(secondary_oam));
    uint16_t v = state->v;
    uint8_t status = state->status;
    uint64_t clocks = state->total_cycles;
    memset(state, 0, sizeof(*state));
    memcpy(state->oam, oam, sizeof(oam));
    memcpy(state->secondary_oam, secondary_oam, sizeof(secondary_oam));
    state->v = v;
    state->status = status;
    state->total_cycles = clocks;
    state->scanline = (int)nes_timing()->scanlines - 1;
    state->oam_bus = 0xFF;
    state->oam_read_latch = 0xFF;
    memset(ppu_ob_expire, 0, sizeof(ppu_ob_expire));
    cpu_set_nmi_line(false);
}

// Secondary OAM is cleared on clocks 1-64. Each following pair of clocks
// reads primary OAM, then copies or tests one byte for the next scanline.
static void ppu_evaluate_sprites(void) {
    if (ppu.dot <= 64) {
        ppu.oam_bus = 0xFF;
        ppu.secondary_oam[ppu.secondary_index & 0x1F] = 0xFF;
        if (!(ppu.dot & 1)) ppu.secondary_index = (uint8_t)((ppu.secondary_index + 1) & 0x1F);
        return;
    }
    if (ppu.dot == 65) {
        ppu.secondary_index = 0;
        ppu.secondary_oam_full = false;
        ppu.secondary_sprite_zero = false;
        ppu.eval_in_range = false;
        ppu.eval_done = false;
        ppu.overflow_count = 0;
    }
    if (ppu.dot & 1) {
        ppu.oam_bus = ppu.oam[ppu.oam_addr];
        if ((ppu.oam_addr & 3) == 2) ppu.oam_bus &= 0xE3;
        return;
    }

    uint8_t n = ppu.oam_addr >> 2;
    uint8_t m = ppu.oam_addr & 3;
    int height = (ppu.ctrl & 0x20) ? 16 : 8;
    bool in_range = ppu.scanline >= ppu.oam_bus && ppu.scanline < ppu.oam_bus + height;
    if (ppu.eval_done) {
        n = (n + 1) & 63;
        ppu.oam_bus = ppu.secondary_oam[ppu.secondary_index & 31];
    } else {
        if (in_range) ppu.eval_in_range = true;
        if (!ppu.secondary_oam_full) {
            ppu.secondary_oam[ppu.secondary_index] = ppu.oam_bus;
            if (ppu.eval_in_range) {
                if (ppu.dot == 66) ppu.secondary_sprite_zero = true;
                m++;
                uint8_t old_secondary = ppu.secondary_index;
                ppu.secondary_index = (uint8_t)((ppu.secondary_index + 1) & 0x1F);
                if (old_secondary == 0x1F) {
                    ppu.secondary_oam_full = true;
                    ppu.secondary_oam_overflowed = true;
                }
                if (m == 4) {
                    m = 0;
                    n = (n + 1) & 63;
                    if (n == 0) ppu.eval_done = true;
                }
                if ((ppu.secondary_index & 3) == 0) {
                    ppu.eval_in_range = false;
                    if (m && !in_range) m = 0;
                }
            } else {
                n = (n + 1) & 63;
                m = 0;
                if (n == 0) ppu.eval_done = true;
            }
        } else {
            ppu.oam_bus = ppu.secondary_oam[ppu.secondary_index & 31];
            if (ppu.eval_in_range) {
                ppu.sprite_status_pending |= 0x20;
                if (++m == 4) {
                    m = 0;
                    n = (n + 1) & 63;
                }
                if (!ppu.overflow_count) ppu.overflow_count = 3;
                else if (--ppu.overflow_count == 0) {
                    ppu.eval_done = true;
                    m = 0;
                }
            } else {
                // Once OAM is full, the broken carry checks tile/attribute/X
                // bytes as Y coordinates while walking diagonally through OAM.
                n = (n + 1) & 63;
                m = (m + 1) & 3;
                if (n == 0) ppu.eval_done = true;
            }
        }
    }
    ppu.oam_addr = (n << 2) | m;
}

static void ppu_fetch_sprite(void) {
    int index = (ppu.dot - 257) / 8;
    int phase = (ppu.dot - 257) & 7;
    ppu.oam_addr = 0;
    ppu.sprite_zero_on_line = ppu.secondary_sprite_zero;
    if (ppu.dot == 257) {
        ppu.sprite_count = 0;
        ppu.secondary_index = 0;
    } else if (!((ppu.dot - 1) & 4)) {
        ppu_increment_secondary_oam();
    }
    ppu.oam_bus = ppu.secondary_oam[ppu.secondary_index & 0x1F];
    if (phase == 0) ppu.sprite_fetch_y = ppu.oam_bus;
    else if (phase == 1) ppu.sprite_fetch_tile = ppu.oam_bus;
    else if (phase == 2) ppu.sprite_fetch_attr = ppu.oam_bus;
    else if (phase == 3) ppu.sprite_fetch_x = ppu.oam_bus;
    if (phase == 0 || phase == 2) {
        ppu_bus_address_phase(0x2000 | (ppu.v & 0x0FFF));
    } else if (phase == 1 || phase == 3) {
        (void)ppu_bus_read_phase(0x2000 | (ppu.v & 0x0FFF), CART_PPU_FETCH_SPRITE);
    } else if (phase == 4) {
        uint8_t y = ppu.sprite_fetch_y;
        uint8_t tile = ppu.sprite_fetch_tile;
        uint8_t attr = ppu.sprite_fetch_attr;
        uint16_t row = (uint16_t)((int)(uint8_t)ppu.scanline - y);
        bool tall = (ppu.ctrl & 0x20) != 0;
        if (attr & 0x80) row ^= tall ? 15 : 7;
        ppu.sprite_fetch_valid = row < (tall ? 16 : 8);
        uint8_t bit = (uint8_t)(1u << index);
        if (ppu.sprite_fetch_valid) {
            ppu.sprite_count++;
            ppu.sprite_valid_mask |= bit;
            ppu.sprite_expired_mask &= (uint8_t)~bit;
        } else {
            ppu.sprite_valid_mask &= (uint8_t)~bit;
            ppu.sprite_active_mask &= (uint8_t)~bit;
            ppu.sprite_counting_mask &= (uint8_t)~bit;
        }
        if (tall) {
            ppu.sprite_fetch_addr = ((tile & 1) << 12) | ((tile & 0xFE) << 4)
                                  | ((row & 8) << 1) | (row & 7);
        } else {
            ppu.sprite_fetch_addr = ((ppu.ctrl & 8) ? 0x1000 : 0) | (tile << 4) | (row & 7);
        }
        ppu.sprite_positions[index] = ppu.sprite_fetch_x;
        ppu.sprite_start_dot[index] = (uint16_t)ppu.sprite_positions[index] + 1;
        ppu.sprite_attributes[index] = attr;
        ppu_bus_address_phase(ppu.sprite_fetch_addr);
    } else if (phase == 5) {
        uint8_t value = ppu_bus_read_phase(ppu.sprite_fetch_addr, CART_PPU_FETCH_SPRITE);
        ppu.sprite_pattern_lo[index] = ppu.sprite_fetch_valid ? value : 0;
    } else if (phase == 6) {
        ppu_bus_address_phase(ppu.sprite_fetch_addr + 8);
    } else if (phase == 7) {
        uint8_t value = ppu_bus_read_phase(ppu.sprite_fetch_addr + 8, CART_PPU_FETCH_SPRITE);
        ppu.sprite_pattern_hi[index] = ppu.sprite_fetch_valid ? value : 0;
    }
}

static void ppu_fetch_background(void) {
    CartPpuFetchSource source = (ppu.ctrl & 0x20) ? CART_PPU_FETCH_BG : CART_PPU_FETCH_SPRITE;
    switch (ppu.dot & 7) {
        case 1:
            ppu_bus_address_phase(0x2000 | (ppu.v & 0x0FFF));
            break;
        case 2:
            ppu.nt_byte = ppu_bus_read_phase(0x2000 | (ppu.v & 0x0FFF), CART_PPU_FETCH_BG);
            ppu.bg_tile_addr = ((ppu.ctrl & 0x10) ? 0x1000 : 0)
                             | (ppu.nt_byte << 4) | ((ppu.v >> 12) & 7);
            break;
        case 3: {
            uint16_t addr = 0x23C0 | (ppu.v & 0x0C00) | ((ppu.v >> 4) & 0x38) | ((ppu.v >> 2) & 7);
            ppu_bus_address_phase(addr);
            break;
        }
        case 4: {
            uint16_t addr = 0x23C0 | (ppu.v & 0x0C00) | ((ppu.v >> 4) & 0x38) | ((ppu.v >> 2) & 7);
            uint8_t attr = ppu_bus_read_phase(addr, CART_PPU_FETCH_BG);
            uint8_t shift = ((ppu.v >> 4) & 4) | (ppu.v & 2);
            ppu.at_byte = (attr >> shift) & 3;
            break;
        }
        case 5:
            ppu_bus_address_phase(ppu.bg_tile_addr);
            break;
        case 6:
            ppu.pt_lo = ppu_bus_read_phase(ppu.bg_tile_addr, source);
            break;
        case 7:
            ppu_bus_address_phase(ppu.bg_tile_addr + 8);
            break;
        case 0:
            ppu.pt_hi = ppu_bus_read_phase(ppu.bg_tile_addr + 8, source);
            break;
        default:
            break;
    }
    if (ppu.scanline < 240 || ppu.dot >= 321) {
        ppu.bg_shift_lo <<= 1;
        ppu.bg_shift_hi = (uint16_t)((ppu.bg_shift_hi << 1) | 1);
        ppu.at_shift_lo <<= 1;
        ppu.at_shift_hi <<= 1;
    }
    if ((ppu.dot & 7) == 0) {
        ppu.bg_shift_lo = (ppu.bg_shift_lo & 0xFF00) | ppu.pt_lo;
        ppu.bg_shift_hi = (ppu.bg_shift_hi & 0xFF00) | ppu.pt_hi;
        ppu.at_shift_lo = (ppu.at_shift_lo & 0xFF00) | ((ppu.at_byte & 1) ? 0xFF : 0);
        ppu.at_shift_hi = (ppu.at_shift_hi & 0xFF00) | ((ppu.at_byte & 2) ? 0xFF : 0);
        ppu_increment_x();
    }
}

static void ppu_clock_sprite_counters(int dot) {
    for (unsigned i = 0; i < 8; ++i) {
        uint8_t bit = (uint8_t)(1u << i);
        if ((ppu.sprite_counting_mask & bit) && dot == ppu.sprite_start_dot[i]) {
            ppu.sprite_counting_mask &= (uint8_t)~bit;
            ppu.sprite_expired_mask |= bit;
            ppu.sprite_active_mask |= bit;
        }
    }
}

static void ppu_render_dot(int x, int y) {
    uint8_t background = 0;
    uint8_t background_palette = 0;
    uint8_t sprite = 0;
    uint8_t sprite_palette = 0;
    bool sprite_behind = false;
    if ((ppu.mask & 8) && (x >= 8 || (ppu.mask & 2))) {
        uint16_t bit = 0x8000 >> ppu.x;
        background = ((ppu.bg_shift_lo & bit) ? 1 : 0) | ((ppu.bg_shift_hi & bit) ? 2 : 0);
        background_palette = ((ppu.at_shift_lo & bit) ? 1 : 0) | ((ppu.at_shift_hi & bit) ? 2 : 0);
    }
    if (ppu.fetches_enabled) {
        uint8_t active = ppu.sprite_skip_clocks ? ppu.sprite_valid_mask : ppu.sprite_active_mask;
        bool show_sprites = (ppu.mask & 0x10) && (x >= 8 || (ppu.mask & 4));
        for (unsigned i = 0; i < 8; ++i) {
            uint8_t slot = (uint8_t)(1u << i);
            if (!(active & slot)) continue;
            uint8_t attr = ppu.sprite_attributes[i];
            int bit = (attr & 0x40) ? 0 : 7;
            uint8_t pixel = ((ppu.sprite_pattern_lo[i] >> bit) & 1)
                          | (((ppu.sprite_pattern_hi[i] >> bit) & 1) << 1);
            if (attr & 0x40) {
                ppu.sprite_pattern_lo[i] >>= 1;
                ppu.sprite_pattern_hi[i] >>= 1;
            } else {
                ppu.sprite_pattern_lo[i] <<= 1;
                ppu.sprite_pattern_hi[i] <<= 1;
            }
            if (!(ppu.sprite_pattern_lo[i] | ppu.sprite_pattern_hi[i]))
                ppu.sprite_active_mask &= (uint8_t)~slot;
            if (!pixel || sprite || !show_sprites) continue;
            sprite = pixel;
            sprite_palette = attr & 3;
            sprite_behind = (attr & 0x20) != 0;
            if (i == 0 && ppu.sprite_zero_on_line && background && x != 255) {
                ppu.sprite_status_pending |= 0x40;
            }
        }
    }
    uint8_t color = ppu_palette[0];
    if (!ppu.rendering_enabled && (ppu.v & 0x3F00) == 0x3F00) color = ppu_read(ppu.v);
    if (sprite && (!sprite_behind || !background)) color = ppu_palette[0x10 + sprite_palette * 4 + sprite];
    else if (background) color = ppu_palette[background_palette * 4 + background];
    bg_opaque[y * 256 + x] = background != 0;
    framebuffer[y * 256 + x] = get_color(color);
}

void ppu_step_dots(int ppu_cycles) {
    for (int i = 0; i < ppu_cycles; ++i) {
        int line = ppu.scanline;
        int dot = ppu.dot;
        bool rendering = ppu.fetches_enabled;
        bool visible = line < 240;
        bool prerender = line == (int)nes_timing()->scanlines - 1;
        ppu.bus_ale_this_dot = false;
        ppu.bus_read_this_dot = false;
        ppu.oam_read_latch = ppu.oam_bus;

        if (ppu.sprite_status_pending) {
            ppu.status |= ppu.sprite_status_pending;
            if (ppu.sprite_status_pending & 0x40) ppu.sprite_zero_hit = true;
            ppu.sprite_status_pending = 0;
        }

        if ((visible || prerender) && rendering && ppu.oam_corruption_pending) {
            ppu_corrupt_oam_row(ppu.oam_corruption_row);
            ppu.oam_corruption_pending = false;
        }

        if (dot == 0) {
            if (visible && rendering && (line > 0 || !ppu.skipped_frame_dot)) {
                // The unused nametable fetch drives a pattern address between
                // scanlines without reading another byte from CHR memory.
                uint16_t address = (ppu.nt_byte << 4) | ((ppu.v >> 12) & 7)
                                 | ((ppu.ctrl & 0x10) ? 0x1000 : 0);
                ppu_bus_address_phase(address);
            } else if (line == 240) {
                ppu_set_bus_address(ppu.v);
            }
            if (visible && rendering) ppu.secondary_index = 0;
            if (line == 0) ppu.skipped_frame_dot = false;
        }
        if (line == (int)nes_timing()->vblank_scanline && dot == 1) {
            cart_notify_vblank_start();
            ppu_begin_vblank();
        }
        if (prerender && dot == 1) {
            ppu_end_vblank();
            ppu.status &= ~(0x40 | 0x20);
            ppu.sprite_status_pending = 0;
            ppu.sprite_zero_hit = false;
            ppu.suppress_vblank = false;
        }
        if (nes_timing()->region == NES_REGION_PAL && line >= 265 && !prerender
            && dot && !(dot & 1))
            ppu.oam_addr++;
        if (visible && dot >= 1 && dot <= 256) {
            // X counters continue counting while rendering is disabled; the
            // pattern shifters hold their data until rendering resumes.
            ppu_clock_sprite_counters(dot);
            ppu_render_dot(dot - 1, line);
            if (rendering) ppu_evaluate_sprites();
        }
        if ((visible || prerender) && rendering && (dot == 63 || dot == 255 || dot == 339))
            ppu.secondary_oam_overflowed = false;
        if ((visible || prerender) && rendering) {
            if (visible && dot == 3) cart_notify_scanline_early();
            if ((dot >= 1 && dot <= 256) || (dot >= 321 && dot <= 336))
                ppu_fetch_background();
            if (dot == 256) ppu_increment_y();
            if (prerender && dot >= 280 && dot <= 304)
                ppu.v = (ppu.v & ~0x7BE0) | (ppu.t & 0x7BE0);
            if (dot >= 257 && dot <= 320) ppu_fetch_sprite();
            if (dot == 257) ppu.v = (ppu.v & ~0x041F) | (ppu.t & 0x041F);
            if (dot == 321) {
                ppu_increment_secondary_oam();
                ppu.oam_bus = ppu.secondary_oam[ppu.secondary_index & 0x1F];
            }
        }

        if ((visible || prerender) && ppu.rendering_enabled) {
            if (dot == 337 || dot == 339) {
                ppu_bus_address_phase(0x2000 | (ppu.v & 0x0FFF));
            } else if (dot == 338 || dot == 340) {
                ppu.nt_byte = ppu_bus_read_phase(0x2000 | (ppu.v & 0x0FFF), CART_PPU_FETCH_BG);
            }
        }

        bool skip_dot = nes_timing()->region == NES_REGION_NTSC && prerender && dot == 339
                     && ppu.odd_frame && ppu.rendering_enabled;
        if ((visible || prerender) && dot == 339) {
            if (ppu.rendering_enabled)
                ppu.sprite_counting_mask |= ppu.sprite_valid_mask & (uint8_t)~ppu.sprite_expired_mask;
            ppu.sprite_active_mask = ppu.sprite_valid_mask & (uint8_t)~ppu.sprite_counting_mask;
            if (skip_dot) {
                for (unsigned sprite = 0; sprite < 8; ++sprite) ppu.sprite_start_dot[sprite]++;
            }
        }
        // PPUMASK reaches the rendering latch after one PPU clock, then
        // reaches the fetch/evaluation circuits after the following clock.
        if (ppu.fetches_enabled != ppu.rendering_enabled) {
            ppu.fetches_enabled = ppu.rendering_enabled;
            if (rendering_line()) {
                if (!ppu.fetches_enabled) {
                    ppu.oam_corruption_row = ppu_oam_corruption_row(dot);
                    ppu.oam_corruption_pending = true;
                    ppu_set_bus_address(ppu.v);
                    if (dot >= 65 && dot <= 256) ppu.oam_addr++;
                }
            }
        }
        ppu.rendering_enabled = (ppu.mask & 0x18) != 0;
        ppu_complete_register_accesses(dot);
        if (ppu.sprite_skip_clocks) ppu.sprite_skip_clocks--;
        ppu.total_cycles++;
        ppu.dot++;
        if (skip_dot) {
            ppu.dot++;
            ppu.skipped_frame_dot = true;
            ppu.sprite_skip_clocks = 2;
        }
        if (ppu.dot == 341) {
            ppu.dot = 0;
            if (++ppu.scanline == (int)nes_timing()->scanlines) {
                ppu.scanline = 0;
                ppu.odd_frame = !ppu.odd_frame;
                ppu.frame_complete = true;
            }
        }
    }
}

void ppu_step(int cpu_cycles) {
    if (cpu_cycles <= 0) return;
    const NesTiming *timing = nes_timing();
    for (int i = 0; i < cpu_cycles; ++i) {
        unsigned clocks = ppu.cpu_clock_phase + timing->cpu_divider;
        ppu_step_dots((int)(clocks / timing->ppu_divider));
        ppu.cpu_clock_phase = clocks % timing->ppu_divider;
    }
}

uint32_t get_color(uint8_t idx) {
    idx &= 0x3F;

    // PPUMASK bit 0: grayscale
    if (ppu.mask & 0x01) idx &= 0x30;

    uint8_t emphasis = ppu.mask;
    if (nes_timing()->region != NES_REGION_NTSC)
        emphasis = (uint8_t)((emphasis & ~0x60u) | ((emphasis & 0x20u) << 1)
                            | ((emphasis & 0x40u) >> 1));

    // If we have emphasis tables, pick the one matching the physical output bits.
    if (ppu__have_emphasis_tables) {
        int e = ((emphasis & 0x20) ? 1 : 0)
              | ((emphasis & 0x40) ? 2 : 0)
              | ((emphasis & 0x80) ? 4 : 0);
        return ppu__emphasis_palettes[e][idx];
    }

    // Otherwise, use the active base palette and apply software emphasis.
    uint32_t c = ppu__active_palette_base[idx];
    float r = (float)((c >> 16) & 0xFF);
    float g = (float)((c >>  8) & 0xFF);
    float b = (float)((c      ) & 0xFF);

    const float ATTEN = 0.60f;
    if (emphasis & 0x20) { g *= ATTEN; b *= ATTEN; }  // emphasize RED
    if (emphasis & 0x40) { r *= ATTEN; b *= ATTEN; }  // emphasize GREEN
    if (emphasis & 0x80) { r *= ATTEN; g *= ATTEN; }  // emphasize BLUE

    int R = (int)(r < 0 ? 0 : (r > 255 ? 255 : r));
    int G = (int)(g < 0 ? 0 : (g > 255 ? 255 : g));
    int B = (int)(b < 0 ? 0 : (b > 255 ? 255 : b));
    return 0xFF000000u | (R << 16) | (G << 8) | B;
}
