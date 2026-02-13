/*
 * mapper.h - NES cartridge mapper interface header
 * 
 * Author: @frankischilling
 * 
 * This header defines the mapper interface and structures for NES cartridge emulation.
 * Provides function pointers for CPU/PPU read/write operations, reset, clock, and
 * mirroring control that mappers can implement.
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

#ifndef MAPPER_H
#define MAPPER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "rom.h"

typedef struct Mapper {
    // CPU space ($6000–$FFFF typically)
    uint8_t (*cpu_read)(uint16_t addr);
    void     (*cpu_write)(uint16_t addr, uint8_t v);

    // PPU pattern space ($0000–$1FFF)
    uint8_t (*ppu_read)(uint16_t addr);
    void     (*ppu_write)(uint16_t addr, uint8_t v);

    // Optional hooks
    void     (*reset)(void);
    void     (*clock)(int cpu_cycles);   // for IRQ-capable mappers later
    Mirroring (*get_mirroring)(void);
} Mapper;

// Global “inserted” cart
extern Mapper *cart;

typedef enum {
    CART_PPU_FETCH_CPU = 0,
    CART_PPU_FETCH_BG,
    CART_PPU_FETCH_SPRITE
} CartPpuFetchSource;

// Front door used by CPU/PPU
uint8_t cart_cpu_read (uint16_t addr);
void    cart_cpu_write(uint16_t addr, uint8_t v);
uint8_t cart_ppu_read (uint16_t addr);
void    cart_ppu_write(uint16_t addr, uint8_t v);
void    cart_set_ppu_fetch_source(CartPpuFetchSource src);

// Mapper-aware nametable access ($2000-$2FFF decoded by PPU)
uint8_t cart_nt_read (uint16_t addr, uint8_t *nt_ram);
void    cart_nt_write(uint16_t addr, uint8_t v, uint8_t *nt_ram);

// Mapper IRQ line helpers (for IRQ-capable mappers such as MMC3)
bool cart_irq_pending(void);
void cart_irq_ack(void);

// Notify mapper about end-of-scanline timing event (used by MMC3 IRQ)
void cart_notify_scanline(void);
// Optional early-scanline timing event (used by MMC5 timing tweaks)
void cart_notify_scanline_early(void);
// Notify mapper when vblank starts (MMC5 in-frame/IRQ state)
void cart_notify_vblank_start(void);

// Battery-backed PRG-RAM persistence (.sav)
void cart_battery_configure(const char *rom_path, bool has_battery);
void cart_battery_flush(void);
void cart_battery_shutdown(void);

// Init from iNES 1.0 header + loaded PRG/CHR blobs
int mapper_init_from_header(const iNESHeader *h,
                            uint8_t *prg, size_t prg_sz,
                            uint8_t *chr, size_t chr_sz);

// Current mirroring for PPU (keeps your existing mirroring_mode in sync)
Mirroring cart_get_mirroring(void);
void      cart_set_mirroring(Mirroring m);

#endif