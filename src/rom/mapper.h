/*
 * mapper.h - NES cartridge mapper interface
 *
 * Author: @frankischilling
 *
 * This header defines the mapper callbacks and cartridge interfaces used for CPU and PPU
 * access, mirroring, IRQs, save memory, scanline and bus hooks, and mapper lifecycle state.
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

typedef struct CartridgeBoard CartridgeBoard;

typedef struct Mapper {
    // CPU-visible cartridge space, usually $6000-$FFFF.
    uint8_t (*cpu_read)(uint16_t addr);
    void     (*cpu_write)(uint16_t addr, uint8_t v);

    // PPU pattern-table space at $0000-$1FFF.
    uint8_t (*ppu_read)(uint16_t addr);
    void     (*ppu_write)(uint16_t addr, uint8_t v);

    // Optional hooks
    void     (*reset)(void);
    void     (*clock)(int cpu_cycles);   // for IRQ-capable mappers later
    Mirroring (*get_mirroring)(void);
} Mapper;

typedef struct FdsImage FdsImage;

// Global “inserted” cart
extern Mapper *cart;

typedef enum {
    CART_PPU_FETCH_CPU = 0,
    CART_PPU_FETCH_BG,
    CART_PPU_FETCH_SPRITE
} CartPpuFetchSource;

// Front door used by CPU/PPU
uint8_t cart_cpu_read (uint16_t addr);
// Resolve floating data lines against the CPU latch, without a data-byte sentinel.
uint8_t cart_cpu_read_bus(uint16_t addr, uint8_t open_bus);
void    cart_cpu_write(uint16_t addr, uint8_t v);
bool cart_read_cpu_register(uint16_t address, uint8_t *value);
void cart_observe_cpu_write(uint16_t address, uint8_t value);
// Clock one real CPU bus cycle while exposing whether it is a write to boards
// whose counters can select CPU-write cycles as their clock source.
void    cart_clock_cpu_cycle(bool write_cycle);
uint8_t cart_ppu_read (uint16_t addr);
void    cart_ppu_write(uint16_t addr, uint8_t v);
void    cart_set_ppu_fetch_source(CartPpuFetchSource src);
// Notify an exact CPU write to $2000. MMC5 does not observe PPU register mirrors.
void    cart_notify_ppu_ctrl_write(uint8_t value);
// Current cartridge expansion-audio contribution, zero when the board has none.
float   cart_expansion_audio(void);
// Scan an EAN-8 or EAN-13 code through the connected Datach reader.
bool    cart_set_barcode(const char *digits);

// Select the MMC3 IRQ counter revision used by compatible MMC3-family boards.
// MMC6 and MC-ACC keep their board-specific IRQ behavior.
bool        cart_set_mmc3_revision_name(const char *name);
const char *cart_mmc3_revision_name(void);
bool        cart_set_dip_switches(unsigned value);
unsigned    cart_dip_switches(void);

typedef enum {
    CART_KARAOKE_A,
    CART_KARAOKE_B,
    CART_KARAOKE_MICROPHONE,
    CART_KARAOKE_INPUT_COUNT
} CartKaraokeInput;

bool cart_set_karaoke_input(CartKaraokeInput input, bool pressed);

// Mapper-aware nametable access ($2000-$2FFF decoded by PPU)
uint8_t cart_nt_read (uint16_t addr, uint8_t *nt_ram);
void    cart_nt_write(uint16_t addr, uint8_t v, uint8_t *nt_ram);

// Mapper IRQ line helpers (for IRQ-capable mappers such as MMC3)
bool cart_irq_pending(void);
void cart_irq_ack(void);
// Console reset signals are separate from cartridge insertion and RAM allocation.
void cart_console_reset(bool soft_reset);
void cart_after_console_reset(void);

// Notify physical PPU bus address changes using monotonic NTSC PPU cycles.
// MMC3 qualifies A12 after three CPU clocks low; palette RAM is internal.
void cart_notify_ppu_address(uint16_t addr, uint64_t ppu_cycle);
// Legacy scanline hook; MMC3 uses the PPU address hook above.
void cart_notify_scanline(void);
// Optional early-scanline timing event (used by MMC5 timing tweaks)
void cart_notify_scanline_early(void);
// Notify mapper when vblank starts (MMC5 in-frame/IRQ state)
void cart_notify_vblank_start(void);

// Persist nonvolatile PRG, CHR and serial EEPROM chips in separate save images.
void cart_battery_configure(const char *rom_path, bool has_battery);
void cart_battery_flush(void);
void cart_battery_shutdown(void);
// Initialize the trainer window after PRG-RAM and battery data have been loaded.
void cart_apply_trainer(const uint8_t trainer[512]);

// Init from iNES/NES 2.0 header + loaded PRG/CHR blobs.
// Returns mapper number, or -1 without replacing the active cart on failure.
int mapper_init_from_header(const iNESHeader *h,
                            uint8_t *prg, size_t prg_sz,
                            uint8_t *chr, size_t chr_sz);
int mapper_init_from_header_metadata(const iNESHeader *h,
                                     uint8_t *prg, size_t prg_sz,
                                     uint8_t *chr, size_t chr_sz,
                                     const RomDatabaseInfo *database);
// Activate a fully validated disk-system image. Takes ownership on success.
int mapper_init_fds(FdsImage *image);
// Activate a prepared StudyBox board. Takes ownership on success.
int mapper_init_studybox(CartridgeBoard *board);
// Flush saves, eject the mapper, and release mapper-owned RAM.
// The caller retains ownership of the PRG/CHR buffers passed to initialization.
void mapper_shutdown(void);

// Current mirroring for PPU (keeps your existing mirroring_mode in sync)
Mirroring cart_get_mirroring(void);
void      cart_set_mirroring(Mirroring m);

#endif
