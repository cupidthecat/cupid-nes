/*
 * rom.h - NES ROM and cartridge data interface
 *
 * Author: @frankischilling
 *
 * This header defines iNES header fields, mirroring modes, loaded PRG and CHR data,
 * cartridge sizes, loader functions, and the helpers used to inspect cartridge metadata.
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

#ifndef ROM_H
#define ROM_H
#include <stdint.h>
#include <stddef.h>

typedef struct __attribute__((packed)) {
    uint8_t signature[4];      // "NES\x1A"
    uint8_t prg_rom_chunks;    // count of 16KB units
    uint8_t chr_rom_chunks;    // count of 8KB  units
    uint8_t flags6;
    uint8_t flags7;
    uint8_t prg_ram_size;      // (iNES 1.0) in 8KB units
    uint8_t flags9;
    uint8_t flags10;
    uint8_t zero[5];
} iNESHeader;

typedef enum {
    MIRROR_HORIZONTAL = 0,
    MIRROR_VERTICAL   = 1,
    MIRROR_SINGLE0    = 2,
    MIRROR_SINGLE1    = 3,
    MIRROR_FOUR       = 4
} Mirroring;

// expose sizes so CPU/PPU can reason about mirroring
extern iNESHeader ines_header;
extern size_t     prg_size;
extern size_t     chr_size;
extern uint8_t   *prg_rom;
extern uint8_t   *chr_rom;

int load_rom(const char *filename);
// Eject the cartridge and release loader-owned buffers; safe to call repeatedly.
void unload_rom(void);
// Load an iNES image without a disk file or battery save path; copies its bytes.
// Failed loads preserve the currently inserted cartridge.
int load_rom_memory(const uint8_t *data, size_t size);
int rom_mapper_number(const iNESHeader *header);

typedef struct {
    size_t prg_ram, prg_nvram;
    size_t chr_ram, chr_nvram;
} RomRamSizes;

// Decode declared RAM capacities, including the iNES 8KB PRG-RAM default.
int rom_ram_sizes(const iNESHeader *header, RomRamSizes *sizes);

// mirroring for PPU
extern int mirroring_mode;

#endif
