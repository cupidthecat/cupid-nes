/*
 * rom.c - NES ROM loading and iNES format parsing
 * 
 * Author: @frankischilling
 * 
 * This file handles loading NES ROM files in iNES format. It parses the iNES header, loads
 * PRG-ROM and CHR-ROM data, handles trainers, determines mirroring modes, and initializes
 * the appropriate mapper. Supports both CHR-ROM and CHR-RAM cartridges.
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "rom.h"
#include "mapper.h"

#define PRG_ROM_BANK_SIZE 0x4000  // 16KB
#define CHR_ROM_BANK_SIZE 0x2000  // 8KB

uint8_t *prg_rom = NULL;
uint8_t *chr_rom = NULL;
iNESHeader ines_header;

size_t prg_size = 0;
size_t chr_size = 0;

int mirroring_mode = 0;

static int is_nes20(const iNESHeader *h) {
    // NES 2.0 if (flags7 & 0x0C) == 0x08
    return ((h->flags7 & 0x0C) == 0x08);
}

int load_rom(const char *filename) {
    cart_battery_shutdown();

    FILE *fp = fopen(filename, "rb");
    if (!fp) { perror("open"); return -1; }

    if (prg_rom) {
        free(prg_rom);
        prg_rom = NULL;
    }
    if (chr_rom) {
        free(chr_rom);
        chr_rom = NULL;
    }

    if (fread(&ines_header, sizeof(ines_header), 1, fp) != 1) {
        fprintf(stderr, "Failed to read iNES header\n");
        fclose(fp);
        return -1;
    }

    if (memcmp(ines_header.signature, "NES\x1A", 4) != 0) {
        fprintf(stderr, "Invalid iNES signature\n");
        fclose(fp);
        return -1;
    }

    int is_nes2 = is_nes20(&ines_header);
    
    mirroring_mode = (ines_header.flags6 & 0x01) ? 1 : 0;

    if (ines_header.flags6 & 0x04) { // trainer
        if (fseek(fp, 512, SEEK_CUR) != 0) {
            fprintf(stderr, "Seek past trainer failed\n");
            fclose(fp);
            return -1;
        }
    }

    // Calculate PRG and CHR sizes based on format
    if (is_nes2) {
        // NES 2.0: use extended size fields
        int prg_msb = (ines_header.flags9 & 0x0F);
        int chr_msb = (ines_header.flags9 & 0xF0) >> 4;
        
        if (prg_msb == 0x0F) {
            // Exponent-multiplier notation (rarely used)
            int exp = (ines_header.prg_rom_chunks & 0xFC) >> 2;
            int mul = (ines_header.prg_rom_chunks & 0x03);
            prg_size = (size_t)(1 << exp) * (mul * 2 + 1);
        } else {
            prg_size = (size_t)((prg_msb << 8) | ines_header.prg_rom_chunks) * PRG_ROM_BANK_SIZE;
        }
        
        if (chr_msb == 0x0F) {
            // Exponent-multiplier notation
            int exp = (ines_header.chr_rom_chunks & 0xFC) >> 2;
            int mul = (ines_header.chr_rom_chunks & 0x03);
            chr_size = (size_t)(1 << exp) * (mul * 2 + 1);
        } else {
            chr_size = (size_t)((chr_msb << 8) | ines_header.chr_rom_chunks) * CHR_ROM_BANK_SIZE;
        }
        
        printf("NES 2.0 format detected\n");
    } else {
        // iNES 1.0: use original fields
        prg_size = (size_t)ines_header.prg_rom_chunks * PRG_ROM_BANK_SIZE;
        chr_size = (size_t)ines_header.chr_rom_chunks * CHR_ROM_BANK_SIZE;
    }

    if (prg_size == 0) {
        fprintf(stderr, "Unsupported PRG size: %zu\n", prg_size);
        fclose(fp);
        return -1;
    }

    size_t prg_alloc_size = (prg_size == PRG_ROM_BANK_SIZE) ? (PRG_ROM_BANK_SIZE * 2) : prg_size;
    prg_rom = (uint8_t*)malloc(prg_alloc_size);
    if (!prg_rom) {
        fprintf(stderr, "PRG alloc failed (%zu bytes)\n", prg_alloc_size);
        fclose(fp);
        return -1;
    }

    if (fread(prg_rom, 1, prg_size, fp) != prg_size) {
        fprintf(stderr, "Failed to read PRG-ROM (%zu bytes)\n", prg_size);
        free(prg_rom);
        prg_rom = NULL;
        fclose(fp);
        return -1;
    }

    if (chr_size > 0) {
        chr_rom = (uint8_t*)malloc(chr_size);
        if (!chr_rom) {
            fprintf(stderr, "CHR alloc failed\n");
            free(prg_rom); prg_rom = NULL;
            fclose(fp);
            return -1;
        }
        if (fread(chr_rom, 1, chr_size, fp) != chr_size) {
            fprintf(stderr, "Failed to read CHR-ROM (%zu bytes)\n", chr_size);
            free(chr_rom); chr_rom = NULL;
            free(prg_rom); prg_rom = NULL;
            fclose(fp);
            return -1;
        }
        // mirror 4KB CHR to 8KB if needed
        if (chr_size == 0x1000) memcpy(&chr_rom[0x1000], chr_rom, 0x1000);
    } else {
        // CHR-RAM
        chr_size = CHR_ROM_BANK_SIZE;
        chr_rom = (uint8_t*)calloc(1, chr_size);
        if (!chr_rom) {
            fprintf(stderr, "CHR-RAM alloc failed\n");
            free(prg_rom); prg_rom = NULL;
            fclose(fp);
            return -1;
        }
    }

    fclose(fp);

    if (prg_size == PRG_ROM_BANK_SIZE) {
        memcpy(&prg_rom[PRG_ROM_BANK_SIZE], prg_rom, PRG_ROM_BANK_SIZE);
        prg_size = 0x8000;
    }

    int mapper_no = mapper_init_from_header(&ines_header, prg_rom, prg_size, chr_rom, chr_size);
    cart_battery_configure(filename, (ines_header.flags6 & 0x02u) != 0);

    // Keep the old global in sync so PPU mirroring stays correct
    Mirroring m = cart_get_mirroring();
    switch (m) {
        case MIRROR_HORIZONTAL: mirroring_mode = 0; break;
        case MIRROR_VERTICAL:   mirroring_mode = 1; break;
        case MIRROR_SINGLE0:    mirroring_mode = 2; break;
        case MIRROR_SINGLE1:    mirroring_mode = 3; break;
        default:                /* four-screen */   mirroring_mode = 1; break;
    }

    printf("Mapper: %d  (CHR %s)\n", mapper_no, (ines_header.chr_rom_chunks==0) ? "RAM" : "ROM");
    return 0;
}
