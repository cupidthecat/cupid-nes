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

uint8_t  prg_rom[0x8000];
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
    FILE *fp = fopen(filename, "rb");
    if (!fp) { perror("open"); return -1; }

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

    if (is_nes20(&ines_header)) {
        fprintf(stderr, "NES 2.0 not supported in this loader (yet)\n");
        fclose(fp);
        return -1;
    }

    mirroring_mode = (ines_header.flags6 & 0x01) ? 1 : 0;

    if (ines_header.flags6 & 0x04) { // trainer
        if (fseek(fp, 512, SEEK_CUR) != 0) {
            fprintf(stderr, "Seek past trainer failed\n");
            fclose(fp);
            return -1;
        }
    }

    prg_size = (size_t)ines_header.prg_rom_chunks * PRG_ROM_BANK_SIZE;
    chr_size = (size_t)ines_header.chr_rom_chunks * CHR_ROM_BANK_SIZE;

    // ---- HARD GUARDS to prevent overflow ----
    if (prg_size == 0 || prg_size > sizeof(prg_rom)) {
        fprintf(stderr, "Unsupported PRG size: %zu (max %zu)\n",
                prg_size, sizeof(prg_rom));
        fclose(fp);
        return -1;
    }

    if (fread(prg_rom, 1, prg_size, fp) != prg_size) {
        fprintf(stderr, "Failed to read PRG-ROM (%zu bytes)\n", prg_size);
        fclose(fp);
        return -1;
    }

    if (chr_size > 0) {
        chr_rom = (uint8_t*)malloc(chr_size);
        if (!chr_rom) { fprintf(stderr, "CHR alloc failed\n"); fclose(fp); return -1; }
        if (fread(chr_rom, 1, chr_size, fp) != chr_size) {
            fprintf(stderr, "Failed to read CHR-ROM (%zu bytes)\n", chr_size);
            free(chr_rom); chr_rom = NULL;
            fclose(fp);
            return -1;
        }
        // mirror 4KB CHR to 8KB if needed
        if (chr_size == 0x1000) memcpy(&chr_rom[0x1000], chr_rom, 0x1000);
    } else {
        // CHR-RAM
        chr_size = CHR_ROM_BANK_SIZE;
        chr_rom = (uint8_t*)calloc(1, chr_size);
        if (!chr_rom) { fprintf(stderr, "CHR-RAM alloc failed\n"); fclose(fp); return -1; }
    }

    fclose(fp);

    if (prg_size == PRG_ROM_BANK_SIZE) {
        memcpy(&prg_rom[PRG_ROM_BANK_SIZE], prg_rom, PRG_ROM_BANK_SIZE);
        prg_size = 0x8000;
    }

    int mapper_no = mapper_init_from_header(&ines_header, prg_rom, prg_size, chr_rom, chr_size);

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

    return 0;
}
