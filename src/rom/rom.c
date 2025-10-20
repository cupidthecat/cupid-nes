// rom.c
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "rom.h"

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

    // If only one 16KB PRG bank, mirror it into the upper half
    if (prg_size == PRG_ROM_BANK_SIZE) {
        memcpy(&prg_rom[PRG_ROM_BANK_SIZE], prg_rom, PRG_ROM_BANK_SIZE);
        prg_size = 0x8000; // now effectively 32KB mapped
    }

    return 0;
}
