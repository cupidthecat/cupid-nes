#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "rom.h"

#define PRG_ROM_BANK_SIZE 0x4000  // 16KB per bank
#define CHR_ROM_BANK_SIZE 0x2000  // 8KB per bank

// Global buffers
uint8_t prg_rom[0x8000];
uint8_t *chr_rom = NULL;
iNESHeader ines_header;  // Global header variable

// Global mirroring mode variable; default to horizontal.
int mirroring_mode = 0;

int load_rom(const char *filename) {
    FILE *fp = fopen(filename, "rb");
    if (!fp) {
        perror("Failed to open ROM file");
        return -1;
    }

    // Read the header into our global header variable.
    if (fread(&ines_header, sizeof(iNESHeader), 1, fp) != 1) {
        perror("Failed to read iNES header");
        fclose(fp);
        return -1;
    }

    // Validate signature
    if (memcmp(ines_header.signature, "NES\x1A", 4) != 0) {
        fprintf(stderr, "Invalid iNES signature.\n");
        fclose(fp);
        return -1;
    }

    // Set mirroring mode: bit 0 of flags6.
    mirroring_mode = (ines_header.flags6 & 0x01) ? 1 : 0;

    // Skip trainer if present.
    if (ines_header.flags6 & 0x04) {
        fseek(fp, 512, SEEK_CUR);
    }

    int prg_size = ines_header.prg_rom_chunks * PRG_ROM_BANK_SIZE;
    if (fread(prg_rom, 1, prg_size, fp) != (size_t)prg_size) {
        fprintf(stderr, "Failed to read PRG-ROM data.\n");
        fclose(fp);
        return -1;
    }

    int chr_size = ines_header.chr_rom_chunks * CHR_ROM_BANK_SIZE;
    if (chr_size > 0) {
        chr_rom = malloc(chr_size);
        if (!chr_rom) {
            fprintf(stderr, "Failed to allocate memory for CHR-ROM.\n");
            fclose(fp);
            return -1;
        }
        if (fread(chr_rom, 1, chr_size, fp) != (size_t)chr_size) {
            fprintf(stderr, "Failed to read CHR-ROM data.\n");
            free(chr_rom);
            fclose(fp);
            return -1;
        }
        // Mirror 4KB CHR-ROM if needed.
        if (chr_size == 0x1000) {
            memcpy(&chr_rom[0x1000], chr_rom, 0x1000);
        }
    } else {
        chr_rom = malloc(CHR_ROM_BANK_SIZE);
        if (!chr_rom) {
            fprintf(stderr, "Failed to allocate memory for CHR-RAM.\n");
            fclose(fp);
            return -1;
        }
        memset(chr_rom, 0, CHR_ROM_BANK_SIZE);
    }
    fclose(fp);

    // For NROM: mirror PRG-ROM if only one bank is present.
    if (ines_header.prg_rom_chunks == 1) {
        memcpy(&prg_rom[PRG_ROM_BANK_SIZE], prg_rom, PRG_ROM_BANK_SIZE);
    }

    return 0;
}