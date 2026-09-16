/* SPDX-License-Identifier: GPL-3.0-or-later
 * iNES and NES 2.0 loading, validation, cartridge setup, and persistence.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <limits.h>
#include "rom.h"
#include "mapper.h"
#include "../system/timing.h"

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

int rom_mapper_number(const iNESHeader *h) {
    if (!h) return -1;
    int mapper_no = h->flags6 >> 4;
    if (is_nes20(h))
        return mapper_no | (h->flags7 & 0xF0) | ((h->prg_ram_size & 0x0F) << 8);
    if ((h->flags7 & 0x0C) == 0) mapper_no |= h->flags7 & 0xF0;
    return mapper_no;
}

static int rom_console_supported(const iNESHeader *h) {
    if (is_nes20(h)) {
        unsigned console = h->flags7 & 0x03u;
        if (console == 0) return 1;
        // Extended console type 0 still identifies a regular NES/Famicom-family machine.
        return console == 3 && (h->zero[2] & 0x0Fu) == 0;
    }
    // Archaic headers have unreliable byte 7 contents.  Only clean iNES headers
    // use its low bits as the VS/PlayChoice console selector.
    if ((h->flags7 & 0x0Cu) == 0)
        return (h->flags7 & 0x03u) == 0;
    return 1;
}

static NesRegion rom_region(const iNESHeader *h) {
    if (is_nes20(h)) {
        switch (h->zero[1] & 0x03u) {
            case 1: return NES_REGION_PAL;
            case 3: return NES_REGION_DENDY;
            case 0:
            case 2:
            default:
                return NES_REGION_NTSC;
        }
    }
    if ((h->flags7 & 0x0Cu) == 0)
        return (h->flags9 & 0x01u) ? NES_REGION_PAL : NES_REGION_NTSC;
    // Archaic iNES headers use byte 7 inconsistently, so later bytes cannot
    // be trusted as timing metadata.
    return NES_REGION_NTSC;
}

static int nes20_rom_size(uint8_t low, uint8_t high, size_t unit, size_t *size) {
    if (high != 0x0F) {
        *size = (((size_t)high << 8) | low) * unit;
        return 0;
    }
    unsigned exponent = low >> 2;
    size_t multiplier = (size_t)(low & 3) * 2 + 1;
    if (exponent >= sizeof(size_t) * CHAR_BIT || multiplier > (SIZE_MAX >> exponent))
        return -1;
    *size = multiplier << exponent;
    return 0;
}

static size_t nes20_ram_size(uint8_t shift) {
    return shift ? (size_t)64 << shift : 0;
}

int rom_ram_sizes(const iNESHeader *header, RomRamSizes *sizes) {
    if (!header || !sizes) return -1;
    memset(sizes, 0, sizeof(*sizes));
    if (is_nes20(header)) {
        sizes->prg_ram = nes20_ram_size(header->flags10 & 0x0F);
        sizes->prg_nvram = nes20_ram_size(header->flags10 >> 4);
        sizes->chr_ram = nes20_ram_size(header->zero[0] & 0x0F);
        sizes->chr_nvram = nes20_ram_size(header->zero[0] >> 4);
    } else {
        // Unknown legacy MMC5 boards expose all eight 8KB RAM banks.  Other
        // iNES boards use the format's conventional 8KB default when byte 8 is zero.
        size_t default_units = rom_mapper_number(header) == 5 ? 8u : 1u;
        size_t prg_ram_bytes = (size_t)(header->prg_ram_size ? header->prg_ram_size : default_units) * 0x2000;
        if (header->flags6 & 2) sizes->prg_nvram = prg_ram_bytes;
        else sizes->prg_ram = prg_ram_bytes;
        if (!header->chr_rom_chunks)
            sizes->chr_ram = rom_mapper_number(header) == 13 ? 0x4000 : 0x2000;
    }
    return 0;
}

static int load_rom_data(const uint8_t *data, size_t size, const char *filename) {
    iNESHeader header;
    if (!data || size < sizeof(header)) {
        fprintf(stderr, "Truncated iNES header\n");
        return -1;
    }
    memcpy(&header, data, sizeof(header));
    if (memcmp(header.signature, "NES\x1A", 4) != 0) {
        fprintf(stderr, "Invalid iNES signature\n");
        return -1;
    }
    if (!rom_console_supported(&header)) {
        fprintf(stderr, "Unsupported NES console type\n");
        return -1;
    }

    size_t new_prg_size, rom_chr_size;
    int nes2 = is_nes20(&header);
    if (nes2) {
        if (nes20_rom_size(header.prg_rom_chunks, header.flags9 & 0x0F,
                           PRG_ROM_BANK_SIZE, &new_prg_size) < 0
            || nes20_rom_size(header.chr_rom_chunks, header.flags9 >> 4,
                              CHR_ROM_BANK_SIZE, &rom_chr_size) < 0) {
            fprintf(stderr, "NES 2.0 ROM size overflows the address space\n");
            return -1;
        }
    } else {
        new_prg_size = (size_t)header.prg_rom_chunks * PRG_ROM_BANK_SIZE;
        rom_chr_size = (size_t)header.chr_rom_chunks * CHR_ROM_BANK_SIZE;
    }
    if (new_prg_size < PRG_ROM_BANK_SIZE) {
        fprintf(stderr, "Unsupported PRG size: %zu\n", new_prg_size);
        return -1;
    }

    size_t offset = sizeof(header);
    const uint8_t *trainer = NULL;
    if (header.flags6 & 0x04) {
        if (size - offset < 512) {
            fprintf(stderr, "Truncated iNES trainer\n");
            return -1;
        }
        trainer = data + offset;
        offset += 512;
    }
    if (new_prg_size > size - offset || rom_chr_size > size - offset - new_prg_size) {
        fprintf(stderr, "Truncated PRG-ROM or CHR-ROM payload\n");
        return -1;
    }

    size_t new_chr_size = rom_chr_size;
    if (!new_chr_size) {
        RomRamSizes ram;
        rom_ram_sizes(&header, &ram);
        new_chr_size = ram.chr_ram + ram.chr_nvram;
        if (!new_chr_size) {
            fprintf(stderr, "Cartridge declares no CHR-ROM or CHR-RAM\n");
            return -1;
        }
    }

    uint8_t *new_prg = (uint8_t *)malloc(new_prg_size);
    uint8_t *new_chr = (uint8_t *)calloc(1, new_chr_size);
    if (!new_prg || !new_chr) {
        fprintf(stderr, "Cartridge allocation failed\n");
        free(new_prg);
        free(new_chr);
        return -1;
    }
    memcpy(new_prg, data + offset, new_prg_size);
    if (rom_chr_size) memcpy(new_chr, data + offset + new_prg_size, rom_chr_size);

    int mapper_no = mapper_init_from_header(&header, new_prg, new_prg_size,
                                            new_chr, new_chr_size);
    if (mapper_no < 0) {
        free(new_prg);
        free(new_chr);
        return -1;
    }

    // Only replace the active cartridge after all parsing and validation succeeds.
    free(prg_rom);
    free(chr_rom);
    ines_header = header;
    prg_rom = new_prg;
    chr_rom = new_chr;
    prg_size = new_prg_size;
    chr_size = new_chr_size;
    cart_battery_configure(filename, filename && (header.flags6 & 0x02));
    if (trainer) cart_apply_trainer(trainer);
    mirroring_mode = (int)cart_get_mirroring();
    nes_set_region(rom_region(&header));

    printf("Mapper: %d  (CHR %s)\n", mapper_no, rom_chr_size ? "ROM" : "RAM");
    return 0;
}

int load_rom_memory(const uint8_t *data, size_t size) {
    return load_rom_data(data, size, NULL);
}

void unload_rom(void) {
    mapper_shutdown();
    free(prg_rom);
    free(chr_rom);
    prg_rom = chr_rom = NULL;
    prg_size = chr_size = 0;
    memset(&ines_header, 0, sizeof(ines_header));
    mirroring_mode = 0;
    nes_set_region(NES_REGION_NTSC);
}

int load_rom(const char *filename) {
    if (!filename) return -1;
    FILE *fp = fopen(filename, "rb");
    if (!fp) { perror("open"); return -1; }
    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        return -1;
    }
    long length = ftell(fp);
    if (length < 0 || fseek(fp, 0, SEEK_SET) != 0) {
        fclose(fp);
        return -1;
    }
    size_t size = (size_t)length;
    uint8_t *data = (uint8_t *)malloc(size ? size : 1);
    if (!data) { fclose(fp); return -1; }
    size_t bytes_read = fread(data, 1, size, fp);
    fclose(fp);
    if (bytes_read != size) {
        fprintf(stderr, "Failed to read ROM file\n");
        free(data);
        return -1;
    }
    int result = load_rom_data(data, size, filename);
    free(data);
    return result;
}
