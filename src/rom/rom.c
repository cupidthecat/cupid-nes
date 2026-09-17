/*
 * rom.c - NES ROM loading and cartridge setup
 *
 * Author: @frankischilling
 *
 * This file parses iNES and NES 2.0 headers, validates ROM sizes and cartridge layouts,
 * loads PRG and CHR data, handles trainers and RAM declarations, selects timing, starts
 * the requested mapper, and manages supported persistent cartridge memory.
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
#include <limits.h>
#include "rom.h"
#include "mapper.h"
#include "fds.h"
#include "../system/timing.h"

#define PRG_ROM_BANK_SIZE 0x4000  // 16KB
#define CHR_ROM_BANK_SIZE 0x2000  // 8KB

uint8_t *prg_rom = NULL;
uint8_t *chr_rom = NULL;
iNESHeader ines_header;

size_t prg_size = 0;
size_t chr_size = 0;

int mirroring_mode = 0;
static int fds_loaded = 0;

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
        int mapper = rom_mapper_number(header);
        // Legacy MMC5 and FME-7 boards default to eight and four 8KB RAM banks.
        // Other iNES boards use the conventional 8KB default. Legacy byte 8 is
        // not reliable enough to override the board default.
        if (mapper != 30) {
            size_t default_units = mapper == 5 ? 8u : mapper == 69 ? 4u : 1u;
            size_t prg_ram_bytes = default_units * 0x2000;
            if (header->flags6 & 2) sizes->prg_nvram = prg_ram_bytes;
            else sizes->prg_ram = prg_ram_bytes;
        }
        if (!header->chr_rom_chunks)
            sizes->chr_ram = mapper == 13 ? 0x4000 : mapper == 30 ? 0x8000 : 0x2000;
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
        size_t prg_units = header.prg_rom_chunks ? header.prg_rom_chunks : 256u;
        new_prg_size = prg_units * PRG_ROM_BANK_SIZE;
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

    if (fds_active() && fds_disk_dirty() && !fds_flush()) {
        fprintf(stderr, "Cannot replace the active FDS disk while modified media is unsaved\n");
        free(new_prg);
        free(new_chr);
        return -1;
    }

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
    fds_loaded = 0;

    printf("Mapper: %d  (CHR %s)\n", mapper_no, rom_chr_size ? "ROM" : "RAM");
    return 0;
}

int load_rom_memory(const uint8_t *data, size_t size) {
    return load_rom_data(data, size, NULL);
}

int load_fds_memory(const uint8_t *disk, size_t disk_size,
                    const uint8_t *bios, size_t bios_size,
                    const char *disk_path, bool write_protected) {
    FdsImage *image = fds_image_create(disk, disk_size, bios, bios_size,
                                       disk_path, write_protected);
    if (!image) {
        fprintf(stderr, "Invalid FDS disk image or BIOS\n");
        return -1;
    }

    if (fds_active() && fds_disk_dirty() && !fds_flush()) {
        fprintf(stderr, "Cannot replace the active FDS disk while modified media is unsaved\n");
        fds_image_destroy(image);
        return -1;
    }

    // The prepared image owns all allocations needed by the new machine, so activation
    // cannot strand the current cartridge after a validation or allocation failure.
    if (mapper_init_fds(image) != 0) {
        fds_image_destroy(image);
        return -1;
    }

    free(prg_rom);
    free(chr_rom);
    prg_rom = NULL;
    chr_rom = NULL;
    prg_size = 0;
    chr_size = 0;
    memset(&ines_header, 0, sizeof(ines_header));
    mirroring_mode = (int)cart_get_mirroring();
    nes_set_region(NES_REGION_NTSC);
    fds_loaded = 1;
    printf("Famicom Disk System: %zu side%s\n", fds_side_count(), fds_side_count() == 1 ? "" : "s");
    return 0;
}

bool rom_is_fds(void) { return fds_loaded != 0; }

bool unload_rom(void) {
    if (fds_active() && fds_disk_dirty() && !fds_flush()) {
        fprintf(stderr, "Cannot unload FDS disk while modified media is unsaved\n");
        return false;
    }
    mapper_shutdown();
    free(prg_rom);
    free(chr_rom);
    prg_rom = chr_rom = NULL;
    prg_size = chr_size = 0;
    memset(&ines_header, 0, sizeof(ines_header));
    mirroring_mode = 0;
    fds_loaded = 0;
    nes_set_region(NES_REGION_NTSC);
    return true;
}

static int read_file(const char *path, uint8_t **data, size_t *size) {
    *data = NULL;
    *size = 0;
    FILE *fp = fopen(path, "rb");
    if (!fp) return -1;
    if (fseek(fp, 0, SEEK_END) != 0) { fclose(fp); return -1; }
    long length = ftell(fp);
    if (length < 0 || fseek(fp, 0, SEEK_SET) != 0) { fclose(fp); return -1; }
    size_t bytes = (size_t)length;
    uint8_t *buffer = (uint8_t *)malloc(bytes ? bytes : 1);
    if (!buffer) { fclose(fp); return -1; }
    size_t bytes_read = fread(buffer, 1, bytes, fp);
    int close_result = fclose(fp);
    if (bytes_read != bytes || close_result != 0) {
        free(buffer);
        return -1;
    }
    *data = buffer;
    *size = bytes;
    return 0;
}

int load_fds(const char *disk_path, const char *bios_path, bool write_protected) {
    if (!disk_path || !bios_path) return -1;
    uint8_t *disk = NULL, *bios = NULL;
    size_t disk_size = 0, bios_size = 0;
    if (read_file(disk_path, &disk, &disk_size) != 0
        || read_file(bios_path, &bios, &bios_size) != 0) {
        fprintf(stderr, "Failed to read FDS disk or BIOS file\n");
        free(disk);
        free(bios);
        return -1;
    }
    int result = load_fds_memory(disk, disk_size, bios, bios_size,
                                 disk_path, write_protected);
    free(disk);
    free(bios);
    return result;
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
