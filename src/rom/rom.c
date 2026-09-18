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
#include "board.h"
#include "fds.h"
#include "game_db.h"
#include "../system/timing.h"
#include "../system/hardware.h"
#include "../system/vs_system.h"
#include "../cpu/cpu.h"
#include "../apu/epsm.h"
#include "../joypad/joypad.h"

#define PRG_ROM_BANK_SIZE 0x4000  // 16KB
#define CHR_ROM_BANK_SIZE 0x2000  // 8KB

uint8_t *prg_rom = NULL;
uint8_t *chr_rom = NULL;
iNESHeader ines_header;

size_t prg_size = 0;
size_t chr_size = 0;

int mirroring_mode = 0;
static int fds_loaded = 0;
static int studybox_loaded = 0;
static bool database_overrides = true;
static RomMetadataSource metadata_source = ROM_METADATA_NONE;
static uint32_t loaded_file_crc32 = 0;
static uint32_t loaded_prg_crc32 = 0;
static uint32_t loaded_prg_chr_crc32 = 0;

static int read_file(const char *path, uint8_t **data, size_t *size);
static bool database_header(const iNESHeader *original, const GameDbEntry *entry,
                            bool headerless, iNESHeader *header);
static void database_metadata(const GameDbEntry *entry, const iNESHeader *original,
                              bool headerless,
                              RomDatabaseInfo *metadata);

bool rom_database_load_file(const char *path) { return game_db_load_file(path); }
bool rom_database_load_memory(const char *text, size_t size) { return game_db_load_memory(text, size); }
void rom_database_clear(void) { game_db_clear(); }
void rom_database_set_overrides(bool enabled) { database_overrides = enabled; }
bool rom_database_overrides_enabled(void) { return database_overrides; }
RomMetadataSource rom_metadata_source(void) { return metadata_source; }
uint32_t rom_file_crc32(void) { return loaded_file_crc32; }
uint32_t rom_prg_crc32(void) { return loaded_prg_crc32; }
uint32_t rom_prg_chr_crc32(void) { return loaded_prg_chr_crc32; }

const char *rom_metadata_source_name(void) {
    switch (metadata_source) {
        case ROM_METADATA_INES: return "iNES";
        case ROM_METADATA_NES20: return "NES 2.0";
        case ROM_METADATA_DATABASE: return "game database";
        case ROM_METADATA_DATABASE_HEADERLESS: return "game database (headerless)";
        case ROM_METADATA_FDS: return "FDS";
        case ROM_METADATA_STUDYBOX: return "StudyBox";
        case ROM_METADATA_NONE:
        default: return "none";
    }
}

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
        if (console == 0 || console == 1) return 1;
        // Extended subtypes identify NES/Famicom, VS, and EPSM hardware.
        unsigned subtype = h->zero[2] & 0x0Fu;
        return console == 3 && (subtype <= 1 || subtype == 4 || subtype == 0x0C);
    }
    // Archaic headers have unreliable byte 7 contents.  Only clean iNES headers
    // use its low bits as the VS/PlayChoice console selector.
    if ((h->flags7 & 0x0Cu) == 0)
        return (h->flags7 & 0x03u) <= 1;
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
        if (mapper != 30 && mapper != 111) {
            size_t default_units = mapper == 5 ? 8u : mapper == 69 ? 4u : 1u;
            size_t prg_ram_bytes = default_units * 0x2000;
            if (header->flags6 & 2) sizes->prg_nvram = prg_ram_bytes;
            else sizes->prg_ram = prg_ram_bytes;
        }
        if (!header->chr_rom_chunks)
            sizes->chr_ram = (mapper == 13 || mapper == 111) ? 0x4000
                           : (mapper == 30 || mapper == 96) ? 0x8000 : 0x2000;
    }
    return 0;
}

static int load_rom_data(const uint8_t *data, size_t size, const char *filename) {
    if (!data || !size) {
        fprintf(stderr, "Empty cartridge image\n");
        return -1;
    }

    uint32_t file_crc = game_db_crc32(data, size);
    iNESHeader original_header = {0};
    iNESHeader header = {0};
    GameDbEntry database_entry = {0};
    RomDatabaseInfo database_info = {0};
    const RomDatabaseInfo *database = NULL;
    const uint8_t *trainer = NULL;
    size_t offset = 0;
    bool headerless = false;
    bool database_applied = false;
    RomMetadataSource source = ROM_METADATA_NONE;

    bool has_ines_header = size >= sizeof(header) && memcmp(data, "NES\x1A", 4) == 0;
    if (has_ines_header) {
        memcpy(&original_header, data, sizeof(original_header));
        header = original_header;
        offset = sizeof(header);
        if (header.flags6 & 0x04) {
            if (size - offset < 512) {
                fprintf(stderr, "Truncated iNES trainer\n");
                return -1;
            }
            trainer = data + offset;
            offset += 512;
        }

        uint32_t payload_crc = game_db_crc32(data + offset, size - offset);
        bool nes2_header = is_nes20(&header);
        if (!nes2_header && database_overrides
            && game_db_lookup(payload_crc, &database_entry)) {
            if (!database_header(&original_header, &database_entry, false, &header)) {
                fprintf(stderr, "Game database entry cannot describe this cartridge\n");
                return -1;
            }
            database_metadata(&database_entry, &original_header, false, &database_info);
            database = &database_info;
            database_applied = true;
            source = ROM_METADATA_DATABASE;
        } else {
            source = nes2_header ? ROM_METADATA_NES20 : ROM_METADATA_INES;
        }
    } else {
        if (!game_db_lookup(file_crc, &database_entry)) {
            fprintf(stderr, size < sizeof(header) ? "Unrecognized cartridge image\n"
                                                   : "Invalid iNES signature\n");
            return -1;
        }
        if (!database_header(NULL, &database_entry, true, &header)) {
            fprintf(stderr, "Game database entry cannot describe this headerless cartridge\n");
            return -1;
        }
        database_metadata(&database_entry, NULL, true, &database_info);
        database = &database_info;
        database_applied = true;
        headerless = true;
        source = ROM_METADATA_DATABASE_HEADERLESS;
    }

    if (!rom_console_supported(&header)) {
        fprintf(stderr, "Unsupported NES console type\n");
        return -1;
    }
    if (!cpu_startup_alignment_valid(rom_region(&header))) {
        fprintf(stderr, "Startup alignment is outside this image's regional dividers\n");
        return -1;
    }

    size_t prg_payload_size = 0, rom_chr_size = 0;
    if (database_applied) {
        prg_payload_size = database_entry.prg_rom_size;
        rom_chr_size = database_entry.chr_rom_size;
    } else if (is_nes20(&header)) {
        if (nes20_rom_size(header.prg_rom_chunks, header.flags9 & 0x0F,
                           PRG_ROM_BANK_SIZE, &prg_payload_size) < 0
            || nes20_rom_size(header.chr_rom_chunks, header.flags9 >> 4,
                              CHR_ROM_BANK_SIZE, &rom_chr_size) < 0) {
            fprintf(stderr, "NES 2.0 ROM size overflows the address space\n");
            return -1;
        }
    } else {
        size_t prg_units = header.prg_rom_chunks ? header.prg_rom_chunks : 256u;
        prg_payload_size = prg_units * PRG_ROM_BANK_SIZE;
        rom_chr_size = (size_t)header.chr_rom_chunks * CHR_ROM_BANK_SIZE;
    }
    if (!prg_payload_size) {
        fprintf(stderr, "Invalid PRG size: 0\n");
        return -1;
    }
    if (offset > size || prg_payload_size > size - offset
        || rom_chr_size > size - offset - prg_payload_size) {
        fprintf(stderr, "Truncated PRG-ROM or CHR-ROM payload\n");
        return -1;
    }

    uint32_t prg_chr_crc = game_db_crc32(data + offset, size - offset);
    uint32_t prg_crc = game_db_crc32(data + offset, prg_payload_size);

    size_t new_chr_size = rom_chr_size;
    if (!new_chr_size) {
        RomRamSizes ram;
        rom_ram_sizes(&header, &ram);
        if (database && database->chr_ram_override) ram.chr_ram = database->chr_ram;
        new_chr_size = ram.chr_ram + ram.chr_nvram;
        if (!new_chr_size && !board_handles_header(&header)) {
            fprintf(stderr, "Cartridge declares no CHR-ROM or CHR-RAM\n");
            return -1;
        }
    }

    VsRomConfig vs_config;
    char vs_reason[96];
    int mapper_number = rom_mapper_number(&header);
    if (!vs_decode_header(&header, mapper_number, prg_payload_size, rom_chr_size,
                          &vs_config, vs_reason, sizeof(vs_reason))) {
        fprintf(stderr, "Unsupported VS System configuration: %s\n", vs_reason);
        return -1;
    }

    NesInputConfiguration input_config;
    bool input_supported = false;
    bool apply_input_config = is_nes20(&header) && !vs_config.enabled && header.zero[4] != 0;
    if (apply_input_config) {
        if (!joypad_resolve_default_input(header.zero[4], &input_config, &input_supported)) {
            fprintf(stderr, "Default input conflicts with explicit input configuration\n");
            return -1;
        }
        if (!input_supported) {
            fprintf(stderr, "Unsupported default input type: %u; keeping current input configuration\n",
                    (unsigned)header.zero[4]);
            apply_input_config = false;
        }
    }

    size_t new_prg_size = prg_payload_size < 256u ? 256u : prg_payload_size;
    uint8_t *new_prg = (uint8_t *)malloc(new_prg_size);
    uint8_t *new_chr = (uint8_t *)calloc(1, new_chr_size ? new_chr_size : 1);
    if (!new_prg || !new_chr) {
        fprintf(stderr, "Cartridge allocation failed\n");
        free(new_prg);
        free(new_chr);
        return -1;
    }
    memcpy(new_prg, data + offset, prg_payload_size);
    for (size_t filled = prg_payload_size; filled < new_prg_size;) {
        size_t chunk = prg_payload_size;
        if (chunk > new_prg_size - filled) chunk = new_prg_size - filled;
        memcpy(new_prg + filled, data + offset, chunk);
        filled += chunk;
    }
    if (rom_chr_size) memcpy(new_chr, data + offset + prg_payload_size, rom_chr_size);
    else if (!board_handles_header(&header))
        nes_initialize_power_on_ram(new_chr, new_chr_size, 0);

    if (fds_active() && fds_disk_dirty() && !fds_flush()) {
        fprintf(stderr, "Cannot replace the active FDS disk while modified media is unsaved\n");
        free(new_prg);
        free(new_chr);
        return -1;
    }
    if (apply_input_config && !joypad_persistent_flush()) {
        fprintf(stderr, "Cannot change input configuration while expansion-device data is unsaved\n");
        free(new_prg);
        free(new_chr);
        return -1;
    }

    bool has_epsm = is_nes20(&header) && (header.flags7 & 3u) == 3
                 && (header.zero[2] & 0x0Fu) == 4;
    EpsmDevice *new_epsm = has_epsm ? epsm_create() : NULL;
    if (has_epsm && !new_epsm) {
        fprintf(stderr, "EPSM allocation failed\n");
        free(new_prg);
        free(new_chr);
        return -1;
    }
    int mapper_no = mapper_init_from_header_metadata(&header, new_prg, new_prg_size,
                                                     new_chr, new_chr_size, database);
    if (mapper_no < 0) {
        epsm_destroy(new_epsm);
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
    vs_commit_config(&vs_config);
    epsm_activate(new_epsm);
    if (trainer) cart_apply_trainer(trainer);
    cart_battery_configure(filename, filename && (header.flags6 & 0x02));
    mirroring_mode = (int)cart_get_mirroring();
    nes_set_region(rom_region(&header));
    fds_loaded = 0;
    studybox_loaded = 0;
    metadata_source = source;
    loaded_file_crc32 = file_crc;
    loaded_prg_crc32 = prg_crc;
    loaded_prg_chr_crc32 = prg_chr_crc;
    if (apply_input_config) (void)joypad_apply_configuration(&input_config);

    printf("Mapper: %d  (CHR %s%s)\n", mapper_no, rom_chr_size ? "ROM" : "RAM",
           headerless ? ", headerless" : "");
    return 0;
}

int load_rom_memory(const uint8_t *data, size_t size) {
    return load_rom_data(data, size, NULL);
}

int load_fds_memory(const uint8_t *disk, size_t disk_size,
                    const uint8_t *bios, size_t bios_size,
                    const char *disk_path, bool write_protected) {
    if (!cpu_startup_alignment_valid(NES_REGION_NTSC)) {
        fprintf(stderr, "FDS startup alignment must fit the NTSC dividers\n");
        return -1;
    }
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
    vs_clear_config();
    epsm_activate(NULL);
    mirroring_mode = (int)cart_get_mirroring();
    nes_set_region(NES_REGION_NTSC);
    fds_loaded = 1;
    studybox_loaded = 0;
    metadata_source = ROM_METADATA_FDS;
    loaded_file_crc32 = game_db_crc32(disk, disk_size);
    loaded_prg_crc32 = 0;
    loaded_prg_chr_crc32 = 0;
    printf("Famicom Disk System: %zu side%s\n", fds_side_count(), fds_side_count() == 1 ? "" : "s");
    return 0;
}

static bool database_mirroring(const GameDbEntry *entry, Mirroring *mirroring) {
    if (!entry || !mirroring || !entry->mirroring) return false;
    switch (entry->mirroring) {
        case 'h': *mirroring = MIRROR_HORIZONTAL; return true;
        case 'v': *mirroring = MIRROR_VERTICAL; return true;
        case '0': *mirroring = MIRROR_SINGLE0; return true;
        case '1': *mirroring = MIRROR_SINGLE1; return true;
        case '4': *mirroring = MIRROR_FOUR; return true;
        default: return false;
    }
}

static bool database_vs_input(uint8_t input_type, uint8_t *header_input) {
    if (!header_input) return false;
    switch (input_type) {
        case 0: /* Unspecified uses the VS standard wiring. */
        case 1: /* StandardControllers */
        case 4: /* VsSystem */
            *header_input = VS_INPUT_STANDARD;
            return true;
        case 5: /* VsSystemSwapped */
            *header_input = VS_INPUT_SWAPPED;
            return true;
        case 6: /* VsSystemSwapAB */
            *header_input = VS_INPUT_SWAP_AB;
            return true;
        case 7: /* VsZapper */
            *header_input = VS_INPUT_ZAPPER;
            return true;
        default:
            return false;
    }
}

static bool database_vs_ppu_code(uint8_t ppu_model, uint8_t *header_code) {
    if (!header_code) return false;
    switch (ppu_model) {
        case 0: /* Ppu2C02: closest supported VS behavior is 2C03. */
        case 1: /* Ppu2C03 */
            *header_code = 0;
            return true;
        case 2: case 3: case 4: case 5: /* 2C04 A-D */
            *header_code = ppu_model;
            return true;
        case 6: case 7: case 8: case 9: case 10: /* 2C05 A-E */
            *header_code = (uint8_t)(ppu_model + 2);
            return true;
        default:
            return false;
    }
}

static void database_metadata(const GameDbEntry *entry, const iNESHeader *original,
                              bool headerless,
                              RomDatabaseInfo *metadata) {
    memset(metadata, 0, sizeof(*metadata));
    metadata->present = true;
    metadata->headerless = headerless;
    metadata->mapper = entry->mapper;
    metadata->submapper_present = entry->submapper_present;
    metadata->submapper = entry->submapper;
    metadata->prg_rom_size = entry->prg_rom_size;
    metadata->chr_rom_size = entry->chr_rom_size;
    snprintf(metadata->board, sizeof(metadata->board), "%s", entry->board);
    snprintf(metadata->chip, sizeof(metadata->chip), "%s", entry->chip);
    metadata->bus_conflicts = entry->bus_conflicts;

    iNESHeader inherited_header = {0};
    if (original) {
        inherited_header = *original;
    } else {
        memcpy(inherited_header.signature, "NES\x1A", 4);
        inherited_header.flags6 = (uint8_t)((entry->mapper & 0x0Fu) << 4);
        inherited_header.flags7 = (uint8_t)(entry->mapper & 0xF0u);
        inherited_header.chr_rom_chunks = entry->chr_rom_size ? 1 : 0;
        if (entry->battery) inherited_header.flags6 |= 0x02u;
    }
    RomRamSizes inherited_ram = {0};
    (void)rom_ram_sizes(&inherited_header, &inherited_ram);

    bool validated = entry->submapper_present;
    metadata->work_ram_override = true;
    metadata->save_ram_override = true;
    metadata->chr_ram_override = true;
    metadata->work_ram = validated || entry->work_ram_size
                       ? entry->work_ram_size : inherited_ram.prg_ram;
    metadata->save_ram = validated || entry->save_ram_size
                       ? entry->save_ram_size : inherited_ram.prg_nvram;
    metadata->chr_ram = validated || entry->chr_ram_size
                      ? entry->chr_ram_size : inherited_ram.chr_ram;
    metadata->mirroring_override = database_mirroring(entry, &metadata->mirroring);
}

static bool database_header(const iNESHeader *original, const GameDbEntry *entry,
                            bool headerless, iNESHeader *header) {
    if (!entry || !header || entry->mapper > 0x0FFFu || !entry->prg_rom_size) return false;

    iNESHeader source = {0};
    if (original) source = *original;
    memset(header, 0, sizeof(*header));
    memcpy(header->signature, "NES\x1A", 4);

    /* Database payload sizes are authoritative byte counts. The synthesized
       header only needs to identify ROM presence; exact geometry travels in
       RomDatabaseInfo and the loader's PRG/CHR buffers. */
    header->prg_rom_chunks = entry->prg_rom_size ? 1 : 0;
    header->chr_rom_chunks = entry->chr_rom_size ? 1 : 0;

    header->flags6 = (uint8_t)((entry->mapper & 0x0Fu) << 4);
    if (!headerless && (source.flags6 & 0x04u)) header->flags6 |= 0x04u;

    bool battery = entry->submapper_present ? entry->battery
                 : entry->battery || (!headerless && (source.flags6 & 0x02u));
    if (battery) header->flags6 |= 0x02u;

    Mirroring mirroring;
    if (database_mirroring(entry, &mirroring)) {
        if (mirroring == MIRROR_VERTICAL) header->flags6 |= 0x01u;
        else if (mirroring == MIRROR_FOUR) header->flags6 |= 0x08u;
    } else if (!headerless) {
        header->flags6 |= source.flags6 & 0x09u;
    }

    unsigned console = headerless ? 0u : source.flags7 & 0x03u;
    NesRegion region = headerless ? NES_REGION_NTSC : rom_region(&source);
    uint8_t vs_descriptor = 0;
    uint8_t header_input = entry->input_type;
    if (entry->system[0]) {
        if (strcmp(entry->system, "NesNtsc") == 0 || strcmp(entry->system, "Famicom") == 0) {
            console = 0;
            region = NES_REGION_NTSC;
        } else if (strcmp(entry->system, "NesPal") == 0) {
            console = 0;
            region = NES_REGION_PAL;
        } else if (strcmp(entry->system, "Dendy") == 0) {
            console = 0;
            region = NES_REGION_DENDY;
        } else if (strcmp(entry->system, "VsSystem") == 0) {
            uint8_t ppu_code = 0;
            if (entry->vs_type > VS_TYPE_RAID_ON_BUNGELING_BAY
                || !database_vs_input(entry->input_type, &header_input)
                || !database_vs_ppu_code(entry->ppu_model, &ppu_code))
                return false;
            console = 1;
            region = NES_REGION_NTSC;
            vs_descriptor = (uint8_t)((entry->vs_type << 4) | ppu_code);
        } else {
            return false;
        }
    }

    header->flags7 = (uint8_t)((entry->mapper & 0xF0u) | 0x08u | console);
    header->prg_ram_size = (uint8_t)(((entry->submapper_present ? entry->submapper : 0u) << 4)
                           | ((entry->mapper >> 8) & 0x0Fu));
    header->flags9 = 0;
    header->zero[1] = region == NES_REGION_PAL ? 1u : region == NES_REGION_DENDY ? 3u : 0u;
    header->zero[2] = vs_descriptor;
    header->zero[4] = header_input;
    return true;
}

bool rom_is_fds(void) { return fds_loaded != 0; }
bool rom_is_studybox(void) { return studybox_loaded != 0; }

int load_studybox_memory(const uint8_t *media, size_t media_size,
                         const uint8_t *bios, size_t bios_size) {
    if (!cpu_startup_alignment_valid(NES_REGION_NTSC)) {
        fprintf(stderr, "StudyBox startup alignment must fit the NTSC dividers\n");
        return -1;
    }
    CartridgeBoard *prepared = board_create_studybox(bios, bios_size, media, media_size);
    if (!prepared) return -1;

    if (fds_active() && fds_disk_dirty() && !fds_flush()) {
        fprintf(stderr, "Cannot replace the active FDS disk while modified media is unsaved\n");
        board_destroy(prepared);
        return -1;
    }
    if (mapper_init_studybox(prepared) != 0) {
        board_destroy(prepared);
        return -1;
    }

    free(prg_rom);
    free(chr_rom);
    prg_rom = NULL;
    chr_rom = NULL;
    prg_size = 0;
    chr_size = 0;
    memset(&ines_header, 0, sizeof(ines_header));
    vs_clear_config();
    epsm_activate(NULL);
    mirroring_mode = (int)cart_get_mirroring();
    nes_set_region(NES_REGION_NTSC);
    fds_loaded = 0;
    studybox_loaded = 1;
    metadata_source = ROM_METADATA_STUDYBOX;
    loaded_file_crc32 = game_db_crc32(media, media_size);
    loaded_prg_crc32 = 0;
    loaded_prg_chr_crc32 = 0;
    printf("StudyBox: STBX tape loaded\n");
    return 0;
}

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
    studybox_loaded = 0;
    metadata_source = ROM_METADATA_NONE;
    loaded_file_crc32 = 0;
    loaded_prg_crc32 = 0;
    loaded_prg_chr_crc32 = 0;
    vs_clear_config();
    epsm_activate(NULL);
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

bool rom_set_fcns_kanji_firmware(const char *path) {
    if (!path) return board_set_fcns_kanji_firmware(NULL, 0);
    uint8_t *data = NULL;
    size_t size = 0;
    if (read_file(path, &data, &size) != 0) {
        fprintf(stderr, "Failed to read FCNS Kanji ROM file\n");
        return false;
    }
    bool valid = board_set_fcns_kanji_firmware(data, size);
    if (!valid) fprintf(stderr, "FCNS Kanji ROM must be exactly 256 KiB\n");
    free(data);
    return valid;
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

int load_studybox(const char *media_path, const char *bios_path) {
    if (!media_path || !bios_path) return -1;
    uint8_t *media = NULL, *bios = NULL;
    size_t media_size = 0, bios_size = 0;
    if (read_file(media_path, &media, &media_size) != 0
        || read_file(bios_path, &bios, &bios_size) != 0) {
        fprintf(stderr, "Failed to read StudyBox media or BIOS file\n");
        free(media);
        free(bios);
        return -1;
    }
    int result = load_studybox_memory(media, media_size, bios, bios_size);
    free(media);
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
    int result;
    if (size >= 4 && memcmp(data, "STBX", 4) == 0) {
        fprintf(stderr, "StudyBox media requires a 256 KiB BIOS\n");
        result = -1;
    } else {
        result = load_rom_data(data, size, filename);
    }
    free(data);
    return result;
}
