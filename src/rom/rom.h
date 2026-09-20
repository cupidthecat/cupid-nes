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
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "nsf.h"

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

typedef struct {
    bool present;
    bool headerless;
    uint16_t mapper;
    bool submapper_present;
    uint8_t submapper;
    size_t prg_rom_size;
    size_t chr_rom_size;
    char board[64];
    char chip[64];
    int8_t bus_conflicts; /* -1 = board default, 0 = disabled, 1 = enabled */
    bool work_ram_override;
    bool save_ram_override;
    bool chr_ram_override;
    size_t work_ram;
    size_t save_ram;
    size_t chr_ram;
    bool mirroring_override;
    Mirroring mirroring;
} RomDatabaseInfo;

typedef enum {
    ROM_METADATA_NONE,
    ROM_METADATA_INES,
    ROM_METADATA_NES20,
    ROM_METADATA_DATABASE,
    ROM_METADATA_DATABASE_HEADERLESS,
    ROM_METADATA_FDS,
    ROM_METADATA_STUDYBOX,
    ROM_METADATA_UNIF,
    ROM_METADATA_NSF
} RomMetadataSource;

// expose sizes so CPU/PPU can reason about mirroring
extern iNESHeader ines_header;
extern size_t     prg_size;
extern size_t     chr_size;
extern uint8_t   *prg_rom;
extern uint8_t   *chr_rom;

int load_rom(const char *filename);
bool rom_set_fcns_kanji_firmware(const char *path);
typedef enum {
    FDS_SAVE_IN_PLACE,
    FDS_SAVE_OVERLAY
} FdsSaveMode;

typedef struct {
    FdsSaveMode mode;
    const char *overlay_path; // NULL derives an IPS path from the image identity.
    bool write_protected;
} FdsLoadOptions;

int load_fds(const char *disk_path, const char *bios_path, bool write_protected);
int load_fds_with_options(const char *disk_path, const char *bios_path,
                           const FdsLoadOptions *options);
int load_studybox(const char *media_path, const char *bios_path);
// Eject the cartridge and release loader-owned buffers. A failed persistent
// write leaves the active machine loaded so the caller can retry.
bool unload_rom(void);
// Flush cartridge, disk, and input-device data before replacing a session.
// A failure keeps the active machine and any unwritten data available.
bool rom_flush_persistent(void);
// Load an iNES image without a disk file or battery save path; copies its bytes.
// Failed loads preserve the currently inserted cartridge.
int load_rom_memory(const uint8_t *data, size_t size);
// Load prepared cartridge/music bytes with a separate persistence identity.
int load_rom_image(const uint8_t *data, size_t size, const char *save_path);
// Test and embedding entry point. Failed validation leaves the active machine untouched.
int load_fds_memory(const uint8_t *disk, size_t disk_size,
                    const uint8_t *bios, size_t bios_size,
                    const char *disk_path, bool write_protected);
int load_fds_memory_options(const uint8_t *disk, size_t disk_size,
                             const uint8_t *bios, size_t bios_size,
                             const char *disk_path, const FdsLoadOptions *options);
int load_studybox_memory(const uint8_t *media, size_t media_size,
                         const uint8_t *bios, size_t bios_size);
bool rom_is_fds(void);
bool rom_is_studybox(void);
bool rom_is_nsf(void);
bool rom_nsf_select_track(unsigned track);
unsigned rom_nsf_current_track(void);
const NsfMetadata *rom_nsf_metadata(void);
int rom_mapper_number(const iNESHeader *header);
bool rom_database_load_file(const char *path);
bool rom_database_load_memory(const char *text, size_t size);
void rom_database_clear(void);
void rom_database_set_overrides(bool enabled);
bool rom_database_overrides_enabled(void);
RomMetadataSource rom_metadata_source(void);
const char *rom_metadata_source_name(void);
uint32_t rom_file_crc32(void);
uint32_t rom_prg_crc32(void);
uint32_t rom_prg_chr_crc32(void);

typedef struct {
    size_t prg_ram, prg_nvram;
    size_t chr_ram, chr_nvram;
} RomRamSizes;

// Decode declared RAM capacities, including the iNES 8KB PRG-RAM default.
int rom_ram_sizes(const iNESHeader *header, RomRamSizes *sizes);
int rom_ram_sizes_with_metadata(const iNESHeader *header, const RomDatabaseInfo *database,
                               RomRamSizes *sizes);

// mirroring for PPU
extern int mirroring_mode;

#endif
