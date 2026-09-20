/*
 * game_db.h - Optional NES cartridge metadata database
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#ifndef CUPID_GAME_DB_H
#define CUPID_GAME_DB_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { GAME_DB_TEXT_FIELD = 64, GAME_DB_MAX_FILE_BYTES = 64 * 1024 * 1024 };

typedef struct {
    uint32_t crc32;
    char system[GAME_DB_TEXT_FIELD];
    char board[GAME_DB_TEXT_FIELD];
    char pcb[GAME_DB_TEXT_FIELD];
    char chip[GAME_DB_TEXT_FIELD];
    uint16_t mapper;
    uint32_t prg_rom_size;
    uint32_t chr_rom_size;
    uint32_t chr_ram_size;
    uint32_t work_ram_size;
    uint32_t save_ram_size;
    bool battery;
    char mirroring;
    uint8_t input_type;
    int8_t bus_conflicts; /* -1 = unspecified, 0 = no, 1 = yes */
    bool submapper_present;
    uint8_t submapper;
    uint8_t vs_type;
    uint8_t ppu_model;
} GameDbEntry;

bool game_db_load_file(const char *path);
bool game_db_load_memory(const char *text, size_t size);
void game_db_clear(void);
bool game_db_lookup(uint32_t crc32, GameDbEntry *entry);
size_t game_db_entry_count(void);
uint32_t game_db_crc32(const uint8_t *data, size_t size);

#ifdef __cplusplus
}
#endif
#endif
