/*
 * special_peripherals.c - Famicom expansion peripheral hardware
 *
 * Author: @frankischilling
 *
 * This file implements expansion devices whose protocols do not fit the
 * standard controller shift register.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "special_peripherals.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

enum { TURBO_FILE_SIZE = 0x2000, TURBO_FILE_BITS = TURBO_FILE_SIZE * 8 };

typedef struct {
    uint8_t data[TURBO_FILE_SIZE];
    uint16_t position;
    uint8_t last_write;
    bool dirty;
    char *save_path;
} TurboFile;

static TurboFile turbo_file;

typedef struct {
    uint8_t data[512];
    uint8_t last_write;
    uint8_t address;
    bool chip_select;
    bool output;
    bool write_enabled;
    uint8_t input_bit;
    uint16_t input_data;
    bool writing;
    bool reading;
    bool dirty;
    char *save_path;
} BattleBox;

static BattleBox battle_box;

static char *storage_path(const char *rom_path, const char *suffix) {
    if (!rom_path || !*rom_path || !suffix) return NULL;
    const char *slash = strrchr(rom_path, '/');
    const char *backslash = strrchr(rom_path, '\\');
    const char *base = slash;
    if (!base || (backslash && backslash > base)) base = backslash;
    const char *dot = strrchr(rom_path, '.');
    if (dot && base && dot < base) dot = NULL;
    size_t stem = dot ? (size_t)(dot - rom_path) : strlen(rom_path);
    size_t suffix_len = strlen(suffix);
    if (stem > SIZE_MAX - suffix_len - 1) return NULL;
    char *path = malloc(stem + suffix_len + 1);
    if (!path) return NULL;
    memcpy(path, rom_path, stem);
    memcpy(path + stem, suffix, suffix_len + 1);
    return path;
}

static bool atomic_save(const char *path, const uint8_t *data, size_t size) {
    if (!path || !data) return false;
    size_t path_len = strlen(path);
    if (path_len > SIZE_MAX - 40) return false;
    char *temporary = malloc(path_len + 40);
    if (!temporary) return false;
    FILE *file = NULL;
    for (unsigned serial = 0; serial < 1000 && !file; ++serial) {
        snprintf(temporary, path_len + 40, "%s.cupid-%u.tmp", path, serial);
        file = fopen(temporary, "wbx");
        if (!file && errno != EEXIST) break;
    }
    if (!file) {
        free(temporary);
        return false;
    }
    bool ok = fwrite(data, 1, size, file) == size;
    if (fclose(file) != 0) ok = false;
    if (ok) {
#ifdef _WIN32
        ok = MoveFileExA(temporary, path,
                         MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0;
#else
        ok = rename(temporary, path) == 0;
#endif
    }
    if (!ok) remove(temporary);
    free(temporary);
    return ok;
}

static bool load_exact(const char *path, uint8_t *output, size_t size) {
    FILE *file = fopen(path, "rb");
    if (!file) return errno == ENOENT;
    uint8_t *temporary = malloc(size);
    if (!temporary) {
        fclose(file);
        return false;
    }
    bool ok = fread(temporary, 1, size, file) == size && fgetc(file) == EOF && !ferror(file);
    if (fclose(file) != 0) ok = false;
    if (ok) memcpy(output, temporary, size);
    free(temporary);
    return ok;
}

void turbo_file_reset_protocol(void) {
    turbo_file.position = 0;
    turbo_file.last_write = 0;
}

uint8_t turbo_file_read(unsigned port) {
    if (port != 1) return 0;
    unsigned position = turbo_file.position;
    return (uint8_t)(((turbo_file.data[position >> 3] >> (position & 7u)) & 1u) << 2);
}

void turbo_file_write(uint8_t value) {
    if (!(value & 0x02)) turbo_file.position = 0;
    if (!(value & 0x04) && (turbo_file.last_write & 0x04)) {
        unsigned position = turbo_file.position;
        uint8_t mask = (uint8_t)(1u << (position & 7u));
        uint8_t old = turbo_file.data[position >> 3];
        uint8_t next = value & 1u ? (uint8_t)(old | mask) : (uint8_t)(old & (uint8_t)~mask);
        if (next != old) {
            turbo_file.data[position >> 3] = next;
            turbo_file.dirty = true;
        }
        turbo_file.position = (uint16_t)((position + 1u) & (TURBO_FILE_BITS - 1u));
    }
    turbo_file.last_write = value;
}

bool turbo_file_flush(void) {
    if (!turbo_file.dirty) return true;
    if (!turbo_file.save_path || !atomic_save(turbo_file.save_path, turbo_file.data,
                                               sizeof(turbo_file.data))) return false;
    turbo_file.dirty = false;
    return true;
}

bool turbo_file_configure(const char *rom_path) {
    if (!turbo_file_flush()) return false;
    char *path = storage_path(rom_path, ".turbofile.sav");
    if (!path) return false;
    uint8_t loaded[TURBO_FILE_SIZE] = {0};
    if (!load_exact(path, loaded, sizeof(loaded))) {
        free(path);
        return false;
    }
    free(turbo_file.save_path);
    turbo_file.save_path = path;
    memcpy(turbo_file.data, loaded, sizeof(loaded));
    turbo_file.dirty = false;
    turbo_file_reset_protocol();
    return true;
}

bool turbo_file_shutdown(void) {
    if (turbo_file.dirty && !turbo_file.save_path) {
        memset(&turbo_file, 0, sizeof(turbo_file));
        return true;
    }
    if (!turbo_file_flush()) return false;
    free(turbo_file.save_path);
    memset(&turbo_file, 0, sizeof(turbo_file));
    return true;
}

static uint16_t battle_word(unsigned chip, unsigned address) {
    unsigned offset = (chip * 128u + (address & 0x7Fu)) * 2u;
    return (uint16_t)(battle_box.data[offset] | ((uint16_t)battle_box.data[offset + 1] << 8));
}

static void battle_store_word(unsigned chip, unsigned address, uint16_t value) {
    unsigned offset = (chip * 128u + (address & 0x7Fu)) * 2u;
    uint8_t low = (uint8_t)value;
    uint8_t high = (uint8_t)(value >> 8);
    if (battle_box.data[offset] != low || battle_box.data[offset + 1] != high) {
        battle_box.data[offset] = low;
        battle_box.data[offset + 1] = high;
        battle_box.dirty = true;
    }
}

void battle_box_reset_protocol(void) {
    battle_box.last_write = 0;
    battle_box.address = 0;
    battle_box.chip_select = false;
    battle_box.output = false;
    battle_box.write_enabled = false;
    battle_box.input_bit = 0;
    battle_box.input_data = 0;
    battle_box.writing = false;
    battle_box.reading = false;
}

uint8_t battle_box_read(unsigned port) {
    if (port != 1) return 0;
    if (battle_box.last_write & 1u) {
        battle_box.chip_select = !battle_box.chip_select;
        battle_box.input_data = 0;
        battle_box.input_bit = 0;
    }
    battle_box.output = !battle_box.output;
    uint8_t value = battle_box.output ? 0x10 : 0;
    if (battle_box.reading) {
        uint16_t word = battle_word(battle_box.chip_select ? 1u : 0u, battle_box.address);
        value |= (uint8_t)(((word >> battle_box.input_bit) & 1u) << 3);
    }
    return value;
}

static void battle_box_command(uint16_t value) {
    if (battle_box.writing) {
        battle_store_word(battle_box.chip_select ? 1u : 0u, battle_box.address, value);
        battle_box.writing = false;
        return;
    }
    battle_box.reading = false;
    uint8_t address = (uint8_t)(value & 0x7Fu);
    uint8_t command = (uint8_t)(((value & 0x7F00u) >> 8) ^ 0x7Fu);
    switch (command) {
        case 0x01:
            battle_box.address = address;
            battle_box.reading = true;
            break;
        case 0x06:
            if (battle_box.write_enabled) {
                battle_box.address = address;
                battle_box.writing = true;
            }
            break;
        case 0x0C:
            if (battle_box.write_enabled) {
                bool changed = false;
                for (size_t i = 0; i < sizeof(battle_box.data); ++i) {
                    if (battle_box.data[i]) changed = true;
                    battle_box.data[i] = 0;
                }
                battle_box.dirty |= changed;
            }
            break;
        case 0x09:
            battle_box.write_enabled = true;
            break;
        case 0x0B:
            battle_box.write_enabled = false;
            break;
        default:
            break;
    }
}

void battle_box_write(uint8_t value) {
    if (!(battle_box.last_write & 1u) && (value & 1u)) {
        if (battle_box.output)
            battle_box.input_data |= (uint16_t)(1u << battle_box.input_bit);
        if (++battle_box.input_bit == 16) {
            battle_box_command(battle_box.input_data);
            battle_box.input_bit = 0;
            battle_box.input_data = 0;
        }
    }
    battle_box.last_write = value;
}

bool battle_box_flush(void) {
    if (!battle_box.dirty) return true;
    if (!battle_box.save_path || !atomic_save(battle_box.save_path, battle_box.data,
                                               sizeof(battle_box.data))) return false;
    battle_box.dirty = false;
    return true;
}

bool battle_box_configure(const char *rom_path) {
    if (!battle_box_flush()) return false;
    char *path = storage_path(rom_path, ".battlebox.sav");
    if (!path) return false;
    uint8_t loaded[sizeof(battle_box.data)] = {0};
    if (!load_exact(path, loaded, sizeof(loaded))) {
        free(path);
        return false;
    }
    free(battle_box.save_path);
    battle_box.save_path = path;
    memcpy(battle_box.data, loaded, sizeof(loaded));
    battle_box.dirty = false;
    battle_box_reset_protocol();
    return true;
}

bool battle_box_shutdown(void) {
    if (battle_box.dirty && !battle_box.save_path) {
        memset(&battle_box, 0, sizeof(battle_box));
        return true;
    }
    if (!battle_box_flush()) return false;
    free(battle_box.save_path);
    memset(&battle_box, 0, sizeof(battle_box));
    return true;
}
