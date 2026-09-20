/*
 * cheats.h - NES cheat-code parsing, runtime application, and persistence
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_CHEATS_H
#define CUPID_CHEATS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    CHEATS_FRONTEND_COMMAND = 0x1343,
    CHEATS_FRONTEND_PANEL = 0x1383,
    CHEATS_ADD_COMMAND = 0x1370,
    CHEATS_TOGGLE_COMMAND = 0x1371
};

typedef enum {
    CHEAT_FORMAT_GAME_GENIE,
    CHEAT_FORMAT_PAR,
    CHEAT_FORMAT_RAW
} CheatFormat;

typedef struct {
    uint32_t id;
    CheatFormat format;
    uint16_t address;
    uint8_t value;
    uint8_t compare;
    bool has_compare;
    bool enabled;
    char code[32];
    char description[96];
} CheatRecord;

typedef enum {
    CHEAT_OK = 0,
    CHEAT_INVALID_ARGUMENT,
    CHEAT_INVALID_CODE,
    CHEAT_OUT_OF_RANGE,
    CHEAT_LIMIT_REACHED,
    CHEAT_DETERMINISTIC_MODE,
    CHEAT_FILE_ERROR
} CheatResult;

void cheats_init(void);
CheatResult cheats_clear(void);
CheatResult cheats_parse(const char *text, CheatRecord *out);
CheatResult cheats_add(const char *code, const char *description, bool enabled,
                       uint32_t *id_out);
CheatResult cheats_edit(uint32_t id, const char *code, const char *description,
                        bool enabled);
CheatResult cheats_remove(uint32_t id);
CheatResult cheats_move(uint32_t id, size_t new_index);
CheatResult cheats_set_enabled(uint32_t id, bool enabled);
size_t cheats_count(void);
bool cheats_at(size_t index, CheatRecord *out);

void cheats_set_execution_policy(uint32_t policy);
uint32_t cheats_compatibility_hash(void);
uint8_t cheats_apply_read(uint16_t address, uint8_t original);

void cheats_set_game_identity(uint32_t crc32);
uint32_t cheats_game_identity(void);
CheatResult cheats_load_file(const char *path);
CheatResult cheats_save_file(const char *path);
const char *cheats_result_message(CheatResult result);

#ifdef __cplusplus
}
#endif

#endif
