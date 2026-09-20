/*
 * game_database.h - Default and explicitly selected cartridge databases
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#ifndef CUPID_UI_GAME_DATABASE_H
#define CUPID_UI_GAME_DATABASE_H

#include "../util/file_io.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FRONTEND_DATABASE_FILENAME "NesDB.txt"

typedef enum {
    FRONTEND_DATABASE_LOADED,
    FRONTEND_DATABASE_ABSENT,
    FRONTEND_DATABASE_ERROR
} FrontendDatabaseOutcome;

typedef struct {
    FrontendDatabaseOutcome outcome;
    NesFileResult file_result;
    bool explicit_path;
    size_t entry_count;
    char path[NES_FILE_PATH_LIMIT + 1];
    char message[256];
} FrontendDatabaseStatus;

// An absent optional default succeeds without replacing an existing database.
// Explicit paths take precedence; malformed or unreadable files preserve the
// active database and do not change the running machine.
bool frontend_database_load(const char *explicit_path, const char *data_directory,
                            FrontendDatabaseStatus *status);

#ifdef __cplusplus
}
#endif
#endif
