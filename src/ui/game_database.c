/*
 * game_database.c - Default and explicitly selected cartridge databases
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#include "game_database.h"
#include "../rom/game_db.h"
#include "../rom/rom.h"
#include <stdlib.h>
#include <string.h>

bool frontend_database_load(const char *explicit_path, const char *data_directory,
                            FrontendDatabaseStatus *status) {
    if (!status) return false;
    memset(status, 0, sizeof(*status));
    status->outcome = FRONTEND_DATABASE_ERROR;
    status->file_result = NES_FILE_INVALID_ARGUMENT;
    status->explicit_path = explicit_path != NULL;
    status->entry_count = game_db_entry_count();
    if ((explicit_path && !*explicit_path)
        || (!explicit_path && (!data_directory || !*data_directory))) {
        snprintf(status->message, sizeof(status->message), "No game database path was provided");
        return false;
    }

    int length;
    if (explicit_path) length = snprintf(status->path, sizeof(status->path), "%s", explicit_path);
    else {
        size_t directory_length = strlen(data_directory);
        char last = data_directory[directory_length - 1];
        bool separator = last == '/';
#ifdef _WIN32
        separator = separator || last == '\\';
#endif
        length = snprintf(status->path, sizeof(status->path), "%s%s%s", data_directory,
                          separator ? "" : "/", FRONTEND_DATABASE_FILENAME);
    }
    if (length < 0 || (size_t)length >= sizeof(status->path)) {
        status->path[0] = '\0';
        snprintf(status->message, sizeof(status->message), "Game database path is too long");
        return false;
    }

    uint8_t *data = NULL;
    size_t size = 0;
    status->file_result = nes_file_read_all(status->path, GAME_DB_MAX_FILE_BYTES, &data, &size);
    if (status->file_result == NES_FILE_NOT_FOUND && !status->explicit_path) {
        status->outcome = FRONTEND_DATABASE_ABSENT;
        snprintf(status->message, sizeof(status->message), "Optional default was not found");
        return true;
    }
    if (status->file_result != NES_FILE_OK) {
        snprintf(status->message, sizeof(status->message), "%s", nes_file_result_message(status->file_result));
        return false;
    }
    bool loaded = rom_database_load_memory((const char *)data, size);
    free(data);
    if (!loaded) {
        snprintf(status->message, sizeof(status->message), "Could not parse the game database");
        return false;
    }
    status->outcome = FRONTEND_DATABASE_LOADED;
    status->entry_count = game_db_entry_count();
    snprintf(status->message, sizeof(status->message), "Loaded %zu %s", status->entry_count,
             status->entry_count == 1 ? "entry" : "entries");
    return true;
}
