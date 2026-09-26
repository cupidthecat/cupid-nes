/*
 * cheat_database.h - Game-matched cheat catalog
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_CHEAT_DATABASE_H
#define CUPID_CHEAT_DATABASE_H
#include "cheats.h"
typedef struct CheatDatabase CheatDatabase;

typedef struct {
    char sha1[41], game[96], description[96], codes[512];
} CheatDatabaseEntry;

CheatDatabase *cheat_database_create(void);
void cheat_database_destroy(CheatDatabase *database);
/* Transactional tab-separated rows: PRG SHA-1, game, description, codes. */
bool cheat_database_load(CheatDatabase *database, const char *path, char *error, size_t size);
bool cheat_database_parse(CheatDatabase *database, const void *text, size_t length, char *error, size_t size);
size_t cheat_database_count(const CheatDatabase *database, const char *sha1);
bool cheat_database_entry(const CheatDatabase *database, const char *sha1, size_t index, CheatDatabaseEntry *out);
CheatResult cheat_database_add(const CheatDatabaseEntry *entry, const char *sha1, bool enabled);
bool cheat_database_game_identity(char sha1[41]);
#endif
