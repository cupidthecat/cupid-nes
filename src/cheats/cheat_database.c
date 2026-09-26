/*
 * cheat_database.c - Bounded catalogs independent of active cheats
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "cheat_database.h"
#include "../rom/rom.h"
#include "../util/file_io.h"
#include "../util/sha1.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { ENTRY_LIMIT = 32768, DATABASE_LIMIT = 16 * 1024 * 1024, CODE_LIMIT = 16 };

struct CheatDatabase {
    CheatDatabaseEntry *entries;
    size_t count;
};

static const CheatDatabaseEntry builtin[] = {
    {"fefa1097449a3a11ebf8c6199e905996c5dc8fbd", "Super Mario Bros. (World)", "Star power", "SSASSA"},
    {"fefa1097449a3a11ebf8c6199e905996c5dc8fbd", "Super Mario Bros. (World)", "Infinite time", "VZTLTN"},
    {"979494e7869ac7ab4815fdbd1dc99f893f713fbf", "Contra (USA)", "Invincibility", "SXKVPZAX"},
    {"979494e7869ac7ab4815fdbd1dc99f893f713fbf", "Contra (USA)", "Blinking invincibility", "SLTIYG"},
    {"2ec08f9341003ded125458df8697ca5ef09d2209", "Mega Man 2 (USA)", "Invincibility", "EIUGVTEY"},
    {"2ec08f9341003ded125458df8697ca5ef09d2209", "Mega Man 2 (USA)", "Infinite health", "SXXTPSSE"}};

static bool fail(char *error, size_t size, const char *message) {
    if (error && size) {
        snprintf(error, size, "%s", message);
    }
    return false;
}

static bool split_codes(const char *text, char codes[CODE_LIMIT][32], size_t *count) {
    *count = 0;
    if (!text || !*text || strlen(text) >= 512) {
        return false;
    }
    const char *start = text;
    for (;;) {
        const char *end = strchr(start, ';');
        size_t length = end ? (size_t)(end - start) : strlen(start);
        if (!length || length >= 32 || *count == CODE_LIMIT) {
            return false;
        }
        memcpy(codes[*count], start, length);
        codes[*count][length] = 0;
        CheatRecord parsed;
        if (cheats_parse(codes[*count], &parsed) != CHEAT_OK) {
            return false;
        }
        ++*count;
        if (!end) {
            return true;
        }
        start = end + 1;
    }
}

CheatDatabase *cheat_database_create(void) {
    return calloc(1, sizeof(CheatDatabase));
}

void cheat_database_destroy(CheatDatabase *database) {
    if (database) {
        free(database->entries);
        free(database);
    }
}

bool cheat_database_parse(CheatDatabase *database, const void *data, size_t length, char *error, size_t size) {
    if (!database || !data || !length || length > DATABASE_LIMIT || memchr(data, 0, length)) {
        return fail(error, size, "Invalid or oversized cheat database");
    }
    char *text = malloc(length + 1);
    if (!text) {
        return fail(error, size, "Cannot allocate cheat database");
    }
    memcpy(text, data, length);
    text[length] = 0;
    CheatDatabase next = {0};
    size_t capacity = 0;
    bool ok = true;
    char *line = text;
    while (*line && ok) {
        char *end = strchr(line, '\n');
        if (end) {
            *end = 0;
        }
        size_t n = strlen(line);
        if (n && line[n - 1] == '\r') {
            line[--n] = 0;
        }
        if (n && line[0] != '#') {
            char *fields[4] = {line};
            for (unsigned i = 1; i < 4; ++i) {
                char *tab = strchr(fields[i - 1], '\t');
                if (!tab) {
                    ok = false;
                    break;
                }
                *tab = 0;
                fields[i] = tab + 1;
            }
            CheatDatabaseEntry entry = {0};
            if (ok) {
                ok = strlen(fields[0]) == 40 && *fields[1] && strlen(fields[1]) < sizeof(entry.game) && *fields[2] &&
                     strlen(fields[2]) < sizeof(entry.description) && strlen(fields[3]) < sizeof(entry.codes) &&
                     !strchr(fields[3], '\t');
                for (unsigned i = 0; ok && i < 40; ++i) {
                    ok = isxdigit((unsigned char)fields[0][i]) != 0;
                    entry.sha1[i] = (char)tolower((unsigned char)fields[0][i]);
                }
                for (unsigned field = 1; ok && field < 4; ++field) {
                    for (const unsigned char *p = (unsigned char *)fields[field]; *p; ++p) {
                        if (*p < 32 || *p == 127) {
                            ok = false;
                            break;
                        }
                    }
                }
                char codes[CODE_LIMIT][32];
                size_t count;
                ok = ok && split_codes(fields[3], codes, &count);
                if (ok) {
                    strcpy(entry.game, fields[1]);
                    strcpy(entry.description, fields[2]);
                    strcpy(entry.codes, fields[3]);
                }
            }
            if (ok && next.count == capacity) {
                size_t grown = capacity ? capacity * 2 : 64;
                if (grown > ENTRY_LIMIT) {
                    ok = false;
                } else {
                    CheatDatabaseEntry *entries = realloc(next.entries, grown * sizeof(*entries));
                    if (!entries) {
                        ok = false;
                    } else {
                        next.entries = entries;
                        capacity = grown;
                    }
                }
            }
            if (ok) {
                next.entries[next.count++] = entry;
            }
        }
        if (!end) {
            break;
        }
        line = end + 1;
    }
    free(text);
    if (!ok) {
        free(next.entries);
        return fail(error, size, "Malformed cheat database row or entry limit exceeded");
    }
    free(database->entries);
    *database = next;
    if (error && size) {
        error[0] = 0;
    }
    return true;
}

bool cheat_database_load(CheatDatabase *database, const char *path, char *error, size_t size) {
    uint8_t *data = NULL;
    size_t length = 0;
    NesFileResult result = nes_file_read_all(path, DATABASE_LIMIT, &data, &length);
    bool ok = result == NES_FILE_OK ? cheat_database_parse(database, data, length, error, size)
                                    : fail(error, size, nes_file_result_message(result));
    free(data);
    return ok;
}

static bool match(const char *a, const char *b) {
    if (!a || !b || strlen(a) != 40 || strlen(b) != 40) {
        return false;
    }
    for (unsigned i = 0; i < 40; ++i) {
        if (tolower((unsigned char)a[i]) != tolower((unsigned char)b[i])) {
            return false;
        }
    }
    return true;
}

bool cheat_database_entry(const CheatDatabase *database, const char *sha1, size_t index, CheatDatabaseEntry *out) {
    for (unsigned source = 0; source < 2; ++source) {
        const CheatDatabaseEntry *entries = source ? (database ? database->entries : NULL) : builtin;
        size_t count = source ? (database ? database->count : 0) : sizeof(builtin) / sizeof(*builtin);
        for (size_t i = 0; i < count; ++i) {
            if (match(entries[i].sha1, sha1)) {
                if (!index--) {
                    if (out) {
                        *out = entries[i];
                    }
                    return true;
                }
            }
        }
    }
    return false;
}

size_t cheat_database_count(const CheatDatabase *database, const char *sha1) {
    size_t count = 0;
    for (size_t i = 0; i < sizeof(builtin) / sizeof(*builtin); ++i) {
        count += match(builtin[i].sha1, sha1);
    }
    if (database) {
        for (size_t i = 0; i < database->count; ++i) {
            count += match(database->entries[i].sha1, sha1);
        }
    }
    return count;
}

bool cheat_database_game_identity(char sha1[41]) {
    if (!sha1 || !prg_rom || !prg_size || rom_is_fds() || rom_is_studybox() || rom_is_nsf()) {
        return false;
    }
    nes_sha1(prg_rom, prg_size, sha1);
    return true;
}

CheatResult cheat_database_add(const CheatDatabaseEntry *entry, const char *sha1, bool enabled) {
    if (!entry || !match(entry->sha1, sha1)) {
        return CHEAT_INVALID_ARGUMENT;
    }
    char codes[CODE_LIMIT][32];
    size_t count;
    if (!split_codes(entry->codes, codes, &count)) {
        return CHEAT_INVALID_CODE;
    }
    const char *pointers[CODE_LIMIT];
    for (size_t i = 0; i < count; ++i) {
        pointers[i] = codes[i];
    }
    return cheats_add_group(pointers, count, entry->description, enabled);
}
