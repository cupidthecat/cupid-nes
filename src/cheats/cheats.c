/*
 * cheats.c - NES cheat decoding, runtime overrides and per-game storage
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "cheats.h"

#include "../system/execution_policy.h"
#include "../util/file_io.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { CHEAT_LIMIT = 256, CHEAT_FILE_LIMIT = 1024 * 1024 };

static CheatRecord records[CHEAT_LIMIT];
static size_t record_count;
static uint32_t next_id = 1;
static uint32_t game_identity;
static uint32_t policy_mask;

static bool deterministic_mutation_blocked(void) {
    uint32_t policy = policy_mask | nes_execution_policy();
    return (policy & (NES_EXECUTION_MOVIE_RECORDING | NES_EXECUTION_MOVIE_PLAYBACK
                      | NES_EXECUTION_NETPLAY)) != 0;
}

static int hex_digit(int c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool parse_hex(const char *text, size_t length, uint32_t *value) {
    if (!text || !length || !value) return false;
    uint32_t result = 0;
    for (size_t i = 0; i < length; ++i) {
        int digit = hex_digit((unsigned char)text[i]);
        if (digit < 0) return false;
        result = (result << 4) | (uint32_t)digit;
    }
    *value = result;
    return true;
}

static int gg_nibble(int c) {
    static const char alphabet[] = "APZLGITYEOXUKSVN";
    c = toupper((unsigned char)c);
    for (int i = 0; i < 16; ++i) if (alphabet[i] == c) return i;
    return -1;
}

static uint32_t decode_bits(uint32_t raw, const uint8_t *bits, size_t count) {
    uint32_t result = 0;
    for (size_t i = 0; i < count; ++i) result = (result << 1) | ((raw >> bits[i]) & 1u);
    return result;
}

static CheatResult parse_game_genie(const char *text, size_t length, CheatRecord *out) {
    if (length != 6 && length != 8) return CHEAT_INVALID_CODE;
    uint32_t raw = 0;
    for (size_t i = 0; i < length; ++i) {
        int nibble = gg_nibble((unsigned char)text[i]);
        if (nibble < 0) return CHEAT_INVALID_CODE;
        raw |= (uint32_t)nibble << (i * 4u);
    }
    static const uint8_t address_bits[15] = {
        14, 13, 12, 19, 22, 21, 20, 7, 10, 9, 8, 15, 18, 17, 16
    };
    uint8_t value_bits[8] = {3, 6, 5, 4, 23, 2, 1, 0};
    uint8_t compare_bits[8] = {27, 30, 29, 28, 23, 26, 25, 24};
    if (length == 8) value_bits[4] = 31;
    out->format = CHEAT_FORMAT_GAME_GENIE;
    out->address = (uint16_t)(0x8000u + decode_bits(raw, address_bits, 15));
    out->value = (uint8_t)decode_bits(raw, value_bits, 8);
    out->has_compare = length == 8;
    out->compare = out->has_compare ? (uint8_t)decode_bits(raw, compare_bits, 8) : 0;
    for (size_t i = 0; i < length; ++i) out->code[i] = (char)toupper((unsigned char)text[i]);
    out->code[length] = '\0';
    return CHEAT_OK;
}

static CheatResult parse_par(const char *text, size_t length, CheatRecord *out) {
    if (length != 8) return CHEAT_INVALID_CODE;
    uint32_t encoded;
    if (!parse_hex(text, length, &encoded)) return CHEAT_INVALID_CODE;
    static const uint8_t shifts[31] = {
        3, 13, 14, 1, 6, 9, 5, 0, 12, 7, 2, 8, 10, 11, 4,
        19, 21, 23, 22, 20, 17, 16, 18,
        29, 31, 24, 26, 25, 30, 27, 28
    };
    uint32_t key = 0x7E5EE93Au, xor_value = 0x5C184B91u, result = 0;
    encoded >>= 1;
    for (int i = 30; i >= 0; --i) {
        if (((key ^ encoded) >> 30) & 1u) {
            result |= 1u << shifts[i];
            key ^= xor_value;
        }
        encoded <<= 1;
        key <<= 1;
    }
    out->format = CHEAT_FORMAT_PAR;
    out->address = (uint16_t)(0x8000u + (result & 0x7FFFu));
    out->value = (uint8_t)(result >> 24);
    out->compare = (uint8_t)(result >> 16);
    out->has_compare = true;
    for (size_t i = 0; i < length; ++i) out->code[i] = (char)toupper((unsigned char)text[i]);
    out->code[length] = '\0';
    return CHEAT_OK;
}

static CheatResult parse_raw(const char *text, size_t length, CheatRecord *out) {
    if (length != 7 && length != 10) return CHEAT_INVALID_CODE;
    if (text[4] != ':' || (length == 10 && text[7] != ':')) return CHEAT_INVALID_CODE;
    uint32_t address, value, compare = 0;
    if (!parse_hex(text, 4, &address) || !parse_hex(text + 5, 2, &value)) return CHEAT_INVALID_CODE;
    if (length == 10 && !parse_hex(text + 8, 2, &compare)) return CHEAT_INVALID_CODE;
    out->format = CHEAT_FORMAT_RAW;
    out->address = (uint16_t)address;
    out->value = (uint8_t)value;
    out->compare = (uint8_t)compare;
    out->has_compare = length == 10;
    if (out->has_compare)
        snprintf(out->code, sizeof(out->code), "%04X:%02X:%02X", (unsigned)address,
                 (unsigned)value, (unsigned)compare);
    else
        snprintf(out->code, sizeof(out->code), "%04X:%02X", (unsigned)address, (unsigned)value);
    return CHEAT_OK;
}

void cheats_init(void) {
    memset(records, 0, sizeof(records));
    record_count = 0;
    next_id = 1;
    game_identity = 0;
    policy_mask = 0;
}

CheatResult cheats_clear(void) {
    if (deterministic_mutation_blocked()) return CHEAT_DETERMINISTIC_MODE;
    record_count = 0;
    return CHEAT_OK;
}

CheatResult cheats_parse(const char *text, CheatRecord *out) {
    if (!text || !out) return CHEAT_INVALID_ARGUMENT;
    size_t length = strlen(text);
    if (!length || length >= sizeof(out->code)) return CHEAT_INVALID_CODE;
    memset(out, 0, sizeof(*out));
    CheatResult result;
    if (length == 6) result = parse_game_genie(text, length, out);
    else if (length == 8) {
        result = parse_game_genie(text, length, out);
        if (result != CHEAT_OK) result = parse_par(text, length, out);
    } else if (length == 7 || length == 10) result = parse_raw(text, length, out);
    else result = CHEAT_INVALID_CODE;
    if (result == CHEAT_OK) out->enabled = true;
    return result;
}

CheatResult cheats_add(const char *code, const char *description, bool enabled,
                       uint32_t *id_out) {
    if (deterministic_mutation_blocked()) return CHEAT_DETERMINISTIC_MODE;
    if (record_count >= CHEAT_LIMIT) return CHEAT_LIMIT_REACHED;
    CheatRecord parsed;
    CheatResult result = cheats_parse(code, &parsed);
    if (result != CHEAT_OK) return result;
    parsed.id = next_id++;
    parsed.enabled = enabled;
    snprintf(parsed.description, sizeof(parsed.description), "%s", description ? description : "");
    records[record_count++] = parsed;
    if (id_out) *id_out = parsed.id;
    return CHEAT_OK;
}

static CheatRecord *find_record(uint32_t id, size_t *index) {
    for (size_t i = 0; i < record_count; ++i) {
        if (records[i].id != id) continue;
        if (index) *index = i;
        return &records[i];
    }
    return NULL;
}

CheatResult cheats_edit(uint32_t id, const char *code, const char *description,
                        bool enabled) {
    if (deterministic_mutation_blocked()) return CHEAT_DETERMINISTIC_MODE;
    CheatRecord *current = find_record(id, NULL);
    if (!current || !code) return CHEAT_INVALID_ARGUMENT;
    CheatRecord parsed;
    CheatResult result = cheats_parse(code, &parsed);
    if (result != CHEAT_OK) return result;
    parsed.id = id;
    parsed.enabled = enabled;
    snprintf(parsed.description, sizeof(parsed.description), "%s", description ? description : "");
    *current = parsed;
    return CHEAT_OK;
}

CheatResult cheats_remove(uint32_t id) {
    if (deterministic_mutation_blocked()) return CHEAT_DETERMINISTIC_MODE;
    size_t index;
    if (!find_record(id, &index)) return CHEAT_INVALID_ARGUMENT;
    memmove(&records[index], &records[index + 1], (record_count - index - 1) * sizeof(records[0]));
    --record_count;
    return CHEAT_OK;
}

CheatResult cheats_move(uint32_t id, size_t new_index) {
    if (deterministic_mutation_blocked()) return CHEAT_DETERMINISTIC_MODE;
    if (new_index >= record_count) return CHEAT_OUT_OF_RANGE;
    size_t old_index;
    CheatRecord *record = find_record(id, &old_index);
    if (!record) return CHEAT_INVALID_ARGUMENT;
    if (old_index == new_index) return CHEAT_OK;
    CheatRecord copy = *record;
    if (old_index < new_index)
        memmove(&records[old_index], &records[old_index + 1], (new_index - old_index) * sizeof(records[0]));
    else
        memmove(&records[new_index + 1], &records[new_index], (old_index - new_index) * sizeof(records[0]));
    records[new_index] = copy;
    return CHEAT_OK;
}

CheatResult cheats_set_enabled(uint32_t id, bool enabled) {
    if (deterministic_mutation_blocked()) return CHEAT_DETERMINISTIC_MODE;
    CheatRecord *record = find_record(id, NULL);
    if (!record) return CHEAT_INVALID_ARGUMENT;
    record->enabled = enabled;
    return CHEAT_OK;
}

size_t cheats_count(void) { return record_count; }
bool cheats_at(size_t index, CheatRecord *out) {
    if (!out || index >= record_count) return false;
    *out = records[index];
    return true;
}

void cheats_set_execution_policy(uint32_t policy) { policy_mask = policy; }

static uint32_t hash_byte(uint32_t hash, uint8_t value) {
    return (hash ^ value) * 16777619u;
}

uint32_t cheats_compatibility_hash(void) {
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < record_count; ++i) {
        const CheatRecord *record = &records[i];
        if (!record->enabled) continue;
        hash = hash_byte(hash, (uint8_t)record->format);
        hash = hash_byte(hash, (uint8_t)record->address);
        hash = hash_byte(hash, (uint8_t)(record->address >> 8));
        hash = hash_byte(hash, record->value);
        hash = hash_byte(hash, record->has_compare ? 1u : 0u);
        if (record->has_compare) hash = hash_byte(hash, record->compare);
    }
    return hash;
}

uint8_t cheats_apply_read(uint16_t address, uint8_t original) {
    uint8_t result = original;
    for (size_t i = 0; i < record_count; ++i) {
        const CheatRecord *record = &records[i];
        if (!record->enabled || record->address != address) continue;
        if (record->has_compare && record->compare != original) continue;
        result = record->value;
    }
    return result;
}

void cheats_set_game_identity(uint32_t crc32) {
    if (game_identity == crc32) return;
    game_identity = crc32;
    record_count = 0;
    next_id = 1;
}
uint32_t cheats_game_identity(void) { return game_identity; }

static bool valid_utf8(const uint8_t *data, size_t size) {
    for (size_t i = 0; i < size;) {
        uint8_t c = data[i++];
        if (!c) return false;
        if (c < 0x80) continue;
        unsigned extra; uint32_t code;
        if ((c & 0xE0) == 0xC0) { extra = 1; code = c & 0x1F; if (code < 2) return false; }
        else if ((c & 0xF0) == 0xE0) { extra = 2; code = c & 0x0F; }
        else if ((c & 0xF8) == 0xF0) { extra = 3; code = c & 0x07; }
        else return false;
        if (i + extra > size) return false;
        for (unsigned j = 0; j < extra; ++j) {
            uint8_t tail = data[i++];
            if ((tail & 0xC0) != 0x80) return false;
            code = (code << 6) | (tail & 0x3F);
        }
        if ((extra == 2 && code < 0x800) || (extra == 3 && code < 0x10000)
            || code > 0x10FFFF || (code >= 0xD800 && code <= 0xDFFF)) return false;
    }
    return true;
}

static size_t escaped_size(const char *text) {
    size_t size = 0;
    for (const unsigned char *p = (const unsigned char *)text; *p; ++p)
        size += (*p == '\\' || *p == '\t' || *p == '\n' || *p == '\r') ? 2 : 1;
    return size;
}

static char *write_escaped(char *out, const char *text) {
    for (const unsigned char *p = (const unsigned char *)text; *p; ++p) {
        if (*p == '\\') { *out++ = '\\'; *out++ = '\\'; }
        else if (*p == '\t') { *out++ = '\\'; *out++ = 't'; }
        else if (*p == '\n') { *out++ = '\\'; *out++ = 'n'; }
        else if (*p == '\r') { *out++ = '\\'; *out++ = 'r'; }
        else *out++ = (char)*p;
    }
    return out;
}

static bool unescape_description(const char *text, char out[96]) {
    size_t used = 0;
    for (size_t i = 0; text[i]; ++i) {
        unsigned char c = (unsigned char)text[i];
        if (c == '\\') {
            c = (unsigned char)text[++i];
            if (!c) return false;
            if (c == 't') c = '\t'; else if (c == 'n') c = '\n'; else if (c == 'r') c = '\r';
            else if (c != '\\') return false;
        }
        if (used + 1 >= 96) return false;
        out[used++] = (char)c;
    }
    out[used] = '\0';
    return true;
}

CheatResult cheats_save_file(const char *path) {
    if (!path || !*path) return CHEAT_INVALID_ARGUMENT;
    size_t size = 32;
    for (size_t i = 0; i < record_count; ++i)
        size += 3 + strlen(records[i].code) + escaped_size(records[i].description);
    char *buffer = malloc(size + 1);
    if (!buffer) return CHEAT_FILE_ERROR;
    char *cursor = buffer;
    int written = snprintf(cursor, size + 1, "CUPID-CHEATS\t1\t%08X\n", game_identity);
    if (written < 0) { free(buffer); return CHEAT_FILE_ERROR; }
    cursor += (size_t)written;
    for (size_t i = 0; i < record_count; ++i) {
        const CheatRecord *record = &records[i];
        *cursor++ = record->enabled ? '1' : '0'; *cursor++ = '\t';
        size_t code_length = strlen(record->code);
        memcpy(cursor, record->code, code_length); cursor += code_length; *cursor++ = '\t';
        cursor = write_escaped(cursor, record->description); *cursor++ = '\n';
    }
    size_t bytes = (size_t)(cursor - buffer);
    NesFileResult file_result = nes_file_write_atomic(path, buffer, bytes);
    free(buffer);
    return file_result == NES_FILE_OK ? CHEAT_OK : CHEAT_FILE_ERROR;
}

static bool parse_header(char *line, uint32_t *identity) {
    static const char prefix[] = "CUPID-CHEATS\t1\t";
    if (strncmp(line, prefix, sizeof(prefix) - 1) != 0) return false;
    const char *hex = line + sizeof(prefix) - 1;
    return strlen(hex) == 8 && parse_hex(hex, 8, identity);
}

CheatResult cheats_load_file(const char *path) {
    if (!path || !*path) return CHEAT_INVALID_ARGUMENT;
    if (deterministic_mutation_blocked()) return CHEAT_DETERMINISTIC_MODE;
    uint8_t *data = NULL; size_t size = 0;
    NesFileResult file_result = nes_file_read_all(path, CHEAT_FILE_LIMIT, &data, &size);
    if (file_result != NES_FILE_OK) return CHEAT_FILE_ERROR;
    if (!valid_utf8(data, size)) { free(data); return CHEAT_INVALID_CODE; }
    char *text = malloc(size + 1);
    if (!text) { free(data); return CHEAT_FILE_ERROR; }
    memcpy(text, data, size); text[size] = '\0'; free(data);

    CheatRecord parsed[CHEAT_LIMIT]; size_t parsed_count = 0; uint32_t parsed_identity = 0;
    char *cursor = text; char *newline = strchr(cursor, '\n');
    if (!newline) { free(text); return CHEAT_INVALID_CODE; }
    *newline = '\0'; if (newline > cursor && newline[-1] == '\r') newline[-1] = '\0';
    if (!parse_header(cursor, &parsed_identity) || parsed_identity != game_identity) {
        free(text); return CHEAT_INVALID_CODE;
    }
    cursor = newline + 1;
    while (*cursor) {
        newline = strchr(cursor, '\n');
        if (newline) *newline = '\0';
        size_t line_length = strlen(cursor);
        if (line_length && cursor[line_length - 1] == '\r') cursor[--line_length] = '\0';
        if (!line_length) { cursor = newline ? newline + 1 : cursor + line_length; if (!newline) break; continue; }
        if (parsed_count >= CHEAT_LIMIT || (cursor[0] != '0' && cursor[0] != '1') || cursor[1] != '\t') {
            free(text); return CHEAT_INVALID_CODE;
        }
        char *code = cursor + 2; char *tab = strchr(code, '\t');
        if (!tab) { free(text); return CHEAT_INVALID_CODE; }
        *tab = '\0';
        CheatRecord record;
        if (cheats_parse(code, &record) != CHEAT_OK || !unescape_description(tab + 1, record.description)) {
            free(text); return CHEAT_INVALID_CODE;
        }
        record.id = (uint32_t)(parsed_count + 1); record.enabled = cursor[0] == '1';
        parsed[parsed_count++] = record;
        if (!newline) break;
        cursor = newline + 1;
    }
    memcpy(records, parsed, parsed_count * sizeof(records[0]));
    record_count = parsed_count; next_id = (uint32_t)parsed_count + 1;
    free(text);
    return CHEAT_OK;
}

const char *cheats_result_message(CheatResult result) {
    switch (result) {
        case CHEAT_OK: return "OK";
        case CHEAT_INVALID_ARGUMENT: return "Invalid argument";
        case CHEAT_INVALID_CODE: return "Invalid cheat code or cheat file";
        case CHEAT_OUT_OF_RANGE: return "Value is outside the supported range";
        case CHEAT_LIMIT_REACHED: return "Cheat list is full";
        case CHEAT_DETERMINISTIC_MODE: return "Cheat changes are disabled during movie or netplay execution";
        case CHEAT_FILE_ERROR: return "Could not read or write the cheat file";
        default: return "Unknown cheat error";
    }
}
