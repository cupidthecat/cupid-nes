/*
 * debug_catalog.c - Transactional symbols and bounded source/reference lookup
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "debug_catalog.h"
#include "opcodes.h"
#include "../rom/rom.h"
#include "../util/file_io.h"
#include "../util/sha1.h"
#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static DebugSymbol symbols[DEBUG_SYMBOL_CAP];
static size_t symbol_count;
static char image_hash[41];

static bool failure(char *error, size_t size, const char *message) {
    if (error && size) {
        snprintf(error, size, "%s", message);
    }
    return false;
}

void debug_catalog_clear(void) {
    symbol_count = 0;
}

void debug_catalog_image_changed(void) {
    debug_catalog_clear();
    if (prg_rom && prg_size) {
        nes_sha1(prg_rom, prg_size, image_hash);
    } else {
        strcpy(image_hash, "0000000000000000000000000000000000000000");
    }
}

size_t debug_symbol_count(void) {
    return symbol_count;
}

bool debug_symbol_at(size_t index, DebugSymbol *out) {
    if (!out || index >= symbol_count) {
        return false;
    }
    *out = symbols[index];
    return true;
}

static bool clean(const char *text, size_t cap) {
    const char *end = memchr(text, 0, cap);
    if (!end) {
        return false;
    }
    for (; text < end; ++text) {
        if ((unsigned char)*text < 32) {
            return false;
        }
    }
    return true;
}

static bool relative_path(const char *path) {
    if (*path == '/' || *path == '\\' || strchr(path, ':') || strchr(path, '\\')) {
        return false;
    }
    const char *part = path;
    while (*part) {
        const char *end = strchr(part, '/');
        size_t len = end ? (size_t)(end - part) : strlen(part);
        if (!len || (len == 2 && !memcmp(part, "..", 2)) || (len == 1 && *part == '.')) {
            return false;
        }
        if (!end) {
            break;
        }
        part = end + 1;
    }
    return true;
}

static bool valid_symbol(const DebugSymbol *s) {
    if (!s || !clean(s->name, sizeof(s->name)) || !*s->name || !clean(s->file, sizeof(s->file)) ||
        !relative_path(s->file) || !clean(s->comment, sizeof(s->comment)) || !s->size ||
        s->size > 65536u - s->address || (!!*s->file != !!s->line)) {
        return false;
    }
    for (const char *p = s->name; *p; ++p) {
        if (!isalnum((unsigned char)*p) && *p != '_' && *p != '.' && *p != '@') {
            return false;
        }
    }
    if (s->key < DEBUG_KEY_CPU) {
        return s->key < prg_size && s->size <= prg_size - s->key;
    }
    return s->key == DEBUG_KEY_CPU + s->address;
}

bool debug_symbol_set(const DebugSymbol *symbol) {
    if (!valid_symbol(symbol)) {
        return false;
    }
    size_t index = symbol_count;
    for (size_t i = 0; i < symbol_count; ++i) {
        if (symbols[i].key == symbol->key) {
            index = i;
        } else if (!strcmp(symbols[i].name, symbol->name)) {
            return false;
        }
    }
    if (index == DEBUG_SYMBOL_CAP) {
        return false;
    }
    symbols[index] = *symbol;
    if (index == symbol_count) {
        ++symbol_count;
    }
    return true;
}

bool debug_symbol_remove(uint64_t key) {
    for (size_t i = 0; i < symbol_count; ++i) {
        if (symbols[i].key == key) {
            memmove(symbols + i, symbols + i + 1, (--symbol_count - i) * sizeof(*symbols));
            return true;
        }
    }
    return false;
}

bool debug_symbol_find(uint64_t key, DebugSymbol *out) {
    const DebugSymbol *best = NULL;
    for (size_t i = 0; i < symbol_count; ++i) {
        if (key >= symbols[i].key && key - symbols[i].key < symbols[i].size && (!best || symbols[i].key > best->key)) {
            best = &symbols[i];
        }
    }
    if (!best) {
        return false;
    }
    if (out) {
        *out = *best;
    }
    return true;
}

bool debug_symbol_resolve(const char *name, uint16_t *address, uint64_t *key) {
    if (!name) {
        return false;
    }
    for (size_t i = 0; i < symbol_count; ++i) {
        if (!strcmp(name, symbols[i].name)) {
            if (address) {
                *address = symbols[i].address;
            }
            if (key) {
                *key = symbols[i].key;
            }
            return true;
        }
    }
    return false;
}

static bool number(const char *text, uint64_t max, uint64_t *out, int base) {
    if (!text || !*text || *text == '-' || *text == '+' || isspace((unsigned char)*text)) {
        return false;
    }
    errno = 0;
    char *end;
    unsigned long long n = strtoull(text, &end, base);
    if (errno || *end || n > max) {
        return false;
    }
    *out = n;
    return true;
}

bool debug_symbol_import(const char *path, char *error, size_t error_size) {
    uint8_t *data = NULL;
    size_t size = 0;
    if (nes_file_read_all(path, 8 * 1024 * 1024, &data, &size) != NES_FILE_OK) {
        return failure(error, error_size, "Cannot read symbol file (8 MiB limit)");
    }
    char *text = malloc(size + 1);
    DebugSymbol *next = calloc(DEBUG_SYMBOL_CAP, sizeof(*next));
    if (!text || !next || memchr(data, 0, size)) {
        free(data);
        free(text);
        free(next);
        return failure(error, error_size, "Invalid symbol text or allocation failure");
    }
    memcpy(text, data, size);
    text[size] = 0;
    free(data);
    char header[64];
    snprintf(header, sizeof(header), "CUPID-SYMBOLS 1 %s", image_hash);
    char *line = strchr(text, '\n');
    bool ok = line != NULL;
    if (line) {
        *line++ = 0;
        size_t n = strlen(text);
        if (n && text[n - 1] == '\r') {
            text[n - 1] = 0;
        }
    }
    ok = ok && !strcmp(text, header);
    size_t count = 0;
    while (ok && line && *line) {
        char *end = strchr(line, '\n');
        if (end) {
            *end++ = 0;
        }
        size_t len = strlen(line);
        if (len && line[len - 1] == '\r') {
            line[--len] = 0;
        }
        if (!len) {
            line = end;
            continue;
        }
        if (count == DEBUG_SYMBOL_CAP) {
            ok = false;
            break;
        }
        char *fields[9] = {line};
        for (unsigned i = 1; i < 9; ++i) {
            char *tab = strchr(fields[i - 1], '\t');
            if (!tab) {
                ok = false;
                break;
            }
            *tab = 0;
            fields[i] = tab + 1;
        }
        if (!ok || strchr(fields[8], '\t')) {
            ok = false;
            break;
        }
        uint64_t key, address, span, source_line, function;
        ok = number(fields[0], DEBUG_KEY_CPU + 65535, &key, 16) && number(fields[1], 65535, &address, 16) &&
             number(fields[2], 65536, &span, 10) && number(fields[3], 1, &function, 10) &&
             number(fields[6], UINT32_MAX, &source_line, 10) && !strcmp(fields[7], "-") &&
             strlen(fields[4]) < sizeof(next[count].name) && strlen(fields[5]) < sizeof(next[count].file) &&
             strlen(fields[8]) < sizeof(next[count].comment);
        if (!ok) {
            break;
        }
        DebugSymbol *s = &next[count];
        s->key = key;
        s->address = (uint16_t)address;
        s->size = (uint32_t)span;
        s->line = (uint32_t)source_line;
        s->function = function != 0;
        strcpy(s->name, fields[4]);
        strcpy(s->file, fields[5]);
        strcpy(s->comment, fields[8]);
        ok = valid_symbol(s);
        for (size_t i = 0; ok && i < count; ++i) {
            if (next[i].key == key || !strcmp(next[i].name, s->name)) {
                ok = false;
            }
        }
        ++count;
        line = end;
    }
    if (ok) {
        memcpy(symbols, next, count * sizeof(*next));
        symbol_count = count;
    }
    free(next);
    free(text);
    if (!ok) {
        return failure(error, error_size, "Invalid, duplicate, or wrong-image symbol record");
    }
    if (error && error_size) {
        *error = 0;
    }
    return true;
}

bool debug_symbol_export(const char *path) {
    NesFileTransaction *tx = NULL;
    if (nes_file_transaction_begin(path, 8 * 1024 * 1024, &tx) != NES_FILE_OK) {
        return false;
    }
    char line[768];
    int n = snprintf(line, sizeof(line), "CUPID-SYMBOLS 1 %s\n", image_hash);
    bool ok = nes_file_transaction_write(tx, line, (size_t)n) == NES_FILE_OK;
    for (size_t i = 0; ok && i < symbol_count; ++i) {
        const DebugSymbol *s = &symbols[i];
        n = snprintf(line, sizeof(line), "%" PRIX64 "\t%04X\t%u\t%u\t%s\t%s\t%u\t-\t%s\n", s->key, s->address, s->size,
                     s->function, s->name, s->file, s->line, s->comment);
        ok = n > 0 && (size_t)n < sizeof(line) && nes_file_transaction_write(tx, line, (size_t)n) == NES_FILE_OK;
    }
    if (!ok) {
        nes_file_transaction_abort(&tx);
        return false;
    }
    return nes_file_transaction_commit(&tx) == NES_FILE_OK;
}

bool debug_source_line(const char *root, const DebugSymbol *symbol, char *out, size_t capacity, char *error,
                       size_t error_size) {
    if (!root || !symbol || !out || !capacity || !valid_symbol(symbol) || !symbol->line) {
        return failure(error, error_size, "No source mapping; use disassembly");
    }
    char path[NES_FILE_PATH_LIMIT];
    int n = snprintf(path, sizeof(path), "%s/%s", *root ? root : ".", symbol->file);
    if (n < 0 || (size_t)n >= sizeof(path)) {
        return failure(error, error_size, "Source path is too long");
    }
    uint8_t *data = NULL;
    size_t size = 0;
    if (nes_file_read_all(path, DEBUG_SOURCE_LIMIT, &data, &size) != NES_FILE_OK) {
        return failure(error, error_size, "Source file missing or exceeds 4 MiB; use disassembly");
    }
    size_t begin = 0;
    uint32_t line = 1;
    while (line < symbol->line && begin < size) {
        if (data[begin++] == '\n') {
            ++line;
        }
    }
    size_t end = begin;
    while (end < size && data[end] != '\n' && data[end] != '\r') {
        ++end;
    }
    bool ok = line == symbol->line && end - begin < capacity && !memchr(data + begin, 0, end - begin);
    if (ok) {
        memcpy(out, data + begin, end - begin);
        out[end - begin] = 0;
    }
    free(data);
    return ok || failure(error, error_size, "Source line absent, binary, or too long");
}

uint32_t debug_source_breakpoint(const DebugSymbol *symbol) {
    if (!valid_symbol(symbol) || !symbol->line) {
        return 0;
    }
    return debugger_add_mapped_breakpoint(DEBUG_BREAK_EXECUTE, symbol->address, symbol->address, symbol->key);
}

size_t debug_references(uint16_t first, uint16_t last, uint64_t physical_start, uint16_t target, bool constants,
                        DebugReference *out, size_t capacity) {
    if (first > last || (!out && capacity)) {
        return 0;
    }
    if (physical_start != DEBUG_KEY_NONE && (!prg_rom || physical_start >= prg_size)) {
        return 0;
    }
    size_t count = 0;
    for (uint32_t pc = first; pc <= last;) {
        uint64_t offset = physical_start + pc - first;
        bool physical = physical_start != DEBUG_KEY_NONE;
        if (physical && (!prg_rom || offset >= prg_size)) {
            break;
        }
        uint8_t bytes[3] = {physical ? prg_rom[offset] : debugger_peek_cpu((uint16_t)pc), 0, 0};
        DebugOpcode op = debugger_opcode(bytes[0]);
        if (op.length > (uint32_t)last - pc + 1 || (physical && op.length > prg_size - offset)) {
            break;
        }
        for (unsigned j = 1; j < op.length; ++j) {
            bytes[j] = physical ? prg_rom[offset + j] : debugger_peek_cpu((uint16_t)(pc + j));
        }
        uint16_t operand = (uint16_t)(bytes[1] | (bytes[2] << 8));
        if (op.mode == REL) {
            operand = (uint16_t)(pc + 2 + (int8_t)bytes[1]);
        }
        bool constant = op.mode == IMM;
        if (op.length > 1 && operand == target && constant == constants) {
            if (count < capacity) {
                DebugReference *r = &out[count];
                *r = (DebugReference){physical ? offset : debug_analysis_key((uint16_t)pc),
                                      (uint16_t)pc,
                                      target,
                                      bytes[0],
                                      op.length,
                                      constant,
                                      op.mode == IND || op.mode == INDX || op.mode == INDY,
                                      ""};
                snprintf(r->instruction, sizeof(r->instruction), "%s %s$%04X%s", op.name, constant ? "#" : "", operand,
                         r->indirect                         ? " (pointer)"
                         : op.mode == ABSX || op.mode == ZPX ? ",X (base)"
                         : op.mode == ABSY || op.mode == ZPY ? ",Y (base)"
                                                             : "");
            }
            ++count;
        }
        pc += op.length;
    }
    return count;
}
