/*
 * debug_capture.c - Observational trace history and table-driven text capture
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "debug_capture.h"
#include "debug_catalog.h"
#include "expression.h"
#include "opcodes.h"
#include "../system/execution_policy.h"
#include "../util/file_io.h"
#include <ctype.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { TRACE_MAX = 1048576, TEXT_CAP = 1024 * 1024, TABLE_CAP = 1024, LINE_CAP = 1024 };

typedef struct {
    uint8_t bytes[8], length;
    char text[64];
} TextEntry;

static struct {
    DebugLogEntry *entries;
    size_t capacity, head, count;
    uint64_t overwritten;
    bool enabled;
    bool pending, executed;
    uint8_t fetch_mask;
    DebugLogEntry next;
    DebugTraceFilter filter;
} log_state;

static struct {
    TextEntry table[TABLE_CAP];
    size_t table_count;
    char output[TEXT_CAP], line[LINE_CAP], previous[LINE_CAP];
    size_t size, line_size;
    uint8_t pending[8];
    size_t pending_size;
    uint16_t first, last;
    bool enabled, reads, writes, deduplicate;
    uint64_t dropped;
} text_state;

static bool fail(char *error, size_t size, const char *message) {
    if (error && size) {
        snprintf(error, size, "%s", message);
    }
    return false;
}

void debug_capture_shutdown(void) {
    free(log_state.entries);
    memset(&log_state, 0, sizeof(log_state));
    memset(&text_state, 0, sizeof(text_state));
}

void debug_capture_discontinuity(void) {
    log_state.pending = false;
    text_state.pending_size = text_state.line_size = 0;
}

bool debug_log_configure(size_t capacity, const DebugTraceFilter *filter, char *error, size_t size) {
    if (!filter || !capacity || capacity > TRACE_MAX || filter->first > filter->last ||
        (filter->columns & ~DEBUG_TRACE_ALL) || !memchr(filter->condition, 0, sizeof(filter->condition))) {
        return fail(error, size, "Invalid trace capacity, range, columns or condition");
    }
    uint32_t result;
    if (*filter->condition && !debug_expression_eval(filter->condition, &result, error, size)) {
        return false;
    }
    DebugLogEntry *entries = calloc(capacity, sizeof(*entries));
    if (!entries) {
        return fail(error, size, "Cannot allocate trace history");
    }
    size_t keep = log_state.count < capacity ? log_state.count : capacity;
    for (size_t i = 0; i < keep; ++i) {
        debug_log_at(log_state.count - keep + i, &entries[i]);
    }
    free(log_state.entries);
    log_state.entries = entries;
    log_state.overwritten += log_state.count - keep;
    log_state.count = keep;
    log_state.capacity = capacity;
    log_state.head = keep % capacity;
    log_state.filter = *filter;
    return true;
}

void debug_log_enable(bool enabled) {
    if (enabled && !log_state.capacity) {
        DebugTraceFilter filter = {0, 65535, "", DEBUG_TRACE_ALL};
        if (!debug_log_configure(65536, &filter, NULL, 0)) {
            return;
        }
    }
    log_state.enabled = enabled;
}

bool debug_log_enabled(void) {
    return log_state.enabled;
}

void debug_log_clear(void) {
    log_state.head = log_state.count = 0;
    log_state.overwritten = 0;
}

size_t debug_log_count(void) {
    return log_state.count;
}

uint64_t debug_log_overwritten(void) {
    return log_state.overwritten;
}

bool debug_log_at(size_t index, DebugLogEntry *out) {
    if (!out || index >= log_state.count) {
        return false;
    }
    size_t oldest = (log_state.head + log_state.capacity - log_state.count) % log_state.capacity;
    *out = log_state.entries[(oldest + index) % log_state.capacity];
    return true;
}

void debug_log_instruction(const CPU *state) {
    log_state.pending = false;
    if (!state || state->halted || !log_state.enabled || nes_execution_policy() != NES_EXECUTION_LIVE ||
        state->pc < log_state.filter.first || state->pc > log_state.filter.last) {
        return;
    }
    uint32_t condition = 1;
    if (*log_state.filter.condition &&
        (!debug_expression_eval(log_state.filter.condition, &condition, NULL, 0) || !condition)) {
        return;
    }
    DebugLogEntry *entry = &log_state.next;
    *entry = (DebugLogEntry){0};
    entry->cpu =
        (DebugTraceEntry){cpu_get_bus_cycle(), state->pc, 0, state->a, state->x, state->y, state->sp, state->status};
    entry->key = debug_analysis_key(state->pc);
    entry->length = 1;
    log_state.fetch_mask = 0;
    log_state.executed = false;
    log_state.pending = true;
}

void debug_log_read(uint16_t address, uint8_t value) {
    if (!log_state.pending || !log_state.executed) {
        return;
    }
    uint16_t delta = (uint16_t)(address - log_state.next.cpu.pc);
    if (delta >= log_state.next.length || (log_state.fetch_mask & (1u << delta))) {
        return;
    }
    log_state.next.bytes[delta] = value;
    log_state.fetch_mask |= (uint8_t)(1u << delta);
    if (!delta) {
        log_state.next.cpu.opcode = value;
        log_state.next.length = debugger_opcode(value).length;
    }
}

void debug_log_opcode(uint8_t opcode) {
    if (!log_state.pending) {
        return;
    }
    log_state.executed = true;
    log_state.next.cpu.opcode = log_state.next.bytes[0] = opcode;
    log_state.next.length = debugger_opcode(opcode).length;
    log_state.fetch_mask = 1;
}

void debug_log_end(void) {
    if (!log_state.pending) {
        return;
    }
    log_state.pending = false;
    if (!log_state.executed) {
        return;
    }
    log_state.entries[log_state.head] = log_state.next;
    log_state.head = (log_state.head + 1) % log_state.capacity;
    if (log_state.count < log_state.capacity) {
        ++log_state.count;
    } else {
        ++log_state.overwritten;
    }
}

static bool append(char *out, size_t cap, size_t *used, const char *format, ...) {
    if (*used >= cap) {
        return false;
    }
    va_list args;
    va_start(args, format);
    int n = vsnprintf(out + *used, cap - *used, format, args);
    va_end(args);
    if (n < 0 || (size_t)n >= cap - *used) {
        return false;
    }
    *used += (size_t)n;
    return true;
}

bool debug_log_format(const DebugLogEntry *entry, char *out, size_t capacity) {
    if (!entry || !out || !capacity) {
        return false;
    }
    size_t used = 0;
    unsigned columns = log_state.filter.columns;
    bool ok = append(out, capacity, &used, "%04X [%" PRIX64 "] %s", entry->cpu.pc, entry->key,
                     debugger_opcode(entry->cpu.opcode).name);
    if (columns & DEBUG_TRACE_BYTES) {
        for (unsigned i = 0; ok && i < entry->length; ++i) {
            ok = append(out, capacity, &used, " %02X", entry->bytes[i]);
        }
    }
    if (columns & DEBUG_TRACE_REGISTERS) {
        ok = ok && append(out, capacity, &used, " A:%02X X:%02X Y:%02X P:%02X SP:%02X", entry->cpu.a, entry->cpu.x,
                          entry->cpu.y, entry->cpu.status, entry->cpu.sp);
    }
    if (columns & DEBUG_TRACE_CYCLES) {
        ok = ok && append(out, capacity, &used, " CYC:%" PRIu64, entry->cpu.cpu_cycle);
    }
    DebugSymbol symbol;
    if ((columns & DEBUG_TRACE_SYMBOLS) && debug_symbol_find(entry->key, &symbol)) {
        ok = ok && append(out, capacity, &used, " %s+%" PRIX64, symbol.name, entry->key - symbol.key);
    }
    return ok;
}

bool debug_log_export(const char *path) {
    NesFileTransaction *tx = NULL;
    if (nes_file_transaction_begin(path, (uint64_t)TRACE_MAX * 512, &tx) != NES_FILE_OK) {
        return false;
    }
    for (size_t i = 0; i < log_state.count; ++i) {
        DebugLogEntry entry;
        char line[512];
        if (!debug_log_at(i, &entry) || !debug_log_format(&entry, line, sizeof(line) - 2)) {
            nes_file_transaction_abort(&tx);
            return false;
        }
        strcat(line, "\n");
        if (nes_file_transaction_write(tx, line, strlen(line)) != NES_FILE_OK) {
            nes_file_transaction_abort(&tx);
            return false;
        }
    }
    return nes_file_transaction_commit(&tx) == NES_FILE_OK;
}

static int hex_digit(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

static bool utf8(const char *text) {
    const unsigned char *p = (const unsigned char *)text;
    while (*p) {
        uint32_t cp = *p++;
        unsigned n = 0;
        uint32_t min = 0;
        if (cp < 128) {
            if (cp < 32 && cp != '\n') {
                return false;
            }
            continue;
        }
        if ((cp & 0xE0) == 0xC0) {
            n = 1;
            cp &= 31;
            min = 128;
        } else if ((cp & 0xF0) == 0xE0) {
            n = 2;
            cp &= 15;
            min = 2048;
        } else if ((cp & 0xF8) == 0xF0) {
            n = 3;
            cp &= 7;
            min = 65536;
        } else {
            return false;
        }
        for (unsigned i = 0; i < n; ++i) {
            if ((*p & 0xC0) != 0x80) {
                return false;
            }
            cp = (cp << 6) | (*p++ & 63);
        }
        if (cp < min || cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF)) {
            return false;
        }
    }
    return true;
}

bool debug_text_table(const char *text, size_t size, char *error, size_t error_size) {
    if (!text || size > 128 * 1024 || memchr(text, 0, size)) {
        return fail(error, error_size, "Invalid table size or embedded NUL");
    }
    TextEntry *table = calloc(TABLE_CAP, sizeof(*table));
    if (!table) {
        return fail(error, error_size, "Cannot allocate text table");
    }
    size_t count = 0, pos = 0;
    bool ok = true;
    while (pos < size && ok) {
        size_t end = pos;
        while (end < size && text[end] != '\n') {
            ++end;
        }
        size_t len = end - pos;
        if (len && text[pos + len - 1] == '\r') {
            --len;
        }
        if (!len || text[pos] == ';') {
            pos = end + 1;
            continue;
        }
        const char *eq = memchr(text + pos, '=', len);
        size_t hex = eq ? (size_t)(eq - (text + pos)) : 0;
        if (!hex || hex > 16 || (hex & 1) || count == TABLE_CAP || len - hex - 1 >= 64) {
            ok = false;
            break;
        }
        TextEntry *entry = &table[count];
        entry->length = (uint8_t)(hex / 2);
        for (size_t i = 0; i < hex; i += 2) {
            int a = hex_digit(text[pos + i]), b = hex_digit(text[pos + i + 1]);
            if (a < 0 || b < 0) {
                ok = false;
                break;
            }
            entry->bytes[i / 2] = (uint8_t)((a << 4) | b);
        }
        memcpy(entry->text, eq + 1, len - hex - 1);
        if (!strcmp(entry->text, "\\n")) {
            strcpy(entry->text, "\n");
        }
        if (!utf8(entry->text)) {
            ok = false;
        }
        for (size_t i = 0; ok && i < count; ++i) {
            size_t min = entry->length < table[i].length ? entry->length : table[i].length;
            if (!memcmp(entry->bytes, table[i].bytes, min)) {
                ok = false;
            }
        }
        ++count;
        pos = end + 1;
    }
    if (ok && count) {
        memcpy(text_state.table, table, count * sizeof(*table));
        text_state.table_count = count;
        text_state.pending_size = text_state.line_size = 0;
    } else {
        ok = false;
    }
    free(table);
    return ok || fail(error, error_size, "Invalid UTF-8, duplicate/prefix key, or table bound exceeded");
}

void debug_text_configure(uint16_t first, uint16_t last, bool reads, bool writes, bool deduplicate) {
    if (first > last) {
        return;
    }
    text_state.first = first;
    text_state.last = last;
    text_state.reads = reads;
    text_state.writes = writes;
    text_state.deduplicate = deduplicate;
    text_state.pending_size = 0;
}

void debug_text_enable(bool enabled) {
    text_state.enabled = enabled;
    if (!enabled) {
        debug_text_flush();
    }
}

bool debug_text_enabled(void) {
    return text_state.enabled;
}

void debug_text_clear(void) {
    text_state.size = text_state.line_size = text_state.pending_size = 0;
    *text_state.output = *text_state.previous = 0;
    text_state.dropped = 0;
}

void debug_text_flush(void) {
    text_state.line[text_state.line_size] = 0;
    if (text_state.line_size && (!text_state.deduplicate || strcmp(text_state.line, text_state.previous))) {
        size_t add = text_state.line_size + 1;
        if (add >= TEXT_CAP - text_state.size) {
            ++text_state.dropped;
        } else {
            memcpy(text_state.output + text_state.size, text_state.line, text_state.line_size);
            text_state.size += text_state.line_size;
            text_state.output[text_state.size++] = '\n';
            text_state.output[text_state.size] = 0;
            strcpy(text_state.previous, text_state.line);
        }
    }
    text_state.line_size = text_state.pending_size = 0;
}

static void emit_text(const char *value) {
    while (*value) {
        if (*value == '\n') {
            debug_text_flush();
            ++value;
            continue;
        }
        size_t bytes = 1;
        unsigned char ch = (unsigned char)*value;
        if (ch >= 0xF0) {
            bytes = 4;
        } else if (ch >= 0xE0) {
            bytes = 3;
        } else if (ch >= 0xC0) {
            bytes = 2;
        }
        if (text_state.line_size + bytes >= LINE_CAP) {
            debug_text_flush();
        }
        memcpy(text_state.line + text_state.line_size, value, bytes);
        text_state.line_size += bytes;
        value += bytes;
    }
}

void debug_text_access(uint16_t address, uint8_t value, bool write) {
    if (!text_state.enabled || nes_execution_policy() != NES_EXECUTION_LIVE || address < text_state.first ||
        address > text_state.last || (write ? !text_state.writes : !text_state.reads)) {
        return;
    }
    if (!text_state.table_count) {
        if (!value || value == '\n') {
            debug_text_flush();
        } else if (value >= 32 && value < 127) {
            char ascii[2] = {(char)value, 0};
            emit_text(ascii);
        }
        return;
    }
    text_state.pending[text_state.pending_size++] = value;
    bool prefix = false;
    for (size_t i = 0; i < text_state.table_count; ++i) {
        TextEntry *entry = &text_state.table[i];
        if (text_state.pending_size > entry->length ||
            memcmp(entry->bytes, text_state.pending, text_state.pending_size)) {
            continue;
        }
        prefix = true;
        if (text_state.pending_size == entry->length) {
            text_state.pending_size = 0;
            emit_text(entry->text);
            return;
        }
    }
    if (!prefix || text_state.pending_size == 8) {
        bool retry = text_state.pending_size > 1;
        text_state.pending_size = 0;
        if (retry) {
            debug_text_access(address, value, write);
        }
    }
}

const char *debug_text_output(void) {
    return text_state.output;
}

size_t debug_text_size(void) {
    return text_state.size;
}

uint64_t debug_text_dropped(void) {
    return text_state.dropped;
}

bool debug_text_export(const char *path) {
    debug_text_flush();
    return nes_file_write_atomic(path, text_state.output, text_state.size) == NES_FILE_OK;
}
