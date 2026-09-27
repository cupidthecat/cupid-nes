/*
 * debug_capture.h - Bounded trace and text capture
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_DEBUG_CAPTURE_H
#define CUPID_DEBUG_CAPTURE_H
#include "debug_analysis.h"

enum {
    DEBUG_TRACE_REGISTERS = 1,
    DEBUG_TRACE_CYCLES = 2,
    DEBUG_TRACE_BYTES = 4,
    DEBUG_TRACE_SYMBOLS = 8,
    DEBUG_TRACE_ALL = 15
};

typedef struct {
    uint16_t first, last;
    char condition[97];
    unsigned columns;
} DebugTraceFilter;

typedef struct {
    DebugTraceEntry cpu;
    uint64_t key;
    uint8_t bytes[3], length;
} DebugLogEntry;

void debug_capture_shutdown(void);
void debug_capture_discontinuity(void);
bool debug_log_configure(size_t capacity, const DebugTraceFilter *filter, char *error, size_t size);
void debug_log_enable(bool enabled);
bool debug_log_enabled(void);
void debug_log_clear(void);
size_t debug_log_count(void);
uint64_t debug_log_overwritten(void);
bool debug_log_at(size_t index, DebugLogEntry *out);
bool debug_log_format(const DebugLogEntry *entry, char *out, size_t capacity);
bool debug_log_export(const char *path);
void debug_log_instruction(const CPU *state);
void debug_log_read(uint16_t address, uint8_t value);
void debug_log_opcode(uint8_t opcode);
void debug_log_end(void);
/* Table: 1..8 hex bytes, '=', UTF-8 text (or \\n), one entry per line.
 * Ambiguous prefixes and duplicate byte keys fail transactionally. */
bool debug_text_table(const char *text, size_t size, char *error, size_t error_size);
void debug_text_configure(uint16_t first, uint16_t last, bool reads, bool writes, bool deduplicate);
void debug_text_enable(bool enabled);
bool debug_text_enabled(void);
void debug_text_clear(void);
void debug_text_flush(void);
void debug_text_access(uint16_t address, uint8_t value, bool write);
const char *debug_text_output(void);
size_t debug_text_size(void);
uint64_t debug_text_dropped(void);
bool debug_text_export(const char *path);
#endif
