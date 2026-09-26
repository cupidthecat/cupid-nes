/*
 * memory_watch.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Memory watch configuration and comparison history. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_DEBUG_MEMORY_WATCH_H
#define CUPID_DEBUG_MEMORY_WATCH_H

#include "expression.h"
#include "memory_view.h"

enum { DEBUG_WATCH_LIMIT = 64, DEBUG_WATCH_EXPORT_LIMIT = 32768 };
typedef uint32_t DebugWatchFormat;
enum { DEBUG_WATCH_HEX, DEBUG_WATCH_UNSIGNED, DEBUG_WATCH_SIGNED, DEBUG_WATCH_BINARY, DEBUG_WATCH_ASCII,
       DEBUG_WATCH_FORMAT_COUNT };
typedef uint32_t DebugWatchSort;
enum { DEBUG_WATCH_SORT_ADDRESS, DEBUG_WATCH_SORT_CURRENT, DEBUG_WATCH_SORT_PREVIOUS,
       DEBUG_WATCH_SORT_CHANGES, DEBUG_WATCH_SORT_LABEL, DEBUG_WATCH_SORT_COUNT };

typedef struct {
    DebugMemorySpace space;
    uint8_t width;
    DebugWatchFormat format;
    bool enabled;
    char expression[DEBUG_EXPRESSION_LIMIT];
    char label[48];
} DebugWatchSpec;

typedef struct {
    DebugWatchSpec spec;
    uint32_t id, address, current, previous, changes;
    bool valid, previous_valid;
    char error[96];
} DebugWatchSample;

/* An array expands to independently editable rows. All rows are validated
 * before anything is added. Zero reports a failure and retains the list. */
uint32_t debug_watch_add(const DebugWatchSpec *spec, size_t count, char *error, size_t error_size);
bool debug_watch_update(uint32_t id, const DebugWatchSpec *spec, char *error, size_t error_size);
bool debug_watch_at(size_t index, DebugWatchSample *out);
void debug_watch_refresh(void);
void debug_watch_capture_previous(void);
void debug_watch_freeze_previous(bool frozen);
bool debug_watch_previous_frozen(void);
bool debug_watch_move(uint32_t id, size_t index);
bool debug_watch_sort(DebugWatchSort column, bool descending);
bool debug_watch_format(uint32_t value, uint8_t width, DebugWatchFormat format, char *out, size_t size);
/* CSV includes configuration, formatted values and visible per-row errors.
 * Failure leaves the destination unchanged. */
bool debug_watch_export(char *out, size_t capacity);

/* Debugger lifecycle internals. Session resets retain specs; shutdown/init
 * remove them. Identifiers share the debugger's existing allocation domain. */
void debug_watch_reset(void);
uint32_t debugger_reserve_identifiers(size_t count);

#endif
