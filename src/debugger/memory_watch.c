/*
 * memory_watch.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Memory watch configuration and comparison history. SPDX-License-Identifier: GPL-3.0-or-later */
#include "memory_watch.h"
#include "debugger.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    DebugWatchSample sample;
    uint64_t session;
    bool sampled;
} WatchRecord;

static WatchRecord watches[DEBUG_WATCH_LIMIT];
static size_t watch_count;
static bool freeze_previous;

static bool fail(char *error, size_t size, const char *message) {
    if (error && size) snprintf(error, size, "%s", message);
    return false;
}

static bool valid_spec(const DebugWatchSpec *spec, char *error, size_t size) {
    if (!spec || spec->width < 1 || spec->width > 4 ||
        spec->space >= DEBUG_MEMORY_SPACE_COUNT || spec->format >= DEBUG_WATCH_FORMAT_COUNT ||
        !memchr(spec->expression, '\0', sizeof(spec->expression)) || !spec->expression[0] ||
        !memchr(spec->label, '\0', sizeof(spec->label))) {
        return fail(error, size, "Invalid watch space, width, format or text");
    }
    return true;
}

static bool read_watch(const DebugWatchSpec *spec, uint32_t *address, uint32_t *value, char *error, size_t size) {
    if (!debug_expression_eval(spec->expression, address, error, size)) return false;
    uint32_t low, high;
    if (!debug_memory_bounds(spec->space, &low, &high) || *address < low || *address > high ||
        (uint32_t)spec->width - 1 > high - *address) {
        return fail(error, size, "Watch extends outside the selected memory space");
    }
    uint8_t bytes[4];
    if (!debug_memory_readable(spec->space, *address, *address + spec->width - 1) ||
        !debug_memory_sample(spec->space, *address, *address + spec->width - 1, bytes, sizeof(bytes))) {
        return fail(error, size, "Address has no readable mapped backing");
    }
    *value = 0;
    for (unsigned i = 0; i < spec->width; ++i) *value |= (uint32_t)bytes[i] << (8 * i);
    return true;
}

static void refresh(WatchRecord *record) {
    DebugWatchSample *row = &record->sample;
    uint64_t session = debugger_session_revision();
    if (record->session != session) {
        record->session = session;
        record->sampled = row->previous_valid = false;
        row->changes = 0;
    }
    if (!row->spec.enabled) {
        row->valid = false;
        snprintf(row->error, sizeof(row->error), "Disabled");
        return;
    }
    uint32_t address, value;
    row->valid = read_watch(&row->spec, &address, &value, row->error, sizeof(row->error));
    if (!row->valid) return;
    row->error[0] = '\0';
    if (!record->sampled || row->address != address) {
        row->previous = value;
        row->previous_valid = true;
        row->changes = 0;
    } else if (row->current != value) {
        if (!freeze_previous) row->previous = row->current;
        if (row->changes != UINT32_MAX) ++row->changes;
    }
    row->current = value;
    row->address = address;
    record->sampled = true;
}

void debug_watch_reset(void) {
    memset(watches, 0, sizeof(watches));
    watch_count = 0;
    freeze_previous = false;
}

uint32_t debug_watch_add(const DebugWatchSpec *spec, size_t count, char *error, size_t error_size) {
    if (!valid_spec(spec, error, error_size)) return 0;
    if (!count || count > DEBUG_WATCH_LIMIT - watch_count) {
        fail(error, error_size, "The watch list holds at most 64 rows");
        return 0;
    }
    WatchRecord pending[DEBUG_WATCH_LIMIT] = {0};
    for (size_t i = 0; i < count; ++i) {
        DebugWatchSample *row = &pending[i].sample;
        row->spec = *spec;
        if (count > 1) {
            int length = snprintf(row->spec.expression, sizeof(row->spec.expression), "(%s)+$%zX", spec->expression,
                                  i * spec->width);
            int label_length = snprintf(row->spec.label, sizeof(row->spec.label), "%s[%zu]", spec->label, i);
            if (length < 0 || (size_t)length >= sizeof(row->spec.expression) || label_length < 0 ||
                (size_t)label_length >= sizeof(row->spec.label)) {
                fail(error, error_size, "Array expression or label is too long");
                return 0;
            }
        }
        if (!read_watch(&row->spec, &row->address, &row->current, error, error_size)) return 0;
        refresh(&pending[i]);
    }
    uint32_t first_id = debugger_reserve_identifiers(count);
    if (!first_id) {
        fail(error, error_size, "The debugger identifier limit was reached");
        return 0;
    }
    for (size_t i = 0; i < count; ++i) {
        pending[i].sample.id = first_id + (uint32_t)i;
        watches[watch_count++] = pending[i];
    }
    if (error && error_size) error[0] = '\0';
    return first_id;
}

bool debug_watch_update(uint32_t id, const DebugWatchSpec *spec, char *error, size_t error_size) {
    if (!valid_spec(spec, error, error_size)) return false;
    for (size_t i = 0; i < watch_count; ++i) {
        WatchRecord *row = &watches[i];
        if (row->sample.id != id) continue;
        bool changed_address = row->sample.spec.space != spec->space || row->sample.spec.width != spec->width ||
                               strcmp(row->sample.spec.expression, spec->expression);
        uint32_t address, value;
        if (changed_address && !read_watch(spec, &address, &value, error, error_size)) return false;
        if (changed_address || row->sample.spec.enabled != spec->enabled) row->sampled = false;
        row->sample.spec = *spec;
        refresh(row);
        if (error && error_size) error[0] = '\0';
        return true;
    }
    return fail(error, error_size, "The selected watch no longer exists");
}

bool debug_watch_at(size_t index, DebugWatchSample *out) {
    if (!out || index >= watch_count) return false;
    *out = watches[index].sample;
    return true;
}

void debug_watch_refresh(void) {
    for (size_t i = 0; i < watch_count; ++i) refresh(&watches[i]);
}

void debug_watch_capture_previous(void) {
    debug_watch_refresh();
    for (size_t i = 0; i < watch_count; ++i) {
        DebugWatchSample *row = &watches[i].sample;
        if (row->valid) {
            row->previous = row->current;
            row->previous_valid = true;
            row->changes = 0;
        }
    }
}

void debug_watch_freeze_previous(bool frozen) { freeze_previous = frozen; }
bool debug_watch_previous_frozen(void) { return freeze_previous; }

bool debug_watch_move(uint32_t id, size_t index) {
    if (index >= watch_count) return false;
    for (size_t i = 0; i < watch_count; ++i) {
        if (watches[i].sample.id != id) continue;
        WatchRecord moved = watches[i];
        if (index < i) memmove(watches + index + 1, watches + index, (i - index) * sizeof(*watches));
        if (index > i) memmove(watches + i, watches + i + 1, (index - i) * sizeof(*watches));
        watches[index] = moved;
        return true;
    }
    return false;
}

static int compare(const DebugWatchSample *a, const DebugWatchSample *b, DebugWatchSort column) {
    if (column == DEBUG_WATCH_SORT_LABEL) return strcmp(a->spec.label, b->spec.label);
    if (column == DEBUG_WATCH_SORT_ADDRESS && a->spec.space != b->spec.space)
        return a->spec.space < b->spec.space ? -1 : 1;
    if (column != DEBUG_WATCH_SORT_ADDRESS && a->valid != b->valid) return a->valid ? -1 : 1;
    uint32_t left = column == DEBUG_WATCH_SORT_ADDRESS ? a->address : column == DEBUG_WATCH_SORT_CURRENT ? a->current :
                    column == DEBUG_WATCH_SORT_PREVIOUS ? a->previous : a->changes;
    uint32_t right = column == DEBUG_WATCH_SORT_ADDRESS ? b->address : column == DEBUG_WATCH_SORT_CURRENT ? b->current :
                     column == DEBUG_WATCH_SORT_PREVIOUS ? b->previous : b->changes;
    int64_t left_number = left, right_number = right;
    if (column == DEBUG_WATCH_SORT_CURRENT || column == DEBUG_WATCH_SORT_PREVIOUS) {
        uint64_t left_range = UINT64_C(1) << (a->spec.width * 8);
        uint64_t right_range = UINT64_C(1) << (b->spec.width * 8);
        if (a->spec.format == DEBUG_WATCH_SIGNED && (left & (left_range >> 1))) left_number -= (int64_t)left_range;
        if (b->spec.format == DEBUG_WATCH_SIGNED && (right & (right_range >> 1))) right_number -= (int64_t)right_range;
    }
    return left_number < right_number ? -1 : left_number > right_number ? 1 : 0;
}

bool debug_watch_sort(DebugWatchSort column, bool descending) {
    if (column >= DEBUG_WATCH_SORT_COUNT) return false;
    debug_watch_refresh();
    for (size_t i = 1; i < watch_count; ++i) {
        WatchRecord row = watches[i];
        size_t at = i;
        while (at) {
            int order = compare(&watches[at - 1].sample, &row.sample, column);
            if (descending ? order >= 0 : order <= 0) break;
            watches[at] = watches[at - 1];
            --at;
        }
        watches[at] = row;
    }
    return true;
}

bool debug_watch_format(uint32_t value, uint8_t width, DebugWatchFormat format, char *out, size_t size) {
    if (!out || width < 1 || width > 4 || format >= DEBUG_WATCH_FORMAT_COUNT) return false;
    unsigned bits = width * 8;
    uint64_t range = UINT64_C(1) << bits;
    value &= (uint32_t)(range - 1);
    char text[40];
    switch (format) {
    case DEBUG_WATCH_HEX: snprintf(text, sizeof(text), "%0*" PRIX32, width * 2, value); break;
    case DEBUG_WATCH_UNSIGNED: snprintf(text, sizeof(text), "%" PRIu32, value); break;
    case DEBUG_WATCH_SIGNED:
        snprintf(text, sizeof(text), "%" PRId64, (int64_t)value - (value & (range >> 1) ? (int64_t)range : 0));
        break;
    case DEBUG_WATCH_BINARY:
        for (unsigned i = 0; i < bits; ++i) text[i] = value & (UINT32_C(1) << (bits - i - 1)) ? '1' : '0';
        text[bits] = '\0';
        break;
    case DEBUG_WATCH_ASCII:
        for (unsigned i = 0; i < width; ++i) {
            uint8_t byte = (uint8_t)(value >> (i * 8));
            text[i] = byte >= 32 && byte < 127 ? (char)byte : '.';
        }
        text[width] = '\0';
        break;
    default: return false;
    }
    if (strlen(text) >= size) return false;
    strcpy(out, text);
    return true;
}

static bool append(char *out, size_t *used, const char *format, ...) {
    va_list args;
    va_start(args, format);
    int length = vsnprintf(out + *used, DEBUG_WATCH_EXPORT_LIMIT - *used, format, args);
    va_end(args);
    if (length < 0 || (size_t)length >= DEBUG_WATCH_EXPORT_LIMIT - *used) return false;
    *used += (size_t)length;
    return true;
}

static bool csv_text(char *out, size_t *used, const char *text) {
    if (!append(out, used, "\"")) return false;
    for (const char *p = text; *p; ++p)
        if (!append(out, used, *p == '"' ? "\"\"" : "%c", *p)) return false;
    return append(out, used, "\"");
}

bool debug_watch_export(char *out, size_t capacity) {
    if (!out) return false;
    char text[DEBUG_WATCH_EXPORT_LIMIT];
    size_t used = 0;
    if (!append(text, &used, "id,space,address,expression,width,format,enabled,current,previous,changes,status,label\n"))
        return false;
    debug_watch_refresh();
    static const char *const formats[] = {"hex", "unsigned", "signed", "binary", "ASCII"};
    for (size_t i = 0; i < watch_count; ++i) {
        const DebugWatchSample *row = &watches[i].sample;
        char current[40] = "", previous[40] = "";
        if (row->valid) debug_watch_format(row->current, row->spec.width, row->spec.format, current, sizeof(current));
        if (row->previous_valid)
            debug_watch_format(row->previous, row->spec.width, row->spec.format, previous, sizeof(previous));
        if (!append(text, &used, "%" PRIu32 ",", row->id) ||
            !csv_text(text, &used, debug_memory_name(row->spec.space)) ||
            !append(text, &used, ",$%04" PRIX32 ",", row->address) || !csv_text(text, &used, row->spec.expression) ||
            !append(text, &used, ",%u,%s,%u,", row->spec.width, formats[row->spec.format], row->spec.enabled) ||
            !csv_text(text, &used, current) || !append(text, &used, ",") || !csv_text(text, &used, previous) ||
            !append(text, &used, ",%" PRIu32 ",", row->changes) || !csv_text(text, &used, row->error) ||
            !append(text, &used, ",") || !csv_text(text, &used, row->spec.label) || !append(text, &used, "\n")) return false;
    }
    if (used >= capacity) return false;
    memcpy(out, text, used + 1);
    return true;
}

uint32_t debugger_add_watch(uint16_t address, uint8_t size, const char *label) {
    DebugWatchSpec spec = {.space = DEBUG_MEMORY_CPU, .width = size, .format = DEBUG_WATCH_HEX, .enabled = true};
    snprintf(spec.expression, sizeof(spec.expression), "$%04X", address);
    snprintf(spec.label, sizeof(spec.label), "%s", label ? label : "");
    return debug_watch_add(&spec, 1, NULL, 0);
}

bool debugger_update_watch(uint32_t id, uint16_t address, uint8_t size, const char *label, bool enabled) {
    for (size_t i = 0; i < watch_count; ++i) {
        const DebugWatchSample *row = &watches[i].sample;
        if (row->id != id) continue;
        DebugWatchSpec spec = row->spec;
        if (row->address != address) snprintf(spec.expression, sizeof(spec.expression), "$%04X", address);
        spec.width = size;
        spec.enabled = enabled;
        snprintf(spec.label, sizeof(spec.label), "%s", label ? label : "");
        return debug_watch_update(id, &spec, NULL, 0);
    }
    return false;
}

bool debugger_remove_watch(uint32_t id) {
    for (size_t i = 0; i < watch_count; ++i) {
        if (watches[i].sample.id != id) continue;
        memmove(watches + i, watches + i + 1, (watch_count - i - 1) * sizeof(*watches));
        --watch_count;
        return true;
    }
    return false;
}

size_t debugger_watch_count(void) { return watch_count; }

bool debugger_watch_at(size_t index, DebugWatch *out, uint32_t *value) {
    if (!out || index >= watch_count) return false;
    DebugWatchSample row = watches[index].sample;
    uint32_t address = row.address, sampled = 0;
    if (row.spec.enabled) (void)read_watch(&row.spec, &address, &sampled, NULL, 0);
    *out = (DebugWatch){.id = row.id, .address = (uint16_t)address, .size = row.spec.width, .enabled = row.spec.enabled};
    snprintf(out->label, sizeof(out->label), "%s", row.spec.label);
    if (value) *value = sampled;
    return true;
}
