/*
 * memory_editor.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Observational hex views and transactional edits. SPDX-License-Identifier: GPL-3.0-or-later */
#include "memory_editor.h"
#include "debugger.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../system/execution_policy.h"

#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool report(char *error, size_t size, const char *message) {
    if (error && size) snprintf(error, size, "%s", message ? message : "");
    return message == NULL;
}

static bool range(DebugMemorySpace space, uint32_t first, size_t count) {
    uint32_t low, high;
    return count && count <= DEBUG_MEMORY_EDIT_LIMIT && debug_memory_bounds(space, &low, &high)
        && first >= low && first <= high && count <= (size_t)(high - first) + 1;
}

static bool location(DebugMemorySpace space, uint32_t address, NesMemoryLocation *out) {
    if (!range(space, address, 1)) return false;
    if (debug_memory_cpu_space(space)) return cpu_debug_memory_location((uint16_t)address, out);
    if (space == DEBUG_MEMORY_OAM) return ppu_debug_oam_location((uint16_t)address, out);
    return ppu_debug_memory_location((uint16_t)address, out);
}

bool debug_memory_inspect(DebugMemorySpace space, uint32_t address, DebugMemoryByte *out) {
    if (!out || !range(space, address, 1)) return false;
    DebugMemoryByte byte = {.offset = address, .backing = "Unmapped / floating"};
    NesMemoryLocation mapped;
    if (location(space, address, &mapped)) {
        byte.value = *mapped.data;
        byte.offset = mapped.offset;
        byte.backing = mapped.backing;
        byte.readable = byte.physical = true;
        byte.writable = mapped.writable;
    } else if (debug_memory_readable(space, address, address)) {
        if (!debug_memory_sample(space, address, address, &byte.value, 1)) return false;
        byte.readable = true;
        byte.backing = debug_memory_cpu_space(space) && address < 0x4020
            ? "CPU register (read only)" : "Mapped value (no editable backing)";
    }
    *out = byte;
    return true;
}

bool debug_memory_capture(DebugMemoryPage *page, DebugMemorySpace space, uint32_t first, bool baseline) {
    uint32_t low, high;
    if (!page || !debug_memory_bounds(space, &low, &high) || first < low || first > high) return false;
    size_t count = (size_t)(high - first) + 1;
    if (count > DEBUG_MEMORY_PAGE_SIZE) count = DEBUG_MEMORY_PAGE_SIZE;
    DebugMemoryByte bytes[DEBUG_MEMORY_PAGE_SIZE];
    bool retain = !baseline && page->valid && page->space == space && page->first == first
        && page->session == debugger_session_revision() && page->count == count;
    for (size_t i = 0; i < count; ++i) {
        if (!debug_memory_inspect(space, first + (uint32_t)i, &bytes[i])) return false;
        if (retain) {
            const DebugMemoryByte *before = &page->baseline[i];
            bytes[i].changed = bytes[i].readable != before->readable
                || bytes[i].physical != before->physical || bytes[i].offset != before->offset
                || strcmp(bytes[i].backing, before->backing)
                || (bytes[i].readable && bytes[i].value != before->value);
        }
    }
    memcpy(page->bytes, bytes, count * sizeof(*bytes));
    if (!retain) memcpy(page->baseline, bytes, count * sizeof(*bytes));
    page->space = space;
    page->first = first;
    page->count = count;
    page->session = debugger_session_revision();
    page->valid = true;
    return true;
}

bool debug_memory_copy(DebugMemorySpace space, uint32_t first, size_t count,
                        uint8_t *out, size_t capacity, char *error, size_t error_size) {
    if (!out || !range(space, first, count) || count > capacity)
        return report(error, error_size, "The selection is outside this memory space");
    uint8_t *staged = malloc(count);
    if (!staged) return report(error, error_size, "Could not allocate the memory selection");
    for (size_t i = 0; i < count; ++i) {
        DebugMemoryByte byte;
        if (!debug_memory_inspect(space, first + (uint32_t)i, &byte) || !byte.readable) {
            free(staged);
            return report(error, error_size, "The selection contains an unmapped or unreadable byte");
        }
        staged[i] = byte.value;
    }
    memcpy(out, staged, count);
    free(staged);
    return report(error, error_size, NULL);
}

typedef struct {
    NesMemoryLocation location;
    uint8_t value;
} EditByte;

static int compare_location(const void *left, const void *right) {
    uintptr_t a = (uintptr_t)((const EditByte *)left)->location.data;
    uintptr_t b = (uintptr_t)((const EditByte *)right)->location.data;
    return (a > b) - (a < b);
}

bool debug_memory_edit(DebugMemorySpace space, uint32_t first, const uint8_t *bytes, size_t count,
                        uint64_t session, bool *changed, char *error, size_t error_size) {
    if (changed) *changed = false;
    if (session != debugger_session_revision())
        return report(error, error_size, "The game or timeline changed; refresh the memory view");
    if (!debugger_is_paused() || nes_execution_policy() != NES_EXECUTION_LIVE)
        return report(error, error_size, "Pause a live game to edit; movie, rewind and netplay sessions are read-only");
    if (!bytes || !range(space, first, count))
        return report(error, error_size, "The edit is outside this memory space");
    EditByte *plan = malloc(count * sizeof(*plan));
    if (!plan) return report(error, error_size, "Could not allocate the memory edit");
    for (size_t i = 0; i < count; ++i) {
        NesMemoryLocation *mapped = &plan[i].location;
        if (!location(space, first + (uint32_t)i, mapped) || !mapped->writable) {
            char message[192];
            snprintf(message, sizeof(message), "$%04" PRIX32 " has no writable backing (ROM, protected or unmapped)",
                     first + (uint32_t)i);
            free(plan);
            return report(error, error_size, message);
        }
        plan[i].value = bytes[i] & mapped->mask;
    }
    qsort(plan, count, sizeof(*plan), compare_location);
    for (size_t i = 1; i < count; ++i) {
        if (plan[i].location.data == plan[i - 1].location.data && plan[i].value != plan[i - 1].value) {
            free(plan);
            return report(error, error_size, "The selection gives conflicting values to the same mirrored byte");
        }
    }
    bool any = false;
    for (size_t i = 0; i < count; ++i) {
        if (i && plan[i].location.data == plan[i - 1].location.data) continue;
        NesMemoryLocation *mapped = &plan[i].location;
        uint8_t value = plan[i].value;
        if (*mapped->data == value && (!mapped->mirror || *mapped->mirror == value)) continue;
        *mapped->data = value;
        if (mapped->mirror) *mapped->mirror = value;
        if (mapped->dirty) *mapped->dirty = true;
        if (mapped->refresh) *mapped->refresh = mapped->refresh_cycle;
        any = true;
    }
    free(plan);
    if (changed) *changed = any;
    return report(error, error_size, NULL);
}

static int hex_digit(unsigned char value) {
    if (value >= '0' && value <= '9') return value - '0';
    if (value >= 'A' && value <= 'F') return value - 'A' + 10;
    if (value >= 'a' && value <= 'f') return value - 'a' + 10;
    return -1;
}

bool debug_memory_parse_bytes(const char *text, bool text_mode, uint8_t *out, size_t capacity,
                               size_t *count, char *error, size_t error_size) {
    if (!text || !out || !count || !capacity || capacity > DEBUG_MEMORY_EDIT_LIMIT || !*text)
        return report(error, error_size, "Enter a bounded, nonempty byte sequence");
    size_t length = 0;
    while (length < DEBUG_MEMORY_TEXT_LIMIT && text[length]) ++length;
    if (length == DEBUG_MEMORY_TEXT_LIMIT)
        return report(error, error_size, "The byte sequence exceeds the text limit");
    if (text_mode) {
        if (length > capacity) return report(error, error_size, "The text exceeds the byte limit");
        memmove(out, text, length);
        *count = length;
        return report(error, error_size, NULL);
    }
    uint8_t *staged = malloc(capacity);
    if (!staged) return report(error, error_size, "Could not allocate the byte sequence");
    size_t used = 0;
    const unsigned char *p = (const unsigned char *)text;
    while (*p) {
        if (isspace(*p) || *p == ',') { ++p; continue; }
        if (*p == '$') ++p;
        else if (*p == '0' && (p[1] == 'x' || p[1] == 'X')) p += 2;
        int high = hex_digit(*p), low = high < 0 ? -1 : hex_digit(p[1]);
        if (high < 0 || low < 0 || used == capacity) {
            free(staged);
            return report(error, error_size, "Use complete hex byte pairs within the byte limit");
        }
        staged[used++] = (uint8_t)(high * 16 + low);
        p += 2;
    }
    if (!used) {
        free(staged);
        return report(error, error_size, "Enter at least one byte");
    }
    memcpy(out, staged, used);
    free(staged);
    *count = used;
    return report(error, error_size, NULL);
}

bool debug_memory_format_hex(const uint8_t *bytes, size_t count, char *out, size_t capacity) {
    if (!bytes || !out || !count || count > DEBUG_MEMORY_EDIT_LIMIT || capacity < count * 3) return false;
    static const char digits[] = "0123456789ABCDEF";
    for (size_t i = 0; i < count; ++i) {
        out[i * 3] = digits[bytes[i] >> 4];
        out[i * 3 + 1] = digits[bytes[i] & 15];
        out[i * 3 + 2] = i + 1 == count ? '\0' : (i + 1) % 16 == 0 ? '\n' : ' ';
    }
    return true;
}

bool debug_memory_find(DebugMemorySpace space, uint32_t first, const uint8_t *pattern, size_t count,
                        bool wrap, uint32_t *found, char *error, size_t error_size) {
    uint32_t low, high;
    if (!pattern || !found || !count || count > DEBUG_MEMORY_PATTERN_LIMIT
        || !debug_memory_bounds(space, &low, &high) || first < low || first > high)
        return report(error, error_size, "Invalid search range or byte sequence");
    size_t length = (size_t)(high - low) + 1;
    if (count > length) return report(error, error_size, "The byte sequence was not found");
    uint8_t *data = malloc(length * 2);
    if (!data) return report(error, error_size, "Could not allocate the search snapshot");
    uint8_t *readable = data + length;
    for (size_t i = 0; i < length; ++i) {
        DebugMemoryByte byte;
        if (!debug_memory_inspect(space, low + (uint32_t)i, &byte)) {
            free(data);
            return report(error, error_size, "Could not inspect the search range");
        }
        data[i] = byte.value;
        readable[i] = byte.readable;
    }
    size_t candidates = wrap ? length : (size_t)(high - first) + 1;
    for (size_t i = 0; i < candidates; ++i) {
        size_t at = ((size_t)(first - low) + i) % length;
        if (at <= length - count && !memcmp(data + at, pattern, count) && !memchr(readable + at, 0, count)) {
            *found = low + (uint32_t)at;
            free(data);
            return report(error, error_size, NULL);
        }
    }
    free(data);
    return report(error, error_size, "The byte sequence was not found");
}
