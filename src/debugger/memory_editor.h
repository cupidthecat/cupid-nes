/*
 * memory_editor.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Observational hex views and transactional edits. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_MEMORY_EDITOR_H
#define CUPID_MEMORY_EDITOR_H

#include "memory_view.h"

enum {
    DEBUG_MEMORY_PAGE_SIZE = 256,
    DEBUG_MEMORY_EDIT_LIMIT = 65536,
    DEBUG_MEMORY_TEXT_LIMIT = DEBUG_MEMORY_EDIT_LIMIT * 3 + 1,
    DEBUG_MEMORY_PATTERN_LIMIT = 256
};

/* No machine pointers escape into a cached view. Backing names are static.
 * Changed includes a different physical backing, even if its value is equal. */
typedef struct {
    const char *backing;
    size_t offset;
    uint8_t value;
    bool readable, writable, physical, changed;
} DebugMemoryByte;

typedef struct {
    DebugMemorySpace space;
    uint32_t first;
    uint64_t session;
    size_t count;
    bool valid;
    DebugMemoryByte bytes[DEBUG_MEMORY_PAGE_SIZE], baseline[DEBUG_MEMORY_PAGE_SIZE];
} DebugMemoryPage;

bool debug_memory_inspect(DebugMemorySpace space, uint32_t address, DebugMemoryByte *out);
bool debug_memory_capture(DebugMemoryPage *page, DebugMemorySpace space, uint32_t first, bool baseline);
/* Reject unreadable bytes and retain out on failure. ROM and observational
 * register values may be copied; floating addresses may not. */
bool debug_memory_copy(DebugMemorySpace space, uint32_t first, size_t count,
                        uint8_t *out, size_t capacity, char *error, size_t error_size);
/* Call under the paused-machine lock. Every byte and alias is resolved before
 * writing. Conflicting values for one physical byte fail without any edits.
 * The caller invalidates its timeline only when changed is true. */
bool debug_memory_edit(DebugMemorySpace space, uint32_t first, const uint8_t *bytes, size_t count,
                        uint64_t session, bool *changed, char *error, size_t error_size);
/* Both outputs are retained on parse failure. Text mode copies UTF-8 bytes;
 * hex mode accepts byte pairs, optional $/0x prefixes, spaces and commas. */
bool debug_memory_parse_bytes(const char *text, bool text_mode, uint8_t *out, size_t capacity,
                               size_t *count, char *error, size_t error_size);
bool debug_memory_format_hex(const uint8_t *bytes, size_t count, char *out, size_t capacity);
/* Matches cannot cross a space boundary or an unreadable byte. */
bool debug_memory_find(DebugMemorySpace space, uint32_t first, const uint8_t *pattern, size_t count,
                        bool wrap, uint32_t *found, char *error, size_t error_size);

#endif
