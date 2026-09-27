/*
 * memory_search.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Snapshot searches over debugger memory. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_DEBUG_MEMORY_SEARCH_H
#define CUPID_DEBUG_MEMORY_SEARCH_H

#include "memory_view.h"

typedef struct DebugMemorySearch DebugMemorySearch;

typedef struct {
    void *context;
    void *(*allocate)(void *context, size_t bytes);
    void (*release)(void *context, void *memory);
} DebugSearchAllocator;

typedef struct {
    DebugMemorySpace space;
    uint32_t first, last;
    uint8_t width; /* 1, 2 or 4 bytes, little endian. */
    bool signed_values;
    bool aligned;
} DebugSearchSpec;

typedef struct {
    uint32_t address;
    uint32_t current, previous, initial;
    uint32_t changes;
} DebugSearchRow;

typedef uint32_t DebugSearchComparison;
enum { DEBUG_SEARCH_EQUAL, DEBUG_SEARCH_DIFFERENT, DEBUG_SEARCH_GREATER, DEBUG_SEARCH_LESS,
       DEBUG_SEARCH_AT_LEAST, DEBUG_SEARCH_AT_MOST, DEBUG_SEARCH_COMPARISON_COUNT };

typedef uint32_t DebugSearchReference;
enum { DEBUG_SEARCH_CONSTANT, DEBUG_SEARCH_PREVIOUS, DEBUG_SEARCH_INITIAL, DEBUG_SEARCH_ADDRESS,
       DEBUG_SEARCH_LAST_FILTER,
       DEBUG_SEARCH_REFERENCE_COUNT };

typedef uint32_t DebugSearchSort;
enum { DEBUG_SEARCH_SORT_ADDRESS, DEBUG_SEARCH_SORT_CURRENT, DEBUG_SEARCH_SORT_PREVIOUS,
       DEBUG_SEARCH_SORT_INITIAL, DEBUG_SEARCH_SORT_CHANGES, DEBUG_SEARCH_SORT_COUNT };

typedef enum {
    DEBUG_SEARCH_OK,
    DEBUG_SEARCH_INVALID,
    DEBUG_SEARCH_NO_SNAPSHOT,
    DEBUG_SEARCH_NO_MEMORY
} DebugSearchResult;

/* At most 65,536 candidates and one undo snapshot. A replacement is staged
 * before publication; allocation failure preserves both snapshots and order.
 * NULL selects malloc/free. A supplied allocator must outlive the search. */
DebugMemorySearch *debug_search_create(const DebugSearchAllocator *allocator);
void debug_search_destroy(DebugMemorySearch *search);
void debug_search_clear(DebugMemorySearch *search);
DebugSearchResult debug_search_start(DebugMemorySearch *search, const DebugSearchSpec *spec);
DebugSearchResult debug_search_filter(DebugMemorySearch *search, DebugSearchComparison comparison,
                                    DebugSearchReference reference, uint32_t value);
DebugSearchResult debug_search_refresh(DebugMemorySearch *search);
bool debug_search_undo(DebugMemorySearch *search);
bool debug_search_can_undo(DebugMemorySearch *search);
bool debug_search_sort(DebugMemorySearch *search, DebugSearchSort column, bool descending);

/* Getters discard snapshots whose debugger session has ended. */
size_t debug_search_count(DebugMemorySearch *search);
size_t debug_search_total(DebugMemorySearch *search);
bool debug_search_spec(DebugMemorySearch *search, DebugSearchSpec *out);
bool debug_search_row(DebugMemorySearch *search, size_t index, DebugSearchRow *out);
int64_t debug_search_number(uint32_t value, uint8_t width, bool signed_values);
const char *debug_search_result_message(DebugSearchResult result);

#endif
