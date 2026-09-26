/*
 * memory_search.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Snapshot searches over debugger memory. SPDX-License-Identifier: GPL-3.0-or-later */
#include "memory_search.h"
#include "debugger.h"

#include <stdlib.h>
#include <string.h>

typedef struct {
    DebugSearchRow row;
    int64_t sort_key;
    uint32_t last_filter;
} SearchEntry;

typedef struct {
    DebugSearchSpec spec;
    size_t count, total;
    uint8_t sample[0x10000];
    SearchEntry entries[];
} SearchSnapshot;

struct DebugMemorySearch {
    DebugSearchAllocator allocator;
    SearchSnapshot *current, *undo;
    uint64_t session;
    DebugSearchSort sort;
    bool descending;
};

static void *allocate_default(void *context, size_t bytes) {
    (void)context;
    return malloc(bytes);
}

static void release_default(void *context, void *memory) {
    (void)context;
    free(memory);
}

static void release_snapshot(DebugMemorySearch *search, SearchSnapshot *snapshot) {
    if (snapshot) {
        search->allocator.release(search->allocator.context, snapshot);
    }
}

void debug_search_clear(DebugMemorySearch *search) {
    if (!search) {
        return;
    }

    release_snapshot(search, search->current);
    release_snapshot(search, search->undo);
    search->current = search->undo = NULL;
    search->session = debugger_session_revision();
}

static bool current_session(DebugMemorySearch *search) {
    if (!search) {
        return false;
    }

    if (search->session != debugger_session_revision()) {
        debug_search_clear(search);
    }

    return search->current != NULL;
}

DebugMemorySearch *debug_search_create(const DebugSearchAllocator *allocator) {
    DebugSearchAllocator selected = {NULL, allocate_default, release_default};
    if (allocator) {
        if (!allocator->allocate || !allocator->release) {
            return NULL;
        }

        selected = *allocator;
    }

    DebugMemorySearch *search = selected.allocate(selected.context, sizeof(*search));
    if (search) {
        memset(search, 0, sizeof(*search));
        search->allocator = selected;
        search->session = debugger_session_revision();
    }

    return search;
}

void debug_search_destroy(DebugMemorySearch *search) {
    if (search) {
        DebugSearchAllocator allocator = search->allocator;
        debug_search_clear(search);
        allocator.release(allocator.context, search);
    }
}

static bool valid_spec(const DebugSearchSpec *spec) {
    uint32_t low, high;
    return spec && debug_memory_bounds(spec->space, &low, &high) && spec->first >= low && spec->last <= high &&
           spec->first <= spec->last && (spec->width == 1 || spec->width == 2 || spec->width == 4) &&
           spec->last - spec->first + 1 >= spec->width;
}

static SearchSnapshot *capture(DebugMemorySearch *search, const DebugSearchSpec *spec, size_t capacity) {
    /* valid_spec and the bounded candidate count precede this allocation. */
    size_t bytes = sizeof(SearchSnapshot) + capacity * sizeof(SearchEntry);
    SearchSnapshot *snapshot = search->allocator.allocate(search->allocator.context, bytes);
    if (!snapshot) {
        return NULL;
    }

    snapshot->spec = *spec;
    snapshot->count = snapshot->total = 0;
    if (!debug_memory_sample(spec->space, spec->first, spec->last, snapshot->sample, sizeof(snapshot->sample))) {
        release_snapshot(search, snapshot);
        return NULL;
    }

    return snapshot;
}

static uint32_t sample_value(const SearchSnapshot *snapshot, uint32_t address) {
    uint32_t value = 0;
    for (unsigned byte = 0; byte < snapshot->spec.width; ++byte) {
        value |= (uint32_t)snapshot->sample[address - snapshot->spec.first + byte] << (8 * byte);
    }

    return value;
}

int64_t debug_search_number(uint32_t value, uint8_t width, bool signed_values) {
    if (width != 1 && width != 2 && width != 4) {
        return 0;
    }

    uint64_t range = UINT64_C(1) << (8 * width);
    value &= (uint32_t)(range - 1);
    return signed_values && (value & (range >> 1)) ? (int64_t)value - (int64_t)range : (int64_t)value;
}

static int compare_entries(const void *left, const void *right) {
    const SearchEntry *a = left, *b = right;
    if (a->sort_key != b->sort_key) {
        return a->sort_key < b->sort_key ? -1 : 1;
    }

    return (a->row.address > b->row.address) - (a->row.address < b->row.address);
}

static void sort_snapshot(DebugMemorySearch *search, SearchSnapshot *snapshot) {
    for (size_t i = 0; i < snapshot->count; ++i) {
        SearchEntry *entry = &snapshot->entries[i];
        uint32_t value;
        switch (search->sort) {
        case DEBUG_SEARCH_SORT_CURRENT: value = entry->row.current; break;
        case DEBUG_SEARCH_SORT_PREVIOUS: value = entry->row.previous; break;
        case DEBUG_SEARCH_SORT_INITIAL: value = entry->row.initial; break;
        case DEBUG_SEARCH_SORT_CHANGES: value = entry->row.changes; break;
        default: value = entry->row.address; break;
        }

        bool signed_value = snapshot->spec.signed_values && search->sort >= DEBUG_SEARCH_SORT_CURRENT &&
                            search->sort <= DEBUG_SEARCH_SORT_INITIAL;
        entry->sort_key = signed_value ? debug_search_number(value, snapshot->spec.width, true) : (int64_t)value;
        if (search->descending) {
            entry->sort_key = -entry->sort_key;
        }
    }

    if (snapshot->count > 1) {
        qsort(snapshot->entries, snapshot->count, sizeof(*snapshot->entries), compare_entries);
    }
}

DebugSearchResult debug_search_start(DebugMemorySearch *search, const DebugSearchSpec *spec) {
    if (!search || !valid_spec(spec)) {
        return DEBUG_SEARCH_INVALID;
    }

    (void)current_session(search);
    uint32_t first = spec->first;
    if (spec->aligned) {
        first += (spec->width - first % spec->width) % spec->width;
    }

    uint32_t step = spec->aligned ? spec->width : 1;
    size_t count = first + spec->width - 1 > spec->last ? 0 : (spec->last - first + 1 - spec->width) / step + 1;
    SearchSnapshot *next = capture(search, spec, count);
    if (!next) {
        return DEBUG_SEARCH_NO_MEMORY;
    }

    next->count = next->total = count;
    for (size_t i = 0; i < count; ++i) {
        uint32_t address = first + (uint32_t)i * step;
        uint32_t value = sample_value(next, address);
        next->entries[i].row = (DebugSearchRow){address, value, value, value, 0};
        next->entries[i].last_filter = value;
    }

    sort_snapshot(search, next);
    debug_search_clear(search);
    search->current = next;
    return DEBUG_SEARCH_OK;
}

static bool matches(DebugSearchComparison comparison, int64_t left, int64_t right) {
    switch (comparison) {
    case DEBUG_SEARCH_EQUAL: return left == right;
    case DEBUG_SEARCH_DIFFERENT: return left != right;
    case DEBUG_SEARCH_GREATER: return left > right;
    case DEBUG_SEARCH_LESS: return left < right;
    case DEBUG_SEARCH_AT_LEAST: return left >= right;
    case DEBUG_SEARCH_AT_MOST: return left <= right;
    default: return false;
    }
}

static DebugSearchResult update(DebugMemorySearch *search, bool filter, DebugSearchComparison comparison,
                                DebugSearchReference reference, uint32_t value) {
    if (!search || comparison >= DEBUG_SEARCH_COMPARISON_COUNT || reference >= DEBUG_SEARCH_REFERENCE_COUNT) {
        return DEBUG_SEARCH_INVALID;
    }

    if (!current_session(search)) {
        return DEBUG_SEARCH_NO_SNAPSHOT;
    }

    SearchSnapshot *before = search->current;
    const DebugSearchSpec *spec = &before->spec;
    if (filter && reference == DEBUG_SEARCH_CONSTANT && spec->width < 4 && value >= (1u << (8 * spec->width))) {
        return DEBUG_SEARCH_INVALID;
    }

    if (filter && reference == DEBUG_SEARCH_ADDRESS &&
        (value < spec->first || value > spec->last || spec->last - value + 1 < spec->width)) {
        return DEBUG_SEARCH_INVALID;
    }

    SearchSnapshot *next = capture(search, spec, before->count);
    if (!next) {
        return DEBUG_SEARCH_NO_MEMORY;
    }

    next->total = before->total;
    for (size_t i = 0; i < before->count; ++i) {
        DebugSearchRow row = before->entries[i].row;
        uint32_t sampled = sample_value(next, row.address);
        uint32_t other = reference == DEBUG_SEARCH_PREVIOUS ? row.current
                         : reference == DEBUG_SEARCH_INITIAL ? row.initial
                         : reference == DEBUG_SEARCH_LAST_FILTER ? before->entries[i].last_filter
                         : reference == DEBUG_SEARCH_ADDRESS ? sample_value(next, value) : value;
        if (filter && !matches(comparison, debug_search_number(sampled, spec->width, spec->signed_values),
                               debug_search_number(other, spec->width, spec->signed_values))) {
            continue;
        }

        if (sampled != row.current && row.changes != UINT32_MAX) {
            ++row.changes;
        }

        row.previous = row.current;
        row.current = sampled;
        next->entries[next->count].row = row;
        next->entries[next->count++].last_filter = filter ? sampled : before->entries[i].last_filter;
    }

    sort_snapshot(search, next);
    release_snapshot(search, search->undo);
    search->undo = before;
    search->current = next;
    return DEBUG_SEARCH_OK;
}

DebugSearchResult debug_search_filter(DebugMemorySearch *search, DebugSearchComparison comparison,
                                    DebugSearchReference reference, uint32_t value) {
    return update(search, true, comparison, reference, value);
}

DebugSearchResult debug_search_refresh(DebugMemorySearch *search) {
    return update(search, false, DEBUG_SEARCH_EQUAL, DEBUG_SEARCH_CONSTANT, 0);
}

bool debug_search_undo(DebugMemorySearch *search) {
    if (!current_session(search) || !search->undo) {
        return false;
    }

    release_snapshot(search, search->current);
    search->current = search->undo;
    search->undo = NULL;
    sort_snapshot(search, search->current);
    return true;
}

bool debug_search_can_undo(DebugMemorySearch *search) {
    return current_session(search) && search->undo;
}

bool debug_search_sort(DebugMemorySearch *search, DebugSearchSort column, bool descending) {
    if (!search || column >= DEBUG_SEARCH_SORT_COUNT) {
        return false;
    }

    search->sort = column;
    search->descending = descending;
    if (current_session(search)) {
        sort_snapshot(search, search->current);
    }

    return true;
}

size_t debug_search_count(DebugMemorySearch *search) {
    return current_session(search) ? search->current->count : 0;
}

size_t debug_search_total(DebugMemorySearch *search) {
    return current_session(search) ? search->current->total : 0;
}

bool debug_search_spec(DebugMemorySearch *search, DebugSearchSpec *out) {
    if (!current_session(search) || !out) {
        return false;
    }

    *out = search->current->spec;
    return true;
}

bool debug_search_row(DebugMemorySearch *search, size_t index, DebugSearchRow *out) {
    if (!current_session(search) || !out || index >= search->current->count) {
        return false;
    }

    *out = search->current->entries[index].row;
    return true;
}

const char *debug_search_result_message(DebugSearchResult result) {
    switch (result) {
    case DEBUG_SEARCH_OK: return "Snapshot captured";
    case DEBUG_SEARCH_NO_SNAPSHOT: return "Choose New search after opening, resetting or restoring a game";
    case DEBUG_SEARCH_NO_MEMORY: return "Not enough memory; the previous search and undo snapshot are unchanged";
    default: return "Invalid memory space, range, width or comparison value";
    }
}
