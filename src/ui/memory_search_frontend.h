/*
 * memory_search_frontend.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Memory Search and Cheat Finder. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_MEMORY_SEARCH_FRONTEND_H
#define CUPID_MEMORY_SEARCH_FRONTEND_H

#include "cheat_frontend.h"

enum {
    MEMORY_SEARCH_PANEL = 0x1385,
    CHEAT_FINDER_PANEL = 0x1386,
    MEMORY_SEARCH_PAGE_SIZE = 12
};

enum {
    MEMORY_SEARCH_SPACE = 1,
    MEMORY_SEARCH_FIRST,
    MEMORY_SEARCH_LAST,
    MEMORY_SEARCH_WIDTH,
    MEMORY_SEARCH_FORMAT,
    MEMORY_SEARCH_ALIGNED,
    MEMORY_SEARCH_COMPARISON,
    MEMORY_SEARCH_REFERENCE,
    MEMORY_SEARCH_VALUE,
    MEMORY_SEARCH_START,
    MEMORY_SEARCH_FILTER,
    MEMORY_SEARCH_REFRESH,
    MEMORY_SEARCH_UNDO,
    MEMORY_SEARCH_INFO,
    MEMORY_SEARCH_PREV_PAGE,
    MEMORY_SEARCH_NEXT_PAGE,
    MEMORY_SEARCH_SELECTION,
    MEMORY_SEARCH_BYTE,
    MEMORY_SEARCH_REPLACEMENT,
    MEMORY_SEARCH_COMPARE,
    MEMORY_SEARCH_DESCRIPTION,
    MEMORY_SEARCH_WATCH,
    MEMORY_SEARCH_SEND,
    MEMORY_SEARCH_TEST,
    MEMORY_SEARCH_COPY,
    MEMORY_SEARCH_ROW = 100,
    MEMORY_SEARCH_SORT = 200
};

typedef struct MemoryToolsFrontend MemoryToolsFrontend;

/* The editor outlives these panels. Registration rolls back only registrations
 * owned by this instance. Each panel has its own search and undo history. */
MemoryToolsFrontend *memory_tools_create(CheatFrontend *editor);
bool memory_tools_register(MemoryToolsFrontend *frontend);
void memory_tools_image_changed(MemoryToolsFrontend *frontend);
void memory_tools_destroy(MemoryToolsFrontend *frontend);
bool memory_tools_panel(unsigned id);

#endif
