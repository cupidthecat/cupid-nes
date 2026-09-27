/*
 * watch_frontend.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Memory watch table and editing controls. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_WATCH_FRONTEND_H
#define CUPID_WATCH_FRONTEND_H

#include "frontend_execution.h"

enum { WATCH_FRONTEND_PANEL = 0x1387, WATCH_PAGE_SIZE = 10 };
enum {
    WATCH_SPACE = 1, WATCH_EXPRESSION, WATCH_WIDTH, WATCH_FORMAT, WATCH_LABEL, WATCH_COUNT,
    WATCH_ADD, WATCH_UPDATE, WATCH_TOGGLE, WATCH_REMOVE, WATCH_PREVIOUS, WATCH_FREEZE,
    WATCH_MOVE_UP, WATCH_MOVE_DOWN, WATCH_COPY, WATCH_EXPORT_PATH, WATCH_EXPORT,
    WATCH_PREV_PAGE, WATCH_NEXT_PAGE, WATCH_INFO, WATCH_SELECTION, WATCH_PICK, WATCH_VALUES,
    WATCH_ROW = 100, WATCH_SORT = 200
};

typedef struct WatchFrontend WatchFrontend;
WatchFrontend *watch_frontend_create(FrontendExecutionRuntime *execution);
bool watch_frontend_register(WatchFrontend *frontend);
void watch_frontend_image_changed(WatchFrontend *frontend);
void watch_frontend_destroy(WatchFrontend *frontend);
/* Select by persistent watch ID when handing a newly created row to this panel. */
bool watch_frontend_select(uint32_t id);

#endif
