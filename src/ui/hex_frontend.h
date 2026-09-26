/*
 * hex_frontend.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Native hex editor controls. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_HEX_FRONTEND_H
#define CUPID_HEX_FRONTEND_H

#include "frontend_execution.h"

enum { HEX_FRONTEND_PANEL = 0x1388, HEX_ROWS = 16, HEX_COLUMNS = 16 };
enum {
    HEX_SPACE = 1, HEX_ADDRESS, HEX_COUNT, HEX_SELECT, HEX_LIVE, HEX_REFRESH, HEX_BASELINE,
    HEX_PREV_PAGE, HEX_NEXT_PAGE, HEX_BYTES, HEX_TEXT_MODE, HEX_APPLY, HEX_COPY, HEX_PASTE,
    HEX_PATTERN, HEX_FIND, HEX_WRAP, HEX_IMPORT_PATH, HEX_IMPORT, HEX_EXPORT_PATH, HEX_EXPORT,
    HEX_WATCH, HEX_INFO, HEX_BACKING, HEX_SELECTION, HEX_PICK, HEX_EXTEND,
    HEX_ROW = 100
};

typedef struct HexFrontend HexFrontend;
HexFrontend *hex_frontend_create(FrontendExecutionRuntime *execution);
bool hex_frontend_register(HexFrontend *frontend);
void hex_frontend_unregister(HexFrontend *frontend);
void hex_frontend_image_changed(HexFrontend *frontend);
void hex_frontend_destroy(HexFrontend *frontend);

#endif
