/*
 * desktop_memory.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Memory search result tables. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_DESKTOP_MEMORY_H
#define CUPID_DESKTOP_MEMORY_H

#include <stdbool.h>
#include "frontend_panels.h"

struct FrontendDesktopUi;
/* A zero direction keeps an operable control focused; +/-1 moves to the next. */
void desktop_memory_focus(struct FrontendDesktopUi *ui, const FrontendPanelModel *model, int direction);
/* Small windows use the shared scrollable panel instead. */
bool desktop_memory_layout(struct FrontendDesktopUi *ui, float width, float height);
bool desktop_watch_layout(struct FrontendDesktopUi *ui, float width, float height);

#endif
