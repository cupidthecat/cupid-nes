/*
 * desktop_tas.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Desktop TAS piano-roll editor. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_DESKTOP_TAS_H
#define CUPID_DESKTOP_TAS_H

#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct DesktopTasEditor DesktopTasEditor;
struct FrontendDesktopUi;

bool desktop_tas_panel(unsigned id);
const char *desktop_tas_edit_title(unsigned control);
void desktop_tas_layout(struct FrontendDesktopUi *ui, float width, float height);
bool desktop_tas_event(struct FrontendDesktopUi *ui, const SDL_Event *event);
bool desktop_tas_commit(struct FrontendDesktopUi *ui, const char *text, char *error, size_t size);
void desktop_tas_finish_paint(struct FrontendDesktopUi *ui);
void desktop_tas_destroy(struct FrontendDesktopUi *ui);

#endif
