/*
 * desktop_hex.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Hex editor grid and input. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_DESKTOP_HEX_H
#define CUPID_DESKTOP_HEX_H

#include <stdbool.h>
#include <SDL2/SDL.h>

struct FrontendDesktopUi;
bool desktop_hex_layout(struct FrontendDesktopUi *ui, float width, float height);
bool desktop_hex_event(struct FrontendDesktopUi *ui, const SDL_Event *event);

#endif
