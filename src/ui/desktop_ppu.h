/*
 * desktop_ppu.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Visual PPU tools hosted by the desktop. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_DESKTOP_PPU_H
#define CUPID_DESKTOP_PPU_H
#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stddef.h>
struct FrontendDesktopUi;
bool desktop_ppu_panel(unsigned id);
bool desktop_ppu_register(void);
void desktop_ppu_unregister(void);
void desktop_ppu_layout(struct FrontendDesktopUi *ui, float width, float height);
bool desktop_ppu_event(struct FrontendDesktopUi *ui, const SDL_Event *event);
bool desktop_ppu_commit(struct FrontendDesktopUi *ui, const char *text, char *error, size_t size);
void desktop_ppu_destroy(struct FrontendDesktopUi *ui);
#endif
