/*
 * desktop_keyboard.h - Family BASIC keyboard window
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_DESKTOP_KEYBOARD_H
#define CUPID_DESKTOP_KEYBOARD_H
#include "desktop_ui.h"

enum { DESKTOP_KEYBOARD_PANEL = 0x2a00 };

bool desktop_keyboard_register(void);
void desktop_keyboard_layout(FrontendDesktopUi *ui, float width, float height);
bool desktop_keyboard_event(FrontendDesktopUi *ui, const SDL_Event *event);
void desktop_keyboard_release(FrontendDesktopUi *ui);
#endif
