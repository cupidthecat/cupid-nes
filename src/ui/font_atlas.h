/*
 * font_atlas.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Cached TrueType text for the desktop. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_FONT_ATLAS_H
#define CUPID_FONT_ATLAS_H
#include <SDL2/SDL.h>
typedef struct DesktopFont DesktopFont;
DesktopFont *desktop_font_create(SDL_Renderer *renderer);
void desktop_font_destroy(DesktopFont *font);
float desktop_font_measure(DesktopFont *font, const char *text, int length, float size);
/* Return a fitting byte prefix without splitting a UTF-8 codepoint. */
int desktop_font_fit(DesktopFont *font, const char *text, int length, float size, float width);
void desktop_font_draw(DesktopFont *font, SDL_Renderer *renderer, const char *text, int length, float x, float y,
                       float size, SDL_Color color);
#endif
