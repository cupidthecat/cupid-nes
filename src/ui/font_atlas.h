/* Cached TrueType text for the desktop. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_FONT_ATLAS_H
#define CUPID_FONT_ATLAS_H
#include <SDL2/SDL.h>
typedef struct DesktopFont DesktopFont;
DesktopFont *desktop_font_create(SDL_Renderer *renderer);
void desktop_font_destroy(DesktopFont *font);
float desktop_font_measure(DesktopFont *font, const char *text, int length, float size);
void desktop_font_draw(DesktopFont *font, SDL_Renderer *renderer, const char *text, int length, float x,
                       float y, float size, SDL_Color color);
#endif
