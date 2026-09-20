/*
 * ui_font.h - Tiny built-in bitmap font for the dependency-free SDL interface
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_UI_FONT_H
#define FRONTEND_UI_FONT_H

#include <SDL2/SDL.h>

int frontend_text_width(const char *text, int scale);
void frontend_draw_text(SDL_Renderer *renderer, int x, int y, int scale,
                        const char *text, Uint8 r, Uint8 g, Uint8 b, Uint8 a);

#endif
