/*
 * ui_font.c - Tiny built-in bitmap font for the dependency-free SDL interface
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "ui_font.h"
#include <ctype.h>
#include <string.h>

typedef struct {
    char character;
    Uint8 rows[7];
} Glyph;

#define GLYPH(c,a,b,d,e,f,g,h) {c,{a,b,d,e,f,g,h}}
static const Glyph glyphs[] = {
    GLYPH('A',14,17,17,31,17,17,17), GLYPH('B',30,17,17,30,17,17,30),
    GLYPH('C',14,17,16,16,16,17,14), GLYPH('D',28,18,17,17,17,18,28),
    GLYPH('E',31,16,16,30,16,16,31), GLYPH('F',31,16,16,30,16,16,16),
    GLYPH('G',14,17,16,23,17,17,15), GLYPH('H',17,17,17,31,17,17,17),
    GLYPH('I',14,4,4,4,4,4,14), GLYPH('J',7,2,2,2,18,18,12),
    GLYPH('K',17,18,20,24,20,18,17), GLYPH('L',16,16,16,16,16,16,31),
    GLYPH('M',17,27,21,21,17,17,17), GLYPH('N',17,25,21,19,17,17,17),
    GLYPH('O',14,17,17,17,17,17,14), GLYPH('P',30,17,17,30,16,16,16),
    GLYPH('Q',14,17,17,17,21,18,13), GLYPH('R',30,17,17,30,20,18,17),
    GLYPH('S',15,16,16,14,1,1,30), GLYPH('T',31,4,4,4,4,4,4),
    GLYPH('U',17,17,17,17,17,17,14), GLYPH('V',17,17,17,17,17,10,4),
    GLYPH('W',17,17,17,21,21,21,10), GLYPH('X',17,17,10,4,10,17,17),
    GLYPH('Y',17,17,10,4,4,4,4), GLYPH('Z',31,1,2,4,8,16,31),
    GLYPH('0',14,17,19,21,25,17,14), GLYPH('1',4,12,4,4,4,4,14),
    GLYPH('2',14,17,1,2,4,8,31), GLYPH('3',30,1,1,14,1,1,30),
    GLYPH('4',2,6,10,18,31,2,2), GLYPH('5',31,16,16,30,1,1,30),
    GLYPH('6',14,16,16,30,17,17,14), GLYPH('7',31,1,2,4,8,8,8),
    GLYPH('8',14,17,17,14,17,17,14), GLYPH('9',14,17,17,15,1,1,14),
    GLYPH('.',0,0,0,0,0,12,12), GLYPH(',',0,0,0,0,4,4,8),
    GLYPH(':',0,12,12,0,12,12,0), GLYPH(';',0,12,12,0,4,4,8),
    GLYPH('-',0,0,0,31,0,0,0), GLYPH('_',0,0,0,0,0,0,31),
    GLYPH('/',1,2,2,4,8,8,16), GLYPH('\\',16,8,8,4,2,2,1),
    GLYPH('(',2,4,8,8,8,4,2), GLYPH(')',8,4,2,2,2,4,8),
    GLYPH('[',14,8,8,8,8,8,14), GLYPH(']',14,2,2,2,2,2,14),
    GLYPH('+',0,4,4,31,4,4,0), GLYPH('=',0,31,0,31,0,0,0),
    GLYPH('%',17,2,4,8,16,17,0), GLYPH('!',4,4,4,4,4,0,4),
    GLYPH('?',14,17,1,2,4,0,4), GLYPH('<',2,4,8,16,8,4,2),
    GLYPH('>',8,4,2,1,2,4,8), GLYPH('|',4,4,4,4,4,4,4),
    GLYPH('*',0,21,14,31,14,21,0), GLYPH('#',10,31,10,10,31,10,0),
    GLYPH('"',10,10,0,0,0,0,0), GLYPH('\'',4,4,0,0,0,0,0)
};
#undef GLYPH

static const Uint8 *glyph_rows(char character) {
    if (character >= 'a' && character <= 'z') character = (char)toupper((unsigned char)character);
    for (size_t i = 0; i < sizeof(glyphs) / sizeof(glyphs[0]); ++i)
        if (glyphs[i].character == character) return glyphs[i].rows;
    return NULL;
}

int frontend_text_width(const char *text, int scale) {
    if (!text || scale < 1) return 0;
    return (int)strlen(text) * 6 * scale;
}

void frontend_draw_text(SDL_Renderer *renderer, int x, int y, int scale,
                        const char *text, Uint8 r, Uint8 g, Uint8 b, Uint8 a) {
    if (!renderer || !text || scale < 1) return;
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    SDL_SetRenderDrawColor(renderer, r, g, b, a);
    float sx, sy;
    SDL_RenderGetScale(renderer, &sx, &sy);
    SDL_RenderSetScale(renderer, 1.0f, 1.0f);
    for (const char *p = text; *p; ++p, x += 6 * scale) {
        if (*p == ' ') continue;
        const Uint8 *rows = glyph_rows(*p);
        if (!rows) {
            SDL_Rect box = {(int)(x*sx), (int)(y*sy), (int)((x+5*scale)*sx)-(int)(x*sx),
                            (int)((y+7*scale)*sy)-(int)(y*sy)};
            SDL_RenderDrawRect(renderer, &box);
            continue;
        }
        for (int row = 0; row < 7; ++row) {
            for (int column = 0; column < 5; ++column) {
                if (!(rows[row] & (1u << (4 - column)))) continue;
                int px = x + column * scale, py = y + row * scale;
                SDL_Rect pixel = {(int)(px*sx), (int)(py*sy),
                                  (int)((px+scale)*sx)-(int)(px*sx),
                                  (int)((py+scale)*sy)-(int)(py*sy)};
                SDL_RenderFillRect(renderer, &pixel);
            }
        }
    }
    SDL_RenderSetScale(renderer, sx, sy);
}
