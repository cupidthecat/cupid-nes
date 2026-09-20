/* Clay layout and SDL rendering. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_CLAY_BACKEND_H
#define CUPID_CLAY_BACKEND_H
#include "../third_party/clay/clay.h"
#include <SDL2/SDL.h>
typedef struct DesktopClay DesktopClay;
typedef struct {
    Clay_ElementId id;
    int kind, index, direction;
    Clay_BoundingBox bounds;
} DesktopHit;
enum {
    HIT_NONE,
    HIT_MENU,
    HIT_MENU_ROW,
    HIT_COMMAND,
    HIT_CATEGORY,
    HIT_SETTING,
    HIT_SETTINGS_BUTTON,
    HIT_PANEL,
    HIT_CLOSE,
    HIT_RECENT,
    HIT_COLOR,
    HIT_CHANNEL,
    HIT_BROWSE,
    HIT_EDIT_OK,
    HIT_EDIT_CANCEL,
    HIT_SCROLL, HIT_SCROLLBAR, HIT_CHOICE, HIT_LOG, HIT_CHOICE_PAGE, HIT_CLEAR_SETTING
};
DesktopClay *desktop_clay_create(SDL_Renderer *renderer);
void desktop_clay_destroy(DesktopClay *clay);
void desktop_clay_begin(DesktopClay *clay, float width, float height, float scale);
void desktop_clay_end(DesktopClay *clay);
Clay_String desktop_clay_string(DesktopClay *clay, const char *text);
Clay_ElementId desktop_clay_hit(DesktopClay *clay, int kind, int index, int direction);
void desktop_clay_block(DesktopClay *clay);
const DesktopHit *desktop_clay_at(DesktopClay *clay, float x, float y);
bool desktop_clay_bounds(DesktopClay *clay, int kind, int index, int direction, SDL_FRect *bounds);
/* Inspect text submitted by the most recent layout for render regression checks. */
bool desktop_clay_contains_text(const DesktopClay *clay, const char *text);
#endif
