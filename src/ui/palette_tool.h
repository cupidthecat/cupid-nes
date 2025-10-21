/*
 * palette_tool.h - Runtime palette tool overlay and picker UI
 */

#ifndef PALETTE_TOOL_H
#define PALETTE_TOOL_H

#include <SDL2/SDL.h>
#include <stdbool.h>

void palette_tool_init(void);
void palette_tool_toggle_overlay(void);
void palette_tool_hide_overlay(void);
bool palette_tool_is_visible(void);
void palette_tool_flash(bool ok);
void palette_tool_handle_event(const SDL_Event *e, SDL_Renderer *renderer);
void palette_tool_draw(SDL_Renderer *renderer, int ww, int hh);
void palette_tool_tick(Uint32 delta_ms);

#endif /* PALETTE_TOOL_H */


