/*
 * palette_tool.h - Runtime palette tool overlay and picker UI
 */

#ifndef PALETTE_TOOL_H
#define PALETTE_TOOL_H

#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stdint.h>

// UI overlay functions
void palette_tool_init(void);
void palette_tool_toggle_overlay(void);
void palette_tool_hide_overlay(void);
bool palette_tool_is_visible(void);
void palette_tool_flash(bool ok);
void palette_tool_handle_event(const SDL_Event *e, SDL_Renderer *renderer);
void palette_tool_draw(SDL_Renderer *renderer, int ww, int hh);
void palette_tool_tick(Uint32 delta_ms);

// Palette management functions
void ppu_palette_reset_default(void);
void ppu_palette_get(uint32_t out[64]);
bool ppu_palette_has_emphasis_tables(void);
int ppu_palette_set_color(int index, uint8_t r, uint8_t g, uint8_t b);
int ppu_palette_load_pal_file(const char *path);
int ppu_palette_load_hex_string(const char *hex_text);

// Access to palette storage (for get_color in ppu.c)
extern uint32_t ppu__active_palette_base[64];
extern uint32_t ppu__emphasis_palettes[8][64];
extern bool     ppu__have_emphasis_tables;

#endif /* PALETTE_TOOL_H */


