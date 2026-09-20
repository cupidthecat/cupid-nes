/*
 * palette_tool.h - Runtime palette editor interface
 *
 * Author: @frankischilling
 *
 * This header defines the palette overlay controls, SDL event and drawing hooks, runtime
 * palette editing functions, palette file parsers, and shared palette storage used by the PPU.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef PALETTE_TOOL_H
#define PALETTE_TOOL_H

#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stdint.h>

// Overlay controls and event handling.
void palette_tool_init(void);
void palette_tool_toggle_overlay(void);
void palette_tool_hide_overlay(void);
bool palette_tool_is_visible(void);

// Runtime palette management.
void ppu_palette_reset_default(void);
void ppu_palette_get(uint32_t out[64]);
bool ppu_palette_has_emphasis_tables(void);
int ppu_palette_set_color(int index, uint8_t r, uint8_t g, uint8_t b);
int ppu_palette_load_pal_file(const char *path);
int ppu_palette_load_hex_string(const char *hex_text);

// Shared palette storage used by the PPU color lookup path.
extern uint32_t ppu__active_palette_base[64];
extern uint32_t ppu__emphasis_palettes[8][64];
extern bool     ppu__have_emphasis_tables;

#endif /* PALETTE_TOOL_H */
