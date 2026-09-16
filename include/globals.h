/*
 * globals.h - Shared display constants and framebuffer declaration
 *
 * Author: @frankischilling
 *
 * This header defines the NES output dimensions and exposes the framebuffer shared by
 * the PPU renderer, SDL frontend, and hardware tests.
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
#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>

#define SCREEN_WIDTH 256
#define SCREEN_HEIGHT 240

// RGBA framebuffer shared by the PPU and SDL frontend.
extern uint32_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];

#endif /* GLOBALS_H */
