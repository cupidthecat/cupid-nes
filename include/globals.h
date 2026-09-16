/* SPDX-License-Identifier: GPL-3.0-or-later
 * Shared display dimensions and framebuffer declaration.
 */
#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>

#define SCREEN_WIDTH 256
#define SCREEN_HEIGHT 240

// RGBA framebuffer shared by the PPU and SDL frontend.
extern uint32_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];

#endif /* GLOBALS_H */
