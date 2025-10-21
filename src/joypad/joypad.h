/*
 * joypad.h - NES controller/joypad header
 * 
 * Author: @frankischilling
 * 
 * This header defines the joypad structure and button constants for NES controller emulation.
 * Provides interfaces for setting button states and reading controller data.
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

#ifndef JOYPAD_H
#define JOYPAD_H

#include <stdint.h>

typedef struct {
    uint8_t buttons;   // bit0..bit7 = A,B,Select,Start,Up,Down,Left,Right
    uint8_t shift;     // latched/shifted copy
    uint8_t strobe;    // last write to $4016 bit0
} Joypad;

enum { BTN_A, BTN_B, BTN_SELECT, BTN_START, BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT };

void    joypad_set(Joypad* jp, int btn, int pressed);
void    joypad_write_strobe(Joypad* jp, uint8_t value);
uint8_t joypad_read(Joypad* jp);

#endif // JOYPAD_H
