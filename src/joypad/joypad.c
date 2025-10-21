/*
 * joypad.c - NES controller/joypad emulation
 * 
 * Author: @frankischilling
 * 
 * This file implements NES controller input handling. It manages button states, shift register
 * behavior for serial reading, and strobe signal handling that matches the NES hardware behavior
 * for reading controller inputs through memory-mapped I/O.
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

#include "joypad.h"

void joypad_set(Joypad* jp, int btn, int pressed){
    if (pressed) jp->buttons |=  (1u << btn);
    else         jp->buttons &= ~(1u << btn);
}
  
void joypad_write_strobe(Joypad* jp, uint8_t v){
    jp->strobe = v & 1;
    if (jp->strobe) jp->shift = jp->buttons; // while strobe=1, continually latch
    else            jp->shift = jp->buttons; // snapshot on 1->0 (OK to write again)
}
  
uint8_t joypad_read(Joypad* jp){
    uint8_t ret = (jp->shift & 1u) | 0x40;     // LSB first, bit6=1 (open bus)
    if (!jp->strobe) jp->shift = (jp->shift >> 1) | 0x80; // shift in 1s after 8 reads
    else             jp->shift = jp->buttons;             // strobe=1 -> always A
    return ret;
}