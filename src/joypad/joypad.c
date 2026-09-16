/*
 * joypad.c - NES controller emulation
 *
 * Author: @frankischilling
 *
 * This file implements controller button state, strobe handling, input latching, and the
 * serial shift behavior used when the CPU reads the standard NES controller ports.
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
    uint8_t old_strobe = jp->strobe;
    jp->strobe = v & 1;
    if (old_strobe && !jp->strobe) jp->shift = jp->buttons;
}
  
uint8_t joypad_read(Joypad* jp){
    if (jp->strobe) jp->shift = jp->buttons; // strobe high exposes the live A button
    uint8_t ret = (jp->shift & 1u);             // LSB first; CPU bus layer supplies open-bus bits
    if (!jp->strobe) jp->shift = (jp->shift >> 1) | 0x80; // shift in 1s after 8 reads
    return ret;
}
