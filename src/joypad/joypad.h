/*
 * joypad.h - NES controller interface
 *
 * Author: @frankischilling
 *
 * This header defines controller button identifiers, latched input state, strobe state,
 * and the functions used to update and read standard NES controller input.
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
#include <stdbool.h>

typedef struct {
    uint8_t buttons;   // bit0..bit7 = A,B,Select,Start,Up,Down,Left,Right
    uint8_t shift;     // latched/shifted copy
    uint8_t strobe;    // last write to $4016 bit0
} Joypad;

enum { BTN_A, BTN_B, BTN_SELECT, BTN_START, BTN_UP, BTN_DOWN, BTN_LEFT, BTN_RIGHT };

enum { NES_INPUT_PLAYERS = 6 };
typedef enum {
    NES_ADAPTER_NONE,
    NES_ADAPTER_FOUR_SCORE,
    NES_ADAPTER_FAMICOM_TWO,
    NES_ADAPTER_FAMICOM_FOUR
} NesInputAdapter;

void    joypad_set(Joypad* jp, int btn, int pressed);
void    joypad_write_strobe(Joypad* jp, uint8_t value);
uint8_t joypad_read(Joypad* jp);
// Port zero is $4016; port one is $4017.
uint8_t joypad_read_port(Joypad *jp, unsigned port);
uint8_t joypad_open_bus_mask(unsigned port);
bool    joypad_clocks_adjacent_reads(void);
void    joypad_set_microphone(bool active);
Joypad *joypad_player(unsigned player);
bool    joypad_set_player(unsigned player, int button, bool pressed);
void    joypad_write_ports(uint8_t value);
NesInputAdapter joypad_adapter(void);
bool    joypad_set_adapter(NesInputAdapter adapter);
bool    joypad_set_adapter_name(const char *name);
const char *joypad_adapter_name(void);

#endif // JOYPAD_H
