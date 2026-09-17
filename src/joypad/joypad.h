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

typedef enum {
    NES_PORT_GAMEPAD,
    NES_PORT_NONE,
    NES_PORT_ARKANOID,
    NES_PORT_POWER_PAD_A,
    NES_PORT_POWER_PAD_B,
    NES_PORT_ZAPPER
} NesPortDevice;

typedef enum {
    NES_EXPANSION_NONE,
    NES_EXPANSION_ARKANOID,
    NES_EXPANSION_FAMILY_TRAINER_A,
    NES_EXPANSION_FAMILY_TRAINER_B,
    NES_EXPANSION_ZAPPER
} NesExpansionDevice;

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
NesPortDevice joypad_port_device(unsigned port);
bool    joypad_set_port_device(unsigned port, NesPortDevice device);
bool    joypad_set_port_device_name(unsigned port, const char *name);
const char *joypad_port_device_name(unsigned port);
NesExpansionDevice joypad_expansion_device(void);
bool    joypad_set_expansion_device(NesExpansionDevice device);
bool    joypad_set_expansion_device_name(const char *name);
const char *joypad_expansion_device_name(void);
bool    joypad_configuration_valid(void);
// Slots zero and one are NES ports; slot two is the Famicom expansion connector.
bool    joypad_set_paddle(unsigned slot, int position, bool fire);
// Mat positions are three rows of four, viewed from left to right on side A.
bool    joypad_set_mat_pad(unsigned slot, unsigned pad, bool pressed);
enum { NES_ZAPPER_MAX_RADIUS = 255 };
bool    joypad_set_zapper(unsigned slot, int x, int y, bool trigger);
unsigned joypad_zapper_radius(void);
bool    joypad_set_zapper_radius(unsigned radius);

#endif // JOYPAD_H
