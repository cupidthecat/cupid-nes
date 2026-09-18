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
    NES_PORT_ZAPPER,
    NES_PORT_SUBOR_MOUSE
} NesPortDevice;

typedef enum {
    NES_EXPANSION_NONE,
    NES_EXPANSION_ARKANOID,
    NES_EXPANSION_FAMILY_TRAINER_A,
    NES_EXPANSION_FAMILY_TRAINER_B,
    NES_EXPANSION_ZAPPER,
    NES_EXPANSION_FAMILY_BASIC,
    NES_EXPANSION_TURBO_FILE,
    NES_EXPANSION_BATTLE_BOX,
    NES_EXPANSION_SUBOR_KEYBOARD,
    NES_EXPANSION_HORI_TRACK,
    NES_EXPANSION_KONAMI_HYPER_SHOT,
    NES_EXPANSION_BANDAI_HYPER_SHOT,
    NES_EXPANSION_PARTY_TAP
} NesExpansionDevice;

typedef enum {
    SUBOR_KEY_A, SUBOR_KEY_B, SUBOR_KEY_C, SUBOR_KEY_D, SUBOR_KEY_E, SUBOR_KEY_F,
    SUBOR_KEY_G, SUBOR_KEY_H, SUBOR_KEY_I, SUBOR_KEY_J, SUBOR_KEY_K, SUBOR_KEY_L,
    SUBOR_KEY_M, SUBOR_KEY_N, SUBOR_KEY_O, SUBOR_KEY_P, SUBOR_KEY_Q, SUBOR_KEY_R,
    SUBOR_KEY_S, SUBOR_KEY_T, SUBOR_KEY_U, SUBOR_KEY_V, SUBOR_KEY_W, SUBOR_KEY_X,
    SUBOR_KEY_Y, SUBOR_KEY_Z,
    SUBOR_KEY_0, SUBOR_KEY_1, SUBOR_KEY_2, SUBOR_KEY_3, SUBOR_KEY_4,
    SUBOR_KEY_5, SUBOR_KEY_6, SUBOR_KEY_7, SUBOR_KEY_8, SUBOR_KEY_9,
    SUBOR_KEY_F1, SUBOR_KEY_F2, SUBOR_KEY_F3, SUBOR_KEY_F4, SUBOR_KEY_F5, SUBOR_KEY_F6,
    SUBOR_KEY_F7, SUBOR_KEY_F8, SUBOR_KEY_F9, SUBOR_KEY_F10, SUBOR_KEY_F11, SUBOR_KEY_F12,
    SUBOR_KEY_KP0, SUBOR_KEY_KP1, SUBOR_KEY_KP2, SUBOR_KEY_KP3, SUBOR_KEY_KP4,
    SUBOR_KEY_KP5, SUBOR_KEY_KP6, SUBOR_KEY_KP7, SUBOR_KEY_KP8, SUBOR_KEY_KP9,
    SUBOR_KEY_KP_ENTER, SUBOR_KEY_KP_DOT, SUBOR_KEY_KP_PLUS, SUBOR_KEY_KP_MULTIPLY,
    SUBOR_KEY_KP_DIVIDE, SUBOR_KEY_KP_MINUS, SUBOR_KEY_NUMLOCK,
    SUBOR_KEY_COMMA, SUBOR_KEY_DOT, SUBOR_KEY_SEMICOLON, SUBOR_KEY_APOSTROPHE,
    SUBOR_KEY_SLASH, SUBOR_KEY_BACKSLASH, SUBOR_KEY_EQUAL, SUBOR_KEY_MINUS,
    SUBOR_KEY_GRAVE, SUBOR_KEY_LEFT_BRACKET, SUBOR_KEY_RIGHT_BRACKET,
    SUBOR_KEY_CAPSLOCK, SUBOR_KEY_PAUSE, SUBOR_KEY_CTRL, SUBOR_KEY_SHIFT, SUBOR_KEY_ALT,
    SUBOR_KEY_SPACE, SUBOR_KEY_BACKSPACE, SUBOR_KEY_TAB, SUBOR_KEY_ESCAPE,
    SUBOR_KEY_ENTER, SUBOR_KEY_END, SUBOR_KEY_HOME, SUBOR_KEY_INSERT, SUBOR_KEY_DELETE,
    SUBOR_KEY_PAGEUP, SUBOR_KEY_PAGEDOWN, SUBOR_KEY_UP, SUBOR_KEY_DOWN,
    SUBOR_KEY_LEFT, SUBOR_KEY_RIGHT, SUBOR_KEY_UNKNOWN1, SUBOR_KEY_UNKNOWN2,
    SUBOR_KEY_UNKNOWN3, SUBOR_KEY_COUNT
} SuborKey;

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
uint8_t joypad_zapper_serial_report(unsigned slot);
unsigned joypad_zapper_radius(void);
bool    joypad_set_zapper_radius(unsigned radius);
bool    joypad_set_subor_key(SuborKey key, bool pressed);
bool    joypad_add_subor_mouse_motion(int dx, int dy);
bool    joypad_set_subor_mouse_buttons(bool left, bool right);
bool    joypad_add_hori_track_motion(int dx, int dy);
bool    joypad_set_party_tap_button(unsigned button, bool pressed);
bool    joypad_persistent_configure(const char *rom_path);
bool    joypad_persistent_flush(void);
bool    joypad_persistent_shutdown(void);

#endif // JOYPAD_H
