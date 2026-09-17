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
#include "../system/hardware.h"
#include "../system/timing.h"
#include <string.h>

static bool microphone_active;
extern Joypad pad1, pad2;
static Joypad expansion_pads[NES_INPUT_PLAYERS - 2];
static NesInputAdapter input_adapter;
static uint8_t adapter_strobe;
static uint8_t adapter_remaining[2];
static uint8_t adapter_signature[2];
static const char *const adapter_names[] = {
    "none", "four-score", "famicom-2", "famicom-4"
};
static NesPortDevice port_devices[2];
static NesExpansionDevice expansion_device;
static const char *const port_device_names[] = {"pad", "none", "arkanoid"};
static const char *const expansion_device_names[] = {"none", "arkanoid"};

typedef struct {
    uint8_t position;
    uint8_t shift;
    uint8_t strobe;
    bool fire;
} Paddle;

static Paddle paddles[3] = {
    {.position = 0x54}, {.position = 0x54}, {.position = 0x54}
};

static uint8_t read_paddle(Paddle *paddle) {
    uint8_t value = (paddle->shift & 0x80u) ? 0 : 1;
    paddle->shift <<= 1;
    return value;
}

static void write_paddle(Paddle *paddle, uint8_t value) {
    uint8_t strobe = value & 1u;
    if (paddle->strobe && !strobe) paddle->shift = paddle->position;
    paddle->strobe = strobe;
}

static void latch_adapter(void) {
    adapter_remaining[0] = adapter_remaining[1] = 16;
    adapter_signature[0] = input_adapter == NES_ADAPTER_FOUR_SCORE ? 0x08 : 0x04;
    adapter_signature[1] = input_adapter == NES_ADAPTER_FOUR_SCORE ? 0x04 : 0x08;
}

static uint8_t read_adapter(unsigned port) {
    if (adapter_strobe) latch_adapter();
    if (adapter_remaining[port]) {
        unsigned player = port;
        if (input_adapter == NES_ADAPTER_FAMICOM_FOUR) player += 2;
        if (--adapter_remaining[port] < 8) player += 2;
        return joypad_read(joypad_player(player));
    }
    uint8_t value = adapter_signature[port] & 1u;
    adapter_signature[port] = (adapter_signature[port] >> 1) | 0x80;
    return value;
}

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

uint8_t joypad_read_port(Joypad *jp, unsigned port) {
    uint8_t value;
    if (input_adapter == NES_ADAPTER_FOUR_SCORE) {
        value = read_adapter(port);
    } else {
        if (port_devices[port] == NES_PORT_ARKANOID)
            value = (read_paddle(&paddles[port]) << 4) | (paddles[port].fire ? 0x08 : 0);
        else
            value = port_devices[port] == NES_PORT_GAMEPAD ? joypad_read(jp) : 0;
        if (input_adapter == NES_ADAPTER_FAMICOM_TWO)
            value |= joypad_read(&expansion_pads[port]) << 1;
        else if (input_adapter == NES_ADAPTER_FAMICOM_FOUR)
            value |= read_adapter(port) << 1;
    }
    if (expansion_device == NES_EXPANSION_ARKANOID)
        value |= port == 0 ? (paddles[2].fire ? 0x02 : 0) : (read_paddle(&paddles[2]) << 1);
    // The second built-in controller's microphone reaches $4016 D2.
    if (port == 0 && nes_console_model() == NES_CONSOLE_HVC001 && microphone_active)
        value |= 0x04;
    return value;
}

uint8_t joypad_open_bus_mask(unsigned port) {
    if (port != 0) return 0xE0;
    switch (nes_console_model()) {
        case NES_CONSOLE_NES101: return 0xE4;
        case NES_CONSOLE_HVC001:
        case NES_CONSOLE_HVC101: return 0xF8;
        default: return 0xE0;
    }
}

bool joypad_clocks_adjacent_reads(void) {
    return nes_console_model() == NES_CONSOLE_HVC001 &&
           nes_timing()->region == NES_REGION_NTSC;
}

void joypad_set_microphone(bool active) {
    microphone_active = active;
}

Joypad *joypad_player(unsigned player) {
    if (player == 0) return &pad1;
    if (player == 1) return &pad2;
    return player < NES_INPUT_PLAYERS ? &expansion_pads[player - 2] : NULL;
}

bool joypad_set_player(unsigned player, int button, bool pressed) {
    Joypad *pad = joypad_player(player);
    if (!pad || (unsigned)button > BTN_RIGHT) return false;
    joypad_set(pad, button, pressed);
    return true;
}

void joypad_write_ports(uint8_t value) {
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player)
        joypad_write_strobe(joypad_player(player), value);
    uint8_t strobe = value & 1u;
    if (adapter_strobe && !strobe) latch_adapter();
    adapter_strobe = strobe;
    for (unsigned port = 0; port < 2; ++port)
        if (port_devices[port] == NES_PORT_ARKANOID) write_paddle(&paddles[port], value);
    if (expansion_device == NES_EXPANSION_ARKANOID) write_paddle(&paddles[2], value);
}

NesInputAdapter joypad_adapter(void) {
    return input_adapter;
}

bool joypad_set_adapter(NesInputAdapter adapter) {
    if ((unsigned)adapter > NES_ADAPTER_FAMICOM_FOUR) return false;
    input_adapter = adapter;
    adapter_strobe = 0;
    memset(adapter_remaining, 0, sizeof(adapter_remaining));
    memset(adapter_signature, 0, sizeof(adapter_signature));
    return true;
}

bool joypad_set_adapter_name(const char *name) {
    if (!name) return false;
    for (unsigned i = 0; i < sizeof(adapter_names) / sizeof(adapter_names[0]); ++i) {
        if (strcmp(name, adapter_names[i]) == 0)
            return joypad_set_adapter((NesInputAdapter)i);
    }
    return false;
}

const char *joypad_adapter_name(void) {
    return adapter_names[input_adapter];
}

NesPortDevice joypad_port_device(unsigned port) {
    return port < 2 ? port_devices[port] : NES_PORT_NONE;
}

bool joypad_set_port_device(unsigned port, NesPortDevice device) {
    if (port >= 2 || (unsigned)device > NES_PORT_ARKANOID) return false;
    port_devices[port] = device;
    paddles[port].strobe = paddles[port].shift = 0;
    return true;
}

bool joypad_set_port_device_name(unsigned port, const char *name) {
    if (!name) return false;
    for (unsigned i = 0; i < sizeof(port_device_names) / sizeof(port_device_names[0]); ++i) {
        if (strcmp(name, port_device_names[i]) == 0)
            return joypad_set_port_device(port, (NesPortDevice)i);
    }
    return false;
}

const char *joypad_port_device_name(unsigned port) {
    return port_device_names[joypad_port_device(port)];
}

NesExpansionDevice joypad_expansion_device(void) {
    return expansion_device;
}

bool joypad_set_expansion_device(NesExpansionDevice device) {
    if ((unsigned)device > NES_EXPANSION_ARKANOID) return false;
    expansion_device = device;
    paddles[2].strobe = paddles[2].shift = 0;
    return true;
}

bool joypad_set_expansion_device_name(const char *name) {
    if (!name) return false;
    for (unsigned i = 0; i < sizeof(expansion_device_names) / sizeof(expansion_device_names[0]); ++i) {
        if (strcmp(name, expansion_device_names[i]) == 0)
            return joypad_set_expansion_device((NesExpansionDevice)i);
    }
    return false;
}

const char *joypad_expansion_device_name(void) {
    return expansion_device_names[expansion_device];
}

bool joypad_configuration_valid(void) {
    if (input_adapter == NES_ADAPTER_FOUR_SCORE &&
        (port_devices[0] != NES_PORT_GAMEPAD || port_devices[1] != NES_PORT_GAMEPAD))
        return false;
    return input_adapter < NES_ADAPTER_FAMICOM_TWO || expansion_device == NES_EXPANSION_NONE;
}

bool joypad_set_paddle(unsigned slot, int position, bool fire) {
    if (slot >= 3) return false;
    if (position < 0x54) position = 0x54;
    if (position > 0xF4) position = 0xF4;
    paddles[slot].position = (uint8_t)position;
    paddles[slot].fire = fire;
    return true;
}
