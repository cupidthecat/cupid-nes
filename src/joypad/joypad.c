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
#include "family_basic.h"
#include "special_peripherals.h"
#include "../system/hardware.h"
#include "../system/timing.h"
#include "../ppu/ppu.h"
#include <string.h>

static bool microphone_active;
extern Joypad pad1, pad2;
extern uint64_t cpu_total_cycles;
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
static const char *const port_device_names[] = {
    "pad", "none", "arkanoid", "power-pad-a", "power-pad-b", "zapper", "subor-mouse"
};
static const char *const expansion_device_names[] = {
    "none", "arkanoid", "family-trainer-a", "family-trainer-b", "zapper", "family-basic",
    "turbo-file", "battle-box", "subor-keyboard", "hori-track", "konami-hyper-shot",
    "bandai-hyper-shot", "party-tap", "pachinko", "exciting-boxing", "jissen-mahjong",
    "barcode-battler"
};

typedef struct {
    int x;
    int y;
    bool trigger;
} Zapper;

static Zapper zappers[3] = {{-1, -1, false}, {-1, -1, false}, {-1, -1, false}};
static unsigned zapper_radius;

static bool zapper_light(const Zapper *zapper) {
    int radius = (int)zapper_radius;
    if (zapper->x < 0 || zapper->y < 0 || zapper->x > 255 + radius || zapper->y > 239 + radius)
        return false;
    int scanline = ppu.scanline == (int)nes_timing()->scanlines - 1 ? -1 : ppu.scanline;
    int cycle = ppu.dot - 1; // PPU.dot names the next clock to execute.
    int left = zapper->x > radius ? zapper->x - radius : 0;
    int right = zapper->x + radius < 255 ? zapper->x + radius : 255;
    int top = zapper->y > radius ? zapper->y - radius : 0;
    int bottom = zapper->y + radius < 239 ? zapper->y + radius : 239;
    for (int y = top; y <= bottom; ++y) {
        // This sensor model retains light through the twentieth following line.
        if (scanline < y || scanline - y > 20) continue;
        for (int x = left; x <= right; ++x) {
            if (scanline == y && cycle <= x) continue;
            if (ppu_pixel_brightness((unsigned)x, (unsigned)y) >= 85) return true;
        }
    }
    return false;
}

static uint8_t read_zapper(unsigned slot) {
    return (zapper_light(&zappers[slot]) ? 0 : 0x08) | (zappers[slot].trigger ? 0x10 : 0);
}

typedef struct {
    uint16_t buttons;
    uint8_t low;
    uint8_t high;
    uint8_t strobe;
} Mat;

static Mat mats[3];
static uint8_t family_trainer_rows;

static bool mat_side_b(unsigned slot) {
    return slot == 2 ? expansion_device == NES_EXPANSION_FAMILY_TRAINER_B
                     : port_devices[slot] == NES_PORT_POWER_PAD_B;
}

static uint8_t mat_button(unsigned slot, unsigned button) {
    if (mat_side_b(slot)) button = (button & ~3u) | (3u - (button & 3u));
    return (uint8_t)((mats[slot].buttons >> button) & 1u);
}

static void latch_mat(unsigned slot) {
    static const uint8_t low_order[] = {1, 0, 4, 8, 5, 9, 10, 6};
    static const uint8_t high_order[] = {3, 2, 11, 7};
    Mat *mat = &mats[slot];
    mat->low = 0;
    mat->high = 0xF0;
    for (unsigned bit = 0; bit < 8; ++bit) mat->low |= mat_button(slot, low_order[bit]) << bit;
    for (unsigned bit = 0; bit < 4; ++bit) mat->high |= mat_button(slot, high_order[bit]) << bit;
}

static uint8_t read_mat(unsigned slot) {
    Mat *mat = &mats[slot];
    if (mat->strobe) latch_mat(slot);
    uint8_t value = ((mat->low & 1u) << 3) | ((mat->high & 1u) << 4);
    mat->low = (mat->low >> 1) | 0x80;
    mat->high = (mat->high >> 1) | 0x80;
    return value;
}

static void write_mat(unsigned slot, uint8_t value) {
    uint8_t strobe = value & 1u;
    if (mats[slot].strobe && !strobe) latch_mat(slot);
    mats[slot].strobe = strobe;
}

static uint8_t read_family_trainer(void) {
    uint8_t pressed = 0;
    for (unsigned row = 0; row < 3; ++row) {
        if (family_trainer_rows & (1u << (2 - row))) continue;
        for (unsigned column = 0; column < 4; ++column)
            pressed |= mat_button(2, row * 4 + column) << (4 - column);
    }
    return (uint8_t)(~pressed & 0x1E);
}

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
        else if (port_devices[port] == NES_PORT_POWER_PAD_A || port_devices[port] == NES_PORT_POWER_PAD_B)
            value = read_mat(port);
        else if (port_devices[port] == NES_PORT_ZAPPER)
            value = read_zapper(port);
        else if (port_devices[port] == NES_PORT_SUBOR_MOUSE)
            value = subor_mouse_read();
        else
            value = port_devices[port] == NES_PORT_GAMEPAD ? joypad_read(jp) : 0;
        if (input_adapter == NES_ADAPTER_FAMICOM_TWO)
            value |= joypad_read(&expansion_pads[port]) << 1;
        else if (input_adapter == NES_ADAPTER_FAMICOM_FOUR)
            value |= read_adapter(port) << 1;
    }
    if (expansion_device == NES_EXPANSION_ARKANOID)
        value |= port == 0 ? (paddles[2].fire ? 0x02 : 0) : (read_paddle(&paddles[2]) << 1);
    else if (port == 1 && (expansion_device == NES_EXPANSION_FAMILY_TRAINER_A
                          || expansion_device == NES_EXPANSION_FAMILY_TRAINER_B))
        value |= read_family_trainer();
    else if (port == 1 && expansion_device == NES_EXPANSION_ZAPPER)
        value |= read_zapper(2);
    else if (expansion_device == NES_EXPANSION_FAMILY_BASIC)
        value |= family_basic_read(port, cpu_total_cycles);
    else if (expansion_device == NES_EXPANSION_TURBO_FILE)
        value |= turbo_file_read(port);
    else if (expansion_device == NES_EXPANSION_BATTLE_BOX)
        value |= battle_box_read(port);
    else if (expansion_device == NES_EXPANSION_SUBOR_KEYBOARD)
        value |= subor_keyboard_read(port);
    else if (port == 0 && expansion_device == NES_EXPANSION_HORI_TRACK)
        value |= hori_track_read(pad1.buttons);
    else if (expansion_device == NES_EXPANSION_KONAMI_HYPER_SHOT)
        value |= konami_hyper_shot_read(port, pad1.buttons, pad2.buttons);
    else if (expansion_device == NES_EXPANSION_BANDAI_HYPER_SHOT)
        value |= port == 0 ? bandai_hyper_shot_read(pad1.buttons) : read_zapper(2);
    else if (expansion_device == NES_EXPANSION_PARTY_TAP)
        value |= party_tap_read(port);
    else if (port == 0 && expansion_device == NES_EXPANSION_PACHINKO)
        value |= pachinko_read(pad1.buttons);
    else if (expansion_device == NES_EXPANSION_EXCITING_BOXING)
        value |= exciting_boxing_read(port);
    else if (expansion_device == NES_EXPANSION_JISSEN_MAHJONG)
        value |= jissen_mahjong_read(port);
    else if (expansion_device == NES_EXPANSION_BARCODE_BATTLER)
        value |= barcode_battler_read(port, cpu_total_cycles, (uint32_t)nes_timing()->cpu_hz);
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
    for (unsigned port = 0; port < 2; ++port) {
        if (port_devices[port] == NES_PORT_ARKANOID) write_paddle(&paddles[port], value);
        else if (port_devices[port] == NES_PORT_POWER_PAD_A || port_devices[port] == NES_PORT_POWER_PAD_B)
            write_mat(port, value);
        else if (port_devices[port] == NES_PORT_SUBOR_MOUSE)
            subor_mouse_write(value);
    }
    if (expansion_device == NES_EXPANSION_ARKANOID) write_paddle(&paddles[2], value);
    else if (expansion_device == NES_EXPANSION_FAMILY_TRAINER_A
             || expansion_device == NES_EXPANSION_FAMILY_TRAINER_B)
        family_trainer_rows = value & 7u;
    else if (expansion_device == NES_EXPANSION_FAMILY_BASIC)
        family_basic_write(value, cpu_total_cycles);
    else if (expansion_device == NES_EXPANSION_TURBO_FILE)
        turbo_file_write(value);
    else if (expansion_device == NES_EXPANSION_BATTLE_BOX)
        battle_box_write(value);
    else if (expansion_device == NES_EXPANSION_SUBOR_KEYBOARD)
        subor_keyboard_write(value);
    else if (expansion_device == NES_EXPANSION_HORI_TRACK)
        hori_track_write(value, pad1.buttons);
    else if (expansion_device == NES_EXPANSION_KONAMI_HYPER_SHOT)
        konami_hyper_shot_write(value);
    else if (expansion_device == NES_EXPANSION_BANDAI_HYPER_SHOT)
        bandai_hyper_shot_write(value, pad1.buttons);
    else if (expansion_device == NES_EXPANSION_PARTY_TAP)
        party_tap_write(value);
    else if (expansion_device == NES_EXPANSION_PACHINKO)
        pachinko_write(value, pad1.buttons);
    else if (expansion_device == NES_EXPANSION_EXCITING_BOXING)
        exciting_boxing_write(value);
    else if (expansion_device == NES_EXPANSION_JISSEN_MAHJONG)
        jissen_mahjong_write(value);
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
    if (port >= 2 || (unsigned)device > NES_PORT_SUBOR_MOUSE) return false;
    if (device == NES_PORT_SUBOR_MOUSE && port != 1) return false;
    port_devices[port] = device;
    paddles[port].strobe = paddles[port].shift = 0;
    mats[port].strobe = mats[port].low = mats[port].high = 0;
    if (device == NES_PORT_SUBOR_MOUSE) subor_mouse_reset();
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
    if ((unsigned)device > NES_EXPANSION_BARCODE_BATTLER) return false;
    expansion_device = device;
    paddles[2].strobe = paddles[2].shift = 0;
    family_trainer_rows = 0;
    family_basic_reset();
    turbo_file_reset_protocol();
    battle_box_reset_protocol();
    subor_keyboard_reset();
    hori_track_reset();
    konami_hyper_shot_reset();
    bandai_hyper_shot_reset();
    party_tap_reset();
    pachinko_reset();
    exciting_boxing_reset();
    jissen_mahjong_reset();
    barcode_battler_reset();
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

bool joypad_set_mat_pad(unsigned slot, unsigned pad, bool pressed) {
    if (slot >= 3 || pad >= 12) return false;
    if (pressed) mats[slot].buttons |= (uint16_t)(1u << pad);
    else mats[slot].buttons &= (uint16_t)~(1u << pad);
    return true;
}

bool joypad_set_zapper(unsigned slot, int x, int y, bool trigger) {
    if (slot >= 3) return false;
    zappers[slot] = (Zapper){x, y, trigger};
    return true;
}

uint8_t joypad_zapper_serial_report(unsigned slot) {
    if (slot >= 3) return 0;
    return (uint8_t)(0x10
        | (zapper_light(&zappers[slot]) ? 0x40 : 0)
        | (zappers[slot].trigger ? 0x80 : 0));
}

unsigned joypad_zapper_radius(void) {
    return zapper_radius;
}

bool joypad_set_zapper_radius(unsigned radius) {
    if (radius > NES_ZAPPER_MAX_RADIUS) return false;
    zapper_radius = radius;
    return true;
}

bool joypad_set_subor_key(SuborKey key, bool pressed) {
    return subor_keyboard_set_key((unsigned)key, pressed);
}

bool joypad_add_subor_mouse_motion(int dx, int dy) {
    if (port_devices[1] != NES_PORT_SUBOR_MOUSE) return false;
    subor_mouse_add_motion(dx, dy);
    return true;
}

bool joypad_set_subor_mouse_buttons(bool left, bool right) {
    if (port_devices[1] != NES_PORT_SUBOR_MOUSE) return false;
    subor_mouse_set_buttons(left, right);
    return true;
}

bool joypad_add_hori_track_motion(int dx, int dy) {
    if (expansion_device != NES_EXPANSION_HORI_TRACK) return false;
    hori_track_add_motion(dx, dy);
    return true;
}

bool joypad_set_party_tap_button(unsigned button, bool pressed) {
    if (expansion_device != NES_EXPANSION_PARTY_TAP) return false;
    return party_tap_set_button(button, pressed);
}

bool joypad_set_pachinko_controls(bool press, bool release) {
    if (expansion_device != NES_EXPANSION_PACHINKO) return false;
    pachinko_set_controls(press, release);
    return true;
}

bool joypad_set_boxing_sensor(unsigned sensor, bool pressed) {
    if (expansion_device != NES_EXPANSION_EXCITING_BOXING) return false;
    return exciting_boxing_set_sensor(sensor, pressed);
}

bool joypad_set_jissen_key(JissenKey key, bool pressed) {
    if (expansion_device != NES_EXPANSION_JISSEN_MAHJONG) return false;
    return jissen_mahjong_set_key((unsigned)key, pressed);
}

bool joypad_scan_barcode_battler(const char *digits) {
    if (expansion_device != NES_EXPANSION_BARCODE_BATTLER) return false;
    return barcode_battler_scan(digits, cpu_total_cycles);
}

bool joypad_persistent_configure(const char *rom_path) {
    if (expansion_device == NES_EXPANSION_TURBO_FILE) return turbo_file_configure(rom_path);
    if (expansion_device == NES_EXPANSION_BATTLE_BOX) return battle_box_configure(rom_path);
    return true;
}

bool joypad_persistent_flush(void) {
    if (expansion_device == NES_EXPANSION_TURBO_FILE) return turbo_file_flush();
    if (expansion_device == NES_EXPANSION_BATTLE_BOX) return battle_box_flush();
    return true;
}

bool joypad_persistent_shutdown(void) {
    bool turbo_ok = turbo_file_shutdown();
    bool battle_ok = battle_box_shutdown();
    return turbo_ok && battle_ok;
}
