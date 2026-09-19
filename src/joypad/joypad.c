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
#include <limits.h>
#include <string.h>

static bool microphone_active;
extern Joypad pad1, pad2;
extern uint64_t cpu_total_cycles;
static Joypad expansion_pads[NES_INPUT_PLAYERS - 2];
static NesInputAdapter input_adapter;
static uint8_t configuration_overrides;
static uint8_t adapter_strobe;
static uint8_t adapter_remaining[2];
static uint8_t adapter_signature[2];
static const char *const adapter_names[] = {
    "none", "four-score", "famicom-2", "famicom-4"
};
static NesPortDevice port_devices[2];
static NesExpansionDevice expansion_device;

static const char *const port_device_names[] = {
    "pad", "none", "arkanoid", "power-pad-a", "power-pad-b", "zapper", "subor-mouse",
    "snes-pad", "snes-mouse", "ntt-keypad", "virtual-boy"
};

static const char *const expansion_device_names[] = {
    "none", "arkanoid", "family-trainer-a", "family-trainer-b", "zapper", "family-basic",
    "turbo-file", "battle-box", "subor-keyboard", "hori-track", "konami-hyper-shot",
    "bandai-hyper-shot", "party-tap", "pachinko", "exciting-boxing", "jissen-mahjong",
    "barcode-battler", "oeka-kids-tablet", "fcns"
};

typedef struct {
    int x;
    int y;
    bool trigger;
} Zapper;

static Zapper zappers[3] = {{-1, -1, false}, {-1, -1, false}, {-1, -1, false}};
static unsigned zapper_radius;

typedef struct {
    uint16_t extra_buttons;
    uint32_t shift;
    bool strobe;
} ExtendedSerialPad;

typedef struct {
    uint32_t shift;
    int dx;
    int dy;
    bool left;
    bool right;
    bool strobe;
    uint8_t sensitivity;
    uint8_t up_flag;
    uint8_t left_flag;
} SnesMouseState;

static ExtendedSerialPad extended_pads[2];
static uint32_t ntt_keys[2];
static uint16_t virtual_boy_extra[2];
static SnesMouseState snes_mice[2];
static uint16_t fcns_keys;
static uint32_t fcns_shift;
static bool fcns_strobe;

static bool extended_device(NesPortDevice device) {
    return device == NES_PORT_SNES_CONTROLLER || device == NES_PORT_NTT_KEYPAD
        || device == NES_PORT_VIRTUAL_BOY;
}

static bool pressed(uint8_t buttons, unsigned bit) {
    return (buttons & (1u << bit)) != 0;
}

static uint32_t fcns_report(void) {
    uint8_t buttons = pad1.buttons;
    bool up = pressed(buttons, BTN_UP), down = pressed(buttons, BTN_DOWN);
    bool left = pressed(buttons, BTN_LEFT), right = pressed(buttons, BTN_RIGHT);
    if (up && down) buttons &= (uint8_t)~((1u << BTN_UP) | (1u << BTN_DOWN));
    if (left && right) buttons &= (uint8_t)~((1u << BTN_LEFT) | (1u << BTN_RIGHT));
    uint32_t report = buttons;
    for (unsigned key = 0; key < 14; ++key)
        if (fcns_keys & (1u << key)) report |= 1u << (8 + key);
    if (fcns_keys & (1u << FCNS_KEY_END)) report |= 1u << 23;
    return report;
}

static void fcns_latch(void) { fcns_shift = fcns_report(); }

static uint8_t fcns_read(unsigned port) {
    if (port != 0) return 0;
    if (fcns_strobe) fcns_latch();
    uint8_t value = (uint8_t)((fcns_shift & 1u) << 1);
    fcns_shift = (fcns_shift >> 1) | 0x800000u;
    return value;
}

static void fcns_write(uint8_t value) {
    bool strobe = (value & 1u) != 0;
    if (fcns_strobe && !strobe) fcns_latch();
    fcns_strobe = strobe;
}

static void fcns_reset(void) {
    fcns_keys = 0;
    fcns_shift = 0;
    fcns_strobe = false;
}

static uint16_t snes_report(unsigned port) {
    uint8_t buttons = joypad_player(port)->buttons;
    uint16_t extra = extended_pads[port].extra_buttons;
    bool up = pressed(buttons, BTN_UP), down = pressed(buttons, BTN_DOWN);
    bool left = pressed(buttons, BTN_LEFT), right = pressed(buttons, BTN_RIGHT);
    if (up && down) up = down = false;
    if (left && right) left = right = false;
    return (pressed(buttons, BTN_B) ? 1u : 0u)
        | ((extra & (1u << SNES_BUTTON_Y)) ? (1u << 1) : 0u)
        | (pressed(buttons, BTN_SELECT) ? (1u << 2) : 0u)
        | (pressed(buttons, BTN_START) ? (1u << 3) : 0u)
        | (up ? (1u << 4) : 0u)
        | (down ? (1u << 5) : 0u)
        | (left ? (1u << 6) : 0u)
        | (right ? (1u << 7) : 0u)
        | (pressed(buttons, BTN_A) ? (1u << 8) : 0u)
        | ((extra & (1u << SNES_BUTTON_X)) ? (1u << 9) : 0u)
        | ((extra & (1u << SNES_BUTTON_L)) ? (1u << 10) : 0u)
        | ((extra & (1u << SNES_BUTTON_R)) ? (1u << 11) : 0u);
}

static uint32_t ntt_report(unsigned port) {
    uint32_t value = snes_report(port) | (1u << 13);
    value |= (ntt_keys[port] & 0x3FFFu) << 16;
    if (ntt_keys[port] & (1u << NTT_KEY_END)) value |= 1u << 31;
    return value;
}

static uint16_t virtual_boy_report(unsigned port) {
    uint8_t buttons = joypad_player(port)->buttons;
    uint16_t extra = virtual_boy_extra[port];
    bool up0 = pressed(buttons, BTN_UP), down0 = pressed(buttons, BTN_DOWN);
    bool left0 = pressed(buttons, BTN_LEFT), right0 = pressed(buttons, BTN_RIGHT);
    bool up1 = (extra & (1u << VB_BUTTON_UP1)) != 0;
    bool down1 = (extra & (1u << VB_BUTTON_DOWN1)) != 0;
    bool left1 = (extra & (1u << VB_BUTTON_LEFT1)) != 0;
    bool right1 = (extra & (1u << VB_BUTTON_RIGHT1)) != 0;
    if (up0 && down0) up0 = down0 = false;
    if (left0 && right0) left0 = right0 = false;
    if (up1 && down1) up1 = down1 = false;
    if (left1 && right1) left1 = right1 = false;
    return (down1 ? 1u : 0u)
        | (left1 ? (1u << 1) : 0u)
        | (pressed(buttons, BTN_SELECT) ? (1u << 2) : 0u)
        | (pressed(buttons, BTN_START) ? (1u << 3) : 0u)
        | (up0 ? (1u << 4) : 0u)
        | (down0 ? (1u << 5) : 0u)
        | (left0 ? (1u << 6) : 0u)
        | (right0 ? (1u << 7) : 0u)
        | (right1 ? (1u << 8) : 0u)
        | (up1 ? (1u << 9) : 0u)
        | ((extra & (1u << VB_BUTTON_L)) ? (1u << 10) : 0u)
        | ((extra & (1u << VB_BUTTON_R)) ? (1u << 11) : 0u)
        | (pressed(buttons, BTN_B) ? (1u << 12) : 0u)
        | (pressed(buttons, BTN_A) ? (1u << 13) : 0u)
        | (1u << 14);
}

static void latch_extended(unsigned port) {
    if (port_devices[port] == NES_PORT_SNES_CONTROLLER)
        extended_pads[port].shift = snes_report(port);
    else if (port_devices[port] == NES_PORT_NTT_KEYPAD)
        extended_pads[port].shift = ntt_report(port);
    else if (port_devices[port] == NES_PORT_VIRTUAL_BOY)
        extended_pads[port].shift = virtual_boy_report(port);
}

static uint8_t read_extended(unsigned port) {
    ExtendedSerialPad *device = &extended_pads[port];
    if (device->strobe) latch_extended(port);
    uint8_t value = (uint8_t)(device->shift & 1u);
    device->shift >>= 1;
    if (port_devices[port] == NES_PORT_NTT_KEYPAD) device->shift |= 0x80000000u;
    else device->shift |= 0x8000u;
    return value;
}

static void write_extended(unsigned port, uint8_t value) {
    ExtendedSerialPad *device = &extended_pads[port];
    bool strobe = (value & 1u) != 0;
    if (device->strobe && !strobe) latch_extended(port);
    device->strobe = strobe;
}

static int mouse_magnitude(int value) {
    if (value == INT_MIN) return 127;
    if (value < 0) value = -value;
    return value > 127 ? 127 : value;
}

static uint8_t mouse_scaled(uint8_t sensitivity, int magnitude) {
    static const uint8_t lut[2][8] = {
        {0, 1, 2, 3, 8, 10, 12, 21},
        {0, 1, 4, 9, 12, 20, 24, 28}
    };
    if (!sensitivity) return (uint8_t)magnitude;
    return lut[sensitivity - 1][magnitude < 7 ? magnitude : 7];
}

static void latch_snes_mouse(unsigned port) {
    SnesMouseState *mouse = &snes_mice[port];
    if (mouse->dx) mouse->left_flag = mouse->dx < 0 ? 0x80 : 0;
    if (mouse->dy) mouse->up_flag = mouse->dy < 0 ? 0x80 : 0;
    int dx = mouse_magnitude(mouse->dx);
    int dy = mouse_magnitude(mouse->dy);
    dx = mouse_scaled(mouse->sensitivity, dx);
    dy = mouse_scaled(mouse->sensitivity, dy);
    uint8_t byte2 = (uint8_t)(0x01 | ((mouse->sensitivity & 3u) << 4)
                    | (mouse->left ? 0x40 : 0) | (mouse->right ? 0x80 : 0));
    uint8_t byte3 = (uint8_t)dy | mouse->up_flag;
    uint8_t byte4 = (uint8_t)dx | mouse->left_flag;
    mouse->shift = ((uint32_t)byte2 << 16) | ((uint32_t)byte3 << 8) | byte4;
    mouse->dx = mouse->dy = 0;
}

static uint8_t read_snes_mouse(unsigned port) {
    SnesMouseState *mouse = &snes_mice[port];
    if (mouse->strobe) {
        latch_snes_mouse(port);
        mouse->sensitivity = (uint8_t)((mouse->sensitivity + 1) % 3);
    }
    uint8_t value = (uint8_t)(mouse->shift >> 31);
    mouse->shift = (mouse->shift << 1) | 1u;
    return value;
}

static void write_snes_mouse(unsigned port, uint8_t value) {
    SnesMouseState *mouse = &snes_mice[port];
    bool strobe = (value & 1u) != 0;
    if (mouse->strobe && !strobe) latch_snes_mouse(port);
    mouse->strobe = strobe;
}

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
        else if (port_devices[port] == NES_PORT_SNES_MOUSE)
            value = read_snes_mouse(port);
        else if (extended_device(port_devices[port]))
            value = read_extended(port);
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
    else if (expansion_device == NES_EXPANSION_OEKA_KIDS_TABLET)
        value |= oeka_kids_tablet_read(port);
    else if (expansion_device == NES_EXPANSION_FCNS_CONTROLLER)
        value |= fcns_read(port);
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
        else if (port_devices[port] == NES_PORT_SNES_MOUSE)
            write_snes_mouse(port, value);
        else if (extended_device(port_devices[port]))
            write_extended(port, value);
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
    else if (expansion_device == NES_EXPANSION_OEKA_KIDS_TABLET)
        oeka_kids_tablet_write(value);
    else if (expansion_device == NES_EXPANSION_FCNS_CONTROLLER)
        fcns_write(value);
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
    if (port >= 2 || (unsigned)device > NES_PORT_VIRTUAL_BOY) return false;
    if (device == NES_PORT_SUBOR_MOUSE && port != 1) return false;
    port_devices[port] = device;
    joypad_player(port)->buttons = 0;
    paddles[port].strobe = paddles[port].shift = 0;
    mats[port].strobe = mats[port].low = mats[port].high = 0;
    memset(&extended_pads[port], 0, sizeof(extended_pads[port]));
    memset(&snes_mice[port], 0, sizeof(snes_mice[port]));
    ntt_keys[port] = 0;
    virtual_boy_extra[port] = 0;
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
    if ((unsigned)device > NES_EXPANSION_FCNS_CONTROLLER) return false;
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
    oeka_kids_tablet_reset();
    fcns_reset();
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

void joypad_set_configuration_overrides(uint8_t mask) {
    configuration_overrides = mask & (NES_INPUT_OVERRIDE_ADAPTER | NES_INPUT_OVERRIDE_PORT1
                                    | NES_INPUT_OVERRIDE_PORT2 | NES_INPUT_OVERRIDE_EXPANSION);
}

uint8_t joypad_configuration_overrides(void) {
    return configuration_overrides;
}

static bool configuration_valid(const NesInputConfiguration *config) {
    if (!config) return false;
    if ((unsigned)config->adapter > NES_ADAPTER_FAMICOM_FOUR
        || (unsigned)config->ports[0] > NES_PORT_VIRTUAL_BOY
        || (unsigned)config->ports[1] > NES_PORT_VIRTUAL_BOY
        || (unsigned)config->expansion > NES_EXPANSION_FCNS_CONTROLLER)
        return false;
    if (config->ports[0] == NES_PORT_SUBOR_MOUSE) return false;
    if (config->adapter == NES_ADAPTER_FOUR_SCORE
        && (config->ports[0] != NES_PORT_GAMEPAD || config->ports[1] != NES_PORT_GAMEPAD))
        return false;
    return config->adapter < NES_ADAPTER_FAMICOM_TWO
        || config->expansion == NES_EXPANSION_NONE;
}

bool joypad_resolve_default_input_for_family(uint8_t input_type, bool famicom,
                                             NesInputConfiguration *config,
                                             bool *supported) {
    if (!config || !supported) return false;
    *supported = true;
    NesInputConfiguration automatic = {
        .adapter = NES_ADAPTER_NONE,
        .ports = {NES_PORT_GAMEPAD, NES_PORT_GAMEPAD},
        .expansion = NES_EXPANSION_NONE
    };

    switch (input_type) {
        case 0x01: break;
        case 0x02: automatic.adapter = NES_ADAPTER_FOUR_SCORE; break;
        case 0x03: automatic.adapter = NES_ADAPTER_FAMICOM_TWO; break;
        case 0x07: automatic.ports[0] = NES_PORT_ZAPPER; break;
        case 0x08:
            if (famicom) automatic.expansion = NES_EXPANSION_ZAPPER;
            else automatic.ports[1] = NES_PORT_ZAPPER;
            break;
        case 0x0A: automatic.expansion = NES_EXPANSION_BANDAI_HYPER_SHOT; break;
        case 0x0B: automatic.ports[1] = NES_PORT_POWER_PAD_A; break;
        case 0x0C: automatic.ports[1] = NES_PORT_POWER_PAD_B; break;
        case 0x0D: automatic.expansion = NES_EXPANSION_FAMILY_TRAINER_A; break;
        case 0x0E: automatic.expansion = NES_EXPANSION_FAMILY_TRAINER_B; break;
        case 0x0F: automatic.ports[1] = NES_PORT_ARKANOID; break;
        case 0x10:
        case 0x11: automatic.expansion = NES_EXPANSION_ARKANOID; break;
        case 0x12: automatic.expansion = NES_EXPANSION_KONAMI_HYPER_SHOT; break;
        case 0x13: automatic.expansion = NES_EXPANSION_PACHINKO; break;
        case 0x14: automatic.expansion = NES_EXPANSION_EXCITING_BOXING; break;
        case 0x15: automatic.expansion = NES_EXPANSION_JISSEN_MAHJONG; break;
        case 0x16: automatic.expansion = NES_EXPANSION_PARTY_TAP; break;
        case 0x17: automatic.expansion = NES_EXPANSION_OEKA_KIDS_TABLET; break;
        case 0x18: automatic.expansion = NES_EXPANSION_BARCODE_BATTLER; break;
        case 0x21: automatic.expansion = NES_EXPANSION_TURBO_FILE; break;
        case 0x22: automatic.expansion = NES_EXPANSION_BATTLE_BOX; break;
        case 0x23: automatic.expansion = NES_EXPANSION_FAMILY_BASIC; break;
        case 0x27:
            automatic.expansion = NES_EXPANSION_SUBOR_KEYBOARD;
            automatic.ports[1] = NES_PORT_SUBOR_MOUSE;
            break;
        case 0x2B:
            automatic.ports[0] = NES_PORT_SNES_CONTROLLER;
            automatic.ports[1] = NES_PORT_SNES_CONTROLLER;
            break;
        case 0x3B: automatic.expansion = NES_EXPANSION_FCNS_CONTROLLER; break;
        default:
            *supported = false;
            *config = (NesInputConfiguration){
                .adapter = input_adapter,
                .ports = {port_devices[0], port_devices[1]},
                .expansion = expansion_device
            };
            return true;
    }

    *config = automatic;
    if (configuration_overrides & NES_INPUT_OVERRIDE_ADAPTER) config->adapter = input_adapter;
    if (configuration_overrides & NES_INPUT_OVERRIDE_PORT1) config->ports[0] = port_devices[0];
    if (configuration_overrides & NES_INPUT_OVERRIDE_PORT2) config->ports[1] = port_devices[1];
    if (configuration_overrides & NES_INPUT_OVERRIDE_EXPANSION) config->expansion = expansion_device;
    return configuration_valid(config);
}

bool joypad_resolve_default_input(uint8_t input_type, NesInputConfiguration *config,
                                  bool *supported) {
    bool famicom = nes_console_model() == NES_CONSOLE_HVC001
                || nes_console_model() == NES_CONSOLE_HVC101;
    return joypad_resolve_default_input_for_family(input_type, famicom, config, supported);
}

bool joypad_apply_configuration(const NesInputConfiguration *config) {
    if (!configuration_valid(config)) return false;
    return joypad_set_adapter(config->adapter)
        && joypad_set_port_device(0, config->ports[0])
        && joypad_set_port_device(1, config->ports[1])
        && joypad_set_expansion_device(config->expansion);
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

bool joypad_set_snes_button(unsigned port, SnesButton button, bool is_pressed) {
    if (port >= 2 || (unsigned)button >= SNES_BUTTON_COUNT) return false;
    if (port_devices[port] != NES_PORT_SNES_CONTROLLER && port_devices[port] != NES_PORT_NTT_KEYPAD)
        return false;
    int nes_button = -1;
    switch (button) {
        case SNES_BUTTON_A: nes_button = BTN_A; break;
        case SNES_BUTTON_B: nes_button = BTN_B; break;
        case SNES_BUTTON_SELECT: nes_button = BTN_SELECT; break;
        case SNES_BUTTON_START: nes_button = BTN_START; break;
        case SNES_BUTTON_UP: nes_button = BTN_UP; break;
        case SNES_BUTTON_DOWN: nes_button = BTN_DOWN; break;
        case SNES_BUTTON_LEFT: nes_button = BTN_LEFT; break;
        case SNES_BUTTON_RIGHT: nes_button = BTN_RIGHT; break;
        default: break;
    }
    if (nes_button >= 0) return joypad_set_player(port, nes_button, is_pressed);
    uint16_t mask = (uint16_t)(1u << button);
    if (is_pressed) extended_pads[port].extra_buttons |= mask;
    else extended_pads[port].extra_buttons &= (uint16_t)~mask;
    return true;
}

bool joypad_add_snes_mouse_motion(unsigned port, int dx, int dy) {
    if (port >= 2 || port_devices[port] != NES_PORT_SNES_MOUSE) return false;
    SnesMouseState *mouse = &snes_mice[port];
    if ((dx > 0 && mouse->dx > INT_MAX - dx) || (dx < 0 && mouse->dx < INT_MIN - dx))
        mouse->dx = dx > 0 ? INT_MAX : INT_MIN;
    else
        mouse->dx += dx;
    if ((dy > 0 && mouse->dy > INT_MAX - dy) || (dy < 0 && mouse->dy < INT_MIN - dy))
        mouse->dy = dy > 0 ? INT_MAX : INT_MIN;
    else
        mouse->dy += dy;
    return true;
}

bool joypad_set_snes_mouse_buttons(unsigned port, bool left, bool right) {
    if (port >= 2 || port_devices[port] != NES_PORT_SNES_MOUSE) return false;
    snes_mice[port].left = left;
    snes_mice[port].right = right;
    return true;
}

bool joypad_set_ntt_key(unsigned port, NttKey key, bool is_pressed) {
    if (port >= 2 || port_devices[port] != NES_PORT_NTT_KEYPAD || (unsigned)key >= NTT_KEY_COUNT)
        return false;
    uint32_t mask = 1u << key;
    if (is_pressed) ntt_keys[port] |= mask;
    else ntt_keys[port] &= ~mask;
    return true;
}

bool joypad_set_fcns_key(FcnsKey key, bool is_pressed) {
    if (expansion_device != NES_EXPANSION_FCNS_CONTROLLER || (unsigned)key >= FCNS_KEY_COUNT)
        return false;
    uint16_t mask = (uint16_t)(1u << key);
    if (is_pressed) fcns_keys |= mask;
    else fcns_keys &= (uint16_t)~mask;
    return true;
}

bool joypad_set_virtual_boy_button(unsigned port, VirtualBoyButton button, bool is_pressed) {
    if (port >= 2 || port_devices[port] != NES_PORT_VIRTUAL_BOY || (unsigned)button >= VB_BUTTON_COUNT)
        return false;
    int nes_button = -1;
    switch (button) {
        case VB_BUTTON_SELECT: nes_button = BTN_SELECT; break;
        case VB_BUTTON_START: nes_button = BTN_START; break;
        case VB_BUTTON_UP0: nes_button = BTN_UP; break;
        case VB_BUTTON_DOWN0: nes_button = BTN_DOWN; break;
        case VB_BUTTON_LEFT0: nes_button = BTN_LEFT; break;
        case VB_BUTTON_RIGHT0: nes_button = BTN_RIGHT; break;
        case VB_BUTTON_B: nes_button = BTN_B; break;
        case VB_BUTTON_A: nes_button = BTN_A; break;
        default: break;
    }
    if (nes_button >= 0) return joypad_set_player(port, nes_button, is_pressed);
    uint16_t mask = (uint16_t)(1u << button);
    if (is_pressed) virtual_boy_extra[port] |= mask;
    else virtual_boy_extra[port] &= (uint16_t)~mask;
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

bool joypad_set_oeka_kids_tablet(int x, int y, bool touch, bool click) {
    if (expansion_device != NES_EXPANSION_OEKA_KIDS_TABLET) return false;
    oeka_kids_tablet_set_state(x, y, touch, click);
    return true;
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
