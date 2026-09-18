/*
 * special_peripherals.c - Famicom expansion peripheral hardware
 *
 * Author: @frankischilling
 *
 * This file implements expansion devices whose protocols do not fit the
 * standard controller shift register.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "special_peripherals.h"
#include "joypad.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

enum { TURBO_FILE_SIZE = 0x2000, TURBO_FILE_BITS = TURBO_FILE_SIZE * 8 };

typedef struct {
    uint8_t data[TURBO_FILE_SIZE];
    uint16_t position;
    uint8_t last_write;
    bool dirty;
    char *save_path;
} TurboFile;

static TurboFile turbo_file;

typedef struct {
    uint8_t data[512];
    uint8_t last_write;
    uint8_t address;
    bool chip_select;
    bool output;
    bool write_enabled;
    uint8_t input_bit;
    uint16_t input_data;
    bool writing;
    bool reading;
    bool dirty;
    char *save_path;
} BattleBox;

static BattleBox battle_box;

typedef struct {
    bool pressed[99];
    uint8_t row;
    uint8_t column;
    bool enabled;
    bool strobe;
} SuborKeyboard;

typedef struct {
    int dx;
    int dy;
    bool left;
    bool right;
    uint8_t state;
    uint8_t packet[3];
    uint8_t packet_pos;
    uint8_t packet_size;
    bool strobe;
} SuborMouse;

static SuborKeyboard subor_keyboard;
static SuborMouse subor_mouse = {.packet_size = 1};

typedef struct {
    int dx;
    int dy;
    uint32_t state;
    bool strobe;
} HoriTrack;

static HoriTrack hori_track;

typedef struct {
    bool enable_p1;
    bool enable_p2;
} KonamiHyperShot;

static KonamiHyperShot konami_hyper_shot = {true, true};

typedef struct {
    uint8_t state;
    bool strobe;
} BandaiHyperShot;

static BandaiHyperShot bandai_hyper_shot;

typedef struct {
    bool buttons[6];
    uint8_t state;
    uint8_t read_count;
    bool strobe;
} PartyTap;

static PartyTap party_tap;

typedef struct {
    uint8_t position;
    uint16_t state;
    bool press;
    bool release;
    bool strobe;
} Pachinko;

static Pachinko pachinko;

typedef struct {
    bool sensors[8];
    uint8_t selected;
} ExcitingBoxing;

static ExcitingBoxing exciting_boxing;

enum { SUBOR_NONE = 0xFF };
static const uint8_t subor_matrix[104] = {
    SUBOR_KEY_4, SUBOR_KEY_G, SUBOR_KEY_F, SUBOR_KEY_C,
    SUBOR_KEY_F2, SUBOR_KEY_E, SUBOR_KEY_5, SUBOR_KEY_V,
    SUBOR_KEY_2, SUBOR_KEY_D, SUBOR_KEY_S, SUBOR_KEY_END,
    SUBOR_KEY_F1, SUBOR_KEY_W, SUBOR_KEY_3, SUBOR_KEY_X,
    SUBOR_KEY_INSERT, SUBOR_KEY_BACKSPACE, SUBOR_KEY_PAGEDOWN, SUBOR_KEY_RIGHT,
    SUBOR_KEY_F8, SUBOR_KEY_PAGEUP, SUBOR_KEY_DELETE, SUBOR_KEY_HOME,
    SUBOR_KEY_9, SUBOR_KEY_I, SUBOR_KEY_L, SUBOR_KEY_COMMA,
    SUBOR_KEY_F5, SUBOR_KEY_O, SUBOR_KEY_0, SUBOR_KEY_DOT,
    SUBOR_KEY_RIGHT_BRACKET, SUBOR_KEY_ENTER, SUBOR_KEY_UP, SUBOR_KEY_LEFT,
    SUBOR_KEY_F7, SUBOR_KEY_LEFT_BRACKET, SUBOR_KEY_BACKSLASH, SUBOR_KEY_DOWN,
    SUBOR_KEY_Q, SUBOR_KEY_CAPSLOCK, SUBOR_KEY_Z, SUBOR_KEY_TAB,
    SUBOR_KEY_ESCAPE, SUBOR_KEY_A, SUBOR_KEY_1, SUBOR_KEY_CTRL,
    SUBOR_KEY_7, SUBOR_KEY_Y, SUBOR_KEY_K, SUBOR_KEY_M,
    SUBOR_KEY_F4, SUBOR_KEY_U, SUBOR_KEY_8, SUBOR_KEY_J,
    SUBOR_KEY_MINUS, SUBOR_KEY_SEMICOLON, SUBOR_KEY_APOSTROPHE, SUBOR_KEY_SLASH,
    SUBOR_KEY_F6, SUBOR_KEY_P, SUBOR_KEY_EQUAL, SUBOR_KEY_SHIFT,
    SUBOR_KEY_T, SUBOR_KEY_H, SUBOR_KEY_N, SUBOR_KEY_SPACE,
    SUBOR_KEY_F3, SUBOR_KEY_R, SUBOR_KEY_6, SUBOR_KEY_B,
    SUBOR_KEY_KP6, SUBOR_KEY_KP_ENTER, SUBOR_KEY_KP4, SUBOR_KEY_KP8,
    SUBOR_NONE, SUBOR_KEY_UNKNOWN1, SUBOR_KEY_UNKNOWN2, SUBOR_KEY_UNKNOWN3,
    SUBOR_KEY_ALT, SUBOR_KEY_KP4, SUBOR_KEY_KP7, SUBOR_KEY_F11,
    SUBOR_KEY_F12, SUBOR_KEY_KP1, SUBOR_KEY_KP2, SUBOR_KEY_KP8,
    SUBOR_KEY_KP_MINUS, SUBOR_KEY_KP_PLUS, SUBOR_KEY_KP_MULTIPLY, SUBOR_KEY_KP9,
    SUBOR_KEY_F10, SUBOR_KEY_KP5, SUBOR_KEY_KP_DIVIDE, SUBOR_KEY_NUMLOCK,
    SUBOR_KEY_GRAVE, SUBOR_KEY_KP6, SUBOR_KEY_PAUSE, SUBOR_KEY_SPACE,
    SUBOR_KEY_F9, SUBOR_KEY_KP3, SUBOR_KEY_KP_DOT, SUBOR_KEY_KP0
};

static char *storage_path(const char *rom_path, const char *suffix) {
    if (!rom_path || !*rom_path || !suffix) return NULL;
    const char *slash = strrchr(rom_path, '/');
    const char *backslash = strrchr(rom_path, '\\');
    const char *base = slash;
    if (!base || (backslash && backslash > base)) base = backslash;
    const char *dot = strrchr(rom_path, '.');
    if (dot && base && dot < base) dot = NULL;
    size_t stem = dot ? (size_t)(dot - rom_path) : strlen(rom_path);
    size_t suffix_len = strlen(suffix);
    if (stem > SIZE_MAX - suffix_len - 1) return NULL;
    char *path = malloc(stem + suffix_len + 1);
    if (!path) return NULL;
    memcpy(path, rom_path, stem);
    memcpy(path + stem, suffix, suffix_len + 1);
    return path;
}

static bool atomic_save(const char *path, const uint8_t *data, size_t size) {
    if (!path || !data) return false;
    size_t path_len = strlen(path);
    if (path_len > SIZE_MAX - 40) return false;
    char *temporary = malloc(path_len + 40);
    if (!temporary) return false;
    FILE *file = NULL;
    for (unsigned serial = 0; serial < 1000 && !file; ++serial) {
        snprintf(temporary, path_len + 40, "%s.cupid-%u.tmp", path, serial);
        file = fopen(temporary, "wbx");
        if (!file && errno != EEXIST) break;
    }
    if (!file) {
        free(temporary);
        return false;
    }
    bool ok = fwrite(data, 1, size, file) == size;
    if (fclose(file) != 0) ok = false;
    if (ok) {
#ifdef _WIN32
        ok = MoveFileExA(temporary, path,
                         MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != 0;
#else
        ok = rename(temporary, path) == 0;
#endif
    }
    if (!ok) remove(temporary);
    free(temporary);
    return ok;
}

static bool load_exact(const char *path, uint8_t *output, size_t size) {
    FILE *file = fopen(path, "rb");
    if (!file) return errno == ENOENT;
    uint8_t *temporary = malloc(size);
    if (!temporary) {
        fclose(file);
        return false;
    }
    bool ok = fread(temporary, 1, size, file) == size && fgetc(file) == EOF && !ferror(file);
    if (fclose(file) != 0) ok = false;
    if (ok) memcpy(output, temporary, size);
    free(temporary);
    return ok;
}

void turbo_file_reset_protocol(void) {
    turbo_file.position = 0;
    turbo_file.last_write = 0;
}

uint8_t turbo_file_read(unsigned port) {
    if (port != 1) return 0;
    unsigned position = turbo_file.position;
    return (uint8_t)(((turbo_file.data[position >> 3] >> (position & 7u)) & 1u) << 2);
}

void turbo_file_write(uint8_t value) {
    if (!(value & 0x02)) turbo_file.position = 0;
    if (!(value & 0x04) && (turbo_file.last_write & 0x04)) {
        unsigned position = turbo_file.position;
        uint8_t mask = (uint8_t)(1u << (position & 7u));
        uint8_t old = turbo_file.data[position >> 3];
        uint8_t next = value & 1u ? (uint8_t)(old | mask) : (uint8_t)(old & (uint8_t)~mask);
        if (next != old) {
            turbo_file.data[position >> 3] = next;
            turbo_file.dirty = true;
        }
        turbo_file.position = (uint16_t)((position + 1u) & (TURBO_FILE_BITS - 1u));
    }
    turbo_file.last_write = value;
}

bool turbo_file_flush(void) {
    if (!turbo_file.dirty) return true;
    if (!turbo_file.save_path || !atomic_save(turbo_file.save_path, turbo_file.data,
                                               sizeof(turbo_file.data))) return false;
    turbo_file.dirty = false;
    return true;
}

bool turbo_file_configure(const char *rom_path) {
    if (!turbo_file_flush()) return false;
    char *path = storage_path(rom_path, ".turbofile.sav");
    if (!path) return false;
    uint8_t loaded[TURBO_FILE_SIZE] = {0};
    if (!load_exact(path, loaded, sizeof(loaded))) {
        free(path);
        return false;
    }
    free(turbo_file.save_path);
    turbo_file.save_path = path;
    memcpy(turbo_file.data, loaded, sizeof(loaded));
    turbo_file.dirty = false;
    turbo_file_reset_protocol();
    return true;
}

bool turbo_file_shutdown(void) {
    if (turbo_file.dirty && !turbo_file.save_path) {
        memset(&turbo_file, 0, sizeof(turbo_file));
        return true;
    }
    if (!turbo_file_flush()) return false;
    free(turbo_file.save_path);
    memset(&turbo_file, 0, sizeof(turbo_file));
    return true;
}

static uint16_t battle_word(unsigned chip, unsigned address) {
    unsigned offset = (chip * 128u + (address & 0x7Fu)) * 2u;
    return (uint16_t)(battle_box.data[offset] | ((uint16_t)battle_box.data[offset + 1] << 8));
}

static void battle_store_word(unsigned chip, unsigned address, uint16_t value) {
    unsigned offset = (chip * 128u + (address & 0x7Fu)) * 2u;
    uint8_t low = (uint8_t)value;
    uint8_t high = (uint8_t)(value >> 8);
    if (battle_box.data[offset] != low || battle_box.data[offset + 1] != high) {
        battle_box.data[offset] = low;
        battle_box.data[offset + 1] = high;
        battle_box.dirty = true;
    }
}

void battle_box_reset_protocol(void) {
    battle_box.last_write = 0;
    battle_box.address = 0;
    battle_box.chip_select = false;
    battle_box.output = false;
    battle_box.write_enabled = false;
    battle_box.input_bit = 0;
    battle_box.input_data = 0;
    battle_box.writing = false;
    battle_box.reading = false;
}

uint8_t battle_box_read(unsigned port) {
    if (port != 1) return 0;
    if (battle_box.last_write & 1u) {
        battle_box.chip_select = !battle_box.chip_select;
        battle_box.input_data = 0;
        battle_box.input_bit = 0;
    }
    battle_box.output = !battle_box.output;
    uint8_t value = battle_box.output ? 0x10 : 0;
    if (battle_box.reading) {
        uint16_t word = battle_word(battle_box.chip_select ? 1u : 0u, battle_box.address);
        value |= (uint8_t)(((word >> battle_box.input_bit) & 1u) << 3);
    }
    return value;
}

static void battle_box_command(uint16_t value) {
    if (battle_box.writing) {
        battle_store_word(battle_box.chip_select ? 1u : 0u, battle_box.address, value);
        battle_box.writing = false;
        return;
    }
    battle_box.reading = false;
    uint8_t address = (uint8_t)(value & 0x7Fu);
    uint8_t command = (uint8_t)(((value & 0x7F00u) >> 8) ^ 0x7Fu);
    switch (command) {
        case 0x01:
            battle_box.address = address;
            battle_box.reading = true;
            break;
        case 0x06:
            if (battle_box.write_enabled) {
                battle_box.address = address;
                battle_box.writing = true;
            }
            break;
        case 0x0C:
            if (battle_box.write_enabled) {
                bool changed = false;
                for (size_t i = 0; i < sizeof(battle_box.data); ++i) {
                    if (battle_box.data[i]) changed = true;
                    battle_box.data[i] = 0;
                }
                battle_box.dirty |= changed;
            }
            break;
        case 0x09:
            battle_box.write_enabled = true;
            break;
        case 0x0B:
            battle_box.write_enabled = false;
            break;
        default:
            break;
    }
}

void battle_box_write(uint8_t value) {
    if (!(battle_box.last_write & 1u) && (value & 1u)) {
        if (battle_box.output)
            battle_box.input_data |= (uint16_t)(1u << battle_box.input_bit);
        if (++battle_box.input_bit == 16) {
            battle_box_command(battle_box.input_data);
            battle_box.input_bit = 0;
            battle_box.input_data = 0;
        }
    }
    battle_box.last_write = value;
}

bool battle_box_flush(void) {
    if (!battle_box.dirty) return true;
    if (!battle_box.save_path || !atomic_save(battle_box.save_path, battle_box.data,
                                               sizeof(battle_box.data))) return false;
    battle_box.dirty = false;
    return true;
}

bool battle_box_configure(const char *rom_path) {
    if (!battle_box_flush()) return false;
    char *path = storage_path(rom_path, ".battlebox.sav");
    if (!path) return false;
    uint8_t loaded[sizeof(battle_box.data)] = {0};
    if (!load_exact(path, loaded, sizeof(loaded))) {
        free(path);
        return false;
    }
    free(battle_box.save_path);
    battle_box.save_path = path;
    memcpy(battle_box.data, loaded, sizeof(loaded));
    battle_box.dirty = false;
    battle_box_reset_protocol();
    return true;
}

bool battle_box_shutdown(void) {
    if (battle_box.dirty && !battle_box.save_path) {
        memset(&battle_box, 0, sizeof(battle_box));
        return true;
    }
    if (!battle_box_flush()) return false;
    free(battle_box.save_path);
    memset(&battle_box, 0, sizeof(battle_box));
    return true;
}

void subor_keyboard_reset(void) {
    subor_keyboard.row = 0;
    subor_keyboard.column = 0;
    subor_keyboard.enabled = false;
    subor_keyboard.strobe = false;
}

bool subor_keyboard_set_key(unsigned key, bool pressed) {
    if (key >= sizeof(subor_keyboard.pressed) / sizeof(subor_keyboard.pressed[0])) return false;
    subor_keyboard.pressed[key] = pressed;
    return true;
}

static uint8_t subor_keyboard_active(void) {
    unsigned base = subor_keyboard.row * 8u + (subor_keyboard.column ? 4u : 0u);
    uint8_t result = 0;
    for (unsigned bit = 0; bit < 4; ++bit) {
        uint8_t key = subor_matrix[base + bit];
        if (key != SUBOR_NONE && subor_keyboard.pressed[key]) result |= (uint8_t)(1u << bit);
    }
    if (subor_keyboard.row == 9 && subor_keyboard.column) result |= 1u;
    return result;
}

uint8_t subor_keyboard_read(unsigned port) {
    if (port != 1) return 0;
    return subor_keyboard.enabled
        ? (uint8_t)((~(unsigned)subor_keyboard_active() << 1) & 0x1Eu) : 0x1E;
}

void subor_keyboard_write(uint8_t value) {
    bool old_strobe = subor_keyboard.strobe;
    subor_keyboard.strobe = (value & 1u) != 0;
    if (old_strobe && !subor_keyboard.strobe) {
        subor_keyboard.row = 0;
        subor_keyboard.column = 0;
    }
    uint8_t previous_column = subor_keyboard.column;
    subor_keyboard.column = (uint8_t)((value >> 1) & 1u);
    subor_keyboard.enabled = (value & 4u) != 0;
    if (subor_keyboard.enabled && previous_column && !subor_keyboard.column)
        subor_keyboard.row = (uint8_t)((subor_keyboard.row + 1u) % 13u);
}

static void subor_mouse_refresh(void) {
    if (subor_mouse.packet_pos + 1u < subor_mouse.packet_size) {
        subor_mouse.packet_pos++;
        subor_mouse.state = subor_mouse.packet[subor_mouse.packet_pos];
        return;
    }
    int dx = subor_mouse.dx;
    int dy = subor_mouse.dy;
    subor_mouse.dx = 0;
    subor_mouse.dy = 0;
    bool leftward = dx < 0;
    bool upward = dy < 0;
    int xmag = dx < 0 ? -dx : dx;
    int ymag = dy < 0 ? -dy : dy;
    if (xmag > 31) xmag = 31;
    if (ymag > 31) ymag = 31;
    if (xmag <= 1 && ymag <= 1) {
        subor_mouse.packet[0] =
            (subor_mouse.left ? 0x80 : 0) |
            (subor_mouse.right ? 0x40 : 0) |
            (leftward && xmag ? 0x30 : (xmag ? 0x10 : 0)) |
            (upward && ymag ? 0x0C : (ymag ? 0x04 : 0));
        subor_mouse.packet[1] = 0;
        subor_mouse.packet[2] = 0;
        subor_mouse.packet_size = 1;
    } else {
        subor_mouse.packet[0] =
            (subor_mouse.left ? 0x80 : 0) |
            (subor_mouse.right ? 0x40 : 0) |
            (leftward ? 0x20 : 0) |
            (xmag & 0x10) |
            (upward ? 0x08 : 0) |
            ((ymag & 0x10) >> 2) | 0x01;
        subor_mouse.packet[1] = (uint8_t)(((xmag & 0x0F) << 2) | 0x02);
        subor_mouse.packet[2] = (uint8_t)(((ymag & 0x0F) << 2) | 0x03);
        subor_mouse.packet_size = 3;
    }
    subor_mouse.packet_pos = 0;
    subor_mouse.state = subor_mouse.packet[0];
}

void subor_mouse_reset(void) {
    memset(&subor_mouse, 0, sizeof(subor_mouse));
    subor_mouse.packet_size = 1;
}

void subor_mouse_add_motion(int dx, int dy) {
    long long x = (long long)subor_mouse.dx + dx;
    long long y = (long long)subor_mouse.dy + dy;
    if (x > 32767) x = 32767;
    if (x < -32768) x = -32768;
    if (y > 32767) y = 32767;
    if (y < -32768) y = -32768;
    subor_mouse.dx = (int)x;
    subor_mouse.dy = (int)y;
}

void subor_mouse_set_buttons(bool left, bool right) {
    subor_mouse.left = left;
    subor_mouse.right = right;
}

uint8_t subor_mouse_read(void) {
    if (subor_mouse.strobe) subor_mouse_refresh();
    uint8_t result = (uint8_t)((subor_mouse.state >> 7) & 1u);
    subor_mouse.state <<= 1;
    return result;
}

void subor_mouse_write(uint8_t value) {
    bool strobe = (value & 1u) != 0;
    if (subor_mouse.strobe && !strobe) subor_mouse_refresh();
    subor_mouse.strobe = strobe;
}

static uint8_t reverse_nibble(unsigned value) {
    value &= 0x0Fu;
    return (uint8_t)(((value & 1u) << 3) | ((value & 2u) << 1)
                   | ((value & 4u) >> 1) | ((value & 8u) >> 3));
}

static void hori_track_refresh(uint8_t buttons) {
    int dx = hori_track.dx;
    int dy = hori_track.dy;
    if (dx < -8) dx = -8;
    if (dx > 7) dx = 7;
    if (dy < -8) dy = -8;
    if (dy > 7) dy = 7;
    hori_track.dx = 0;
    hori_track.dy = 0;
    uint8_t movement = (uint8_t)((~reverse_nibble((unsigned)dy) & 0x0Fu)
                      | ((~reverse_nibble((unsigned)dx) & 0x0Fu) << 4));
    hori_track.state = (uint32_t)buttons | ((uint32_t)movement << 8) | (0x09u << 16);
}

void hori_track_reset(void) {
    memset(&hori_track, 0, sizeof(hori_track));
}

void hori_track_add_motion(int dx, int dy) {
    long long x = (long long)hori_track.dx + dx;
    long long y = (long long)hori_track.dy + dy;
    if (x > 32767) x = 32767;
    if (x < -32768) x = -32768;
    if (y > 32767) y = 32767;
    if (y < -32768) y = -32768;
    hori_track.dx = (int)x;
    hori_track.dy = (int)y;
}

void hori_track_write(uint8_t value, uint8_t buttons) {
    bool strobe = (value & 1u) != 0;
    if (hori_track.strobe && !strobe) hori_track_refresh(buttons);
    hori_track.strobe = strobe;
}

uint8_t hori_track_read(uint8_t buttons) {
    if (hori_track.strobe) hori_track_refresh(buttons);
    uint8_t output = (uint8_t)((hori_track.state & 1u) << 1);
    hori_track.state >>= 1;
    return output;
}

void konami_hyper_shot_reset(void) {
    konami_hyper_shot.enable_p1 = true;
    konami_hyper_shot.enable_p2 = true;
}

void konami_hyper_shot_write(uint8_t value) {
    konami_hyper_shot.enable_p2 = (value & 0x02u) == 0;
    konami_hyper_shot.enable_p1 = (value & 0x04u) == 0;
}

uint8_t konami_hyper_shot_read(unsigned port, uint8_t player1, uint8_t player2) {
    if (port != 1) return 0;
    uint8_t value = 0;
    if (konami_hyper_shot.enable_p1) {
        if (player1 & (1u << BTN_A)) value |= 0x02;
        if (player1 & (1u << BTN_B)) value |= 0x04;
    }
    if (konami_hyper_shot.enable_p2) {
        if (player2 & (1u << BTN_A)) value |= 0x08;
        if (player2 & (1u << BTN_B)) value |= 0x10;
    }
    return value;
}

void bandai_hyper_shot_reset(void) {
    memset(&bandai_hyper_shot, 0, sizeof(bandai_hyper_shot));
}

void bandai_hyper_shot_write(uint8_t value, uint8_t buttons) {
    bool strobe = (value & 1u) != 0;
    if (bandai_hyper_shot.strobe && !strobe) bandai_hyper_shot.state = buttons;
    bandai_hyper_shot.strobe = strobe;
}

uint8_t bandai_hyper_shot_read(uint8_t buttons) {
    if (bandai_hyper_shot.strobe) bandai_hyper_shot.state = buttons;
    uint8_t output = (uint8_t)((bandai_hyper_shot.state & 1u) << 1);
    bandai_hyper_shot.state >>= 1;
    return output;
}

static void party_tap_latch(void) {
    party_tap.state = 0;
    for (unsigned button = 0; button < 6; ++button)
        if (party_tap.buttons[button]) party_tap.state |= (uint8_t)(1u << button);
    party_tap.read_count = 0;
}

void party_tap_reset(void) {
    party_tap.state = 0;
    party_tap.read_count = 0;
    party_tap.strobe = false;
}

bool party_tap_set_button(unsigned button, bool pressed) {
    if (button >= 6) return false;
    party_tap.buttons[button] = pressed;
    return true;
}

void party_tap_write(uint8_t value) {
    bool strobe = (value & 1u) != 0;
    if (party_tap.strobe && !strobe) party_tap_latch();
    party_tap.strobe = strobe;
}

uint8_t party_tap_read(unsigned port) {
    if (port != 1) return 0;
    if (party_tap.strobe) party_tap_latch();
    if (party_tap.read_count >= 2) return 0x14;
    uint8_t value = (uint8_t)((party_tap.state & 7u) << 2);
    party_tap.state >>= 3;
    party_tap.read_count++;
    return value;
}

static uint8_t reverse_byte(uint8_t value) {
    value = (uint8_t)(((value & 0x55u) << 1) | ((value & 0xAAu) >> 1));
    value = (uint8_t)(((value & 0x33u) << 2) | ((value & 0xCCu) >> 2));
    return (uint8_t)((value << 4) | (value >> 4));
}

static void pachinko_latch(uint8_t buttons) {
    if (pachinko.press && pachinko.position < 0x63) pachinko.position++;
    else if (pachinko.release && pachinko.position > 0) pachinko.position--;
    uint8_t position = reverse_byte(pachinko.position);
    pachinko.state = (uint16_t)(buttons | ((uint16_t)(uint8_t)~position << 8));
}

void pachinko_reset(void) {
    pachinko.position = 0;
    pachinko.state = 0;
    pachinko.strobe = false;
}

void pachinko_set_controls(bool press, bool release) {
    pachinko.press = press;
    pachinko.release = release;
}

void pachinko_write(uint8_t value, uint8_t buttons) {
    bool strobe = (value & 1u) != 0;
    if (pachinko.strobe && !strobe) pachinko_latch(buttons);
    pachinko.strobe = strobe;
}

uint8_t pachinko_read(uint8_t buttons) {
    if (pachinko.strobe) pachinko_latch(buttons);
    uint8_t output = (uint8_t)((pachinko.state & 1u) << 1);
    pachinko.state >>= 1;
    return output;
}

void exciting_boxing_reset(void) {
    exciting_boxing.selected = 0;
}

bool exciting_boxing_set_sensor(unsigned sensor, bool pressed) {
    if (sensor >= 8) return false;
    exciting_boxing.sensors[sensor] = pressed;
    return true;
}

void exciting_boxing_write(uint8_t value) {
    exciting_boxing.selected = (uint8_t)((value >> 1) & 1u);
}

uint8_t exciting_boxing_read(unsigned port) {
    if (port != 1) return 0;
    unsigned base = exciting_boxing.selected ? 4u : 0u;
    uint8_t value = 0;
    for (unsigned bit = 0; bit < 4; ++bit)
        if (!exciting_boxing.sensors[base + bit]) value |= (uint8_t)(1u << (bit + 1u));
    return value;
}
