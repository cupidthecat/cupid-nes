/*
 * peripheral_input.c - Host keyboard and pointer routing for NES peripherals
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "peripheral_input.h"
#include "../joypad/joypad.h"
#include "../joypad/family_basic.h"
#include <stdio.h>

bool extended_port_key_event(const SDL_KeyboardEvent *event) {
    if (!event) return false;
    bool down = event->type == SDL_KEYDOWN;
    NesPortDevice device = joypad_port_device(0);
    NttKey keypad_key;
    bool keypad_event = true;
    switch (event->keysym.scancode) {
        case SDL_SCANCODE_KP_0: keypad_key = NTT_KEY_0; break;
        case SDL_SCANCODE_KP_1: keypad_key = NTT_KEY_1; break;
        case SDL_SCANCODE_KP_2: keypad_key = NTT_KEY_2; break;
        case SDL_SCANCODE_KP_3: keypad_key = NTT_KEY_3; break;
        case SDL_SCANCODE_KP_4: keypad_key = NTT_KEY_4; break;
        case SDL_SCANCODE_KP_5: keypad_key = NTT_KEY_5; break;
        case SDL_SCANCODE_KP_6: keypad_key = NTT_KEY_6; break;
        case SDL_SCANCODE_KP_7: keypad_key = NTT_KEY_7; break;
        case SDL_SCANCODE_KP_8: keypad_key = NTT_KEY_8; break;
        case SDL_SCANCODE_KP_9: keypad_key = NTT_KEY_9; break;
        case SDL_SCANCODE_KP_MULTIPLY: keypad_key = NTT_KEY_STAR; break;
        case SDL_SCANCODE_KP_DIVIDE: keypad_key = NTT_KEY_POUND; break;
        case SDL_SCANCODE_KP_PERIOD: keypad_key = NTT_KEY_PERIOD; break;
        case SDL_SCANCODE_C: keypad_key = NTT_KEY_C; break;
        case SDL_SCANCODE_E: keypad_key = NTT_KEY_END; break;
        default: keypad_event = false; break;
    }
    if (keypad_event) {
        bool handled = false;
        if (joypad_expansion_device() == NES_EXPANSION_FCNS_CONTROLLER)
            handled |= joypad_set_fcns_key((FcnsKey)keypad_key, down);
        for (unsigned port = 0; port < 2; ++port) {
            if (joypad_port_device(port) == NES_PORT_NTT_KEYPAD) {
                handled |= joypad_set_ntt_key(port, keypad_key, down);
            }
        }
        if (handled) return true;
    }

    if (device == NES_PORT_SNES_CONTROLLER || device == NES_PORT_NTT_KEYPAD) {
        SnesButton button;
        switch (event->keysym.scancode) {
            case SDL_SCANCODE_A: button = SNES_BUTTON_Y; break;
            case SDL_SCANCODE_S: button = SNES_BUTTON_X; break;
            case SDL_SCANCODE_Q: button = SNES_BUTTON_L; break;
            case SDL_SCANCODE_W: button = SNES_BUTTON_R; break;
            default: return false;
        }
        return joypad_set_snes_button(0, button, down);
    }
    if (device == NES_PORT_VIRTUAL_BOY) {
        VirtualBoyButton button;
        switch (event->keysym.scancode) {
            case SDL_SCANCODE_I: button = VB_BUTTON_UP1; break;
            case SDL_SCANCODE_K: button = VB_BUTTON_DOWN1; break;
            case SDL_SCANCODE_J: button = VB_BUTTON_LEFT1; break;
            case SDL_SCANCODE_L: button = VB_BUTTON_RIGHT1; break;
            case SDL_SCANCODE_Q: button = VB_BUTTON_L; break;
            case SDL_SCANCODE_E: button = VB_BUTTON_R; break;
            default: return false;
        }
        return joypad_set_virtual_boy_button(0, button, down);
    }
    return false;
}

bool mat_key_event(const SDL_KeyboardEvent *event) {
    static const SDL_Keycode keys[] = {
        SDLK_1, SDLK_2, SDLK_3, SDLK_4,
        SDLK_q, SDLK_w, SDLK_e, SDLK_r,
        SDLK_a, SDLK_s, SDLK_d, SDLK_f
    };
    bool handled = false;
    for (unsigned pad = 0; pad < 12; ++pad) {
        if (event->keysym.sym != keys[pad]) continue;
        for (unsigned slot = 0; slot < 3; ++slot) {
            bool active = slot == 2
                ? joypad_expansion_device() == NES_EXPANSION_FAMILY_TRAINER_A
                    || joypad_expansion_device() == NES_EXPANSION_FAMILY_TRAINER_B
                : joypad_port_device(slot) == NES_PORT_POWER_PAD_A
                    || joypad_port_device(slot) == NES_PORT_POWER_PAD_B;
            if (active) {
                joypad_set_mat_pad(slot, pad, event->type == SDL_KEYDOWN);
                handled = true;
            }
        }
        break;
    }
    return handled;
}

static bool tape_capture_pending;

bool finish_tape_capture(const char *path) {
    family_basic_tape_stop();
    if (!tape_capture_pending) return true;
    if (!path || !family_basic_tape_save_file(path)) {
        fprintf(stderr, "Could not save tape; the captured signal remains in memory\n");
        return false;
    }
    tape_capture_pending = false;
    return true;
}

bool family_basic_key_event(const SDL_KeyboardEvent *event,
                            const char *play_path, const char *record_path,
                            uint64_t cpu_cycles) {
    if (joypad_expansion_device() != NES_EXPANSION_FAMILY_BASIC) return false;
    static const SDL_Scancode keys[] = {
        SDL_SCANCODE_F8, SDL_SCANCODE_RETURN, SDL_SCANCODE_LEFTBRACKET, SDL_SCANCODE_RIGHTBRACKET,
        SDL_SCANCODE_RALT, SDL_SCANCODE_RSHIFT, SDL_SCANCODE_BACKSLASH, SDL_SCANCODE_F12,
        SDL_SCANCODE_F7, SDL_SCANCODE_GRAVE, SDL_SCANCODE_APOSTROPHE, SDL_SCANCODE_SEMICOLON,
        SDL_SCANCODE_F9, SDL_SCANCODE_SLASH, SDL_SCANCODE_MINUS, SDL_SCANCODE_EQUALS,
        SDL_SCANCODE_F6, SDL_SCANCODE_O, SDL_SCANCODE_L, SDL_SCANCODE_K,
        SDL_SCANCODE_PERIOD, SDL_SCANCODE_COMMA, SDL_SCANCODE_P, SDL_SCANCODE_0,
        SDL_SCANCODE_F5, SDL_SCANCODE_I, SDL_SCANCODE_U, SDL_SCANCODE_J,
        SDL_SCANCODE_M, SDL_SCANCODE_N, SDL_SCANCODE_9, SDL_SCANCODE_8,
        SDL_SCANCODE_F4, SDL_SCANCODE_Y, SDL_SCANCODE_G, SDL_SCANCODE_H,
        SDL_SCANCODE_B, SDL_SCANCODE_V, SDL_SCANCODE_7, SDL_SCANCODE_6,
        SDL_SCANCODE_F3, SDL_SCANCODE_T, SDL_SCANCODE_R, SDL_SCANCODE_D,
        SDL_SCANCODE_F, SDL_SCANCODE_C, SDL_SCANCODE_5, SDL_SCANCODE_4,
        SDL_SCANCODE_F2, SDL_SCANCODE_W, SDL_SCANCODE_S, SDL_SCANCODE_A,
        SDL_SCANCODE_X, SDL_SCANCODE_Z, SDL_SCANCODE_E, SDL_SCANCODE_3,
        SDL_SCANCODE_F1, SDL_SCANCODE_ESCAPE, SDL_SCANCODE_Q, SDL_SCANCODE_LCTRL,
        SDL_SCANCODE_LSHIFT, SDL_SCANCODE_LALT, SDL_SCANCODE_1, SDL_SCANCODE_2,
        SDL_SCANCODE_HOME, SDL_SCANCODE_UP, SDL_SCANCODE_RIGHT, SDL_SCANCODE_LEFT,
        SDL_SCANCODE_DOWN, SDL_SCANCODE_SPACE, SDL_SCANCODE_DELETE, SDL_SCANCODE_INSERT
    };
    _Static_assert(sizeof(keys) / sizeof(keys[0]) == FB_KEY_COUNT, "Complete keyboard matrix");
    bool down = event->type == SDL_KEYDOWN;
    if (event->keysym.scancode == SDL_SCANCODE_F10 && down && !event->repeat) {
        if (record_path) {
            if (family_basic_tape_mode() != FB_TAPE_RECORDING && finish_tape_capture(record_path)) {
                family_basic_tape_record(cpu_cycles);
                tape_capture_pending = true;
            }
        } else if (play_path) {
            family_basic_tape_play(cpu_cycles);
        }
    } else if (event->keysym.scancode == SDL_SCANCODE_F11 && down && !event->repeat) {
        finish_tape_capture(record_path);
    } else if (event->keysym.scancode == SDL_SCANCODE_BACKSPACE) {
        family_basic_set_host_key(FB_KEY_DELETE, down, false);
    } else {
        for (unsigned key = 0; key < FB_KEY_COUNT; ++key) {
            if (event->keysym.scancode == keys[key]) {
                family_basic_set_host_key((FamilyBasicKey)key, down, false);
                break;
            }
        }
    }
    return true;
}

static bool subor_modifier_event(SDL_Scancode scancode, bool down) {
    static bool sides[3][2];
    static const SuborKey keys[] = {SUBOR_KEY_CTRL, SUBOR_KEY_SHIFT, SUBOR_KEY_ALT};
    unsigned key;
    unsigned side;
    switch (scancode) {
        case SDL_SCANCODE_LCTRL: key = 0; side = 0; break;
        case SDL_SCANCODE_RCTRL: key = 0; side = 1; break;
        case SDL_SCANCODE_LSHIFT: key = 1; side = 0; break;
        case SDL_SCANCODE_RSHIFT: key = 1; side = 1; break;
        case SDL_SCANCODE_LALT: key = 2; side = 0; break;
        case SDL_SCANCODE_RALT: key = 2; side = 1; break;
        default: return false;
    }
    sides[key][side] = down;
    joypad_set_subor_key(keys[key], sides[key][0] || sides[key][1]);
    return true;
}

bool subor_key_event(const SDL_KeyboardEvent *event) {
    if (!event || joypad_expansion_device() != NES_EXPANSION_SUBOR_KEYBOARD) return false;
    static const SDL_Scancode keys[SUBOR_KEY_COUNT] = {
        [SUBOR_KEY_A] = SDL_SCANCODE_A, [SUBOR_KEY_B] = SDL_SCANCODE_B,
        [SUBOR_KEY_C] = SDL_SCANCODE_C, [SUBOR_KEY_D] = SDL_SCANCODE_D,
        [SUBOR_KEY_E] = SDL_SCANCODE_E, [SUBOR_KEY_F] = SDL_SCANCODE_F,
        [SUBOR_KEY_G] = SDL_SCANCODE_G, [SUBOR_KEY_H] = SDL_SCANCODE_H,
        [SUBOR_KEY_I] = SDL_SCANCODE_I, [SUBOR_KEY_J] = SDL_SCANCODE_J,
        [SUBOR_KEY_K] = SDL_SCANCODE_K, [SUBOR_KEY_L] = SDL_SCANCODE_L,
        [SUBOR_KEY_M] = SDL_SCANCODE_M, [SUBOR_KEY_N] = SDL_SCANCODE_N,
        [SUBOR_KEY_O] = SDL_SCANCODE_O, [SUBOR_KEY_P] = SDL_SCANCODE_P,
        [SUBOR_KEY_Q] = SDL_SCANCODE_Q, [SUBOR_KEY_R] = SDL_SCANCODE_R,
        [SUBOR_KEY_S] = SDL_SCANCODE_S, [SUBOR_KEY_T] = SDL_SCANCODE_T,
        [SUBOR_KEY_U] = SDL_SCANCODE_U, [SUBOR_KEY_V] = SDL_SCANCODE_V,
        [SUBOR_KEY_W] = SDL_SCANCODE_W, [SUBOR_KEY_X] = SDL_SCANCODE_X,
        [SUBOR_KEY_Y] = SDL_SCANCODE_Y, [SUBOR_KEY_Z] = SDL_SCANCODE_Z,
        [SUBOR_KEY_0] = SDL_SCANCODE_0, [SUBOR_KEY_1] = SDL_SCANCODE_1,
        [SUBOR_KEY_2] = SDL_SCANCODE_2, [SUBOR_KEY_3] = SDL_SCANCODE_3,
        [SUBOR_KEY_4] = SDL_SCANCODE_4, [SUBOR_KEY_5] = SDL_SCANCODE_5,
        [SUBOR_KEY_6] = SDL_SCANCODE_6, [SUBOR_KEY_7] = SDL_SCANCODE_7,
        [SUBOR_KEY_8] = SDL_SCANCODE_8, [SUBOR_KEY_9] = SDL_SCANCODE_9,
        [SUBOR_KEY_F1] = SDL_SCANCODE_F1, [SUBOR_KEY_F2] = SDL_SCANCODE_F2,
        [SUBOR_KEY_F3] = SDL_SCANCODE_F3, [SUBOR_KEY_F4] = SDL_SCANCODE_F4,
        [SUBOR_KEY_F5] = SDL_SCANCODE_F5, [SUBOR_KEY_F6] = SDL_SCANCODE_F6,
        [SUBOR_KEY_F7] = SDL_SCANCODE_F7, [SUBOR_KEY_F8] = SDL_SCANCODE_F8,
        [SUBOR_KEY_F9] = SDL_SCANCODE_F9, [SUBOR_KEY_F10] = SDL_SCANCODE_F10,
        [SUBOR_KEY_F11] = SDL_SCANCODE_F11, [SUBOR_KEY_F12] = SDL_SCANCODE_F12,
        [SUBOR_KEY_KP0] = SDL_SCANCODE_KP_0, [SUBOR_KEY_KP1] = SDL_SCANCODE_KP_1,
        [SUBOR_KEY_KP2] = SDL_SCANCODE_KP_2, [SUBOR_KEY_KP3] = SDL_SCANCODE_KP_3,
        [SUBOR_KEY_KP4] = SDL_SCANCODE_KP_4, [SUBOR_KEY_KP5] = SDL_SCANCODE_KP_5,
        [SUBOR_KEY_KP6] = SDL_SCANCODE_KP_6, [SUBOR_KEY_KP7] = SDL_SCANCODE_KP_7,
        [SUBOR_KEY_KP8] = SDL_SCANCODE_KP_8, [SUBOR_KEY_KP9] = SDL_SCANCODE_KP_9,
        [SUBOR_KEY_KP_ENTER] = SDL_SCANCODE_KP_ENTER, [SUBOR_KEY_KP_DOT] = SDL_SCANCODE_KP_PERIOD,
        [SUBOR_KEY_KP_PLUS] = SDL_SCANCODE_KP_PLUS,
        [SUBOR_KEY_KP_MULTIPLY] = SDL_SCANCODE_KP_MULTIPLY,
        [SUBOR_KEY_KP_DIVIDE] = SDL_SCANCODE_KP_DIVIDE,
        [SUBOR_KEY_KP_MINUS] = SDL_SCANCODE_KP_MINUS,
        [SUBOR_KEY_NUMLOCK] = SDL_SCANCODE_NUMLOCKCLEAR,
        [SUBOR_KEY_COMMA] = SDL_SCANCODE_COMMA, [SUBOR_KEY_DOT] = SDL_SCANCODE_PERIOD,
        [SUBOR_KEY_SEMICOLON] = SDL_SCANCODE_SEMICOLON,
        [SUBOR_KEY_APOSTROPHE] = SDL_SCANCODE_APOSTROPHE,
        [SUBOR_KEY_SLASH] = SDL_SCANCODE_SLASH, [SUBOR_KEY_BACKSLASH] = SDL_SCANCODE_BACKSLASH,
        [SUBOR_KEY_EQUAL] = SDL_SCANCODE_EQUALS, [SUBOR_KEY_MINUS] = SDL_SCANCODE_MINUS,
        [SUBOR_KEY_GRAVE] = SDL_SCANCODE_GRAVE,
        [SUBOR_KEY_LEFT_BRACKET] = SDL_SCANCODE_LEFTBRACKET,
        [SUBOR_KEY_RIGHT_BRACKET] = SDL_SCANCODE_RIGHTBRACKET,
        [SUBOR_KEY_CAPSLOCK] = SDL_SCANCODE_CAPSLOCK, [SUBOR_KEY_PAUSE] = SDL_SCANCODE_PAUSE,
        [SUBOR_KEY_SPACE] = SDL_SCANCODE_SPACE,
        [SUBOR_KEY_BACKSPACE] = SDL_SCANCODE_BACKSPACE, [SUBOR_KEY_TAB] = SDL_SCANCODE_TAB,
        [SUBOR_KEY_ESCAPE] = SDL_SCANCODE_ESCAPE, [SUBOR_KEY_ENTER] = SDL_SCANCODE_RETURN,
        [SUBOR_KEY_END] = SDL_SCANCODE_END, [SUBOR_KEY_HOME] = SDL_SCANCODE_HOME,
        [SUBOR_KEY_INSERT] = SDL_SCANCODE_INSERT, [SUBOR_KEY_DELETE] = SDL_SCANCODE_DELETE,
        [SUBOR_KEY_PAGEUP] = SDL_SCANCODE_PAGEUP, [SUBOR_KEY_PAGEDOWN] = SDL_SCANCODE_PAGEDOWN,
        [SUBOR_KEY_UP] = SDL_SCANCODE_UP, [SUBOR_KEY_DOWN] = SDL_SCANCODE_DOWN,
        [SUBOR_KEY_LEFT] = SDL_SCANCODE_LEFT, [SUBOR_KEY_RIGHT] = SDL_SCANCODE_RIGHT
    };
    bool down = event->type == SDL_KEYDOWN;
    if (subor_modifier_event(event->keysym.scancode, down)) return true;
    for (unsigned key = 0; key < SUBOR_KEY_COUNT; ++key) {
        if (keys[key] != SDL_SCANCODE_UNKNOWN && event->keysym.scancode == keys[key]) {
            joypad_set_subor_key((SuborKey)key, down);
            return true;
        }
    }
    return false;
}

bool party_tap_key_event(const SDL_KeyboardEvent *event) {
    if (!event || joypad_expansion_device() != NES_EXPANSION_PARTY_TAP) return false;
    static const SDL_Keycode keys[] = {SDLK_1, SDLK_2, SDLK_3, SDLK_4, SDLK_5, SDLK_6};
    for (unsigned button = 0; button < 6; ++button) {
        if (event->keysym.sym == keys[button]) {
            joypad_set_party_tap_button(button, event->type == SDL_KEYDOWN);
            return true;
        }
    }
    return false;
}

bool boxing_key_event(const SDL_KeyboardEvent *event) {
    if (!event || joypad_expansion_device() != NES_EXPANSION_EXCITING_BOXING) return false;
    static const SDL_Keycode keys[] = {
        SDLK_1, SDLK_2, SDLK_3, SDLK_4, SDLK_5, SDLK_6, SDLK_7, SDLK_8
    };
    for (unsigned sensor = 0; sensor < 8; ++sensor) {
        if (event->keysym.sym == keys[sensor]) {
            joypad_set_boxing_sensor(sensor, event->type == SDL_KEYDOWN);
            return true;
        }
    }
    return false;
}

bool jissen_key_event(const SDL_KeyboardEvent *event) {
    if (!event || joypad_expansion_device() != NES_EXPANSION_JISSEN_MAHJONG) return false;
    JissenKey key;
    if (event->keysym.sym >= SDLK_a && event->keysym.sym <= SDLK_n) {
        key = (JissenKey)(JISSEN_KEY_A + (event->keysym.sym - SDLK_a));
    } else {
        switch (event->keysym.sym) {
            case SDLK_RSHIFT: key = JISSEN_KEY_SELECT; break;
            case SDLK_RETURN: key = JISSEN_KEY_START; break;
            case SDLK_1: key = JISSEN_KEY_KAN; break;
            case SDLK_2: key = JISSEN_KEY_PON; break;
            case SDLK_3: key = JISSEN_KEY_CHII; break;
            case SDLK_4: key = JISSEN_KEY_RIICHI; break;
            case SDLK_5: key = JISSEN_KEY_RON; break;
            default: return false;
        }
    }
    joypad_set_jissen_key(key, event->type == SDL_KEYDOWN);
    return true;
}

void oeka_kids_pointer_event(int pointer_x, int pointer_y, bool pointer_on_screen,
                             uint32_t buttons) {
    if (joypad_expansion_device() != NES_EXPANSION_OEKA_KIDS_TABLET) return;
    bool click = (buttons & SDL_BUTTON_LMASK) != 0;
    bool touch = click || (pointer_on_screen && pointer_y >= 48);
    joypad_set_oeka_kids_tablet(pointer_x, pointer_y, touch, click);
}
