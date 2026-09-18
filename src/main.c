/*
 * main.c - Main entry point for Cupid NES Emulator
 *
 * Author: @frankischilling
 *
 * This file initializes the loaded cartridge, CPU, PPU, APU, SDL video and audio, input,
 * palette tools, frame pacing, and the main emulation loop. It also handles controller
 * input, reset events, palette file loading, and final emulator shutdown.
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

#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include "rom/rom.h"
#include "rom/fds.h"
#include "cpu/cpu.h"
#include "ppu/ppu.h"
#include "joypad/joypad.h"
#include "joypad/family_basic.h"
#include "../include/globals.h"
#include "apu/apu.h"
#include "apu/epsm.h"
#include <time.h>
#include "rom/mapper.h"
#include <math.h>
#include "ui/palette_tool.h"
#include "system/timing.h"
#include "system/hardware.h"
#include "system/vs_system.h"
#include "video/ntsc_composite.h"

#define AUDIO_SAMPLE_RATE 44100
#define AUDIO_BUFFER_SAMPLES 1024

// SDL presents the framebuffer that the PPU fills.
uint32_t framebuffer[SCREEN_WIDTH * SCREEN_HEIGHT];
static uint32_t composite_framebuffer[NTSC_COMPOSITE_WIDTH * NTSC_COMPOSITE_HEIGHT];

Joypad pad1 = {0}, pad2 = {0};

static SDL_GameController *controllers[NES_INPUT_PLAYERS];
static void open_controller(int device) {
    if (!SDL_IsGameController(device)) return;
    SDL_JoystickID id = SDL_JoystickGetDeviceInstanceID(device);
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        if (controllers[player] &&
            SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(controllers[player])) == id)
            return;
    }
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        if (!controllers[player]) {
            controllers[player] = SDL_GameControllerOpen(device);
            return;
        }
    }
}

static void controller_event(const SDL_Event *event) {
    if (event->type == SDL_CONTROLLERDEVICEADDED) {
        open_controller(event->cdevice.which);
        return;
    }
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        if (!controllers[player]) continue;
        SDL_JoystickID id = SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(controllers[player]));
        if (event->type == SDL_CONTROLLERDEVICEREMOVED && event->cdevice.which == id) {
            SDL_GameControllerClose(controllers[player]);
            controllers[player] = NULL;
            joypad_player(player)->buttons = 0;
            if (player < 2) {
                NesPortDevice device = joypad_port_device(player);
                if (device == NES_PORT_SNES_CONTROLLER || device == NES_PORT_NTT_KEYPAD) {
                    joypad_set_snes_button(player, SNES_BUTTON_X, false);
                    joypad_set_snes_button(player, SNES_BUTTON_Y, false);
                    joypad_set_snes_button(player, SNES_BUTTON_L, false);
                    joypad_set_snes_button(player, SNES_BUTTON_R, false);
                } else if (device == NES_PORT_VIRTUAL_BOY) {
                    joypad_set_virtual_boy_button(player, VB_BUTTON_DOWN1, false);
                    joypad_set_virtual_boy_button(player, VB_BUTTON_LEFT1, false);
                    joypad_set_virtual_boy_button(player, VB_BUTTON_RIGHT1, false);
                    joypad_set_virtual_boy_button(player, VB_BUTTON_UP1, false);
                    joypad_set_virtual_boy_button(player, VB_BUTTON_L, false);
                    joypad_set_virtual_boy_button(player, VB_BUTTON_R, false);
                }
            }
        } else if ((event->type == SDL_CONTROLLERBUTTONDOWN || event->type == SDL_CONTROLLERBUTTONUP)
                   && event->cbutton.which == id) {
            bool down = event->type == SDL_CONTROLLERBUTTONDOWN;
            int button = -1;
            switch (event->cbutton.button) {
                case SDL_CONTROLLER_BUTTON_A: button = BTN_A; break;
                case SDL_CONTROLLER_BUTTON_B: button = BTN_B; break;
                case SDL_CONTROLLER_BUTTON_BACK: button = BTN_SELECT; break;
                case SDL_CONTROLLER_BUTTON_START: button = BTN_START; break;
                case SDL_CONTROLLER_BUTTON_DPAD_UP: button = BTN_UP; break;
                case SDL_CONTROLLER_BUTTON_DPAD_DOWN: button = BTN_DOWN; break;
                case SDL_CONTROLLER_BUTTON_DPAD_LEFT: button = BTN_LEFT; break;
                case SDL_CONTROLLER_BUTTON_DPAD_RIGHT: button = BTN_RIGHT; break;
                default: break;
            }
            if (button >= 0) joypad_set_player(player, button, down);
            if (player < 2) {
                NesPortDevice device = joypad_port_device(player);
                if (device == NES_PORT_SNES_CONTROLLER || device == NES_PORT_NTT_KEYPAD) {
                    if (event->cbutton.button == SDL_CONTROLLER_BUTTON_X)
                        joypad_set_snes_button(player, SNES_BUTTON_X, down);
                    else if (event->cbutton.button == SDL_CONTROLLER_BUTTON_Y)
                        joypad_set_snes_button(player, SNES_BUTTON_Y, down);
                    else if (event->cbutton.button == SDL_CONTROLLER_BUTTON_LEFTSHOULDER)
                        joypad_set_snes_button(player, SNES_BUTTON_L, down);
                    else if (event->cbutton.button == SDL_CONTROLLER_BUTTON_RIGHTSHOULDER)
                        joypad_set_snes_button(player, SNES_BUTTON_R, down);
                } else if (device == NES_PORT_VIRTUAL_BOY) {
                    if (event->cbutton.button == SDL_CONTROLLER_BUTTON_LEFTSHOULDER)
                        joypad_set_virtual_boy_button(player, VB_BUTTON_L, down);
                    else if (event->cbutton.button == SDL_CONTROLLER_BUTTON_RIGHTSHOULDER)
                        joypad_set_virtual_boy_button(player, VB_BUTTON_R, down);
                }
            }
        } else if (event->type == SDL_CONTROLLERAXISMOTION && event->caxis.which == id
                   && player < 2 && joypad_port_device(player) == NES_PORT_VIRTUAL_BOY) {
            const int deadzone = 16000;
            if (event->caxis.axis == SDL_CONTROLLER_AXIS_RIGHTX) {
                joypad_set_virtual_boy_button(player, VB_BUTTON_LEFT1, event->caxis.value < -deadzone);
                joypad_set_virtual_boy_button(player, VB_BUTTON_RIGHT1, event->caxis.value > deadzone);
            } else if (event->caxis.axis == SDL_CONTROLLER_AXIS_RIGHTY) {
                joypad_set_virtual_boy_button(player, VB_BUTTON_UP1, event->caxis.value < -deadzone);
                joypad_set_virtual_boy_button(player, VB_BUTTON_DOWN1, event->caxis.value > deadzone);
            }
        }
    }
}

static bool extended_port_key_event(const SDL_KeyboardEvent *event) {
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

static bool mat_key_event(const SDL_KeyboardEvent *event) {
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

static bool finish_tape_capture(const char *path) {
    family_basic_tape_stop();
    if (!tape_capture_pending) return true;
    if (!path || !family_basic_tape_save_file(path)) {
        fprintf(stderr, "Could not save tape; the captured signal remains in memory\n");
        return false;
    }
    tape_capture_pending = false;
    return true;
}

static bool family_basic_key_event(const SDL_KeyboardEvent *event,
                                   const char *play_path, const char *record_path) {
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
                family_basic_tape_record(cpu_total_cycles);
                tape_capture_pending = true;
            }
        } else if (play_path) {
            family_basic_tape_play(cpu_total_cycles);
        }
    } else if (event->keysym.scancode == SDL_SCANCODE_F11 && down && !event->repeat) {
        finish_tape_capture(record_path);
    } else if (event->keysym.scancode == SDL_SCANCODE_BACKSPACE) {
        family_basic_set_key(FB_KEY_DELETE, down);
    } else {
        for (unsigned key = 0; key < FB_KEY_COUNT; ++key) {
            if (event->keysym.scancode == keys[key]) {
                family_basic_set_key((FamilyBasicKey)key, down);
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

static bool subor_key_event(const SDL_KeyboardEvent *event) {
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

static bool party_tap_key_event(const SDL_KeyboardEvent *event) {
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

static bool boxing_key_event(const SDL_KeyboardEvent *event) {
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

static bool jissen_key_event(const SDL_KeyboardEvent *event) {
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

static void oeka_kids_pointer_event(int pointer_x, int pointer_y, bool pointer_on_screen,
                                    uint32_t buttons) {
    if (joypad_expansion_device() != NES_EXPANSION_OEKA_KIDS_TABLET) return;
    bool click = (buttons & SDL_BUTTON_LMASK) != 0;
    bool touch = click || (pointer_on_screen && pointer_y >= 48);
    joypad_set_oeka_kids_tablet(pointer_x, pointer_y, touch, click);
}

int main(int argc, char *argv[]) {
    SDL_AudioSpec want;
    SDL_AudioSpec have;
    SDL_AudioDeviceID audio_dev = 0;

    const char *rom_path = NULL;
    const char *barcode = NULL;
    const char *barcode_battler = NULL;
    const char *tape_play_path = NULL;
    const char *tape_record_path = NULL;
    const char *fds_bios_path = NULL;
    size_t fds_frontend_side = 0;
    bool fds_side_set = false;
    bool fds_start_ejected = false;
    bool fds_start_write_protected = false;
    bool vs_dip_set = false;
    uint16_t vs_dips = 0;
    uint8_t input_overrides = 0;
    bool startup_phase_set = false;
    bool startup_seed_set = false;
    unsigned startup_cpu_offset = 0, startup_ppu_phase = 0;
    uint32_t startup_seed = 0;
    bool power_on_seed_set = false;
    uint32_t power_on_seed = 0;
    const char *epsm_adpcm_path = NULL;
    bool ntsc_composite_requested = false;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--console") == 0) {
            if (++i == argc || !nes_set_console_model_name(argv[i])) {
                fprintf(stderr, "Console must be nes-001, nes-101, famicom, or av-famicom\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--cpu-revision") == 0) {
            if (++i == argc) {
                fprintf(stderr, "CPU revision must be early-2a03 or late-2a03\n");
                return 1;
            }
            if (strcmp(argv[i], "early-2a03") == 0)
                apu_set_cpu_revision(APU_CPU_REVISION_EARLY_2A03);
            else if (strcmp(argv[i], "late-2a03") == 0)
                apu_set_cpu_revision(APU_CPU_REVISION_LATE_2A03);
            else {
                fprintf(stderr, "CPU revision must be early-2a03 or late-2a03\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--cpu-test-mode") == 0) {
            cpu_set_test_mode(true);
        } else if (strcmp(argv[i], "--epsm-adpcm") == 0) {
            if (++i == argc) {
                fprintf(stderr, "--epsm-adpcm requires an 8 KiB YMF288 ADPCM ROM file\n");
                return 1;
            }
            epsm_adpcm_path = argv[i];
        } else if (strcmp(argv[i], "--startup-phase") == 0) {
            if (++i == argc || startup_phase_set || startup_seed_set) {
                fprintf(stderr, "Choose one startup phase CPU:PPU or startup seed\n");
                return 1;
            }
            char *end;
            errno = 0;
            unsigned long cpu_offset = strtoul(argv[i], &end, 10);
            if (errno || argv[i][0] < '0' || argv[i][0] > '9' || *end != ':' || cpu_offset > 15) {
                fprintf(stderr, "Startup phase must be CPU:PPU in regional master clocks\n");
                return 1;
            }
            const char *ppu_text = end + 1;
            errno = 0;
            unsigned long ppu_phase = strtoul(ppu_text, &end, 10);
            if (errno || ppu_text[0] < '0' || ppu_text[0] > '9' || *end || ppu_phase > 4) {
                fprintf(stderr, "Startup phase must be CPU:PPU in regional master clocks\n");
                return 1;
            }
            startup_cpu_offset = (unsigned)cpu_offset;
            startup_ppu_phase = (unsigned)ppu_phase;
            startup_phase_set = true;
        } else if (strcmp(argv[i], "--startup-seed") == 0) {
            if (++i == argc || startup_phase_set || startup_seed_set) {
                fprintf(stderr, "Choose one startup phase CPU:PPU or startup seed\n");
                return 1;
            }
            char *end;
            errno = 0;
            unsigned long long seed = strtoull(argv[i], &end, 10);
            if (errno || argv[i][0] < '0' || argv[i][0] > '9' || *end || seed > UINT32_MAX) {
                fprintf(stderr, "Startup seed must be an integer from 0 to 4294967295\n");
                return 1;
            }
            startup_seed = (uint32_t)seed;
            startup_seed_set = true;
        } else if (strcmp(argv[i], "--ram-power-on") == 0) {
            if (++i == argc || !nes_set_ram_power_on_state_name(argv[i])) {
                fprintf(stderr, "RAM power-on state must be default, zero, ones, or random\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--power-on-seed") == 0) {
            if (++i == argc || power_on_seed_set) {
                fprintf(stderr, "Power-on seed must be an integer from 0 to 4294967295\n");
                return 1;
            }
            char *end;
            errno = 0;
            unsigned long long seed = strtoull(argv[i], &end, 10);
            if (errno || argv[i][0] < '0' || argv[i][0] > '9' || *end || seed > UINT32_MAX) {
                fprintf(stderr, "Power-on seed must be an integer from 0 to 4294967295\n");
                return 1;
            }
            power_on_seed = (uint32_t)seed;
            power_on_seed_set = true;
        } else if (strcmp(argv[i], "--random-vblank") == 0) {
            nes_set_randomize_vblank(true);
        } else if (strcmp(argv[i], "--ppu-revision") == 0) {
            if (++i == argc || !ppu_set_revision_name(argv[i])) {
                fprintf(stderr, "PPU revision must be 2c02-pre-e or 2c02e-plus\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--ppu-oam-row-corruption") == 0) {
            ppu_set_oam_row_corruption_worst_case(true);
        } else if (strcmp(argv[i], "--ppu-startup-restriction") == 0) {
            ppu_set_startup_write_restriction(true);
        } else if (strcmp(argv[i], "--ppu-oam-decay") == 0) {
            ppu_set_oam_decay(true);
        } else if (strcmp(argv[i], "--ppu-reset-suppression") == 0) {
            ppu_set_reset_suppression(true);
        } else if (strcmp(argv[i], "--video-filter") == 0) {
            if (++i == argc) {
                fprintf(stderr, "Video filter must be direct or ntsc-composite\n");
                return 1;
            }
            if (strcmp(argv[i], "direct") == 0) {
                ntsc_composite_requested = false;
            } else if (strcmp(argv[i], "ntsc-composite") == 0) {
                ntsc_composite_requested = true;
            } else {
                fprintf(stderr, "Video filter must be direct or ntsc-composite\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--mmc3-revision") == 0) {
            if (++i == argc || !cart_set_mmc3_revision_name(argv[i])) {
                fprintf(stderr, "MMC3 revision must be standard or a\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--cart-dip") == 0) {
            if (++i == argc) {
                fprintf(stderr, "Cartridge DIP value must be an integer from 0 to 255\n");
                return 1;
            }
            char *end = NULL;
            errno = 0;
            unsigned long value = strtoul(argv[i], &end, 0);
            if (errno || end == argv[i] || *end || argv[i][0] == '-' || value > 0xFFu
                || !cart_set_dip_switches((unsigned)value)) {
                fprintf(stderr, "Cartridge DIP value must be an integer from 0 to 255\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--adapter") == 0) {
            if (++i == argc || !joypad_set_adapter_name(argv[i])) {
                fprintf(stderr, "Adapter must be none, four-score, famicom-2, or famicom-4\n");
                return 1;
            }
            input_overrides |= NES_INPUT_OVERRIDE_ADAPTER;
        } else if (strcmp(argv[i], "--port1") == 0 || strcmp(argv[i], "--port2") == 0) {
            unsigned port = argv[i][6] == '2' ? 1 : 0;
            if (++i == argc || !joypad_set_port_device_name(port, argv[i])) {
                fprintf(stderr, "Port device must be pad, none, arkanoid, power-pad-a, power-pad-b, zapper, subor-mouse (port 2 only), snes-pad, snes-mouse, ntt-keypad, or virtual-boy\n");
                return 1;
            }
            input_overrides |= port ? NES_INPUT_OVERRIDE_PORT2 : NES_INPUT_OVERRIDE_PORT1;
        } else if (strcmp(argv[i], "--expansion") == 0) {
            if (++i == argc || !joypad_set_expansion_device_name(argv[i])) {
                fprintf(stderr, "Expansion device must be none, arkanoid, family-trainer-a, family-trainer-b, zapper, family-basic, turbo-file, battle-box, subor-keyboard, hori-track, konami-hyper-shot, bandai-hyper-shot, party-tap, pachinko, exciting-boxing, jissen-mahjong, barcode-battler, or oeka-kids-tablet\n");
                return 1;
            }
            input_overrides |= NES_INPUT_OVERRIDE_EXPANSION;
        } else if (strcmp(argv[i], "--zapper-radius") == 0) {
            if (++i == argc) {
                fprintf(stderr, "Zapper radius must be an integer from 0 to 255\n");
                return 1;
            }
            char *end;
            unsigned long radius = strtoul(argv[i], &end, 10);
            if (end == argv[i] || *end || radius > NES_ZAPPER_MAX_RADIUS
                || !joypad_set_zapper_radius((unsigned)radius)) {
                fprintf(stderr, "Zapper radius must be an integer from 0 to 255\n");
                return 1;
            }
        } else if (strcmp(argv[i], "--vs-dip") == 0) {
            if (++i == argc) {
                fprintf(stderr, "VS DIP value must be an integer from 0 to 65535\n");
                return 1;
            }
            char *end;
            unsigned long value = strtoul(argv[i], &end, 0);
            if (end == argv[i] || *end || value > 0xFFFFu) {
                fprintf(stderr, "VS DIP value must be an integer from 0 to 65535\n");
                return 1;
            }
            vs_dips = (uint16_t)value;
            vs_dip_set = true;
        } else if (strcmp(argv[i], "--barcode") == 0) {
            if (++i == argc) {
                fprintf(stderr, "Barcode requires 8 or 13 decimal digits\n");
                return 1;
            }
            barcode = argv[i];
        } else if (strcmp(argv[i], "--barcode-battler") == 0) {
            if (++i == argc) {
                fprintf(stderr, "Barcode Battler scan requires 8 or 13 decimal digits\n");
                return 1;
            }
            barcode_battler = argv[i];
        } else if (strcmp(argv[i], "--tape-play") == 0 || strcmp(argv[i], "--tape-record") == 0) {
            bool record = strcmp(argv[i], "--tape-record") == 0;
            if (++i == argc || tape_play_path || tape_record_path) {
                fprintf(stderr, "Choose one tape file with --tape-play or --tape-record\n");
                return 1;
            }
            if (record) tape_record_path = argv[i];
            else tape_play_path = argv[i];
        } else if (strcmp(argv[i], "--fds-bios") == 0) {
            if (++i == argc) {
                fprintf(stderr, "--fds-bios requires an 8KB BIOS file\n");
                return 1;
            }
            fds_bios_path = argv[i];
        } else if (strcmp(argv[i], "--fds-side") == 0) {
            if (++i == argc) {
                fprintf(stderr, "--fds-side requires a side number starting at 1\n");
                return 1;
            }
            char *end = NULL;
            unsigned long side = strtoul(argv[i], &end, 10);
            if (!side || *end) {
                fprintf(stderr, "FDS side must be a positive number\n");
                return 1;
            }
            fds_frontend_side = (size_t)(side - 1);
            fds_side_set = true;
        } else if (strcmp(argv[i], "--fds-eject") == 0) {
            fds_start_ejected = true;
        } else if (strcmp(argv[i], "--fds-write-protect") == 0) {
            fds_start_write_protected = true;
        } else if (argv[i][0] == '-' || rom_path) {
            fprintf(stderr, "Unexpected argument: %s\n", argv[i]);
            return 1;
        } else {
            rom_path = argv[i];
        }
    }
    if (!rom_path) {
        printf("Usage: %s [--console MODEL] [--cpu-revision REVISION] "
               "[--cpu-test-mode] "
               "[--epsm-adpcm FILE] "
               "[--startup-phase CPU:PPU | --startup-seed SEED] "
               "[--ram-power-on STATE] [--power-on-seed SEED] [--random-vblank] "
               "[--ppu-revision REVISION] [--ppu-oam-row-corruption] "
               "[--ppu-startup-restriction] [--ppu-oam-decay] [--ppu-reset-suppression] "
               "[--video-filter direct|ntsc-composite] "
               "[--mmc3-revision REVISION] [--cart-dip VALUE] "
               "[--adapter TYPE] [--port1 DEVICE] [--port2 DEVICE] "
               "[--expansion DEVICE] [--barcode DIGITS] [--barcode-battler DIGITS] "
               "[--zapper-radius PIXELS] [--vs-dip VALUE] [--tape-play FILE | --tape-record FILE] "
               "[--fds-bios BIOS] [--fds-side N] "
               "[--fds-eject] [--fds-write-protect] <rom-file>\n", argv[0]);
        return 1;
    }
    if (!fds_bios_path && (fds_side_set || fds_start_ejected || fds_start_write_protected)) {
        fprintf(stderr, "FDS media options require --fds-bios\n");
        return 1;
    }
    if (!joypad_configuration_valid()) {
        fprintf(stderr, "An adapter and another device cannot share the same connector\n");
        return 1;
    }
    if ((tape_play_path || tape_record_path) && joypad_expansion_device() != NES_EXPANSION_FAMILY_BASIC) {
        fprintf(stderr, "Tape input requires --expansion family-basic\n");
        return 1;
    }
    joypad_set_configuration_overrides(input_overrides);
    
    printf("Console: %s\n", nes_console_model_name());
    printf("CPU revision: %s\n", apu_get_cpu_revision() == APU_CPU_REVISION_EARLY_2A03
           ? "early-2a03" : "late-2a03");
    printf("RAM power-on state: %s\n", nes_ram_power_on_state_name());
    printf("Random power-on VBL flag: %s\n", nes_randomize_vblank_enabled() ? "enabled" : "disabled");
    if (power_on_seed_set) printf("Power-on seed: %llu\n", (unsigned long long)power_on_seed);
    printf("PPU revision: %s\n", ppu_revision_name());
    printf("CPU test-register reads: %s\n", cpu_test_mode_enabled() ? "enabled" : "disabled");
    printf("PPU OAM row corruption: %s\n",
           ppu_oam_row_corruption_worst_case() ? "worst-case" : "compatibility");
    printf("PPU startup write restriction: %s\n",
           ppu_startup_write_restriction_enabled() ? "enabled" : "compatibility");
    printf("PPU OAM decay: %s\n", ppu_oam_decay_enabled() ? "enabled" : "compatibility");
    printf("PPU soft-reset suppression: %s\n",
           ppu_reset_suppression_enabled() ? "enabled" : "disabled");
    printf("MMC3 revision: %s\n", cart_mmc3_revision_name());
    printf("Input adapter: %s\n", joypad_adapter_name());
    printf("Loading ROM: %s\n", rom_path);
    if (epsm_adpcm_path && !epsm_load_adpcm_file(epsm_adpcm_path)) {
        fprintf(stderr, "Could not load the 8 KiB YMF288 ADPCM ROM: %s\n", epsm_adpcm_path);
        return 1;
    }
    if (power_on_seed_set) nes_seed_power_on_random(power_on_seed);
    int load_result = fds_bios_path
        ? load_fds(rom_path, fds_bios_path, fds_start_write_protected)
        : load_rom(rom_path);
    if(load_result != 0) {
        fprintf(stderr, "Failed to load ROM\n");
        return 1;
    }
    if (epsm_enabled()) {
        printf("EPSM: 8 MHz YMF288, stereo output\n");
        if (!epsm_has_adpcm_rom())
            fprintf(stderr, "EPSM percussion uses zero-filled data without --epsm-adpcm FILE\n");
    }
    bool ntsc_composite_active = ntsc_composite_requested
        && ntsc_composite_supported(nes_timing()->region, vs_enabled());
    if (ntsc_composite_requested && !ntsc_composite_active)
        printf("Video filter: direct (NTSC composite is unavailable for this hardware)\n");
    else
        printf("Video filter: %s\n", ntsc_composite_active ? "ntsc-composite" : "direct");
    if (startup_phase_set && !cpu_set_startup_alignment(startup_cpu_offset, startup_ppu_phase)) {
        fprintf(stderr, "Startup phase must be CPU 0..%u and PPU 0..%u for this image\n",
                (unsigned)nes_timing()->cpu_divider - 1, (unsigned)nes_timing()->ppu_divider - 1);
        unload_rom();
        return 1;
    }
    if (startup_seed_set) cpu_seed_startup_alignment(startup_seed);
    if (!joypad_persistent_configure(rom_path)) {
        fprintf(stderr, "Failed to load expansion-device storage\n");
        unload_rom();
        return 1;
    }
    if (vs_dip_set && !vs_set_dip_switches(vs_dips)) {
        fprintf(stderr, "--vs-dip requires a VS System image\n");
        unload_rom();
        return 1;
    }
    if (barcode && !cart_set_barcode(barcode)) {
        fprintf(stderr, "Barcode input requires a Datach cartridge and 8 or 13 decimal digits\n");
        unload_rom();
        return 1;
    }
    if (barcode) printf("Press F8 to scan the configured barcode\n");
    if (rom_is_fds()) {
        if (fds_side_set && !fds_insert_disk(fds_frontend_side)) {
            fprintf(stderr, "FDS side is outside the loaded disk image\n");
            unload_rom();
            return 1;
        }
        if (fds_start_ejected) fds_eject_disk();
    }
    cpu_total_cycles = 0;
    if (barcode_battler && !joypad_scan_barcode_battler(barcode_battler)) {
        fprintf(stderr, "Barcode Battler input requires --expansion barcode-battler and 8 or 13 decimal digits\n");
        unload_rom();
        return 1;
    }
    if (barcode_battler) printf("Press F8 to scan the configured Barcode Battler code\n");
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    // Print ROM metadata at startup so mapper selection can be checked from the log.
    if (!rom_is_fds()) {
        printf("=== ROM Header Info ===\n");
        printf("Signature: %c%c%c 0x%02X\n",
               ines_header.signature[0],
               ines_header.signature[1],
               ines_header.signature[2],
               ines_header.signature[3]);
        printf("PRG-ROM Chunks: %d\n", ines_header.prg_rom_chunks);
        printf("CHR-ROM Chunks: %d\n", ines_header.chr_rom_chunks);
        printf("Flags6: 0x%02X\n", ines_header.flags6);
        printf("Flags7: 0x%02X\n", ines_header.flags7);
        printf("Mirroring: %s\n", (mirroring_mode == 0 ? "Horizontal" : "Vertical"));
        printf("=======================\n");
    }
    
    if (!rom_is_fds() && (ines_header.prg_rom_chunks > 1 || (ines_header.flags6 & 0xF0))) {
        printf("WARNING: This ROM likely uses a mapper (mapper number: %d).\n",
            (ines_header.flags7 & 0xF0) | ((ines_header.flags6 & 0xF0) >> 4));
    }
    
    if (!rom_is_fds())
        printf("Mapper detected: %d\n", ((ines_header.flags7 & 0xF0) | ((ines_header.flags6 & 0xF0) >> 4)));


    printf("Resetting CPU...\n");
    if (!cpu_power_on(&cpu)) {
        fprintf(stderr, "Invalid CPU startup alignment\n");
        unload_rom();
        return 1;
    }
    CpuStartupAlignment alignment = cpu_get_startup_alignment();
    printf("Startup alignment: CPU %u, PPU %u%s\n", (unsigned)alignment.cpu_offset,
           (unsigned)alignment.ppu_phase, startup_seed_set ? " (seeded)" : "");
    if (startup_seed_set) printf("Startup seed: %llu\n", (unsigned long long)startup_seed);
    vs_power_on_secondary();
    if (vs_enabled()) {
        printf("VS System: %s, PPU model %u, DIP $%04X\n",
               vs_dual_system() ? "dual" : "single", (unsigned)vs_ppu_model(),
               (unsigned)vs_dip_switches());
        printf("VS controls: 5-8 coin slots, F1/F2 service buttons\n");
    }
    if (tape_play_path && !family_basic_tape_load_file(tape_play_path)) {
        fprintf(stderr, "Could not load tape: %s\n", tape_play_path);
        unload_rom();
        return 1;
    }
    if (tape_play_path || tape_record_path)
        printf("Family BASIC tape: F10 starts the tape; F11 stops and saves a recording\n");
    printf("CPU state after reset:\n");
    printf("  PC: 0x%04X\n", cpu.pc);
    printf("  SP: 0x%02X\n", cpu.sp);
    printf("  A: 0x%02X\n", cpu.a);
    printf("  X: 0x%02X\n", cpu.x);
    printf("  Y: 0x%02X\n", cpu.y);
    printf("  Status: 0x%02X\n", cpu.status);

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_GAMECONTROLLER) != 0) {
        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
        return 1;
    }
    for (int device = 0; device < SDL_NumJoysticks(); ++device) open_controller(device);

    // Configure audio before starting the main loop.
    memset(&want, 0, sizeof want);
    memset(&have, 0, sizeof have);
    want.freq = AUDIO_SAMPLE_RATE;
    want.format = AUDIO_F32;
    want.channels = epsm_enabled() ? 2 : 1;
    want.samples = AUDIO_BUFFER_SAMPLES;
    want.callback = epsm_enabled() ? apu_sdl_stereo_callback : vs_audio_callback;

    audio_dev = SDL_OpenAudioDevice(NULL, 0, &want, &have, 0);
    if (!audio_dev) {
        fprintf(stderr, "Warning: audio disabled (%s)\n", SDL_GetError());
    } else {
        printf("=== Audio Info ===\n");
        printf("Requested: %d Hz, Got: %d Hz\n", want.freq, have.freq);
        printf("Requested: %d samples buffer, Got: %d samples\n", want.samples, have.samples);
        printf("Cycles per sample: %.6f\n", nes_timing()->cpu_hz / have.freq);
        printf("==================\n");
        vs_audio_init(have.freq);
        SDL_PauseAudioDevice(audio_dev, 0);
    }

    int video_width = ntsc_composite_active ? NTSC_COMPOSITE_WIDTH : (int)vs_video_width();
    int video_height = ntsc_composite_active ? NTSC_COMPOSITE_HEIGHT : SCREEN_HEIGHT;
    int window_width = (int)vs_video_width() * 2;
    int window_height = SCREEN_HEIGHT * 2;
    SDL_Window *window = SDL_CreateWindow("Cupid NES Emulator",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, window_width, window_height, SDL_WINDOW_SHOWN);
    if(!window) {
        fprintf(stderr, "SDL_CreateWindow Error: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if(!renderer) {
        fprintf(stderr, "SDL_CreateRenderer Error: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Texture *texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, video_width, video_height);
    if(!texture) {
        fprintf(stderr, "SDL_CreateTexture Error: %s\n", SDL_GetError());
        return 1;
    }

    bool running = true;
    SDL_Event e;

    palette_tool_init();
    const double performance_frequency = (double)SDL_GetPerformanceFrequency();
    double frame_deadline = (double)SDL_GetPerformanceCounter();
    
    while (running) {
        Uint32 frameStart = SDL_GetTicks();
        uint64_t frame_start_cycles = cpu_total_cycles;
    
        while (SDL_PollEvent(&e)) {
            controller_event(&e);
            bool main_mouse_event = e.type == SDL_MOUSEMOTION
                ? e.motion.windowID == SDL_GetWindowID(window)
                : (e.type == SDL_MOUSEBUTTONDOWN || e.type == SDL_MOUSEBUTTONUP)
                    && e.button.windowID == SDL_GetWindowID(window);
            if (main_mouse_event) {
                int mouse_x, mouse_y, window_width, window_height;
                uint32_t buttons = SDL_GetMouseState(&mouse_x, &mouse_y);
                SDL_GetWindowSize(window, &window_width, &window_height);
                int position = window_width > 0 ? 0x54 + 160 * mouse_x / window_width : 0x54;
                bool pointer_on_screen = mouse_x >= 0 && mouse_y >= 0 && mouse_x < window_width
                    && mouse_y < window_height;
                bool zapper_on_screen = pointer_on_screen && !(buttons & SDL_BUTTON_RMASK);
                int pointer_x = pointer_on_screen ? 256 * mouse_x / window_width : -1;
                int pointer_y = pointer_on_screen ? 240 * mouse_y / window_height : -1;
                int aim_x = zapper_on_screen ? pointer_x : -1;
                int aim_y = zapper_on_screen ? pointer_y : -1;
                bool trigger = (buttons & (SDL_BUTTON_LMASK | SDL_BUTTON_RMASK)) != 0;
                for (unsigned slot = 0; slot < 3; ++slot) {
                    joypad_set_paddle(slot, position, (buttons & SDL_BUTTON_LMASK) != 0);
                    joypad_set_zapper(slot, aim_x, aim_y, trigger);
                }
                if (joypad_port_device(1) == NES_PORT_SUBOR_MOUSE) {
                    if (e.type == SDL_MOUSEMOTION)
                        joypad_add_subor_mouse_motion(e.motion.xrel, e.motion.yrel);
                    joypad_set_subor_mouse_buttons((buttons & SDL_BUTTON_LMASK) != 0,
                                                   (buttons & SDL_BUTTON_RMASK) != 0);
                }
                for (unsigned port = 0; port < 2; ++port) {
                    if (joypad_port_device(port) != NES_PORT_SNES_MOUSE) continue;
                    if (e.type == SDL_MOUSEMOTION)
                        joypad_add_snes_mouse_motion(port, e.motion.xrel, e.motion.yrel);
                    joypad_set_snes_mouse_buttons(port, (buttons & SDL_BUTTON_LMASK) != 0,
                                                  (buttons & SDL_BUTTON_RMASK) != 0);
                }
                if (e.type == SDL_MOUSEMOTION
                    && joypad_expansion_device() == NES_EXPANSION_HORI_TRACK)
                    joypad_add_hori_track_motion(e.motion.xrel, e.motion.yrel);
                if (joypad_expansion_device() == NES_EXPANSION_PACHINKO)
                    joypad_set_pachinko_controls((buttons & SDL_BUTTON_LMASK) != 0,
                                                 (buttons & SDL_BUTTON_RMASK) != 0);
                oeka_kids_pointer_event(pointer_x, pointer_y, pointer_on_screen, buttons);
            }
            if (e.type == SDL_QUIT) {
                if (rom_is_fds() && !fds_flush()) {
                    fprintf(stderr, "Failed to save modified FDS media; keeping the emulator open\n");
                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "FDS Save Error",
                        "The modified disk image could not be saved. The emulator will remain open so the media changes are not discarded.", window);
                } else if (!joypad_persistent_flush()) {
                    fprintf(stderr, "Failed to save expansion-device storage; keeping the emulator open\n");
                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Peripheral Save Error",
                        "Expansion-device storage could not be saved. The emulator will remain open so the changes are not discarded.", window);
                } else {
                    running = false;
                }
            }
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)
                && family_basic_key_event(&e.key, tape_play_path, tape_record_path)) continue;
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)
                && subor_key_event(&e.key)) continue;
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)
                && party_tap_key_event(&e.key)) continue;
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)
                && boxing_key_event(&e.key)) continue;
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)
                && jissen_key_event(&e.key)) continue;
            if ((e.type == SDL_KEYDOWN || e.type == SDL_KEYUP)
                && e.key.windowID == SDL_GetWindowID(window)
                && extended_port_key_event(&e.key)) continue;
            palette_tool_handle_event(&e, renderer);
            
            if (e.type == SDL_KEYDOWN || e.type == SDL_KEYUP) {
                if (mat_key_event(&e.key)) continue;
                int down = (e.type == SDL_KEYDOWN);
    
                switch (e.key.keysym.sym) {
                    case SDLK_F10:
                        if (down && rom_is_fds()) fds_set_write_protected(!fds_write_protected());
                        break;
                    case SDLK_F9:
                        if (down && rom_is_fds() && fds_side_count()) {
                            fds_frontend_side = (fds_frontend_side + 1) % fds_side_count();
                            (void)fds_insert_disk(fds_frontend_side);
                        }
                        break;
                    case SDLK_F8:
                        if (down && !e.key.repeat && barcode) cart_set_barcode(barcode);
                        if (down && !e.key.repeat && barcode_battler)
                            joypad_scan_barcode_battler(barcode_battler);
                        if (down && !e.key.repeat && rom_is_fds()) {
                            if (fds_disk_inserted()) fds_eject_disk();
                            else (void)fds_insert_disk(fds_frontend_side);
                        }
                        break;
                    case SDLK_F7:
                        if (down) { palette_tool_toggle_overlay(); }
                        break;
                    case SDLK_F6:
                        if (down) { ppu_palette_reset_default(); palette_tool_flash(true); }
                        break;
                    case SDLK_r:
                        if (down) {
                            if (audio_dev) SDL_LockAudioDevice(audio_dev);
                            ppu_soft_reset(&ppu);
                            apu_soft_reset(&apu);
                            cpu_soft_reset(&cpu);
                            vs_soft_reset();
                            if (audio_dev) SDL_UnlockAudioDevice(audio_dev);
                        }
                        break;
                    case SDLK_5: if (vs_enabled()) vs_set_coin(0, down != 0); break;
                    case SDLK_6: if (vs_enabled()) vs_set_coin(1, down != 0); break;
                    case SDLK_7: if (vs_dual_system()) vs_set_coin(2, down != 0); break;
                    case SDLK_8: if (vs_dual_system()) vs_set_coin(3, down != 0); break;
                    case SDLK_F1: if (vs_enabled()) vs_set_service(0, down != 0); break;
                    case SDLK_F2: if (vs_dual_system()) vs_set_service(1, down != 0); break;
                    case SDLK_z:        joypad_set_player(0, BTN_A,      down); break;
                    case SDLK_x:        joypad_set_player(0, BTN_B,      down); break;
                    case SDLK_RSHIFT:   joypad_set_player(0, BTN_SELECT, down); break;
                    case SDLK_RETURN:   joypad_set_player(0, BTN_START,  down); break;
                    case SDLK_UP:       joypad_set_player(0, BTN_UP,     down); break;
                    case SDLK_DOWN:     joypad_set_player(0, BTN_DOWN,   down); break;
                    case SDLK_LEFT:     joypad_set_player(0, BTN_LEFT,   down); break;
                    case SDLK_RIGHT:    joypad_set_player(0, BTN_RIGHT,  down); break;
                    case SDLK_m:        joypad_set_microphone(down != 0); break;
                    default: break;
                }

                // Ctrl+V accepts the palette text formats handled by the parser.
                if (down && (e.key.keysym.sym == SDLK_v)) {
                    const SDL_Keymod mods = SDL_GetModState();
                    if ((mods & KMOD_CTRL) != 0) {
                        if (SDL_HasClipboardText()) {
                            char *txt = SDL_GetClipboardText();
                            if (txt) {
                                int rc = ppu_palette_load_hex_string(txt);
                                palette_tool_flash(rc == 0);
                                SDL_free(txt);
                                if (rc != 0) {
                                    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_WARNING, "Palette Paste Error",
                                        "Clipboard text did not contain a valid palette.\n\nAccepts: 64 x RRGGBB tokens, or raw 192/1536 hex bytes.", NULL);
                                }
                            }
                        }
                    }
                }
            }

            // Dropped .pal files use the same validation as explicit palette loads.
            if (e.type == SDL_DROPFILE) {
                char *dropped_f = e.drop.file;
                if (dropped_f) {
                    int rc = ppu_palette_load_pal_file(dropped_f);
                    palette_tool_flash(rc == 0);
                    if (rc != 0) {
                        SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Palette Load Error",
                            "Failed to load .pal file. Expected 192 or 1536 bytes.", NULL);
                    }
                    SDL_free(dropped_f);
                }
            }
        }

        // A successful quit flush must be the last chance for emulation to mutate
        // writable disk media. Do not run another frame after accepting SDL_QUIT.
        if (!running) break;
    
        // Run CPU steps until the PPU completes the current frame.
        vs_start_frame();
        while (!ppu.frame_complete) {
            vs_cpu_step();
        }

        // Presentation filters consume captured PPU signal data after emulation has
        // finished the frame, so they cannot change beam timing or light-sensor input.
        const uint32_t *presented_frame = vs_video_framebuffer();
        if (ntsc_composite_active) {
            ntsc_composite_filter_frame(ppu.pixel_signal, ppu.completed_video_phase,
                                        composite_framebuffer);
            presented_frame = composite_framebuffer;
        }
        SDL_UpdateTexture(texture, NULL, presented_frame, video_width * sizeof(uint32_t));
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);
        int ww = 0, hh = 0; SDL_GetRendererOutputSize(renderer, &ww, &hh);
        if (palette_tool_is_visible()) { palette_tool_draw(renderer, ww, hh); }
        SDL_RenderPresent(renderer);
    
        Uint32 frameTime = SDL_GetTicks() - frameStart;
        palette_tool_tick(frameTime);
        frame_deadline += (double)(cpu_total_cycles - frame_start_cycles)
                        * performance_frequency / nes_timing()->cpu_hz;
        double current_ticks = (double)SDL_GetPerformanceCounter();
        if (frame_deadline > current_ticks) {
            // Carry fractional milliseconds into the next deadline instead of
            // running every frame early after truncating SDL's delay argument.
            SDL_Delay((Uint32)((frame_deadline - current_ticks) * 1000.0 / performance_frequency));
        } else if (current_ticks - frame_deadline > performance_frequency * 0.25) {
            frame_deadline = current_ticks;
        }
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    if (audio_dev) SDL_CloseAudioDevice(audio_dev);
    apu_audio_shutdown_state(&apu);
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player)
        if (controllers[player]) SDL_GameControllerClose(controllers[player]);
    bool tape_saved = finish_tape_capture(tape_record_path);
    bool tape_failed = family_basic_tape_failed();
    bool peripheral_saved = joypad_persistent_shutdown();
    if (tape_failed) fprintf(stderr, "Tape recording stopped because the capture buffer could not grow\n");
    family_basic_shutdown();
    if (!unload_rom()) {
        fprintf(stderr, "Failed to unload modified FDS media\n");
        SDL_Quit();
        return 1;
    }
    SDL_Quit();
    return tape_saved && !tape_failed && peripheral_saved ? 0 : 1;
}
