/*
 * host_input.c - SDL controller assignment and bound host input
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "host_input.h"
#include "../joypad/joypad.h"
#include "../rom/mapper.h"
#include "peripheral_input.h"
#include <string.h>

static SDL_GameController *controllers[NES_INPUT_PLAYERS];
static unsigned held_keys[SDL_NUM_SCANCODES];
static uint8_t held_buttons[NES_INPUT_PLAYERS][SDL_CONTROLLER_BUTTON_MAX];
static uint8_t held_axes[NES_INPUT_PLAYERS][2];
/* Netplay can defer mutation of the emulated pad; track host state separately. */
static uint8_t published_buttons[NES_INPUT_PLAYERS];
/* Remember the action at press time so changing profiles cannot strand a hold. */
static unsigned held_shortcuts[NES_INPUT_PLAYERS][SDL_CONTROLLER_BUTTON_MAX];

static void publish_player(unsigned player) {
    uint8_t buttons = held_axes[player][0] | held_axes[player][1];
    for (unsigned b = 0; b < SDL_CONTROLLER_BUTTON_MAX; ++b) buttons |= held_buttons[player][b];
    for (unsigned sc = 1; sc < SDL_NUM_SCANCODES; ++sc) {
        unsigned target = held_keys[sc];
        if (target && (target - 1) / 8 == player) buttons |= (uint8_t)(1u << ((target - 1) % 8));
    }
    uint8_t changed = buttons ^ published_buttons[player];
    for (unsigned b = 0; b < 8; ++b)
        if (changed & (1u << b)) (void)joypad_set_player(player, (int)b, (buttons & (1u << b)) != 0);
    if (player == 0) {
        if (changed & (1u << BTN_A)) (void)cart_set_karaoke_input(CART_KARAOKE_A, (buttons & (1u << BTN_A)) != 0);
        if (changed & (1u << BTN_B)) (void)cart_set_karaoke_input(CART_KARAOKE_B, (buttons & (1u << BTN_B)) != 0);
    }
    published_buttons[player] = buttons;
}

static void refresh_shortcut_holds(FrontendExecutionRuntime *execution) {
    if (!execution) {
        return;
    }
    bool fast = false, rewind = false;
    for (unsigned p = 0; p < NES_INPUT_PLAYERS; ++p) {
        for (unsigned b = 0; b < SDL_CONTROLLER_BUTTON_MAX; ++b) {
            fast |= held_shortcuts[p][b] == FRONTEND_SHORTCUT_FAST_FORWARD_HOLD + 1;
            rewind |= held_shortcuts[p][b] == FRONTEND_SHORTCUT_REWIND + 1;
        }
    }
    frontend_execution_controller_hold(execution, FRONTEND_SHORTCUT_FAST_FORWARD_HOLD, fast);
    frontend_execution_controller_hold(execution, FRONTEND_SHORTCUT_REWIND, rewind);
}

static void release_extended_player(unsigned player) {
    if (player >= NES_INPUT_PLAYERS) return;
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
}

void frontend_host_input_release_all(void) {
    memset(held_shortcuts, 0, sizeof(held_shortcuts));
    memset(held_keys, 0, sizeof(held_keys));
    memset(held_buttons, 0, sizeof(held_buttons));
    memset(held_axes, 0, sizeof(held_axes));
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        uint8_t buttons = published_buttons[player] | joypad_player(player)->buttons;
        for (unsigned b = 0; b < 8; ++b)
            if (buttons & (1u << b)) (void)joypad_set_player(player, (int)b, false);
        published_buttons[player] = 0;
        release_extended_player(player);
    }
    (void)cart_set_karaoke_input(CART_KARAOKE_A, false);
    (void)cart_set_karaoke_input(CART_KARAOKE_B, false);
    frontend_peripheral_release_host_input();
    joypad_set_microphone(false);
    (void)cart_set_karaoke_input(CART_KARAOKE_MICROPHONE, false);
}

static bool device_guid_matches(int device, const char *expected) {
    if (!expected || !*expected) return false;
    char guid[FRONTEND_SETTINGS_GUID_TEXT];
    SDL_JoystickGetGUIDString(SDL_JoystickGetDeviceGUID(device), guid, sizeof(guid));
    return SDL_strcasecmp(guid, expected) == 0;
}

static void open_controller(int device, const FrontendSettings *settings) {
    if (!SDL_IsGameController(device)) return;
    SDL_JoystickID id = SDL_JoystickGetDeviceInstanceID(device);
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        if (controllers[player]
            && SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(controllers[player])) == id)
            return;
    }
    if (settings) {
        for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
            if (!controllers[player] && settings->device_guid[player][0]
                && device_guid_matches(device, settings->device_guid[player])) {
                controllers[player] = SDL_GameControllerOpen(device);
                return;
            }
        }
    }
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        if (!controllers[player] && (!settings || !settings->device_guid[player][0])) {
            controllers[player] = SDL_GameControllerOpen(device);
            return;
        }
    }
}

void frontend_host_input_open_controllers(const FrontendSettings *settings) {
    for (int device = 0; device < SDL_NumJoysticks(); ++device) open_controller(device, settings);
}

void frontend_host_input_event(const SDL_Event *event, const FrontendSettings *settings,
                               FrontendExecutionRuntime *execution) {
    if (!event) return;
    if (event->type == SDL_CONTROLLERDEVICEADDED) {
        open_controller(event->cdevice.which, settings);
        return;
    }
    const FrontendBindingProfile *profile = settings
        ? frontend_settings_active_profile_const(settings) : NULL;
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        if (!controllers[player]) continue;
        SDL_JoystickID id = SDL_JoystickInstanceID(
            SDL_GameControllerGetJoystick(controllers[player]));
        if (event->type == SDL_CONTROLLERDEVICEREMOVED && event->cdevice.which == id) {
            SDL_GameControllerClose(controllers[player]);
            controllers[player] = NULL;
            release_extended_player(player);
            memset(held_buttons[player], 0, sizeof(held_buttons[player]));
            memset(held_axes[player], 0, sizeof(held_axes[player]));
            publish_player(player);
            memset(held_shortcuts[player], 0, sizeof(held_shortcuts[player]));
            refresh_shortcut_holds(execution);
        } else if ((event->type == SDL_CONTROLLERBUTTONDOWN
                    || event->type == SDL_CONTROLLERBUTTONUP)
                   && event->cbutton.which == id) {
            bool down = event->type == SDL_CONTROLLERBUTTONDOWN;
            unsigned physical = event->cbutton.button;
            if (physical >= SDL_CONTROLLER_BUTTON_MAX) continue;
            if (!down && held_buttons[player][physical]) {
                held_buttons[player][physical] = 0;
                publish_player(player);
            }
            if (!down && physical < SDL_CONTROLLER_BUTTON_MAX && held_shortcuts[player][physical]) {
                held_shortcuts[player][physical] = 0;
                refresh_shortcut_holds(execution);
                continue;
            }
            FrontendShortcut shortcut;
            if (profile && execution
                && frontend_profile_shortcut_gamepad(
                    profile, (SDL_GameControllerButton)event->cbutton.button, &shortcut)) {
                if (shortcut == FRONTEND_SHORTCUT_FAST_FORWARD_HOLD || shortcut == FRONTEND_SHORTCUT_REWIND) {
                    if (physical < SDL_CONTROLLER_BUTTON_MAX) {
                        held_shortcuts[player][physical] = down ? (unsigned)shortcut + 1 : 0;
                        refresh_shortcut_holds(execution);
                    }
                } else {
                    (void)frontend_execution_handle_shortcut_action(execution, shortcut, down, false);
                }
                continue;
            }
            unsigned button;
            if (profile && frontend_profile_player_gamepad(
                    profile, player, (SDL_GameControllerButton)event->cbutton.button, &button)) {
                if (down) held_buttons[player][physical] = (uint8_t)(1u << button);
                publish_player(player);
            }
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
        } else if (event->type == SDL_CONTROLLERAXISMOTION && event->caxis.which == id) {
            const int deadzone = 16000;
            if (event->caxis.axis == SDL_CONTROLLER_AXIS_LEFTX || event->caxis.axis == SDL_CONTROLLER_AXIS_LEFTY) {
                bool horizontal = event->caxis.axis == SDL_CONTROLLER_AXIS_LEFTX;
                unsigned negative = horizontal ? BTN_LEFT : BTN_UP, positive = horizontal ? BTN_RIGHT : BTN_DOWN;
                held_axes[player][horizontal ? 0 : 1] = event->caxis.value < -deadzone ? (uint8_t)(1u << negative)
                    : event->caxis.value > deadzone ? (uint8_t)(1u << positive) : 0;
                publish_player(player);
            }
            if (player >= 2 || joypad_port_device(player) != NES_PORT_VIRTUAL_BOY) continue;
            if (event->caxis.axis == SDL_CONTROLLER_AXIS_RIGHTX) {
                joypad_set_virtual_boy_button(player, VB_BUTTON_LEFT1,
                                              event->caxis.value < -deadzone);
                joypad_set_virtual_boy_button(player, VB_BUTTON_RIGHT1,
                                              event->caxis.value > deadzone);
            } else if (event->caxis.axis == SDL_CONTROLLER_AXIS_RIGHTY) {
                joypad_set_virtual_boy_button(player, VB_BUTTON_UP1,
                                              event->caxis.value < -deadzone);
                joypad_set_virtual_boy_button(player, VB_BUTTON_DOWN1,
                                              event->caxis.value > deadzone);
            }
        }
    }
}

bool frontend_host_input_bound_player_key(const FrontendBindingProfile *profile,
                                          const SDL_KeyboardEvent *event) {
    if (!event || (unsigned)event->keysym.scancode >= SDL_NUM_SCANCODES ||
        (event->type != SDL_KEYDOWN && event->type != SDL_KEYUP)) return false;
    unsigned sc = (unsigned)event->keysym.scancode;
    if (held_keys[sc]) {
        unsigned player = (held_keys[sc] - 1) / 8;
        if (event->type == SDL_KEYUP) held_keys[sc] = 0;
        publish_player(player);
        return true;
    }
    unsigned player, button;
    if (!profile || event->type != SDL_KEYDOWN || event->repeat ||
        !frontend_profile_player_key(profile, event, &player, &button)) return false;
    held_keys[sc] = player * 8 + button + 1;
    publish_player(player);
    return true;
}

void frontend_host_input_shutdown(void) {
    frontend_host_input_release_all();
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        if (controllers[player]) SDL_GameControllerClose(controllers[player]);
        controllers[player] = NULL;
    }
}
