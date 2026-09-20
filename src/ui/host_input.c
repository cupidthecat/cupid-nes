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

static SDL_GameController *controllers[NES_INPUT_PLAYERS];

static void release_player(unsigned player) {
    if (player >= NES_INPUT_PLAYERS) return;
    joypad_player(player)->buttons = 0;
    if (player == 0) {
        (void)cart_set_karaoke_input(CART_KARAOKE_A, false);
        (void)cart_set_karaoke_input(CART_KARAOKE_B, false);
    }
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
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) release_player(player);
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
            release_player(player);
        } else if ((event->type == SDL_CONTROLLERBUTTONDOWN
                    || event->type == SDL_CONTROLLERBUTTONUP)
                   && event->cbutton.which == id) {
            bool down = event->type == SDL_CONTROLLERBUTTONDOWN;
            FrontendShortcut shortcut;
            if (profile && execution
                && frontend_profile_shortcut_gamepad(
                    profile, (SDL_GameControllerButton)event->cbutton.button, &shortcut)) {
                (void)frontend_execution_handle_shortcut_action(execution, shortcut, down, false);
                continue;
            }
            unsigned button;
            if (profile && frontend_profile_player_gamepad(
                    profile, player, (SDL_GameControllerButton)event->cbutton.button, &button)) {
                joypad_set_player(player, button, down);
                if (player == 0 && button == BTN_A)
                    (void)cart_set_karaoke_input(CART_KARAOKE_A, down);
                else if (player == 0 && button == BTN_B)
                    (void)cart_set_karaoke_input(CART_KARAOKE_B, down);
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
        } else if (event->type == SDL_CONTROLLERAXISMOTION && event->caxis.which == id
                   && player < 2 && joypad_port_device(player) == NES_PORT_VIRTUAL_BOY) {
            const int deadzone = 16000;
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
    unsigned player, button;
    if (!profile || !event
        || !frontend_profile_player_key(profile, event, &player, &button)) return false;
    bool down = event->type == SDL_KEYDOWN;
    (void)joypad_set_player(player, (int)button, down);
    if (player == 0 && button == BTN_A)
        (void)cart_set_karaoke_input(CART_KARAOKE_A, down);
    else if (player == 0 && button == BTN_B)
        (void)cart_set_karaoke_input(CART_KARAOKE_B, down);
    return true;
}

void frontend_host_input_shutdown(void) {
    frontend_host_input_release_all();
    for (unsigned player = 0; player < NES_INPUT_PLAYERS; ++player) {
        if (controllers[player]) SDL_GameControllerClose(controllers[player]);
        controllers[player] = NULL;
    }
}
