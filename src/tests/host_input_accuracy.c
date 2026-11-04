/*
 * host_input_accuracy.c - Mixed host input, remapping and analog controller regressions
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/host_input.h"
#include "../joypad/joypad.h"
#include "../joypad/special_peripherals.h"
#include "../ui/peripheral_input.h"
#include "../replay/input_event.h"
#include <stdio.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "Host input failure %d: %s\n", __LINE__, #x); ++failures; } } while (0)

static bool defer_input(const NesInputEvent *event, void *context) {
    unsigned *transitions = context;
    if (event->type == NES_INPUT_EVENT_PLAYER_BUTTON && event->a == 0 && event->b == BTN_A)
        *transitions = (*transitions << 1) | (unsigned)event->c;
    return false;
}

static int peripheral_releases(void) {
    int failures = 0;
    NesExpansionDevice previous = joypad_expansion_device();
    const struct {
        NesExpansionDevice device;
        SDL_Scancode key;
        SDL_Keycode symbol;
        bool (*event)(const SDL_KeyboardEvent *);
    } cases[] = {{NES_EXPANSION_SUBOR_KEYBOARD, SDL_SCANCODE_RSHIFT, SDLK_RSHIFT, subor_key_event},
                 {NES_EXPANSION_PARTY_TAP, SDL_SCANCODE_1, SDLK_1, party_tap_key_event},
                 {NES_EXPANSION_EXCITING_BOXING, SDL_SCANCODE_2, SDLK_2, boxing_key_event},
                 {NES_EXPANSION_JISSEN_MAHJONG, SDL_SCANCODE_A, SDLK_a, jissen_key_event},
                 {NES_EXPANSION_FAMILY_TRAINER_A, SDL_SCANCODE_Q, SDLK_q, mat_key_event},
                 {NES_EXPANSION_FCNS_CONTROLLER, SDL_SCANCODE_KP_5, SDLK_KP_5, extended_port_key_event}};
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        CHECK(joypad_set_expansion_device(cases[i].device));
        frontend_host_input_release_all();
        NesStateWriter before, held, after;
        nes_state_writer_init(&before, NES_STATE_MAX_SIZE);
        nes_state_writer_init(&held, NES_STATE_MAX_SIZE);
        nes_state_writer_init(&after, NES_STATE_MAX_SIZE);
        CHECK(joypad_state_capture(&before) && special_peripherals_state_capture(&before));
        SDL_KeyboardEvent key = {.type = SDL_KEYDOWN};
        key.keysym.scancode = cases[i].key;
        key.keysym.sym = cases[i].symbol;
        CHECK(cases[i].event(&key));
        CHECK(joypad_state_capture(&held) && special_peripherals_state_capture(&held));
        CHECK(before.size == held.size && memcmp(before.data, held.data, before.size));
        frontend_host_input_release_all();
        /* A new left Shift release must not resurrect a pre-focus right Shift. */
        if (cases[i].device == NES_EXPANSION_SUBOR_KEYBOARD) {
            key.keysym.scancode = SDL_SCANCODE_LSHIFT;
            CHECK(subor_key_event(&key));
            key.type = SDL_KEYUP;
            CHECK(subor_key_event(&key));
        }
        CHECK(joypad_state_capture(&after) && special_peripherals_state_capture(&after));
        CHECK(before.size == after.size && !memcmp(before.data, after.data, before.size));
        nes_state_writer_destroy(&before);
        nes_state_writer_destroy(&held);
        nes_state_writer_destroy(&after);
    }
    CHECK(joypad_set_expansion_device(previous));
    return failures;
}

int test_host_input_accuracy(void) {
    int failures = 0;
    CHECK(SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER) == 0);
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    FrontendBindingProfile *profile = frontend_settings_active_profile(&settings);
    FrontendExecutionRuntime runtime;
    frontend_execution_init(&runtime, NULL, 44100, NULL, NULL, NULL, NULL);
    frontend_host_input_release_all();
    int device = SDL_JoystickAttachVirtual(SDL_JOYSTICK_TYPE_GAMECONTROLLER, SDL_CONTROLLER_AXIS_MAX,
                                          SDL_CONTROLLER_BUTTON_MAX, 0);
    CHECK(device >= 0);
    if (device < 0) return failures;
    SDL_JoystickID id = SDL_JoystickGetDeviceInstanceID(device);
    frontend_host_input_open_controllers(&settings);
    SDL_Event pad = {.type = SDL_CONTROLLERBUTTONDOWN};
    pad.cbutton.which = id;
    pad.cbutton.button = SDL_CONTROLLER_BUTTON_A;
    SDL_KeyboardEvent key = {.type = SDL_KEYDOWN};
    key.keysym.scancode = profile->players[0][BTN_A].key;
    CHECK(frontend_host_input_bound_player_key(profile, &key));
    frontend_host_input_event(&pad, &settings, &runtime);
    key.type = SDL_KEYUP;
    CHECK(frontend_host_input_bound_player_key(profile, &key));
    CHECK(joypad_player(0)->buttons & (1u << BTN_A));
    pad.type = SDL_CONTROLLERBUTTONUP;
    frontend_host_input_event(&pad, &settings, &runtime);
    CHECK(!(joypad_player(0)->buttons & (1u << BTN_A)));

    /* Both release paths must remember the action selected at press time. */
    pad.type = SDL_CONTROLLERBUTTONDOWN;
    frontend_host_input_event(&pad, &settings, &runtime);
    profile->players[0][BTN_A].gamepad_button = SDL_CONTROLLER_BUTTON_B;
    pad.type = SDL_CONTROLLERBUTTONUP;
    frontend_host_input_event(&pad, &settings, &runtime);
    CHECK(!(joypad_player(0)->buttons & (1u << BTN_A)));
    key.type = SDL_KEYDOWN;
    CHECK(frontend_host_input_bound_player_key(profile, &key));
    profile->players[0][BTN_A].key = SDL_SCANCODE_C;
    key.type = SDL_KEYUP;
    CHECK(frontend_host_input_bound_player_key(profile, &key));
    CHECK(!(joypad_player(0)->buttons & (1u << BTN_A)));

    /* D-pad and analog stick can be used together without cancelling holds. */
    SDL_Event axis = {.type = SDL_CONTROLLERAXISMOTION};
    axis.caxis.which = id;
    axis.caxis.axis = SDL_CONTROLLER_AXIS_LEFTX;
    axis.caxis.value = 24000;
    frontend_host_input_event(&axis, &settings, &runtime);
    CHECK(joypad_player(0)->buttons & (1u << BTN_RIGHT));
    pad.type = SDL_CONTROLLERBUTTONDOWN;
    pad.cbutton.button = SDL_CONTROLLER_BUTTON_DPAD_RIGHT;
    frontend_host_input_event(&pad, &settings, &runtime);
    axis.caxis.value = 0;
    frontend_host_input_event(&axis, &settings, &runtime);
    CHECK(joypad_player(0)->buttons & (1u << BTN_RIGHT));
    axis.caxis.value = 24000;
    frontend_host_input_event(&axis, &settings, &runtime);
    pad.type = SDL_CONTROLLERBUTTONUP;
    frontend_host_input_event(&pad, &settings, &runtime);
    CHECK(joypad_player(0)->buttons & (1u << BTN_RIGHT));
    axis.caxis.value = -24000;
    frontend_host_input_event(&axis, &settings, &runtime);
    CHECK((joypad_player(0)->buttons & ((1u << BTN_LEFT) | (1u << BTN_RIGHT))) == (1u << BTN_LEFT));
    axis.caxis.value = 1000;
    frontend_host_input_event(&axis, &settings, &runtime);
    CHECK(!(joypad_player(0)->buttons & ((1u << BTN_LEFT) | (1u << BTN_RIGHT))));
    axis.caxis.axis = SDL_CONTROLLER_AXIS_LEFTY;
    axis.caxis.value = -24000;
    frontend_host_input_event(&axis, &settings, &runtime);
    CHECK(joypad_player(0)->buttons & (1u << BTN_UP));
    axis.caxis.value = 24000;
    frontend_host_input_event(&axis, &settings, &runtime);
    CHECK((joypad_player(0)->buttons & ((1u << BTN_UP) | (1u << BTN_DOWN))) == (1u << BTN_DOWN));

    profile->shortcuts[FRONTEND_SHORTCUT_FAST_FORWARD_HOLD].gamepad_button = SDL_CONTROLLER_BUTTON_RIGHTSHOULDER;
    pad.type = SDL_CONTROLLERBUTTONDOWN;
    pad.cbutton.button = SDL_CONTROLLER_BUTTON_RIGHTSHOULDER;
    frontend_host_input_event(&pad, &settings, &runtime);
    frontend_execution_handle_shortcut_action(&runtime, FRONTEND_SHORTCUT_FAST_FORWARD_HOLD, true, false);
    pad.type = SDL_CONTROLLERBUTTONUP;
    frontend_host_input_event(&pad, &settings, &runtime);
    CHECK(runtime.execution.fast_forward_held);
    frontend_execution_handle_shortcut_action(&runtime, FRONTEND_SHORTCUT_FAST_FORWARD_HOLD, false, false);
    CHECK(!runtime.execution.fast_forward_held);

    key.type = SDL_KEYDOWN;
    key.keysym.scancode = profile->players[0][BTN_A].key;
    CHECK(frontend_host_input_bound_player_key(profile, &key));
    pad.type = SDL_CONTROLLERDEVICEREMOVED;
    pad.cdevice.which = id;
    frontend_host_input_event(&pad, &settings, &runtime);
    CHECK(joypad_player(0)->buttons == (1u << BTN_A));
    key.type = SDL_KEYUP;
    CHECK(frontend_host_input_bound_player_key(profile, &key));
    CHECK(joypad_player(0)->buttons == 0);
    /* The network owner may queue input without changing the local machine. */
    unsigned transitions = 0;
    nes_input_event_set_observer(defer_input, &transitions);
    key.type = SDL_KEYDOWN;
    CHECK(frontend_host_input_bound_player_key(profile, &key));
    CHECK(joypad_player(0)->buttons == 0);
    key.type = SDL_KEYUP;
    CHECK(frontend_host_input_bound_player_key(profile, &key));
    CHECK(transitions == 2u); /* Press (1), then release (0). */
    nes_input_event_clear_observer();
    frontend_host_input_shutdown();
    frontend_execution_shutdown(&runtime);
    failures += peripheral_releases();
    CHECK(SDL_JoystickDetachVirtual(device) == 0);
    SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER);
    printf("Host input: %s\n", failures ? "FAIL" : "PASS");
    return failures;
}
