/*
 * gtk_input_accuracy.c - Native input queue, key aliases and focus regressions
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/gtk_internal.h"
#include "../ui/gtk_desktop.h"
#include "../joypad/joypad.h"
#include <stdio.h>
#include <string.h>

#define CHECK(expression)                                                                                              \
    do {                                                                                                               \
        if (!(expression)) {                                                                                           \
            fprintf(stderr, "GTK input regression failed at line %d: %s\n", __LINE__, #expression);                    \
            ok = false;                                                                                                \
            goto cleanup;                                                                                              \
        }                                                                                                              \
    } while (0)

/* Called by the full GTK smoke executable using its isolated, paused fixture. */
bool test_gtk_input_accuracy(FrontendDesktopUi *ui) {
    if (!ui || !ui->gtk || !ui->execution) {
        return false;
    }
    CupidGtkDesktop *d = ui->gtk;
    bool ok = true;
    int old_x = d->mouse_x, old_y = d->mouse_y;
    uint32_t old_buttons = d->mouse_buttons, old_consumed = d->consumed_mouse_buttons;
    bool old_held[SDL_NUM_SCANCODES];
    memcpy(old_held, d->held, sizeof(old_held));
    uint8_t old_players[NES_INPUT_PLAYERS];
    for (unsigned p = 0; p < NES_INPUT_PLAYERS; ++p) {
        old_players[p] = joypad_player(p)->buttons;
    }
    bool old_rewind = ui->execution->rewind_held, old_fast = ui->execution->execution.fast_forward_held;
    SDL_Keymod old_mods = SDL_GetModState();
    Uint32 window_id = SDL_GetWindowID(ui->window);

    const struct {
        guint numeric, alias;
        SDL_Scancode expected;
    } keys[] = {{GDK_KEY_KP_0, GDK_KEY_KP_Insert, SDL_SCANCODE_KP_0},
                {GDK_KEY_KP_1, GDK_KEY_KP_End, SDL_SCANCODE_KP_1},
                {GDK_KEY_KP_2, GDK_KEY_KP_Down, SDL_SCANCODE_KP_2},
                {GDK_KEY_KP_3, GDK_KEY_KP_Page_Down, SDL_SCANCODE_KP_3},
                {GDK_KEY_KP_4, GDK_KEY_KP_Left, SDL_SCANCODE_KP_4},
                {GDK_KEY_KP_5, GDK_KEY_KP_Begin, SDL_SCANCODE_KP_5},
                {GDK_KEY_KP_6, GDK_KEY_KP_Right, SDL_SCANCODE_KP_6},
                {GDK_KEY_KP_7, GDK_KEY_KP_Home, SDL_SCANCODE_KP_7},
                {GDK_KEY_KP_8, GDK_KEY_KP_Up, SDL_SCANCODE_KP_8},
                {GDK_KEY_KP_9, GDK_KEY_KP_Page_Up, SDL_SCANCODE_KP_9},
                {GDK_KEY_KP_Decimal, GDK_KEY_KP_Delete, SDL_SCANCODE_KP_PERIOD},
                {GDK_KEY_KP_Enter, GDK_KEY_KP_Enter, SDL_SCANCODE_KP_ENTER},
                {GDK_KEY_KP_Add, GDK_KEY_KP_Add, SDL_SCANCODE_KP_PLUS},
                {GDK_KEY_KP_Subtract, GDK_KEY_KP_Subtract, SDL_SCANCODE_KP_MINUS},
                {GDK_KEY_KP_Multiply, GDK_KEY_KP_Multiply, SDL_SCANCODE_KP_MULTIPLY},
                {GDK_KEY_KP_Divide, GDK_KEY_KP_Divide, SDL_SCANCODE_KP_DIVIDE},
                {GDK_KEY_KP_Equal, GDK_KEY_KP_Equal, SDL_SCANCODE_KP_EQUALS}};

    for (unsigned i = 0; i < G_N_ELEMENTS(keys); ++i) {
        CHECK(cupid_gtk_scancode(keys[i].numeric) == keys[i].expected);
        CHECK(cupid_gtk_scancode(keys[i].alias) == keys[i].expected);
    }
    CHECK(cupid_gtk_scancode(GDK_KEY_KP_Left) != cupid_gtk_scancode(GDK_KEY_Left));
    CHECK(cupid_gtk_scancode(GDK_KEY_KP_Enter) != cupid_gtk_scancode(GDK_KEY_Return));

    SDL_FlushEvents(SDL_MOUSEMOTION, SDL_MOUSEBUTTONUP);

    const struct {
        Uint32 type;
        Uint8 button;
        int x, y;
        Uint32 state;
    } pointer[] = {{SDL_MOUSEBUTTONDOWN, SDL_BUTTON_LEFT, 11, 19, SDL_BUTTON_LMASK},
                   {SDL_MOUSEBUTTONDOWN, SDL_BUTTON_RIGHT, 37, 43, SDL_BUTTON_LMASK | SDL_BUTTON_RMASK},
                   {SDL_MOUSEMOTION, 0, 73, 83, SDL_BUTTON_LMASK | SDL_BUTTON_RMASK},
                   {SDL_MOUSEBUTTONUP, SDL_BUTTON_LEFT, 97, 101, SDL_BUTTON_RMASK},
                   {SDL_MOUSEBUTTONUP, SDL_BUTTON_RIGHT, -7, 229, 0},
                   {SDL_MOUSEMOTION, 0, 131, -17, SDL_BUTTON_MMASK},
                   {SDL_MOUSEBUTTONUP, SDL_BUTTON_MIDDLE, 139, 157, 0}};

    d->consumed_mouse_buttons = 0;
    /* The producer has already reached the last event before consumption starts. */
    d->mouse_buttons = 0;
    d->mouse_x = 139;
    d->mouse_y = 157;
    for (unsigned i = 0; i < G_N_ELEMENTS(pointer); ++i) {
        SDL_Event event;
        memset(&event, 0, sizeof(event));
        event.type = pointer[i].type;
        if (event.type == SDL_MOUSEMOTION) {
            event.motion.windowID = window_id;
            event.motion.x = pointer[i].x;
            event.motion.y = pointer[i].y;
            event.motion.state = pointer[i].state;
        } else {
            event.button.windowID = window_id;
            event.button.button = pointer[i].button;
            event.button.x = pointer[i].x;
            event.button.y = pointer[i].y;
            event.button.state = event.type == SDL_MOUSEBUTTONDOWN ? SDL_PRESSED : SDL_RELEASED;
        }
        CHECK(SDL_PushEvent(&event) == 1);
    }
    for (unsigned i = 0; i < G_N_ELEMENTS(pointer); ++i) {
        SDL_Event event;
        int x = 0, y = 0;
        CHECK(SDL_PeepEvents(&event, 1, SDL_GETEVENT, SDL_MOUSEMOTION, SDL_MOUSEBUTTONUP) == 1);
        CHECK(event.type == pointer[i].type);
        CHECK(cupid_gtk_pointer(ui, &event, &x, &y) == pointer[i].state);
        CHECK(x == pointer[i].x && y == pointer[i].y);
    }

    /* Exercise the same release operation as the inactive-window callback.
     * Xvfb without a window manager may keep is-active true even after hide. */
    CHECK(SDL_GetHintBoolean(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, SDL_FALSE) == SDL_TRUE);
    SDL_FlushEvents(SDL_KEYDOWN, SDL_KEYUP);
    SDL_FlushEvents(SDL_MOUSEMOTION, SDL_MOUSEBUTTONUP);
    memset(d->held, 0, sizeof(d->held));
    const SDL_Scancode held[] = {SDL_SCANCODE_A, SDL_SCANCODE_KP_1, SDL_SCANCODE_LSHIFT};
    for (unsigned i = 0; i < G_N_ELEMENTS(held); ++i) {
        d->held[held[i]] = true;
    }
    SDL_SetModState(KMOD_LSHIFT);
    /* Seed the same player state written by mapped gamepad-down events. */
    for (unsigned p = 0; p < NES_INPUT_PLAYERS; ++p) {
        CHECK(joypad_set_player(p, BTN_A, true));
        CHECK(joypad_set_player(p, BTN_RIGHT, true));
        CHECK(joypad_player(p)->buttons != 0);
    }
    ui->execution->rewind_held = true;
    execution_control_set_fast_forward_held(&ui->execution->execution, true);
    d->mouse_x = 211;
    d->mouse_y = 173;
    d->mouse_buttons = d->consumed_mouse_buttons = SDL_BUTTON_LMASK | SDL_BUTTON_RMASK;
    cupid_gtk_release_input(d);
    CHECK(!d->mouse_buttons && SDL_GetModState() == KMOD_NONE);
    CHECK(!ui->execution->rewind_held && !ui->execution->execution.fast_forward_held);
    for (unsigned p = 0; p < NES_INPUT_PLAYERS; ++p) {
        CHECK(joypad_player(p)->buttons == 0);
    }
    for (unsigned i = 0; i < SDL_NUM_SCANCODES; ++i) {
        CHECK(!d->held[i]);
    }
    unsigned released = 0;
    SDL_Event event;
    while (SDL_PeepEvents(&event, 1, SDL_GETEVENT, SDL_KEYUP, SDL_KEYUP) == 1) {
        CHECK(event.key.windowID == window_id);
        unsigned bit = 0;
        for (unsigned i = 0; i < G_N_ELEMENTS(held); ++i) {
            if (event.key.keysym.scancode == held[i]) {
                bit = 1u << i;
            }
        }
        CHECK(bit && !(released & bit));
        released |= bit;
    }
    CHECK(released == (1u << G_N_ELEMENTS(held)) - 1);
    unsigned mouse_released = 0;
    while (SDL_PeepEvents(&event, 1, SDL_GETEVENT, SDL_MOUSEBUTTONUP, SDL_MOUSEBUTTONUP) == 1) {
        CHECK(event.button.windowID == window_id);
        unsigned bit = SDL_BUTTON(event.button.button);
        CHECK((bit & (SDL_BUTTON_LMASK | SDL_BUTTON_RMASK)) && !(mouse_released & bit));
        mouse_released |= bit;
        int x = 0, y = 0;
        CHECK(cupid_gtk_pointer(ui, &event, &x, &y) == ((SDL_BUTTON_LMASK | SDL_BUTTON_RMASK) & ~mouse_released));
        CHECK(x == 211 && y == 173);
    }
    CHECK(mouse_released == (SDL_BUTTON_LMASK | SDL_BUTTON_RMASK));
    CHECK(d->consumed_mouse_buttons == 0);
    /* Repeated releases must be harmless, including the background-input hint. */
    cupid_gtk_release_input(d);
    CHECK(SDL_GetHintBoolean(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, SDL_FALSE) == SDL_TRUE);
    CHECK(SDL_PeepEvents(&event, 1, SDL_GETEVENT, SDL_KEYUP, SDL_KEYUP) == 0);
    CHECK(SDL_PeepEvents(&event, 1, SDL_GETEVENT, SDL_MOUSEBUTTONUP, SDL_MOUSEBUTTONUP) == 0);
cleanup:
    SDL_FlushEvents(SDL_KEYDOWN, SDL_KEYUP);
    SDL_FlushEvents(SDL_MOUSEMOTION, SDL_MOUSEBUTTONUP);
    memcpy(d->held, old_held, sizeof(old_held));
    SDL_SetModState(old_mods);
    d->mouse_x = old_x;
    d->mouse_y = old_y;
    d->mouse_buttons = old_buttons;
    d->consumed_mouse_buttons = old_consumed;
    for (unsigned p = 0; p < NES_INPUT_PLAYERS; ++p) {
        joypad_player(p)->buttons = old_players[p];
    }
    ui->execution->rewind_held = old_rewind;
    execution_control_set_fast_forward_held(&ui->execution->execution, old_fast);
    frontend_execution_refresh_audio(ui->execution);
    printf("GTK input queue coordinates, keypad aliases and focus releases: %s\n", ok ? "PASS" : "FAIL");
    return ok;
}
