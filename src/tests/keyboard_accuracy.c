/*
 * keyboard_accuracy.c - Virtual keyboard matrix and native input checks
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../joypad/family_basic.h"
#include "../joypad/joypad.h"
#include "../system/execution_policy.h"
#include "../ui/desktop_keyboard.h"
#include "../ui/desktop_internal.h"
#include "../ui/frontend_panels.h"
#include <stdio.h>

static int failures;
#define CHECK(x)                                                                                                       \
    do {                                                                                                               \
        if (!(x)) {                                                                                                    \
            fprintf(stderr, "Keyboard check %d: %s\n", __LINE__, #x);                                                  \
            ++failures;                                                                                                \
        }                                                                                                              \
    } while (0)

static void render(FrontendDesktopUi *ui) {
    desktop_clay_begin(ui->clay, 900, 600, ui->ui_scale);
    desktop_keyboard_layout(ui, 900, 600);
    desktop_clay_end(ui->clay);
}

static bool mouse(FrontendDesktopUi *ui, FamilyBasicKey key, Uint32 type, Uint8 button) {
    SDL_FRect rect;
    if (!desktop_clay_bounds(ui->clay, HIT_KEYBOARD_KEY, key, 0, &rect)) {
        return false;
    }
    SDL_Event event = {.type = type};
    event.button.windowID = SDL_GetWindowID(ui->window);
    event.button.button = button;
    event.button.x = (int)((rect.x + rect.w / 2) * ui->ui_scale);
    event.button.y = (int)((rect.y + rect.h / 2) * ui->ui_scale);
    return frontend_desktop_handle_event(ui, &event);
}

static void physical(FrontendDesktopUi *ui, SDL_Scancode key, bool down) {
    SDL_Event event = {.type = down ? SDL_KEYDOWN : SDL_KEYUP};
    event.key.windowID = SDL_GetWindowID(ui->window);
    event.key.keysym.scancode = key;
    CHECK(frontend_desktop_handle_event(ui, &event));
}

static void touch(FrontendDesktopUi *ui, FamilyBasicKey key, SDL_FingerID finger, bool down) {
    SDL_FRect rect;
    CHECK(desktop_clay_bounds(ui->clay, HIT_KEYBOARD_KEY, key, 0, &rect));
    SDL_Event event = {.type = down ? SDL_FINGERDOWN : SDL_FINGERUP};
    event.tfinger.windowID = SDL_GetWindowID(ui->window);
    event.tfinger.fingerId = finger;
    event.tfinger.x = (rect.x + rect.w / 2) / 900;
    event.tfinger.y = (rect.y + rect.h / 2) / 600;
    CHECK(frontend_desktop_handle_event(ui, &event));
}

static void focus_lost(FrontendDesktopUi *ui) {
    SDL_Event event = {.type = SDL_WINDOWEVENT};
    event.window.windowID = SDL_GetWindowID(ui->window);
    event.window.event = SDL_WINDOWEVENT_FOCUS_LOST;
    (void)frontend_desktop_handle_event(ui, &event);
}

int test_keyboard_accuracy(void) {
    failures = 0;
    SDL_setenv("SDL_VIDEODRIVER", "dummy", 1);
    CHECK(SDL_InitSubSystem(SDL_INIT_VIDEO) == 0);
    SDL_Window *window = SDL_CreateWindow("Keyboard checks", 0, 0, 900, 600, SDL_WINDOW_HIDDEN);
    SDL_Renderer *renderer = window ? SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE) : NULL;
    CHECK(renderer != NULL);
    if (!renderer) {
        if (window) {
            SDL_DestroyWindow(window);
        }
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
        return failures;
    }
    FrontendSettings settings;
    frontend_settings_defaults(&settings);
    FrontendDesktopUi ui;
    frontend_desktop_init(&ui, window, renderer, &settings, NULL, NULL, NULL);
    frontend_panels_reset();
    CHECK(desktop_keyboard_register());
    frontend_panel_set_session_active(true);
    ui.panel_open = true;
    ui.panel_id = DESKTOP_KEYBOARD_PANEL;
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    CHECK(joypad_set_expansion_device(NES_EXPANSION_FAMILY_BASIC));
    family_basic_reset();
    render(&ui);
    for (unsigned key = 0; key < FB_KEY_COUNT; ++key) {
        CHECK(mouse(&ui, (FamilyBasicKey)key, SDL_MOUSEBUTTONDOWN, SDL_BUTTON_LEFT));
        CHECK(family_basic_key_pressed((FamilyBasicKey)key));
        family_basic_write(5, 0);
        for (unsigned row = 0; row < key / 8; ++row) {
            family_basic_write(6, 0);
            family_basic_write(4, 0);
        }
        if (key % 8 >= 4) {
            family_basic_write(6, 0);
        }
        CHECK(family_basic_read(1, 0) == (uint8_t)(0x1e & ~(1u << (key % 4 + 1))));
        CHECK(mouse(&ui, (FamilyBasicKey)key, SDL_MOUSEBUTTONUP, SDL_BUTTON_LEFT));
        CHECK(!family_basic_key_pressed((FamilyBasicKey)key));
        CHECK(family_basic_read(1, 0) == 0x1e);
    }
    physical(&ui, SDL_SCANCODE_A, true);
    CHECK(mouse(&ui, FB_KEY_A, SDL_MOUSEBUTTONDOWN, SDL_BUTTON_LEFT));
    CHECK(mouse(&ui, FB_KEY_A, SDL_MOUSEBUTTONUP, SDL_BUTTON_LEFT));
    CHECK(family_basic_key_pressed(FB_KEY_A));
    physical(&ui, SDL_SCANCODE_A, false);
    CHECK(!family_basic_key_pressed(FB_KEY_A));
    touch(&ui, FB_KEY_LEFT_SHIFT, 10, true);
    touch(&ui, FB_KEY_A, 11, true);
    CHECK(family_basic_key_pressed(FB_KEY_LEFT_SHIFT) && family_basic_key_pressed(FB_KEY_A));
    touch(&ui, FB_KEY_A, 11, false);
    CHECK(family_basic_key_pressed(FB_KEY_LEFT_SHIFT) && !family_basic_key_pressed(FB_KEY_A));
    touch(&ui, FB_KEY_LEFT_SHIFT, 10, false);
    CHECK(!family_basic_key_pressed(FB_KEY_LEFT_SHIFT));
    CHECK(mouse(&ui, FB_KEY_CONTROL, SDL_MOUSEBUTTONDOWN, SDL_BUTTON_RIGHT));
    physical(&ui, SDL_SCANCODE_B, true);
    CHECK(family_basic_key_pressed(FB_KEY_CONTROL) && family_basic_key_pressed(FB_KEY_B));
    focus_lost(&ui);
    CHECK(!family_basic_key_pressed(FB_KEY_CONTROL) && !family_basic_key_pressed(FB_KEY_B));
    const float scales[] = {1, 1.5f, 2};
    for (unsigned i = 0; i < 3; ++i) {
        ui.ui_scale = scales[i];
        SDL_SetWindowSize(window, (int)(900 * ui.ui_scale), (int)(600 * ui.ui_scale));
        render(&ui);
        CHECK(mouse(&ui, FB_KEY_SPACE, SDL_MOUSEBUTTONDOWN, SDL_BUTTON_LEFT));
        CHECK(family_basic_key_pressed(FB_KEY_SPACE));
        CHECK(mouse(&ui, FB_KEY_SPACE, SDL_MOUSEBUTTONUP, SDL_BUTTON_LEFT));
        CHECK(desktop_clay_error_count(ui.clay) == 0);
    }
    CHECK(nes_execution_set_policy(NES_EXECUTION_MOVIE_PLAYBACK));
    render(&ui);
    physical(&ui, SDL_SCANCODE_A, true);
    CHECK(!family_basic_key_pressed(FB_KEY_A));
    SDL_FRect rect;
    CHECK(!desktop_clay_bounds(ui.clay, HIT_KEYBOARD_KEY, FB_KEY_A, 0, &rect));
    CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    desktop_keyboard_release(&ui);
    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    frontend_desktop_shutdown(&ui);
    frontend_panels_reset();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
    printf("Virtual keyboard: matrix scan, shared input, touch, focus and scale checks, %d failures\n", failures);
    return failures;
}
