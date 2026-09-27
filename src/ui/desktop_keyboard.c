/*
 * desktop_keyboard.c - Mouse, touch and physical Family BASIC keys
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "desktop_keyboard.h"
#include "desktop_internal.h"
#include "frontend_panels.h"
#include "peripheral_input.h"
#include "../joypad/family_basic.h"
#include "../joypad/joypad.h"
#include "../cpu/cpu.h"
#include "../system/execution_policy.h"
#include <string.h>

typedef struct {
    FamilyBasicKey key;
    const char *label;
    float units;
} KeyboardKey;

#define KEY(name, label) {FB_KEY_##name, label, 1}
static const KeyboardKey keys[] = {KEY(F1, "F1"),
                                   KEY(F2, "F2"),
                                   KEY(F3, "F3"),
                                   KEY(F4, "F4"),
                                   KEY(F5, "F5"),
                                   KEY(F6, "F6"),
                                   KEY(F7, "F7"),
                                   KEY(F8, "F8"),
                                   KEY(1, "1"),
                                   KEY(2, "2"),
                                   KEY(3, "3"),
                                   KEY(4, "4"),
                                   KEY(5, "5"),
                                   KEY(6, "6"),
                                   KEY(7, "7"),
                                   KEY(8, "8"),
                                   KEY(9, "9"),
                                   KEY(0, "0"),
                                   KEY(MINUS, "-"),
                                   KEY(CARET, "^"),
                                   KEY(YEN, "Yen"),
                                   KEY(STOP, "Stop"),
                                   KEY(ESCAPE, "Esc"),
                                   KEY(Q, "Q"),
                                   KEY(W, "W"),
                                   KEY(E, "E"),
                                   KEY(R, "R"),
                                   KEY(T, "T"),
                                   KEY(Y, "Y"),
                                   KEY(U, "U"),
                                   KEY(I, "I"),
                                   KEY(O, "O"),
                                   KEY(P, "P"),
                                   KEY(AT, "@"),
                                   KEY(LEFT_BRACKET, "["),
                                   KEY(RETURN, "Return"),
                                   KEY(CONTROL, "Ctrl"),
                                   KEY(A, "A"),
                                   KEY(S, "S"),
                                   KEY(D, "D"),
                                   KEY(F, "F"),
                                   KEY(G, "G"),
                                   KEY(H, "H"),
                                   KEY(J, "J"),
                                   KEY(K, "K"),
                                   KEY(L, "L"),
                                   KEY(SEMICOLON, ";"),
                                   KEY(COLON, ":"),
                                   KEY(RIGHT_BRACKET, "]"),
                                   KEY(KANA, "Kana"),
                                   {FB_KEY_LEFT_SHIFT, "Shift", 1.5f},
                                   KEY(Z, "Z"),
                                   KEY(X, "X"),
                                   KEY(C, "C"),
                                   KEY(V, "V"),
                                   KEY(B, "B"),
                                   KEY(N, "N"),
                                   KEY(M, "M"),
                                   KEY(COMMA, ","),
                                   KEY(PERIOD, "."),
                                   KEY(SLASH, "/"),
                                   KEY(UNDERSCORE, "_"),
                                   {FB_KEY_RIGHT_SHIFT, "Shift", 1.5f},
                                   KEY(GRPH, "Grph"),
                                   {FB_KEY_SPACE, "Space", 5},
                                   KEY(CLEAR_HOME, "Home"),
                                   KEY(INSERT, "Ins"),
                                   KEY(DELETE, "Del"),
                                   KEY(UP, "Up"),
                                   KEY(LEFT, "Left"),
                                   KEY(DOWN, "Down"),
                                   KEY(RIGHT, "Right")};
#undef KEY
_Static_assert(sizeof(keys) / sizeof(keys[0]) == FB_KEY_COUNT, "Every matrix key has a visible keycap");
static const unsigned row_count[] = {8, 14, 14, 14, 13, 9};

static struct {
    Uint32 owner;

    struct {
        bool used;
        SDL_FingerID id;
        int key;
    } fingers[16];

    int mouse_key;
    bool initialized;
    bool latched[FB_KEY_COUNT];
} keyboard;

static bool ready(void) {
    return frontend_panel_session_active() && joypad_expansion_device() == NES_EXPANSION_FAMILY_BASIC &&
           !(nes_execution_policy() &
             (NES_EXECUTION_MOVIE_PLAYBACK | NES_EXECUTION_NETPLAY | NES_EXECUTION_REWIND | NES_EXECUTION_SPECULATIVE));
}

static void update_key(int key) {
    if (key < 0 || key >= FB_KEY_COUNT) {
        return;
    }
    bool pressed = keyboard.latched[key] || keyboard.mouse_key == key;
    for (unsigned i = 0; i < 16; ++i) {
        pressed |= keyboard.fingers[i].used && keyboard.fingers[i].key == key;
    }
    (void)family_basic_set_host_key((FamilyBasicKey)key, pressed, true);
}

void desktop_keyboard_release(FrontendDesktopUi *ui) {
    if (ui && (!keyboard.initialized || keyboard.owner != SDL_GetWindowID(ui->window))) {
        return;
    }
    family_basic_release_host_keys();
    memset(&keyboard, 0, sizeof(keyboard));
    keyboard.mouse_key = -1;
}

static void observe(FrontendDesktopUi *ui) {
    Uint32 owner = SDL_GetWindowID(ui->window);
    if (!keyboard.initialized || keyboard.owner != owner) {
        desktop_keyboard_release(NULL);
        keyboard.owner = owner;
        keyboard.initialized = true;
    }
    if (!ready()) {
        desktop_keyboard_release(ui);
        keyboard.owner = owner;
        keyboard.initialized = true;
    }
}

void desktop_keyboard_layout(FrontendDesktopUi *ui, float width, float height) {
    (void)height;
    observe(ui);
    bool enabled = ready();
    CLAY_AUTO_ID({.layout = {.layoutDirection = CLAY_TOP_TO_BOTTOM,
                             .childGap = 8,
                             .sizing = {.width = CLAY_SIZING_FIXED(width)}}}) {
        CLAY_TEXT(CLAY_STRING("Family BASIC Keyboard"),
                  CLAY_TEXT_CONFIG({.fontSize = 22, .textColor = {232, 237, 247, 255}}));
        const char *status =
            enabled ? "Hold a key with the mouse or touch. Right-click to latch a key. Physical keys light up here."
                    : "Select the Family BASIC expansion keyboard in Controllers settings. Playback owns its recorded "
                      "input.";
        CLAY_TEXT(desktop_clay_string(ui->clay, status),
                  CLAY_TEXT_CONFIG({.fontSize = 13, .textColor = {165, 181, 207, 255}}));
        unsigned offset = 0;
        for (unsigned row = 0; row < 6; ++row) {
            float units = 0;
            for (unsigned n = 0; n < row_count[row]; ++n) {
                units += keys[offset + n].units;
            }
            float unit = (width - 4 * (row_count[row] - 1)) / units;
            CLAY_AUTO_ID({.layout = {.childGap = 4}}) {
                for (unsigned n = 0; n < row_count[row]; ++n) {
                    const KeyboardKey *key = &keys[offset + n];
                    bool pressed = family_basic_key_pressed(key->key);
                    Clay_ElementId id = desktop_clay_hit(ui->clay, enabled ? HIT_KEYBOARD_KEY : HIT_NONE, key->key, 0);
                    CLAY(id,
                         {.layout = {.sizing = {.width = CLAY_SIZING_FIXED(unit * key->units),
                                                .height = CLAY_SIZING_FIXED(40)},
                                     .childAlignment = {.x = CLAY_ALIGN_X_CENTER, .y = CLAY_ALIGN_Y_CENTER}},
                          .backgroundColor = pressed ? (Clay_Color){67, 84, 125, 255} : (Clay_Color){39, 47, 64, 255},
                          .border = {.color = {100, 117, 149, 255}, .width = {1, 1, 1, 1, 0}}}) {
                        CLAY_TEXT(desktop_clay_string(ui->clay, key->label),
                                  CLAY_TEXT_CONFIG({.fontSize = 12, .textColor = {232, 237, 247, 255}}));
                    }
                }
            }
            offset += row_count[row];
        }
    }
}

static int key_at(FrontendDesktopUi *ui, float x, float y) {
    const DesktopHit *hit = desktop_clay_at(ui->clay, x, y);
    return hit && hit->kind == HIT_KEYBOARD_KEY ? hit->index : -1;
}

bool desktop_keyboard_event(FrontendDesktopUi *ui, const SDL_Event *event) {
    bool own = ui->panel_open && ui->panel_id == DESKTOP_KEYBOARD_PANEL;
    if (!own || ui->edit_text_active || ui->settings_open || ui->open_menu >= 0 ||
        (event->type == SDL_WINDOWEVENT &&
         (event->window.event == SDL_WINDOWEVENT_FOCUS_LOST || event->window.event == SDL_WINDOWEVENT_CLOSE))) {
        if (keyboard.initialized && keyboard.owner == SDL_GetWindowID(ui->window)) {
            desktop_keyboard_release(ui);
        }
        return false;
    }
    observe(ui);
    if (!ready()) {
        return event->type == SDL_KEYDOWN || event->type == SDL_KEYUP || event->type == SDL_TEXTINPUT;
    }
    if (event->type == SDL_KEYDOWN || event->type == SDL_KEYUP) {
        (void)family_basic_key_event(&event->key, NULL, NULL, cpu_total_cycles);
        return true;
    }
    if (event->type == SDL_TEXTINPUT) {
        return true;
    }
    float scale = ui->ui_scale > 0 ? ui->ui_scale : 1;
    if (event->type == SDL_MOUSEBUTTONDOWN && event->button.which != SDL_TOUCH_MOUSEID) {
        int key = key_at(ui, event->button.x / scale, event->button.y / scale);
        if (key < 0) {
            return false;
        }
        if (event->button.button == SDL_BUTTON_RIGHT) {
            keyboard.latched[key] = !keyboard.latched[key];
        } else if (event->button.button == SDL_BUTTON_LEFT) {
            int previous = keyboard.mouse_key;
            keyboard.mouse_key = key;
            update_key(previous);
        } else {
            return false;
        }
        update_key(key);
        return true;
    }
    if (event->type == SDL_MOUSEBUTTONUP && event->button.button == SDL_BUTTON_LEFT && keyboard.mouse_key >= 0) {
        int key = keyboard.mouse_key;
        keyboard.mouse_key = -1;
        update_key(key);
        return true;
    }
    if (event->type == SDL_FINGERDOWN || event->type == SDL_FINGERUP || event->type == SDL_FINGERMOTION) {
        int slot = -1;
        for (unsigned i = 0; i < 16; ++i) {
            if (keyboard.fingers[i].used && keyboard.fingers[i].id == event->tfinger.fingerId) {
                slot = (int)i;
            }
        }
        if (slot < 0 && event->type == SDL_FINGERDOWN) {
            for (unsigned i = 0; i < 16; ++i) {
                if (!keyboard.fingers[i].used) {
                    slot = (int)i;
                    break;
                }
            }
        }
        if (slot < 0) {
            return true;
        }
        int previous = keyboard.fingers[slot].used ? keyboard.fingers[slot].key : -1;
        int width, height;
        SDL_GetWindowSize(ui->window, &width, &height);
        keyboard.fingers[slot].id = event->tfinger.fingerId;
        keyboard.fingers[slot].used = event->type != SDL_FINGERUP;
        keyboard.fingers[slot].key = event->type == SDL_FINGERUP ? -1
                                                                 : key_at(ui, event->tfinger.x * width / scale,
                                                                          event->tfinger.y * height / scale);
        update_key(previous);
        update_key(keyboard.fingers[slot].key);
        return true;
    }
    return false;
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t size) {
    (void)context;
    (void)error;
    (void)size;
    model->status = "Family BASIC keyboard";
    return true;
}

bool desktop_keyboard_register(void) {
    FrontendPanelInfo existing;
    if (frontend_panel_get(DESKTOP_KEYBOARD_PANEL, &existing)) {
        return true;
    }
    FrontendPanelSpec panel = {
        DESKTOP_KEYBOARD_PANEL, "Family BASIC Keyboard", "Media", FRONTEND_PANEL_NEEDS_SESSION, snapshot, NULL, NULL};
    return frontend_panel_register(&panel);
}
