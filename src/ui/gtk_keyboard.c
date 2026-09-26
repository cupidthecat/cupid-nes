/*
 * gtk_keyboard.c - Native Family BASIC keyboard
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "gtk_keyboard.h"
#ifdef CUPID_GTK
#include "../joypad/family_basic.h"
#include "../joypad/joypad.h"
#include "../debugger/debugger.h"
#include "../system/execution_policy.h"

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

typedef struct Keyboard Keyboard;

typedef struct {
    Keyboard *owner;
    FamilyBasicKey key;
    GtkWidget *button;
    bool held, latched;
} Key;

struct Keyboard {
    GtkWidget *root, *status, *rows;
    Key keys[FB_KEY_COUNT];
    uint64_t session;
};

static Keyboard *active_owner;

static bool ready(void) {
    return frontend_panel_session_active() && joypad_expansion_device() == NES_EXPANSION_FAMILY_BASIC &&
           !(nes_execution_policy() &
             (NES_EXECUTION_MOVIE_PLAYBACK | NES_EXECUTION_NETPLAY | NES_EXECUTION_REWIND | NES_EXECUTION_SPECULATIVE));
}

static void release_keys(Keyboard *k) {
    for (unsigned i = 0; i < FB_KEY_COUNT; ++i) {
        Key *key = &k->keys[i];
        if (key->held || key->latched) {
            family_basic_set_host_key(key->key, false, true);
        }
        key->held = key->latched = false;
    }
    if (active_owner == k) {
        active_owner = NULL;
    }
}

static void send_key(Key *key) {
    if (!ready()) {
        release_keys(key->owner);
        return;
    }
    /* A late release from a former owner must not clear the new owner's keys. */
    if (!key->held && !key->latched && active_owner != key->owner) {
        return;
    }
    if (active_owner && active_owner != key->owner) {
        release_keys(active_owner);
    }
    active_owner = key->owner;
    family_basic_set_host_key(key->key, key->held || key->latched, true);
}

static void pressed(GtkGestureClick *gesture, int count, double x, double y, gpointer data) {
    (void)count;
    (void)x;
    (void)y;
    Key *key = data;
    if (gtk_gesture_single_get_current_button(GTK_GESTURE_SINGLE(gesture)) == 3) {
        key->latched = !key->latched;
    } else {
        key->held = true;
    }
    send_key(key);
    gtk_gesture_set_state(GTK_GESTURE(gesture), GTK_EVENT_SEQUENCE_CLAIMED);
}

static void released(GtkGestureClick *gesture, int count, double x, double y, gpointer data) {
    (void)gesture;
    (void)count;
    (void)x;
    (void)y;
    Key *key = data;
    key->held = false;
    send_key(key);
}

static void cancelled(GtkGesture *gesture, GdkEventSequence *sequence, gpointer data) {
    (void)gesture;
    (void)sequence;
    Key *key = data;
    key->held = false;
    send_key(key);
}

static gboolean key_down(GtkEventControllerKey *controller, guint value, guint code, GdkModifierType mods,
                         gpointer data) {
    (void)controller;
    (void)code;
    (void)mods;
    if (value != GDK_KEY_space && value != GDK_KEY_Return) {
        return FALSE;
    }
    Key *key = data;
    key->held = true;
    send_key(key);
    return TRUE;
}

static void key_up(GtkEventControllerKey *controller, guint value, guint code, GdkModifierType mods, gpointer data) {
    (void)controller;
    (void)code;
    (void)mods;
    if (value == GDK_KEY_space || value == GDK_KEY_Return) {
        Key *key = data;
        key->held = false;
        send_key(key);
    }
}

static void lost_focus(GtkEventControllerFocus *controller, gpointer data) {
    (void)controller;
    release_keys(data);
}

static void window_active(GObject *window, GParamSpec *spec, gpointer data) {
    (void)spec;
    Keyboard *k = g_object_get_data(G_OBJECT(data), "cupid-keyboard");
    if (k && !gtk_window_is_active(GTK_WINDOW(window))) {
        release_keys(k);
    }
}

static void unmapped(GtkWidget *widget, gpointer data) {
    (void)widget;
    release_keys(data);
}

static void release_clicked(GtkButton *button, gpointer data) {
    (void)button;
    release_keys(data);
}

static void destroyed(gpointer data) {
    release_keys(data);
    g_free(data);
}

void cupid_gtk_keyboard_refresh(GtkWidget *root) {
    Keyboard *k = g_object_get_data(G_OBJECT(root), "cupid-keyboard");
    if (!k) {
        return;
    }
    if (!ready() || k->session != debugger_session_revision()) {
        release_keys(k);
    }
    k->session = debugger_session_revision();
    gtk_widget_set_sensitive(k->rows, ready());
    gtk_label_set_text(
        GTK_LABEL(k->status),
        ready() ? "Hold a key to type. Right-click to latch a modifier; Release all clears held keys."
                : "Select the Family BASIC expansion keyboard. Recorded playback and netplay input are read-only.");
    for (unsigned i = 0; i < FB_KEY_COUNT; ++i) {
        Key *key = &k->keys[i];
        if (family_basic_key_pressed(key->key)) {
            gtk_widget_add_css_class(key->button, "suggested-action");
        } else {
            gtk_widget_remove_css_class(key->button, "suggested-action");
        }
    }
}

GtkWidget *cupid_gtk_keyboard_new(CupidGtkTool *tool) {
    Keyboard *k = g_new0(Keyboard, 1);
    k->root = gtk_box_new(GTK_ORIENTATION_VERTICAL, 8);
    cupid_gtk_margins(k->root, 12);
    k->status = cupid_gtk_label("");
    gtk_label_set_wrap(GTK_LABEL(k->status), TRUE);
    gtk_box_append(GTK_BOX(k->root), k->status);
    k->rows = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_box_append(GTK_BOX(k->root), cupid_gtk_scroll(k->rows));
    unsigned offset = 0;
    for (unsigned row = 0; row < G_N_ELEMENTS(row_count); ++row) {
        GtkWidget *box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);
        gtk_box_append(GTK_BOX(k->rows), box);
        for (unsigned n = 0; n < row_count[row]; ++n, ++offset) {
            Key *key = &k->keys[offset];
            key->owner = k;
            key->key = keys[offset].key;
            key->button = gtk_button_new_with_label(keys[offset].label);
            gtk_widget_set_size_request(key->button, (int)(42 * keys[offset].units), 42);
            gtk_widget_set_hexpand(key->button, TRUE);
            gtk_box_append(GTK_BOX(box), key->button);
            GtkGesture *gesture = gtk_gesture_click_new();
            gtk_gesture_single_set_button(GTK_GESTURE_SINGLE(gesture), 0);
            gtk_event_controller_set_propagation_phase(GTK_EVENT_CONTROLLER(gesture), GTK_PHASE_CAPTURE);
            g_signal_connect(gesture, "pressed", G_CALLBACK(pressed), key);
            g_signal_connect(gesture, "released", G_CALLBACK(released), key);
            g_signal_connect(gesture, "cancel", G_CALLBACK(cancelled), key);
            gtk_widget_add_controller(key->button, GTK_EVENT_CONTROLLER(gesture));
            GtkEventController *controller = gtk_event_controller_key_new();
            gtk_event_controller_set_propagation_phase(controller, GTK_PHASE_CAPTURE);
            g_signal_connect(controller, "key-pressed", G_CALLBACK(key_down), key);
            g_signal_connect(controller, "key-released", G_CALLBACK(key_up), key);
            gtk_widget_add_controller(key->button, controller);
        }
    }
    GtkWidget *release = gtk_button_new_with_label("Release all");
    gtk_widget_set_halign(release, GTK_ALIGN_END);
    gtk_widget_set_size_request(release, 100, -1);
    g_signal_connect(release, "clicked", G_CALLBACK(release_clicked), k);
    gtk_box_append(GTK_BOX(k->root), release);
    GtkEventController *focus = gtk_event_controller_focus_new();
    g_signal_connect(focus, "leave", G_CALLBACK(lost_focus), k);
    gtk_widget_add_controller(k->root, focus);
    g_signal_connect(k->root, "unmap", G_CALLBACK(unmapped), k);
    g_object_set_data_full(G_OBJECT(k->root), "cupid-keyboard", k, destroyed);
    if (tool && tool->window) {
        g_signal_connect_object(tool->window, "notify::is-active", G_CALLBACK(window_active), k->root, 0);
    }
    cupid_gtk_keyboard_refresh(k->root);
    return k->root;
}
#endif
