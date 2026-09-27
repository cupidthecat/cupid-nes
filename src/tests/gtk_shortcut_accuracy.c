/*
 * gtk_shortcut_accuracy.c - GTK key signals through the desktop command path
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/gtk_internal.h"
#include "../ui/gtk_desktop.h"
#include "../ui/frontend_commands.h"
#include <stdio.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "GTK shortcut failure %d: %s\n", __LINE__, #x); return false; } } while (0)

static bool count_command(void *context, char *error, size_t size) {
    (void)error; (void)size;
    ++*(unsigned *)context;
    return true;
}

static bool send_key(FrontendDesktopUi *ui, GtkEventController *controller,
                     guint key, GdkModifierType mods, bool down) {
    SDL_FlushEvents(SDL_KEYDOWN, SDL_KEYUP);
    if (down) {
        gboolean handled = FALSE;
        g_signal_emit_by_name(controller, "key-pressed", key, 0u, mods, &handled);
        if (!handled) return false;
    } else g_signal_emit_by_name(controller, "key-released", key, 0u, mods);
    SDL_Event event;
    if (SDL_PeepEvents(&event, 1, SDL_GETEVENT, SDL_KEYDOWN, SDL_KEYUP) != 1) return false;
    return frontend_desktop_handle_event(ui, &event);
}

/* Run before registering the full fixture's tools: commands here are harmless
 * counters so save/load/reset/open bindings can all be exercised without dialogs. */
bool test_gtk_shortcuts(FrontendDesktopUi *ui) {
    frontend_commands_reset();
    frontend_command_set_session_active(true);
    unsigned calls[FRONTEND_SHORTCUT_COUNT] = {0};
    for (unsigned i = 0; i < FRONTEND_SHORTCUT_COUNT; ++i) {
        FrontendCommandSpec command = {.id = frontend_execution_shortcut_command((FrontendShortcut)i),
            .label = "Shortcut fixture", .handler = count_command, .userdata = &calls[i]};
        CHECK(frontend_command_register(&command));
    }
    GListModel *controllers = gtk_widget_observe_controllers(ui->gtk->picture);
    GtkEventController *keys = NULL;
    for (unsigned i = 0; i < g_list_model_get_n_items(controllers); ++i) {
        GtkEventController *controller = g_list_model_get_item(controllers, i);
        if (GTK_IS_EVENT_CONTROLLER_KEY(controller)) { keys = controller; break; }
        g_object_unref(controller);
    }
    g_object_unref(controllers);
    CHECK(keys);
    gtk_window_present(GTK_WINDOW(ui->gtk->window));
    CHECK(gtk_widget_grab_focus(ui->gtk->picture));
    for (unsigned i = 0; i < 30; ++i) { cupid_gtk_dispatch(); SDL_Delay(10); }
    CHECK(!cupid_gtk_captured(ui));
    const guint keyvals[] = {GDK_KEY_p, GDK_KEY_period, GDK_KEY_r, GDK_KEY_r, GDK_KEY_r,
        GDK_KEY_f, GDK_KEY_f, GDK_KEY_1, GDK_KEY_2, GDK_KEY_3, GDK_KEY_o,
        GDK_KEY_F5, GDK_KEY_F6, GDK_KEY_F5, GDK_KEY_F6, GDK_KEY_BackSpace, GDK_KEY_a, GDK_KEY_m};
    CHECK(G_N_ELEMENTS(keyvals) == FRONTEND_SHORTCUT_COUNT);
    FrontendBindingProfile *profile = frontend_settings_active_profile(ui->settings);
    CHECK(profile);
    for (unsigned i = 0; i < FRONTEND_SHORTCUT_COUNT; ++i) {
        SDL_Keymod mods = profile->shortcuts[i].modifiers;
        GdkModifierType gmods = (mods & KMOD_CTRL ? GDK_CONTROL_MASK : 0) |
            (mods & KMOD_SHIFT ? GDK_SHIFT_MASK : 0) | (mods & KMOD_ALT ? GDK_ALT_MASK : 0);
        CHECK(send_key(ui, keys, keyvals[i], gmods, true));
        CHECK(send_key(ui, keys, keyvals[i], gmods, true));
        if (i == FRONTEND_SHORTCUT_REWIND) CHECK(ui->execution->rewind_held);
        else if (i == FRONTEND_SHORTCUT_FAST_FORWARD_HOLD) CHECK(ui->execution->execution.fast_forward_held);
        else CHECK(calls[i] == 1);
        /* Releasing Ctrl first must still end rewind / fast-forward. */
        (void)send_key(ui, keys, keyvals[i], 0, false);
        CHECK(!ui->execution->rewind_held && !ui->execution->execution.fast_forward_held);
    }
    FrontendHostBinding saved = profile->shortcuts[FRONTEND_SHORTCUT_REWIND];
    profile->shortcuts[FRONTEND_SHORTCUT_REWIND].key = SDL_SCANCODE_Q;
    profile->shortcuts[FRONTEND_SHORTCUT_REWIND].modifiers = KMOD_SHIFT;
    CHECK(!send_key(ui, keys, GDK_KEY_BackSpace, GDK_CONTROL_MASK, true));
    CHECK(!ui->execution->rewind_held);
    (void)send_key(ui, keys, GDK_KEY_BackSpace, 0, false);
    CHECK(send_key(ui, keys, GDK_KEY_q, GDK_SHIFT_MASK, true));
    CHECK(ui->execution->rewind_held);
    GtkWidget *toolbar = gtk_widget_get_next_sibling(ui->gtk->menubar);
    CHECK(gtk_widget_grab_focus(gtk_widget_get_first_child(toolbar)));
    CHECK(!ui->execution->rewind_held);
    CHECK(send_key(ui, keys, GDK_KEY_q, GDK_SHIFT_MASK, true));
    CHECK(!ui->execution->rewind_held);
    (void)send_key(ui, keys, GDK_KEY_q, 0, false);
    CHECK(gtk_widget_grab_focus(ui->gtk->picture));
    CHECK(send_key(ui, keys, GDK_KEY_q, GDK_SHIFT_MASK, true));
    CHECK(ui->execution->rewind_held);
    (void)send_key(ui, keys, GDK_KEY_q, 0, false);
    CHECK(!ui->execution->rewind_held);
    profile->shortcuts[FRONTEND_SHORTCUT_REWIND] = saved;
    const struct {guint key; GdkModifierType mods; unsigned command;} fixed[] = {
        {GDK_KEY_comma, GDK_CONTROL_MASK, FRONTEND_COMMAND_SETTINGS},
        {GDK_KEY_Return, GDK_ALT_MASK, FRONTEND_COMMAND_FULLSCREEN}, {GDK_KEY_F7, 0, 0x1B00}};
    unsigned fixed_calls = 0;
    for (unsigned i = 0; i < G_N_ELEMENTS(fixed); ++i) {
        FrontendCommandSpec command = {.id = fixed[i].command, .label = "Desktop shortcut fixture",
            .handler = count_command, .userdata = &fixed_calls};
        CHECK(frontend_command_register(&command));
        CHECK(send_key(ui, keys, fixed[i].key, fixed[i].mods, true));
        (void)send_key(ui, keys, fixed[i].key, fixed[i].mods, false);
        CHECK(fixed_calls == i + 1);
    }
    cupid_gtk_release_input(ui->gtk);
    SDL_FlushEvents(SDL_KEYDOWN, SDL_KEYUP);
    g_object_unref(keys);
    frontend_commands_reset();
    puts("GTK shortcuts: all profile bindings, repeats, remapping, release, focus and desktop keys PASS");
    return true;
}
