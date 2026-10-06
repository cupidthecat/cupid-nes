/*
 * gtk_state_accuracy.c - Save-state dialogs through native menu and shortcut actions
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/gtk_internal.h"
#include "../ui/gtk_desktop.h"
#include "../ui/state_frontend.h"
#include "../ui/platform_frontend.h"
#include <stdio.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "GTK state failure %d: %s\n", __LINE__, #x); return false; } } while (0)

typedef struct {
    GtkWindow *parent;
    bool seen, save, correct;
    unsigned checks;
} DialogProbe;

static gboolean cancel_state_dialog(gpointer data) {
    DialogProbe *probe = data;
    GtkNativeDialog *dialog = g_object_get_data(G_OBJECT(probe->parent), "cupid-file-dialog");
    if (dialog) {
        probe->seen = true;
        if (gtk_native_dialog_get_modal(dialog)) probe->checks |= 1;
        if (gtk_native_dialog_get_transient_for(dialog) == probe->parent) probe->checks |= 2;
        if (!g_strcmp0(gtk_native_dialog_get_title(dialog), probe->save ? "Save State" : "Load State"))
            probe->checks |= 4;
        if (gtk_file_chooser_get_action(GTK_FILE_CHOOSER(dialog)) ==
            (probe->save ? GTK_FILE_CHOOSER_ACTION_SAVE : GTK_FILE_CHOOSER_ACTION_OPEN)) probe->checks |= 8;
        GListModel *filters = gtk_file_chooser_get_filters(GTK_FILE_CHOOSER(dialog));
        GtkFileFilter *states = g_list_model_get_item(filters, 0);
        GtkFileFilter *all = g_list_model_get_item(filters, 1);
        if (g_list_model_get_n_items(filters) == 2 && states && all &&
            !g_strcmp0(gtk_file_filter_get_name(states), "Save states") &&
            !g_strcmp0(gtk_file_filter_get_name(all), "All files")) probe->checks |= 16;
        probe->correct = probe->checks == 31;
        g_clear_object(&states);
        g_clear_object(&all);
        g_object_unref(filters);
        g_signal_emit_by_name(dialog, "response", GTK_RESPONSE_CANCEL);
    }
    return G_SOURCE_REMOVE;
}

static char *find_action(GMenuModel *menu, const char *label) {
    for (int i = 0; i < g_menu_model_get_n_items(menu); ++i) {
        char *text = NULL, *action = NULL;
        g_menu_model_get_item_attribute(menu, i, G_MENU_ATTRIBUTE_LABEL, "s", &text);
        if (text && g_str_has_prefix(text, label)) {
            g_menu_model_get_item_attribute(menu, i, G_MENU_ATTRIBUTE_ACTION, "s", &action);
        }
        g_free(text);
        if (action) return action;
        const char *links[] = {G_MENU_LINK_SECTION, G_MENU_LINK_SUBMENU};
        for (unsigned j = 0; j < G_N_ELEMENTS(links); ++j) {
            GMenuModel *child = g_menu_model_get_item_link(menu, i, links[j]);
            if (!child) continue;
            action = find_action(child, label);
            g_object_unref(child);
            if (action) return action;
        }
    }
    return NULL;
}

bool test_gtk_state_dialogs(FrontendDesktopUi *ui) {
    char original[FRONTEND_SETTINGS_PATH_TEXT];
    g_strlcpy(original, ui->settings->state_file_path, sizeof(original));
    for (unsigned mode = 0; mode < 4; ++mode) {
        bool save = (mode & 1u) == 0;
        /* Remembered paths must still prompt; quick slots have separate commands. */
        g_strlcpy(ui->settings->state_file_path, mode < 2 ? "" : "remembered.cst",
                  sizeof(ui->settings->state_file_path));
        if (mode >= 2) {
            /* A canceled native chooser can still own focus while it closes.
             * Wait for the game view before sending the next shortcut. */
            gint64 deadline = g_get_monotonic_time() + 3000000;
            do {
                gtk_window_present(GTK_WINDOW(ui->gtk->window));
                gtk_widget_grab_focus(ui->gtk->picture);
                cupid_gtk_dispatch();
                if (!cupid_gtk_captured(ui)) break;
                SDL_Delay(10);
            } while (g_get_monotonic_time() < deadline);
            CHECK(!cupid_gtk_captured(ui));
        }
        DialogProbe probe = {.parent = GTK_WINDOW(ui->gtk->window), .save = save};
        guint source = g_idle_add(cancel_state_dialog, &probe);
        bool invoked;
        if (mode < 2) {
            char *action = find_action(gtk_popover_menu_bar_get_menu_model(GTK_POPOVER_MENU_BAR(ui->gtk->menubar)),
                                       save ? "Save State File" : "Load State File");
            invoked = action && gtk_widget_activate_action(ui->gtk->window, action, NULL);
            g_free(action);
        } else {
            SDL_Event event = {.type = SDL_KEYDOWN};
            event.key.windowID = SDL_GetWindowID(ui->window);
            event.key.keysym.scancode = save ? SDL_SCANCODE_F5 : SDL_SCANCODE_F6;
            event.key.keysym.sym = save ? SDLK_F5 : SDLK_F6;
            event.key.keysym.mod = KMOD_CTRL;
            invoked = frontend_desktop_handle_event(ui, &event);
            event.type = SDL_KEYUP;
            (void)frontend_desktop_handle_event(ui, &event);
        }
        if (g_main_context_find_source_by_id(NULL, source)) g_source_remove(source);
        if (!invoked || !probe.seen || !probe.correct) {
            fprintf(stderr, "State dialog mode %u: invoked=%d seen=%d checks=%u active=%d focus=%d\n",
                    mode, invoked, probe.seen, probe.checks,
                    gtk_window_is_active(GTK_WINDOW(ui->gtk->window)), gtk_widget_has_focus(ui->gtk->picture));
        }
        CHECK(invoked && probe.seen && probe.correct);
        CHECK(!strcmp(ui->settings->state_file_path, mode < 2 ? "" : "remembered.cst"));
        CHECK(!g_object_get_data(G_OBJECT(ui->gtk->window), "cupid-file-dialog"));
        CHECK(frontend_execution_paused(ui->execution));
    }
    g_strlcpy(ui->settings->state_file_path, original, sizeof(ui->settings->state_file_path));
    puts("GTK save-state menu and shortcut dialogs: PASS");
    return true;
}
