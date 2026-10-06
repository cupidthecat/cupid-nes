/*
 * gtk_status_accuracy.c - Status refresh, expiry and narrow-window layout
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/gtk_internal.h"
#include "../ui/gtk_desktop.h"
#include "../ui/state_frontend.h"
#include <stdio.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "GTK status failure %d: %s\n", __LINE__, #x); return false; } } while (0)

static void render_status(FrontendDesktopUi *ui, const char *state) {
    frontend_desktop_render(ui, 256, 240, "Status fixture", "NTSC", state);
}

static void changed(GObject *object, GParamSpec *property, gpointer context) {
    (void)object;
    (void)property;
    ++*(unsigned *)context;
}

static GtkWidget *find_button(GtkWidget *widget, const char *label) {
    if (GTK_IS_BUTTON(widget) && !g_strcmp0(gtk_button_get_label(GTK_BUTTON(widget)), label)) return widget;
    for (GtkWidget *child = gtk_widget_get_first_child(widget); child; child = gtk_widget_get_next_sibling(child)) {
        GtkWidget *found = find_button(child, label);
        if (found) return found;
    }
    return NULL;
}

bool test_gtk_status_accuracy(FrontendDesktopUi *ui) {
    bool show_fps = ui->settings->show_fps;
    unsigned slot = ui->settings->state_slot;
    frontend_desktop_set_status(ui, "Old operation failed");
    render_status(ui, "Running");
    CHECK(strstr(gtk_label_get_text(GTK_LABEL(ui->gtk->status)), "Old operation failed"));
    /* Exercise the real expiry path without waiting six seconds. */
    ui->status_until = SDL_GetTicks();
    render_status(ui, "Paused");
    CHECK(!strstr(gtk_label_get_text(GTK_LABEL(ui->gtk->status)), "Old operation failed"));
    CHECK(strstr(gtk_label_get_text(GTK_LABEL(ui->gtk->status)), "Paused"));
    render_status(ui, "Rewinding");
    CHECK(strstr(gtk_label_get_text(GTK_LABEL(ui->gtk->status)), "Rewinding"));
    GtkWidget *details = gtk_widget_get_next_sibling(ui->gtk->status);
    CHECK(GTK_IS_LABEL(details));
    ui->settings->show_fps = true;
    ui->settings->state_slot = 9;
    frontend_desktop_set_fps(ui, 59.9);
    frontend_desktop_set_status(ui, "State file saved");
    render_status(ui, "Running");
    CHECK(strstr(gtk_label_get_text(GTK_LABEL(details)), "59.9 fps"));
    CHECK(strstr(gtk_label_get_text(GTK_LABEL(details)), "Slot 10"));
    CHECK(!strcmp(gtk_label_get_text(GTK_LABEL(ui->gtk->status)), "State file saved"));
    frontend_desktop_set_fps(ui, 0);
    render_status(ui, "Paused");
    CHECK(strstr(gtk_label_get_text(GTK_LABEL(details)), "0.0 fps"));
    ui->settings->show_fps = false;
    render_status(ui, "Paused");
    CHECK(!strstr(gtk_label_get_text(GTK_LABEL(details)), "fps"));
    unsigned updates = 0;
    gulong observer = g_signal_connect(ui->gtk->status, "notify::label", G_CALLBACK(changed), &updates);
    gulong info_observer = g_signal_connect(details, "notify::label", G_CALLBACK(changed), &updates);
    for (unsigned i = 0; i < 20; ++i) render_status(ui, "Paused");
    g_signal_handler_disconnect(ui->gtk->status, observer);
    g_signal_handler_disconnect(details, info_observer);
    CHECK(updates == 0);
    char long_message[sizeof(ui->status)];
    memset(long_message, 'W', sizeof(long_message) - 1);
    long_message[sizeof(long_message) - 1] = 0;
    frontend_desktop_set_status(ui, long_message);
    render_status(ui, "Paused");
    int minimum = 0;
    gtk_widget_measure(ui->gtk->status, GTK_ORIENTATION_HORIZONTAL, -1, &minimum, NULL, NULL, NULL);
    CHECK(minimum < 100);
    CHECK(!strcmp(gtk_widget_get_tooltip_text(ui->gtk->status), long_message));
    FrontendDesktopUi *model = cupid_gtk_open(ui, 1, STATE_PANEL);
    CupidGtkTool *tool = NULL;
    for (CupidGtkTool *t = ui->gtk->tools; t; t = t->next) if (&t->ui == model) tool = t;
    CHECK(tool);
    cupid_gtk_tool_status(tool, "Stale panel failure");
    tool->ui.status_until = SDL_GetTicks();
    ui->gtk->refreshed = SDL_GetTicks() - 100;
    render_status(ui, "Paused");
    CHECK(!strstr(gtk_label_get_text(GTK_LABEL(tool->status)), "Stale panel failure"));
    cupid_gtk_tool_status(tool, "Previous write failed");
    GtkWidget *save = find_button(tool->content, "Save selected slot");
    CHECK(save && gtk_widget_get_sensitive(save));
    g_signal_emit_by_name(save, "clicked");
    CHECK(!tool->ui.status[0]);
    ui->gtk->refreshed = SDL_GetTicks() - 100;
    render_status(ui, "Paused");
    CHECK(!strcmp(gtk_label_get_text(GTK_LABEL(tool->status)), "State slot saved"));
    CHECK(desktop_invoke_command(ui, STATE_COMMAND_SAVE_SLOT));
    CHECK(!strcmp(ui->status, "State slot saved"));
    gtk_widget_set_visible(tool->window, FALSE);
    frontend_desktop_set_status(ui, NULL);
    ui->settings->show_fps = show_fps;
    ui->settings->state_slot = slot;
    puts("GTK status expiry, live FPS, selected slot and bounded layout: PASS");
    return true;
}
