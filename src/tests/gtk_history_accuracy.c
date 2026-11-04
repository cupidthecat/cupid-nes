/*
 * gtk_history_accuracy.c - Native history controls and live-state isolation
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../ui/gtk_internal.h"
#include "../ui/history_frontend.h"
#include "../ui/gtk_desktop.h"
#include "../../include/globals.h"
#include <string.h>
extern uint8_t ram[0x800];
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "GTK history %d: %s\n", __LINE__, #x); return false; } } while (0)

static GtkWidget *find(GtkWidget *root, GType type, const char *label) {
    if (G_TYPE_CHECK_INSTANCE_TYPE(root, type) && (!label ||
        (GTK_IS_BUTTON(root) && !g_strcmp0(gtk_button_get_label(GTK_BUTTON(root)), label)))) return root;
    for (GtkWidget *w = gtk_widget_get_first_child(root); w; w = gtk_widget_get_next_sibling(w)) {
        GtkWidget *found = find(w, type, label);
        if (found) return found;
    }
    return NULL;
}

static void pump(FrontendDesktopUi *ui) {
    for (unsigned i = 0; i < 8; ++i) {
        frontend_desktop_render(ui, 256, 240, "History fixture", "NTSC", "Paused");
        SDL_Delay(10);
    }
}

bool test_gtk_history(FrontendDesktopUi *ui, const char *out) {
    frontend_panel_set_session_active(true);
    FrontendHistory *history = frontend_history_create(ui->execution);
    CHECK(history);
    CHECK(nes_rewind_configure(&ui->execution->rewind, 4, NES_REWIND_DEFAULT_MEMORY_LIMIT));
    NesStateResult result;
    for (unsigned i = 0; i < 4; ++i) {
        ram[7] = (uint8_t)i;
        for (unsigned p = 0; p < 256 * 240; ++p) framebuffer[p] = 0xff000000u | ((p + i * 32) & 255) << 8;
        CHECK(nes_rewind_capture(&ui->execution->rewind, &result) == NES_REPLAY_OK);
        nes_rewind_complete(&ui->execution->rewind, true);
    }
    ram[7] = 99;
    NesStateBlob live = {0}, after = {0};
    CHECK(nes_state_capture(&live) == NES_STATE_OK);
    FrontendDesktopUi *model = cupid_gtk_open(ui, 1, HISTORY_PANEL);
    CHECK(model);
    CupidGtkTool *tool = ui->gtk->tools;
    while (tool && &tool->ui != model) tool = tool->next;
    CHECK(tool);
    pump(ui);
    FrontendHistoryState state;
    CHECK(frontend_history_state(history, &state) && state.open && state.index == 0);
    CHECK(frontend_execution_paused(ui->execution) && ram[7] == 99);
    GtkWidget *timeline = find(tool->content, GTK_TYPE_SCALE, NULL);
    CHECK(timeline);
    gtk_range_set_value(GTK_RANGE(timeline), 2);
    pump(ui);
    CHECK(frontend_history_state(history, &state) && state.index == 2 && ram[7] == 99);
    GtkWidget *picture = find(tool->content, GTK_TYPE_PICTURE, NULL);
    CHECK(picture);
    GdkPaintable *image = gdk_paintable_get_current_image(gtk_picture_get_paintable(GTK_PICTURE(picture)));
    CHECK(GDK_IS_TEXTURE(image));
    uint32_t *pixels = g_new(uint32_t, state.width * 240);
    gdk_texture_download(GDK_TEXTURE(image), (guchar *)pixels, state.width * sizeof(uint32_t));
    CHECK(!memcmp(pixels, state.pixels, state.width * 240 * sizeof(uint32_t)));
    g_free(pixels);
    g_object_unref(image);
    GtkWidget *set = find(tool->content, GTK_TYPE_BUTTON, "Set bookmark");
    CHECK(set);
    g_signal_emit_by_name(set, "clicked");
    CHECK(frontend_history_state(history, &state) && state.bookmarks[0] == 2);
    CHECK(frontend_history_action(history, HISTORY_FIRST, NULL, 0, NULL, 0));
    CHECK(frontend_history_action(history, HISTORY_BOOKMARK_JUMP, NULL, 0, NULL, 0));
    CHECK(frontend_history_state(history, &state) && state.index == 2);
    CHECK(frontend_history_action(history, HISTORY_VOLUME, NULL, 0, NULL, 0));
    GtkWidget *play = find(tool->content, GTK_TYPE_BUTTON, "Play");
    CHECK(play);
    g_signal_emit_by_name(play, "clicked");
    for (unsigned i = 0; i < 15; ++i) {
        frontend_history_tick(history);
        frontend_desktop_render(ui, 256, 240, "History fixture", "NTSC", "Paused");
        SDL_Delay(10);
    }
    CHECK(frontend_history_state(history, &state) && !state.playing && state.index == 3);
    CHECK(nes_state_capture(&after) == NES_STATE_OK && live.size == after.size && !memcmp(live.data, after.data, live.size));
    nes_state_blob_free(&after);
    GtkWidget *toggle = find(tool->content, GTK_TYPE_TOGGLE_BUTTON, "Bookmarks");
    CHECK(toggle);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(toggle), TRUE);
    pump(ui);
    /* Save the actual native window, including the frame and bookmark preview. */
    GdkPaintable *paint = gtk_widget_paintable_new(tool->window);
    GtkSnapshot *snapshot = gtk_snapshot_new();
    gdk_paintable_snapshot(paint, GDK_SNAPSHOT(snapshot), gtk_widget_get_width(tool->window), gtk_widget_get_height(tool->window));
    GskRenderNode *node = gtk_snapshot_free_to_node(snapshot);
    CHECK(node);
    GskRenderer *renderer = gtk_native_get_renderer(GTK_NATIVE(tool->window));
    GdkTexture *texture = gsk_renderer_render_texture(renderer, node, NULL);
    char path[4096];
    g_snprintf(path, sizeof(path), "%s/history.png", out);
    CHECK(texture && gdk_texture_save_to_png(texture, path));
    g_object_unref(texture);
    gsk_render_node_unref(node);
    g_object_unref(paint);
    gboolean handled = FALSE;
    g_signal_emit_by_name(tool->window, "close-request", &handled);
    CHECK(frontend_history_state(history, &state) && !state.open);
    CHECK(!ui->execution->suspend_reasons && ram[7] == 99);
    CHECK(cupid_gtk_open(ui, 1, HISTORY_PANEL) == model);
    pump(ui);
    CHECK(frontend_history_state(history, &state) && state.open && state.index == 0);
    GtkWidget *resume = find(tool->content, GTK_TYPE_BUTTON, "Resume gameplay");
    CHECK(resume);
    g_signal_emit_by_name(resume, "clicked");
    CHECK(ram[7] == 0 && !ui->execution->rewind.count);
    CHECK(frontend_history_state(history, &state) && !state.open && !frontend_execution_paused(ui->execution));
    nes_state_blob_free(&live);
    /* The desktop outlives this controller; widget cleanup must tolerate it. */
    frontend_history_destroy(history);
    puts("GTK history slider/playback/bookmark/export controls/resume/lifecycle: PASS");
    return true;
}
