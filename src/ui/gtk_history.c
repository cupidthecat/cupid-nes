/*
 * gtk_history.c - Native history playback, exports and bookmark previews
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "gtk_internal.h"
#include "history_frontend.h"
#include "platform_frontend.h"
#include <math.h>

typedef struct {
    CupidGtkTool *tool;
    GtkWidget *root, *picture, *viewport, *play, *timeline, *time, *position, *range, *volume;
    GtkWidget *export, *controls, *bookmarks, *slot, *mark[HISTORY_BOOKMARKS], *thumb[HISTORY_BOOKMARKS];
    GtkNativeDialog *dialog;
    uint64_t revision, dialog_revision;
    unsigned file_action;
    size_t marks[HISTORY_BOOKMARKS];
    double pending_position;
    bool updating, opened;
} GtkHistory;

static FrontendHistory *controller(GtkHistory *h) {
    return h->tool->ui.execution ? h->tool->ui.execution->history_viewer : NULL;
}

static bool invoke(GtkHistory *h, unsigned id, const char *value, int selected) {
    char error[256] = {0};
    bool ok = frontend_history_action(controller(h), id, value, selected, error, sizeof(error));
    if (!ok && error[0]) cupid_gtk_tool_status(h->tool, error);
    return ok;
}

static void clicked(GtkButton *widget, gpointer data) {
    GtkHistory *h = data;
    invoke(h, GPOINTER_TO_UINT(g_object_get_data(G_OBJECT(widget), "history-action")), NULL, 0);
    cupid_gtk_history_refresh(h->root);
}

static GtkWidget *button(GtkHistory *h, GtkWidget *box, const char *text, unsigned id, const char *tip) {
    GtkWidget *widget = gtk_button_new_with_label(text);
    g_object_set_data(G_OBJECT(widget), "history-action", GUINT_TO_POINTER(id));
    g_signal_connect(widget, "clicked", G_CALLBACK(clicked), h);
    gtk_widget_set_tooltip_text(widget, tip);
    gtk_box_append(GTK_BOX(box), widget);
    return widget;
}

static void scrub(GtkRange *range, gpointer data) {
    GtkHistory *h = data;
    if (!h->updating) h->pending_position = gtk_range_get_value(range);
}

static void go_to(GtkSpinButton *spin, gpointer data) {
    GtkHistory *h = data;
    if (!h->updating) h->pending_position = gtk_spin_button_get_value(spin) - 1;
}

static void volume(GtkRange *range, gpointer data) {
    GtkHistory *h = data;
    if (!h->updating) invoke(h, HISTORY_VOLUME, NULL, (int)gtk_range_get_value(range));
}

static void bookmark_jump(GtkButton *widget, gpointer data) {
    GtkHistory *h = data;
    invoke(h, HISTORY_BOOKMARK_JUMP, NULL, GPOINTER_TO_INT(g_object_get_data(G_OBJECT(widget), "slot")));
    cupid_gtk_history_refresh(h->root);
}

static void bookmark_set(GtkButton *widget, gpointer data) {
    (void)widget;
    GtkHistory *h = data;
    invoke(h, HISTORY_BOOKMARK_SET, NULL, (int)gtk_drop_down_get_selected(GTK_DROP_DOWN(h->slot)));
    cupid_gtk_history_refresh(h->root);
}

static void show_bookmarks(GtkToggleButton *toggle, gpointer data) {
    GtkHistory *h = data;
    gtk_widget_set_visible(h->bookmarks, gtk_toggle_button_get_active(toggle));
}

static void file_response(GtkNativeDialog *dialog, int response, gpointer data) {
    GtkHistory *h = data;
    h->dialog = NULL;
    FrontendHistoryState state;
    if (response == GTK_RESPONSE_ACCEPT && frontend_history_state(controller(h), &state) && state.open) {
        GFile *file = gtk_file_chooser_get_file(GTK_FILE_CHOOSER(dialog));
        char *path = file ? g_file_get_path(file) : NULL;
        if (path && state.revision == h->dialog_revision) invoke(h, h->file_action, path, 0);
        else cupid_gtk_tool_status(h->tool, "History changed; choose the frame and export again");
        g_free(path);
        g_clear_object(&file);
    }
    gtk_native_dialog_destroy(dialog);
    g_object_unref(dialog);
}

static void save_file(GtkButton *widget, gpointer data) {
    GtkHistory *h = data;
    if (h->dialog) { gtk_native_dialog_show(h->dialog); return; }
    FrontendHistoryState state;
    if (!frontend_history_state(controller(h), &state) || !state.open) return;
    if (state.playing) invoke(h, HISTORY_PLAY, NULL, 0);
    h->dialog_revision = state.revision;
    h->file_action = GPOINTER_TO_UINT(g_object_get_data(G_OBJECT(widget), "history-action"));
    unsigned type = h->file_action == HISTORY_SAVE_STATE ? FRONTEND_SAVE_STATE : FRONTEND_SAVE_MOVIE;
    const FrontendFileDialogInfo *info = frontend_file_dialog_info(true, type);
    GtkFileChooserNative *dialog = gtk_file_chooser_native_new(info->title, GTK_WINDOW(h->tool->window),
        GTK_FILE_CHOOSER_ACTION_SAVE, "Save", "Cancel");
    gtk_native_dialog_set_modal(GTK_NATIVE_DIALOG(dialog), TRUE);
    if (type == FRONTEND_SAVE_STATE) cupid_gtk_file_filters(GTK_FILE_CHOOSER(dialog), true, type);
    else {
        GtkFileFilter *filter = gtk_file_filter_new();
        gtk_file_filter_set_name(filter, "Cupid input movie (*.movie, *.cmv)");
        gtk_file_filter_add_pattern(filter, "*.movie");
        gtk_file_filter_add_pattern(filter, "*.cmv");
        gtk_file_chooser_add_filter(GTK_FILE_CHOOSER(dialog), filter);
        g_object_unref(filter);
    }
    gtk_file_chooser_set_current_name(GTK_FILE_CHOOSER(dialog), type == FRONTEND_SAVE_STATE ? "history.state" : "history.movie");
    h->dialog = GTK_NATIVE_DIALOG(dialog);
    g_signal_connect(dialog, "response", G_CALLBACK(file_response), h);
    gtk_native_dialog_show(h->dialog);
}

static void scale_changed(GObject *object, GParamSpec *spec, gpointer data) {
    (void)spec;
    GtkHistory *h = data;
    unsigned scale = gtk_drop_down_get_selected(GTK_DROP_DOWN(object)) + 1;
    FrontendHistoryState state;
    if (scale <= 10 && frontend_history_state(controller(h), &state))
        gtk_window_set_default_size(GTK_WINDOW(h->tool->window), MAX(640, (int)(state.width * scale)), 240 * scale + 170);
}

static gboolean key(GtkEventControllerKey *event, guint keyval, guint code, GdkModifierType mods, gpointer data) {
    (void)event; (void)code;
    GtkHistory *h = data;
    unsigned id = keyval == GDK_KEY_space ? HISTORY_PLAY : keyval == GDK_KEY_Left ? HISTORY_PREVIOUS :
        keyval == GDK_KEY_Right ? HISTORY_NEXT : keyval == GDK_KEY_Home ? HISTORY_FIRST :
        keyval == GDK_KEY_End ? HISTORY_LAST : keyval == GDK_KEY_Escape ? HISTORY_CLOSE : 0;
    if (mods & (GDK_CONTROL_MASK | GDK_ALT_MASK)) id = 0;
    if (GTK_IS_EDITABLE(gtk_window_get_focus(GTK_WINDOW(h->tool->window)))) return FALSE;
    if (keyval >= GDK_KEY_0 && keyval <= GDK_KEY_9 && !(mods & GDK_ALT_MASK)) {
        invoke(h, mods & GDK_CONTROL_MASK ? HISTORY_BOOKMARK_SET : HISTORY_BOOKMARK_JUMP, NULL, keyval - GDK_KEY_0);
    } else if (id) invoke(h, id, NULL, 0);
    else return FALSE;
    cupid_gtk_history_refresh(h->root);
    return TRUE;
}

static void video_click(GtkGestureClick *gesture, int count, double x, double y, gpointer data) {
    (void)gesture; (void)count; (void)x; (void)y;
    GtkHistory *h = data;
    invoke(h, HISTORY_PLAY, NULL, 0);
}

static void cleanup(gpointer data) {
    GtkHistory *h = data;
    if (h->dialog) {
        g_signal_handlers_disconnect_by_data(h->dialog, h);
        gtk_native_dialog_destroy(h->dialog);
        g_object_unref(h->dialog);
    }
    frontend_history_close(controller(h));
    g_free(h);
}

void cupid_gtk_history_open(GtkWidget *widget) {
    GtkHistory *h = g_object_get_data(G_OBJECT(widget), "history");
    if (!h) return;
    h->opened = false;
    h->pending_position = -1;
    for (unsigned i = 0; i < HISTORY_BOOKMARKS; ++i) {
        h->marks[i] = SIZE_MAX;
        gtk_picture_set_paintable(GTK_PICTURE(h->thumb[i]), NULL);
    }
    invoke(h, HISTORY_OPEN, NULL, 0);
    cupid_gtk_history_refresh(widget);
}

GtkWidget *cupid_gtk_history_new(CupidGtkTool *tool) {
    GtkHistory *h = g_new0(GtkHistory, 1);
    h->tool = tool;
    h->pending_position = -1;
    h->revision = UINT64_MAX;
    h->root = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_widget_add_css_class(h->root, "history-viewer");
    g_object_set_data_full(G_OBJECT(h->root), "history", h, cleanup);
    frontend_history_use_native(controller(h));
    GtkWidget *menu = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    cupid_gtk_margins(menu, 6);
    gtk_box_append(GTK_BOX(h->root), menu);
    GtkWidget *file = gtk_menu_button_new();
    gtk_menu_button_set_label(GTK_MENU_BUTTON(file), "File");
    GtkWidget *popover = gtk_popover_new(), *items = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    cupid_gtk_margins(items, 6);
    gtk_popover_set_child(GTK_POPOVER(popover), items);
    gtk_menu_button_set_popover(GTK_MENU_BUTTON(file), popover);
    gtk_box_append(GTK_BOX(menu), file);
    GtkWidget *save = button(h, items, "Create save state…", HISTORY_SAVE_STATE, "Save the selected frame");
    h->export = button(h, items, "Export movie range…", HISTORY_EXPORT, "Export the marked range with its starting state");
    g_signal_handlers_disconnect_by_func(save, G_CALLBACK(clicked), h);
    g_signal_handlers_disconnect_by_func(h->export, G_CALLBACK(clicked), h);
    g_signal_connect(save, "clicked", G_CALLBACK(save_file), h);
    g_signal_connect(h->export, "clicked", G_CALLBACK(save_file), h);
    button(h, items, "Resume gameplay", HISTORY_RESUME, "Continue from this frame and discard later history");
    button(h, items, "Close", HISTORY_CLOSE, "Return to the unchanged live position");
    button(h, menu, "Open history", HISTORY_OPEN, "Open retained rewind history");
    button(h, menu, "Resume gameplay", HISTORY_RESUME, "Continue playing from the selected frame");
    const char *scales[] = {"1×", "2×", "3×", "4×", "5×", "6×", "7×", "8×", "9×", "10×", NULL};
    GtkWidget *scale = gtk_drop_down_new_from_strings(scales);
    gtk_drop_down_set_selected(GTK_DROP_DOWN(scale), 1);
    g_signal_connect(scale, "notify::selected", G_CALLBACK(scale_changed), h);
    gtk_widget_set_tooltip_text(scale, "Video scale");
    gtk_box_append(GTK_BOX(menu), scale);
    GtkWidget *show = gtk_toggle_button_new_with_label("Bookmarks");
    gtk_box_append(GTK_BOX(menu), show);
    g_signal_connect(show, "toggled", G_CALLBACK(show_bookmarks), h);
    GtkWidget *body = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_box_append(GTK_BOX(h->root), body);
    gtk_widget_set_vexpand(body, TRUE);
    h->picture = gtk_picture_new();
    gtk_widget_add_css_class(h->picture, "history-video");
    gtk_picture_set_content_fit(GTK_PICTURE(h->picture), GTK_CONTENT_FIT_FILL);
    gtk_picture_set_can_shrink(GTK_PICTURE(h->picture), TRUE);
    gtk_widget_set_hexpand(h->picture, TRUE);
    gtk_widget_set_vexpand(h->picture, TRUE);
    h->viewport = gtk_aspect_frame_new(.5f, .5f, 256.f / 240, FALSE);
    gtk_widget_set_hexpand(h->viewport, TRUE);
    gtk_widget_set_vexpand(h->viewport, TRUE);
    gtk_aspect_frame_set_child(GTK_ASPECT_FRAME(h->viewport), h->picture);
    gtk_box_append(GTK_BOX(body), h->viewport);
    GtkGesture *click = gtk_gesture_click_new();
    gtk_gesture_single_set_button(GTK_GESTURE_SINGLE(click), 1);
    g_signal_connect(click, "released", G_CALLBACK(video_click), h);
    gtk_widget_add_controller(h->picture, GTK_EVENT_CONTROLLER(click));
    GtkWidget *marks = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    h->bookmarks = cupid_gtk_scroll(marks);
    gtk_widget_set_size_request(h->bookmarks, 180, -1);
    gtk_box_append(GTK_BOX(body), h->bookmarks);
    gtk_widget_set_visible(h->bookmarks, FALSE);
    const char *slots[] = {"Slot 0", "Slot 1", "Slot 2", "Slot 3", "Slot 4", "Slot 5", "Slot 6", "Slot 7", "Slot 8", "Slot 9", NULL};
    h->slot = gtk_drop_down_new_from_strings(slots);
    gtk_box_append(GTK_BOX(marks), h->slot);
    GtkWidget *set = gtk_button_new_with_label("Set bookmark");
    gtk_widget_set_tooltip_text(set, "Ctrl+0–9 stores a bookmark; 0–9 jumps to it");
    g_signal_connect(set, "clicked", G_CALLBACK(bookmark_set), h);
    gtk_box_append(GTK_BOX(marks), set);
    for (unsigned i = 0; i < HISTORY_BOOKMARKS; ++i) {
        GtkWidget *jump = gtk_button_new(), *box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
        h->thumb[i] = gtk_picture_new();
        gtk_widget_set_size_request(h->thumb[i], 128, 60);
        h->mark[i] = cupid_gtk_label("");
        gtk_box_append(GTK_BOX(box), h->thumb[i]);
        gtk_box_append(GTK_BOX(box), h->mark[i]);
        gtk_button_set_child(GTK_BUTTON(jump), box);
        g_object_set_data(G_OBJECT(jump), "slot", GUINT_TO_POINTER(i));
        g_signal_connect(jump, "clicked", G_CALLBACK(bookmark_jump), h);
        gtk_box_append(GTK_BOX(marks), jump);
        h->marks[i] = SIZE_MAX;
    }
    h->controls = gtk_box_new(GTK_ORIENTATION_VERTICAL, 4);
    cupid_gtk_margins(h->controls, 6);
    gtk_box_append(GTK_BOX(h->root), h->controls);
    h->timeline = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0, 1, 1);
    gtk_scale_set_draw_value(GTK_SCALE(h->timeline), FALSE);
    gtk_widget_set_tooltip_text(h->timeline, "Scrub history; Home/End jump to its boundaries");
    g_signal_connect(h->timeline, "value-changed", G_CALLBACK(scrub), h);
    gtk_box_append(GTK_BOX(h->controls), h->timeline);
    GtkWidget *bar = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);
    gtk_box_append(GTK_BOX(h->controls), bar);
    button(h, bar, "|◀", HISTORY_FIRST, "First frame (Home)");
    button(h, bar, "◀", HISTORY_PREVIOUS, "Previous frame (Left)");
    h->play = button(h, bar, "Play", HISTORY_PLAY, "Play / pause (Space or click video)");
    button(h, bar, "▶", HISTORY_NEXT, "Next frame (Right)");
    button(h, bar, "▶|", HISTORY_LAST, "Last frame (End)");
    h->time = cupid_gtk_label("");
    gtk_widget_set_hexpand(h->time, TRUE);
    gtk_box_append(GTK_BOX(bar), h->time);
    h->position = gtk_spin_button_new_with_range(1, 1, 1);
    gtk_widget_set_tooltip_text(h->position, "Frame number, starting at 1");
    g_signal_connect(h->position, "value-changed", G_CALLBACK(go_to), h);
    gtk_box_append(GTK_BOX(bar), h->position);
    gtk_box_append(GTK_BOX(bar), cupid_gtk_label("Volume"));
    h->volume = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, 0, 100, 1);
    gtk_widget_set_size_request(h->volume, 100, -1);
    gtk_widget_set_tooltip_text(h->volume, "History volume");
    gtk_scale_set_draw_value(GTK_SCALE(h->volume), FALSE);
    g_signal_connect(h->volume, "value-changed", G_CALLBACK(volume), h);
    gtk_box_append(GTK_BOX(bar), h->volume);
    GtkWidget *range = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_box_append(GTK_BOX(h->controls), range);
    button(h, range, "Set start", HISTORY_RANGE_FIRST, "Start the movie export range at this frame");
    button(h, range, "Set end", HISTORY_RANGE_LAST, "End the movie export range at this frame (inclusive)");
    h->range = cupid_gtk_label("");
    gtk_box_append(GTK_BOX(range), h->range);
    GtkEventController *keys = gtk_event_controller_key_new();
    gtk_event_controller_set_propagation_phase(keys, GTK_PHASE_CAPTURE);
    g_signal_connect(keys, "key-pressed", G_CALLBACK(key), h);
    gtk_widget_add_controller(h->root, keys);
    cupid_gtk_history_open(h->root);
    return h->root;
}

void cupid_gtk_history_refresh(GtkWidget *widget) {
    GtkHistory *h = g_object_get_data(G_OBJECT(widget), "history");
    if (!h) return;
    if (h->pending_position >= 0) {
        char number[32];
        g_snprintf(number, sizeof(number), "%.0f", h->pending_position + 1);
        h->pending_position = -1;
        invoke(h, HISTORY_POSITION, number, 0);
    }
    FrontendHistoryState state;
    if (!frontend_history_state(controller(h), &state)) return;
    if (!state.open && h->opened) {
        h->opened = false;
        gtk_widget_set_visible(h->tool->window, FALSE);
        return;
    }
    h->opened = state.open;
    gtk_widget_set_sensitive(h->controls, state.open);
    gtk_widget_set_sensitive(h->export, state.can_export);
    if (state.status) gtk_label_set_text(GTK_LABEL(h->tool->status), state.status);
    if (!state.open) return;
    h->updating = true;
    float aspect = h->tool->ui.settings && h->tool->ui.settings->aspect_mode == FRONTEND_ASPECT_4_3
        ? (4.f / 3) * (state.width / 256) : (float)state.width / 240;
    gtk_aspect_frame_set_ratio(GTK_ASPECT_FRAME(h->viewport), aspect);
    gtk_range_set_range(GTK_RANGE(h->timeline), 0, MAX(1., (double)state.count - 1));
    gtk_range_set_value(GTK_RANGE(h->timeline), state.index);
    gtk_widget_set_sensitive(h->timeline, state.count > 1);
    gtk_spin_button_set_range(GTK_SPIN_BUTTON(h->position), 1, state.count);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(h->position), state.index + 1);
    gtk_range_set_value(GTK_RANGE(h->volume), state.volume);
    gtk_button_set_label(GTK_BUTTON(h->play), state.playing ? "Pause" : "Play");
    unsigned ms = (unsigned)llround(state.index * 1000. / state.fps);
    unsigned total = (unsigned)llround((state.count - 1) * 1000. / state.fps);
    char text[128];
    g_snprintf(text, sizeof(text), "%02u:%02u.%03u / %02u:%02u.%03u", ms / 60000, ms / 1000 % 60, ms % 1000,
        total / 60000, total / 1000 % 60, total % 1000);
    gtk_label_set_text(GTK_LABEL(h->time), text);
    g_snprintf(text, sizeof(text), "Export frames %zu–%zu%s", state.range_first + 1, state.range_last + 1,
        state.can_export ? "" : " (input log incomplete)");
    gtk_label_set_text(GTK_LABEL(h->range), text);
    if (h->revision != state.revision) {
        cupid_gtk_picture(h->picture, state.pixels, state.width, 240, state.width * sizeof(uint32_t));
        h->revision = state.revision;
    }
    for (unsigned i = 0; i < HISTORY_BOOKMARKS; ++i) {
        bool occupied = state.bookmarks[i] != SIZE_MAX;
        if (occupied) g_snprintf(text, sizeof(text), "%u · Frame %zu", i, state.bookmarks[i] + 1);
        else g_snprintf(text, sizeof(text), "%u · Empty", i);
        gtk_label_set_text(GTK_LABEL(h->mark[i]), text);
        if (occupied && h->marks[i] != state.bookmarks[i] && state.bookmarks[i] == state.index)
            cupid_gtk_picture(h->thumb[i], state.pixels, state.width, 240, state.width * sizeof(uint32_t));
        h->marks[i] = state.bookmarks[i];
    }
    h->updating = false;
}
