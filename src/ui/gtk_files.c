/*
 * gtk_files.c - Native synchronous desktop file chooser bridge
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "gtk_internal.h"
#include "platform_frontend.h"
#include <string.h>

GtkFileFilter *cupid_gtk_file_filter(bool save, unsigned type) {
    const FrontendFileDialogInfo *kind = frontend_file_dialog_info(save, type);
    if (!kind || kind->directory) {
        return NULL;
    }
    GtkFileFilter *filter = gtk_file_filter_new();
    gtk_file_filter_set_name(filter, kind->label);
    char **patterns = g_strsplit(kind->pattern, ";", -1);
    for (unsigned i = 0; patterns[i]; ++i) {
        if (g_str_has_prefix(patterns[i], "*.")) {
            gtk_file_filter_add_suffix(filter, patterns[i] + 2);
        } else {
            gtk_file_filter_add_pattern(filter, patterns[i]);
        }
    }
    g_strfreev(patterns);
    return filter;
}

void cupid_gtk_file_filters(GtkFileChooser *chooser, bool save, unsigned type) {
    const FrontendFileDialogInfo *kind = frontend_file_dialog_info(save, type);
    GtkFileFilter *filter = cupid_gtk_file_filter(save, type);
    if (!filter) {
        return;
    }
    gtk_file_chooser_add_filter(chooser, filter);
    g_object_unref(filter);
    filter = gtk_file_filter_new();
    gtk_file_filter_set_name(filter, "All files");
    gtk_file_filter_add_pattern(filter, "*");
    gtk_file_chooser_add_filter(chooser, filter);
    g_object_unref(filter);
    if (save) {
        char *name = g_strdup_printf("untitled.%s", kind->extension ? kind->extension : "bin");
        gtk_file_chooser_set_current_name(chooser, name);
        g_free(name);
    }
}

void cupid_gtk_file_location(GtkFileChooser *chooser, const char *path) {
    if (!path || !*path) {
        return;
    }
    char *absolute = g_canonicalize_filename(path, NULL);
    GFile *file = g_file_new_for_path(absolute);
    if (g_file_test(absolute, G_FILE_TEST_IS_DIR)) {
        gtk_file_chooser_set_current_folder(chooser, file, NULL);
    } else if (gtk_file_chooser_get_action(chooser) == GTK_FILE_CHOOSER_ACTION_SAVE) {
        GFile *folder = g_file_get_parent(file);
        char *name = g_file_get_basename(file);
        if (folder) {
            gtk_file_chooser_set_current_folder(chooser, folder, NULL);
        }
        gtk_file_chooser_set_current_name(chooser, name);
        g_free(name);
        g_clear_object(&folder);
    } else {
        gtk_file_chooser_set_file(chooser, file, NULL);
    }
    g_object_unref(file);
    g_free(absolute);
}

typedef struct {
    GtkNativeDialog *dialog;
    GMainLoop *loop;
    int response;
    bool finished;
} FileRequest;

static GtkWindow *chooser_parent;
static FileRequest *active_request;

static void finish_request(FileRequest *request, int response) {
    request->response = response;
    request->finished = true;
    g_main_loop_quit(request->loop);
}

static void chooser_response(GtkNativeDialog *dialog, int response, gpointer data) {
    (void)dialog;
    finish_request(data, response);
}

static void chooser_error(char *error, size_t size, const char *message) {
    if (error && size) {
        g_strlcpy(error, message, size);
    }
}

static bool choose_file(bool save, unsigned type, char *path, size_t path_size, char *error, size_t error_size,
                        void *context) {
    (void)context;
    if (error && error_size) {
        error[0] = 0;
    }
    if (!chooser_parent || active_request || !path || path_size < 2) {
        chooser_error(error, error_size, "The file chooser is unavailable or already open");
        return false;
    }
    const FrontendFileDialogInfo *kind = frontend_file_dialog_info(save, type);
    if (!kind) {
        chooser_error(error, error_size, "Unknown file type");
        return false;
    }
    GtkFileChooserAction action = kind->directory ? GTK_FILE_CHOOSER_ACTION_SELECT_FOLDER
                                  : save          ? GTK_FILE_CHOOSER_ACTION_SAVE
                                                  : GTK_FILE_CHOOSER_ACTION_OPEN;
    GtkWindow *parent = g_object_ref(chooser_parent);
    GListModel *windows = gtk_window_get_toplevels();
    for (guint i = 0; i < g_list_model_get_n_items(windows); ++i) {
        GtkWindow *window = g_list_model_get_item(windows, i);
        if (gtk_widget_get_visible(GTK_WIDGET(window)) && gtk_window_is_active(window) &&
            gtk_widget_has_css_class(GTK_WIDGET(window), "cupid-desktop")) {
            g_set_object(&parent, window);
        }
        g_object_unref(window);
    }
    GtkFileChooserNative *native =
        gtk_file_chooser_native_new(kind->title, parent, action, save ? "Save" : "Open", "Cancel");
    GtkFileChooser *chooser = GTK_FILE_CHOOSER(native);
    gtk_native_dialog_set_modal(GTK_NATIVE_DIALOG(native), TRUE);
    cupid_gtk_file_filters(chooser, save, type);
    cupid_gtk_file_location(chooser, path);
    FileRequest request = {GTK_NATIVE_DIALOG(native), g_main_loop_new(NULL, FALSE), GTK_RESPONSE_CANCEL, false};
    active_request = &request;
    /* Also permits a headless smoke test to cancel the real nested dialog. */
    g_object_set_data(G_OBJECT(parent), "cupid-file-dialog", native);
    g_signal_connect(native, "response", G_CALLBACK(chooser_response), &request);
    gtk_native_dialog_show(request.dialog);
    /* Only GTK events run here; the caller's emulation loop stays suspended. */
    if (!request.finished) {
        g_main_loop_run(request.loop);
    }
    bool accepted = false;
    if (request.response == GTK_RESPONSE_ACCEPT) {
        GFile *file = gtk_file_chooser_get_file(chooser);
        char *selected = file ? g_file_get_path(file) : NULL;
        if (!selected) {
            chooser_error(error, error_size, "Choose a local file or folder");
        } else {
            accepted = frontend_parse_dialog_output(selected, strlen(selected), path, path_size, error, error_size);
        }
        g_free(selected);
        g_clear_object(&file);
    }
    g_object_set_data(G_OBJECT(parent), "cupid-file-dialog", NULL);
    g_signal_handlers_disconnect_by_data(native, &request);
    gtk_native_dialog_destroy(request.dialog);
    g_object_unref(native);
    g_main_loop_unref(request.loop);
    active_request = NULL;
    g_object_unref(parent);
    return accepted;
}

void cupid_gtk_files_shutdown(void) {
    frontend_set_file_chooser(NULL, NULL);
    if (active_request) {
        gtk_native_dialog_hide(active_request->dialog);
        finish_request(active_request, GTK_RESPONSE_CANCEL);
    }
    if (chooser_parent) {
        g_object_remove_weak_pointer(G_OBJECT(chooser_parent), (gpointer *)&chooser_parent);
        chooser_parent = NULL;
    }
}

void cupid_gtk_files_install(GtkWindow *parent) {
    cupid_gtk_files_shutdown();
    if (!parent) {
        return;
    }
    chooser_parent = parent;
    g_object_add_weak_pointer(G_OBJECT(parent), (gpointer *)&chooser_parent);
    frontend_set_file_chooser(choose_file, NULL);
}
