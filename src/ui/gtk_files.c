/*
 * gtk_files.c - Native synchronous desktop file chooser bridge
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "gtk_internal.h"
#include "platform_frontend.h"
#include <string.h>

typedef struct {
    const char *title, *label, *pattern, *extension;
    bool save, directory;
} FileKind;

static const FileKind open_dialogs[] = {
    {"Open Game", "Supported images", "*.nes;*.unf;*.unif;*.fds;*.qd;*.nsf;*.nsfe;*.stbx;*.bin;*.zip;*.7z", NULL, false,
     false},
    {"Play Input Movie", "Input movies and TAS projects", "*.cmv;*.movie;*.fm2;*.fm3;*.ctas", "fm2", false, false},
    {"Load State", "Save states", "*.cst;*.cstate;*.state", "cst", false, false},
    {"Load Palette", "Palette files", "*.pal", "pal", false, false},
    {"Apply Patch", "IPS, UPS or BPS patches", "*.ips;*.ups;*.bps", NULL, false, false},
    {"Load Tape", "Family BASIC tapes", "*.tap", "tap", false, false},
    {"Choose Firmware", "Firmware images", "*.bin;*.rom;*.bios", NULL, false, false},
    {"Choose Game Database", "Game databases", "*.txt;*.csv", "txt", false, false},
    {"Install HD Pack", "HD pack archives", "*.zip", "zip", false, false},
    {"Load Lua Script", "Lua scripts", "*.lua", "lua", false, false},
    {"Load Cheats", "Cheat files", "*.txt;*.cht", "txt", false, false},
    {"Choose a Folder", "Folders", "*", NULL, false, true},
    {"Import Memory", "Binary memory dumps", "*.bin;*.dump", "bin", false, false},
    {"Load Cheat Database", "Tab-separated cheat catalogs", "*.tsv;*.txt", "tsv", false, false},
    {"Convert Legacy Movie", "FCM input movies", "*.fcm", "fcm", false, false}};

static const FileKind save_dialogs[] = {
    {"Save Screenshot", "PNG images", "*.png", "png", true, false},
    {"Record Audio", "Wave audio", "*.wav", "wav", true, false},
    {"Record Video", "AVI video", "*.avi", "avi", true, false},
    {"Record Input Movie", "Input movies and TAS projects", "*.cmv;*.movie;*.fm2;*.fm3;*.ctas", "fm2", true, false},
    {"Save State", "Save states", "*.cst;*.cstate;*.state", "cst", true, false},
    {"Save Tape", "Family BASIC tapes", "*.tap", "tap", true, false},
    {"Export HD Pack", "HD pack archives", "*.zip", "zip", true, false},
    {"Save Cheats", "Cheat files", "*.txt;*.cht", "txt", true, false},
    {"Choose Disk Overlay", "Disk overlays", "*.ips", "ips", true, false},
    {"Save TAS Project", "Editable TAS projects", "*.ctas", "ctas", true, false},
    {"Export TAS Movie", "Input movies and projects", "*.fm2;*.fm3", "fm2", true, false},
    {"Export Table", "CSV tables", "*.csv", "csv", true, false},
    {"Export Memory", "Binary memory dumps", "*.bin;*.dump", "bin", true, false}};

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
    if (type >= (save ? G_N_ELEMENTS(save_dialogs) : G_N_ELEMENTS(open_dialogs))) {
        chooser_error(error, error_size, "Unknown file type");
        return false;
    }
    const FileKind *kind = save ? &save_dialogs[type] : &open_dialogs[type];
    GtkFileChooserAction action = kind->directory ? GTK_FILE_CHOOSER_ACTION_SELECT_FOLDER
                                  : save          ? GTK_FILE_CHOOSER_ACTION_SAVE
                                                  : GTK_FILE_CHOOSER_ACTION_OPEN;
    GtkFileChooserNative *native =
        gtk_file_chooser_native_new(kind->title, chooser_parent, action, save ? "Save" : "Open", "Cancel");
    GtkFileChooser *chooser = GTK_FILE_CHOOSER(native);
    gtk_native_dialog_set_modal(GTK_NATIVE_DIALOG(native), TRUE);
    if (!kind->directory) {
        GtkFileFilter *filter = gtk_file_filter_new();
        gtk_file_filter_set_name(filter, kind->label);
        char **patterns = g_strsplit(kind->pattern, ";", -1);
        for (unsigned i = 0; patterns[i]; ++i) {
            gtk_file_filter_add_pattern(filter, patterns[i]);
        }
        g_strfreev(patterns);
        gtk_file_chooser_add_filter(chooser, filter);
        g_object_unref(filter);
        filter = gtk_file_filter_new();
        gtk_file_filter_set_name(filter, "All files");
        gtk_file_filter_add_pattern(filter, "*");
        gtk_file_chooser_add_filter(chooser, filter);
        g_object_unref(filter);
    }
    if (save) {
        char *name = g_strdup_printf("untitled.%s", kind->extension ? kind->extension : "bin");
        gtk_file_chooser_set_current_name(chooser, name);
        g_free(name);
    }
    FileRequest request = {GTK_NATIVE_DIALOG(native), g_main_loop_new(NULL, FALSE), GTK_RESPONSE_CANCEL, false};
    active_request = &request;
    /* Also permits a headless smoke test to cancel the real nested dialog. */
    GtkWindow *parent = g_object_ref(chooser_parent);
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
