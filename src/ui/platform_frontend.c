/*
 * platform_frontend.c - Desktop platform entry and file dialogs
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "platform_frontend.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void set_error(char *error, size_t error_size, const char *message) {
    if (error && error_size) snprintf(error, error_size, "%s", message);
}

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commdlg.h>
#include <shellapi.h>

static char *wide_to_utf8(const wchar_t *text) {
    if (!text) return NULL;
    int bytes = WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, text, -1,
                                    NULL, 0, NULL, NULL);
    if (bytes <= 0) return NULL;
    char *utf8 = (char *)malloc((size_t)bytes);
    if (!utf8) return NULL;
    if (!WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, text, -1,
                             utf8, bytes, NULL, NULL)) {
        free(utf8);
        return NULL;
    }
    return utf8;
}

int frontend_application_entry_utf8(int argc, char **argv,
                                    FrontendApplicationEntry entry) {
    if (!entry) return 1;
    int wide_argc = 0;
    wchar_t **wide_argv = CommandLineToArgvW(GetCommandLineW(), &wide_argc);
    if (!wide_argv || wide_argc <= 0) {
        if (wide_argv) LocalFree(wide_argv);
        return entry(argc, argv);
    }
    char **utf8_argv = (char **)calloc((size_t)wide_argc + 1, sizeof(*utf8_argv));
    if (!utf8_argv) {
        LocalFree(wide_argv);
        return entry(argc, argv);
    }
    bool valid = true;
    for (int i = 0; i < wide_argc; ++i) {
        utf8_argv[i] = wide_to_utf8(wide_argv[i]);
        if (!utf8_argv[i]) {
            valid = false;
            break;
        }
    }
    LocalFree(wide_argv);
    int result = valid ? entry(wide_argc, utf8_argv) : entry(argc, argv);
    for (int i = 0; i < wide_argc; ++i) free(utf8_argv[i]);
    free(utf8_argv);
    return result;
}

bool frontend_open_image_dialog(char *path, size_t path_size,
                                char *error, size_t error_size) {
    if (!path || path_size < 2) return false;
    wchar_t selected[32768] = {0};
    static const wchar_t filters[] =
        L"Supported images\0*.nes;*.unf;*.unif;*.fds;*.nsf;*.nsfe;*.stbx;*.zip;*.7z\0"
        L"All files\0*.*\0\0";
    OPENFILENAMEW dialog;
    memset(&dialog, 0, sizeof(dialog));
    dialog.lStructSize = sizeof(dialog);
    dialog.lpstrFilter = filters;
    dialog.lpstrFile = selected;
    dialog.nMaxFile = (DWORD)(sizeof(selected) / sizeof(selected[0]));
    dialog.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    if (!GetOpenFileNameW(&dialog)) {
        DWORD code = CommDlgExtendedError();
        if (code) set_error(error, error_size, "The file dialog could not be opened");
        else if (error && error_size) error[0] = '\0';
        return false;
    }
    char *utf8 = wide_to_utf8(selected);
    if (!utf8) {
        set_error(error, error_size, "The selected path is not valid Unicode");
        return false;
    }
    size_t length = strlen(utf8);
    if (length >= path_size) {
        free(utf8);
        set_error(error, error_size, "The selected path is too long");
        return false;
    }
    memcpy(path, utf8, length + 1);
    free(utf8);
    if (error && error_size) error[0] = '\0';
    return true;
}

#else

int frontend_application_entry_utf8(int argc, char **argv,
                                    FrontendApplicationEntry entry) {
    return entry ? entry(argc, argv) : 1;
}

static bool read_dialog_output(FILE *pipe, char *path, size_t path_size) {
    if (!fgets(path, (int)path_size, pipe)) return false;
    size_t length = strlen(path);
    while (length && (path[length - 1] == '\n' || path[length - 1] == '\r'))
        path[--length] = '\0';
    return length != 0;
}

bool frontend_open_image_dialog(char *path, size_t path_size,
                                char *error, size_t error_size) {
    if (!path || path_size < 2) return false;
    path[0] = '\0';
    FILE *pipe = popen("zenity --file-selection --title='Open Game' 2>/dev/null", "r");
    if (!pipe) {
        set_error(error, error_size, "No desktop file dialog is available");
        return false;
    }
    bool selected = read_dialog_output(pipe, path, path_size);
    int status = pclose(pipe);
    if (!selected || status != 0) {
        path[0] = '\0';
        if (error && error_size) error[0] = '\0';
        return false;
    }
    if (error && error_size) error[0] = '\0';
    return true;
}

#endif
