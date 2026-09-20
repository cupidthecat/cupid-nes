/*
 * platform_frontend.c - Desktop platform entry and file dialogs
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef _WIN32
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#endif
#include "platform_frontend.h"
#include "../util/file_io.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void set_error(char *error, size_t error_size, const char *message) {
    if (error && error_size) snprintf(error, error_size, "%s", message);
}

bool frontend_parse_dialog_output(const char *output, size_t output_size,
                                   char *path, size_t path_size,
                                   char *error, size_t error_size) {
    if (error && error_size) error[0] = '\0';
    if (!output || !path || path_size < 2 || output_size > NES_FILE_PATH_LIMIT + 2u) {
        set_error(error, error_size, "The selected path is too long or invalid");
        return false;
    }
    if (output_size && output[output_size - 1u] == '\n') {
        --output_size;
        if (output_size && output[output_size - 1u] == '\r') --output_size;
    }
    if (!output_size) return false;
    if (output_size >= path_size) {
        set_error(error, error_size, "The selected path is too long");
        return false;
    }
    for (size_t i = 0; i < output_size; ++i) {
        unsigned char value = (unsigned char)output[i];
        if (value < 0x20 || value == 0x7F) {
            set_error(error, error_size, "The file dialog returned an ambiguous path");
            return false;
        }
    }
    memmove(path, output, output_size);
    path[output_size] = '\0';
    return true;
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
    (void)argc;
    (void)argv;
    if (!entry) return 1;
    int wide_argc = 0;
    wchar_t **wide_argv = CommandLineToArgvW(GetCommandLineW(), &wide_argc);
    if (!wide_argv || wide_argc <= 0) {
        if (wide_argv) LocalFree(wide_argv);
        fprintf(stderr, "Could not read the Windows Unicode command line\n");
        return 1;
    }
    char **utf8_argv = (char **)calloc((size_t)wide_argc + 1, sizeof(*utf8_argv));
    if (!utf8_argv) {
        LocalFree(wide_argv);
        fprintf(stderr, "Could not allocate the Windows Unicode command line\n");
        return 1;
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
    if (!valid) fprintf(stderr, "Could not convert the Windows command line to UTF-8\n");
    int result = valid ? entry(wide_argc, utf8_argv) : 1;
    for (int i = 0; i < wide_argc; ++i) free(utf8_argv[i]);
    free(utf8_argv);
    return result;
}

static bool native_file_dialog(unsigned kind, char *path, size_t path_size,
                                 char *error, size_t error_size) {
    if (!path || path_size < 2) return false;
    wchar_t selected[32768] = {0};
    static const wchar_t image_filters[] =
        L"Supported images\0*.nes;*.unf;*.unif;*.fds;*.qd;*.nsf;*.nsfe;*.stbx;*.bin;*.zip;*.7z\0"
        L"All files\0*.*\0\0";
    static const wchar_t png_filters[] = L"PNG image\0*.png\0All files\0*.*\0\0";
    static const wchar_t wav_filters[] = L"PCM wave audio\0*.wav\0All files\0*.*\0\0";
    static const wchar_t avi_filters[] = L"AVI video\0*.avi\0All files\0*.*\0\0";
    const wchar_t *const filters[] = {image_filters, png_filters, wav_filters, avi_filters};
    const wchar_t *const extensions[] = {NULL, L"png", L"wav", L"avi"};
    if (kind >= sizeof(filters) / sizeof(filters[0])) return false;
    OPENFILENAMEW dialog;
    memset(&dialog, 0, sizeof(dialog));
    dialog.lStructSize = sizeof(dialog);
    dialog.lpstrFilter = filters[kind];
    dialog.lpstrDefExt = extensions[kind];
    dialog.lpstrFile = selected;
    dialog.nMaxFile = (DWORD)(sizeof(selected) / sizeof(selected[0]));
    dialog.Flags = OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR
        | (kind ? OFN_OVERWRITEPROMPT : OFN_FILEMUSTEXIST);
    BOOL accepted = kind ? GetSaveFileNameW(&dialog) : GetOpenFileNameW(&dialog);
    if (!accepted) {
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
    bool valid = frontend_parse_dialog_output(utf8, strlen(utf8), path, path_size, error, error_size);
    free(utf8);
    return valid;
}

#else

#include <sys/wait.h>

int frontend_application_entry_utf8(int argc, char **argv,
                                    FrontendApplicationEntry entry) {
    return entry ? entry(argc, argv) : 1;
}

static bool native_file_dialog(unsigned kind, char *path, size_t path_size,
                                 char *error, size_t error_size) {
    static const char *const commands[] = {
        "if command -v zenity >/dev/null 2>&1; then exec zenity --file-selection --title='Open Game'; "
        "elif command -v kdialog >/dev/null 2>&1; then exec kdialog --getopenfilename .; else exit 127; fi",
        "if command -v zenity >/dev/null 2>&1; then exec zenity --file-selection --save --confirm-overwrite "
        "--title='Save Screenshot' --file-filter='PNG images | *.png'; "
        "elif command -v kdialog >/dev/null 2>&1; then exec kdialog --getsavefilename . '*.png|PNG images'; else exit 127; fi",
        "if command -v zenity >/dev/null 2>&1; then exec zenity --file-selection --save --confirm-overwrite "
        "--title='Record Audio' --file-filter='Wave audio | *.wav'; "
        "elif command -v kdialog >/dev/null 2>&1; then exec kdialog --getsavefilename . '*.wav|Wave audio'; else exit 127; fi",
        "if command -v zenity >/dev/null 2>&1; then exec zenity --file-selection --save --confirm-overwrite "
        "--title='Record Video' --file-filter='AVI video | *.avi'; "
        "elif command -v kdialog >/dev/null 2>&1; then exec kdialog --getsavefilename . '*.avi|AVI video'; else exit 127; fi"
    };
    if (error && error_size) error[0] = '\0';
    if (!path || path_size < 2 || path_size > NES_FILE_PATH_LIMIT
        || kind >= sizeof(commands) / sizeof(commands[0])) return false;
    char *selection = malloc(path_size + 2u);
    if (!selection) {
        set_error(error, error_size, "Could not allocate the selected path");
        return false;
    }
    FILE *pipe = popen(commands[kind], "r");
    if (!pipe) {
        free(selection);
        set_error(error, error_size, "No desktop file dialog is available");
        return false;
    }
    size_t count = 0;
    bool truncated = false;
    int next;
    while ((next = fgetc(pipe)) != EOF) {
        if (count < path_size + 2u) selection[count++] = (char)next;
        else truncated = true;
    }
    bool read_error = ferror(pipe) != 0;
    int status = pclose(pipe);
    bool accepted = false;
    if (read_error || status == -1 || !WIFEXITED(status)) {
        set_error(error, error_size, "The desktop file dialog failed");
    } else if (WEXITSTATUS(status) == 127) {
        set_error(error, error_size, "Install zenity or kdialog, or enter a path in the settings panel");
    } else if (WEXITSTATUS(status) != 0) {
        if (WEXITSTATUS(status) != 1 || count)
            set_error(error, error_size, "The desktop file dialog could not select a path");
    } else if (truncated) {
        set_error(error, error_size, "The selected path is too long");
    } else {
        accepted = frontend_parse_dialog_output(selection, count, path, path_size, error, error_size);
    }
    free(selection);
    return accepted;
}

#endif

bool frontend_open_image_dialog(char *path, size_t path_size,
                                char *error, size_t error_size) {
    return native_file_dialog(0, path, path_size, error, error_size);
}

bool frontend_save_file_dialog(FrontendSaveFileType type, char *path, size_t path_size,
                                char *error, size_t error_size) {
    if ((unsigned)type > FRONTEND_SAVE_AVI) {
        set_error(error, error_size, "The requested output format is not supported");
        return false;
    }
    return native_file_dialog((unsigned)type + 1u, path, path_size, error, error_size);
}
