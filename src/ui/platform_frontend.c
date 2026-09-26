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

typedef struct {
    const char *title;
    const char *label;
    const char *pattern;
    const char *extension;
    bool save;
    bool directory;
} FileDialog;

static const FileDialog open_dialogs[] = {
    {"Open Game", "Supported images", "*.nes;*.unf;*.unif;*.fds;*.qd;*.nsf;*.nsfe;*.stbx;*.bin;*.zip;*.7z", NULL, false, false},
    {"Play Input Movie", "Input movies and TAS projects", "*.cmv;*.movie;*.fm2;*.fm3;*.ctas", "fm2", false, false},
    {"Load State", "Save states", "*.cst;*.state", "cst", false, false},
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
    {"Convert Legacy Movie", "FCM input movies", "*.fcm", "fcm", false, false}
};

static const FileDialog save_dialogs[] = {
    {"Save Screenshot", "PNG images", "*.png", "png", true, false},
    {"Record Audio", "Wave audio", "*.wav", "wav", true, false},
    {"Record Video", "AVI video", "*.avi", "avi", true, false},
    {"Record Input Movie", "Input movies and TAS projects", "*.cmv;*.movie;*.fm2;*.fm3;*.ctas", "fm2", true, false},
    {"Save State", "Save states", "*.cst;*.state", "cst", true, false},
    {"Save Tape", "Family BASIC tapes", "*.tap", "tap", true, false},
    {"Export HD Pack", "HD pack archives", "*.zip", "zip", true, false},
    {"Save Cheats", "Cheat files", "*.txt;*.cht", "txt", true, false},
    {"Choose Disk Overlay", "Disk overlays", "*.ips", "ips", true, false},
    {"Save TAS Project", "Editable TAS projects", "*.ctas", "ctas", true, false},
    {"Export TAS Movie", "FCEUX movies and projects", "*.fm2;*.fm3", "fm2", true, false},
    {"Export Table", "CSV tables", "*.csv", "csv", true, false},
    {"Export Memory", "Binary memory dumps", "*.bin;*.dump", "bin", true, false}
};

static FrontendFileChooser file_chooser;
static void *file_chooser_context;
void frontend_set_file_chooser(FrontendFileChooser chooser, void *context) {
    file_chooser = chooser;
    file_chooser_context = context;
}


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
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0602
#endif
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commdlg.h>
#include <shellapi.h>
#include <shlobj.h>

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

static bool native_file_dialog(const FileDialog *kind, char *path, size_t path_size,
                                 char *error, size_t error_size) {
    if (error && error_size) error[0] = '\0';
    if (!kind || !path || path_size < 2) return false;
    wchar_t selected[32768] = {0};
    wchar_t title[128] = {0}, extension[16] = {0}, filters[512] = {0};
    if (!MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, kind->title, -1, title, 128)) return false;
    if (kind->directory) {
        HRESULT initialized = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED);
        BROWSEINFOW browse = {0};
        browse.lpszTitle = title;
        browse.ulFlags = BIF_RETURNONLYFSDIRS | BIF_EDITBOX;
        if (SUCCEEDED(initialized)) browse.ulFlags |= BIF_NEWDIALOGSTYLE;
        PIDLIST_ABSOLUTE item = SHBrowseForFolderW(&browse);
        bool selected_item = item != NULL;
        bool accepted = item && SHGetPathFromIDListEx(item, selected, 32768, GPFIDL_DEFAULT);
        if (item) CoTaskMemFree((void *)item);
        if (SUCCEEDED(initialized)) CoUninitialize();
        if (!accepted) {
            if (selected_item) set_error(error, error_size, "The selected folder has no usable local path");
            return false;
        }
    } else {
        int used = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, kind->label, -1, filters, 512);
        int pattern = used > 0
            ? MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, kind->pattern, -1, filters + used, 512 - used) : 0;
        if (!used || !pattern || used + pattern + 16 >= 512) return false;
        static const wchar_t all_files[] = L"All files\0*.*\0\0";
        memcpy(filters + used + pattern, all_files, sizeof(all_files));
        if (kind->extension
            && !MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, kind->extension, -1, extension, 16)) return false;
        OPENFILENAMEW dialog = {0};
        dialog.lStructSize = sizeof(dialog);
        dialog.lpstrTitle = title;
        dialog.lpstrFilter = filters;
        dialog.lpstrDefExt = kind->extension ? extension : NULL;
        dialog.lpstrFile = selected;
        dialog.nMaxFile = (DWORD)(sizeof(selected) / sizeof(selected[0]));
        dialog.Flags = OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR
            | (kind->save ? OFN_OVERWRITEPROMPT : OFN_FILEMUSTEXIST);
        BOOL accepted = kind->save ? GetSaveFileNameW(&dialog) : GetOpenFileNameW(&dialog);
        if (!accepted) {
            if (CommDlgExtendedError()) set_error(error, error_size, "The file dialog could not be opened");
            return false;
        }
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

static bool native_file_dialog(const FileDialog *kind, char *path, size_t path_size,
                                 char *error, size_t error_size) {
    if (error && error_size) error[0] = '\0';
    if (!kind || !path || path_size < 2 || path_size > NES_FILE_PATH_LIMIT) return false;
    /* Every interpolated field belongs to the constant descriptor tables above.
     * A user's path is returned on stdout and never enters this shell program. */
    char patterns[256], command[2048];
    snprintf(patterns, sizeof(patterns), "%s", kind->pattern);
    for (char *p = patterns; *p; ++p) if (*p == ';') *p = ' ';
    int length;
    if (kind->directory) {
        length = snprintf(command, sizeof(command),
            "if command -v zenity >/dev/null 2>&1; then exec zenity --file-selection --directory --title='%s'; "
            "elif command -v kdialog >/dev/null 2>&1; then exec kdialog --getexistingdirectory . --title '%s'; else exit 127; fi",
            kind->title, kind->title);
    } else {
        length = snprintf(command, sizeof(command),
            "if command -v zenity >/dev/null 2>&1; then exec zenity --file-selection %s "
            "--title='%s' --file-filter='%s | %s' --file-filter='All files | *'; "
            "elif command -v kdialog >/dev/null 2>&1; then exec kdialog %s . '%s|%s' --title '%s'; else exit 127; fi",
            kind->save ? "--save --confirm-overwrite" : "", kind->title, kind->label, patterns,
            kind->save ? "--getsavefilename" : "--getopenfilename", patterns, kind->label, kind->title);
    }
    if (length < 0 || (size_t)length >= sizeof(command)) return false;
    char *selection = malloc(path_size + 2u);
    if (!selection) {
        set_error(error, error_size, "Could not allocate the selected path");
        return false;
    }
    FILE *pipe = popen(command, "r");
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
    return frontend_open_file_dialog(FRONTEND_OPEN_IMAGE, path, path_size, error, error_size);
}

bool frontend_save_file_dialog(FrontendSaveFileType type, char *path, size_t path_size,
                                char *error, size_t error_size) {
    if ((unsigned)type >= sizeof(save_dialogs) / sizeof(save_dialogs[0])) {
        set_error(error, error_size, "The requested output format is not supported");
        return false;
    }
    if (file_chooser) return file_chooser(true, (unsigned)type, path, path_size, error, error_size, file_chooser_context);
    return native_file_dialog(&save_dialogs[type], path, path_size, error, error_size);
}

bool frontend_open_movie_dialog(char *path, size_t path_size,
                                char *error, size_t error_size) {
    return frontend_open_file_dialog(FRONTEND_OPEN_MOVIE, path, path_size, error, error_size);
}

bool frontend_open_file_dialog(FrontendOpenFileType type, char *path, size_t path_size,
                                char *error, size_t error_size) {
    if ((unsigned)type >= sizeof(open_dialogs) / sizeof(open_dialogs[0])) {
        set_error(error, error_size, "The requested input format is not supported");
        return false;
    }
    if (file_chooser) return file_chooser(false, (unsigned)type, path, path_size, error, error_size, file_chooser_context);
    return native_file_dialog(&open_dialogs[type], path, path_size, error, error_size);
}

bool frontend_select_folder_dialog(char *path, size_t path_size,
                                    char *error, size_t error_size) {
    return frontend_open_file_dialog(FRONTEND_OPEN_FOLDER, path, path_size, error, error_size);
}
