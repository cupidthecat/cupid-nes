/*
 * platform_frontend.h - Desktop platform entry and file dialogs
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef PLATFORM_FRONTEND_H
#define PLATFORM_FRONTEND_H

#include <stdbool.h>
#include <stddef.h>

typedef int (*FrontendApplicationEntry)(int argc, char **argv);
typedef enum {
    FRONTEND_SAVE_PNG,
    FRONTEND_SAVE_WAV,
    FRONTEND_SAVE_AVI,
    FRONTEND_SAVE_MOVIE,
    FRONTEND_SAVE_STATE,
    FRONTEND_SAVE_TAPE,
    FRONTEND_SAVE_HD_PACK,
    FRONTEND_SAVE_CHEATS,
    FRONTEND_SAVE_DISK_OVERLAY
} FrontendSaveFileType;

typedef enum {
    FRONTEND_OPEN_IMAGE,
    FRONTEND_OPEN_MOVIE,
    FRONTEND_OPEN_STATE,
    FRONTEND_OPEN_PALETTE,
    FRONTEND_OPEN_PATCH,
    FRONTEND_OPEN_TAPE,
    FRONTEND_OPEN_FIRMWARE,
    FRONTEND_OPEN_DATABASE,
    FRONTEND_OPEN_HD_PACK,
    FRONTEND_OPEN_SCRIPT,
    FRONTEND_OPEN_CHEATS,
    FRONTEND_OPEN_FOLDER
} FrontendOpenFileType;

/* Optional host chooser; NULL restores the native platform dialog. */
typedef bool (*FrontendFileChooser)(bool save, unsigned type, char *path, size_t path_size,
                                     char *error, size_t error_size, void *context);
void frontend_set_file_chooser(FrontendFileChooser chooser, void *context);

int frontend_application_entry_utf8(int argc, char **argv,
                                    FrontendApplicationEntry entry);
bool frontend_open_image_dialog(char *path, size_t path_size,
                                char *error, size_t error_size);
bool frontend_open_movie_dialog(char *path, size_t path_size,
                                char *error, size_t error_size);
bool frontend_save_file_dialog(FrontendSaveFileType type, char *path, size_t path_size,
                                char *error, size_t error_size);
bool frontend_open_file_dialog(FrontendOpenFileType type, char *path, size_t path_size,
                                char *error, size_t error_size);
bool frontend_select_folder_dialog(char *path, size_t path_size,
                                    char *error, size_t error_size);
/* Resolve an existing directory and encode its absolute UTF-8 path as a file
 * URI. Shell metacharacters never become command text. Failure retains uri. */
bool frontend_folder_uri(const char *path, char *uri, size_t uri_size,
                          char *error, size_t error_size);
bool frontend_open_folder(const char *path, char *error, size_t error_size);
// Decode one complete line returned by a desktop chooser. On failure the
// previous path is retained; truncated or ambiguous output is never a path.
bool frontend_parse_dialog_output(const char *output, size_t output_size,
                                   char *path, size_t path_size,
                                   char *error, size_t error_size);

#endif
