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
    FRONTEND_SAVE_AVI
} FrontendSaveFileType;

int frontend_application_entry_utf8(int argc, char **argv,
                                    FrontendApplicationEntry entry);
bool frontend_open_image_dialog(char *path, size_t path_size,
                                char *error, size_t error_size);
bool frontend_save_file_dialog(FrontendSaveFileType type, char *path, size_t path_size,
                                char *error, size_t error_size);
// Decode one complete line returned by a desktop chooser. On failure the
// previous path is retained; truncated or ambiguous output is never a path.
bool frontend_parse_dialog_output(const char *output, size_t output_size,
                                   char *path, size_t path_size,
                                   char *error, size_t error_size);

#endif
