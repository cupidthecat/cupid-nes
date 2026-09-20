/*
 * app_paths.h - Shared application storage locations
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef APP_PATHS_H
#define APP_PATHS_H

#include <stdbool.h>
#include <stddef.h>

bool frontend_paths_init(const char *data_dir_override, char *error, size_t error_size);
void frontend_paths_shutdown(void);
const char *frontend_paths_data_dir(void);
const char *frontend_paths_config_file(void);
const char *frontend_paths_recent_file(void);
bool frontend_paths_join(char *dst, size_t dst_size, const char *relative);

#endif
