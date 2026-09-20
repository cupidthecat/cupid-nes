/*
 * app_paths.c - Shared application storage locations
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "app_paths.h"
#include <SDL2/SDL.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#define FRONTEND_PATH_SEPARATOR '\\'
#else
#define FRONTEND_PATH_SEPARATOR '/'
#endif

static char *data_dir;
static char config_file[4096];
static char recent_file[4096];

static char *copy_path(const char *path) {
    if (!path || !*path) return NULL;
    size_t size = strlen(path) + 2;
    char *copy = (char *)malloc(size);
    if (!copy) return NULL;
    strcpy(copy, path);
    size_t length = strlen(copy);
    if (copy[length - 1] != '/' && copy[length - 1] != '\\') {
        copy[length++] = FRONTEND_PATH_SEPARATOR;
        copy[length] = '\0';
    }
    return copy;
}

bool frontend_paths_join(char *dst, size_t dst_size, const char *relative) {
    if (!dst || !dst_size || !data_dir || !relative || !*relative) return false;
    int written = snprintf(dst, dst_size, "%s%s", data_dir, relative);
    return written >= 0 && (size_t)written < dst_size;
}

bool frontend_paths_init(const char *data_dir_override, char *error, size_t error_size) {
    frontend_paths_shutdown();
    if (error && error_size) error[0] = '\0';
    if (data_dir_override && *data_dir_override) {
        data_dir = copy_path(data_dir_override);
    } else {
        char *pref = SDL_GetPrefPath("cupidthecat", "cupid-nes");
        if (pref) {
            data_dir = copy_path(pref);
            SDL_free(pref);
        }
    }
    if (!data_dir
        || !frontend_paths_join(config_file, sizeof(config_file), "settings.ini")
        || !frontend_paths_join(recent_file, sizeof(recent_file), "recent.ini")) {
        if (error && error_size)
            snprintf(error, error_size, "Could not resolve application data directory");
        frontend_paths_shutdown();
        return false;
    }
    return true;
}

void frontend_paths_shutdown(void) {
    free(data_dir);
    data_dir = NULL;
    config_file[0] = '\0';
    recent_file[0] = '\0';
}

const char *frontend_paths_data_dir(void) {
    return data_dir;
}

const char *frontend_paths_config_file(void) {
    return config_file[0] ? config_file : NULL;
}

const char *frontend_paths_recent_file(void) {
    return recent_file[0] ? recent_file : NULL;
}
