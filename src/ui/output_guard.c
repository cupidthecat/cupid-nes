/*
 * output_guard.c - Validate output destinations before replacing files
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "output_guard.h"
#include "frontend_execution.h"
#include "app_paths.h"
#include "../rom/fds.h"
#include "../util/file_io.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool path_error(char *error, size_t error_size, const char *message) {
    if (error && error_size) snprintf(error, error_size, "%s", message);
    return false;
}

bool frontend_output_path_excludes(const char *path,
                                    const char *const *protected_paths, size_t count,
                                    char *error, size_t error_size) {
    if (error && error_size) error[0] = '\0';
    if (!path || !*path || (count && !protected_paths))
        return path_error(error, error_size, "Choose an output path first");
    for (size_t i = 0; i < count; ++i) {
        if (!protected_paths[i] || !*protected_paths[i]) continue;
        bool same = false;
        NesFileResult result = nes_file_same(path, protected_paths[i], &same);
        if (result == NES_FILE_OK && same)
            return path_error(error, error_size,
                              "Output cannot replace an active image, firmware, or save file");
        if (result != NES_FILE_OK && result != NES_FILE_NOT_FOUND)
            return path_error(error, error_size, "Could not verify the selected output path");
    }
    return true;
}

static bool output_path_allowed(const char *path,
                                   const FrontendExecutionRuntime *execution,
                                   const char *const *protected_paths, size_t count,
                                   char *error, size_t error_size, bool movie_save) {
    if (!frontend_output_path_excludes(path, protected_paths, count, error, error_size)) return false;
    if (!execution) return true;
    if (!frontend_output_path_excludes(path, execution->protected_paths, execution->protected_path_count,
                                       error, error_size)) return false;
    if (!movie_save && nes_movie_mode(execution->movie) != NES_MOVIE_IDLE) {
        NesMovieProgress progress;
        nes_movie_progress(execution->movie, &progress);
        const char *movie_path = progress.path;
        if (!frontend_output_path_excludes(path, &movie_path, 1, error, error_size)) return false;
    }
    const char *active[] = {
        execution->rom_path, execution->save_identity,
        execution->fds_bios_path, execution->studybox_bios_path, fds_save_path(),
        frontend_paths_config_file(), frontend_paths_recent_file()
    };
    if (!frontend_output_path_excludes(path, active, sizeof(active) / sizeof(active[0]),
                                       error, error_size)) return false;
    const char *identity = execution->save_identity ? execution->save_identity : execution->rom_path;
    if (!identity || !*identity) return true;
    const char *slash = strrchr(identity, '/');
    const char *backslash = strrchr(identity, '\\');
    if (!slash || (backslash && backslash > slash)) slash = backslash;
    const char *dot = strrchr(identity, '.');
    if (dot && slash && dot < slash) dot = NULL;
    size_t stem = dot ? (size_t)(dot - identity) : strlen(identity);
    if (stem >= NES_FILE_PATH_LIMIT - 32)
        return path_error(error, error_size, "Active save path is too long to verify");
    char *candidate = malloc(stem + 32);
    if (!candidate) return path_error(error, error_size, "Not enough memory to verify the output path");
    memcpy(candidate, identity, stem);
    static const char *const suffixes[] = {
        ".sav", ".chr.sav", ".flash.sav", ".chr.flash.sav", ".eeprom128",
        ".eeprom256", ".turbofile.sav", ".battlebox.sav"
    };
    bool allowed = true;
    for (size_t i = 0; allowed && i < sizeof(suffixes) / sizeof(suffixes[0]); ++i) {
        memcpy(candidate + stem, suffixes[i], strlen(suffixes[i]) + 1);
        const char *reserved = candidate;
        allowed = frontend_output_path_excludes(path, &reserved, 1, error, error_size);
    }
    free(candidate);
    return allowed;
}

bool frontend_output_path_allowed(const char *path, const FrontendExecutionRuntime *execution,
                                   const char *const *protected_paths, size_t count,
                                   char *error, size_t error_size) {
    return output_path_allowed(path, execution, protected_paths, count, error, error_size, false);
}

bool frontend_movie_output_path_allowed(const char *path, const FrontendExecutionRuntime *execution,
                                         const char *const *protected_paths, size_t count,
                                         char *error, size_t error_size) {
    return output_path_allowed(path, execution, protected_paths, count, error, error_size, true);
}
