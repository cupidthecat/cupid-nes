/*
 * output_guard.h - Keep generated output separate from active media
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_FRONTEND_OUTPUT_GUARD_H
#define CUPID_FRONTEND_OUTPUT_GUARD_H

#include <stdbool.h>
#include <stddef.h>

struct FrontendExecutionRuntime;

bool frontend_output_path_excludes(const char *path,
                                    const char *const *protected_paths, size_t count,
                                    char *error, size_t error_size);
bool frontend_output_path_allowed(const char *path,
                                   const struct FrontendExecutionRuntime *execution,
                                   const char *const *protected_paths, size_t count,
                                   char *error, size_t error_size);
/* An explicit movie save can replace its own source; other outputs cannot. */
bool frontend_movie_output_path_allowed(const char *path,
                                         const struct FrontendExecutionRuntime *execution,
                                         const char *const *protected_paths, size_t count,
                                         char *error, size_t error_size);

#endif
