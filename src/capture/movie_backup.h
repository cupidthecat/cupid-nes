/* movie_backup.h - Recoverable movie and TAS replacement
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef NES_MOVIE_BACKUP_H
#define NES_MOVIE_BACKUP_H
#include "../util/file_io.h"

enum { NES_MOVIE_BACKUP_PATH = 1024, NES_MOVIE_BACKUP_MAX = 100 };

typedef struct {
    bool enabled;
    unsigned retained;
    char directory[NES_MOVIE_BACKUP_PATH];
} NesMovieBackupOptions;

void nes_movie_backup_defaults(NesMovieBackupOptions *options);
bool nes_movie_backup_valid(const NesMovieBackupOptions *options);
NesFileResult nes_movie_backup_path(const char *path, const NesMovieBackupOptions *options, unsigned version,
                                    char *output, size_t capacity);
NesFileResult nes_movie_backup_write(const char *path, const void *data, size_t size,
                                     const NesMovieBackupOptions *options);
/* Application-thread policy shared by movie and TAS save callers. */
void nes_movie_backup_set_policy(const NesMovieBackupOptions *options);
NesFileResult nes_movie_save_atomic(const char *path, const void *data, size_t size);
#endif
