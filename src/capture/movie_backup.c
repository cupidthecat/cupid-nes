/* movie_backup.c - Stage new data, preserve the prior version, then replace
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "movie_backup.h"
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

static NesMovieBackupOptions policy = {true, 5, ""};

void nes_movie_backup_defaults(NesMovieBackupOptions *options) {
    if (options) {
        *options = (NesMovieBackupOptions){true, 5, ""};
    }
}

bool nes_movie_backup_valid(const NesMovieBackupOptions *options) {
    return options && options->retained >= 1 && options->retained <= NES_MOVIE_BACKUP_MAX &&
           memchr(options->directory, 0, sizeof(options->directory));
}

NesFileResult nes_movie_backup_path(const char *path, const NesMovieBackupOptions *options, unsigned version,
                                    char *output, size_t capacity) {
    if (!path || !*path || !nes_movie_backup_valid(options) || !version || version > NES_MOVIE_BACKUP_MAX || !output ||
        !capacity) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    const char *base = path;
    uint64_t hash = UINT64_C(14695981039346656037);
    for (const char *p = path; *p; ++p) {
        hash = (hash ^ (uint8_t)*p) * UINT64_C(1099511628211);
        if (*p == '/' || *p == '\\') {
            base = p + 1;
        }
    }
    if (!*base) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    int length;
    if (options->directory[0]) {
        length = snprintf(output, capacity, "%s/%s.%016" PRIx64 ".bak.%03u", options->directory, base, hash, version);
    } else {
        length = snprintf(output, capacity, "%s.bak.%03u", path, version);
    }
    return length >= 0 && (size_t)length < capacity ? NES_FILE_OK : NES_FILE_TOO_LARGE;
}

static NesFileResult rotate(const char *path, const NesMovieBackupOptions *options, const uint8_t *previous,
                            size_t previous_size) {
    char source[NES_FILE_PATH_LIMIT], destination[NES_FILE_PATH_LIMIT];
    /* Copy oldest first. Every slot is replaced atomically, and the active
     * destination is untouched until all required recovery copies exist. */
    for (unsigned i = options->retained; i > 1; --i) {
        NesFileResult result = nes_movie_backup_path(path, options, i - 1u, source, sizeof(source));
        if (result != NES_FILE_OK) {
            return result;
        }
        result = nes_movie_backup_path(path, options, i, destination, sizeof(destination));
        if (result != NES_FILE_OK) {
            return result;
        }
        uint8_t *bytes = NULL;
        size_t size = 0;
        result = nes_file_read_all(source, 512u * 1024u * 1024u, &bytes, &size);
        if (result == NES_FILE_NOT_FOUND) {
            continue;
        }
        if (result == NES_FILE_OK) {
            result = nes_file_write_atomic(destination, bytes, size);
        }
        free(bytes);
        if (result != NES_FILE_OK) {
            return result;
        }
    }
    NesFileResult result = nes_movie_backup_path(path, options, 1, destination, sizeof(destination));
    if (result == NES_FILE_OK) {
        result = nes_file_write_atomic(destination, previous, previous_size);
    }
    return result;
}

NesFileResult nes_movie_backup_write(const char *path, const void *data, size_t size,
                                     const NesMovieBackupOptions *options) {
    if (!nes_movie_backup_valid(options) || (!data && size)) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    if (!options->enabled) {
        return nes_file_write_atomic(path, data, size);
    }
    NesFileTransaction *staged = NULL;
    NesFileResult result = nes_file_transaction_begin(path, size ? size : 1u, &staged);
    if (result == NES_FILE_OK) {
        result = nes_file_transaction_write(staged, data, size);
    }
    uint8_t *previous = NULL;
    size_t previous_size = 0;
    if (result == NES_FILE_OK) {
        result = nes_file_read_all(path, 512u * 1024u * 1024u, &previous, &previous_size);
        if (result == NES_FILE_NOT_FOUND) {
            result = NES_FILE_OK;
        } else if (result == NES_FILE_OK) {
            result = rotate(path, options, previous, previous_size);
        }
    }
    free(previous);
    if (result == NES_FILE_OK) {
        result = nes_file_transaction_commit(&staged);
    }
    nes_file_transaction_abort(&staged);
    /* Prune only after replacement succeeds. Existing recovery slots survive
     * failures and an interrupted commit. This also applies a lowered count. */
    if (result == NES_FILE_OK) {
        for (unsigned i = options->retained + 1; i <= NES_MOVIE_BACKUP_MAX; ++i) {
            char name[NES_FILE_PATH_LIMIT];
            if (nes_movie_backup_path(path, options, i, name, sizeof(name)) == NES_FILE_OK) {
                (void)nes_file_remove(name);
            }
        }
    }
    return result;
}

void nes_movie_backup_set_policy(const NesMovieBackupOptions *options) {
    if (nes_movie_backup_valid(options)) {
        policy = *options;
    }
}

NesFileResult nes_movie_save_atomic(const char *path, const void *data, size_t size) {
    return nes_movie_backup_write(path, data, size, &policy);
}
