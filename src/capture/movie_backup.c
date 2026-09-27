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

static NesFileResult stage_copy(const char *path, const uint8_t *bytes, size_t size, NesFileTransaction **copy) {
    NesFileResult result = nes_file_transaction_begin(path, size, copy);
    if (result == NES_FILE_OK) {
        result = nes_file_transaction_write(*copy, bytes, size);
    }
    return result;
}

static NesFileResult rotate(const char *path, const NesMovieBackupOptions *options, const uint8_t *previous,
                            size_t previous_size, bool *prune) {
    char source[NES_FILE_PATH_LIMIT], destination[NES_FILE_PATH_LIMIT];
    NesFileTransaction *copies[NES_MOVIE_BACKUP_MAX + 1] = {NULL};
    NesFileResult result = NES_FILE_OK;
    /* Stage every copy before replacing history, so a failed source read or
     * temporary-file write leaves all existing generations available. */
    for (unsigned i = options->retained; i > 1; --i) {
        result = nes_movie_backup_path(path, options, i - 1u, source, sizeof(source));
        if (result != NES_FILE_OK) {
            goto cleanup;
        }
        result = nes_movie_backup_path(path, options, i, destination, sizeof(destination));
        if (result != NES_FILE_OK) {
            goto cleanup;
        }
        uint8_t *bytes = NULL;
        size_t size = 0;
        result = nes_file_read_all(source, 512u * 1024u * 1024u, &bytes, &size);
        if (result == NES_FILE_NOT_FOUND) {
            prune[i] = true;
            continue;
        }
        if (result == NES_FILE_OK) {
            result = stage_copy(destination, bytes, size, &copies[i]);
        }
        free(bytes);
        if (result != NES_FILE_OK) {
            goto cleanup;
        }
    }
    result = nes_movie_backup_path(path, options, 1, destination, sizeof(destination));
    if (result == NES_FILE_OK) {
        result = stage_copy(destination, previous, previous_size, &copies[1]);
    }
    for (unsigned i = options->retained; i && result == NES_FILE_OK; --i) {
        if (copies[i]) {
            result = nes_file_transaction_commit(&copies[i]);
        }
    }
cleanup:
    for (unsigned i = 1; i <= options->retained; ++i) {
        nes_file_transaction_abort(&copies[i]);
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
    bool prune[NES_MOVIE_BACKUP_MAX + 1] = {false};
    if (result == NES_FILE_OK) {
        result = nes_file_read_all(path, 512u * 1024u * 1024u, &previous, &previous_size);
        if (result == NES_FILE_NOT_FOUND) {
            result = NES_FILE_OK;
        } else if (result == NES_FILE_OK) {
            result = rotate(path, options, previous, previous_size, prune);
        }
    }
    free(previous);
    if (result == NES_FILE_OK) {
        result = nes_file_transaction_commit(&staged);
    }
    nes_file_transaction_abort(&staged);
    /* Remove stale gap destinations and excess generations only after the
     * active movie has been replaced successfully. */
    if (result == NES_FILE_OK) {
        for (unsigned i = 1; i <= NES_MOVIE_BACKUP_MAX; ++i) {
            if (i <= options->retained && !prune[i]) {
                continue;
            }
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
