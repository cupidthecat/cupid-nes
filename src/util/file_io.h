/*
 * file_io.h - UTF-8 file paths, bounded reads, and atomic replacement
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#ifndef CUPID_FILE_IO_H
#define CUPID_FILE_IO_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { NES_FILE_PATH_LIMIT = 32768 };

typedef enum {
    NES_FILE_OK = 0,
    NES_FILE_NOT_FOUND,
    NES_FILE_INVALID_ARGUMENT,
    NES_FILE_TOO_LARGE,
    NES_FILE_OUT_OF_MEMORY,
    NES_FILE_IO_ERROR
} NesFileResult;

FILE *nes_file_open(const char *path, const char *mode);
// On success the caller owns *data, including for an empty file. On failure
// both outputs are cleared and the caller has nothing to release.
NesFileResult nes_file_read_all(const char *path, size_t limit, uint8_t **data, size_t *size);
// Write and flush a unique sibling file before replacing the destination.
// A failure leaves any existing destination in place.
NesFileResult nes_file_write_atomic(const char *path, const void *data, size_t size);
NesFileResult nes_file_remove(const char *path);
const char *nes_file_result_message(NesFileResult result);

#ifdef __cplusplus
}
#endif
#endif
