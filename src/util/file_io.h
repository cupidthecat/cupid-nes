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
#include <stdbool.h>
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
// Stream a bounded output into a unique sibling file. Commit flushes the stream
// and atomically replaces the destination; abort leaves the destination alone.
// Seeking is limited to bytes already written, which permits container headers
// to be finalized without creating sparse or uninitialized ranges.
typedef struct NesFileTransaction NesFileTransaction;
NesFileResult nes_file_transaction_begin(const char *path, uint64_t limit,
                                          NesFileTransaction **out);
NesFileResult nes_file_transaction_write(NesFileTransaction *transaction,
                                          const void *data, size_t size);
NesFileResult nes_file_transaction_seek(NesFileTransaction *transaction, uint64_t position);
uint64_t nes_file_transaction_position(const NesFileTransaction *transaction);
uint64_t nes_file_transaction_size(const NesFileTransaction *transaction);
// Commit and abort consume the transaction and clear the caller's pointer.
NesFileResult nes_file_transaction_commit(NesFileTransaction **transaction);
void nes_file_transaction_abort(NesFileTransaction **transaction);
NesFileResult nes_file_remove(const char *path);
// Compare existing file identities, following links. A missing path returns
// NES_FILE_NOT_FOUND with *same false; identical valid spellings return true.
NesFileResult nes_file_same(const char *left, const char *right, bool *same);
const char *nes_file_result_message(NesFileResult result);

#ifdef __cplusplus
}
#endif
#endif
