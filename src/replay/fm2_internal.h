/*
 * fm2_internal.h - private helpers for the FM2/FM3 codec
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_REPLAY_FM2_INTERNAL_H
#define CUPID_REPLAY_FM2_INTERNAL_H

#include "fm2.h"

#include <stdarg.h>

typedef struct {
    const uint8_t *data;
    size_t size;
    size_t offset;
    NesFm2Limits limits;
    size_t text_bytes;
} NesFm2Reader;

void nes_fm2__diagnostic_clear(NesFm2Diagnostic *diagnostic);
NesFm2Result nes_fm2__fail(NesFm2Diagnostic *diagnostic, NesFm2Result result, size_t byte_offset, size_t frame_index,
                           const char *format, ...);
bool nes_fm2__checked_add(size_t a, size_t b, size_t *out);
bool nes_fm2__checked_mul(size_t a, size_t b, size_t *out);
char *nes_fm2__dup_slice(const uint8_t *data, size_t size);
NesFm2Result nes_fm2__copy_blob(const NesFm2Blob *source, NesFm2Blob *out, NesFm2Diagnostic *diagnostic);
NesFm2Result nes_fm2__copy_string_list(const NesFm2StringList *source, NesFm2StringList *out,
                                       NesFm2Diagnostic *diagnostic);

NesFm2Result nes_fm2__parse_movie(NesFm2Reader *reader, NesFm2Movie *movie, NesFm2Diagnostic *diagnostic);
NesFm2Result nes_fm2__measure_movie(const NesFm2Movie *movie, NesFm2Container container, size_t *size,
                                    NesFm2Diagnostic *diagnostic);
NesFm2Result nes_fm2__write_movie(const NesFm2Movie *movie, NesFm2Container container, uint8_t *destination,
                                  size_t capacity, size_t *written, NesFm2Diagnostic *diagnostic);

#endif
