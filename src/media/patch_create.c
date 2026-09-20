/*
 * patch_create.c - IPS overlays for writable media
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "patch.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint8_t *data;
    size_t size;
    size_t capacity;
    size_t limit;
    NesPatchResult error;
} PatchOutput;

static bool append(PatchOutput *out, const void *data, size_t size) {
    if (size > out->limit - out->size) {
        out->error = NES_PATCH_TOO_LARGE;
        return false;
    }

    size_t needed = out->size + size;
    if (needed > out->capacity) {
        size_t capacity = out->capacity ? out->capacity : 256;
        while (capacity < needed) {
            if (capacity > out->limit / 2) {
                capacity = out->limit;
                break;
            }

            capacity *= 2;
        }

        uint8_t *replacement = (uint8_t *)realloc(out->data, capacity);
        if (!replacement) {
            out->error = NES_PATCH_OUT_OF_MEMORY;
            return false;
        }

        out->data = replacement;
        out->capacity = capacity;
    }

    memcpy(out->data + out->size, data, size);
    out->size += size;
    return true;
}

static bool changed(const uint8_t *source, size_t size, const uint8_t *target, size_t offset) {
    return offset >= size || source[offset] != target[offset];
}

NesPatchResult nes_patch_create_ips(const uint8_t *source, size_t source_size,
                                    const uint8_t *target, size_t target_size,
                                    size_t limit, uint8_t **output, size_t *output_size) {
    if (output) {
        *output = NULL;
    }

    if (output_size) {
        *output_size = 0;
    }

    if (!output || !output_size || (!source && source_size) || (!target && target_size)) {
        return NES_PATCH_INVALID;
    }

    /* IPS has 24-bit record offsets and an optional 24-bit truncation field. */
    if (target_size > 0x1000000u || (target_size < source_size && target_size > 0xffffffu)) {
        return NES_PATCH_TOO_LARGE;
    }

    PatchOutput out = {NULL, 0, 0, limit, NES_PATCH_OK};
    if (!append(&out, "PATCH", 5)) {
        goto failed;
    }

    size_t position = 0;
    while (position < target_size) {
        if (!changed(source, source_size, target, position)) {
            position++;
            continue;
        }

        size_t start = position;
        /* A record cannot start at the three bytes spelling the EOF marker. */
        if (start == 0x454f46u) {
            start--;
        }

        position++;
        while (position < target_size && position - start < 65535
               && changed(source, source_size, target, position)) {
            position++;
        }

        size_t count = position - start;
        bool repeat = count >= 4;
        for (size_t i = start + 1; repeat && i < position; i++) {
            repeat = target[i] == target[start];
        }

        uint8_t header[8] = {
            (uint8_t)(start >> 16), (uint8_t)(start >> 8), (uint8_t)start,
            (uint8_t)(count >> 8), (uint8_t)count, 0, 0, 0
        };
        if (repeat) {
            header[3] = 0;
            header[4] = 0;
            header[5] = (uint8_t)(count >> 8);
            header[6] = (uint8_t)count;
            header[7] = target[start];
            if (!append(&out, header, sizeof(header))) {
                goto failed;
            }
        } else if (!append(&out, header, 5) || !append(&out, target + start, count)) {
            goto failed;
        }
    }

    if (!append(&out, "EOF", 3)) {
        goto failed;
    }

    if (target_size < source_size) {
        uint8_t truncate[3] = {(uint8_t)(target_size >> 16), (uint8_t)(target_size >> 8), (uint8_t)target_size};
        if (!append(&out, truncate, sizeof(truncate))) {
            goto failed;
        }
    }

    *output = out.data;
    *output_size = out.size;
    return NES_PATCH_OK;

failed:
    free(out.data);
    return out.error;
}
