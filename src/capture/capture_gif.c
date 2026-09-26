/* capture_gif.c - RGB332 palette, LZW and emulated-time GIF delays
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "capture_gif.h"
#include <stdlib.h>
#include <string.h>

struct NesCaptureGif {
    NesFileTransaction *file;
    NesFileResult error;
    unsigned width, height, scale;
    uint32_t numerator, denominator;
    uint64_t phase, limit;
};

typedef struct {
    uint8_t *data;
    size_t size;
    uint32_t bits;
    unsigned count;
} GifBits;

static void put16(uint8_t *p, unsigned n) {
    p[0] = (uint8_t)n;
    p[1] = (uint8_t)(n >> 8);
}

static void code(GifBits *out, unsigned value, unsigned width) {
    out->bits |= (uint32_t)value << out->count;
    out->count += width;
    while (out->count >= 8) {
        out->data[out->size++] = (uint8_t)out->bits;
        out->bits >>= 8;
        out->count -= 8;
    }
}

static unsigned palette_index(uint32_t pixel) {
    return ((pixel >> 16) & 0xE0u) | ((pixel >> 11) & 0x1Cu) | ((pixel >> 6) & 3u);
}

static unsigned pixel_at(const NesCaptureGif *gif, const NesCaptureFrame *frame, size_t i) {
    unsigned width = gif->width * gif->scale;
    size_t y = i / width / gif->scale;
    size_t x = i % width / gif->scale;
    return palette_index(frame->pixels[y * frame->stride + x]);
}

static size_t encode(const NesCaptureGif *gif, const NesCaptureFrame *frame, uint8_t *data) {
    int32_t keys[8192];
    uint16_t values[8192];
    memset(keys, 0xFF, sizeof(keys));
    GifBits out = {data, 0, 0, 0};
    unsigned next = 258, width = 9;
    size_t count = (size_t)gif->width * gif->height * gif->scale * gif->scale;
    unsigned prefix = pixel_at(gif, frame, 0);
    code(&out, 256, width);
    for (size_t i = 1; i < count; ++i) {
        unsigned suffix = pixel_at(gif, frame, i);
        int32_t key = (int32_t)((prefix << 8) | suffix);
        unsigned slot = ((unsigned)key * 2654435761u) >> 19;
        while (keys[slot] != -1 && keys[slot] != key) {
            slot = (slot + 1u) & 8191u;
        }
        if (keys[slot] == key) {
            prefix = values[slot];
        } else {
            code(&out, prefix, width);
            if (next < 4096) {
                keys[slot] = key;
                values[slot] = (uint16_t)next++;
                if (next > (1u << width) && width < 12) {
                    ++width;
                }
            } else {
                code(&out, 256, width);
                memset(keys, 0xFF, sizeof(keys));
                next = 258;
                width = 9;
            }
            prefix = suffix;
        }
    }
    code(&out, prefix, width);
    /* The decoder creates its final dictionary entry after this prefix. */
    if (next == (1u << width) && width < 12) {
        ++width;
    }
    code(&out, 257, width);
    if (out.count) {
        out.data[out.size++] = (uint8_t)out.bits;
    }
    return out.size;
}

NesFileResult nes_capture_gif_open(const char *path, unsigned width, unsigned height, unsigned scale,
                                   uint32_t fps_numerator, uint32_t fps_denominator, uint64_t limit,
                                   NesCaptureGif **out) {
    if (out) {
        *out = NULL;
    }
    if (!out || !width || !height || scale < 1 || scale > 4 || width > NES_CAPTURE_MAX_DIMENSION / scale ||
        height > NES_CAPTURE_MAX_DIMENSION / scale || !fps_numerator || !fps_denominator ||
        (double)fps_numerator / fps_denominator < 1 || (double)fps_numerator / fps_denominator > 100 || limit < 802) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    NesCaptureGif *gif = calloc(1, sizeof(*gif));
    if (!gif) {
        return NES_FILE_OUT_OF_MEMORY;
    }
    gif->width = width;
    gif->height = height;
    gif->scale = scale;
    gif->numerator = fps_numerator;
    gif->denominator = fps_denominator;
    gif->limit = limit;
    NesFileResult result = nes_file_transaction_begin(path, limit, &gif->file);
    uint8_t header[800] = {0};
    memcpy(header, "GIF89a", 6);
    put16(header + 6, width * scale);
    put16(header + 8, height * scale);
    header[10] = 0xF7;
    for (unsigned i = 0; i < 256; ++i) {
        header[13 + i * 3] = (uint8_t)(((i >> 5) * 255u + 3u) / 7u);
        header[14 + i * 3] = (uint8_t)((((i >> 2) & 7u) * 255u + 3u) / 7u);
        header[15 + i * 3] = (uint8_t)((i & 3u) * 85u);
    }
    const uint8_t loop[] = {0x21, 0xFF, 11, 'N', 'E', 'T', 'S', 'C', 'A', 'P', 'E', '2', '.', '0', 3, 1, 0, 0, 0};
    memcpy(header + 781, loop, sizeof(loop));
    if (result == NES_FILE_OK) {
        result = nes_file_transaction_write(gif->file, header, sizeof(header));
    }
    if (result != NES_FILE_OK) {
        nes_capture_gif_abort(&gif);
        return result;
    }
    *out = gif;
    return NES_FILE_OK;
}

NesFileResult nes_capture_gif_frame(NesCaptureGif *gif, const NesCaptureFrame *frame) {
    if (!gif || !frame || !frame->pixels || frame->width != gif->width || frame->height != gif->height ||
        frame->stride < frame->width || frame->stride > SIZE_MAX / frame->height / 4u) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    if (gif->error != NES_FILE_OK) {
        return gif->error;
    }
    size_t pixels = (size_t)gif->width * gif->height * gif->scale * gif->scale;
    uint8_t *compressed = malloc(pixels * 2u + 32u);
    if (!compressed) {
        return NES_FILE_OUT_OF_MEMORY;
    }
    size_t size = encode(gif, frame, compressed);
    uint64_t bytes = 20u + size + (size + 254u) / 255u;
    if (nes_file_transaction_size(gif->file) + bytes + 1u > gif->limit) {
        free(compressed);
        return NES_FILE_TOO_LARGE;
    }
    uint64_t phase = gif->phase + (uint64_t)gif->denominator * 100u;
    unsigned delay = (unsigned)(phase / gif->numerator);
    uint8_t header[19] = {0x21, 0xF9, 4, 4, 0, 0, 0, 0, 0x2C, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8};
    put16(header + 4, delay);
    put16(header + 13, gif->width * gif->scale);
    put16(header + 15, gif->height * gif->scale);
    NesFileResult result = nes_file_transaction_write(gif->file, header, sizeof(header));
    for (size_t offset = 0; result == NES_FILE_OK && offset < size;) {
        uint8_t count = (uint8_t)(size - offset > 255 ? 255 : size - offset);
        result = nes_file_transaction_write(gif->file, &count, 1);
        if (result == NES_FILE_OK) {
            result = nes_file_transaction_write(gif->file, compressed + offset, count);
        }
        offset += count;
    }
    if (result == NES_FILE_OK) {
        result = nes_file_transaction_write(gif->file, "", 1);
    }
    free(compressed);
    gif->error = result;
    if (result == NES_FILE_OK) {
        gif->phase = phase % gif->numerator;
    }
    return result;
}

uint64_t nes_capture_gif_size(const NesCaptureGif *gif) {
    return gif ? nes_file_transaction_size(gif->file) + 1u : 0;
}

void nes_capture_gif_abort(NesCaptureGif **ptr) {
    if (!ptr || !*ptr) {
        return;
    }
    nes_file_transaction_abort(&(*ptr)->file);
    free(*ptr);
    *ptr = NULL;
}

NesFileResult nes_capture_gif_close(NesCaptureGif **ptr) {
    if (!ptr || !*ptr) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    NesCaptureGif *gif = *ptr;
    NesFileResult result = gif->error;
    if (result == NES_FILE_OK) {
        result = nes_file_transaction_write(gif->file, ";", 1);
    }
    if (result == NES_FILE_OK) {
        result = nes_file_transaction_commit(&gif->file);
    }
    nes_capture_gif_abort(ptr);
    return result;
}
