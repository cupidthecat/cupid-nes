/*
 * capture_png.c - Screenshots from a selected presentation frame
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "capture_writer.h"
#include "../third_party/miniz/miniz.h"
#include <stdlib.h>

NesFileResult nes_capture_png_memory(const NesCaptureFrame *frame,
                                     uint8_t **data, size_t *output_size) {
    if (data) *data = NULL;
    if (output_size) *output_size = 0;
    if (!data || !output_size) return NES_FILE_INVALID_ARGUMENT;
    if (!frame || !frame->pixels || !frame->width || !frame->height
        || frame->width > NES_CAPTURE_MAX_DIMENSION || frame->height > NES_CAPTURE_MAX_DIMENSION
        || frame->stride < frame->width
        || frame->stride > SIZE_MAX / frame->height / sizeof(uint32_t)) return NES_FILE_INVALID_ARGUMENT;
    size_t size = (size_t)frame->width * frame->height * 4u;
    uint8_t *rgba = malloc(size);
    if (!rgba) return NES_FILE_OUT_OF_MEMORY;
    for (unsigned y = 0; y < frame->height; ++y) {
        for (unsigned x = 0; x < frame->width; ++x) {
            uint32_t pixel = frame->pixels[(size_t)y * frame->stride + x];
            uint8_t *output = rgba + ((size_t)y * frame->width + x) * 4u;
            output[0] = (uint8_t)(pixel >> 16);
            output[1] = (uint8_t)(pixel >> 8);
            output[2] = (uint8_t)pixel;
            output[3] = (uint8_t)(pixel >> 24);
        }
    }
    size_t png_size = 0;
    void *png = tdefl_write_image_to_png_file_in_memory(rgba, (int)frame->width,
                                                        (int)frame->height, 4, &png_size);
    free(rgba);
    if (!png) return NES_FILE_OUT_OF_MEMORY;
    uint8_t *copy = malloc(png_size ? png_size : 1);
    if (!copy) {
        mz_free(png);
        return NES_FILE_OUT_OF_MEMORY;
    }
    memcpy(copy, png, png_size);
    mz_free(png);
    *data = copy;
    *output_size = png_size;
    return NES_FILE_OK;
}

NesFileResult nes_capture_png(const char *path, const NesCaptureFrame *frame) {
    uint8_t *png = NULL;
    size_t png_size = 0;
    NesFileResult encoded = nes_capture_png_memory(frame, &png, &png_size);
    if (encoded != NES_FILE_OK) return encoded;
    NesFileResult result = nes_file_write_atomic(path, png, png_size);
    free(png);
    return result;
}
