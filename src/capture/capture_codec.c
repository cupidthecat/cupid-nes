/* capture_codec.c - Independently decodable lossless ZMBV keyframes
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "capture_codec.h"
#include "../third_party/miniz/miniz.h"
#include <stdlib.h>
#include <string.h>

NesFileResult nes_capture_zmbv(const NesCaptureFrame *frame, unsigned level, uint8_t **data, size_t *size) {
    if (data) {
        *data = NULL;
    }
    if (size) {
        *size = 0;
    }
    if (!data || !size || !frame || !frame->pixels || !frame->width || !frame->height ||
        frame->width > NES_CAPTURE_MAX_DIMENSION || frame->height > NES_CAPTURE_MAX_DIMENSION ||
        frame->stride < frame->width || frame->stride > SIZE_MAX / frame->height / 4u || level > 9) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    size_t bytes = (size_t)frame->width * frame->height * 4u;
    uint8_t *raw = malloc(bytes);
    mz_ulong capacity = mz_compressBound((mz_ulong)bytes);
    uint8_t *output = malloc((size_t)capacity + 7u);
    if (!raw || !output) {
        free(raw);
        free(output);
        return NES_FILE_OUT_OF_MEMORY;
    }
    for (unsigned y = 0; y < frame->height; ++y) {
        for (unsigned x = 0; x < frame->width; ++x) {
            uint32_t pixel = frame->pixels[(size_t)y * frame->stride + x];
            size_t offset = ((size_t)y * frame->width + x) * 4u;
            raw[offset] = (uint8_t)pixel;
            raw[offset + 1] = (uint8_t)(pixel >> 8);
            raw[offset + 2] = (uint8_t)(pixel >> 16);
            raw[offset + 3] = 0;
        }
    }
    /* Keyframe, version 0.1, zlib, BGR32, 16 by 16 motion blocks. */
    const uint8_t header[] = {1, 0, 1, 1, 8, 16, 16};
    memcpy(output, header, sizeof(header));
    int result = mz_compress2(output + 7, &capacity, raw, (mz_ulong)bytes, (int)level);
    free(raw);
    if (result != MZ_OK) {
        free(output);
        return result == MZ_MEM_ERROR ? NES_FILE_OUT_OF_MEMORY : NES_FILE_IO_ERROR;
    }
    *data = output;
    *size = (size_t)capacity + 7u;
    return NES_FILE_OK;
}
