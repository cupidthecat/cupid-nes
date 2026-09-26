/* capture_codec.h - Lossless AVI frame encoding
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef NES_CAPTURE_CODEC_H
#define NES_CAPTURE_CODEC_H
#include "capture_writer.h"
NesFileResult nes_capture_zmbv(const NesCaptureFrame *frame, unsigned level, uint8_t **data, size_t *size);
#endif
