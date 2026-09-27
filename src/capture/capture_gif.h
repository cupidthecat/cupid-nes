/* capture_gif.h - Animated GIF recording
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef NES_CAPTURE_GIF_H
#define NES_CAPTURE_GIF_H
#include "capture_writer.h"
typedef struct NesCaptureGif NesCaptureGif;
NesFileResult nes_capture_gif_open(const char *path, unsigned width, unsigned height, unsigned scale,
                                   uint32_t fps_numerator, uint32_t fps_denominator, uint64_t limit,
                                   NesCaptureGif **out);
NesFileResult nes_capture_gif_frame(NesCaptureGif *gif, const NesCaptureFrame *frame);
NesFileResult nes_capture_gif_close(NesCaptureGif **gif);
void nes_capture_gif_abort(NesCaptureGif **gif);
uint64_t nes_capture_gif_size(const NesCaptureGif *gif);
#endif
