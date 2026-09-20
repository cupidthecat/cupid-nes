/*
 * capture_writer.h - PNG screenshots and portable PCM/AVI recording
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef NES_CAPTURE_WRITER_H
#define NES_CAPTURE_WRITER_H

#include "../util/file_io.h"

enum { NES_CAPTURE_MAX_DIMENSION = 4096 };

typedef struct {
    const uint32_t *pixels;
    unsigned width;
    unsigned height;
    size_t stride;
} NesCaptureFrame;

typedef struct NesCaptureStream NesCaptureStream;

// Pixels are 0xAARRGGBB, rows top to bottom, stride in pixels.
NesFileResult nes_capture_png(const char *path, const NesCaptureFrame *frame);

// All recordings contain signed 16-bit little-endian stereo PCM. AVI video is
// uncompressed 24-bit BGR with a rational frame rate. A byte limit includes the
// final container header and index. TOO_LARGE leaves a valid stream available
// for closing; I/O failures discard its temporary output when closed.
NesFileResult nes_capture_wav_open(const char *path, unsigned sample_rate,
                                    uint64_t byte_limit, NesCaptureStream **out);
NesFileResult nes_capture_avi_open(const char *path, unsigned width, unsigned height,
                                    unsigned sample_rate, uint32_t fps_numerator,
                                    uint32_t fps_denominator, uint64_t byte_limit,
                                    NesCaptureStream **out);
NesFileResult nes_capture_write_audio(NesCaptureStream *stream, const int16_t *samples,
                                       size_t frames);
NesFileResult nes_capture_write_frame(NesCaptureStream *stream,
                                       const NesCaptureFrame *frame,
                                       const int16_t *samples, size_t audio_frames);
NesFileResult nes_capture_close(NesCaptureStream **stream);
void nes_capture_abort(NesCaptureStream **stream);
uint64_t nes_capture_audio_frames(const NesCaptureStream *stream);
uint32_t nes_capture_video_frames(const NesCaptureStream *stream);
uint64_t nes_capture_file_size(const NesCaptureStream *stream);

#endif
