/*
 * capture_session.h - Capture synchronized to completed emulation frames
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef NES_CAPTURE_SESSION_H
#define NES_CAPTURE_SESSION_H

#include "capture_writer.h"

typedef struct {
    uint32_t numerator;
    uint32_t denominator;
} NesCaptureFrameRate;

typedef struct {
    unsigned sample_rate;
    uint64_t byte_limit;
    bool displayed_output;
} NesCaptureOptions;

typedef struct {
    bool recording;
    bool video;
    unsigned width;
    unsigned height;
    unsigned sample_rate;
    NesCaptureFrameRate frame_rate;
    uint64_t completed_frames;
    uint64_t audio_frames;
    uint64_t bytes;
} NesCaptureInfo;

typedef struct {
    void *implementation;
    NesCaptureInfo info;
    char error[256];
} NesCaptureSession;

void nes_capture_options_defaults(NesCaptureOptions *options);
NesCaptureFrameRate nes_capture_frame_rate(void);
NesFileResult nes_capture_session_start(NesCaptureSession *session, const char *path,
                                        bool video, const NesCaptureFrame *frame,
                                        const NesCaptureOptions *options);
// Call before running a real frame. A debugger pause may leave that frame open;
// repeated begin calls retain it until a complete frame is presented.
bool nes_capture_session_begin_frame(NesCaptureSession *session);
// Call once per completed real frame, before drawing UI overlays. The frame is
// the chosen raw or displayed output; it can be NULL for an audio recording.
NesFileResult nes_capture_session_end_frame(NesCaptureSession *session,
                                            const NesCaptureFrame *frame);
// Stop before an explicit state load, rewind, power cycle, or image switch.
// Finalization retains all complete frames; partial emulation is not appended.
NesFileResult nes_capture_session_stop(NesCaptureSession *session);
void nes_capture_session_discard(NesCaptureSession *session);

#endif
