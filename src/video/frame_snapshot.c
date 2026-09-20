/*
 * frame_snapshot.c - Retain complete pixels and composite signal together
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "frame_snapshot.h"
#include <string.h>

enum { FRAME_PIXELS = 256 * 240, FRAME_SIDES = 2 };

typedef struct {
    uint32_t pixels[FRAME_PIXELS];
    uint16_t signal[FRAME_PIXELS];
    NesCompletedVideoFrame view;
} FrameStorage;

static FrameStorage completed[FRAME_SIDES];

void nes_video_snapshot_complete(unsigned side, const uint32_t *pixels,
                                 const uint16_t *signal, unsigned phase,
                                 uint64_t frame_number) {
    if (side >= FRAME_SIDES || !pixels || !signal) return;
    FrameStorage *frame = &completed[side];
    memcpy(frame->pixels, pixels, sizeof(frame->pixels));
    memcpy(frame->signal, signal, sizeof(frame->signal));
    frame->view = (NesCompletedVideoFrame){frame->pixels, frame->signal, frame_number, phase};
}

const NesCompletedVideoFrame *nes_video_snapshot_frame(unsigned side) {
    return side < FRAME_SIDES && completed[side].view.pixels ? &completed[side].view : NULL;
}

void nes_video_snapshot_reset(unsigned side) {
    if (side < FRAME_SIDES) memset(&completed[side].view, 0, sizeof(completed[side].view));
}

void nes_video_snapshot_reset_all(void) {
    for (unsigned side = 0; side < FRAME_SIDES; ++side) nes_video_snapshot_reset(side);
}
