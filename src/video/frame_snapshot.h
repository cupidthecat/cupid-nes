/*
 * frame_snapshot.h - Completed display frames outside machine state
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_VIDEO_FRAME_SNAPSHOT_H
#define CUPID_VIDEO_FRAME_SNAPSHOT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const uint32_t *pixels;
    const uint16_t *signal;
    uint64_t frame_number;
    unsigned phase;
} NesCompletedVideoFrame;

/* Called at the PPU's frame boundary, before the remaining instruction clocks
 * can render the next frame's first pixels. No emulated state is changed. */
void nes_video_snapshot_complete(unsigned side, const uint32_t *pixels,
                                 const uint16_t *signal, unsigned phase,
                                 uint64_t frame_number);
const NesCompletedVideoFrame *nes_video_snapshot_frame(unsigned side);
void nes_video_snapshot_reset(unsigned side);
void nes_video_snapshot_reset_all(void);

#ifdef __cplusplus
}
#endif

#endif
