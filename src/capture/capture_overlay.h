/* capture_overlay.h - Shared game display and recording overlays
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef NES_CAPTURE_OVERLAY_H
#define NES_CAPTURE_OVERLAY_H
#include "capture_writer.h"
#include "../replay/fm2.h"

enum {
    NES_OVERLAY_INPUT = 1,
    NES_OVERLAY_SUBTITLE = 2,
    NES_OVERLAY_FRAME = 4,
    NES_OVERLAY_STATUS = 8,
    NES_OVERLAY_ALL = 15,
    NES_OVERLAY_TEXT = 512
};

typedef enum {
    NES_OVERLAY_TOP_LEFT,
    NES_OVERLAY_TOP_RIGHT,
    NES_OVERLAY_BOTTOM_LEFT,
    NES_OVERLAY_BOTTOM_RIGHT
} NesOverlayPosition;
typedef struct NesMovieSubtitles NesMovieSubtitles;
/* Compile without changing FM2 metadata. Invalid data leaves the old track
 * untouched. The most recent entry wins; equal starts retain file order. */
NesFileResult nes_movie_subtitles_load(NesMovieSubtitles **track, const NesFm2StringList *records);
void nes_movie_subtitles_free(NesMovieSubtitles **track);
const char *nes_movie_subtitle_at(const NesMovieSubtitles *track, uint64_t frame, unsigned duration);

typedef struct {
    uint8_t pads[6];
    unsigned active_players;
    uint64_t frame;
    bool movie_active;
    char subtitle[NES_OVERLAY_TEXT];
    char status[64];
} NesCaptureOverlayState;

typedef struct {
    void *implementation;
    uint32_t *pixels;
    size_t capacity;
} NesCaptureOverlay;

/* Snapshot only after authoritative input injection/frame completion. This
 * renderer never reads host events or advances the emulated timeline. */
NesFileResult nes_capture_overlay_compose(NesCaptureOverlay *overlay, const NesCaptureFrame *source,
                                          const NesCaptureOverlayState *state, unsigned mask,
                                          NesOverlayPosition position, NesCaptureFrame *output);
void nes_capture_overlay_destroy(NesCaptureOverlay *overlay);
#endif
