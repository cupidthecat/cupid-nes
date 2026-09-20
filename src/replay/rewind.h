/*
 * rewind.h - Bounded rewind history and speculative run-ahead
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_REWIND_H
#define CUPID_REWIND_H

#include <stdbool.h>
#include <stddef.h>
#include "../state/state.h"
#include "../video/video_trace.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    NES_REWIND_MAX_FRAMES = 3600
};

#define NES_REWIND_DEFAULT_MEMORY_LIMIT ((size_t)256u * 1024u * 1024u)

typedef enum {
    NES_REPLAY_OK = 0,
    NES_REPLAY_DISABLED,
    NES_REPLAY_EMPTY,
    NES_REPLAY_CONFLICT,
    NES_REPLAY_UNSUPPORTED_HOST_STATE,
    NES_REPLAY_STATE_ERROR,
    NES_REPLAY_FRAME_ERROR
} NesReplayResult;

typedef bool (*NesReplayFrameRunner)(void *userdata);

typedef struct {
    NesStateBlob *entries;
    size_t capacity;
    size_t count;
    size_t head;
    size_t total_bytes;
    size_t max_bytes;
} NesRewindHistory;

void nes_rewind_init(NesRewindHistory *history);
void nes_rewind_destroy(NesRewindHistory *history);

/* Reconfiguration keeps no old timeline. Allocation failure leaves the old
 * history and limits unchanged. A zero frame count disables rewind. */
bool nes_rewind_configure(NesRewindHistory *history, size_t max_frames,
                          size_t max_bytes);
void nes_rewind_clear(NesRewindHistory *history);
size_t nes_rewind_count(const NesRewindHistory *history);
size_t nes_rewind_capacity(const NesRewindHistory *history);
size_t nes_rewind_bytes(const NesRewindHistory *history);

/* Capture the current authoritative frame boundary. History capture is
 * intentionally restricted to live execution so movies and netplay have one
 * owner for their timelines. */
NesReplayResult nes_rewind_capture(NesRewindHistory *history,
                                   NesStateResult *state_result);

/* Restore and consume the newest saved frame boundary. */
NesReplayResult nes_rewind_step(NesRewindHistory *history,
                                NesStateResult *state_result);

/* Advance one authoritative frame. When run_ahead_frames is non-zero, retain
 * the video produced by the final speculative frame while restoring the
 * machine to the state immediately after the authoritative frame. */
NesReplayResult nes_runahead_execute(unsigned run_ahead_frames,
                                     NesReplayFrameRunner runner,
                                     void *userdata,
                                     NesStateResult *state_result);

/* The future-view frame is host presentation state. It never replaces the
 * authoritative PPU framebuffer serialized by save states. The pointer stays
 * valid until the next run-ahead call or explicit clear. */
const uint32_t *nes_runahead_presented_frame(unsigned *width, unsigned *height);
/* These signals and tile observations belong to the same speculative frame as
 * the pixels above. They remain valid after restoring the authoritative PPU and
 * after the live trace buffers are reused. A missing tile trace returns NULL. */
const uint16_t *nes_runahead_presented_signal(unsigned side, unsigned *phase);
const NesVideoTraceFrame *nes_runahead_presented_trace(unsigned side);
void nes_runahead_clear_presented_frame(void);
void nes_runahead_shutdown(void);

const char *nes_replay_result_string(NesReplayResult result);
bool nes_replay_host_state_supported(void);

#ifdef __cplusplus
}
#endif
#endif
