/*
 * history_view.c - Non-destructive inspection of authoritative rewind snapshots
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "history_view.h"
#include "frame_snapshot.h"
#include "../system/execution_policy.h"
#include "../system/vs_system.h"
#include "../util/file_io.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint32_t pixels[256 * 240];
    uint16_t signal[256 * 240];
    unsigned phase;
    uint64_t frame;
    bool valid;
} HistoryVideoBackup;

static const NesStateBlob *entry_at(const NesRewindHistory *history, size_t index) {
    if (!history || !history->entries || !history->capacity || history->count > history->capacity ||
        history->head >= history->capacity || index >= history->count) {
        return NULL;
    }
    return &history->entries[(history->head + index) % history->capacity];
}

static NesReplayResult allowed(const NesRewindHistory *history, size_t index) {
    if (nes_execution_policy() != NES_EXECUTION_LIVE) {
        return NES_REPLAY_CONFLICT;
    }
    if (!nes_replay_host_state_supported()) {
        return NES_REPLAY_UNSUPPORTED_HOST_STATE;
    }
    return entry_at(history, index) ? NES_REPLAY_OK : NES_REPLAY_EMPTY;
}

NesReplayResult nes_history_preview(const NesRewindHistory *history, size_t index, uint32_t *pixels, size_t capacity,
                                    unsigned *width, NesStateResult *state) {
    if (state) {
        *state = NES_STATE_OK;
    }
    NesReplayResult permitted = allowed(history, index);
    if (permitted != NES_REPLAY_OK) {
        return permitted;
    }
    if (!pixels || !width || capacity < 512u * 240u) {
        if (state) {
            *state = NES_STATE_ERROR_ARGUMENT;
        }
        return NES_REPLAY_STATE_ERROR;
    }
    NesStateBlob live = {0};
    HistoryVideoBackup *video = calloc(2, sizeof(*video));
    if (!video) {
        if (state) {
            *state = NES_STATE_ERROR_OUT_OF_MEMORY;
        }
        return NES_REPLAY_STATE_ERROR;
    }
    for (unsigned side = 0; side < 2; ++side) {
        const NesCompletedVideoFrame *frame = nes_video_snapshot_frame(side);
        if (!frame) {
            continue;
        }
        memcpy(video[side].pixels, frame->pixels, sizeof(video[side].pixels));
        memcpy(video[side].signal, frame->signal, sizeof(video[side].signal));
        video[side].phase = frame->phase;
        video[side].frame = frame->frame_number;
        video[side].valid = true;
    }
    NesStateRestore *restore = NULL;
    NesStateResult result = nes_state_capture(&live);
    if (result == NES_STATE_OK) {
        result = nes_state_prepare_restore(live.data, live.size, &restore);
    }
    nes_state_blob_free(&live);
    if (result != NES_STATE_OK) {
        free(video);
        if (state) {
            *state = result;
        }
        return NES_REPLAY_STATE_ERROR;
    }
    if (!nes_execution_set_policy(NES_EXECUTION_REWIND)) {
        free(video);
        nes_state_restore_free(restore);
        return NES_REPLAY_CONFLICT;
    }
    const NesStateBlob *entry = entry_at(history, index);
    result = nes_state_restore(entry->data, entry->size);
    bool copied = false;
    unsigned historical_width = 0;
    if (result == NES_STATE_OK) {
        historical_width = vs_video_width();
        copied = historical_width <= 512 && vs_video_copy_completed_frame(pixels, capacity);
    }
    NesStateResult restored = nes_state_apply_prepared(restore);
    if (restored == NES_STATE_OK) {
        for (unsigned side = 0; side < 2; ++side) {
            if (video[side].valid) {
                nes_video_snapshot_complete(side, video[side].pixels, video[side].signal, video[side].phase,
                                            video[side].frame);
            }
        }
    }
    free(video);
    nes_state_restore_free(restore);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    if (restored != NES_STATE_OK) {
        result = restored;
    }
    if (state) {
        *state = result;
    }
    if (result != NES_STATE_OK) {
        return NES_REPLAY_STATE_ERROR;
    }
    if (!copied) {
        return NES_REPLAY_FRAME_ERROR;
    }
    *width = historical_width;
    return NES_REPLAY_OK;
}

NesReplayResult nes_history_resume(NesRewindHistory *history, size_t index, NesStateResult *state) {
    if (state) {
        *state = NES_STATE_OK;
    }
    NesReplayResult permitted = allowed(history, index);
    if (permitted != NES_REPLAY_OK) {
        return permitted;
    }
    const NesStateBlob *entry = entry_at(history, index);
    if (!nes_execution_set_policy(NES_EXECUTION_REWIND)) {
        return NES_REPLAY_CONFLICT;
    }
    NesStateResult result = nes_state_restore(entry->data, entry->size);
    (void)nes_execution_set_policy(NES_EXECUTION_LIVE);
    if (state) {
        *state = result;
    }
    if (result != NES_STATE_OK) {
        return NES_REPLAY_STATE_ERROR;
    }
    while (history->count > index) {
        NesStateBlob *last = &history->entries[(history->head + history->count - 1) % history->capacity];
        history->total_bytes -= last->size;
        nes_state_blob_free(last);
        --history->count;
    }
    if (!history->count) {
        history->head = 0;
    }
    nes_runahead_clear_presented_frame();
    nes_video_trace_reset_side(0);
    nes_video_trace_reset_side(1);
    return NES_REPLAY_OK;
}

NesStateResult nes_history_save(const NesRewindHistory *history, size_t index, const char *path) {
    if (allowed(history, index) != NES_REPLAY_OK || !path || !*path) {
        return NES_STATE_ERROR_ARGUMENT;
    }
    const NesStateBlob *entry = entry_at(history, index);
    NesStateRestore *validated = NULL;
    NesStateResult result = nes_state_prepare_restore(entry->data, entry->size, &validated);
    nes_state_restore_free(validated);
    if (result != NES_STATE_OK) {
        return result;
    }
    return nes_file_write_atomic(path, entry->data, entry->size) == NES_FILE_OK ? NES_STATE_OK : NES_STATE_ERROR_IO;
}
