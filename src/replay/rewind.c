/*
 * rewind.c - Bounded rewind history and speculative run-ahead
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "rewind.h"
#include "../system/execution_policy.h"
#include "../system/vs_system.h"
#include "../video/video_trace.h"

#if defined(__has_include)
#  if __has_include("../debugger/lua_runtime.h")
#    include "../debugger/lua_runtime.h"
#    define CUPID_REPLAY_HAS_LUA 1
#  endif
#endif
#ifndef CUPID_REPLAY_HAS_LUA
#define CUPID_REPLAY_HAS_LUA 0
#endif

#include <stdlib.h>
#include <string.h>

enum {
    RUNAHEAD_PRESENTATION_MAX_WIDTH = 512,
    RUNAHEAD_PRESENTATION_HEIGHT = 240
};

static uint32_t runahead_presentation[RUNAHEAD_PRESENTATION_MAX_WIDTH
                                      * RUNAHEAD_PRESENTATION_HEIGHT];
static unsigned runahead_presentation_width;
static bool runahead_presentation_valid;

static void set_state_result(NesStateResult *out, NesStateResult result) {
    if (out) *out = result;
}

static bool live_timeline(void) {
    return nes_execution_policy() == NES_EXECUTION_LIVE;
}

void nes_runahead_clear_presented_frame(void) {
    runahead_presentation_valid = false;
    runahead_presentation_width = 0;
}

const uint32_t *nes_runahead_presented_frame(unsigned *width, unsigned *height) {
    if (width) *width = runahead_presentation_valid ? runahead_presentation_width : 0;
    if (height) *height = runahead_presentation_valid ? RUNAHEAD_PRESENTATION_HEIGHT : 0;
    return runahead_presentation_valid ? runahead_presentation : NULL;
}

bool nes_replay_host_state_supported(void) {
#if CUPID_REPLAY_HAS_LUA
    if (debugger_lua_loaded()) return false;
#endif
    return true;
}

static void free_entry(NesStateBlob *entry) {
    if (entry) nes_state_blob_free(entry);
}

void nes_rewind_init(NesRewindHistory *history) {
    if (!history) return;
    memset(history, 0, sizeof(*history));
}

void nes_rewind_clear(NesRewindHistory *history) {
    if (!history || !history->entries || !history->capacity) {
        if (history) {
            history->count = 0;
            history->head = 0;
            history->total_bytes = 0;
        }
        return;
    }
    for (size_t i = 0; i < history->count; ++i) {
        size_t index = (history->head + i) % history->capacity;
        free_entry(&history->entries[index]);
    }
    history->count = 0;
    history->head = 0;
    history->total_bytes = 0;
}

void nes_rewind_destroy(NesRewindHistory *history) {
    if (!history) return;
    nes_rewind_clear(history);
    free(history->entries);
    memset(history, 0, sizeof(*history));
}

bool nes_rewind_configure(NesRewindHistory *history, size_t max_frames,
                          size_t max_bytes) {
    if (!history || max_frames > NES_REWIND_MAX_FRAMES) return false;
    if (max_frames && !max_bytes) return false;

    NesStateBlob *replacement = NULL;
    if (max_frames) {
        if (max_frames > SIZE_MAX / sizeof(*replacement)) return false;
        replacement = (NesStateBlob *)calloc(max_frames, sizeof(*replacement));
        if (!replacement) return false;
    }

    nes_rewind_clear(history);
    free(history->entries);
    history->entries = replacement;
    history->capacity = max_frames;
    history->max_bytes = max_bytes;
    return true;
}

size_t nes_rewind_count(const NesRewindHistory *history) {
    return history ? history->count : 0;
}

size_t nes_rewind_capacity(const NesRewindHistory *history) {
    return history ? history->capacity : 0;
}

size_t nes_rewind_bytes(const NesRewindHistory *history) {
    return history ? history->total_bytes : 0;
}

static void discard_oldest(NesRewindHistory *history) {
    if (!history || !history->count || !history->capacity) return;
    NesStateBlob *entry = &history->entries[history->head];
    if (history->total_bytes >= entry->size) history->total_bytes -= entry->size;
    else history->total_bytes = 0;
    free_entry(entry);
    history->head = (history->head + 1) % history->capacity;
    history->count--;
}

NesReplayResult nes_rewind_capture(NesRewindHistory *history,
                                   NesStateResult *state_result) {
    set_state_result(state_result, NES_STATE_OK);
    if (!history || !history->capacity || !history->max_bytes)
        return NES_REPLAY_DISABLED;
    if (!live_timeline()) return NES_REPLAY_CONFLICT;
    if (!nes_replay_host_state_supported()) return NES_REPLAY_UNSUPPORTED_HOST_STATE;

    NesStateBlob captured = {0};
    NesStateResult state = nes_state_capture(&captured);
    if (state != NES_STATE_OK) {
        set_state_result(state_result, state);
        return NES_REPLAY_STATE_ERROR;
    }
    if (captured.size > history->max_bytes) {
        nes_state_blob_free(&captured);
        set_state_result(state_result, NES_STATE_ERROR_UNSUPPORTED);
        return NES_REPLAY_STATE_ERROR;
    }

    while (history->count &&
           (history->count >= history->capacity
            || captured.size > history->max_bytes - history->total_bytes)) {
        discard_oldest(history);
    }

    size_t index = (history->head + history->count) % history->capacity;
    history->entries[index] = captured;
    history->total_bytes += captured.size;
    history->count++;
    return NES_REPLAY_OK;
}

NesReplayResult nes_rewind_step(NesRewindHistory *history,
                                NesStateResult *state_result) {
    set_state_result(state_result, NES_STATE_OK);
    if (!history || !history->capacity) return NES_REPLAY_DISABLED;
    if (!live_timeline()) return NES_REPLAY_CONFLICT;
    if (!nes_replay_host_state_supported()) return NES_REPLAY_UNSUPPORTED_HOST_STATE;
    if (!history->count) return NES_REPLAY_EMPTY;

    size_t index = (history->head + history->count - 1) % history->capacity;
    NesStateBlob *entry = &history->entries[index];
    uint32_t previous_policy = nes_execution_policy();
    if (!nes_execution_set_policy(previous_policy | NES_EXECUTION_REWIND))
        return NES_REPLAY_CONFLICT;
    NesStateResult state = nes_state_restore(entry->data, entry->size);
    (void)nes_execution_set_policy(previous_policy);
    if (state != NES_STATE_OK) {
        set_state_result(state_result, state);
        return NES_REPLAY_STATE_ERROR;
    }

    nes_runahead_clear_presented_frame();
    nes_video_trace_reset_side(0);
    nes_video_trace_reset_side(1);
    if (history->total_bytes >= entry->size) history->total_bytes -= entry->size;
    else history->total_bytes = 0;
    free_entry(entry);
    history->count--;
    if (!history->count) history->head = 0;
    return NES_REPLAY_OK;
}

NesReplayResult nes_runahead_execute(unsigned run_ahead_frames,
                                     NesReplayFrameRunner runner,
                                     void *userdata,
                                     NesStateResult *state_result) {
    set_state_result(state_result, NES_STATE_OK);
    nes_runahead_clear_presented_frame();
    if (!runner) return NES_REPLAY_FRAME_ERROR;
    if (!live_timeline()) return NES_REPLAY_CONFLICT;
    if (!nes_replay_host_state_supported()) return NES_REPLAY_UNSUPPORTED_HOST_STATE;

    if (!runner(userdata)) return NES_REPLAY_FRAME_ERROR;
    if (!run_ahead_frames) return NES_REPLAY_OK;

    NesStateBlob authoritative = {0};
    NesStateResult state = nes_state_capture(&authoritative);
    if (state != NES_STATE_OK) {
        set_state_result(state_result, state);
        return NES_REPLAY_STATE_ERROR;
    }

    NesStateRestore *restore = NULL;
    state = nes_state_prepare_restore(authoritative.data, authoritative.size, &restore);
    nes_state_blob_free(&authoritative);
    if (state != NES_STATE_OK) {
        set_state_result(state_result, state);
        return NES_REPLAY_STATE_ERROR;
    }

    uint32_t previous_policy = nes_execution_policy();
    if (!nes_execution_set_policy(previous_policy | NES_EXECUTION_SPECULATIVE)) {
        nes_state_restore_free(restore);
        return NES_REPLAY_CONFLICT;
    }

    NesReplayResult result = NES_REPLAY_OK;
    for (unsigned i = 0; i < run_ahead_frames; ++i) {
        if (!runner(userdata)) {
            result = NES_REPLAY_FRAME_ERROR;
            break;
        }
    }

    unsigned presentation_width = vs_video_width();
    size_t presentation_pixels = (size_t)presentation_width * RUNAHEAD_PRESENTATION_HEIGHT;
    if (result == NES_REPLAY_OK) {
        if (presentation_width > RUNAHEAD_PRESENTATION_MAX_WIDTH
            || !vs_video_copy_frame(runahead_presentation, presentation_pixels)) {
            result = NES_REPLAY_STATE_ERROR;
            set_state_result(state_result, NES_STATE_ERROR_UNSUPPORTED);
        } else {
            runahead_presentation_width = presentation_width;
            runahead_presentation_valid = true;
        }
    }

    state = nes_state_apply_prepared(restore);
    (void)nes_execution_set_policy(previous_policy);
    nes_state_restore_free(restore);
    if (state != NES_STATE_OK) {
        nes_runahead_clear_presented_frame();
        set_state_result(state_result, state);
        return NES_REPLAY_STATE_ERROR;
    }
    if (result != NES_REPLAY_OK) nes_runahead_clear_presented_frame();
    return result;
}

const char *nes_replay_result_string(NesReplayResult result) {
    switch (result) {
        case NES_REPLAY_OK: return "ok";
        case NES_REPLAY_DISABLED: return "disabled";
        case NES_REPLAY_EMPTY: return "rewind history is empty";
        case NES_REPLAY_CONFLICT: return "execution mode conflict";
        case NES_REPLAY_UNSUPPORTED_HOST_STATE: return "active host state cannot be replayed safely";
        case NES_REPLAY_STATE_ERROR: return "save-state operation failed";
        case NES_REPLAY_FRAME_ERROR: return "frame execution failed";
        default: return "unknown replay result";
    }
}
