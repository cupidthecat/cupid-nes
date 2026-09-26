/*
 * state_recorder.c - Periodic recovery history with transactional retention
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "state_recorder.h"
#include "recovery_store.h"
#include "output_guard.h"
#include "../system/execution_policy.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool recorder_error(StateRecorder *r, char *error, size_t size, const char *message) {
    if (r) {
        snprintf(r->status, sizeof(r->status), "%s", message);
    }
    if (error && size) {
        snprintf(error, size, "%s", message);
    }
    return false;
}

static bool slot_path(const StateRecorder *r, unsigned slot, char *path, size_t capacity) {
    char suffix[40];
    snprintf(suffix, sizeof(suffix), ".recorder-%02u.cstate", slot);
    return slot <= STATE_RECORDER_MAX && recovery_path(r->directory, r->key, suffix, path, capacity);
}

static bool manifest_path(const StateRecorder *r, char *path, size_t capacity) {
    return recovery_path(r->directory, r->key, ".recorder-index", path, capacity);
}

void state_recorder_init(StateRecorder *r, StateRuntime *state, const char *directory) {
    if (!r) {
        return;
    }
    memset(r, 0, sizeof(*r));
    r->state = state;
    if (directory && strlen(directory) < sizeof(r->directory)) {
        strcpy(r->directory, directory);
    }
    r->options = (StateRecorderOptions){false, false, 3600, 10};
}

bool state_recorder_configure(StateRecorder *r, const StateRecorderOptions *options) {
    if (!r || !options || !options->interval || options->interval > 86400000 || !options->retain ||
        options->retain > STATE_RECORDER_MAX) {
        return false;
    }
    r->options = *options;
    state_recorder_restart_clock(r);
    return true;
}

void state_recorder_restart_clock(StateRecorder *r) {
    if (!r) {
        return;
    }
    r->frames = 0;
    r->milliseconds = 0;
}

/* Delete only files in the reserved namespace that the committed index no longer references. */
static bool cleanup_unused(StateRecorder *r) {
    bool used[STATE_RECORDER_MAX + 1] = {0};
    for (size_t i = 0; i < r->count; ++i) {
        used[r->slots[i]] = true;
    }
    bool ok = true;
    for (unsigned slot = 0; slot <= STATE_RECORDER_MAX; ++slot) {
        char path[4096];
        if (!used[slot] && slot_path(r, slot, path, sizeof(path))) {
            if (!r->state || !frontend_output_path_allowed(path, r->state->execution, r->state->protected_paths,
                                                           r->state->protected_path_count, NULL, 0)) {
                ok = false;
                continue;
            }
            NesFileResult result = nes_file_remove(path);
            if (result != NES_FILE_OK && result != NES_FILE_NOT_FOUND) {
                ok = false;
            }
        }
    }
    return ok;
}

bool state_recorder_image_changed(StateRecorder *r, const FrontendSession *session, char *error, size_t error_size) {
    if (!r) {
        return false;
    }
    r->running = false;
    r->count = 0;
    r->key[0] = 0;
    state_recorder_restart_clock(r);
    if (!session || !session->active) {
        return true;
    }
    if (!recovery_game_key(session, r->key)) {
        return recorder_error(r, error, error_size, "No stable image identity for recovery history");
    }
    char path[4096];
    if (!manifest_path(r, path, sizeof(path))) {
        return false;
    }
    uint8_t *bytes = NULL;
    size_t size = 0;
    NesFileResult result = nes_file_read_all(path, STATE_RECORDER_MAX + 9, &bytes, &size);
    if (result == NES_FILE_OK) {
        bool valid = size >= 9 && !memcmp(bytes, "CUPREC01", 8) && bytes[8] <= STATE_RECORDER_MAX &&
                     size == (size_t)bytes[8] + 9;
        bool seen[STATE_RECORDER_MAX + 1] = {0};
        for (size_t i = 9; valid && i < size; ++i) {
            if (bytes[i] > STATE_RECORDER_MAX || seen[bytes[i]]) {
                valid = false;
            } else {
                seen[bytes[i]] = true;
            }
        }
        if (!valid) {
            free(bytes);
            return recorder_error(r, error, error_size, "Recovery index is corrupt; history was kept");
        }
        r->count = bytes[8];
        for (size_t i = 0; i < r->count; ++i) {
            r->slots[i] = bytes[i + 9];
        }
        free(bytes);
    } else if (result != NES_FILE_NOT_FOUND) {
        return recorder_error(r, error, error_size, nes_file_result_message(result));
    }
    r->running = r->options.automatic;
    if (!cleanup_unused(r)) {
        snprintf(r->status, sizeof(r->status), "History loaded; old snapshot cleanup needs retry");
    } else {
        snprintf(r->status, sizeof(r->status), "%zu recovery snapshots", r->count);
    }
    return true;
}

bool state_recorder_capture(StateRecorder *r, char *error, size_t error_size) {
    if (!r || !r->key[0] || !r->state) {
        return false;
    }
    bool used[STATE_RECORDER_MAX + 1] = {0};
    for (size_t i = 0; i < r->count; ++i) {
        used[r->slots[i]] = true;
    }
    unsigned spare = 0;
    while (spare <= STATE_RECORDER_MAX && used[spare]) {
        ++spare;
    }
    char path[4096], index_path[4096];
    if (!slot_path(r, spare, path, sizeof(path)) || !manifest_path(r, index_path, sizeof(index_path))) {
        return false;
    }
    if (!frontend_output_path_allowed(index_path, r->state->execution, r->state->protected_paths,
                                      r->state->protected_path_count, error, error_size)) {
        r->running = false;
        return false;
    }
    if (!state_runtime_save_path(r->state, path, error, error_size)) {
        r->running = false;
        return recorder_error(r, error, error_size,
                              "Snapshot write failed; recorder stopped and existing history was kept");
    }
    size_t keep = r->count < r->options.retain ? r->count : r->options.retain - 1;
    uint8_t manifest[STATE_RECORDER_MAX + 9];
    memcpy(manifest, "CUPREC01", 8);
    manifest[8] = (uint8_t)(keep + 1);
    for (size_t i = 0; i < keep; ++i) {
        manifest[9 + i] = (uint8_t)r->slots[r->count - keep + i];
    }
    manifest[9 + keep] = (uint8_t)spare;
    NesFileResult result = nes_file_write_atomic(index_path, manifest, keep + 10);
    if (result != NES_FILE_OK) {
        r->running = false;
        return recorder_error(r, error, error_size, "Recovery index write failed; existing history was kept");
    }
    r->count = keep + 1;
    for (size_t i = 0; i < r->count; ++i) {
        r->slots[i] = manifest[9 + i];
    }
    state_recorder_restart_clock(r);
    bool cleaned = cleanup_unused(r);
    snprintf(r->status, sizeof(r->status),
             cleaned ? "Recorded snapshot; %zu retained" : "Recorded snapshot; %zu retained, cleanup will retry",
             r->count);
    if (error && error_size) {
        error[0] = 0;
    }
    return true;
}

bool state_recorder_tick(StateRecorder *r, uint64_t emulated_milliseconds, char *error, size_t error_size) {
    if (!r || !r->running || !r->key[0] || nes_execution_policy() != NES_EXECUTION_LIVE) {
        return true;
    }
    ++r->frames;
    if (UINT64_MAX - r->milliseconds < emulated_milliseconds) {
        r->milliseconds = UINT64_MAX;
    } else {
        r->milliseconds += emulated_milliseconds;
    }
    if ((r->options.time_based ? r->milliseconds : r->frames) < r->options.interval) {
        return true;
    }
    return state_recorder_capture(r, error, error_size);
}

bool state_recorder_restore(StateRecorder *r, size_t index, char *error, size_t error_size) {
    if (!r || index >= r->count) {
        return false;
    }
    char path[4096];
    if (!slot_path(r, r->slots[index], path, sizeof(path)) ||
        !state_runtime_load_path(r->state, path, error, error_size)) {
        return false;
    }
    state_recorder_restart_clock(r);
    return true;
}
