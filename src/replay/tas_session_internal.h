/*
 * tas_session_internal.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Private TAS session state. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_TAS_SESSION_INTERNAL_H
#define CUPID_TAS_SESSION_INTERNAL_H

#include "tas_session.h"
#include "tas_project.h"
#include "input_event.h"
#include "../state/state.h"

enum { TAS_CHECKPOINT_SLOTS = 256 };

#define TAS_DEFAULT_CACHE_BYTES ((size_t)64u * 1024u * 1024u)

typedef struct {
    NesStateBlob state;
    size_t frame;
    size_t lag_count;
    size_t selected_side;
} TasCheckpoint;

struct NesTasSession {
    NesTasProject *project;
    char *path;
    char error[192];
    NesMovieResult last_result;
    NesStateRestore *resume;
    NesStateBlob initial;
    uint32_t previous_policy;
    bool active;
    bool read_only;
    bool recording;
    bool auto_save;
    bool frame_in_progress;
    bool seeking;
    bool take_open;
    bool completed_pending;
    bool pending_record;
    bool take_changed;
    NesFm2Frame pending_input;
    size_t frame;
    size_t seek_target;
    size_t lag_count;
    size_t selected_side;
    uint64_t seen_revision;
    uint64_t saved_revision;
    uint64_t generation;
    uint64_t input_polls;
    unsigned record_players;
    NesTasRecordMode record_mode;
    uint8_t live_pads[4];
    NesFm2Zapper live_zappers[2];
    bool last_zapper_pressed[2];
    bool pending_zapper_pressed[2];
    uint8_t hold[4];
    uint8_t autofire[4];
    unsigned fire_on[4];
    unsigned fire_off[4];
    uint8_t pending_commands;
    TasCheckpoint checkpoints[TAS_CHECKPOINT_SLOTS];
    size_t checkpoint_count;
    size_t checkpoint_bytes;
    size_t cache_limit;
    unsigned cache_interval;
};

NesMovieResult tas_result(NesTasSession *session, NesMovieResult result, const char *message);
NesMovieResult tas_model_result(NesTasSession *session, NesTasResult result);
bool tas_extension(const char *path, const char *extension);
char *tas_copy_path(const char *path);
void tas_clear_checkpoints(NesTasSession *session);
void tas_invalidate_after(NesTasSession *session, size_t frame);
void tas_capture_checkpoint(NesTasSession *session);
NesMovieResult tas_restore_checkpoint(NesTasSession *session, size_t frame);
bool tas_input_observer(const NesInputEvent *event, void *userdata);
NesMovieResult tas_record_frame(NesTasSession *session);
NesMovieResult tas_finish_take(NesTasSession *session);
bool tas_apply_frame(NesTasSession *session, const NesFm2Frame *input);

#endif
