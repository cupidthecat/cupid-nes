/*
 * tas_timeline.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Movie cursor and bounded executable checkpoints. SPDX-License-Identifier: GPL-3.0-or-later */
#include "tas_session_internal.h"
#include "../joypad/joypad.h"
#include "../system/execution_policy.h"
#include "../video/video_trace.h"
#include <stdlib.h>
#include <string.h>

static void remove_checkpoint(NesTasSession *session, size_t index) {
    session->checkpoint_bytes -= session->checkpoints[index].state.size;
    nes_state_blob_free(&session->checkpoints[index].state);
    --session->checkpoint_count;
    memmove(session->checkpoints + index, session->checkpoints + index + 1,
            (session->checkpoint_count - index) * sizeof(*session->checkpoints));
    memset(session->checkpoints + session->checkpoint_count, 0, sizeof(*session->checkpoints));
}

void tas_clear_checkpoints(NesTasSession *session) {
    while (session->checkpoint_count) {
        remove_checkpoint(session, session->checkpoint_count - 1);
    }
}

void tas_invalidate_after(NesTasSession *session, size_t frame) {
    for (size_t i = 0; i < session->checkpoint_count;) {
        if (session->checkpoints[i].frame > frame) {
            remove_checkpoint(session, i);
        } else {
            ++i;
        }
    }
}

void tas_capture_checkpoint(NesTasSession *session) {
    if (!session->frame || session->frame_in_progress || !session->cache_limit ||
        session->frame % session->cache_interval) {
        return;
    }
    for (size_t i = 0; i < session->checkpoint_count; ++i) {
        if (session->checkpoints[i].frame == session->frame) {
            return;
        }
    }
    NesStateBlob state = {0};
    if (nes_state_capture(&state) != NES_STATE_OK) {
        return;
    }
    if (state.size > session->cache_limit) {
        nes_state_blob_free(&state);
        return;
    }
    while (session->checkpoint_count && (session->checkpoint_count == TAS_CHECKPOINT_SLOTS ||
                                         state.size > session->cache_limit - session->checkpoint_bytes)) {
        remove_checkpoint(session, 0);
    }
    TasCheckpoint *entry = &session->checkpoints[session->checkpoint_count++];
    *entry = (TasCheckpoint){state, session->frame, session->lag_count, session->selected_side};
    session->checkpoint_bytes += state.size;
}

NesMovieResult tas_restore_checkpoint(NesTasSession *session, size_t target) {
    const NesStateBlob *state = &session->initial;
    size_t frame = 0, lag_count = 0, selected_side = 0;
    for (size_t i = 0; i < session->checkpoint_count; ++i) {
        const TasCheckpoint *entry = &session->checkpoints[i];
        if (entry->frame <= target && entry->frame >= frame) {
            state = &entry->state;
            frame = entry->frame;
            lag_count = entry->lag_count;
            selected_side = entry->selected_side;
        }
    }
    NesStateResult result = nes_state_restore(state->data, state->size);
    if (result != NES_STATE_OK) {
        return tas_result(session,
                          result == NES_STATE_ERROR_OUT_OF_MEMORY ? NES_MOVIE_OUT_OF_MEMORY : NES_MOVIE_STATE_ERROR,
                          nes_state_result_string(result));
    }
    session->frame = frame;
    session->lag_count = lag_count;
    session->selected_side = selected_side;
    session->frame_in_progress = session->completed_pending = session->pending_record = false;
    session->pending_commands = 0;
    nes_video_trace_reset_side(0);
    nes_video_trace_reset_side(1);
    return tas_result(session, NES_MOVIE_OK, NULL);
}

NesMovieResult nes_tas_session_set_cache(NesTasSession *session, size_t bytes, unsigned interval) {
    if (!session || !interval || interval > 3600 || bytes > (size_t)1024 * 1024 * 1024) {
        return tas_result(session, NES_MOVIE_INVALID_ARGUMENT, NULL);
    }
    session->cache_limit = bytes;
    session->cache_interval = interval;
    while (session->checkpoint_count && session->checkpoint_bytes > bytes) {
        remove_checkpoint(session, 0);
    }
    return tas_result(session, NES_MOVIE_OK, NULL);
}

static size_t cache_valid_through(const NesTasSession *session) {
    NesTasChange change = nes_tas_project_change_since(session->project, session->seen_revision);
    size_t frames = nes_tas_project_frame_count(session->project);
    return change.first_changed_frame < frames ? change.first_changed_frame : frames;
}

void nes_tas_session_cache_info(const NesTasSession *session, NesTasCacheInfo *out) {
    if (!out) {
        return;
    }
    *out = (NesTasCacheInfo){0};
    if (!session) {
        return;
    }
    out->byte_limit = session->cache_limit;
    out->interval = session->cache_interval;
    if (!session->active) {
        return;
    }
    out->initial_bytes = session->initial.size;
    size_t valid_through = cache_valid_through(session);
    for (size_t i = 0; i < session->checkpoint_count; ++i) {
        const TasCheckpoint *entry = &session->checkpoints[i];
        if (entry->frame <= valid_through) {
            ++out->checkpoint_count;
            out->checkpoint_bytes += entry->state.size;
        }
    }
}

bool nes_tas_session_cache_entry(const NesTasSession *session, size_t index, NesTasCacheEntry *out) {
    if (out) {
        *out = (NesTasCacheEntry){0};
    }
    if (!session || !session->active || !out) {
        return false;
    }
    size_t valid_through = cache_valid_through(session);
    for (size_t i = 0; i < session->checkpoint_count; ++i) {
        const TasCheckpoint *entry = &session->checkpoints[i];
        if (entry->frame > valid_through) {
            continue;
        }
        if (index-- == 0) {
            *out = (NesTasCacheEntry){entry->frame, entry->state.size, entry->lag_count, false};
            return true;
        }
    }
    return false;
}

bool nes_tas_session_cache_before(const NesTasSession *session, size_t frame, NesTasCacheEntry *out) {
    if (out) {
        *out = (NesTasCacheEntry){0};
    }
    if (!session || !session->active || !session->initial.data || !out ||
        frame > nes_tas_project_frame_count(session->project)) {
        return false;
    }
    *out = (NesTasCacheEntry){0, session->initial.size, 0, true};
    size_t valid_through = cache_valid_through(session);
    for (size_t i = 0; i < session->checkpoint_count; ++i) {
        const TasCheckpoint *entry = &session->checkpoints[i];
        if (entry->frame <= frame && entry->frame <= valid_through && entry->frame > out->frame) {
            *out = (NesTasCacheEntry){entry->frame, entry->state.size, entry->lag_count, false};
        }
    }
    return true;
}

NesMovieResult nes_tas_session_clear_cache(NesTasSession *session, size_t after_frame) {
    if (!session || !session->active) {
        return tas_result(session, NES_MOVIE_CONFLICT, NULL);
    }
    if (after_frame > nes_tas_project_frame_count(session->project)) {
        return tas_result(session, NES_MOVIE_INVALID_ARGUMENT, "Frame is outside the movie timeline.");
    }
    tas_invalidate_after(session, after_frame);
    return tas_result(session, NES_MOVIE_OK, NULL);
}

NesMovieResult nes_tas_session_reconcile(NesTasSession *session) {
    if (!session || !session->active) {
        return tas_result(session, NES_MOVIE_CONFLICT, NULL);
    }
    NesTasChange change = nes_tas_project_change_since(session->project, session->seen_revision);
    if (change.revision == session->seen_revision) {
        return NES_MOVIE_OK;
    }
    if (change.first_changed_frame != SIZE_MAX) {
        tas_invalidate_after(session, change.first_changed_frame);
        if (change.first_changed_frame < session->frame ||
            (change.first_changed_frame == session->frame && session->frame_in_progress)) {
            size_t target = session->seeking ? session->seek_target : session->frame;
            size_t count = nes_tas_project_frame_count(session->project);
            if (target > count) {
                target = count;
            }
            size_t from = change.first_changed_frame < target ? change.first_changed_frame : target;
            NesMovieResult restored = tas_restore_checkpoint(session, from);
            if (restored != NES_MOVIE_OK) {
                return restored;
            }
            session->seek_target = target;
            session->seeking = session->frame < target;
        }
    }
    session->seen_revision = change.revision;
    return NES_MOVIE_OK;
}

NesMovieResult nes_tas_session_seek(NesTasSession *session, size_t frame) {
    if (!session || !session->active) {
        return tas_result(session, NES_MOVIE_CONFLICT, NULL);
    }
    if (frame > nes_tas_project_frame_count(session->project)) {
        return tas_result(session, NES_MOVIE_INVALID_ARGUMENT, "Frame is outside the movie timeline.");
    }
    NesMovieResult finished = tas_finish_take(session);
    if (finished != NES_MOVIE_OK) {
        return finished;
    }
    session->recording = false;
    session->pending_commands = 0;
    NesMovieResult reconciled = nes_tas_session_reconcile(session);
    if (reconciled != NES_MOVIE_OK) {
        return reconciled;
    }
    if (frame < session->frame || session->frame_in_progress) {
        NesMovieResult restored = tas_restore_checkpoint(session, frame);
        if (restored != NES_MOVIE_OK) {
            return restored;
        }
    } else {
        size_t latest = session->frame;
        for (size_t i = 0; i < session->checkpoint_count; ++i) {
            if (session->checkpoints[i].frame <= frame && session->checkpoints[i].frame > latest) {
                latest = session->checkpoints[i].frame;
            }
        }
        if (latest > session->frame) {
            NesMovieResult restored = tas_restore_checkpoint(session, frame);
            if (restored != NES_MOVIE_OK) {
                return restored;
            }
        }
    }
    session->seek_target = frame;
    session->seeking = session->frame < frame;
    return tas_result(session, NES_MOVIE_OK, NULL);
}

void nes_tas_session_cancel_seek(NesTasSession *session) {
    if (session) {
        session->seeking = false;
        session->seek_target = session->frame;
    }
}

NesMovieResult nes_tas_session_frame_boundary(NesTasSession *session) {
    if (!session || !session->active) {
        return tas_result(session, NES_MOVIE_CONFLICT, NULL);
    }
    if (session->completed_pending) {
        NesMovieResult committed = nes_tas_session_frame_complete(session, true);
        if (committed != NES_MOVIE_OK) {
            return committed;
        }
    }
    NesMovieResult reconciled = nes_tas_session_reconcile(session);
    if (reconciled != NES_MOVIE_OK) {
        return reconciled;
    }
    if (session->frame_in_progress) {
        return NES_MOVIE_OK;
    }
    const NesFm2Frame *recorded = nes_tas_project_frame(session->project, session->frame);
    session->pending_record = session->recording && !session->seeking;
    if (!recorded && !session->pending_record) {
        return tas_result(session, NES_MOVIE_COMPLETE, NULL);
    }
    tas_capture_checkpoint(session);
    session->pending_input = recorded ? *recorded : (NesFm2Frame){0};
    if (session->pending_record) {
        session->pending_input.commands |= session->pending_commands;
        for (unsigned player = 0; player < 4; ++player) {
            if (!(session->record_players & (1u << player))) {
                continue;
            }
            unsigned period = session->fire_on[player] + session->fire_off[player];
            uint8_t fire = period && session->frame % period < session->fire_on[player] ? session->autofire[player] : 0;
            session->pending_input.pads[player] = session->live_pads[player] | session->hold[player] | fire;
            const NesFm2Movie *movie = nes_tas_project_movie(session->project);
            if (player < 2 && movie->ports[player] == NES_FM2_PORT_ZAPPER) {
                const NesFm2Frame *previous =
                    session->frame ? nes_tas_project_frame(session->project, session->frame - 1) : NULL;
                NesFm2Zapper old = previous ? previous->zappers[player] : (NesFm2Zapper){0};
                NesFm2Zapper next = session->live_zappers[player];
                bool pressed = (next.button & 3u) != 0;
                next.bogo = old.bogo ? (uint8_t)(old.bogo - 1) : 0;
                if (pressed && !session->last_zapper_pressed[player]) {
                    next.bogo = 5;
                } else {
                    next.button = old.button;
                }
                session->pending_zapper_pressed[player] = pressed;
                session->pending_input.zappers[player] = next;
            }
        }
    }
    if (!tas_apply_frame(session, &session->pending_input)) {
        return tas_result(session, NES_MOVIE_EVENT_ERROR, NULL);
    }
    session->pending_commands = 0;
    session->input_polls = joypad_poll_count();
    session->frame_in_progress = true;
    return tas_result(session, NES_MOVIE_OK, NULL);
}

NesMovieResult nes_tas_session_frame_complete(NesTasSession *session, bool completed) {
    if (!session || !session->active) {
        return tas_result(session, NES_MOVIE_CONFLICT, NULL);
    }
    if (!session->frame_in_progress || !completed) {
        return NES_MOVIE_OK;
    }
    if (session->pending_record) {
        session->completed_pending = true;
        NesMovieResult result = tas_record_frame(session);
        if (result != NES_MOVIE_OK) {
            return result;
        }
        for (unsigned player = 0; player < 2; ++player) {
            if (session->record_players & (1u << player)) {
                session->last_zapper_pressed[player] = session->pending_zapper_pressed[player];
            }
        }
    }
    bool lagged = joypad_poll_count() == session->input_polls;
    (void)nes_tas_set_lag(session->project, session->frame, lagged ? NES_TAS_LAG_YES : NES_TAS_LAG_NO);
    session->lag_count += lagged;
    ++session->frame;
    session->frame_in_progress = session->completed_pending = session->pending_record = false;
    if (session->seeking && session->frame >= session->seek_target) {
        session->seeking = false;
    }
    return tas_result(session, NES_MOVIE_OK, NULL);
}

NesMovieResult nes_tas_session_bookmark_set(NesTasSession *session, unsigned slot, const char *name) {
    if (!session || !session->active || session->read_only || session->frame_in_progress) {
        return tas_result(session, NES_MOVIE_CONFLICT, NULL);
    }
    NesMovieResult result = tas_finish_take(session);
    if (result != NES_MOVIE_OK) {
        return result;
    }
    return tas_model_result(session, nes_tas_bookmark_set(session->project, slot, session->frame, name));
}

NesMovieResult nes_tas_session_bookmark_load(NesTasSession *session, unsigned slot) {
    if (!session || !session->active || session->read_only || session->frame_in_progress) {
        return tas_result(session, NES_MOVIE_CONFLICT, NULL);
    }
    NesTasBookmarkView bookmark;
    if (!nes_tas_bookmark(session->project, slot, &bookmark) || !bookmark.occupied) {
        return tas_result(session, NES_MOVIE_INVALID_ARGUMENT, "This branch slot is empty.");
    }
    size_t keyframe = bookmark.key_frame;
    NesMovieResult finished = tas_finish_take(session);
    if (finished != NES_MOVIE_OK) {
        return finished;
    }
    NesTasResult edited = nes_tas_edit_begin(session->project);
    if (edited != NES_TAS_OK) {
        return tas_model_result(session, edited);
    }
    edited = nes_tas_bookmark_deploy(session->project, slot);
    if (edited == NES_TAS_OK) {
        edited = nes_tas_increment_rerecord(session->project);
    }
    if (edited != NES_TAS_OK) {
        nes_tas_edit_cancel(session->project);
        return tas_model_result(session, edited);
    }
    edited = nes_tas_edit_end(session->project);
    if (edited != NES_TAS_OK) {
        return tas_model_result(session, edited);
    }
    return nes_tas_session_seek(session, keyframe);
}
