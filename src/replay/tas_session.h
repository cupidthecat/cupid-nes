/*
 * tas_session.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* TAS playback and recording. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_REPLAY_TAS_SESSION_H
#define CUPID_REPLAY_TAS_SESSION_H

#include "fm2.h"
#include "movie.h"
#include "../state/state.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct NesTasProject NesTasProject;
typedef struct NesTasSession NesTasSession;

typedef enum { NES_TAS_RECORD_OVERWRITE, NES_TAS_RECORD_INSERT } NesTasRecordMode;

typedef struct {
    bool active;
    bool read_only;
    bool recording;
    bool seeking;
    bool dirty;
    bool frame_in_progress;
    bool auto_save;
    size_t frame;
    size_t total_frames;
    size_t seek_target;
    size_t lag_count;
    size_t checkpoint_count;
    size_t checkpoint_bytes;
    uint64_t rerecord_count;
    unsigned record_players;
    NesTasRecordMode record_mode;
    const char *path;
    const char *error;
    /* Process-local activation identity for views and dialogs. Not serialized. */
    uint64_t generation;
} NesTasProgress;

typedef struct {
    size_t checkpoint_count;
    size_t checkpoint_bytes;
    size_t initial_bytes;
    size_t byte_limit;
    unsigned interval;
} NesTasCacheInfo;

typedef struct {
    size_t frame;
    size_t bytes;
    size_t lag_count;
    bool initial;
} NesTasCacheEntry;

NesTasSession *nes_tas_session_create(void);
void nes_tas_session_destroy(NesTasSession *session);

/* Open starts at the movie's power-on boundary. New uses the loaded game's
 * controller and region settings. Both retain the live machine for Stop. */
NesMovieResult nes_tas_session_open(NesTasSession *session, const char *path);
NesMovieResult nes_tas_session_new(NesTasSession *session, const char *path, bool record);
NesMovieResult nes_tas_session_stop(NesTasSession *session);
/* Explicitly discard unsaved editing work and restore the preceding game. */
NesMovieResult nes_tas_session_discard(NesTasSession *session);
bool nes_tas_session_active(const NesTasSession *session);
NesMovieMode nes_tas_session_mode(const NesTasSession *session);
void nes_tas_session_progress(const NesTasSession *session, NesTasProgress *progress);
const char *nes_tas_session_error(const NesTasSession *session);

NesMovieResult nes_tas_session_frame_boundary(NesTasSession *session);
NesMovieResult nes_tas_session_frame_complete(NesTasSession *session, bool completed);

/* Seeking restores a valid checkpoint immediately, then frame_boundary and
 * frame_complete rebuild the remaining frames. The frontend can cancel or
 * continue drawing between frames. The target names the next input row. */
NesMovieResult nes_tas_session_seek(NesTasSession *session, size_t frame);
void nes_tas_session_cancel_seek(NesTasSession *session);
bool nes_tas_session_seeking(const NesTasSession *session);
NesMovieResult nes_tas_session_reconcile(NesTasSession *session);
NesMovieResult nes_tas_session_set_cache(NesTasSession *session, size_t bytes, unsigned interval);
/* These observational views exclude invalid checkpoints even before reconcile.
 * They never restore state, advance playback, or expose owned state buffers.
 * Entries are in capture order; the startup state is reported separately. */
void nes_tas_session_cache_info(const NesTasSession *session, NesTasCacheInfo *out);
bool nes_tas_session_cache_entry(const NesTasSession *session, size_t index, NesTasCacheEntry *out);
bool nes_tas_session_cache_before(const NesTasSession *session, size_t frame, NesTasCacheEntry *out);
/* Drop checkpoints strictly after this input boundary. Zero clears the entire
 * optional cache; the session's required startup state always remains owned. */
NesMovieResult nes_tas_session_clear_cache(NesTasSession *session, size_t after_frame);

NesTasProject *nes_tas_session_project(NesTasSession *session);
const NesTasProject *nes_tas_session_project_const(const NesTasSession *session);
NesMovieResult nes_tas_session_set_read_only(NesTasSession *session, bool read_only);
NesMovieResult nes_tas_session_set_recording(NesTasSession *session, bool recording, NesTasRecordMode mode,
                                             unsigned players);
/* Queue reset, power, disk or VS commands for the next completed recording row. */
NesMovieResult nes_tas_session_queue_commands(NesTasSession *session, uint8_t commands);
NesMovieResult nes_tas_session_set_hold(NesTasSession *session, unsigned player, uint8_t buttons);
NesMovieResult nes_tas_session_set_autofire(NesTasSession *session, unsigned player, uint8_t buttons,
                                            unsigned on_frames, unsigned off_frames);
uint8_t nes_tas_session_hold(const NesTasSession *session, unsigned player);
uint8_t nes_tas_session_autofire(const NesTasSession *session, unsigned player);

/* Saving is explicit for opened movies. A new recording also saves on Stop.
 * Exporting FM2 never discards the open project's branches or undo history. */
NesMovieResult nes_tas_session_save(NesTasSession *session, const char *path);
NesMovieResult nes_tas_session_set_path(NesTasSession *session, const char *path);
NesMovieResult nes_tas_session_bookmark_set(NesTasSession *session, unsigned slot, const char *name);
NesMovieResult nes_tas_session_bookmark_load(NesTasSession *session, unsigned slot);

/* Movie states bind the cursor and native machine state to the movie GUID,
 * startup state, cheats, and the exact input prefix. They cannot be loaded as
 * unrelated live-game states or after an incompatible earlier input edit. */
NesStateResult nes_tas_session_capture_state(NesTasSession *session, NesStateBlob *out);
NesStateResult nes_tas_session_load_state(NesTasSession *session, const char *path);

#ifdef __cplusplus
}
#endif
#endif
