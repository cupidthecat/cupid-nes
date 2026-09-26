/*
 * tas_frontend.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Desktop TAS actions. SPDX-License-Identifier: GPL-3.0-or-later */
#include "tas_frontend.h"
#include "fcm_frontend.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "platform_frontend.h"
#include "output_guard.h"
#include "../debugger/debugger.h"
#include "../system/execution_policy.h"
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void error_text(char *error, size_t capacity, const char *text) {
    if (error && capacity) {
        snprintf(error, capacity, "%s", text ? text : "");
    }
}

static bool result(FrontendExecutionRuntime *runtime, NesMovieResult value, char *error, size_t capacity) {
    const char *text = value == NES_MOVIE_OK ? "" : nes_tas_session_error(nes_movie_tas(runtime->movie));
    if (!text[0] && value != NES_MOVIE_OK) {
        text = nes_movie_result_string(value);
    }
    error_text(error, capacity, text);
    if (value != NES_MOVIE_OK) {
        snprintf(runtime->movie_status, sizeof(runtime->movie_status), "%s", text);
    }
    return value == NES_MOVIE_OK;
}

static void pause_restored(FrontendExecutionRuntime *runtime) {
    debugger_reset_session();
    runtime->debugger_pause_revision = debugger_pause_revision();
    execution_control_set_paused(&runtime->execution, true);
    runtime->execution.frame_advance_pending = false;
    frontend_command_set_checked(FRONTEND_COMMAND_PAUSE, true);
    ++runtime->timing_revision;
    if (runtime->restore_handler) {
        runtime->restore_handler(runtime->restore_userdata);
    }
}

static bool before_change(FrontendExecutionRuntime *runtime, char *error, size_t capacity) {
    return !runtime->before_machine_change ||
           runtime->before_machine_change(runtime->machine_change_context, error, capacity);
}

bool tas_frontend_open(FrontendExecutionRuntime *runtime, const char *path, bool create, char *error, size_t capacity) {
    if (!runtime || !runtime->movie || !path || !*path) {
        return false;
    }
    if (strlen(path) >= FRONTEND_MOVIE_PATH_CAPACITY) {
        error_text(error, capacity, "The movie path is too long for this window.");
        return false;
    }
    if (nes_movie_mode(runtime->movie) != NES_MOVIE_IDLE || nes_execution_policy() != NES_EXECUTION_LIVE) {
        error_text(error, capacity, "Stop the current movie or network session before opening another project.");
        return false;
    }
    if (create && !frontend_movie_output_path_allowed(path, runtime, runtime->protected_paths,
                                                      runtime->protected_path_count, error, capacity)) {
        return false;
    }
    if (!before_change(runtime, error, capacity)) {
        return false;
    }
    frontend_execution_begin_machine_change(runtime);
    NesMovieResult opened = create ? nes_tas_session_new(nes_movie_tas(runtime->movie), path, false)
                                   : nes_movie_play_start(runtime->movie, path);
    if (opened == NES_MOVIE_OK) {
        snprintf(runtime->movie_path, sizeof(runtime->movie_path), "%s", path);
        frontend_execution_clear_timeline(runtime);
        runtime->tas_pause_frame = SIZE_MAX;
        pause_restored(runtime);
    }
    frontend_execution_end_machine_change_preserving_audio(runtime);
    if (opened != NES_MOVIE_OK && !create) {
        error_text(error, capacity, nes_movie_error(runtime->movie));
        return false;
    }
    return result(runtime, opened, error, capacity);
}

bool tas_frontend_save(FrontendExecutionRuntime *runtime, const char *path, char *error, size_t capacity) {
    if (!runtime || !path || !*path) {
        return false;
    }
    char selected[FRONTEND_MOVIE_PATH_CAPACITY];
    if (!frontend_parse_dialog_output(path, strlen(path), selected, sizeof(selected), error, capacity)) {
        return false;
    }
    if (!frontend_movie_output_path_allowed(selected, runtime, runtime->protected_paths, runtime->protected_path_count,
                                            error, capacity)) {
        return false;
    }
    frontend_execution_begin_machine_change(runtime);
    NesMovieResult saved = nes_tas_session_save(nes_movie_tas(runtime->movie), selected);
    frontend_execution_end_machine_change_preserving_audio(runtime);
    if (saved == NES_MOVIE_OK) {
        NesTasProgress progress;
        nes_tas_session_progress(nes_movie_tas(runtime->movie), &progress);
        if (progress.path) {
            snprintf(runtime->movie_path, sizeof(runtime->movie_path), "%s", progress.path);
        }
    }
    return result(runtime, saved, error, capacity);
}

bool tas_frontend_seek(FrontendExecutionRuntime *runtime, size_t frame, char *error, size_t capacity) {
    if (!runtime || !runtime->movie) {
        return false;
    }
    if (!before_change(runtime, error, capacity)) {
        return false;
    }
    frontend_execution_begin_machine_change(runtime);
    NesMovieResult seek = nes_tas_session_seek(nes_movie_tas(runtime->movie), frame);
    if (seek == NES_MOVIE_OK) {
        pause_restored(runtime);
    }
    frontend_execution_end_machine_change_preserving_audio(runtime);
    return result(runtime, seek, error, capacity);
}

bool tas_frontend_changed(FrontendExecutionRuntime *runtime, char *error, size_t capacity) {
    if (!runtime || !runtime->movie) {
        return false;
    }
    frontend_execution_begin_machine_change(runtime);
    NesMovieResult changed = nes_tas_session_reconcile(nes_movie_tas(runtime->movie));
    if (changed == NES_MOVIE_OK) {
        pause_restored(runtime);
    }
    frontend_execution_end_machine_change_preserving_audio(runtime);
    return result(runtime, changed, error, capacity);
}

bool tas_frontend_bookmark_load(FrontendExecutionRuntime *runtime, unsigned slot, char *error, size_t capacity) {
    if (!runtime || !before_change(runtime, error, capacity)) {
        return false;
    }
    frontend_execution_begin_machine_change(runtime);
    NesMovieResult loaded = nes_tas_session_bookmark_load(nes_movie_tas(runtime->movie), slot);
    if (loaded == NES_MOVIE_OK) {
        pause_restored(runtime);
    }
    frontend_execution_end_machine_change_preserving_audio(runtime);
    return result(runtime, loaded, error, capacity);
}

bool tas_frontend_discard(FrontendExecutionRuntime *runtime, char *error, size_t capacity) {
    if (!runtime || !before_change(runtime, error, capacity)) {
        return false;
    }
    frontend_execution_begin_machine_change(runtime);
    NesMovieResult discarded = nes_tas_session_discard(nes_movie_tas(runtime->movie));
    if (discarded == NES_MOVIE_OK) {
        frontend_execution_clear_timeline(runtime);
        pause_restored(runtime);
    }
    frontend_execution_end_machine_change_preserving_audio(runtime);
    return result(runtime, discarded, error, capacity);
}

bool tas_frontend_parse_frame(const char *text, size_t *frame) {
    if (!text || !*text || !frame || *text == '-' || *text == '+') {
        return false;
    }
    for (const char *p = text; *p; ++p) {
        if (*p < '0' || *p > '9') {
            return false;
        }
    }
    errno = 0;
    char *end;
    uintmax_t value = strtoumax(text, &end, 10);
    if (errno || *end || value > SIZE_MAX) {
        return false;
    }
    *frame = (size_t)value;
    return true;
}

static bool back(void *context, char *error, size_t capacity) {
    FrontendExecutionRuntime *runtime = context;
    NesTasProgress progress;
    nes_tas_session_progress(nes_movie_tas(runtime->movie), &progress);
    return tas_frontend_seek(runtime, progress.frame ? progress.frame - 1 : 0, error, capacity);
}

static bool panel_snapshot(void *context, FrontendPanelModel *model, char *error, size_t capacity) {
    (void)error;
    (void)capacity;
    FrontendExecutionRuntime *runtime = context;
    NesTasProgress progress;
    nes_tas_session_progress(nes_movie_tas(runtime->movie), &progress);
    static char status[192];
    snprintf(status, sizeof(status), "%s | Frame %zu/%zu | Lag %zu | Rerecords %" PRIu64 " | Cache %.1f MiB",
             progress.recording ? "Recording"
             : progress.active  ? "Playback"
                                : "Closed",
             progress.frame, progress.total_frames, progress.lag_count, progress.rerecord_count,
             (double)progress.checkpoint_bytes / (1024 * 1024));
    model->status = status;

    const struct {
        unsigned id;
        const char *label;
        bool enabled;
    } actions[] = {{TAS_CONTROL_OPEN, "Open movie or project...", !progress.active},
                   {TAS_CONTROL_NEW, "New TAS project...", !progress.active},
                   {TAS_CONTROL_SAVE, "Save", progress.active},
                   {TAS_CONTROL_SAVE_AS, "Save as / Export...", progress.active},
                   {TAS_CONTROL_STOP, "Close movie and restore game", progress.active}};

    for (size_t i = 0; i < sizeof(actions) / sizeof(actions[0]); ++i) {
        FrontendPanelControl control = {.id = actions[i].id,
                                        .type = FRONTEND_PANEL_ACTION,
                                        .label = actions[i].label,
                                        .enabled = actions[i].enabled};
        if (!frontend_panel_add_control(model, &control)) {
            return false;
        }
    }
    FrontendPanelControl readonly = {.id = TAS_CONTROL_READ_ONLY,
                                     .type = FRONTEND_PANEL_CHECKBOX,
                                     .label = "Read-only playback",
                                     .selected = progress.read_only,
                                     .enabled = progress.active};
    return frontend_panel_add_control(model, &readonly);
}

static bool panel_action(void *context, unsigned id, const char *value, int selected, char *error, size_t capacity) {
    FrontendExecutionRuntime *runtime = context;
    NesTasSession *session = nes_movie_tas(runtime->movie);
    NesTasProgress progress;
    nes_tas_session_progress(session, &progress);
    char path[FRONTEND_MOVIE_PATH_CAPACITY] = {0};
    switch (id) {
    case TAS_CONTROL_OPEN:
    case TAS_CONTROL_NEW:
        if (value && *value) {
            snprintf(path, sizeof(path), "%s", value);
        } else if (!(id == TAS_CONTROL_OPEN
                         ? frontend_open_movie_dialog(path, sizeof(path), error, capacity)
                         : frontend_save_file_dialog(FRONTEND_SAVE_TAS_PROJECT, path, sizeof(path), error, capacity))) {
            return false;
        }
        return tas_frontend_open(runtime, path, id == TAS_CONTROL_NEW, error, capacity);
    case TAS_CONTROL_SAVE:
        return progress.path && tas_frontend_save(runtime, progress.path, error, capacity);
    case TAS_CONTROL_SAVE_AS:
        if (value && *value) {
            snprintf(path, sizeof(path), "%s", value);
        } else if (!frontend_save_file_dialog(FRONTEND_SAVE_TAS_PROJECT, path, sizeof(path), error, capacity)) {
            return false;
        }
        return tas_frontend_save(runtime, path, error, capacity);
    case TAS_CONTROL_STOP:
        return frontend_execution_movie_stop(runtime, error, capacity);
    case TAS_CONTROL_DISCARD:
        return tas_frontend_discard(runtime, error, capacity);
    case TAS_CONTROL_READ_ONLY:
        return result(runtime, nes_tas_session_set_read_only(session, selected != 0), error, capacity);
    case TAS_CONTROL_RECORD:
        return result(
            runtime,
            nes_tas_session_set_recording(session, selected != 0, progress.record_mode, progress.record_players), error,
            capacity);
    case TAS_CONTROL_RECORD_MODE:
        return result(runtime,
                      nes_tas_session_set_recording(session, progress.recording, (NesTasRecordMode)selected,
                                                    progress.record_players),
                      error, capacity);
    case TAS_CONTROL_PLAYERS:
        return result(
            runtime,
            nes_tas_session_set_recording(session, progress.recording, progress.record_mode, (unsigned)selected), error,
            capacity);
    case TAS_CONTROL_SEEK: {
        size_t frame;
        if (!tas_frontend_parse_frame(value, &frame)) {
            error_text(error, capacity, "Enter a frame number.");
            return false;
        }
        return tas_frontend_seek(runtime, frame, error, capacity);
    }
    case TAS_CONTROL_SKIP_LAG:
        runtime->tas_skip_lag = selected != 0;
        return true;
    case TAS_CONTROL_PAUSE_FRAME:
        if (!tas_frontend_parse_frame(value, &runtime->tas_pause_frame)) {
            error_text(error, capacity, "Enter a frame number.");
            return false;
        }
        return true;
    default:
        error_text(error, capacity, "Unknown TAS action.");
        return false;
    }
}

bool tas_frontend_register(FrontendExecutionRuntime *runtime) {
    FrontendPanelSpec panel = {.id = TAS_PANEL,
                               .title = "TAS Editor",
                               .category = "Tools",
                               .flags = FRONTEND_PANEL_NEEDS_SESSION,
                               .snapshot = panel_snapshot,
                               .action = panel_action,
                               .userdata = runtime};
    FrontendCommandSpec step = {.id = TAS_COMMAND_FRAME_BACK,
                                .label = "TAS Frame Back",
                                .menu = "Emulation",
                                .shortcut = "",
                                .flags = FRONTEND_COMMAND_NEEDS_SESSION,
                                .handler = back,
                                .userdata = runtime};
    if (!frontend_panel_register(&panel)) {
        return false;
    }
    if (!frontend_command_register(&step)) {
        frontend_panel_unregister(TAS_PANEL);
        return false;
    }
    if (!fcm_frontend_register(runtime)) {
        frontend_panel_unregister(TAS_PANEL);
        frontend_command_unregister(TAS_COMMAND_FRAME_BACK);
        return false;
    }
    return true;
}

void tas_frontend_unregister(void) {
    fcm_frontend_unregister();
    frontend_panel_unregister(TAS_PANEL);
    frontend_command_unregister(TAS_COMMAND_FRAME_BACK);
}
