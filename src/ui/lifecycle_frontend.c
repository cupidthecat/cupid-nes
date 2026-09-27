/*
 * lifecycle_frontend.c - Session recovery controls and persistent preferences
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "lifecycle_frontend.h"
#include "recovery_store.h"
#include "output_guard.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "../system/execution_policy.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { RESUME = 1, AUTO, TIME, INTERVAL, RETAIN, RUN, SNAPSHOTS, RESTORE, CAPTURE, LOAD, SAVE };

static bool prefs_path(LifecycleFrontend *r, char *path, size_t size) {
    return recovery_path(r->directory, NULL, "session-tools.ini", path, size);
}

bool lifecycle_save_preferences(LifecycleFrontend *r, char *error, size_t error_size) {
    char path[4096], text[200];
    if (!r || !prefs_path(r, path, sizeof(path))) {
        return false;
    }
    int length = snprintf(text, sizeof(text), "version=1\nresume=%u\nautomatic=%u\ntime=%u\ninterval=%u\nretain=%u\n",
                          r->resume_on_start, r->recorder.options.automatic, r->recorder.options.time_based,
                          r->recorder.options.interval, r->recorder.options.retain);
    NesFileResult result = nes_file_write_atomic(path, text, (size_t)length);
    if (error && error_size) {
        snprintf(error, error_size, "%s", result == NES_FILE_OK ? "" : nes_file_result_message(result));
    }
    return result == NES_FILE_OK;
}

bool lifecycle_init(LifecycleFrontend *r, FrontendSessionActions *actions, StateRuntime *state, const char *directory,
                    char *error, size_t error_size) {
    if (!r || !actions || !state || !directory || strlen(directory) >= sizeof(r->directory)) {
        return false;
    }
    memset(r, 0, sizeof(*r));
    r->actions = actions;
    r->state = state;
    strcpy(r->directory, directory);
    state_recorder_init(&r->recorder, state, directory);
    char path[4096];
    if (!prefs_path(r, path, sizeof(path))) {
        return false;
    }
    uint8_t *bytes = NULL;
    size_t size = 0;
    NesFileResult result = nes_file_read_all(path, 199, &bytes, &size);
    if (result == NES_FILE_NOT_FOUND) {
        return true;
    }
    if (result != NES_FILE_OK) {
        if (error && error_size) {
            snprintf(error, error_size, "%s", nes_file_result_message(result));
        }
        return false;
    }
    char text[200];
    memcpy(text, bytes, size);
    text[size] = 0;
    free(bytes);
    unsigned resume, automatic, time, interval, retain;
    int end = 0;
    bool valid = !memchr(text, 0, size) &&
                 sscanf(text, "version=1\nresume=%u\nautomatic=%u\ntime=%u\ninterval=%u\nretain=%u\n%n", &resume,
                        &automatic, &time, &interval, &retain, &end) == 5 &&
                 (size_t)end == size && resume <= 1 && automatic <= 1 && time <= 1;
    if (valid) {
        StateRecorderOptions options = {automatic != 0, time != 0, interval, retain};
        valid = state_recorder_configure(&r->recorder, &options);
    }
    if (!valid) {
        if (error && error_size) {
            snprintf(error, error_size, "Invalid session preferences; defaults remain active");
        }
        return false;
    }
    r->resume_on_start = resume != 0;
    return true;
}

bool lifecycle_save_last(LifecycleFrontend *r, char *error, size_t error_size) {
    if (!r || !r->actions || !r->actions->session) {
        return false;
    }
    if (!r->actions->session->active) {
        return true;
    }
    if (nes_execution_policy() != NES_EXECUTION_LIVE) {
        if (error && error_size) {
            snprintf(error, error_size, "Stop deterministic play before saving the last session");
        }
        return false;
    }
    char key[41], path[4096];
    if (!recovery_game_key(r->actions->session, key) ||
        !recovery_path(r->directory, key, ".session", path, sizeof(path))) {
        return false;
    }
    char pointer_path[4096];
    if (!recovery_path(r->directory, NULL, "last-session", pointer_path, sizeof(pointer_path)) ||
        !frontend_output_path_allowed(pointer_path, r->state->execution, r->state->protected_paths,
                                      r->state->protected_path_count, error, error_size)) {
        return false;
    }
    NesStateBlob blob = {0};
    if (!state_runtime_capture(r->state, path, &blob, error, error_size)) {
        return false;
    }
    bool saved = recovery_save_session(r->directory, r->actions->session, &blob, error, error_size);
    nes_state_blob_free(&blob);
    return saved;
}

bool lifecycle_load_last(void *context, char *error, size_t error_size) {
    LifecycleFrontend *r = context;
    if (!r) {
        return false;
    }
    FrontendImageRequest request;
    NesStateBlob state = {0};
    if (!recovery_read_session(r->directory, &request, &state, error, error_size)) {
        return false;
    }
    bool loaded = frontend_session_action_open_state(r->actions, &request, &state, error, error_size);
    nes_state_blob_free(&state);
    if (loaded) {
        lifecycle_image_changed(r);
    }
    return loaded;
}

bool lifecycle_resume_startup(LifecycleFrontend *r, bool explicit_image, char *error, size_t error_size) {
    if (!r) {
        return false;
    }
    if (explicit_image || !r->resume_on_start) {
        return true;
    }
    return lifecycle_load_last(r, error, error_size);
}

void lifecycle_image_changed(LifecycleFrontend *r) {
    if (!r) {
        return;
    }
    r->selected = 0;
    (void)state_recorder_image_changed(&r->recorder, r->actions->session, r->status, sizeof(r->status));
}

static bool snapshot(void *context, FrontendPanelModel *model, char *error, size_t error_size) {
    LifecycleFrontend *r = context;
    bool active = r->actions->session->active;
    bool live = active && nes_execution_policy() == NES_EXECUTION_LIVE;
    snprintf(r->interval, sizeof(r->interval), "%u", r->recorder.options.interval);
    snprintf(r->retain, sizeof(r->retain), "%u", r->recorder.options.retain);
    for (size_t i = 0; i < r->recorder.count; ++i) {
        snprintf(r->labels[i], sizeof(r->labels[i]), "Snapshot %zu%s", i + 1,
                 i + 1 == r->recorder.count ? " (newest)" : "");
        r->items[i] = r->labels[i];
    }
    FrontendPanelControl controls[] = {
        {RESUME, FRONTEND_PANEL_CHECKBOX, "Resume last session on startup", NULL, NULL, 0, r->resume_on_start, true,
         false},
        {LOAD, FRONTEND_PANEL_ACTION, "Load Last Session", NULL, NULL, 0, 0,
         nes_execution_policy() == NES_EXECUTION_LIVE, false},
        {AUTO, FRONTEND_PANEL_CHECKBOX, "Start recorder when a game loads", NULL, NULL, 0,
         r->recorder.options.automatic, true, false},
        {TIME, FRONTEND_PANEL_CHECKBOX, "Use emulated milliseconds instead of frames", NULL, NULL, 0,
         r->recorder.options.time_based, true, false},
        {INTERVAL, FRONTEND_PANEL_TEXT, "Interval (frames or milliseconds)", r->interval, NULL, 0, 0, true, false},
        {RETAIN, FRONTEND_PANEL_TEXT, "Snapshots to retain (1-64)", r->retain, NULL, 0, 0, true, false},
        {SAVE, FRONTEND_PANEL_ACTION, "Save preferences", NULL, NULL, 0, 0, true, false},
        {RUN, FRONTEND_PANEL_CHECKBOX, "Recorder running", NULL, NULL, 0, r->recorder.running, live, false},
        {CAPTURE, FRONTEND_PANEL_ACTION, "Record snapshot now", NULL, NULL, 0, 0, live, false},
        {SNAPSHOTS, FRONTEND_PANEL_LIST, "Recovery history (oldest first)", NULL, r->items, r->recorder.count,
         r->selected, live, true},
        {RESTORE, FRONTEND_PANEL_ACTION, "Restore selected snapshot", NULL, NULL, 0, 0, live && r->recorder.count != 0,
         false}};
    for (size_t i = 0; i < sizeof(controls) / sizeof(controls[0]); ++i) {
        if (!frontend_panel_add_control(model, &controls[i])) {
            return false;
        }
    }
    model->status = r->recorder.status[0] ? r->recorder.status : "Recovery history is separate from slots and rewind";
    if (error && error_size) {
        error[0] = 0;
    }
    return true;
}

static bool action(void *context, unsigned id, const char *value, int selected, char *error, size_t size) {
    LifecycleFrontend *r = context;
    if (id == LOAD) {
        return lifecycle_load_last(r, error, size);
    }
    if (id == SAVE) {
        return lifecycle_save_preferences(r, error, size);
    }
    if (id == CAPTURE) {
        return state_recorder_capture(&r->recorder, error, size);
    }
    if (id == RESTORE) {
        return r->selected >= 0 && state_recorder_restore(&r->recorder, (size_t)r->selected, error, size);
    }
    if (id == SNAPSHOTS && selected >= 0 && (size_t)selected < r->recorder.count) {
        r->selected = selected;
        return true;
    }
    if (id == RUN) {
        if (!r->actions->session->active || nes_execution_policy() != NES_EXECUTION_LIVE) {
            return false;
        }
        r->recorder.running = selected != 0;
        state_recorder_restart_clock(&r->recorder);
        return true;
    }
    StateRecorderOptions previous = r->recorder.options, options = previous;
    bool old_resume = r->resume_on_start;
    if (id == RESUME) {
        r->resume_on_start = selected != 0;
    } else if (id == AUTO) {
        options.automatic = selected != 0;
    } else if (id == TIME) {
        options.time_based = selected != 0;
    } else if ((id == INTERVAL || id == RETAIN) && value && *value >= '0' && *value <= '9') {
        char *end;
        errno = 0;
        unsigned long number = strtoul(value, &end, 10);
        if (errno || *end || number > 86400000) {
            return false;
        }
        if (id == INTERVAL) {
            options.interval = (unsigned)number;
        } else {
            options.retain = (unsigned)number;
        }
    } else {
        return false;
    }
    if (!state_recorder_configure(&r->recorder, &options) || !lifecycle_save_preferences(r, error, size)) {
        r->resume_on_start = old_resume;
        r->recorder.options = previous;
        return false;
    }
    return true;
}

bool lifecycle_register_ui(LifecycleFrontend *r) {
    const FrontendCommandSpec commands[] = {
        {LIFECYCLE_LOAD_LAST, "Load Last Session", "File", NULL, 0, lifecycle_load_last, r},
        {LIFECYCLE_UNLOAD, "Power Off / Unload Game", "File", NULL, FRONTEND_COMMAND_NEEDS_SESSION,
         frontend_session_action_unload, r->actions}};
    if (!frontend_command_register(&commands[0])) {
        return false;
    }
    if (!frontend_command_register(&commands[1])) {
        frontend_command_unregister(LIFECYCLE_LOAD_LAST);
        return false;
    }
    FrontendPanelSpec panel = {LIFECYCLE_PANEL, "Session and State Recorder", "Tools", 0, snapshot, action, r};
    if (!frontend_panel_register(&panel)) {
        lifecycle_unregister_ui();
        return false;
    }
    return true;
}

void lifecycle_unregister_ui(void) {
    frontend_panel_unregister(LIFECYCLE_PANEL);
    frontend_command_unregister(LIFECYCLE_LOAD_LAST);
    frontend_command_unregister(LIFECYCLE_UNLOAD);
}
