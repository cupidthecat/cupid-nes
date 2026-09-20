/*
 * replay_frontend.c - Rewind, run-ahead, and input movie controls
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "replay_frontend.h"
#include "frontend_commands.h"
#include "frontend_execution.h"
#include "frontend_panels.h"
#include "platform_frontend.h"
#include "../system/execution_policy.h"

#include <stdio.h>

static const char *const history_items[] = {
    "Off", "2 seconds", "5 seconds", "10 seconds", "30 seconds", "60 seconds"
};
static const unsigned history_seconds[] = {0, 2, 5, 10, 30, 60};
static const char *const runahead_items[] = {
    "Off", "1 frame", "2 frames", "3 frames", "4 frames"
};
static const char *const movie_start_items[] = {"Current state", "Power-on"};
static char status_text[160];
static char movie_status_text[192];

static void set_error(char *error, size_t error_size, const char *message) {
    if (error && error_size) snprintf(error, error_size, "%s", message);
}

static bool command_rewind(void *userdata, char *error, size_t error_size) {
    FrontendExecutionRuntime *runtime = userdata;
    if (!frontend_execution_rewind_step(runtime, error, error_size)) return false;
    execution_control_set_paused(&runtime->execution, true);
    frontend_command_set_checked(FRONTEND_COMMAND_PAUSE, true);
    frontend_execution_refresh_audio(runtime);
    return true;
}

static bool command_runahead_cycle(void *userdata, char *error, size_t error_size) {
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    if (nes_execution_policy() != NES_EXECUTION_LIVE) {
        set_error(error, error_size, "Stop the replay session before changing run-ahead");
        return false;
    }
    unsigned next = (frontend_execution_run_ahead(runtime) + 1u) % 5u;
    if (!frontend_execution_set_run_ahead(runtime, next)) {
        set_error(error, error_size, "Run-ahead setting is outside the supported range");
        return false;
    }
    return true;
}

static bool command_movie_record(void *userdata, char *error, size_t error_size) {
    return frontend_execution_movie_record((FrontendExecutionRuntime *)userdata,
                                           error, error_size);
}

static bool command_movie_play(void *userdata, char *error, size_t error_size) {
    return frontend_execution_movie_play((FrontendExecutionRuntime *)userdata,
                                         error, error_size);
}

static bool command_movie_stop(void *userdata, char *error, size_t error_size) {
    return frontend_execution_movie_stop((FrontendExecutionRuntime *)userdata,
                                         error, error_size);
}

static bool choose_movie(FrontendExecutionRuntime *runtime, bool save,
                          char *error, size_t error_size) {
    if (!runtime || !runtime->movie) return false;
    NesMovieMode mode = nes_movie_mode(runtime->movie);
    if (mode == NES_MOVIE_PLAYBACK || (!save && mode != NES_MOVIE_IDLE)) {
        set_error(error, error_size, "Stop the input movie before selecting a file to play");
        return false;
    }
    char path[FRONTEND_MOVIE_PATH_CAPACITY] = {0};
    bool selected = save
        ? frontend_save_file_dialog(FRONTEND_SAVE_MOVIE, path, sizeof(path), error, error_size)
        : frontend_open_movie_dialog(path, sizeof(path), error, error_size);
    return selected && frontend_execution_movie_set_path(runtime, path, error, error_size);
}

void replay_frontend_refresh(FrontendExecutionRuntime *runtime) {
    if (!runtime) return;
    NesMovieMode mode = nes_movie_mode(runtime->movie);
    bool live = nes_execution_policy() == NES_EXECUTION_LIVE;
    bool start = runtime->movie && mode == NES_MOVIE_IDLE && live;
    (void)frontend_command_set_enabled(REPLAY_COMMAND_MOVIE_RECORD, start);
    (void)frontend_command_set_enabled(REPLAY_COMMAND_MOVIE_PLAY, start);
    (void)frontend_command_set_enabled(REPLAY_COMMAND_MOVIE_STOP, mode != NES_MOVIE_IDLE);
    (void)frontend_command_set_checked(REPLAY_COMMAND_MOVIE_RECORD, mode == NES_MOVIE_RECORDING);
    (void)frontend_command_set_checked(REPLAY_COMMAND_MOVIE_PLAY, mode == NES_MOVIE_PLAYBACK);
    (void)frontend_command_set_enabled(REPLAY_COMMAND_REWIND_FRAME,
                                      live && frontend_execution_rewind_available(runtime) != 0);
    (void)frontend_command_set_enabled(REPLAY_COMMAND_RUNAHEAD_CYCLE, live);
}

static int selected_history(unsigned seconds) {
    for (size_t i = 0; i < sizeof(history_seconds) / sizeof(history_seconds[0]); ++i)
        if (history_seconds[i] == seconds) return (int)i;
    return -1;
}

static bool replay_panel_snapshot(void *userdata, FrontendPanelModel *model,
                                  char *error, size_t error_size) {
    (void)error;
    (void)error_size;
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    if (!runtime || !model) return false;
    replay_frontend_refresh(runtime);
    bool live = nes_execution_policy() == NES_EXECUTION_LIVE;

    FrontendPanelControl history = {
        .id = REPLAY_CONTROL_HISTORY,
        .type = FRONTEND_PANEL_CHOICE,
        .label = "Rewind history",
        .items = history_items,
        .item_count = sizeof(history_items) / sizeof(history_items[0]),
        .selected = selected_history(frontend_execution_rewind_seconds(runtime)),
        .enabled = live
    };
    FrontendPanelControl runahead = {
        .id = REPLAY_CONTROL_RUNAHEAD,
        .type = FRONTEND_PANEL_CHOICE,
        .label = "Run-ahead",
        .items = runahead_items,
        .item_count = sizeof(runahead_items) / sizeof(runahead_items[0]),
        .selected = (int)frontend_execution_run_ahead(runtime),
        .enabled = live
    };
    FrontendPanelControl rewind = {
        .id = REPLAY_CONTROL_REWIND,
        .type = FRONTEND_PANEL_ACTION,
        .label = "Rewind",
        .enabled = live && frontend_execution_rewind_available(runtime) != 0
    };
    FrontendPanelControl clear = {
        .id = REPLAY_CONTROL_CLEAR,
        .type = FRONTEND_PANEL_ACTION,
        .label = "Clear rewind history",
        .enabled = live && frontend_execution_rewind_available(runtime) != 0
    };

    size_t bytes = nes_rewind_bytes(&runtime->rewind);
    snprintf(status_text, sizeof(status_text),
             "%zu frame%s available, %.1f MiB; run-ahead %u frame%s",
             frontend_execution_rewind_available(runtime),
             frontend_execution_rewind_available(runtime) == 1 ? "" : "s",
             (double)bytes / (1024.0 * 1024.0),
             frontend_execution_run_ahead(runtime),
             frontend_execution_run_ahead(runtime) == 1 ? "" : "s");
    FrontendPanelControl status = {
        .id = REPLAY_CONTROL_STATUS,
        .type = FRONTEND_PANEL_TEXT,
        .label = "Status",
        .value = status_text,
        .enabled = true,
        .read_only = true
    };
    NesMovieProgress progress;
    frontend_execution_movie_progress(runtime, &progress);
    bool idle = progress.mode == NES_MOVIE_IDLE;
    FrontendPanelControl movie_path = {
        .id = REPLAY_CONTROL_MOVIE_PATH,
        .type = FRONTEND_PANEL_FILE_SAVE,
        .selected = FRONTEND_SAVE_MOVIE,
        .label = "Input movie path",
        .value = runtime->movie_path,
        .enabled = progress.mode != NES_MOVIE_PLAYBACK
    };
    FrontendPanelControl movie_start = {
        .id = REPLAY_CONTROL_MOVIE_START,
        .type = FRONTEND_PANEL_CHOICE,
        .label = "Input movie start",
        .items = movie_start_items,
        .item_count = sizeof(movie_start_items) / sizeof(movie_start_items[0]),
        .selected = (int)(idle ? runtime->movie_start_kind : progress.start_kind),
        .enabled = idle
    };
    FrontendPanelControl movie_open_file = {
        .id = REPLAY_CONTROL_MOVIE_OPEN_FILE,
        .type = FRONTEND_PANEL_ACTION,
        .label = "Choose a movie to play...",
        .enabled = idle && live
    };
    FrontendPanelControl movie_save_file = {
        .id = REPLAY_CONTROL_MOVIE_SAVE_FILE,
        .type = FRONTEND_PANEL_ACTION,
        .label = "Choose recording destination...",
        .enabled = progress.mode != NES_MOVIE_PLAYBACK
    };
    FrontendPanelControl movie_record = {
        .id = REPLAY_CONTROL_MOVIE_RECORD,
        .type = FRONTEND_PANEL_ACTION,
        .label = "Record input movie",
        .enabled = runtime->movie && idle && live
    };
    FrontendPanelControl movie_play = {
        .id = REPLAY_CONTROL_MOVIE_PLAY,
        .type = FRONTEND_PANEL_ACTION,
        .label = "Play input movie",
        .enabled = runtime->movie && idle && live
    };
    FrontendPanelControl movie_stop = {
        .id = REPLAY_CONTROL_MOVIE_STOP,
        .type = FRONTEND_PANEL_ACTION,
        .label = "Stop input movie",
        .enabled = !idle
    };
    const char *mode = progress.mode == NES_MOVIE_RECORDING ? "Recording"
                     : progress.mode == NES_MOVIE_PLAYBACK ? "Playback" : "Stopped";
    snprintf(movie_status_text, sizeof(movie_status_text),
             "%s | %s start | frame %llu/%llu | %zu events | %s",
             mode, progress.start_kind == NES_MOVIE_START_POWER_ON ? "power-on" : "state",
             (unsigned long long)progress.frame,
             (unsigned long long)progress.total_frames, progress.event_count,
             nes_movie_result_string(progress.last_result));
    FrontendPanelControl movie_status = {
        .id = REPLAY_CONTROL_MOVIE_STATUS,
        .type = FRONTEND_PANEL_TEXT,
        .label = "Input movie status",
        .value = movie_status_text,
        .enabled = true,
        .read_only = true
    };
    model->status = runtime->movie_status[0] ? runtime->movie_status
        : "Stopping an input movie returns to the session you had before it started.";

    return frontend_panel_add_control(model, &history)
        && frontend_panel_add_control(model, &runahead)
        && frontend_panel_add_control(model, &rewind)
        && frontend_panel_add_control(model, &clear)
        && frontend_panel_add_control(model, &status)
        && frontend_panel_add_control(model, &movie_path)
        && frontend_panel_add_control(model, &movie_open_file)
        && frontend_panel_add_control(model, &movie_save_file)
        && frontend_panel_add_control(model, &movie_start)
        && frontend_panel_add_control(model, &movie_record)
        && frontend_panel_add_control(model, &movie_play)
        && frontend_panel_add_control(model, &movie_stop)
        && frontend_panel_add_control(model, &movie_status);
}

static bool replay_panel_action(void *userdata, unsigned control_id,
                                const char *value, int selected,
                                char *error, size_t error_size) {
    (void)value;
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    if (!runtime) return false;
    if (control_id >= REPLAY_CONTROL_HISTORY && control_id <= REPLAY_CONTROL_CLEAR
        && nes_execution_policy() != NES_EXECUTION_LIVE) {
        set_error(error, error_size, "Stop the replay session before changing rewind or run-ahead");
        return false;
    }
    switch (control_id) {
        case REPLAY_CONTROL_HISTORY:
            if (selected < 0 || (size_t)selected >= sizeof(history_seconds) / sizeof(history_seconds[0])
                || !frontend_execution_set_rewind_seconds(runtime, history_seconds[selected])) {
                set_error(error, error_size, "Invalid rewind history duration");
                return false;
            }
            return true;
        case REPLAY_CONTROL_RUNAHEAD:
            if (selected < 0 || (size_t)selected >= sizeof(runahead_items) / sizeof(runahead_items[0])
                || !frontend_execution_set_run_ahead(runtime, (unsigned)selected)) {
                set_error(error, error_size, "Invalid run-ahead frame count");
                return false;
            }
            return true;
        case REPLAY_CONTROL_REWIND:
            return command_rewind(runtime, error, error_size);
        case REPLAY_CONTROL_CLEAR:
            frontend_execution_clear_timeline(runtime);
            return true;
        case REPLAY_CONTROL_MOVIE_PATH:
            return frontend_execution_movie_set_path(runtime, value, error, error_size);
        case REPLAY_CONTROL_MOVIE_START:
            if (selected < 0 || selected > NES_MOVIE_START_POWER_ON
                || !frontend_execution_movie_set_start_kind(runtime,
                                                            (NesMovieStartKind)selected)) {
                set_error(error, error_size, "Stop the input movie before changing its start mode");
                return false;
            }
            return true;
        case REPLAY_CONTROL_MOVIE_RECORD:
            return frontend_execution_movie_record(runtime, error, error_size);
        case REPLAY_CONTROL_MOVIE_PLAY:
            return frontend_execution_movie_play(runtime, error, error_size);
        case REPLAY_CONTROL_MOVIE_STOP:
            return frontend_execution_movie_stop(runtime, error, error_size);
        case REPLAY_CONTROL_MOVIE_OPEN_FILE:
            return choose_movie(runtime, false, error, error_size);
        case REPLAY_CONTROL_MOVIE_SAVE_FILE:
            return choose_movie(runtime, true, error, error_size);
        default:
            set_error(error, error_size, "Unknown replay control");
            return false;
    }
}

bool replay_frontend_register(FrontendExecutionRuntime *runtime) {
    if (!runtime) return false;
    FrontendCommandSpec rewind = {
        .id = REPLAY_COMMAND_REWIND_FRAME,
        .label = "Rewind",
        .menu = "Emulation",
        .shortcut = "Ctrl+Backspace",
        .flags = FRONTEND_COMMAND_NEEDS_SESSION | FRONTEND_COMMAND_MOMENTARY,
        .handler = command_rewind,
        .userdata = runtime
    };
    FrontendCommandSpec runahead = {
        .id = REPLAY_COMMAND_RUNAHEAD_CYCLE,
        .label = "Cycle Run-Ahead",
        .menu = "Emulation",
        .shortcut = "Ctrl+Shift+A",
        .flags = FRONTEND_COMMAND_NEEDS_SESSION,
        .handler = command_runahead_cycle,
        .userdata = runtime
    };
    FrontendCommandSpec movie_record = {
        .id = REPLAY_COMMAND_MOVIE_RECORD,
        .label = "Record Input Movie",
        .menu = "Emulation",
        .shortcut = "",
        .flags = FRONTEND_COMMAND_NEEDS_SESSION | FRONTEND_COMMAND_CHECKABLE,
        .handler = command_movie_record,
        .userdata = runtime
    };
    FrontendCommandSpec movie_play = {
        .id = REPLAY_COMMAND_MOVIE_PLAY,
        .label = "Play Input Movie",
        .menu = "Emulation",
        .shortcut = "",
        .flags = FRONTEND_COMMAND_NEEDS_SESSION | FRONTEND_COMMAND_CHECKABLE,
        .handler = command_movie_play,
        .userdata = runtime
    };
    FrontendCommandSpec movie_stop = {
        .id = REPLAY_COMMAND_MOVIE_STOP,
        .label = "Stop Input Movie",
        .menu = "Emulation",
        .shortcut = "",
        .flags = FRONTEND_COMMAND_NEEDS_SESSION,
        .handler = command_movie_stop,
        .userdata = runtime
    };
    FrontendPanelSpec panel = {
        .id = REPLAY_PANEL,
        .title = "Rewind, Run-Ahead and Movies",
        .category = "Emulation",
        .flags = FRONTEND_PANEL_NEEDS_SESSION,
        .snapshot = replay_panel_snapshot,
        .action = replay_panel_action,
        .userdata = runtime
    };
    if (!frontend_command_register(&rewind)) return false;
    if (!frontend_command_register(&runahead)) {
        (void)frontend_command_unregister(REPLAY_COMMAND_REWIND_FRAME);
        return false;
    }
    if (!frontend_command_register(&movie_record)
        || !frontend_command_register(&movie_play)
        || !frontend_command_register(&movie_stop)) {
        (void)frontend_command_unregister(REPLAY_COMMAND_MOVIE_STOP);
        (void)frontend_command_unregister(REPLAY_COMMAND_MOVIE_PLAY);
        (void)frontend_command_unregister(REPLAY_COMMAND_MOVIE_RECORD);
        (void)frontend_command_unregister(REPLAY_COMMAND_RUNAHEAD_CYCLE);
        (void)frontend_command_unregister(REPLAY_COMMAND_REWIND_FRAME);
        return false;
    }
    if (!frontend_panel_register(&panel)) {
        (void)frontend_command_unregister(REPLAY_COMMAND_MOVIE_STOP);
        (void)frontend_command_unregister(REPLAY_COMMAND_MOVIE_PLAY);
        (void)frontend_command_unregister(REPLAY_COMMAND_MOVIE_RECORD);
        (void)frontend_command_unregister(REPLAY_COMMAND_RUNAHEAD_CYCLE);
        (void)frontend_command_unregister(REPLAY_COMMAND_REWIND_FRAME);
        return false;
    }
    replay_frontend_refresh(runtime);
    return true;
}

void replay_frontend_unregister(void) {
    (void)frontend_panel_unregister(REPLAY_PANEL);
    (void)frontend_command_unregister(REPLAY_COMMAND_MOVIE_STOP);
    (void)frontend_command_unregister(REPLAY_COMMAND_MOVIE_PLAY);
    (void)frontend_command_unregister(REPLAY_COMMAND_MOVIE_RECORD);
    (void)frontend_command_unregister(REPLAY_COMMAND_RUNAHEAD_CYCLE);
    (void)frontend_command_unregister(REPLAY_COMMAND_REWIND_FRAME);
}
