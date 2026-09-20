/*
 * replay_frontend.c - Rewind and run-ahead command/panel registration
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

#include <stdio.h>

static const char *const history_items[] = {
    "Off", "2 seconds", "5 seconds", "10 seconds", "30 seconds", "60 seconds"
};
static const unsigned history_seconds[] = {0, 2, 5, 10, 30, 60};
static const char *const runahead_items[] = {
    "Off", "1 frame", "2 frames", "3 frames", "4 frames"
};
static char status_text[160];

static void set_error(char *error, size_t error_size, const char *message) {
    if (error && error_size) snprintf(error, error_size, "%s", message);
}

static bool command_rewind(void *userdata, char *error, size_t error_size) {
    return frontend_execution_rewind_step((FrontendExecutionRuntime *)userdata,
                                          error, error_size);
}

static bool command_runahead_cycle(void *userdata, char *error, size_t error_size) {
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    unsigned next = (frontend_execution_run_ahead(runtime) + 1u) % 5u;
    if (!frontend_execution_set_run_ahead(runtime, next)) {
        set_error(error, error_size, "Run-ahead setting is outside the supported range");
        return false;
    }
    return true;
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

    FrontendPanelControl history = {
        .id = REPLAY_CONTROL_HISTORY,
        .type = FRONTEND_PANEL_CHOICE,
        .label = "Rewind history",
        .items = history_items,
        .item_count = sizeof(history_items) / sizeof(history_items[0]),
        .selected = selected_history(frontend_execution_rewind_seconds(runtime)),
        .enabled = true
    };
    FrontendPanelControl runahead = {
        .id = REPLAY_CONTROL_RUNAHEAD,
        .type = FRONTEND_PANEL_CHOICE,
        .label = "Run-ahead",
        .items = runahead_items,
        .item_count = sizeof(runahead_items) / sizeof(runahead_items[0]),
        .selected = (int)frontend_execution_run_ahead(runtime),
        .enabled = true
    };
    FrontendPanelControl rewind = {
        .id = REPLAY_CONTROL_REWIND,
        .type = FRONTEND_PANEL_ACTION,
        .label = "Rewind one frame",
        .enabled = frontend_execution_rewind_available(runtime) != 0
    };
    FrontendPanelControl clear = {
        .id = REPLAY_CONTROL_CLEAR,
        .type = FRONTEND_PANEL_ACTION,
        .label = "Clear rewind history",
        .enabled = frontend_execution_rewind_available(runtime) != 0
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

    return frontend_panel_add_control(model, &history)
        && frontend_panel_add_control(model, &runahead)
        && frontend_panel_add_control(model, &rewind)
        && frontend_panel_add_control(model, &clear)
        && frontend_panel_add_control(model, &status);
}

static bool replay_panel_action(void *userdata, unsigned control_id,
                                const char *value, int selected,
                                char *error, size_t error_size) {
    (void)value;
    FrontendExecutionRuntime *runtime = (FrontendExecutionRuntime *)userdata;
    if (!runtime) return false;
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
            return frontend_execution_rewind_step(runtime, error, error_size);
        case REPLAY_CONTROL_CLEAR:
            frontend_execution_clear_timeline(runtime);
            return true;
        default:
            set_error(error, error_size, "Unknown replay control");
            return false;
    }
}

bool replay_frontend_register(FrontendExecutionRuntime *runtime) {
    if (!runtime) return false;
    FrontendCommandSpec rewind = {
        .id = REPLAY_COMMAND_REWIND_FRAME,
        .label = "Rewind One Frame",
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
    FrontendPanelSpec panel = {
        .id = REPLAY_PANEL,
        .title = "Rewind and Run-Ahead",
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
    if (!frontend_panel_register(&panel)) {
        (void)frontend_command_unregister(REPLAY_COMMAND_RUNAHEAD_CYCLE);
        (void)frontend_command_unregister(REPLAY_COMMAND_REWIND_FRAME);
        return false;
    }
    return true;
}

void replay_frontend_unregister(void) {
    (void)frontend_panel_unregister(REPLAY_PANEL);
    (void)frontend_command_unregister(REPLAY_COMMAND_RUNAHEAD_CYCLE);
    (void)frontend_command_unregister(REPLAY_COMMAND_REWIND_FRAME);
}
