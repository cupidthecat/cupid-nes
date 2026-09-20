/*
 * state_frontend.c - Save-state commands and frontend panel
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "state_frontend.h"
#include "frontend_commands.h"
#include "frontend_panels.h"
#include "platform_frontend.h"
#include "../state/state.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    STATE_CONTROL_SLOT = 1,
    STATE_CONTROL_PATH,
    STATE_CONTROL_SAVE_SLOT,
    STATE_CONTROL_LOAD_SLOT,
    STATE_CONTROL_SAVE_FILE,
    STATE_CONTROL_LOAD_FILE,
    STATE_CONTROL_BROWSE_SAVE,
    STATE_CONTROL_BROWSE_LOAD
};

static const char *const slot_names[NES_STATE_SLOT_COUNT] = {
    "Slot 1", "Slot 2", "Slot 3", "Slot 4", "Slot 5",
    "Slot 6", "Slot 7", "Slot 8", "Slot 9", "Slot 10"
};

static void report_result(FrontendStateRuntime *runtime, NesStateResult result,
                          const char *success, char *error, size_t error_size) {
    const char *message = result == NES_STATE_OK ? success : nes_state_result_string(result);
    if (runtime) snprintf(runtime->status, sizeof(runtime->status), "%s", message);
    if (error && error_size)
        snprintf(error, error_size, "%s", result == NES_STATE_OK ? "" : message);
}

static bool before_load(FrontendStateRuntime *runtime, char *error, size_t error_size) {
    return !runtime->before_load || runtime->before_load(runtime->context, error, error_size);
}

static void after_load(FrontendStateRuntime *runtime, bool loaded) {
    if (runtime->after_load) runtime->after_load(runtime->context, loaded);
}

static NesStateResult capture_and_write(FrontendStateRuntime *runtime, const char *path) {
    NesStateBlob blob = {0};
    NesStateResult result = nes_state_capture(&blob);
    if (runtime->after_capture) runtime->after_capture(runtime->context, result == NES_STATE_OK);
    if (result != NES_STATE_OK) return result;
    NesFileResult written = nes_file_write_atomic(path, blob.data, blob.size);
    nes_state_blob_free(&blob);
    switch (written) {
        case NES_FILE_OK: return NES_STATE_OK;
        case NES_FILE_OUT_OF_MEMORY: return NES_STATE_ERROR_OUT_OF_MEMORY;
        case NES_FILE_INVALID_ARGUMENT: return NES_STATE_ERROR_ARGUMENT;
        default: return NES_STATE_ERROR_IO;
    }
}

static bool save_slot(void *context, char *error, size_t error_size) {
    FrontendStateRuntime *runtime = context;
    char path[NES_FILE_PATH_LIMIT];
    NesStateResult result = nes_state_slot_path(runtime->slot_directory, runtime->settings->state_slot,
                                               path, sizeof(path));
    if (result != NES_STATE_OK) {
        report_result(runtime, result, "", error, error_size);
        return false;
    }
    if (runtime->before_save && !runtime->before_save(runtime->context, path, error, error_size)) return false;
    result = capture_and_write(runtime, path);
    report_result(runtime, result, "State slot saved", error, error_size);
    return result == NES_STATE_OK;
}

static bool load_slot(void *context, char *error, size_t error_size) {
    FrontendStateRuntime *runtime = context;
    if (!before_load(runtime, error, error_size)) return false;
    NesStateResult result = nes_state_load_slot(runtime->slot_directory, runtime->settings->state_slot);
    report_result(runtime, result, "State slot loaded", error, error_size);
    after_load(runtime, result == NES_STATE_OK);
    return result == NES_STATE_OK;
}

static bool save_file(void *context, char *error, size_t error_size) {
    FrontendStateRuntime *runtime = context;
    if (!runtime->settings->state_file_path[0]) {
        if (error && error_size) snprintf(error, error_size, "Choose a state file path first");
        return false;
    }
    const char *path = runtime->settings->state_file_path;
    if (runtime->before_save && !runtime->before_save(runtime->context, path, error, error_size)) return false;
    NesStateResult result = capture_and_write(runtime, path);
    report_result(runtime, result, "State file saved", error, error_size);
    return result == NES_STATE_OK;
}

static bool load_file(void *context, char *error, size_t error_size) {
    FrontendStateRuntime *runtime = context;
    if (!runtime->settings->state_file_path[0]) {
        if (error && error_size) snprintf(error, error_size, "Choose a state file path first");
        return false;
    }
    if (!before_load(runtime, error, error_size)) return false;
    NesStateResult result = nes_state_load_file(runtime->settings->state_file_path);
    report_result(runtime, result, "State file loaded", error, error_size);
    after_load(runtime, result == NES_STATE_OK);
    return result == NES_STATE_OK;
}

static bool state_snapshot(void *context, FrontendPanelModel *model,
                           char *error, size_t error_size) {
    FrontendStateRuntime *runtime = context;
    if (!runtime || !runtime->settings) return false;
    snprintf(runtime->slot_text, sizeof(runtime->slot_text), "Slot %u",
             runtime->settings->state_slot + 1u);
    FrontendPanelControl controls[] = {
        {STATE_CONTROL_SLOT, FRONTEND_PANEL_CHOICE, "Quick state slot", runtime->slot_text,
         slot_names, NES_STATE_SLOT_COUNT, (int)runtime->settings->state_slot, true, false},
        {STATE_CONTROL_PATH, FRONTEND_PANEL_FILE_SAVE, "State file", runtime->settings->state_file_path,
         NULL, 0, FRONTEND_SAVE_STATE, true, false},
        {STATE_CONTROL_SAVE_SLOT, FRONTEND_PANEL_ACTION, "Save selected slot", NULL,
         NULL, 0, 0, true, false},
        {STATE_CONTROL_LOAD_SLOT, FRONTEND_PANEL_ACTION, "Load selected slot", NULL,
         NULL, 0, 0, true, false},
        {STATE_CONTROL_SAVE_FILE, FRONTEND_PANEL_ACTION, "Save state file", NULL,
         NULL, 0, 0, true, false},
        {STATE_CONTROL_LOAD_FILE, FRONTEND_PANEL_ACTION, "Load state file", NULL,
         NULL, 0, 0, true, false},
        {STATE_CONTROL_BROWSE_SAVE, FRONTEND_PANEL_ACTION, "Choose save destination...", NULL, NULL, 0, 0, true, false},
        {STATE_CONTROL_BROWSE_LOAD, FRONTEND_PANEL_ACTION, "Choose existing state...", NULL, NULL, 0, 0, true, false}
    };
    for (size_t i = 0; i < sizeof(controls) / sizeof(controls[0]); ++i)
        if (!frontend_panel_add_control(model, &controls[i])) return false;
    model->status = runtime->status[0] ? runtime->status : "Save states are image-specific";
    if (error && error_size) error[0] = '\0';
    return true;
}

static bool state_action(void *context, unsigned id, const char *value, int selected,
                         char *error, size_t error_size) {
    FrontendStateRuntime *runtime = context;
    if (id == STATE_CONTROL_SLOT && selected >= 0 && selected < NES_STATE_SLOT_COUNT) {
        runtime->settings->state_slot = (unsigned)selected;
        return true;
    }
    if (id == STATE_CONTROL_PATH && value) {
        if (strlen(value) >= sizeof(runtime->settings->state_file_path)) {
            if (error && error_size) snprintf(error, error_size, "State path is too long");
            return false;
        }
        strcpy(runtime->settings->state_file_path, value);
        return true;
    }
    if (id==STATE_CONTROL_BROWSE_SAVE || id==STATE_CONTROL_BROWSE_LOAD) {
        char path[FRONTEND_SETTINGS_PATH_TEXT]={0};
        bool chosen=id==STATE_CONTROL_BROWSE_SAVE
            ? frontend_save_file_dialog(FRONTEND_SAVE_STATE,path,sizeof(path),error,error_size)
            : frontend_open_file_dialog(FRONTEND_OPEN_STATE,path,sizeof(path),error,error_size);
        if(chosen)strcpy(runtime->settings->state_file_path,path);
        return chosen;
    }
    switch (id) {
        case STATE_CONTROL_SAVE_SLOT: return save_slot(runtime, error, error_size);
        case STATE_CONTROL_LOAD_SLOT: return load_slot(runtime, error, error_size);
        case STATE_CONTROL_SAVE_FILE: return save_file(runtime, error, error_size);
        case STATE_CONTROL_LOAD_FILE: return load_file(runtime, error, error_size);
        default: break;
    }
    if (error && error_size) snprintf(error, error_size, "Unknown state control");
    return false;
}

void frontend_state_init(FrontendStateRuntime *runtime, FrontendSettings *settings,
                         const char *slot_directory) {
    if (!runtime) return;
    memset(runtime, 0, sizeof(*runtime));
    runtime->settings = settings;
    runtime->slot_directory = slot_directory;
}

void frontend_state_set_hooks(FrontendStateRuntime *runtime,
                              FrontendBeforeStateLoad before_load_hook,
                              FrontendAfterStateLoad after_load_hook,
                              void *context) {
    if (!runtime) return;
    runtime->before_load = before_load_hook;
    runtime->after_load = after_load_hook;
    runtime->context = context;
}

void frontend_state_set_save_hooks(FrontendStateRuntime *runtime,
                                   FrontendBeforeStateSave before_save_hook,
                                   FrontendAfterStateCapture after_capture_hook) {
    if (!runtime) return;
    runtime->before_save = before_save_hook;
    runtime->after_capture = after_capture_hook;
}

bool frontend_state_register_ui(FrontendStateRuntime *runtime) {
    if (!runtime || !runtime->settings || !runtime->slot_directory) return false;
    const FrontendCommandSpec commands[] = {
        {STATE_COMMAND_SAVE_SLOT, "Quick Save State", "File", "F5",
         FRONTEND_COMMAND_NEEDS_SESSION, save_slot, runtime},
        {STATE_COMMAND_LOAD_SLOT, "Quick Load State", "File", "F6",
         FRONTEND_COMMAND_NEEDS_SESSION, load_slot, runtime},
        {STATE_COMMAND_SAVE_FILE, "Save State File", "File", "Ctrl+F5",
         FRONTEND_COMMAND_NEEDS_SESSION, save_file, runtime},
        {STATE_COMMAND_LOAD_FILE, "Load State File", "File", "Ctrl+F6",
         FRONTEND_COMMAND_NEEDS_SESSION, load_file, runtime}
    };
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
        if (!frontend_command_register(&commands[i])) {
            for (size_t j = 0; j < i; ++j) (void)frontend_command_unregister(commands[j].id);
            return false;
        }
    }
    FrontendPanelSpec panel = {
        .id = STATE_PANEL, .title = "Save States", .category = "Tools",
        .flags = FRONTEND_PANEL_NEEDS_SESSION,
        .snapshot = state_snapshot, .action = state_action, .userdata = runtime
    };
    if (!frontend_panel_register(&panel)) {
        for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i)
            (void)frontend_command_unregister(commands[i].id);
        return false;
    }
    return true;
}

void frontend_state_unregister_ui(void) {
    (void)frontend_command_unregister(STATE_COMMAND_SAVE_SLOT);
    (void)frontend_command_unregister(STATE_COMMAND_LOAD_SLOT);
    (void)frontend_command_unregister(STATE_COMMAND_SAVE_FILE);
    (void)frontend_command_unregister(STATE_COMMAND_LOAD_FILE);
    (void)frontend_panel_unregister(STATE_PANEL);
}
