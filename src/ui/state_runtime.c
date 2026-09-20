/*
 * state_runtime.c - Live save-state lifecycle integration
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "state_runtime.h"
#include "output_guard.h"
#include "frontend_commands.h"
#include "../debugger/debugger.h"
#include "../system/execution_policy.h"
#include <stdio.h>
#include <string.h>

static bool before_load(void *context, char *error, size_t error_size) {
    StateRuntime *runtime = context;
    if (!runtime || !runtime->execution) return false;
    if (nes_execution_policy() != NES_EXECUTION_LIVE) {
        if (error && error_size)
            snprintf(error, error_size, "Stop the replay session before loading a state");
        return false;
    }
    FrontendExecutionRuntime *execution = runtime->execution;
    runtime->paused_before_load = execution->execution.paused;
    if (execution->before_machine_change
        && !execution->before_machine_change(execution->machine_change_context,
                                             error, error_size)) return false;
    frontend_execution_begin_machine_change(execution);
    runtime->machine_locked = true;
    return true;
}

static void after_load(void *context, bool loaded) {
    StateRuntime *runtime = context;
    if (!runtime) return;
    if (loaded) {
        frontend_execution_clear_timeline(runtime->execution);
        debugger_reset_session();
        runtime->execution->debugger_pause_revision = debugger_pause_revision();
        execution_control_set_paused(&runtime->execution->execution, runtime->paused_before_load);
        (void)frontend_command_set_checked(FRONTEND_COMMAND_PAUSE, runtime->paused_before_load);
        if (runtime->restored) runtime->restored(runtime->restored_context);
    }
    if (runtime->machine_locked) {
        frontend_execution_end_machine_change_preserving_audio(runtime->execution);
        runtime->machine_locked = false;
    }
}

static bool before_save(void *context, const char *path, char *error, size_t error_size) {
    StateRuntime *runtime = context;
    if (!runtime || !runtime->execution || runtime->save_audio_device) return false;
    const FrontendExecutionRuntime *execution = runtime->execution;
    if (!frontend_output_path_allowed(path, execution, runtime->protected_paths,
                                      runtime->protected_path_count, error, error_size)) return false;
    if (execution->audio_device && *execution->audio_device) {
        runtime->save_audio_device = *execution->audio_device;
        SDL_LockAudioDevice(runtime->save_audio_device);
    }
    return true;
}

static void after_capture(void *context, bool captured) {
    (void)captured;
    StateRuntime *runtime = context;
    if (runtime && runtime->save_audio_device) {
        SDL_UnlockAudioDevice(runtime->save_audio_device);
        runtime->save_audio_device = 0;
    }
}

void state_runtime_init(StateRuntime *runtime, FrontendSettings *settings,
                        const char *slot_directory, FrontendExecutionRuntime *execution) {
    if (!runtime) return;
    memset(runtime, 0, sizeof(*runtime));
    runtime->execution = execution;
    frontend_state_init(&runtime->frontend, settings, slot_directory);
    frontend_state_set_hooks(&runtime->frontend, before_load, after_load, runtime);
    frontend_state_set_save_hooks(&runtime->frontend, before_save, after_capture);
}

void state_runtime_set_protected_paths(StateRuntime *runtime,
                                       const char *const *paths, size_t count) {
    if (!runtime || (count && !paths)) return;
    runtime->protected_paths = paths;
    runtime->protected_path_count = count;
}

void state_runtime_set_restored(StateRuntime *runtime, StateRuntimeRestored restored,
                                void *context) {
    if (!runtime) return;
    runtime->restored = restored;
    runtime->restored_context = context;
}

bool state_runtime_register_ui(StateRuntime *runtime) {
    return runtime && runtime->execution && frontend_state_register_ui(&runtime->frontend);
}

void state_runtime_shutdown(StateRuntime *runtime) {
    if (!runtime) return;
    if (runtime->machine_locked) {
        frontend_execution_end_machine_change_preserving_audio(runtime->execution);
        runtime->machine_locked = false;
    }
    after_capture(runtime, false);
    frontend_state_unregister_ui();
    frontend_state_set_save_hooks(&runtime->frontend, NULL, NULL);
    frontend_state_set_hooks(&runtime->frontend, NULL, NULL, NULL);
    runtime->execution = NULL;
    runtime->restored = NULL;
    runtime->restored_context = NULL;
}
