/*
 * state_runtime.c - Live save-state lifecycle integration
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "state_runtime.h"
#include "../debugger/debugger.h"
#include <string.h>

static bool before_load(void *context, char *error, size_t error_size) {
    StateRuntime *runtime = context;
    if (!runtime || !runtime->execution) return false;
    FrontendExecutionRuntime *execution = runtime->execution;
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
        frontend_execution_sync_debugger(runtime->execution);
        if (runtime->restored) runtime->restored(runtime->restored_context);
    }
    if (runtime->machine_locked) {
        frontend_execution_end_machine_change(runtime->execution);
        runtime->machine_locked = false;
    }
}

void state_runtime_init(StateRuntime *runtime, FrontendSettings *settings,
                        const char *slot_directory, FrontendExecutionRuntime *execution) {
    if (!runtime) return;
    memset(runtime, 0, sizeof(*runtime));
    runtime->execution = execution;
    frontend_state_init(&runtime->frontend, settings, slot_directory);
    frontend_state_set_hooks(&runtime->frontend, before_load, after_load, runtime);
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
        frontend_execution_end_machine_change(runtime->execution);
        runtime->machine_locked = false;
    }
    frontend_state_unregister_ui();
    frontend_state_set_hooks(&runtime->frontend, NULL, NULL, NULL);
    runtime->execution = NULL;
    runtime->restored = NULL;
    runtime->restored_context = NULL;
}
