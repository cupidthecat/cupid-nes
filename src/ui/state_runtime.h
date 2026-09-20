/*
 * state_runtime.h - Live save-state lifecycle integration
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_STATE_RUNTIME_H
#define FRONTEND_STATE_RUNTIME_H

#include "frontend_execution.h"
#include "state_frontend.h"

typedef void (*StateRuntimeRestored)(void *context);

typedef struct {
    FrontendStateRuntime frontend;
    FrontendExecutionRuntime *execution;
    StateRuntimeRestored restored;
    void *restored_context;
    bool machine_locked;
    bool paused_before_load;
    SDL_AudioDeviceID save_audio_device;
    const char *const *protected_paths;
    size_t protected_path_count;
} StateRuntime;

void state_runtime_init(StateRuntime *runtime, FrontendSettings *settings,
                        const char *slot_directory, FrontendExecutionRuntime *execution);
void state_runtime_set_restored(StateRuntime *runtime, StateRuntimeRestored restored,
                                void *context);
void state_runtime_set_protected_paths(StateRuntime *runtime,
                                       const char *const *paths, size_t count);
bool state_runtime_register_ui(StateRuntime *runtime);
void state_runtime_shutdown(StateRuntime *runtime);

#endif
