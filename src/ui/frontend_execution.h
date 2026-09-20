/*
 * frontend_execution.h - SDL execution controls and machine commands
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_EXECUTION_H
#define FRONTEND_EXECUTION_H

#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stddef.h>
#include "execution_control.h"
#include "../replay/rewind.h"

typedef struct FrontendExecutionRuntime {
    ExecutionControl execution;
    SDL_AudioDeviceID *audio_device;
    int audio_output_rate;
    const char *rom_path;
    const char *fds_bios_path;
    const char *studybox_bios_path;
    size_t *fds_side;
    uint64_t debugger_pause_revision;
    NesRewindHistory rewind;
    unsigned rewind_seconds;
    unsigned run_ahead_frames;
    NesReplayResult replay_status;
    NesStateResult replay_state_status;
    bool (*before_machine_change)(void *context, char *error, size_t error_size);
    void *machine_change_context;
} FrontendExecutionRuntime;

void frontend_execution_init(FrontendExecutionRuntime *runtime,
                             SDL_AudioDeviceID *audio_device, int audio_output_rate,
                             const char *rom_path, const char *fds_bios_path,
                             const char *studybox_bios_path, size_t *fds_side);
bool frontend_execution_register_commands(FrontendExecutionRuntime *runtime);
bool frontend_execution_handle_shortcut(FrontendExecutionRuntime *runtime,
                                        const SDL_KeyboardEvent *event);
bool frontend_execution_run_frame(FrontendExecutionRuntime *runtime);
void frontend_execution_sync_debugger(FrontendExecutionRuntime *runtime);
bool frontend_execution_paused(const FrontendExecutionRuntime *runtime);
double frontend_execution_speed(const FrontendExecutionRuntime *runtime);

bool frontend_execution_set_rewind_seconds(FrontendExecutionRuntime *runtime,
                                           unsigned seconds);
unsigned frontend_execution_rewind_seconds(const FrontendExecutionRuntime *runtime);
size_t frontend_execution_rewind_available(const FrontendExecutionRuntime *runtime);
bool frontend_execution_rewind_step(FrontendExecutionRuntime *runtime,
                                    char *error, size_t error_size);
void frontend_execution_clear_timeline(FrontendExecutionRuntime *runtime);
void frontend_execution_shutdown(FrontendExecutionRuntime *runtime);

bool frontend_execution_set_run_ahead(FrontendExecutionRuntime *runtime, unsigned frames);
unsigned frontend_execution_run_ahead(const FrontendExecutionRuntime *runtime);
NesReplayResult frontend_execution_replay_status(const FrontendExecutionRuntime *runtime,
                                                 NesStateResult *state_result);

#endif
