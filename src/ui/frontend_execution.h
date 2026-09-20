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

typedef struct {
    ExecutionControl execution;
    SDL_AudioDeviceID *audio_device;
    int audio_output_rate;
    const char *rom_path;
    const char *fds_bios_path;
    const char *studybox_bios_path;
    size_t *fds_side;
} FrontendExecutionRuntime;

void frontend_execution_init(FrontendExecutionRuntime *runtime,
                             SDL_AudioDeviceID *audio_device, int audio_output_rate,
                             const char *rom_path, const char *fds_bios_path,
                             const char *studybox_bios_path, size_t *fds_side);
bool frontend_execution_register_commands(FrontendExecutionRuntime *runtime);
bool frontend_execution_handle_shortcut(FrontendExecutionRuntime *runtime,
                                        const SDL_KeyboardEvent *event);
bool frontend_execution_run_frame(FrontendExecutionRuntime *runtime);
bool frontend_execution_paused(const FrontendExecutionRuntime *runtime);
double frontend_execution_speed(const FrontendExecutionRuntime *runtime);

#endif
