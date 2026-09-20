/*
 * execution_control.h - Frontend emulation execution state
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef EXECUTION_CONTROL_H
#define EXECUTION_CONTROL_H

#include <stdbool.h>

typedef struct {
    bool paused;
    bool frame_advance_pending;
    bool fast_forward_held;
    bool fast_forward_toggled;
    bool loading_fast_forward;
    double speed;
    double fast_forward_speed;
} ExecutionControl;

typedef bool (*ExecutionFrameRunner)(void *userdata);

void execution_control_init(ExecutionControl *control);
void execution_control_set_paused(ExecutionControl *control, bool paused);
void execution_control_toggle_paused(ExecutionControl *control);
bool execution_control_request_frame(ExecutionControl *control);
bool execution_control_should_run_frame(const ExecutionControl *control);
void execution_control_frame_complete(ExecutionControl *control);
bool execution_control_set_speed(ExecutionControl *control, double speed);
bool execution_control_set_fast_forward_speed(ExecutionControl *control, double speed);
void execution_control_set_fast_forward_held(ExecutionControl *control, bool held);
void execution_control_toggle_fast_forward(ExecutionControl *control);
void execution_control_set_loading_fast_forward(ExecutionControl *control, bool active);
bool execution_control_fast_forward_active(const ExecutionControl *control);
double execution_control_effective_speed(const ExecutionControl *control);
bool execution_control_run_frame(ExecutionControl *control,
                                 ExecutionFrameRunner runner, void *userdata);

#endif
