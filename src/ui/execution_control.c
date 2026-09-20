/*
 * execution_control.c - Frontend emulation execution state
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "execution_control.h"
#include <math.h>
#include <string.h>

enum {
    EXECUTION_SPEED_MIN_PERCENT = 10,
    EXECUTION_SPEED_MAX_PERCENT = 1600
};

static bool valid_speed(double speed) {
    return isfinite(speed)
        && speed >= EXECUTION_SPEED_MIN_PERCENT / 100.0
        && speed <= EXECUTION_SPEED_MAX_PERCENT / 100.0;
}

void execution_control_init(ExecutionControl *control) {
    if (!control) return;
    memset(control, 0, sizeof(*control));
    control->speed = 1.0;
    control->fast_forward_speed = 4.0;
}

void execution_control_set_paused(ExecutionControl *control, bool paused) {
    if (!control) return;
    control->paused = paused;
    if (!paused) control->frame_advance_pending = false;
}

void execution_control_toggle_paused(ExecutionControl *control) {
    if (!control) return;
    execution_control_set_paused(control, !control->paused);
}

bool execution_control_request_frame(ExecutionControl *control) {
    if (!control || !control->paused) return false;
    control->frame_advance_pending = true;
    return true;
}

bool execution_control_should_run_frame(const ExecutionControl *control) {
    return control && (!control->paused || control->frame_advance_pending);
}

void execution_control_frame_complete(ExecutionControl *control) {
    if (!control || !control->paused) return;
    control->frame_advance_pending = false;
}

bool execution_control_set_speed(ExecutionControl *control, double speed) {
    if (!control || !valid_speed(speed)) return false;
    control->speed = speed;
    return true;
}

bool execution_control_set_fast_forward_speed(ExecutionControl *control, double speed) {
    if (!control || !valid_speed(speed)) return false;
    control->fast_forward_speed = speed;
    return true;
}

void execution_control_set_fast_forward_held(ExecutionControl *control, bool held) {
    if (control) control->fast_forward_held = held;
}

void execution_control_toggle_fast_forward(ExecutionControl *control) {
    if (control) control->fast_forward_toggled = !control->fast_forward_toggled;
}

bool execution_control_fast_forward_active(const ExecutionControl *control) {
    return control && (control->fast_forward_held || control->fast_forward_toggled);
}

double execution_control_effective_speed(const ExecutionControl *control) {
    if (!control) return 1.0;
    return execution_control_fast_forward_active(control)
        ? control->fast_forward_speed : control->speed;
}

bool execution_control_run_frame(ExecutionControl *control,
                                 ExecutionFrameRunner runner, void *userdata) {
    if (!execution_control_should_run_frame(control) || !runner) return false;
    if (!runner(userdata)) return false;
    execution_control_frame_complete(control);
    return true;
}
