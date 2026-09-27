/*
 * state_recorder.h - Periodic recovery history with transactional retention
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_STATE_RECORDER_H
#define CUPID_STATE_RECORDER_H
#include "state_runtime.h"
#include "frontend_session.h"

enum { STATE_RECORDER_MAX = 64 };

typedef struct {
    bool automatic;
    bool time_based;
    unsigned interval; /* frames, or milliseconds of completed emulation */
    unsigned retain;
} StateRecorderOptions;

typedef struct {
    StateRuntime *state;
    char directory[2048], key[41];
    StateRecorderOptions options;
    bool running;
    uint64_t frames, milliseconds;
    unsigned slots[STATE_RECORDER_MAX]; /* oldest to newest */
    size_t count;
    char status[192];
} StateRecorder;

void state_recorder_init(StateRecorder *recorder, StateRuntime *state, const char *directory);
bool state_recorder_configure(StateRecorder *recorder, const StateRecorderOptions *options);
bool state_recorder_image_changed(StateRecorder *recorder, const FrontendSession *session, char *error,
                                  size_t error_size);
void state_recorder_restart_clock(StateRecorder *recorder);
/* Call once per completed, non-speculative frame, excluding pause and rewind. */
bool state_recorder_tick(StateRecorder *recorder, uint64_t emulated_milliseconds, char *error, size_t error_size);
bool state_recorder_capture(StateRecorder *recorder, char *error, size_t error_size);
bool state_recorder_restore(StateRecorder *recorder, size_t index, char *error, size_t error_size);
#endif
