/*
 * lifecycle_frontend.h - Session recovery controls and persistent preferences
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_LIFECYCLE_FRONTEND_H
#define CUPID_LIFECYCLE_FRONTEND_H
#include "session_actions.h"
#include "state_recorder.h"

enum { LIFECYCLE_PANEL = 0x26a0, LIFECYCLE_LOAD_LAST = 0x26a1, LIFECYCLE_UNLOAD = 0x26a2 };

typedef struct {
    FrontendSessionActions *actions;
    StateRuntime *state;
    StateRecorder recorder;
    bool resume_on_start;
    char directory[2048];
    char status[256], interval[32], retain[32];
    char labels[STATE_RECORDER_MAX][64];
    const char *items[STATE_RECORDER_MAX];
    int selected;
} LifecycleFrontend;

bool lifecycle_init(LifecycleFrontend *runtime, FrontendSessionActions *actions, StateRuntime *state,
                    const char *directory, char *error, size_t error_size);
bool lifecycle_register_ui(LifecycleFrontend *runtime);
void lifecycle_unregister_ui(void);
bool lifecycle_save_preferences(LifecycleFrontend *runtime, char *error, size_t error_size);
/* Call before teardown, after capture/device finalization succeeds. Failure cancels exit. */
bool lifecycle_save_last(LifecycleFrontend *runtime, char *error, size_t error_size);
bool lifecycle_load_last(void *context, char *error, size_t error_size);
bool lifecycle_resume_startup(LifecycleFrontend *runtime, bool explicit_image, char *error, size_t error_size);
void lifecycle_image_changed(LifecycleFrontend *runtime);
#endif
