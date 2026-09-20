/*
 * session_actions.h - Live image replacement and reload actions
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef FRONTEND_SESSION_ACTIONS_H
#define FRONTEND_SESSION_ACTIONS_H

#include <stdbool.h>
#include <stddef.h>
#include "frontend_execution.h"
#include "frontend_session.h"
#include "settings.h"

typedef struct {
    FrontendSession *session;
    FrontendSettings *settings;
    const char *recent_path;
    FrontendExecutionRuntime *execution;
    void (*image_changed)(void *context);
    void *image_changed_context;
} FrontendSessionActions;

void frontend_session_actions_init(FrontendSessionActions *actions,
                                   FrontendSession *session,
                                   FrontendSettings *settings,
                                   const char *recent_path);
void frontend_session_actions_set_image_changed(FrontendSessionActions *actions,
                                                void (*callback)(void *context),
                                                void *context);
void frontend_session_actions_set_execution(FrontendSessionActions *actions,
                                            FrontendExecutionRuntime *execution);
void frontend_image_request_apply_settings(FrontendImageRequest *request,
                                           const FrontendSettings *settings);
bool frontend_session_action_open(void *userdata, char *error, size_t error_size);
bool frontend_session_action_reload(void *userdata, char *error, size_t error_size);
bool frontend_session_action_open_path(FrontendSessionActions *actions, const char *path,
                                       char *error, size_t error_size);

#endif
