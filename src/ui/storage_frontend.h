/* Storage location and database controls. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_STORAGE_FRONTEND_H
#define CUPID_STORAGE_FRONTEND_H
#include "session_actions.h"
#include "capture_frontend.h"
typedef struct {
    FrontendSessionActions *actions;
    FrontendExecutionRuntime *execution;
    NesCaptureFrontend *capture;
    const char *database_path;
    unsigned selected;
    char paths[12][4096];
    char status[256];
} FrontendStorage;
bool frontend_storage_register(FrontendStorage *storage,FrontendSessionActions *actions,
                               FrontendExecutionRuntime *execution,const char *database_path);
void frontend_storage_unregister(void);
#endif
