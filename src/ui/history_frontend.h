/*
 * history_frontend.h - Visual rewind timeline controls
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_HISTORY_FRONTEND_H
#define CUPID_HISTORY_FRONTEND_H
#include "frontend_execution.h"
typedef struct FrontendHistory FrontendHistory;
FrontendHistory *frontend_history_create(FrontendExecutionRuntime *execution);
void frontend_history_destroy(FrontendHistory *history);
/* Close before replacing/resetting the session or changing its rewind limits. */
void frontend_history_close(FrontendHistory *history);
bool frontend_history_event(FrontendHistory *history, const SDL_Event *event);
void frontend_history_tick(FrontendHistory *history);
#endif
