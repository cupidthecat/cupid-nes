/*
 * replay_frontend.h - Rewind and run-ahead command/panel registration
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef REPLAY_FRONTEND_H
#define REPLAY_FRONTEND_H

#include <stdbool.h>

typedef struct FrontendExecutionRuntime FrontendExecutionRuntime;

enum {
    REPLAY_COMMAND_REWIND_FRAME = 0x1700,
    REPLAY_COMMAND_RUNAHEAD_CYCLE = 0x1701,
    REPLAY_PANEL = 0x1700,
    REPLAY_CONTROL_HISTORY = 1,
    REPLAY_CONTROL_RUNAHEAD,
    REPLAY_CONTROL_REWIND,
    REPLAY_CONTROL_CLEAR,
    REPLAY_CONTROL_STATUS
};

bool replay_frontend_register(FrontendExecutionRuntime *runtime);
void replay_frontend_unregister(void);

#endif
