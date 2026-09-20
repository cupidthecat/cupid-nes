/*
 * replay_frontend.h - Rewind, run-ahead, and input movie controls
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
    REPLAY_COMMAND_MOVIE_RECORD = 0x1702,
    REPLAY_COMMAND_MOVIE_PLAY = 0x1703,
    REPLAY_COMMAND_MOVIE_STOP = 0x1704,
    REPLAY_PANEL = 0x1700,
    REPLAY_CONTROL_HISTORY = 1,
    REPLAY_CONTROL_RUNAHEAD,
    REPLAY_CONTROL_REWIND,
    REPLAY_CONTROL_CLEAR,
    REPLAY_CONTROL_STATUS,
    REPLAY_CONTROL_MOVIE_PATH,
    REPLAY_CONTROL_MOVIE_START,
    REPLAY_CONTROL_MOVIE_RECORD,
    REPLAY_CONTROL_MOVIE_PLAY,
    REPLAY_CONTROL_MOVIE_STOP,
    REPLAY_CONTROL_MOVIE_STATUS,
    REPLAY_CONTROL_MOVIE_OPEN_FILE,
    REPLAY_CONTROL_MOVIE_SAVE_FILE
};

bool replay_frontend_register(FrontendExecutionRuntime *runtime);
void replay_frontend_refresh(FrontendExecutionRuntime *runtime);
void replay_frontend_unregister(void);

#endif
