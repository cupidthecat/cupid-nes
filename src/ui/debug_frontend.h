/*
 * debug_frontend.h - Desktop debugger, viewer, and Lua controls
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef DEBUG_FRONTEND_H
#define DEBUG_FRONTEND_H

#include "frontend_execution.h"

enum {
    VIEW_CONTROL_MODE = 1,
    VIEW_CONTROL_ADDRESS,
    VIEW_CONTROL_SNAPSHOT,
    VIEW_CONTROL_WATCH_ADDRESS,
    VIEW_CONTROL_WATCH_SIZE,
    VIEW_CONTROL_WATCH_LABEL,
    VIEW_CONTROL_WATCH_ADD,
    VIEW_CONTROL_WATCH_LIST,
    VIEW_CONTROL_WATCH_TOGGLE,
    VIEW_CONTROL_WATCH_REMOVE,
    VIEW_CONTROL_TRACE_LAST
};

enum {
    LUA_CONTROL_PATH = 1,
    LUA_CONTROL_LOAD,
    LUA_CONTROL_UNLOAD,
    LUA_CONTROL_BUDGET,
    LUA_CONTROL_STATE,
    LUA_CONTROL_OVERLAY,
    LUA_CONTROL_CLEAR_OVERLAY,
    LUA_CONTROL_ERROR,
    LUA_CONTROL_LOG
};

typedef struct DebugFrontend DebugFrontend;

DebugFrontend *debug_frontend_create(FrontendExecutionRuntime *execution);
bool debug_frontend_register_ui(DebugFrontend *frontend);
void debug_frontend_image_changed(DebugFrontend *frontend);
void debug_frontend_destroy(DebugFrontend *frontend);

#endif
