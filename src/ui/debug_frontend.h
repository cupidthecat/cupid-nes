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

typedef struct DebugFrontend DebugFrontend;

DebugFrontend *debug_frontend_create(FrontendExecutionRuntime *execution);
bool debug_frontend_register_ui(DebugFrontend *frontend);
void debug_frontend_image_changed(DebugFrontend *frontend);
void debug_frontend_destroy(DebugFrontend *frontend);

#endif
