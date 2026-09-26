/*
 * debug_tools_frontend.h - Native debugger analysis panels
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_DEBUG_TOOLS_FRONTEND_H
#define CUPID_DEBUG_TOOLS_FRONTEND_H
#include "frontend_execution.h"

enum { DEBUG_TOOLS_PANEL_FIRST = 0x2400, DEBUG_TOOLS_PANEL_COUNT = 9 };

enum {
    DEBUG_TOOL_INFO = 1,
    DEBUG_TOOL_START,
    DEBUG_TOOL_CLEAR,
    DEBUG_TOOL_ROWS,
    DEBUG_TOOL_PREVIOUS,
    DEBUG_TOOL_NEXT,
    DEBUG_TOOL_FOLLOW,
    DEBUG_TOOL_COPY,
    DEBUG_TOOL_EXPORT_PATH,
    DEBUG_TOOL_EXPORT,
    DEBUG_TOOL_IMPORT_PATH,
    DEBUG_TOOL_IMPORT,
    DEBUG_TOOL_FIRST,
    DEBUG_TOOL_LAST,
    DEBUG_TOOL_APPLY,
    DEBUG_TOOL_FILTER,
    DEBUG_TOOL_OPTION,
    DEBUG_TOOL_ADDRESS,
    DEBUG_TOOL_NAME,
    DEBUG_TOOL_SPAN,
    DEBUG_TOOL_FILE,
    DEBUG_TOOL_LINE,
    DEBUG_TOOL_COMMENT,
    DEBUG_TOOL_ADD,
    DEBUG_TOOL_REMOVE,
    DEBUG_TOOL_ROOT,
    DEBUG_TOOL_BREAK,
    DEBUG_TOOL_PREVIEW,
    DEBUG_TOOL_CAPACITY,
    DEBUG_TOOL_COLUMNS,
    DEBUG_TOOL_PHYSICAL,
    DEBUG_TOOL_READS,
    DEBUG_TOOL_WRITES,
    DEBUG_TOOL_DEDUP,
    DEBUG_TOOL_DISASSEMBLY,
    DEBUG_TOOL_RETURN
};
typedef struct DebugToolsFrontend DebugToolsFrontend;
DebugToolsFrontend *debug_tools_frontend_create(FrontendExecutionRuntime *execution);
bool debug_tools_frontend_register(DebugToolsFrontend *frontend);
void debug_tools_frontend_unregister(DebugToolsFrontend *frontend);
void debug_tools_frontend_image_changed(DebugToolsFrontend *frontend);
void debug_tools_frontend_destroy(DebugToolsFrontend *frontend);
#endif
