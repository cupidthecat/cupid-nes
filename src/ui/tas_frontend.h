/*
 * tas_frontend.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Desktop TAS actions. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_TAS_FRONTEND_H
#define CUPID_TAS_FRONTEND_H

#include "frontend_execution.h"
#include "../replay/tas_session.h"
#include "../replay/tas_project.h"

enum {
    TAS_PANEL = 0x1750,
    TAS_COMMAND_FRAME_BACK = 0x1751,
    TAS_CONTROL_OPEN = 1,
    TAS_CONTROL_NEW,
    TAS_CONTROL_SAVE,
    TAS_CONTROL_SAVE_AS,
    TAS_CONTROL_STOP,
    TAS_CONTROL_READ_ONLY,
    TAS_CONTROL_RECORD,
    TAS_CONTROL_RECORD_MODE,
    TAS_CONTROL_PLAYERS,
    TAS_CONTROL_SEEK,
    TAS_CONTROL_SKIP_LAG,
    TAS_CONTROL_PAUSE_FRAME,
    TAS_CONTROL_STATUS,
    TAS_CONTROL_DISCARD
};

bool tas_frontend_register(FrontendExecutionRuntime *runtime);
void tas_frontend_unregister(void);
bool tas_frontend_open(FrontendExecutionRuntime *runtime, const char *path, bool create, char *error, size_t capacity);
bool tas_frontend_save(FrontendExecutionRuntime *runtime, const char *path, char *error, size_t capacity);
bool tas_frontend_seek(FrontendExecutionRuntime *runtime, size_t frame, char *error, size_t capacity);
bool tas_frontend_changed(FrontendExecutionRuntime *runtime, char *error, size_t capacity);
bool tas_frontend_bookmark_load(FrontendExecutionRuntime *runtime, unsigned slot, char *error, size_t capacity);
bool tas_frontend_discard(FrontendExecutionRuntime *runtime, char *error, size_t capacity);
bool tas_frontend_parse_frame(const char *text, size_t *frame);

#endif
