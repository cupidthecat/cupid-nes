/*
 * header_editor_frontend.h - Cartridge header editor
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_HEADER_EDITOR_FRONTEND_H
#define CUPID_HEADER_EDITOR_FRONTEND_H
#include <stdbool.h>
struct FrontendExecutionRuntime;

enum {
    HEADER_EDITOR_PANEL = 0x2b00,
    HEADER_EDITOR_OPEN = 1,
    HEADER_EDITOR_SAVE,
    HEADER_EDITOR_RESET,
    HEADER_EDITOR_FIELD_BASE = 100
};

bool header_editor_frontend_register(struct FrontendExecutionRuntime *execution);
void header_editor_frontend_unregister(void);
#endif
