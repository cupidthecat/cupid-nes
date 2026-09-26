/*
 * assembler_frontend.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Multiline assembler controls. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_ASSEMBLER_FRONTEND_H
#define CUPID_ASSEMBLER_FRONTEND_H

#include "frontend_execution.h"

enum { ASSEMBLER_FRONTEND_PANEL = 0x1389 };

enum {
    ASSEMBLER_SPACE = 1,
    ASSEMBLER_ADDRESS,
    ASSEMBLER_SOURCE,
    ASSEMBLER_PREVIEW,
    ASSEMBLER_BYTES,
    ASSEMBLER_BACKING,
    ASSEMBLER_APPLY,
    ASSEMBLER_PC,
    ASSEMBLER_CONTEXT,
    ASSEMBLER_SYNTAX
};

/* ASSEMBLER_SOURCE accepts a bounded multiline program. ASSEMBLER_BYTES is
 * the complete generated listing; ASSEMBLER_CONTEXT retains existing memory
 * disassembly. Source/target/address edits invalidate the transactional preview. */
typedef struct AssemblerFrontend AssemblerFrontend;
AssemblerFrontend *assembler_frontend_create(FrontendExecutionRuntime *execution);
bool assembler_frontend_register(AssemblerFrontend *frontend);
void assembler_frontend_unregister(AssemblerFrontend *frontend);
void assembler_frontend_image_changed(AssemblerFrontend *frontend);
void assembler_frontend_destroy(AssemblerFrontend *frontend);

#endif
