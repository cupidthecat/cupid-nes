/*
 * opcodes.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Shared 6502 instruction metadata. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_DEBUG_OPCODES_H
#define CUPID_DEBUG_OPCODES_H

#include <stdint.h>

typedef enum { IMP, ACC, IMM, ZP, ZPX, ZPY, ABS, ABSX, ABSY, IND, INDX, INDY, REL } DebugAddressMode;

typedef struct {
    const char *name;
    DebugAddressMode mode;
    uint8_t length;
} DebugOpcode;

DebugOpcode debugger_opcode(uint8_t opcode);

#endif
