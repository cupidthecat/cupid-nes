/*
 * assembler.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Single-instruction 6502 assembly. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_DEBUG_ASSEMBLER_H
#define CUPID_DEBUG_ASSEMBLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum { DEBUG_ASSEMBLY_TEXT_LIMIT = 127 };

typedef struct {
    uint16_t address;
    uint8_t bytes[3];
    uint8_t length;
} DebugAssembly;

/* No emulated reads or writes. Failure retains out. Numeric operands use
 * decimal, $/0x hex or %binary. Hex/binary width can force absolute mode.
 * Canonical official encodings win; duplicate undocumented encodings fail. */
bool debugger_assemble(uint16_t address, const char *source, DebugAssembly *out, char *error, size_t error_size);

#endif
