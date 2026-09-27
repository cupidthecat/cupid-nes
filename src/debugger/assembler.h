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

enum {
    DEBUG_ASSEMBLY_PROGRAM_TEXT_LIMIT = 32768,
    DEBUG_ASSEMBLY_PROGRAM_LIMIT = 4096,
    DEBUG_ASSEMBLY_LISTING_LIMIT = DEBUG_ASSEMBLY_PROGRAM_LIMIT * 40 + 1
};

typedef struct {
    uint16_t address;
    unsigned length;
    uint8_t bytes[DEBUG_ASSEMBLY_PROGRAM_LIMIT];
    char listing[DEBUG_ASSEMBLY_LISTING_LIMIT];
} DebugAssemblyProgram;

/* Two passes; case-sensitive labels, one instruction per line, semicolon comments.
 * Symbolic addresses use absolute width (no zero-page relaxation). No directives
 * or expressions. Failure retains out, with a one-based source line diagnostic.
 * A single instruction retains legacy address-wrap behavior; application still
 * rejects a target extending beyond its memory space. No machine access. */
bool debugger_assemble_program(uint16_t address, const char *source, DebugAssemblyProgram *out, char *error,
                               size_t error_size);

#endif
