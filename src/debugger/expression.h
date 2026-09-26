/*
 * expression.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Bounded debugger address expressions. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_DEBUG_EXPRESSION_H
#define CUPID_DEBUG_EXPRESSION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum { DEBUG_EXPRESSION_LIMIT = 96 };

/* Hexadecimal literals, #decimal, %binary, CPU registers, parentheses and
 * unsigned arithmetic/bitwise operators. Overflow and division by zero fail.
 * Evaluation observes the CPU; it never executes an emulated memory access. */
bool debug_expression_eval(const char *text, uint32_t *out, char *error, size_t error_size);

#endif
