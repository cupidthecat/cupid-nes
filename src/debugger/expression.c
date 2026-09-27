/*
 * expression.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Bounded debugger address expressions. SPDX-License-Identifier: GPL-3.0-or-later */
#include "expression.h"
#include "debugger.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    const char *cursor;
    const char *error;
    DebugCpuSnapshot cpu;
} Parser;

static void spaces(Parser *p) {
    while (isspace((unsigned char)*p->cursor)) ++p->cursor;
}

static bool expression(Parser *p, unsigned precedence, unsigned depth, uint32_t *out);

static bool atom(Parser *p, unsigned depth, uint32_t *out) {
    spaces(p);
    if (depth > 16) {
        p->error = "Expression nesting exceeds 16 levels";
        return false;
    }
    if (*p->cursor == '(') {
        ++p->cursor;
        if (!expression(p, 1, depth + 1, out)) return false;
        spaces(p);
        if (*p->cursor++ != ')') {
            p->error = "Expected a closing parenthesis";
            return false;
        }
        return true;
    }
    if (*p->cursor == '+' || *p->cursor == '~') {
        char op = *p->cursor++;
        if (!atom(p, depth + 1, out)) return false;
        if (op == '~') *out = ~*out;
        return true;
    }

    unsigned base = 16;
    bool explicit_base = true;
    if (*p->cursor == '$') ++p->cursor;
    else if (*p->cursor == '#') { ++p->cursor; base = 10; }
    else if (*p->cursor == '%') { ++p->cursor; base = 2; }
    else if (p->cursor[0] == '0' && (p->cursor[1] == 'x' || p->cursor[1] == 'X')) p->cursor += 2;
    else explicit_base = false;

    const char *start = p->cursor;
    while (isalnum((unsigned char)*p->cursor) || *p->cursor == '_') ++p->cursor;
    size_t length = (size_t)(p->cursor - start);
    if (!length) {
        p->error = "Expected a number, CPU register or parenthesis";
        return false;
    }
    if (!explicit_base && length <= 2) {
        char name[3] = {0};
        for (size_t i = 0; i < length; ++i) name[i] = (char)toupper((unsigned char)start[i]);
        if (!strcmp(name, "PC")) { *out = p->cpu.cpu.pc; return true; }
        if (!strcmp(name, "SP")) { *out = p->cpu.cpu.sp; return true; }
        if (!strcmp(name, "A")) { *out = p->cpu.cpu.a; return true; }
        if (!strcmp(name, "X")) { *out = p->cpu.cpu.x; return true; }
        if (!strcmp(name, "Y")) { *out = p->cpu.cpu.y; return true; }
    }
    uint32_t value = 0;
    for (size_t i = 0; i < length; ++i) {
        unsigned c = (unsigned)toupper((unsigned char)start[i]);
        unsigned digit = c >= '0' && c <= '9' ? c - '0' : c >= 'A' && c <= 'F' ? c - 'A' + 10 : 16;
        if (digit >= base) {
            p->error = "Unknown register or invalid numeric literal";
            return false;
        }
        if (value > (UINT32_MAX - digit) / base) {
            p->error = "Numeric literal exceeds 32 bits";
            return false;
        }
        value = value * base + digit;
    }
    *out = value;
    return true;
}

static unsigned operator_precedence(const char *text, unsigned *length) {
    *length = 1;
    switch (*text) {
    case '|': return 1;
    case '^': return 2;
    case '&': return 3;
    case '<': case '>':
        if (text[1] == text[0]) { *length = 2; return 4; }
        return 0;
    case '+': case '-': return 5;
    case '*': case '/': return 6;
    default: return 0;
    }
}

static bool expression(Parser *p, unsigned precedence, unsigned depth, uint32_t *out) {
    uint32_t left;
    if (!atom(p, depth, &left)) return false;
    for (;;) {
        spaces(p);
        unsigned length, next = operator_precedence(p->cursor, &length);
        if (next < precedence) break;
        char op = *p->cursor;
        p->cursor += length;
        uint32_t right;
        if (!expression(p, next + 1, depth + 1, &right)) return false;
        uint64_t result = left;
        switch (op) {
        case '|': result = left | right; break;
        case '^': result = left ^ right; break;
        case '&': result = left & right; break;
        case '+': result = (uint64_t)left + right; break;
        case '*': result = (uint64_t)left * right; break;
        case '-':
            if (right > left) { p->error = "Address arithmetic is negative"; return false; }
            result = left - right;
            break;
        case '/':
            if (!right) { p->error = "Division by zero"; return false; }
            result = left / right;
            break;
        case '<': case '>':
            if (right >= 32) { p->error = "Shift count must be below 32"; return false; }
            result = op == '<' ? (uint64_t)left << right : left >> right;
            break;
        default: return false;
        }
        if (result > UINT32_MAX) { p->error = "Address arithmetic exceeds 32 bits"; return false; }
        left = (uint32_t)result;
    }
    *out = left;
    return true;
}

bool debug_expression_eval(const char *text, uint32_t *out, char *error, size_t error_size) {
    Parser parser = {.cursor = text, .error = "Invalid address expression"};
    bool valid = text && out;
    if (valid) {
        size_t length = 0;
        while (length < DEBUG_EXPRESSION_LIMIT && text[length]) ++length;
        valid = length > 0 && length < DEBUG_EXPRESSION_LIMIT;
    }
    uint32_t value = 0;
    if (valid) {
        debugger_get_cpu(&parser.cpu);
        valid = expression(&parser, 1, 0, &value);
        if (valid && *parser.cursor) {
            parser.error = "Unexpected text after the address expression";
            valid = false;
        }
    }
    if (valid) *out = value;
    if (error && error_size) snprintf(error, error_size, "%s", valid ? "" : parser.error);
    return valid;
}
