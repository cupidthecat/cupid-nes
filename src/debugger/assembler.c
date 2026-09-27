/*
 * assembler.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Single-instruction 6502 assembly. SPDX-License-Identifier: GPL-3.0-or-later */
#include "assembler.h"
#include "opcodes.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static bool report(char *error, size_t size, const char *message) {
    if (error && size) {
        snprintf(error, size, "%s", message ? message : "");
    }

    return !message;
}

static void skip_space(const char **cursor) {
    while (isspace((unsigned char)**cursor)) {
        ++*cursor;
    }
}

static bool take(const char **cursor, char wanted) {
    skip_space(cursor);
    if (toupper((unsigned char)**cursor) != wanted) {
        return false;
    }

    ++*cursor;
    return true;
}

static bool end_of_line(const char *cursor) {
    skip_space(&cursor);
    return !*cursor || *cursor == ';';
}

static int digit(unsigned char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }

    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }

    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }

    return -1;
}

static bool number(const char **cursor, bool immediate, uint16_t *value, bool *wide) {
    skip_space(cursor);
    const char *p = *cursor;
    bool negative = *p == '-';
    if (negative) {
        ++p;
    }

    unsigned base = 10;
    if (*p == '$') {
        base = 16;
        ++p;
    } else if (*p == '0' && (p[1] == 'x' || p[1] == 'X')) {
        base = 16;
        p += 2;
    } else if (*p == '%') {
        base = 2;
        ++p;
    }

    unsigned parsed = 0, digits = 0;
    int next;
    while ((next = digit((unsigned char)*p)) >= 0 && (unsigned)next < base) {
        if (parsed > (UINT16_MAX - (unsigned)next) / base) {
            return false;
        }

        parsed = parsed * base + (unsigned)next;
        ++digits;
        ++p;
    }

    if (!digits || (base == 16 && digits > 4) || (base == 2 && digits > 16) ||
        (negative && (!immediate || base != 10 || parsed > 128))) {
        return false;
    }

    *wide = parsed > 255 || (base == 16 && digits > 2) || (base == 2 && digits > 8);
    *value = (uint16_t)(negative ? (256u - parsed) & 255u : parsed);
    *cursor = p;
    return true;
}

/* Return -1 for unsupported syntax, -2 for multiple encodings. */
static int opcode_for(const char *name, DebugAddressMode mode) {
    int found = -1;
    if (mode == IMP && !strcmp(name, "NOP")) {
        return 0xEA;
    }

    if (mode == IMM && !strcmp(name, "SBC")) {
        return 0xE9;
    }

    for (unsigned i = 0; i < 256; ++i) {
        DebugOpcode info = debugger_opcode((uint8_t)i);
        if (info.mode == mode && !strcmp(name, info.name)) {
            if (found >= 0) {
                return -2;
            }

            found = (int)i;
        }
    }

    return found;
}

bool debugger_assemble(uint16_t address, const char *source, DebugAssembly *out, char *error, size_t error_size) {
    if (!source || !out) {
        return report(error, error_size, "An instruction and output buffer are required");
    }

    size_t length = 0;
    while (length <= DEBUG_ASSEMBLY_TEXT_LIMIT && source[length]) {
        if (source[length] == '\r' || source[length] == '\n') {
            return report(error, error_size, "Enter one instruction on a single line");
        }

        ++length;
    }

    if (!length || length > DEBUG_ASSEMBLY_TEXT_LIMIT) {
        return report(error, error_size, "Enter one instruction of at most 127 characters");
    }

    const char *p = source;
    skip_space(&p);
    char name[4] = {0};
    for (unsigned i = 0; i < 3; ++i) {
        if (!isalpha((unsigned char)*p)) {
            return report(error, error_size, "Use a three-letter 6502 mnemonic");
        }

        name[i] = (char)toupper((unsigned char)*p++);
    }

    if (*p && *p != ';' && !isspace((unsigned char)*p)) {
        return report(error, error_size, "Separate the mnemonic and operand with a space");
    }

    bool known = false;
    for (unsigned i = 0; i < 256; ++i) {
        if (!strcmp(debugger_opcode((uint8_t)i).name, name)) {
            known = true;
            break;
        }
    }

    if (!known) {
        return report(error, error_size, "Unknown 6502 mnemonic");
    }

    skip_space(&p);
    DebugAddressMode mode = IMP;
    uint16_t operand = 0;
    bool wide = false;
    if (end_of_line(p)) {
        if (opcode_for(name, IMP) == -1 && opcode_for(name, ACC) >= 0) {
            mode = ACC;
        }
    } else if (toupper((unsigned char)*p) == 'A' && end_of_line(p + 1)) {
        mode = ACC;
        ++p;
    } else if (take(&p, '#')) {
        mode = IMM;
        if (!number(&p, true, &operand, &wide) || operand > 255) {
            return report(error, error_size, "An immediate operand must fit in one byte");
        }
    } else if (take(&p, '(')) {
        if (!number(&p, false, &operand, &wide)) {
            return report(error, error_size, "Invalid indirect address");
        }

        if (take(&p, ',')) {
            if (!take(&p, 'X') || !take(&p, ')')) {
                return report(error, error_size, "Use (address,X) for indexed indirect addressing");
            }

            mode = INDX;
        } else {
            if (!take(&p, ')')) {
                return report(error, error_size, "The indirect address needs a closing parenthesis");
            }

            mode = IND;
            if (take(&p, ',')) {
                if (!take(&p, 'Y')) {
                    return report(error, error_size, "Use (address),Y for indirect indexed addressing");
                }

                mode = INDY;
            }
        }

        if (mode != IND && operand > 255) {
            return report(error, error_size, "An indexed indirect pointer must be in zero page");
        }
    } else {
        if (!number(&p, false, &operand, &wide)) {
            return report(error, error_size, "Use a decimal, $hex, 0xhex or %binary operand within $0000-$FFFF");
        }

        mode = ABS;
        DebugAddressMode small = ZP;
        if (take(&p, ',')) {
            if (take(&p, 'X')) {
                mode = ABSX;
                small = ZPX;
            } else if (take(&p, 'Y')) {
                mode = ABSY;
                small = ZPY;
            } else {
                return report(error, error_size, "An index register must be X or Y");
            }
        }

        if (mode == ABS && opcode_for(name, REL) >= 0) {
            mode = REL;
            uint16_t delta = (uint16_t)(operand - (uint16_t)(address + 2u));
            if (delta > 127 && delta < 0xFF80) {
                return report(error, error_size,
                              "The branch target must be -128 to +127 bytes from the next instruction");
            }

            operand = (uint8_t)delta;
        } else if (!wide && opcode_for(name, small) != -1) {
            mode = small;
        }
    }

    if (!end_of_line(p)) {
        return report(error, error_size, "Unexpected text after the operand; enter one instruction");
    }

    int opcode = opcode_for(name, mode);
    if (opcode == -2) {
        return report(error, error_size,
                      "Several undocumented opcodes share this syntax; select exact bytes in the hex editor");
    }

    if (opcode < 0) {
        return report(error, error_size, "This instruction does not support the requested addressing mode");
    }

    DebugAssembly assembled = {address, {(uint8_t)opcode, 0, 0}, debugger_opcode((uint8_t)opcode).length};
    if (assembled.length > 1) {
        assembled.bytes[1] = (uint8_t)operand;
    }

    if (assembled.length > 2) {
        assembled.bytes[2] = (uint8_t)(operand >> 8);
    }

    *out = assembled;
    return report(error, error_size, NULL);
}

typedef struct {
    char name[64];
    uint16_t address;
} AssemblyLabel;

static bool symbol_start(unsigned char c) {
    return isalpha(c) || c == '_';
}

static bool symbol_char(unsigned char c) {
    return isalnum(c) || c == '_';
}

/* Decode only staged bytes; never read emulated memory for the generated pane. */
static void generated_instruction(const DebugAssembly *one, char *text, size_t size) {
    DebugOpcode opcode = debugger_opcode(one->bytes[0]);
    unsigned value = one->bytes[1] | ((unsigned)one->bytes[2] << 8);
    char operand[24] = "";
    switch (opcode.mode) {
    case IMP:
        break;
    case ACC:
        snprintf(operand, sizeof(operand), " A");
        break;
    case IMM:
        snprintf(operand, sizeof(operand), " #$%02X", value);
        break;
    case ZP:
        snprintf(operand, sizeof(operand), " $%02X", value);
        break;
    case ZPX:
        snprintf(operand, sizeof(operand), " $%02X,X", value);
        break;
    case ZPY:
        snprintf(operand, sizeof(operand), " $%02X,Y", value);
        break;
    case ABS:
        snprintf(operand, sizeof(operand), " $%04X", value);
        break;
    case ABSX:
        snprintf(operand, sizeof(operand), " $%04X,X", value);
        break;
    case ABSY:
        snprintf(operand, sizeof(operand), " $%04X,Y", value);
        break;
    case IND:
        snprintf(operand, sizeof(operand), " ($%04X)", value);
        break;
    case INDX:
        snprintf(operand, sizeof(operand), " ($%02X,X)", value);
        break;
    case INDY:
        snprintf(operand, sizeof(operand), " ($%02X),Y", value);
        break;
    case REL:
        value = (uint16_t)(one->address + 2 + (value < 128 ? (int)value : (int)value - 256));
        snprintf(operand, sizeof(operand), " $%04X", value);
        break;
    }
    snprintf(text, size, "%s%s", opcode.name, operand);
}

bool debugger_assemble_program(uint16_t address, const char *source, DebugAssemblyProgram *out, char *error,
                               size_t error_size) {
    if (!source || !out) {
        return report(error, error_size, "Source and output are required");
    }
    size_t size = 0;
    while (size <= DEBUG_ASSEMBLY_PROGRAM_TEXT_LIMIT && source[size]) {
        ++size;
    }
    if (size > DEBUG_ASSEMBLY_PROGRAM_TEXT_LIMIT) {
        return report(error, error_size, "Source exceeds 32768 characters");
    }
    DebugAssemblyProgram *staged = calloc(1, sizeof(*staged));
    if (!staged) {
        return report(error, error_size, "Could not allocate assembly output");
    }
    AssemblyLabel labels[256];
    unsigned label_count = 0, line_number = 0;
    char message[256] = "", diagnostic[300];
    bool ok = false;
    staged->address = address;
    for (unsigned pass = 0; pass < 2; ++pass) {
        const char *cursor = source;
        unsigned used = 0, instructions = 0;
        size_t listing_used = 0;
        line_number = 0;
        while (*cursor) {
            ++line_number;
            const char *end = cursor;
            while (*end && *end != '\n' && *end != '\r') {
                ++end;
            }
            size_t length = (size_t)(end - cursor);
            if (length > DEBUG_ASSEMBLY_TEXT_LIMIT) {
                strcpy(message, "A source line exceeds 127 characters");
                goto done;
            }
            char line[DEBUG_ASSEMBLY_TEXT_LIMIT + 1];
            memcpy(line, cursor, length);
            line[length] = 0;
            cursor = end;
            if (*cursor == '\r') {
                ++cursor;
            }
            if (*cursor == '\n') {
                ++cursor;
            }
            char *comment = strchr(line, ';');
            if (comment) {
                *comment = 0;
            }
            const char *p = line;
            skip_space(&p);
            const char *name = p;
            if (symbol_start((unsigned char)*p)) {
                while (symbol_char((unsigned char)*p)) {
                    ++p;
                }
                const char *name_end = p;
                skip_space(&p);
                if (*p == ':') {
                    size_t n = (size_t)(name_end - name);
                    if (n >= sizeof(labels[0].name) || (unsigned)address + used > UINT16_MAX) {
                        strcpy(message, "Label is too long or outside address space");
                        goto done;
                    }
                    if (!pass) {
                        for (unsigned i = 0; i < label_count; ++i) {
                            if (strlen(labels[i].name) == n && !memcmp(labels[i].name, name, n)) {
                                strcpy(message, "Duplicate label");
                                goto done;
                            }
                        }
                        if (label_count == 256) {
                            strcpy(message, "Too many labels (maximum 256)");
                            goto done;
                        }
                        memcpy(labels[label_count].name, name, n);
                        labels[label_count].name[n] = 0;
                        labels[label_count++].address = (uint16_t)(address + used);
                    }
                    ++p;
                    skip_space(&p);
                } else {
                    p = name;
                }
            } else {
                p = name;
            }
            if (!*p) {
                continue;
            }
            if ((unsigned)address + used > UINT16_MAX) {
                strcpy(message, "Program crosses the end of address space");
                goto done;
            }
            char instruction[DEBUG_ASSEMBLY_TEXT_LIMIT + 1];
            snprintf(instruction, sizeof(instruction), "%s", p);
            /* Resolve a bare operand symbol, including indexed and indirect forms.
             * Immediate label expressions are deliberately unsupported. */
            const char *operand = p;
            while (isalpha((unsigned char)*operand)) {
                ++operand;
            }
            skip_space(&operand);
            if (*operand == '(') {
                ++operand;
                skip_space(&operand);
            }
            if (symbol_start((unsigned char)*operand) &&
                !(toupper((unsigned char)*operand) == 'A' && end_of_line(operand + 1))) {
                const char *tail = operand;
                while (symbol_char((unsigned char)*tail)) {
                    ++tail;
                }
                size_t n = (size_t)(tail - operand);
                unsigned value = 0;
                if (pass) {
                    unsigned i;
                    for (i = 0; i < label_count; ++i) {
                        if (strlen(labels[i].name) == n && !memcmp(labels[i].name, operand, n)) {
                            break;
                        }
                    }
                    if (i == label_count) {
                        strcpy(message, "Undefined label");
                        goto done;
                    }
                    value = labels[i].address;
                } else {
                    char mnemonic[4] = {0};
                    for (unsigned i = 0; i < 3 && p[i]; ++i) {
                        mnemonic[i] = (char)toupper((unsigned char)p[i]);
                    }
                    if (opcode_for(mnemonic, REL) >= 0) {
                        value = (uint16_t)(address + used + 2);
                    }
                }
                int written =
                    snprintf(instruction, sizeof(instruction), "%.*s$%04X%s", (int)(operand - p), p, value, tail);
                if (written < 0 || (size_t)written >= sizeof(instruction)) {
                    strcpy(message, "Expanded instruction exceeds line limit");
                    goto done;
                }
            }
            DebugAssembly one;
            if (!debugger_assemble((uint16_t)(address + used), instruction, &one, message, sizeof(message))) {
                goto done;
            }
            if (used + one.length > DEBUG_ASSEMBLY_PROGRAM_LIMIT) {
                strcpy(message, "Program exceeds 4096 bytes");
                goto done;
            }
            if (pass) {
                memcpy(staged->bytes + used, one.bytes, one.length);
                char bytes[10] = "";
                for (unsigned i = 0; i < one.length; ++i) {
                    snprintf(bytes + i * 3, sizeof(bytes) - i * 3, "%02X ", one.bytes[i]);
                }
                char decoded[32];
                generated_instruction(&one, decoded, sizeof(decoded));
                int n = snprintf(staged->listing + listing_used, sizeof(staged->listing) - listing_used,
                                 "$%04X  %-9s %s\n", one.address, bytes, decoded);
                if (n < 0 || (size_t)n >= sizeof(staged->listing) - listing_used) {
                    strcpy(message, "Generated listing exceeds its size limit");
                    goto done;
                }
                listing_used += (size_t)n;
            }
            used += one.length;
            ++instructions;
            if (instructions > 1 && (unsigned)address + used > 65536u) {
                strcpy(message, "Program crosses the end of address space");
                goto done;
            }
        }
        if (!used) {
            strcpy(message, "Enter at least one instruction");
            goto done;
        }
        staged->length = used;
    }
    *out = *staged;
    ok = true;
done:
    free(staged);
    if (ok) {
        return report(error, error_size, NULL);
    }
    snprintf(diagnostic, sizeof(diagnostic), "Line %u: %s", line_number, message);
    return report(error, error_size, diagnostic);
}
