/*
 * disassembly.c - Complete 6502 opcode names and addressing modes
 *
 * Copyright (C) 2014-2026 Sour and contributors
 * Copyright (C) 2026 Francis Hagan
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License, version 3 or later.
 * This program is distributed without any warranty.
 */
#include "debugger.h"
#include <stdio.h>
#include <string.h>

typedef enum {
    IMP, ACC, IMM, ZP, ZPX, ZPY, ABS, ABSX, ABSY, IND, INDX, INDY, REL
} AddressMode;

static const char opcode_names[256][4] = {
    "BRK","ORA","STP","SLO","NOP","ORA","ASL","SLO","PHP","ORA","ASL","ANC","NOP","ORA","ASL","SLO",
    "BPL","ORA","STP","SLO","NOP","ORA","ASL","SLO","CLC","ORA","NOP","SLO","NOP","ORA","ASL","SLO",
    "JSR","AND","STP","RLA","BIT","AND","ROL","RLA","PLP","AND","ROL","ANC","BIT","AND","ROL","RLA",
    "BMI","AND","STP","RLA","NOP","AND","ROL","RLA","SEC","AND","NOP","RLA","NOP","AND","ROL","RLA",
    "RTI","EOR","STP","SRE","NOP","EOR","LSR","SRE","PHA","EOR","LSR","ALR","JMP","EOR","LSR","SRE",
    "BVC","EOR","STP","SRE","NOP","EOR","LSR","SRE","CLI","EOR","NOP","SRE","NOP","EOR","LSR","SRE",
    "RTS","ADC","STP","RRA","NOP","ADC","ROR","RRA","PLA","ADC","ROR","ARR","JMP","ADC","ROR","RRA",
    "BVS","ADC","STP","RRA","NOP","ADC","ROR","RRA","SEI","ADC","NOP","RRA","NOP","ADC","ROR","RRA",
    "NOP","STA","NOP","SAX","STY","STA","STX","SAX","DEY","NOP","TXA","ANE","STY","STA","STX","SAX",
    "BCC","STA","STP","SHA","STY","STA","STX","SAX","TYA","STA","TXS","TAS","SHY","STA","SHX","SHA",
    "LDY","LDA","LDX","LAX","LDY","LDA","LDX","LAX","TAY","LDA","TAX","LAX","LDY","LDA","LDX","LAX",
    "BCS","LDA","STP","LAX","LDY","LDA","LDX","LAX","CLV","LDA","TSX","LAS","LDY","LDA","LDX","LAX",
    "CPY","CMP","NOP","DCP","CPY","CMP","DEC","DCP","INY","CMP","DEX","SBX","CPY","CMP","DEC","DCP",
    "BNE","CMP","STP","DCP","NOP","CMP","DEC","DCP","CLD","CMP","NOP","DCP","NOP","CMP","DEC","DCP",
    "CPX","SBC","NOP","ISC","CPX","SBC","INC","ISC","INX","SBC","NOP","SBC","CPX","SBC","INC","ISC",
    "BEQ","SBC","STP","ISC","NOP","SBC","INC","ISC","SED","SBC","NOP","ISC","NOP","SBC","INC","ISC"
};

static const uint8_t opcode_modes[256] = {
    IMP, INDX, IMP, INDX, ZP,  ZP,  ZP,  ZP,  IMP, IMM,  ACC, IMM,  ABS,  ABS,  ABS,  ABS,
    REL, INDY, IMP, INDY, ZPX, ZPX, ZPX, ZPX, IMP, ABSY, IMP, ABSY, ABSX, ABSX, ABSX, ABSX,
    ABS, INDX, IMP, INDX, ZP,  ZP,  ZP,  ZP,  IMP, IMM,  ACC, IMM,  ABS,  ABS,  ABS,  ABS,
    REL, INDY, IMP, INDY, ZPX, ZPX, ZPX, ZPX, IMP, ABSY, IMP, ABSY, ABSX, ABSX, ABSX, ABSX,
    IMP, INDX, IMP, INDX, ZP,  ZP,  ZP,  ZP,  IMP, IMM,  ACC, IMM,  ABS,  ABS,  ABS,  ABS,
    REL, INDY, IMP, INDY, ZPX, ZPX, ZPX, ZPX, IMP, ABSY, IMP, ABSY, ABSX, ABSX, ABSX, ABSX,
    IMP, INDX, IMP, INDX, ZP,  ZP,  ZP,  ZP,  IMP, IMM,  ACC, IMM,  IND,  ABS,  ABS,  ABS,
    REL, INDY, IMP, INDY, ZPX, ZPX, ZPX, ZPX, IMP, ABSY, IMP, ABSY, ABSX, ABSX, ABSX, ABSX,
    IMM, INDX, IMM, INDX, ZP,  ZP,  ZP,  ZP,  IMP, IMM,  IMP, IMM,  ABS,  ABS,  ABS,  ABS,
    REL, INDY, IMP, INDY, ZPX, ZPX, ZPY, ZPY, IMP, ABSY, IMP, ABSY, ABSX, ABSX, ABSY, ABSY,
    IMM, INDX, IMM, INDX, ZP,  ZP,  ZP,  ZP,  IMP, IMM,  IMP, IMM,  ABS,  ABS,  ABS,  ABS,
    REL, INDY, IMP, INDY, ZPX, ZPX, ZPY, ZPY, IMP, ABSY, IMP, ABSY, ABSX, ABSX, ABSY, ABSY,
    IMM, INDX, IMM, INDX, ZP,  ZP,  ZP,  ZP,  IMP, IMM,  IMP, IMM,  ABS,  ABS,  ABS,  ABS,
    REL, INDY, IMP, INDY, ZPX, ZPX, ZPX, ZPX, IMP, ABSY, IMP, ABSY, ABSX, ABSX, ABSX, ABSX,
    IMM, INDX, IMM, INDX, ZP,  ZP,  ZP,  ZP,  IMP, IMM,  IMP, IMM,  ABS,  ABS,  ABS,  ABS,
    REL, INDY, IMP, INDY, ZPX, ZPX, ZPX, ZPX, IMP, ABSY, IMP, ABSY, ABSX, ABSX, ABSX, ABSX
};

bool debugger_disassemble(uint16_t address, DebugDisassembly *out) {
    if (!out) return false;
    memset(out, 0, sizeof(*out));
    out->address = address;
    uint8_t opcode = debugger_peek_cpu(address);
    AddressMode mode = (AddressMode)opcode_modes[opcode];
    out->bytes[0] = opcode;
    out->length = mode == IMP || mode == ACC ? 1u
                : mode == ABS || mode == ABSX || mode == ABSY || mode == IND ? 3u : 2u;
    for (unsigned i = 1; i < out->length; ++i)
        out->bytes[i] = debugger_peek_cpu((uint16_t)(address + i));

    unsigned byte = out->bytes[1];
    unsigned word = byte | ((unsigned)out->bytes[2] << 8);
    char operand[24] = "";
    switch (mode) {
        case IMP: break;
        case ACC: snprintf(operand, sizeof(operand), " A"); break;
        case IMM: snprintf(operand, sizeof(operand), " #$%02X", byte); break;
        case ZP: snprintf(operand, sizeof(operand), " $%02X", byte); break;
        case ZPX: snprintf(operand, sizeof(operand), " $%02X,X", byte); break;
        case ZPY: snprintf(operand, sizeof(operand), " $%02X,Y", byte); break;
        case ABS: snprintf(operand, sizeof(operand), " $%04X", word); break;
        case ABSX: snprintf(operand, sizeof(operand), " $%04X,X", word); break;
        case ABSY: snprintf(operand, sizeof(operand), " $%04X,Y", word); break;
        case IND: snprintf(operand, sizeof(operand), " ($%04X)", word); break;
        case INDX: snprintf(operand, sizeof(operand), " ($%02X,X)", byte); break;
        case INDY: snprintf(operand, sizeof(operand), " ($%02X),Y", byte); break;
        case REL: {
            int displacement = byte < 0x80 ? (int)byte : (int)byte - 256;
            unsigned target = (uint16_t)((int)address + 2 + displacement);
            snprintf(operand, sizeof(operand), " $%04X", target);
            break;
        }
    }

    char bytes[9];
    if (out->length == 1) snprintf(bytes, sizeof(bytes), "%02X      ", opcode);
    else if (out->length == 2) snprintf(bytes, sizeof(bytes), "%02X %02X   ", opcode, byte);
    else snprintf(bytes, sizeof(bytes), "%02X %02X %02X", opcode, byte, (unsigned)out->bytes[2]);
    snprintf(out->text, sizeof(out->text), "%04X  %s %s%s", (unsigned)address, bytes, opcode_names[opcode], operand);
    return true;
}
