/*
 * assembler_accuracy.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* 6502 assembly and guarded frontend edits. SPDX-License-Identifier: GPL-3.0-or-later */
#include "board_tests.h"
#include "../apu/apu.h"
#include "../debugger/assembler.h"
#include "../debugger/debugger.h"
#include "../debugger/memory_editor.h"
#include "../state/state.h"
#include "../system/execution_policy.h"
#include "../system/vs_system.h"
#include "../ui/assembler_frontend.h"
#include "../ui/debug_frontend.h"
#include "../ui/frontend_commands.h"
#include "../ui/frontend_panels.h"
#include "../ui/watch_frontend.h"
#include "../../include/globals.h"

static char assembly_error[256];

static int official_encodings(void) {
    /* Fixed instruction/byte pairs cover all 151 official encodings. */
    static const struct {
        const char *source;
        uint8_t bytes[3], length;
    } cases[] = {{"BRK", {0x00}, 1},
                 {"ORA ($34,X)", {0x01, 0x34}, 2},
                 {"ORA $34", {0x05, 0x34}, 2},
                 {"ASL $34", {0x06, 0x34}, 2},
                 {"PHP", {0x08}, 1},
                 {"ORA #$34", {0x09, 0x34}, 2},
                 {"ASL A", {0x0A}, 1},
                 {"ORA $1234", {0x0D, 0x34, 0x12}, 3},
                 {"ASL $1234", {0x0E, 0x34, 0x12}, 3},
                 {"BPL $8000", {0x10, 0xFE}, 2},
                 {"ORA ($34),Y", {0x11, 0x34}, 2},
                 {"ORA $34,X", {0x15, 0x34}, 2},
                 {"ASL $34,X", {0x16, 0x34}, 2},
                 {"CLC", {0x18}, 1},
                 {"ORA $1234,Y", {0x19, 0x34, 0x12}, 3},
                 {"ORA $1234,X", {0x1D, 0x34, 0x12}, 3},
                 {"ASL $1234,X", {0x1E, 0x34, 0x12}, 3},
                 {"JSR $1234", {0x20, 0x34, 0x12}, 3},
                 {"AND ($34,X)", {0x21, 0x34}, 2},
                 {"BIT $34", {0x24, 0x34}, 2},
                 {"AND $34", {0x25, 0x34}, 2},
                 {"ROL $34", {0x26, 0x34}, 2},
                 {"PLP", {0x28}, 1},
                 {"AND #$34", {0x29, 0x34}, 2},
                 {"ROL A", {0x2A}, 1},
                 {"BIT $1234", {0x2C, 0x34, 0x12}, 3},
                 {"AND $1234", {0x2D, 0x34, 0x12}, 3},
                 {"ROL $1234", {0x2E, 0x34, 0x12}, 3},
                 {"BMI $8000", {0x30, 0xFE}, 2},
                 {"AND ($34),Y", {0x31, 0x34}, 2},
                 {"AND $34,X", {0x35, 0x34}, 2},
                 {"ROL $34,X", {0x36, 0x34}, 2},
                 {"SEC", {0x38}, 1},
                 {"AND $1234,Y", {0x39, 0x34, 0x12}, 3},
                 {"AND $1234,X", {0x3D, 0x34, 0x12}, 3},
                 {"ROL $1234,X", {0x3E, 0x34, 0x12}, 3},
                 {"RTI", {0x40}, 1},
                 {"EOR ($34,X)", {0x41, 0x34}, 2},
                 {"EOR $34", {0x45, 0x34}, 2},
                 {"LSR $34", {0x46, 0x34}, 2},
                 {"PHA", {0x48}, 1},
                 {"EOR #$34", {0x49, 0x34}, 2},
                 {"LSR A", {0x4A}, 1},
                 {"JMP $1234", {0x4C, 0x34, 0x12}, 3},
                 {"EOR $1234", {0x4D, 0x34, 0x12}, 3},
                 {"LSR $1234", {0x4E, 0x34, 0x12}, 3},
                 {"BVC $8000", {0x50, 0xFE}, 2},
                 {"EOR ($34),Y", {0x51, 0x34}, 2},
                 {"EOR $34,X", {0x55, 0x34}, 2},
                 {"LSR $34,X", {0x56, 0x34}, 2},
                 {"CLI", {0x58}, 1},
                 {"EOR $1234,Y", {0x59, 0x34, 0x12}, 3},
                 {"EOR $1234,X", {0x5D, 0x34, 0x12}, 3},
                 {"LSR $1234,X", {0x5E, 0x34, 0x12}, 3},
                 {"RTS", {0x60}, 1},
                 {"ADC ($34,X)", {0x61, 0x34}, 2},
                 {"ADC $34", {0x65, 0x34}, 2},
                 {"ROR $34", {0x66, 0x34}, 2},
                 {"PLA", {0x68}, 1},
                 {"ADC #$34", {0x69, 0x34}, 2},
                 {"ROR A", {0x6A}, 1},
                 {"JMP ($1234)", {0x6C, 0x34, 0x12}, 3},
                 {"ADC $1234", {0x6D, 0x34, 0x12}, 3},
                 {"ROR $1234", {0x6E, 0x34, 0x12}, 3},
                 {"BVS $8000", {0x70, 0xFE}, 2},
                 {"ADC ($34),Y", {0x71, 0x34}, 2},
                 {"ADC $34,X", {0x75, 0x34}, 2},
                 {"ROR $34,X", {0x76, 0x34}, 2},
                 {"SEI", {0x78}, 1},
                 {"ADC $1234,Y", {0x79, 0x34, 0x12}, 3},
                 {"ADC $1234,X", {0x7D, 0x34, 0x12}, 3},
                 {"ROR $1234,X", {0x7E, 0x34, 0x12}, 3},
                 {"STA ($34,X)", {0x81, 0x34}, 2},
                 {"STY $34", {0x84, 0x34}, 2},
                 {"STA $34", {0x85, 0x34}, 2},
                 {"STX $34", {0x86, 0x34}, 2},
                 {"DEY", {0x88}, 1},
                 {"TXA", {0x8A}, 1},
                 {"STY $1234", {0x8C, 0x34, 0x12}, 3},
                 {"STA $1234", {0x8D, 0x34, 0x12}, 3},
                 {"STX $1234", {0x8E, 0x34, 0x12}, 3},
                 {"BCC $8000", {0x90, 0xFE}, 2},
                 {"STA ($34),Y", {0x91, 0x34}, 2},
                 {"STY $34,X", {0x94, 0x34}, 2},
                 {"STA $34,X", {0x95, 0x34}, 2},
                 {"STX $34,Y", {0x96, 0x34}, 2},
                 {"TYA", {0x98}, 1},
                 {"STA $1234,Y", {0x99, 0x34, 0x12}, 3},
                 {"TXS", {0x9A}, 1},
                 {"STA $1234,X", {0x9D, 0x34, 0x12}, 3},
                 {"LDY #$34", {0xA0, 0x34}, 2},
                 {"LDA ($34,X)", {0xA1, 0x34}, 2},
                 {"LDX #$34", {0xA2, 0x34}, 2},
                 {"LDY $34", {0xA4, 0x34}, 2},
                 {"LDA $34", {0xA5, 0x34}, 2},
                 {"LDX $34", {0xA6, 0x34}, 2},
                 {"TAY", {0xA8}, 1},
                 {"LDA #$34", {0xA9, 0x34}, 2},
                 {"TAX", {0xAA}, 1},
                 {"LDY $1234", {0xAC, 0x34, 0x12}, 3},
                 {"LDA $1234", {0xAD, 0x34, 0x12}, 3},
                 {"LDX $1234", {0xAE, 0x34, 0x12}, 3},
                 {"BCS $8000", {0xB0, 0xFE}, 2},
                 {"LDA ($34),Y", {0xB1, 0x34}, 2},
                 {"LDY $34,X", {0xB4, 0x34}, 2},
                 {"LDA $34,X", {0xB5, 0x34}, 2},
                 {"LDX $34,Y", {0xB6, 0x34}, 2},
                 {"CLV", {0xB8}, 1},
                 {"LDA $1234,Y", {0xB9, 0x34, 0x12}, 3},
                 {"TSX", {0xBA}, 1},
                 {"LDY $1234,X", {0xBC, 0x34, 0x12}, 3},
                 {"LDA $1234,X", {0xBD, 0x34, 0x12}, 3},
                 {"LDX $1234,Y", {0xBE, 0x34, 0x12}, 3},
                 {"CPY #$34", {0xC0, 0x34}, 2},
                 {"CMP ($34,X)", {0xC1, 0x34}, 2},
                 {"CPY $34", {0xC4, 0x34}, 2},
                 {"CMP $34", {0xC5, 0x34}, 2},
                 {"DEC $34", {0xC6, 0x34}, 2},
                 {"INY", {0xC8}, 1},
                 {"CMP #$34", {0xC9, 0x34}, 2},
                 {"DEX", {0xCA}, 1},
                 {"CPY $1234", {0xCC, 0x34, 0x12}, 3},
                 {"CMP $1234", {0xCD, 0x34, 0x12}, 3},
                 {"DEC $1234", {0xCE, 0x34, 0x12}, 3},
                 {"BNE $8000", {0xD0, 0xFE}, 2},
                 {"CMP ($34),Y", {0xD1, 0x34}, 2},
                 {"CMP $34,X", {0xD5, 0x34}, 2},
                 {"DEC $34,X", {0xD6, 0x34}, 2},
                 {"CLD", {0xD8}, 1},
                 {"CMP $1234,Y", {0xD9, 0x34, 0x12}, 3},
                 {"CMP $1234,X", {0xDD, 0x34, 0x12}, 3},
                 {"DEC $1234,X", {0xDE, 0x34, 0x12}, 3},
                 {"CPX #$34", {0xE0, 0x34}, 2},
                 {"SBC ($34,X)", {0xE1, 0x34}, 2},
                 {"CPX $34", {0xE4, 0x34}, 2},
                 {"SBC $34", {0xE5, 0x34}, 2},
                 {"INC $34", {0xE6, 0x34}, 2},
                 {"INX", {0xE8}, 1},
                 {"SBC #$34", {0xE9, 0x34}, 2},
                 {"NOP", {0xEA}, 1},
                 {"CPX $1234", {0xEC, 0x34, 0x12}, 3},
                 {"SBC $1234", {0xED, 0x34, 0x12}, 3},
                 {"INC $1234", {0xEE, 0x34, 0x12}, 3},
                 {"BEQ $8000", {0xF0, 0xFE}, 2},
                 {"SBC ($34),Y", {0xF1, 0x34}, 2},
                 {"SBC $34,X", {0xF5, 0x34}, 2},
                 {"INC $34,X", {0xF6, 0x34}, 2},
                 {"SED", {0xF8}, 1},
                 {"SBC $1234,Y", {0xF9, 0x34, 0x12}, 3},
                 {"SBC $1234,X", {0xFD, 0x34, 0x12}, 3},
                 {"INC $1234,X", {0xFE, 0x34, 0x12}, 3}};

    _Static_assert(sizeof(cases) / sizeof(*cases) == 151, "All official encodings are covered");
    for (size_t i = 0; i < sizeof(cases) / sizeof(*cases); ++i) {
        DebugAssembly result;
        memset(&result, 0xCC, sizeof(result));
        bool ok = debugger_assemble(0x8000, cases[i].source, &result, assembly_error, sizeof(assembly_error));
        if (!ok) {
            fprintf(stderr, "%s: %s\n", cases[i].source, assembly_error);
        }
        BOARD_CHECK(ok && !assembly_error[0]);
        BOARD_CHECK(result.address == 0x8000 && result.length == cases[i].length);
        BOARD_CHECK(!memcmp(result.bytes, cases[i].bytes, sizeof(result.bytes)));
    }
    return 0;
}

static int operand_boundaries(void) {
    static const struct {
        uint16_t address;
        const char *source;
        uint8_t bytes[3], length;
    } cases[] = {{0, " lda\t#-128 ; signed byte", {0xA9, 0x80}, 2},
                 {0, "LDA #-1", {0xA9, 0xFF}, 2},
                 {0, "LDA #-0", {0xA9, 0}, 2},
                 {0, "LDA #255", {0xA9, 0xFF}, 2},
                 {0, "LDA #%11111111", {0xA9, 0xFF}, 2},
                 {0, "LDA 16", {0xA5, 0x10}, 2},
                 {0, "LDA 256", {0xAD, 0, 1}, 3},
                 {0, "LDA $0010", {0xAD, 0x10, 0}, 3},
                 {0, "LDA $00ff,X", {0xBD, 0xFF, 0}, 3},
                 {0, "LDA 0X0010", {0xAD, 0x10, 0}, 3},
                 {0, "LDA 0x10", {0xA5, 0x10}, 2},
                 {0, "LDA %000000001", {0xAD, 1, 0}, 3},
                 {0, "LDA %00000001", {0xA5, 1}, 2},
                 {0, "LDA ( $ff , x )", {0xA1, 0xFF}, 2},
                 {0, "LDA ( 255 ) , y", {0xB1, 0xFF}, 2},
                 {0, "JMP $01", {0x4C, 1, 0}, 3},
                 {0, "JMP ($FFFF)", {0x6C, 0xFF, 0xFF}, 3},
                 {0, "ASL", {0x0A}, 1},
                 {0, "ror a; accumulator", {0x6A}, 1},
                 {0, "NOP; comment", {0xEA}, 1},
                 {0, "NOP $1234", {0x0C, 0x34, 0x12}, 3},
                 {0, "SLO $12", {0x07, 0x12}, 2},
                 {0, "RLA $1234", {0x2F, 0x34, 0x12}, 3},
                 {0, "SRE ($12,X)", {0x43, 0x12}, 2},
                 {0, "RRA ($12),Y", {0x73, 0x12}, 2},
                 {0, "SAX $12,Y", {0x97, 0x12}, 2},
                 {0, "LAX $1234,Y", {0xBF, 0x34, 0x12}, 3},
                 {0, "DCP $12,X", {0xD7, 0x12}, 2},
                 {0, "ISC $1234,X", {0xFF, 0x34, 0x12}, 3},
                 {0, "ALR #$FF", {0x4B, 0xFF}, 2},
                 {0, "ARR #$FF", {0x6B, 0xFF}, 2},
                 {0, "ANE #$12", {0x8B, 0x12}, 2},
                 {0, "SBX #$12", {0xCB, 0x12}, 2},
                 {0, "SHA ($12),Y", {0x93, 0x12}, 2},
                 {0, "TAS $1234,Y", {0x9B, 0x34, 0x12}, 3},
                 {0, "SHY $1234,X", {0x9C, 0x34, 0x12}, 3},
                 {0, "SHX $1234,Y", {0x9E, 0x34, 0x12}, 3},
                 {0, "LAS $1234,Y", {0xBB, 0x34, 0x12}, 3},
                 {0x8000, "BNE $8081", {0xD0, 0x7F}, 2},
                 {0x8000, "BNE $7F82", {0xD0, 0x80}, 2},
                 {0xFFFE, "BEQ $0000", {0xF0, 0}, 2},
                 {0xFFFF, "BEQ $0001", {0xF0, 0}, 2},
                 {0xFFFF, "BEQ $0080", {0xF0, 0x7F}, 2},
                 {0x0000, "BEQ $FF82", {0xF0, 0x80}, 2}};

    for (size_t i = 0; i < sizeof(cases) / sizeof(*cases); ++i) {
        DebugAssembly result;
        BOARD_CHECK(
            debugger_assemble(cases[i].address, cases[i].source, &result, assembly_error, sizeof(assembly_error)));
        BOARD_CHECK(result.address == cases[i].address && result.length == cases[i].length);
        BOARD_CHECK(!memcmp(result.bytes, cases[i].bytes, sizeof(result.bytes)) && !assembly_error[0]);
    }
    return 0;
}

static int rejected_syntax(void) {
    static const char *const invalid[] = {"",
                                          " ",
                                          "; no instruction",
                                          "LD",
                                          "LDAA $12",
                                          "XYZ",
                                          "LDA#$12",
                                          "LDA",
                                          "LDA A",
                                          "STA #1",
                                          "RTS $12",
                                          "JMP ($12),Y",
                                          "STX $0012,Y",
                                          "LDA #",
                                          "LDA #256",
                                          "LDA #-129",
                                          "LDA #-$01",
                                          "LDA #+1",
                                          "LDA #$100",
                                          "LDA $10000",
                                          "LDA 65536",
                                          "LDA $00000",
                                          "LDA %00000000000000001",
                                          "LDA %2",
                                          "LDA 0x",
                                          "LDA $",
                                          "LDA -1",
                                          "LDA 12garbage",
                                          "LDA $12 extra",
                                          "LDA ($12",
                                          "LDA ($12,Y)",
                                          "LDA ($100,X)",
                                          "LDA ($100),Y",
                                          "LDA ($12),X",
                                          "LDA ( )",
                                          "LDA $12,Z",
                                          "LDA $12,XX",
                                          "LDA $12,X,Y",
                                          "LDA [$12]",
                                          "LDA ($12))",
                                          "BNE $8082",
                                          "BNE $7F81",
                                          "BNE #1",
                                          "BNE $8000,X",
                                          "NOP NOP",
                                          "ANC #1",
                                          "STP",
                                          "NOP #1",
                                          "NOP $12",
                                          "NOP $1234,X",
                                          "NOP; comment\nLDA #1",
                                          "LDA\n#$12",
                                          "NOP\rSTA $12",
                                          "LDA #\xFF"};
    DebugAssembly before, result;
    memset(&before, 0xA5, sizeof(before));
    for (size_t i = 0; i < sizeof(invalid) / sizeof(*invalid); ++i) {
        memcpy(&result, &before, sizeof(result));
        bool rejected = !debugger_assemble(0x8000, invalid[i], &result, assembly_error, sizeof(assembly_error));
        if (!rejected) {
            fprintf(stderr, "Unexpectedly accepted: %s\n", invalid[i]);
        }
        BOARD_CHECK(rejected && assembly_error[0] && !memcmp(&before, &result, sizeof(result)));
    }
    char bounded[DEBUG_ASSEMBLY_TEXT_LIMIT + 2];
    memset(bounded, ' ', sizeof(bounded));
    memcpy(bounded, "NOP;", 4);
    bounded[DEBUG_ASSEMBLY_TEXT_LIMIT] = '\0';
    BOARD_CHECK(debugger_assemble(0, bounded, &result, NULL, 0) && result.bytes[0] == 0xEA);
    bounded[DEBUG_ASSEMBLY_TEXT_LIMIT] = ' ';
    bounded[DEBUG_ASSEMBLY_TEXT_LIMIT + 1] = '\0';
    memcpy(&result, &before, sizeof(result));
    BOARD_CHECK(!debugger_assemble(0, bounded, &result, NULL, 0) && !memcmp(&result, &before, sizeof(result)));
    BOARD_CHECK(!debugger_assemble(0, NULL, &result, NULL, 0));
    BOARD_CHECK(!debugger_assemble(0, "NOP", NULL, assembly_error, 1) && !assembly_error[0]);
    return 0;
}

static bool act(unsigned id, const char *value, int selected) {
    return frontend_panel_action(ASSEMBLER_FRONTEND_PANEL, id, value, selected, assembly_error, sizeof(assembly_error));
}

static bool control(unsigned id, FrontendPanelControl *out) {
    FrontendPanelControl controls[16];
    FrontendPanelModel model = {.controls = controls, .capacity = 16};
    if (!frontend_panel_snapshot(ASSEMBLER_FRONTEND_PANEL, &model, NULL, 0)) {
        return false;
    }
    for (size_t i = 0; i < model.count; ++i) {
        if (controls[i].id == id) {
            *out = controls[i];
            return true;
        }
    }
    return false;
}

static bool preview_at(int space, unsigned address, const char *source) {
    char text[16];
    snprintf(text, sizeof(text), "$%04X", address);
    return act(ASSEMBLER_SPACE, NULL, space) && act(ASSEMBLER_ADDRESS, text, 0) && act(ASSEMBLER_SOURCE, source, 0) &&
           act(ASSEMBLER_PREVIEW, NULL, 0);
}

static bool consume_unchanged(NesStateBlob *before) {
    NesStateBlob after = {0};
    bool equal = nes_state_capture(&after) == NES_STATE_OK && before->size == after.size &&
                 !memcmp(before->data, after.data, before->size);
    nes_state_blob_free(&after);
    nes_state_blob_free(before);
    return equal;
}

static int preview_and_apply(FrontendExecutionRuntime *execution, DebugFrontend *debug) {
    FrontendPanelControl c;
    BOARD_CHECK(control(ASSEMBLER_APPLY, &c) && !c.enabled && !act(ASSEMBLER_APPLY, NULL, 0));
    NesStateBlob before = {0};
    BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
    bool previewed = preview_at(0, 0x40, "LDA #$42");
    bool displayed = control(ASSEMBLER_BYTES, &c) && c.read_only && strstr(c.value, "A9 42");
    bool unchanged = consume_unchanged(&before);
    BOARD_CHECK(previewed && displayed && unchanged);
    BOARD_CHECK(control(ASSEMBLER_APPLY, &c) && c.enabled && act(ASSEMBLER_APPLY, NULL, 0));
    BOARD_CHECK(read_mem(0x40) == 0xA9 && read_mem(0x41) == 0x42);
    DebugDisassembly disassembly;
    BOARD_CHECK(debugger_disassemble(0x40, &disassembly) && strstr(disassembly.text, "LDA #$42"));
    BOARD_CHECK(preview_at(0, 0x840, "LDX #$23") && act(ASSEMBLER_APPLY, NULL, 0));
    BOARD_CHECK(read_mem(0x40) == 0xA2 && read_mem(0x41) == 0x23);
    BOARD_CHECK(preview_at(1, 0x60, "JMP $1234") && act(ASSEMBLER_APPLY, NULL, 0));
    BOARD_CHECK(read_mem(0x60) == 0x4C && read_mem(0x61) == 0x34 && read_mem(0x62) == 0x12);
    BOARD_CHECK(preview_at(2, 0x6000, "LDA #7") && act(ASSEMBLER_APPLY, NULL, 0));
    BOARD_CHECK(read_mem(0x6000) == 0xA9 && read_mem(0x6001) == 7);
    BOARD_CHECK(preview_at(0, 0x40, "NOP"));
    write_mem(0x40, 0x57);
    BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
    bool refused = !act(ASSEMBLER_APPLY, NULL, 0);
    unchanged = consume_unchanged(&before);
    BOARD_CHECK(refused && unchanged && read_mem(0x40) == 0x57 && read_mem(0x41) == 0x23);
    BOARD_CHECK(preview_at(0, 0x40, "NOP"));
    debugger_resume();
    debugger_pause();
    frontend_execution_sync_debugger(execution);
    BOARD_CHECK(!act(ASSEMBLER_APPLY, NULL, 0) && read_mem(0x40) == 0x57);
    BOARD_CHECK(preview_at(0, 0x40, "NOP"));
    debug_frontend_image_changed(debug);
    debugger_pause();
    frontend_execution_sync_debugger(execution);
    BOARD_CHECK(!act(ASSEMBLER_APPLY, NULL, 0) && read_mem(0x40) == 0x57);
    BOARD_CHECK(preview_at(0, 0x40, "NOP") && act(ASSEMBLER_SOURCE, "LDY #3", 0));
    BOARD_CHECK(control(ASSEMBLER_APPLY, &c) && !c.enabled && !act(ASSEMBLER_APPLY, NULL, 0));
    cpu.pc = 0x70;
    BOARD_CHECK(act(ASSEMBLER_PC, NULL, 0) && control(ASSEMBLER_ADDRESS, &c) && !strcmp(c.value, "$0070"));
    BOARD_CHECK(act(ASSEMBLER_ADDRESS, "PC + #2", 0) && act(ASSEMBLER_PREVIEW, NULL, 0));
    BOARD_CHECK(act(ASSEMBLER_APPLY, NULL, 0) && read_mem(0x72) == 0xA0 && read_mem(0x73) == 3);
    frontend_panel_set_session_active(false);
    refused = !act(ASSEMBLER_PREVIEW, NULL, 0) && !act(ASSEMBLER_APPLY, NULL, 0);
    frontend_panel_set_session_active(true);
    BOARD_CHECK(refused);
    return 0;
}

static int protected_targets(void) {
    const struct {
        int space;
        unsigned address;
    } targets[] = {{0, 0x2002}, {0, 0x2004}, {0, 0x2007}, {0, 0x4015}, {0, 0x4016}, {0, 0x5000}, {0, 0x8000},
                   {0, 0xFFFF}, {0, 0x1FFF}, {1, 0x7FF},  {1, 0x800},  {2, 0x5FFF}, {2, 0x7FFF}, {2, 0x8000}};

    for (size_t i = 0; i < sizeof(targets) / sizeof(*targets); ++i) {
        NesStateBlob before = {0};
        BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
        bool previewed = preview_at(targets[i].space, targets[i].address, "LDA #$55");
        FrontendPanelControl c;
        bool disabled = control(ASSEMBLER_APPLY, &c) && !c.enabled;
        bool refused = !act(ASSEMBLER_APPLY, NULL, 0);
        bool unchanged = consume_unchanged(&before);
        BOARD_CHECK(previewed && disabled && refused && unchanged);
    }
    BOARD_CHECK(!preview_at(0, 0x10000, "NOP") && !act(ASSEMBLER_APPLY, NULL, 0));
    char oversized[DEBUG_ASSEMBLY_TEXT_LIMIT + 2];
    memset(oversized, 'x', sizeof(oversized) - 1);
    oversized[sizeof(oversized) - 1] = '\0';
    BOARD_CHECK(preview_at(0, 0x40, "NOP"));
    BOARD_CHECK(!act(ASSEMBLER_SOURCE, oversized, 0) && !act(ASSEMBLER_ADDRESS, oversized, 0));
    FrontendPanelControl c;
    BOARD_CHECK(control(ASSEMBLER_SOURCE, &c) && !strcmp(c.value, "NOP"));
    BOARD_CHECK(control(ASSEMBLER_ADDRESS, &c) && !strcmp(c.value, "$0040"));
    BOARD_CHECK(!act(ASSEMBLER_SPACE, NULL, -1) && !act(ASSEMBLER_SPACE, NULL, 3));
    BOARD_CHECK(!act(0xFFFF, NULL, 0));
    BOARD_CHECK(!preview_at(0, 0x40, "LDA #256") && !act(ASSEMBLER_APPLY, NULL, 0));
    return 0;
}

static uint64_t history_hash(const NesRewindHistory *history) {
    uint64_t hash = UINT64_C(14695981039346656037);
    for (size_t i = 0; i < history->capacity; ++i) {
        const NesStateBlob *entry = &history->entries[i];
        hash = (hash ^ entry->size) * UINT64_C(1099511628211);
        for (size_t j = 0; j < entry->size; ++j) {
            hash = (hash ^ entry->data[j]) * UINT64_C(1099511628211);
        }
    }
    return hash;
}

static int timeline_and_policy(FrontendExecutionRuntime *execution) {
    cpu.pc = 0x8000;
    cpu.halted = false;
    write_mem(0x120, 0xEA);
    debugger_resume();
    execution_control_set_paused(&execution->execution, false);
    BOARD_CHECK(frontend_execution_set_rewind_seconds(execution, 1));
    BOARD_CHECK(frontend_execution_set_run_ahead(execution, 2));
    BOARD_CHECK(frontend_execution_run_frame(execution) && frontend_execution_run_frame(execution));
    debugger_pause();
    frontend_execution_sync_debugger(execution);
    BOARD_CHECK(frontend_execution_rewind_available(execution) == 2);
    size_t total = nes_rewind_bytes(&execution->rewind), head = execution->rewind.head;
    uint64_t history = history_hash(&execution->rewind);
    unsigned width = 0, height = 0;
    const uint32_t *pixels = nes_runahead_presented_frame(&width, &height);
    uint32_t saved[256 * 240];
    BOARD_CHECK(pixels && width == 256 && height == 240);
    memcpy(saved, pixels, sizeof(saved));
    BOARD_CHECK(preview_at(0, 0x120, "NOP"));
    NesStateBlob before = {0};
    BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
    bool applied = act(ASSEMBLER_APPLY, NULL, 0);
    bool unchanged = consume_unchanged(&before);
    BOARD_CHECK(applied && unchanged);
    BOARD_CHECK(frontend_execution_rewind_available(execution) == 2 && execution->rewind.head == head);
    BOARD_CHECK(nes_rewind_bytes(&execution->rewind) == total && history_hash(&execution->rewind) == history);
    BOARD_CHECK(nes_runahead_presented_frame(NULL, NULL) == pixels && !memcmp(pixels, saved, sizeof(saved)));
    BOARD_CHECK(preview_at(0, 0x120, "LDA #1"));
    const uint32_t policies[] = {
        NES_EXECUTION_MOVIE_RECORDING, NES_EXECUTION_MOVIE_PLAYBACK,
        NES_EXECUTION_NETPLAY,         NES_EXECUTION_SPECULATIVE,
        NES_EXECUTION_REWIND,          NES_EXECUTION_MOVIE_PLAYBACK | NES_EXECUTION_SPECULATIVE};
    for (size_t i = 0; i < sizeof(policies) / sizeof(*policies); ++i) {
        BOARD_CHECK(nes_execution_set_policy(policies[i]));
        BOARD_CHECK(nes_state_capture(&before) == NES_STATE_OK);
        FrontendPanelControl c;
        bool disabled = control(ASSEMBLER_APPLY, &c) && !c.enabled;
        bool refused = !act(ASSEMBLER_APPLY, NULL, 0);
        unchanged = consume_unchanged(&before);
        BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
        BOARD_CHECK(disabled && refused && unchanged && read_mem(0x120) == 0xEA);
        BOARD_CHECK(frontend_execution_rewind_available(execution) == 2 && execution->rewind.head == head);
        BOARD_CHECK(nes_rewind_bytes(&execution->rewind) == total && history_hash(&execution->rewind) == history);
        BOARD_CHECK(nes_runahead_presented_frame(NULL, NULL) == pixels && !memcmp(pixels, saved, sizeof(saved)));
    }
    BOARD_CHECK(act(ASSEMBLER_APPLY, NULL, 0) && read_mem(0x120) == 0xA9 && read_mem(0x121) == 1);
    BOARD_CHECK(!frontend_execution_rewind_available(execution) && !nes_rewind_bytes(&execution->rewind));
    BOARD_CHECK(nes_runahead_presented_frame(NULL, NULL) == NULL);
    BOARD_CHECK(frontend_execution_set_rewind_seconds(execution, 0));
    BOARD_CHECK(frontend_execution_set_run_ahead(execution, 0));
    return 0;
}

static bool foreign_snapshot(void *context, FrontendPanelModel *model, char *error, size_t error_size) {
    (void)context;
    (void)error;
    (void)error_size;
    model->status = "Retained owner";
    return true;
}

static int registration_rollback(FrontendExecutionRuntime *execution) {
    const unsigned collisions[] = {ASSEMBLER_FRONTEND_PANEL, WATCH_FRONTEND_PANEL};
    for (size_t i = 0; i < sizeof(collisions) / sizeof(*collisions); ++i) {
        FrontendPanelSpec foreign = {collisions[i], "Retained owner", "Tools", 0, foreign_snapshot, NULL, NULL};
        BOARD_CHECK(frontend_panel_register(&foreign));
        DebugFrontend *debug = debug_frontend_create(execution);
        BOARD_CHECK(debug);
        bool refused = !debug_frontend_register_ui(debug);
        FrontendPanelInfo info;
        bool retained = frontend_panel_count() == 1 && frontend_command_count() == 0 &&
                        frontend_panel_get(collisions[i], &info) && !strcmp(info.title, "Retained owner");
        debug_frontend_destroy(debug);
        retained = retained && frontend_panel_get(collisions[i], &info);
        BOARD_CHECK(refused && retained);
        BOARD_CHECK(frontend_panel_unregister(collisions[i]));
        debug = debug_frontend_create(execution);
        BOARD_CHECK(debug && debug_frontend_register_ui(debug));
        BOARD_CHECK(!debug_frontend_register_ui(debug));
        debug_frontend_destroy(debug);
        BOARD_CHECK(frontend_panel_count() == 0 && frontend_command_count() == 0);
    }
    AssemblerFrontend *first = assembler_frontend_create(execution);
    AssemblerFrontend *second = assembler_frontend_create(execution);
    BOARD_CHECK(first && second && assembler_frontend_register(first));
    BOARD_CHECK(!assembler_frontend_register(second));
    assembler_frontend_destroy(second);
    FrontendPanelInfo info;
    bool retained = frontend_panel_get(ASSEMBLER_FRONTEND_PANEL, &info);
    assembler_frontend_destroy(first);
    BOARD_CHECK(retained && frontend_panel_count() == 0 && !assembler_frontend_create(NULL));
    return 0;
}

static int frontend_controls(void) {
    frontend_commands_reset();
    frontend_panels_reset();
    BOARD_CHECK(nes_execution_set_policy(NES_EXECUTION_LIVE));
    BoardImage image = {0};
    BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0x2000, true));
    image.data[10] = 7;
    uint8_t *prg = image.data + sizeof(iNESHeader);
    prg[0] = 0x4C;
    prg[1] = 0;
    prg[2] = 0x80;
    for (unsigned vector = 0x7FFA; vector < 0x8000; vector += 2) {
        prg[vector] = 0;
        prg[vector + 1] = 0x80;
    }
    cpu_use_default_startup_alignment();
    cpu_set_test_mode(false);
    memset(&pad1, 0, sizeof(pad1));
    memset(&pad2, 0, sizeof(pad2));
    cpu_total_cycles = 0;
    apu_power_on(&apu);
    int loaded = board_image_load(&image);
    board_image_free(&image);
    BOARD_CHECK(loaded == 0);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    BOARD_CHECK(cpu_power_on(&cpu));
    vs_power_on_secondary();
    ppu_begin_frame_render(framebuffer);
    FrontendExecutionRuntime execution;
    frontend_execution_init(&execution, NULL, 44100, NULL, NULL, NULL, NULL);
    debugger_pause();
    frontend_execution_sync_debugger(&execution);
    DebugFrontend *debug = debug_frontend_create(&execution);
    BOARD_CHECK(debug && debug_frontend_register_ui(debug));
    frontend_panel_set_session_active(true);
    int failures = preview_and_apply(&execution, debug);
    failures += protected_targets();
    failures += timeline_and_policy(&execution);
    debug_frontend_destroy(debug);
    failures += registration_rollback(&execution);
    frontend_execution_shutdown(&execution);
    debugger_shutdown();
    (void)unload_rom();
    frontend_panels_reset();
    frontend_commands_reset();
    return failures;
}

int test_assembler_accuracy(void) {
    int failures = official_encodings();
    failures += operand_boundaries();
    failures += rejected_syntax();
    failures += frontend_controls();
    printf("6502 assembler: %s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures;
}
