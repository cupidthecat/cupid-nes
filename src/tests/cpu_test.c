#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"

// External memory areas (simulate PRG-ROM and RAM)
extern uint8_t prg_rom[0x8000];
extern uint8_t ram[0x0800];

// Reset test helper: clears memory and resets CPU to a known state.
void reset_test(CPU *cpu) {
    for (int i = 0; i < 0x0800; i++)
        ram[i] = 0;
    for (int i = 0; i < 0x8000; i++)
        prg_rom[i] = 0;
    cpu_reset(cpu);
    cpu->pc = 0x8000; // For testing, force PC to 0x8000.
}

//-----------------------
// LDA Instruction Tests
//-----------------------

// Immediate
void test_LDA_immediate() {
    CPU cpu;
    reset_test(&cpu);
    prg_rom[0] = 0xA9; // LDA Immediate
    prg_rom[1] = 0x55; // Value: 0x55
    prg_rom[2] = 0x00; // BRK
    cpu_step(&cpu);
    assert(cpu.a == 0x55);
    printf("LDA Immediate: PASS\n");
}

// Zero Page
void test_LDA_zero_page() {
    CPU cpu;
    reset_test(&cpu);
    ram[0x0010] = 0x77; // Store 0x77 in zero page address 0x10
    prg_rom[0] = 0xA5;  // LDA Zero Page
    prg_rom[1] = 0x10;
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    assert(cpu.a == 0x77);
    printf("LDA Zero Page: PASS\n");
}

// Zero Page, X
void test_LDA_zero_page_x() {
    CPU cpu;
    reset_test(&cpu);
    cpu.x = 0x03;
    ram[0x0015] = 0x88; // 0x12 + 0x03 = 0x15 should contain 0x88
    prg_rom[0] = 0xB5;  // LDA Zero Page,X
    prg_rom[1] = 0x12;
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    assert(cpu.a == 0x88);
    printf("LDA Zero Page,X: PASS\n");
}

// Absolute
void test_LDA_absolute() {
    CPU cpu;
    reset_test(&cpu);
    // Write expected value 0xAA into PRG-ROM at effective address 0x9000.
    prg_rom[0x9000 - 0x8000] = 0xAA;
    prg_rom[0] = 0xAD; // LDA Absolute opcode
    prg_rom[1] = 0x00; // Low byte of 0x9000
    prg_rom[2] = 0x90; // High byte of 0x9000
    prg_rom[3] = 0x00; // BRK
    cpu_step(&cpu);
    assert(cpu.a == 0xAA);
    printf("LDA Absolute: PASS\n");
}

// Absolute, X
void test_LDA_absolute_x() {
    CPU cpu;
    reset_test(&cpu);
    cpu.x = 0x05;
    // Base address 0x9000, effective = 0x9000 + 0x05 = 0x9005.
    prg_rom[0x9005 - 0x8000] = 0xBB;
    prg_rom[0] = 0xBD; // LDA Absolute,X opcode
    prg_rom[1] = 0x00; // Low byte of 0x9000
    prg_rom[2] = 0x90; // High byte of 0x9000
    prg_rom[3] = 0x00; // BRK
    cpu_step(&cpu);
    assert(cpu.a == 0xBB);
    printf("LDA Absolute,X: PASS\n");
}

// Absolute, Y
void test_LDA_absolute_y() {
    CPU cpu;
    reset_test(&cpu);
    cpu.y = 0x05;
    // Base address 0x9000, effective = 0x9000 + 0x05 = 0x9005.
    prg_rom[0x9005 - 0x8000] = 0xCC;
    prg_rom[0] = 0xB9; // LDA Absolute,Y opcode
    prg_rom[1] = 0x00; // Low byte of 0x9000
    prg_rom[2] = 0x90; // High byte of 0x9000
    prg_rom[3] = 0x00; // BRK
    cpu_step(&cpu);
    assert(cpu.a == 0xCC);
    printf("LDA Absolute,Y: PASS\n");
}

// (Indirect,X)
void test_LDA_indirect_x() {
    CPU cpu;
    reset_test(&cpu);
    cpu.x = 0x04;
    // Setup pointer: ($20 + X) = $20 + 0x04 = 0x24.
    // Store pointer in zero page so that it yields 0x0010.
    ram[0x0024] = 0x10;   // Low byte of target address: 0x10
    ram[0x0025] = 0x00;   // High byte => target address = 0x0010
    // Place expected value at RAM address 0x0010.
    ram[0x0010] = 0xDD;
    prg_rom[0] = 0xA1; // LDA (Indirect,X)
    prg_rom[1] = 0x20;
    prg_rom[2] = 0x00; // BRK
    cpu_step(&cpu);
    assert(cpu.a == 0xDD);
    printf("LDA (Indirect,X): PASS\n");
}

// (Indirect),Y
void test_LDA_indirect_y() {
    CPU cpu;
    reset_test(&cpu);
    cpu.y = 0x04;
    // Setup pointer in zero page: at $20.
    ram[0x0020] = 0x10; // Low byte => base address = 0x0010
    ram[0x0021] = 0x00; // High byte
    // With Y = 0x04, effective address = 0x0010 + 0x04 = 0x0014.
    ram[0x0014] = 0xEE; // Expected value at effective address.
    prg_rom[0] = 0xB1; // LDA (Indirect),Y
    prg_rom[1] = 0x20;
    prg_rom[2] = 0x00; // BRK
    cpu_step(&cpu);
    assert(cpu.a == 0xEE);
    printf("LDA (Indirect),Y: PASS\n");
}

//-----------------------
// STA Instruction Tests
//-----------------------

// STA Zero Page
void test_STA_zero_page() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x77;
    prg_rom[0] = 0x85; // STA Zero Page opcode
    prg_rom[1] = 0x10;
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    assert(ram[0x10] == 0x77);
    printf("STA Zero Page: PASS\n");
}

// STA Zero Page, X
void test_STA_zero_page_x() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x88;
    cpu.x = 0x03;
    prg_rom[0] = 0x95; // STA Zero Page,X opcode
    prg_rom[1] = 0x10; // 0x10 + X = 0x13
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    assert(ram[0x13] == 0x88);
    printf("STA Zero Page,X: PASS\n");
}

// STA Absolute
void test_STA_absolute() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x99;
    prg_rom[0] = 0x8D; // STA Absolute opcode
    prg_rom[1] = 0x00; // Low byte of 0x9000
    prg_rom[2] = 0x90; // High byte of 0x9000
    prg_rom[3] = 0x00; // BRK
    cpu_step(&cpu);
    assert(prg_rom[0x9000 - 0x8000] == 0x99);
    printf("STA Absolute: PASS\n");
}

// STA Absolute,X
void test_STA_absolute_x() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0xAA;
    cpu.x = 0x05;
    prg_rom[0] = 0x9D; // STA Absolute,X opcode
    prg_rom[1] = 0x00; // Low byte of 0x9000
    prg_rom[2] = 0x90; // High byte of 0x9000
    prg_rom[3] = 0x00; // BRK
    cpu_step(&cpu);
    // Effective address = 0x9000 + 0x05 = 0x9005.
    assert(prg_rom[0x9005 - 0x8000] == 0xAA);
    printf("STA Absolute,X: PASS\n");
}

// STA Absolute,Y
void test_STA_absolute_y() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0xBB;
    cpu.y = 0x05;
    prg_rom[0] = 0x99; // STA Absolute,Y opcode
    prg_rom[1] = 0x00; // Low byte of 0x9000
    prg_rom[2] = 0x90; // High byte of 0x9000
    prg_rom[3] = 0x00; // BRK
    cpu_step(&cpu);
    // Effective address = 0x9000 + 0x05 = 0x9005.
    assert(prg_rom[0x9005 - 0x8000] == 0xBB);
    printf("STA Absolute,Y: PASS\n");
}

//-----------------------
// ADC & SBC Tests
//-----------------------

// ADC Immediate
void test_ADC_immediate() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x10;
    prg_rom[0] = 0x69; // ADC Immediate opcode
    prg_rom[1] = 0x05;
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    assert(cpu.a == 0x15);
    printf("ADC Immediate: PASS\n");
}

// ADC (Indirect,X) addressing mode test
void test_ADC_indirect_x() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x20;
    cpu.x = 0x04;
    // Setup pointer: ($20 + X) = $20 + 0x04 = 0x24.
    // Store pointer in zero page so that it yields 0x0010 (not 0x8010).
    ram[0x0024] = 0x10;   // Low byte of target address: 0x10
    ram[0x0025] = 0x00;   // High byte = 0x00 -> effective pointer = 0x0010.
    // Place expected value at RAM address 0x0010.
    ram[0x0010] = 0x03;
    prg_rom[0] = 0x61; // ADC (Indirect,X) opcode
    prg_rom[1] = 0x20; // Operand: zero page address 0x20
    prg_rom[2] = 0x00; // BRK
    cpu_step(&cpu);
    // ADC: 0x20 + 0x03 = 0x23.
    assert(cpu.a == 0x23);
    printf("ADC (Indirect,X): PASS\n");
}

// ADC (Indirect),Y addressing mode test
void test_ADC_indirect_y() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x20;
    cpu.y = 0x04;
    // Setup pointer in zero page: at $20.
    ram[0x0020] = 0x10; // Low byte = 0x10
    ram[0x0021] = 0x00; // High byte = 0x00, so pointer = 0x0010.
    // With Y = 0x04, effective address = 0x0010 + 0x04 = 0x0014.
    ram[0x0014] = 0x05; // Expected value: 5.
    prg_rom[0] = 0x71; // ADC (Indirect),Y opcode
    prg_rom[1] = 0x20; // Operand: zero page address 0x20
    prg_rom[2] = 0x00; // BRK
    cpu_step(&cpu);
    // ADC: 0x20 + 0x05 = 0x25.
    assert(cpu.a == 0x25);
    printf("ADC (Indirect),Y: PASS\n");
}

// SBC Immediate
void test_SBC_immediate() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x10;
    cpu.status |= CARRY_FLAG; // Set carry flag for proper subtraction
    prg_rom[0] = 0xE9; // SBC Immediate opcode
    prg_rom[1] = 0x05;
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    assert(cpu.a == 0x0B); // 0x10 - 0x05 = 0x0B
    printf("SBC Immediate: PASS\n");
}

// SBC (Indirect,X) addressing mode test
void test_SBC_indirect_x() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x50;
    cpu.x = 0x04;
    // Setup pointer: ($20 + X) = $20 + 0x04 = 0x24.
    ram[0x0024] = 0x10;
    ram[0x0025] = 0x00; // Set high byte to 0x00 for RAM address.
    ram[0x0010] = 0x05; // Expected value at effective address.
    cpu.status |= CARRY_FLAG; // Set carry so no borrow.
    prg_rom[0] = 0xE1; // SBC (Indirect,X) opcode
    prg_rom[1] = 0x20;
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    // SBC: 0x50 - 0x05 = 0x4B.
    assert(cpu.a == 0x4B);
    printf("SBC (Indirect,X): PASS\n");
}

// SBC (Indirect),Y addressing mode test
void test_SBC_indirect_y() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x50;
    cpu.y = 0x04;
    ram[0x0020] = 0x10;
    ram[0x0021] = 0x00; // Set high byte to 0x00 for pointer.
    // With Y = 0x04, effective address = 0x0010 + 0x04 = 0x0014.
    ram[0x0014] = 0x06; // Expected value: 6.
    cpu.status |= CARRY_FLAG; // Set carry.
    prg_rom[0] = 0xF1; // SBC (Indirect),Y opcode
    prg_rom[1] = 0x20;
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    // SBC: 0x50 - 0x06 = 0x4A.
    assert(cpu.a == 0x4A);
    printf("SBC (Indirect),Y: PASS\n");
}

//-----------------------
// Logical Operations Tests
//-----------------------

// AND Immediate
void test_AND_immediate() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0xF0;
    prg_rom[0] = 0x29; // AND Immediate opcode
    prg_rom[1] = 0x0F; // 0xF0 & 0x0F = 0x00
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    assert(cpu.a == 0x00);
    printf("AND Immediate: PASS\n");
}

// ORA Immediate
void test_ORA_immediate() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x0F;
    prg_rom[0] = 0x09; // ORA Immediate opcode
    prg_rom[1] = 0xF0; // 0x0F | 0xF0 = 0xFF
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    assert(cpu.a == 0xFF);
    printf("ORA Immediate: PASS\n");
}

// EOR Immediate
void test_EOR_immediate() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0xAA;
    prg_rom[0] = 0x49; // EOR Immediate opcode
    prg_rom[1] = 0xFF; // 0xAA ^ 0xFF = 0x55
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    assert(cpu.a == 0x55);
    printf("EOR Immediate: PASS\n");
}

//-----------------------
// Compare Instructions
//-----------------------

// CMP Immediate
void test_CMP_immediate() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x50;
    prg_rom[0] = 0xC9; // CMP Immediate opcode
    prg_rom[1] = 0x50; // equal
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    assert(cpu.status & ZERO_FLAG);
    assert(cpu.status & CARRY_FLAG);
    printf("CMP Immediate: PASS\n");
}

// CPX Immediate
void test_CPX_immediate() {
    CPU cpu;
    reset_test(&cpu);
    cpu.x = 0x20;
    prg_rom[0] = 0xE0; // CPX Immediate opcode
    prg_rom[1] = 0x20; // equal
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    assert(cpu.status & ZERO_FLAG);
    printf("CPX Immediate: PASS\n");
}

// CPY Immediate
void test_CPY_immediate() {
    CPU cpu;
    reset_test(&cpu);
    cpu.y = 0x30;
    prg_rom[0] = 0xC0; // CPY Immediate opcode
    prg_rom[1] = 0x30; // equal
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    assert(cpu.status & ZERO_FLAG);
    printf("CPY Immediate: PASS\n");
}

//-----------------------
// Transfer & Increment/Decrement
//-----------------------

// TAX
void test_TAX() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0xAA;
    prg_rom[0] = 0xAA; // TAX opcode
    prg_rom[1] = 0x00;
    cpu_step(&cpu);
    assert(cpu.x == 0xAA);
    printf("TAX: PASS\n");
}

// TXA
void test_TXA() {
    CPU cpu;
    reset_test(&cpu);
    cpu.x = 0x55;
    prg_rom[0] = 0x8A; // TXA opcode
    prg_rom[1] = 0x00;
    cpu_step(&cpu);
    assert(cpu.a == 0x55);
    printf("TXA: PASS\n");
}

// DEX
void test_DEX() {
    CPU cpu;
    reset_test(&cpu);
    cpu.x = 0x10;
    prg_rom[0] = 0xCA; // DEX opcode
    prg_rom[1] = 0x00;
    cpu_step(&cpu);
    assert(cpu.x == 0x0F);
    printf("DEX: PASS\n");
}

// INX
void test_INX() {
    CPU cpu;
    reset_test(&cpu);
    cpu.x = 0x10;
    prg_rom[0] = 0xE8; // INX opcode
    prg_rom[1] = 0x00;
    cpu_step(&cpu);
    assert(cpu.x == 0x11);
    printf("INX: PASS\n");
}

// LDY Immediate
void test_LDY_immediate() {
    CPU cpu;
    reset_test(&cpu);
    prg_rom[0] = 0xA0; // LDY Immediate opcode
    prg_rom[1] = 0x22;
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    assert(cpu.y == 0x22);
    printf("LDY Immediate: PASS\n");
}

// LDX Immediate
void test_LDX_immediate() {
    CPU cpu;
    reset_test(&cpu);
    prg_rom[0] = 0xA2; // LDX Immediate opcode
    prg_rom[1] = 0x33;
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    assert(cpu.x == 0x33);
    printf("LDX Immediate: PASS\n");
}

//-----------------------
// Jump & Subroutine
//-----------------------

// JMP Absolute
void test_JMP_absolute() {
    CPU cpu;
    reset_test(&cpu);
    prg_rom[0] = 0x4C; // JMP Absolute opcode
    prg_rom[1] = 0x00; // Low byte of 0x9000
    prg_rom[2] = 0x90; // High byte of 0x9000
    cpu_step(&cpu);
    assert(cpu.pc == 0x9000);
    printf("JMP Absolute: PASS\n");
}

// JMP Indirect
void test_JMP_indirect() {
    CPU cpu;
    reset_test(&cpu);
    // Setup pointer in RAM: at $0200 -> $1234 (this test is fine as-is because it uses RAM)
    ram[0x0200] = 0x34;
    ram[0x0201] = 0x12;
    prg_rom[0] = 0x6C; // JMP Indirect opcode
    prg_rom[1] = 0x00;
    prg_rom[2] = 0x02;
    cpu_step(&cpu);
    assert(cpu.pc == 0x1234);
    printf("JMP Indirect: PASS\n");
}

// JSR and RTS test
void test_JSR_RTS() {
    CPU cpu;
    reset_test(&cpu);
    prg_rom[0] = 0x20; // JSR $8005
    prg_rom[1] = 0x05;
    prg_rom[2] = 0x80;
    prg_rom[3] = 0x00; // BRK (main program end)
    // Subroutine at $8005: LDA #$77, RTS
    prg_rom[0x0005] = 0xA9; // LDA Immediate
    prg_rom[0x0006] = 0x77; // Value 0x77
    prg_rom[0x0007] = 0x60; // RTS
    cpu_step(&cpu); // Execute JSR: jumps to subroutine at 0x8005.
    cpu_step(&cpu); // Execute LDA #$77: loads 0x77 into accumulator.
    cpu_step(&cpu); // Execute RTS: returns to main program.
    assert(cpu.a == 0x77);
    printf("JSR/RTS: PASS\n");
}

// BRK
void test_BRK() {
    CPU cpu;
    reset_test(&cpu);
    prg_rom[0] = 0x00; // BRK opcode
    cpu_step(&cpu);
    assert(cpu.status & BREAK_FLAG);
    printf("BRK: PASS\n");
}

//-----------------------
// Stack Operations
//-----------------------

// PHP and PLP
void test_PHP_PLP() {
    CPU cpu;
    reset_test(&cpu);
    cpu.status = 0xAA;
    prg_rom[0] = 0x08; // PHP opcode
    prg_rom[1] = 0x28; // PLP opcode
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    cpu_step(&cpu);
    assert(cpu.status == 0xAA);
    printf("PHP/PLP: PASS\n");
}

// PHA and PLA
void test_PHA_PLA() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x55;
    prg_rom[0] = 0x48; // PHA opcode
    prg_rom[1] = 0xA9; // LDA Immediate opcode
    prg_rom[2] = 0x00; // LDA #$00 (clear A)
    prg_rom[3] = 0x68; // PLA opcode (restore A)
    prg_rom[4] = 0x00;
    cpu_step(&cpu);
    cpu_step(&cpu);
    cpu_step(&cpu);
    cpu_step(&cpu);
    assert(cpu.a == 0x55);
    printf("PHA/PLA: PASS\n");
}

//-----------------------
// BIT Instruction Tests
//-----------------------

// BIT Zero Page
void test_BIT_zero_page() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0xFF;
    ram[0x0010] = 0x80; // Test value with negative bit set
    prg_rom[0] = 0x24; // BIT Zero Page opcode
    prg_rom[1] = 0x10;
    cpu_step(&cpu);
    // Should set Negative flag and clear Zero flag.
    assert((cpu.status & (ZERO_FLAG|NEGATIVE_FLAG)) == NEGATIVE_FLAG);
    printf("BIT Zero Page: PASS\n");
}

// BIT Absolute
void test_BIT_absolute() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x0F;
    prg_rom[0x9000 - 0x8000] = 0xF0;
    prg_rom[0] = 0x2C; // BIT Absolute opcode
    prg_rom[1] = 0x00; // Low byte of 0x9000
    prg_rom[2] = 0x90; // High byte of 0x9000
    prg_rom[3] = 0x00; // BRK
    cpu_step(&cpu);
    // 0x0F & 0xF0 == 0, so ZERO flag should be set.
    assert(cpu.status & ZERO_FLAG);
    printf("BIT Absolute: PASS\n");
}

//-----------------------
// Branch Instructions Tests
//-----------------------

// Test BEQ and BNE using relative addressing.
void test_branch_instructions() {
    CPU cpu;
    reset_test(&cpu);
    // Program: LDA #$00, BEQ +0, LDA #$FF, BRK
    prg_rom[0] = 0xA9; // LDA Immediate
    prg_rom[1] = 0x00; // Value 0x00 (sets ZERO flag)
    prg_rom[2] = 0xF0; // BEQ opcode
    prg_rom[3] = 0x00; // Relative offset 0: branch target = PC (no offset)
    prg_rom[4] = 0xA9; // LDA Immediate
    prg_rom[5] = 0xFF; // Value 0xFF
    prg_rom[6] = 0x00; // BRK
    cpu_step(&cpu); // Execute LDA #$00; PC becomes 0x8002.
    cpu_step(&cpu); // Execute BEQ; branch: PC becomes 0x8004.
    cpu_step(&cpu); // Execute LDA #$FF; PC becomes 0x8006.
    assert(cpu.a == 0xFF);
    printf("Branch BEQ: PASS\n");

    reset_test(&cpu);
    // Program: LDA #$01, BNE +0, LDA #$FF, BRK
    prg_rom[0] = 0xA9; // LDA Immediate
    prg_rom[1] = 0x01; // Value 0x01 (non-zero, ZERO flag clear)
    prg_rom[2] = 0xD0; // BNE opcode
    prg_rom[3] = 0x00; // Relative offset 0: branch target = PC (no offset)
    prg_rom[4] = 0xA9; // LDA Immediate
    prg_rom[5] = 0xFF; // Value 0xFF
    prg_rom[6] = 0x00; // BRK
    cpu_step(&cpu); // Execute LDA #$01; PC becomes 0x8002.
    cpu_step(&cpu); // Execute BNE; branch: PC becomes 0x8004.
    cpu_step(&cpu); // Execute LDA #$FF; PC becomes 0x8006.
    assert(cpu.a == 0xFF);
    printf("Branch BNE: PASS\n");
}

//-----------------------
// Shift/Rotate Instructions Tests
//-----------------------

// ASL Accumulator
void test_ASL_accumulator() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x40; // 0x40 << 1 = 0x80, carry clear.
    prg_rom[0] = 0x0A; // ASL Accumulator opcode
    prg_rom[1] = 0x00;
    cpu_step(&cpu);
    assert(cpu.a == 0x80);
    assert(!(cpu.status & CARRY_FLAG));
    printf("ASL Accumulator: PASS\n");
}

// ASL Zero Page
void test_ASL_zero_page() {
    CPU cpu;
    reset_test(&cpu);
    ram[0x0010] = 0x80; // 0x80 << 1 = 0x00 with carry set.
    prg_rom[0] = 0x06; // ASL Zero Page opcode
    prg_rom[1] = 0x10;
    prg_rom[2] = 0x00;
    cpu_step(&cpu);
    assert(ram[0x0010] == 0x00);
    assert(cpu.status & CARRY_FLAG);
    printf("ASL Zero Page: PASS\n");
}

// LSR Accumulator
void test_LSR_accumulator() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x03; // 0x03 >> 1 = 0x01, carry set because bit0 was 1.
    prg_rom[0] = 0x4A; // LSR Accumulator opcode
    prg_rom[1] = 0x00;
    cpu_step(&cpu);
    assert(cpu.a == 0x01);
    assert(cpu.status & CARRY_FLAG);
    printf("LSR Accumulator: PASS\n");
}

// Register transfer tests
void test_TAY_TYA() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0xAA;
    prg_rom[0] = 0xA8; // TAY
    prg_rom[1] = 0x98; // TYA
    cpu_step(&cpu);
    assert(cpu.y == 0xAA);
    cpu_step(&cpu);
    assert(cpu.a == 0xAA);
    printf("TAY/TYA: PASS\n");
}

void test_TSX_TXS() {
    CPU cpu;
    reset_test(&cpu);
    cpu.sp = 0xFD;
    prg_rom[0] = 0xBA; // TSX
    cpu_step(&cpu);
    assert(cpu.x == 0xFD);
    
    cpu.x = 0xEE;
    prg_rom[1] = 0x9A; // TXS
    cpu_step(&cpu);
    assert(cpu.sp == 0xEE);
    printf("TSX/TXS: PASS\n");
}

// INC/DEC tests
void test_INC_absolute() {
    CPU cpu;
    reset_test(&cpu);
    ram[0x0234] = 0x7F;
    prg_rom[0] = 0xEE; // INC Absolute
    prg_rom[1] = 0x34;
    prg_rom[2] = 0x02;
    cpu_step(&cpu);
    assert(ram[0x0234] == 0x80);
    assert(cpu.status & NEGATIVE_FLAG);
    printf("INC Absolute: PASS\n");
}

void test_DEC_zero_page_x() {
    CPU cpu;
    reset_test(&cpu);
    cpu.x = 0x05;
    ram[0x15] = 0x01;
    prg_rom[0] = 0xD6; // DEC Zero Page,X
    prg_rom[1] = 0x10;
    cpu_step(&cpu);
    assert(ram[0x15] == 0x00);
    assert(cpu.status & ZERO_FLAG);
    printf("DEC Zero Page,X: PASS\n");
}

// Rotate tests
void test_ROL_accumulator() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x40;
    // Set the initial carry flag; it will be shifted into bit0.
    cpu.status |= CARRY_FLAG;
    prg_rom[0] = 0x2A; // Opcode for ROL A
    cpu_step(&cpu);
    // After ROL:
    // - The accumulator should be (0x40 << 1) | 1 = 0x80 | 1 = 0x81.
    // - The new carry should be the old bit7 of 0x40, which is 0.
    assert(cpu.a == 0x81);
    assert((cpu.status & CARRY_FLAG) == 0); // Expect the carry flag to be clear.
    printf("ROL Accumulator: PASS\n");
}

// ROR Accumulator
void test_ROR_accumulator() {
    CPU cpu;
    reset_test(&cpu);
    cpu.a = 0x03;          // 00000011 (LSB = 1)
    cpu.status |= CARRY_FLAG; // Pre-set carry to shift into bit7
    prg_rom[0] = 0x6A;     // ROR Accumulator opcode
    cpu_step(&cpu);
    // Expected: (0x03 >> 1) = 0x01 then (carry<<7)=0x80, so 0x01|0x80 = 0x81.
    // New carry becomes old bit0 (1).
    assert(cpu.a == 0x81);
    assert(cpu.status & CARRY_FLAG); // New carry should be set.
    assert(cpu.status & NEGATIVE_FLAG); // 0x81 has bit7 set.
    printf("ROR Accumulator: PASS\n");
}

// ROR Zero Page
void test_ROR_zero_page() {
    CPU cpu;
    reset_test(&cpu);
    ram[0x20] = 0x01;      // 00000001 (LSB = 1)
    cpu.status &= ~CARRY_FLAG; // Ensure carry is clear.
    prg_rom[0] = 0x66;     // ROR Zero Page opcode
    prg_rom[1] = 0x20;
    cpu_step(&cpu);
    // Expected: 0x01 >> 1 = 0x00 then (carry<<7)=0x00, result 0x00.
    // New carry becomes 1 because original LSB was 1.
    assert(ram[0x20] == 0x00);
    assert(cpu.status & CARRY_FLAG);
    printf("ROR Zero Page: PASS\n");
}

// ROR Zero Page, X
void test_ROR_zero_page_x() {
    CPU cpu;
    reset_test(&cpu);
    cpu.x = 0x05;
    ram[0x15] = 0x81;      // 10000001 (LSB = 1)
    prg_rom[0] = 0x76;     // ROR Zero Page,X opcode
    prg_rom[1] = 0x10;     // Effective address: 0x10 + 0x05 = 0x15
    cpu_step(&cpu);
    // Expected: 0x81 >> 1 = 0x40, with new carry from bit0 = 1.
    assert(ram[0x15] == 0x40);
    assert(cpu.status & CARRY_FLAG);
    printf("ROR Zero Page,X: PASS\n");
}

// ROR Absolute
void test_ROR_absolute() {
    CPU cpu;
    reset_test(&cpu);
    // Set initial value to 0x8D (10001101) so that bit0 is 1.
    prg_rom[0x9000 - 0x8000] = 0x8D;
    prg_rom[0] = 0x6E;     // ROR Absolute opcode
    prg_rom[1] = 0x00;
    prg_rom[2] = 0x90;
    cpu.status |= CARRY_FLAG;
    cpu_step(&cpu);
    // Calculation: 0x8D >> 1 = 0x46; (carry<<7)=0x80; 0x46 | 0x80 = 0xC6.
    // New carry becomes 1 (old LSB of 0x8D).
    assert(prg_rom[0x9000 - 0x8000] == 0xC6);
    assert(cpu.status & NEGATIVE_FLAG);
    assert(cpu.status & CARRY_FLAG);
    printf("ROR Absolute: PASS\n");
}

// ROR Absolute, X remains unchanged
void test_ROR_absolute_x() {
    CPU cpu;
    reset_test(&cpu);
    cpu.x = 0x05;
    prg_rom[0x9005 - 0x8000] = 0x00; // 00000000 (LSB = 0)
    prg_rom[0] = 0x7E;     // ROR Absolute,X opcode
    prg_rom[1] = 0x00;
    prg_rom[2] = 0x90;
    cpu.status |= CARRY_FLAG; // Pre-set carry; will shift in 1.
    cpu_step(&cpu);
    // Expected: 0x00 >> 1 = 0x00, then (carry<<7)=0x80, so 0x80.
    // New carry becomes 0 because original bit0 was 0.
    assert(prg_rom[0x9005 - 0x8000] == 0x80);
    assert(cpu.status & NEGATIVE_FLAG);
    assert(!(cpu.status & CARRY_FLAG));
    printf("ROR Absolute,X: PASS\n");
}

// RTI (Return from Interrupt)
// This test simulates an interrupt sequence by manually placing values on the stack.
// We set up the stack so that after RTI, the processor status should be 0xAA and
// the PC should be 0x1234.
void test_RTI() {
    CPU cpu;
    reset_test(&cpu);
    // Set the stack pointer to 0xFC so that the next three pushes will be at:
    // 0x0100 + 0xFD, 0x0100 + 0xFE, and 0x0100 + 0xFF.
    cpu.sp = 0xFC;
    // Place the status value (0xAA) at stack location 0x0100 + 0xFD.
    ram[0x0100 + 0xFD - 0x0000] = 0xAA;
    // Place the return address low and high bytes for 0x1234.
    ram[0x0100 + 0xFE - 0x0000] = 0x34; // low byte
    ram[0x0100 + 0xFF - 0x0000] = 0x12; // high byte
    // Insert RTI opcode at the current PC.
    prg_rom[0] = 0x40; // RTI opcode
    cpu_step(&cpu);
    // After RTI, the CPU should have pulled status and PC.
    assert(cpu.status == 0xAA);
    assert(cpu.pc == 0x1234);
    printf("RTI: PASS\n");
}

// STX tests

// STX Zero Page (opcode 0x86)
void test_STX_zero_page() {
    CPU cpu;
    reset_test(&cpu);
    cpu.x = 0x55;
    prg_rom[0] = 0x86; // STX Zero Page opcode
    prg_rom[1] = 0x10; // address 0x10
    prg_rom[2] = 0x00; // BRK
    cpu_step(&cpu);
    assert(ram[0x10] == 0x55);
    printf("STX Zero Page: PASS\n");
}

// STX Zero Page, Y (opcode 0x96)
void test_STX_zero_page_y() {
    CPU cpu;
    reset_test(&cpu);
    cpu.x = 0x66;
    cpu.y = 0x04;
    // Effective address = (operand + Y) & 0xFF
    prg_rom[0] = 0x96; // STX Zero Page,Y opcode
    prg_rom[1] = 0x10; // 0x10 + 0x04 = 0x14
    prg_rom[2] = 0x00; // BRK
    cpu_step(&cpu);
    assert(ram[0x14] == 0x66);
    printf("STX Zero Page,Y: PASS\n");
}

// STX Absolute (opcode 0x8E)
void test_STX_absolute() {
    CPU cpu;
    reset_test(&cpu);
    cpu.x = 0x77;
    prg_rom[0] = 0x8E; // STX Absolute opcode
    prg_rom[1] = 0x00; // low byte of address 0x9000
    prg_rom[2] = 0x90; // high byte of address 0x9000
    prg_rom[3] = 0x00; // BRK
    cpu_step(&cpu);
    assert(prg_rom[0x9000 - 0x8000] == 0x77);
    printf("STX Absolute: PASS\n");
}

// STY tests

// STY Zero Page (opcode 0x84)
void test_STY_zero_page() {
    CPU cpu;
    reset_test(&cpu);
    cpu.y = 0x88;
    prg_rom[0] = 0x84; // STY Zero Page opcode
    prg_rom[1] = 0x20; // address 0x20
    prg_rom[2] = 0x00; // BRK
    cpu_step(&cpu);
    assert(ram[0x20] == 0x88);
    printf("STY Zero Page: PASS\n");
}

// STY Zero Page, X (opcode 0x94)
void test_STY_zero_page_x() {
    CPU cpu;
    reset_test(&cpu);
    cpu.y = 0x99;
    cpu.x = 0x03;
    // Effective address = (operand + X) & 0xFF, 0x20 + 0x03 = 0x23.
    prg_rom[0] = 0x94; // STY Zero Page,X opcode
    prg_rom[1] = 0x20;
    prg_rom[2] = 0x00; // BRK
    cpu_step(&cpu);
    assert(ram[0x23] == 0x99);
    printf("STY Zero Page,X: PASS\n");
}

// STY Absolute (opcode 0x8C)
void test_STY_absolute() {
    CPU cpu;
    reset_test(&cpu);
    cpu.y = 0xAA;
    prg_rom[0] = 0x8C; // STY Absolute opcode
    prg_rom[1] = 0x00; // low byte of address 0x9000
    prg_rom[2] = 0x90; // high byte of address 0x9000
    prg_rom[3] = 0x00; // BRK
    cpu_step(&cpu);
    assert(prg_rom[0x9000 - 0x8000] == 0xAA);
    printf("STY Absolute: PASS\n");
}

// NOP (No Operation)
// This test ensures that executing NOP (0xEA) simply advances the PC without changing registers.
void test_NOP() {
    CPU cpu;
    reset_test(&cpu);
    uint16_t initial_pc = cpu.pc;
    // Set a known value in a register to verify it remains unchanged.
    cpu.a = 0x55;
    prg_rom[0] = 0xEA; // NOP opcode
    prg_rom[1] = 0x00; // BRK
    cpu_step(&cpu);
    // PC should have advanced by one.
    assert(cpu.pc == initial_pc + 1);
    // A register remains unchanged.
    assert(cpu.a == 0x55);
    printf("NOP: PASS\n");
}

// Flag Modifier Tests

// CLC (Clear Carry Flag, 0x18)
// Precondition: The carry flag is set. After executing CLC the carry flag should be cleared.
void test_CLC() {
    CPU cpu;
    reset_test(&cpu);
    cpu.status |= CARRY_FLAG;  // Ensure carry is set.
    prg_rom[0] = 0x18;         // CLC opcode
    prg_rom[1] = 0x00;         // BRK (end test)
    cpu_step(&cpu);
    assert(!(cpu.status & CARRY_FLAG));
    printf("CLC: PASS\n");
}

// SEC (Set Carry Flag, 0x38)
// Precondition: The carry flag is clear. After executing SEC the carry flag should be set.
void test_SEC() {
    CPU cpu;
    reset_test(&cpu);
    cpu.status &= ~CARRY_FLAG; // Ensure carry is clear.
    prg_rom[0] = 0x38;         // SEC opcode
    prg_rom[1] = 0x00;         // BRK
    cpu_step(&cpu);
    assert(cpu.status & CARRY_FLAG);
    printf("SEC: PASS\n");
}

// CLI (Clear Interrupt Disable, 0x58)
// Precondition: The interrupt flag is set. After executing CLI the interrupt flag should be cleared.
void test_CLI() {
    CPU cpu;
    reset_test(&cpu);
    cpu.status |= INTERRUPT_FLAG;  // Ensure interrupt flag is set.
    prg_rom[0] = 0x58;             // CLI opcode
    prg_rom[1] = 0x00;             // BRK
    cpu_step(&cpu);
    assert(!(cpu.status & INTERRUPT_FLAG));
    printf("CLI: PASS\n");
}

// SEI (Set Interrupt Disable, 0x78)
// Precondition: The interrupt flag is clear. After executing SEI the interrupt flag should be set.
void test_SEI() {
    CPU cpu;
    reset_test(&cpu);
    cpu.status &= ~INTERRUPT_FLAG;  // Ensure interrupt flag is clear.
    prg_rom[0] = 0x78;             // SEI opcode
    prg_rom[1] = 0x00;             // BRK
    cpu_step(&cpu);
    assert(cpu.status & INTERRUPT_FLAG);
    printf("SEI: PASS\n");
}

// CLD (Clear Decimal Mode, 0xD8)
// Precondition: The decimal flag is set. After executing CLD the decimal flag should be cleared.
void test_CLD() {
    CPU cpu;
    reset_test(&cpu);
    cpu.status |= DECIMAL_FLAG;  // Set decimal flag.
    prg_rom[0] = 0xD8;           // CLD opcode
    prg_rom[1] = 0x00;           // BRK
    cpu_step(&cpu);
    assert(!(cpu.status & DECIMAL_FLAG));
    printf("CLD: PASS\n");
}

// SED (Set Decimal Mode, 0xF8)
// Precondition: The decimal flag is clear. After executing SED the decimal flag should be set.
void test_SED() {
    CPU cpu;
    reset_test(&cpu);
    cpu.status &= ~DECIMAL_FLAG; // Ensure decimal flag is clear.
    prg_rom[0] = 0xF8;           // SED opcode
    prg_rom[1] = 0x00;           // BRK
    cpu_step(&cpu);
    assert(cpu.status & DECIMAL_FLAG);
    printf("SED: PASS\n");
}

// CLV (Clear Overflow Flag, 0xB8)
// Precondition: The overflow flag is set. After executing CLV the overflow flag should be cleared.
void test_CLV() {
    CPU cpu;
    reset_test(&cpu);
    cpu.status |= OVERFLOW_FLAG; // Set overflow flag.
    prg_rom[0] = 0xB8;           // CLV opcode
    prg_rom[1] = 0x00;           // BRK
    cpu_step(&cpu);
    assert(!(cpu.status & OVERFLOW_FLAG));
    printf("CLV: PASS\n");
}

void run_cpu_instruction_tests() {
    printf("Running CPU Instruction Tests...\n");
    test_LDA_immediate();
    test_LDA_zero_page();
    test_LDA_zero_page_x();
    test_LDA_absolute();
    test_LDA_absolute_x();
    test_LDA_absolute_y();
    test_LDA_indirect_x();
    test_LDA_indirect_y();

    test_STA_zero_page();
    test_STA_zero_page_x();
    test_STA_absolute();
    test_STA_absolute_x();
    test_STA_absolute_y();

    test_ADC_immediate();
    test_ADC_indirect_x();
    test_ADC_indirect_y();

    test_SBC_immediate();
    test_SBC_indirect_x();
    test_SBC_indirect_y();

    test_AND_immediate();
    test_ORA_immediate();
    test_EOR_immediate();

    test_CMP_immediate();
    test_CPX_immediate();
    test_CPY_immediate();

    test_TAX();
    test_TXA();
    test_DEX();
    test_INX();
    test_LDY_immediate();
    test_LDX_immediate();

    test_JMP_absolute();
    test_JMP_indirect();
    test_JSR_RTS();
    test_BRK();

    test_PHP_PLP();
    test_PHA_PLA();

    test_BIT_zero_page();
    test_BIT_absolute();

    test_branch_instructions();

    test_ASL_accumulator();
    test_ASL_zero_page();
    test_LSR_accumulator();
    
    test_TAY_TYA();
    test_TSX_TXS();
    test_INC_absolute();
    test_DEC_zero_page_x();
    test_ROL_accumulator();
    test_ROR_accumulator();
    test_ROR_zero_page();
    test_ROR_zero_page_x();
    test_ROR_absolute();
    test_ROR_absolute_x();
    
    test_RTI();
    test_STX_zero_page();
    test_STX_zero_page_y();
    test_STX_absolute();
    test_STY_zero_page();
    test_STY_zero_page_x();
    test_STY_absolute();
    test_NOP();
    
    test_CLC();
    test_SEC();
    test_CLI();
    test_SEI();
    test_CLD();
    test_SED();
    test_CLV();

    printf("CPU Instruction Tests Completed.\n");
}

int main() {
    run_cpu_instruction_tests();
    return 0;
}
