// cpu.c
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "cpu.h"
#include "../ppu/ppu.h"

uint8_t ram[0x0800];        // 2KB internal RAM
uint8_t apu_io[0x18];       // APU + I/O registers

void cpu_reset(CPU* cpu) {
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->sp = 0xFD;  // Initialize to 0xFD (valid stack range is 0x00-0xFF)
    cpu->status = 0x24;
    // Read reset vector from PRG-ROM area
    cpu->pc = (prg_rom[0x7FFD] << 8) | prg_rom[0x7FFC];
}

uint8_t read_mem(uint16_t addr) {
    if (addr <= 0x1FFF) return ram[addr & 0x07FF];
    else if (addr >= 0x2000 && addr <= 0x3FFF) {
        uint16_t reg = 0x2000 | (addr & 7);
        return ppu_reg_read(reg);
    } else if (addr == 0x4016) {
        return joypad_read(&pad1);  // Changed from apu_io[0x16]
    } else if (addr == 0x4017) {
        return joypad_read(&pad2);  // Added joypad 2 support
    } else if (addr <= 0x401F) {
        return apu_io[addr - 0x4000];
    } else if (addr >= 0x8000) {
        return prg_rom[addr - 0x8000];
    }
    return 0;
}

void write_mem(uint16_t addr, uint8_t value) {
    if (addr <= 0x1FFF) { ram[addr & 0x07FF] = value; return; }
    if (addr >= 0x2000 && addr <= 0x3FFF) { ppu_reg_write(0x2000 | (addr & 7), value); return; }
    if (addr == 0x4014) { ppu_oam_dma(value); return; }  // OAM DMA copy
    if (addr == 0x4016) {          // STROBE
        joypad_write_strobe(&pad1, value);
        joypad_write_strobe(&pad2, value);
        return;
    }
    if (addr <= 0x401F) { apu_io[addr - 0x4000] = value; return; }
    /* writes to ROM usually ignored */
}

void cpu_nmi(CPU* cpu) {
    uint16_t pc = cpu->pc;
    write_mem(0x0100 + cpu->sp--, (pc >> 8));
    write_mem(0x0100 + cpu->sp--, (pc & 0xFF));
    write_mem(0x0100 + cpu->sp--, cpu->status & ~BREAK_FLAG);
    cpu->status |= INTERRUPT_FLAG;
    cpu->pc = read_mem(0xFFFA) | (read_mem(0xFFFB) << 8);
}

// Helper function to set zero and negative flags
static void set_zn_flags(CPU* cpu, uint8_t value) {
    cpu->status = (cpu->status & ~(ZERO_FLAG|NEGATIVE_FLAG)) |
                  (value == 0 ? ZERO_FLAG : 0) |
                  (value & 0x80 ? NEGATIVE_FLAG : 0);
}

// Addressing mode helpers
static uint16_t get_zpg_address(CPU* cpu) {
    return read_mem(cpu->pc++);
}

static uint16_t get_zpgx_address(CPU* cpu) {
    return (read_mem(cpu->pc++) + cpu->x) & 0xFF;
}

static uint16_t get_abs_address(CPU* cpu) {
    uint16_t addr = read_mem(cpu->pc);
    addr |= read_mem(cpu->pc + 1) << 8;
    cpu->pc += 2;
    return addr;
}

static uint16_t get_absx_address(CPU* cpu) {
    uint16_t base = read_mem(cpu->pc) | (read_mem(cpu->pc + 1) << 8);
    cpu->pc += 2;
    return base + cpu->x;
}

static uint16_t get_absy_address(CPU* cpu) {
    uint16_t base = read_mem(cpu->pc) | (read_mem(cpu->pc + 1) << 8);
    cpu->pc += 2;
    return base + cpu->y;
}

static void branch_if(CPU* cpu, bool condition) {
    int8_t offset = read_mem(cpu->pc++);
    if(condition) {
        cpu->pc += offset;
    }
}

// Shift/Rotate helpers
static void asl(CPU* cpu, uint8_t* value) {
    cpu->status = (cpu->status & ~CARRY_FLAG) | (*value & 0x80 ? CARRY_FLAG : 0);
    *value <<= 1;
    set_zn_flags(cpu, *value);
}

static void lsr(CPU* cpu, uint8_t* value) {
    cpu->status = (cpu->status & ~CARRY_FLAG) | (*value & 0x01 ? CARRY_FLAG : 0);
    *value >>= 1;
    set_zn_flags(cpu, *value);
}

// Helpers for rotate operations
static void rol(CPU* cpu, uint8_t* value) {
    uint8_t new_carry = (*value >> 7) & 1;
    *value = (*value << 1) | (cpu->status & CARRY_FLAG ? 1 : 0);
    cpu->status = (cpu->status & ~CARRY_FLAG) | (new_carry ? CARRY_FLAG : 0);
    set_zn_flags(cpu, *value);
}

// ADC and SBC helpers
static inline void do_adc(CPU* cpu, uint8_t o) {
    uint16_t s = cpu->a + o + ((cpu->status & CARRY_FLAG) ? 1 : 0);
    uint8_t r = (uint8_t)s;
    // C, Z, V, N
    cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG))
                | ((s > 0xFF) ? CARRY_FLAG : 0)
                | ((r == 0) ? ZERO_FLAG : 0)
                | (((cpu->a ^ r) & (o ^ r) & 0x80) ? OVERFLOW_FLAG : 0)
                | (r & 0x80 ? NEGATIVE_FLAG : 0)
                | UNUSED_FLAG;             // keep U set
    cpu->a = r;
}

static inline void do_sbc(CPU* cpu, uint8_t o) {
    uint16_t d = cpu->a - o - ((cpu->status & CARRY_FLAG) ? 0 : 1);
    uint8_t r = (uint8_t)d;
    cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG))
                | ((d < 0x100) ? CARRY_FLAG : 0)
                | ((r == 0) ? ZERO_FLAG : 0)
                | (((cpu->a ^ r) & (~o ^ r) & 0x80) ? OVERFLOW_FLAG : 0)
                | (r & 0x80 ? NEGATIVE_FLAG : 0)
                | UNUSED_FLAG;
    cpu->a = r;
}

// Helper function for ROR
static void ror(CPU* cpu, uint8_t* value) {
    // Save the old carry.
    uint8_t old_carry = (cpu->status & CARRY_FLAG) ? 1 : 0;
    // The new carry becomes the old bit0.
    uint8_t new_carry = *value & 0x01;
    // Shift the value right and insert the old carry into bit7.
    *value = (*value >> 1) | (old_carry << 7);
    // Update the status flags: clear CARRY, ZERO, and NEGATIVE,
    // then set them based on the result.
    cpu->status = (cpu->status & ~(CARRY_FLAG | ZERO_FLAG | NEGATIVE_FLAG)) |
                  (new_carry ? CARRY_FLAG : 0) |
                  ((*value == 0) ? ZERO_FLAG : 0) |
                  ((*value & 0x80) ? NEGATIVE_FLAG : 0);
}

// Indexed Indirect: (indirect,X)
static uint16_t get_indirect_x(CPU* cpu) {
    uint8_t zpg = read_mem(cpu->pc++);
    uint16_t ptr = (zpg + cpu->x) & 0xFF;
    return read_mem(ptr) | (read_mem((ptr + 1) & 0xFF) << 8);
}

// Indirect Indexed: (indirect),Y 
static uint16_t get_indirect_y(CPU* cpu) {
    uint8_t zpg = read_mem(cpu->pc++);
    uint16_t base = read_mem(zpg) | (read_mem((zpg + 1) & 0xFF) << 8);
    return base + cpu->y;
}

// Absolute Indirect (for JMP)
static uint16_t get_abs_indirect(CPU* cpu) {
    uint16_t ptr = get_abs_address(cpu);
    // Handle 6502 page boundary bug
    return read_mem(ptr) | (read_mem((ptr & 0xFF00) | ((ptr + 1) & 0xFF)) << 8);
}

static uint16_t get_zpgy_address(CPU* cpu) {
    return (read_mem(cpu->pc++) + cpu->y) & 0xFF;
}

static inline void inc_then_sbc(CPU* cpu, uint16_t addr) {
    uint8_t v = read_mem(addr) + 1;
    write_mem(addr, v);
    do_sbc(cpu, v);   // you already have do_sbc()
}

void execute(CPU* cpu, uint8_t opcode) {
    switch(opcode) {
        case 0x88: // DEY
            cpu->y--;
            // use your helper so Z/N are correct
            // (same helper you already use for TAX/INX/etc.)
            set_zn_flags(cpu, cpu->y);
            break;

        case 0xC8: // INY
            cpu->y++;
            set_zn_flags(cpu, cpu->y);
            break;

        case 0xA9: // LDA Immediate
            cpu->a = read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->a);
            break;

        case 0x48: // PHA - Push Accumulator onto Stack
            if (cpu->sp == 0x00) {
                printf("Stack overflow!\n");
                return;
            }
            write_mem(0x0100 + cpu->sp, cpu->a);
            cpu->sp--;
            break;

        case 0x68: // PLA - Pull Accumulator from Stack
            if (cpu->sp == 0xFF) {
                printf("Stack underflow!\n");
                return;
            }
            cpu->sp++;
            cpu->a = read_mem(0x0100 + cpu->sp);
            set_zn_flags(cpu, cpu->a);
            break;

        case 0x08: // PHP - Push Processor Status onto Stack
            if (cpu->sp == 0x00) {
                printf("Stack overflow!\n");
                return;
            }
            write_mem(0x0100 + cpu->sp, cpu->status);
            cpu->sp--; 
            break;

        case 0x28: // PLP - Pull Processor Status from Stack
            if (cpu->sp == 0xFF) {
                printf("Stack underflow!\n");
                return;
            }
            cpu->sp++;
            cpu->status = read_mem(0x0100 + cpu->sp);
            break;

        case 0x00: // BRK - Break (halts execution)
            cpu->status |= BREAK_FLAG;
            break;

        case 0xEA: // NOP
            break;

        case JSR_OPCODE: // JSR - Jump to Subroutine
            {
                // Fetch operand bytes (little endian)
                uint8_t low = read_mem(cpu->pc);
                uint8_t high = read_mem(cpu->pc + 1);
                cpu->pc += 2;  // Advance PC past the operand bytes
                uint16_t target_addr = (high << 8) | low;
                
                // Push return address (PC - 1) onto the stack
                uint16_t return_addr = cpu->pc - 1;
                write_mem(0x0100 + cpu->sp, (return_addr >> 8) & 0xFF); // High byte
                cpu->sp--;
                write_mem(0x0100 + cpu->sp, return_addr & 0xFF);        // Low byte
                cpu->sp--;
                
                // Jump to subroutine
                cpu->pc = target_addr;
            }
            break;

        case RTS_OPCODE: // RTS - Return from Subroutine
            {
                // Pull return address from the stack
                cpu->sp++;
                uint8_t low = read_mem(0x0100 + cpu->sp);
                cpu->sp++;
                uint8_t high = read_mem(0x0100 + cpu->sp);
                uint16_t return_addr = (high << 8) | low;
                // Set PC to return address + 1
                cpu->pc = return_addr + 1;
            }
            break;

        case 0x29: // AND Immediate
            cpu->a &= read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->a);
            break;

        case 0x09: // ORA Immediate
            cpu->a |= read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->a);
            break;

        case 0x49: // EOR Immediate
            cpu->a ^= read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->a);
            break;

        case 0xC9: // CMP Immediate
            {
                uint8_t operand = read_mem(cpu->pc++);
                uint8_t result = cpu->a - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;

        case 0xE0: // CPX Immediate
            {
                uint8_t operand = read_mem(cpu->pc++);
                uint8_t result = cpu->x - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->x >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;

        case 0xC0: // CPY Immediate
            {
                uint8_t operand = read_mem(cpu->pc++);
                uint8_t result = cpu->y - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->y >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;

        case 0xAA: // TAX
            cpu->x = cpu->a;
            set_zn_flags(cpu, cpu->x);
            break;

        case 0x8A: // TXA
            cpu->a = cpu->x;
            set_zn_flags(cpu, cpu->a);
            break;

        case 0xCA: // DEX
            cpu->x--;
            set_zn_flags(cpu, cpu->x);
            break;

        case 0xE8: // INX
            cpu->x++;
            set_zn_flags(cpu, cpu->x);
            break;

        case 0xA0: // LDY Immediate
            cpu->y = read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->y);
            break;

        case 0xA2: // LDX Immediate
            cpu->x = read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->x);
            break;

        // LDX Zero Page (opcode 0xA6)
        case 0xA6: // LDX Zero Page
            cpu->x = read_mem(get_zpg_address(cpu));
            set_zn_flags(cpu, cpu->x);
            break;

        // LDX Absolute (opcode 0xAE)
        case 0xAE: // LDX Absolute
            cpu->x = read_mem(get_abs_address(cpu));
            set_zn_flags(cpu, cpu->x);
            break;

        // LDX Absolute,X (opcode 0xBE)
        case 0xBE: // LDX Absolute,Y
            cpu->x = read_mem(get_absy_address(cpu));
            set_zn_flags(cpu, cpu->x);
            break;

        case 0x4C: // JMP Absolute
            {
                uint16_t addr = read_mem(cpu->pc);
                addr |= read_mem(cpu->pc + 1) << 8;
                cpu->pc = addr;
            }
            break;

        case 0x18: // CLC
            cpu->status &= ~CARRY_FLAG;
            break;

        case 0x38: // SEC
            cpu->status |= CARRY_FLAG;
            break;

        case 0xD8: // CLD
            cpu->status &= ~DECIMAL_FLAG;
            break;

        case 0x58: // CLI
            cpu->status &= ~INTERRUPT_FLAG;
            break;

        case 0xB8: // CLV
            cpu->status &= ~OVERFLOW_FLAG;
            break;

        case 0x78: // SEI
            cpu->status |= INTERRUPT_FLAG;
            break;

        case 0xF8: // SED
            cpu->status |= DECIMAL_FLAG;
            break;

        case 0xA5: // LDA Zero Page
            cpu->a = read_mem(get_zpg_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        case 0xB5: // LDA Zero Page,X
            cpu->a = read_mem(get_zpgx_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        case 0xAD: // LDA Absolute
            cpu->a = read_mem(get_abs_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        case 0xBD: // LDA Absolute,X
            cpu->a = read_mem(get_absx_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        case 0x85: // STA Zero Page
            write_mem(get_zpg_address(cpu), cpu->a);
            break;

        case 0x95: // STA Zero Page,X
            write_mem(get_zpgx_address(cpu), cpu->a);
            break;

        case 0x8D: // STA Absolute
            write_mem(get_abs_address(cpu), cpu->a);
            break;

        case 0x9D: // STA Absolute,X
            write_mem(get_absx_address(cpu), cpu->a);
            break;

        case 0x99: // STA Absolute,Y
            write_mem(get_absy_address(cpu), cpu->a);
            break;

        case 0x90: // BCC
            branch_if(cpu, !(cpu->status & CARRY_FLAG));
            break;

        case 0xB0: // BCS
            branch_if(cpu, (cpu->status & CARRY_FLAG));
            break;

        case 0xF0: // BEQ
            branch_if(cpu, (cpu->status & ZERO_FLAG));
            break;

        case 0xD0: // BNE
            branch_if(cpu, !(cpu->status & ZERO_FLAG));
            break;

        case 0x69: // ADC Immediate
            {
                uint8_t operand = read_mem(cpu->pc++);
                uint16_t sum = cpu->a + operand + (cpu->status & CARRY_FLAG ? 1 : 0);
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             (sum > 0xFF ? CARRY_FLAG : 0) |
                             ((sum & 0xFF) == 0 ? ZERO_FLAG : 0) |
                             (((cpu->a ^ sum) & (operand ^ sum) & 0x80) ? OVERFLOW_FLAG : 0) |
                             (sum & 0x80 ? NEGATIVE_FLAG : 0);
                cpu->a = sum & 0xFF;
            }
            break;

        // SBC Immediate (official 0xE9) and unofficial alias 0xEB
        case 0xE9: {
            uint8_t operand = read_mem(cpu->pc++);
            uint16_t diff = cpu->a - operand - (cpu->status & CARRY_FLAG ? 0 : 1);
            cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                        (diff < 0x100 ? CARRY_FLAG : 0) |
                        ((diff & 0xFF) == 0 ? ZERO_FLAG : 0) |
                        (((cpu->a ^ diff) & (~operand ^ diff) & 0x80) ? OVERFLOW_FLAG : 0) |
                        (diff & 0x80 ? NEGATIVE_FLAG : 0);
            cpu->a = diff & 0xFF;
        } break;

        case 0xEB: { // unofficial: SBC #imm
            uint8_t operand = read_mem(cpu->pc++);
            uint16_t diff = cpu->a - operand - (cpu->status & CARRY_FLAG ? 0 : 1);
            cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                        (diff < 0x100 ? CARRY_FLAG : 0) |
                        ((diff & 0xFF) == 0 ? ZERO_FLAG : 0) |
                        (((cpu->a ^ diff) & (~operand ^ diff) & 0x80) ? OVERFLOW_FLAG : 0) |
                        (diff & 0x80 ? NEGATIVE_FLAG : 0);
            cpu->a = diff & 0xFF;
        } break;

        case 0x24: // BIT Zero Page
            {
                uint8_t val = read_mem(get_zpg_address(cpu));
                cpu->status = (cpu->status & ~(ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a & val) == 0 ? ZERO_FLAG : 0) |
                             (val & 0x40 ? OVERFLOW_FLAG : 0) |
                             (val & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;

        case 0x86: // STX Zero Page
            write_mem(get_zpg_address(cpu), cpu->x);
            break;

        case 0x96: // STX Zero Page,Y
            write_mem((read_mem(cpu->pc++) + cpu->y) & 0xFF, cpu->x);
            break;

        case 0x8E: // STX Absolute
            write_mem(get_abs_address(cpu), cpu->x);
            break;

        case 0x84: // STY Zero Page
            write_mem(get_zpg_address(cpu), cpu->y);
            break;

        case 0x94: // STY Zero Page,X
            write_mem((read_mem(cpu->pc++) + cpu->x) & 0xFF, cpu->y);
            break;

        case 0x8C: // STY Absolute
            write_mem(get_abs_address(cpu), cpu->y);
            break;

        case 0x0A: // ASL A
            asl(cpu, &cpu->a);
            break;

        case 0x06: // ASL Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = read_mem(addr);
                asl(cpu, &val);
                write_mem(addr, val);
            }
            break;

        case 0xE6: // INC Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = read_mem(addr) + 1;
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;

        case 0xC6: // DEC Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = read_mem(addr) - 1;
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;

        case 0x30: // BMI
            branch_if(cpu, cpu->status & NEGATIVE_FLAG);
            break;

        case 0x10: // BPL
            branch_if(cpu, !(cpu->status & NEGATIVE_FLAG));
            break;

        case 0x70: // BVS
            branch_if(cpu, cpu->status & OVERFLOW_FLAG);
            break;

        case 0x50: // BVC
            branch_if(cpu, !(cpu->status & OVERFLOW_FLAG));
            break;

        case 0x40: // RTI
            cpu->sp++;
            cpu->status = read_mem(0x0100 + cpu->sp);
            cpu->sp++;
            cpu->pc = read_mem(0x0100 + cpu->sp);
            cpu->sp++;
            cpu->pc |= read_mem(0x0100 + cpu->sp) << 8;
            break;

        case 0x6C: // JMP Indirect
            cpu->pc = get_abs_indirect(cpu);
            break;

        case 0xA1: // LDA (indirect,X)
            cpu->a = read_mem(get_indirect_x(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        case 0xB1: // LDA (indirect),Y
            cpu->a = read_mem(get_indirect_y(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        case 0x81: // STA (indirect,X)
            write_mem(get_indirect_x(cpu), cpu->a);
            break;

        case 0x91: // STA (indirect),Y
            write_mem(get_indirect_y(cpu), cpu->a);
            break;

        case 0x61: // ADC (indirect,X)
            {
                uint8_t operand = read_mem(get_indirect_x(cpu));
                uint16_t sum = cpu->a + operand + (cpu->status & CARRY_FLAG ? 1 : 0);
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             (sum > 0xFF ? CARRY_FLAG : 0) |
                             ((sum & 0xFF) == 0 ? ZERO_FLAG : 0) |
                             (((cpu->a ^ sum) & (operand ^ sum) & 0x80) ? OVERFLOW_FLAG : 0) |
                             (sum & 0x80 ? NEGATIVE_FLAG : 0);
                cpu->a = sum & 0xFF;
            }
            break;

        case 0x71: // ADC (indirect),Y
            {
                uint8_t operand = read_mem(get_indirect_y(cpu));
                uint16_t sum = cpu->a + operand + (cpu->status & CARRY_FLAG ? 1 : 0);
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             (sum > 0xFF ? CARRY_FLAG : 0) |
                             ((sum & 0xFF) == 0 ? ZERO_FLAG : 0) |
                             (((cpu->a ^ sum) & (operand ^ sum) & 0x80) ? OVERFLOW_FLAG : 0) |
                             (sum & 0x80 ? NEGATIVE_FLAG : 0);
                cpu->a = sum & 0xFF;
            }
            break;

        case 0xE1: // SBC (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t operand = read_mem(addr);
                uint16_t diff = cpu->a - operand - (cpu->status & CARRY_FLAG ? 0 : 1);
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             (diff < 0x100 ? CARRY_FLAG : 0) |
                             ((diff & 0xFF) == 0 ? ZERO_FLAG : 0) |
                             (((cpu->a ^ diff) & (~operand ^ diff) & 0x80) ? OVERFLOW_FLAG : 0) |
                             (diff & 0x80 ? NEGATIVE_FLAG : 0);
                cpu->a = diff & 0xFF;
            }
            break;

        case 0xF1: // SBC (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t operand = read_mem(addr);
                uint16_t diff = cpu->a - operand - (cpu->status & CARRY_FLAG ? 0 : 1);
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             (diff < 0x100 ? CARRY_FLAG : 0) |
                             ((diff & 0xFF) == 0 ? ZERO_FLAG : 0) |
                             (((cpu->a ^ diff) & (~operand ^ diff) & 0x80) ? OVERFLOW_FLAG : 0) |
                             (diff & 0x80 ? NEGATIVE_FLAG : 0);
                cpu->a = diff & 0xFF;
            }
            break;

        case 0x2C: // BIT Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = read_mem(addr);
                cpu->status = (cpu->status & ~(ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a & val) == 0 ? ZERO_FLAG : 0) |
                             (val & 0x40 ? OVERFLOW_FLAG : 0) |
                             (val & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;

        case 0xB6: // LDX Zero Page,Y
            cpu->x = read_mem(get_zpgy_address(cpu));
            set_zn_flags(cpu, cpu->x);
            break;

        case 0xB9: // LDA Absolute,Y
            cpu->a = read_mem(get_absy_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        case 0x4A: // LSR A
            lsr(cpu, &cpu->a);
            break;

        case 0xA8: // TAY
            cpu->y = cpu->a;
            set_zn_flags(cpu, cpu->y);
            break;

        case 0x98: // TYA
            cpu->a = cpu->y;
            set_zn_flags(cpu, cpu->a);
            break;

        case 0xBA: // TSX
            cpu->x = cpu->sp;
            set_zn_flags(cpu, cpu->x);
            break;

        case 0x9A: // TXS
            cpu->sp = cpu->x;
            break;

        case 0xEE: // INC Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = read_mem(addr) + 1;
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;

        case 0xF6: // INC Zero Page,X
            {
                uint16_t addr = (read_mem(cpu->pc++) + cpu->x) & 0xFF;
                uint8_t val = read_mem(addr) + 1;
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;

        case 0xCE: // DEC Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = read_mem(addr) - 1;
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;

        case 0xD6: // DEC Zero Page,X
            {
                uint16_t addr = (read_mem(cpu->pc++) + cpu->x) & 0xFF;
                uint8_t val = read_mem(addr) - 1;
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;

        case 0x2A: // ROL A
            rol(cpu, &cpu->a);
            break;

        case 0x26: // ROL Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = read_mem(addr);
                rol(cpu, &val);
                write_mem(addr, val);
            }
            break;

        case 0x6A: // ROR Accumulator
            ror(cpu, &cpu->a);
            break;  

        case 0x66: // ROR Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t value = read_mem(addr);
                ror(cpu, &value);
                write_mem(addr, value);
            }
            break;

        case 0x76: // ROR Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t value = read_mem(addr);
                ror(cpu, &value);
                write_mem(addr, value);
            }
            break;
        
        case 0x6E: // ROR Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t value = read_mem(addr);
                ror(cpu, &value);
                write_mem(addr, value);
            }
            break;
        
        case 0x7E: // ROR Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t value = read_mem(addr);
                ror(cpu, &value);
                write_mem(addr, value);
            }
            break;

        // ORA (bitwise OR with A)
        case 0x05: /* zpg   */ cpu->a |= read_mem(get_zpg_address(cpu)); set_zn_flags(cpu, cpu->a); break;
        case 0x15: /* zpg,X */ cpu->a |= read_mem(get_zpgx_address(cpu)); set_zn_flags(cpu, cpu->a); break;
        case 0x0D: /* abs   */ cpu->a |= read_mem(get_abs_address(cpu));  set_zn_flags(cpu, cpu->a); break;
        case 0x1D: /* abs,X */ cpu->a |= read_mem(get_absx_address(cpu)); set_zn_flags(cpu, cpu->a); break;
        case 0x19: /* abs,Y */ cpu->a |= read_mem(get_absy_address(cpu)); set_zn_flags(cpu, cpu->a); break;
        case 0x01: /* (ind,X)*/cpu->a |= read_mem(get_indirect_x(cpu));   set_zn_flags(cpu, cpu->a); break;
        case 0x11: /* (ind),Y*/cpu->a |= read_mem(get_indirect_y(cpu));   set_zn_flags(cpu, cpu->a); break;

        // AND (bitwise AND with A) – some palette tests use these too
        case 0x25: /* zpg   */ cpu->a &= read_mem(get_zpg_address(cpu)); set_zn_flags(cpu, cpu->a); break;
        case 0x35: /* zpg,X */ cpu->a &= read_mem(get_zpgx_address(cpu));set_zn_flags(cpu, cpu->a); break;
        case 0x2D: /* abs   */ cpu->a &= read_mem(get_abs_address(cpu)); set_zn_flags(cpu, cpu->a); break;
        case 0x3D: /* abs,X */ cpu->a &= read_mem(get_absx_address(cpu));set_zn_flags(cpu, cpu->a); break;
        case 0x39: /* abs,Y */ cpu->a &= read_mem(get_absy_address(cpu));set_zn_flags(cpu, cpu->a); break;
        case 0x21: /* (ind,X)*/cpu->a &= read_mem(get_indirect_x(cpu));  set_zn_flags(cpu, cpu->a); break;
        case 0x31: /* (ind),Y*/cpu->a &= read_mem(get_indirect_y(cpu));  set_zn_flags(cpu, cpu->a); break;

        // EOR (bitwise XOR with A)
        case 0x45: /* zpg   */ cpu->a ^= read_mem(get_zpg_address(cpu)); set_zn_flags(cpu, cpu->a); break;
        case 0x55: /* zpg,X */ cpu->a ^= read_mem(get_zpgx_address(cpu));set_zn_flags(cpu, cpu->a); break;
        case 0x4D: /* abs   */ cpu->a ^= read_mem(get_abs_address(cpu)); set_zn_flags(cpu, cpu->a); break;
        case 0x5D: /* abs,X */ cpu->a ^= read_mem(get_absx_address(cpu));set_zn_flags(cpu, cpu->a); break;
        case 0x59: /* abs,Y */ cpu->a ^= read_mem(get_absy_address(cpu));set_zn_flags(cpu, cpu->a); break;
        case 0x41: /* (ind,X)*/cpu->a ^= read_mem(get_indirect_x(cpu));  set_zn_flags(cpu, cpu->a); break;
        case 0x51: /* (ind),Y*/cpu->a ^= read_mem(get_indirect_y(cpu));  set_zn_flags(cpu, cpu->a); break;

        // LDY (load Y)
        case 0xA4: /* zpg   */ cpu->y = read_mem(get_zpg_address(cpu));  set_zn_flags(cpu, cpu->y); break;
        case 0xB4: /* zpg,X */ cpu->y = read_mem(get_zpgx_address(cpu)); set_zn_flags(cpu, cpu->y); break;
        case 0xAC: /* abs   */ cpu->y = read_mem(get_abs_address(cpu));  set_zn_flags(cpu, cpu->y); break;
        case 0xBC: /* abs,X */ cpu->y = read_mem(get_absx_address(cpu)); set_zn_flags(cpu, cpu->y); break;

        // -- CMP
        case 0xC5: { uint8_t o = read_mem(get_zpg_address(cpu));  uint8_t r = cpu->a - o;
            cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                        | ((cpu->a >= o) ? CARRY_FLAG : 0)
                        | ((r == 0) ? ZERO_FLAG : 0)
                        | (r & 0x80 ? NEGATIVE_FLAG : 0); } break;
        case 0xD5: { uint8_t o = read_mem(get_zpgx_address(cpu)); uint8_t r = cpu->a - o;
            cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                        | ((cpu->a >= o) ? CARRY_FLAG : 0)
                        | ((r == 0) ? ZERO_FLAG : 0)
                        | (r & 0x80 ? NEGATIVE_FLAG : 0); } break;
        case 0xCD: { uint8_t o = read_mem(get_abs_address(cpu));  uint8_t r = cpu->a - o;
            cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                        | ((cpu->a >= o) ? CARRY_FLAG : 0)
                        | ((r == 0) ? ZERO_FLAG : 0)
                        | (r & 0x80 ? NEGATIVE_FLAG : 0); } break;
        case 0xDD: { uint8_t o = read_mem(get_absx_address(cpu)); uint8_t r = cpu->a - o;
            cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                        | ((cpu->a >= o) ? CARRY_FLAG : 0)
                        | ((r == 0) ? ZERO_FLAG : 0)
                        | (r & 0x80 ? NEGATIVE_FLAG : 0); } break;
        case 0xD9: { uint8_t o = read_mem(get_absy_address(cpu)); uint8_t r = cpu->a - o;
            cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                        | ((cpu->a >= o) ? CARRY_FLAG : 0)
                        | ((r == 0) ? ZERO_FLAG : 0)
                        | (r & 0x80 ? NEGATIVE_FLAG : 0); } break;
        case 0xC1: { uint8_t o = read_mem(get_indirect_x(cpu));   uint8_t r = cpu->a - o;
            cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                        | ((cpu->a >= o) ? CARRY_FLAG : 0)
                        | ((r == 0) ? ZERO_FLAG : 0)
                        | (r & 0x80 ? NEGATIVE_FLAG : 0); } break;
        case 0xD1: { uint8_t o = read_mem(get_indirect_y(cpu));   uint8_t r = cpu->a - o;
            cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                        | ((cpu->a >= o) ? CARRY_FLAG : 0)
                        | ((r == 0) ? ZERO_FLAG : 0)
                        | (r & 0x80 ? NEGATIVE_FLAG : 0); } break;

        // -- CPX
        case 0xE4: { uint8_t o = read_mem(get_zpg_address(cpu));  uint8_t r = cpu->x - o;
            cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                        | ((cpu->x >= o) ? CARRY_FLAG : 0)
                        | ((r == 0) ? ZERO_FLAG : 0)
                        | (r & 0x80 ? NEGATIVE_FLAG : 0); } break;
        case 0xEC: { uint8_t o = read_mem(get_abs_address(cpu));  uint8_t r = cpu->x - o;
            cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                        | ((cpu->x >= o) ? CARRY_FLAG : 0)
                        | ((r == 0) ? ZERO_FLAG : 0)
                        | (r & 0x80 ? NEGATIVE_FLAG : 0); } break;

        // -- CPY
        case 0xC4: { uint8_t o = read_mem(get_zpg_address(cpu));  uint8_t r = cpu->y - o;
            cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                        | ((cpu->y >= o) ? CARRY_FLAG : 0)
                        | ((r == 0) ? ZERO_FLAG : 0)
                        | (r & 0x80 ? NEGATIVE_FLAG : 0); } break;
        case 0xCC: { uint8_t o = read_mem(get_abs_address(cpu));  uint8_t r = cpu->y - o;
            cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                        | ((cpu->y >= o) ? CARRY_FLAG : 0)
                        | ((r == 0) ? ZERO_FLAG : 0)
                        | (r & 0x80 ? NEGATIVE_FLAG : 0); } break;
        // ADC
        case 0x65: do_adc(cpu, read_mem(get_zpg_address(cpu)));  break;
        case 0x75: do_adc(cpu, read_mem(get_zpgx_address(cpu))); break;
        case 0x6D: do_adc(cpu, read_mem(get_abs_address(cpu)));  break;
        case 0x7D: do_adc(cpu, read_mem(get_absx_address(cpu))); break;
        case 0x79: do_adc(cpu, read_mem(get_absy_address(cpu))); break;

        // SBC
        case 0xE5: do_sbc(cpu, read_mem(get_zpg_address(cpu)));  break;
        case 0xF5: do_sbc(cpu, read_mem(get_zpgx_address(cpu))); break;
        case 0xED: do_sbc(cpu, read_mem(get_abs_address(cpu)));  break;
        case 0xFD: do_sbc(cpu, read_mem(get_absx_address(cpu))); break;
        case 0xF9: do_sbc(cpu, read_mem(get_absy_address(cpu))); break;

        // ===== LSR (memory) =====
        case 0x46: { uint16_t a = get_zpg_address(cpu);  uint8_t v = read_mem(a); lsr(cpu, &v); write_mem(a, v); } break;
        case 0x56: { uint16_t a = get_zpgx_address(cpu); uint8_t v = read_mem(a); lsr(cpu, &v); write_mem(a, v); } break;
        case 0x4E: { uint16_t a = get_abs_address(cpu);  uint8_t v = read_mem(a); lsr(cpu, &v); write_mem(a, v); } break;
        case 0x5E: { uint16_t a = get_absx_address(cpu); uint8_t v = read_mem(a); lsr(cpu, &v); write_mem(a, v); } break;

        // ===== ASL (memory) =====
        case 0x0E: { uint16_t a = get_abs_address(cpu);  uint8_t v = read_mem(a); asl(cpu, &v); write_mem(a, v); } break;
        case 0x16: { uint16_t a = get_zpgx_address(cpu); uint8_t v = read_mem(a); asl(cpu, &v); write_mem(a, v); } break;
        case 0x1E: { uint16_t a = get_absx_address(cpu); uint8_t v = read_mem(a); asl(cpu, &v); write_mem(a, v); } break;

        // ===== ROL (memory) =====
        case 0x2E: { uint16_t a = get_abs_address(cpu);  uint8_t v = read_mem(a); rol(cpu, &v); write_mem(a, v); } break;
        case 0x36: { uint16_t a = get_zpgx_address(cpu); uint8_t v = read_mem(a); rol(cpu, &v); write_mem(a, v); } break;
        case 0x3E: { uint16_t a = get_absx_address(cpu); uint8_t v = read_mem(a); rol(cpu, &v); write_mem(a, v); } break;

        // ===== INC/DEC abs,X =====
        case 0xFE: { uint16_t a = get_absx_address(cpu); uint8_t v = read_mem(a) + 1; write_mem(a, v); set_zn_flags(cpu, v); } break;
        case 0xDE: { uint16_t a = get_absx_address(cpu); uint8_t v = read_mem(a) - 1; write_mem(a, v); set_zn_flags(cpu, v); } break;

        // ===== Undocumented NOPs (consume operands, do nothing) =====
        // one-byte NOPs with a zpg operand
        case 0x04: case 0x44: case 0x64: cpu->pc++; break;
        // zpg,X operand
        case 0x14: case 0x34: case 0x54: case 0x74: case 0xD4: case 0xF4:
            cpu->pc++;
            break;
        // absolute operand (2 bytes)
        case 0x0C: cpu->pc += 2; break;
        // absolute,X operand (2 bytes)
        case 0x1C: case 0x3C: case 0x5C: case 0x7C: case 0xDC: case 0xFC:
            cpu->pc += 2;
            break;

        case 0x1A: case 0x3A: case 0x5A: case 0x7A: case 0xDA: case 0xFA:
            /* do nothing */
            break;
        // ===== DCP (undocumented) = DEC mem then CMP A,mem =====
        case 0xC7: { uint16_t a = get_zpg_address(cpu);  uint8_t m = read_mem(a)-1; write_mem(a,m);
                    uint8_t r = cpu->a - m; cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                    | ((cpu->a >= m)?CARRY_FLAG:0) | ((r==0)?ZERO_FLAG:0) | (r&0x80?NEGATIVE_FLAG:0); } break;
        case 0xD7: { uint16_t a = get_zpgx_address(cpu); uint8_t m = read_mem(a)-1; write_mem(a,m);
                    uint8_t r = cpu->a - m; cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                    | ((cpu->a >= m)?CARRY_FLAG:0) | ((r==0)?ZERO_FLAG:0) | (r&0x80?NEGATIVE_FLAG:0); } break;
        case 0xCF: { uint16_t a = get_abs_address(cpu);  uint8_t m = read_mem(a)-1; write_mem(a,m);
                    uint8_t r = cpu->a - m; cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                    | ((cpu->a >= m)?CARRY_FLAG:0) | ((r==0)?ZERO_FLAG:0) | (r&0x80?NEGATIVE_FLAG:0); } break;
        case 0xDF: { uint16_t a = get_absx_address(cpu); uint8_t m = read_mem(a)-1; write_mem(a,m);
                    uint8_t r = cpu->a - m; cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                    | ((cpu->a >= m)?CARRY_FLAG:0) | ((r==0)?ZERO_FLAG:0) | (r&0x80?NEGATIVE_FLAG:0); } break;
        case 0xDB: { uint16_t a = get_absy_address(cpu); uint8_t m = read_mem(a)-1; write_mem(a,m);
                    uint8_t r = cpu->a - m; cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                    | ((cpu->a >= m)?CARRY_FLAG:0) | ((r==0)?ZERO_FLAG:0) | (r&0x80?NEGATIVE_FLAG:0); } break;
        case 0xC3: { uint16_t a = get_indirect_x(cpu);  uint8_t m = read_mem(a)-1; write_mem(a,m);
                    uint8_t r = cpu->a - m; cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                    | ((cpu->a >= m)?CARRY_FLAG:0) | ((r==0)?ZERO_FLAG:0) | (r&0x80?NEGATIVE_FLAG:0); } break;
        case 0xD3: { uint16_t a = get_indirect_y(cpu);  uint8_t m = read_mem(a)-1; write_mem(a,m);
                    uint8_t r = cpu->a - m; cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG))
                    | ((cpu->a >= m)?CARRY_FLAG:0) | ((r==0)?ZERO_FLAG:0) | (r&0x80?NEGATIVE_FLAG:0); } break;
        
        // ===== RLA (undocumented) =====
        // Rotate Left memory, then AND with accumulator
        case 0x27: { uint16_t a = get_zpg_address(cpu);  uint8_t v = read_mem(a); rol(cpu, &v); write_mem(a, v); cpu->a &= v; set_zn_flags(cpu, cpu->a); } break;
        case 0x37: { uint16_t a = get_zpgx_address(cpu); uint8_t v = read_mem(a); rol(cpu, &v); write_mem(a, v); cpu->a &= v; set_zn_flags(cpu, cpu->a); } break;
        case 0x2F: { uint16_t a = get_abs_address(cpu);  uint8_t v = read_mem(a); rol(cpu, &v); write_mem(a, v); cpu->a &= v; set_zn_flags(cpu, cpu->a); } break;
        case 0x3F: { uint16_t a = get_absx_address(cpu); uint8_t v = read_mem(a); rol(cpu, &v); write_mem(a, v); cpu->a &= v; set_zn_flags(cpu, cpu->a); } break;
        case 0x3B: { uint16_t a = get_absy_address(cpu); uint8_t v = read_mem(a); rol(cpu, &v); write_mem(a, v); cpu->a &= v; set_zn_flags(cpu, cpu->a); } break;
        case 0x23: { uint16_t a = get_indirect_x(cpu);  uint8_t v = read_mem(a); rol(cpu, &v); write_mem(a, v); cpu->a &= v; set_zn_flags(cpu, cpu->a); } break;
        case 0x33: { uint16_t a = get_indirect_y(cpu);  uint8_t v = read_mem(a); rol(cpu, &v); write_mem(a, v); cpu->a &= v; set_zn_flags(cpu, cpu->a); } break;
        case 0x80: case 0x82: case 0x89: case 0xC2: case 0xE2:
            cpu->pc++;  // consume the immediate operand
            break;
        case 0x02: case 0x12: case 0x22: case 0x32:
        case 0x42: case 0x52: case 0x62: case 0x72:
        case 0x92: case 0xB2: case 0xD2: case 0xF2:
                /* do nothing */
            break;

        // ===== SLO (undocumented) : ASL mem; A |= mem =====
        case 0x07: { uint16_t a=get_zpg_address(cpu);  uint8_t v=read_mem(a); asl(cpu,&v); write_mem(a,v); cpu->a |= v; set_zn_flags(cpu,cpu->a);} break; // zpg
        case 0x17: { uint16_t a=get_zpgx_address(cpu); uint8_t v=read_mem(a); asl(cpu,&v); write_mem(a,v); cpu->a |= v; set_zn_flags(cpu,cpu->a);} break; // zpg,X
        case 0x0F: { uint16_t a=get_abs_address(cpu);  uint8_t v=read_mem(a); asl(cpu,&v); write_mem(a,v); cpu->a |= v; set_zn_flags(cpu,cpu->a);} break; // abs
        case 0x1F: { uint16_t a=get_absx_address(cpu); uint8_t v=read_mem(a); asl(cpu,&v); write_mem(a,v); cpu->a |= v; set_zn_flags(cpu,cpu->a);} break; // abs,X
        case 0x1B: { uint16_t a=get_absy_address(cpu); uint8_t v=read_mem(a); asl(cpu,&v); write_mem(a,v); cpu->a |= v; set_zn_flags(cpu,cpu->a);} break; // abs,Y
        case 0x03: { uint16_t a=get_indirect_x(cpu);  uint8_t v=read_mem(a); asl(cpu,&v); write_mem(a,v); cpu->a |= v; set_zn_flags(cpu,cpu->a);} break; // (ind,X)
        case 0x13: { uint16_t a=get_indirect_y(cpu);  uint8_t v=read_mem(a); asl(cpu,&v); write_mem(a,v); cpu->a |= v; set_zn_flags(cpu,cpu->a);} break; // (ind),Y
        // ===== SRE (undocumented) : LSR mem; A ^= mem =====
        case 0x47: { uint16_t a=get_zpg_address(cpu);  uint8_t v=read_mem(a); lsr(cpu,&v); write_mem(a,v); cpu->a ^= v; set_zn_flags(cpu,cpu->a);} break; // zpg
        case 0x57: { uint16_t a=get_zpgx_address(cpu); uint8_t v=read_mem(a); lsr(cpu,&v); write_mem(a,v); cpu->a ^= v; set_zn_flags(cpu,cpu->a);} break; // zpg,X
        case 0x4F: { uint16_t a=get_abs_address(cpu);  uint8_t v=read_mem(a); lsr(cpu,&v); write_mem(a,v); cpu->a ^= v; set_zn_flags(cpu,cpu->a);} break; // abs
        case 0x5F: { uint16_t a=get_absx_address(cpu); uint8_t v=read_mem(a); lsr(cpu,&v); write_mem(a,v); cpu->a ^= v; set_zn_flags(cpu,cpu->a);} break; // abs,X
        case 0x5B: { uint16_t a=get_absy_address(cpu); uint8_t v=read_mem(a); lsr(cpu,&v); write_mem(a,v); cpu->a ^= v; set_zn_flags(cpu,cpu->a);} break; // abs,Y
        case 0x43: { uint16_t a=get_indirect_x(cpu);  uint8_t v=read_mem(a); lsr(cpu,&v); write_mem(a,v); cpu->a ^= v; set_zn_flags(cpu,cpu->a);} break; // (ind,X)
        case 0x53: { uint16_t a=get_indirect_y(cpu);  uint8_t v=read_mem(a); lsr(cpu,&v); write_mem(a,v); cpu->a ^= v; set_zn_flags(cpu,cpu->a);} break; // (ind),Y
        
        // ===== RRA (undocumented) : ROR mem; A += mem + C =====
        case 0x67: { uint16_t a=get_zpg_address(cpu);  uint8_t v=read_mem(a); ror(cpu,&v); write_mem(a,v); do_adc(cpu, v);} break; // zpg
        case 0x77: { uint16_t a=get_zpgx_address(cpu); uint8_t v=read_mem(a); ror(cpu,&v); write_mem(a,v); do_adc(cpu, v);} break; // zpg,X
        case 0x6F: { uint16_t a=get_abs_address(cpu);  uint8_t v=read_mem(a); ror(cpu,&v); write_mem(a,v); do_adc(cpu, v);} break; // abs
        case 0x7F: { uint16_t a=get_absx_address(cpu); uint8_t v=read_mem(a); ror(cpu,&v); write_mem(a,v); do_adc(cpu, v);} break; // abs,X
        case 0x7B: { uint16_t a=get_absy_address(cpu); uint8_t v=read_mem(a); ror(cpu,&v); write_mem(a,v); do_adc(cpu, v);} break; // abs,Y
        case 0x63: { uint16_t a=get_indirect_x(cpu);  uint8_t v=read_mem(a); ror(cpu,&v); write_mem(a,v); do_adc(cpu, v);} break; // (ind,X)
        case 0x73: { uint16_t a=get_indirect_y(cpu);  uint8_t v=read_mem(a); ror(cpu,&v); write_mem(a,v); do_adc(cpu, v);} break; // (ind),Y

        // ===== LAX (undocumented) : A=X=mem (or imm) =====
        case 0xA7: { uint8_t v=read_mem(get_zpg_address(cpu));  cpu->a=cpu->x=v; set_zn_flags(cpu, v);} break; // zpg
        case 0xB7: { uint8_t v=read_mem(get_zpgy_address(cpu)); cpu->a=cpu->x=v; set_zn_flags(cpu, v);} break; // zpg,Y
        case 0xAF: { uint8_t v=read_mem(get_abs_address(cpu));  cpu->a=cpu->x=v; set_zn_flags(cpu, v);} break; // abs
        case 0xBF: { uint8_t v=read_mem(get_absy_address(cpu)); cpu->a=cpu->x=v; set_zn_flags(cpu, v);} break; // abs,Y
        case 0xA3: { uint8_t v=read_mem(get_indirect_x(cpu));   cpu->a=cpu->x=v; set_zn_flags(cpu, v);} break; // (ind,X)
        case 0xB3: { uint8_t v=read_mem(get_indirect_y(cpu));   cpu->a=cpu->x=v; set_zn_flags(cpu, v);} break; // (ind),Y
        case 0xAB: { uint8_t v=read_mem(cpu->pc++);             cpu->a=cpu->x=v; set_zn_flags(cpu, v);} break; // imm

        // ===== SAX (undocumented) : mem = A & X =====
        case 0x87: { write_mem(get_zpg_address(cpu),  cpu->a & cpu->x); } break; // zpg
        case 0x97: { write_mem(get_zpgy_address(cpu), cpu->a & cpu->x); } break; // zpg,Y
        case 0x8F: { write_mem(get_abs_address(cpu),  cpu->a & cpu->x); } break; // abs
        case 0x83: { write_mem(get_indirect_x(cpu),   cpu->a & cpu->x); } break; // (ind,X)

        // ===== ISC/ISB (undocumented): INC mem; SBC mem =====
        case 0xE7: inc_then_sbc(cpu, get_zpg_address(cpu));  break;   // zpg
        case 0xF7: inc_then_sbc(cpu, get_zpgx_address(cpu)); break;   // zpg,X
        case 0xEF: inc_then_sbc(cpu, get_abs_address(cpu));  break;   // abs
        case 0xFF: inc_then_sbc(cpu, get_absx_address(cpu)); break;   // abs,X
        case 0xFB: inc_then_sbc(cpu, get_absy_address(cpu)); break;   // abs,Y
        case 0xE3: inc_then_sbc(cpu, get_indirect_x(cpu));   break;   // (ind,X)
        case 0xF3: inc_then_sbc(cpu, get_indirect_y(cpu));   break;   // (ind),Y

        // ===== LAS/LAR (undocumented) : A=X=SP = mem[abs,Y] & SP =====
        case 0xBB: {
            uint8_t m = read_mem(get_absy_address(cpu));
            uint8_t r = m & cpu->sp;
            cpu->a = cpu->x = cpu->sp = r;
            set_zn_flags(cpu, r);   // sets Z/N, others unaffected
        } break;

        // AHX/SHA abs,Y  (store (A & X & (high(addr)+1)) to [abs+Y])
        case 0x9F: {
            // compute effective address abs,Y without changing flags
            uint16_t base = read_mem(cpu->pc) | (read_mem(cpu->pc + 1) << 8);
            cpu->pc += 2;
            uint16_t addr = base + cpu->y;
            uint8_t high_plus1 = (uint8_t)(((addr >> 8) + 1) & 0xFF);
            uint8_t v = (cpu->a & cpu->x & high_plus1);
            write_mem(addr, v);
        } break;

        // AHX/SHA (ind),Y 0x93
        case 0x93: {
            uint16_t addr = get_indirect_y(cpu);
            uint8_t high_plus1 = (uint8_t)(((addr >> 8) + 1) & 0xFF);
            write_mem(addr, (cpu->a & cpu->x & high_plus1));
        } break;

        // SHX abs,Y 0x9E  (store X & (high(addr)+1))
        case 0x9E: {
            uint16_t base = read_mem(cpu->pc) | (read_mem(cpu->pc + 1) << 8);
            cpu->pc += 2;
            uint16_t addr = base + cpu->y;
            uint8_t high_plus1 = (uint8_t)(((addr >> 8) + 1) & 0xFF);
            write_mem(addr, (cpu->x & high_plus1));
        } break;

        // SHY abs,X 0x9C  (store Y & (high(addr)+1))
        case 0x9C: {
            uint16_t base = read_mem(cpu->pc) | (read_mem(cpu->pc + 1) << 8);
            cpu->pc += 2;
            uint16_t addr = base + cpu->x;
            uint8_t high_plus1 = (uint8_t)(((addr >> 8) + 1) & 0xFF);
            write_mem(addr, (cpu->y & high_plus1));
        } break;

        // TAS/SHS abs,Y 0x9B  (SP = A & X; store SP & (high(addr)+1))
        case 0x9B: {
            uint16_t base = read_mem(cpu->pc) | (read_mem(cpu->pc + 1) << 8);
            cpu->pc += 2;
            uint16_t addr = base + cpu->y;
            cpu->sp = (cpu->a & cpu->x);
            uint8_t high_plus1 = (uint8_t)(((addr >> 8) + 1) & 0xFF);
            write_mem(addr, (cpu->sp & high_plus1));
        } break;

        // -- JMP
        default:
            printf("Invalid opcode encountered: 0x%02X at PC: 0x%04X\n", opcode, cpu->pc);
            break;
    }
}

int cpu_step(CPU* cpu) {
    uint8_t opcode = read_mem(cpu->pc++);
    execute(cpu, opcode);
    return 1;
}