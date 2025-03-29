// cpu.c
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "cpu.h"

uint8_t ram[0x0800];        // 2KB internal RAM
uint8_t apu_io[0x18];       // APU + I/O registers

void cpu_reset(CPU* cpu) {
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->sp = 0xFD;
    cpu->status = 0x24;
    // Read reset vector from PRG-ROM area
    cpu->pc = (prg_rom[0x7FFD] << 8) | prg_rom[0x7FFC];
}

uint8_t read_mem(uint16_t addr) {
    // Validate address
    if (addr > 0xFFFF) {
        printf("Invalid memory read at: 0x%04X\n", addr);
        return 0;
    }

    if(addr <= 0x1FFF) {
        return ram[addr % 0x0800];
    } else if(addr <= 0x401F) {
        return apu_io[addr - 0x4000];
    } else if(addr >= 0x8000) {
        return prg_rom[addr - 0x8000];
    }
    return 0;
}

void write_mem(uint16_t addr, uint8_t value) {
    // Validate address
    if (addr > 0xFFFF) {
        printf("Invalid memory write at: 0x%04X\n", addr);
        return;
    }

    if(addr <= 0x1FFF) {
        ram[addr % 0x0800] = value;
    } else if(addr <= 0x401F) {
        apu_io[addr - 0x4000] = value;
    } else if(addr >= 0x8000) {
        prg_rom[addr - 0x8000] = value;
    }
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
    cpu->status = (cpu->status & ~CARRY_FLAG) | (new_carry << CARRY_FLAG);
    set_zn_flags(cpu, *value);
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

void execute(CPU* cpu, uint8_t opcode) {
    // Add opcode validation
    if (opcode == 0xFF) {
        printf("Invalid opcode encountered: 0x%02X\n", opcode);
        return;
    }

    // Add stack pointer validation
    if (cpu->sp < 0x01 || cpu->sp > 0xFF) {
        printf("Invalid stack pointer: 0x%02X\n", cpu->sp);
        return;
    }

    switch(opcode) {
        case 0xA9: // LDA Immediate
            cpu->a = read_mem(cpu->pc++);
            cpu->status = (cpu->a == 0) ? ZERO_FLAG :
                          (cpu->a & NEGATIVE_FLAG) ? NEGATIVE_FLAG : 0;
            break;

        case 0x48: // PHA - Push Accumulator onto Stack
            write_mem(0x0100 + cpu->sp, cpu->a);
            cpu->sp--;  // Stack pointer decrements after push
            break;

        case 0x68: // PLA - Pull Accumulator from Stack
            cpu->sp++;  // Increment SP before pulling the value
            cpu->a = read_mem(0x0100 + cpu->sp);
            cpu->status = (cpu->a == 0) ? ZERO_FLAG :
                          (cpu->a & NEGATIVE_FLAG) ? NEGATIVE_FLAG : 0;
            break;

        case 0x08: // PHP - Push Processor Status onto Stack
            write_mem(0x0100 + cpu->sp, cpu->status);
            cpu->sp--; 
            break;

        case 0x28: // PLP - Pull Processor Status from Stack
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
        case 0xBE: // LDX Absolute,X
            cpu->x = read_mem(get_absx_address(cpu));
            set_zn_flags(cpu, cpu->x);
            break;

        // Handling an illegal/unimplemented opcode.
        case 0xFF:
            // For now, we simply treat it as a NOP.
            // (Alternatively, you might log an error or implement specific illegal behavior.)
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

        case 0xE9: // SBC Immediate
            {
                uint8_t operand = read_mem(cpu->pc++);
                uint16_t diff = cpu->a - operand - (cpu->status & CARRY_FLAG ? 0 : 1);
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             (diff < 0x100 ? CARRY_FLAG : 0) |
                             ((diff & 0xFF) == 0 ? ZERO_FLAG : 0) |
                             (((cpu->a ^ diff) & (~operand ^ diff) & 0x80) ? OVERFLOW_FLAG : 0) |
                             (diff & 0x80 ? NEGATIVE_FLAG : 0);
                cpu->a = diff & 0xFF;
            }
            break;

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

        default:
            printf("Unknown opcode: 0x%02X\n", opcode);
            break;
    }
}

int cpu_step(CPU* cpu) {
    uint8_t opcode = read_mem(cpu->pc++);
    execute(cpu, opcode);
    return 1;
}