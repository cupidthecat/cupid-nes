/*
 * cpu.c - 6502 CPU emulation core
 * 
 * Author: @frankischilling
 * 
 * This file implements the MOS 6502 CPU emulator used in the NES. It handles instruction
 * execution, memory access, addressing modes, CPU registers, and interrupt handling (NMI, IRQ, BRK).
 * Includes support for all official opcodes and many unofficial ones, with accurate cycle counting
 * and timing behavior including edge cases for interrupt latency.
 * 
 * This file is part of Cupid NES Emulator.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "cpu.h"
#include "../ppu/ppu.h"
#include "../apu/apu.h"
#include "../rom/mapper.h"
#include "../joypad/joypad.h"

uint8_t ram[0x0800];        // 2KB internal RAM
#define APU_IO_SIZE 0x20              // cover $4000-$401F
uint8_t apu_io[APU_IO_SIZE];          // 32 bytes
#define APU_IDX(a) ((uint16_t)((a) - 0x4000))  // index helper
// CPU-side open-bus latch (distinct from the PPU's $200x open-bus)
static uint8_t cpu_open_bus = 0;
static inline uint8_t bus_get(void) { return cpu_open_bus; }
static inline void    bus_set(uint8_t v) { cpu_open_bus = v; }
CPU cpu;
extern Joypad pad1, pad2;

// Cycles per opcode (officials; many unofficials match their closest base)
static const uint8_t cyc[256] = {
    /*00*/ 7,6,2,8,3,3,5,5,3,2,2,2,4,4,6,6,
    /*10*/ 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
    /*20*/ 6,6,2,8,3,3,5,5,4,2,2,2,4,4,6,6,
    /*30*/ 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
    /*40*/ 6,6,2,8,3,3,5,5,3,2,2,2,3,4,6,6,
    /*50*/ 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
    /*60*/ 6,6,2,8,3,3,5,5,4,2,2,2,5,4,6,6,
    /*70*/ 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
    /*80*/ 2,6,2,6,3,3,3,3,2,2,2,2,4,4,4,4,
    /*90*/ 2,6,2,6,4,4,4,3,2,5,2,5,5,5,5,5,
    /*A0*/ 2,6,2,6,3,3,3,3,2,2,2,2,4,4,4,4,
    /*B0*/ 2,5,2,5,4,4,4,4,2,4,2,4,4,4,4,4,
    /*C0*/ 2,6,2,8,3,3,5,5,2,2,2,2,4,4,6,6,
    /*D0*/ 2,5,2,8,3,4,6,6,2,4,2,7,4,4,7,7,
    /*E0*/ 2,6,2,8,3,3,5,5,2,2,2,2,4,4,6,6,
    /*F0*/ 2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
};

void cpu_reset(CPU* cpu) {
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->sp = 0xFD;  // Initialize to 0xFD (valid stack range is 0x00-0xFF)
    cpu->status = 0x24;
    // Read reset vector from PRG-ROM area
    uint8_t lo = cart_cpu_read(0xFFFC);
    uint8_t hi = cart_cpu_read(0xFFFD);
    cpu->pc = ((uint16_t)hi << 8) | lo;
    cpu->irq_delay = 0;
    cpu->sei_delay = 0;
}

uint8_t read_mem(uint16_t addr) {
    if (addr <= 0x1FFF) { uint8_t v = ram[addr & 0x07FF]; bus_set(v); return v; }

    if (addr >= 0x2000 && addr <= 0x3FFF) {
        uint8_t v = ppu_reg_read(0x2000 | (addr & 7));
        return v; // ppu_reg_read handles PPU open bus internally
    }

    // APU + I/O $4000-$4017
    if (addr >= 0x4000 && addr <= 0x4017) {
        if (addr == 0x4016) { uint8_t v = joypad_read(&pad1); bus_set(v); return v; }
        if (addr == 0x4017) { uint8_t v = joypad_read(&pad2); bus_set(v); return v; }  // NEW
        if (addr == 0x4015) { uint8_t v = apu_read(addr);    bus_set(v); return v; }
        return bus_get(); // write-only regs -> open bus
    }

    // *** Unallocated I/O space $4018-$40FF must read as open bus ***
    if (addr >= 0x4018 && addr <= 0x40FF) return bus_get();

    // Expansion/cart space (mapper regs, PRG-RAM, etc.)
    if (addr >= 0x4100 && addr <= 0x5FFF) {
        uint8_t v = cart_cpu_read(addr);
        bus_set(v);
        return v;
    }

    // Cart space $6000-$FFFF
    if (addr >= 0x6000) { uint8_t v = cart_cpu_read(addr); bus_set(v); return v; }

    return bus_get();
}

void write_mem(uint16_t addr, uint8_t value) {
    bus_set(value); // writes still put value on the CPU bus latch

    if (addr <= 0x1FFF) { ram[addr & 0x07FF] = value; return; }

    if (addr >= 0x2000 && addr <= 0x3FFF) { ppu_reg_write(0x2000 | (addr & 7), value); return; }

    if (addr >= 0x4000 && addr <= 0x4017) {
        if (addr == 0x4014) { ppu_oam_dma(value); return; }
        if (addr == 0x4016) { joypad_write_strobe(&pad1, value); joypad_write_strobe(&pad2, value); return; }
        apu_write(addr, value);
        return;
    }

    // *** Ignore writes to $4018-$40FF (unallocated I/O, open bus) ***
    if (addr >= 0x4018 && addr <= 0x40FF) return;

    // Expansion/cart
    if (addr >= 0x4100 && addr <= 0x5FFF) { cart_cpu_write(addr, value); return; }

    if (addr >= 0x6000) { cart_cpu_write(addr, value); return; }
}

// Helper function to read the next byte without advancing the PC
static inline void dummy_read_next(CPU* cpu) {
    (void)read_mem(cpu->pc);   // PC already points to the byte after the opcode
}

void cpu_nmi(CPU* cpu) {
    uint16_t pc = cpu->pc;
    write_mem(0x0100 + cpu->sp--, (pc >> 8) & 0xFF);
    write_mem(0x0100 + cpu->sp--, pc & 0xFF);
    write_mem(0x0100 + cpu->sp--, (cpu->status & ~BREAK_FLAG) | UNUSED_FLAG);
    cpu->status |= INTERRUPT_FLAG;
    cpu->pc = read_mem(0xFFFA) | (read_mem(0xFFFB) << 8);
}

void cpu_irq(CPU* cpu) {
    uint16_t pc = cpu->pc;
    write_mem(0x0100 + cpu->sp--, (pc >> 8) & 0xFF);
    write_mem(0x0100 + cpu->sp--, pc & 0xFF);
    // Push P with B=0, U=1 for IRQ
    uint8_t p = (cpu->status & ~BREAK_FLAG) | UNUSED_FLAG;
    write_mem(0x0100 + cpu->sp--, p);
    cpu->status |= INTERRUPT_FLAG;
    cpu->pc = read_mem(0xFFFE) | (read_mem(0xFFFF) << 8);
}

// Helper function to set zero and negative flags
static void set_zn_flags(CPU* cpu, uint8_t value) {
    cpu->status = (cpu->status & ~(ZERO_FLAG|NEGATIVE_FLAG)) |
                  (value == 0 ? ZERO_FLAG : 0) |
                  (value & 0x80 ? NEGATIVE_FLAG : 0);
}

// Helper function indirect Y with optional page-cross penalty for READS
static uint16_t get_indirect_y_read(CPU* cpu) {
    uint8_t zpg = read_mem(cpu->pc++);
    uint16_t base = read_mem(zpg) | (read_mem((zpg + 1) & 0xFF) << 8);
    uint16_t addr = base + cpu->y;
    if ((base & 0xFF00) != (addr & 0xFF00)) cpu->extra_cycles += 1;  // add +1 on page cross
    return addr;
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
        uint16_t old = cpu->pc;
        cpu->pc += offset;
        cpu->extra_cycles += 1; // taken branch
        if ((old & 0xFF00) != (cpu->pc & 0xFF00)) {
            cpu->extra_cycles += 1; // page crossed
        }
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
    uint16_t ptr = read_mem(cpu->pc++);
    ptr |= read_mem(cpu->pc++) << 8;
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

// Absolute,Y with page-cross penalty for READS
static uint16_t get_absy_read(CPU* cpu) {
    uint16_t lo = read_mem(cpu->pc);
    uint16_t hi = read_mem(cpu->pc + 1);
    cpu->pc += 2;
    uint16_t base = (hi << 8) | lo;
    uint16_t addr = base + cpu->y;
    if ((base & 0xFF00) != (addr & 0xFF00)) cpu->extra_cycles += 1;
    return addr;
}

static uint16_t get_absx_read(CPU* cpu) {
    uint16_t lo = read_mem(cpu->pc);
    uint16_t hi = read_mem(cpu->pc + 1);
    cpu->pc += 2;
    uint16_t base = (hi << 8) | lo;
    uint16_t addr = base + cpu->x;
    if ((base & 0xFF00) != (addr & 0xFF00)) cpu->extra_cycles += 1;
    return addr;
}

static inline uint16_t get_absx_write(CPU* c){ uint16_t a = get_absx_address(c); (void)read_mem(a); return a; }
static inline uint16_t get_absy_write(CPU* c){ uint16_t a = get_absy_address(c); (void)read_mem(a); return a; }
static inline uint16_t get_indy_write(CPU* c){ uint16_t a = get_indirect_y(c);  (void)read_mem(a); return a; }

void execute(CPU* cpu, uint8_t opcode) {
    switch(opcode) {
        // ========== CONTROL FLOW ==========
        case 0x00: { // BRK
            // 1) Dummy read of the signature byte *and* advance PC (BRK behaves like 2-byte)
            (void)read_mem(cpu->pc++);
        
            // 2) Push return address = PC after the dummy read (i.e., address of next instruction)
            uint16_t ret = cpu->pc;
            write_mem(0x0100 + cpu->sp--, (ret >> 8) & 0xFF);
            write_mem(0x0100 + cpu->sp--, ret & 0xFF);
        
            // 3) Push status with B=1, U=1
            uint8_t p = (cpu->status | BREAK_FLAG | UNUSED_FLAG);
            write_mem(0x0100 + cpu->sp--, p);
        
            // 4) Set I
            cpu->status |= INTERRUPT_FLAG;
        
            // 5) Load IRQ/BRK vector
            cpu->pc = read_mem(0xFFFE) | (read_mem(0xFFFF) << 8);
        } break;        
        case 0xEA: // NOP
            dummy_read_next(cpu);  // Add dummy read for 1-byte NOP
            break;
        case 0x4C: // JMP Absolute
            {
                uint16_t addr = read_mem(cpu->pc);
                cpu->pc++;
                addr |= read_mem(cpu->pc) << 8;
                cpu->pc++;
                cpu->pc = addr;
            }
            break;
        case 0x6C: // JMP Indirect
            cpu->pc = get_abs_indirect(cpu);
            break;

        // ========== SUBROUTINES ==========
        case JSR_OPCODE: // 0x20
        {
            uint8_t low = read_mem(cpu->pc);
            uint8_t high = read_mem(cpu->pc + 1);
            cpu->pc += 2;  // PC now points to the instruction after JSR
            
            // Push return address (which is PC - 1, the last byte of JSR instruction)
            uint16_t return_addr = cpu->pc - 1;
            write_mem(0x0100 + cpu->sp, (return_addr >> 8) & 0xFF);
            cpu->sp--;
            write_mem(0x0100 + cpu->sp, return_addr & 0xFF);
            cpu->sp--;
            
            uint16_t target_addr = (high << 8) | low;
            cpu->pc = target_addr;
        }
        break;
        
        // FIXED: RTS - Return from Subroutine
        case RTS_OPCODE: // 0x60
        {
            dummy_read_next(cpu);  // Required dummy fetch
            cpu->sp++;
            uint8_t low  = read_mem(0x0100 + cpu->sp);
            cpu->sp++;
            uint8_t high = read_mem(0x0100 + cpu->sp);
            uint16_t return_addr = (high << 8) | low;
            
            // Return address on stack is the last byte of JSR, so add 1 to skip past it
            cpu->pc = return_addr + 1;
        }
        break;
        case 0x40: // RTI - Return from Interrupt
        {
            dummy_read_next(cpu);              // RTI dummy read
            // Pull P (SP increments before each pull)
            cpu->sp++;
            uint8_t newP = read_mem(0x0100 + cpu->sp);
            cpu->status = newP | UNUSED_FLAG;
            // Cancel any pending CLI/PLP latency. After RTI no IRQ should be taken "ignoring I".
            cpu->irq_delay = 0;

            // Pull PC — DO NOT add 1 (unlike RTS)
            cpu->sp++;
            uint8_t pcl = read_mem(0x0100 + cpu->sp);
            cpu->sp++;
            uint8_t pch = read_mem(0x0100 + cpu->sp);
            cpu->pc = ((uint16_t)pch << 8) | pcl;
        }
        break;

        // ========== STACK OPERATIONS ==========
        case 0x48: // PHA - Push Accumulator
            dummy_read_next(cpu);  // Add dummy read for 1-byte PHA
            write_mem(0x0100 + cpu->sp, cpu->a);
            cpu->sp--;
            break;
        case 0x68: // PLA - Pull Accumulator
            dummy_read_next(cpu);  // Add dummy read for 1-byte PLA
            cpu->sp++;
            cpu->a = read_mem(0x0100 + cpu->sp);
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x08: // PHP - Push Processor Status
            dummy_read_next(cpu);  // Add dummy read for 1-byte PHP
            write_mem(0x0100 + cpu->sp, cpu->status | BREAK_FLAG | UNUSED_FLAG);
            cpu->sp--;
            break;
        case 0x28: // PLP - Pull Processor Status
            dummy_read_next(cpu);  // Add dummy read for 1-byte PLP
            cpu->sp++;
            {
                uint8_t oldP = cpu->status;
                uint8_t newP = read_mem(0x0100 + cpu->sp);
                cpu->status = newP | UNUSED_FLAG;            // force U=1
                bool oldI = (oldP & INTERRUPT_FLAG) != 0;
                bool newI = (newP & INTERRUPT_FLAG) != 0;
                // Only SEI gets the 0→1 window; PLP should not open it.
                if (oldI && !newI) cpu->irq_delay = 2;      // I: 1→0 deferral
            }
            break;

        // ========== BRANCHES ==========
        case 0x90: // BCC - Branch if Carry Clear
            branch_if(cpu, !(cpu->status & CARRY_FLAG));
            break;
        case 0xB0: // BCS - Branch if Carry Set
            branch_if(cpu, (cpu->status & CARRY_FLAG));
            break;
        case 0xF0: // BEQ - Branch if Equal (Zero)
            branch_if(cpu, (cpu->status & ZERO_FLAG));
            break;
        case 0xD0: // BNE - Branch if Not Equal
            branch_if(cpu, !(cpu->status & ZERO_FLAG));
            break;
        case 0x30: // BMI - Branch if Minus (Negative)
            branch_if(cpu, cpu->status & NEGATIVE_FLAG);
            break;
        case 0x10: // BPL - Branch if Plus
            branch_if(cpu, !(cpu->status & NEGATIVE_FLAG));
            break;
        case 0x70: // BVS - Branch if Overflow Set
            branch_if(cpu, cpu->status & OVERFLOW_FLAG);
            break;
        case 0x50: // BVC - Branch if Overflow Clear
            branch_if(cpu, !(cpu->status & OVERFLOW_FLAG));
            break;

        // ========== FLAGS ==========
        case 0x18: // CLC - Clear Carry Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte CLC
            cpu->status &= ~CARRY_FLAG;
            break;
        case 0x38: // SEC - Set Carry Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte SEC
            cpu->status |= CARRY_FLAG;
            break;
        case 0xD8: // CLD - Clear Decimal Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte CLD
            cpu->status &= ~DECIMAL_FLAG;
            break;
        case 0xF8: // SED - Set Decimal Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte SED
            cpu->status |= DECIMAL_FLAG;
            break;
        case 0x58: // CLI - Clear Interrupt Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte CLI
            {
                bool was_set = (cpu->status & INTERRUPT_FLAG) != 0;
                cpu->status &= ~INTERRUPT_FLAG;
                // Start 2-phase CLI window: 2=block once, then 1=allow once ignoring I.
                if (was_set) cpu->irq_delay = 2;
            }
            break;
        case 0x78: // SEI - Set Interrupt Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte SEI
            { bool was_clear = (cpu->status & INTERRUPT_FLAG) == 0;
              cpu->status |= INTERRUPT_FLAG;
              cpu->sei_delay = was_clear ? 1 : 0;   // only create the 1-op window on 0→1
            }
            break;
        case 0xB8: // CLV - Clear Overflow Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte CLV
            cpu->status &= ~OVERFLOW_FLAG;
            break;

        // ========== LOAD ACCUMULATOR ==========
        case 0xA9: // LDA Immediate
            cpu->a = read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->a);
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
            cpu->a = read_mem(get_absx_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0xB9: // LDA Absolute,Y
            cpu->a = read_mem(get_absy_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        case 0xA1: // LDA (indirect,X)
            cpu->a = read_mem(get_indirect_x(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0xB1: // LDA (indirect),Y
            cpu->a = read_mem(get_indirect_y_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        // ========== STORE ACCUMULATOR ==========
        case 0x85: // STA Zero Page
            write_mem(get_zpg_address(cpu), cpu->a);
            break;
        case 0x95: // STA Zero Page,X
            write_mem(get_zpgx_address(cpu), cpu->a);
            break;
        case 0x8D: // STA Absolute
            write_mem(get_abs_address(cpu), cpu->a);
            break;
        case 0x9D: { // STA abs,X
            uint16_t a = get_absx_write(cpu);
            write_mem(a, cpu->a);
        } break;
        case 0x99: { // STA abs,Y
            uint16_t a = get_absy_write(cpu);
            write_mem(a, cpu->a);
        } break;   
        case 0x81: // STA (indirect,X)
            write_mem(get_indirect_x(cpu), cpu->a);
            break;
        case 0x91: { // STA (ind),Y
            uint16_t a = get_indy_write(cpu);
            write_mem(a, cpu->a);
        } break;

        // ========== LOAD X REGISTER ==========
        case 0xA2: // LDX Immediate
            cpu->x = read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->x);
            break;
        case 0xA6: // LDX Zero Page
            cpu->x = read_mem(get_zpg_address(cpu));
            set_zn_flags(cpu, cpu->x);
            break;
        case 0xB6: // LDX Zero Page,Y
            cpu->x = read_mem(get_zpgy_address(cpu));
            set_zn_flags(cpu, cpu->x);
            break;
        case 0xAE: // LDX Absolute
            cpu->x = read_mem(get_abs_address(cpu));
            set_zn_flags(cpu, cpu->x);
            break;
        case 0xBE: // LDX Absolute,Y
            cpu->x = read_mem(get_absy_read(cpu));
            set_zn_flags(cpu, cpu->x);
            break;

        // ========== STORE X REGISTER ==========
        case 0x86: // STX Zero Page
            write_mem(get_zpg_address(cpu), cpu->x);
            break;
        case 0x96: // STX Zero Page,Y
            write_mem((read_mem(cpu->pc++) + cpu->y) & 0xFF, cpu->x);
            break;
        case 0x8E: // STX Absolute
            write_mem(get_abs_address(cpu), cpu->x);
            break;

        // ========== LOAD Y REGISTER ==========
        case 0xA0: // LDY Immediate
            cpu->y = read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->y);
            break;
        case 0xA4: // LDY Zero Page
            cpu->y = read_mem(get_zpg_address(cpu));
            set_zn_flags(cpu, cpu->y);
            break;
        case 0xB4: // LDY Zero Page,X
            cpu->y = read_mem(get_zpgx_address(cpu));
            set_zn_flags(cpu, cpu->y);
            break;
        case 0xAC: // LDY Absolute
            cpu->y = read_mem(get_abs_address(cpu));
            set_zn_flags(cpu, cpu->y);
            break;
        case 0xBC: // LDY Absolute,X
            cpu->y = read_mem(get_absx_read(cpu));
            set_zn_flags(cpu, cpu->y);
            break;

        // ========== STORE Y REGISTER ==========
        case 0x84: // STY Zero Page
            write_mem(get_zpg_address(cpu), cpu->y);
            break;
        case 0x94: // STY Zero Page,X
            write_mem((read_mem(cpu->pc++) + cpu->x) & 0xFF, cpu->y);
            break;
        case 0x8C: // STY Absolute
            write_mem(get_abs_address(cpu), cpu->y);
            break;

        // ========== REGISTER TRANSFERS ==========
        case 0xAA: // TAX - Transfer A to X
            dummy_read_next(cpu);  // Add dummy read for 1-byte TAX
            cpu->x = cpu->a;
            set_zn_flags(cpu, cpu->x);
            break;
        case 0x8A: // TXA - Transfer X to A
            dummy_read_next(cpu);  // Add dummy read for 1-byte TXA
            cpu->a = cpu->x;
            set_zn_flags(cpu, cpu->a);
            break;
        case 0xA8: // TAY - Transfer A to Y
            dummy_read_next(cpu);  // Add dummy read for 1-byte TAY
            cpu->y = cpu->a;
            set_zn_flags(cpu, cpu->y);
            break;
        case 0x98: // TYA - Transfer Y to A
            dummy_read_next(cpu);  // Add dummy read for 1-byte TYA
            cpu->a = cpu->y;
            set_zn_flags(cpu, cpu->a);
            break;
        case 0xBA: // TSX - Transfer SP to X
            dummy_read_next(cpu);  // Add dummy read for 1-byte TSX
            cpu->x = cpu->sp;
            set_zn_flags(cpu, cpu->x);
            break;
        case 0x9A: // TXS - Transfer X to SP
            dummy_read_next(cpu);  // Add dummy read for 1-byte TXS
            cpu->sp = cpu->x;
            break;

        // ========== INCREMENT/DECREMENT REGISTERS ==========
        case 0xE8: // INX - Increment X
            dummy_read_next(cpu);  // Add dummy read for 1-byte INX
            cpu->x++;
            set_zn_flags(cpu, cpu->x);
            break;
        case 0xCA: // DEX - Decrement X
            dummy_read_next(cpu);  // Add dummy read for 1-byte DEX
            cpu->x--;
            set_zn_flags(cpu, cpu->x);
            break;
        case 0xC8: // INY - Increment Y
            dummy_read_next(cpu);  // Add dummy read for 1-byte INY
            cpu->y++;
            set_zn_flags(cpu, cpu->y);
            break;
        case 0x88: // DEY - Decrement Y
            dummy_read_next(cpu);  // Add dummy read for 1-byte DEY
            cpu->y--;
            set_zn_flags(cpu, cpu->y);
            break;

        // ========== LOGICAL OPERATIONS - AND ==========
        case 0x29: // AND Immediate
            cpu->a &= read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x25: // AND Zero Page
            cpu->a &= read_mem(get_zpg_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x35: // AND Zero Page,X
            cpu->a &= read_mem(get_zpgx_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x2D: // AND Absolute
            cpu->a &= read_mem(get_abs_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x3D: // AND Absolute,X
            cpu->a &= read_mem(get_absx_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x39: // AND Absolute,Y
            cpu->a &= read_mem(get_absy_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        case 0x21: // AND (indirect,X)
            cpu->a &= read_mem(get_indirect_x(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x31: // AND (indirect),Y
            cpu->a &= read_mem(get_indirect_y_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        // ========== LOGICAL OPERATIONS - ORA ==========
        case 0x09: // ORA Immediate
            cpu->a |= read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x05: // ORA Zero Page
            cpu->a |= read_mem(get_zpg_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x15: // ORA Zero Page,X
            cpu->a |= read_mem(get_zpgx_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x0D: // ORA Absolute
            cpu->a |= read_mem(get_abs_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x1D: // ORA Absolute,X
            cpu->a |= read_mem(get_absx_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x19: // ORA Absolute,Y
            cpu->a |= read_mem(get_absy_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        case 0x01: // ORA (indirect,X)
            cpu->a |= read_mem(get_indirect_x(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x11: // ORA (indirect),Y
            cpu->a |= read_mem(get_indirect_y_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        // ========== LOGICAL OPERATIONS - EOR ==========
        case 0x49: // EOR Immediate
            cpu->a ^= read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x45: // EOR Zero Page
            cpu->a ^= read_mem(get_zpg_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x55: // EOR Zero Page,X
            cpu->a ^= read_mem(get_zpgx_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x4D: // EOR Absolute
            cpu->a ^= read_mem(get_abs_address(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x5D: // EOR Absolute,X
            cpu->a ^= read_mem(get_absx_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x59: // EOR Absolute,Y
            cpu->a ^= read_mem(get_absy_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x41: // EOR (indirect,X)
            cpu->a ^= read_mem(get_indirect_x(cpu));
            set_zn_flags(cpu, cpu->a);
            break;
        case 0x51: // EOR (indirect),Y
            cpu->a ^= read_mem(get_indirect_y_read(cpu));
            set_zn_flags(cpu, cpu->a);
            break;

        // ========== ARITHMETIC - ADD ==========
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
        case 0x65: 
            do_adc(cpu, read_mem(get_zpg_address(cpu)));  
            break;
        case 0x75: 
            do_adc(cpu, read_mem(get_zpgx_address(cpu))); 
            break;
        case 0x6D: 
            do_adc(cpu, read_mem(get_abs_address(cpu)));  
            break;
        case 0x7D: 
            do_adc(cpu, read_mem(get_absx_read(cpu)));
            break;
        case 0x79: 
            do_adc(cpu, read_mem(get_absy_read(cpu)));
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
                uint8_t operand = read_mem(get_indirect_y_read(cpu));
                uint16_t sum = cpu->a + operand + (cpu->status & CARRY_FLAG ? 1 : 0);
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             (sum > 0xFF ? CARRY_FLAG : 0) |
                             ((sum & 0xFF) == 0 ? ZERO_FLAG : 0) |
                             (((cpu->a ^ sum) & (operand ^ sum) & 0x80) ? OVERFLOW_FLAG : 0) |
                             (sum & 0x80 ? NEGATIVE_FLAG : 0);
                cpu->a = sum & 0xFF;
            }
            break;

        // ========== ARITHMETIC - SUBTRACT ==========
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
        case 0xEB: // SBC Immediate (Unofficial)
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
        case 0xE5: 
            do_sbc(cpu, read_mem(get_zpg_address(cpu)));  
            break;
        case 0xF5: 
            do_sbc(cpu, read_mem(get_zpgx_address(cpu))); 
            break;
        case 0xED: 
            do_sbc(cpu, read_mem(get_abs_address(cpu)));  
            break;
        case 0xFD: 
            do_sbc(cpu, read_mem(get_absx_read(cpu)));
            break;
        case 0xF9: 
            do_sbc(cpu, read_mem(get_absy_read(cpu))); 
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
        case 0xF1:
            do_sbc(cpu, read_mem(get_indirect_y_read(cpu)));
            break;

        // ========== COMPARISON ==========
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
        case 0xC5: // CMP Zero Page
            {
                uint8_t operand = read_mem(get_zpg_address(cpu));
                uint8_t result = cpu->a - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xD5: // CMP Zero Page,X
            {
                uint8_t operand = read_mem(get_zpgx_address(cpu));
                uint8_t result = cpu->a - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xCD: // CMP Absolute
            {
                uint8_t operand = read_mem(get_abs_address(cpu));
                uint8_t result = cpu->a - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xDD: // CMP Absolute,X
            {
                uint8_t operand = read_mem(get_absx_read(cpu));
                uint8_t result = cpu->a - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xD9: // CMP Absolute,Y
            {
                uint8_t operand = read_mem(get_absy_read(cpu));
                uint8_t result = cpu->a - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xC1: // CMP (indirect,X)
            {
                uint8_t operand = read_mem(get_indirect_x(cpu));
                uint8_t result = cpu->a - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xD1: // CMP (indirect),Y
            {
                uint8_t operand = read_mem(get_indirect_y_read(cpu));
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
        case 0xE4: // CPX Zero Page
            {
                uint8_t operand = read_mem(get_zpg_address(cpu));
                uint8_t result = cpu->x - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->x >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xEC: // CPX Absolute
            {
                uint8_t operand = read_mem(get_abs_address(cpu));
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
        case 0xC4: // CPY Zero Page
            {
                uint8_t operand = read_mem(get_zpg_address(cpu));
                uint8_t result = cpu->y - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->y >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xCC: // CPY Absolute
            {
                uint8_t operand = read_mem(get_abs_address(cpu));
                uint8_t result = cpu->y - operand;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->y >= operand) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;

        // ========== BIT TEST ==========
        case 0x24: // BIT Zero Page
            {
                uint8_t val = read_mem(get_zpg_address(cpu));
                cpu->status = (cpu->status & ~(ZERO_FLAG|OVERFLOW_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a & val) == 0 ? ZERO_FLAG : 0) |
                             (val & 0x40 ? OVERFLOW_FLAG : 0) |
                             (val & 0x80 ? NEGATIVE_FLAG : 0);
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

        // ========== SHIFT LEFT ==========
        case 0x0A: // ASL A
            dummy_read_next(cpu);  // Add dummy read for 1-byte ASL A
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
        case 0x16: // ASL Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = read_mem(addr);
                asl(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x0E: // ASL Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = read_mem(addr);
                asl(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x1E: // ASL Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = read_mem(addr);
                asl(cpu, &val);
                write_mem(addr, val);
            }
            break;

        // ========== SHIFT RIGHT ==========
        case 0x4A: // LSR A
            dummy_read_next(cpu);  // Add dummy read for 1-byte LSR A
            lsr(cpu, &cpu->a);
            break;
        case 0x46: // LSR Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = read_mem(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x56: // LSR Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = read_mem(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x4E: // LSR Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = read_mem(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x5E: // LSR Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = read_mem(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
            }
            break;

        // ========== ROTATE LEFT ==========
        case 0x2A: // ROL A
            dummy_read_next(cpu);  // Add dummy read for 1-byte ROL A
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
        case 0x36: // ROL Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = read_mem(addr);
                rol(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x2E: // ROL Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = read_mem(addr);
                rol(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x3E: // ROL Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = read_mem(addr);
                rol(cpu, &val);
                write_mem(addr, val);
            }
            break;

        // ========== ROTATE RIGHT ==========
        case 0x6A: // ROR A
            dummy_read_next(cpu);  // Add dummy read for 1-byte ROR A
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

        // ========== INCREMENT MEMORY ==========
        case 0xE6: // INC Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
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
        case 0xEE: // INC Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = read_mem(addr) + 1;
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;
        case 0xFE: // INC Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = read_mem(addr) + 1;
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;

        // ========== DECREMENT MEMORY ==========
        case 0xC6: // DEC Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
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
        case 0xCE: // DEC Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = read_mem(addr) - 1;
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;
        case 0xDE: // DEC Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = read_mem(addr) - 1;
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;

        // ========== UNDOCUMENTED OPCODES - NOPs ==========
        // One-byte NOPs with zero page operand
        case 0x04: case 0x44: case 0x64:
            cpu->pc++;
            break;
        // Zero page,X operand
        case 0x14: case 0x34: case 0x54: case 0x74: case 0xD4: case 0xF4:
            cpu->pc++;
            break;
        // Absolute operand (2 bytes)
        case 0x0C:
            cpu->pc += 2;
            break;
        // Absolute,X operand (2 bytes)
        case 0x1C: case 0x3C: case 0x5C: case 0x7C: case 0xDC: case 0xFC:
            cpu->pc += 2;
            break;
        // Implied NOPs (1 byte) - Add dummy reads
        case 0x1A: case 0x3A: case 0x5A: case 0x7A: case 0xDA: case 0xFA:
            dummy_read_next(cpu);  // Add dummy read for undocumented implied NOPs
            break;
        // Halt-like opcodes - Add dummy reads
        case 0x02: case 0x12: case 0x22: case 0x32:
        case 0x42: case 0x52: case 0x62: case 0x72:
        case 0x92: case 0xB2: case 0xD2: case 0xF2:
            dummy_read_next(cpu);  // Add dummy read for halt-like opcodes
            break;
        // Immediate NOPs (consume operand)
        case 0x80: case 0x82: case 0x89: case 0xC2: case 0xE2:
            cpu->pc++;
            break;

        // ========== UNDOCUMENTED - DCP (DEC then CMP) ==========
        case 0xC7: // DCP Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t mem = read_mem(addr) - 1;
                write_mem(addr, mem);
                uint8_t result = cpu->a - mem;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= mem) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xD7: // DCP Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t mem = read_mem(addr) - 1;
                write_mem(addr, mem);
                uint8_t result = cpu->a - mem;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= mem) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xCF: // DCP Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t mem = read_mem(addr) - 1;
                write_mem(addr, mem);
                uint8_t result = cpu->a - mem;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= mem) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xDF: // DCP Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t mem = read_mem(addr) - 1;
                write_mem(addr, mem);
                uint8_t result = cpu->a - mem;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= mem) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xDB: // DCP Absolute,Y
            {
                uint16_t addr = get_absy_address(cpu);
                uint8_t mem = read_mem(addr) - 1;
                write_mem(addr, mem);
                uint8_t result = cpu->a - mem;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= mem) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xC3: // DCP (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t mem = read_mem(addr) - 1;
                write_mem(addr, mem);
                uint8_t result = cpu->a - mem;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= mem) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;
        case 0xD3: // DCP (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t mem = read_mem(addr) - 1;
                write_mem(addr, mem);
                uint8_t result = cpu->a - mem;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= mem) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;

        // ========== UNDOCUMENTED - RLA (ROL then AND) ==========
        case 0x27: // RLA Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = read_mem(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x37: // RLA Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = read_mem(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x2F: // RLA Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = read_mem(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x3F: // RLA Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = read_mem(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x3B: // RLA Absolute,Y
            {
                uint16_t addr = get_absy_address(cpu);
                uint8_t val = read_mem(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x23: // RLA (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t val = read_mem(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x33: // RLA (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t val = read_mem(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;

        // ========== UNDOCUMENTED - SLO (ASL then ORA) ==========
        case 0x07: // SLO Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = read_mem(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x17: // SLO Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = read_mem(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x0F: // SLO Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = read_mem(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x1F: // SLO Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = read_mem(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x1B: // SLO Absolute,Y
            {
                uint16_t addr = get_absy_address(cpu);
                uint8_t val = read_mem(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x03: // SLO (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t val = read_mem(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x13: // SLO (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t val = read_mem(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;

        // ========== UNDOCUMENTED - SRE (LSR then EOR) ==========
        case 0x47: // SRE Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = read_mem(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x57: // SRE Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = read_mem(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x4F: // SRE Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = read_mem(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x5F: // SRE Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = read_mem(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x5B: // SRE Absolute,Y
            {
                uint16_t addr = get_absy_address(cpu);
                uint8_t val = read_mem(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x43: // SRE (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t val = read_mem(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x53: // SRE (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t val = read_mem(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;

        // ========== UNDOCUMENTED - RRA (ROR then ADC) ==========
        case 0x67: // RRA Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = read_mem(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x77: // RRA Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = read_mem(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x6F: // RRA Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = read_mem(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x7F: // RRA Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = read_mem(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x7B: // RRA Absolute,Y
            {
                uint16_t addr = get_absy_address(cpu);
                uint8_t val = read_mem(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x63: // RRA (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t val = read_mem(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x73: // RRA (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t val = read_mem(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;

        // ========== UNDOCUMENTED - LAX (Load A and X) ==========
        case 0xA7: // LAX Zero Page
            {
                uint8_t val = read_mem(get_zpg_address(cpu));
                cpu->a = cpu->x = val;
                set_zn_flags(cpu, val);
            }
            break;
        case 0xB7: // LAX Zero Page,Y
            {
                uint8_t val = read_mem(get_zpgy_address(cpu));
                cpu->a = cpu->x = val;
                set_zn_flags(cpu, val);
            }
            break;
        case 0xAF: // LAX Absolute
            {
                uint8_t val = read_mem(get_abs_address(cpu));
                cpu->a = cpu->x = val;
                set_zn_flags(cpu, val);
            }
            break;
        case 0xBF: // LAX Absolute,Y
            {
                uint8_t val = read_mem(get_absy_address(cpu));
                cpu->a = cpu->x = val;
                set_zn_flags(cpu, val);
            }
            break;
        case 0xA3: // LAX (indirect,X)
            {
                uint8_t val = read_mem(get_indirect_x(cpu));
                cpu->a = cpu->x = val;
                set_zn_flags(cpu, val);
            }
            break;
        case 0xB3: // LAX (indirect),Y
            {
                uint8_t val = read_mem(get_indirect_y(cpu));
                cpu->a = cpu->x = val;
                set_zn_flags(cpu, val);
            }
            break;
        case 0xAB: // LAX Immediate (unofficial)
            {
                uint8_t val = read_mem(cpu->pc++);
                cpu->a = cpu->x = val;
                set_zn_flags(cpu, val);
            }
            break;

        // ========== UNDOCUMENTED - SAX (Store A AND X) ==========
        case 0x87: // SAX Zero Page
            write_mem(get_zpg_address(cpu), cpu->a & cpu->x);
            break;
        case 0x97: // SAX Zero Page,Y
            write_mem(get_zpgy_address(cpu), cpu->a & cpu->x);
            break;
        case 0x8F: // SAX Absolute
            write_mem(get_abs_address(cpu), cpu->a & cpu->x);
            break;
        case 0x83: // SAX (indirect,X)
            write_mem(get_indirect_x(cpu), cpu->a & cpu->x);
            break;

        // ========== UNDOCUMENTED - ISC (INC then SBC) ==========
        case 0xE7: // ISC Zero Page
            inc_then_sbc(cpu, get_zpg_address(cpu));
            break;
        case 0xF7: // ISC Zero Page,X
            inc_then_sbc(cpu, get_zpgx_address(cpu));
            break;
        case 0xEF: // ISC Absolute
            inc_then_sbc(cpu, get_abs_address(cpu));
            break;
        case 0xFF: // ISC Absolute,X
            inc_then_sbc(cpu, get_absx_address(cpu));
            break;
        case 0xFB: // ISC Absolute,Y
            inc_then_sbc(cpu, get_absy_address(cpu));
            break;
        case 0xE3: // ISC (indirect,X)
            inc_then_sbc(cpu, get_indirect_x(cpu));
            break;
        case 0xF3: // ISC (indirect),Y
            inc_then_sbc(cpu, get_indirect_y(cpu));
            break;

        // ========== UNDOCUMENTED - LAS (Load A, X, SP) ==========
        case 0xBB: // LAS Absolute,Y
            {
                uint8_t mem = read_mem(get_absy_address(cpu));
                uint8_t result = mem & cpu->sp;
                cpu->a = cpu->x = cpu->sp = result;
                set_zn_flags(cpu, result);
            }
            break;

        // ========== UNDOCUMENTED - SHX/SHY/SHA/TAS ==========
        case 0x9F: // AHX/SHA Absolute,Y
            {
                uint16_t base = read_mem(cpu->pc) | (read_mem(cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint16_t addr = base + cpu->y;
                uint8_t high_plus1 = (uint8_t)(((addr >> 8) + 1) & 0xFF);
                uint8_t val = (cpu->a & cpu->x & high_plus1);
                write_mem(addr, val);
            }
            break;
        case 0x93: // AHX/SHA (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t high_plus1 = (uint8_t)(((addr >> 8) + 1) & 0xFF);
                write_mem(addr, (cpu->a & cpu->x & high_plus1));
            }
            break;
        case 0x9E: // SHX Absolute,Y
            {
                uint16_t base = read_mem(cpu->pc) | (read_mem(cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint16_t addr = base + cpu->y;
                uint8_t high_plus1 = (uint8_t)(((addr >> 8) + 1) & 0xFF);
                write_mem(addr, (cpu->x & high_plus1));
            }
            break;
        case 0x9C: // SHY Absolute,X
            {
                uint16_t base = read_mem(cpu->pc) | (read_mem(cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint16_t addr = base + cpu->x;
                uint8_t high_plus1 = (uint8_t)(((addr >> 8) + 1) & 0xFF);
                write_mem(addr, (cpu->y & high_plus1));
            }
            break;
        case 0x9B: // TAS/SHS Absolute,Y
            {
                uint16_t base = read_mem(cpu->pc) | (read_mem(cpu->pc + 1) << 8);
                cpu->pc += 2;
                uint16_t addr = base + cpu->y;
                cpu->sp = (cpu->a & cpu->x);
                uint8_t high_plus1 = (uint8_t)(((addr >> 8) + 1) & 0xFF);
                write_mem(addr, (cpu->sp & high_plus1));
            }
            break;

        default:
            printf("Invalid opcode encountered: 0x%02X at PC: 0x%04X\n", opcode, cpu->pc);
            break;
    }
}

int cpu_step(CPU* cpu) {
    uint8_t opcode = read_mem(cpu->pc++);
    cpu->extra_cycles = 0;
    execute(cpu, opcode);

    // How many cycles did that opcode consume?
    int cycles = cyc[opcode] + cpu->extra_cycles;

    // Advance APU timing by *actual* CPU cycles
    apu_step(&apu, cycles);

    // ----- IRQ decision (6502 latency) -----
    int cli_phase = cpu->irq_delay;           // 2: block, 1: allow once ignoring I, 0: normal
    if (cpu->irq_delay > 0) cpu->irq_delay--; // decay the phase

    bool just_set_I = (cpu->sei_delay != 0);  // SEI creates a 0→1 window
    bool irq_line   = apu_irq_pending(&apu);

    // If SEI just executed, treat I as 0 for this boundary.
    bool i_effective = (cpu->status & INTERRUPT_FLAG) && !just_set_I;

    bool block_due_to_cli = (cli_phase == 2) && !just_set_I;   // SEI overrides CLI block
    bool ignore_I_once    = (cli_phase == 1) || just_set_I;    // allow ignoring I

    if (irq_line && !block_due_to_cli) {
        bool gate = ignore_I_once ? false : i_effective;
        if (!gate) cpu_irq(cpu);
    }
    cpu->sei_delay = 0; // consume the SEI one-shot
    return cycles;
}