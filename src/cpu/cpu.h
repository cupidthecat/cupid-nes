/*
 * cpu.h - 6502 CPU emulation header
 * 
 * Author: @frankischilling
 * 
 * This header defines the CPU structure, status flags, and function prototypes for the
 * 6502 CPU emulator. It includes register definitions, memory access functions, and
 * instruction execution interfaces.
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

#ifndef CPU_H
#define CPU_H

#include <stdint.h>
#include <stdbool.h>
#include "../joypad/joypad.h"

typedef struct { 
    uint8_t a;         // Accumulator
    uint8_t x;         // X register
    uint8_t y;         // Y register
    uint16_t pc;       // Program Counter
    uint8_t sp;        // Stack Pointer
    uint8_t status;    // Status register (NV-BDIZC)
    bool halted;       // JAM stops instruction execution until reset
} CPU;

typedef enum {
    CARRY_FLAG     = 0x01,
    ZERO_FLAG      = 0x02,
    INTERRUPT_FLAG = 0x04,
    DECIMAL_FLAG   = 0x08,
    BREAK_FLAG     = 0x10,
    UNUSED_FLAG    = 0x20,  
    OVERFLOW_FLAG  = 0x40,
    NEGATIVE_FLAG  = 0x80,
    JSR_OPCODE     = 0x20,
    RTS_OPCODE     = 0x60
} StatusFlags;

extern uint8_t memory[0x10000];

extern Joypad pad1, pad2;
extern CPU cpu;
extern uint64_t cpu_total_cycles;

void cpu_power_on(CPU* cpu);
void cpu_soft_reset(CPU* cpu);
void cpu_reset(CPU* cpu);
uint8_t read_mem(uint16_t addr);
void write_mem(uint16_t addr, uint8_t value);
// Timestamp of the current CPU bus cycle.
uint64_t cpu_get_bus_cycle(void);
void execute(CPU* cpu, uint8_t opcode);
// Clocks CPU, PPU, APU and cartridge together; callers must not clock the PPU again.
int cpu_step(CPU* cpu);
void cpu_nmi(CPU *cpu);
void cpu_set_nmi_line(bool asserted);
// Inject an already-latched NMI for tests; hardware uses cpu_set_nmi_line.
void cpu_request_nmi(void);
void cpu_irq(CPU *cpu);

#endif // CPU_H
