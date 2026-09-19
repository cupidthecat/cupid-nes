/*
 * cpu.h - MOS 6502 CPU emulation interface
 *
 * Author: @frankischilling
 *
 * This header defines CPU registers, status flags, controller references, bus helpers,
 * interrupt lines, reset functions, and instruction stepping interfaces shared by the
 * emulator and its hardware regression tests.
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
#include "../system/timing.h"

typedef struct {
    uint8_t cpu_offset;
    uint8_t ppu_phase;
} CpuStartupAlignment;

typedef struct { 
    uint8_t a;         // Accumulator
    uint8_t x;         // X register
    uint8_t y;         // Y register
    uint16_t pc;       // Program Counter
    uint8_t sp;        // Stack Pointer
    uint8_t status;    // Status register (NV-BDIZC)
    bool halted;       // JAM stops instruction execution until reset
} CPU;

typedef struct {
    uint8_t external_bus;
    uint8_t internal_bus;
    bool nmi_pending;
    bool nmi_line;
    bool nmi_previous_line;
    bool nmi_ready;
    bool nmi_injected;
    bool irq_polled;
    bool irq_ready;
    bool oam_dma_pending;
    uint8_t oam_dma_page;
    uint8_t ppu_master_phase;
    CpuStartupAlignment startup_alignment;
    bool joypad_read_valid;
    uint16_t joypad_read_addr;
    uint64_t joypad_read_cycle;
    uint8_t joypad_read_value;
    uint8_t joypad_write_pending;
    uint8_t joypad_write_value;
} CpuRuntimeState;

typedef struct {
    CPU cpu;
    uint8_t ram[0x0800];
    uint64_t total_cycles;
    CpuRuntimeState runtime;
} CpuMachineContext;

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

// Explicit offsets use master clocks and must fit the selected region's dividers.
void cpu_use_default_startup_alignment(void);
bool cpu_set_startup_alignment(unsigned cpu_offset, unsigned ppu_phase);
void cpu_seed_startup_alignment(uint32_t seed);
bool cpu_startup_alignment_valid(NesRegion region);
CpuStartupAlignment cpu_get_startup_alignment(void);
void cpu_set_test_mode(bool enabled);
bool cpu_test_mode_enabled(void);
bool cpu_power_on(CPU* cpu);
void cpu_soft_reset(CPU* cpu);
void cpu_reset(CPU* cpu);
void cpu_clear_internal_ram(void);
// Select an independent CPU bus/RAM context. NULL selects the ordinary console.
void cpu_select_machine(CpuMachineContext *context);
uint8_t read_mem(uint16_t addr);
void write_mem(uint16_t addr, uint8_t value);
uint8_t cpu_peek_internal_ram(uint16_t addr);
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
