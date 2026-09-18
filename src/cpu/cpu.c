/*
 * cpu.c - MOS 6502 CPU emulation core
 *
 * Author: @frankischilling
 *
 * This file implements the NES CPU core, including instruction execution, addressing
 * modes, bus reads and writes, interrupts, DMA timing, controller I/O, and official and
 * undocumented 6502 opcodes used by supported software.
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
#include "../apu/epsm.h"
#include "../rom/mapper.h"
#include "../joypad/joypad.h"
#include "../system/timing.h"
#include "../system/vs_system.h"

uint8_t ram[0x0800];        // 2KB internal RAM
#define APU_IO_SIZE 0x20              // cover $4000-$401F
uint8_t apu_io[APU_IO_SIZE];          // 32 bytes
#define APU_IDX(a) ((uint16_t)((a) - 0x4000))
uint64_t cpu_total_cycles = 0;
static CpuRuntimeState main_runtime = {.ppu_master_phase = 3};
static CpuRuntimeState *cpu_runtime = &main_runtime;
static uint8_t *cpu_ram = ram;
static uint64_t *cpu_cycles = &cpu_total_cycles;
static enum { ALIGNMENT_DEFAULT, ALIGNMENT_EXPLICIT, ALIGNMENT_SEEDED } alignment_mode;
static CpuStartupAlignment configured_alignment;
static uint32_t alignment_random_state;
static bool cpu_test_mode;

void cpu_set_test_mode(bool enabled) { cpu_test_mode = enabled; }
bool cpu_test_mode_enabled(void) { return cpu_test_mode; }

void cpu_use_default_startup_alignment(void) {
    alignment_mode = ALIGNMENT_DEFAULT;
}

bool cpu_set_startup_alignment(unsigned cpu_offset, unsigned ppu_phase) {
    const NesTiming *timing = nes_timing();
    if (cpu_offset >= timing->cpu_divider || ppu_phase >= timing->ppu_divider)
        return false;
    configured_alignment = (CpuStartupAlignment){(uint8_t)cpu_offset, (uint8_t)ppu_phase};
    alignment_mode = ALIGNMENT_EXPLICIT;
    return true;
}

void cpu_seed_startup_alignment(uint32_t seed) {
    alignment_random_state = seed;
    alignment_mode = ALIGNMENT_SEEDED;
}

bool cpu_startup_alignment_valid(NesRegion region) {
    const NesTiming *timing = nes_timing_for_region(region);
    return alignment_mode != ALIGNMENT_EXPLICIT
        || (configured_alignment.cpu_offset < timing->cpu_divider
            && configured_alignment.ppu_phase < timing->ppu_divider);
}

CpuStartupAlignment cpu_get_startup_alignment(void) {
    return cpu_runtime->startup_alignment;
}

static uint8_t random_alignment_offset(uint32_t limit) {
    uint32_t value;
    uint32_t threshold = (uint32_t)(0u - limit) % limit;
    do {
        alignment_random_state = alignment_random_state * 1664525u + 1013904223u;
        value = alignment_random_state;
        value ^= value >> 16;
    } while (value < threshold);
    return (uint8_t)(value % limit);
}

#define cpu_external_bus (cpu_runtime->external_bus)
#define cpu_internal_bus (cpu_runtime->internal_bus)
#define cpu_nmi_pending (cpu_runtime->nmi_pending)
#define cpu_nmi_line (cpu_runtime->nmi_line)
#define cpu_nmi_previous_line (cpu_runtime->nmi_previous_line)
#define cpu_nmi_ready (cpu_runtime->nmi_ready)
#define cpu_nmi_injected (cpu_runtime->nmi_injected)
#define cpu_irq_polled (cpu_runtime->irq_polled)
#define cpu_irq_ready (cpu_runtime->irq_ready)
#define cpu_oam_dma_pending (cpu_runtime->oam_dma_pending)
#define cpu_oam_dma_page (cpu_runtime->oam_dma_page)
#define ppu_master_phase (cpu_runtime->ppu_master_phase)
#define joypad_read_valid (cpu_runtime->joypad_read_valid)
#define joypad_read_addr (cpu_runtime->joypad_read_addr)
#define joypad_read_cycle (cpu_runtime->joypad_read_cycle)
#define joypad_read_value (cpu_runtime->joypad_read_value)
#define joypad_write_pending (cpu_runtime->joypad_write_pending)
#define joypad_write_value (cpu_runtime->joypad_write_value)
#define active_cpu_cycles (*cpu_cycles)

// CPU-side data-bus latches (distinct from the PPU's $200x open-bus).
static inline uint8_t bus_get(void) { return cpu_external_bus; }
static inline uint8_t bus_get_internal(void) { return cpu_internal_bus; }
static inline void bus_set(uint8_t v) {
    cpu_external_bus = v;
    cpu_internal_bus = v;
}
static inline void bus_set_external(uint8_t v) { cpu_external_bus = v; }
static inline void bus_set_internal(uint8_t v) { cpu_internal_bus = v; }
CPU cpu;
extern Joypad pad1, pad2;
static CPU *running_cpu = NULL;
static bool in_bus_cycle = false;

typedef enum {
    BUS_LATCH_BOTH,
    BUS_LATCH_INTERNAL,
    BUS_LATCH_EXTERNAL,
} BusLatchTarget;

static uint8_t read_bus(uint16_t addr);
static uint8_t read_bus_target(uint16_t addr, BusLatchTarget target);
static uint8_t finish_bus_read_target(uint16_t addr, BusLatchTarget target, uint8_t value);
static void process_pending_dma(uint16_t read_addr, bool opcode_fetch);

uint64_t cpu_get_bus_cycle(void) {
    return active_cpu_cycles;
}

void cpu_select_machine(CpuMachineContext *context) {
    running_cpu = NULL;
    in_bus_cycle = false;
    if (context) {
        cpu_runtime = &context->runtime;
        cpu_ram = context->ram;
        cpu_cycles = &context->total_cycles;
    } else {
        cpu_runtime = &main_runtime;
        cpu_ram = ram;
        cpu_cycles = &cpu_total_cycles;
    }
}

static void clock_ppu_master(unsigned clocks) {
    const NesTiming *timing = nes_timing();
    epsm_clock_master(clocks, (uint32_t)(timing->cpu_hz * timing->cpu_divider));
    unsigned phase = ppu_master_phase + clocks;
    ppu_step_dots((int)(phase / timing->ppu_divider));
    ppu_master_phase = (uint8_t)(phase % timing->ppu_divider);
}

static void begin_cpu_cycle(bool read) {
    const NesTiming *timing = nes_timing();
    unsigned start_clocks = timing->cpu_divider / 2;
    in_bus_cycle = true;
    clock_ppu_master(read ? start_clocks - 1 : start_clocks + 1);
    active_cpu_cycles++;
    cart_clock_cpu_cycle(!read);
    apu_step(apu_active_state(), 1);
    if (joypad_write_pending && --joypad_write_pending == 0) {
        epsm_write_4016(cpu_external_bus, joypad_write_value);
        joypad_write_ports(joypad_write_value);
    }
}

static void end_cpu_cycle(bool read) {
    const NesTiming *timing = nes_timing();
    unsigned start_clocks = timing->cpu_divider / 2;
    unsigned end_clocks = timing->cpu_divider - start_clocks;
    clock_ppu_master(read ? end_clocks + 1 : end_clocks - 1);
    cpu_nmi_ready = cpu_nmi_pending;
    if (cpu_nmi_line && !cpu_nmi_previous_line) cpu_nmi_pending = true;
    cpu_nmi_previous_line = cpu_nmi_line;
    cpu_irq_ready = cpu_irq_polled;
    cpu_irq_polled = (apu_irq_pending(apu_active_state()) || cart_irq_pending()
                      || epsm_irq_pending()
                      || vs_external_irq_pending()) &&
                     !(running_cpu->status & INTERRUPT_FLAG);
    in_bus_cycle = false;
}

static uint8_t read_joypad_port(Joypad *pad, uint16_t addr) {
    uint64_t cycle = cpu_get_bus_cycle();
    // The original NTSC Famicom clocks each read. Other models hold the
    // controller read line low across adjacent accesses to the same port.
    bool repeat = !joypad_clocks_adjacent_reads() && running_cpu &&
                  joypad_read_valid && joypad_read_addr == addr &&
                  cycle == joypad_read_cycle + 1;
    uint8_t value = repeat ? joypad_read_value : joypad_read_port(pad, addr - 0x4016u);
    joypad_read_addr = addr;
    joypad_read_cycle = cycle;
    joypad_read_value = value;
    joypad_read_valid = running_cpu != NULL;
    return value;
}

static uint16_t read_mem_word(uint16_t addr) {
    uint8_t lo = read_mem(addr);
    uint8_t hi = read_mem((uint16_t)(addr + 1));
    return (uint16_t)(lo | ((uint16_t)hi << 8));
}

static uint16_t read_zpg_word(uint8_t addr) {
    uint8_t lo = read_mem(addr);
    uint8_t hi = read_mem((uint8_t)(addr + 1));
    return (uint16_t)(lo | ((uint16_t)hi << 8));
}

#ifdef VBL_TIMING_LOG
#define NMI_LOG(fmt, ...) do { \
    fprintf(stderr, "[NMI cpu=%llu] " fmt "\n", \
            (unsigned long long)cpu_total_cycles, ##__VA_ARGS__); \
} while (0)
#else
#define NMI_LOG(...) do {} while (0)
#endif

#ifdef NMI_BRK_DEBUG
static uint32_t nmi_brk_log_count = 0;
static const uint32_t nmi_brk_log_limit = 12000;
#define NMI_BRK_LOG(fmt, ...) do { \
    if (nmi_brk_log_count < nmi_brk_log_limit) { \
        fprintf(stderr, "[NMI/BRK cpu=%llu] " fmt "\n", \
                (unsigned long long)cpu_total_cycles, ##__VA_ARGS__); \
        nmi_brk_log_count++; \
    } \
} while (0)
#else
#define NMI_BRK_LOG(...) do {} while (0)
#endif

static uint8_t reset_bus_read(uint16_t addr) {
    begin_cpu_cycle(true);
    uint8_t value = read_bus(addr);
    end_cpu_cycle(true);
    return finish_bus_read_target(addr, BUS_LATCH_BOTH, value);
}

static void cpu_reset_sequence(CPU* cpu) {
    running_cpu = cpu;
    in_bus_cycle = false;
    joypad_read_valid = false;

    (void)reset_bus_read(cpu->pc);
    (void)reset_bus_read(cpu->pc);
    (void)reset_bus_read((uint16_t)(0x0100u | cpu->sp));
    cpu->sp--;
    (void)reset_bus_read((uint16_t)(0x0100u | cpu->sp));
    cpu->sp--;
    (void)reset_bus_read((uint16_t)(0x0100u | cpu->sp));
    cpu->sp--;
    uint8_t lo = reset_bus_read(0xFFFC);
    uint8_t hi = reset_bus_read(0xFFFD);
    cpu->pc = (uint16_t)(lo | ((uint16_t)hi << 8));

    running_cpu = NULL;
    in_bus_cycle = false;
    cpu->halted = false;
    cpu_nmi_pending = false;
    cpu_nmi_previous_line = cpu_nmi_line;
    cpu_nmi_ready = cpu_nmi_injected = false;
    cpu_irq_polled = cpu_irq_ready = false;
    cpu_oam_dma_pending = false;
    joypad_read_valid = false;
    joypad_write_pending = 0;
}

bool cpu_power_on(CPU* cpu) {
    if (!cpu || !cpu_startup_alignment_valid(nes_timing()->region)) return false;
    CpuStartupAlignment alignment = {0, (uint8_t)(nes_timing()->ppu_divider - 1)};
    if (alignment_mode == ALIGNMENT_EXPLICIT) {
        alignment = configured_alignment;
    } else if (alignment_mode == ALIGNMENT_SEEDED) {
        alignment.cpu_offset = random_alignment_offset(nes_timing()->cpu_divider);
        alignment.ppu_phase = random_alignment_offset(nes_timing()->ppu_divider);
    }
    running_cpu = NULL;
    in_bus_cycle = false;
    active_cpu_cycles = 0;
    ppu_master_phase = alignment.ppu_phase;
    cpu_runtime->startup_alignment = alignment;
    cpu_external_bus = 0;
    cpu_internal_bus = 0;
    joypad_read_valid = false;
    joypad_write_pending = 0;
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->pc = 0;
    cpu->sp = 0;
    cpu->status = INTERRUPT_FLAG | UNUSED_FLAG;
    cpu->halted = false;
    epsm_power_on();
    // The default starts the PPU one clock before reset. Optional CPU offset
    // adds master clocks before the same seven bus accesses; no CPU cycle is skipped.
    clock_ppu_master(nes_timing()->ppu_divider + alignment.cpu_offset);
    cpu_reset_sequence(cpu);
    return true;
}

void cpu_soft_reset(CPU* cpu) {
    cpu->status = (cpu->status | INTERRUPT_FLAG | UNUSED_FLAG) & ~BREAK_FLAG;
    cpu_reset_sequence(cpu);
}

void cpu_reset(CPU* cpu) {
    cpu_power_on(cpu);
}

static inline void bus_latch(BusLatchTarget target, uint8_t value) {
    if (target != BUS_LATCH_EXTERNAL) bus_set_internal(value);
    if (target != BUS_LATCH_INTERNAL) bus_set_external(value);
}

static uint8_t read_bus_target(uint16_t addr, BusLatchTarget target) {
    if (addr <= 0x1FFF) {
        uint8_t v = cpu_ram[addr & 0x07FF];
        bus_latch(target, v);
        return v;
    }

    if (addr >= 0x2000 && addr <= 0x3FFF) {
        uint8_t v = ppu_reg_read(0x2000 | (addr & 7));
        bus_latch(target, v);
        return v; // ppu_reg_read handles PPU open bus internally
    }

    // APU + I/O $4000-$4017
    if (addr >= 0x4000 && addr <= 0x4017) {
        if (addr == 0x4016) {
            if (vs_enabled()) {
                uint8_t v = vs_read_controller_port(0);
                bus_latch(target, v);
                return v;
            }
            uint8_t mask = joypad_open_bus_mask(0);
            uint8_t v = (uint8_t)((bus_get() & mask) | (read_joypad_port(&pad1, addr) & ~mask));
            bus_latch(target, v);
            return v;
        }
        if (addr == 0x4017) {
            if (vs_enabled()) {
                uint8_t v = vs_read_controller_port(1);
                bus_latch(target, v);
                return v;
            }
            uint8_t mask = joypad_open_bus_mask(1);
            uint8_t v = (uint8_t)((bus_get() & mask) | (read_joypad_port(&pad2, addr) & ~mask));
            bus_latch(target, v);
            return v;
        }
        if (addr == 0x4015) {
            // $4015 does not drive all data lines; bit 5 comes from open bus.
            uint8_t status = apu_read(addr);
            uint8_t v = (uint8_t)((status & 0xDFu) | (bus_get_internal() & 0x20u));
            bus_set_internal(v);
            return v;
        }
        uint8_t v = bus_get(); // write-only regs -> external open bus
        bus_latch(target, v);
        return v;
    }

    if (cpu_test_mode && addr >= 0x4018 && addr <= 0x401A) {
        uint8_t value = apu_read_test_output(addr);
        bus_latch(target, value);
        return value;
    }

    // The cartridge decides which expansion registers, ROM and RAM drive the bus.
    if (addr >= 0x4020) {
        uint8_t protected_value;
        if (vs_protection_read(addr, &protected_value)) {
            bus_latch(target, protected_value);
            return protected_value;
        }
        uint8_t v = cart_cpu_read_bus(addr, bus_get());
        bus_latch(target, v);
        return v;
    }

    uint8_t v = bus_get();
    bus_latch(target, v);
    return v;
}

static uint8_t read_bus(uint16_t addr) {
    return read_bus_target(addr, BUS_LATCH_BOTH);
}

static uint8_t finish_bus_read_target(uint16_t addr, BusLatchTarget target, uint8_t value) {
    if (addr >= 0x2000 && addr <= 0x3FFF) {
        value = ppu_reg_read_finish((uint16_t)(0x2000 | (addr & 7)), value);
        bus_latch(target, value);
    }
    return value;
}

static void write_bus(uint16_t addr, uint8_t value) {
    uint8_t previous_bus = bus_get();
    bus_set(value); // writes still put value on the CPU bus latch

    if (addr <= 0x1FFF) { cpu_ram[addr & 0x07FF] = value; return; }

    if (addr >= 0x2000 && addr <= 0x3FFF) {
        // Cartridge address decoding sees the CPU address before PPU mirroring.
        if (addr == 0x2000 && !vs_ppu_is_2c05()) cart_notify_ppu_ctrl_write(value);
        ppu_reg_write_cpu(0x2000 | (addr & 7), value, previous_bus);
        return;
    }

    if (addr >= 0x4000 && addr <= 0x4017) {
        if (addr == 0x4014) {
            cpu_oam_dma_page = value;
            cpu_oam_dma_pending = true;
            return;
        }
        if (addr == 0x4016) {
            if (vs_enabled()) {
                vs_write_4016(value);
                return;
            }
            if (!running_cpu || !in_bus_cycle) {
                epsm_write_4016(value, value);
                joypad_write_ports(value);
                joypad_write_pending = 0;
            } else {
                joypad_write_value = value;
                // OUT pins update at the next PUT boundary. A write on GET remains
                // pending long enough for an immediately following PUT write to replace it.
                joypad_write_pending = (active_cpu_cycles & 1u) ? 1 : 2;
            }
            return;
        }
        apu_write(addr, value);
        return;
    }

    if (addr >= 0x401C && addr <= 0x401F) epsm_write_port(addr, value);
    if (addr >= 0x4020) cart_cpu_write(addr, value);
}

static uint8_t read_mem_cycle(uint16_t addr, bool opcode_fetch) {
    if (!running_cpu || in_bus_cycle) return read_bus(addr);
    process_pending_dma(addr, opcode_fetch);
    begin_cpu_cycle(true);
    uint8_t value = read_bus(addr);
    end_cpu_cycle(true);
    return finish_bus_read_target(addr, BUS_LATCH_BOTH, value);
}

uint8_t read_mem(uint16_t addr) {
    return read_mem_cycle(addr, false);
}

void write_mem(uint16_t addr, uint8_t value) {
    if (!running_cpu || in_bus_cycle) {
        write_bus(addr, value);
        return;
    }
    begin_cpu_cycle(false);
    write_bus(addr, value);
    end_cpu_cycle(false);
}

static inline void dummy_read_next(CPU* cpu) {
    (void)read_mem(cpu->pc);   // PC already points to the byte after the opcode
}

static inline uint8_t rmw_fetch(uint16_t addr) {
    uint8_t value = read_mem(addr);
    write_mem(addr, value);
    return value;
}

void cpu_nmi(CPU* cpu) {
    cpu_nmi_pending = true;
    cpu_irq(cpu);
}

void cpu_set_nmi_line(bool asserted) {
    cpu_nmi_line = asserted;
}

void cpu_request_nmi(void) {
    cpu_nmi_pending = cpu_nmi_injected = true;
}

void cpu_irq(CPU* cpu) {
    NMI_BRK_LOG("cpu_irq enter pc=%04X sp=%02X p=%02X", cpu->pc, cpu->sp, cpu->status);
    (void)read_mem_cycle(cpu->pc, true);
    dummy_read_next(cpu);
    uint16_t pc = cpu->pc;
    write_mem(0x0100 + cpu->sp--, (pc >> 8) & 0xFF);
    write_mem(0x0100 + cpu->sp--, pc & 0xFF);
    // An NMI detected before this point replaces the IRQ vector.
    uint16_t vector = cpu_nmi_pending ? 0xFFFA : 0xFFFE;
    if (cpu_nmi_pending) cpu_nmi_pending = cpu_nmi_injected = false;
    // Push P with B=0, U=1 for IRQ
    uint8_t p = (cpu->status & ~BREAK_FLAG) | UNUSED_FLAG;
    write_mem(0x0100 + cpu->sp--, p);
    cpu->status |= INTERRUPT_FLAG;
    cpu->pc = read_mem_word(vector);
    NMI_BRK_LOG("cpu_irq vector=%04X pushedP=%02X newSP=%02X", cpu->pc, p, cpu->sp);
}

static void set_zn_flags(CPU* cpu, uint8_t value) {
    cpu->status = (cpu->status & ~(ZERO_FLAG|NEGATIVE_FLAG)) |
                  (value == 0 ? ZERO_FLAG : 0) |
                  (value & 0x80 ? NEGATIVE_FLAG : 0);
}

static uint16_t get_indirect_y_read(CPU* cpu) {
    uint8_t zpg = read_mem(cpu->pc++);
    uint16_t base = read_zpg_word(zpg);
    uint16_t addr = base + cpu->y;
    if ((base & 0xFF00) != (addr & 0xFF00)) {
        uint16_t dummy = (base & 0xFF00) | (addr & 0x00FF); // no carry into high byte
        (void)read_mem(dummy);
    }
    return addr;
}

static uint16_t get_zpg_address(CPU* cpu) {
    return read_mem(cpu->pc++);
}

static uint16_t get_zpgx_address(CPU* cpu) {
    uint8_t base = read_mem(cpu->pc++);
    (void)read_mem(base);
    return (uint8_t)(base + cpu->x);
}

static uint16_t get_abs_address(CPU* cpu) {
    uint16_t addr = read_mem(cpu->pc);
    addr |= read_mem(cpu->pc + 1) << 8;
    cpu->pc += 2;
    return addr;
}

static uint16_t get_absx_address(CPU* cpu) {
    uint16_t base = get_abs_address(cpu);
    uint16_t addr = (uint16_t)(base + cpu->x);
    (void)read_mem((base & 0xFF00u) | (addr & 0x00FFu));
    return addr;
}

static uint16_t get_absy_address(CPU* cpu) {
    uint16_t base = get_abs_address(cpu);
    uint16_t addr = (uint16_t)(base + cpu->y);
    (void)read_mem((base & 0xFF00u) | (addr & 0x00FFu));
    return addr;
}

static void branch_if(CPU* cpu, bool condition) {
    int8_t offset = read_mem(cpu->pc++);
    if(condition) {
        uint16_t old = cpu->pc;
        // Taken branches poll on their second cycle and again if a page correction follows.
        cpu_irq_polled = cpu_irq_ready;
        (void)read_mem(old);
        cpu->pc += offset;
        if ((old & 0xFF00) != (cpu->pc & 0xFF00)) {
            cpu_irq_polled |= cpu_irq_ready;
            (void)read_mem((old & 0xFF00u) | (cpu->pc & 0x00FFu));
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
    (void)read_mem(zpg);
    return read_zpg_word((uint8_t)(zpg + cpu->x));
}

// Indirect Indexed: (indirect),Y 
static uint16_t get_indirect_y(CPU* cpu) {
    uint8_t zpg = read_mem(cpu->pc++);
    uint16_t base = read_zpg_word(zpg);
    uint16_t addr = (uint16_t)(base + cpu->y);
    (void)read_mem((base & 0xFF00u) | (addr & 0x00FFu));
    return addr;
}

// Absolute Indirect (for JMP)
static uint16_t get_abs_indirect(CPU* cpu) {
    uint16_t ptr = read_mem(cpu->pc++);
    ptr |= read_mem(cpu->pc++) << 8;
    // Handle 6502 page boundary bug
    uint8_t lo = read_mem(ptr);
    uint8_t hi = read_mem((ptr & 0xFF00) | ((ptr + 1) & 0xFF));
    return (uint16_t)(lo | ((uint16_t)hi << 8));
}

static uint16_t get_zpgy_address(CPU* cpu) {
    uint8_t base = read_mem(cpu->pc++);
    (void)read_mem(base);
    return (uint8_t)(base + cpu->y);
}

static inline void inc_then_sbc(CPU* cpu, uint16_t addr) {
    uint8_t v = (uint8_t)(rmw_fetch(addr) + 1);
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
    if ((base & 0xFF00) != (addr & 0xFF00)) {
        uint16_t dummy = (base & 0xFF00) | (addr & 0x00FF); // no carry into high byte
        (void)read_mem(dummy);
    }
    return addr;
}

static uint16_t get_absx_read(CPU* cpu) {
    uint16_t lo = read_mem(cpu->pc);
    uint16_t hi = read_mem(cpu->pc + 1);
    cpu->pc += 2;
    uint16_t base = (hi << 8) | lo;
    uint16_t addr = base + cpu->x;
    if ((base & 0xFF00) != (addr & 0xFF00)) {
        uint16_t dummy = (base & 0xFF00) | (addr & 0x00FF); // no carry into high byte
        (void)read_mem(dummy);
    }
    return addr;
}

static void masked_indexed_store(uint16_t base, uint8_t index, uint8_t reg) {
    uint16_t addr = (uint16_t)(base + index);
    uint64_t before_dummy = active_cpu_cycles;
    (void)read_mem((base & 0xFF00u) | (addr & 0x00FFu));
    bool dma_interrupted_dummy = active_cpu_cycles - before_dummy > 1;
    uint8_t value = dma_interrupted_dummy ? reg : (uint8_t)(reg & (uint8_t)((base >> 8) + 1));
    if ((base & 0xFF00u) != (addr & 0xFF00u)) {
        addr = (uint16_t)((addr & 0x00FFu) | ((uint16_t)((addr >> 8) & reg) << 8));
    }
    write_mem(addr, value);
}

void execute(CPU* cpu, uint8_t opcode) {
    switch(opcode) {
        // Control flow.
        case 0x00: { // BRK
            // 1) Dummy read of the signature byte *and* advance PC (BRK behaves like 2-byte)
            (void)read_mem(cpu->pc++);
        
            // 2) Push return address = PC after the dummy read (i.e., address of next instruction)
            uint16_t ret = cpu->pc;
            write_mem(0x0100 + cpu->sp--, (ret >> 8) & 0xFF);
            write_mem(0x0100 + cpu->sp--, ret & 0xFF);
            // An NMI detected during the stack pushes can replace BRK's vector.
            bool nmi_interrupts_brk = cpu_nmi_pending;
        
            // 3) Push status with B=1, U=1
            uint8_t p = (cpu->status | BREAK_FLAG | UNUSED_FLAG);
            write_mem(0x0100 + cpu->sp--, p);
        
            // 4) Set I
            cpu->status |= INTERRUPT_FLAG;
        
            // 5) Load vector (NMI has priority if it interrupts BRK)
            if (nmi_interrupts_brk) {
                cpu_nmi_pending = false;
                cpu_nmi_injected = false;
                NMI_LOG("BRK sequence interrupted by NMI");
                cpu->pc = read_mem_word(0xFFFA);
                NMI_BRK_LOG("BRK->NMI vector=%04X pushedP=%02X", cpu->pc, p);
            } else {
                cpu->pc = read_mem_word(0xFFFE);
                NMI_BRK_LOG("BRK->IRQ vector=%04X pushedP=%02X", cpu->pc, p);
            }
            // An edge detected during the vector fetch waits for the handler's first instruction.
            cpu_nmi_ready = false;
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

        // Subroutines.
        case JSR_OPCODE: // 0x20
        {
            // 6502 JSR bus order:
            // 2) read target low from PC, increment PC
            // 3) dummy read from stack
            // 4) push PCH of return address
            // 5) push PCL of return address
            // 6) read target high from PC and jump
            uint8_t low = read_mem(cpu->pc++);
            (void)read_mem((uint16_t)(0x0100u + cpu->sp));

            write_mem((uint16_t)(0x0100u + cpu->sp), (uint8_t)((cpu->pc >> 8) & 0xFFu));
            cpu->sp--;
            write_mem((uint16_t)(0x0100u + cpu->sp), (uint8_t)(cpu->pc & 0xFFu));
            cpu->sp--;

            uint8_t high = read_mem(cpu->pc);
            cpu->pc = (uint16_t)(((uint16_t)high << 8) | low);
        }
        break;
        
        // FIXED: RTS - Return from Subroutine
        case RTS_OPCODE: // 0x60
        {
            dummy_read_next(cpu);  // Required dummy fetch
            (void)read_mem(0x0100 + cpu->sp);
            cpu->sp++;
            uint8_t low  = read_mem(0x0100 + cpu->sp);
            cpu->sp++;
            uint8_t high = read_mem(0x0100 + cpu->sp);
            uint16_t return_addr = (high << 8) | low;
            (void)read_mem(return_addr);
            
            // Return address on stack is the last byte of JSR, so add 1 to skip past it
            cpu->pc = return_addr + 1;
        }
        break;
        case 0x40: // RTI - Return from Interrupt
        {
            dummy_read_next(cpu);              // RTI dummy read
            (void)read_mem(0x0100 + cpu->sp);
            // Pull P (SP increments before each pull)
            cpu->sp++;
            uint8_t newP = read_mem(0x0100 + cpu->sp);
            cpu->status = (newP & ~BREAK_FLAG) | UNUSED_FLAG;

            // RTI restores the saved PC directly; unlike RTS, it does not add one.
            cpu->sp++;
            uint8_t pcl = read_mem(0x0100 + cpu->sp);
            cpu->sp++;
            uint8_t pch = read_mem(0x0100 + cpu->sp);
            cpu->pc = ((uint16_t)pch << 8) | pcl;
            NMI_BRK_LOG("RTI to %04X newP=%02X", cpu->pc, cpu->status);
        }
        break;

        // Stack operations.
        case 0x48: // PHA - Push Accumulator
            dummy_read_next(cpu);  // Add dummy read for 1-byte PHA
            write_mem(0x0100 + cpu->sp, cpu->a);
            cpu->sp--;
            break;
        case 0x68: // PLA - Pull Accumulator
            dummy_read_next(cpu);  // Add dummy read for 1-byte PLA
            (void)read_mem(0x0100 + cpu->sp);
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
            (void)read_mem(0x0100 + cpu->sp);
            cpu->sp++;
            {
                uint8_t newP = read_mem(0x0100 + cpu->sp);
                cpu->status = (newP & ~BREAK_FLAG) | UNUSED_FLAG;
            }
            break;

        // Branches.
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

        // Status flags.
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
            cpu->status &= ~INTERRUPT_FLAG;
            break;
        case 0x78: // SEI - Set Interrupt Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte SEI
            cpu->status |= INTERRUPT_FLAG;
            break;
        case 0xB8: // CLV - Clear Overflow Flag
            dummy_read_next(cpu);  // Add dummy read for 1-byte CLV
            cpu->status &= ~OVERFLOW_FLAG;
            break;

        // Accumulator loads.
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

        // Accumulator stores.
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
            uint16_t a = get_absx_address(cpu);
            write_mem(a, cpu->a);
        } break;
        case 0x99: { // STA abs,Y
            uint16_t a = get_absy_address(cpu);
            write_mem(a, cpu->a);
        } break;   
        case 0x81: // STA (indirect,X)
            write_mem(get_indirect_x(cpu), cpu->a);
            break;
        case 0x91: { // STA (ind),Y
            uint16_t a = get_indirect_y(cpu);
            write_mem(a, cpu->a);
        } break;

        // X-register loads.
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

        // X-register stores.
        case 0x86: // STX Zero Page
            write_mem(get_zpg_address(cpu), cpu->x);
            break;
        case 0x96: // STX Zero Page,Y
            write_mem(get_zpgy_address(cpu), cpu->x);
            break;
        case 0x8E: // STX Absolute
            write_mem(get_abs_address(cpu), cpu->x);
            break;

        // Y-register loads.
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

        // Y-register stores.
        case 0x84: // STY Zero Page
            write_mem(get_zpg_address(cpu), cpu->y);
            break;
        case 0x94: // STY Zero Page,X
            write_mem(get_zpgx_address(cpu), cpu->y);
            break;
        case 0x8C: // STY Absolute
            write_mem(get_abs_address(cpu), cpu->y);
            break;

        // Register transfers.
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

        // Register increments and decrements.
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

        // AND operations.
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

        // ANC: AND the operand, then copy the negative flag into carry.
        case 0x0B: // ANC Immediate
        case 0x2B: // ANC Immediate (alternate opcode)
            cpu->a &= read_mem(cpu->pc++);
            set_zn_flags(cpu, cpu->a);
            if (cpu->a & 0x80) cpu->status |= CARRY_FLAG;
            else               cpu->status &= ~CARRY_FLAG;
            break;

        // ALR: AND the operand, then shift the accumulator right.
        case 0x4B: // ALR Immediate
            cpu->a &= read_mem(cpu->pc++);
            lsr(cpu, &cpu->a);
            break;

        case 0x8B: { // XAA/ANE Immediate
            // Silicon-dependent analog behavior is modeled with a fixed $EE mask.
            uint8_t imm = read_mem(cpu->pc++);
            cpu->a = (uint8_t)((cpu->a | 0xEEu) & cpu->x & imm);
            set_zn_flags(cpu, cpu->a);
            break;
        }

        // ARR: AND the operand, rotate right, then apply its unusual flag rules.
        case 0x6B: // ARR Immediate
            {
                uint8_t imm = read_mem(cpu->pc++);
                uint8_t carry_in = (cpu->status & CARRY_FLAG) ? 1 : 0;
                uint8_t tmp = cpu->a & imm;
                uint8_t result = (tmp >> 1) | (carry_in << 7);

                cpu->a = result;
                set_zn_flags(cpu, cpu->a);

                if (result & 0x40) cpu->status |= CARRY_FLAG;
                else               cpu->status &= ~CARRY_FLAG;

                if (((result >> 6) ^ (result >> 5)) & 0x01) cpu->status |= OVERFLOW_FLAG;
                else                                        cpu->status &= ~OVERFLOW_FLAG;
            }
            break;

        // AXS/SBX undocumented opcode.
        case 0xCB: // AXS/SBX Immediate: X = (A & X) - imm
            {
                uint8_t imm = read_mem(cpu->pc++);
                uint8_t and_ax = cpu->a & cpu->x;
                uint8_t result = (uint8_t)(and_ax - imm);

                if (and_ax >= imm) cpu->status |= CARRY_FLAG;
                else               cpu->status &= ~CARRY_FLAG;

                cpu->x = result;
                set_zn_flags(cpu, cpu->x);
            }
            break;

        // ORA operations.
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

        // EOR operations.
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

        // Addition.
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

        // Subtraction.
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

        // Comparisons.
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

        // BIT tests.
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

        // Left shifts.
        case 0x0A: // ASL A
            dummy_read_next(cpu);  // Add dummy read for 1-byte ASL A
            asl(cpu, &cpu->a);
            break;
        case 0x06: // ASL Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x16: // ASL Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x0E: // ASL Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x1E: // ASL Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
            }
            break;

        // Right shifts.
        case 0x4A: // LSR A
            dummy_read_next(cpu);  // Add dummy read for 1-byte LSR A
            lsr(cpu, &cpu->a);
            break;
        case 0x46: // LSR Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x56: // LSR Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x4E: // LSR Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x5E: // LSR Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
            }
            break;

        // Left rotates.
        case 0x2A: // ROL A
            dummy_read_next(cpu);  // Add dummy read for 1-byte ROL A
            rol(cpu, &cpu->a);
            break;
        case 0x26: // ROL Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x36: // ROL Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x2E: // ROL Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
            }
            break;
        case 0x3E: // ROL Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
            }
            break;

        // Right rotates.
        case 0x6A: // ROR A
            dummy_read_next(cpu);  // Add dummy read for 1-byte ROR A
            ror(cpu, &cpu->a);
            break;
        case 0x66: // ROR Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t value = rmw_fetch(addr);
                ror(cpu, &value);
                write_mem(addr, value);
            }
            break;
        case 0x76: // ROR Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t value = rmw_fetch(addr);
                ror(cpu, &value);
                write_mem(addr, value);
            }
            break;
        case 0x6E: // ROR Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t value = rmw_fetch(addr);
                ror(cpu, &value);
                write_mem(addr, value);
            }
            break;
        case 0x7E: // ROR Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t value = rmw_fetch(addr);
                ror(cpu, &value);
                write_mem(addr, value);
            }
            break;

        // Memory increments.
        case 0xE6: // INC Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) + 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;
        case 0xF6: // INC Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) + 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;
        case 0xEE: // INC Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) + 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;
        case 0xFE: // INC Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) + 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;

        // Memory decrements.
        case 0xC6: // DEC Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;
        case 0xD6: // DEC Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;
        case 0xCE: // DEC Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;
        case 0xDE: // DEC Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, val);
                set_zn_flags(cpu, val);
            }
            break;

        // Undocumented NOP variants.
        // NOP Zero Page
        case 0x04: case 0x44: case 0x64:
            {
                uint8_t zpg = read_mem(cpu->pc++);
                (void)read_mem(zpg);
            }
            break;
        // NOP Zero Page,X
        case 0x14: case 0x34: case 0x54: case 0x74: case 0xD4: case 0xF4:
            {
                uint8_t zpg = read_mem(cpu->pc++);
                (void)read_mem(zpg); // indexed addressing dummy read
                (void)read_mem((uint8_t)(zpg + cpu->x));
            }
            break;
        // NOP Absolute
        case 0x0C:
            {
                uint16_t addr = get_abs_address(cpu);
                (void)read_mem(addr);
            }
            break;
        // NOP Absolute,X
        case 0x1C: case 0x3C: case 0x5C: case 0x7C: case 0xDC: case 0xFC:
            (void)read_mem(get_absx_read(cpu));
            break;
        // Implied NOPs (1 byte) - Add dummy reads
        case 0x1A: case 0x3A: case 0x5A: case 0x7A: case 0xDA: case 0xFA:
            dummy_read_next(cpu);  // Add dummy read for undocumented implied NOPs
            break;
        // JAM: neither interrupts nor later instructions can restart the CPU.
        case 0x02: case 0x12: case 0x22: case 0x32:
        case 0x42: case 0x52: case 0x62: case 0x72:
        case 0x92: case 0xB2: case 0xD2: case 0xF2:
            dummy_read_next(cpu);
            cpu->pc--;
            cpu->halted = true;
            cpu_nmi_ready = cpu_irq_ready = false;
            break;
        // Immediate NOPs (consume operand)
        case 0x80: case 0x82: case 0x89: case 0xC2: case 0xE2:
            (void)read_mem(cpu->pc++);
            break;

        // DCP: decrement memory, then compare with the accumulator.
        case 0xC7: // DCP Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t mem = (uint8_t)(rmw_fetch(addr) - 1);
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
                uint8_t mem = (uint8_t)(rmw_fetch(addr) - 1);
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
                uint8_t mem = (uint8_t)(rmw_fetch(addr) - 1);
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
                uint8_t mem = (uint8_t)(rmw_fetch(addr) - 1);
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
                uint8_t mem = (uint8_t)(rmw_fetch(addr) - 1);
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
                uint8_t mem = (uint8_t)(rmw_fetch(addr) - 1);
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
                uint8_t mem = (uint8_t)(rmw_fetch(addr) - 1);
                write_mem(addr, mem);
                uint8_t result = cpu->a - mem;
                cpu->status = (cpu->status & ~(CARRY_FLAG|ZERO_FLAG|NEGATIVE_FLAG)) |
                             ((cpu->a >= mem) ? CARRY_FLAG : 0) |
                             ((result == 0) ? ZERO_FLAG : 0) |
                             (result & 0x80 ? NEGATIVE_FLAG : 0);
            }
            break;

        // RLA: rotate memory left, then AND it with the accumulator.
        case 0x27: // RLA Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x37: // RLA Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x2F: // RLA Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x3F: // RLA Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x3B: // RLA Absolute,Y
            {
                uint16_t addr = get_absy_address(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x23: // RLA (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x33: // RLA (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t val = rmw_fetch(addr);
                rol(cpu, &val);
                write_mem(addr, val);
                cpu->a &= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;

        // SLO: shift memory left, then OR it with the accumulator.
        case 0x07: // SLO Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x17: // SLO Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x0F: // SLO Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x1F: // SLO Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x1B: // SLO Absolute,Y
            {
                uint16_t addr = get_absy_address(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x03: // SLO (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x13: // SLO (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t val = rmw_fetch(addr);
                asl(cpu, &val);
                write_mem(addr, val);
                cpu->a |= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;

        // SRE: shift memory right, then XOR it with the accumulator.
        case 0x47: // SRE Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x57: // SRE Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x4F: // SRE Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x5F: // SRE Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x5B: // SRE Absolute,Y
            {
                uint16_t addr = get_absy_address(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x43: // SRE (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;
        case 0x53: // SRE (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t val = rmw_fetch(addr);
                lsr(cpu, &val);
                write_mem(addr, val);
                cpu->a ^= val;
                set_zn_flags(cpu, cpu->a);
            }
            break;

        // RRA: rotate memory right, then add it to the accumulator.
        case 0x67: // RRA Zero Page
            {
                uint16_t addr = get_zpg_address(cpu);
                uint8_t val = rmw_fetch(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x77: // RRA Zero Page,X
            {
                uint16_t addr = get_zpgx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x6F: // RRA Absolute
            {
                uint16_t addr = get_abs_address(cpu);
                uint8_t val = rmw_fetch(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x7F: // RRA Absolute,X
            {
                uint16_t addr = get_absx_address(cpu);
                uint8_t val = rmw_fetch(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x7B: // RRA Absolute,Y
            {
                uint16_t addr = get_absy_address(cpu);
                uint8_t val = rmw_fetch(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x63: // RRA (indirect,X)
            {
                uint16_t addr = get_indirect_x(cpu);
                uint8_t val = rmw_fetch(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;
        case 0x73: // RRA (indirect),Y
            {
                uint16_t addr = get_indirect_y(cpu);
                uint8_t val = rmw_fetch(addr);
                ror(cpu, &val);
                write_mem(addr, val);
                do_adc(cpu, val);
            }
            break;

        // LAX loads the same value into A and X.
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
                uint8_t val = read_mem(get_absy_read(cpu));
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
                uint8_t val = read_mem(get_indirect_y_read(cpu));
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

        // SAX stores A AND X.
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

        // ISC increments memory, then subtracts it from the accumulator.
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

        // LAS loads A, X, and the stack pointer from the same masked value.
        case 0xBB: // LAS Absolute,Y
            {
                uint8_t mem = read_mem(get_absy_read(cpu));
                uint8_t result = mem & cpu->sp;
                cpu->a = cpu->x = cpu->sp = result;
                set_zn_flags(cpu, result);
            }
            break;

        // Unstable SHX, SHY, SHA, and TAS stores.
        case 0x9F: // AHX/SHA Absolute,Y
            masked_indexed_store(get_abs_address(cpu), cpu->y, cpu->a & cpu->x);
            break;
        case 0x93: // AHX/SHA (indirect),Y
            {
                uint8_t zpg = read_mem(cpu->pc++);
                masked_indexed_store(read_zpg_word(zpg), cpu->y, cpu->a & cpu->x);
            }
            break;
        case 0x9E: // SHX Absolute,Y
            masked_indexed_store(get_abs_address(cpu), cpu->y, cpu->x);
            break;
        case 0x9C: // SHY Absolute,X
            masked_indexed_store(get_abs_address(cpu), cpu->x, cpu->y);
            break;
        case 0x9B: // TAS/SHS Absolute,Y
            masked_indexed_store(get_abs_address(cpu), cpu->y, cpu->a & cpu->x);
            cpu->sp = cpu->a & cpu->x;
            break;

        default:
            printf("Invalid opcode encountered: 0x%02X at PC: 0x%04X\n", opcode, cpu->pc);
            break;
    }
}

static uint8_t read_dma_bus(uint16_t address, uint16_t halted_address) {
    if ((halted_address & 0xFFE0u) != 0x4000) {
        // Internal CPU registers are decoded during DMA only when the halted CPU selected them.
        if (address >= 0x4015 && address <= 0x401A) return bus_get();
        return read_bus(address);
    }

    uint16_t internal = 0x4000 | (address & 0x1Fu);
    if (internal == 0x4015) {
        uint8_t value = read_bus(internal);
        if (address != internal) (void)read_bus_target(address, BUS_LATCH_EXTERNAL);
        return value;
    }
    if ((internal == 0x4016 || internal == 0x4017) && address != internal) {
        uint8_t controller = read_bus(internal);
        uint8_t external = read_bus_target(address, BUS_LATCH_EXTERNAL);
        uint8_t mask = joypad_open_bus_mask(internal - 0x4016u);
        bus_set_external((uint8_t)((external & mask) | (controller & ~mask)));
        return (uint8_t)((external & mask) | (external & controller & ~mask));
    }
    return read_bus(address);
}

static uint8_t finish_dma_bus_read(uint16_t address, uint16_t halted_address, uint8_t value) {
    if ((halted_address & 0xFFE0u) != 0x4000) {
        if (address >= 0x4015 && address <= 0x401A) return value;
        return finish_bus_read_target(address, BUS_LATCH_BOTH, value);
    }

    uint16_t internal = (uint16_t)(0x4000 | (address & 0x1Fu));
    if (internal == 0x4015) {
        return value;
    }
    if ((internal == 0x4016 || internal == 0x4017) && address != internal) {
        return value;
    }
    return finish_bus_read_target(address, BUS_LATCH_BOTH, value);
}

static void cancel_active_dmc_dma(bool *dmc, bool *need_halt, bool *need_dummy) {
    APU *active = apu_active_state();
    *dmc = false;
    *need_halt = false;
    *need_dummy = false;
    active->dmc.dma_pending = false;
    active->dmc.dma_halt_started = false;
    active->dmc.dma_abort_requested = false;
}

static void begin_dma_transfer_cycle(bool *dmc, bool *need_halt, bool *need_dummy) {
    APU *active = apu_active_state();
    if (*dmc && active->dmc.dma_abort_requested) {
        cancel_active_dmc_dma(dmc, need_halt, need_dummy);
    } else if (*dmc && *need_halt) {
        *need_halt = false;
        active->dmc.dma_halt_started = true;
    } else if (*dmc && *need_dummy) {
        *need_dummy = false;
    }
    begin_cpu_cycle(true);
}

static void process_pending_dma(uint16_t read_addr, bool opcode_fetch) {
    APU *active = apu_active_state();
    bool oam = cpu_oam_dma_pending;
    bool dmc = apu_dmc_dma_pending(active);
    if (!oam && !dmc) return;
    if (nes_timing()->region == NES_REGION_PAL && !opcode_fetch) return;
    uint16_t oam_base = (uint16_t)cpu_oam_dma_page << 8;
    cpu_oam_dma_pending = false;
    unsigned oam_offset = 0;
    bool oam_has_byte = false;
    uint8_t oam_byte = 0;
    bool dmc_need_halt = dmc;
    bool dmc_need_dummy = dmc;

    // Halt can succeed only on a CPU read; all earlier writes have already completed.
    if (dmc) {
        dmc_need_halt = false;
        active->dmc.dma_halt_started = true;
    }
    begin_cpu_cycle(true);
    uint8_t halt_value = read_bus(read_addr);
    end_cpu_cycle(true);
    (void)finish_bus_read_target(read_addr, BUS_LATCH_BOTH, halt_value);

    // A disable that matures during the halt cycle aborts before the dummy cycle.
    if (dmc && active->dmc.dma_abort_requested) {
        cancel_active_dmc_dma(&dmc, &dmc_need_halt, &dmc_need_dummy);
        if (!oam) return;
    }

    while (oam || dmc || apu_dmc_dma_pending(active)) {
        if (dmc && !apu_dmc_dma_pending(active) && !active->dmc.dma_abort_requested) {
            dmc = false;
            dmc_need_halt = false;
            dmc_need_dummy = false;
        } else if (!dmc && apu_dmc_dma_pending(active)) {
            // Pick up requests raised by a just-completed DMC transfer as well as
            // requests raised while OAM DMA is running. OAM cycles can supply the
            // new transfer's halt and dummy cycles.
            dmc = true;
            dmc_need_halt = true;
            dmc_need_dummy = true;
        }
        if (!oam && !dmc) break;
        bool get = (active_cpu_cycles & 1u) == 0;
        bool dmc_get = get && dmc && !dmc_need_halt && !dmc_need_dummy;
        bool oam_get = get && oam && !dmc_get;
        bool oam_put = !get && oam && oam_has_byte;

        if (dmc_get) {
            begin_dma_transfer_cycle(&dmc, &dmc_need_halt, &dmc_need_dummy);
            uint8_t value = read_dma_bus(apu_dmc_dma_address(active), read_addr);
            end_cpu_cycle(true);
            value = finish_dma_bus_read(apu_dmc_dma_address(active), read_addr, value);
            apu_dmc_dma_complete(active, value);
            dmc = false;
            dmc_need_halt = false;
            dmc_need_dummy = false;
        } else if (oam_get) {
            begin_dma_transfer_cycle(&dmc, &dmc_need_halt, &dmc_need_dummy);
            uint16_t oam_address = (uint16_t)(oam_base + oam_offset);
            oam_byte = read_dma_bus(oam_address, read_addr);
            oam_has_byte = true;
            end_cpu_cycle(true);
            oam_byte = finish_dma_bus_read(oam_address, read_addr, oam_byte);
        } else if (oam_put) {
            begin_dma_transfer_cycle(&dmc, &dmc_need_halt, &dmc_need_dummy);
            write_bus(0x2004, oam_byte);
            oam_has_byte = false;
            if (++oam_offset == 256) oam = false;
            end_cpu_cycle(true);
        } else {
            begin_dma_transfer_cycle(&dmc, &dmc_need_halt, &dmc_need_dummy);
            uint8_t dummy = read_bus(read_addr);
            end_cpu_cycle(true);
            (void)finish_bus_read_target(read_addr, BUS_LATCH_BOTH, dummy);
        }
    }
}

int cpu_step(CPU* cpu) {
    uint64_t start = active_cpu_cycles;
    running_cpu = cpu;
    if (cpu->halted) {
        (void)read_mem_cycle(cpu->pc, true);
    } else if (cpu_nmi_injected) {
        cpu_nmi_injected = false;
        cpu_irq(cpu);
    } else {
        uint8_t opcode = read_mem_cycle(cpu->pc++, true);
        execute(cpu, opcode);
        if (!cpu->halted && (cpu_irq_ready || cpu_nmi_ready)) cpu_irq(cpu);
    }
    running_cpu = NULL;
    return (int)(active_cpu_cycles - start);
}
