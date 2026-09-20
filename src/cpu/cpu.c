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
#include <string.h>
#include "cpu.h"
#include "cpu_observer.h"
#include "../ppu/ppu.h"
#include "../apu/apu.h"
#include "../apu/epsm.h"
#include "../rom/mapper.h"
#include "../joypad/joypad.h"
#include "../system/hardware.h"
#include "../system/execution_policy.h"
#include "../system/timing.h"
#include "../system/vs_system.h"
#include "../debugger/debugger.h"
#include "../cheats/cheats.h"

uint8_t ram[0x0800];        // 2KB internal RAM
#define APU_IO_SIZE 0x20              // cover $4000-$401F
uint8_t apu_io[APU_IO_SIZE];          // 32 bytes
#define APU_IDX(a) ((uint16_t)((a) - 0x4000))
uint64_t cpu_total_cycles = 0;
static CpuRuntimeState main_runtime = {.ppu_master_phase = 3};
static CpuRuntimeState *cpu_runtime = &main_runtime;
static uint8_t *cpu_ram = ram;
static uint64_t *cpu_cycles = &cpu_total_cycles;
static CpuStartupAlignmentMode alignment_mode;
static CpuStartupAlignment configured_alignment;
static uint32_t alignment_random_state;
static bool cpu_test_mode;

void cpu_set_test_mode(bool enabled) {
    if (!nes_execution_allows_host_configuration()) return;
    cpu_test_mode = enabled;
}
bool cpu_test_mode_enabled(void) { return cpu_test_mode; }

void cpu_use_default_startup_alignment(void) {
    if (!nes_execution_allows_host_configuration()) return;
    alignment_mode = CPU_STARTUP_ALIGNMENT_DEFAULT;
}

bool cpu_set_startup_alignment(unsigned cpu_offset, unsigned ppu_phase) {
    const NesTiming *timing = nes_timing();
    if (cpu_offset >= timing->cpu_divider || ppu_phase >= timing->ppu_divider)
        return false;
    if (!nes_execution_allows_host_configuration()) return false;
    configured_alignment = (CpuStartupAlignment){(uint8_t)cpu_offset, (uint8_t)ppu_phase};
    alignment_mode = CPU_STARTUP_ALIGNMENT_EXPLICIT;
    return true;
}

void cpu_seed_startup_alignment(uint32_t seed) {
    if (!nes_execution_allows_host_configuration()) return;
    alignment_random_state = seed;
    alignment_mode = CPU_STARTUP_ALIGNMENT_SEEDED;
}

bool cpu_startup_alignment_valid(NesRegion region) {
    const NesTiming *timing = nes_timing_for_region(region);
    return alignment_mode != CPU_STARTUP_ALIGNMENT_EXPLICIT
        || (configured_alignment.cpu_offset < timing->cpu_divider
            && configured_alignment.ppu_phase < timing->ppu_divider);
}

CpuStartupAlignment cpu_get_startup_alignment(void) {
    return cpu_runtime->startup_alignment;
}

CpuStartupAlignmentMode cpu_get_startup_alignment_mode(void) {
    return alignment_mode;
}

CpuStartupAlignment cpu_get_configured_startup_alignment(void) {
    return configured_alignment;
}

uint32_t cpu_get_startup_alignment_seed(void) {
    return alignment_random_state;
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

uint8_t cpu_peek_internal_ram(uint16_t addr) {
    uint8_t *expanded = cart_cpu_ram_8k();
    return expanded ? expanded[addr & 0x1FFF] : cpu_ram[addr & 0x07FF];
}

uint8_t cpu_debug_peek(uint16_t addr) {
    if (addr <= 0x1FFF) return cpu_peek_internal_ram(addr);
    if (addr <= 0x3FFF) return ppu_debug_peek_register((uint16_t)(0x2000 | (addr & 7u)));
    if (addr >= 0x4000 && addr <= 0x4017) {
        if (addr == 0x4015) {
            uint8_t status = apu_debug_peek_status();
            return (uint8_t)((status & 0xDFu) | (bus_get_internal() & 0x20u));
        }
        if (addr == 0x4016 || addr == 0x4017) {
            unsigned port = addr - 0x4016u;
            uint8_t raw = vs_enabled() ? vs_debug_peek_controller_port(port)
                                       : joypad_debug_peek_port(port ? &pad2 : &pad1, port);
            uint8_t mask = vs_enabled() ? 0u : joypad_open_bus_mask(port);
            return (uint8_t)((bus_get() & mask) | (raw & (uint8_t)~mask));
        }
        return bus_get();
    }
    if (cpu_test_mode && addr >= 0x4018 && addr <= 0x401A)
        return apu_read_test_output(addr);
    if (addr >= 0x4020) return cart_cpu_peek_bus(addr, bus_get());
    return bus_get();
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
    cpu_irq_polled = ((!cart_nsf_active() && apu_irq_pending(apu_active_state())) || cart_irq_pending()
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

    /* Keep NMI edges clocked during reset, after discarding earlier requests. */
    cpu_nmi_pending = false;
    cpu_nmi_previous_line = cpu_nmi_line;
    cpu_nmi_ready = cpu_nmi_injected = false;

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
    cpu_irq_polled = cpu_irq_ready = false;
    cpu_oam_dma_pending = false;
    joypad_read_valid = false;
    joypad_write_pending = 0;
}

bool cpu_power_on(CPU* cpu) {
    if (!cpu || !cpu_startup_alignment_valid(nes_timing()->region)) return false;
    cart_console_reset(false);
    cart_irq_ack();
    vs_clear_external_irq();
    uint8_t *expanded = cart_cpu_ram_8k();
    nes_initialize_power_on_ram(expanded ? expanded : cpu_ram, expanded ? 0x2000 : 0x0800, 0x00);
    CpuStartupAlignment alignment = {0, (uint8_t)(nes_timing()->ppu_divider - 1)};
    if (alignment_mode == CPU_STARTUP_ALIGNMENT_EXPLICIT) {
        alignment = configured_alignment;
    } else if (alignment_mode == CPU_STARTUP_ALIGNMENT_SEEDED) {
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
    cart_after_console_reset();
    return true;
}

void cpu_soft_reset(CPU* cpu) {
    cart_console_reset(true);
    cart_irq_ack();
    vs_clear_external_irq();
    epsm_clear_irq_source();
    cpu->status = (cpu->status | INTERRUPT_FLAG | UNUSED_FLAG) & ~BREAK_FLAG;
    cpu_reset_sequence(cpu);
    cart_after_console_reset();
}

void cpu_reset(CPU* cpu) {
    cpu_power_on(cpu);
}

void cpu_clear_internal_ram(void) { memset(cpu_ram, 0, 0x0800); }

static inline void bus_latch(BusLatchTarget target, uint8_t value) {
    if (target != BUS_LATCH_EXTERNAL) bus_set_internal(value);
    if (target != BUS_LATCH_INTERNAL) bus_set_external(value);
}

static uint8_t read_bus_target(uint16_t addr, BusLatchTarget target) {
    if (addr <= 0x1FFF) {
        uint8_t v = cpu_peek_internal_ram(addr);
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
        uint8_t cartridge_value;
        if (addr == 0x4011 && cart_read_cpu_register(addr, &cartridge_value)) {
            bus_latch(target, cartridge_value);
            return cartridge_value;
        }
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
    value = cheats_apply_read(addr, value);
    debugger_on_cpu_read(addr, &value);
    return value;
}

static void write_bus_raw(uint16_t addr, uint8_t value) {
    uint8_t previous_bus = bus_get();
    bus_set(value); // writes still put value on the CPU bus latch

    if (addr <= 0x1FFF) {
        uint8_t *expanded = cart_cpu_ram_8k();
        if (expanded) expanded[addr] = value;
        else cpu_ram[addr & 0x07FF] = value;
        return;
    }

    if (addr >= 0x2000 && addr <= 0x3FFF) {
        // Cartridge address decoding sees the CPU address before PPU mirroring.
        cart_observe_cpu_write(addr, value);
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
    if (addr >= 0x4020) {
        cart_cpu_write(addr, value);
        apu_audio_refresh(apu_active_state());
    }
}

static void write_bus(uint16_t addr, uint8_t value) {
    write_bus_raw(addr, value);
    debugger_on_cpu_write(addr, value);
    nes_cpu_write_observe(addr, value);
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
#include "cpu_opcodes_primary.h"
#include "cpu_opcodes_secondary.h"
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
    if (!debugger_before_instruction(cpu)) {
        running_cpu = NULL;
        return 0;
    }
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

#undef ppu_master_phase
#undef joypad_read_valid
#undef joypad_read_addr
#undef joypad_read_cycle
#undef joypad_read_value
#undef joypad_write_pending
#undef joypad_write_value

typedef struct {
    CPU cpu;
    uint8_t ram[0x0800];
    uint8_t apu_io[APU_IO_SIZE];
    uint64_t total_cycles;
    CpuRuntimeState runtime;
    uint8_t alignment_mode;
    CpuStartupAlignment configured_alignment;
    uint32_t alignment_random_state;
    bool test_mode;
} CpuSavedState;

static bool cpu_state_write_cpu(NesStateWriter *writer, const CPU *state) {
    return nes_state_write_u8(writer, state->a)
        && nes_state_write_u8(writer, state->x)
        && nes_state_write_u8(writer, state->y)
        && nes_state_write_u16(writer, state->pc)
        && nes_state_write_u8(writer, state->sp)
        && nes_state_write_u8(writer, state->status)
        && nes_state_write_bool(writer, state->halted);
}

static bool cpu_state_read_cpu(NesStateReader *reader, CPU *state) {
    return nes_state_read_u8(reader, &state->a)
        && nes_state_read_u8(reader, &state->x)
        && nes_state_read_u8(reader, &state->y)
        && nes_state_read_u16(reader, &state->pc)
        && nes_state_read_u8(reader, &state->sp)
        && nes_state_read_u8(reader, &state->status)
        && nes_state_read_bool(reader, &state->halted);
}

static bool cpu_state_write_runtime(NesStateWriter *writer, const CpuRuntimeState *state) {
    return nes_state_write_u8(writer, state->external_bus)
        && nes_state_write_u8(writer, state->internal_bus)
        && nes_state_write_bool(writer, state->nmi_pending)
        && nes_state_write_bool(writer, state->nmi_line)
        && nes_state_write_bool(writer, state->nmi_previous_line)
        && nes_state_write_bool(writer, state->nmi_ready)
        && nes_state_write_bool(writer, state->nmi_injected)
        && nes_state_write_bool(writer, state->irq_polled)
        && nes_state_write_bool(writer, state->irq_ready)
        && nes_state_write_bool(writer, state->oam_dma_pending)
        && nes_state_write_u8(writer, state->oam_dma_page)
        && nes_state_write_u8(writer, state->ppu_master_phase)
        && nes_state_write_u8(writer, state->startup_alignment.cpu_offset)
        && nes_state_write_u8(writer, state->startup_alignment.ppu_phase)
        && nes_state_write_bool(writer, state->joypad_read_valid)
        && nes_state_write_u16(writer, state->joypad_read_addr)
        && nes_state_write_u64(writer, state->joypad_read_cycle)
        && nes_state_write_u8(writer, state->joypad_read_value)
        && nes_state_write_u8(writer, state->joypad_write_pending)
        && nes_state_write_u8(writer, state->joypad_write_value);
}

static bool cpu_state_read_runtime(NesStateReader *reader, CpuRuntimeState *state) {
    return nes_state_read_u8(reader, &state->external_bus)
        && nes_state_read_u8(reader, &state->internal_bus)
        && nes_state_read_bool(reader, &state->nmi_pending)
        && nes_state_read_bool(reader, &state->nmi_line)
        && nes_state_read_bool(reader, &state->nmi_previous_line)
        && nes_state_read_bool(reader, &state->nmi_ready)
        && nes_state_read_bool(reader, &state->nmi_injected)
        && nes_state_read_bool(reader, &state->irq_polled)
        && nes_state_read_bool(reader, &state->irq_ready)
        && nes_state_read_bool(reader, &state->oam_dma_pending)
        && nes_state_read_u8(reader, &state->oam_dma_page)
        && nes_state_read_u8(reader, &state->ppu_master_phase)
        && nes_state_read_u8(reader, &state->startup_alignment.cpu_offset)
        && nes_state_read_u8(reader, &state->startup_alignment.ppu_phase)
        && nes_state_read_bool(reader, &state->joypad_read_valid)
        && nes_state_read_u16(reader, &state->joypad_read_addr)
        && nes_state_read_u64(reader, &state->joypad_read_cycle)
        && nes_state_read_u8(reader, &state->joypad_read_value)
        && nes_state_read_u8(reader, &state->joypad_write_pending)
        && nes_state_read_u8(reader, &state->joypad_write_value);
}

bool cpu_machine_state_capture(NesStateWriter *writer, const CpuMachineContext *context) {
    if (!writer || !context || running_cpu || in_bus_cycle) return false;
    return cpu_state_write_cpu(writer, &context->cpu)
        && nes_state_write_bytes(writer, context->ram, sizeof(context->ram))
        && nes_state_write_u64(writer, context->total_cycles)
        && cpu_state_write_runtime(writer, &context->runtime);
}

bool cpu_machine_state_decode(NesStateReader *reader, CpuMachineContext *context) {
    if (!reader || !context) return false;
    memset(context, 0, sizeof(*context));
    if (!cpu_state_read_cpu(reader, &context->cpu)
        || !nes_state_read_bytes(reader, context->ram, sizeof(context->ram))
        || !nes_state_read_u64(reader, &context->total_cycles)
        || !cpu_state_read_runtime(reader, &context->runtime)
        || context->runtime.ppu_master_phase >= nes_timing()->ppu_divider) return false;
    return nes_state_reader_remaining(reader) == 0;
}

static bool cpu_state_decode(NesStateReader *reader, CpuSavedState *saved) {
    memset(saved, 0, sizeof(*saved));
    if (!cpu_state_read_cpu(reader, &saved->cpu)
        || !nes_state_read_bytes(reader, saved->ram, sizeof(saved->ram))
        || !nes_state_read_bytes(reader, saved->apu_io, sizeof(saved->apu_io))
        || !nes_state_read_u64(reader, &saved->total_cycles)
        || !cpu_state_read_runtime(reader, &saved->runtime)
        || !nes_state_read_u8(reader, &saved->alignment_mode)
        || !nes_state_read_u8(reader, &saved->configured_alignment.cpu_offset)
        || !nes_state_read_u8(reader, &saved->configured_alignment.ppu_phase)
        || !nes_state_read_u32(reader, &saved->alignment_random_state)
        || !nes_state_read_bool(reader, &saved->test_mode)) return false;
    if (saved->alignment_mode > CPU_STARTUP_ALIGNMENT_SEEDED) return false;
    if (saved->runtime.ppu_master_phase >= nes_timing()->ppu_divider) return false;
    return nes_state_reader_remaining(reader) == 0;
}

bool cpu_state_capture(NesStateWriter *writer) {
    if (!writer || running_cpu || in_bus_cycle) return false;
    return cpu_state_write_cpu(writer, &cpu)
        && nes_state_write_bytes(writer, ram, sizeof(ram))
        && nes_state_write_bytes(writer, apu_io, sizeof(apu_io))
        && nes_state_write_u64(writer, cpu_total_cycles)
        && cpu_state_write_runtime(writer, &main_runtime)
        && nes_state_write_u8(writer, (uint8_t)alignment_mode)
        && nes_state_write_u8(writer, configured_alignment.cpu_offset)
        && nes_state_write_u8(writer, configured_alignment.ppu_phase)
        && nes_state_write_u32(writer, alignment_random_state)
        && nes_state_write_bool(writer, cpu_test_mode);
}

bool cpu_state_validate(NesStateReader *reader) {
    CpuSavedState saved;
    return reader && cpu_state_decode(reader, &saved);
}

bool cpu_state_apply(NesStateReader *reader) {
    CpuSavedState saved;
    if (!reader || !cpu_state_decode(reader, &saved)) return false;
    cpu_select_machine(NULL);
    cpu = saved.cpu;
    memcpy(ram, saved.ram, sizeof(ram));
    memcpy(apu_io, saved.apu_io, sizeof(apu_io));
    cpu_total_cycles = saved.total_cycles;
    main_runtime = saved.runtime;
    alignment_mode = saved.alignment_mode;
    configured_alignment = saved.configured_alignment;
    alignment_random_state = saved.alignment_random_state;
    cpu_test_mode = saved.test_mode;
    running_cpu = NULL;
    in_bus_cycle = false;
    return true;
}
