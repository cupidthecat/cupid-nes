/*
 * cpu_accuracy.c - CPU and controller hardware regression tests
 *
 * Author: @frankischilling
 *
 * This file tests CPU bus ordering, interrupt timing, DMA behavior, reset sequencing,
 * controller reads, open bus behavior, official instructions, and undocumented opcodes.
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
#include <stdio.h>
#include <string.h>
#include "../cpu/cpu.h"
#include "../apu/apu.h"
#include "../ppu/ppu.h"
#include "../rom/mapper.h"
#include "../system/timing.h"

extern uint8_t ram[0x0800];

typedef struct {
    uint16_t addr;
    uint8_t value;
    bool write;
    uint64_t cycle;
    uint64_t ppu_cycle;
    uint32_t apu_cycle;
} BusEvent;

static uint8_t fixture_memory[0x10000];
static BusEvent bus_events[1024];
static size_t bus_count;
static uint64_t mapper_clocks;
static uint64_t nmi_assert_cycle, nmi_clear_cycle, irq_assert_cycle, dma_request_cycle;

static void prepare_dmc(uint16_t address);

static void raise_frame_irq(void) {
    apu.frame_irq = true;
    apu.frame_irq_source = true;
}

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static void record_bus(uint16_t addr, uint8_t value, bool write) {
    if (bus_count < sizeof(bus_events) / sizeof(bus_events[0])) {
        bus_events[bus_count] = (BusEvent){addr, value, write, cpu_get_bus_cycle(),
                                         ppu.total_cycles, apu.cycle_in_seq};
    }
    bus_count++;
}

static uint8_t fixture_read(uint16_t addr) {
    uint8_t value = fixture_memory[addr];
    record_bus(addr, value, false);
    return value;
}

static void fixture_write(uint16_t addr, uint8_t value) {
    record_bus(addr, value, true);
    fixture_memory[addr] = value;
}

static uint8_t fixture_ppu_read(uint16_t addr) {
    return fixture_memory[addr & 0x1FFFu];
}

static void fixture_ppu_write(uint16_t addr, uint8_t value) {
    fixture_memory[addr & 0x1FFFu] = value;
}

static void fixture_clock(int cycles) {
    mapper_clocks += (unsigned)cycles;
    if (cpu_total_cycles == nmi_assert_cycle) cpu_set_nmi_line(true);
    if (cpu_total_cycles == nmi_clear_cycle) cpu_set_nmi_line(false);
    if (cpu_total_cycles == irq_assert_cycle) raise_frame_irq();
    if (cpu_total_cycles == dma_request_cycle) {
        apu.dmc.start_delay = 0;
        apu.dmc.sample_buffer_empty = true;
        apu.dmc.dma_pending = true;
    }
}

static Mapper fixture_mapper = {
    .cpu_read = fixture_read,
    .cpu_write = fixture_write,
    .ppu_read = fixture_ppu_read,
    .ppu_write = fixture_ppu_write,
    .clock = fixture_clock,
};

static void reset_fixture_region(NesRegion region) {
    memset(fixture_memory, 0, sizeof(fixture_memory));
    memset(ram, 0, 0x0800);
    memset(&cpu, 0, sizeof(cpu));
    memset(&pad1, 0, sizeof(pad1));
    memset(&pad2, 0, sizeof(pad2));
    memset(&ines_header, 0, sizeof(ines_header));
    cart = &fixture_mapper;
    cart_irq_ack();
    mapper_clocks = 0;
    nmi_assert_cycle = nmi_clear_cycle = irq_assert_cycle = dma_request_cycle = 0;
    nes_set_region(region);
    cpu_total_cycles = 0;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    fixture_memory[0xFFFB] = 0x90;
    fixture_memory[0xFFFD] = 0x80;
    fixture_memory[0xFFFF] = 0xA0;
    cpu_power_on(&cpu);

    // Instruction-level regressions use a zero-based timeline after separately
    // validating the physical power-on sequence.
    cpu_total_cycles = 0;
    mapper_clocks = 0;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    apu.frame_reset_pending = false;
    apu.frame_reset_delay = 0;
    apu.cycle_in_seq = 0;
    bus_count = 0;
}

static void reset_fixture(void) {
    reset_fixture_region(NES_REGION_NTSC);
}

static void program(uint8_t op, uint8_t lo, uint8_t hi) {
    fixture_memory[0x8000] = op;
    fixture_memory[0x8001] = lo;
    fixture_memory[0x8002] = hi;
}

static int check_reads(const uint16_t *addresses, size_t count) {
    CHECK(bus_count == count);
    for (size_t i = 0; i < count; ++i) {
        CHECK(bus_events[i].addr == addresses[i]);
        CHECK(!bus_events[i].write);
    }
    return 0;
}

static int controller_latching(void) {
    Joypad jp = {0};
    joypad_write_strobe(&jp, 1);
    CHECK(joypad_read(&jp) == 0);
    joypad_set(&jp, BTN_A, 1);
    CHECK(joypad_read(&jp) == 1);
    CHECK(joypad_read(&jp) == 1);
    joypad_set(&jp, BTN_A, 0);
    CHECK(joypad_read(&jp) == 0);

    jp.buttons = 0xA5;
    joypad_write_strobe(&jp, 0);
    jp.buttons = 0x5A;
    for (unsigned bit = 0; bit < 8; ++bit) {
        joypad_write_strobe(&jp, 0); // No new falling edge: retain the snapshot.
        CHECK(joypad_read(&jp) == ((0xA5u >> bit) & 1u));
    }
    for (unsigned bit = 0; bit < 8; ++bit) CHECK(joypad_read(&jp) == 1);
    joypad_write_strobe(&jp, 1);
    joypad_write_strobe(&jp, 0);
    CHECK(joypad_read(&jp) == 0);
    CHECK(joypad_read(&jp) == 1);

    reset_fixture();
    pad1.buttons = 1;
    write_mem(0x4016, 1);
    write_mem(0x4016, 0);
    write_mem(0x4018, 0xE0);
    CHECK(read_mem(0x4016) == 0xE1);
    CHECK(read_mem(0x4017) == 0xE0);

    reset_fixture();
    pad1.buttons = 0x02;
    write_mem(0x4016, 1);
    write_mem(0x4016, 0);
    program(0x1E, 0x16, 0x40); // ASL $4016,X reads the same port on adjacent cycles.
    CHECK(cpu_step(&cpu) == 7);
    CHECK(joypad_read(&pad1) == 1); // B remains next: the adjacent reads shifted only A.

    reset_fixture();
    pad1.buttons = 0x02;
    write_mem(0x4016, 1);
    write_mem(0x4016, 0);
    program(0xAD, 0x16, 0x40);
    memcpy(&fixture_memory[0x8003], &fixture_memory[0x8000], 3);
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x40);
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x41); // Separate reads still advance the pad.

    // $4016 OUT changes on PUT boundaries. The first DEC alignment applies its
    // temporary high write; shifting the instruction by one CPU cycle suppresses it.
    for (unsigned phase = 0; phase < 2; ++phase) {
        reset_fixture();
        cpu_total_cycles = phase;
        pad1.buttons = 0;
        pad1.shift = 0xFF;
        pad1.strobe = 0;
        program(0xCE, 0x16, 0x40); // DEC $4016 writes $41 then $40.
        fixture_memory[0x8003] = 0xEA;
        CHECK(cpu_step(&cpu) == 6);
        CHECK(cpu_step(&cpu) == 2); // Let the final queued OUT value reach the pins.
        CHECK(pad1.strobe == 0);
        CHECK(pad1.shift == (phase ? 0xFF : 0x00));
    }
    return 0;
}

static int open_bus_and_cart_decoding(void) {
    reset_fixture();
    write_mem(0x4018, 0xA5);
    raise_frame_irq();
    CHECK(read_mem(0x4015) == 0x60);
    CHECK(apu.frame_irq && !apu.frame_irq_source && apu.frame_irq_clear_delay == 1);
    apu_step(&apu, 1);
    CHECK(!apu.frame_irq);
    CHECK(read_mem(0x4018) == 0xA5);
    // Floating reads copy the external bus back onto the internal CPU data bus.
    CHECK(read_mem(0x4015) == 0x20);

    cart = NULL;
    fixture_memory[0x6000] = 0x42;
    CHECK(read_mem(0x4020) == 0xA5);
    CHECK(read_mem(0x6000) == 0xA5);
    write_mem(0x6000, 0x55);
    CHECK(fixture_memory[0x6000] == 0x42);
    CHECK(bus_count == 0);
    cart = &fixture_mapper;
    static const uint16_t addresses[] = {0x4020, 0x40FF, 0x4100, 0x5FFF, 0x6000, 0xFFFF};
    for (size_t i = 0; i < sizeof(addresses) / sizeof(addresses[0]); ++i) {
        fixture_memory[addresses[i]] = 0xFF;
        write_mem(0x4018, 0xA5);
        CHECK(read_mem(addresses[i]) == 0xFF); // A driven $FF is data, never an unmapped sentinel.
        CHECK(read_mem(0x4018) == 0xFF);
    }
    fixture_memory[0x401F] = 0x73;
    size_t count = bus_count;
    CHECK(read_mem(0x401F) == 0xFF && bus_count == count);
    ines_header.flags7 = 0x08;
    // Cartridge callbacks own RAM and register decoding instead of CPU header guesses.
    ines_header.flags10 = 0;
    write_mem(0x6000, 0x99);
    CHECK(read_mem(0x6000) == 0x99);
    program(0xAD, 0x20, 0x40);
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0xFF);
    return 0;
}

static int nop_and_zero_page_cycles(void) {
    static const uint8_t immediate_nops[] = {0x80, 0x82, 0x89, 0xC2, 0xE2};
    for (size_t i = 0; i < sizeof(immediate_nops); ++i) {
        reset_fixture();
        program(immediate_nops[i], 0xA5, 0);
        CHECK(cpu_step(&cpu) == 2);
        CHECK(cpu.pc == 0x8002);
        CHECK(read_mem(0x4018) == 0xA5);
        const uint16_t reads[] = {0x8000, 0x8001};
        CHECK(check_reads(reads, 2) == 0);
        CHECK(bus_events[0].cycle == 1 && bus_events[1].cycle == 2);
    }
    reset_fixture();
    program(0xD4, 0xFF, 0);
    cpu.x = 2;
    ram[1] = 0x73;
    CHECK(cpu_step(&cpu) == 4);
    CHECK(read_mem(0x4018) == 0x73);
    reset_fixture();
    program(0x97, 0xFF, 0);
    cpu.a = 0xF3;
    cpu.x = 0xAF;
    cpu.y = 2;
    CHECK(cpu_step(&cpu) == 4);
    CHECK(ram[1] == 0xA3);
    return 0;
}

static int xaa_immediate(void) {
    static const uint8_t accumulators[] = {0, 0x11, 0x80, 0xFF};
    static const uint8_t indexes[] = {0, 0x0F, 0xF0, 0xFF};
    const uint8_t preserved = UNUSED_FLAG | INTERRUPT_FLAG | DECIMAL_FLAG | CARRY_FLAG | OVERFLOW_FLAG;
    reset_fixture();
    program(0x8B, 0, 0);
    for (size_t a = 0; a < sizeof(accumulators); ++a) {
        for (size_t x = 0; x < sizeof(indexes); ++x) {
            for (unsigned imm = 0; imm < 256; ++imm) {
                cpu.a = accumulators[a];
                cpu.x = indexes[x];
                cpu.y = 0xA6;
                cpu.sp = 0xF0;
                cpu.pc = 0x8000;
                cpu.status = preserved | ZERO_FLAG | NEGATIVE_FLAG;
                fixture_memory[0x8001] = (uint8_t)imm;
                bus_count = 0;
                CHECK(cpu_step(&cpu) == 2);
                // The selected silicon model preserves A's bits 0 and 4 through the OR mask.
                uint8_t expected = (uint8_t)((accumulators[a] | 0xEEu) & indexes[x] & imm);
                uint8_t flags = preserved | (expected == 0 ? ZERO_FLAG : 0) | (expected & NEGATIVE_FLAG);
                CHECK(cpu.a == expected && cpu.status == flags);
                CHECK(cpu.x == indexes[x] && cpu.y == 0xA6 && cpu.sp == 0xF0);
                CHECK(cpu.pc == 0x8002);
                CHECK(bus_count == 2 && bus_events[1].addr == 0x8001);
                CHECK(bus_events[1].value == imm);
            }
        }
    }
    return 0;
}

static int unofficial_immediate_semantics(void) {
    static const struct {
        uint8_t op, a, x, imm, carry, result_a, result_x, flags;
    } cases[] = {
        {0x0B, 0xFF, 0x55, 0x80, 0, 0x80, 0x55, NEGATIVE_FLAG | CARRY_FLAG | OVERFLOW_FLAG},
        {0x0B, 0x7F, 0x55, 0x80, 1, 0x00, 0x55, ZERO_FLAG | OVERFLOW_FLAG},
        {0x2B, 0xA5, 0x55, 0xF0, 0, 0xA0, 0x55, NEGATIVE_FLAG | CARRY_FLAG | OVERFLOW_FLAG},
        {0x2B, 0x7F, 0x55, 0xFF, 1, 0x7F, 0x55, OVERFLOW_FLAG},
        {0x4B, 0xFF, 0x55, 0x03, 0, 0x01, 0x55, CARRY_FLAG | OVERFLOW_FLAG},
        {0x4B, 0xFF, 0x55, 0x02, 1, 0x01, 0x55, OVERFLOW_FLAG},
        {0x4B, 0x80, 0x55, 0xFF, 1, 0x40, 0x55, OVERFLOW_FLAG},
        {0x4B, 0x01, 0x55, 0xFE, 1, 0x00, 0x55, ZERO_FLAG | OVERFLOW_FLAG},
        {0x6B, 0xFF, 0x55, 0xFF, 0, 0x7F, 0x55, CARRY_FLAG},
        {0x6B, 0xFF, 0x55, 0xFF, 1, 0xFF, 0x55, CARRY_FLAG | NEGATIVE_FLAG},
        {0x6B, 0x80, 0x55, 0xFF, 0, 0x40, 0x55, CARRY_FLAG | OVERFLOW_FLAG},
        {0x6B, 0x40, 0x55, 0xFF, 0, 0x20, 0x55, OVERFLOW_FLAG},
        {0x6B, 0x00, 0x55, 0xFF, 0, 0x00, 0x55, ZERO_FLAG},
        {0x6B, 0x00, 0x55, 0xFF, 1, 0x80, 0x55, NEGATIVE_FLAG},
        {0xAB, 0x80, 0x11, 0x00, 1, 0x00, 0x00, ZERO_FLAG | CARRY_FLAG | OVERFLOW_FLAG},
        {0xAB, 0x01, 0x11, 0x80, 0, 0x80, 0x80, NEGATIVE_FLAG | OVERFLOW_FLAG},
        {0xAB, 0x00, 0x80, 0x11, 1, 0x11, 0x11, CARRY_FLAG | OVERFLOW_FLAG},
        {0xCB, 0xFF, 0x10, 0x01, 0, 0xFF, 0x0F, CARRY_FLAG | OVERFLOW_FLAG},
        {0xCB, 0xFF, 0x10, 0x20, 1, 0xFF, 0xF0, NEGATIVE_FLAG | OVERFLOW_FLAG},
        {0xCB, 0x0F, 0xF0, 0x00, 0, 0x0F, 0x00, ZERO_FLAG | CARRY_FLAG | OVERFLOW_FLAG},
        {0xCB, 0x0F, 0x0A, 0x0B, 1, 0x0F, 0xFF, NEGATIVE_FLAG | OVERFLOW_FLAG},
        {0xCB, 0xFF, 0xFF, 0x80, 0, 0xFF, 0x7F, CARRY_FLAG | OVERFLOW_FLAG},
    };
    const uint8_t preserved = UNUSED_FLAG | INTERRUPT_FLAG | DECIMAL_FLAG;
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset_fixture();
        program(cases[i].op, cases[i].imm, 0);
        cpu.a = cases[i].a;
        cpu.x = cases[i].x;
        cpu.y = 0x5A;
        cpu.status = preserved | OVERFLOW_FLAG | NEGATIVE_FLAG | ZERO_FLAG |
                     (cases[i].carry ? CARRY_FLAG : 0);
        CHECK(cpu_step(&cpu) == 2);
        CHECK(cpu.a == cases[i].result_a && cpu.x == cases[i].result_x);
        CHECK(cpu.status == (preserved | cases[i].flags));
        CHECK(cpu.pc == 0x8002 && cpu.y == 0x5A && cpu.sp == 0xFD);
        CHECK(bus_count == 2 && bus_events[1].addr == 0x8001 && bus_events[1].value == cases[i].imm);
    }
    return 0;
}

static int indexed_read_penalties(void) {
    static const uint8_t abs_ops[] = {0xBF, 0xBB, 0x1C, 0x3C, 0x5C, 0x7C, 0xDC, 0xFC};
    for (size_t i = 0; i < sizeof(abs_ops); ++i) {
        for (unsigned cross = 0; cross < 2; ++cross) {
            reset_fixture();
            program(abs_ops[i], cross ? 0xFF : 0, 0x90);
            cpu.x = cpu.y = 1;
            cpu.sp = 0xF0;
            uint16_t addr = cross ? 0x9100 : 0x9001;
            fixture_memory[addr] = 0xAC;
            CHECK(cpu_step(&cpu) == (int)(4 + cross));
            CHECK(cpu.pc == 0x8003);
            CHECK(bus_count == 4 + cross);
            CHECK(bus_events[3].addr == (cross ? 0x9000 : addr));
            CHECK(bus_events[bus_count - 1].addr == addr);
            if (abs_ops[i] == 0xBF) CHECK(cpu.a == 0xAC && cpu.x == 0xAC);
            if (abs_ops[i] == 0xBB) CHECK(cpu.a == 0xA0 && cpu.x == 0xA0 && cpu.sp == 0xA0);
        }
    }
    for (unsigned cross = 0; cross < 2; ++cross) {
        reset_fixture();
        program(0xB3, 0xFF, 0);
        ram[0xFF] = cross ? 0xFF : 0;
        ram[0] = 0x90;
        cpu.y = 1;
        fixture_memory[cross ? 0x9100 : 0x9001] = 0xAB;
        CHECK(cpu_step(&cpu) == (int)(5 + cross));
        CHECK(cpu.a == 0xAB && cpu.x == 0xAB);
        CHECK(bus_count == 3 + cross);
        CHECK(bus_events[2].cycle == 5);
    }
    return 0;
}

static int indexed_rmw_bus_order(void) {
    static const struct { uint8_t op, result; } cases[] = {
        {0x1E, 2}, {0x5E, 0}, {0x3E, 2}, {0x7E, 0}, {0xFE, 2}, {0xDE, 0},
        {0xDF, 0}, {0xDB, 0}, {0x3F, 2}, {0x3B, 2}, {0x1F, 2}, {0x1B, 2},
        {0x5F, 0}, {0x5B, 0}, {0x7F, 0}, {0x7B, 0}, {0xFF, 2}, {0xFB, 2},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        for (unsigned cross = 0; cross < 2; ++cross) {
            reset_fixture();
            program(cases[i].op, cross ? 0xFF : 0, 0x90);
            cpu.x = cpu.y = 1;
            uint16_t addr = cross ? 0x9100 : 0x9001;
            fixture_memory[addr] = 1;
            CHECK(cpu_step(&cpu) == 7);
            CHECK(bus_count == 7);
            CHECK(bus_events[3].addr == (cross ? 0x9000 : addr));
            CHECK(!bus_events[3].write);
            CHECK(bus_events[4].addr == addr && !bus_events[4].write);
            CHECK(bus_events[5].addr == addr && bus_events[5].write && bus_events[5].value == 1);
            CHECK(bus_events[6].addr == addr && bus_events[6].write);
            CHECK(bus_events[6].value == cases[i].result);
            for (size_t j = 0; j < 7; ++j) CHECK(bus_events[j].cycle == j + 1);
            CHECK(cpu_get_bus_cycle() == 7);
        }
    }

    static const uint8_t indirect_ops[] = {0xD3, 0x33, 0x13, 0x53, 0x73, 0xF3};
    for (size_t i = 0; i < sizeof(indirect_ops); ++i) {
        reset_fixture();
        program(indirect_ops[i], 0xFF, 0);
        ram[0xFF] = 0xFF;
        ram[0] = 0x90;
        cpu.y = 1;
        fixture_memory[0x9100] = 1;
        CHECK(cpu_step(&cpu) == 8);
        CHECK(bus_count == 6);
        CHECK(bus_events[2].addr == 0x9000 && bus_events[2].cycle == 5);
        CHECK(bus_events[3].addr == 0x9100 && !bus_events[3].write);
        CHECK(bus_events[4].write && bus_events[4].value == 1 && bus_events[4].cycle == 7);
        CHECK(bus_events[5].write && bus_events[5].cycle == 8);
    }

    reset_fixture();
    program(0xE3, 0xFE, 0); // ISC ($FE,X), including its unindexed zero-page read.
    cpu.x = 1;
    ram[0xFF] = 0;
    ram[0] = 0x90;
    CHECK(cpu_step(&cpu) == 8);
    CHECK(bus_count == 5);
    CHECK(bus_events[2].addr == 0x9000 && bus_events[2].cycle == 6);
    CHECK(bus_events[3].cycle == 7 && bus_events[4].cycle == 8);
    fixture_memory[0x8002] = 0x8D;
    fixture_memory[0x8003] = 0;
    fixture_memory[0x8004] = 0xA0;
    CHECK(cpu_step(&cpu) == 4);
    CHECK(bus_events[8].write && bus_events[8].cycle == 12);
    return 0;
}

static int branch_bus_order(void) {
    reset_fixture();
    program(0xD0, 5, 0);
    CHECK(cpu_step(&cpu) == 3 && cpu.pc == 0x8007);
    const uint16_t same_page[] = {0x8000, 0x8001, 0x8002};
    CHECK(check_reads(same_page, 3) == 0);

    reset_fixture();
    cpu.pc = 0x80FD;
    fixture_memory[0x80FD] = 0xD0;
    fixture_memory[0x80FE] = 2;
    CHECK(cpu_step(&cpu) == 4 && cpu.pc == 0x8101);
    const uint16_t forward[] = {0x80FD, 0x80FE, 0x80FF, 0x8001};
    CHECK(check_reads(forward, 4) == 0);

    reset_fixture();
    cpu.pc = 0x8100;
    fixture_memory[0x8100] = 0xD0;
    fixture_memory[0x8101] = 0xFC;
    CHECK(cpu_step(&cpu) == 4 && cpu.pc == 0x80FE);
    const uint16_t backward[] = {0x8100, 0x8101, 0x8102, 0x81FE};
    CHECK(check_reads(backward, 4) == 0);

    reset_fixture();
    program(0xD0, 5, 0);
    cpu.status |= ZERO_FLAG;
    CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x8002);
    const uint16_t not_taken[] = {0x8000, 0x8001};
    CHECK(check_reads(not_taken, 2) == 0);
    return 0;
}

static int stack_and_indirect_jump(void) {
    reset_fixture();
    program(0x08, 0x28, 0); // PHP / PLP
    cpu.status = UNUSED_FLAG | DECIMAL_FLAG | CARRY_FLAG;
    CHECK(cpu_step(&cpu) == 3);
    CHECK(ram[0x1FD] == (cpu.status | BREAK_FLAG));
    CHECK(cpu_step(&cpu) == 4);
    CHECK(cpu.status == (UNUSED_FLAG | DECIMAL_FLAG | CARRY_FLAG));

    reset_fixture();
    program(0x40, 0, 0);
    cpu.sp = 0xFA;
    ram[0x1FB] = 0xFF;
    ram[0x1FC] = 0x34;
    ram[0x1FD] = 0x12;
    CHECK(cpu_step(&cpu) == 6);
    CHECK(cpu.pc == 0x1234 && cpu.status == 0xEF && cpu.sp == 0xFD);

    reset_fixture();
    program(0x60, 0, 0);
    ram[0x1FE] = 0x12;
    ram[0x1FF] = 0x90;
    CHECK(cpu_step(&cpu) == 6 && cpu.pc == 0x9013);
    const uint16_t rts_reads[] = {0x8000, 0x8001, 0x9012};
    CHECK(check_reads(rts_reads, 3) == 0);
    CHECK(bus_events[2].cycle == 6);

    reset_fixture();
    cpu.pc = 0x01FD;
    cpu.sp = 0xFF;
    ram[0x1FD] = 0x20;
    ram[0x1FE] = 0xCD;
    ram[0x1FF] = 0xAB;
    CHECK(cpu_step(&cpu) == 6);
    CHECK(cpu.pc == 0x01CD); // The stack write replaces the high target byte before it is read.

    reset_fixture();
    program(0x6C, 0xFF, 0x90);
    fixture_memory[0x90FF] = 0x34;
    fixture_memory[0x9000] = 0x12;
    fixture_memory[0x9100] = 0x56;
    CHECK(cpu_step(&cpu) == 5 && cpu.pc == 0x1234);
    const uint16_t jmp_reads[] = {0x8000, 0x8001, 0x8002, 0x90FF, 0x9000};
    CHECK(check_reads(jmp_reads, 5) == 0);
    return 0;
}

static int interrupt_entry(void) {
    reset_fixture();
    program(0xEA, 0, 0);
    cpu.status = UNUSED_FLAG | DECIMAL_FLAG | CARRY_FLAG;
    raise_frame_irq();
    CHECK(cpu_step(&cpu) == 9);
    CHECK(cpu_total_cycles == 9 && apu.cycle_in_seq == 9);
    CHECK(cpu.pc == 0xA000 && cpu.sp == 0xFA);
    CHECK(ram[0x1FD] == 0x80 && ram[0x1FC] == 1);
    CHECK(ram[0x1FB] == (UNUSED_FLAG | DECIMAL_FLAG | CARRY_FLAG));
    const uint16_t irq_reads[] = {0x8000, 0x8001, 0x8001, 0x8001, 0xFFFE, 0xFFFF};
    CHECK(check_reads(irq_reads, 6) == 0);
    CHECK(bus_events[2].cycle == 3 && bus_events[5].cycle == 9);

    static const uint8_t next_ops[] = {0xEA, 0x00, 0x18, 0x38, 0xA9};
    for (size_t i = 0; i < sizeof(next_ops); ++i) {
        reset_fixture();
        program(next_ops[i], 0, 0);
        cpu.a = 0x55;
        cpu.status |= CARRY_FLAG;
        ram[0x1FE] = BREAK_FLAG;
        cpu_request_nmi();
        CHECK(cpu_step(&cpu) == 7);
        CHECK(cpu.pc == 0x9000 && cpu.a == 0x55 && cpu.sp == 0xFA);
        CHECK(cpu.status & CARRY_FLAG);
        CHECK(ram[0x1FD] == 0x80 && ram[0x1FC] == 0);
        CHECK(!(ram[0x1FB] & BREAK_FLAG));
        const uint16_t nmi_reads[] = {0x8000, 0x8000, 0xFFFA, 0xFFFB};
        CHECK(check_reads(nmi_reads, 4) == 0);
        CHECK(bus_events[2].cycle == 6 && bus_events[3].cycle == 7);
    }

    reset_fixture();
    program(0, 0xEA, 0);
    CHECK(cpu_step(&cpu) == 7);
    CHECK(cpu.pc == 0xA000 && ram[0x1FC] == 2);
    CHECK((ram[0x1FB] & (BREAK_FLAG | UNUSED_FLAG)) == (BREAK_FLAG | UNUSED_FLAG));
    CHECK(!(cpu.status & BREAK_FLAG));

    reset_fixture();
    program(0, 0xEA, 0);
    ppu.scanline = 0;
    ppu.dot = 0;
    cpu_set_nmi_line(true);
    CHECK(cpu_step(&cpu) == 7);
    CHECK(cpu.pc == 0x9000 && ram[0x1FC] == 2 && (ram[0x1FB] & BREAK_FLAG));

    reset_fixture();
    program(0x8D, 0, 0x20); // Enabling NMI in vblank asserts the pin on the final STA cycle.
    fixture_memory[0x8003] = 0xEA;
    fixture_memory[0x9000] = fixture_memory[0x9001] = 0xEA;
    cpu.a = 0x80;
    ppu.scanline = 241;
    ppu.dot = 20;
    ppu.status = 0x80;
    CHECK(cpu_step(&cpu) == 4 && cpu.pc == 0x8003);
    CHECK(cpu_step(&cpu) == 9 && cpu.pc == 0x9000);
    CHECK(ram[0x1FC] == 4);
    CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x9001); // A held level is not a second edge.
    CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x9002);
    return 0;
}

static int irq_mask_latency(void) {
    reset_fixture();
    program(0x58, 0xEA, 0); // CLI must allow the following NOP to complete.
    raise_frame_irq();
    CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x8001);
    CHECK(cpu_step(&cpu) == 9 && cpu.pc == 0xA000);
    CHECK(ram[0x1FC] == 2);

    reset_fixture();
    program(0x78, 0, 0);
    cpu.status &= ~INTERRUPT_FLAG;
    raise_frame_irq();
    CHECK(cpu_step(&cpu) == 9 && cpu.pc == 0xA000);
    CHECK(ram[0x1FB] & INTERRUPT_FLAG);

    reset_fixture();
    program(0x28, 0, 0);
    cpu.status &= ~INTERRUPT_FLAG;
    cpu.sp = 0xFC;
    ram[0x1FD] = INTERRUPT_FLAG | BREAK_FLAG;
    raise_frame_irq();
    CHECK(cpu_step(&cpu) == 11 && cpu.pc == 0xA000);
    CHECK(!(cpu.status & BREAK_FLAG));

    reset_fixture();
    program(0x28, 0xEA, 0);
    cpu.sp = 0xFC;
    ram[0x1FD] = BREAK_FLAG;
    raise_frame_irq();
    CHECK(cpu_step(&cpu) == 4 && cpu.pc == 0x8001);
    CHECK(cpu_step(&cpu) == 9 && cpu.pc == 0xA000);

    reset_fixture();
    program(0x40, 0, 0);
    cpu.sp = 0xFA;
    ram[0x1FB] = 0;
    ram[0x1FC] = 0;
    ram[0x1FD] = 0x90;
    raise_frame_irq();
    CHECK(cpu_step(&cpu) == 13 && cpu.pc == 0xA000); // RTI uses the restored I immediately.
    return 0;
}

static int frame_irq_acknowledgment(void) {
    for (unsigned phase = 0; phase < 2; ++phase) {
        reset_fixture();
        cpu_total_cycles = phase;
        program(0xAD, 0x15, 0x40); // LDA $4015
        fixture_memory[0x8003] = 0xEA;
        cpu.status &= ~INTERRUPT_FLAG;
        irq_assert_cycle = phase + 4;

        CHECK(cpu_step(&cpu) == 4 && cpu.pc == 0x8003);
        CHECK(cpu.a & 0x40);
        CHECK(apu.frame_irq);
        CHECK(!apu.frame_irq_source && !apu_irq_pending(&apu));
        CHECK(apu.frame_irq_clear_delay == (phase ? 2 : 1));
        CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x8004);
    }

    reset_fixture();
    program(0xAD, 0x15, 0x40); // The IRQ is sampled one cycle before the status read.
    cpu.status &= ~INTERRUPT_FLAG;
    irq_assert_cycle = 3;
    CHECK(cpu_step(&cpu) == 11 && cpu.pc == 0xA000);
    CHECK(cpu.a & 0x40);
    CHECK(!apu.frame_irq_source && !apu_irq_pending(&apu));
    CHECK(ram[0x1FC] == 3);
    return 0;
}

static int masked_store_addresses(void) {
    static const uint8_t ops[] = {0x9F, 0x93, 0x9E, 0x9C, 0x9B};
    static const uint8_t values[] = {0xFF, 0x80};
    for (size_t i = 0; i < sizeof(ops); ++i) {
        for (size_t j = 0; j < sizeof(values); ++j) {
            for (unsigned cross = 0; cross < 2; ++cross) {
                reset_fixture();
                uint8_t reg = values[j];
                cpu.a = cpu.x = reg;
                cpu.y = 1;
                if (ops[i] == 0x9C) { cpu.x = 1; cpu.y = reg; }
                program(ops[i], cross ? 0xFF : 0, 0x8F);
                if (ops[i] == 0x93) {
                    fixture_memory[0x8001] = 0xFF;
                    ram[0xFF] = cross ? 0xFF : 0;
                    ram[0] = 0x8F;
                }
                CHECK(cpu_step(&cpu) == (ops[i] == 0x93 ? 6 : 5));
                CHECK(bus_count == (ops[i] == 0x93 ? 4u : 5u));
                CHECK(bus_events[bus_count - 2].addr == (cross ? 0x8F00 : 0x8F01));
                uint8_t expected = reg & 0x90;
                uint16_t addr = cross ? (uint16_t)(expected << 8) : 0x8F01;
                CHECK(bus_events[bus_count - 1].addr == addr);
                CHECK(bus_events[bus_count - 1].write);
                CHECK(bus_events[bus_count - 1].value == expected);
                if (ops[i] == 0x9B) CHECK(cpu.sp == reg);
            }
        }
    }

    // A DMC halt during the final indexed dummy read removes the high-byte mask
    // from the value driven by all five unstable store opcodes.
    for (size_t i = 0; i < sizeof(ops); ++i) {
        reset_fixture();
        cpu.a = cpu.x = cpu.y = 0xFF;
        program(ops[i], 0x00, 0x8F);
        unsigned request_cycle = 3;
        if (ops[i] == 0x93) {
            fixture_memory[0x8001] = 0xFF;
            ram[0xFF] = 0x00;
            ram[0] = 0x8F;
            request_cycle = 4;
        }
        fixture_memory[0xC000] = 0x55;
        prepare_dmc(0xC000);
        dma_request_cycle = request_cycle;
        int cycles = cpu_step(&cpu);
        CHECK(cycles > (ops[i] == 0x93 ? 6 : 5));
        CHECK(bus_events[bus_count - 1].write);
        CHECK(bus_events[bus_count - 1].addr == 0x8FFF);
        CHECK(bus_events[bus_count - 1].value == 0xFF);
        if (ops[i] == 0x9B) CHECK(cpu.sp == 0xFF);
    }
    return 0;
}

static int reset_bus_sequence(void) {
    reset_fixture();
    cpu.a = 0x11;
    cpu.x = 0x22;
    cpu.y = 0x33;
    cpu.pc = 0x8123;
    cpu.sp = 0x44;
    cpu.status = CARRY_FLAG | ZERO_FLAG | DECIMAL_FLAG | BREAK_FLAG;
    cpu.halted = true;
    fixture_memory[0xFFFC] = 0x34;
    fixture_memory[0xFFFD] = 0x92;
    write_mem(0x4014, 2);
    apu.dmc.dma_pending = true;
    apu.dmc.bytes_remaining = 1;
    bus_count = 0;
    mapper_clocks = 0;
    cpu_total_cycles = 0;

    ppu_soft_reset(&ppu);
    apu_soft_reset(&apu);
    cpu_soft_reset(&cpu);

    CHECK(cpu_total_cycles == 7 && mapper_clocks == 7);
    CHECK(cpu.a == 0x11 && cpu.x == 0x22 && cpu.y == 0x33);
    CHECK(cpu.sp == 0x41 && cpu.pc == 0x9234 && !cpu.halted);
    CHECK(cpu.status == (CARRY_FLAG | ZERO_FLAG | DECIMAL_FLAG | INTERRUPT_FLAG | UNUSED_FLAG));
    CHECK(bus_count == 4);
    CHECK(bus_events[0].addr == 0x8123 && bus_events[1].addr == 0x8123);
    CHECK(bus_events[2].addr == 0xFFFC && bus_events[3].addr == 0xFFFD);
    CHECK(!apu_dmc_dma_pending(&apu));
    fixture_memory[0x9234] = 0xEA;
    CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x9235);

    cpu.a = 0xAA;
    cpu.x = 0xBB;
    cpu.y = 0xCC;
    cpu.pc = 0x8123;
    cpu.sp = 0x55;
    cpu.status = 0xFF;
    cpu_total_cycles = 99;
    mapper_clocks = 0;
    bus_count = 0;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    fixture_memory[0xFFFC] = 0x00;
    fixture_memory[0xFFFD] = 0x80;
    cpu_power_on(&cpu);
    CHECK(cpu_total_cycles == 7 && mapper_clocks == 7);
    CHECK(cpu.a == 0 && cpu.x == 0 && cpu.y == 0);
    CHECK(cpu.sp == 0xFD && cpu.pc == 0x8000);
    CHECK(cpu.status == (INTERRUPT_FLAG | UNUSED_FLAG));
    CHECK(bus_count == 2 && bus_events[0].addr == 0xFFFC && bus_events[1].addr == 0xFFFD);
    CHECK(ppu.total_cycles == 22); // One startup PPU clock plus seven CPU cycles.
    cpu_soft_reset(&cpu);
    CHECK(cpu_total_cycles == 14 && mapper_clocks == 14);
    CHECK(ppu.total_cycles == 43); // Soft reset does not insert another startup clock.
    return 0;
}

static int halt_and_reset(void) {
    static const uint8_t ops[] = {0x02,0x12,0x22,0x32,0x42,0x52,0x62,0x72,0x92,0xB2,0xD2,0xF2};
    for (size_t i = 0; i < sizeof(ops); ++i) {
        reset_fixture();
        program(ops[i], 0xE8, 0);
        cpu.status &= ~INTERRUPT_FLAG;
        raise_frame_irq();
        CHECK(cpu_step(&cpu) == 2 && cpu.halted);
        CHECK(cpu.pc == 0x8000 && cpu.sp == 0xFD);
        cpu_request_nmi();
        CHECK(cpu_step(&cpu) == 1);
        CHECK(cpu.pc == 0x8000 && cpu.sp == 0xFD && cpu.x == 0);
        cpu.status |= BREAK_FLAG;
        cpu_soft_reset(&cpu);
        CHECK(!cpu.halted && !(cpu.status & BREAK_FLAG) && (cpu.status & UNUSED_FLAG));
        fixture_memory[0x8000] = 0xEA;
        CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x8001);
    }
    return 0;
}

static int regional_bus_timing_and_pal_dma(void) {
    static const NesRegion regions[] = {NES_REGION_NTSC, NES_REGION_PAL, NES_REGION_DENDY};
    static const uint64_t first_read_dot[] = {2, 1, 2};
    for (size_t i = 0; i < sizeof(regions) / sizeof(regions[0]); ++i) {
        reset_fixture_region(regions[i]);
        program(0xEA, 0, 0);
        CHECK(cpu_step(&cpu) == 2);
        CHECK(bus_count == 2);
        CHECK(bus_events[0].ppu_cycle == first_read_dot[i]);
        CHECK(ppu.total_cycles == 6);
        CHECK(mapper_clocks == 2 && apu.cycle_in_seq == 2);
    }

    reset_fixture_region(NES_REGION_PAL);
    program(0xAD, 0x00, 0x90);
    fixture_memory[0x8003] = 0xEA;
    fixture_memory[0x9000] = 0x5A;
    fixture_memory[0xC000] = 0x73;
    prepare_dmc(0xC000);
    dma_request_cycle = 2;
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x5A);
    CHECK(apu_dmc_dma_pending(&apu));
    CHECK(cpu_step(&cpu) == 5 && cpu.pc == 0x8004);
    CHECK(!apu_dmc_dma_pending(&apu) && apu.dmc.sample_buffer == 0x73);
    CHECK(bus_events[4].addr == 0x8003 && bus_events[5].addr == 0x8003);
    CHECK(bus_events[6].addr == 0xC000 && bus_events[7].addr == 0x8003);

    nes_set_region(NES_REGION_NTSC);
    return 0;
}

static int bus_cycle_interrupt_polling(void) {
    reset_fixture();
    program(0xAD, 0, 0x90);
    fixture_memory[0x9000] = 0xA5;
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0xA5);
    CHECK(mapper_clocks == 4 && apu.cycle_in_seq == 4 && ppu.total_cycles == 12);
    for (size_t i = 0; i < 4; ++i) {
        CHECK(bus_events[i].cycle == i + 1);
        CHECK(bus_events[i].apu_cycle == i + 1);
        CHECK(bus_events[i].ppu_cycle == (i + 1) * 3 - 1);
    }
    (void)read_mem(0x9000);
    // Reading state outside instruction execution must not advance the mapper clock.
    CHECK(cpu_total_cycles == 4 && mapper_clocks == 4);

    for (unsigned edge = 1; edge <= 2; ++edge) {
        reset_fixture();
        program(0xEA, 0xEA, 0);
        cpu.status &= ~INTERRUPT_FLAG;
        irq_assert_cycle = edge;
        if (edge == 2) CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x8001);
        CHECK(cpu_step(&cpu) == 9 && cpu.pc == 0xA000);
        CHECK(ram[0x1FC] == edge);
    }

    reset_fixture();
    // The final read clears an IRQ that the CPU sampled on the previous cycle.
    program(0xAD, 0x15, 0x40);
    cpu.status &= ~INTERRUPT_FLAG;
    irq_assert_cycle = 3;
    CHECK(cpu_step(&cpu) == 11 && cpu.pc == 0xA000);
    CHECK(!apu.frame_irq && ram[0x1FC] == 3);

    reset_fixture();
    program(0xEA, 0xEA, 0);
    nmi_assert_cycle = nmi_clear_cycle = 1;
    // An NMI pulse that ends before the sampling point is not latched.
    CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x8001);
    CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x8002);

    reset_fixture();
    program(0xEA, 0xEA, 0);
    nmi_assert_cycle = 2;
    nmi_clear_cycle = 3;
    CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x8001);
    CHECK(cpu_step(&cpu) == 9 && cpu.pc == 0x9000);
    CHECK(ram[0x1FC] == 2); // A sampled edge remains pending after the line falls.

    for (unsigned edge = 4; edge <= 5; ++edge) {
        reset_fixture();
        program(0, 0, 0);
        fixture_memory[0xA000] = 0xEA;
        nmi_assert_cycle = edge;
        CHECK(cpu_step(&cpu) == 7);
        CHECK(ram[0x1FB] & BREAK_FLAG);
        CHECK(cpu.pc == (edge == 4 ? 0x9000 : 0xA000));
        if (edge == 5) {
            CHECK(cpu_step(&cpu) == 9 && cpu.pc == 0x9000);
            CHECK(ram[0x1F9] == 1); // An edge after vector selection waits for the handler's NOP.
        }
    }

    for (unsigned edge = 6; edge <= 7; ++edge) {
        reset_fixture();
        program(0xEA, 0, 0);
        fixture_memory[0xA000] = 0xEA;
        cpu.status &= ~INTERRUPT_FLAG;
        irq_assert_cycle = 1;
        nmi_assert_cycle = edge;
        CHECK(cpu_step(&cpu) == 9);
        CHECK(cpu.pc == (edge == 6 ? 0x9000 : 0xA000));
        CHECK(!(ram[0x1FB] & BREAK_FLAG));
        if (edge == 7) CHECK(cpu_step(&cpu) == 9 && cpu.pc == 0x9000);
    }

    reset_fixture();
    program(0xD0, 2, 0);
    fixture_memory[0x8004] = 0xEA;
    cpu.status &= ~INTERRUPT_FLAG;
    irq_assert_cycle = 2;
    CHECK(cpu_step(&cpu) == 3 && cpu.pc == 0x8004);
    CHECK(cpu_step(&cpu) == 9 && cpu.pc == 0xA000);
    CHECK(ram[0x1FC] == 5);

    reset_fixture();
    cpu.pc = 0x80FD;
    fixture_memory[0x80FD] = 0xD0;
    fixture_memory[0x80FE] = 2;
    cpu.status &= ~INTERRUPT_FLAG;
    irq_assert_cycle = 2;
    CHECK(cpu_step(&cpu) == 11 && cpu.pc == 0xA000);
    CHECK(ram[0x1FD] == 0x81 && ram[0x1FC] == 1);
    return 0;
}

static void prepare_dmc(uint16_t address) {
    apu.dmc.current_addr = address;
    apu.dmc.bytes_remaining = 1;
    apu.dmc.enabled = true;
    apu.dmc.sample_buffer_empty = false;
    apu.dmc.dma_pending = false;
    apu.dmc.timer = 1000;
}

static void prepare_dmc_reload_collision(uint16_t address) {
    apu.dmc.sample_addr = address;
    apu.dmc.sample_len = 1;
    apu.dmc.current_addr = address;
    apu.dmc.bytes_remaining = 1;
    apu.dmc.enabled = true;
    apu.dmc.sample_buffer_empty = true;
    apu.dmc.bits_remaining = 1;
    apu.dmc.timer = 2;
    apu.dmc.dma_pending = true;
    apu.dmc.dma_halt_started = false;
    apu.dmc.dma_abort_requested = false;
}

static int dma_arbitration(void) {
    reset_fixture();
    program(0xEE, 0, 0x90); // A pending read DMA cannot interrupt either RMW write.
    fixture_memory[0x8003] = 0xEA;
    fixture_memory[0x9000] = 1;
    fixture_memory[0xC000] = 0x73;
    prepare_dmc(0xC000);
    dma_request_cycle = 4;
    CHECK(cpu_step(&cpu) == 6 && apu_dmc_dma_pending(&apu));
    CHECK(bus_count == 6 && bus_events[4].write && bus_events[5].write);
    CHECK(bus_events[4].cycle == 5 && bus_events[5].cycle == 6);
    CHECK(cpu_step(&cpu) == 5 && !apu_dmc_dma_pending(&apu));
    CHECK(bus_events[8].addr == 0xC000 && bus_events[8].cycle == 9);
    CHECK(apu.dmc.sample_buffer == 0x73);

    reset_fixture();
    program(0xEA, 0, 0);
    prepare_dmc(0xC000);
    apu.dmc.dma_pending = true;
    apu.dmc.disable_delay = 1;
    CHECK(cpu_step(&cpu) == 3); // Disabling during halt cancels the remaining DMA cycles.
    CHECK(bus_count == 3 && bus_events[0].addr == 0x8000 && bus_events[1].addr == 0x8000);
    CHECK(!apu_dmc_dma_pending(&apu) && apu.dmc.bytes_remaining == 0);

    for (unsigned internal = 0; internal < 2; ++internal) {
        reset_fixture();
        program(0xAD, 0x16, 0x40);
        pad1.buttons = 2;
        write_mem(0x4016, 1);
        write_mem(0x4016, 0);
        uint16_t address = internal ? 0xC016 : 0xC000;
        fixture_memory[address] = 0xE1;
        prepare_dmc(address);
        dma_request_cycle = 3;
        CHECK(cpu_step(&cpu) == 8);
        if (internal) {
            // Repeated internal decoding holds the controller read line through the DMA get.
            CHECK(cpu.a == 0xE0 && apu.dmc.sample_buffer == 0xE0);
            CHECK(joypad_read(&pad1) == 1);
        } else {
            CHECK(cpu.a == 0xE1 && apu.dmc.sample_buffer == 0xE1);
            CHECK(joypad_read(&pad1) == 0); // The halt read discarded A before the CPU read B.
        }
    }

    reset_fixture();
    program(0xAD, 0x16, 0x40);
    pad1.buttons = 1;
    write_mem(0x4016, 1);
    write_mem(0x4016, 0);
    fixture_memory[0xC016] = 0xE0;
    prepare_dmc(0xC016);
    dma_request_cycle = 3;
    CHECK(cpu_step(&cpu) == 8);
    // Controller data can dominate the open-bus latch without defeating a
    // simultaneously driven zero on the byte delivered to the DMC.
    CHECK(apu.dmc.sample_buffer == 0xE0);

    reset_fixture();
    program(0xAD, 0x15, 0x40);
    fixture_memory[0xC015] = 0x20;
    prepare_dmc(0xC015);
    dma_request_cycle = 3;
    CHECK(cpu_step(&cpu) == 8);
    CHECK(!(cpu.a & 0x20));
    CHECK(!(apu.dmc.sample_buffer & 0x20)); // Internal $4015 wins the conflicted DMA byte.
    CHECK(read_mem(0x4018) & 0x20); // $4015 changed only the internal latch.
    CHECK(read_mem(0x4015) & 0x20); // Floating read copied the external bit back internally.

    reset_fixture();
    program(0xEA, 0, 0);
    fixture_memory[0xC000] = 0x73;
    prepare_dmc(0xC000);
    apu.dmc.dma_pending = true;
    apu.dmc.disable_delay = 2;
    CHECK(cpu_step(&cpu) == 5);
    CHECK(!apu_dmc_dma_pending(&apu) && apu.dmc.bytes_remaining == 0);
    bool saw_late_get = false;
    for (size_t i = 0; i < bus_count; ++i)
        if (!bus_events[i].write && bus_events[i].addr == 0xC000) saw_late_get = true;
    CHECK(saw_late_get); // Once the dummy cycle has begun, the aligned get is too late to suppress.

    reset_fixture();
    program(0xEA, 0, 0);
    memset(ppu.oam, 0, sizeof(ppu.oam));
    for (unsigned i = 0; i < 256; ++i) ram[0x200 + i] = (uint8_t)(i ^ 0xA5);
    fixture_memory[0xC000] = 0x73;
    prepare_dmc(0xC000);
    dma_request_cycle = 10;
    write_mem(0x4014, 2);
    CHECK(cpu_step(&cpu) == 518); // DMC consumes one OAM get slot and its following put slot.
    CHECK(mapper_clocks == 518 && apu.cycle_in_seq == 518 && ppu.total_cycles == 1554);
    CHECK(apu.dmc.sample_buffer == 0x73 && !apu_dmc_dma_pending(&apu));
    for (unsigned i = 0; i < 256; ++i) {
        uint8_t value = (uint8_t)(i ^ 0xA5);
        CHECK(ppu.oam[i] == (i % 4 == 2 ? (value & 0xE3u) : value));
    }

    reset_fixture();
    program(0x0E, 0x14, 0x40); // Both writes hit $4014; the second page replaces the first.
    fixture_memory[0x8003] = 0xEA;
    fixture_memory[0x8080] = 0x55;
    CHECK(cpu_step(&cpu) == 6);
    CHECK(cpu_step(&cpu) == 516);
    CHECK(ppu.oam[0] == 0x0E && ppu.oam[0x80] == 0x55);
    return 0;
}

static int dmc_revision_dma(void) {
    for (unsigned later = 0; later < 2; ++later) {
        reset_fixture();
        CHECK(apu_set_cpu_revision(later ? APU_CPU_REVISION_LATE_2A03 : APU_CPU_REVISION_EARLY_2A03));
        program(0xEA, 0, 0);
        fixture_memory[0xC000] = 0xA5;
        prepare_dmc_reload_collision(0xC000);

        CHECK(cpu_step(&cpu) == (later ? 9 : 5));
        size_t dmc_reads = 0;
        for (size_t i = 0; i < bus_count; ++i)
            if (!bus_events[i].write && bus_events[i].addr == 0xC000) dmc_reads++;
        CHECK(dmc_reads == (later ? 2u : 1u));
        CHECK(!apu_dmc_dma_pending(&apu));

        if (later) {
            CHECK(bus_count == 9);
            CHECK(bus_events[2].addr == 0xC000 && bus_events[2].cycle == 3);
            CHECK(bus_events[3].addr == 0x8000 && bus_events[3].cycle == 4);
            CHECK(bus_events[4].addr == 0x8000 && bus_events[4].cycle == 5);
            CHECK(bus_events[5].addr == 0x8000 && bus_events[5].cycle == 6);
            CHECK(bus_events[6].addr == 0xC000 && bus_events[6].cycle == 7);
            CHECK(apu.dmc.shift_reg == 0xA5 && apu.dmc.sample_buffer == 0xA5);
            CHECK(!apu.dmc.sample_buffer_empty && apu.dmc.bytes_remaining == 0);
        }
    }

    for (unsigned later = 0; later < 2; ++later) {
        reset_fixture();
        CHECK(apu_set_cpu_revision(later ? APU_CPU_REVISION_LATE_2A03 : APU_CPU_REVISION_EARLY_2A03));
        program(0xEA, 0, 0);
        fixture_memory[0xC000] = 0x5A;
        prepare_dmc_reload_collision(0xC000);
        memset(ppu.oam, 0, sizeof(ppu.oam));
        for (unsigned i = 0; i < 256; ++i) ram[0x200 + i] = (uint8_t)(i ^ 0x5A);
        write_mem(0x4014, 2);

        (void)cpu_step(&cpu);
        size_t dmc_reads = 0;
        uint64_t dmc_cycles[2] = {0, 0};
        for (size_t i = 0; i < bus_count; ++i) {
            if (!bus_events[i].write && bus_events[i].addr == 0xC000) {
                if (dmc_reads < 2) dmc_cycles[dmc_reads] = bus_events[i].cycle;
                dmc_reads++;
            }
        }
        CHECK(dmc_reads == (later ? 2u : 1u));
        CHECK(dmc_cycles[0] == 3);
        if (later) CHECK(dmc_cycles[1] == 7);
        CHECK(!apu_dmc_dma_pending(&apu));
        for (unsigned i = 0; i < 256; ++i) {
            uint8_t value = (uint8_t)(i ^ 0x5A);
            CHECK(ppu.oam[i] == (i % 4 == 2 ? (value & 0xE3u) : value));
        }
    }

    CHECK(apu_set_cpu_revision(APU_CPU_REVISION_EARLY_2A03));
    return 0;
}

static int dma_cycle_accounting(void) {
    for (unsigned odd = 0; odd < 2; ++odd) {
        reset_fixture();
        cpu_total_cycles = odd;
        program(0x9D, 0, 0x40); // Five-cycle STA $4000,X reaches $4014.
        fixture_memory[0x8003] = 0xEA;
        for (unsigned i = 0; i < 256; ++i) ram[0x200 + i] = (uint8_t)i;
        memset(ppu.oam, 0, sizeof(ppu.oam));
        cpu.x = 0x14;
        cpu.a = 2;
        CHECK(cpu_step(&cpu) == 5);
        CHECK(ppu.oam[1] == 0); // Copying waits for the next CPU read.
        CHECK(cpu_step(&cpu) == (int)(515 + odd));
        CHECK(cpu.pc == 0x8004);
        for (unsigned i = 0; i < 256; ++i)
            CHECK(ppu.oam[i] == (uint8_t)((i % 4 == 2) ? (i & 0xE3u) : i));
        CHECK(ppu.total_cycles == (520 + odd) * 3);
    }
    for (unsigned odd = 0; odd < 2; ++odd) {
        reset_fixture();
        cpu_total_cycles = odd;
        program(0xEA, 0, 0);
        fixture_memory[0xFFFF] = 0x75;
        apu.dmc.current_addr = 0xFFFF;
        apu.dmc.bytes_remaining = 1;
        apu.dmc.dma_pending = true;
        apu.dmc.irq_enable = true;
        apu.dmc.timer = 1000;
        CHECK(cpu_step(&cpu) == (int)(5 + odd));
        CHECK(cpu_total_cycles == 5 + 2 * odd && apu.cycle_in_seq == 5 + odd);
        CHECK(ppu.total_cycles == (5 + odd) * 3);
        CHECK(apu.dmc.sample_buffer == 0x75 && !apu.dmc.sample_buffer_empty);
        CHECK(apu.dmc.current_addr == 0x8000 && apu.dmc.bytes_remaining == 0);
        CHECK(apu.dmc.irq_flag && !apu_dmc_dma_pending(&apu));
        CHECK(bus_events[2 + odd].addr == 0xFFFF);
        CHECK(bus_events[2 + odd].cycle == 3 + 2 * odd);
    }
    return 0;
}

static int startup_phase_selection(void) {
    static const NesRegion regions[] = {NES_REGION_NTSC, NES_REGION_PAL, NES_REGION_DENDY};
    for (size_t region = 0; region < sizeof(regions) / sizeof(regions[0]); ++region) {
        cpu_use_default_startup_alignment();
        reset_fixture_region(regions[region]);
        const NesTiming *timing = nes_timing();
        for (unsigned cpu_offset = 0; cpu_offset < timing->cpu_divider; ++cpu_offset) {
            for (unsigned phase = 0; phase < timing->ppu_divider; ++phase) {
                CHECK(cpu_set_startup_alignment(cpu_offset, phase));
                CHECK(!cpu_set_startup_alignment(timing->cpu_divider, phase));
                CHECK(!cpu_set_startup_alignment(cpu_offset, timing->ppu_divider));
                ppu_power_on(&ppu);
                apu_power_on(&apu);
                bus_count = 0;
                mapper_clocks = 0;
                CHECK(cpu_power_on(&cpu));
                CpuStartupAlignment actual = cpu_get_startup_alignment();
                CHECK(actual.cpu_offset == cpu_offset && actual.ppu_phase == phase);
                CHECK(cpu_total_cycles == 7 && mapper_clocks == 7 && cpu.sp == 0xFD);
                CHECK(cpu.pc == 0x8000 && cpu.a == 0 && cpu.x == 0 && cpu.y == 0);
                CHECK(cpu.status == (INTERRUPT_FLAG | UNUSED_FLAG));
                CHECK(bus_count == 2 && bus_events[0].addr == 0xFFFC && bus_events[1].addr == 0xFFFD);
                unsigned initial = phase + timing->ppu_divider + cpu_offset;
                for (unsigned vector = 0; vector < 2; ++vector) {
                    unsigned clocks = initial + (5 + vector) * timing->cpu_divider
                                    + timing->cpu_divider / 2 - 1;
                    CHECK(bus_events[vector].cycle == 6 + vector);
                    CHECK(bus_events[vector].ppu_cycle == clocks / timing->ppu_divider);
                }
                CHECK(ppu.total_cycles == (initial + 7u * timing->cpu_divider) / timing->ppu_divider);
                cpu.a = 0xA5;
                ram[0x12] = 0xC3;
                cpu_soft_reset(&cpu);
                CHECK(cpu_total_cycles == 14 && mapper_clocks == 14 && cpu.sp == 0xFA);
                CHECK(cpu.a == 0xA5 && ram[0x12] == 0xC3);
                CHECK(ppu.total_cycles == (initial + 14u * timing->cpu_divider) / timing->ppu_divider);
                actual = cpu_get_startup_alignment();
                CHECK(actual.cpu_offset == cpu_offset && actual.ppu_phase == phase);
            }
        }
    }

    cpu_use_default_startup_alignment();
    reset_fixture_region(NES_REGION_PAL);
    CHECK(cpu_set_startup_alignment(15, 4));
    CHECK(!cpu_startup_alignment_valid(NES_REGION_NTSC));
    CHECK(!cpu_startup_alignment_valid(NES_REGION_DENDY));
    uint8_t image[16 + 0x4000 + 0x2000] = {0};
    memcpy(image, "NES\x1A", 4);
    image[4] = 1;
    image[5] = 1;
    image[7] = 8;
    CHECK(load_rom_memory(image, sizeof(image)) == -1);
    CHECK(cart == &fixture_mapper && nes_timing()->region == NES_REGION_PAL);
    nes_set_region(NES_REGION_NTSC);
    uint64_t previous_cycles = cpu_total_cycles;
    uint64_t previous_ppu = ppu.total_cycles;
    cpu.a = 0x5A;
    CHECK(!cpu_power_on(&cpu));
    CHECK(cpu.a == 0x5A && cpu_total_cycles == previous_cycles && ppu.total_cycles == previous_ppu);
    cpu_use_default_startup_alignment();
    return 0;
}

static int seeded_startup_alignment(void) {
    BusEvent first_trace[64][4];
    CpuStartupAlignment first_alignment[64];
    cpu_use_default_startup_alignment();
    reset_fixture();
    program(0xEA, 0xEA, 0);
    unsigned phases_seen = 0;
    unsigned cpu_offsets_seen = 0;
    for (unsigned pass = 0; pass < 2; ++pass) {
        cpu_seed_startup_alignment(123456789u);
        for (unsigned boot = 0; boot < 64; ++boot) {
            ppu_power_on(&ppu);
            apu_power_on(&apu);
            bus_count = 0;
            CHECK(cpu_power_on(&cpu));
            CpuStartupAlignment actual = cpu_get_startup_alignment();
            CHECK(actual.cpu_offset < nes_timing()->cpu_divider);
            CHECK(actual.ppu_phase < nes_timing()->ppu_divider);
            phases_seen |= 1u << actual.ppu_phase;
            cpu_offsets_seen |= 1u << actual.cpu_offset;
            CHECK(cpu_step(&cpu) == 2 && bus_count == 4);
            if (!pass) {
                first_alignment[boot] = actual;
                memcpy(first_trace[boot], bus_events, sizeof(first_trace[boot]));
            } else {
                CHECK(actual.cpu_offset == first_alignment[boot].cpu_offset);
                CHECK(actual.ppu_phase == first_alignment[boot].ppu_phase);
                for (unsigned event = 0; event < 4; ++event) {
                    CHECK(bus_events[event].addr == first_trace[boot][event].addr);
                    CHECK(bus_events[event].value == first_trace[boot][event].value);
                    CHECK(bus_events[event].cycle == first_trace[boot][event].cycle);
                    CHECK(bus_events[event].ppu_cycle == first_trace[boot][event].ppu_cycle);
                }
            }
        }
    }
    CHECK(phases_seen == 0x0F);
    CHECK(cpu_offsets_seen == 0x0FFF);
    CpuStartupAlignment actual = cpu_get_startup_alignment();
    uint64_t before = ppu.total_cycles;
    cpu_seed_startup_alignment(0);
    cpu_soft_reset(&cpu);
    CpuStartupAlignment after = cpu_get_startup_alignment();
    CHECK(after.cpu_offset == actual.cpu_offset && after.ppu_phase == actual.ppu_phase);
    CHECK(ppu.total_cycles - before == 21);
    cpu_use_default_startup_alignment();
    return 0;
}

static int startup_phase_register_race(void) {
    for (unsigned phase = 0; phase < 4; phase += 3) {
        cpu_use_default_startup_alignment();
        reset_fixture();
        CHECK(cpu_set_startup_alignment(0, phase));
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        CHECK(cpu_power_on(&cpu));
        program(0xAD, 0x02, 0x20);
        ppu.scanline = 240;
        ppu.dot = 332;
        ppu.status = 0;
        ppu.suppress_vblank = false;
        CHECK(cpu_step(&cpu) == 4);
        CHECK((cpu.a & 0x80) == (phase ? 0x80 : 0));
    }
    cpu_use_default_startup_alignment();
    return 0;
}

static int read_test_mode_register(uint16_t addr, uint8_t expected) {
    program(0xAD, (uint8_t)addr, (uint8_t)(addr >> 8));
    cpu.pc = 0x8000;
    CHECK(cpu_step(&cpu) == 4);
    CHECK(cpu.a == expected);
    return 0;
}

static int cpu_diagnostic_output_latches(void) {
    reset_fixture();
    cpu_set_test_mode(true);

    apu.pulse1.enabled = true;
    apu.pulse1.lc.length = 10;
    apu.pulse1.env.constant_volume = true;
    apu.pulse1.env.volume = 5;
    apu.pulse1.timer_reload = 100;
    apu.pulse1.timer = 0;
    apu.pulse1.duty = 0;
    apu.pulse1.duty_step = 0;
    apu.noise.enabled = true;
    apu.noise.lc.length = 10;
    apu.noise.env.constant_volume = true;
    apu.noise.env.volume = 6;
    apu.noise.period = 1000;
    apu.noise.timer = 0;
    apu.noise.lfsr = 4;
    apu.cpu_cycle_odd = false;

    fixture_memory[0x8000] = 0xEA; // A CPU cycle clocks both channel DAC latches.
    cpu.pc = 0x8000;
    CHECK(cpu_step(&cpu) == 2);
    CHECK(apu.pulse1.output_level == 5 && apu.noise.output_level == 6);

    // Keep the next channel edge beyond the disable/read sequence. $4015 clears
    // the length counters immediately, but the DACs retain their last outputs.
    apu.pulse1.timer = 100;
    apu.noise.timer = 100;
    static const uint8_t disable_and_read[] = {
        0xA9, 0x00,                   // LDA #$00
        0x8D, 0x15, 0x40,             // STA $4015
        0xAD, 0x18, 0x40, 0x85, 0x10, // LDA $4018 / STA $10
        0xAD, 0x19, 0x40, 0x85, 0x11  // LDA $4019 / STA $11
    };
    memcpy(fixture_memory + 0x8000, disable_and_read, sizeof(disable_and_read));
    cpu.pc = 0x8000;
    CHECK(cpu_step(&cpu) == 2);
    CHECK(cpu_step(&cpu) == 4);
    CHECK(cpu_step(&cpu) == 4);
    CHECK(cpu_step(&cpu) == 3);
    CHECK(cpu_step(&cpu) == 4);
    CHECK(cpu_step(&cpu) == 3);
    CHECK(apu.pulse1.lc.length == 0 && apu.noise.lc.length == 0);
    CHECK(ram[0x10] == 0x05 && ram[0x11] == 0x60);

    // Once each timer reaches its next edge, the disabled channels drive zero.
    apu.pulse1.timer = 0;
    apu.noise.timer = 0;
    apu.cpu_cycle_odd = false;
    fixture_memory[0x8000] = 0xAD; fixture_memory[0x8001] = 0x18; fixture_memory[0x8002] = 0x40;
    fixture_memory[0x8003] = 0x85; fixture_memory[0x8004] = 0x12;
    fixture_memory[0x8005] = 0xAD; fixture_memory[0x8006] = 0x19; fixture_memory[0x8007] = 0x40;
    fixture_memory[0x8008] = 0x85; fixture_memory[0x8009] = 0x13;
    cpu.pc = 0x8000;
    CHECK(cpu_step(&cpu) == 4);
    CHECK(cpu_step(&cpu) == 3);
    CHECK(cpu_step(&cpu) == 4);
    CHECK(cpu_step(&cpu) == 3);
    CHECK(ram[0x12] == 0 && ram[0x13] == 0);
    CHECK(apu.pulse1.output_level == 0 && apu.noise.output_level == 0);
    cpu_set_test_mode(false);
    return 0;
}

static int cpu_diagnostic_output_reads(void) {
    reset_fixture();
    CHECK(!cpu_test_mode_enabled());
    for (uint16_t addr = 0x4018; addr <= 0x401F; ++addr) {
        write_mem(0x401B, 0xA5);
        CHECK(read_mem(addr) == 0xA5);
    }
    cpu_set_test_mode(true);
    apu.pulse1.enabled = apu.pulse2.enabled = true;
    apu.pulse1.lc.length = apu.pulse2.lc.length = 10;
    apu.pulse1.env.constant_volume = apu.pulse2.env.constant_volume = true;
    apu.pulse1.env.volume = 3;
    apu.pulse2.env.volume = 10;
    apu.pulse1.timer_reload = apu.pulse2.timer_reload = 100;
    apu.pulse1.timer = apu.pulse2.timer = 1000;
    apu.pulse1.duty_step = apu.pulse2.duty_step = 1;
    apu.pulse1.output_level = 3;
    apu.pulse2.output_level = 10;
    apu.tri.output_level = 11;
    apu.noise.enabled = true;
    apu.noise.lc.length = 10;
    apu.noise.env.constant_volume = true;
    apu.noise.env.volume = 6;
    apu.noise.lfsr = 0x4000;
    apu.noise.timer = 1000;
    apu.noise.output_level = 6;
    write_mem(0x4011, 0x65);
    raise_frame_irq();
    CHECK(read_test_mode_register(0x4018, 0xA3) == 0);
    CHECK(read_test_mode_register(0x4019, 0x6B) == 0);
    CHECK(read_test_mode_register(0x401A, 0x65) == 0);
    CHECK(apu.frame_irq_source); // Output reads do not acknowledge the frame counter.

    apu.pulse1.timer = 0;
    apu.noise.lfsr = 2;
    apu.noise.timer = 0;
    apu.noise.period = 4068;
    apu.tri.step = 0;
    apu.tri.lc.length = 1;
    apu.tri.linear_counter = 1;
    apu.tri.timer = 0;
    apu.tri.timer_reload = 100;
    apu.dmc.timer = 0;
    apu.dmc.shift_reg = 1;
    apu.dmc.bits_remaining = 8;
    apu.dmc.silence = false;
    apu_step(&apu, 1);
    CHECK(read_test_mode_register(0x4018, 0xA0) == 0);
    CHECK(read_test_mode_register(0x4019, 0x0E) == 0);
    CHECK(read_test_mode_register(0x401A, 0x67) == 0);
    write_mem(0x4018, 0xFF);
    write_mem(0x4019, 0xFF);
    write_mem(0x401A, 0xFF);
    CHECK(read_mem(0x401A) == 0x67); // The profile does not add writable test registers.
    for (uint16_t addr = 0x401B; addr <= 0x401F; ++addr) {
        write_mem(0x401B, 0xA5);
        CHECK(read_mem(addr) == 0xA5);
    }
    write_mem(0x401B, 0xA5);
    CHECK((read_mem(0x4015) & 0x60) == 0x60);
    CHECK(!apu.frame_irq_source && read_mem(0x401B) == 0xA5);

    APU secondary = {0};
    apu_power_on(&secondary);
    apu_select_machine(&secondary);
    write_mem(0x4011, 0x2B);
    CHECK(read_test_mode_register(0x401A, 0x2B) == 0);
    apu_select_machine(NULL);
    CHECK(read_mem(0x401A) == 0x67);
    apu_soft_reset(&apu);
    cpu_soft_reset(&cpu);
    CHECK(cpu_test_mode_enabled() && read_mem(0x401A) == 0);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    CHECK(cpu_test_mode_enabled());
    cpu_set_test_mode(false);
    for (uint16_t addr = 0x4018; addr <= 0x401F; ++addr) {
        write_mem(0x401B, 0x5A);
        CHECK(read_mem(addr) == 0x5A);
    }
    CHECK(read_test_mode_register(0x4018, 0x40) == 0); // High operand byte remains on the bus.
    return 0;
}

int test_cpu_accuracy(void) {
    static int (*const tests[])(void) = {
        controller_latching, open_bus_and_cart_decoding, nop_and_zero_page_cycles,
        xaa_immediate, unofficial_immediate_semantics, indexed_read_penalties,
        indexed_rmw_bus_order, branch_bus_order, stack_and_indirect_jump,
        interrupt_entry, irq_mask_latency, frame_irq_acknowledgment, masked_store_addresses,
        reset_bus_sequence, halt_and_reset, regional_bus_timing_and_pal_dma,
        bus_cycle_interrupt_polling, dma_arbitration, dmc_revision_dma, dma_cycle_accounting,
        startup_phase_selection, seeded_startup_alignment, startup_phase_register_race,
        cpu_diagnostic_output_latches, cpu_diagnostic_output_reads,
    };
    Mapper *saved_cart = cart;
    iNESHeader saved_header = ines_header;
    int failures = 0;
    size_t count = sizeof(tests) / sizeof(tests[0]);
    for (size_t i = 0; i < count; ++i) failures += tests[i]();
    cart = saved_cart;
    ines_header = saved_header;
    printf("CPU/joypad accuracy: %s (%zu groups, %d failing)\n", failures ? "FAIL" : "PASS", count, failures);
    return failures;
}
