/* SPDX-License-Identifier: GPL-3.0-or-later
 * Cartridge regression tests using synthetic PRG/CHR images and public APIs.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif
#include "../rom/mapper.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../system/timing.h"
#include "../../include/globals.h"

extern uint64_t cpu_total_cycles;

static uint8_t fixture_prg[0x200000];
static uint8_t fixture_chr[0x20000];

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static iNESHeader header_for(unsigned mapper, size_t prg_bytes, bool chr_ram) {
    iNESHeader h = {0};
    memcpy(h.signature, "NES\x1A", 4);
    h.prg_rom_chunks = (uint8_t)(prg_bytes / 0x4000);
    h.chr_rom_chunks = chr_ram ? 0 : 1;
    h.flags6 = (uint8_t)(mapper << 4);
    h.flags7 = (uint8_t)(mapper & 0xF0);
    return h;
}

static int fixture_with_header(const iNESHeader *h, size_t prg_bytes, size_t chr_bytes) {
    // Finish using the previous borrowed buffers before refilling them.
    mapper_shutdown();
    for (size_t i = 0; i < prg_bytes; ++i) fixture_prg[i] = (uint8_t)(i / 0x2000);
    for (size_t i = 0; i < chr_bytes; ++i) fixture_chr[i] = (uint8_t)(i / 0x0400);
    cpu_total_cycles = 0;
    return mapper_init_from_header(h, fixture_prg, prg_bytes, fixture_chr, chr_bytes);
}

static int fixture(unsigned mapper, size_t prg_bytes, size_t chr_bytes, bool chr_ram) {
    iNESHeader h = header_for(mapper, prg_bytes, chr_ram);
    return fixture_with_header(&h, prg_bytes, chr_bytes);
}

static void serial_write(uint16_t addr, uint8_t value) {
    for (unsigned bit = 0; bit < 5; ++bit) {
        cpu_total_cycles += 2;
        cart_cpu_write(addr, (uint8_t)((value >> bit) & 1));
    }
}

static int test_small_cartridges(void) {
    const unsigned boards[] = {0, 3, 13};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        CHECK(fixture(boards[i], 0x4000, 0x4000, boards[i] == 13) == (int)boards[i]);
        CHECK(cart_cpu_read(0x8000) == 0);
        CHECK(cart_cpu_read(0xBFFF) == 1);
        CHECK(cart_cpu_read(0xC000) == 0);
        CHECK(cart_cpu_read(0xFFFF) == 1);
    }
    CHECK(fixture(0, 0x4000, 0x1000, false) == 0);
    CHECK(cart_ppu_read(0x0FFF) == 3 && cart_ppu_read(0x1FFF) == 3);
    cart_ppu_write(0x1FFF, 0xA5);
    CHECK(cart_ppu_read(0x0FFF) == 3);
    CHECK(fixture(0, 0x4000, 0x1000, true) == 0);
    cart_ppu_write(0x1FFF, 0xA5);
    CHECK(cart_ppu_read(0x0FFF) == 0xA5);
    return 0;
}

static int test_mmc1_banks_and_ram(void) {
    CHECK(fixture(1, 0x20000, 0x8000, false) == 1);
    CHECK(cart_cpu_read(0xC000) == 14);
    serial_write(0xE000, 3);
    CHECK(cart_cpu_read(0x8000) == 6 && cart_cpu_read(0xC000) == 14);
    serial_write(0x8000, 0x00);
    CHECK(cart_cpu_read(0x8000) == 4 && cart_cpu_read(0xC000) == 6);
    serial_write(0x8000, 0x0A);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 6);
    serial_write(0x8000, 0x1F);
    serial_write(0xA000, 1);
    serial_write(0xC000, 3);
    CHECK(cart_ppu_read(0x0000) == 4 && cart_ppu_read(0x1000) == 12);
    serial_write(0x8000, 0x0F);
    serial_write(0xA000, 3);
    CHECK(cart_ppu_read(0x0000) == 8 && cart_ppu_read(0x1000) == 12);
    cart_cpu_write(0x6123, 0xA6);
    serial_write(0xE000, 0x13);
    CHECK(cart_cpu_read(0x6123) == 0xFF);
    cart_cpu_write(0x6123, 0x55);
    CHECK(cart_cpu_read(0x8000) == 6);
    serial_write(0xE000, 3);
    CHECK(cart_cpu_read(0x6123) == 0xA6);
    CHECK(fixture(1, 0x4000, 0x1000, false) == 1);
    serial_write(0x8000, 0);
    serial_write(0xE000, 3);
    serial_write(0xA000, 3);
    CHECK(cart_cpu_read(0xC000) == 0 && cart_ppu_read(0x1FFF) == 3);
    return 0;
}

static int test_mmc1_serial_timing(void) {
    CHECK(fixture(1, 0x20000, 0x2000, true) == 1);
    cpu_total_cycles = 100;
    cart_cpu_write(0xE000, 1);
    cpu_total_cycles = 101;
    cart_cpu_write(0xE000, 0); // Consecutive RMW write must not enter the serial buffer.
    for (unsigned i = 0; i < 4; ++i) {
        cpu_total_cycles += 2;
        cart_cpu_write(0xE000, 0);
    }
    CHECK(cart_cpu_read(0x8000) == 2);
    serial_write(0x8000, 3);
    cpu_total_cycles += 2;
    cart_cpu_write(0xE000, 1);
    cpu_total_cycles++;
    cart_cpu_write(0x8000, 0x80); // Reset is accepted even on the consecutive cycle.
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    CHECK(cart_cpu_read(0xC000) == 14);
    serial_write(0xE000, 2);
    CHECK(cart_cpu_read(0x8000) == 4);
    return 0;
}

static int test_mmc1_outer_and_fixed_banks(void) {
    CHECK(fixture(1, 0x80000, 0x2000, true) == 1);
    CHECK(cart_cpu_read(0xC000) == 30);
    serial_write(0xA000, 0x10);
    CHECK(cart_cpu_read(0x8000) == 32 && cart_cpu_read(0xC000) == 62);
    serial_write(0x8000, 0x1C);
    serial_write(0xC000, 0);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 30);
    serial_write(0xA000, 0x10);
    CHECK(cart_cpu_read(0xC000) == 62);
    iNESHeader h = header_for(1, 0x8000, true);
    h.flags7 = 0x08;
    h.prg_ram_size = 0x50;
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x8000, 0x2000) == 1);
    serial_write(0xE000, 1);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 2);
    return 0;
}

static void latch_banks(void) {
    cart_cpu_write(0xB000, 1);
    cart_cpu_write(0xC000, 2);
    cart_cpu_write(0xD000, 3);
    cart_cpu_write(0xE000, 4);
}

static int test_mmc2_banks_and_latches(void) {
    CHECK(fixture(9, 0x20000, 0x8000, false) == 9);
    cart_cpu_write(0xA000, 2);
    CHECK(cart_cpu_read(0x8000) == 2);
    CHECK(cart_cpu_read(0xA000) == 13);
    CHECK(cart_cpu_read(0xC000) == 14);
    CHECK(cart_cpu_read(0xFFFF) == 15);
    latch_banks();
    CHECK(cart_ppu_read(0) == 8 && cart_ppu_read(0x1000) == 16);
    CHECK(cart_ppu_read(0x0FD9) == 11); // Only $0FD8 is decoded in the left table.
    CHECK(cart_ppu_read(0) == 8);
    CHECK(cart_ppu_read(0x0FD8) == 11); // Triggering read still uses the old bank.
    CHECK(cart_ppu_read(0) == 4);
    CHECK(cart_ppu_read(0x0FE8) == 7);
    CHECK(cart_ppu_read(0) == 8);
    CHECK(cart_ppu_read(0x1FDF) == 19);
    CHECK(cart_ppu_read(0x1000) == 12);
    CHECK(cart_ppu_read(0x1FEF) == 15);
    CHECK(cart_ppu_read(0x1000) == 16);
    cart_ppu_write(0x1000, 0x55);
    CHECK(cart_ppu_read(0x1000) == 16);
    cart_cpu_write(0xF000, 1);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    return 0;
}

static int test_mmc4_latches_and_chr_ram(void) {
    CHECK(fixture(10, 0x20000, 0x8000, false) == 10);
    cart_cpu_write(0xA000, 2);
    CHECK(cart_cpu_read(0x8000) == 4 && cart_cpu_read(0xFFFF) == 15);
    latch_banks();
    CHECK(cart_ppu_read(0) == 8 && cart_ppu_read(0x1000) == 16);
    CHECK(cart_ppu_read(0x0FDF) == 11);
    CHECK(cart_ppu_read(0) == 4 && cart_ppu_read(0x1000) == 16);
    CHECK(cart_ppu_read(0x0FEF) == 7);
    CHECK(cart_ppu_read(0) == 8);
    for (unsigned mapper = 9; mapper <= 10; ++mapper) {
        CHECK(fixture(mapper, 0x20000, 0x8000, true) == (int)mapper);
        latch_banks();
        cart_ppu_write(0x1000, 0xA5);
        (void)cart_ppu_read(0x1FD8);
        CHECK(cart_ppu_read(0x1000) == 12);
        cart_ppu_write(0x1000, 0x5A);
        (void)cart_ppu_read(0x1FE8);
        CHECK(cart_ppu_read(0x1000) == 0xA5);
        (void)cart_ppu_read(0x1FD8);
        CHECK(cart_ppu_read(0x1000) == 0x5A);
    }
    return 0;
}

static int test_mmc3_banks_and_protection(void) {
    iNESHeader h = header_for(4, 0x20000, true);
    h.flags6 |= 8;
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_cpu_write(0x8000, 6);
    cart_cpu_write(0x8001, 3);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xC000) == 14);
    cart_cpu_write(0x8000, 0x46);
    CHECK(cart_cpu_read(0x8000) == 14 && cart_cpu_read(0xC000) == 3);
    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 5);
    CHECK(cart_ppu_read(0) == 4 && cart_ppu_read(0x0400) == 5);
    cart_cpu_write(0x8000, 0x80);
    CHECK(cart_ppu_read(0x1000) == 4 && cart_ppu_read(0x1400) == 5);
    cart_ppu_write(0x1400, 0x56);
    cart_cpu_write(0x8000, 0);
    CHECK(cart_ppu_read(0x0400) == 0x56);
    cart_cpu_write(0xA001, 0x80);
    cart_cpu_write(0x6000, 0xAB);
    cart_cpu_write(0xA001, 0xC0);
    cart_cpu_write(0x6000, 0x55);
    CHECK(cart_cpu_read(0x6000) == 0xAB);
    cart_cpu_write(0xA001, 0);
    CHECK(cart_cpu_read(0x6000) == 0xFF);
    cart_cpu_write(0x6000, 0x11);
    cart_cpu_write(0xA001, 0x80);
    CHECK(cart_cpu_read(0x6000) == 0xAB);
    cart_cpu_write(0xA000, 1);
    CHECK(cart_get_mirroring() == MIRROR_FOUR);
    uint8_t nt[0x1000] = {0};
    for (unsigned i = 0; i < 4; ++i) cart_nt_write((uint16_t)(0x2000 + i * 0x400), (uint8_t)(i + 1), nt);
    for (unsigned i = 0; i < 4; ++i) CHECK(cart_nt_read((uint16_t)(0x2000 + i * 0x400), nt) == i + 1);
    return 0;
}

static void a12_pulse(uint64_t cycle) {
    cart_notify_ppu_address(0x2000, cycle);
    cart_notify_ppu_address(0x1000, cycle + 9);
}

static int test_mmc3_irq_edges(void) {
    CHECK(fixture(4, 0x20000, 0x2000, false) == 4);
    cart_cpu_write(0xC000, 2);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(0); // Loading the counter does not assert a nonzero latch.
    CHECK(!cart_irq_pending());
    cart_notify_ppu_address(0x2000, 12);
    cart_notify_ppu_address(0x1000, 18); // Two CPU clocks is insufficient.
    cart_notify_ppu_address(0x1000, 30); // A held-high address is not another edge.
    CHECK(!cart_irq_pending());
    a12_pulse(36);
    CHECK(!cart_irq_pending());
    cart_notify_ppu_address(0x2000, 48);
    cart_notify_ppu_address(0x2001, 54); // Repeated low addresses do not restart the filter.
    cart_notify_ppu_address(0x1000, 57);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    a12_pulse(60);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xE001, 0);
    cart_notify_scanline();
    CHECK(!cart_irq_pending());
    a12_pulse(72);
    CHECK(cart_irq_pending());
    return 0;
}

static int test_mmc3_render_trace(void) {
    CHECK(fixture(4, 0x20000, 0x2000, false) == 4);
    cart_cpu_write(0xC000, 1);
    cart_cpu_write(0xE001, 0);
    // BG at $0000, sprites at $1000: short sprite-fetch gaps are filtered.
    for (unsigned dot = 1; dot < 257; dot += 8) {
        cart_notify_ppu_address(0x2000, dot);
        cart_notify_ppu_address(0, dot + 4);
    }
    for (unsigned dot = 257; dot < 321; dot += 8) {
        cart_notify_ppu_address(0x2000, dot);
        cart_notify_ppu_address(0x1000, dot + 4);
    }
    CHECK(!cart_irq_pending()); // Exactly one qualified edge loaded the latch.
    cart_notify_ppu_address(0x2000, 321);
    cart_notify_ppu_address(0x1000, 602);
    CHECK(cart_irq_pending());

    CHECK(fixture(4, 0x20000, 0x2000, false) == 4);
    cart_cpu_write(0xC000, 1);
    cart_cpu_write(0xE001, 0);
    // Omitting the dot-0 pattern address would produce a spurious qualified
    // nine-dot gap from the dummy nametable fetch to the next line's BG fetch.
    cart_notify_ppu_address(0x2000, 321);
    cart_notify_ppu_address(0x1000, 325);
    CHECK(!cart_irq_pending());
    cart_notify_ppu_address(0x2000, 337);
    cart_notify_ppu_address(0x1000, 346);
    CHECK(!cart_irq_pending()); // First qualifying edge loads one.
    cart_notify_ppu_address(0x2000, 598);
    cart_notify_ppu_address(0x1000, 666);
    CHECK(cart_irq_pending());

    CHECK(fixture(4, 0x20000, 0x2000, false) == 4);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x1000, 333);
    cart_notify_ppu_address(0x2000, 337);
    // Visible dot 0 drives a pattern address without reading CHR. It splits
    // the low interval, so neither this pulse nor the dot-5 fetch clocks IRQ.
    cart_notify_ppu_address(0x1000, 341);
    cart_notify_ppu_address(0x2000, 342);
    cart_notify_ppu_address(0x1000, 346);
    CHECK(!cart_irq_pending());
    cart_notify_ppu_address(0x2000, 598);
    cart_notify_ppu_address(0x1000, 666);
    CHECK(cart_irq_pending()); // The long sprite-fetch interval still qualifies.
    return 0;
}

static int test_simple_mapper_registers(void) {
    CHECK(fixture(2, 0x100000, 0x2000, true) == 2);
    cart_cpu_write(0x8000, 32);
    CHECK(cart_cpu_read(0x8000) == 64 && cart_cpu_read(0xFFFF) == 127);
    CHECK(fixture(7, 0x80000, 0x2000, true) == 7);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0x8000, 0x18);
    CHECK(cart_cpu_read(0x8000) == 32 && cart_get_mirroring() == MIRROR_SINGLE1);
    CHECK(fixture(11, 0x20000, 0x20000, false) == 11);
    cart_cpu_write(0x8000, 0x52);
    CHECK(cart_cpu_read(0x8000) == 8 && cart_ppu_read(0) == 40);
    CHECK(fixture(13, 0x8000, 0x4000, true) == 13);
    for (unsigned bank = 0; bank < 4; ++bank) {
        cart_cpu_write(0x8000, (uint8_t)bank);
        cart_ppu_write(0x1000, (uint8_t)(0xA0 + bank));
    }
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL && cart_ppu_read(0) == 0xA0);
    for (unsigned bank = 0; bank < 4; ++bank) {
        cart_cpu_write(0x8000, (uint8_t)bank);
        CHECK(cart_ppu_read(0x1000) == 0xA0 + bank);
    }
    return 0;
}

static int test_bus_conflict_submappers(void) {
    const unsigned boards[] = {2, 3, 7};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        for (unsigned submapper = 1; submapper <= 2; ++submapper) {
            iNESHeader h = header_for(boards[i], 0x20000, false);
            h.flags7 |= 0x08;
            h.prg_ram_size = (uint8_t)(submapper << 4);
            CHECK(fixture_with_header(&h, 0x20000, 0x8000) == (int)boards[i]);
            fixture_prg[0] = 1;
            cart_cpu_write(0x8000, 3);
            unsigned selected = submapper == 2 ? 1 : 3;
            if (boards[i] == 3) CHECK(cart_ppu_read(0) == selected * 8);
            else CHECK(cart_cpu_read(0x8001) == selected * (boards[i] == 2 ? 2 : 4));
        }
    }
    return 0;
}

static int test_mapper15_modes(void) {
    static const struct { uint16_t address; uint8_t value, pages[4]; } cases[] = {
        {0x8000, 0x02, {4, 5, 6, 7}}, {0x8000, 0x82, {5, 4, 7, 6}},
        {0x8001, 0x02, {4, 5, 14, 15}}, {0x8001, 0x82, {5, 6, 15, 16}},
        {0x8002, 0x02, {4, 4, 4, 4}}, {0x8002, 0x82, {5, 5, 5, 5}},
        {0x8003, 0x02, {4, 5, 4, 5}}, {0x8003, 0x82, {5, 6, 5, 6}},
        {0x8000, 0x42, {132, 133, 134, 135}}
    };
    CHECK(fixture(15, sizeof(fixture_prg), 0x2000, true) == 15);
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        cart_cpu_write(cases[i].address, cases[i].value);
        for (unsigned slot = 0; slot < 4; ++slot)
            CHECK(cart_cpu_read((uint16_t)(0x8000 + slot * 0x2000)) == cases[i].pages[slot]);
        CHECK(cart_get_mirroring() == ((cases[i].value & 0x40) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL));
    }
    for (unsigned mode = 0; mode < 4; ++mode) {
        cart_cpu_write(0x8002, 0);
        cart_ppu_write(0, 0x42);
        cart_cpu_write((uint16_t)(0x8000 | mode), 0);
        cart_ppu_write(0, 0x69);
        CHECK(cart_ppu_read(0) == ((mode == 1 || mode == 2) ? 0x69 : 0x42));
    }
    return 0;
}

static void mmc5_enter_frame(uint8_t *nt) {
    (void)cart_nt_read(0x2000, nt);
    (void)cart_nt_read(0x2000, nt);
    (void)cart_nt_read(0x2000, nt);
    (void)cart_nt_read(0x23C0, nt);
    (void)cart_nt_read(0x2001, nt);
}

static void mmc5_next_scanline(uint8_t *nt) {
    (void)cart_nt_read(0x2000, nt);
    (void)cart_nt_read(0x2000, nt);
    (void)cart_nt_read(0x2000, nt);
    (void)cart_nt_read(0x23C0, nt);
}

static int test_mmc5_memory_windows(void) {
    CHECK(fixture(5, 0x20000, 0x8000, false) == 5);
    CHECK(cart_cpu_read(0xE000) == 15);
    cart_cpu_write(0x5102, 2);
    cart_cpu_write(0x5103, 1);
    cart_cpu_write(0x6000, 0xA5);
    CHECK(cart_cpu_read(0x8000) == 0xA5);
    cart_cpu_write(0x8000, 0x5A);
    CHECK(cart_cpu_read(0x6000) == 0x5A);
    cart_cpu_write(0x5102, 0);
    cart_cpu_write(0x8000, 0x11);
    CHECK(cart_cpu_read(0x8000) == 0x5A);
    cart_cpu_write(0x5102, 2);
    cart_cpu_write(0x5114, 0x82);
    cart_cpu_write(0x8000, 0x11);
    CHECK(cart_cpu_read(0x8000) == 2 && cart_cpu_read(0x6000) == 0x5A);
    cart_cpu_write(0x5117, 3);
    CHECK(cart_cpu_read(0xE000) == 3); // $5117 cannot select RAM.
    cart_cpu_write(0x5100, 0);
    cart_cpu_write(0x5117, 8);
    CHECK(cart_cpu_read(0x8000) == 8 && cart_cpu_read(0xE000) == 11);
    cart_cpu_write(0x5100, 1);
    cart_cpu_write(0x5115, 0x84);
    CHECK(cart_cpu_read(0x8000) == 4 && cart_cpu_read(0xA000) == 5);
    CHECK(cart_cpu_read(0xC000) == 8 && cart_cpu_read(0xE000) == 9);
    cart_cpu_write(0x5115, 0);
    CHECK(cart_cpu_read(0x8000) == 0x5A && cart_cpu_read(0xA000) == 0);
    cart_cpu_write(0x5100, 2);
    cart_cpu_write(0x5115, 0x84);
    cart_cpu_write(0x5116, 0);
    CHECK(cart_cpu_read(0x8000) == 4 && cart_cpu_read(0xA000) == 5);
    CHECK(cart_cpu_read(0xC000) == 0x5A && cart_cpu_read(0xE000) == 8);
    return 0;
}

static int test_mmc5_exram_and_irq(void) {
    CHECK(fixture(5, 0x20000, 0x8000, false) == 5);
    uint8_t nt[0x1000] = {0};
    cart_cpu_write(0x5104, 2);
    cart_cpu_write(0x5C00, 0xA5);
    CHECK(cart_cpu_read(0x5C00) == 0xA5);
    cart_cpu_write(0x5104, 3);
    cart_cpu_write(0x5C00, 0x11);
    CHECK(cart_cpu_read(0x5C00) == 0xA5);
    cart_cpu_write(0x5104, 0);
    cart_cpu_write(0x5C00, 0x55);
    cart_cpu_write(0x5104, 2);
    CHECK(cart_cpu_read(0x5C00) == 0);
    mmc5_enter_frame(nt);
    cart_cpu_write(0x5104, 0);
    cart_cpu_write(0x5C00, 0x55);
    cart_cpu_write(0x5104, 2);
    CHECK(cart_cpu_read(0x5C00) == 0x55);
    cart_cpu_write(0x5203, 1);
    cart_cpu_write(0x5204, 0x80);
    mmc5_next_scanline(nt);
    CHECK(cart_irq_pending());
    CHECK(cart_cpu_read(0x5204) == 0xC0 && !cart_irq_pending());
    cart_cpu_write(0x5203, 2);
    mmc5_next_scanline(nt);
    CHECK(cart_irq_pending());
    (void)cart_cpu_read(0xFFFA);
    CHECK(!cart_irq_pending() && cart_cpu_read(0x5204) == 0);
    mmc5_enter_frame(nt);
    CHECK(cart_cpu_read(0x5204) == 0x40);
    CHECK(cart != NULL && cart->clock != NULL);
    cart->clock(2);
    CHECK(cart_cpu_read(0x5204) == 0x40);
    cart->clock(1);
    CHECK(cart_cpu_read(0x5204) == 0);
    cart_cpu_write(0x5105, 0xFF);
    cart_cpu_write(0x5106, 0x37);
    cart_cpu_write(0x5107, 2);
    CHECK(cart_nt_read(0x2000, nt) == 0x37 && cart_nt_read(0x23C0, nt) == 0xAA);
    cart_cpu_write(0x5205, 0xFF);
    cart_cpu_write(0x5206, 0xFF);
    CHECK(cart_cpu_read(0x5205) == 1 && cart_cpu_read(0x5206) == 0xFE);
    return 0;
}

static int test_mmc5_chr_fetch_modes(void) {
    CHECK(fixture(5, 0x20000, 0x20000, false) == 5);
    cart_set_ppu_fetch_source(CART_PPU_FETCH_CPU);
    CHECK(cart_ppu_read(0x0000) == 0 && cart_ppu_read(0x1FFF) == 7);

    cart_cpu_write(0x5101, 3);
    cart_cpu_write(0x5120, 4);
    cart_cpu_write(0x5127, 11);
    cart_cpu_write(0x5128, 20);
    cart_cpu_write(0x5129, 21);
    cart_cpu_write(0x512A, 22);
    cart_cpu_write(0x512B, 23);
    CHECK(cart_ppu_read(0x0000) == 4); // 8x8 mode forces set A even after a B write.
    write_mem(0x2000, 0x20);
    CHECK(cart_ppu_read(0x0000) == 4); // The 8x8 B write must not reappear later.
    cart_cpu_write(0x5128, 20);
    cart_cpu_write(0x5129, 21);
    cart_cpu_write(0x512A, 22);
    cart_cpu_write(0x512B, 23);
    CHECK(cart_ppu_read(0x0000) == 20 && cart_ppu_read(0x1000) == 20);
    write_mem(0x2008, 0); // PPU sees the mirror; MMC5 still sees large sprites.
    CHECK(ppu.ctrl == 0 && cart_ppu_read(0x0000) == 20);
    cart_set_ppu_fetch_source(CART_PPU_FETCH_BG);
    CHECK(cart_ppu_read(0x0C00) == 23);
    cart_set_ppu_fetch_source(CART_PPU_FETCH_SPRITE);
    CHECK(cart_ppu_read(0x0000) == 4 && cart_ppu_read(0x1C00) == 11);
    cart_set_ppu_fetch_source(CART_PPU_FETCH_CPU);
    write_mem(0x2000, 0);
    CHECK(cart_ppu_read(0x0000) == 4);
    write_mem(0x3FF8, 0x20);
    CHECK(ppu.ctrl == 0x20 && cart_ppu_read(0x0000) == 4);
    write_mem(0x2000, 0);

    cart_cpu_write(0x5101, 0);
    cart_cpu_write(0x5127, 2);
    cart_set_ppu_fetch_source(CART_PPU_FETCH_SPRITE);
    CHECK(cart_ppu_read(0x0000) == 16 && cart_ppu_read(0x1C00) == 23);
    cart_cpu_write(0x5101, 1);
    cart_cpu_write(0x5123, 3);
    cart_cpu_write(0x5127, 5);
    CHECK(cart_ppu_read(0x0000) == 12 && cart_ppu_read(0x1000) == 20);
    cart_cpu_write(0x5101, 2);
    cart_cpu_write(0x5121, 2);
    cart_cpu_write(0x5123, 4);
    CHECK(cart_ppu_read(0x0000) == 4 && cart_ppu_read(0x0800) == 8);
    return 0;
}

static int test_mmc5_extended_rendering(void) {
    CHECK(fixture(5, 0x20000, 0x20000, false) == 5);
    uint8_t nt[0x1000] = {0};
    nt[5] = 0x2A;
    cart_cpu_write(0x5104, 2);
    cart_cpu_write(0x5C05, 0x83); // Palette 2, 4KB CHR bank 3.
    mmc5_enter_frame(nt);
    cart_cpu_write(0x5104, 1);
    CHECK(cart_nt_read(0x2005, nt) == 0x2A);
    CHECK(cart_nt_read(0x23C0, nt) == 0xAA);
    CHECK(cart_ppu_read(0x0123) == 12);
    CHECK(cart_ppu_read(0x012B) == 12);
    CHECK(cart_ppu_read(0x0123) == 0);
    cart_cpu_write(0x5104, 2);
    cart_cpu_write(0x5C00, 0x5A);
    cart_cpu_write(0x5C01, 0x6B);
    cart_cpu_write(0x5FC0, 0x02);
    fixture_chr[0x4000 + 0x120] = 0x90;
    fixture_chr[0x4000 + 0x121] = 0x91;
    cart_cpu_write(0x5200, 0x84); // Left-side split through column 3.
    cart_cpu_write(0x5201, 0);
    cart_cpu_write(0x5202, 4);
    cart_cpu_write(0x5104, 0);
    uint8_t split_tile = 0;
    for (unsigned i = 0; i < 46; ++i)
        split_tile = cart_nt_read((uint16_t)(0x2100 + (i & 0x1Fu)), nt);
    CHECK(split_tile == 0x5A);
    CHECK(cart_nt_read(0x23C0, nt) == 0xAA);
    CHECK(cart_ppu_read(0x0123) == 0x90);
    CHECK(cart_nt_read(0x211E, nt) == 0x6B);
    CHECK(cart_ppu_read(0x0123) == 0x91);
    cart->clock(3);
    CHECK((cart_cpu_read(0x5204) & 0x40) == 0);
    CHECK(cart_ppu_read(0x0123) == 0);

    CHECK(fixture(5, 0x20000, 0x20000, false) == 5);
    uint8_t repeated_nt[0x1000] = {0};
    repeated_nt[5] = 0x2A;
    cart_cpu_write(0x5104, 2);
    cart_cpu_write(0x5C05, 0x83);
    mmc5_enter_frame(repeated_nt);
    cart_cpu_write(0x5104, 1);
    fixture_chr[0x3000 + 0x3C0] = 0x6D;
    CHECK(cart_nt_read(0x2005, repeated_nt) == 0x2A);
    CHECK(cart_nt_read(0x23C0, repeated_nt) == 0xAA);
    CHECK(cart_nt_read(0x23C0, repeated_nt) == 0x6D);
    CHECK(cart_nt_read(0x23C0, repeated_nt) == 0x6D);
    return 0;
}

static void mmc5_prepare_render(void) {
    ppu_power_on(&ppu);
    ppu.scanline = (int)nes_timing()->scanlines - 1;
    ppu.dot = 0;
    ppu.v = 0;
    ppu.t = 0;
    ppu.x = 0;
    ppu.ctrl = 0;
    ppu.mask = 0x0A; // Background, including the left eight pixels.
    ppu.rendering_enabled = true;
    ppu.fetches_enabled = true;
    ppu_palette[0] = 0x0F;
    ppu_begin_frame_render(framebuffer);
}

static int test_mmc5_rendered_ppu_paths(void) {
    CHECK(fixture(5, 0x20000, 0x20000, false) == 5);
    memset(fixture_chr, 0, sizeof(fixture_chr));
    mmc5_prepare_render();
    for (unsigned tile = 0; tile < 64; ++tile) ppu_vram[tile] = 1;
    for (unsigned row = 0; row < 8; ++row) {
        fixture_chr[0x3000 + 0x10 + row] = 0xFF;
        fixture_chr[0x3000 + 0x18 + row] = 0;
    }
    cart_cpu_write(0x5104, 2);
    for (unsigned tile = 0; tile < 0x3C0; ++tile)
        cart_cpu_write((uint16_t)(0x5C00 + tile), 0xC3); // Palette 3, 4KB bank 3.
    cart_cpu_write(0x5104, 1);
    cart_cpu_write(0x5203, 1);
    cart_cpu_write(0x5204, 0x80);
    ppu_palette[13] = 0x21;
    ppu_palette[14] = 0x16;
    ppu_palette[15] = 0x30;
    ppu_step_dots(341 * 3);
    CHECK(framebuffer[1 * 256] == get_color(0x21));
    CHECK(framebuffer[1 * 256 + 7] == get_color(0x21));
    CHECK(bg_opaque[1 * 256] && bg_opaque[1 * 256 + 7]);
    CHECK(cart_irq_pending());
    CHECK((cart_cpu_read(0x5204) & 0xC0) == 0xC0);

    CHECK(fixture(5, 0x20000, 0x20000, false) == 5);
    memset(fixture_chr, 0, sizeof(fixture_chr));
    mmc5_prepare_render();
    for (unsigned tile = 0; tile < 0x3C0; ++tile) ppu_vram[tile] = 1;
    for (unsigned row = 0; row < 8; ++row) {
        fixture_chr[0x0010 + row] = 0xFF;          // Normal tile 1.
        fixture_chr[0x0018 + row] = 0;
        fixture_chr[0x4000 + 0x20 + row] = 0xFF; // Split tile 2 in bank 4.
        fixture_chr[0x4000 + 0x28 + row] = 0;
    }
    cart_cpu_write(0x5104, 2);
    for (unsigned tile = 0; tile < 0x3C0; ++tile)
        cart_cpu_write((uint16_t)(0x5C00 + tile), 2);
    for (unsigned attr = 0x3C0; attr < 0x400; ++attr)
        cart_cpu_write((uint16_t)(0x5C00 + attr), 0x55); // Split palette 1.
    cart_cpu_write(0x5104, 0);
    cart_cpu_write(0x5200, 0x88); // Left-side split through tile column 7.
    cart_cpu_write(0x5201, 0);
    cart_cpu_write(0x5202, 4);
    ppu_palette[1] = 0x11; // Normal palette 0, pixel 1.
    ppu_palette[5] = 0x21; // Split palette 1, pixel 1.
    ppu_step_dots(341 * 3);
    CHECK(framebuffer[1 * 256 + 63] == get_color(0x21));
    CHECK(framebuffer[1 * 256 + 64] == get_color(0x11));
    CHECK(bg_opaque[1 * 256 + 63] && bg_opaque[1 * 256 + 64]);

    CHECK(fixture(5, 0x20000, 0x20000, false) == 5);
    memset(fixture_chr, 0, sizeof(fixture_chr));
    mmc5_prepare_render();
    for (unsigned tile = 0; tile < 0x3C0; ++tile) ppu_vram[tile] = 1;
    for (unsigned row = 0; row < 8; ++row) {
        fixture_chr[0x0010 + row] = 0xFF;
        fixture_chr[0x4000 + 0x20 + row] = 0xFF;
    }
    cart_cpu_write(0x5104, 2);
    for (unsigned tile = 0; tile < 0x3C0; ++tile)
        cart_cpu_write((uint16_t)(0x5C00 + tile), 2);
    for (unsigned attr = 0x3C0; attr < 0x400; ++attr)
        cart_cpu_write((uint16_t)(0x5C00 + attr), 0x55);
    cart_cpu_write(0x5104, 0);
    cart_cpu_write(0x5200, 0xC8); // Same boundary, split on the right side.
    cart_cpu_write(0x5202, 4);
    ppu_palette[1] = 0x11;
    ppu_palette[5] = 0x21;
    ppu_step_dots(341 * 3);
    CHECK(framebuffer[1 * 256 + 63] == get_color(0x11));
    CHECK(framebuffer[1 * 256 + 64] == get_color(0x21));

    CHECK(fixture(5, 0x20000, 0x20000, false) == 5);
    memset(fixture_chr, 0, sizeof(fixture_chr));
    uint8_t physical_nt[0x1000] = {0};
    cart_cpu_write(0x5104, 2);
    for (unsigned tile = 0; tile < 0x3C0; ++tile)
        cart_cpu_write((uint16_t)(0x5C00 + tile), 0xC3); // Palette 3, 4KB bank 3.
    mmc5_enter_frame(physical_nt);
    // Place the mapper immediately before the final sprite slot's two dummy
    // nametable reads without tripping its three-identical-read scanline detector.
    for (unsigned read = 0; read < 45; ++read)
        (void)cart_nt_read((uint16_t)(0x2000 + (read & 1u)), physical_nt);
    cart_cpu_write(0x5104, 1);
    for (unsigned row = 0; row < 8; ++row) fixture_chr[0x3008 + row] = 0x5A;
    ppu_power_on(&ppu);
    memset(ppu.secondary_oam, 0, sizeof(ppu.secondary_oam));
    ppu.scanline = 0;
    ppu.dot = 313;
    ppu.mask = 0x18;
    ppu.rendering_enabled = true;
    ppu.fetches_enabled = true;
    // The first dummy read remains in the sprite CHR-A window; the second moves
    // the mapper out of it and arms extended attributes. The following physical
    // CHR reads therefore consume the palette slot and then the selected bank.
    ppu_step_dots(8);
    CHECK(ppu.dot == 321);
    CHECK(ppu.sprite_pattern_lo[7] == 0xFF);
    CHECK(ppu.sprite_pattern_hi[7] == 0x5A);
    return 0;
}

static int test_mmc5_audio_and_pcm(void) {
    CHECK(fixture(5, 0x20000, 0x8000, false) == 5);
    CHECK(cart != NULL && cart->clock != NULL && cart_expansion_audio() == 0.0f);

    // The fixed envelope/length divider clocks immediately, then every floor(CPU/240) cycles.
    // A high-register write stages its length reload until the end of the next CPU clock.
    int frame_period = (int)(nes_timing()->cpu_hz / 240.0);
    cart_cpu_write(0x5015, 0x01);
    cart_cpu_write(0x5000, 0x10);
    cart_cpu_write(0x5002, 0);
    cart_cpu_write(0x5003, 0x18); // Length-table entry 3 starts at 2.
    CHECK((cart_cpu_read(0x5015) & 1) == 0);
    cart->clock(1);
    CHECK((cart_cpu_read(0x5015) & 1) != 0);
    cart->clock(frame_period);
    CHECK((cart_cpu_read(0x5015) & 1) != 0);
    cart->clock(frame_period);
    CHECK((cart_cpu_read(0x5015) & 1) == 0);

    // Reload and length-halt writes collide with the old counter on the next
    // CPU clock: the frame tick sees the old halt state, then reload arbitration
    // and the new halt state commit at the end of that clock.
    CHECK(fixture(5, 0x20000, 0x8000, false) == 5);
    cart_cpu_write(0x5015, 0x01);
    cart_cpu_write(0x5000, 0x10);
    cart_cpu_write(0x5003, 0x38); // Length 6.
    cart->clock(1);
    cart->clock(frame_period); // 6 -> 5.
    cart->clock(frame_period - 1);
    cart_cpu_write(0x5000, 0x30); // Stage halt.
    cart_cpu_write(0x5003, 0x18); // Stage length 2 with previous counter 5.
    cart->clock(1);               // Old counter becomes 4; reload loses collision.
    cart_cpu_write(0x5000, 0x10);
    cart->clock(1);               // Commit unhalt before the next frame tick.
    cart->clock(frame_period - 1); // 4 -> 3.
    cart->clock(frame_period);     // 3 -> 2.
    cart->clock(frame_period);     // 2 -> 1.
    CHECK((cart_cpu_read(0x5015) & 1) != 0);
    cart->clock(frame_period);     // 1 -> 0.
    CHECK((cart_cpu_read(0x5015) & 1) == 0);

    CHECK(fixture(5, 0x20000, 0x8000, false) == 5);
    cart_cpu_write(0x5015, 0x01);
    cart_cpu_write(0x5000, 0xDF); // Duty 3, constant volume 15.
    cart_cpu_write(0x5002, 0);
    cart_cpu_write(0x5003, 0x08); // Load length with a period below 8.
    cart->clock(1);
    CHECK(cart_expansion_audio() == 0.0f);
    cart->clock(4);
    float one_pulse = cart_expansion_audio();
    CHECK(one_pulse < 0.0f); // MMC5 pulses are not muted for periods below 8.
    CHECK(one_pulse < -0.125f && one_pulse > -0.127f);
    cart_cpu_write(0x5001, 0xFF); // No sweep unit.
    CHECK(cart_expansion_audio() == one_pulse);

    CHECK(fixture(5, 0x20000, 0x8000, false) == 5);
    cart_cpu_write(0x5015, 0x03);
    cart_cpu_write(0x5000, 0xDF);
    cart_cpu_write(0x5002, 0);
    cart_cpu_write(0x5003, 0x08);
    cart_cpu_write(0x5004, 0xDF);
    cart_cpu_write(0x5006, 0);
    cart_cpu_write(0x5007, 0x08);
    cart->clock(5);
    CHECK(cart_expansion_audio() < one_pulse);
    CHECK(cart_expansion_audio() < -0.251f && cart_expansion_audio() > -0.253f);
    cart_cpu_write(0x5015, 0);
    cart->clock(2);
    CHECK(cart_expansion_audio() == 0.0f);

    cart_cpu_write(0x5010, 0x00);
    cart_cpu_write(0x5011, 0);
    CHECK(!cart_irq_pending()); // Trip state is retained while IRQ output is disabled.
    cart_cpu_write(0x5010, 0x80);
    CHECK(cart_irq_pending());
    cart_cpu_write(0x5011, 0x40);
    CHECK(!cart_irq_pending()); // A nonzero DAC assignment clears the trip state.
    cart_cpu_write(0x5010, 0x00);
    cart_cpu_write(0x5011, 0);
    CHECK(!cart_irq_pending());
    CHECK(cart_cpu_read(0x5010) == 0x01); // Status masks disabled IRQs but still acknowledges the trip.
    cart_cpu_write(0x5010, 0x80);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0x5011, 0);
    CHECK(cart_irq_pending());
    CHECK(cart_cpu_read(0x5010) == 0x81 && !cart_irq_pending());
    cart_cpu_write(0x5011, 0x80);
    float pcm = cart_expansion_audio();
    CHECK(pcm < -0.357f && pcm > -0.360f);
    cart_cpu_write(0x5010, 0x81);
    cart_cpu_write(0x5011, 0x20); // Direct DAC writes are ignored in read mode.
    CHECK(cart_expansion_audio() == pcm);
    cart_cpu_write(0x5114, 0x81);
    CHECK(cart_cpu_read(0x8000) == 1 && !cart_irq_pending());
    float pcm_one = cart_expansion_audio();
    CHECK(pcm_one < 0.0f && pcm_one > pcm);
    cart_cpu_write(0x5114, 0x80);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_irq_pending());
    CHECK(cart_expansion_audio() == pcm_one);
    CHECK(cart_cpu_read(0x5010) == 0x81 && !cart_irq_pending());
    return 0;
}

static int test_header_and_mapper_rejection(void) {
    CHECK(fixture(0, 0x4000, 0x2000, true) == 0);
    iNESHeader h = header_for(0, 0x4000, true);
    Mapper *previous = cart;
    h.flags7 = 0x08;
    h.prg_ram_size = 1;
    CHECK(rom_mapper_number(&h) == 0x100);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x4000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0xFFFF) == 1);
    h.flags6 = 0x40;
    h.prg_ram_size = 0x20;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x4000, fixture_chr, 0x2000) == -1);
    h.flags7 = 0xFC;
    CHECK(rom_mapper_number(&h) == 4); // Legacy header ignores contaminated upper mapper bits.
    h.flags7 = 0xF0;
    CHECK(rom_mapper_number(&h) == 0xF4);
    CHECK(mapper_init_from_header(NULL, fixture_prg, 0x4000, fixture_chr, 0x2000) == -1);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0, fixture_chr, 0x2000) == -1);
    h = header_for(5, 0x20000, false);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x120000, fixture_chr, 0x2000) == -1);
    uint8_t *large_chr = (uint8_t *)calloc(1, 0x101000);
    CHECK(large_chr != NULL);
    int large_chr_result = mapper_init_from_header(&h, fixture_prg, 0x20000, large_chr, 0x101000);
    free(large_chr);
    CHECK(large_chr_result == -1);
    CHECK(cart == previous);
    return 0;
}

static uint8_t *image_for(const iNESHeader *h, size_t prg_bytes, size_t chr_bytes, size_t *size) {
    size_t trainer_bytes = (h->flags6 & 4) ? 512 : 0;
    *size = sizeof(*h) + trainer_bytes + prg_bytes + chr_bytes;
    uint8_t *image = (uint8_t *)calloc(1, *size);
    if (!image) return NULL;
    memcpy(image, h, sizeof(*h));
    for (size_t i = 0; i < trainer_bytes; ++i) image[sizeof(*h) + i] = (uint8_t)i;
    memset(image + sizeof(*h) + trainer_bytes, 0x5C, prg_bytes);
    memset(image + sizeof(*h) + trainer_bytes + prg_bytes, 0xA5, chr_bytes);
    return image;
}

static int test_loader_trainers_and_sizes(void) {
    iNESHeader h = header_for(0, 0x4000, true);
    h.flags6 |= 4;
    size_t size;
    uint8_t *image = image_for(&h, 0x4000, 0, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    memset(image, 0, size);
    free(image);
    CHECK(loaded == 0 && prg_size == 0x4000);
    CHECK(cart_cpu_read(0x8000) == 0x5C && cart_cpu_read(0xC000) == 0x5C);
    CHECK(cart_cpu_read(0x7001) == 1 && cart_cpu_read(0x71FF) == 0xFF);
    CHECK(cart_cpu_read(0x7200) == 0);

    h = header_for(0, 0x4000, false);
    h.flags7 = 0x08;
    h.flags9 = 0xF0;
    h.chr_rom_chunks = 12 << 2; // 2^12 bytes of CHR-ROM.
    image = image_for(&h, 0x4000, 0x1000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && chr_size == 0x1000);
    CHECK(cart_ppu_read(0) == 0xA5 && cart_ppu_read(0x1FFF) == 0xA5);
    cart_ppu_write(0x1FFF, 0x55);
    CHECK(cart_ppu_read(0x0FFF) == 0xA5);

    h.flags9 = 0x10;
    h.chr_rom_chunks = 0; // The extended count makes this ROM, not RAM.
    image = image_for(&h, 0x4000, 0x200000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && chr_size == 0x200000);
    cart_ppu_write(0, 0x55);
    CHECK(cart_ppu_read(0) == 0xA5);

    h = header_for(13, 0x8000, true);
    image = image_for(&h, 0x8000, 0, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && chr_size == 0x4000);
    for (unsigned i = 0; i < 4; ++i) {
        cart_cpu_write(0x8000, (uint8_t)i);
        cart_ppu_write(0x1000, (uint8_t)(i + 1));
    }
    for (unsigned i = 0; i < 4; ++i) {
        cart_cpu_write(0x8000, (uint8_t)i);
        CHECK(cart_ppu_read(0x1000) == i + 1);
    }
    h = header_for(0, 0x4000, true);
    h.flags7 = 0x08;
    h.zero[0] = 6; // 4KB CHR-RAM, mirrored through the PPU's 8KB window.
    image = image_for(&h, 0x4000, 0, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && chr_size == 0x1000);
    cart_ppu_write(0x1000, 0x91);
    CHECK(cart_ppu_read(0) == 0x91);

    // Trainer bytes are loaded at the CPU-visible $7000-$71FF window even on
    // the two-socket MMC5 RAM layout.
    h = header_for(5, 0x20000, false);
    h.flags7 = 8;
    h.flags6 |= 0x06;
    h.flags10 = 0x77;
    image = image_for(&h, 0x20000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && cart_cpu_read(0x7001) == 1 && cart_cpu_read(0x71FF) == 0xFF);
    cart_cpu_write(0x5113, 4);
    CHECK(cart_cpu_read(0x7000) == 0 && cart_cpu_read(0x7001) == 0 && cart_cpu_read(0x71FF) == 0);
    return 0;
}

static int test_loader_rejection_preserves_cart(void) {
    iNESHeader h = header_for(0, 0x4000, true);
    size_t size;
    uint8_t *image = image_for(&h, 0x4000, 0, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0);
    uint8_t *previous_prg = prg_rom;
    uint8_t *previous_chr = chr_rom;
    iNESHeader previous_header = ines_header;
    CHECK(load_rom_memory(NULL, 0) == -1);
    CHECK(load_rom_memory((const uint8_t *)&h, sizeof(h) - 1) == -1);
    CHECK(load_rom_memory((const uint8_t *)&h, sizeof(h)) == -1);
    h.signature[0] = 0;
    CHECK(load_rom_memory((const uint8_t *)&h, sizeof(h)) == -1);
    h.signature[0] = 'N';
    h.flags6 = 4;
    CHECK(load_rom_memory((const uint8_t *)&h, sizeof(h)) == -1);
    h.flags6 = 0;
    h.flags7 = 0x08;
    h.flags9 = 0x0F;
    h.prg_rom_chunks = 0xFF; // 7 * 2^63 overflows size_t on 64-bit hosts.
    CHECK(load_rom_memory((const uint8_t *)&h, sizeof(h)) == -1);
    h.prg_rom_chunks = 32 << 2; // Representable on 64-bit, but no 4GB payload follows.
    CHECK(load_rom_memory((const uint8_t *)&h, sizeof(h)) == -1);
    h = header_for(0, 0x4000, false);
    image = image_for(&h, 0x4000, 0, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size); // Header declares missing CHR bytes.
    free(image);
    CHECK(loaded == -1);
    h = header_for(0, 0x4000, true);
    h.flags7 = 8;
    h.prg_ram_size = 1;
    image = image_for(&h, 0x4000, 0, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size); // Mapper $100 must not alias mapper zero.
    free(image);
    CHECK(loaded == -1);
    CHECK(prg_rom == previous_prg && chr_rom == previous_chr);
    CHECK(memcmp(&ines_header, &previous_header, sizeof(ines_header)) == 0);
    CHECK(cart_cpu_read(0xFFFF) == 0x5C);
    return 0;
}

static int test_loader_region_and_console_type(void) {
    iNESHeader h = header_for(0, 0x4000, false);
    h.flags9 = 1;
    size_t size;
    uint8_t *image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && nes_timing()->region == NES_REGION_PAL);

    h = header_for(0, 0x4000, false);
    h.flags7 = 0x04; // Archaic iNES: later header bytes are unreliable padding.
    h.flags9 = 1;
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && nes_timing()->region == NES_REGION_NTSC);

    h = header_for(0, 0x4000, false);
    h.flags7 = 0x08;
    h.zero[1] = 3;
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && nes_timing()->region == NES_REGION_DENDY);
    Mapper *previous_cart = cart;
    uint8_t *previous_prg = prg_rom;

    h.flags7 = 0x09; // VS System is not implemented by the console core.
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == -1 && cart == previous_cart && prg_rom == previous_prg);
    CHECK(nes_timing()->region == NES_REGION_DENDY);

    h.flags7 = 0x0B;
    h.zero[2] = 4; // An extended-console expansion not provided by this core.
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == -1 && cart == previous_cart && nes_timing()->region == NES_REGION_DENDY);

    h.zero[1] = 2; // Dual-compatible cartridges default to NTSC timing.
    h.zero[2] = 0; // Extended console type 0 is the ordinary NES family.
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && nes_timing()->region == NES_REGION_NTSC);

    h = header_for(0, 0x4000, false);
    h.flags7 = 2; // PlayChoice-10 in a clean legacy header.
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == -1 && nes_timing()->region == NES_REGION_NTSC);
    unload_rom();
    CHECK(nes_timing()->region == NES_REGION_NTSC);
    return 0;
}

static int test_ram_header_sizes(void) {
    RomRamSizes ram;
    iNESHeader h = header_for(0, 0x4000, true);
    CHECK(rom_ram_sizes(&h, &ram) == 0);
    CHECK(ram.prg_ram == 0x2000 && ram.prg_nvram == 0);
    CHECK(ram.chr_ram == 0x2000 && ram.chr_nvram == 0);
    h.flags6 |= 2;
    h.prg_ram_size = 4;
    CHECK(rom_ram_sizes(&h, &ram) == 0);
    CHECK(ram.prg_ram == 0 && ram.prg_nvram == 0x8000);
    h = header_for(13, 0x8000, true);
    CHECK(rom_ram_sizes(&h, &ram) == 0 && ram.chr_ram == 0x4000);
    h = header_for(5, 0x20000, true);
    CHECK(rom_ram_sizes(&h, &ram) == 0 && ram.prg_ram == 0x10000 && ram.prg_nvram == 0);
    h.flags6 |= 2;
    CHECK(rom_ram_sizes(&h, &ram) == 0 && ram.prg_ram == 0 && ram.prg_nvram == 0x10000);

    h.flags7 = 8;
    h.prg_ram_size = 0x50; // Byte 8 is mapper metadata in NES 2.0.
    for (unsigned shift = 0; shift < 16; ++shift) {
        h.flags10 = (uint8_t)(shift | (shift << 4));
        h.zero[0] = h.flags10;
        size_t expected = shift ? (size_t)64 << shift : 0;
        CHECK(rom_ram_sizes(&h, &ram) == 0);
        CHECK(ram.prg_ram == expected && ram.prg_nvram == expected);
        CHECK(ram.chr_ram == expected && ram.chr_nvram == expected);
    }
    CHECK(rom_ram_sizes(NULL, &ram) == -1 && rom_ram_sizes(&h, NULL) == -1);
    return 0;
}

static int test_prg_ram_capacity(void) {
    iNESHeader h = header_for(0, 0x4000, true);
    h.flags7 = 8;
    h.zero[0] = 7;
    h.flags10 = 1; // 128 bytes, not an 8KB allocation.
    CHECK(fixture_with_header(&h, 0x4000, 0x2000) == 0);
    cart_cpu_write(0x6000, 0xA6);
    cart_cpu_write(0x607F, 0x56);
    CHECK(cart_cpu_read(0x6080) == 0xA6 && cart_cpu_read(0x7FFF) == 0x56);
    cart_cpu_write(0x7F80, 0x91);
    CHECK(cart_cpu_read(0x6000) == 0x91);
    h.flags10 = 0;
    CHECK(fixture_with_header(&h, 0x4000, 0x2000) == 0);
    cart_cpu_write(0x6000, 0xA6);
    CHECK(cart_cpu_read(0x6000) == 0xFF && cart_cpu_read(0x7FFF) == 0xFF);
    h.flags10 = 7;
    CHECK(fixture_with_header(&h, 0x4000, 0x2000) == 0);
    cart_cpu_write(0x6000, 0xA6);
    CHECK(cart_cpu_read(0x6080) == 0);

    h.flags10 = 8; // NROM has no banking for a declared 16KB PRG RAM.
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x4000, fixture_chr, 0x2000) == -1);
    CHECK(cart_cpu_read(0x6000) == 0xA6);
    h.flags10 = 0x77;
    h.flags6 |= 2;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x4000, fixture_chr, 0x2000) == -1);
    CHECK(cart_cpu_read(0x6000) == 0xA6);
    return 0;
}

static int test_mmc1_banked_ram(void) {
    iNESHeader h = header_for(1, 0x20000, true);
    h.prg_ram_size = 4;
    h.flags6 |= 2;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    for (unsigned bank = 0; bank < 4; ++bank) {
        serial_write(0xA000, (uint8_t)(bank << 2));
        cart_cpu_write(0x6000, (uint8_t)(0xA0 + bank));
        cart_cpu_write(0x7FFF, (uint8_t)(0xB0 + bank));
    }
    for (unsigned bank = 0; bank < 4; ++bank) {
        serial_write(0xA000, (uint8_t)(bank << 2));
        CHECK(cart_cpu_read(0x6000) == 0xA0 + bank && cart_cpu_read(0x7FFF) == 0xB0 + bank);
    }
    serial_write(0x8000, 0x1C);
    serial_write(0xC000, 4);
    CHECK(cart_cpu_read(0x6000) == 0xA1);
    serial_write(0x8000, 0x0C); // In 8KB CHR mode only CHR0 supplies RAM bank bits.
    CHECK(cart_cpu_read(0x6000) == 0xA3);
    serial_write(0xE000, 0x10);
    cart_cpu_write(0x6000, 0x55);
    CHECK(cart_cpu_read(0x6000) == 0xFF);
    serial_write(0xE000, 0);
    CHECK(cart_cpu_read(0x6000) == 0xA3);

    h.prg_ram_size = 2;
    h.flags6 &= (uint8_t)~2u;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    cart_cpu_write(0x6000, 1);
    serial_write(0xA000, 4);
    cart_cpu_write(0x6000, 2);
    serial_write(0xA000, 8);
    CHECK(cart_cpu_read(0x6000) == 1);
    serial_write(0xA000, 4);
    CHECK(cart_cpu_read(0x6000) == 2);

    h.flags7 = 8;
    h.prg_ram_size = 0;
    h.flags6 |= 2;
    h.flags10 = 0x77; // SOROM: one 8KB work chip and one 8KB battery chip.
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    cart_cpu_write(0x6000, 0x35);
    serial_write(0xA000, 8);
    cart_cpu_write(0x6000, 0x73);
    serial_write(0xA000, 4); // Bit 2 does not select the second SOROM chip.
    CHECK(cart_cpu_read(0x6000) == 0x35);
    serial_write(0x8000, 0x1C);
    serial_write(0xC000, 8);
    CHECK(cart_cpu_read(0x6000) == 0x73);
    serial_write(0x8000, 0x0C);
    CHECK(cart_cpu_read(0x6000) == 0x35);
    return 0;
}

static void mmc5_unlock_ram(void) {
    cart_cpu_write(0x5102, 2);
    cart_cpu_write(0x5103, 1);
}

static int test_mmc5_banked_ram(void) {
    iNESHeader h = header_for(5, 0x20000, true);
    h.flags7 = 8;
    h.zero[0] = 7;
    const uint8_t sizes[] = {7, 9, 10, 11}; // 8KB, 32KB, 64KB, 128KB chips.
    for (size_t chip = 0; chip < sizeof(sizes); ++chip) {
        for (unsigned persistent = 0; persistent < 2; ++persistent) {
            h.flags10 = (uint8_t)(sizes[chip] << (persistent ? 4 : 0));
            h.flags6 = (uint8_t)(0x50 | (persistent ? 2 : 0));
            CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 5);
            mmc5_unlock_ram();
            unsigned banks = (unsigned)(((size_t)64 << sizes[chip]) / 0x2000);
            for (unsigned bank = 0; bank < banks; ++bank) {
                cart_cpu_write(0x5113, (uint8_t)bank);
                cart_cpu_write(0x6000, (uint8_t)(0x90 + bank));
                cart_cpu_write(0x7FFF, (uint8_t)(0xB0 + bank));
            }
            for (unsigned bank = 0; bank < banks; ++bank) {
                cart_cpu_write(0x5113, (uint8_t)bank);
                CHECK(cart_cpu_read(0x6000) == 0x90 + bank);
                CHECK(cart_cpu_read(0x7FFF) == 0xB0 + bank);
                cart_cpu_write(0x5114, (uint8_t)bank);
                CHECK(cart_cpu_read(0x8000) == 0x90 + bank);
                cart_cpu_write(0x8000, (uint8_t)(0xC0 + bank));
                CHECK(cart_cpu_read(0x6000) == 0xC0 + bank);
            }
            if (banks <= 4) {
                cart_cpu_write(0x5113, 4);
                CHECK(cart_cpu_read(0x6000) == 0xFF); // Unpopulated second socket.
                cart_cpu_write(0x5114, 4);
                cart_cpu_write(0x8000, 0x77);
                CHECK(cart_cpu_read(0x8000) == 0xFF);
                cart_cpu_write(0x5113, 0);
                CHECK(cart_cpu_read(0x6000) == 0xC0);
            }
            if (banks == 1) {
                cart_cpu_write(0x5113, 3);
                CHECK(cart_cpu_read(0x6000) == 0xC0);
            } else {
                cart_cpu_write(0x5100, 1);
                cart_cpu_write(0x5115, 3); // Ignore low bit for a 16KB window.
                CHECK(cart_cpu_read(0x8000) == 0xC2 && cart_cpu_read(0xA000) == 0xC3);
                cart_cpu_write(0xBFFF, 0x65);
                cart_cpu_write(0x5113, 3);
                CHECK(cart_cpu_read(0x7FFF) == 0x65);
                cart_cpu_write(0x5100, 2);
                cart_cpu_write(0x5116, 1);
                CHECK(cart_cpu_read(0xC000) == 0xC1);
                cart_cpu_write(0x5103, 0);
                cart_cpu_write(0xC000, 0x55);
                CHECK(cart_cpu_read(0xC000) == 0xC1);
            }
        }
    }
    h.flags6 = 0x52;
    h.flags10 = 0x77;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 5);
    mmc5_unlock_ram();
    cart_cpu_write(0x6000, 0x12);
    cart_cpu_write(0x5113, 4);
    cart_cpu_write(0x6000, 0x34);
    for (unsigned bank = 0; bank < 8; ++bank) {
        cart_cpu_write(0x5113, (uint8_t)bank);
        CHECK(cart_cpu_read(0x6000) == (bank < 4 ? 0x12 : 0x34));
    }
    h = header_for(5, 0x20000, true);
    h.prg_ram_size = 8; // Legacy header: eight linearly banked 8KB pages.
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 5);
    mmc5_unlock_ram();
    for (unsigned bank = 0; bank < 8; ++bank) {
        cart_cpu_write(0x5113, (uint8_t)bank);
        cart_cpu_write(0x6000, (uint8_t)(bank + 1));
    }
    for (unsigned bank = 0; bank < 8; ++bank) {
        cart_cpu_write(0x5113, (uint8_t)bank);
        CHECK(cart_cpu_read(0x6000) == bank + 1);
    }
    return 0;
}

static int test_loader_ram_layouts(void) {
    iNESHeader h = header_for(1, 0x4000, true);
    h.flags7 = 8;
    h.flags10 = 9;
    h.zero[0] = 7;
    size_t size;
    uint8_t *image = image_for(&h, 0x4000, 0, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0);
    serial_write(0xA000, 12);
    cart_cpu_write(0x6000, 0xA7);
    uint8_t *previous_prg = prg_rom;
    uint8_t *previous_chr = chr_rom;
    static const struct { uint8_t prg, chr, rom, flags; } invalid[] = {
        {10, 7, 0, 0}, // More RAM than MMC1 can address.
        {0x99, 7, 0, 2}, // Separate large RAM chips lack board selection.
        {7, 0x77, 0, 2}, // Mixed volatile/nonvolatile CHR chips.
        {7, 7, 1, 0}, // CHR-ROM plus CHR-RAM is not wired by these mappers.
        {7, 0, 0, 0}, // NES 2.0 explicitly declares no CHR memory.
        {0x70, 7, 0, 0} // NVRAM requires the battery flag.
    };
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        h.flags10 = invalid[i].prg;
        h.zero[0] = invalid[i].chr;
        h.chr_rom_chunks = invalid[i].rom;
        h.flags6 = (uint8_t)(0x10 | invalid[i].flags);
        image = image_for(&h, 0x4000, invalid[i].rom ? 0x2000 : 0, &size);
        CHECK(image != NULL);
        loaded = load_rom_memory(image, size);
        free(image);
        CHECK(loaded == -1 && prg_rom == previous_prg && chr_rom == previous_chr);
        CHECK(cart_cpu_read(0x6000) == 0xA7);
    }
    return 0;
}

typedef struct {
    char directory[96];
    char rom[128], prg_save[128], chr_save[128];
} SaveFixture;

static int save_fixture_begin(SaveFixture *paths) {
    for (unsigned attempt = 0; attempt < 1000; ++attempt) {
        snprintf(paths->directory, sizeof(paths->directory), ".mapper-ram-test-%llu-%u",
                 (unsigned long long)time(NULL), attempt);
#ifdef _WIN32
        int result = _mkdir(paths->directory);
#else
        int result = mkdir(paths->directory, 0700);
#endif
        if (result == 0) {
            snprintf(paths->rom, sizeof(paths->rom), "%s/cart.nes", paths->directory);
            snprintf(paths->prg_save, sizeof(paths->prg_save), "%s/cart.sav", paths->directory);
            snprintf(paths->chr_save, sizeof(paths->chr_save), "%s/cart.chr.sav", paths->directory);
            return 0;
        }
        if (errno != EEXIST) return -1;
    }
    return -1;
}

static int save_fixture_end(const SaveFixture *paths) {
    cart_battery_shutdown();
    int result = 0;
    if (remove(paths->prg_save) != 0 && errno != ENOENT) result = 1;
    if (remove(paths->chr_save) != 0 && errno != ENOENT) result = 1;
#ifdef _WIN32
    if (_rmdir(paths->directory) != 0) result = 1;
#else
    if (rmdir(paths->directory) != 0) result = 1;
#endif
    return result;
}

static long saved_file_size(const char *path) {
    FILE *fp = fopen(path, "rb");
    if (!fp) return -1;
    long size = fseek(fp, 0, SEEK_END) == 0 ? ftell(fp) : -1;
    fclose(fp);
    return size;
}

static int saved_byte(const char *path, long offset) {
    FILE *fp = fopen(path, "rb");
    if (!fp) return -1;
    int value = fseek(fp, offset, SEEK_SET) == 0 ? fgetc(fp) : -1;
    fclose(fp);
    return value;
}

static int prg_persistence_cases(const SaveFixture *paths) {
    uint8_t legacy_save[0x2000];
    for (size_t i = 0; i < sizeof(legacy_save); ++i) legacy_save[i] = (uint8_t)(i ^ 0xA5);
    FILE *fp = fopen(paths->prg_save, "wb");
    CHECK(fp != NULL);
    size_t written = fwrite(legacy_save, 1, sizeof(legacy_save), fp);
    int closed = fclose(fp);
    CHECK(written == sizeof(legacy_save) && closed == 0);
    iNESHeader h = header_for(1, 0x20000, true);
    h.prg_ram_size = 4;
    h.flags6 |= 2;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    cart_battery_configure(paths->rom, true);
    CHECK(cart_cpu_read(0x6000) == 0xA5 && cart_cpu_read(0x7FFF) == 0x5A);
    serial_write(0xA000, 12);
    CHECK(cart_cpu_read(0x6000) == 0 && cart_cpu_read(0x7FFF) == 0);
    cart_cpu_write(0x6000, 0x73);
    cart_cpu_write(0x7FFF, 0x39);
    cart_ppu_write(0, 0x22); // Legacy CHR-RAM is volatile.
    cart_battery_flush();
    CHECK(saved_file_size(paths->prg_save) == 0x8000);
    CHECK(saved_file_size(paths->chr_save) == -1);
    uint8_t restored_prefix[sizeof(legacy_save)];
    fp = fopen(paths->prg_save, "rb");
    CHECK(fp != NULL);
    size_t bytes_read = fread(restored_prefix, 1, sizeof(restored_prefix), fp);
    closed = fclose(fp);
    CHECK(bytes_read == sizeof(restored_prefix) && closed == 0);
    CHECK(memcmp(restored_prefix, legacy_save, sizeof(legacy_save)) == 0);
    CHECK(saved_byte(paths->prg_save, 0x6000) == 0x73 && saved_byte(paths->prg_save, 0x7FFF) == 0x39);
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    cart_battery_configure(paths->rom, true);
    serial_write(0xA000, 12);
    CHECK(cart_cpu_read(0x6000) == 0x73 && cart_cpu_read(0x7FFF) == 0x39);

    h.flags7 = 8;
    h.prg_ram_size = 0;
    h.flags10 = 0x77;
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    cart_battery_configure(paths->rom, true);
    cart_cpu_write(0x6000, 0x66);
    serial_write(0xA000, 8);
    cart_cpu_write(0x6000, 0x99);
    cart_battery_flush();
    CHECK(saved_file_size(paths->prg_save) == 0x2000);
    CHECK(saved_byte(paths->prg_save, 0) == 0x66);
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    cart_battery_configure(paths->rom, true);
    CHECK(cart_cpu_read(0x6000) == 0x66);
    serial_write(0xA000, 8);
    CHECK(cart_cpu_read(0x6000) == 0); // The work chip was not serialized.
    return 0;
}

static int test_prg_nvram_persistence(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    int result = prg_persistence_cases(&paths);
    return result | save_fixture_end(&paths);
}

static int mmc5_persistence_cases(const SaveFixture *paths) {
    iNESHeader h = header_for(5, 0x20000, true);
    h.flags6 |= 2;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 5);
    cart_battery_configure(paths->rom, true);
    cart_cpu_write(0x5102, 2);
    cart_cpu_write(0x5103, 1);
    cart_cpu_write(0x5113, 0);
    cart_cpu_write(0x6000, 0x35);
    cart_cpu_write(0x5104, 2);
    cart_cpu_write(0x5C00, 0x53);
    cart_cpu_write(0x5FFF, 0xA7);
    cart_battery_flush();
    CHECK(saved_file_size(paths->prg_save) == 0x10400);
    CHECK(saved_byte(paths->prg_save, 0) == 0x35);
    CHECK(saved_byte(paths->prg_save, 0x10000) == 0x53);
    CHECK(saved_byte(paths->prg_save, 0x103FF) == 0xA7);

    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 5);
    cart_battery_configure(paths->rom, true);
    cart_cpu_write(0x5113, 0);
    cart_cpu_write(0x5104, 2);
    CHECK(cart_cpu_read(0x6000) == 0x35);
    CHECK(cart_cpu_read(0x5C00) == 0x53 && cart_cpu_read(0x5FFF) == 0xA7);

    // Old saves may end in PRG RAM or partway through the appended ExRAM.
    const size_t short_sizes[] = {3, 0x10002};
    for (unsigned i = 0; i < sizeof(short_sizes) / sizeof(short_sizes[0]); ++i) {
        cart_battery_shutdown();
        uint8_t *contents = (uint8_t *)calloc(1, short_sizes[i]);
        CHECK(contents != NULL);
        contents[0] = 0x42;
        contents[short_sizes[i] - 1] = 0xA3;
        FILE *fp = fopen(paths->prg_save, "wb");
        if (!fp) { free(contents); return 1; }
        size_t written = fwrite(contents, 1, short_sizes[i], fp);
        int closed = fclose(fp);
        free(contents);
        CHECK(written == short_sizes[i] && closed == 0);
        CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 5);
        cart_battery_configure(paths->rom, true);
        cart_cpu_write(0x5113, 0);
        cart_cpu_write(0x5104, 2);
        CHECK(cart_cpu_read(0x6000) == 0x42 && cart_cpu_read(0x7FFF) == 0);
        CHECK(cart_cpu_read(0x6002) == (i ? 0 : 0xA3));
        CHECK(cart_cpu_read(0x5C01) == (i ? 0xA3 : 0));
        CHECK(cart_cpu_read(0x5C02) == 0 && cart_cpu_read(0x5FFF) == 0);
    }
    return 0;
}

static int test_mmc5_persistence(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    int result = mmc5_persistence_cases(&paths);
    return result | save_fixture_end(&paths);
}

static int chr_persistence_cases(const SaveFixture *paths) {
    iNESHeader h = header_for(1, 0x20000, true);
    h.flags7 = 8;
    h.flags6 |= 2;
    h.flags10 = 7;
    h.zero[0] = 0x90; // 32KB CHR-NVRAM, plus a separate volatile PRG chip.
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 1);
    cart_battery_configure(paths->rom, true);
    serial_write(0x8000, 0x1C);
    for (unsigned bank = 0; bank < 8; ++bank) {
        serial_write(0xA000, (uint8_t)bank);
        cart_ppu_write(0, (uint8_t)(0x30 + bank));
        cart_ppu_write(0x0FFF, (uint8_t)(0x50 + bank));
    }
    cart_cpu_write(0x6000, 0xA6);
    cart_battery_flush();
    CHECK(saved_file_size(paths->prg_save) == -1);
    CHECK(saved_file_size(paths->chr_save) == 0x8000);
    CHECK(saved_byte(paths->chr_save, 0x7000) == 0x37 && saved_byte(paths->chr_save, 0x7FFF) == 0x57);
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 1);
    cart_battery_configure(paths->rom, true);
    CHECK(cart_cpu_read(0x6000) == 0);
    serial_write(0x8000, 0x1C);
    for (unsigned bank = 0; bank < 8; ++bank) {
        serial_write(0xA000, (uint8_t)bank);
        CHECK(cart_ppu_read(0) == 0x30 + bank && cart_ppu_read(0x0FFF) == 0x50 + bank);
    }

    // Replacing a loaded ROM must flush both old chips before freeing their storage.
    h.flags10 = 0x70;
    h.zero[0] = 0x70;
    size_t image_size;
    uint8_t *image = image_for(&h, 0x20000, 0, &image_size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, image_size);
    free(image);
    CHECK(loaded == 0);
    cart_battery_configure(paths->rom, true);
    cart_cpu_write(0x6000, 0x8D);
    cart_ppu_write(0, 0xD8);
    h = header_for(0, 0x4000, true);
    image = image_for(&h, 0x4000, 0, &image_size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, image_size);
    free(image);
    CHECK(loaded == 0 && cart_cpu_read(0x6000) == 0 && cart_ppu_read(0) == 0);
    CHECK(saved_file_size(paths->prg_save) == 0x2000);
    CHECK(saved_file_size(paths->chr_save) == 0x2000);
    CHECK(saved_byte(paths->prg_save, 0) == 0x8D && saved_byte(paths->chr_save, 0) == 0xD8);
    return 0;
}

static int test_chr_nvram_persistence(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    int result = chr_persistence_cases(&paths);
    return result | save_fixture_end(&paths);
}

static int chr_writer_cases(const SaveFixture *paths) {
    const unsigned boards[] = {0, 1, 2, 3, 4, 5, 7, 9, 10, 11, 13, 15};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        iNESHeader h = header_for(boards[i], 0x8000, true);
        h.flags7 |= 8;
        h.flags6 |= 2;
        h.zero[0] = boards[i] == 13 ? 0x80 : 0x70;
        size_t bytes = boards[i] == 13 ? 0x4000 : 0x2000;
        CHECK(fixture_with_header(&h, 0x8000, bytes) == (int)boards[i]);
        cart_battery_configure(paths->rom, true);
        if (boards[i] == 15) cart_cpu_write(0x8002, 0);
        cart_ppu_write(7, 0xA6);
        CHECK(cart_ppu_read(7) == 0xA6);
        cart_battery_flush();
        CHECK(saved_file_size(paths->chr_save) == (long)bytes);
        CHECK(saved_byte(paths->chr_save, 7) == 0xA6);
        CHECK(saved_file_size(paths->prg_save) == -1);
        cart_battery_shutdown();
        CHECK(remove(paths->chr_save) == 0);
    }
    return 0;
}

static int test_chr_nvram_writers(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    int result = chr_writer_cases(&paths);
    return result | save_fixture_end(&paths);
}

static iNESHeader mmc6_header(bool battery) {
    iNESHeader h = header_for(4, 0x20000, false);
    h.flags7 = 8;
    h.prg_ram_size = 0x10;
    h.flags10 = battery ? 0x40 : 4;
    if (battery) h.flags6 |= 2;
    return h;
}

static int test_mmc6_ram_mirroring(void) {
    iNESHeader h = mmc6_header(false);
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_cpu_write(0xA001, 0xF0); // Writes are ignored until global RAM enable.
    cart_cpu_write(0x8000, 0x20);
    CHECK(cart_cpu_read_bus(0x7000, 0xA6) == 0xA6);
    cart_cpu_write(0xA001, 0xF0);
    for (unsigned offset = 0; offset < 0x400; ++offset)
        cart_cpu_write((uint16_t)(0x7000 + offset), (uint8_t)(offset ^ (offset >> 1) ^ 0x5A));
    for (unsigned offset = 0; offset < 0x1000; ++offset) {
        unsigned index = offset & 0x3FF;
        uint8_t expected = (uint8_t)(index ^ (index >> 1) ^ 0x5A);
        CHECK(cart_cpu_read_bus((uint16_t)(0x7000 + offset), 0xA6) == expected);
        CHECK(cart_cpu_read_bus((uint16_t)(0x6000 + offset), 0xA6) == 0xA6);
        cart_cpu_write((uint16_t)(0x6000 + offset), 0);
    }
    CHECK(cart_cpu_read(0x7000) == 0x5A);
    cart_cpu_write(0x7FFF, 0x91);
    CHECK(cart_cpu_read(0x73FF) == 0x91 && cart_cpu_read(0x77FF) == 0x91);
    cart_cpu_write(0x8000, 0);
    CHECK(cart_cpu_read_bus(0x7000, 0xD3) == 0xD3);
    cart_cpu_write(0xA001, 0xF0);
    cart_cpu_write(0x8000, 0x20);
    CHECK(cart_cpu_read_bus(0x7000, 0xD3) == 0xD3); // Disabled state cleared the protect latch.
    cart_cpu_write(0xA001, 0xF0);
    CHECK(cart_cpu_read(0x7000) == 0x5A && cart_cpu_read(0x7FFF) == 0x91);
    h.flags10 = 7;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x8000) == -1);
    CHECK(cart_cpu_read(0x7FFF) == 0x91); // MMC6 cannot be expanded to 8KB by a header.
    h.flags10 = 0;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x8000) == -1);
    return 0;
}

static int test_mmc6_protection(void) {
    iNESHeader h = mmc6_header(false);
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_cpu_write(0x8000, 0x20);
    for (unsigned protection = 0; protection <= 0xF0; protection += 0x10) {
        cart_cpu_write(0xA001, 0xF0);
        cart_cpu_write(0x7001, 0x11);
        cart_cpu_write(0x7201, 0x22);
        cart_cpu_write(0xBFFF, (uint8_t)(protection | 0x0F)); // Register mirror; low bits ignored.
        cart_cpu_write(0x7401, 0x33);
        cart_cpu_write(0x7601, 0x44);
        bool first_write = (protection & 0x30) == 0x30;
        bool second_write = (protection & 0xC0) == 0xC0;
        uint8_t first_data = first_write ? 0x33 : 0x11;
        uint8_t second_data = second_write ? 0x44 : 0x22;
        uint8_t first_read = !(protection & 0xA0) ? 0xA6
                           : (protection & 0x20) ? first_data : 0;
        uint8_t second_read = !(protection & 0xA0) ? 0xA6
                            : (protection & 0x80) ? second_data : 0;
        CHECK(cart_cpu_read_bus(0x7C01, 0xA6) == first_read);
        CHECK(cart_cpu_read_bus(0x7E01, 0xA6) == second_read);
        cart_cpu_write(0xA001, 0xF0);
        CHECK(cart_cpu_read(0x7001) == first_data && cart_cpu_read(0x7201) == second_data);
    }
    return 0;
}

static int test_mmc6_banks_and_irq(void) {
    for (unsigned submapper = 0; submapper < 2; ++submapper) {
        iNESHeader h = mmc6_header(false);
        h.prg_ram_size = (uint8_t)(submapper << 4);
        if (!submapper) h.flags10 = 7;
        CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
        cart_cpu_write(0x9FFE, 0x26);
        cart_cpu_write(0x9FFF, 3);
        CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xC000) == 14);
        cart_cpu_write(0x8000, 0x66);
        CHECK(cart_cpu_read(0x8000) == 14 && cart_cpu_read(0xC000) == 3);
        cart_cpu_write(0x8000, 0xA0);
        cart_cpu_write(0x8001, 5);
        CHECK(cart_ppu_read(0x1000) == 4 && cart_ppu_read(0x1400) == 5);
        cart_cpu_write(0xA000, 1);
        CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
        cart_cpu_write(0xA000, 0);
        CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
        cart_cpu_write(0xC000, 1);
        cart_cpu_write(0xC001, 0);
        cart_cpu_write(0xE001, 0);
        a12_pulse(0);
        CHECK(!cart_irq_pending());
        cart_cpu_write(0xC000, 0);
        a12_pulse(12);
        CHECK(cart_irq_pending());
        cart_cpu_write(0xE000, 0);
        cart_cpu_write(0xE001, 0);
        a12_pulse(24);
        CHECK(cart_irq_pending()); // Sharp MMC3 and MMC6 both reassert on a zero reload.
        cart_cpu_write(0xE000, 0);
        cart_cpu_write(0xC001, 0);
        cart_cpu_write(0x8000, 0); // Disabling RAM does not disable the IRQ counter.
        cart_cpu_write(0xE001, 0);
        a12_pulse(36);
        CHECK(cart_irq_pending());
    }
    return 0;
}

static int test_cartridge_bus_reads(void) {
    mapper_shutdown();
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
    CHECK(fixture(0, 0x4000, 0x2000, true) == 0);
    fixture_prg[0] = 0xFF;
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0xFF);
    cart_cpu_write(0x6000, 0xFF);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0xFF);
    CHECK(cart_cpu_read_bus(0x4020, 0x56) == 0x56);
    CHECK(cart_cpu_read_bus(0x5FFF, 0x56) == 0x56);
    CHECK(cart_cpu_read(0x5FFF) == 0xFF); // Bus input from the preceding call does not leak.
    iNESHeader h = header_for(0, 0x4000, true);
    h.flags7 = 8;
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x4000, 0x2000) == 0);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    CHECK(fixture(1, 0x4000, 0x2000, true) == 1);
    cart_cpu_write(0x6000, 0xFF);
    serial_write(0xE000, 0x10);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    serial_write(0xE000, 0);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0xFF);
    CHECK(fixture(4, 0x4000, 0x2000, true) == 4);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    cart_cpu_write(0xA001, 0x80);
    cart_cpu_write(0x6000, 0xFF);
    cart_cpu_write(0xA001, 0xC0);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0xFF);
    cart_cpu_write(0xA001, 0);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);

    h = header_for(5, 0x20000, true);
    h.flags7 = 8;
    h.flags10 = 7;
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 5);
    CHECK(cart_cpu_read_bus(0x5203, 0x56) == 0x56);
    cart_cpu_write(0x5104, 2);
    cart_cpu_write(0x5C00, 0xFF);
    CHECK(cart_cpu_read_bus(0x5C00, 0x56) == 0xFF);
    cart_cpu_write(0x5104, 0);
    CHECK(cart_cpu_read_bus(0x5C00, 0x56) == 0x56);
    cart_cpu_write(0x5205, 0xFF);
    cart_cpu_write(0x5206, 1);
    CHECK(cart_cpu_read_bus(0x5205, 0x56) == 0xFF);
    CHECK(cart_cpu_read_bus(0x5206, 0x56) == 0);
    CHECK(cart_cpu_read_bus(0x5204, 0xFF) == 0x3F);
    cart_cpu_write(0x5203, 1);
    cart_cpu_write(0x5204, 0x80);
    uint8_t mmc5_nt[0x1000] = {0};
    mmc5_enter_frame(mmc5_nt);
    mmc5_next_scanline(mmc5_nt);
    CHECK(cart_irq_pending());
    CHECK(cart_cpu_read_bus(0x5204, 0x96) == 0xD6);
    CHECK(!cart_irq_pending());
    CHECK(cart_cpu_read_bus(0x5204, 0x96) == 0x56);
    cart_cpu_write(0x5113, 4);
    cart_cpu_write(0x5114, 4);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
    cart_cpu_write(0x5114, 0x80);
    fixture_prg[0] = 0xFF;
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0xFF);
    return 0;
}

static int mmc6_persistence_cases(const SaveFixture *paths) {
    iNESHeader h = mmc6_header(true);
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_battery_configure(paths->rom, true);
    cart_cpu_write(0x8000, 0x20);
    cart_cpu_write(0xA001, 0xF0);
    cart_cpu_write(0x7000, 0x35);
    cart_cpu_write(0x7200, 0x53);
    cart_cpu_write(0x7FFF, 0xFF);
    cart_battery_flush();
    CHECK(saved_file_size(paths->prg_save) == 0x400);
    CHECK(saved_byte(paths->prg_save, 0) == 0x35 && saved_byte(paths->prg_save, 0x200) == 0x53);
    CHECK(saved_byte(paths->prg_save, 0x3FF) == 0xFF);
    CHECK(saved_file_size(paths->chr_save) == -1);
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_battery_configure(paths->rom, true);
    cart_cpu_write(0x8000, 0x20);
    cart_cpu_write(0xA001, 0xF0);
    CHECK(cart_cpu_read(0x7400) == 0x35 && cart_cpu_read(0x7600) == 0x53);
    CHECK(cart_cpu_read_bus(0x7FFF, 0x56) == 0xFF);
    return 0;
}

static int test_mmc6_persistence(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    int result = mmc6_persistence_cases(&paths);
    return result | save_fixture_end(&paths);
}

static int test_cartridge_unload(void) {
    iNESHeader h = header_for(0, 0x4000, true);
    size_t image_size;
    uint8_t *image = image_for(&h, 0x4000, 0, &image_size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, image_size);
    free(image);
    CHECK(loaded == 0 && cart != NULL && prg_rom != NULL && chr_rom != NULL);
    unload_rom();
    unload_rom();
    CHECK(cart == NULL && prg_rom == NULL && chr_rom == NULL);
    CHECK(prg_size == 0 && chr_size == 0 && !cart_irq_pending() && mirroring_mode == 0);
    const iNESHeader empty = {0};
    CHECK(memcmp(&ines_header, &empty, sizeof(empty)) == 0);
    CHECK(cart_cpu_read_bus(0x8000, 0xA6) == 0xA6);
    CHECK(fixture(0, 0x4000, 0x2000, true) == 0);
    cart_ppu_write(0, 0xA6);
    unload_rom(); // External fixture arrays remain caller-owned.
    CHECK(fixture_chr[0] == 0xA6 && fixture_prg[0x3FFF] == 1);
    CHECK(fixture(0, 0x4000, 0x2000, true) == 0);
    CHECK(cart_cpu_read(0xFFFF) == 1);
    return 0;
}

int test_mapper_accuracy(void) {
    static int (*const tests[])(void) = {
        test_small_cartridges, test_mmc1_banks_and_ram, test_mmc1_serial_timing,
        test_mmc1_outer_and_fixed_banks, test_mmc2_banks_and_latches,
        test_mmc4_latches_and_chr_ram, test_mmc3_banks_and_protection,
        test_mmc3_irq_edges, test_mmc3_render_trace, test_simple_mapper_registers,
        test_bus_conflict_submappers, test_mapper15_modes, test_mmc5_memory_windows,
        test_mmc5_exram_and_irq, test_mmc5_chr_fetch_modes, test_mmc5_extended_rendering,
        test_mmc5_rendered_ppu_paths, test_mmc5_audio_and_pcm, test_header_and_mapper_rejection,
        test_loader_trainers_and_sizes, test_loader_rejection_preserves_cart,
        test_loader_region_and_console_type,
        test_ram_header_sizes, test_prg_ram_capacity, test_mmc1_banked_ram,
        test_mmc5_banked_ram, test_loader_ram_layouts, test_prg_nvram_persistence,
        test_mmc5_persistence,
        test_chr_nvram_persistence, test_chr_nvram_writers,
        test_mmc6_ram_mirroring, test_mmc6_protection, test_mmc6_banks_and_irq,
        test_cartridge_bus_reads, test_mmc6_persistence, test_cartridge_unload
    };
    int failures = 0;
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) failures += tests[i]();
    unload_rom();
    printf("Mapper accuracy: %zu groups, %d failures\n", sizeof(tests) / sizeof(tests[0]), failures);
    return failures;
}
