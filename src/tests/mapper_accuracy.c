/*
 * mapper_accuracy.c - Cartridge mapper regression tests
 *
 * Author: @frankischilling
 *
 * This file uses synthetic PRG and CHR images to test mapper banking, mirroring, IRQs,
 * RAM layouts, save persistence, bus conflicts, MMC5 features, and loader error handling.
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
#include "../apu/apu.h"
#include "../system/timing.h"
#include "../../include/globals.h"

extern uint64_t cpu_total_cycles;

static uint8_t fixture_prg[0x200000];
static uint8_t fixture_chr[0x80000];
static uint8_t *image_for(const iNESHeader *h, size_t prg_bytes, size_t chr_bytes, size_t *size);

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

static int test_mapper105_competition_board(void) {
    iNESHeader h = header_for(105, 0x40000, true);
    CHECK(cart_set_dip_switches(0));
    CHECK(fixture_with_header(&h, 0x40000, 0x2000) == 105);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 2);

    serial_write(0xA000, 0x00);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 2);
    serial_write(0xA000, 0x10);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 2);
    serial_write(0xA000, 0x06);
    CHECK(cart_cpu_read(0x8000) == 12 && cart_cpu_read(0xC000) == 14);

    serial_write(0xA000, 0x08);
    CHECK(cart_cpu_read(0x8000) == 16 && cart_cpu_read(0xC000) == 30);
    serial_write(0xE000, 3);
    CHECK(cart_cpu_read(0x8000) == 22 && cart_cpu_read(0xC000) == 30);
    serial_write(0x8000, 0x08);
    CHECK(cart_cpu_read(0x8000) == 16 && cart_cpu_read(0xC000) == 22);
    serial_write(0x8000, 0x00);
    CHECK(cart_cpu_read(0x8000) == 20 && cart_cpu_read(0xC000) == 22);

    cart_cpu_write(0x6000, 0xA5);
    CHECK(cart_cpu_read(0x6000) == 0xA5);
    serial_write(0xE000, 0x10);
    CHECK(cart_cpu_read_bus(0x6000, 0x5A) == 0x5A);
    serial_write(0xE000, 0);
    CHECK(cart_cpu_read(0x6000) == 0xA5);

    serial_write(0xA000, 0x00);
    CHECK(!cart_irq_pending());
    cart->clock(0x1FFFFFFF);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_irq_ack();
    cart->clock(100);
    CHECK(!cart_irq_pending());

    serial_write(0xA000, 0x10);
    CHECK(!cart_irq_pending());
    CHECK(cart_set_dip_switches(3));
    serial_write(0xA000, 0x00);
    cart->clock(0x25FFFFFF);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());

    cart->reset();
    CHECK(!cart_irq_pending() && cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 2);
    CHECK(cart_set_dip_switches(0));

    size_t image_size;
    uint8_t *image = image_for(&h, 0x40000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    free(image);
    uint8_t *previous = prg_rom;
    uint8_t previous_value = cart_cpu_read(0x8000);
    h.flags7 |= 0x08;
    h.prg_ram_size = 0x10;
    h.zero[0] = 7;
    image = image_for(&h, 0x40000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == -1);
    free(image);
    CHECK(prg_rom == previous && cart_cpu_read(0x8000) == previous_value);
    return 0;
}

static int test_mapper105_fixed_chr_and_serial_timing(void) {
    CHECK(fixture(105, 0x40000, 0x2000, true) == 105);
    cart_ppu_write(0x0003, 0xA5);
    cart_ppu_write(0x1003, 0x5A);
    serial_write(0x8000, 0x1C);
    serial_write(0xA000, 3);
    serial_write(0xC000, 0);
    CHECK(cart_ppu_read(0x0003) == 0xA5 && cart_ppu_read(0x1003) == 0x5A);
    cart_ppu_write(0x0003, 0x7B);
    CHECK(fixture_chr[3] == 0x7B && fixture_chr[0x1003] == 0x5A);

    CHECK(cart_set_dip_switches(0));
    cart->reset();
    serial_write(0xA000, 0);
    cart->clock(0x20000000);
    CHECK(cart_irq_pending());
    cart_irq_ack();
    ++cpu_total_cycles;
    cart_cpu_write(0xC000, 0);
    cart->clock(1);
    CHECK(!cart_irq_pending()); // The consecutive write must not shift or restart the timer.
    for (unsigned bit = 0; bit < 4; ++bit) {
        cpu_total_cycles += 2;
        cart_cpu_write(0xC000, 0);
        cart->clock(1);
        CHECK(!cart_irq_pending());
    }
    cpu_total_cycles += 2;
    cart_cpu_write(0xC000, 0);
    cart->clock(1);
    CHECK(cart_irq_pending()); // A completed register write updates timer control.
    serial_write(0xA000, 0x10);
    CHECK(!cart_irq_pending());
    cpu_total_cycles += 2;
    cart_cpu_write(0x8000, 0x80);
    CHECK(!cart_irq_pending() && cart_ppu_read(0x0003) == 0x7B);

    CHECK(fixture(105, 0x40000, 0x2000, false) == 105);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    serial_write(0xA000, 0x1F);
    cart_ppu_write(0x0123, 0x69);
    CHECK(cart_ppu_read(0x0123) == 0x23 && fixture_chr[0x0123] == 0);
    return 0;
}

static int test_mapper105_dips_and_cpu_irq(void) {
    CHECK(fixture(105, 0x40000, 0x2000, true) == 105);
    for (unsigned dips = 0; dips < 16; ++dips) {
        CHECK(cart_set_dip_switches(dips));
        CHECK(!cart_set_dip_switches(256) && cart_dip_switches() == dips);
        cart->reset();
        serial_write(0xA000, 0);
        uint32_t limit = 0x20000000u | (dips << 25);
        cart->clock((int)(limit - 1));
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
    }
    cart->reset();
    CHECK(cart_set_dip_switches(15));
    serial_write(0xA000, 0);
    cart->clock(0x21000000);
    CHECK(!cart_irq_pending());
    CHECK(cart_set_dip_switches(0));
    cart->clock(1);
    CHECK(cart_irq_pending());

    cart->reset();
    fixture_prg[0x7FFC] = 0;
    fixture_prg[0x7FFD] = 2;
    fixture_prg[0x7FFE] = 0;
    fixture_prg[0x7FFF] = 3;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu) && cpu.pc == 0x0200);
    write_mem(0x0200, 0xEA);
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    serial_write(0xA000, 0);
    cart->clock(0x1FFFFFFF);
    CHECK(cpu_step(&cpu) == 9 && cpu.pc == 0x0300 && cart_irq_pending());
    serial_write(0xA000, 0x10);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_mapper232_multicart_banks(void) {
    iNESHeader h = header_for(232, 0x40000, true);
    for (unsigned submapper = 0; submapper <= 1; ++submapper) {
        if (submapper) {
            h.flags7 |= 0x08;
            h.prg_ram_size = 0x10;
            h.flags10 = 7;
            h.zero[0] = 7;
        }
        CHECK(fixture_with_header(&h, 0x40000, 0x2000) == 232);
        CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 6);
        static const uint8_t standard_values[4] = {0x00, 0x08, 0x10, 0x18};
        static const uint8_t swapped_values[4] = {0x00, 0x10, 0x08, 0x18};
        for (unsigned block = 0; block < 4; ++block) {
            uint8_t outer = submapper ? swapped_values[block] : standard_values[block];
            cart_cpu_write(0x8000, (uint8_t)(outer | 0xE7));
            for (unsigned page = 0; page < 4; ++page) {
                cart_cpu_write(0xC000, (uint8_t)(0xFC | page));
                CHECK(cart_cpu_read(0x8000) == 2u * (block * 4u + page));
                CHECK(cart_cpu_read(0xC000) == 2u * (block * 4u + 3u));
            }
        }
        cart_ppu_write(0x1234, (uint8_t)(0x70 + submapper));
        CHECK(cart_ppu_read(0x1234) == 0x70 + submapper);
        cart->reset();
        CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 6);
    }

    CHECK(fixture(71, 0x40000, 0x2000, true) == 71);
    cart_cpu_write(0xC000, 2);
    CHECK(cart_cpu_read(0x8000) == 4);

    h.flags7 |= 0x08;
    h.prg_ram_size = 0x20;
    h.flags10 = 7;
    h.zero[0] = 7;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x40000, fixture_chr, 0x2000) == -1);
    CHECK(cart_cpu_read(0x8000) == 4);
    return 0;
}

static int test_mapper232_loader_and_cpu_bus(void) {
    for (unsigned submapper = 0; submapper <= 1; ++submapper) {
        iNESHeader h = header_for(232, 0x40000, true);
        h.flags6 |= 1;
        h.flags7 |= 0x08;
        h.prg_ram_size = (uint8_t)(submapper << 4);
        h.zero[0] = 7;
        size_t size;
        uint8_t *image = image_for(&h, 0x40000, 0, &size);
        CHECK(image != NULL);
        for (size_t i = 0; i < 0x40000; ++i)
            image[sizeof(h) + i] = (uint8_t)(i / 0x4000);
        CHECK(load_rom_memory(image, size) == 0);
        CHECK(rom_mapper_number(&ines_header) == 232);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        CHECK(cpu_power_on(&cpu));
        CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 3);
        CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
        write_mem(0xBFFF, submapper ? 0x10 : 0x08);
        write_mem(0xFFFF, 0xFE);
        CHECK(read_mem(0x8000) == 6 && read_mem(0xBFFF) == 6);
        CHECK(read_mem(0xC000) == 7 && read_mem(0xFFFF) == 7);
        write_mem(0x7FFF, 0x18);
        CHECK(read_mem(0x8000) == 6 && cart_get_mirroring() == MIRROR_VERTICAL);
        cart_ppu_write(0x1FFF, 0xA6);

        Mapper *previous = cart;
        uint8_t *previous_prg = prg_rom;
        image[8] = 0x20;
        CHECK(load_rom_memory(image, size) == -1);
        memcpy(image, &h, sizeof(h));
        CHECK(load_rom_memory(image, size - 1) == -1);
        iNESHeader oversized = h;
        oversized.prg_rom_chunks = 32;
        size_t oversized_size;
        uint8_t *oversized_image = image_for(&oversized, 0x80000, 0, &oversized_size);
        CHECK(oversized_image != NULL);
        int loaded = load_rom_memory(oversized_image, oversized_size);
        free(oversized_image);
        CHECK(loaded == -1);
        image[11] = 8;
        CHECK(load_rom_memory(image, size) == -1);
        free(image);
        CHECK(cart == previous && prg_rom == previous_prg);
        CHECK(read_mem(0x8000) == 6 && read_mem(0xFFFF) == 7);
        CHECK(cart_ppu_read(0x1FFF) == 0xA6);
    }
    return 0;
}

static int test_mmc3_revision_a_irq(void) {
    iNESHeader h = header_for(4, 0x20000, false);
    size_t image_size = 0;
    uint8_t *image = image_for(&h, 0x20000, 0x2000, &image_size);
    CHECK(image != NULL);

    CHECK(cart_set_mmc3_revision_name("standard"));
    CHECK(load_rom_memory(image, image_size) == 0);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(0);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(18);
    CHECK(cart_irq_pending());

    CHECK(cart_set_mmc3_revision_name("a"));
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(0);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(18);
    CHECK(!cart_irq_pending());

    cart_cpu_write(0xC000, 1);
    cart_cpu_write(0xC001, 0);
    a12_pulse(36);
    CHECK(!cart_irq_pending());
    a12_pulse(54);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    a12_pulse(72);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xE001, 0);
    a12_pulse(90);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(108);
    CHECK(!cart_irq_pending());

    CHECK(!cart_set_mmc3_revision_name("unknown"));
    CHECK(strcmp(cart_mmc3_revision_name(), "a") == 0);

    h.flags7 |= 0x08;
    h.prg_ram_size = 0x10;
    h.flags10 = 4;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 4);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(126);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(144);
    CHECK(cart_irq_pending()); // MMC6 retains repeated zero-counter IRQs under profile A.

    h.prg_ram_size = 0x30;
    h.flags10 = 0;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 4);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x1000, 0);
    cart_notify_ppu_address(0x2000, 1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    for (uint64_t edge = 1; edge <= 8; ++edge) {
        cart_notify_ppu_address(0x1000, edge * 2);
        cart_notify_ppu_address(0x2000, edge * 2 + 1);
        CHECK(cart_irq_pending() == (edge == 8));
    }
    CHECK(cart_set_mmc3_revision_name("standard"));
    return 0;
}

static int test_mmc3_revision_a_cpu_irq(void) {
    CHECK(cart_set_mmc3_revision_name("a"));
    CHECK(fixture(4, 0x20000, 0x2000, false) == 4);
    fixture_prg[0x1FFFC] = 0;
    fixture_prg[0x1FFFD] = 2;
    fixture_prg[0x1FFFE] = 0;
    fixture_prg[0x1FFFF] = 3;
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    cart->reset();
    for (unsigned i = 0; i < 16; ++i) write_mem((uint16_t)(0x200 + i), 0xEA);
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    uint64_t edge = ppu.total_cycles;
    cart_notify_ppu_address(0x2000, edge);
    cart_notify_ppu_address(0x1000, edge + 6);
    CHECK(!cart_irq_pending());
    CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x201);
    edge = ppu.total_cycles;
    cart_notify_ppu_address(0x2000, edge);
    cart_notify_ppu_address(0x1000, edge + 9);
    CHECK(cart_irq_pending());
    CHECK(cpu_step(&cpu) == 9 && cpu.pc == 0x300);
    CHECK(read_mem(0x1FC) == 0x02); // IRQ follows the NOP's final polling cycle.
    cart_cpu_write(0xE000, 0);
    CHECK(!cart_irq_pending());
    CHECK(cart_set_mmc3_revision_name("standard"));
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

static int test_taito_banks_aliases_and_mirroring(void) {
    CHECK(fixture(33, 0x80000, 0x20000, false) == 33);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_cpu_read(0xC000) == 62 && cart_cpu_read(0xE000) == 63);

    cart_cpu_write(0x9000, 0x7F); // $9000 aliases $8000.
    cart_cpu_write(0xD001, 0x42); // $D001 aliases $8001.
    CHECK(cart_cpu_read(0x8000) == 63 && cart_cpu_read(0xA000) == 2);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    cart_cpu_write(0x8000, 3);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);

    cart_cpu_write(0x9002, 3);
    cart_cpu_write(0xD003, 5);
    cart_cpu_write(0xB000, 12);
    cart_cpu_write(0xB001, 13);
    cart_cpu_write(0xF002, 14);
    cart_cpu_write(0xF003, 15);
    static const uint8_t expected_chr[] = {6, 7, 10, 11, 12, 13, 14, 15};
    for (unsigned slot = 0; slot < 8; ++slot)
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == expected_chr[slot]);

    uint8_t nt[0x1000] = {0};
    cart_nt_write(0x2000, 0x33, nt);
    CHECK(cart_nt_read(0x2800, nt) == 0x33);
    CHECK(cart_nt_read(0x2400, nt) == 0);
    CHECK(!cart_irq_pending());
    a12_pulse(0);
    a12_pulse(12);
    CHECK(!cart_irq_pending() && cart->clock == NULL);

    cart->reset();
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_ppu_read(0x0012) == 0x12 && cart_ppu_read(0x1C34) == 0x34);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL && !cart_irq_pending());

    CHECK(fixture(48, 0x80000, 0x20000, false) == 48);
    CHECK(cart != NULL && cart->clock != NULL);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_ppu_read(0x0012) == 0x12 && cart_ppu_read(0x1C34) == 0x34);
    cart_cpu_write(0xE000, 0);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0x9000, 0x41); // Bank write does not carry mapper 33's mirroring bit.
    cart_cpu_write(0x9001, 4);
    CHECK(cart_cpu_read(0x8000) == 1 && cart_cpu_read(0xA000) == 4);
    CHECK(cart_cpu_read(0xC000) == 62 && cart_cpu_read(0xE000) == 63);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0x9002, 7);
    cart_cpu_write(0x9003, 9);
    cart_cpu_write(0xB000, 20);
    cart_cpu_write(0xB001, 21);
    cart_cpu_write(0xB002, 22);
    cart_cpu_write(0xB003, 23);
    static const uint8_t expected_chr48[] = {14, 15, 18, 19, 20, 21, 22, 23};
    for (unsigned slot = 0; slot < 8; ++slot)
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == expected_chr48[slot]);
    cart_cpu_write(0xE001, 0x40); // Only $E000 is decoded in this register group.
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0xF000, 0x40); // $F000 aliases $E000.
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);

    memset(nt, 0, sizeof(nt));
    cart_nt_write(0x2000, 0x48, nt);
    CHECK(cart_nt_read(0x2400, nt) == 0x48);
    CHECK(cart_nt_read(0x2800, nt) == 0);
    cart->reset();
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL && !cart_irq_pending());
    return 0;
}

static int test_taito48_irq(void) {
    CHECK(fixture(48, 0x20000, 0x2000, false) == 48);
    cart_cpu_write(0xC000, 0xFD); // Inverted reload value is 2 on the original board.
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xC002, 0);
    a12_pulse(0);
    CHECK(!cart_irq_pending());
    cart_notify_ppu_address(0x2000, 12);
    cart_notify_ppu_address(0x1000, 18); // Two CPU clocks low does not qualify.
    CHECK(!cart_irq_pending());
    a12_pulse(24);
    CHECK(!cart_irq_pending());
    a12_pulse(36);
    CHECK(!cart_irq_pending()); // Counter reached zero, but assertion is delayed.
    cart->clock(21);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());

    cart_cpu_write(0xD000, 0xFF); // $D000 aliases $C000 and acknowledges the line.
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xD001, 0);
    cart_cpu_write(0xD002, 0);
    a12_pulse(48); // Reload zero schedules an IRQ on this qualifying edge.
    cart->clock(22);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xD003, 0);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xC001, 0);
    a12_pulse(60);
    cart->clock(22);
    CHECK(!cart_irq_pending()); // Disabled counter activity cannot schedule another IRQ.

    cart_cpu_write(0xC002, 0);
    cart_cpu_write(0xC001, 0);
    a12_pulse(72);
    cart->clock(5);
    cart_cpu_write(0xC003, 0);
    cart->clock(16);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending()); // Disabling the counter does not cancel an already latched delay.

    iNESHeader h = header_for(48, 0x20000, false);
    h.flags7 |= 0x08;
    h.prg_ram_size = 0x10; // Submapper 1.
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 48);
    cart_cpu_write(0xC000, 0xFF); // Submapper 1 adds one after inversion.
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xC002, 0);
    a12_pulse(0);
    cart->clock(6);
    CHECK(!cart_irq_pending()); // First edge loaded one, so no delay was scheduled.
    a12_pulse(12);
    cart->clock(5);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart->reset();
    CHECK(!cart_irq_pending());
    cart->clock(32);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_taito48_cpu_irq(void) {
    for (unsigned submapper = 0; submapper < 2; ++submapper) {
        iNESHeader h = header_for(48, 0x20000, false);
        h.flags7 |= 8;
        h.prg_ram_size = (uint8_t)(submapper << 4);
        CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 48);
        fixture_prg[0x1FFFC] = 0;
        fixture_prg[0x1FFFD] = 2;
        fixture_prg[0x1FFFE] = 0;
        fixture_prg[0x1FFFF] = 3;
        nes_set_region(NES_REGION_NTSC);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        cpu_power_on(&cpu);
        cart->reset();
        for (unsigned i = 0; i < 32; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
        cpu.status &= (uint8_t)~INTERRUPT_FLAG;
        cart_cpu_write(0xC000, submapper ? 0 : 0xFF);
        cart_cpu_write(0xC001, 0);
        cart_cpu_write(0xC002, 0);
        a12_pulse(0);
        unsigned delay = submapper ? 6 : 22;
        for (unsigned elapsed = 2; elapsed < delay; elapsed += 2) {
            CHECK(cpu_step(&cpu) == 2);
            CHECK(!cart_irq_pending());
        }
        CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
        for (unsigned step = 0; step < 3 && cpu.pc != 0x0300; ++step) (void)cpu_step(&cpu);
        CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG));
        CHECK(cart_irq_pending());
        write_mem(0xC003, 0);
        CHECK(!cart_irq_pending());
        write_mem(0x8000, 3);
        cpu_soft_reset(&cpu);
        CHECK(cart_cpu_read(0x8000) == 3);
    }
    return 0;
}

static int test_rambo1_banks_and_modes(void) {
    CHECK(fixture(64, 0x200000, 0x20000, false) == 64);
    CHECK(cart != NULL && cart->clock != NULL);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 2 && cart_cpu_read(0xE000) == 0xFF);
    for (unsigned slot = 0; slot < 8; ++slot)
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == slot);

    cart_cpu_write(0x9FFE, 6);
    cart_cpu_write(0x9FFF, 5);
    cart_cpu_write(0x8000, 7);
    cart_cpu_write(0x8001, 7);
    cart_cpu_write(0x8000, 15);
    cart_cpu_write(0x8001, 9);
    CHECK(cart_cpu_read(0x8000) == 5 && cart_cpu_read(0xA000) == 7);
    CHECK(cart_cpu_read(0xC000) == 9 && cart_cpu_read(0xE000) == 0xFF);
    cart_cpu_write(0x8000, 0x46);
    CHECK(cart_cpu_read(0x8000) == 9 && cart_cpu_read(0xA000) == 7);
    CHECK(cart_cpu_read(0xC000) == 5 && cart_cpu_read(0xE000) == 0xFF);

    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 10);
    cart_cpu_write(0x8000, 1);
    cart_cpu_write(0x8001, 20);
    for (unsigned reg = 2; reg <= 5; ++reg) {
        cart_cpu_write(0x8000, (uint8_t)reg);
        cart_cpu_write(0x8001, (uint8_t)(28 + reg));
    }
    static const uint8_t paired_chr[] = {10, 11, 20, 21, 30, 31, 32, 33};
    for (unsigned slot = 0; slot < 8; ++slot)
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == paired_chr[slot]);

    cart_cpu_write(0x8000, 0x28);
    cart_cpu_write(0x8001, 12);
    cart_cpu_write(0x8000, 0x29);
    cart_cpu_write(0x8001, 22);
    static const uint8_t one_k_chr[] = {10, 12, 20, 22, 30, 31, 32, 33};
    for (unsigned slot = 0; slot < 8; ++slot)
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == one_k_chr[slot]);
    cart_cpu_write(0x8000, 0xA0);
    static const uint8_t inverted_chr[] = {30, 31, 32, 33, 10, 12, 20, 22};
    for (unsigned slot = 0; slot < 8; ++slot)
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == inverted_chr[slot]);

    uint8_t nt[0x1000] = {0};
    cart_cpu_write(0xA000, 1);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    cart_nt_write(0x2000, 0x64, nt);
    CHECK(cart_nt_read(0x2400, nt) == 0x64 && cart_nt_read(0x2800, nt) == 0);
    cart_cpu_write(0xA000, 0);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    memset(nt, 0, sizeof(nt));
    cart_nt_write(0x2000, 0x46, nt);
    CHECK(cart_nt_read(0x2800, nt) == 0x46 && cart_nt_read(0x2400, nt) == 0);

    cart->reset();
    CHECK(!cart_irq_pending() && cart_get_mirroring() == MIRROR_VERTICAL);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 2 && cart_cpu_read(0xE000) == 0xFF);
    for (unsigned slot = 0; slot < 8; ++slot)
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == slot);
    return 0;
}

static int test_rambo1_irq_sources(void) {
    CHECK(fixture(64, 0x20000, 0x2000, false) == 64);

    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 1);
    cart_cpu_write(0xE001, 0);
    cart->clock(3);
    CHECK(!cart_irq_pending());
    cart->clock(1); // Divide-by-four counter clock schedules the one-cycle IRQ delay.
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    CHECK(!cart_irq_pending());

    cart_cpu_write(0xC001, 1);
    cart_cpu_write(0xE001, 0);
    cart->clock(2);
    cart_notify_ppu_address(0x2000, 0);
    cart_notify_ppu_address(0x1000, 60); // PPU edges are ignored in CPU-cycle mode.
    cart_cpu_write(0xC001, 0);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1); // The old CPU prescaler is allowed one final counter clock.
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);

    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x2000, 100);
    cart_notify_ppu_address(0x1000, 129);
    cart->clock(2);
    CHECK(!cart_irq_pending()); // Twenty-nine PPU cycles low is too short.
    cart_notify_ppu_address(0x2000, 140);
    cart_notify_ppu_address(0x1000, 170);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);

    cart_cpu_write(0xC000, 2);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    for (unsigned edge = 0; edge < 3; ++edge) {
        uint64_t low = 200 + edge * 40;
        cart_notify_ppu_address(0x2000, low);
        cart_notify_ppu_address(0x1000, low + 30);
        cart->clock(2);
        CHECK(!cart_irq_pending());
    }
    cart_notify_ppu_address(0x2000, 320);
    cart_notify_ppu_address(0x1000, 350);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);

    cart_cpu_write(0xC001, 1);
    cart_cpu_write(0xE001, 0);
    cart->clock(4);
    cart->reset();
    CHECK(!cart_irq_pending());
    cart->clock(16);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_rambo158_nametables(void) {
    CHECK(fixture(158, 0x20000, 0x20000, false) == 158);
    uint8_t nt[0x1000] = {0};
    nt[0] = 0x10;
    nt[0x400] = 0x20;
    CHECK(cart_nt_read(0x2000, nt) == 0x10 && cart_nt_read(0x2400, nt) == 0x20);
    CHECK(cart_nt_read(0x2800, nt) == 0x10 && cart_nt_read(0x2C00, nt) == 0x20);

    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 0x80);
    cart_cpu_write(0x8000, 1);
    cart_cpu_write(0x8001, 0x00);
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x2400, nt) == 0x20);
    CHECK(cart_nt_read(0x2800, nt) == 0x10 && cart_nt_read(0x2C00, nt) == 0x10);
    cart_cpu_write(0xA000, 1); // Mapper 158 does not use the RAMBO-1 mirroring register.
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x2800, nt) == 0x10);

    cart_cpu_write(0x8000, 0x82);
    cart_cpu_write(0x8001, 0x80);
    cart_cpu_write(0x8000, 0x83);
    cart_cpu_write(0x8001, 0x00);
    cart_cpu_write(0x8000, 0x84);
    cart_cpu_write(0x8001, 0x80);
    cart_cpu_write(0x8000, 0x85);
    cart_cpu_write(0x8001, 0x00);
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x2400, nt) == 0x10);
    CHECK(cart_nt_read(0x2800, nt) == 0x20 && cart_nt_read(0x2C00, nt) == 0x10);

    cart_cpu_write(0x8000, 0x80);
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x2400, nt) == 0x10);
    cart_cpu_write(0x8000, 0x28);
    cart_cpu_write(0x8001, 0x80); // The nametable latch decodes only the lower three register bits.
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x2400, nt) == 0x20);
    cart_cpu_write(0x8000, 0xA3);
    cart_cpu_write(0x8001, 0x00);

    cart_nt_write(0x2001, 0xA1, nt);
    cart_nt_write(0x2401, 0xB1, nt);
    CHECK(nt[0x401] == 0xA1 && nt[1] == 0xB1);
    cart->reset();
    CHECK(cart_nt_read(0x2000, nt) == 0x10 && cart_nt_read(0x2400, nt) == 0x20);
    CHECK(cart_nt_read(0x2800, nt) == 0x10 && cart_nt_read(0x2C00, nt) == 0x20);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_rambo1_ram_and_odd_chr_banks(void) {
    const unsigned boards[] = {64, 158};
    for (unsigned i = 0; i < 2; ++i) {
        CHECK(fixture(boards[i], 0x20000, 0x8000, true) == (int)boards[i]);
        cart_cpu_write(0x8000, 0);
        cart_cpu_write(0x8001, 5);
        CHECK(cart_ppu_read(0) == 5 && cart_ppu_read(0x0400) == 5);
        cart_ppu_write(0x0123, 0xA6);
        CHECK(cart_ppu_read(0x0523) == 0xA6);
        cart_cpu_write(0x8000, 0x28);
        cart_cpu_write(0x8001, 6);
        CHECK(cart_ppu_read(0x0123) == 0xA6 && cart_ppu_read(0x0523) == 6);
        cart_ppu_write(0x0523, 0x69);
        cart_cpu_write(0x8000, 0xA0);
        CHECK(cart_ppu_read(0x1123) == 0xA6 && cart_ppu_read(0x1523) == 0x69);
        cart_cpu_write(0x6000, 0x35);
        cart_cpu_write(0x7FFF, 0x53);
        for (unsigned protection = 0; protection <= 0xC0; protection += 0x40) {
            cart_cpu_write(0xA001, (uint8_t)protection);
            CHECK(cart_cpu_read(0x6000) == 0x35 && cart_cpu_read(0x7FFF) == 0x53);
            cart_cpu_write(0x6123, (uint8_t)(protection | 0x15));
            CHECK(cart_cpu_read(0x6123) == (protection | 0x15));
        }
        CHECK(cart_cpu_read_bus(0x5000, 0x96) == 0x96);
        cart->reset();
        CHECK(cart_cpu_read(0x6000) == 0x35 && cart_ppu_read(0x1523) == 0xA6);
        iNESHeader h = header_for(boards[i], 0x20000, false);
        h.flags7 |= 8;
        CHECK(fixture_with_header(&h, 0x20000, 0x2000) == (int)boards[i]);
        cart_cpu_write(0x6000, 0xFF);
        CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
        cart_ppu_write(0, 0xFF);
        CHECK(cart_ppu_read(0) == 0);
    }
    return 0;
}

static int test_rambo1_irq_boundaries(void) {
    CHECK(fixture(64, 0x20000, 0x2000, false) == 64);
    const uint8_t reloads[] = {0, 1, 2, 254, 255};
    const unsigned first_clocks[] = {1, 2, 4, 256, 1};
    const unsigned later_clocks[] = {1, 2, 3, 255, 256};
    for (unsigned test = 0; test < sizeof(reloads); ++test) {
        cart->reset();
        cart_cpu_write(0xC000, reloads[test]);
        cart_cpu_write(0xC001, 1);
        cart_cpu_write(0xE001, 0);
        cart->clock((int)(first_clocks[test] * 4));
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
        cart_cpu_write(0xE000, 0);
        cart_cpu_write(0xE001, 0);
        cart->clock((int)(later_clocks[test] * 4 - 1));
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
    }

    for (unsigned phase = 0; phase < 4; ++phase) {
        cart->reset();
        cart_cpu_write(0xC000, 0);
        cart_cpu_write(0xC001, 1);
        cart_cpu_write(0xE001, 0);
        cart->clock((int)phase);
        cart_cpu_write(0xC001, 0);
        cart->clock((int)(4 - phase));
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
        cart_cpu_write(0xE000, 0);
        cart_cpu_write(0xE001, 0);
        cart->clock(16);
        CHECK(!cart_irq_pending()); // Switching to PPU mode allows exactly one final CPU counter clock.
    }

    cart->reset();
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 1);
    cart_cpu_write(0xE001, 0);
    cart->clock(3);
    cart_cpu_write(0xC001, 1);
    cart->clock(4);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart->clock(1);
    CHECK(cart_irq_pending()); // An IRQ already in the output delay survives acknowledgment.
    cart_cpu_write(0xE000, 0);
    cart->clock(20);
    CHECK(!cart_irq_pending());

    cart->reset();
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x2000, 89330);
    cart_notify_ppu_address(0x2001, 89342);
    cart_notify_ppu_address(0x1000, 89360);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart->clock(1);
    CHECK(cart_irq_pending());
    return 0;
}

static int test_rambo1_cpu_and_rendering_irq(void) {
    const unsigned boards[] = {64, 158};
    for (unsigned board = 0; board < 2; ++board) {
        CHECK(fixture(boards[board], 0x20000, 0x2000, false) == (int)boards[board]);
        fixture_prg[0x1FFFC] = 0;
        fixture_prg[0x1FFFD] = 2;
        fixture_prg[0x1FFFE] = 0;
        fixture_prg[0x1FFFF] = 3;
        nes_set_region(NES_REGION_NTSC);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        cpu_power_on(&cpu);
        for (unsigned i = 0; i < 16; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
        cart_cpu_write(0xC000, 0);
        cart_cpu_write(0xC001, 1);
        cart_cpu_write(0xE001, 0);
        CHECK(cpu_step(&cpu) == 2 && !cart_irq_pending());
        CHECK(cpu_step(&cpu) == 2 && !cart_irq_pending());
        CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
        cpu.status &= (uint8_t)~INTERRUPT_FLAG;
        for (unsigned step = 0; step < 3 && cpu.pc != 0x0300; ++step) (void)cpu_step(&cpu);
        CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG));
        cart_cpu_write(0xE000, 0);
        CHECK(!cart_irq_pending());

        ppu_power_on(&ppu);
        ppu_step_dots(341 * 262 * 2);
        write_mem(0x2000, 8);
        write_mem(0x2001, 0x18);
        cart->reset();
        cpu.pc = 0x0200;
        cpu.status = (uint8_t)(UNUSED_FLAG | INTERRUPT_FLAG);
        write_mem(0x0200, 0x4C);
        write_mem(0x0201, 0);
        write_mem(0x0202, 2);
        cart_cpu_write(0xC000, 0);
        cart_cpu_write(0xC001, 0);
        cart_cpu_write(0xE001, 0);
        for (unsigned step = 0; step < 1000 && !cart_irq_pending(); ++step) (void)cpu_step(&cpu);
        CHECK(cart_irq_pending()); // Rendering fetches reach the cartridge's physical A12 filter.
        cart_cpu_write(0xE000, 0);
        cart_cpu_write(0x8000, 6);
        cart_cpu_write(0x8001, 5);
        cpu_soft_reset(&cpu);
        CHECK(cart_cpu_read(0x8000) == 5);
    }
    return 0;
}

static int test_rambo1_loader(void) {
    const unsigned boards[] = {64, 158};
    for (unsigned board = 0; board < 2; ++board) {
        iNESHeader h = header_for(boards[board], 0x20000, false);
        h.flags7 |= 8;
        h.chr_rom_chunks = 32;
        size_t size;
        uint8_t *image = image_for(&h, 0x20000, 0x40000, &size);
        CHECK(image != NULL);
        image[sizeof(h) + 0x20000 + 0x3FC00] = 0x96;
        int loaded = load_rom_memory(image, size);
        free(image);
        CHECK(loaded == 0 && rom_mapper_number(&ines_header) == (int)boards[board]);
        cart_cpu_write(0x8000, 2);
        cart_cpu_write(0x8001, 0xFF);
        CHECK(cart_ppu_read(0x1000) == 0x96);
        cart_cpu_write(0xC001, 1);
        cart_cpu_write(0xE001, 0);
        cart->clock(5);
        CHECK(cart_irq_pending());
        uint8_t *old_prg = prg_rom, *old_chr = chr_rom;
        iNESHeader rejected[] = {h, h, h};
        rejected[0].prg_ram_size = 0x10;
        rejected[1].flags10 = 8;
        rejected[2].chr_rom_chunks = 33;
        for (unsigned i = 0; i < sizeof(rejected) / sizeof(rejected[0]); ++i) {
            image = image_for(&rejected[i], 0x20000, (size_t)rejected[i].chr_rom_chunks * 0x2000, &size);
            CHECK(image != NULL);
            loaded = load_rom_memory(image, size);
            free(image);
            CHECK(loaded == -1 && prg_rom == old_prg && chr_rom == old_chr && cart_irq_pending());
            CHECK(cart_ppu_read(0x1000) == 0x96);
        }
        CHECK(mapper_init_from_header(&h, fixture_prg, 0x20001, fixture_chr, 0x2000) == -1);
        CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x2001) == -1);
        CHECK(prg_rom == old_prg && chr_rom == old_chr && cart_irq_pending());
        image = image_for(&h, 0x20000, 0x40000, &size);
        CHECK(image != NULL);
        loaded = load_rom_memory(image, size - 1);
        free(image);
        CHECK(loaded == -1 && prg_rom == old_prg && cart_irq_pending());
    }
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
    fixture_prg[0] = 0xFF;
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

static uint16_t vrc6_test_addr(unsigned mapper, uint16_t logical) {
    if (mapper != 26) return logical;
    return (uint16_t)((logical & 0xFFFCu) | ((logical & 1u) << 1) | ((logical & 2u) >> 1));
}

static void vrc6_test_write(unsigned mapper, uint16_t logical, uint8_t value) {
    cart_cpu_write(vrc6_test_addr(mapper, logical), value);
}

static int test_vrc6_startup_banks_wiring_and_ram(void) {
    static const unsigned mappers[] = {24, 26};
    for (size_t variant = 0; variant < sizeof(mappers) / sizeof(mappers[0]); ++variant) {
        unsigned mapper = mappers[variant];
        CHECK(fixture(mapper, 0x40000, 0x40000, false) == (int)mapper);
        CHECK(cart != NULL && cart->clock != NULL);

        // Only the final $E000-$FFFF PRG window is selected at power-on.
        CHECK(cart_cpu_read_bus(0x8000, 0x58) == 0x58);
        CHECK(cart_cpu_read_bus(0xA000, 0xA5) == 0xA5);
        CHECK(cart_cpu_read_bus(0xC000, 0x3C) == 0x3C);
        CHECK(cart_cpu_read(0xE000) == 31);
        CHECK(cart_ppu_read(0x0400) == 0); // CHR-ROM windows start unselected.
        cart_cpu_write(0x6123, 0xA6);
        CHECK(cart_cpu_read_bus(0x6123, 0x58) == 0xA6);

        vrc6_test_write(mapper, 0x8000, 3);
        vrc6_test_write(mapper, 0xC000, 9);
        CHECK(cart_cpu_read(0x8000) == 6 && cart_cpu_read(0xA000) == 7);
        CHECK(cart_cpu_read(0xC000) == 9 && cart_cpu_read(0xE000) == 31);

        // The VRC6b swaps A0/A1; these logical writes must behave identically.
        vrc6_test_write(mapper, 0xD000, 1);
        CHECK(cart_cpu_read_bus(0x6123, 0x58) == 0x58);
        cart_cpu_write(0x6123, 0xFF);
        vrc6_test_write(mapper, 0xD001, 3);
        vrc6_test_write(mapper, 0xD002, 5);
        vrc6_test_write(mapper, 0xD003, 7);
        vrc6_test_write(mapper, 0xE000, 9);
        vrc6_test_write(mapper, 0xE001, 11);
        vrc6_test_write(mapper, 0xE002, 13);
        vrc6_test_write(mapper, 0xE003, 15);
        vrc6_test_write(mapper, 0xB003, 0x00);
        static const uint8_t chr_pages[8] = {1, 3, 5, 7, 9, 11, 13, 15};
        for (unsigned slot = 0; slot < 8; ++slot)
            CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == chr_pages[slot]);

        // Mode 1 duplicates 1 KiB pages unless bit 5 enables adjacent pages.
        vrc6_test_write(mapper, 0xD000, 4);
        vrc6_test_write(mapper, 0xB003, 0x01);
        CHECK(cart_ppu_read(0x0000) == 4 && cart_ppu_read(0x0400) == 4);
        vrc6_test_write(mapper, 0xB003, 0x21);
        CHECK(cart_ppu_read(0x0000) == 4 && cart_ppu_read(0x0400) == 5);

        // Mode 2 keeps four 1 KiB pages and pairs the upper half.
        vrc6_test_write(mapper, 0xE000, 8);
        vrc6_test_write(mapper, 0xB003, 0x22);
        CHECK(cart_ppu_read(0x0000) == 4);
        CHECK(cart_ppu_read(0x1000) == 8 && cart_ppu_read(0x1400) == 9);

        // A PPU banking write applies B003.7 to the initial RAM mapping.
        CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
        cart_cpu_write(0x6000, 0xA5);
        CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
        vrc6_test_write(mapper, 0xB003, 0x80);
        CHECK(cart_cpu_read(0x6000) == 0);
        CHECK(cart_cpu_read(0x6123) == 0xA6);
        cart_cpu_write(0x6123, 0xA5);
        CHECK(cart_cpu_read(0x6123) == 0xA5);
        vrc6_test_write(mapper, 0xB003, 0x00);
        CHECK(cart_cpu_read_bus(0x6123, 0x56) == 0x56);
        vrc6_test_write(mapper, 0xB003, 0x80);
        CHECK(cart_cpu_read(0x6123) == 0xA5);

        // Audio registers use the same A0/A1 wiring variant.
        vrc6_test_write(mapper, 0x9000, 0x8F);
        vrc6_test_write(mapper, 0x9002, 0x80);
        float pulse = cart_expansion_audio();
        CHECK(pulse < -0.224f && pulse > -0.226f);
    }

    // CHR-RAM is linear at startup, then follows programmed CHR banks.
    for (size_t variant = 0; variant < sizeof(mappers) / sizeof(mappers[0]); ++variant) {
        unsigned mapper = mappers[variant];
        CHECK(fixture(mapper, 0x40000, 0x2000, true) == (int)mapper);
        cart_ppu_write(0x0400, 0xA6);
        CHECK(cart_ppu_read(0x0400) == 0xA6);
        vrc6_test_write(mapper, 0xD000, 3);
        cart_ppu_write(0x0000, 0x53);
        vrc6_test_write(mapper, 0xD000, 4);
        cart_ppu_write(0x0000, 0x64);
        vrc6_test_write(mapper, 0xD000, 3);
        CHECK(cart_ppu_read(0x0000) == 0x53);
        vrc6_test_write(mapper, 0xD000, 4);
        CHECK(cart_ppu_read(0x0000) == 0x64);
    }
    return 0;
}

static int test_vrc6_nametable_modes(void) {
    static const unsigned mappers[] = {24, 26};
    for (size_t variant = 0; variant < sizeof(mappers) / sizeof(mappers[0]); ++variant) {
        unsigned mapper = mappers[variant];
        CHECK(fixture(mapper, 0x40000, 0x40000, false) == (int)mapper);
        uint8_t nt[0x1000] = {0};
        nt[0x000] = 0x11;
        nt[0x400] = 0x22;

        // Register-controlled CIRAM routing.
        vrc6_test_write(mapper, 0xE000, 0);
        vrc6_test_write(mapper, 0xE001, 1);
        vrc6_test_write(mapper, 0xE002, 1);
        vrc6_test_write(mapper, 0xE003, 0);
        vrc6_test_write(mapper, 0xB003, 0x01);
        CHECK(cart_nt_read(0x2000, nt) == 0x11 && cart_nt_read(0x2400, nt) == 0x22);
        CHECK(cart_nt_read(0x2800, nt) == 0x22 && cart_nt_read(0x2C00, nt) == 0x11);
        cart_nt_write(0x2C10, 0x66, nt);
        CHECK(nt[0x010] == 0x66 && cart_nt_read(0x2010, nt) == 0x66);

        vrc6_test_write(mapper, 0xB003, 0x20);
        CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
        CHECK(cart_nt_read(0x2000, nt) == 0x11 && cart_nt_read(0x2400, nt) == 0x22);
        CHECK(cart_nt_read(0x2800, nt) == 0x11 && cart_nt_read(0x2C00, nt) == 0x22);
        vrc6_test_write(mapper, 0xB003, 0x23);
        CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
        CHECK(cart_nt_read(0x2000, nt) == 0x11 && cart_nt_read(0x2400, nt) == 0x11);
        CHECK(cart_nt_read(0x2800, nt) == 0x22 && cart_nt_read(0x2C00, nt) == 0x22);
        vrc6_test_write(mapper, 0xB003, 0x28);
        CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
        vrc6_test_write(mapper, 0xB003, 0x2B);
        CHECK(cart_get_mirroring() == MIRROR_SINGLE1);

        // B003.4 maps nametable reads through CHR-ROM; writes stay read-only.
        vrc6_test_write(mapper, 0xE000, 20);
        vrc6_test_write(mapper, 0xE001, 21);
        vrc6_test_write(mapper, 0xE002, 22);
        vrc6_test_write(mapper, 0xE003, 23);
        vrc6_test_write(mapper, 0xB003, 0x11);
        CHECK(cart_nt_read(0x2000, nt) == 20 && cart_nt_read(0x2400, nt) == 21);
        CHECK(cart_nt_read(0x2800, nt) == 22 && cart_nt_read(0x2C00, nt) == 23);
        cart_nt_write(0x2000, 0xEE, nt);
        CHECK(cart_nt_read(0x2000, nt) == 20);
    }
    return 0;
}

static int test_vrc6_irq_timing(void) {
    static const unsigned mappers[] = {24, 26};
    for (size_t variant = 0; variant < sizeof(mappers) / sizeof(mappers[0]); ++variant) {
        unsigned mapper = mappers[variant];
        CHECK(fixture(mapper, 0x40000, 0x40000, false) == (int)mapper);

        vrc6_test_write(mapper, 0xF000, 0xFF);
        vrc6_test_write(mapper, 0xF001, 0x02);
        cart->clock(113);
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());

        vrc6_test_write(mapper, 0xF000, 0xFE);
        vrc6_test_write(mapper, 0xF001, 0x07);
        cart->clock(1);
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
        vrc6_test_write(mapper, 0xF002, 0);
        CHECK(!cart_irq_pending());
        cart->clock(2);
        CHECK(cart_irq_pending());

        vrc6_test_write(mapper, 0xF001, 0x06);
        cart->clock(2);
        CHECK(cart_irq_pending());
        vrc6_test_write(mapper, 0xF002, 0);
        CHECK(!cart_irq_pending());
        cart->clock(4);
        CHECK(!cart_irq_pending());
    }
    return 0;
}

static int test_vrc6_pulse_and_saw_audio(void) {
    CHECK(fixture(24, 0x40000, 0x40000, false) == 24);

    vrc6_test_write(24, 0x9000, 0x1F);
    vrc6_test_write(24, 0x9001, 15);
    vrc6_test_write(24, 0x9002, 0x80);
    float pulse = cart_expansion_audio();
    CHECK(pulse < -0.224f && pulse > -0.226f);
    cart->clock(16);
    CHECK(cart_expansion_audio() == pulse);
    cart->clock(1);
    CHECK(cart_expansion_audio() == 0.0f);

    // Frequency shift and halt affect timing but halt preserves the current level.
    CHECK(fixture(24, 0x40000, 0x40000, false) == 24);
    vrc6_test_write(24, 0x9003, 0x02);
    vrc6_test_write(24, 0x9000, 0x1F);
    vrc6_test_write(24, 0x9001, 15);
    vrc6_test_write(24, 0x9002, 0x80);
    cart->clock(1);
    pulse = cart_expansion_audio();
    CHECK(pulse < 0.0f);
    vrc6_test_write(24, 0x9003, 0x03);
    cart->clock(100);
    CHECK(cart_expansion_audio() == pulse);
    vrc6_test_write(24, 0x9003, 0x02);
    cart->clock(1);
    CHECK(cart_expansion_audio() == 0.0f);

    CHECK(fixture(24, 0x40000, 0x40000, false) == 24);
    vrc6_test_write(24, 0x9003, 0x04);
    vrc6_test_write(24, 0x9000, 0x1F);
    vrc6_test_write(24, 0x9001, 0xFF);
    vrc6_test_write(24, 0x9002, 0x80);
    cart->clock(1);
    CHECK(cart_expansion_audio() < 0.0f);
    cart->clock(1);
    CHECK(cart_expansion_audio() == 0.0f);

    // Pulse 2 is independent and disabling a pulse resets its duty phase.
    vrc6_test_write(24, 0x9002, 0x00);
    vrc6_test_write(24, 0xA000, 0x85);
    vrc6_test_write(24, 0xA002, 0x80);
    float pulse2 = cart_expansion_audio();
    CHECK(pulse2 < -0.074f && pulse2 > -0.076f);

    // Saw adds its six-bit rate every second divider step and resets after 14 steps.
    CHECK(fixture(24, 0x40000, 0x40000, false) == 24);
    vrc6_test_write(24, 0xB000, 8);
    vrc6_test_write(24, 0xB001, 0);
    vrc6_test_write(24, 0xB002, 0x80);
    CHECK(cart_expansion_audio() == 0.0f);
    cart->clock(1);
    CHECK(cart_expansion_audio() == 0.0f);
    cart->clock(1);
    float saw = cart_expansion_audio();
    CHECK(saw < -0.014f && saw > -0.016f);
    cart->clock(12);
    CHECK(cart_expansion_audio() == 0.0f);
    vrc6_test_write(24, 0xB002, 0x00);
    CHECK(cart_expansion_audio() == 0.0f);
    vrc6_test_write(24, 0xB002, 0x80);
    cart->clock(2);
    CHECK(cart_expansion_audio() == saw);
    return 0;
}

static int test_vrc6_loader_rejection_preserves_cart(void) {
    CHECK(fixture(24, 0x40000, 0x40000, false) == 24);
    vrc6_test_write(24, 0x8000, 3);
    vrc6_test_write(24, 0xB003, 0x80);
    cart_cpu_write(0x6123, 0xA7);
    Mapper *previous = cart;

    iNESHeader invalid = header_for(24, 0x40000, false);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x42000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 6 && cart_cpu_read(0x6123) == 0xA7);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x42000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 6 && cart_cpu_read(0x6123) == 0xA7);

    invalid.flags7 |= 0x08;
    invalid.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 6 && cart_cpu_read(0x6123) == 0xA7);
    return 0;
}

static int test_vrc6_cpu_boot_and_odd_prg_size(void) {
    const unsigned mappers[] = {24, 26};
    const uint16_t registers[] = {0x8000, 0x9000, 0x9002, 0xF000, 0xF001};
    const uint8_t values[] = {1, 0x8F, 0x80, 0xFE, 0x06};
    for (unsigned variant = 0; variant < 2; ++variant) {
        for (unsigned region = 0; region < 3; ++region) {
            unsigned mapper = mappers[variant];
            iNESHeader h = header_for(mapper, 0x6000, true);
            h.flags7 |= 0x08;
            h.prg_rom_chunks = (13u << 2) | 1u; // NES 2.0: 2^13 * 3 bytes, three 8 KiB pages.
            h.flags9 = 0x0F;
            h.flags10 = 7;
            h.zero[0] = 7;
            h.zero[1] = region == 2 ? 3 : (uint8_t)region;
            size_t size;
            uint8_t *image = image_for(&h, 0x6000, 0, &size);
            CHECK(image != NULL);
            for (size_t i = 0; i < 0x6000; ++i)
                image[sizeof(h) + i] = (uint8_t)(i / 0x2000);
            size_t cursor = sizeof(h) + 0x4000;
            for (unsigned reg = 0; reg < sizeof(values); ++reg) {
                uint16_t address = vrc6_test_addr(mapper, registers[reg]);
                image[cursor++] = 0xA9;
                image[cursor++] = values[reg];
                image[cursor++] = 0x8D;
                image[cursor++] = (uint8_t)address;
                image[cursor++] = (uint8_t)(address >> 8);
            }
            memset(image + cursor, 0xEA, 16);
            image[sizeof(h) + 0x5FFC] = 0x00;
            image[sizeof(h) + 0x5FFD] = 0xE0;
            image[sizeof(h) + 0x5FFE] = 0x00;
            image[sizeof(h) + 0x5FFF] = 0x03;
            CHECK(load_rom_memory(image, size) == 0);
            ppu_power_on(&ppu);
            apu_power_on(&apu);
            cpu_power_on(&cpu);
            CHECK(cpu.pc == 0xE000);
            for (unsigned instruction = 0; instruction < 10; ++instruction) cpu_step(&cpu);
            CHECK(cart_cpu_read(0x8100) == 2 && cart_cpu_read(0xA100) == 0);
            CHECK(cart_expansion_audio() < -0.224f && cart_expansion_audio() > -0.226f);
            cpu_step(&cpu);
            CHECK(cart_irq_pending());
            cpu.status &= (uint8_t)~INTERRUPT_FLAG;
            for (unsigned step = 0; step < 4 && cpu.pc != 0x0300; ++step) cpu_step(&cpu);
            CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG));

            Mapper *previous = cart;
            iNESHeader active = ines_header;
            image[8] = 0x10;
            CHECK(load_rom_memory(image, size) == -1);
            memcpy(image, &h, sizeof(h));
            CHECK(load_rom_memory(image, size - 1) == -1);
            free(image);
            CHECK(cart == previous && memcmp(&ines_header, &active, sizeof(active)) == 0);
            CHECK(cart_cpu_read(0x8100) == 2 && cart_irq_pending());
            vrc6_test_write(mapper, 0xF001, 0);
            vrc6_test_write(mapper, 0x8000, 0xFF);
            CHECK(cart_cpu_read(0x8100) == 0 && cart_cpu_read(0xA100) == 1);
        }
    }
    nes_set_region(NES_REGION_NTSC);
    return 0;
}

static int test_colordreams_bus_conflicts(void) {
    const uint16_t write_addr = 0x8123;
    const size_t bank_offset = write_addr - 0x8000;

    CHECK(fixture(11, 0x20000, 0x20000, false) == 11);

    // The first write must be masked by the byte in the bank mapped before the write.
    fixture_prg[bank_offset] = 0x52;
    fixture_prg[3 * 0x8000 + bank_offset] = 0x01;
    cart_cpu_write(write_addr, 0xF3);
    CHECK(cart_cpu_read(0x8000) == 8);
    CHECK(cart_ppu_read(0) == 40);

    // A ROM byte with every bit set leaves the CPU value unchanged.
    fixture_prg[2 * 0x8000 + bank_offset] = 0xFF;
    cart_cpu_write(write_addr, 0x31);
    CHECK(cart_cpu_read(0x8000) == 4);
    CHECK(cart_ppu_read(0) == 24);

    // Once bank 1 is mapped, the next conflict must use bank 1 at the write address.
    fixture_prg[1 * 0x8000 + bank_offset] = 0xA2;
    cart_cpu_write(write_addr, 0xF3);
    CHECK(cart_cpu_read(0x8000) == 8);
    CHECK(cart_ppu_read(0) == 80);
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

static iNESHeader action53_header(size_t prg_bytes, uint8_t chr_ram_shift) {
    iNESHeader h = header_for(28, prg_bytes, true);
    h.flags7 |= 0x08;
    h.prg_ram_size = 0;
    h.flags10 = 0;
    h.zero[0] = chr_ram_shift;
    return h;
}

static int test_action53_banks_and_mirroring(void) {
    iNESHeader h = action53_header(0x200000, 9); // 32KB CHR-RAM.
    CHECK(fixture_with_header(&h, 0x200000, 0x8000) == 28);

    // Power-up guarantees only the fixed last 16KB page at $C000.
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read(0xC000) == 0xFE);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL && cart->reset == NULL);

    // $81 selects the outer register. Mode $38 is 16KB banking with $8000 fixed.
    cart_cpu_write(0x5000, 0x81);
    CHECK(cart_cpu_read_bus(0x8123, 0x96) == 0x96);
    cart_cpu_write(0x8000, 0x12);
    cart_cpu_write(0x5000, 0x80);
    cart_cpu_write(0x8000, 0x38);
    cart_cpu_write(0x5000, 0x01);
    cart_cpu_write(0x8000, 0x06);
    CHECK(cart_cpu_read(0x8000) == 0x48 && cart_cpu_read(0xC000) == 0x4C);

    // Slot-select flips which 16KB half is fixed; 32KB mode keeps the pair adjacent.
    cart_cpu_write(0x5000, 0x80);
    cart_cpu_write(0x8000, 0x3C);
    CHECK(cart_cpu_read(0x8000) == 0x4C && cart_cpu_read(0xC000) == 0x4A);
    cart_cpu_write(0x8000, 0x20);
    CHECK(cart_cpu_read(0x8000) == 0x48 && cart_cpu_read(0xC000) == 0x4A);

    // Mirroring can be one-screen, vertical, or horizontal depending on mode and data bit.
    cart_cpu_write(0x8000, 0x00);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0x5000, 0x01);
    cart_cpu_write(0x8000, 0x10);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    cart_cpu_write(0x5000, 0x80);
    cart_cpu_write(0x8000, 0x02);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0x8000, 0x03);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);

    // Four 8KB CHR-RAM pages are independently writable; upper selector bits alias.
    cart_cpu_write(0x5000, 0x00);
    cart_cpu_write(0x8000, 0x00);
    cart_ppu_write(0x0123, 0xA0);
    cart_cpu_write(0x8000, 0x01);
    cart_ppu_write(0x0123, 0xB1);
    cart_cpu_write(0x8000, 0x02);
    cart_ppu_write(0x0123, 0xC2);
    cart_cpu_write(0x8000, 0x03);
    cart_ppu_write(0x0123, 0xD3);
    cart_cpu_write(0x8000, 0x05);
    CHECK(cart_ppu_read(0x0123) == 0xB1);

    // Addresses outside $5000-$5FFF do not change the selected register.
    cart_cpu_write(0x5000, 0x00);
    cart_cpu_write(0x4FFF, 0x81);
    cart_cpu_write(0x8000, 0x02);
    CHECK(cart_ppu_read(0x0123) == 0xC2);

    // Unsupported CHR-RAM geometry fails transactionally through the loader.
    size_t image_size;
    iNESHeader valid = action53_header(0x20000, 9);
    uint8_t *image = image_for(&valid, 0x20000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    free(image);
    uint8_t *previous_prg = prg_rom;
    uint8_t *previous_chr = chr_rom;
    iNESHeader previous_header = ines_header;
    iNESHeader invalid = action53_header(0x20000, 10); // 64KB exceeds the board's CHR-RAM lines.
    image = image_for(&invalid, 0x20000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == -1);
    free(image);
    CHECK(prg_rom == previous_prg && chr_rom == previous_chr);
    CHECK(memcmp(&ines_header, &previous_header, sizeof(ines_header)) == 0);
    CHECK(cart_cpu_read(0xC000) == 0x5C);
    unload_rom();
    return 0;
}

static int test_action53_game_sizes(void) {
    iNESHeader h = action53_header(0x200000, 9);
    CHECK(fixture_with_header(&h, 0x200000, 0x8000) == 28);
    cart_cpu_write(0x5FFF, 0x81);
    cart_cpu_write(0xFFFF, 0x13);
    cart_cpu_write(0x5000, 1);
    cart_cpu_write(0x8000, 13);
    const uint8_t switched16[] = {39, 37, 37, 45};
    const uint8_t pair32[] = {38, 38, 34, 42};
    for (unsigned game = 0; game < 4; ++game) {
        cart_cpu_write(0x5000, 0x80);
        cart_cpu_write(0x8000, (uint8_t)(game << 4));
        CHECK(cart_cpu_read(0x8000) == pair32[game] * 2);
        CHECK(cart_cpu_read(0xC000) == (pair32[game] + 1) * 2);
        for (unsigned slot = 0; slot < 2; ++slot) {
            cart_cpu_write(0x8000, (uint8_t)((game << 4) | 8 | (slot << 2)));
            CHECK(cart_cpu_read(slot ? 0x8000 : 0xC000) == switched16[game] * 2);
            CHECK(cart_cpu_read(slot ? 0xC000 : 0x8000) == (38 + slot) * 2);
        }
    }
    return 0;
}

static int test_action53_largest_image(void) {
    iNESHeader h = action53_header(0x800000, 9);
    h.flags9 = 2;
    size_t bytes;
    uint8_t *image = image_for(&h, 0x800000, 0, &bytes);
    CHECK(image != NULL);
    for (size_t bank = 0; bank < 512; ++bank) {
        image[sizeof(h) + bank * 0x4000] = (uint8_t)bank;
        image[sizeof(h) + bank * 0x4000 + 1] = (uint8_t)(bank >> 8);
    }
    int loaded = load_rom_memory(image, bytes);
    free(image);
    CHECK(loaded == 0);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
    CHECK(cart_cpu_read(0xC000) == 0xFF && cart_cpu_read(0xC001) == 1);
    cart_cpu_write(0x5000, 0x81);
    cart_cpu_write(0x8000, 0xFF);
    CHECK(cart_cpu_read(0x8000) == 0xFE && cart_cpu_read(0x8001) == 1);
    CHECK(cart_cpu_read(0xC000) == 0xFF && cart_cpu_read(0xC001) == 1);
    uint8_t *old_prg = prg_rom;
    h = action53_header(0x20000, 0);
    h.chr_rom_chunks = 1;
    image = image_for(&h, 0x20000, 0x2000, &bytes);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, bytes);
    free(image);
    CHECK(loaded == -1 && prg_rom == old_prg && cart_cpu_read(0x8001) == 1);
    return 0;
}

static iNESHeader unrom512_header(uint8_t submapper, bool battery, uint8_t chr_ram_shift) {
    iNESHeader h = header_for(30, 0x40000, true);
    h.flags7 |= 0x08;
    h.prg_ram_size = (uint8_t)(submapper << 4);
    h.flags10 = 0;
    h.zero[0] = chr_ram_shift;
    if (battery) h.flags6 |= 0x02;
    return h;
}

static void unrom512_flash_command(uint8_t command) {
    cart_cpu_write(0xC000, 0x01);
    cart_cpu_write(0x9555, 0xAA);
    cart_cpu_write(0xC000, 0x00);
    cart_cpu_write(0xAAAA, 0x55);
    cart_cpu_write(0xC000, 0x01);
    cart_cpu_write(0x9555, command);
}

static void unrom512_flash_erase_prefix(void) {
    unrom512_flash_command(0x80);
    cart_cpu_write(0xC000, 0x01);
    cart_cpu_write(0x9555, 0xAA);
    cart_cpu_write(0xC000, 0x00);
    cart_cpu_write(0xAAAA, 0x55);
}

static int test_unrom512_banks_flash_and_mirroring(void) {
    iNESHeader legacy = header_for(30, 0x40000, true);
    RomRamSizes ram;
    CHECK(rom_ram_sizes(&legacy, &ram) == 0);
    CHECK(ram.prg_ram == 0 && ram.prg_nvram == 0 && ram.chr_ram == 0x8000 && ram.chr_nvram == 0);
    CHECK(fixture_with_header(&legacy, 0x40000, 0x8000) == 30);

    iNESHeader h = unrom512_header(1, true, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 30);

    cart_cpu_write(0xC000, 0x03);
    CHECK(cart_cpu_read(0x8000) == 6 && cart_cpu_read(0xC000) == 30);
    cart_cpu_write(0xC000, 0x63);
    cart_ppu_write(0x0123, 0xD3);
    cart_cpu_write(0xC000, 0x23);
    cart_ppu_write(0x0123, 0xB1);
    cart_cpu_write(0xC000, 0x63);
    CHECK(cart_ppu_read(0x0123) == 0xD3);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL); // Submapper 1 ignores bit 7.

    // A valid byte-program command can only clear flash bits.
    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 0x03);
    CHECK(cart_cpu_read(0x8123) == 6);
    cart_cpu_write(0x8123, 0x04);
    CHECK(cart_cpu_read(0x8123) == 0x04);
    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 0x03);
    cart_cpu_write(0x8123, 0xFF);
    CHECK(cart_cpu_read(0x8123) == 0x04);

    // Invalid and interrupted unlock sequences do not alter program storage.
    cart_cpu_write(0xC000, 0x05);
    uint8_t untouched = cart_cpu_read(0x8234);
    cart_cpu_write(0xC000, 0x01);
    cart_cpu_write(0x9555, 0xAA);
    cart_cpu_write(0xC000, 0x00);
    cart_cpu_write(0xAAAA, 0x54);
    cart_cpu_write(0xC000, 0x05);
    cart_cpu_write(0x8234, 0x00);
    CHECK(cart_cpu_read(0x8234) == untouched);
    cart_cpu_write(0xC000, 0x05);
    untouched = cart_cpu_read(0x8234);
    cart_cpu_write(0xC000, 0x01);
    cart_cpu_write(0x9554, 0xAA); // Unlock data at the wrong physical address.
    cart_cpu_write(0xC000, 0x00);
    cart_cpu_write(0xAAAA, 0x55);
    cart_cpu_write(0xC000, 0x01);
    cart_cpu_write(0x9555, 0xA0);
    cart_cpu_write(0xC000, 0x05);
    cart_cpu_write(0x8234, 0x00);
    CHECK(cart_cpu_read(0x8234) == untouched);

    // Software ID mode responds through the currently selected physical bank.
    unrom512_flash_command(0x90);
    cart_cpu_write(0xC000, 0x00);
    CHECK(cart_cpu_read(0x8000) == 0xBF && cart_cpu_read(0x8001) == 0xB7);
    cart_cpu_write(0x8000, 0xF0);
    CHECK(cart_cpu_read(0x8000) == 0);

    // Sector erase uses the selected physical bank and restores the whole 4KB sector.
    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 0x04);
    cart_cpu_write(0x8123, 0x00);
    CHECK(cart_cpu_read(0x8123) == 0x00);
    unrom512_flash_erase_prefix();
    cart_cpu_write(0xC000, 0x04);
    cart_cpu_write(0x8123, 0x30);
    CHECK(cart_cpu_read(0x8123) == 0xFF && cart_cpu_read(0x8FFF) == 0xFF);

    // Chip erase follows the six-write sequence as well.
    unrom512_flash_erase_prefix();
    cart_cpu_write(0xC000, 0x01);
    cart_cpu_write(0x9555, 0x10);
    CHECK(cart_cpu_read(0x8000) == 0xFF && cart_cpu_read(0xC000) == 0xFF);

    // Read-only boards treat the same addresses as bank writes and keep PRG immutable.
    h = unrom512_header(1, false, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    cart_cpu_write(0x8000, 0x03);
    CHECK(cart_cpu_read(0x8000) == 6);
    uint8_t readonly = fixture_prg[3 * 0x4000 + 0x123];
    unrom512_flash_command(0xA0);
    cart_cpu_write(0x8000, 0x03);
    cart_cpu_write(0x8123, 0x00);
    CHECK(fixture_prg[3 * 0x4000 + 0x123] == readonly);

    // Switchable one-screen and submapper 3 H/V mirroring use latch bit 7.
    h = unrom512_header(1, false, 9);
    h.flags6 |= 0x08;
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0x8000, 0x80);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    h = unrom512_header(3, false, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0x8000, 0x00);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    cart_cpu_write(0x8000, 0x80);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);

    // Submapper 4 reserves $8000-$BFFF for its LED register.
    h = unrom512_header(4, false, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    cart_cpu_write(0xC000, 0x03);
    CHECK(cart_cpu_read(0x8000) == 6);
    cart_cpu_write(0x8000, 0x07);
    CHECK(cart_cpu_read(0x8000) == 6);

    // Four-screen boards use the top CHR-RAM page as cartridge nametable RAM.
    h = unrom512_header(1, false, 9);
    h.flags6 |= 0x09;
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    uint8_t nt[0x1000] = {0};
    cart_nt_write(0x2000, 0x20, nt);
    cart_nt_write(0x2C00, 0x2C, nt);
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x2C00, nt) == 0x2C);
    CHECK(fixture_chr[0x6000] == 0x20 && fixture_chr[0x6C00] == 0x2C);
    cart_nt_write(0x3000, 0x30, nt);
    cart_nt_write(0x3E00, 0x3E, nt);
    CHECK(cart_nt_read(0x3000, nt) == 0x30 && cart_nt_read(0x3E00, nt) == 0x3E);
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && fixture_chr[0x7000] == 0x30);
    ppu_power_on(&ppu);
    ppu_write(0x2E12, 0xA6);
    ppu_write(0x3E12, 0x69);
    CHECK(ppu_read(0x2E12) == 0xA6 && ppu_read(0x3E12) == 0x69);
    CHECK(fixture_chr[0x6E12] == 0xA6 && fixture_chr[0x7E12] == 0x69);
    cart_cpu_write(0x8000, 0x60);
    CHECK(cart_ppu_read(0x0E12) == 0xA6 && cart_ppu_read(0x1E12) == 0x69);
    ppu_write(0x3F00, 0x2A);
    CHECK(ppu_read(0x3F00) == 0x2A && fixture_chr[0x7F00] != 0x2A);

    // Malformed flash metadata and undersized four-screen CHR fail transactionally.
    iNESHeader valid = unrom512_header(1, false, 9);
    size_t image_size;
    uint8_t *image = image_for(&valid, 0x40000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    free(image);
    uint8_t *previous_prg = prg_rom;
    uint8_t *previous_chr = chr_rom;
    iNESHeader previous_header = ines_header;
    iNESHeader invalid = unrom512_header(5, true, 9);
    image = image_for(&invalid, 0x40000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == -1);
    free(image);
    invalid = unrom512_header(1, false, 8);
    invalid.flags6 |= 0x09;
    image = image_for(&invalid, 0x40000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == -1);
    free(image);
    CHECK(prg_rom == previous_prg && chr_rom == previous_chr);
    CHECK(memcmp(&ines_header, &previous_header, sizeof(ines_header)) == 0);
    unload_rom();
    return 0;
}

static int test_unrom512_physical_flash_address(void) {
    iNESHeader h = unrom512_header(1, true, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    cart_cpu_write(0xC000, 0x13);
    CHECK(cart_cpu_read(0x8123) == 6);
    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 0x13);
    cart_cpu_write(0x8123, 0);
    CHECK(cart_cpu_read(0x8123) == 6); // Unpopulated flash addresses do not wrap for writes.
    unrom512_flash_erase_prefix();
    cart_cpu_write(0xC000, 0x13);
    cart_cpu_write(0x8123, 0x30);
    CHECK(cart_cpu_read(0x8123) == 6);
    cart_cpu_write(0xC000, 3);
    CHECK(cart_cpu_read(0x8123) == 6);

    // The same physical address exists on a 512KB flash chip.
    h.prg_rom_chunks = 32;
    CHECK(fixture_with_header(&h, 0x80000, 0x8000) == 30);
    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 0x13);
    cart_cpu_write(0x8123, 0);
    CHECK(cart_cpu_read(0x8123) == 0);
    cart_cpu_write(0xC000, 3);
    CHECK(cart_cpu_read(0x8123) == 6);
    unrom512_flash_erase_prefix();
    cart_cpu_write(0xC000, 0x13);
    cart_cpu_write(0x8123, 0x30);
    CHECK(cart_cpu_read(0x8123) == 0xFF && cart_cpu_read(0x9000) == 38);

    // Undefined commands consume their third command cycle without changing ID mode.
    unrom512_flash_command(0x90);
    unrom512_flash_command(0x00);
    cart_cpu_write(0x8000, 0xF0);
    CHECK(cart_cpu_read(0x8000) == 0xBF);
    cart_cpu_write(0x8000, 0xF0);
    CHECK(cart_cpu_read(0x8000) == 2);
    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 3);
    cart_cpu_write(0x8123, 0xF0); // Program data $F0 is not an ID-exit command.
    CHECK(cart_cpu_read(0x8123) == 0);

    for (size_t prg_bytes = 0x4000; prg_bytes <= 0x80000; prg_bytes *= 2) {
        h.prg_rom_chunks = (uint8_t)(prg_bytes / 0x4000);
        h.flags6 &= (uint8_t)~2u;
        CHECK(fixture_with_header(&h, prg_bytes, 0x8000) == 30);
        cart_cpu_write(0x8000, 31);
        CHECK(cart_cpu_read(0x8000) == (prg_bytes / 0x4000 - 1) * 2);
        CHECK(cart_cpu_read(0xC000) == (prg_bytes / 0x4000 - 1) * 2);
    }
    h = unrom512_header(2, true, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    memset(fixture_prg, 0xFF, 0x40000);
    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 3);
    cart_cpu_write(0x8123, 0x96);
    CHECK(cart_cpu_read(0x8123) == 0x96);
    return 0;
}

static int test_unrom512_cpu_flash(void) {
    iNESHeader h = unrom512_header(1, true, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    const uint8_t program[] = {
        0xA9, 1, 0x8D, 0, 0xC0,
        0xA9, 0xAA, 0x8D, 0x55, 0x95,
        0xA9, 0, 0x8D, 0, 0xC0,
        0xA9, 0x55, 0x8D, 0xAA, 0xAA,
        0xA9, 1, 0x8D, 0, 0xC0,
        0xA9, 0xA0, 0x8D, 0x55, 0x95,
        0xA9, 3, 0x8D, 0, 0xC0,
        0xA9, 4, 0x8D, 0x23, 0x81,
        0xAD, 0x23, 0x81
    };
    for (size_t i = 0; i < sizeof(program); ++i)
        write_mem((uint16_t)(0x0200 + i), program[i]);
    cpu.pc = 0x0200;
    for (unsigned i = 0; i < 8; ++i) {
        CHECK(cpu_step(&cpu) == 2);
        CHECK(cpu_step(&cpu) == 4);
    }
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 4);
    CHECK(cart_cpu_read(0xC000) == 30);
    cpu_soft_reset(&cpu);
    CHECK(cart_cpu_read(0x8123) == 4);
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
    // Status masks disabled IRQs but still acknowledges the trip.
    CHECK(cart_cpu_read(0x5010) == 0x01);
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

static int test_mmc1a_ram_revision(void) {
    const unsigned boards[] = {1, 155};
    for (size_t board = 0; board < sizeof(boards) / sizeof(boards[0]); ++board) {
        for (unsigned nes2 = 0; nes2 < 2; ++nes2) {
            iNESHeader h = header_for(boards[board], 0x20000, false);
            h.chr_rom_chunks = 4;
            if (nes2) {
                h.flags7 |= 8;
                h.flags10 = 7;
            }
            size_t size;
            uint8_t *image = image_for(&h, 0x20000, 0x8000, &size);
            CHECK(image != NULL);
            for (size_t i = 0; i < 0x20000; ++i)
                image[sizeof(h) + i] = (uint8_t)(i / 0x2000);
            for (size_t i = 0; i < 0x8000; ++i)
                image[sizeof(h) + 0x20000 + i] = (uint8_t)(i / 0x0400);
            int loaded = load_rom_memory(image, size);
            free(image);
            CHECK(loaded == 0 && rom_mapper_number(&ines_header) == (int)boards[board]);

            cart_cpu_write(0x6123, 0xA6);
            serial_write(0xE000, 0x13);
            CHECK(cart_cpu_read(0x8000) == 6 && cart_cpu_read(0xC000) == 14);
            CHECK(cart_cpu_read_bus(0x6123, 0x56) == (board ? 0xA6 : 0x56));
            cart_cpu_write(0x6123, 0x55);
            serial_write(0xE000, 3);
            CHECK(cart_cpu_read(0x6123) == (board ? 0x55 : 0xA6));

            static const uint8_t prg_pages[4][2] = {{4, 6}, {4, 6}, {0, 6}, {6, 14}};
            static const Mirroring mirrors[4] = {
                MIRROR_SINGLE0, MIRROR_SINGLE1, MIRROR_VERTICAL, MIRROR_HORIZONTAL
            };
            serial_write(0xA000, 1);
            serial_write(0xC000, 3);
            for (unsigned mode = 0; mode < 4; ++mode) {
                serial_write(0x8000, (uint8_t)(0x10 | (mode << 2) | mode));
                CHECK(cart_cpu_read(0x8000) == prg_pages[mode][0]);
                CHECK(cart_cpu_read(0xC000) == prg_pages[mode][1]);
                CHECK(cart_get_mirroring() == mirrors[mode]);
                CHECK(cart_ppu_read(0) == 4 && cart_ppu_read(0x1000) == 12);
            }
            serial_write(0x8000, 0x0C);
            serial_write(0xA000, 3);
            CHECK(cart_ppu_read(0) == 8 && cart_ppu_read(0x1000) == 12);
            cart_ppu_write(0, 0x77);
            CHECK(cart_ppu_read(0) == 8);
            cart->reset();
            CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 14);
            CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
            CHECK(cart_cpu_read(0x6123) == (board ? 0x55 : 0xA6));
        }
    }
    return 0;
}

static int test_mmc1a_cpu_serial_writes(void) {
    CHECK(fixture(155, 0x20000, 0x2000, true) == 155);
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    for (uint16_t addr = 0; addr < 0x0800; ++addr) write_mem(addr, 0);
    fixture_prg[0x1E000] = 1;
    const uint8_t program[] = {
        0xEE, 0x00, 0xE0,       // INC $E000: serial bit 1, then an ignored adjacent write.
        0xA9, 0x00,             // LDA #0
        0x8D, 0x00, 0xE0,       // STA $E000
        0x8D, 0x00, 0xE0,
        0x8D, 0x00, 0xE0,
        0xA9, 0x01,             // LDA #1
        0x8D, 0x00, 0xE0,       // Commit $11, selecting PRG bank 1 with bit 4 set.
        0xA9, 0xA6,
        0x8D, 0x23, 0x61,       // STA $6123
        0xAD, 0x23, 0x61        // LDA $6123
    };
    for (size_t i = 0; i < sizeof(program); ++i)
        write_mem((uint16_t)(0x0200 + i), program[i]);
    cpu.pc = 0x0200;
    const int cycles[] = {6, 2, 4, 4, 4, 2, 4, 2, 4, 4};
    for (size_t i = 0; i < sizeof(cycles) / sizeof(cycles[0]); ++i)
        CHECK(cpu_step(&cpu) == cycles[i]);
    CHECK(cpu.a == 0xA6 && cart_cpu_read(0x6123) == 0xA6);
    CHECK(cart_cpu_read(0x8000) == 2 && cart_cpu_read(0xC000) == 14);

    // An adjacent reset-bit write is accepted and discards the partial shift.
    serial_write(0x8000, 3);
    fixture_prg[0x4000] = 0x7F;
    const uint8_t reset_program[] = {0xEE, 0x00, 0xC0}; // INC $C000 writes $7F, then $80.
    for (size_t i = 0; i < sizeof(reset_program); ++i)
        write_mem((uint16_t)(0x0200 + i), reset_program[i]);
    cpu.pc = 0x0200;
    CHECK(cpu_step(&cpu) == 6);
    CHECK(cart_cpu_read(0xC000) == 14 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    serial_write(0xE000, 2);
    CHECK(cart_cpu_read(0x8000) == 4 && cart_cpu_read(0x6123) == 0xA6);
    cart_ppu_write(0x0123, 0x69);
    CHECK(cart_ppu_read(0x0123) == 0x69);
    return 0;
}

static int test_mmc1a_ram_layouts_and_loader(void) {
    iNESHeader h = header_for(155, 0x80000, true);
    h.flags7 |= 8;
    h.flags10 = 9;
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x80000, 0x2000) == 155);
    serial_write(0xE000, 0x10);
    for (unsigned bank = 0; bank < 4; ++bank) {
        serial_write(0xA000, (uint8_t)(0x10 | (bank << 2)));
        cart_cpu_write(0x6000, (uint8_t)(0xA0 + bank));
        cart_cpu_write(0x7FFF, (uint8_t)(0xB0 + bank));
    }
    for (unsigned bank = 0; bank < 4; ++bank) {
        serial_write(0xA000, (uint8_t)(0x10 | (bank << 2)));
        CHECK(cart_cpu_read(0x6000) == 0xA0 + bank && cart_cpu_read(0x7FFF) == 0xB0 + bank);
        CHECK(cart_cpu_read(0x8000) == 32 && cart_cpu_read(0xC000) == 62);
    }
    serial_write(0x8000, 0x1C);
    serial_write(0xC000, 4);
    CHECK(cart_cpu_read(0x6000) == 0xA1);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 30);
    serial_write(0x8000, 0x0C);
    CHECK(cart_cpu_read(0x6000) == 0xA3 && cart_cpu_read(0xC000) == 62);

    h.flags6 |= 2;
    h.flags10 = 0x77;
    CHECK(fixture_with_header(&h, 0x80000, 0x2000) == 155);
    serial_write(0xE000, 0x10);
    cart_cpu_write(0x6000, 0x35);
    serial_write(0xA000, 8);
    cart_cpu_write(0x6000, 0x73);
    serial_write(0xA000, 4);
    CHECK(cart_cpu_read(0x6000) == 0x35);
    serial_write(0xA000, 8);
    CHECK(cart_cpu_read(0x6000) == 0x73);

    size_t size;
    uint8_t *image = image_for(&h, 0x80000, 0, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0);
    serial_write(0xE000, 0x10);
    cart_cpu_write(0x6000, 0x96);
    uint8_t *previous_prg = prg_rom;
    uint8_t *previous_chr = chr_rom;
    iNESHeader invalid[] = {h, h, h, h};
    invalid[0].prg_ram_size = 0x50; // Fixed-PRG submapper 5 belongs to mapper 1.
    invalid[1].flags10 = 10;        // More than 32KB of PRG-RAM.
    invalid[2].zero[0] = 0x77;     // Separate volatile and nonvolatile CHR chips.
    invalid[3].flags6 &= (uint8_t)~2u;
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        image = image_for(&invalid[i], 0x80000, 0, &size);
        CHECK(image != NULL);
        loaded = load_rom_memory(image, size);
        free(image);
        CHECK(loaded == -1 && prg_rom == previous_prg && chr_rom == previous_chr);
        CHECK(cart_cpu_read(0x6000) == 0x96);
    }

    h.flags6 &= (uint8_t)~2u;
    h.flags10 = 0;
    CHECK(fixture_with_header(&h, 0x80000, 0x2000) == 155);
    serial_write(0xE000, 0x10);
    cart_cpu_write(0x6000, 0x77);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    return 0;
}

static iNESHeader mcacc_header(void) {
    iNESHeader h = header_for(4, 0x20000, false);
    h.flags7 = 8;
    h.prg_ram_size = 0x30;
    h.flags10 = 7;
    h.chr_rom_chunks = 4;
    return h;
}

static void mcacc_falling_edge(uint64_t cycle) {
    cart_notify_ppu_address(0x1000, cycle);
    cart_notify_ppu_address(0x2000, cycle + 1);
}

static int test_mcacc_irq_divider(void) {
    iNESHeader h = mcacc_header();
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_cpu_write(0xC000, 2);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x2000, 0);
    for (unsigned pulse = 1; pulse <= 17; ++pulse) {
        uint64_t cycle = (uint64_t)pulse * 4;
        cart_notify_ppu_address(0x1000, cycle);
        cart_notify_ppu_address(0x1001, cycle + 1);
        CHECK(!cart_irq_pending());
        cart_notify_ppu_address(0x2000, cycle + 2);
        cart_notify_ppu_address(0x2001, cycle + 3);
        CHECK(cart_irq_pending() == (pulse == 17));
    }
    cart_cpu_write(0xE000, 0);
    CHECK(!cart_irq_pending());

    cart->reset();
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xE001, 0);
    mcacc_falling_edge(100);
    CHECK(cart_irq_pending()); // A one-dot high pulse is sufficient.
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    CHECK(!cart_irq_pending()); // Acknowledgment and enable do not reset divider phase.
    for (unsigned pulse = 1; pulse <= 8; ++pulse) {
        mcacc_falling_edge(100 + (uint64_t)pulse * 2);
        CHECK(cart_irq_pending() == (pulse == 8));
    }

    // Reset clears the remembered high level and the divider as well as the IRQ.
    cart_notify_ppu_address(0x1000, 120);
    cart->reset();
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x2000, 121);
    CHECK(!cart_irq_pending());
    mcacc_falling_edge(122);
    CHECK(cart_irq_pending());
    return 0;
}

static int test_mcacc_irq_reload_and_enable(void) {
    iNESHeader h = mcacc_header();
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_cpu_write(0xC000, 1);
    for (unsigned pulse = 0; pulse < 9; ++pulse) mcacc_falling_edge((uint64_t)pulse * 2);
    CHECK(!cart_irq_pending()); // Disabled IRQ output does not stop counting.
    cart_cpu_write(0xE001, 0);
    for (unsigned pulse = 1; pulse <= 16; ++pulse) {
        mcacc_falling_edge(18 + (uint64_t)pulse * 2);
        CHECK(cart_irq_pending() == (pulse == 16));
    }
    cart_cpu_write(0xE000, 0);

    cart->reset();
    cart_cpu_write(0xC000, 4);
    cart_cpu_write(0xE001, 0);
    for (unsigned pulse = 0; pulse < 3; ++pulse) mcacc_falling_edge(100 + (uint64_t)pulse * 2);
    cart_cpu_write(0xC000, 0);
    cart_notify_ppu_address(0x1000, 108);
    cart_cpu_write(0xDFFF, 0); // $C001 mirror resets the divider without forgetting A12.
    CHECK(!cart_irq_pending());
    cart_notify_ppu_address(0x2000, 109);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xC001, 0);
    CHECK(cart_irq_pending()); // Reload requests do not acknowledge an asserted IRQ.
    cart_cpu_write(0xFFFE, 0); // $E000 mirror acknowledges it.
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xFFFF, 0);
    cart_notify_ppu_address(0x2001, 110);
    CHECK(!cart_irq_pending());
    mcacc_falling_edge(111);
    CHECK(cart_irq_pending());
    return 0;
}

static int test_mcacc_banks_ram_and_loader(void) {
    iNESHeader h = mcacc_header();
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 14 && cart_cpu_read(0xE000) == 15);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0x9FFE, 6);
    cart_cpu_write(0x9FFF, 3);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xC000) == 14);
    cart_cpu_write(0x8000, 0x46);
    CHECK(cart_cpu_read(0x8000) == 14 && cart_cpu_read(0xC000) == 3);
    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 5);
    CHECK(cart_ppu_read(0) == 4 && cart_ppu_read(0x0400) == 5);
    cart_cpu_write(0x8000, 0x80);
    CHECK(cart_ppu_read(0x1000) == 4 && cart_ppu_read(0x1400) == 5);
    cart_ppu_write(0x1000, 0x77);
    CHECK(cart_ppu_read(0x1000) == 4);
    cart_cpu_write(0xA000, 1);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);

    // MC-ACC leaves its populated RAM window writable across $A001 settings.
    for (unsigned protect = 0; protect <= 0xC0; protect += 0x40) {
        cart_cpu_write(0xA001, (uint8_t)protect);
        cart_cpu_write(0x6000, (uint8_t)(protect | 0x35));
        cart_cpu_write(0x7FFF, (uint8_t)(protect | 0x1A));
        CHECK(cart_cpu_read(0x6000) == (protect | 0x35));
        CHECK(cart_cpu_read(0x7FFF) == (protect | 0x1A));
    }
    cart->reset();
    CHECK(cart_cpu_read(0x6000) == 0xF5 && !cart_irq_pending());

    size_t size;
    uint8_t *image = image_for(&h, 0x20000, 0x8000, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && ines_header.prg_ram_size == 0x30);
    cart_cpu_write(0x6000, 0x96);
    uint8_t *previous = prg_rom;
    iNESHeader invalid[] = {h, h, h};
    invalid[0].prg_ram_size = 0x20;
    invalid[1].prg_ram_size = 0x40;
    invalid[2].flags10 = 8;
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        image = image_for(&invalid[i], 0x20000, 0x8000, &size);
        CHECK(image != NULL);
        loaded = load_rom_memory(image, size);
        free(image);
        CHECK(loaded == -1 && prg_rom == previous && cart_cpu_read(0x6000) == 0x96);
    }
    h.flags10 = 0;
    h.flags6 |= 8;
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_cpu_write(0x6000, 0x77);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    cart_cpu_write(0xA000, 1);
    CHECK(cart_get_mirroring() == MIRROR_FOUR);
    return 0;
}

static int mcacc_cpu_ppu_address(uint16_t address) {
    const uint8_t program[] = {
        0xA9, (uint8_t)(address >> 8), 0x8D, 0x06, 0x20,
        0xA9, (uint8_t)address, 0x8D, 0x06, 0x20,
        0xEA // Let the delayed PPU address assignment reach the physical bus.
    };
    for (size_t i = 0; i < sizeof(program); ++i)
        write_mem((uint16_t)(0x0200 + i), program[i]);
    cpu.pc = 0x0200;
    const int cycles[] = {2, 4, 2, 4, 2};
    for (size_t i = 0; i < sizeof(cycles) / sizeof(cycles[0]); ++i)
        CHECK(cpu_step(&cpu) == cycles[i]);
    return 0;
}

static int test_mcacc_cpu_ppu_irq_path(void) {
    iNESHeader h = mcacc_header();
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    ppu_step_dots(341 * 262 * 2);
    write_mem(0x2001, 0);
    (void)read_mem(0x2002);
    fixture_prg[0x1FFFE] = 0x00;
    fixture_prg[0x1FFFF] = 0x03;
    write_mem(0xC000, 1);
    write_mem(0xC001, 0);
    write_mem(0xE001, 0);
    for (unsigned pulse = 1; pulse <= 9; ++pulse) {
        CHECK(mcacc_cpu_ppu_address(0x1000) == 0);
        CHECK(!cart_irq_pending());
        CHECK(mcacc_cpu_ppu_address(0x2000) == 0);
        CHECK(cart_irq_pending() == (pulse == 9));
    }
    for (unsigned i = 0; i < 8; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
    cpu.pc = 0x0200;
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    for (unsigned step = 0; step < 3 && cpu.pc != 0x0300; ++step) (void)cpu_step(&cpu);
    CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG));
    CHECK(cart_irq_pending());
    write_mem(0xE000, 0);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_taito_loader_transaction(void) {
    size_t image_size;
    iNESHeader h = header_for(33, 0x8000, false);
    uint8_t *image = image_for(&h, 0x8000, 0x2000, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    image = NULL;
    CHECK(rom_mapper_number(&ines_header) == 33 && cart != NULL);
    uint8_t *previous_prg = prg_rom;
    uint8_t *previous_chr = chr_rom;

    h.flags7 |= 0x08; // NES 2.0 mapper 33 with an unsupported 16KB PRG-RAM layout.
    h.flags10 = 8;
    image = image_for(&h, 0x8000, 0x2000, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == -1);
    free(image);
    image = NULL;
    CHECK(prg_rom == previous_prg && chr_rom == previous_chr);
    CHECK(rom_mapper_number(&ines_header) == 33);

    h = header_for(48, 0x8000, false);
    h.flags7 |= 0x08;
    h.prg_ram_size = 0x10; // Supported mapper 48 submapper 1.
    image = image_for(&h, 0x8000, 0x2000, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    image = NULL;
    CHECK(rom_mapper_number(&ines_header) == 48 && cart != NULL && cart->clock != NULL);
    previous_prg = prg_rom;
    previous_chr = chr_rom;

    h.prg_ram_size = 0x20; // Unknown mapper 48 submapper 2.
    image = image_for(&h, 0x8000, 0x2000, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == -1);
    free(image);
    CHECK(prg_rom == previous_prg && chr_rom == previous_chr);
    CHECK(rom_mapper_number(&ines_header) == 48);
    return 0;
}

static iNESHeader tqrom_header(size_t chr_bytes) {
    iNESHeader h = header_for(119, 0x20000, false);
    h.flags7 |= 0x08;
    h.flags10 = 7; // 8KB volatile PRG-RAM.
    h.zero[0] = 7; // 8KB volatile CHR-RAM alongside CHR-ROM.
    h.chr_rom_chunks = (uint8_t)(chr_bytes / 0x2000);
    return h;
}

static uint8_t *tqrom_image(const iNESHeader *h, size_t chr_bytes, size_t *size) {
    uint8_t *image = image_for(h, 0x20000, chr_bytes, size);
    if (!image) return NULL;
    size_t prg_offset = sizeof(*h);
    size_t chr_offset = prg_offset + 0x20000;
    for (size_t i = 0; i < 0x20000; ++i)
        image[prg_offset + i] = (uint8_t)(i / 0x2000);
    for (size_t i = 0; i < chr_bytes; ++i)
        image[chr_offset + i] = (uint8_t)(i / 0x0400);
    return image;
}

static int test_tqrom_mixed_chr_memory(void) {
    const size_t chr_bytes = 0x40000;
    iNESHeader h = tqrom_header(chr_bytes);
    size_t image_size;
    uint8_t *image = tqrom_image(&h, chr_bytes, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == 0);

    // TQROM keeps MMC3 PRG banking, work RAM, and IRQ behavior.
    cart_cpu_write(0x8000, 6);
    cart_cpu_write(0x8001, 3);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xC000) == 14);
    cart_cpu_write(0xA001, 0x80);
    cart_cpu_write(0x6000, 0xA6);
    CHECK(cart_cpu_read(0x6000) == 0xA6);
    cart_cpu_write(0xA000, 1);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    cart_cpu_write(0xA000, 0);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(0);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    CHECK(!cart_irq_pending());

    // CHR pages $40-$7F select the board's 8KB RAM before ROM bank wrapping.
    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 0x40);
    cart_ppu_write(0x0000, 0xD0);
    cart_ppu_write(0x0400, 0xD1);
    CHECK(cart_ppu_read(0x0000) == 0xD0 && cart_ppu_read(0x0400) == 0xD1);

    cart_cpu_write(0x8000, 2);
    cart_cpu_write(0x8001, 0x43);
    cart_ppu_write(0x1000, 0xD3);
    CHECK(cart_ppu_read(0x1000) == 0xD3);
    cart_cpu_write(0x8001, 0x80);
    CHECK(cart_ppu_read(0x1000) == 0x80);
    cart_ppu_write(0x1000, 0xEE);
    CHECK(cart_ppu_read(0x1000) == 0x80); // CHR-ROM stays read-only.
    cart_cpu_write(0x8001, 0x43);
    CHECK(cart_ppu_read(0x1000) == 0xD3);
    cart_cpu_write(0x8001, 0x4B);
    CHECK(cart_ppu_read(0x1000) == 0xD3); // RAM banks alias every eight pages.

    // Inverted CHR mode moves the 1KB and paired registers to the opposite half.
    cart_cpu_write(0x8000, 0x82);
    cart_cpu_write(0x8001, 0x45);
    cart_ppu_write(0x0000, 0xE5);
    CHECK(cart_ppu_read(0x0000) == 0xE5);
    cart_cpu_write(0x8001, 0x85);
    CHECK(cart_ppu_read(0x0000) == 0x85);
    cart_ppu_write(0x0000, 0x5E);
    CHECK(cart_ppu_read(0x0000) == 0x85);
    cart_cpu_write(0x8001, 0x45);
    CHECK(cart_ppu_read(0x0000) == 0xE5);
    cart_cpu_write(0x8000, 0x80);
    cart_cpu_write(0x8001, 0x40);
    CHECK(cart_ppu_read(0x1000) == 0xD0 && cart_ppu_read(0x1400) == 0xD1);

    // A mapper reset restores MMC3 registers without erasing cartridge RAM.
    cart->reset();
    CHECK(!cart_irq_pending() && cart_ppu_read(0x0000) == 0);
    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 0x40);
    CHECK(cart_ppu_read(0x0000) == 0xD0);

    // Replacing the cartridge clears mapper-owned CHR-RAM.
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    image = NULL;
    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 0x40);
    CHECK(cart_ppu_read(0x0000) == 0);

    // Legacy mapper 119 headers imply the board's fixed 8KB CHR-RAM.
    iNESHeader legacy = header_for(119, 0x20000, false);
    legacy.chr_rom_chunks = (uint8_t)(chr_bytes / 0x2000);
    image = tqrom_image(&legacy, chr_bytes, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    image = NULL;
    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 0x40);
    CHECK(cart_ppu_read(0x0000) == 0);
    cart_ppu_write(0x0000, 0x6C);

    uint8_t *previous_prg = prg_rom;
    uint8_t *previous_chr = chr_rom;
    iNESHeader invalid[] = {
        tqrom_header(chr_bytes), tqrom_header(chr_bytes), tqrom_header(chr_bytes),
        tqrom_header(chr_bytes), tqrom_header(0), tqrom_header(0x80000)
    };
    invalid[0].zero[0] = 0;    // Missing CHR-RAM.
    invalid[1].zero[0] = 6;    // 4KB is too small for TQROM.
    invalid[2].zero[0] = 8;    // 16KB is not this board's RAM geometry.
    invalid[3].zero[0] = 0x70; // CHR-NVRAM is a different storage model.
    static const size_t invalid_chr_bytes[] = {
        0x40000, 0x40000, 0x40000, 0x40000, 0, 0x80000
    };
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        image = tqrom_image(&invalid[i], invalid_chr_bytes[i], &image_size);
        CHECK(image != NULL);
        int loaded = load_rom_memory(image, image_size);
        free(image);
        CHECK(loaded == -1 && prg_rom == previous_prg && chr_rom == previous_chr);
        cart_cpu_write(0x8000, 0);
        cart_cpu_write(0x8001, 0x40);
        CHECK(cart_ppu_read(0x0000) == 0x6C);
    }
    return 0;
}

static uint8_t *txsrom_image(size_t *size) {
    const size_t prg_bytes = 0x20000, chr_bytes = 0x40000;
    iNESHeader h = header_for(118, prg_bytes, false);
    h.flags7 |= 8;
    h.flags10 = 7;
    h.chr_rom_chunks = (uint8_t)(chr_bytes / 0x2000);
    uint8_t *image = image_for(&h, prg_bytes, chr_bytes, size);
    if (!image) return NULL;
    for (size_t i = 0; i < prg_bytes; ++i) image[sizeof(h) + i] = (uint8_t)(i / 0x2000);
    for (size_t i = 0; i < chr_bytes; ++i) image[sizeof(h) + prg_bytes + i] = (uint8_t)(i / 0x0400);
    return image;
}

static int test_txsrom_nametable_routing(void) {
    size_t image_size;
    uint8_t *image = txsrom_image(&image_size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, image_size);
    free(image);
    CHECK(loaded == 0);
    const uint16_t offset = 0x012;
    memset(ppu_vram, 0, NT_RAM_SIZE);
    ppu_vram[offset] = 0x10;
    ppu_vram[0x400 + offset] = 0x20;
    cart_cpu_write(0x9FFE, 0);
    cart_cpu_write(0x9FFF, 0x80);
    cart_cpu_write(0x8000, 1);
    cart_cpu_write(0x8001, 0);
    CHECK(cart_ppu_read(0) == 0x80 && cart_ppu_read(0x0400) == 0x81);
    CHECK(ppu_read(0x2000 + offset) == 0x20 && ppu_read(0x2400 + offset) == 0x20);
    CHECK(ppu_read(0x2800 + offset) == 0x10 && ppu_read(0x2C00 + offset) == 0x10);
    ppu_write(0x2400 + offset, 0xA1);
    ppu_write(0x2C00 + offset, 0xB2);
    CHECK(ppu_vram[0x400 + offset] == 0xA1 && ppu_vram[offset] == 0xB2);
    CHECK(ppu_read(0x3000 + offset) == 0xA1 && ppu_read(0x3800 + offset) == 0xB2);

    cart_cpu_write(0x8000, 0x82);
    CHECK(ppu_read(0x2000 + offset) == 0xA1 && ppu_read(0x2400 + offset) == 0xA1);
    CHECK(ppu_read(0x2800 + offset) == 0xB2 && ppu_read(0x2C00 + offset) == 0xB2);
    cart_cpu_write(0x8001, 0x80);
    cart_cpu_write(0x8000, 0x83);
    cart_cpu_write(0x8001, 0);
    cart_cpu_write(0x8000, 0x84);
    cart_cpu_write(0x8001, 0x80);
    cart_cpu_write(0x8000, 0x85);
    cart_cpu_write(0x8001, 0);
    CHECK(cart_ppu_read(0) == 0x80 && cart_ppu_read(0x0800) == 0x80);
    CHECK(ppu_read(0x2000 + offset) == 0xA1 && ppu_read(0x2400 + offset) == 0xB2);
    CHECK(ppu_read(0x2800 + offset) == 0xA1 && ppu_read(0x2C00 + offset) == 0xB2);
    for (unsigned reg = 0; reg < 8; ++reg) {
        if (reg >= 2 && reg <= 5) continue;
        cart_cpu_write(0x8000, (uint8_t)(0x80 | reg));
        cart_cpu_write(0x8001, 0xFF);
        CHECK(ppu_read(0x2000 + offset) == 0xA1 && ppu_read(0x2400 + offset) == 0xB2);
        CHECK(ppu_read(0x2800 + offset) == 0xA1 && ppu_read(0x2C00 + offset) == 0xB2);
    }
    cart_cpu_write(0xA000, 1);
    cart_cpu_write(0xBFFE, 0);
    CHECK(ppu_read(0x2000 + offset) == 0xA1 && ppu_read(0x2400 + offset) == 0xB2);
    CHECK(ppu_read(0x2800 + offset) == 0xA1 && ppu_read(0x2C00 + offset) == 0xB2);
    cart_cpu_write(0x8000, 6);
    cart_cpu_write(0x8001, 3);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xC000) == 14);
    cart_cpu_write(0xA001, 0x80);
    cart_cpu_write(0x6000, 0x6A);
    CHECK(cart_cpu_read(0x6000) == 0x6A);
    cart_cpu_write(0xA001, 0xC0);
    cart_cpu_write(0x6000, 0xFF);
    CHECK(cart_cpu_read(0x6000) == 0x6A);
    cart_cpu_write(0xA001, 0);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    // Nametable accesses above have already driven A12 at the running PPU time.
    uint64_t irq_cycle = ppu.total_cycles;
    cart_notify_ppu_address(0x1000, irq_cycle);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x2000, irq_cycle + 3);
    cart_notify_ppu_address(0x1000, irq_cycle + 9);
    CHECK(!cart_irq_pending());
    a12_pulse(irq_cycle + 12);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_txsrom_startup_ram_and_loader(void) {
    const uint8_t flags[] = {0, 1, 8};
    const uint8_t pages[][4] = {{0, 0, 1, 1}, {0, 1, 0, 1}, {0, 1, 2, 3}};
    uint8_t nt[0x1000] = {0};
    for (unsigned page = 0; page < 4; ++page) nt[page * 0x400] = (uint8_t)(0x10 + page);
    for (unsigned i = 0; i < sizeof(flags); ++i) {
        iNESHeader h = header_for(118, 0x20000, true);
        h.flags6 |= flags[i];
        CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 118);
        for (unsigned page = 0; page < 4; ++page)
            CHECK(cart_nt_read((uint16_t)(0x2000 + page * 0x400), nt) == 0x10 + pages[i][page]);
        cart_cpu_write(0x8000, 0);
        cart_cpu_write(0x8001, 0x84);
        cart_ppu_write(0x0123, 0x96);
        cart_cpu_write(0x8001, 4);
        CHECK(cart_ppu_read(0x0123) == 0x96);
        CHECK(cart_nt_read(0x2000, nt) == 0x10 && cart_nt_read(0x2400, nt) == 0x10);
        cart_cpu_write(0x8001, 0x80);
        cart->reset();
        for (unsigned page = 0; page < 4; ++page)
            CHECK(cart_nt_read((uint16_t)(0x2000 + page * 0x400), nt) == 0x10 + pages[i][page]);
    }
    size_t bytes;
    uint8_t *image = txsrom_image(&bytes);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, bytes);
    CHECK(loaded == 0);
    uint8_t *old_prg = prg_rom, *old_chr = chr_rom;
    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 0x80);
    iNESHeader *h = (iNESHeader *)image;
    h->prg_ram_size = 0x10;
    CHECK(load_rom_memory(image, bytes) == -1);
    h->prg_ram_size = 0;
    h->flags10 = 8;
    CHECK(load_rom_memory(image, bytes) == -1);
    h->flags10 = 7;
    CHECK(load_rom_memory(image, bytes - 1) == -1);
    free(image);
    CHECK(prg_rom == old_prg && chr_rom == old_chr);
    CHECK(cart_ppu_read(0) == 0x80 && cart_nt_read(0x2000, nt) == 0x11);
    return 0;
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

    h.flags7 = 0x09; // VS hardware requires NTSC rather than the current Dendy region.
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == -1 && cart == previous_cart && prg_rom == previous_prg);
    CHECK(nes_timing()->region == NES_REGION_DENDY);

    h.flags7 = 0x0B;
    h.zero[2] = 5; // An extended console subtype not provided by this core.
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
    CHECK(ram.prg_ram == 0 && ram.prg_nvram == 0x2000);
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
    h.flags7 |= 8;
    h.prg_ram_size = 0;
    h.flags6 |= 2;
    h.flags10 = 0x90; // SXROM: 32KB battery-backed PRG RAM.
    h.zero[0] = 7;
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

    h.flags10 = 8; // 16KB volatile PRG RAM.
    h.flags6 &= (uint8_t)~2u;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    cart_cpu_write(0x6000, 1);
    serial_write(0xA000, 4);
    cart_cpu_write(0x6000, 2);
    serial_write(0xA000, 8);
    CHECK(cart_cpu_read(0x6000) == 1);
    serial_write(0xA000, 4);
    CHECK(cart_cpu_read(0x6000) == 2);

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
    char rom[128], prg_save[128], chr_save[128], flash_save[128];
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
            snprintf(paths->flash_save, sizeof(paths->flash_save), "%s/cart.flash.sav", paths->directory);
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
    if (remove(paths->flash_save) != 0 && errno != ENOENT) result = 1;
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
    h.flags7 |= 8;
    h.prg_ram_size = 0;
    h.flags6 |= 2;
    h.flags10 = 0x90;
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    cart_battery_configure(paths->rom, true);
    CHECK(cart_cpu_read(0x6000) == 0xA5 && cart_cpu_read(0x7FFF) == 0x5A);
    serial_write(0xA000, 12);
    CHECK(cart_cpu_read(0x6000) == 0 && cart_cpu_read(0x7FFF) == 0);
    cart_cpu_write(0x6000, 0x73);
    cart_cpu_write(0x7FFF, 0x39);
    cart_ppu_write(0, 0x22); // CHR-RAM is volatile.
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

    h.flags10 = 0x77;
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

static int vs_nvram_persistence_cases(const SaveFixture *paths) {
    iNESHeader h = header_for(99, 0x8000, false);
    h.flags7 |= 0x09;
    h.flags6 |= 2;
    h.prg_ram_size = 0;
    h.flags10 = 0x50; // Explicit 2 KiB battery-backed RAM.
    size_t image_size;
    uint8_t *image = image_for(&h, 0x8000, 0x2000, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    cart_battery_configure(paths->rom, true);
    cart_cpu_write(0x6000, 0x62);
    cart_cpu_write(0x67FF, 0xC3);
    cart_battery_flush();
    CHECK(saved_file_size(paths->prg_save) == 0x800);
    CHECK(saved_byte(paths->prg_save, 0) == 0x62 && saved_byte(paths->prg_save, 0x7FF) == 0xC3);
    CHECK(unload_rom());
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    cart_battery_configure(paths->rom, true);
    CHECK(cart_cpu_read(0x6000) == 0x62 && cart_cpu_read(0x67FF) == 0xC3);
    return 0;
}

static int test_vs_nvram_persistence(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    int result = vs_nvram_persistence_cases(&paths);
    unload_rom();
    return result | save_fixture_end(&paths);
}

static int test_unrom512_flash_persistence(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    iNESHeader h = unrom512_header(1, true, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    cart_battery_configure(paths.rom, true);

    // Latching banks alone must not create or dirty a flash save.
    cart_cpu_write(0xC000, 0x07);
    cart_cpu_write(0xC000, 0x02);
    cart_battery_flush();
    CHECK(saved_file_size(paths.flash_save) == -1);

    // A failed replacement keeps the flash dirty so a later flush can retry.
#ifdef _WIN32
    CHECK(_mkdir(paths.flash_save) == 0);
#else
    CHECK(mkdir(paths.flash_save, 0700) == 0);
#endif

    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 0x03);
    cart_cpu_write(0x8123, 0x04);
    CHECK(cart_cpu_read(0x8123) == 0x04);
    cart_battery_flush();
#ifdef _WIN32
    CHECK(_rmdir(paths.flash_save) == 0);
#else
    CHECK(rmdir(paths.flash_save) == 0);
#endif
    cart_battery_flush();
    CHECK(saved_file_size(paths.flash_save) == 0x40000);
    CHECK(saved_byte(paths.flash_save, 3 * 0x4000 + 0x123) == 0x04);
    CHECK(saved_file_size(paths.prg_save) == -1);

    // Reinitialization restores pristine fixture bytes before the flash save is loaded.
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    CHECK(cart_cpu_read(0x8123) == 0);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0xC000, 0x03);
    CHECK(cart_cpu_read(0x8123) == 0x04);

    // An erased sector is also persisted and restored.
    unrom512_flash_erase_prefix();
    cart_cpu_write(0xC000, 0x03);
    cart_cpu_write(0x8123, 0x30);
    CHECK(cart_cpu_read(0x8123) == 0xFF);
    cart_battery_flush();
    CHECK(saved_byte(paths.flash_save, 3 * 0x4000 + 0x123) == 0xFF);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0xC000, 0x03);
    CHECK(cart_cpu_read(0x8123) == 0xFF);

    return save_fixture_end(&paths);
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

static void sunsoft69_command(uint8_t command, uint8_t value) {
    cart_cpu_write(0x8000, command);
    cart_cpu_write(0xA000, value);
}

static void sunsoft5b_register(uint8_t reg, uint8_t value) {
    cart_cpu_write(0xC000, reg);
    cart_cpu_write(0xE000, value);
}

static void prepare_mapper_cpu_nops(void) {
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    for (unsigned i = 0; i < 0x200; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
    cpu.pc = 0x0200;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
}

static void run_mapper_nops(unsigned count) {
    for (unsigned i = 0; i < count; ++i) (void)cpu_step(&cpu);
}

static int test_sunsoft69_banks_ram_and_startup(void) {
    CHECK(fixture(69, 0x80000, 0x20000, false) == 69);
    CHECK(cart != NULL && cart->clock != NULL);
    CHECK(cart_cpu_read(0x6000) == 0);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
    CHECK(cart_cpu_read_bus(0xA000, 0x56) == 0x56);
    CHECK(cart_cpu_read_bus(0xC000, 0x56) == 0x56);
    CHECK(cart_cpu_read(0xE000) == 63);
    CHECK(cart_ppu_read(0x0123) == 0x23); // CHR-ROM starts unmapped.

    for (unsigned slot = 0; slot < 8; ++slot) {
        sunsoft69_command((uint8_t)slot, (uint8_t)(24 + slot));
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == (uint8_t)(24 + slot));
    }
    sunsoft69_command(9, 5);
    sunsoft69_command(10, 17);
    sunsoft69_command(11, 31);
    CHECK(cart_cpu_read(0x8000) == 5);
    CHECK(cart_cpu_read(0xA000) == 17);
    CHECK(cart_cpu_read(0xC000) == 31);
    CHECK(cart_cpu_read(0xE000) == 63);

    sunsoft69_command(12, 0);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    sunsoft69_command(12, 1);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    sunsoft69_command(12, 2);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    sunsoft69_command(12, 3);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);

    iNESHeader h = header_for(69, 0x20000, false);
    h.flags7 |= 0x08; // NES 2.0
    h.flags10 = 9;    // 32 KiB volatile PRG-RAM
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 69);
    CHECK(cart_cpu_read(0x6000) == 0); // Command 8 powers up as PRG-ROM bank 0.
    sunsoft69_command(8, 0x40);
    CHECK(cart_cpu_read_bus(0x6123, 0x56) == 0x56);
    cart_cpu_write(0x6123, 0x11);
    sunsoft69_command(8, 0xC3);
    cart_cpu_write(0x6123, 0xA5);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    sunsoft69_command(8, 0xC0);
    CHECK(cart_cpu_read(0x6123) == 0);
    sunsoft69_command(8, 0x43);
    CHECK(cart_cpu_read_bus(0x6123, 0x56) == 0x56);
    cart_cpu_write(0x6123, 0x33);
    sunsoft69_command(8, 0xC3);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    sunsoft69_command(8, 3);
    CHECK(cart_cpu_read(0x6123) == 3);

    CHECK(fixture(69, 0x20000, 0x2000, true) == 69);
    CHECK(cart_ppu_read(0x0423) == 1); // CHR-RAM is linearly mapped at power-on.
    sunsoft69_command(0, 7);
    cart_ppu_write(0x0123, 0xA6);
    CHECK(cart_ppu_read(0x0123) == 0xA6);
    sunsoft69_command(0, 0);
    CHECK(cart_ppu_read(0x0123) == 0);
    sunsoft69_command(0, 7);
    CHECK(cart_ppu_read(0x0123) == 0xA6);
    return 0;
}

static int test_sunsoft69_legacy_ram_defaults(void) {
    for (unsigned battery = 0; battery < 2; ++battery) {
        iNESHeader h = header_for(69, 0x20000, false);
        h.flags6 |= (uint8_t)(battery << 1);
        h.prg_ram_size = 0;
        RomRamSizes sizes;
        CHECK(rom_ram_sizes(&h, &sizes) == 0);
        CHECK(sizes.prg_ram == (battery ? 0u : 0x8000u));
        CHECK(sizes.prg_nvram == (battery ? 0x8000u : 0u));
        CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 69);
        for (unsigned bank = 0; bank < 4; ++bank) {
            sunsoft69_command(8, (uint8_t)(0xC0u | bank));
            cart_cpu_write(0x6123, (uint8_t)(0xA0u + bank));
        }
        for (unsigned bank = 0; bank < 4; ++bank) {
            sunsoft69_command(8, (uint8_t)(0xC0u | bank));
            CHECK(cart_cpu_read(0x6123) == (uint8_t)(0xA0u + bank));
        }
        h.prg_ram_size = 1;
        CHECK(rom_ram_sizes(&h, &sizes) == 0);
        CHECK(sizes.prg_ram + sizes.prg_nvram == 0x8000);
    }
    return 0;
}

static int test_sunsoft69_irq_cpu_clock(void) {
    CHECK(fixture(69, 0x20000, 0x2000, false) == 69);
    prepare_mapper_cpu_nops();

    sunsoft69_command(14, 1);
    sunsoft69_command(15, 0);
    sunsoft69_command(13, 0x81);
    CHECK(!cart_irq_pending());
    CHECK(cpu_step(&cpu) == 2);
    CHECK(cart_irq_pending()); // 0001 -> 0000 -> FFFF on the two NOP cycles.

    sunsoft69_command(13, 0x80);
    CHECK(!cart_irq_pending());
    sunsoft69_command(14, 0);
    sunsoft69_command(15, 0);
    CHECK(cpu_step(&cpu) == 2);
    CHECK(!cart_irq_pending()); // Counter runs, IRQ output is disabled.
    sunsoft69_command(13, 0x01);
    CHECK(!cart_irq_pending()); // Enabling output later does not replay the old edge.

    sunsoft69_command(14, 0);
    sunsoft69_command(15, 0);
    sunsoft69_command(13, 0x81);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
    sunsoft69_command(13, 0x00);
    CHECK(!cart_irq_pending());
    run_mapper_nops(4);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_sunsoft5b_tone_noise_envelope(void) {
    CHECK(fixture(69, 0x20000, 0x2000, false) == 69);
    sunsoft5b_register(0, 1);
    sunsoft5b_register(1, 0);
    sunsoft5b_register(7, 0x3E); // A tone enabled, all noise and B/C tone disabled.
    sunsoft5b_register(8, 15);
    prepare_mapper_cpu_nops();
    // CPU power-on contributed seven mapper clocks. Eight more leave the
    // shared divide-by-16 phase one clock short of its first PSG tick.
    run_mapper_nops(4);
    CHECK(cart_expansion_audio() == 0.0f);
    run_mapper_nops(1); // Cross clock 16 and toggle tone A high.
    float full_level = cart_expansion_audio();
    CHECK(full_level < -0.125f && full_level > -0.127f);
    cart_cpu_write(0xC000, 0x18); // Nonzero selector high nibble blocks data writes.
    cart_cpu_write(0xE000, 0);
    CHECK(cart_expansion_audio() == full_level);
    run_mapper_nops(7);
    CHECK(cart_expansion_audio() == full_level);
    run_mapper_nops(1); // Cross clock 32 and toggle tone A low.
    CHECK(cart_expansion_audio() == 0.0f);

    CHECK(fixture(69, 0x20000, 0x2000, false) == 69);
    sunsoft5b_register(6, 1);
    sunsoft5b_register(7, 0x37); // Constant tone gate, A noise enabled; B/C muted.
    sunsoft5b_register(8, 15);
    prepare_mapper_cpu_nops();
    float noise_high = cart_expansion_audio();
    CHECK(noise_high < -0.125f && noise_high > -0.127f);
    run_mapper_nops(12); // Global clock 31: only one half of the noise period elapsed.
    CHECK(cart_expansion_audio() == noise_high);
    run_mapper_nops(1); // Cross global clock 32 and advance the 17-bit LFSR once.
    CHECK(cart_expansion_audio() == 0.0f);

    CHECK(fixture(69, 0x20000, 0x2000, false) == 69);
    sunsoft5b_register(7, 0x3F); // Disabled generators are high mixer inputs.
    sunsoft5b_register(8, 0x10); // Channel A uses the shared envelope.
    sunsoft5b_register(0x0B, 1);
    sunsoft5b_register(0x0C, 0);
    sunsoft5b_register(0x0D, 0x0C); // Repeating rising saw envelope.
    prepare_mapper_cpu_nops();
    CHECK(cart_expansion_audio() == 0.0f);
    run_mapper_nops(4);
    CHECK(cart_expansion_audio() == 0.0f);
    run_mapper_nops(1); // Cross clock 16: level 1 still aliases silence.
    CHECK(cart_expansion_audio() == 0.0f);
    run_mapper_nops(8); // Level 2 is the first nonzero 1.5 dB step.
    float first_envelope_level = cart_expansion_audio();
    CHECK(first_envelope_level < -0.0008f && first_envelope_level > -0.0009f);
    run_mapper_nops(29 * 8); // Reach level 31 without reading private PSG state.
    CHECK(cart_expansion_audio() < -0.125f && cart_expansion_audio() > -0.127f);
    run_mapper_nops(8); // The next envelope period restarts the saw at zero.
    CHECK(cart_expansion_audio() == 0.0f);
    return 0;
}

static int test_sunsoft69_persistence_and_loader(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    iNESHeader h = header_for(69, 0x20000, false);
    h.flags7 |= 0x08;
    h.flags6 |= 0x02;
    h.flags10 = 0x90; // 32 KiB PRG-NVRAM
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 69);
    cart_battery_configure(paths.rom, true);
    sunsoft69_command(8, 0xC2);
    cart_cpu_write(0x6123, 0xA5);
    sunsoft69_command(8, 0xC3);
    cart_cpu_write(0x6123, 0x5A);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x8000);
    CHECK(saved_byte(paths.prg_save, 0x4000 + 0x123) == 0xA5);
    CHECK(saved_byte(paths.prg_save, 0x6000 + 0x123) == 0x5A);

    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 69);
    cart_battery_configure(paths.rom, true);
    sunsoft69_command(8, 0xC2);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    sunsoft69_command(8, 0xC3);
    CHECK(cart_cpu_read(0x6123) == 0x5A);

    Mapper *previous = cart;
    sunsoft69_command(9, 3);
    CHECK(cart_cpu_read(0x8000) == 3);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x80001, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);
    h.flags10 = 0xE0; // 1 MiB NVRAM exceeds the six-bit 8 KiB bank selector.
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);
    return save_fixture_end(&paths);
}

static iNESHeader namco210_header(unsigned submapper, bool ram) {
    iNESHeader h = header_for(210, 0x20000, false);
    h.flags7 |= 0x08;
    h.prg_ram_size = (uint8_t)(submapper << 4);
    h.flags10 = ram ? 7 : 0;
    return h;
}

static void namco163_ram_write(uint8_t address, uint8_t value) {
    cart_cpu_write(0xF800, address);
    cart_cpu_write(0x4800, value);
}

static int test_namco163_banks_ram_and_nametables(void) {
    CHECK(fixture(19, 0x40000, 0x20000, false) == 19);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
    CHECK(cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_cpu_read_bus(0xC000, 0x69) == 0x69);
    CHECK(cart_cpu_read(0xE000) == 31);

    cart_cpu_write(0xE000, 3);
    cart_cpu_write(0xE800, 5);
    cart_cpu_write(0xF000, 7);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xA000) == 5);
    CHECK(cart_cpu_read(0xC000) == 7 && cart_cpu_read(0xE000) == 31);

    cart_cpu_write(0x8000, 4);
    cart_cpu_write(0xA800, 9);
    CHECK(cart_ppu_read(0x0000) == 4 && cart_ppu_read(0x1400) == 9);

    memset(ppu_vram, 0, sizeof(ppu_vram));
    ppu_vram[0x400] = 0xA5;
    cart_cpu_write(0x8000, 0xE1);
    CHECK(cart_ppu_read(0x0000) == 0xA5);
    cart_cpu_write(0xE800, 0x45); // Force low pattern banks to CHR even for $E0-$FF values.
    cart_cpu_write(0x8000, 0xE1);
    CHECK(cart_ppu_read(0x0000) == (0xE1u % 0x80u));

    ppu_vram[0] = 0x35;
    cart_cpu_write(0xC000, 0xE0);
    cart_cpu_write(0xC800, 3);
    CHECK(cart_nt_read(0x2000, ppu_vram) == 0x35);
    CHECK(cart_nt_read(0x2400, ppu_vram) == 3);
    cart_nt_write(0x2001, 0x53, ppu_vram);
    CHECK(ppu_vram[1] == 0x53);

    cart_cpu_write(0xF800, 0x40); // Global RAM writes enabled, all four 2 KiB blocks writable.
    cart_cpu_write(0x6123, 0x11);
    cart_cpu_write(0x6923, 0x22);
    cart_cpu_write(0x7123, 0x33);
    cart_cpu_write(0x7923, 0x44);
    CHECK(cart_cpu_read(0x6123) == 0x11 && cart_cpu_read(0x6923) == 0x22);
    CHECK(cart_cpu_read(0x7123) == 0x33 && cart_cpu_read(0x7923) == 0x44);
    cart_cpu_write(0xF800, 0x45); // Protect blocks 0 and 2 while keeping global write enable.
    cart_cpu_write(0x6123, 0xA1);
    cart_cpu_write(0x6923, 0xA2);
    cart_cpu_write(0x7123, 0xA3);
    cart_cpu_write(0x7923, 0xA4);
    CHECK(cart_cpu_read(0x6123) == 0x11 && cart_cpu_read(0x6923) == 0xA2);
    CHECK(cart_cpu_read(0x7123) == 0x33 && cart_cpu_read(0x7923) == 0xA4);
    cart_cpu_write(0xF800, 0x05); // Global write disable preserves readable RAM.
    cart_cpu_write(0x6923, 0x55);
    CHECK(cart_cpu_read(0x6923) == 0xA2);
    return 0;
}

static int test_namco163_irq_and_audio(void) {
    CHECK(fixture(19, 0x20000, 0x20000, false) == 19);
    CHECK(cart != NULL && cart->clock != NULL && cart_expansion_audio() == 0.0f);

    cart_cpu_write(0x5000, 0xFD);
    cart_cpu_write(0x5800, 0xFF);
    cart->clock(1);
    CHECK(!cart_irq_pending() && cart_cpu_read(0x5000) == 0xFE);
    cart->clock(1);
    CHECK(cart_irq_pending() && cart_cpu_read(0x5000) == 0xFF);
    cart_cpu_write(0x5000, 0);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0x5800, 0x7F);
    cart->clock(8);
    CHECK(!cart_irq_pending() && cart_cpu_read(0x5800) == 0x7F);

    fixture_prg[0] = 0xEA;
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cart_cpu_write(0xE000, 0);
    cpu.pc = 0x8000;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
    cart_cpu_write(0x5000, 0xFD);
    cart_cpu_write(0x5800, 0xFF);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
    cart_cpu_write(0x5800, 0);

    cart_cpu_write(0xF800, 0x80);
    cart_cpu_write(0x4800, 0x12);
    cart_cpu_write(0x4800, 0x34);
    cart_cpu_write(0xF800, 0x80);
    CHECK(cart_cpu_read(0x4800) == 0x12);
    CHECK(cart_cpu_read(0x4800) == 0x34);

    cart->reset();
    namco163_ram_write(0x00, 0x00);
    namco163_ram_write(0x78, 0x01);
    namco163_ram_write(0x79, 0x00);
    namco163_ram_write(0x7A, 0x00);
    namco163_ram_write(0x7B, 0x00);
    namco163_ram_write(0x7C, 0x00);
    namco163_ram_write(0x7D, 0x00);
    namco163_ram_write(0x7E, 0x00);
    namco163_ram_write(0x7F, 0x0F);
    cart->clock(14);
    CHECK(cart_expansion_audio() == 0.0f);
    cart->clock(1);
    CHECK(cart_expansion_audio() > 0.47f && cart_expansion_audio() < 0.49f);

    float held = cart_expansion_audio();
    cart_cpu_write(0xE000, 0x40);
    cart->clock(30);
    CHECK(cart_expansion_audio() == held);

    cart->reset();
    namco163_ram_write(0x78, 1);
    namco163_ram_write(0x70, 2);
    namco163_ram_write(0x7F, 0x10); // Channels seven and six share the 15-cycle sequencer.
    cart->clock(14);
    cart_cpu_write(0xF800, 0x79);
    CHECK(cart_cpu_read(0x4800) == 0);
    cart_cpu_write(0xE000, 0x40);
    cart->clock(60);
    cart_cpu_write(0xE000, 0);
    cart->clock(1);
    cart_cpu_write(0xF800, 0x79);
    CHECK(cart_cpu_read(0x4800) == 1);
    cart_cpu_write(0xF800, 0x71);
    CHECK(cart_cpu_read(0x4800) == 0);
    cart->clock(15);
    cart_cpu_write(0xF800, 0x71);
    CHECK(cart_cpu_read(0x4800) == 2);
    cart->clock(15);
    cart_cpu_write(0xF800, 0x79);
    CHECK(cart_cpu_read(0x4800) == 2);
    return 0;
}

static int test_namco175_340_variants(void) {
    iNESHeader h = namco210_header(1, true);
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 210);
    CHECK(cart_cpu_read(0xE000) == 15);
    CHECK(cart_cpu_read_bus(0x4800, 0xA6) == 0xA6);
    CHECK(cart_cpu_read_bus(0x5000, 0x53) == 0x53);
    cart_cpu_write(0x6123, 0x11);
    CHECK(cart_cpu_read(0x6123) == 0);
    cart_cpu_write(0xC000, 1);
    cart_cpu_write(0x6123, 0xA5);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0x6123, 0x5A);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    cart_cpu_write(0x5000, 0xFF);
    cart_cpu_write(0x5800, 0xFF);
    cart->clock(8);
    CHECK(!cart_irq_pending() && cart_expansion_audio() == 0.0f);

    h = namco210_header(2, true);
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 210);
    CHECK(cart_cpu_read_bus(0x6000, 0x69) == 0x69);
    cart_cpu_write(0x6000, 0xA5);
    CHECK(cart_cpu_read_bus(0x6000, 0x69) == 0x69);
    cart_cpu_write(0xE000, 0x03);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0xE000, 0x43);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0xE000, 0x83);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    cart_cpu_write(0xE000, 0xC3);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    memset(ppu_vram, 0, sizeof(ppu_vram));
    ppu_vram[0x400] = 0x96;
    cart_cpu_write(0xC000, 0xE1);
    CHECK(cart_nt_read(0x2000, ppu_vram) == 0x96);
    cart_cpu_write(0xE000, 0x03);
    CHECK(cart_nt_read(0x2000, ppu_vram) == 0);
    CHECK(cart_nt_read(0x2400, ppu_vram) == 0);
    cart_cpu_write(0x5000, 0xFF);
    cart_cpu_write(0x5800, 0xFF);
    cart->clock(8);
    CHECK(!cart_irq_pending() && cart_cpu_read_bus(0x5000, 0x53) == 0x53);
    return 0;
}

static int test_namco163_persistence_and_loader(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    iNESHeader h = header_for(19, 0x20000, false);
    h.flags6 |= 0x02;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 19);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0xF800, 0x40);
    cart_cpu_write(0x6123, 0xA5);
    namco163_ram_write(0, 0x5A);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x2080);
    CHECK(saved_byte(paths.prg_save, 0x123) == 0xA5);
    CHECK(saved_byte(paths.prg_save, 0x2000) == 0x5A);

    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 19);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0xF800, 0x40);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    cart_cpu_write(0xF800, 0);
    CHECK(cart_cpu_read(0x4800) == 0x5A);

    // Phase bytes are writable audio RAM, including writes performed by the chip.
    namco163_ram_write(0x78, 1);
    cart_battery_flush();
    CHECK(saved_byte(paths.prg_save, 0x2079) == 0);
    cart->clock(15);
    cart_battery_flush();
    CHECK(saved_byte(paths.prg_save, 0x2079) == 1);
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 19);
    cart_battery_configure(paths.rom, true);
    cart->clock(15); // No CPU write after loading the saved audio state.
    cart_battery_flush();
    CHECK(saved_byte(paths.prg_save, 0x2079) == 2);

    cart_cpu_write(0xE000, 3);
    Mapper *previous = cart;
    iNESHeader invalid = namco210_header(3, true);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x20000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x80001, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);
    return save_fixture_end(&paths);
}

static iNESHeader mapper34_header(unsigned submapper, bool chr_ram, bool nes2) {
    iNESHeader h = header_for(34, 0x40000, chr_ram);
    if (nes2) {
        h.flags7 |= 0x08;
        h.prg_ram_size = (uint8_t)(submapper << 4);
        h.flags10 = 7;
        if (chr_ram) h.zero[0] = 7;
    }
    return h;
}

static int test_mapper34_bnrom_banking_and_ram(void) {
    iNESHeader h = mapper34_header(2, true, true);
    CHECK(fixture_with_header(&h, 0x40000, 0x2000) == 34);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xE000) == 3);
    CHECK(cart_ppu_read(0x0123) == 0);
    cart_ppu_write(0x0123, 0xA5);
    CHECK(cart_ppu_read(0x0123) == 0xA5);
    cart_cpu_write(0x6123, 0x5A);
    CHECK(cart_cpu_read(0x6123) == 0x5A);

    cart_cpu_write(0xE000, 3); // Bank-0 ROM drives 3 here, so the bus-conflicted write keeps 3.
    CHECK(cart_cpu_read(0x8000) == 12 && cart_cpu_read(0xE000) == 15);
    cart_cpu_write(0x8000, 7); // Current ROM drives 12; 7 & 12 selects bank 4.
    CHECK(cart_cpu_read(0x8000) == 16);
    cart_cpu_write(0xE000, 0xFF); // Current ROM drives 19; wrapped bank value selects bank 3.
    CHECK(cart_cpu_read(0x8000) == 12);

    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0x6123) == 0x5A);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    return 0;
}

static int test_mapper34_nina_banks_and_selection(void) {
    iNESHeader h = mapper34_header(1, false, true);
    CHECK(fixture_with_header(&h, 0x40000, 0x20000) == 34);
    CHECK(cart_cpu_read(0x8000) == 0);
    CHECK(cart_ppu_read(0x0000) == 0x00 && cart_ppu_read(0x1001) == 0x01);

    cart_cpu_write(0x7FFD, 5);
    cart_cpu_write(0x7FFE, 9);
    cart_cpu_write(0x7FFF, 17);
    CHECK(cart_cpu_read(0x8000) == 20);
    CHECK(cart_ppu_read(0x0000) == 36 && cart_ppu_read(0x1000) == 68);
    CHECK(cart_cpu_read(0x7FFD) == 5 && cart_cpu_read(0x7FFE) == 9 && cart_cpu_read(0x7FFF) == 17);

    cart_cpu_write(0x7FFC, 0xA5);
    CHECK(cart_cpu_read(0x7FFC) == 0xA5 && cart_cpu_read(0x8000) == 20);
    cart_cpu_write(0x8000, 2); // NINA does not decode the BNROM register range.
    CHECK(cart_cpu_read(0x8000) == 20);
    cart_cpu_write(0x7FFE, 0xFF);
    CHECK(cart_ppu_read(0x0000) == 124); // 255 wraps across 32 4 KiB CHR banks.

    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 0);
    CHECK(cart_ppu_read(0x0000) == 0x00 && cart_ppu_read(0x1001) == 0x01);
    CHECK(cart_cpu_read(0x7FFC) == 0xA5);

    iNESHeader legacy_nina = header_for(34, 0x40000, false);
    CHECK(fixture_with_header(&legacy_nina, 0x40000, 0x2000) == 34);
    cart_cpu_write(0x7FFD, 2);
    CHECK(cart_cpu_read(0x8000) == 8);

    iNESHeader legacy_bnrom = header_for(34, 0x40000, true);
    CHECK(fixture_with_header(&legacy_bnrom, 0x40000, 0x2000) == 34);
    cart_cpu_write(0xE000, 2);
    CHECK(cart_cpu_read(0x8000) == 8);
    return 0;
}

static int test_mapper34_loader_preserves_cart(void) {
    iNESHeader good = mapper34_header(1, false, true);
    CHECK(fixture_with_header(&good, 0x40000, 0x2000) == 34);
    cart_cpu_write(0x7FFD, 3);
    cart_cpu_write(0x6123, 0xA6);
    Mapper *previous = cart;

    iNESHeader invalid = mapper34_header(3, false, true);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 12 && cart_cpu_read(0x6123) == 0xA6);

    invalid = mapper34_header(2, false, true); // Explicit BNROM cannot use CHR ROM.
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 12);

    invalid = mapper34_header(1, true, true); // Explicit NINA-001 requires CHR ROM.
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 12);

    invalid = mapper34_header(1, false, true);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x4001, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 12);
    return 0;
}

static int test_mapper34_image_loading(void) {
    for (unsigned submapper = 1; submapper <= 2; ++submapper) {
        bool chr_ram = submapper == 2;
        size_t chr_bytes = chr_ram ? 0 : 0x10000;
        iNESHeader h = mapper34_header(submapper, chr_ram, true);
        h.chr_rom_chunks = (uint8_t)(chr_bytes / 0x2000);
        size_t size;
        uint8_t *image = image_for(&h, 0x40000, chr_bytes, &size);
        CHECK(image != NULL);
        for (size_t i = 0; i < 0x40000; ++i)
            image[sizeof(h) + i] = (uint8_t)(i / 0x2000);
        for (size_t i = 0; i < chr_bytes; ++i)
            image[sizeof(h) + 0x40000 + i] = (uint8_t)(i / 0x0400);
        CHECK(load_rom_memory(image, size) == 0);
        CHECK(rom_mapper_number(&ines_header) == 34);
        cart_cpu_write(chr_ram ? 0xE000 : 0x7FFD, 3);
        cart_cpu_write(0x6123, 0xA6);
        CHECK(cart_cpu_read(0x8000) == 12 && cart_cpu_read(0x6123) == 0xA6);
        if (chr_ram) {
            cart_ppu_write(0x0123, 0xE4);
        } else {
            cart_cpu_write(0x7FFE, 5);
            cart_cpu_write(0x7FFF, 7);
            CHECK(cart_ppu_read(0x0123) == 20 && cart_ppu_read(0x1123) == 28);
            cart_ppu_write(0x0123, 0xE4);
            CHECK(cart_ppu_read(0x0123) == 20);
        }
        Mapper *previous = cart;
        iNESHeader active = ines_header;
        image[8] = 0x30; // Unsupported board metadata must leave the selected banks and RAM intact.
        CHECK(load_rom_memory(image, size) == -1);
        memcpy(image, &h, sizeof(h));
        CHECK(load_rom_memory(image, size - 1) == -1);
        free(image);
        CHECK(cart == previous && memcmp(&ines_header, &active, sizeof(active)) == 0);
        CHECK(cart_cpu_read(0x8000) == 12 && cart_cpu_read(0x6123) == 0xA6);
        CHECK(cart_ppu_read(0x0123) == (chr_ram ? 0xE4 : 20));
    }
    return 0;
}

static int test_gxrom_banks_reset_and_ram(void) {
    CHECK(fixture(66, 0x20000, 0x8000, false) == 66);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_ppu_read(0x0000) == 0);
    cart_cpu_write(0x8000, 0x31);
    CHECK(cart_cpu_read(0x8000) == 12 && cart_ppu_read(0x0000) == 8);
    cart_cpu_write(0xFFFF, 0x23);
    CHECK(cart_cpu_read(0x8000) == 8 && cart_ppu_read(0x0000) == 24);

    fixture_prg[2 * 0x8000] = 0x00;
    cart_cpu_write(0x8000, 0x11); // No bus conflict: visible ROM zero must not clear register bits.
    CHECK(cart_cpu_read(0x8000) == 4 && cart_ppu_read(0x0000) == 8);

    cart_cpu_write(0x6123, 0xA5);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    uint8_t before = cart_ppu_read(0x0123);
    cart_ppu_write(0x0123, 0x5A);
    CHECK(cart_ppu_read(0x0123) == before);
    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 0 && cart_ppu_read(0x0000) == 0);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    return 0;
}

static int test_gxrom_chr_ram_and_loader(void) {
    CHECK(fixture(66, 0x10000, 0x2000, true) == 66);
    cart_ppu_write(0x0123, 0xA6);
    cart_cpu_write(0x8000, 0x33);
    CHECK(cart_cpu_read(0x8000) == 4);
    CHECK(cart_ppu_read(0x0123) == 0xA6);

    iNESHeader h = header_for(66, 0x20000, false);
    h.chr_rom_chunks = 4;
    size_t size;
    uint8_t *image = image_for(&h, 0x20000, 0x8000, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && rom_mapper_number(&ines_header) == 66);
    cart_cpu_write(0x8000, 0x32);
    CHECK(cart_cpu_read(0x8000) == 0x5C && cart_ppu_read(0x0000) == 0xA5);

    Mapper *previous = cart;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20001, fixture_chr, 0x8000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0x5C);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x8001) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0x5C);

    h.flags7 |= 0x08;
    h.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x8000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0x5C);
    return 0;
}

static int test_mapper71_variants_and_mirroring(void) {
    CHECK(fixture(71, 0x20000, 0x2000, true) == 71);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 14);
    cart_cpu_write(0x8000, 3);
    CHECK(cart_cpu_read(0x8000) == 6);
    cart_cpu_write(0x9000, 0x10);
    CHECK(cart_cpu_read(0x8000) == 6 && cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0xA000, 0);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1 && cart_cpu_read(0x8000) == 6);
    uint8_t nt[0x1000] = {0};
    cart_nt_write(0x2001, 0x5A, nt);
    CHECK(nt[0x401] == 0x5A && cart_nt_read(0x2C01, nt) == 0x5A);
    cart_cpu_write(0xA000, 0x10);
    CHECK(cart_nt_read(0x2001, nt) == 0);
    cart_nt_write(0x2401, 0xB6, nt);
    CHECK(nt[1] == 0xB6 && nt[0x401] == 0x5A);
    cart_cpu_write(0xA000, 0);
    CHECK(cart_nt_read(0x2801, nt) == 0x5A);
    cart_cpu_write(0xC000, 5);
    CHECK(cart_cpu_read(0x8000) == 10 && cart_cpu_read(0xC000) == 14);
    cart_cpu_write(0x6123, 0xA5);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    cart_ppu_write(0x0123, 0x5A);
    CHECK(cart_ppu_read(0x0123) == 0x5A);
    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 0 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    cart_cpu_write(0x8000, 2); // Legacy mode returns to ordinary bank decoding on reset.
    CHECK(cart_cpu_read(0x8000) == 4);

    iNESHeader h = header_for(71, 0x20000, true);
    h.flags7 |= 0x08;
    h.prg_ram_size = 0x10;
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 71);
    cart_cpu_write(0x8000, 0x10);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0xB000, 0);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    cart_cpu_write(0xC000, 6);
    CHECK(cart_cpu_read(0x8000) == 12);
    cart->reset();
    cart_cpu_write(0x8000, 0x10);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_get_mirroring() == MIRROR_SINGLE0);
    return 0;
}

static int test_mapper71_loader_and_chr_rom(void) {
    iNESHeader h = header_for(71, 0x20000, false);
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 71);
    uint8_t chr_before = cart_ppu_read(0x0123);
    cart_ppu_write(0x0123, 0xA6);
    CHECK(cart_ppu_read(0x0123) == chr_before);
    cart_cpu_write(0xC000, 2); // No bus conflict on the BF909x bank register.
    CHECK(cart_cpu_read(0x8000) == 4);
    Mapper *previous = cart;

    iNESHeader invalid = h;
    invalid.flags7 |= 0x08;
    invalid.prg_ram_size = 0x20;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x20000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 4);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20001, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 4);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x4000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 4);
    return 0;
}

static int test_mapper71_image_loading(void) {
    for (unsigned submapper = 0; submapper <= 1; ++submapper) {
        iNESHeader h = header_for(71, 0x20000, true);
        h.flags7 |= 0x08;
        h.prg_ram_size = (uint8_t)(submapper << 4);
        h.flags10 = 7;
        h.zero[0] = 7;
        size_t size;
        uint8_t *image = image_for(&h, 0x20000, 0, &size);
        CHECK(image != NULL);
        for (size_t i = 0; i < 0x20000; ++i)
            image[sizeof(h) + i] = (uint8_t)(i / 0x4000);
        CHECK(load_rom_memory(image, size) == 0);
        CHECK(rom_mapper_number(&ines_header) == 71 && cart_cpu_read(0xC000) == 7);
        cart_cpu_write(0x8000, 3);
        CHECK(cart_cpu_read(0x8000) == (submapper ? 0 : 3));
        cart_cpu_write(0xC000, 5);
        cart_cpu_write(0x6123, 0xA6);
        cart_ppu_write(0x0123, 0xC7);
        Mapper *previous = cart;
        iNESHeader active = ines_header;
        image[8] = 0x20;
        CHECK(load_rom_memory(image, size) == -1);
        memcpy(image, &h, sizeof(h));
        CHECK(load_rom_memory(image, size - 1) == -1);
        free(image);
        CHECK(cart == previous && memcmp(&ines_header, &active, sizeof(active)) == 0);
        CHECK(cart_cpu_read(0x8000) == 5 && cart_cpu_read(0xC000) == 7);
        CHECK(cart_cpu_read(0x6123) == 0xA6 && cart_ppu_read(0x0123) == 0xC7);
    }
    return 0;
}

static int test_legacy_loader_ram_and_large_prg_metadata(void) {
    const struct {
        unsigned mapper;
        size_t prg_bytes;
        size_t chr_bytes;
        bool chr_ram;
    } ram_cases[] = {
        {34, 0x40000, 0x2000, true},
        {66, 0x20000, 0x2000, false},
        {71, 0x20000, 0x2000, true},
        {206, 0x20000, 0x2000, false},
    };
    for (size_t i = 0; i < sizeof(ram_cases) / sizeof(ram_cases[0]); ++i) {
        iNESHeader h = header_for(ram_cases[i].mapper, ram_cases[i].prg_bytes,
                                  ram_cases[i].chr_ram);
        h.prg_ram_size = 2; // Legacy byte 8 does not override the board's 8 KiB default.
        CHECK(fixture_with_header(&h, ram_cases[i].prg_bytes, ram_cases[i].chr_bytes)
              == (int)ram_cases[i].mapper);
        cart_cpu_write(0x6123, 0xA5);
        cart_cpu_write(0x7FFF, 0x5A);
        CHECK(cart_cpu_read(0x6123) == 0xA5 && cart_cpu_read(0x7FFF) == 0x5A);
    }

    iNESHeader bnrom = header_for(34, 0x400000, true);
    CHECK(bnrom.prg_rom_chunks == 0);
    size_t bnrom_size;
    uint8_t *bnrom_image = image_for(&bnrom, 0x400000, 0, &bnrom_size);
    CHECK(bnrom_image != NULL);
    for (size_t bank = 0; bank < 128; ++bank)
        memset(bnrom_image + sizeof(bnrom) + bank * 0x8000, (uint8_t)bank, 0x8000);
    bnrom_image[sizeof(bnrom) + 0x7FFF] = 0xFF;
    CHECK(load_rom_memory(bnrom_image, bnrom_size) == 0);
    CHECK(cart_cpu_read(0x8000) == 0);
    cart_cpu_write(0xFFFF, 0x7F);
    CHECK(cart_cpu_read(0x8000) == 0x7F);
    Mapper *previous = cart;
    CHECK(load_rom_memory(bnrom_image, bnrom_size - 1) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0x7F);
    free(bnrom_image);

    iNESHeader bf909x = header_for(71, 0x400000, false);
    CHECK(bf909x.prg_rom_chunks == 0);
    size_t bf909x_size;
    uint8_t *bf909x_image = image_for(&bf909x, 0x400000, 0x2000, &bf909x_size);
    CHECK(bf909x_image != NULL);
    for (size_t bank = 0; bank < 256; ++bank)
        memset(bf909x_image + sizeof(bf909x) + bank * 0x4000, (uint8_t)bank, 0x4000);
    CHECK(load_rom_memory(bf909x_image, bf909x_size) == 0);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 0xFF);
    cart_cpu_write(0xC000, 0xFE);
    CHECK(cart_cpu_read(0x8000) == 0xFE && cart_cpu_read(0xC000) == 0xFF);
    previous = cart;
    CHECK(load_rom_memory(bf909x_image, bf909x_size - 1) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0xFE && cart_cpu_read(0xC000) == 0xFF);
    free(bf909x_image);

    iNESHeader nes2_zero = mapper34_header(2, true, true);
    nes2_zero.prg_rom_chunks = 0;
    size_t nes2_zero_size;
    uint8_t *nes2_zero_image = image_for(&nes2_zero, 0, 0, &nes2_zero_size);
    CHECK(nes2_zero_image != NULL);
    previous = cart;
    CHECK(load_rom_memory(nes2_zero_image, nes2_zero_size) == -1);
    free(nes2_zero_image);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0xFE && cart_cpu_read(0xC000) == 0xFF);
    return 0;
}

static void namco108_write_bank_at(uint16_t select_address, uint16_t data_address,
                                   uint8_t reg, uint8_t value) {
    cart_cpu_write(select_address, (uint8_t)(0xF8u | reg));
    cart_cpu_write(data_address, value);
}

static void namco108_write_bank(uint8_t reg, uint8_t value) {
    namco108_write_bank_at(0x9FFEu, 0x9FFFu, reg, value);
}

static int test_namco108_banks_aliases_and_irq_absence(void) {
    CHECK(fixture(206, 0x200000, 0x40000, false) == 206);
    CHECK(cart != NULL && cart->clock == NULL);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 254 && cart_cpu_read(0xE000) == 255);
    CHECK(cart_ppu_read(0x0000) == 0 && cart_ppu_read(0x0400) == 1);
    CHECK(cart_ppu_read(0x0800) == 2 && cart_ppu_read(0x0C00) == 3);
    CHECK(cart_ppu_read(0x1000) == 4 && cart_ppu_read(0x1C00) == 7);

    // Only A0 distinguishes the selector and data ports anywhere in $8000-$9FFF.
    namco108_write_bank_at(0x8ABCu, 0x91FDu, 6, 0xC3);
    namco108_write_bank_at(0x9000u, 0x8001u, 7, 0x85);
    CHECK(cart_cpu_read(0x8000) == 0xC3 && cart_cpu_read(0xA000) == 0x85);
    CHECK(cart_cpu_read(0xC000) == 254 && cart_cpu_read(0xE000) == 255);

    // Only the two paired CHR registers ignore the low data bit.
    namco108_write_bank(0, 0x49);
    namco108_write_bank(1, 0x4D);
    namco108_write_bank(2, 0x85);
    namco108_write_bank(3, 0xA2);
    namco108_write_bank(4, 0xC7);
    namco108_write_bank(5, 0xF8);
    CHECK(cart_ppu_read(0x0000) == 0x48 && cart_ppu_read(0x0400) == 0x49);
    CHECK(cart_ppu_read(0x0800) == 0x4C && cart_ppu_read(0x0C00) == 0x4D);
    CHECK(cart_ppu_read(0x1000) == 0x85 && cart_ppu_read(0x1400) == 0xA2);
    CHECK(cart_ppu_read(0x1800) == 0xC7 && cart_ppu_read(0x1C00) == 0xF8);

    Mirroring mirr = cart_get_mirroring();
    cart_cpu_write(0xA000, 1);
    cart_cpu_write(0xA001, 0);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    for (unsigned i = 0; i < 8; ++i) {
        cart_notify_ppu_address(0x0000, i * 12);
        cart_notify_ppu_address(0x1000, i * 12 + 9);
    }
    CHECK(cart_get_mirroring() == mirr && !cart_irq_pending());

    cart_cpu_write(0x6123, 0xA5);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 254 && cart_cpu_read(0xE000) == 255);
    CHECK(cart_ppu_read(0x0000) == 0 && cart_ppu_read(0x1000) == 4);
    CHECK(cart_cpu_read(0x6123) == 0xA5 && cart_get_mirroring() == mirr);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_namco108_submapper_loader_and_chr_ram(void) {
    iNESHeader h = header_for(206, 0x8000, false);
    h.flags7 |= 0x08;
    h.prg_ram_size = 0x10;
    h.flags6 |= 1;
    CHECK(fixture_with_header(&h, 0x8000, 0x40000) == 206);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 2 && cart_cpu_read(0xE000) == 3);
    namco108_write_bank(6, 3);
    namco108_write_bank(7, 2);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 2 && cart_cpu_read(0xE000) == 3);
    cart_cpu_write(0xA000, 0);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xE000) == 3);

    iNESHeader ram_h = header_for(206, 0x8000, true);
    ram_h.flags7 |= 0x08;
    ram_h.flags10 = 0;
    ram_h.zero[0] = 12;
    CHECK(fixture_with_header(&ram_h, 0x8000, 0x40000) == 206);
    namco108_write_bank(2, 0x85);
    cart_ppu_write(0x1000, 0xA6);
    CHECK(cart_ppu_read(0x1000) == 0xA6);
    namco108_write_bank(2, 5);
    CHECK(cart_ppu_read(0x1000) == 5);
    namco108_write_bank(2, 0x85);
    CHECK(cart_ppu_read(0x1000) == 0xA6);

    iNESHeader load_h = header_for(206, 0x200000, false);
    load_h.chr_rom_chunks = 32;
    size_t size;
    uint8_t *image = image_for(&load_h, 0x200000, 0x40000, &size);
    CHECK(image != NULL);
    for (size_t i = 0; i < 0x200000; ++i)
        image[sizeof(load_h) + i] = (uint8_t)(i / 0x2000);
    for (size_t i = 0; i < 0x40000; ++i)
        image[sizeof(load_h) + 0x200000 + i] = (uint8_t)(i / 0x400);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(rom_mapper_number(&ines_header) == 206);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xE000) == 255);
    namco108_write_bank(6, 0xC3);
    namco108_write_bank(2, 0x85);
    cart_cpu_write(0x6123, 0xA6);
    CHECK(cart_cpu_read(0x8000) == 0xC3 && cart_ppu_read(0x1000) == 0x85);

    Mapper *previous = cart;
    iNESHeader active = ines_header;
    iNESHeader invalid = load_h;
    invalid.flags7 |= 0x08;
    invalid.prg_ram_size = 0x20;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x200000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0xC3);

    invalid = h;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x10000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0xC3);
    CHECK(mapper_init_from_header(&load_h, fixture_prg, 0x202000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0xC3);
    CHECK(mapper_init_from_header(&load_h, fixture_prg, 0x200000, fixture_chr, 0x40400) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0xC3);

    image = image_for(&load_h, 0x200000, 0x40000, &size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, size - 1) == -1);
    free(image);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0xC3);
    CHECK(memcmp(&ines_header, &active, sizeof(active)) == 0);
    CHECK(cart_ppu_read(0x1000) == 0x85 && cart_cpu_read(0x6123) == 0xA6);
    return 0;
}

static void jaleco18_write_bank(uint16_t reg, uint8_t value) {
    uint16_t alias = (uint16_t)(reg | 0x0FFCu);
    cart_cpu_write(alias, value & 0x0F);
    cart_cpu_write((uint16_t)(alias | 1u), value >> 4);
}

static void jaleco18_write_irq_reload(uint16_t value) {
    for (unsigned nibble = 0; nibble < 4; ++nibble)
        cart_cpu_write((uint16_t)(0xE000u + nibble), (uint8_t)(value >> (nibble * 4)));
}

static int test_jaleco18_banks_ram_and_mirroring(void) {
    CHECK(fixture(18, 0x40000, 0x20000, false) == 18);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_cpu_read_bus(0xC000, 0x69) == 0x69 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_ppu_read(0x0012) == 0x12 && cart_ppu_read(0x1C34) == 0x34);

    jaleco18_write_bank(0x8000, 0x13);
    jaleco18_write_bank(0x8002, 0x0B);
    jaleco18_write_bank(0x9000, 0x1E);
    CHECK(cart_cpu_read(0x8000) == 0x13 && cart_cpu_read(0xA000) == 0x0B);
    CHECK(cart_cpu_read(0xC000) == 0x1E && cart_cpu_read(0xE000) == 31);

    static const uint16_t chr_regs[8] = {
        0xA000, 0xA002, 0xB000, 0xB002, 0xC000, 0xC002, 0xD000, 0xD002
    };
    for (unsigned slot = 0; slot < 8; ++slot) {
        uint8_t bank = (uint8_t)(0x20 + slot * 3);
        jaleco18_write_bank(chr_regs[slot], bank);
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == bank);
    }

    cart_cpu_write(0x6123, 0xA6);
    CHECK(cart_cpu_read(0x6123) == 0xA6);
    for (unsigned mode = 0; mode < 4; ++mode) {
        cart_cpu_write(0xFFFE, (uint8_t)mode);
        CHECK(cart_get_mirroring() == (Mirroring)mode);
    }

    cart->reset();
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_ppu_read(0x0012) == 0x12 && !cart_irq_pending());
    CHECK(cart_cpu_read(0x6123) == 0xA6 && cart_get_mirroring() == MIRROR_HORIZONTAL);

    iNESHeader h = header_for(18, 0x4000, false);
    h.flags7 = 0x18;
    h.flags10 = 0;
    CHECK(fixture_with_header(&h, 0x4000, 0x2000) == 18);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    cart_cpu_write(0x6000, 0xA5);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    h = header_for(18, 0x20000, true);
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 18);
    cart_ppu_write(0x0123, 0xA6);
    cart_ppu_write(0x0523, 0x69);
    CHECK(cart_ppu_read(0x0123) == 0xA6 && cart_ppu_read(0x0523) == 0x69);
    cart_cpu_write(0xA003, 0); // The unwritten CHR register contains zero, regardless of initial RAM wiring.
    CHECK(cart_ppu_read(0x0523) == 0xA6);
    cart_ppu_write(0x0523, 0x35);
    CHECK(cart_ppu_read(0x0123) == 0x35);
    return 0;
}

static int test_jaleco18_irq_and_cpu_clock(void) {
    CHECK(fixture(18, 0x4000, 0x2000, false) == 18);
    CHECK(cart != NULL && cart->clock != NULL);
    static const struct { uint16_t reload; uint8_t control; } modes[] = {
        {0x1002, 0x01}, {0x0102, 0x03}, {0x0012, 0x05}, {0x0002, 0x09}
    };
    for (unsigned i = 0; i < sizeof(modes) / sizeof(modes[0]); ++i) {
        jaleco18_write_irq_reload(modes[i].reload);
        cart_cpu_write(0xF000, 0);
        cart_cpu_write(0xF001, modes[i].control);
        uint16_t mask = i == 0 ? 0xFFFF : i == 1 ? 0x0FFF : i == 2 ? 0x00FF : 0x000F;
        unsigned count = modes[i].reload & mask;
        cart->clock((int)(count - 1));
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
        cart_cpu_write(0xFFF0, 0); // $F000 alias reloads and acknowledges.
        CHECK(!cart_irq_pending());
    }

    jaleco18_write_irq_reload(3);
    cart_cpu_write(0xF000, 0);
    cart_cpu_write(0xF001, 1);
    cart->clock(1);
    cart_cpu_write(0xF001, 0);
    CHECK(!cart_irq_pending());
    cart->clock(20);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xF001, 1);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xF001, 0);
    CHECK(!cart_irq_pending());

    fixture_prg[0] = 0xEA;
    jaleco18_write_bank(0x8000, 0);
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cpu.pc = 0x8000;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
    jaleco18_write_irq_reload(2);
    cart_cpu_write(0xF000, 0);
    cart_cpu_write(0xF001, 1);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
    fixture_prg[0x3FFE] = 0;
    fixture_prg[0x3FFF] = 3;
    for (unsigned i = 0; i < 8; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
    cpu.pc = 0x0200;
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    for (unsigned step = 0; step < 3 && cpu.pc != 0x0300; ++step) (void)cpu_step(&cpu);
    CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG) && cart_irq_pending());
    cart_cpu_write(0xF001, 0);
    cpu_soft_reset(&cpu);
    CHECK(cart_cpu_read(0x8000) == 0xEA);
    return 0;
}

static int test_jaleco18_irq_width_transitions(void) {
    CHECK(fixture(18, 0x20000, 0x2000, false) == 18);
    const unsigned period[] = {65536, 4096, 256, 16, 16, 256};
    const uint8_t control[] = {1, 3, 5, 9, 15, 7};
    for (unsigned mode = 0; mode < sizeof(control); ++mode) {
        jaleco18_write_irq_reload(0);
        cart_cpu_write(0xF000, 0);
        cart_cpu_write(0xF001, control[mode]);
        cart->clock((int)(period[mode] - 1));
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
    }
    jaleco18_write_irq_reload(0x1230);
    cart_cpu_write(0xF000, 0);
    cart_cpu_write(0xF001, 9);
    cart->clock(16);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xF001, 1);
    cart->clock(0x122F);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending()); // Narrow counters leave the upper bits intact.
    return 0;
}

static int test_jaleco18_loader_validation(void) {
    CHECK(fixture(18, 0x4000, 0x2000, false) == 18);
    cart_cpu_write(0x6000, 0xA7);
    jaleco18_write_bank(0x8000, 1);
    Mapper *previous = cart;
    CHECK(mapper_init_from_header(&(iNESHeader){.signature={'N','E','S',0x1A}, .prg_rom_chunks=1,
        .chr_rom_chunks=1, .flags6=0x20, .flags7=0x10}, fixture_prg, 0x200001,
        fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 1 && cart_cpu_read(0x6000) == 0xA7);

    iNESHeader h = header_for(18, 0x4000, false);
    h.flags7 = 0x18;
    h.flags10 = 8; // 16KB PRG-RAM exceeds the board's fixed $6000-$7FFF window.
    size_t size;
    uint8_t *image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == -1 && cart == previous);
    CHECK(cart_cpu_read(0x8000) == 1 && cart_cpu_read(0x6000) == 0xA7);
    return 0;
}

static int test_irem32_banks_variants_and_ram(void) {
    CHECK(fixture(32, 0x40000, 0x20000, false) == 32);
    CHECK(cart != NULL && cart->clock == NULL);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x1C12) == 0x12);

    cart_cpu_write(0x8ABC, 0x3F);
    CHECK(cart_cpu_read_bus(0xA000, 0x56) == 0x56);
    cart_cpu_write(0xA7FF, 0x22);
    CHECK(cart_cpu_read(0x8000) == 31 && cart_cpu_read(0xA000) == 2);
    cart_cpu_write(0x9ACE, 0x02);
    CHECK(cart_cpu_read(0x8000) == 30 && cart_cpu_read(0xC000) == 31);
    CHECK(cart_cpu_read(0xA000) == 2 && cart_cpu_read(0xE000) == 31);
    cart_cpu_write(0x9FFF, 0x01);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL && cart_cpu_read(0x8000) == 31);
    CHECK(cart_cpu_read(0xC000) == 30);
    cart_cpu_write(0x9000, 0);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);

    for (unsigned slot = 0; slot < 8; ++slot) {
        uint8_t bank = (uint8_t)(0x30 + slot);
        cart_cpu_write((uint16_t)(0xBFF8u + slot), bank);
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == bank);
    }
    cart_cpu_write(0x6000, 0xA6);
    CHECK(cart_cpu_read(0x6000) == 0xA6 && !cart_irq_pending());

    cart->reset();
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL && cart_cpu_read(0x6000) == 0xA6);
    cart_cpu_write(0x9000, 2);
    CHECK(cart_cpu_read(0x8000) == 30 && cart_cpu_read(0xA000) == 0);
    CHECK(cart_cpu_read(0xC000) == 0);

    iNESHeader h = header_for(32, 0x40000, false);
    h.flags7 |= 8;
    h.prg_ram_size = 0x10; // Submapper 1: fixed 8+8+16F PRG layout.
    h.flags10 = 7;
    CHECK(fixture_with_header(&h, 0x40000, 0x20000) == 32);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0x8000, 5);
    cart_cpu_write(0xA000, 7);
    cart_cpu_write(0x9000, 2);
    CHECK(cart_cpu_read(0x8000) == 5 && cart_cpu_read(0xA000) == 7);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);

    Mapper *previous = cart;
    uint8_t previous_bank = cart_cpu_read(0x8000);
    h.prg_ram_size = 0x20;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x40000, fixture_chr, 0x20000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == previous_bank);
    return 0;
}

static int test_irem65_banks_decode_and_ram(void) {
    CHECK(fixture(65, 0x40000, 0x20000, false) == 65);
    CHECK(cart != NULL && cart->clock != NULL);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x1C12) == 0x12);

    cart_cpu_write(0x8000, 9);
    cart_cpu_write(0xA000, 11);
    cart_cpu_write(0xC000, 13);
    CHECK(cart_cpu_read(0x8000) == 9 && cart_cpu_read(0xA000) == 11);
    CHECK(cart_cpu_read(0xC000) == 13 && cart_cpu_read(0xE000) == 31);
    cart_cpu_write(0x8FFF, 4);
    cart_cpu_write(0xA001, 5);
    cart_cpu_write(0xC001, 6);
    CHECK(cart_cpu_read(0x8000) == 9 && cart_cpu_read(0xA000) == 11 && cart_cpu_read(0xC000) == 13);

    cart_cpu_write(0x9001, 0x80);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    cart_cpu_write(0x9001, 0);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0x9000, 0x80);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);

    for (unsigned slot = 0; slot < 8; ++slot) {
        uint8_t bank = (uint8_t)(0x40 + slot);
        cart_cpu_write((uint16_t)(0xB000u + slot), bank);
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == bank);
    }
    cart_cpu_write(0xB008, 0x22);
    CHECK(cart_ppu_read(0) == 0x40);
    cart_cpu_write(0x6123, 0x5A);
    CHECK(cart_cpu_read(0x6123) == 0x5A);
    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_cpu_read(0x6123) == 0x5A && !cart_irq_pending());
    CHECK(fixture(65, 0x6000, 0x2000, false) == 65);
    CHECK(cart_cpu_read(0xC000) == 2 && cart_cpu_read(0xE000) == 2);
    return 0;
}

static int test_irem65_irq_and_loader_validation(void) {
    CHECK(fixture(65, 0x4000, 0x2000, false) == 65);
    cart_cpu_write(0x9005, 0x01);
    cart_cpu_write(0x9006, 0x02);
    cart_cpu_write(0x9004, 0);
    cart_cpu_write(0x9003, 0x80);
    cart->clock(0x101);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart->clock(8);
    CHECK(cart_irq_pending()); // Expiration disables the one-shot counter.
    cart_cpu_write(0x9003, 0);
    CHECK(!cart_irq_pending());

    cart_cpu_write(0x9005, 0);
    cart_cpu_write(0x9006, 3);
    cart_cpu_write(0x9004, 0);
    cart_cpu_write(0x9003, 0x80);
    cart->clock(1);
    cart_cpu_write(0x9004, 0);
    CHECK(!cart_irq_pending());
    cart->clock(2);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());

    cart_cpu_write(0x9003, 0);
    fixture_prg[0] = 0xEA;
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cpu.pc = 0x8000;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
    cart_cpu_write(0x9005, 0);
    cart_cpu_write(0x9006, 2);
    cart_cpu_write(0x9004, 0);
    cart_cpu_write(0x9003, 0x80);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());

    fixture_prg[0x3FFE] = 0;
    fixture_prg[0x3FFF] = 3;
    for (unsigned i = 0; i < 8; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
    cpu.pc = 0x0200;
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    for (unsigned step = 0; step < 3 && cpu.pc != 0x0300; ++step) (void)cpu_step(&cpu);
    CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG) && cart_irq_pending());
    write_mem(0x9007, 0);
    CHECK(cart_irq_pending());

    cart_cpu_write(0x9003, 0);
    cart_cpu_write(0x6000, 0xA7);
    Mapper *previous = cart;
    iNESHeader h = header_for(65, 0x4000, false);
    h.flags7 |= 8;
    h.flags10 = 8; // The board exposes one unbanked 8KB PRG-RAM window.
    size_t size;
    uint8_t *image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == -1 && cart == previous && cart_cpu_read(0x6000) == 0xA7);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x200001, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x6000) == 0xA7);
    return 0;
}

static int test_irem_ram_and_irq_boundaries(void) {
    const unsigned mappers[] = {32, 65};
    for (unsigned i = 0; i < 2; ++i) {
        CHECK(fixture(mappers[i], 0x20000, 0x2000, true) == (int)mappers[i]);
        cart_ppu_write(0x0123, 0xA6);
        cart_ppu_write(0x0523, 0x69);
        CHECK(cart_ppu_read(0x0123) == 0xA6 && cart_ppu_read(0x0523) == 0x69);
        cart_cpu_write(0xB001, 0);
        CHECK(cart_ppu_read(0x0523) == 0xA6);
        cart_ppu_write(0x0523, 0x35);
        CHECK(cart_ppu_read(0x0123) == 0x35);
    }
    cart_cpu_write(0x9003, 0x80);
    cart->clock(65535);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0x9004, 0);
    cart->clock(65536);
    CHECK(!cart_irq_pending()); // Reload acknowledges but does not re-enable an expired counter.
    cart_cpu_write(0x9003, 0x80);
    cart->clock(65536);
    CHECK(cart_irq_pending());
    cart_cpu_write(0x9006, 3);
    cart_cpu_write(0x9004, 0);
    cart_cpu_write(0x9003, 0x80);
    cart->clock(1);
    cart_cpu_write(0x9003, 0);
    cart->clock(100);
    cart_cpu_write(0x9003, 0x80);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0x9004, 0);
    cart_cpu_write(0x9003, 0x80);
    cart->clock(1);
    cart_cpu_write(0x9005, 0xFF);
    cart_cpu_write(0x9006, 0xFF);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending()); // Reload-latch writes leave the running counter unchanged.
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

typedef struct {
    unsigned mapper;
    uint8_t submapper;
    uint16_t reg1_offset;
    uint16_t reg2_offset;
    uint16_t reg3_offset;
    bool has_irq;
    bool vrc2;
    bool vrc2a;
} Vrc24Case;

static const Vrc24Case vrc24_cases[] = {
    {21, 0, 0x0002, 0x0004, 0x0006, true,  false, false},
    {21, 1, 0x0002, 0x0004, 0x0006, true,  false, false},
    {21, 2, 0x0040, 0x0080, 0x00C0, true,  false, false},
    {22, 0, 0x0002, 0x0001, 0x0003, false, true,  true },
    {23, 0, 0x0001, 0x0002, 0x0003, true,  false, false},
    {23, 1, 0x0001, 0x0002, 0x0003, true,  false, false},
    {23, 2, 0x0004, 0x0008, 0x000C, true,  false, false},
    {23, 3, 0x0001, 0x0002, 0x0003, false, true,  false},
    {25, 0, 0x0002, 0x0001, 0x0003, true,  false, false},
    {25, 1, 0x0002, 0x0001, 0x0003, true,  false, false},
    {25, 2, 0x0008, 0x0004, 0x000C, true,  false, false},
    {25, 3, 0x0002, 0x0001, 0x0003, false, true,  false},
    {27, 0, 0x0001, 0x0002, 0x0003, true,  false, false},
    {183,0, 0x0004, 0x0008, 0x000C, true,  false, false},
};

static iNESHeader vrc24_header(unsigned mapper, uint8_t submapper, bool ram) {
    iNESHeader h = header_for(mapper, 0x40000, false);
    h.flags7 |= 0x08;
    h.prg_ram_size = (uint8_t)(submapper << 4);
    h.flags10 = ram ? 7 : 0;
    return h;
}

static int test_vrc24_variant_register_wiring(void) {
    for (size_t i = 0; i < sizeof(vrc24_cases) / sizeof(vrc24_cases[0]); ++i) {
        const Vrc24Case *tc = &vrc24_cases[i];
        iNESHeader h = vrc24_header(tc->mapper, tc->submapper, true);
        CHECK(fixture_with_header(&h, 0x40000, 0x40000) == (int)tc->mapper);
        CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 0);
        CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);

        cart_cpu_write(0x8000, 3);
        cart_cpu_write(0xA000, 5);
        CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xA000) == 5);
        CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);

        cart_cpu_write(0x9000, tc->vrc2 ? 1 : 2);
        CHECK(cart_get_mirroring() == (tc->vrc2 ? MIRROR_HORIZONTAL : MIRROR_SINGLE0));
        cart_cpu_write(0x9000, 0);
        CHECK(cart_get_mirroring() == MIRROR_VERTICAL);

        cart_cpu_write(0xB000, 2);
        cart_cpu_write((uint16_t)(0xB000u + tc->reg1_offset), 1);
        uint8_t expected_chr = tc->vrc2a ? 9 : 0x12;
        CHECK(cart_ppu_read(0x0000) == expected_chr);

        if (!tc->vrc2 && !(tc->mapper == 23 && tc->submapper == 0)) {
            cart_cpu_write((uint16_t)(0x9000u + tc->reg2_offset), 2);
            CHECK(cart_cpu_read(0x8000) == 30 && cart_cpu_read(0xA000) == 5);
            CHECK(cart_cpu_read(0xC000) == 3 && cart_cpu_read(0xE000) == 31);
        }
        CHECK((cart->clock != NULL) == tc->has_irq);
    }

    // Legacy submapper-zero boards accept both known address-line aliases.
    CHECK(fixture(21, 0x40000, 0x40000, false) == 21);
    cart_cpu_write(0xB000, 2);
    cart_cpu_write(0xB002, 1);
    CHECK(cart_ppu_read(0) == 0x12);
    cart_cpu_write(0xB040, 2);
    CHECK(cart_ppu_read(0) == 0x22);

    CHECK(fixture(23, 0x40000, 0x40000, false) == 23);
    cart_cpu_write(0xB000, 2);
    cart_cpu_write(0xB001, 1);
    CHECK(cart_ppu_read(0) == 0x12);
    cart_cpu_write(0xB004, 2);
    CHECK(cart_ppu_read(0) == 0x22);

    CHECK(fixture(25, 0x40000, 0x40000, false) == 25);
    cart_cpu_write(0xB000, 2);
    cart_cpu_write(0xB002, 1);
    CHECK(cart_ppu_read(0) == 0x12);
    cart_cpu_write(0xB008, 2);
    CHECK(cart_ppu_read(0) == 0x22);
    return 0;
}

static int test_vrc24_ram_latch_and_mapper183_window(void) {
    iNESHeader h = vrc24_header(22, 0, false);
    CHECK(fixture_with_header(&h, 0x40000, 0x40000) == 22);
    CHECK(cart_cpu_read_bus(0x6000, 0xA6) == 0xA6);
    cart_cpu_write(0x6000, 1);
    CHECK(cart_cpu_read_bus(0x6000, 0xA6) == 0xA7);
    cart_cpu_write(0x6FFF, 0);
    CHECK(cart_cpu_read_bus(0x6123, 0x5B) == 0x5A);
    CHECK(cart_cpu_read_bus(0x7000, 0x5B) == 0x5B);

    h = vrc24_header(23, 3, true);
    CHECK(fixture_with_header(&h, 0x40000, 0x40000) == 23);
    cart_cpu_write(0x6123, 0xA5);
    CHECK(cart_cpu_read(0x6123) == 0xA5);

    h = vrc24_header(183, 0, false);
    CHECK(fixture_with_header(&h, 0x40000, 0x40000) == 183);
    CHECK(cart_cpu_read(0x6000) == 0);
    cart_cpu_write(0x6005, 0xFF);
    CHECK(cart_cpu_read(0x6000) == 5 && cart_cpu_read(0x7FFF) == 5);
    cart_cpu_write(0x7FFE, 0);
    CHECK(cart_cpu_read(0x6000) == 14);
    return 0;
}

static int test_vrc24_irq_variants_and_phase(void) {
    for (size_t i = 0; i < sizeof(vrc24_cases) / sizeof(vrc24_cases[0]); ++i) {
        const Vrc24Case *tc = &vrc24_cases[i];
        iNESHeader h = vrc24_header(tc->mapper, tc->submapper, true);
        CHECK(fixture_with_header(&h, 0x40000, 0x40000) == (int)tc->mapper);
        if (!tc->has_irq) {
            CHECK(cart->clock == NULL);
            cart_cpu_write(0xF000, 0x0E);
            cart_cpu_write((uint16_t)(0xF000u + tc->reg1_offset), 0x0F);
            CHECK(!cart_irq_pending());
            continue;
        }

        cart_cpu_write(0xF000, 0x0E);
        cart_cpu_write((uint16_t)(0xF000u + tc->reg1_offset), 0x0F);
        cart_cpu_write((uint16_t)(0xF000u + tc->reg2_offset), 0x07);
        cart->clock(1);
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
        cart_cpu_write((uint16_t)(0xF000u + tc->reg3_offset), 0);
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
    }

    iNESHeader h = vrc24_header(21, 1, true);
    CHECK(fixture_with_header(&h, 0x40000, 0x40000) == 21);
    cart_cpu_write(0xF000, 0x0F);
    cart_cpu_write(0xF002, 0x0F);
    cart_cpu_write(0xF004, 0x02);
    cart->clock(113);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xF006, 0);
    CHECK(!cart_irq_pending());
    cart->clock(228);
    CHECK(!cart_irq_pending());

    fixture_prg[0] = 0xEA;
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cart_cpu_write(0x8000, 0);
    cpu.pc = 0x8000;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
    cart_cpu_write(0xF000, 0x0E);
    cart_cpu_write(0xF002, 0x0F);
    cart_cpu_write(0xF004, 0x06);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
    return 0;
}

static int test_vrc24_loader_rejection_preserves_cart(void) {
    iNESHeader active = vrc24_header(21, 1, true);
    CHECK(fixture_with_header(&active, 0x40000, 0x40000) == 21);
    cart_cpu_write(0x8000, 3);
    cart_cpu_write(0x6123, 0xA7);
    Mapper *previous = cart;

    iNESHeader invalid = vrc24_header(21, 3, true);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3 && cart_cpu_read(0x6123) == 0xA7);

    invalid = vrc24_header(22, 0, true);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x42000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3 && cart_cpu_read(0x6123) == 0xA7);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x80000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3 && cart_cpu_read(0x6123) == 0xA7);

    invalid = vrc24_header(23, 3, false);
    invalid.flags10 = 8; // 16KB RAM exceeds the single $6000-$7FFF chip window.
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3 && cart_cpu_read(0x6123) == 0xA7);
    return 0;
}

static iNESHeader vrc7_header(uint8_t submapper, bool chr_ram, bool ram) {
    iNESHeader h = header_for(85, 0x80000, chr_ram);
    h.flags7 |= 0x08;
    h.prg_ram_size = (uint8_t)(submapper << 4);
    h.flags10 = ram ? 7 : 0;
    if (chr_ram) h.zero[0] = 7;
    else h.chr_rom_chunks = 0x20;
    return h;
}

static void vrc7_audio_write(uint8_t reg, uint8_t value) {
    cart_cpu_write(0x9010, reg);
    cart_cpu_write(0x9030, value);
}

static int test_vrc7_banks_wiring_and_ram(void) {
    for (unsigned submapper = 0; submapper <= 2; ++submapper) {
        iNESHeader h = vrc7_header((uint8_t)submapper, false, true);
        CHECK(fixture_with_header(&h, 0x80000, 0x40000) == 85);
        CHECK(cart != NULL && cart->clock != NULL);
        CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
        CHECK(cart_cpu_read_bus(0xA000, 0xA6) == 0xA6);
        CHECK(cart_cpu_read_bus(0xC000, 0x3C) == 0x3C);
        CHECK(cart_cpu_read(0xE000) == 63);
        CHECK(cart_ppu_read(0x0400) == 0); // CHR-ROM pages start unmapped.

        cart_cpu_write(0x8000, 3);
        cart_cpu_write(0x9000, 9);
        CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xC000) == 9);
        if (submapper == 0) {
            cart_cpu_write(0x8008, 4);
            CHECK(cart_cpu_read(0xA000) == 4);
            cart_cpu_write(0x8010, 6);
            CHECK(cart_cpu_read(0xA000) == 6);
            cart_cpu_write(0xA008, 5);
            CHECK(cart_ppu_read(0x0400) == 5);
            cart_cpu_write(0xA010, 7);
            CHECK(cart_ppu_read(0x0400) == 7);
        } else if (submapper == 1) {
            cart_cpu_write(0x8008, 5);
            CHECK(cart_cpu_read(0xA000) == 5);
            cart_cpu_write(0x8010, 7); // A4 is not connected on VRC7b.
            CHECK(cart_cpu_read(0x8000) == 7 && cart_cpu_read(0xA000) == 5);
            cart_cpu_write(0xA008, 6);
            CHECK(cart_ppu_read(0x0400) == 6);
            cart_cpu_write(0xA010, 8);
            CHECK(cart_ppu_read(0x0000) == 8 && cart_ppu_read(0x0400) == 6);
        } else {
            cart_cpu_write(0x8008, 5); // A3 is not connected on VRC7a.
            CHECK(cart_cpu_read(0x8000) == 5);
            cart_cpu_write(0x8010, 7);
            CHECK(cart_cpu_read(0xA000) == 7);
            cart_cpu_write(0xA008, 6);
            CHECK(cart_ppu_read(0x0000) == 6);
            cart_cpu_write(0xA010, 8);
            CHECK(cart_ppu_read(0x0400) == 8);
        }

        CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
        cart_cpu_write(0x6000, 0xA5);
        CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
        cart_cpu_write(0xE000, 0x82);
        CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
        cart_cpu_write(0x6123, 0xA5);
        CHECK(cart_cpu_read(0x6123) == 0xA5);
        cart_cpu_write(0xE000, 0x81);
        CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL && cart_cpu_read(0x6123) == 0xA5);
        cart_cpu_write(0xE000, 0x00);
        CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
        CHECK(cart_cpu_read_bus(0x6123, 0x56) == 0x56);
    }

    // CHR-RAM is linearly available before bank registers are written.
    iNESHeader ram_header = vrc7_header(2, true, true);
    CHECK(fixture_with_header(&ram_header, 0x80000, 0x2000) == 85);
    cart_ppu_write(0x0400, 0xA7);
    CHECK(cart_ppu_read(0x0400) == 0xA7);
    cart_cpu_write(0xA010, 3);
    cart_ppu_write(0x0400, 0x53);
    CHECK(cart_ppu_read(0x0C00) == 0x53);
    ram_header.zero[0] = 6; // A declared 4KB RAM aliases across the 8KB pattern window.
    CHECK(fixture_with_header(&ram_header, 0x80000, 0x1000) == 85);
    cart_ppu_write(0x1C23, 0xB6);
    CHECK(cart_ppu_read(0x0C23) == 0xB6);
    cart_cpu_write(0xA010, 7);
    CHECK(cart_ppu_read(0x0423) == 0xB6);
    return 0;
}

static int test_vrc7_irq_timing(void) {
    for (unsigned submapper = 0; submapper <= 2; ++submapper) {
        iNESHeader h = vrc7_header((uint8_t)submapper, false, true);
        CHECK(fixture_with_header(&h, 0x80000, 0x40000) == 85);
        uint16_t secondary = submapper == 2 ? 0x10u : 0x08u;
        cart_cpu_write((uint16_t)(0xE000u + secondary), 0xFE);
        cart_cpu_write(0xF000, 0x07);
        cart->clock(1);
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
        cart_cpu_write((uint16_t)(0xF000u + secondary), 0);
        CHECK(!cart_irq_pending());
        cart->clock(2);
        CHECK(cart_irq_pending());
    }

    iNESHeader h = vrc7_header(1, false, true);
    CHECK(fixture_with_header(&h, 0x80000, 0x40000) == 85);
    cart_cpu_write(0xE008, 0xFF);
    cart_cpu_write(0xF000, 0x02);
    cart->clock(113);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xF008, 0);
    CHECK(!cart_irq_pending());

    fixture_prg[0] = 0xEA;
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cart_cpu_write(0x8000, 0);
    cpu.pc = 0x8000;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
    cart_cpu_write(0xE008, 0xFE);
    cart_cpu_write(0xF000, 0x06);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
    return 0;
}

static void vrc7_program_custom_patch(void) {
    static const uint8_t patch[8] = {0x01,0x01,0x10,0x00,0xF5,0xF5,0x4F,0x4F};
    for (unsigned reg = 0; reg < 8; ++reg) vrc7_audio_write((uint8_t)reg, patch[reg]);
}

static void vrc7_key_channel0(uint8_t instrument, bool key_on) {
    vrc7_audio_write(0x30, (uint8_t)(instrument << 4));
    vrc7_audio_write(0x10, 0x80);
    vrc7_audio_write(0x20, (uint8_t)(0x08 | (key_on ? 0x10 : 0)));
}

static int test_vrc7_fm_audio(void) {
    iNESHeader h = vrc7_header(2, false, true);
    CHECK(fixture_with_header(&h, 0x80000, 0x40000) == 85);
    nes_set_region(NES_REGION_NTSC);
    CHECK(cart_expansion_audio() == 0.0f);

    vrc7_program_custom_patch();
    vrc7_key_channel0(0, true);
    cart->clock(35);
    CHECK(cart_expansion_audio() == 0.0f);
    cart->clock(1);
    CHECK(cart_expansion_audio() == 0.0f); // First chip tick starts the envelope at zero output.

    static const int16_t expected_trace[16] = {
        -693, -453, -787, -1120, -1394, -1645, -1828, -1956,
        -2031, -2037, -1988, -1873, -1708, -1492, -1231, -934
    };
    float custom_trace[16];
    float custom_energy = 0.0f;
    for (unsigned i = 0; i < 16; ++i) {
        cart->clock(36);
        custom_trace[i] = cart_expansion_audio();
        CHECK(custom_trace[i] == (float)expected_trace[i] * (1.0f / 5000.0f));
        custom_energy += custom_trace[i] < 0.0f ? -custom_trace[i] : custom_trace[i];
    }
    CHECK(custom_energy > 0.001f);

    cart->reset();
    nes_set_region(NES_REGION_PAL);
    vrc7_program_custom_patch();
    vrc7_key_channel0(0, true);
    cart->clock(34);
    for (unsigned i = 0; i < 16; ++i) {
        float previous = cart_expansion_audio();
        cart->clock(33);
        CHECK(cart_expansion_audio() == previous);
        cart->clock(1);
        CHECK(cart_expansion_audio() == custom_trace[i]);
    }

    cart->reset();
    nes_set_region(NES_REGION_NTSC);
    vrc7_program_custom_patch();
    vrc7_key_channel0(0, true);
    cart->clock(36);
    for (unsigned i = 0; i < 16; ++i) {
        cart->clock(36);
        CHECK(cart_expansion_audio() == custom_trace[i]);
    }

    cart->reset();
    nes_set_region(NES_REGION_NTSC);
    vrc7_key_channel0(1, true);
    float preset_energy = 0.0f;
    bool differs = false;
    cart->clock(36 * 32);
    for (unsigned i = 0; i < 16; ++i) {
        cart->clock(36);
        float sample = cart_expansion_audio();
        preset_energy += sample < 0.0f ? -sample : sample;
        if (sample != custom_trace[i]) differs = true;
    }
    CHECK(preset_energy > 0.0f && differs);

    cart->reset();
    nes_set_region(NES_REGION_NTSC);
    vrc7_program_custom_patch();
    vrc7_key_channel0(0, true);
    cart->clock(36 * 32);
    CHECK(cart_expansion_audio() != 0.0f);
    vrc7_audio_write(0x20, 0x08); // Key off enters release on both operators.
    cart->clock(36 * 16384);
    CHECK(cart_expansion_audio() == 0.0f);

    vrc7_key_channel0(0, true);
    cart->clock(36 * 8);
    CHECK(cart_expansion_audio() != 0.0f);
    cart_cpu_write(0xE000, 0x40);
    CHECK(cart_expansion_audio() == 0.0f);
    vrc7_audio_write(0x20, 0x08); // Ignored while audio is muted.
    cart->clock(36 * 8);
    CHECK(cart_expansion_audio() == 0.0f);
    cart_cpu_write(0xE000, 0x00);
    cart->clock(36);
    CHECK(cart_expansion_audio() != 0.0f);
    return 0;
}

static int test_vrc7_register_boundaries(void) {
    iNESHeader h = vrc7_header(2, false, true);
    CHECK(fixture_with_header(&h, 0x80000, 0x40000) == 85);
    nes_set_region(NES_REGION_NTSC);
    vrc7_program_custom_patch();
    vrc7_key_channel0(0, true);
    float trace[128];
    for (unsigned i = 0; i < 128; ++i) {
        cart->clock(36);
        trace[i] = cart_expansion_audio();
    }
    cart->reset();
    vrc7_program_custom_patch();
    vrc7_key_channel0(0, true);
    for (unsigned i = 0; i < 128; ++i) {
        for (unsigned reg = 0x40; reg <= 0xFF; ++reg)
            vrc7_audio_write((uint8_t)reg, (uint8_t)(i ^ reg));
        cart->clock(36);
        CHECK(cart_expansion_audio() == trace[i]);
    }

    // High aliases inside the 64-register window still reach channel zero.
    cart->reset();
    vrc7_program_custom_patch();
    vrc7_audio_write(0x39, 0);
    vrc7_audio_write(0x19, 0x80);
    vrc7_audio_write(0x29, 0x18);
    for (unsigned i = 0; i < 128; ++i) {
        cart->clock(36);
        CHECK(cart_expansion_audio() == trace[i]);
    }
    return 0;
}

static int test_vrc7_loader_rejection_preserves_cart(void) {
    iNESHeader active = vrc7_header(1, false, true);
    CHECK(fixture_with_header(&active, 0x80000, 0x40000) == 85);
    cart_cpu_write(0x8000, 3);
    cart_cpu_write(0xE000, 0x80);
    cart_cpu_write(0x6123, 0xA7);
    Mapper *previous = cart;

    iNESHeader invalid = vrc7_header(3, false, true);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x80000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3 && cart_cpu_read(0x6123) == 0xA7);

    invalid = vrc7_header(1, false, true);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x82000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3 && cart_cpu_read(0x6123) == 0xA7);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x80000, fixture_chr, 0x42000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3 && cart_cpu_read(0x6123) == 0xA7);

    invalid.flags10 = 8;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x80000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3 && cart_cpu_read(0x6123) == 0xA7);
    return 0;
}

static int test_namco108_variants(void) {
    CHECK(fixture(76, 0x200000, 0x80000, false) == 76);
    CHECK(cart != NULL && cart->clock == NULL);
    CHECK(cart_ppu_read(0x0000) == 8 && cart_ppu_read(0x0800) == 10);
    CHECK(cart_ppu_read(0x1000) == 12 && cart_ppu_read(0x1800) == 14);
    namco108_write_bank(2, 3);
    namco108_write_bank(3, 7);
    namco108_write_bank(4, 11);
    namco108_write_bank(5, 15);
    CHECK(cart_ppu_read(0x0000) == 6 && cart_ppu_read(0x0400) == 7);
    CHECK(cart_ppu_read(0x0800) == 14 && cart_ppu_read(0x0C00) == 15);
    CHECK(cart_ppu_read(0x1000) == 22 && cart_ppu_read(0x1800) == 30);
    namco108_write_bank(6, 0x33);
    namco108_write_bank(7, 0x55);
    CHECK(cart_cpu_read(0x8000) == 0x33 && cart_cpu_read(0xA000) == 0x55);
    CHECK(cart_cpu_read(0xC000) == 254 && cart_cpu_read(0xE000) == 255);

    CHECK(fixture(88, 0x200000, 0x20000, false) == 88);
    CHECK(cart_ppu_read(0x1000) == 0x44 && cart_ppu_read(0x1C00) == 0x47);
    namco108_write_bank(0, 0x7F);
    namco108_write_bank(1, 0x43);
    namco108_write_bank(2, 0x05);
    namco108_write_bank(3, 0x3A);
    CHECK(cart_ppu_read(0x0000) == 0x3E && cart_ppu_read(0x0400) == 0x3F);
    CHECK(cart_ppu_read(0x0800) == 0x02 && cart_ppu_read(0x0C00) == 0x03);
    CHECK(cart_ppu_read(0x1000) == 0x45 && cart_ppu_read(0x1400) == 0x7A);
    namco108_write_bank(4, 0x09);
    namco108_write_bank(5, 0xBF);
    CHECK(cart_ppu_read(0x1800) == 0x49 && cart_ppu_read(0x1C00) == 0x7F);
    cart->reset();
    CHECK(cart_ppu_read(0x1000) == 0x44 && cart_ppu_read(0x1C00) == 0x47);

    CHECK(fixture(95, 0x200000, 0x40000, false) == 95);
    uint8_t nt[0x1000] = {0};
    nt[0] = 0x10;
    nt[0x400] = 0x20;
    CHECK(cart_nt_read(0x2000, nt) == 0x10 && cart_nt_read(0x2800, nt) == 0x20);
    cart_cpu_write(0xA001, 0); // An odd register write also connects the nametable selectors.
    CHECK(cart_nt_read(0x2000, nt) == 0x10 && cart_nt_read(0x2800, nt) == 0x10);
    cart->reset();
    CHECK(cart_nt_read(0x2800, nt) == 0x20);
    namco108_write_bank(0, 0x20);
    namco108_write_bank(1, 0x02);
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x2400, nt) == 0x20);
    CHECK(cart_nt_read(0x2800, nt) == 0x10 && cart_nt_read(0x2C00, nt) == 0x10);
    namco108_write_bank(1, 0x22);
    CHECK(cart_nt_read(0x2800, nt) == 0x20 && cart_nt_read(0x2C00, nt) == 0x20);
    cart_nt_write(0x2001, 0x35, nt);
    CHECK(nt[0x401] == 0x35 && cart_nt_read(0x2401, nt) == 0x35);

    CHECK(fixture(154, 0x200000, 0x20000, false) == 154);
    CHECK(cart_ppu_read(0x1000) == 0x44 && cart_ppu_read(0x1C00) == 0x47);
    cart_cpu_write(0x8000, 0x40);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    cart_cpu_write(0x8001, 0x45);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1 && cart_ppu_read(0) == 4);
    cart_cpu_write(0xA000, 0);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0 && cart_ppu_read(0) == 4);
    cart_notify_ppu_address(0, 0);
    cart_notify_ppu_address(0x1000, 12);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_namco108_variant_image_loading(void) {
    const unsigned boards[] = {76, 88, 95, 154};
    for (unsigned i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        iNESHeader h = header_for(boards[i], 0x20000, false);
        h.flags7 |= 8;
        h.chr_rom_chunks = 16;
        size_t image_size;
        uint8_t *image = image_for(&h, 0x20000, 0x20000, &image_size);
        CHECK(image != NULL);
        for (unsigned bank = 0; bank < 16; ++bank)
            memset(image + sizeof(h) + bank * 0x2000, (int)bank, 0x2000);
        CHECK(load_rom_memory(image, image_size) == 0);
        free(image);
        CHECK(rom_mapper_number(&ines_header) == (int)boards[i]);
        CHECK(cart != NULL && cart->clock == NULL);
        for (unsigned reg = 6; reg <= 7; ++reg) {
            write_mem(0x9FFE, (uint8_t)(0xC0 | reg));
            write_mem(0x9FFF, (uint8_t)(3 + reg));
            CHECK(read_mem((uint16_t)(0x8000 + (reg - 6) * 0x2000)) == 3 + reg);
        }
        CHECK(read_mem(0xC000) == 14 && read_mem(0xE000) == 15);
        write_mem(0xC000, 0);
        write_mem(0xC001, 0);
        write_mem(0xE001, 0);
        cart_notify_ppu_address(0, 0);
        cart_notify_ppu_address(0x1000, 12);
        CHECK(!cart_irq_pending());
        CHECK(unload_rom());
    }
    return 0;
}

static int test_namco108_variant_loader_rejection(void) {
    CHECK(fixture(76, 0x200000, 0x80000, false) == 76);
    namco108_write_bank(6, 3);
    Mapper *previous = cart;
    iNESHeader h = header_for(76, 0x200000, false);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x200000, fixture_chr, 0x80400) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);
    h = header_for(88, 0x200000, false);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x200000, fixture_chr, 0x20400) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);
    h = header_for(95, 0x200000, false);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x202000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);
    h = header_for(154, 0x200000, false);
    h.flags7 |= 8;
    h.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x200000, fixture_chr, 0x20000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);
    return 0;
}

static int test_vrc1_banks_mirroring_and_reset(void) {
    const unsigned mappers[] = {75, 151};
    for (size_t i = 0; i < sizeof(mappers) / sizeof(mappers[0]); ++i) {
        CHECK(fixture(mappers[i], 0x20000, 0x20000, false) == (int)mappers[i]);
        CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
        CHECK(cart_cpu_read_bus(0xA000, 0x69) == 0x69);
        CHECK(cart_cpu_read_bus(0xC000, 0xA6) == 0xA6);
        CHECK(cart_cpu_read(0xE000) == 15);
        CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x1456) == 0x56);

        cart_cpu_write(0x8123, 3);
        cart_cpu_write(0xAFFF, 5);
        cart_cpu_write(0xC456, 7);
        CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xA000) == 5);
        CHECK(cart_cpu_read(0xC000) == 7 && cart_cpu_read(0xE000) == 15);

        cart_cpu_write(0xE000, 3);
        CHECK(cart_ppu_read(0) == 12 && cart_ppu_read(0x1000) == 0);
        cart_cpu_write(0xF000, 4);
        CHECK(cart_ppu_read(0) == 12 && cart_ppu_read(0x1000) == 16);
        cart_cpu_write(0x9000, 0x06);
        CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
        CHECK(cart_ppu_read(0) == 76 && cart_ppu_read(0x1000) == 80);
        cart_cpu_write(0xEABC, 9);
        CHECK(cart_ppu_read(0) == 100 && cart_ppu_read(0x1000) == 80);
        cart_cpu_write(0xF123, 2);
        CHECK(cart_ppu_read(0) == 100 && cart_ppu_read(0x1000) == 72);
        cart_cpu_write(0x9000, 1);
        CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
        CHECK(cart_ppu_read(0) == 36 && cart_ppu_read(0x1000) == 8);

        cart->reset();
        CHECK(cart_cpu_read_bus(0x8000, 0x35) == 0x35);
        CHECK(cart_cpu_read_bus(0xA000, 0x53) == 0x53);
        CHECK(cart_cpu_read_bus(0xC000, 0x96) == 0x96);
        CHECK(cart_cpu_read(0xE000) == 15);
        CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x1456) == 0x56);
        CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    }

    iNESHeader four = header_for(75, 0x20000, false);
    four.flags6 |= 0x08;
    CHECK(fixture_with_header(&four, 0x20000, 0x20000) == 75);
    CHECK(cart_get_mirroring() == MIRROR_FOUR);
    cart_cpu_write(0x9000, 1);
    CHECK(cart_get_mirroring() == MIRROR_FOUR);

    iNESHeader ram = header_for(75, 0x20000, true);
    CHECK(fixture_with_header(&ram, 0x20000, 0x2000) == 75);
    cart_ppu_write(0x0123, 0xA6);
    cart_ppu_write(0x1123, 0x69);
    CHECK(cart_ppu_read(0x0123) == 0xA6 && cart_ppu_read(0x1123) == 0x69);
    cart_cpu_write(0xE000, 1);
    cart_ppu_write(0x0123, 0x35);
    CHECK(cart_ppu_read(0x0123) == 0x35 && cart_ppu_read(0x1123) == 0xA6);
    cart_cpu_write(0xF000, 1);
    CHECK(cart_ppu_read(0x1123) == 0x35);
    return 0;
}

static int test_vrc1_loader_rejection_preserves_cart(void) {
    CHECK(fixture(75, 0x20000, 0x20000, false) == 75);
    cart_cpu_write(0x8000, 3);
    Mapper *previous = cart;

    iNESHeader invalid = header_for(75, 0x20000, false);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x200001, fixture_chr, 0x20000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x20000, fixture_chr, 0x20001) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);

    invalid.flags7 |= 0x08;
    invalid.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x20000, fixture_chr, 0x20000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);
    return 0;
}

static void vrc3_write_reload(uint16_t reload) {
    cart_cpu_write(0x8000, (uint8_t)reload);
    cart_cpu_write(0x9000, (uint8_t)(reload >> 4));
    cart_cpu_write(0xA000, (uint8_t)(reload >> 8));
    cart_cpu_write(0xB000, (uint8_t)(reload >> 12));
}

static int test_vrc3_banks_irq_and_reset(void) {
    CHECK(fixture(73, 0x20000, 0x2000, false) == 73);
    CHECK(cart != NULL && cart->clock != NULL);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
    CHECK(cart_cpu_read(0xC000) == 14 && cart_cpu_read(0xE000) == 15);
    CHECK(cart_ppu_read(0x0123) == 0 && cart_ppu_read(0x1C12) == 7);

    cart_cpu_write(0xFABC, 3);
    CHECK(cart_cpu_read(0x8000) == 6 && cart_cpu_read(0xA000) == 7);
    CHECK(cart_cpu_read(0xC000) == 14 && cart_cpu_read(0xE000) == 15);

    vrc3_write_reload(0x1234);
    cart_cpu_write(0x8000, 0xFA);
    cart_cpu_write(0x9000, 0xEB);
    cart_cpu_write(0xA000, 0xDC);
    cart_cpu_write(0xB000, 0xCD);
    cart_cpu_write(0xC000, 2);
    cart->clock((int)(0x10000u - 0xDCBAu - 1u));
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());

    vrc3_write_reload(0xFFFE);
    cart_cpu_write(0xC000, 3);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xDFFF, 0);
    CHECK(!cart_irq_pending());
    cart->clock(2);
    CHECK(cart_irq_pending());

    cart_cpu_write(0xC000, 0);
    CHECK(!cart_irq_pending());
    cart->clock(32);
    CHECK(!cart_irq_pending());

    vrc3_write_reload(0xABFE);
    cart_cpu_write(0xC000, 6);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xD000, 0);
    CHECK(!cart_irq_pending());
    cart->clock(4);
    CHECK(!cart_irq_pending());

    cart_cpu_write(0x6000, 0xA6);
    cart->reset();
    CHECK(!cart_irq_pending() && cart_cpu_read(0x6000) == 0xA6);
    CHECK(cart_cpu_read_bus(0x8000, 0x35) == 0x35 && cart_cpu_read(0xC000) == 14);
    return 0;
}

static int test_vrc3_cpu_irq_and_loader(void) {
    CHECK(fixture(73, 0x4000, 0x2000, false) == 73);
    fixture_prg[0] = 0xEA;
    fixture_prg[0x3FFE] = 0;
    fixture_prg[0x3FFF] = 3;
    cart_cpu_write(0xF000, 0);
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cpu.pc = 0x8000;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
    vrc3_write_reload(0xFFFE);
    cart_cpu_write(0xC000, 2);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());

    for (unsigned i = 0; i < 8; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
    cpu.pc = 0x0200;
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    for (unsigned step = 0; step < 3 && cpu.pc != 0x0300; ++step) (void)cpu_step(&cpu);
    CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG) && cart_irq_pending());
    cart_cpu_write(0xD000, 0);
    CHECK(!cart_irq_pending());

    cart_cpu_write(0x6000, 0xA7);
    Mapper *previous = cart;
    iNESHeader invalid = header_for(73, 0x20000, false);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x22000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x6000) == 0xA7);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x20000, fixture_chr, 0x4000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x6000) == 0xA7);
    invalid.flags7 |= 0x08;
    invalid.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x20000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x6000) == 0xA7);
    return 0;
}

static int test_sunsoft3_banks_mirroring_and_irq(void) {
    CHECK(fixture(67, 0x40000, 0x80000, false) == 67);
    CHECK(cart != NULL && cart->clock != NULL);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x18A6) == 0xA6);

    const uint16_t chr_regs[] = {0x8ABC, 0x9FFF, 0xA923, 0xBFFE};
    for (unsigned slot = 0; slot < 4; ++slot) {
        uint8_t bank = (uint8_t)(5 + slot * 3);
        cart_cpu_write(chr_regs[slot], bank);
        CHECK(cart_ppu_read((uint16_t)(slot * 0x800)) == (uint8_t)(bank * 2));
        CHECK(cart_ppu_read((uint16_t)(slot * 0x800 + 0x7FF)) == (uint8_t)(bank * 2 + 1));
    }
    uint8_t before = cart_ppu_read(0);
    cart_cpu_write(0x9000, 0x7F);
    CHECK(cart_ppu_read(0) == before);

    cart_cpu_write(0xFFFF, 7);
    CHECK(cart_cpu_read(0x8000) == 14 && cart_cpu_read(0xA000) == 15);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);

    uint8_t nt[0x1000] = {0};
    cart_cpu_write(0xE800, 0);
    cart_nt_write(0x2000, 0x35, nt);
    CHECK(cart_nt_read(0x2800, nt) == 0x35 && cart_nt_read(0x2400, nt) == 0);
    memset(nt, 0, sizeof(nt));
    cart_cpu_write(0xEFFF, 1);
    cart_nt_write(0x2000, 0x53, nt);
    CHECK(cart_nt_read(0x2400, nt) == 0x53 && cart_nt_read(0x2800, nt) == 0);
    memset(nt, 0, sizeof(nt));
    cart_cpu_write(0xE800, 2);
    cart_nt_write(0x2000, 0x69, nt);
    CHECK(cart_nt_read(0x2C00, nt) == 0x69 && cart_get_mirroring() == MIRROR_SINGLE0);
    memset(nt, 0, sizeof(nt));
    cart_cpu_write(0xE800, 3);
    cart_nt_write(0x2400, 0x96, nt);
    CHECK(cart_nt_read(0x2C00, nt) == 0x96 && cart_get_mirroring() == MIRROR_SINGLE1);

    cart_cpu_write(0xC800, 0x12);
    cart_cpu_write(0xD800, 0);
    cart_cpu_write(0xC800, 0);
    cart_cpu_write(0xCFFF, 1);
    cart_cpu_write(0xD800, 0x10);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart->clock(16);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xD800, 0);
    CHECK(!cart_irq_pending());
    cart->clock(16);
    CHECK(!cart_irq_pending());

    cart->reset();
    CHECK(!cart_irq_pending() && cart_cpu_read_bus(0x8000, 0xA6) == 0xA6);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_ppu_read(0x0123) == 0x23);
    return 0;
}

static int test_sunsoft3_cpu_irq_and_loader(void) {
    CHECK(fixture(67, 0x4000, 0x2000, false) == 67);
    fixture_prg[0] = 0xEA;
    fixture_prg[0x3FFE] = 0;
    fixture_prg[0x3FFF] = 3;
    cart_cpu_write(0xF800, 0);
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cpu.pc = 0x8000;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
    cart_cpu_write(0xC800, 0);
    cart_cpu_write(0xC800, 1);
    cart_cpu_write(0xD800, 0x10);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());

    for (unsigned i = 0; i < 8; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
    cpu.pc = 0x0200;
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    for (unsigned step = 0; step < 3 && cpu.pc != 0x0300; ++step) (void)cpu_step(&cpu);
    CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG));

    cart_cpu_write(0xD800, 0);
    cart_cpu_write(0x6000, 0xA7);
    Mapper *previous = cart;
    iNESHeader invalid = header_for(67, 0x40000, false);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x42000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x6000) == 0xA7);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x80200) == -1);
    CHECK(cart == previous && cart_cpu_read(0x6000) == 0xA7);
    invalid.flags7 |= 0x08;
    invalid.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x6000) == 0xA7);
    return 0;
}

static int test_sunsoft4_banks_nametables_and_timer(void) {
    CHECK(fixture(68, 0x40000, 0x40000, false) == 68);
    CHECK(cart != NULL && cart->clock != NULL);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 14);
    CHECK(cart_cpu_read(0xE000) == 15 && cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x18A6) == 0xA6);

    for (unsigned slot = 0; slot < 4; ++slot) {
        uint8_t bank = (uint8_t)(3 + slot * 5);
        cart_cpu_write((uint16_t)(0x8000u + slot * 0x1000u), bank);
        CHECK(cart_ppu_read((uint16_t)(slot * 0x800u)) == (uint8_t)(bank * 2));
    }

    uint8_t nt[0x1000] = {0};
    cart_cpu_write(0xC000, 1);
    cart_cpu_write(0xD000, 2);
    cart_cpu_write(0xE000, 0x10);
    CHECK(cart_nt_read(0x2000, nt) == 0x81 && cart_nt_read(0x2400, nt) == 0x82);
    CHECK(cart_nt_read(0x2800, nt) == 0x81 && cart_nt_read(0x2C00, nt) == 0x82);
    cart_cpu_write(0xE000, 0x11);
    CHECK(cart_nt_read(0x2000, nt) == 0x81 && cart_nt_read(0x2400, nt) == 0x81);
    CHECK(cart_nt_read(0x2800, nt) == 0x82 && cart_nt_read(0x2C00, nt) == 0x82);
    for (unsigned mode = 2; mode <= 3; ++mode) {
        cart_cpu_write(0xE000, (uint8_t)(0x10 | mode));
        for (unsigned page = 0; page < 4; ++page)
            CHECK(cart_nt_read((uint16_t)(0x2000 + page * 0x400), nt) == 0x7F + mode);
    }
    cart_cpu_write(0xE000, 0);
    cart_nt_write(0x2000, 0x35, nt);
    CHECK(cart_nt_read(0x2800, nt) == 0x35);
    cart_cpu_write(0xE000, 0x10);
    CHECK(cart_nt_read(0x2000, nt) == 0x81);
    cart_cpu_write(0xE000, 0);
    CHECK(cart_nt_read(0x2000, nt) == 0x35);

    cart_cpu_write(0xF000, 0x10);
    CHECK(cart_cpu_read_bus(0x8000, 0x69) == 0x69);
    cart_cpu_write(0x6123, 0xA6);
    CHECK(cart_cpu_read(0x6123) == 0xA6 && cart_cpu_read(0x8000) == 16);
    cart->clock(107519);
    CHECK(cart_cpu_read(0x8000) == 16);
    cart->clock(1);
    CHECK(cart_cpu_read_bus(0x8000, 0x53) == 0x53);
    cart_cpu_write(0x6123, 0xA7);
    CHECK(cart_cpu_read(0x8000) == 16 && cart_cpu_read(0x6123) == 0xA7);
    cart_cpu_write(0xF000, 0x1B);
    CHECK(cart_cpu_read(0x8000) == 6 && cart_cpu_read(0xC000) == 14);
    cart->clock(107520);
    CHECK(cart_cpu_read(0x8000) == 6);
    cart_cpu_write(0xF000, 0x08);
    CHECK(cart_cpu_read_bus(0x6123, 0x96) == 0x96 && cart_cpu_read(0x8000) == 0);
    cart_cpu_write(0x6000, 0x22);
    cart_cpu_write(0xF000, 0x18);
    CHECK(cart_cpu_read(0x6123) == 0xA7);

    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 14);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    return 0;
}

static int test_sunsoft4_chr_ram_persistence_and_loader(void) {
    iNESHeader ram = header_for(68, 0x20000, true);
    CHECK(fixture_with_header(&ram, 0x20000, 0x2000) == 68);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xD000, 1);
    cart_cpu_write(0xE000, 0x10);
    uint8_t nt[0x1000] = {0};
    cart_nt_write(0x2001, 0x35, nt);
    cart_nt_write(0x2401, 0x53, nt);
    CHECK(cart_nt_read(0x2001, nt) == 0x35 && cart_nt_read(0x2401, nt) == 0x53);
    cart_cpu_write(0xE000, 0);
    CHECK(cart_nt_read(0x2001, nt) == 0 && cart_nt_read(0x2401, nt) == 0);
    cart_cpu_write(0xE000, 0x10);
    CHECK(cart_nt_read(0x2001, nt) == 0x35 && cart_nt_read(0x2401, nt) == 0x53);

    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    iNESHeader save = header_for(68, 0x20000, false);
    save.flags7 |= 8;
    save.flags6 |= 2;
    save.flags10 = 0x70;
    save.chr_rom_chunks = 0x20;
    CHECK(fixture_with_header(&save, 0x20000, 0x40000) == 68);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0xF000, 0x18);
    cart_cpu_write(0x6000, 0xA6);
    cart_cpu_write(0x7FFF, 0x69);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x2000);
    CHECK(fixture_with_header(&save, 0x20000, 0x40000) == 68);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0xF000, 0x18);
    CHECK(cart_cpu_read(0x6000) == 0xA6 && cart_cpu_read(0x7FFF) == 0x69);
    int result = save_fixture_end(&paths);
    CHECK(result == 0);

    CHECK(fixture(68, 0x20000, 0x2000, false) == 68);
    cart_cpu_write(0xF000, 0x1B);
    Mapper *previous = cart;
    iNESHeader invalid = header_for(68, 0x20000, false);
    invalid.flags6 |= 0x08;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x20000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 6);
    invalid.flags6 &= (uint8_t)~0x08u;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x42000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 6);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x20000, fixture_chr, 0x80200) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 6);
    return 0;
}

static int test_sunsoft4_cpu_licensed_reads(void) {
    iNESHeader h = header_for(68, 0x40000, false);
    size_t image_size;
    uint8_t *image = image_for(&h, 0x40000, 0x2000, &image_size);
    CHECK(image != NULL);
    image[sizeof(h) + 8 * 0x4000] = 0xA6;
    image[sizeof(h) + 7 * 0x4000 + 0x3FFC] = 0;
    image[sizeof(h) + 7 * 0x4000 + 0x3FFD] = 2;
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu) && cpu.pc == 0x0200);
    const uint8_t read_program[] = {0xAD, 0x00, 0x80};
    const uint8_t refresh_program[] = {0x8D, 0x00, 0x60};
    for (unsigned i = 0; i < 3; ++i) {
        write_mem((uint16_t)(0x0200 + i), read_program[i]);
        write_mem((uint16_t)(0x0300 + i), refresh_program[i]);
    }
    write_mem(0xF000, 0x10);
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x80); // Unlicensed ROM leaves the operand high byte on the bus.
    cpu.pc = 0x0300;
    cpu.a = 0x53;
    CHECK(cpu_step(&cpu) == 4 && read_mem(0x6000) == 0x53);
    cart->clock(107515);
    cpu.pc = 0x0200;
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0xA6);
    cpu.pc = 0x0200;
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x80);
    cpu.pc = 0x0300;
    CHECK(cpu_step(&cpu) == 4);
    cart->clock(107516);
    cpu.pc = 0x0200;
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x80); // Expiry on the final data cycle blocks that read.
    CHECK(unload_rom());
    return 0;
}

static int test_sunsoft_discrete_boards(void) {
    CHECK(fixture(89, 0x20000, 0x20000, false) == 89);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read(0xC000) == 14);
    cart_cpu_write(0x8ABC, 0xD5);
    CHECK(cart_cpu_read(0x8000) == 10 && cart_cpu_read(0xC000) == 14);
    CHECK(cart_ppu_read(0) == 13 * 8 && cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0x8000, 0x55);
    CHECK(cart_cpu_read(0x8000) == 10 && cart_ppu_read(0) == 5 * 8);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0x8000, 0x5D);
    CHECK(cart_cpu_read(0x8000) == 10 && cart_ppu_read(0) == 5 * 8);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    cart_cpu_write(0x8000, 0x6D);
    CHECK(cart_cpu_read(0x8000) == 12 && cart_ppu_read(0) == 5 * 8);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    cart_cpu_write(0xFFFF, 0x7A);
    CHECK(cart_cpu_read(0x8000) == 14 && cart_ppu_read(0) == 2 * 8);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    uint8_t prg_before = cart_cpu_read(0x8000);
    cart_cpu_write(0x6000, 0xA6);
    CHECK(cart_cpu_read(0x6000) == 0xA6 && cart_cpu_read(0x8000) == prg_before);
    cart->reset();
    CHECK(cart_cpu_read_bus(0x8000, 0x69) == 0x69);

    CHECK(fixture(93, 0x20000, 0x2000, false) == 93);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0x8000, 0x51);
    CHECK(cart_cpu_read(0x8000) == 10 && cart_cpu_read(0xC000) == 14);
    CHECK(cart_ppu_read(0x0123) == 0);
    cart_cpu_write(0xFFFF, 0x70);
    CHECK(cart_cpu_read(0x8000) == 14 && cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0x9000, 0x31);
    CHECK(cart_cpu_read(0x8000) == 6 && cart_ppu_read(0x1FFF) == 7);
    cart->reset();
    CHECK(cart_ppu_read(0x0123) == 0x23 && cart_cpu_read_bus(0x8000, 0x35) == 0x35);

    CHECK(fixture(93, 0x20000, 0x2000, true) == 93);
    cart_ppu_write(0x0123, 0xA6);
    CHECK(cart_ppu_read(0x0123) == 0xA6);
    cart_cpu_write(0x8000, 0);
    cart_ppu_write(0x0123, 0x53);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0x8000, 1);
    CHECK(cart_ppu_read(0x0123) == 0xA6);
    cart->reset();
    CHECK(cart_ppu_read(0x0123) == 0xA6);

    CHECK(fixture(184, 0x8000, 0x8000, false) == 184);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xFFFF) == 3);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0x5FFF, 0x76);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0x6000, 0x26);
    CHECK(cart_ppu_read(0) == 24 && cart_ppu_read(0x1000) == 24);
    cart_cpu_write(0x7FFF, 0x31);
    CHECK(cart_ppu_read(0) == 4 && cart_ppu_read(0x1000) == 28);
    cart_cpu_write(0x8000, 0x77);
    CHECK(cart_ppu_read(0) == 4 && cart_cpu_read(0x8000) == 0);
    cart->reset();
    CHECK(cart_ppu_read(0x0123) == 0x23);
    return 0;
}

static int test_sunsoft_discrete_loader_rejection(void) {
    CHECK(fixture(89, 0x20000, 0x20000, false) == 89);
    cart_cpu_write(0x8000, 0x31);
    Mapper *previous = cart;
    iNESHeader h = header_for(89, 0x20000, false);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x22000, fixture_chr, 0x20000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 6);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x22000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 6);
    h = header_for(93, 0x20000, false);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x4000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 6);
    h = header_for(184, 0x8000, false);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x10000, fixture_chr, 0x8000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 6);
    return 0;
}

static int test_sunsoft_discrete_image_loading(void) {
    const unsigned boards[] = {89, 93, 184};
    for (unsigned i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        bool ram = boards[i] == 93;
        iNESHeader h = header_for(boards[i], 0x8000, ram);
        h.flags7 |= 8;
        if (ram) h.zero[0] = 7;
        size_t image_size;
        uint8_t *image = image_for(&h, 0x8000, ram ? 0 : 0x2000, &image_size);
        CHECK(image != NULL);
        CHECK(load_rom_memory(image, image_size) == 0);
        CHECK(rom_mapper_number(&ines_header) == (int)boards[i]);
        write_mem(boards[i] == 184 ? 0x6000 : 0x8000, 1);
        CHECK(read_mem(0xC000) == 0x5C);
        if (ram) {
            cart_ppu_write(0x123, 0x69);
            CHECK(cart_ppu_read(0x123) == 0x69);
        } else {
            CHECK(cart_ppu_read(0x123) == 0xA5);
        }
        Mapper *previous = cart;
        CHECK(load_rom_memory(image, image_size - 1) == -1);
        CHECK(cart == previous && read_mem(0xC000) == 0x5C);
        free(image);
        CHECK(unload_rom());
    }
    return 0;
}

static int test_taito_x1005_and_207(void) {
    const unsigned mappers[] = {80, 207};
    for (size_t i = 0; i < sizeof(mappers) / sizeof(mappers[0]); ++i) {
        CHECK(fixture(mappers[i], 0x40000, 0x40000, false) == (int)mappers[i]);
        CHECK(cart != NULL && cart->clock == NULL);
        CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
        CHECK(cart_cpu_read_bus(0xA000, 0x69) == 0x69);
        CHECK(cart_cpu_read_bus(0xC000, 0xA6) == 0xA6);
        CHECK(cart_cpu_read(0xE000) == 31);
        CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x1456) == 0x56);

        cart_cpu_write(0x7EFA, 3);
        cart_cpu_write(0x7EFC, 5);
        cart_cpu_write(0x7EFE, 7);
        CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xA000) == 5);
        CHECK(cart_cpu_read(0xC000) == 7 && cart_cpu_read(0xE000) == 31);

        cart_cpu_write(0x7EF0, 0x84);
        cart_cpu_write(0x7EF1, 0x46);
        cart_cpu_write(0x7EF2, 9);
        cart_cpu_write(0x7EF3, 11);
        cart_cpu_write(0x7EF4, 13);
        cart_cpu_write(0x7EF5, 15);
        const uint8_t expected_chr[] = {0x84, 0x85, 0x46, 0x47, 9, 11, 13, 15};
        for (unsigned slot = 0; slot < 8; ++slot)
            CHECK(cart_ppu_read((uint16_t)(slot * 0x400u)) == expected_chr[slot]);

        CHECK(cart_cpu_read_bus(0x7F00, 0x35) == 0x35);
        cart_cpu_write(0x7F00, 0xA6);
        cart_cpu_write(0x7EF8, 0xA3);
        CHECK(cart_cpu_read(0x7F00) == 0 && cart_cpu_read(0x7F80) == 0);
        cart_cpu_write(0x7F00, 0x35);
        CHECK(cart_cpu_read(0x7F00) == 0x35 && cart_cpu_read(0x7F80) == 0x35);
        cart_cpu_write(0x7F80, 0x53);
        CHECK(cart_cpu_read(0x7F00) == 0x53 && cart_cpu_read(0x7F80) == 0x53);
        cart_cpu_write(0x7EF9, 0);
        CHECK(cart_cpu_read_bus(0x7F00, 0x96) == 0x96);
        cart_cpu_write(0x7EF8, 0xA3);
        CHECK(cart_cpu_read(0x7F00) == 0x53 && cart_cpu_read(0x7F80) == 0x53);

        if (mappers[i] == 80) {
            cart_cpu_write(0x7EF6, 1);
            CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
            cart_cpu_write(0x7EF7, 0);
            CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
        } else {
            uint8_t nt[0x1000] = {0};
            nt[0] = 0x10;
            nt[0x400] = 0x20;
            CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x2400, nt) == 0x20);
            CHECK(cart_nt_read(0x2800, nt) == 0x10 && cart_nt_read(0x2C00, nt) == 0x10);
            cart_nt_write(0x2001, 0xA1, nt);
            CHECK(nt[0x401] == 0xA1 && cart_nt_read(0x2401, nt) == 0xA1);
            Mirroring before = cart_get_mirroring();
            cart_cpu_write(0x7EF6, 1);
            CHECK(cart_get_mirroring() == before);
        }

        cart->reset();
        CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read(0xE000) == 31);
        CHECK(cart_cpu_read_bus(0x7F00, 0x69) == 0x69);
        CHECK(cart_ppu_read(0x0123) == 0x23);
    }
    return 0;
}

static int test_taito_x1017_banks_chr_and_ram(void) {
    CHECK(fixture(82, 0x40000, 0x40000, false) == 82);
    CHECK(cart != NULL && cart->clock == NULL);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_ppu_read(0x0123) == 0x23);

    cart_cpu_write(0x7EFA, 12);
    cart_cpu_write(0x7EFB, 20);
    cart_cpu_write(0x7EFC, 28);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xA000) == 5);
    CHECK(cart_cpu_read(0xC000) == 7 && cart_cpu_read(0xE000) == 31);

    const uint8_t chr_regs[] = {9, 13, 15, 18, 21, 24};
    for (unsigned i = 0; i < 6; ++i)
        cart_cpu_write((uint16_t)(0x7EF0u + i), chr_regs[i]);
    CHECK(cart_ppu_read(0x0000) == 8 && cart_ppu_read(0x0400) == 9);
    CHECK(cart_ppu_read(0x0800) == 12 && cart_ppu_read(0x0C00) == 13);
    CHECK(cart_ppu_read(0x1000) == 15 && cart_ppu_read(0x1400) == 18);
    CHECK(cart_ppu_read(0x1800) == 21 && cart_ppu_read(0x1C00) == 24);

    cart_cpu_write(0x7EF6, 3);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    CHECK(cart_ppu_read(0x0000) == 15 && cart_ppu_read(0x0400) == 18);
    CHECK(cart_ppu_read(0x0800) == 21 && cart_ppu_read(0x0C00) == 24);
    CHECK(cart_ppu_read(0x1000) == 8 && cart_ppu_read(0x1400) == 9);
    CHECK(cart_ppu_read(0x1800) == 12 && cart_ppu_read(0x1C00) == 13);

    CHECK(cart_cpu_read_bus(0x6000, 0x35) == 0x35);
    cart_cpu_write(0x7EF7, 0xCA);
    cart_cpu_write(0x6000, 0x35);
    cart_cpu_write(0x6400, 0x53);
    CHECK(cart_cpu_read(0x6000) == 0x35 && cart_cpu_read(0x6400) == 0x53);
    CHECK(cart_cpu_read_bus(0x6800, 0x69) == 0x69);
    cart_cpu_write(0x7EF8, 0x69);
    cart_cpu_write(0x6800, 0xA6);
    cart_cpu_write(0x6C00, 0x7A);
    CHECK(cart_cpu_read(0x6800) == 0xA6 && cart_cpu_read(0x6C00) == 0x7A);
    CHECK(cart_cpu_read_bus(0x7000, 0x96) == 0x96);
    cart_cpu_write(0x7EF9, 0x84);
    cart_cpu_write(0x7000, 0x96);
    CHECK(cart_cpu_read(0x7000) == 0x96 && cart_cpu_read_bus(0x7400, 0x35) == 0x35);

    cart_cpu_write(0x7EF8, 0x68);
    CHECK(cart_cpu_read_bus(0x6800, 0x53) == 0x53);
    cart_cpu_write(0x7EF8, 0x69);
    CHECK(cart_cpu_read(0x6800) == 0xA6 && cart_cpu_read(0x6C00) == 0x7A);

    cart->reset();
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    CHECK(cart_cpu_read_bus(0x8000, 0x69) == 0x69 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    return 0;
}

static int test_taito_x1_persistence_and_loader(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);

    iNESHeader h80 = header_for(80, 0x40000, false);
    h80.flags6 |= 2;
    CHECK(fixture_with_header(&h80, 0x40000, 0x40000) == 80);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0x7EF8, 0xA3);
    cart_cpu_write(0x7F00, 0xA6);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x100);
    CHECK(saved_byte(paths.prg_save, 0) == 0xA6 && saved_byte(paths.prg_save, 0x80) == 0xA6);
    CHECK(fixture_with_header(&h80, 0x40000, 0x40000) == 80);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0x7EF8, 0xA3);
    CHECK(cart_cpu_read(0x7F00) == 0xA6 && cart_cpu_read(0x7F80) == 0xA6);
    CHECK(save_fixture_end(&paths) == 0);

    CHECK(save_fixture_begin(&paths) == 0);
    iNESHeader h82 = header_for(82, 0x40000, false);
    h82.flags6 |= 2;
    CHECK(fixture_with_header(&h82, 0x40000, 0x40000) == 82);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0x7EF7, 0xCA);
    cart_cpu_write(0x7EF8, 0x69);
    cart_cpu_write(0x7EF9, 0x84);
    cart_cpu_write(0x6000, 0x35);
    cart_cpu_write(0x6800, 0x53);
    cart_cpu_write(0x7000, 0x96);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x1400);
    CHECK(fixture_with_header(&h82, 0x40000, 0x40000) == 82);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0x7EF7, 0xCA);
    cart_cpu_write(0x7EF8, 0x69);
    cart_cpu_write(0x7EF9, 0x84);
    CHECK(cart_cpu_read(0x6000) == 0x35 && cart_cpu_read(0x6800) == 0x53);
    CHECK(cart_cpu_read(0x7000) == 0x96);
    CHECK(save_fixture_end(&paths) == 0);

    CHECK(fixture(207, 0x40000, 0x40000, false) == 207);
    cart_cpu_write(0x7EFA, 3);
    Mapper *previous = cart;
    iNESHeader invalid = header_for(207, 0x40000, false);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x202000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x40400) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);

    invalid.flags7 |= 8;
    invalid.flags10 = 7;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);

    iNESHeader valid80 = header_for(80, 0x40000, false);
    valid80.flags7 |= 8;
    valid80.flags10 = 2;
    CHECK(fixture_with_header(&valid80, 0x40000, 0x40000) == 80);
    iNESHeader valid82 = header_for(82, 0x40000, false);
    valid82.flags7 |= 8;
    valid82.flags10 = 0;
    CHECK(fixture_with_header(&valid82, 0x40000, 0x40000) == 82);
    iNESHeader valid207 = header_for(207, 0x40000, false);
    valid207.flags7 |= 8;
    valid207.flags10 = 2;
    CHECK(fixture_with_header(&valid207, 0x40000, 0x40000) == 207);
    return 0;
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
        test_mapper105_competition_board,
        test_mapper105_fixed_chr_and_serial_timing, test_mapper105_dips_and_cpu_irq,
        test_mapper232_multicart_banks,
        test_mapper232_loader_and_cpu_bus,
        test_mmc1a_ram_revision, test_mmc1a_cpu_serial_writes, test_mmc1a_ram_layouts_and_loader,
        test_mmc1_outer_and_fixed_banks, test_mmc2_banks_and_latches,
        test_mmc4_latches_and_chr_ram, test_mmc3_banks_and_protection,
        test_mmc3_irq_edges, test_mmc3_revision_a_irq, test_mmc3_revision_a_cpu_irq,
        test_mmc3_render_trace, test_tqrom_mixed_chr_memory,
        test_mcacc_irq_divider, test_mcacc_irq_reload_and_enable,
        test_mcacc_banks_ram_and_loader, test_mcacc_cpu_ppu_irq_path,
        test_taito_banks_aliases_and_mirroring, test_taito48_irq, test_taito_loader_transaction,
        test_taito48_cpu_irq,
        test_txsrom_nametable_routing, test_txsrom_startup_ram_and_loader,
        test_rambo1_banks_and_modes, test_rambo1_irq_sources, test_rambo158_nametables,
        test_rambo1_ram_and_odd_chr_banks, test_rambo1_irq_boundaries,
        test_rambo1_cpu_and_rendering_irq, test_rambo1_loader,
        test_simple_mapper_registers,
        test_vrc6_startup_banks_wiring_and_ram, test_vrc6_nametable_modes,
        test_vrc6_irq_timing, test_vrc6_pulse_and_saw_audio,
        test_vrc6_loader_rejection_preserves_cart,
        test_vrc6_cpu_boot_and_odd_prg_size,
        test_colordreams_bus_conflicts, test_bus_conflict_submappers,
        test_mapper15_modes, test_action53_banks_and_mirroring,
        test_action53_game_sizes, test_action53_largest_image,
        test_unrom512_banks_flash_and_mirroring, test_mmc5_memory_windows,
        test_unrom512_physical_flash_address, test_unrom512_cpu_flash,
        test_mmc5_exram_and_irq, test_mmc5_chr_fetch_modes, test_mmc5_extended_rendering,
        test_mmc5_rendered_ppu_paths, test_mmc5_audio_and_pcm, test_header_and_mapper_rejection,
        test_loader_trainers_and_sizes, test_loader_rejection_preserves_cart,
        test_loader_region_and_console_type,
        test_ram_header_sizes, test_prg_ram_capacity, test_mmc1_banked_ram,
        test_mmc5_banked_ram, test_loader_ram_layouts, test_prg_nvram_persistence,
        test_vs_nvram_persistence,
        test_unrom512_flash_persistence,
        test_mmc5_persistence,
        test_chr_nvram_persistence, test_chr_nvram_writers,
        test_mmc6_ram_mirroring, test_mmc6_protection, test_mmc6_banks_and_irq,
        test_namco163_banks_ram_and_nametables, test_namco163_irq_and_audio,
        test_namco175_340_variants, test_namco163_persistence_and_loader,
        test_mapper34_bnrom_banking_and_ram, test_mapper34_nina_banks_and_selection,
        test_mapper34_loader_preserves_cart, test_mapper34_image_loading,
        test_gxrom_banks_reset_and_ram, test_gxrom_chr_ram_and_loader,
        test_mapper71_variants_and_mirroring, test_mapper71_loader_and_chr_rom,
        test_mapper71_image_loading, test_legacy_loader_ram_and_large_prg_metadata,
        test_namco108_banks_aliases_and_irq_absence, test_namco108_submapper_loader_and_chr_ram,
        test_namco108_variants, test_namco108_variant_loader_rejection,
        test_namco108_variant_image_loading,
        test_sunsoft69_banks_ram_and_startup, test_sunsoft69_legacy_ram_defaults,
        test_sunsoft69_irq_cpu_clock,
        test_sunsoft5b_tone_noise_envelope, test_sunsoft69_persistence_and_loader,
        test_jaleco18_banks_ram_and_mirroring, test_jaleco18_irq_and_cpu_clock,
        test_jaleco18_irq_width_transitions,
        test_jaleco18_loader_validation,
        test_irem32_banks_variants_and_ram, test_irem65_banks_decode_and_ram,
        test_irem65_irq_and_loader_validation,
        test_irem_ram_and_irq_boundaries,
        test_vrc24_variant_register_wiring, test_vrc24_ram_latch_and_mapper183_window,
        test_vrc24_irq_variants_and_phase, test_vrc24_loader_rejection_preserves_cart,
        test_vrc7_banks_wiring_and_ram, test_vrc7_irq_timing, test_vrc7_fm_audio,
        test_vrc7_register_boundaries,
        test_vrc7_loader_rejection_preserves_cart,
        test_vrc1_banks_mirroring_and_reset, test_vrc1_loader_rejection_preserves_cart,
        test_vrc3_banks_irq_and_reset, test_vrc3_cpu_irq_and_loader,
        test_sunsoft3_banks_mirroring_and_irq, test_sunsoft3_cpu_irq_and_loader,
        test_sunsoft4_banks_nametables_and_timer, test_sunsoft4_chr_ram_persistence_and_loader,
        test_sunsoft4_cpu_licensed_reads,
        test_sunsoft_discrete_boards, test_sunsoft_discrete_loader_rejection,
        test_sunsoft_discrete_image_loading,
        test_taito_x1005_and_207, test_taito_x1017_banks_chr_and_ram,
        test_taito_x1_persistence_and_loader,
        test_cartridge_bus_reads, test_mmc6_persistence, test_cartridge_unload
    };
    int failures = 0;
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) failures += tests[i]();
    unload_rom();
    printf("Mapper accuracy: %zu groups, %d failures\n", sizeof(tests) / sizeof(tests[0]), failures);
    return failures;
}
