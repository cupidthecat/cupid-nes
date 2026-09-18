/*
 * board_unlicensed_115_accuracy.c - Multicart protection, PPU latches, and VRC timing
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#include "board_tests.h"
#include <time.h>

static bool prg8_115(uint16_t address, unsigned bank) {
    return read_mem(address) == (uint8_t)(bank * 2)
        && read_mem((uint16_t)(address + 1)) == (uint8_t)(bank >> 7)
        && read_mem((uint16_t)(address + 0x1FFF)) == (uint8_t)(bank * 2 + 1);
}

static bool prg16_115(uint16_t address, unsigned bank) {
    return prg8_115(address, bank * 2) && prg8_115((uint16_t)(address + 0x2000), bank * 2 + 1);
}

static bool prg32_115(unsigned bank) {
    return prg16_115(0x8000, bank * 2) && prg16_115(0xC000, bank * 2 + 1);
}

static bool chr4_115(uint16_t address, unsigned bank) {
    return ppu_read(address) == (uint8_t)(bank * 4)
        && ppu_read((uint16_t)(address + 1)) == (uint8_t)(bank >> 6)
        && ppu_read((uint16_t)(address + 0xFFF)) == (uint8_t)(bank * 4 + 3);
}

static bool chr8_115(unsigned bank) { return chr4_115(0, bank * 2) && chr4_115(0x1000, bank * 2 + 1); }

static int store115(uint16_t address, uint8_t value) {
    const uint8_t code[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(code); ++byte) write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int cycles115(unsigned cycles) {
    if (cycles & 1) {
        BOARD_CHECK(cycles >= 3);
        write_mem(0x0300, 0x4C);
        write_mem(0x0301, 0);
        write_mem(0x0302, 3);
        cpu.pc = 0x0300;
        BOARD_CHECK(cpu_step(&cpu) == 3);
        cycles -= 3;
    }
    write_mem(0x0300, 0xEA);
    for (unsigned elapsed = 0; elapsed < cycles; elapsed += 2) {
        cpu.pc = 0x0300;
        BOARD_CHECK(cpu_step(&cpu) == 2);
    }
    return 0;
}

static int mirror115(Mirroring expected) {
    BOARD_CHECK(cart_get_mirroring() == expected);
    ppu_write(0x2000, 0xA5);
    if (expected == MIRROR_HORIZONTAL || expected == MIRROR_VERTICAL) {
        ppu_write(expected == MIRROR_HORIZONTAL ? 0x2800 : 0x2400, 0x5A);
        BOARD_CHECK(ppu_read(expected == MIRROR_HORIZONTAL ? 0x2400 : 0x2800) == 0xA5);
        BOARD_CHECK(ppu_read(0x2C00) == 0x5A);
    } else {
        BOARD_CHECK(ppu_read(0x2400) == 0xA5 && ppu_read(0x2800) == 0xA5 && ppu_read(0x2C00) == 0xA5);
    }
    BOARD_CHECK(ppu_read(0x3000) == 0xA5);
    return 0;
}

static int test_331_outer_bank_and_chr_halves(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 331, 0x80000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_115(0x8000, 0) && prg16_115(0xC000, 7));
    BOARD_CHECK(chr4_115(0, 0) && chr4_115(0x1000, 0));
    BOARD_CHECK(store115(0xBFFF, 0xB5) == 0 && store115(0xDFFF, 0xDA) == 0);
    BOARD_CHECK(prg16_115(0x8000, 5) && prg16_115(0xC000, 7));
    BOARD_CHECK(chr4_115(0, 22) && chr4_115(0x1000, 27));
    BOARD_CHECK(store115(0xFFFF, 0xF6) == 0 && prg16_115(0x8000, 21) && prg16_115(0xC000, 23));
    BOARD_CHECK(chr4_115(0, 86) && chr4_115(0x1000, 91) && mirror115(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store115(0xE000, 0x0B) == 0 && prg16_115(0x8000, 28) && prg16_115(0xC000, 29));
    BOARD_CHECK(chr4_115(0, 118) && chr4_115(0x1000, 123) && mirror115(MIRROR_VERTICAL) == 0);
    BOARD_CHECK(store115(0x9FFF, 0) == 0 && store115(0x7FFF, 0) == 0 && prg16_115(0x8000, 28));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg16_115(0x8000, 28) && chr4_115(0, 118));
    board_image_free(&image);
    return 0;
}

static int test_332_register_lock_and_startup(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 332, 0x400000, 0x20000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_add_trainer(&image, 0x3E));
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_115(0x8000, 0) && prg16_115(0xC000, 1));
    BOARD_CHECK(ppu_read(0x123) == 0x23 && ppu_read(0x456) == 0x56 && read_mem(0x7000) == 0x3E);
    BOARD_CHECK(store115(0x6FFF, 9) == 0 && chr8_115(9) && read_mem(0x6FFF) == 0);
    BOARD_CHECK(store115(0x6FFE, 0x1D) == 0 && prg16_115(0x8000, 29) && prg16_115(0xC000, 29));
    BOARD_CHECK(mirror115(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store115(0x6000, 0xC5) == 0 && prg16_115(0x8000, 196) && prg16_115(0xC000, 197));
    BOARD_CHECK(store115(0x6000, 0x3A) == 0 && prg16_115(0x8000, 58) && prg16_115(0xC000, 58));
    BOARD_CHECK(store115(0x6001, 3) == 0 && store115(0x6000, 0) == 0 && chr8_115(9));
    BOARD_CHECK(prg16_115(0x8000, 58) && store115(0x7000, 0xA5) == 0 && read_mem(0x7000) == 0xA5);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(store115(0x6001, 2) == 0 && chr8_115(9) && prg16_115(0xC000, 58));
    BOARD_CHECK(read_mem(0x7000) == 0xA5 && board_image_load(&image) == 0);
    BOARD_CHECK(store115(0x6001, 2) == 0 && chr8_115(2) && prg16_115(0x8000, 0));
    board_image_free(&image);
    return 0;
}

static int test_336_and_349_prg_modes(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 336, 0x100000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_115(0x8000, 0) && prg16_115(0xC000, 7));
    BOARD_CHECK(store115(0xFFFF, 0xED) == 0 && prg16_115(0x8000, 45) && prg16_115(0xC000, 47));
    ppu_write(0x123, 0x6D);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg16_115(0x8000, 45) && ppu_read(0x123) == 0x6D && mirror115(MIRROR_HORIZONTAL) == 0);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 349, 0x80000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_115(0x8000, 0) && prg16_115(0xC000, 1));
    BOARD_CHECK(store115(0x80DB, 0xFF) == 0 && prg16_115(0x8000, 27) && prg16_115(0xC000, 27));
    BOARD_CHECK(mirror115(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store115(0x8015, 0) == 0 && prg16_115(0x8000, 20) && prg16_115(0xC000, 21));
    BOARD_CHECK(store115(0x8852, 0xAA) == 0 && prg16_115(0x8000, 18) && prg16_115(0xC000, 23));
    BOARD_CHECK(store115(0x8813, 0) == 0 && prg16_115(0x8000, 19) && prg16_115(0xC000, 23));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg16_115(0x8000, 0) && prg16_115(0xC000, 1) && chr8_115(0));
    BOARD_CHECK(mirror115(MIRROR_VERTICAL) == 0);
    board_image_free(&image);
    return 0;
}

static int test_487_register_modes_and_decode(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 487, 0x200000, 0x200000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0 && prg32_115(0) && chr8_115(0));
    BOARD_CHECK(store115(0x4100, 0x0D) == 0 && prg32_115(0) && chr8_115(1));
    BOARD_CHECK(store115(0x4180, 0x85) == 0 && prg32_115(5) && chr8_115(21));
    BOARD_CHECK(mirror115(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store115(0x5FFF, 0x44) == 0 && prg32_115(5) && chr8_115(21));
    BOARD_CHECK(store115(0x5100, 2) == 0 && prg32_115(4) && chr8_115(18));
    const uint16_t ignored[] = {0x4080, 0x4200, 0x5080, 0x8000, 0xFFFF};
    for (unsigned reg = 0; reg < sizeof(ignored) / sizeof(ignored[0]); ++reg) {
        BOARD_CHECK(store115(ignored[reg], 0xFF) == 0 && prg32_115(4) && chr8_115(18));
    }
    BOARD_CHECK(store115(0x4180, 0x65) == 0 && prg32_115(20) && chr8_115(82));
    BOARD_CHECK(store115(0x4100, 0x0D) == 0 && prg32_115(20) && chr8_115(82));
    BOARD_CHECK(store115(0xFFFF, 0x51) == 0 && prg32_115(21) && chr8_115(85));
    BOARD_CHECK(store115(0x6000, 0x6D) == 0 && read_mem(0x6000) == 0x6D && prg32_115(21));
    BOARD_CHECK(cart_cpu_read_bus(0x4180, 0xA5) == 0xA5 && chr8_115(85));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg32_115(0) && chr8_115(0) && read_mem(0x6000) == 0x6D && mirror115(MIRROR_VERTICAL) == 0);
    board_image_free(&image);
    return 0;
}

static int test_518_ppu_address_latch_and_open_bus(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 518, 0x40000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_115(0x8000, 0) && prg16_115(0xC000, 0));
    BOARD_CHECK(chr4_115(0, 0) && chr4_115(0x1000, 1));
    BOARD_CHECK(store115(0x5000, 5) == 0 && prg16_115(0x8000, 5) && prg16_115(0xC000, 0));
    BOARD_CHECK(store115(0x5200, 3) == 0 && cart_get_mirroring() == MIRROR_VERTICAL);
    (void)ppu_read(0x2800);
    BOARD_CHECK(chr4_115(0, 1) && chr4_115(0x1000, 1));
    (void)ppu_read(0x3000);
    BOARD_CHECK(chr4_115(0, 1));
    (void)ppu_read(0x2400);
    BOARD_CHECK(chr4_115(0, 0));
    ppu.scanline = 0; ppu.dot = 1; ppu.v = 0x800;
    ppu.mask = 0x08; ppu.rendering_enabled = true; ppu.fetches_enabled = true;
    ppu_step_dots(2);
    BOARD_CHECK(chr4_115(0, 1));
    ppu.v = 0; ppu.dot = 9;
    ppu_step_dots(2);
    BOARD_CHECK(chr4_115(0, 0));
    ppu.mask = 0; ppu.rendering_enabled = false; ppu.fetches_enabled = false;
    BOARD_CHECK(store115(0x5000, 5) == 0 && mirror115(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store115(0x5200, 7) == 0 && prg16_115(0x8000, 10) && prg16_115(0xC000, 11));
    BOARD_CHECK(store115(0x5200, 0) == 0 && prg16_115(0x8000, 10) && cart_get_mirroring() == MIRROR_HORIZONTAL);
    BOARD_CHECK(store115(0x5001, 0) == 0 && store115(0x5201, 0) == 0 && prg16_115(0xC000, 11));
    BOARD_CHECK(store115(0x5000, 0x45) == 0 && cart_cpu_read_bus(0x8000, 0xA5) == 0xA5);
    BOARD_CHECK(cart_cpu_read_bus(0xFFFF, 0x5A) == 0x5A && cart_get_mirroring() == MIRROR_VERTICAL);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cart_cpu_read_bus(0xC000, 0x6D) == 0x6D);
    BOARD_CHECK(store115(0xFFFF, 0) == 0 && cart_cpu_read_bus(0x8000, 0xA5) == 0xA5);
    BOARD_CHECK(store115(0x5000, 5) == 0 && prg16_115(0x8000, 5) && prg16_115(0xC000, 0));
    board_image_free(&image);
    return 0;
}

static int test_519_dip_reads_and_reset(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 519, 0x20000, 0x20000, true));
    for (unsigned nibble = 0; nibble < 16; ++nibble)
        image.data[16 + 5 * 0x4000 + 0x120 + nibble] = (uint8_t)(0xA0 + nibble);
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_115(0x8000, 0) && prg16_115(0xC000, 1));
    BOARD_CHECK(store115(0x80C5, 0x1B) == 0 && cart_set_dip_switches(10));
    BOARD_CHECK(read_mem(0x8123) == 0xAA && read_mem(0xC12F) == 0xAA && chr8_115(11));
    BOARD_CHECK(store115(0x81FF, 0) == 0 && read_mem(0x8123) == 0xAA && chr8_115(11));
    BOARD_CHECK(cart_set_dip_switches(5) && read_mem(0x8120) == 0xA5);
    BOARD_CHECK(store115(0x8085, 4) == 0 && read_mem(0x8123) == 0xA3 && chr8_115(4));
    BOARD_CHECK(store115(0x8004, 8) == 0 && prg16_115(0x8000, 4) && prg16_115(0xC000, 5));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg16_115(0x8000, 0) && prg16_115(0xC000, 1) && chr8_115(0));
    BOARD_CHECK(mirror115(MIRROR_VERTICAL) == 0 && cart_set_dip_switches(0));
    board_image_free(&image);
    return 0;
}

static int test_521_single_register_and_fixed_bank(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 521, 0x40000, 0, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_115(0x8000, 0) && prg16_115(0xC000, 8));
    BOARD_CHECK(store115(0x5020, 0xFD) == 0 && prg16_115(0x8000, 5) && prg16_115(0xC000, 8));
    const uint16_t ignored[] = {0x501F, 0x5021, 0x5120, 0x8000, 0xFFFF};
    for (unsigned reg = 0; reg < sizeof(ignored) / sizeof(ignored[0]); ++reg)
        BOARD_CHECK(store115(ignored[reg], 0) == 0 && prg16_115(0x8000, 5));
    BOARD_CHECK(cart_cpu_read_bus(0x5020, 0x5A) == 0x5A && store115(0x6000, 0x6D) == 0);
    ppu_write(0x123, 0xA5);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg16_115(0x8000, 5) && prg16_115(0xC000, 8) && ppu_read(0x123) == 0xA5);
    BOARD_CHECK(read_mem(0x6000) == 0x6D && mirror115(MIRROR_HORIZONTAL) == 0);
    board_image_free(&image);
    return 0;
}

static int test_529_rom_and_ram_bank_wiring(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 529, 0x100000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg8_115(0x8000, 0) && prg8_115(0xA000, 0));
    BOARD_CHECK(prg8_115(0xC000, 30) && prg8_115(0xE000, 127));
    BOARD_CHECK(store115(0xA03F, 0x13) == 0 && prg8_115(0x8000, 38) && prg8_115(0xA000, 39));
    BOARD_CHECK(store115(0x9020, 2) == 0 && prg8_115(0x8000, 30) && prg8_115(0xC000, 38));
    const uint16_t chr_regs[] = {0xB000, 0xB020, 0xC000, 0xC020, 0xD000, 0xD020, 0xE000, 0xE020};
    for (unsigned slot = 0; slot < 8; ++slot) {
        BOARD_CHECK(store115(chr_regs[slot], (uint8_t)(slot + 1)) == 0);
        BOARD_CHECK(store115((uint16_t)(chr_regs[slot] | 0x10), (uint8_t)(slot + 17)) == 0);
        unsigned page = (slot + 17) * 16 + slot + 1;
        BOARD_CHECK(ppu_read((uint16_t)(slot * 0x400)) == (uint8_t)page);
        BOARD_CHECK(ppu_read((uint16_t)(slot * 0x400 + 1)) == page / 256);
    }
    BOARD_CHECK(store115(0x9004, 3) == 0 && mirror115(MIRROR_SINGLE1) == 0);
    BOARD_CHECK(store115(0x9000, 0x83) == 0 && mirror115(MIRROR_SINGLE1) == 0);
    BOARD_CHECK(store115(0x9001, 2) == 0 && mirror115(MIRROR_SINGLE0) == 0);
    BOARD_CHECK(store115(0x9000, 0) == 0 && mirror115(MIRROR_VERTICAL) == 0);
    BOARD_CHECK(store115(0x9000, 1) == 0 && mirror115(MIRROR_HORIZONTAL) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_115(0x8000, 30) && prg8_115(0xC000, 38) && ppu_read(1) == 1);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 529, 0x100000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0 && store115(0xA000, 3) == 0);
    ppu_write(0x123, 0x6D);
    BOARD_CHECK(store115(0xB030, 8) == 0 && prg8_115(0x8000, 38) && prg8_115(0xC000, 62));
    BOARD_CHECK(prg8_115(0xA000, 7) && prg8_115(0xE000, 127) && ppu_read(0x123) == 0x6D);
    BOARD_CHECK(store115(0x902A, 2) == 0 && prg8_115(0x8000, 62) && prg8_115(0xC000, 38));
    BOARD_CHECK(store115(0xE015, 0) == 0 && prg8_115(0x8000, 30) && prg8_115(0xC000, 6));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_115(0x8000, 30) && prg8_115(0xC000, 6) && ppu_read(0x123) == 0x6D);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 529, 0x100000, 0x80000, true));
    image.data[11] = 7;
    BOARD_CHECK(board_image_load(&image) == 0 && store115(0xA000, 3) == 0);
    BOARD_CHECK(store115(0xB000, 8) == 0 && prg8_115(0x8000, 38) && chr8_115(0));
    ppu_write(0x123, 0x6D);
    BOARD_CHECK(ppu_read(0x123) == 0 && ppu_read(0x1000) == 4);
    board_image_free(&image);
    return 0;
}

static int test_529_irq_cpu_and_scanline_modes(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 529, 0x40000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0xF000, 0x0E);
    write_mem(0xF004, 0x0F);
    BOARD_CHECK(store115(0xF008, 7) == 0 && !cart_irq_pending());
    BOARD_CHECK(cycles115(2) == 0 && cart_irq_pending());
    write_mem(0xF03F, 0);
    BOARD_CHECK(!cart_irq_pending() && cycles115(2) == 0 && cart_irq_pending());
    write_mem(0xF002, 6);
    BOARD_CHECK(!cart_irq_pending() && cycles115(2) == 0 && cart_irq_pending());
    write_mem(0xF003, 0);
    BOARD_CHECK(!cart_irq_pending() && cycles115(20) == 0 && !cart_irq_pending());

    write_mem(0xF000, 0x0F);
    BOARD_CHECK(store115(0xF022, 3) == 0);
    BOARD_CHECK(cycles115(113) == 0 && !cart_irq_pending());
    BOARD_CHECK(cycles115(3) == 0 && cart_irq_pending());
    write_mem(0xF003, 0);
    BOARD_CHECK(cycles115(111) == 0 && !cart_irq_pending());
    BOARD_CHECK(cycles115(2) == 0 && cart_irq_pending());

    write_mem(0xF000, 0x0D);
    BOARD_CHECK(store115(0xF002, 3) == 0 && cycles115(340) == 0 && !cart_irq_pending());
    BOARD_CHECK(store115(0xF002, 3) == 0 && cycles115(341) == 0 && cart_irq_pending());
    write_mem(0xF003, 0);
    BOARD_CHECK(!cart_irq_pending());
    cpu_soft_reset(&cpu);
    BOARD_CHECK(!cart_irq_pending() && cycles115(333) == 0 && !cart_irq_pending());
    BOARD_CHECK(cycles115(2) == 0 && cart_irq_pending());
    write_mem(0xF002, 0);
    BOARD_CHECK(!cart_irq_pending() && cycles115(400) == 0 && !cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static int test_530_scrambled_prg_and_chr_lines(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 530, 0x40000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && cart_cpu_read_bus(0x8000, 0xA5) == 0xA5);
    BOARD_CHECK(cart_cpu_read_bus(0xA000, 0x5A) == 0x5A && prg8_115(0xC000, 30) && prg8_115(0xE000, 31));
    BOARD_CHECK(store115(0x8000, 2) == 0 && prg8_115(0x8000, 8));
    BOARD_CHECK(store115(0xA000, 8) == 0 && prg8_115(0xA000, 2));
    BOARD_CHECK(store115(0x8FF0, 15) == 0 && prg8_115(0x8000, 15));
    const uint16_t registers[] = {0xA008, 0xA00A, 0xC000, 0xC002, 0xC008, 0xC00A, 0xE000, 0xE002};
    const uint8_t high_writes[] = {2, 4, 1, 8, 3, 5, 6, 15};
    const uint8_t high_results[] = {4, 2, 1, 8, 5, 3, 6, 15};
    for (unsigned slot = 0; slot < 8; ++slot) {
        BOARD_CHECK(store115((uint16_t)(registers[slot] | 0x7F0), (uint8_t)(slot + 1)) == 0);
        BOARD_CHECK(store115((uint16_t)(registers[slot] | 0x7F1), high_writes[slot]) == 0);
        BOARD_CHECK(ppu_read((uint16_t)(slot * 0x400)) == high_results[slot] * 16 + slot + 1);
    }
    BOARD_CHECK(store115(0x8008, 1) == 0 && mirror115(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store115(0xA007, 0) == 0 && store115(0xB000, 0) == 0 && prg8_115(0xA000, 2));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_115(0x8000, 15) && ppu_read(0) == 65 && ppu_read(0x1C00) == 248);
    board_image_free(&image);
    return 0;
}

static int test_discrete115_geometry_and_replacement(void) {
    const unsigned ids[] = {331, 332, 336, 349, 487, 518, 519, 521, 529, 530};
    for (unsigned item = 0; item < sizeof(ids) / sizeof(ids[0]); ++item) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[item], 0x6000, 0x600, true));
        image.data[8] |= 0xF0;
        memset(image.data + 16, 0xA7, 0x6000);
        memset(image.data + 16 + 0x6000, 0xB8, 0x600);
        BOARD_CHECK(board_image_load(&image) == 0);
        uint16_t reg = ids[item] == 332 ? 0x6001 : ids[item] == 487 ? 0x4100
                     : ids[item] == 518 ? 0x5000 : ids[item] == 521 ? 0x5020 : 0x8000;
        BOARD_CHECK(store115(reg, 0) == 0 && read_mem(0x8000) == 0xA7 && ppu_read(0x123) == 0xB8);
        ppu_write(0x123, 0xCC);
        BOARD_CHECK(ppu_read(0x123) == 0xB8 && load_rom_memory(image.data, image.size - 1) < 0);
        BOARD_CHECK(read_mem(0x8000) == 0xA7 && ppu_read(0x123) == 0xB8);
        image.data[7] |= 2;
        BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
        BOARD_CHECK(read_mem(0x8000) == 0xA7 && ppu_read(0x123) == 0xB8);
        board_image_free(&image);
        BOARD_CHECK(board_image_create(&image, ids[item], 0x6000, 0, true));
        image.data[11] = 3;
        BOARD_CHECK(board_image_load(&image) == 0 && store115(reg, 0xFF) == 0);
        ppu_write(0x123, 0x5A);
        BOARD_CHECK(ppu_read(0x1F23) == 0x5A);
        ppu_write(0x1F23, 0xA5);
        BOARD_CHECK(ppu_read(0x123) == 0xA5);
        board_image_free(&image);
    }
    return 0;
}

static int test_332_battery_and_chr_storage(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 332, 0x100000, 0, true));
    image.data[6] |= 2;
    image.data[10] = 0x70;
    image.data[11] = 0x77;
    BOARD_CHECK(board_image_add_trainer(&image, 0x3E));
    char stem[96], path[112], save[112], chr_save[120];
    snprintf(stem, sizeof(stem), "build/discrete115-%lu-%lu", (unsigned long)time(NULL), (unsigned long)clock());
    snprintf(path, sizeof(path), "%s.nes", stem);
    snprintf(save, sizeof(save), "%s.sav", stem);
    snprintf(chr_save, sizeof(chr_save), "%s.chr.sav", stem);
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t written = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(written == image.size && closed == 0 && load_rom(path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu) && read_mem(0x7000) == 0x3E);
    BOARD_CHECK(store115(0x7000, 0xA5) == 0 && store115(0x6001, 1) == 0);
    ppu_write(0x123, 0x6D);
    BOARD_CHECK(store115(0x6000, 0x2A) == 0 && read_mem(0x6000) == 0 && read_mem(0x6001) == 0);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0 && ppu_read(0x123) == 0x6D);
    BOARD_CHECK(read_mem(0x7000) == 0xA5 && unload_rom() && load_rom(path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu) && read_mem(0x7000) == 0xA5 && ppu_read(0x123) == 0);
    BOARD_CHECK(store115(0x6001, 1) == 0 && ppu_read(0x123) == 0x6D && unload_rom());
    file = fopen(save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0 && ftell(file) == 0x2000);
    BOARD_CHECK(fseek(file, 0x1000, SEEK_SET) == 0 && fgetc(file) == 0xA5 && fclose(file) == 0);
    file = fopen(chr_save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0 && ftell(file) == 0x2000);
    BOARD_CHECK(fseek(file, 0x123, SEEK_SET) == 0 && fgetc(file) == 0x6D && fclose(file) == 0);
    BOARD_CHECK(remove(path) == 0 && remove(save) == 0 && remove(chr_save) == 0);
    board_image_free(&image);
    return 0;
}

int test_board_unlicensed_115_accuracy(void) {
    int failures = 0;
    failures += test_331_outer_bank_and_chr_halves();
    failures += test_332_register_lock_and_startup();
    failures += test_336_and_349_prg_modes();
    failures += test_487_register_modes_and_decode();
    failures += test_518_ppu_address_latch_and_open_bus();
    failures += test_519_dip_reads_and_reset();
    failures += test_521_single_register_and_fixed_bank();
    failures += test_529_rom_and_ram_bank_wiring();
    failures += test_529_irq_cpu_and_scanline_modes();
    failures += test_530_scrambled_prg_and_chr_lines();
    failures += test_discrete115_geometry_and_replacement();
    failures += test_332_battery_and_chr_storage();
    unload_rom();
    printf("Discrete cartridge group 115: 12 groups, %d failures\n", failures);
    return failures;
}
