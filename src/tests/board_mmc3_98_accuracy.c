/*
 * board_mmc3_98_accuracy.c - MMC3 variant regressions for mappers 263-366
 *
 * Copyright (C) 2026 Francis Hagan
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#include "board_tests.h"

static bool prg8_is(uint16_t address, unsigned bank) {
    return read_mem(address) == (uint8_t)(bank * 2)
        && read_mem((uint16_t)(address + 1)) == (uint8_t)(bank >> 7);
}

static bool chr1_is(uint16_t address, unsigned bank) {
    return ppu_read(address) == (uint8_t)bank
        && ppu_read((uint16_t)(address + 1)) == (uint8_t)(bank >> 8);
}

static bool cpu_write_abs(uint16_t address, uint8_t value) {
    write_mem(0x0200, 0xA9);
    write_mem(0x0201, value);
    write_mem(0x0202, 0x8D);
    write_mem(0x0203, (uint8_t)address);
    write_mem(0x0204, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4;
}

static bool advance_three_cpu_cycles(void) {
    write_mem(0x0200, 0x4C);
    write_mem(0x0201, 0x00);
    write_mem(0x0202, 0x02);
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 3;
}

static bool qualified_edge(uint64_t tag) {
    cart_notify_ppu_address(0x1000, tag);
    cart_notify_ppu_address(0x0000, tag + 1);
    if (!advance_three_cpu_cycles()) return false;
    cart_notify_ppu_address(0x1000, tag + 2);
    return true;
}

static int test_263_268(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 263, 0x80000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cpu_write_abs(0x8000, 0x21));
    BOARD_CHECK(cpu_write_abs(0x9000, 0x03));
    BOARD_CHECK(prg8_is(0x8000, 5));
    BOARD_CHECK(cpu_write_abs(0xA000, 0x02));
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    BOARD_CHECK(cpu_write_abs(0xC000, 0));
    BOARD_CHECK(cpu_write_abs(0xD000, 0));
    BOARD_CHECK(cpu_write_abs(0xF000, 0));
    BOARD_CHECK(qualified_edge(100) && cart_irq_pending());
    BOARD_CHECK(cpu_write_abs(0xE000, 0) && !cart_irq_pending());
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 268, 0x100000, 0x80000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x6000, 0x41);
    BOARD_CHECK(prg8_is(0x8000, 0x10));
    write_mem(0xA001, 0x80);
    write_mem(0x6003, 0x80);
    write_mem(0x6000, 0x5A);
    BOARD_CHECK(cart_cpu_read_bus(0x6000, 0) == 0x5A);
    BOARD_CHECK(prg8_is(0x8000, 0x10));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_is(0x8000, 0));
    write_mem(0x6002, 3);
    write_mem(0x6003, 0x10);
    BOARD_CHECK(chr1_is(0x0000, 24));
    board_image_free(&image);
    return 0;
}

static int test_287_292(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 287, 0x80000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cart_set_dip_switches(0));
    write_mem(0x6003, 0);
    BOARD_CHECK(prg8_is(0x8000, 0x30));
    BOARD_CHECK(chr1_is(0x0000, 0x180));
    BOARD_CHECK(cart_set_dip_switches(1));
    write_mem(0x6004, 0);
    BOARD_CHECK(prg8_is(0x8000, 0x30) && prg8_is(0xE000, 0x33));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_is(0x8000, 0x30));
    board_image_free(&image);
    BOARD_CHECK(cart_set_dip_switches(0));

    BOARD_CHECK(board_image_create(&image, 292, 0x80000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x006A, 3);
    write_mem(0x00FF, 0x42);
    write_mem(0x6000, 0xC5);
    BOARD_CHECK(prg8_is(0x8000, 5));
    BOARD_CHECK(cart_cpu_read_bus(0x6000, 0xA5) == 0);
    BOARD_CHECK(chr1_is(0x0000, 6));
    BOARD_CHECK(cart_cpu_read_bus(0x8000, 0xA5) == 10);
    write_mem(0x6000, 0);
    BOARD_CHECK(cart_cpu_read_bus(0x6000, 0xA5) == 0);
    BOARD_CHECK(chr1_is(0x1000, 8));
    BOARD_CHECK(cart_cpu_read_bus(0x8000, 0xA5) == 0);
    board_image_free(&image);
    return 0;
}

static int test_313_325(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 313, 0x80000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x8000, 0) && chr1_is(0x0000, 0));
    for (unsigned block = 1; block < 4; ++block) {
        cpu_soft_reset(&cpu);
        BOARD_CHECK(prg8_is(0x8000, block << 4));
        BOARD_CHECK(chr1_is(0x0000, block << 7));
    }
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_is(0x8000, 0) && chr1_is(0x0000, 0));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 325, 0x40000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8000, 6);
    write_mem(0x8008, 4);
    BOARD_CHECK(prg8_is(0x8000, 8));
    write_mem(0x8000, 2);
    write_mem(0x8008, 2);
    BOARD_CHECK(chr1_is(0x1000, 0x20));
    write_mem(0xA000, 1);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    BOARD_CHECK(cpu_write_abs(0xC000, 0));
    BOARD_CHECK(cpu_write_abs(0xC004, 0));
    BOARD_CHECK(cpu_write_abs(0xE004, 0));
    BOARD_CHECK(qualified_edge(200) && cart_irq_pending());
    BOARD_CHECK(cpu_write_abs(0xE000, 0) && !cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static int test_333_348(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 333, 0x100000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x8000, 0) && prg8_is(0xE000, 3));
    write_mem(0x9000, 0x1C);
    BOARD_CHECK(prg8_is(0x8000, 0x30) && prg8_is(0xE000, 0x3F));
    BOARD_CHECK(chr1_is(0x0000, 0x180));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_is(0x8000, 0x30));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 348, 0x100000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x6800, 4);
    BOARD_CHECK(prg8_is(0x8000, 0x10) && chr1_is(0x0000, 0x80));
    write_mem(0x6800, 0x0C);
    BOARD_CHECK(prg8_is(0x8000, 0x30) && prg8_is(0xC000, 0x32)
                && prg8_is(0xE000, 0x33));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_is(0x8000, 0x30));
    write_mem(0x8000, 6);
    BOARD_CHECK(prg8_is(0x8000, 0));
    board_image_free(&image);
    return 0;
}

static int test_366_and_shared_irq(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 366, 0x100000, 0x80000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0xA001, 0x80);
    write_mem(0x7000, 0x20);
    BOARD_CHECK(prg8_is(0x8000, 0x20));
    write_mem(0x60B0, 0);
    BOARD_CHECK(prg8_is(0x8000, 0x30) && chr1_is(0x0000, 0x180));
    write_mem(0x6000, 0x5A);
    BOARD_CHECK(cart_cpu_read_bus(0x6000, 0) == 0x5A);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_is(0x8000, 0));
    board_image_free(&image);

    static const unsigned ids[] = {268, 287, 292, 313, 333, 348, 366};
    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        BOARD_CHECK(board_image_create(&image, ids[i], 0x40000, 0x40000, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(cpu_write_abs(0xC000, 0));
        BOARD_CHECK(cpu_write_abs(0xC001, 0));
        BOARD_CHECK(cpu_write_abs(0xE001, 0));
        BOARD_CHECK(qualified_edge(300 + i * 10) && cart_irq_pending());
        BOARD_CHECK(cpu_write_abs(0xE000, 0) && !cart_irq_pending());
        board_image_free(&image);
    }
    return 0;
}

static int test_geometry_and_transaction(void) {
    static const unsigned ids[] = {263,268,287,292,313,325,333,348,366};
    BoardImage image;
    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        BOARD_CHECK(board_image_create(&image, ids[i], 0x6000, 0x2800, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        uint8_t before = read_mem(0x8000);
        BOARD_CHECK(ppu_read(0x0000) <= 9);
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
        BOARD_CHECK(read_mem(0x8000) == before);
        board_image_free(&image);
    }
    return 0;
}

int test_board_mmc3_98_accuracy(void) {
    int failures = 0;
    failures += test_263_268();
    failures += test_287_292();
    failures += test_313_325();
    failures += test_333_348();
    failures += test_366_and_shared_irq();
    failures += test_geometry_and_transaction();
    unload_rom();
    printf("MMC3 #98 accuracy: 6 groups, %d failures\n", failures);
    return failures;
}
