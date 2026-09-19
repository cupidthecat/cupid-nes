/*
 * board_mmc3_97_accuracy.c - MMC3 variant regressions for mappers 217-262
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

static int test_217_219(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 217, 0x100000, 0x100000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x8000, 0x60) && prg8_is(0xE000, 0x7F));
    write_mem(0x5001, 0x08);
    BOARD_CHECK(prg8_is(0x8000, 0) && prg8_is(0xC000, 0x1E));
    write_mem(0x5007, 0);
    write_mem(0x8000, 6);
    write_mem(0x8001, 5);
    BOARD_CHECK(prg8_is(0x8000, 5));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_is(0x8000, 0x65));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 219, 0x80000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x8000, 0x3C) && prg8_is(0xE000, 0x3F));
    write_mem(0x8002, 0x26);
    write_mem(0x8001, 0x3C);
    BOARD_CHECK(prg8_is(0x8000, 15));
    write_mem(0x8000, 0x08);
    write_mem(0x8001, 2);
    write_mem(0x8000, 0x09);
    write_mem(0x8001, 6);
    BOARD_CHECK(chr1_is(0x0000, 0x22));
    board_image_free(&image);
    return 0;
}

static int test_224_238(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 224, 0x100000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x8000, 0) && prg8_is(0xC000, 0x3E));
    write_mem(0x5000, 4);
    BOARD_CHECK(prg8_is(0x8000, 0x40) && prg8_is(0xC000, 0x7E));
    write_mem(0x8000, 0x46);
    write_mem(0x8001, 3);
    BOARD_CHECK(prg8_is(0xA000, 0x43) && prg8_is(0xC000, 0x41));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 238, 0x40000, 0x20000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4020, 1);
    BOARD_CHECK(cart_cpu_read_bus(0x4020, 0xA5) == 2);
    write_mem(0x7FFF, 3);
    BOARD_CHECK(cart_cpu_read_bus(0x5000, 0x5A) == 3);
    BOARD_CHECK(prg8_is(0x8000, 0));
    board_image_free(&image);
    return 0;
}

static int test_245_249(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 245, 0x100000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    ppu_write(0x0000, 0x31);
    ppu_write(0x1000, 0x42);
    BOARD_CHECK(ppu_read(0x0000) == 0x31 && ppu_read(0x1000) == 0x42);
    write_mem(0x8000, 0x80);
    BOARD_CHECK(ppu_read(0x0000) == 0x42 && ppu_read(0x1000) == 0x31);
    write_mem(0x8000, 0);
    write_mem(0x8001, 2);
    BOARD_CHECK(prg8_is(0x8000, 0x40) && prg8_is(0xE000, 0x7F));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 249, 0x100000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8000, 6);
    write_mem(0x8001, 0x0B);
    BOARD_CHECK(prg8_is(0x8000, 0x0B));
    write_mem(0x5000, 2);
    BOARD_CHECK(prg8_is(0x8000, 0x0D));
    write_mem(0x8000, 2);
    write_mem(0x8001, 0x12);
    BOARD_CHECK(chr1_is(0x1000, 0x42));
    board_image_free(&image);
    return 0;
}

static int test_250_254(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 250, 0x40000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8006, 0);
    write_mem(0x8405, 0);
    BOARD_CHECK(prg8_is(0x8000, 5));
    BOARD_CHECK(cpu_write_abs(0xC003, 0));
    BOARD_CHECK(cpu_write_abs(0xC401, 0));
    BOARD_CHECK(cpu_write_abs(0xE401, 0));
    for (unsigned i = 0; i < 3; ++i) {
        BOARD_CHECK(qualified_edge(100 + i * 10));
        BOARD_CHECK(!cart_irq_pending());
    }
    BOARD_CHECK(qualified_edge(140) && cart_irq_pending());
    BOARD_CHECK(cpu_write_abs(0xE000, 0) && !cart_irq_pending());
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 254, 0x40000, 0x20000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0xA001, 0x80);
    write_mem(0x6000, 0x5A);
    BOARD_CHECK(cart_cpu_read_bus(0x6000, 0) == 0xDA);
    write_mem(0x8000, 6);
    BOARD_CHECK(cart_cpu_read_bus(0x6000, 0) == 0x5A);
    board_image_free(&image);
    return 0;
}

static int test_258_259(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 258, 0x80000, 0x20000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cart_cpu_read_bus(0x5006, 0x80) == 0x8F);
    write_mem(0x5000, 0xA3);
    BOARD_CHECK(prg8_is(0x8000, 4) && prg8_is(0xE000, 7));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_is(0x8000, 0));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 259, 0x40000, 0x20000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x6000, 5);
    BOARD_CHECK(prg8_is(0x8000, 0));
    write_mem(0xA001, 0x80);
    write_mem(0x6000, 5);
    BOARD_CHECK(prg8_is(0x8000, 10) && prg8_is(0xC000, 10));
    write_mem(0x6000, 0x0D);
    BOARD_CHECK(prg8_is(0x8000, 24) && prg8_is(0xC000, 26));
    board_image_free(&image);
    return 0;
}

static int test_260_262_and_geometry(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 260, 0x100000, 0x100000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cart_set_dip_switches(0x0B));
    BOARD_CHECK(cart_cpu_read_bus(0x5000, 0xE0) == 0x0B);
    write_mem(0x5001, 3);
    write_mem(0x5002, 2);
    write_mem(0x5000, 4);
    BOARD_CHECK(prg8_is(0x8000, 6) && prg8_is(0xC000, 6));
    write_mem(0x5003, 0x80);
    write_mem(0x5001, 7);
    BOARD_CHECK(prg8_is(0x8000, 6));
    cpu_soft_reset(&cpu);
    write_mem(0x5001, 7);
    BOARD_CHECK(prg8_is(0x8000, 0));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 262, 0x40000, 0x40000, true));
    image.data[11] = 7;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cart_cpu_read_bus(0x4100, 0xA5) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cart_cpu_read_bus(0x4100, 0xA5) == 0xFF);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cart_cpu_read_bus(0x4100, 0xA5) == 0);
    write_mem(0x4100, 0x40);
    ppu_write(0x0123, 0x77);
    ppu_write(0x1123, 0x66);
    BOARD_CHECK(ppu_read(0x0123) == 0x77 && ppu_read(0x1123) == 0x66);
    board_image_free(&image);

    static const unsigned ids[] = {217,219,224,238,245,249,250,254,258,259,260,262};
    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        BOARD_CHECK(board_image_create(&image, ids[i], 0x6000, 0x2800, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        uint8_t before = read_mem(0x8000);
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
        BOARD_CHECK(read_mem(0x8000) == before);
        board_image_free(&image);
    }
    cart_set_dip_switches(0);
    return 0;
}

int test_board_mmc3_97_accuracy(void) {
    int failures = 0;
    failures += test_217_219();
    failures += test_224_238();
    failures += test_245_249();
    failures += test_250_254();
    failures += test_258_259();
    failures += test_260_262_and_geometry();
    unload_rom();
    printf("MMC3 #97 accuracy: 6 groups, %d failures\n", failures);
    return failures;
}
