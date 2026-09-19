/*
 * board_whirlwind_117_accuracy.c - Whirlwind board regressions for issue 117
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

static bool prg4_is(uint16_t address, unsigned bank) {
    return read_mem(address) == (uint8_t)bank
        && read_mem((uint16_t)(address + 1)) == (uint8_t)(bank >> 8);
}

static bool prg8_is(uint16_t address, unsigned bank) {
    return read_mem(address) == (uint8_t)(bank * 2)
        && read_mem((uint16_t)(address + 1)) == (uint8_t)(bank >> 7);
}

static bool chr8_is_page_zero(void) {
    return ppu_read(0x0123) == 0
        && ppu_read(0x1C34) == 7;
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

static bool cpu_nop(void) {
    write_mem(0x0200, 0xEA);
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 2;
}

static bool cpu_nops(unsigned count) {
    for (unsigned i = 0; i < count; ++i) {
        if (!cpu_nop()) return false;
    }
    return true;
}

static int test_mapper40(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 40, 0x10000, 0x2000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x6000, 6));
    BOARD_CHECK(prg8_is(0x8000, 4) && prg8_is(0xA000, 5) && prg8_is(0xE000, 7));
    BOARD_CHECK(chr8_is_page_zero());
    BOARD_CHECK(cart_cpu_read_bus(0xC000, 0xA5) == 0xA5);
    BOARD_CHECK(cpu_write_abs(0xE000, 2) && prg8_is(0xC000, 2));
    BOARD_CHECK(chr8_is_page_zero());

    write_mem(0xA000, 0);
    BOARD_CHECK(cpu_nops(2047));
    BOARD_CHECK(!cart_irq_pending());
    BOARD_CHECK(cpu_nop());
    BOARD_CHECK(cart_irq_pending());
    BOARD_CHECK(cpu_write_abs(0x8000, 0) && !cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static int test_lh32_125(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 125, 0x20000, 0x2000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x6000, 0));
    BOARD_CHECK(prg8_is(0x8000, 12) && prg8_is(0xA000, 13) && prg8_is(0xE000, 15));
    write_mem(0xC000, 0x5A);
    BOARD_CHECK(read_mem(0xC000) == 0x5A);
    BOARD_CHECK(cpu_write_abs(0x6000, 3) && prg8_is(0x6000, 3));
    BOARD_CHECK(read_mem(0xC000) == 0x5A);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_is(0x6000, 3) && read_mem(0xC000) == 0x5A);
    board_image_free(&image);
    return 0;
}

static int test_smb2j_304(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 304, 0x18000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg4_is(0x5000, 21) && prg4_is(0x6000, 22) && prg4_is(0x7000, 23));
    BOARD_CHECK(prg4_is(0x8000, 0) && prg4_is(0xC000, 4));
    BOARD_CHECK(chr8_is_page_zero());
    BOARD_CHECK(cpu_write_abs(0x4022, 1));
    BOARD_CHECK(prg4_is(0x8000, 4) && prg4_is(0xC000, 8));
    BOARD_CHECK(chr8_is_page_zero());

    write_mem(0x4122, 1);
    BOARD_CHECK(cpu_nops(2047));
    BOARD_CHECK(!cart_irq_pending());
    BOARD_CHECK(cpu_nop());
    BOARD_CHECK(cart_irq_pending());
    BOARD_CHECK(cpu_write_abs(0x4122, 0) && !cart_irq_pending());
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 304, 0x8000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg4_is(0x8000, 0));
    write_mem(0x4022, 1);
    BOARD_CHECK(prg4_is(0x8000, 0));
    board_image_free(&image);
    return 0;
}

static int test_lh51_309(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 309, 0x20000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x8000, 0) && prg8_is(0xA000, 13)
                && prg8_is(0xC000, 14) && prg8_is(0xE000, 15));
    write_mem(0x8000, 5);
    BOARD_CHECK(prg8_is(0x8000, 5));
    write_mem(0xE000, 8);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    write_mem(0xE000, 0);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    board_image_free(&image);
    return 0;
}

static int test_lh10_522_and_transactional_load(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 522, 0x20000, 0x2000, true));
    ((iNESHeader *)image.data)->flags10 = 7;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x6000, 14));
    BOARD_CHECK(prg8_is(0x8000, 0) && prg8_is(0xA000, 0) && prg8_is(0xE000, 15));
    write_mem(0xC000, 0x5A);
    write_mem(0xDFFF, 0xA5);
    BOARD_CHECK(read_mem(0xC000) == 0x5A && read_mem(0xDFFF) == 0xA5);

    write_mem(0x8000, 6);
    write_mem(0x8001, 3);
    write_mem(0x8000, 7);
    write_mem(0x8001, 4);
    BOARD_CHECK(prg8_is(0x8000, 3) && prg8_is(0xA000, 4));
    BOARD_CHECK(read_mem(0xC000) == 0x5A && read_mem(0xDFFF) == 0xA5);
    BOARD_CHECK(chr8_is_page_zero());

    uint8_t before = read_mem(0x8000);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x8000) == before && read_mem(0xC000) == 0x5A);
    board_image_free(&image);
    return 0;
}

int test_board_whirlwind_117_accuracy(void) {
    int failures = 0;
    failures += test_mapper40();
    failures += test_lh32_125();
    failures += test_smb2j_304();
    failures += test_lh51_309();
    failures += test_lh10_522_and_transactional_load();
    unload_rom();
    printf("Whirlwind #117 accuracy: 5 groups, %d failures\n", failures);
    return failures;
}
