/*
 * board_mmc3_96_accuracy.c - MMC3 variant regressions for mappers 126-215
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

static int test_126_134(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 126, 0x200000, 0x200000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x8000, 0));
    write_mem(0x6000, 0x02);
    BOARD_CHECK(prg8_is(0x8000, 0x20));
    write_mem(0x6003, 0x83);
    BOARD_CHECK(prg8_is(0x8000, 0x20) && prg8_is(0xA000, 0x21)
                && prg8_is(0xC000, 0x22) && prg8_is(0xE000, 0x23));
    write_mem(0x6000, 0x04);
    BOARD_CHECK(prg8_is(0x8000, 0x20));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 134, 0x80000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x6001, 0x22);
    BOARD_CHECK(prg8_is(0x8000, 0x20) && chr1_is(0x0000, 0x100));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_is(0x8000, 0) && chr1_is(0x0000, 0));
    board_image_free(&image);
    return 0;
}

static int test_165_182(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 165, 0x40000, 0x40000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    ppu_write(0x0000, 0xA5);
    BOARD_CHECK(ppu_read(0x0000) == 0xA5 && chr1_is(0x1000, 4));
    write_mem(0x8000, 0);
    write_mem(0x8001, 8);
    BOARD_CHECK(chr1_is(0x0000, 8));
    write_mem(0x8000, 1);
    write_mem(0x8001, 12);
    cart_notify_ppu_address(0x0FE8, 1);
    cart_notify_ppu_address(0x0000, 2);
    BOARD_CHECK(chr1_is(0x0000, 12));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 182, 0x40000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8001, 1);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    write_mem(0xA000, 6);
    write_mem(0xC000, 9);
    BOARD_CHECK(chr1_is(0x1000, 9));
    board_image_free(&image);
    return 0;
}

static int test_187_196(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 187, 0x100000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x5000) == 0x83);
    write_mem(0x8001, 5);
    BOARD_CHECK(prg8_is(0x8000, 0));
    write_mem(0x8000, 6);
    write_mem(0x8001, 5);
    BOARD_CHECK(prg8_is(0x8000, 5));
    write_mem(0x5000, 0x83);
    BOARD_CHECK(prg8_is(0x8000, 6) && prg8_is(0xC000, 6));
    write_mem(0x8001, 5);
    BOARD_CHECK(prg8_is(0x8000, 6));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 196, 0x80000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x6000, 0x21);
    BOARD_CHECK(prg8_is(0x8000, 12) && prg8_is(0xA000, 13)
                && prg8_is(0xC000, 14) && prg8_is(0xE000, 15));
    board_image_free(&image);
    return 0;
}

static int test_197_198(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 197, 0x40000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8000, 0);
    write_mem(0x8001, 4);
    write_mem(0x8000, 2);
    write_mem(0x8001, 6);
    write_mem(0x8000, 3);
    write_mem(0x8001, 8);
    BOARD_CHECK(chr1_is(0x0000, 8) && chr1_is(0x0C00, 11));
    BOARD_CHECK(chr1_is(0x1000, 12) && chr1_is(0x1800, 16));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 198, 0x100000, 0, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x8000, 0) && prg8_is(0xA000, 1)
                && prg8_is(0xC000, 0x7E) && prg8_is(0xE000, 0x7F));
    write_mem(0x5000, 0xA5);
    BOARD_CHECK(read_mem(0x5000) == 0xA5 && read_mem(0x6000) == 0xA5
                && read_mem(0x7000) == 0xA5);
    write_mem(0x8000, 6);
    write_mem(0x8001, 0x45);
    BOARD_CHECK(prg8_is(0x8000, 0x45));
    board_image_free(&image);
    return 0;
}

static int test_199_205(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 199, 0x100000, 0x80000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    ppu_write(0x0000, 0x5A);
    BOARD_CHECK(ppu_read(0x0000) == 0x5A);
    write_mem(0xA000, 2);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    write_mem(0xA000, 3);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    write_mem(0x8000, 0x08);
    write_mem(0x8001, 4);
    BOARD_CHECK(prg8_is(0xC000, 4));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 205, 0x100000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x6000, 3);
    BOARD_CHECK(prg8_is(0x8000, 0x30) && chr1_is(0x0000, 0x180));
    board_image_free(&image);
    return 0;
}

static int test_208_215_and_replacement(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 208, 0x80000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x8000, 12) && prg8_is(0xE000, 15));
    write_mem(0x6800, 0x10);
    BOARD_CHECK(prg8_is(0x8000, 8) && prg8_is(0xE000, 11));
    write_mem(0x5000, 0);
    write_mem(0x5801, 0);
    BOARD_CHECK(read_mem(0x5801) == 0x59);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 215, 0x100000, 0x100000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x8000, 0x60));
    write_mem(0x5001, 1);
    BOARD_CHECK(prg8_is(0x8000, 0x20));
    write_mem(0x5000, 0x80);
    BOARD_CHECK(prg8_is(0x8000, 0x20) && prg8_is(0xC000, 0x20));
    write_mem(0x5000, 0xA0);
    BOARD_CHECK(prg8_is(0x8000, 0x20) && prg8_is(0xC000, 0x22));

    uint8_t before = read_mem(0xC000);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0xC000) == before);
    board_image_free(&image);
    return 0;
}

int test_board_mmc3_96_accuracy(void) {
    int failures = 0;
    failures += test_126_134();
    failures += test_165_182();
    failures += test_187_196();
    failures += test_197_198();
    failures += test_199_205();
    failures += test_208_215_and_replacement();
    unload_rom();
    printf("MMC3 #96 accuracy: 6 groups, %d failures\n", failures);
    return failures;
}
