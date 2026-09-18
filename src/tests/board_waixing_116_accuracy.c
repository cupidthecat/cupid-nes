/*
 * board_waixing_116_accuracy.c - Waixing board regressions for issue 116
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

static bool prg16_is(uint16_t address, unsigned bank) {
    return read_mem(address) == (uint8_t)(bank * 4)
        && read_mem((uint16_t)(address + 1)) == (uint8_t)(bank >> 6);
}

static bool prg32_is(unsigned bank) {
    return read_mem(0x8000) == (uint8_t)(bank * 8)
        && read_mem(0x8001) == (uint8_t)(bank >> 5);
}

static bool chr1_is(uint16_t address, unsigned bank) {
    return ppu_read(address) == (uint8_t)bank
        && ppu_read((uint16_t)(address + 1)) == (uint8_t)(bank >> 8);
}

static bool chr8_is(unsigned bank) {
    return ppu_read(0) == (uint8_t)(bank * 8)
        && ppu_read(1) == (uint8_t)(bank >> 5);
}

static int test_162_164(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 162, 0x200000, 0x2000, false));
    BOARD_CHECK(board_image_load(&image) == 0 && prg32_is(3) && chr8_is(0));
    write_mem(0x5000, 0x0A);
    write_mem(0x5200, 1);
    write_mem(0x5300, 5);
    BOARD_CHECK(prg32_is(0x1A));
    write_mem(0x5100, 2);
    write_mem(0x5300, 4);
    BOARD_CHECK(prg32_is(0x1B));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 164, 0x100000, 0x2000, false));
    BOARD_CHECK(board_image_load(&image) == 0 && prg32_is(0x0F));
    write_mem(0x5000, 3);
    BOARD_CHECK(prg32_is(3));
    write_mem(0x5100, 1);
    BOARD_CHECK(prg32_is(0x13));
    board_image_free(&image);
    return 0;
}

static int test_176_fk23c(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 176, 0x100000, 0x100000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x8000, 0x40) && prg8_is(0xA000, 0x41));
    BOARD_CHECK(prg8_is(0xC000, 0x7E) && prg8_is(0xE000, 0x7F));

    write_mem(0x8000, 6);
    write_mem(0x8001, 5);
    BOARD_CHECK(prg8_is(0x8000, 0x45));
    write_mem(0xA000, 1);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);

    write_mem(0xA001, 0x80);
    write_mem(0x6000, 0x5A);
    BOARD_CHECK(read_mem(0x6000) == 0x5A);
    write_mem(0xA001, 0);
    BOARD_CHECK(cart_cpu_read_bus(0x6000, 0xA7) == 0xA7);

    write_mem(0xA001, 0x22);
    write_mem(0x6000, 0x66);
    write_mem(0xA001, 0x23);
    write_mem(0x6000, 0x77);
    write_mem(0xA001, 0x22);
    BOARD_CHECK(read_mem(0x6000) == 0x66);

    write_mem(0xA001, 0);
    write_mem(0x5013, 2);
    write_mem(0x8000, 8);
    write_mem(0x8001, 9);
    BOARD_CHECK(prg8_is(0xC000, 0x49));

    write_mem(0xC000, 1);
    write_mem(0xC001, 0);
    write_mem(0xE001, 0);
    cart_notify_ppu_address(0x0000, 100);
    cart_notify_ppu_address(0x1000, 109);
    cart_notify_ppu_address(0x0000, 200);
    cart_notify_ppu_address(0x1000, 210);
    BOARD_CHECK(!cart_irq_pending());
    cart_notify_ppu_address(0x0000, 300);
    cart_notify_ppu_address(0x1000, 310);
    BOARD_CHECK(!cart_irq_pending());
    cart_clock_cpu_cycle(false);
    BOARD_CHECK(!cart_irq_pending());
    cart_clock_cpu_cycle(false);
    BOARD_CHECK(cart_irq_pending());
    write_mem(0xE000, 0);
    BOARD_CHECK(!cart_irq_pending());

    board_image_free(&image);
    return 0;
}

static int test_178_242(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 178, 0x100000, 0x2000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg16_is(0x8000, 0) && prg16_is(0xC000, 1));
    write_mem(0x4803, 2);
    write_mem(0x6000, 0x5A);
    write_mem(0x4803, 1);
    write_mem(0x6000, 0x66);
    write_mem(0x4803, 2);
    BOARD_CHECK(read_mem(0x6000) == 0x5A);
    write_mem(0x4801, 5);
    write_mem(0x4802, 1);
    write_mem(0x4800, 3);
    BOARD_CHECK(prg16_is(0x8000, 13) && prg16_is(0xC000, 15));
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 242, 0x80000, 0x2000, false));
    BOARD_CHECK(board_image_load(&image) == 0 && prg32_is(0));
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    write_mem(0x80AA, 0);
    BOARD_CHECK(prg32_is(5) && cart_get_mirroring() == MIRROR_HORIZONTAL);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg32_is(0) && cart_get_mirroring() == MIRROR_VERTICAL);
    board_image_free(&image);
    return 0;
}

static int test_252(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 252, 0x40000, 0x40000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8000, 3);
    write_mem(0xA000, 4);
    BOARD_CHECK(prg8_is(0x8000, 3) && prg8_is(0xA000, 4));
    write_mem(0xB000, 5);
    write_mem(0xB004, 2);
    BOARD_CHECK(chr1_is(0, 0x25));
    ppu_write(0, 0xA5);
    BOARD_CHECK(ppu_read(0) == 0xA5);

    write_mem(0xF000, 0x0E);
    write_mem(0xF004, 0x0F);
    write_mem(0xF008, 0x06);
    cart_clock_cpu_cycle(false);
    BOARD_CHECK(!cart_irq_pending());
    cart_clock_cpu_cycle(false);
    BOARD_CHECK(cart_irq_pending());
    write_mem(0xF00C, 0);
    BOARD_CHECK(!cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static int test_253(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 253, 0x40000, 0x40000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8010, 3);
    write_mem(0xA010, 4);
    BOARD_CHECK(prg8_is(0x8000, 3) && prg8_is(0xA000, 4));
    write_mem(0x9400, 3);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_SINGLE1);

    write_mem(0xB000, 4);
    ppu_write(0, 0x5A);
    BOARD_CHECK(ppu_read(0) == 0x5A);
    write_mem(0xB000, 8);
    write_mem(0xB004, 8);
    BOARD_CHECK(chr1_is(0, 0x88));

    write_mem(0xF000, 0x0E);
    write_mem(0xF004, 0x0F);
    write_mem(0xF008, 2);
    for (unsigned i = 0; i < 114; ++i) cart_clock_cpu_cycle(false);
    BOARD_CHECK(!cart_irq_pending());
    for (unsigned i = 0; i < 114; ++i) cart_clock_cpu_cycle(false);
    BOARD_CHECK(cart_irq_pending());
    write_mem(0xF000, 0);
    BOARD_CHECK(!cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static int test_286_and_transactional_load(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 286, 0x20000, 0x10000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cart_set_dip_switches(0));
    write_mem(0xA01A, 0);
    BOARD_CHECK(prg8_is(0x8000, 10));
    write_mem(0x8807, 0);
    BOARD_CHECK(chr1_is(0x1000, 14));
    BOARD_CHECK(cart_set_dip_switches(1));
    write_mem(0xA02B, 0);
    BOARD_CHECK(prg8_is(0x8000, 11));

    uint8_t before = read_mem(0x8000);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x8000) == before);
    cart_set_dip_switches(0);
    board_image_free(&image);
    return 0;
}

int test_board_waixing_116_accuracy(void) {
    int failures = 0;
    failures += test_162_164();
    failures += test_176_fk23c();
    failures += test_178_242();
    failures += test_252();
    failures += test_253();
    failures += test_286_and_transactional_load();
    unload_rom();
    cart_set_dip_switches(0);
    printf("Waixing #116 accuracy: 6 groups, %d failures\n", failures);
    return failures;
}
