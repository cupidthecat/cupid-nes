/*
 * board_mmc3_accuracy.c - MMC3-derived multicart and protection-board tests
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

static bool advance_cpu_cycles(unsigned cycles) {
    if (cycles == 2) {
        write_mem(0x0200, 0xEA);
    } else if (cycles == 3) {
        write_mem(0x0200, 0x4C);
        write_mem(0x0201, 0x00);
        write_mem(0x0202, 0x02);
    } else {
        return false;
    }
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == (int)cycles;
}

static void mmc3_sync_a12(uint64_t tag) {
    cart_notify_ppu_address(0x1000, tag);
}

static int check_mmc3_a12_clock(NesRegion region, uint64_t phase_tag) {
    nes_set_region(region);
    mmc3_sync_a12(phase_tag);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x0000, phase_tag + 1);
    BOARD_CHECK(advance_cpu_cycles(2));
    cart_notify_ppu_address(0x1000, phase_tag + 1000);
    BOARD_CHECK(!cart_irq_pending());
    cart_notify_ppu_address(0x0000, phase_tag + 2);
    BOARD_CHECK(advance_cpu_cycles(3));
    cart_notify_ppu_address(0x1000, phase_tag + 2000);
    BOARD_CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    return 0;
}

static int test_mmc3_12_and_14(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 12, 0x20000, 0x80000, true));
    image.data[8] |= 0xF0;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x8000, 0) && chr1_is(0x0000, 0));
    write_mem(0x4020, 0x11);
    BOARD_CHECK(chr1_is(0x0000, 0x100) && chr1_is(0x1000, 0x104));

    BOARD_CHECK(cart_set_mmc3_revision_name("standard"));
    BOARD_CHECK(check_mmc3_a12_clock(NES_REGION_NTSC, 1) == 0);
    BOARD_CHECK(check_mmc3_a12_clock(NES_REGION_PAL, 2) == 0);
    BOARD_CHECK(check_mmc3_a12_clock(NES_REGION_DENDY, 7) == 0);
    nes_set_region(NES_REGION_NTSC);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 14, 0x40000, 0x80000, true));
    image.data[8] |= 0xD0;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8000, 3);
    write_mem(0xA000, 5);
    write_mem(0x9000, 1);
    write_mem(0xB000, 1);
    write_mem(0xB001, 2);
    BOARD_CHECK(prg8_is(0x8000, 3) && prg8_is(0xA000, 5));
    BOARD_CHECK(chr1_is(0x0000, 0x21) && cart_get_mirroring() == MIRROR_HORIZONTAL);
    write_mem(0xA131, 2);
    write_mem(0x8000, 6);
    write_mem(0x8001, 7);
    BOARD_CHECK(prg8_is(0x8000, 7));
    board_image_free(&image);
    return 0;
}

static int test_mmc3_37_44_47(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 37, 0x80000, 0x40000, true));
    image.data[8] |= 0xB0;
    image.data[10] = 7;
    BOARD_CHECK(board_image_add_trainer(&image, 0x5A));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x7000) == 0x5A);
    write_mem(0x6000, 7);
    BOARD_CHECK(prg8_is(0x8000, 0) && chr1_is(0x0000, 0));
    write_mem(0xA001, 0x80);
    write_mem(0x6000, 7);
    BOARD_CHECK(prg8_is(0x8000, 0x20) && chr1_is(0x0000, 0x80)
                && read_mem(0x7000) == 0x5A);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_is(0x8000, 0) && chr1_is(0x0000, 0));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 44, 0x100000, 0x100000, true));
    image.data[8] |= 0xA0;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0xA001, 6);
    BOARD_CHECK(prg8_is(0x8000, 0x60) && chr1_is(0x0000, 0x300));
    write_mem(0xA001, 7);
    BOARD_CHECK(prg8_is(0x8000, 0x60) && chr1_is(0x0000, 0x300));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 47, 0x40000, 0x40000, true));
    image.data[8] |= 0x90;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0xA001, 0x80);
    write_mem(0x6000, 1);
    BOARD_CHECK(prg8_is(0x8000, 0x10) && chr1_is(0x0000, 0x80));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_is(0x8000, 0) && chr1_is(0x0000, 0));
    board_image_free(&image);
    return 0;
}

static int test_mmc3_45_49_52(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 45, 0x80000, 0x80000, true));
    image.data[8] |= 0x80;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x6000, 0x80);
    write_mem(0x6000, 0x20);
    write_mem(0x6000, 0x0F);
    write_mem(0x6000, 0x00);
    BOARD_CHECK(prg8_is(0x8000, 0x20) && chr1_is(0x0000, 0x80));
    write_mem(0x6000, 0);
    write_mem(0x6000, 0);
    write_mem(0x6000, 0x0F);
    write_mem(0x6000, 0x40);
    unsigned locked_prg = read_mem(0x8000);
    write_mem(0x6000, 0xFF);
    BOARD_CHECK(read_mem(0x8000) == locked_prg);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 49, 0x80000, 0x80000, true));
    image.data[8] |= 0x70;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0xA001, 0x80);
    write_mem(0x6000, 0xB1);
    BOARD_CHECK(prg8_is(0x8000, 0x20) && chr1_is(0x0000, 0x100));
    write_mem(0x6000, 0x60);
    BOARD_CHECK(prg8_is(0x8000, 8) && chr1_is(0x0000, 0x80));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 52, 0x100000, 0x100000, true));
    image.data[8] |= 0x60;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0xA001, 0x80);
    write_mem(0x6000, 0x0D);
    BOARD_CHECK(prg8_is(0x8000, 0x50) && chr1_is(0x0000, 0x200));
    write_mem(0x6000, 0x8D);
    unsigned locked_chr = ppu_read(0x0000);
    write_mem(0x6000, 0x5A);
    BOARD_CHECK(ppu_read(0x0000) == locked_chr && read_mem(0x6000) == 0x5A);
    board_image_free(&image);
    return 0;
}

static int test_mmc3_114_115(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 114, 0x80000, 0x40000, true));
    image.data[8] |= 0x50;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x5000, 0x83);
    BOARD_CHECK(prg8_is(0x8000, 6) && prg8_is(0xC000, 6));
    write_mem(0x5000, 0);
    write_mem(0xA000, 6);
    write_mem(0xC000, 9);
    BOARD_CHECK(chr1_is(0x1000, 9));

    mmc3_sync_a12(11);
    cart_cpu_write(0xA001, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x0000, 12);
    BOARD_CHECK(advance_cpu_cycles(3));
    cart_notify_ppu_address(0x1000, 1111);
    BOARD_CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x0000, 13);
    BOARD_CHECK(advance_cpu_cycles(3));
    cart_notify_ppu_address(0x1000, 2222);
    BOARD_CHECK(!cart_irq_pending());
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 115, 0x80000, 0x80000, true));
    image.data[8] |= 0x40;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x5080, 0xA7);
    BOARD_CHECK(read_mem(0x5000) == 0xA7);
    write_mem(0x4100, 0x84);
    BOARD_CHECK(prg8_is(0x8000, 8) && prg8_is(0xC000, 8));
    write_mem(0x4101, 1);
    BOARD_CHECK(chr1_is(0x0000, 0x100));
    board_image_free(&image);
    return 0;
}

static int test_mmc3_121_123_and_replacement(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 121, 0x80000, 0x100000, true));
    image.data[8] |= 0x30;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x8000, 0));
    write_mem(0x5000, 2);
    BOARD_CHECK(read_mem(0x5000) == 0x42);
    write_mem(0x5180, 0x80);
    BOARD_CHECK(prg8_is(0x8000, 0x20) && chr1_is(0x1000, 0x104));
    write_mem(0x8003, 0x28);
    write_mem(0x8001, 3);
    BOARD_CHECK(prg8_is(0xC000, 0x30));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 123, 0x80000, 0x40000, true));
    image.data[8] |= 0x20;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x5800, 0x45);
    BOARD_CHECK(prg8_is(0x8000, 10) && prg8_is(0xC000, 10));
    write_mem(0x5800, 0);
    write_mem(0x8000, 2);
    write_mem(0x8001, 9);
    BOARD_CHECK(chr1_is(0x0800, 8) && chr1_is(0x0C00, 9));

    write_mem(0x5800, 0x45);
    uint8_t before = read_mem(0x8000);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x8000) == before && prg8_is(0xC000, 10));
    board_image_free(&image);
    return 0;
}

int test_board_mmc3_accuracy(void) {
    int failures = 0;
    failures += test_mmc3_12_and_14();
    failures += test_mmc3_37_44_47();
    failures += test_mmc3_45_49_52();
    failures += test_mmc3_114_115();
    failures += test_mmc3_121_123_and_replacement();
    unload_rom();
    cart_set_mmc3_revision_name("standard");
    printf("MMC3 board accuracy: 5 groups, %d failures\n", failures);
    return failures;
}
