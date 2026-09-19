/*
 * board_unlicensed_114_accuracy.c - Multicart address and RAM-source regressions
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

static bool prg16_114(uint16_t address, unsigned bank) {
    return read_mem(address) == (uint8_t)(bank * 4)
        && read_mem((uint16_t)(address + 1)) == (uint8_t)(bank >> 6)
        && read_mem((uint16_t)(address + 0x3FFF)) == (uint8_t)(bank * 4 + 3);
}

static bool prg32_114(unsigned bank) {
    return prg16_114(0x8000, bank * 2) && prg16_114(0xC000, bank * 2 + 1);
}

static bool chr8_114(unsigned bank) {
    return ppu_read(0) == (uint8_t)(bank * 8) && ppu_read(1) == (uint8_t)(bank >> 5)
        && ppu_read(0x1FFF) == (uint8_t)(bank * 8 + 7);
}

static int store114(uint16_t address, uint8_t value) {
    const uint8_t code[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(code); ++byte) write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int mirror114(Mirroring expected) {
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

static int test_274_register_windows_and_reset(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 274, 0x200000, 0x2000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_114(0x8000, 0) && prg16_114(0xC000, 0));
    BOARD_CHECK(store114(0x9FFF, 0x1D) == 0 && prg16_114(0x8000, 1) && mirror114(MIRROR_VERTICAL) == 0);
    BOARD_CHECK(store114(0xBFFF, 0x65) == 0 && prg16_114(0x8000, 1) && prg16_114(0xC000, 101));
    BOARD_CHECK(store114(0xC000, 0x32) == 0 && prg16_114(0x8000, 61) && prg16_114(0xC000, 50));
    BOARD_CHECK(store114(0xFFFF, 0x76) == 0 && prg16_114(0x8000, 125) && prg16_114(0xC000, 118));
    BOARD_CHECK(store114(0x6000, 0x5A) == 0 && read_mem(0x6000) == 0x5A && chr8_114(0));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg16_114(0x8000, 0) && prg16_114(0xC000, 0));
    BOARD_CHECK(read_mem(0x6000) == 0x5A && chr8_114(0) && mirror114(MIRROR_HORIZONTAL) == 0);
    board_image_free(&image);
    return 0;
}

static int test_283_fixed_low_rom(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 283, 0x80000, 0x2000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0 && prg32_114(7));
    BOARD_CHECK(read_mem(0x6000) == 64 && read_mem(0x7FFF) == 65);
    BOARD_CHECK(store114(0x6000, 0xA5) == 0 && store114(0x7FFF, 0x5A) == 0);
    BOARD_CHECK(read_mem(0x6000) == 64 && read_mem(0x7FFF) == 65 && prg32_114(7));
    BOARD_CHECK(store114(0xFFFF, 0xFA) == 0 && prg32_114(2));
    BOARD_CHECK(store114(0x5FFF, 0) == 0 && prg32_114(2) && chr8_114(0));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg32_114(7) && read_mem(0x6000) == 64 && mirror114(MIRROR_HORIZONTAL) == 0);
    board_image_free(&image);
    return 0;
}

static int test_285_banks_and_nametables(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 285, 0x80000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_114(0x8000, 0) && prg16_114(0xC000, 7));
    BOARD_CHECK(mirror114(MIRROR_VERTICAL) == 0);
    ppu_write(0x123, 0x6D);
    BOARD_CHECK(store114(0xFFFF, 0x35) == 0 && prg16_114(0x8000, 29) && prg16_114(0xC000, 31));
    BOARD_CHECK(store114(0x8000, 0x4B) == 0 && prg16_114(0x8000, 10) && prg16_114(0xC000, 11));
    BOARD_CHECK(mirror114(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store114(0xBFFF, 0x82) == 0 && prg16_114(0x8000, 2) && prg16_114(0xC000, 7));
    BOARD_CHECK(mirror114(MIRROR_SINGLE0) == 0);
    BOARD_CHECK(store114(0xD555, 0xA3) == 0 && prg16_114(0x8000, 19) && prg16_114(0xC000, 23));
    BOARD_CHECK(mirror114(MIRROR_SINGLE1) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg16_114(0x8000, 19) && prg16_114(0xC000, 23) && ppu_read(0x123) == 0x6D);
    board_image_free(&image);
    return 0;
}

static int test_288_300_prg_chr_latches(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 288, 0x20000, 0x10000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg32_114(0) && chr8_114(0));
    BOARD_CHECK(store114(0xFFFF, 0) == 0 && prg32_114(3) && chr8_114(7));
    BOARD_CHECK(store114(0xBFFD, 0xFF) == 0 && prg32_114(3) && chr8_114(5));
    BOARD_CHECK(store114(0x800A, 0x80) == 0 && prg32_114(1) && chr8_114(2));
    BOARD_CHECK(store114(0x7FFF, 0) == 0 && prg32_114(1) && chr8_114(2));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg32_114(1) && chr8_114(2) && mirror114(MIRROR_HORIZONTAL) == 0);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 300, 0x20000, 0x10000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_114(0xC000, 0) && chr8_114(0));
    BOARD_CHECK(store114(0xFFFE, 0x15) == 0 && prg16_114(0x8000, 5) && prg16_114(0xC000, 5));
    BOARD_CHECK(chr8_114(5) && mirror114(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store114(0x8000, 0xF7) == 0 && prg16_114(0x8000, 5) && chr8_114(5));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg16_114(0x8000, 5) && chr8_114(5));
    board_image_free(&image);
    return 0;
}

static int test_289_modes_and_ram_overlay(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 289, 0x400000, 0, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_add_trainer(&image, 0x3E));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x7000) == 0x3E);
    BOARD_CHECK(store114(0x7FFF, 0x28) == 0 && prg16_114(0x8000, 40) && prg16_114(0xC000, 40));
    BOARD_CHECK(store114(0xFFFF, 5) == 0 && prg16_114(0x8000, 45) && prg16_114(0xC000, 45));
    BOARD_CHECK(store114(0x7FFE, 9) == 0 && prg16_114(0x8000, 44) && prg16_114(0xC000, 45));
    BOARD_CHECK(mirror114(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store114(0x6000, 10) == 0 && prg16_114(0x8000, 45) && prg16_114(0xC000, 47));
    BOARD_CHECK(store114(0x6000, 11) == 0 && store114(0x8000, 2) == 0 && store114(0x6001, 0x40) == 0);
    BOARD_CHECK(prg16_114(0x8000, 45) && prg16_114(0xC000, 47));
    BOARD_CHECK(read_mem(0x6000) == 0 && read_mem(0x6001) == 0 && read_mem(0x7000) == 0x3E);
    BOARD_CHECK(store114(0x6000, 6) == 0 && prg16_114(0x8000, 64) && prg16_114(0xC000, 71));
    BOARD_CHECK(mirror114(MIRROR_VERTICAL) == 0);
    BOARD_CHECK(store114(0x6000, 0) == 0 && prg16_114(0x8000, 66) && prg16_114(0xC000, 66));
    ppu_write(0x123, 0x6D);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg16_114(0x8000, 66) && prg16_114(0xC000, 66) && ppu_read(0x123) == 0x6D);
    BOARD_CHECK(store114(0x7000, 0xA5) == 0 && read_mem(0x7000) == 0x3E);
    BOARD_CHECK(prg16_114(0x8000, 64) && prg16_114(0xC000, 65));
    board_image_free(&image);
    return 0;
}

static int test_301_dip_and_missing_chip(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 301, 0x80000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && cart_set_dip_switches(0));
    BOARD_CHECK(prg16_114(0x8000, 0) && prg16_114(0xC000, 0));
    BOARD_CHECK(store114(0x8076, 0) == 0 && prg16_114(0x8000, 29) && prg16_114(0xC000, 24));
    BOARD_CHECK(mirror114(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store114(0x80E8, 0xFF) == 0 && prg16_114(0x8000, 26) && prg16_114(0xC000, 26));
    BOARD_CHECK(store114(0x8274, 0) == 0 && prg16_114(0x8000, 29) && prg16_114(0xC000, 31));
    BOARD_CHECK(mirror114(MIRROR_VERTICAL) == 0 && cart_set_dip_switches(1));
    BOARD_CHECK(store114(0x8176, 0) == 0 && cart_cpu_read_bus(0x8000, 0xA5) == 0xA5);
    BOARD_CHECK(cart_cpu_read_bus(0xFFFF, 0x5A) == 0x5A && cart_get_mirroring() == MIRROR_VERTICAL);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cart_cpu_read_bus(0xC000, 0x6D) == 0x6D && chr8_114(0));
    BOARD_CHECK(cart_set_dip_switches(0) && cart_cpu_read_bus(0x8000, 0xA5) == 0xA5);
    BOARD_CHECK(store114(0x8176, 0xFF) == 0 && prg16_114(0x8000, 29) && prg16_114(0xC000, 24));
    BOARD_CHECK(mirror114(MIRROR_HORIZONTAL) == 0);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 301, 0x200000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && cart_set_dip_switches(1));
    BOARD_CHECK(store114(0x8176, 0) == 0 && prg16_114(0x8000, 93) && prg16_114(0xC000, 88));
    BOARD_CHECK(cart_set_dip_switches(0));
    board_image_free(&image);
    return 0;
}

static int test_314_retained_lower_window(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 314, 0x100000, 0x100000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_114(0x8000, 7) && prg16_114(0xC000, 7));
    BOARD_CHECK(store114(0x5001, 0x95) == 0 && prg16_114(0x8000, 42) && prg16_114(0xC000, 43));
    BOARD_CHECK(store114(0x5000, 0x26) == 0 && prg16_114(0x8000, 42) && prg16_114(0xC000, 42));
    BOARD_CHECK(chr8_114(3) && mirror114(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store114(0x5001, 0x43) == 0 && prg16_114(0x8000, 42) && prg16_114(0xC000, 7));
    BOARD_CHECK(store114(0x5002, 0x1B) == 0 && chr8_114(111));
    BOARD_CHECK(store114(0x5003, 0xFF) == 0 && store114(0x8000, 0) == 0 && chr8_114(111));
    const uint16_t ignored[] = {0x4FFF, 0x5004, 0x5100, 0x7FFF};
    for (unsigned i = 0; i < sizeof(ignored) / sizeof(ignored[0]); ++i) {
        BOARD_CHECK(store114(ignored[i], 0x80) == 0);
        BOARD_CHECK(prg16_114(0x8000, 42) && prg16_114(0xC000, 7) && chr8_114(111));
    }
    BOARD_CHECK(cart_cpu_read_bus(0x5000, 0xA5) == 0xA5);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg16_114(0x8000, 7) && prg16_114(0xC000, 7) && chr8_114(0));
    BOARD_CHECK(mirror114(MIRROR_VERTICAL) == 0);
    board_image_free(&image);
    return 0;
}

static int test_319_masks_and_address_decode(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 319, 0x20000, 0x10000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_114(0x8000, 0) && prg16_114(0xC000, 0));
    BOARD_CHECK(store114(0x6000, 0x70) == 0 && chr8_114(7));
    BOARD_CHECK(store114(0x6001, 0x71) == 0 && chr8_114(3));
    BOARD_CHECK(store114(0x7FFB, 0x72) == 0 && chr8_114(5));
    BOARD_CHECK(store114(0xFFF8, 0x73) == 0 && chr8_114(1));
    BOARD_CHECK(store114(0x6004, 0xE8) == 0 && prg16_114(0x8000, 1) && prg16_114(0xC000, 5));
    BOARD_CHECK(mirror114(MIRROR_VERTICAL) == 0 && read_mem(0x6000) == 0 && read_mem(0x6004) == 0);
    BOARD_CHECK(store114(0x9FFC, 0) == 0 && store114(0xDFFF, 0xFF) == 0);
    BOARD_CHECK(prg16_114(0x8000, 1) && prg16_114(0xC000, 5) && chr8_114(1));
    BOARD_CHECK(store114(0x7FFC, 0xB0) == 0 && prg16_114(0x8000, 6) && prg16_114(0xC000, 6));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg16_114(0x8000, 6) && prg16_114(0xC000, 6) && chr8_114(1));
    board_image_free(&image);
    return 0;
}

static int test_320_outer_register_aliases(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 320, 0x200000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_114(0x8000, 0) && prg16_114(0xC000, 15));
    BOARD_CHECK(store114(0x8000, 13) == 0 && prg16_114(0x8000, 13) && prg16_114(0xC000, 15));
    BOARD_CHECK(store114(0xF0E2, 13) == 0 && prg16_114(0x8000, 29) && prg16_114(0xC000, 31));
    BOARD_CHECK(store114(0xF0F2, 13) == 0 && prg16_114(0x8000, 21) && prg16_114(0xC000, 23));
    BOARD_CHECK(store114(0xF0DF, 3) == 0 && prg16_114(0x8000, 19) && prg16_114(0xC000, 23));
    BOARD_CHECK(store114(0xF100, 11) == 0 && prg16_114(0x8000, 19) && prg16_114(0xC000, 23));
    BOARD_CHECK(store114(0xF0E7, 11) == 0 && prg16_114(0x8000, 59) && prg16_114(0xC000, 63));
    ppu_write(0x123, 0x6D);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg16_114(0x8000, 59) && prg16_114(0xC000, 63) && ppu_read(0x123) == 0x6D);
    BOARD_CHECK(mirror114(MIRROR_HORIZONTAL) == 0);
    board_image_free(&image);
    return 0;
}

static int test_328_read_protection_boundaries(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 328, 0x8000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_114(0x8000, 0) && prg16_114(0xC000, 0));
    for (unsigned slot = 0; slot < 4; ++slot)
        BOARD_CHECK(ppu_read((uint16_t)(slot * 0x800)) == 0 && ppu_read((uint16_t)(slot * 0x800 + 0x400)) == 1);
    const uint16_t protected_addresses[] = {0xCE80, 0xCEFF, 0xFE80, 0xFEFF};
    for (unsigned sample = 0; sample < 64; ++sample) {
        for (unsigned reg = 0; reg < 4; ++reg)
            BOARD_CHECK((read_mem(protected_addresses[reg]) & 0xF2) == 0xF2);
        BOARD_CHECK(read_mem(0xCE7F) == 0 && read_mem(0xCF00) == 0);
        BOARD_CHECK(read_mem(0xFE7F) == 3 && read_mem(0xFF00) == 3);
    }
    BOARD_CHECK(store114(0x8000, 0xFF) == 0 && store114(0xCE80, 0) == 0 && prg16_114(0x8000, 0));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0xC000) == 0 && ppu_read(0x1C00) == 1 && mirror114(MIRROR_HORIZONTAL) == 0);
    board_image_free(&image);
    return 0;
}

static int test_329_independent_ram_banks(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 329, 0x100000, 0, true));
    image.data[10] = 9;
    BOARD_CHECK(board_image_load(&image) == 0);
    const uint8_t registers[] = {0x05, 0x49, 0x93, 0xDF};
    const unsigned prg_banks[] = {5, 9, 19, 31};
    for (unsigned bank = 0; bank < 4; ++bank) {
        BOARD_CHECK(store114(0x8000, registers[bank]) == 0 && prg32_114(prg_banks[bank]));
        BOARD_CHECK(read_mem(0x6000) == 0 && read_mem(0x7FFF) == 0);
        BOARD_CHECK(store114(0x6000, (uint8_t)(0xA0 + bank)) == 0);
        BOARD_CHECK(store114(0x7FFF, (uint8_t)(0xB0 + bank)) == 0);
    }
    for (unsigned bank = 0; bank < 4; ++bank) {
        BOARD_CHECK(store114(0xFFFF, registers[bank]) == 0 && prg32_114(prg_banks[bank]));
        BOARD_CHECK(read_mem(0x6000) == 0xA0 + bank && read_mem(0x7FFF) == 0xB0 + bank);
    }
    ppu_write(0x123, 0x6D);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg32_114(31) && read_mem(0x6000) == 0xA3 && read_mem(0x7FFF) == 0xB3);
    BOARD_CHECK(ppu_read(0x123) == 0x6D);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 329, 0x100000, 0, true));
    image.data[10] = 3;
    BOARD_CHECK(board_image_load(&image) == 0 && store114(0x6000, 0x5A) == 0);
    BOARD_CHECK(read_mem(0x7E00) == 0x5A && store114(0x8000, 0xDF) == 0);
    BOARD_CHECK(prg32_114(31) && read_mem(0x6000) == 0x5A && read_mem(0x7E00) == 0x5A);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 329, 0x100000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0 && store114(0xFFFF, 0xC0) == 0);
    BOARD_CHECK(cart_cpu_read_bus(0x6000, 0x5A) == 0x5A && cart_cpu_read_bus(0x7FFF, 0xA5) == 0xA5);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 329, 0x100000, 0, true));
    image.data[6] |= 2;
    image.data[10] = 0x70;
    BOARD_CHECK(board_image_load(&image) == 0 && store114(0x6000, 0x6D) == 0);
    BOARD_CHECK(store114(0xFFFF, 0xC7) == 0 && prg32_114(7) && read_mem(0x6000) == 0x6D);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0 && read_mem(0x6000) == 0x6D);
    board_image_free(&image);
    return 0;
}

static int test_discrete114_geometry_and_replacement(void) {
    const unsigned ids[] = {274, 283, 285, 288, 289, 300, 301, 314, 319, 320, 328, 329};
    for (unsigned item = 0; item < sizeof(ids) / sizeof(ids[0]); ++item) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[item], 0x6000, 0x600, true));
        image.data[8] |= 0xF0;
        memset(image.data + 16, 0xA7, 0x6000);
        memset(image.data + 16 + 0x6000, 0xB8, 0x600);
        BOARD_CHECK(board_image_load(&image) == 0);
        uint16_t reg = ids[item] == 289 || ids[item] == 319 ? 0x6000 : ids[item] == 314 ? 0x5000 : 0x8000;
        BOARD_CHECK(store114(reg, 0xFF) == 0);
        BOARD_CHECK(read_mem(0x8000) == 0xA7 && ppu_read(0x123) == 0xB8);
        ppu_write(0x123, 0xCC);
        BOARD_CHECK(ppu_read(0x123) == 0xB8);
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
        BOARD_CHECK(read_mem(0x8000) == 0xA7 && ppu_read(0x123) == 0xB8);
        board_image_set_unsupported_console(&image);
        BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
        BOARD_CHECK(read_mem(0x8000) == 0xA7 && ppu_read(0x123) == 0xB8);
        board_image_free(&image);
        BOARD_CHECK(board_image_create(&image, ids[item], 0x6000, 0, true));
        image.data[11] = 3;
        BOARD_CHECK(board_image_load(&image) == 0);
        ppu_write(0x123, 0x5A);
        BOARD_CHECK(ppu_read(0x1F23) == 0x5A);
        BOARD_CHECK(store114(reg, 0xFF) == 0);
        ppu_write(0x1F23, 0xA5);
        BOARD_CHECK(ppu_read(0x123) == 0xA5);
        board_image_free(&image);
    }
    return 0;
}

static int test_329_work_and_save_ownership(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 329, 0x100000, 0, true));
    image.data[6] |= 2;
    image.data[10] = 0x79;
    BOARD_CHECK(board_image_add_trainer(&image, 0x3E));
    char stem[96], path[112], save[112];
    snprintf(stem, sizeof(stem), "build/discrete114-%lu-%lu", (unsigned long)time(NULL), (unsigned long)clock());
    snprintf(path, sizeof(path), "%s.nes", stem);
    snprintf(save, sizeof(save), "%s.sav", stem);
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t written = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(written == image.size && closed == 0);
    uint8_t saved[0x2000];
    memset(saved, 0xA5, sizeof(saved));
    file = fopen(save, "wb");
    BOARD_CHECK(file != NULL);
    written = fwrite(saved, 1, sizeof(saved), file);
    closed = fclose(file);
    BOARD_CHECK(written == sizeof(saved) && closed == 0 && load_rom(path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu) && read_mem(0x6000) == 0 && read_mem(0x7000) == 0x3E);
    BOARD_CHECK(store114(0x6000, 0x6D) == 0 && store114(0x7000, 0x7C) == 0);
    BOARD_CHECK(store114(0x8000, 0xC3) == 0 && read_mem(0x6000) == 0 && store114(0x6000, 0x5A) == 0);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0 && read_mem(0x6000) == 0x5A);
    BOARD_CHECK(unload_rom() && load_rom(path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu) && read_mem(0x6000) == 0 && read_mem(0x7000) == 0x3E);
    BOARD_CHECK(store114(0x8000, 0xC3) == 0 && read_mem(0x6000) == 0 && unload_rom());
    file = fopen(save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0 && ftell(file) == 0x2000);
    BOARD_CHECK(fseek(file, 0, SEEK_SET) == 0 && fread(saved, 1, sizeof(saved), file) == sizeof(saved));
    BOARD_CHECK(fclose(file) == 0);
    for (unsigned byte = 0; byte < sizeof(saved); ++byte) BOARD_CHECK(saved[byte] == 0xA5);
    BOARD_CHECK(remove(path) == 0 && remove(save) == 0);
    board_image_free(&image);
    return 0;
}

int test_board_unlicensed_114_accuracy(void) {
    int failures = 0;
    failures += test_274_register_windows_and_reset();
    failures += test_283_fixed_low_rom();
    failures += test_285_banks_and_nametables();
    failures += test_288_300_prg_chr_latches();
    failures += test_289_modes_and_ram_overlay();
    failures += test_301_dip_and_missing_chip();
    failures += test_314_retained_lower_window();
    failures += test_319_masks_and_address_decode();
    failures += test_320_outer_register_aliases();
    failures += test_328_read_protection_boundaries();
    failures += test_329_independent_ram_banks();
    failures += test_discrete114_geometry_and_replacement();
    failures += test_329_work_and_save_ownership();
    unload_rom();
    printf("Discrete cartridge group 114: 13 groups, %d failures\n", failures);
    return failures;
}
