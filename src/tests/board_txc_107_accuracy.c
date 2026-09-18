/*
 * board_txc_107_accuracy.c - TXC cartridge regressions
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
#include <time.h>

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
    return ppu_read(0x0000) == (uint8_t)(bank * 8)
        && ppu_read(0x0001) == (uint8_t)(bank >> 5);
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

static bool write_image_file(const char *path, const BoardImage *image) {
    FILE *file = fopen(path, "wb");
    if (!file) return false;
    size_t written = fwrite(image->data, 1, image->size, file);
    return fclose(file) == 0 && written == image->size;
}

static int test_36_61(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 36, 0x100000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg32_is(0) && chr8_is(0));
    write_mem(0x4102, 0x20);
    write_mem(0x4100, 0);
    write_mem(0x8000, 0);
    BOARD_CHECK(prg32_is(2));
    write_mem(0x4200, 5);
    BOARD_CHECK(chr8_is(5));
    BOARD_CHECK(cart_cpu_read_bus(0x4100, 0xCF) == 0xEF);
    BOARD_CHECK(cart_cpu_read_bus(0x4101, 0xA5) == 0xA5);
    BOARD_CHECK(prg32_is(2));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg32_is(2) && chr8_is(5));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 61, 0x80000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg16_is(0x8000, 0) && prg16_is(0xC000, 1));
    write_mem(0x80A5, 0);
    BOARD_CHECK(prg16_is(0x8000, 10) && prg16_is(0xC000, 11));
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    write_mem(0x80B5, 0);
    BOARD_CHECK(prg16_is(0x8000, 11) && prg16_is(0xC000, 11));
    write_mem(0x8012, 0);
    BOARD_CHECK(prg16_is(0x8000, 4) && prg16_is(0xC000, 4));
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL && chr8_is(0));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg16_is(0x8000, 4));
    board_image_free(&image);
    return 0;
}

static int test_132_172(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 132, 0x10000, 0x10000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4102, 4);
    write_mem(0x4100, 0);
    write_mem(0x8000, 0);
    BOARD_CHECK(prg32_is(1) && chr8_is(0));
    BOARD_CHECK(cart_cpu_read_bus(0x4100, 0xF0) == 0xF4);
    BOARD_CHECK(cart_cpu_read_bus(0x4020, 0xA5) == 0xA5);
    BOARD_CHECK(read_mem(0x8000) == 8);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg32_is(1));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 172, 0x8000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4102, 0x12);
    write_mem(0x4100, 0);
    write_mem(0x8000, 0);
    BOARD_CHECK(chr8_is(0x1D));
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    BOARD_CHECK(cart_cpu_read_bus(0x4100, 0xC0) == 0xED);
    BOARD_CHECK(cart_cpu_read_bus(0x4101, 0xA5) == 0xA5);
    write_mem(0x4101, 0);
    write_mem(0x8000, 0);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL && chr8_is(0x1D));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(chr8_is(0x1D) && cart_get_mirroring() == MIRROR_HORIZONTAL);
    board_image_free(&image);
    return 0;
}

static int test_173(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 173, 0x8000, 0x10000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4102, 3);
    write_mem(0x4100, 0);
    write_mem(0x8000, 0);
    BOARD_CHECK(prg32_is(0) && chr8_is(7));
    write_mem(0x4101, 1);
    BOARD_CHECK(chr8_is(5));
    BOARD_CHECK(cart_cpu_read_bus(0x4100, 0xF0) == 0xFB);
    BOARD_CHECK(chr8_is(7));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 173, 0x8000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(chr8_is(0));
    write_mem(0x4101, 1);
    BOARD_CHECK(ppu_read(0x0123) == 0x23);
    (void)cart_cpu_read_bus(0x4100, 0xF0);
    BOARD_CHECK(chr8_is(0));
    board_image_free(&image);
    return 0;
}

static int test_189(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 189, 0x40000, 0x40000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg8_is(0x8000, 0) && prg8_is(0xE000, 3));
    write_mem(0x4120, 0x52);
    BOARD_CHECK(prg8_is(0x8000, 28) && prg8_is(0xE000, 31));
    write_mem(0x8000, 2);
    write_mem(0x8001, 9);
    BOARD_CHECK(chr1_is(0x1000, 9));
    BOARD_CHECK(prg8_is(0x8000, 28));
    write_mem(0xA001, 0x80);
    BOARD_CHECK(cart_cpu_read_bus(0x6000, 0xA5) == 0);
    write_mem(0x6000, 0x11);
    BOARD_CHECK(prg8_is(0x8000, 4) && cart_cpu_read_bus(0x6000, 0xA5) == 0);
    BOARD_CHECK(cpu_write_abs(0xC000, 0));
    BOARD_CHECK(cpu_write_abs(0xC001, 0));
    BOARD_CHECK(cpu_write_abs(0xE001, 0));
    BOARD_CHECK(qualified_edge(100) && cart_irq_pending());
    BOARD_CHECK(cpu_write_abs(0xE000, 0) && !cart_irq_pending());
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg8_is(0x8000, 4));
    board_image_free(&image);
    return 0;
}

static int test_189_persistence(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 189, 0x40000, 0x40000, true));
    image.data[6] |= 2;
    image.data[10] = 0x70;
    char path[160], save[160];
    snprintf(path, sizeof(path), "build/board-txc-189-%lu-%lu.nes",
             (unsigned long)time(NULL), (unsigned long)clock());
    memcpy(save, path, strlen(path) + 1);
    strcpy(strrchr(save, '.'), ".sav");
    BOARD_CHECK(write_image_file(path, &image));
    uint8_t seed[0x2000] = {0};
    seed[0] = 0x5A;
    FILE *file = fopen(save, "wb");
    BOARD_CHECK(file != NULL);
    BOARD_CHECK(fwrite(seed, 1, sizeof(seed), file) == sizeof(seed));
    BOARD_CHECK(fclose(file) == 0);
    BOARD_CHECK(load_rom(path) == 0);
    BOARD_CHECK(cpu_power_on(&cpu));
    write_mem(0xA001, 0x80);
    BOARD_CHECK(read_mem(0x6000) == 0x5A);
    write_mem(0x6000, 0x23);
    BOARD_CHECK(read_mem(0x6000) == 0x5A && prg8_is(0x8000, 12));
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(path) == 0);
    BOARD_CHECK(cpu_power_on(&cpu));
    write_mem(0xA001, 0x80);
    BOARD_CHECK(read_mem(0x6000) == 0x5A);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(remove(save) == 0 && remove(path) == 0);
    board_image_free(&image);
    return 0;
}

static int test_299(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 299, 0x100000, 0x100000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg32_is(0) && chr8_is(0));
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    write_mem(0x8000, 0xB2);
    BOARD_CHECK(prg32_is(3) && chr8_is(14));
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg32_is(0) && chr8_is(0));
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    board_image_free(&image);
    return 0;
}

static int test_geometry_and_transaction(void) {
    static const unsigned ids[] = {36,61,132,172,173,189,299};
    BoardImage image;
    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        BOARD_CHECK(board_image_create(&image, ids[i], 0x6000, 0x2800, true));
        image.data[8] |= 0x50;
        image.data[10] = 7;
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0x0000) == 0);
        if (ids[i] != 189) {
            uint8_t ram_value = (uint8_t)(0x40 + i);
            write_mem(0x6000, ram_value);
            BOARD_CHECK(read_mem(0x6000) == ram_value);
        }
        uint8_t before = read_mem(0x8000);
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
        BOARD_CHECK(read_mem(0x8000) == before);
        board_image_free(&image);
    }
    return 0;
}

int test_board_txc_107_accuracy(void) {
    int failures = 0;
    failures += test_36_61();
    failures += test_132_172();
    failures += test_173();
    failures += test_189();
    failures += test_189_persistence();
    failures += test_299();
    failures += test_geometry_and_transaction();
    unload_rom();
    printf("TXC #107 accuracy: 7 groups, %d failures\n", failures);
    return failures;
}
