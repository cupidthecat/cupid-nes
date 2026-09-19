/*
 * board_taito_accuracy.c - X1-017 reversed banking and RAM permission tests
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

static int taito_store(uint16_t address, uint8_t value) {
    const uint8_t code[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(code); ++byte)
        write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int test_taito552_banks(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 552, 0x80000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4800, 0xA5);
    BOARD_CHECK(read_mem(0x8000) == 0xA5 && read_mem(0xC000) == 0xA5);
    BOARD_CHECK(read_mem(0xE000) == 126 && read_mem(0xFFFF) == 127);
    BOARD_CHECK(ppu_read(0x1234) == 0x34);
    static const uint8_t inputs[] = {1, 2, 4, 8, 16, 32, 0x19, 0xCA};
    static const uint8_t banks[] = {32, 16, 8, 4, 2, 1, 38, 20};
    for (unsigned slot = 0; slot < 3; ++slot) {
        for (unsigned fixture = 0; fixture < sizeof(inputs); ++fixture) {
            BOARD_CHECK(taito_store((uint16_t)(0x7EFA + slot), inputs[fixture]) == 0);
            BOARD_CHECK(read_mem((uint16_t)(0x8000 + slot * 0x2000)) == banks[fixture] * 2);
            BOARD_CHECK(read_mem((uint16_t)(0x9FFF + slot * 0x2000)) == banks[fixture] * 2 + 1);
            BOARD_CHECK(read_mem(0xE000) == 126);
        }
    }
    const uint8_t chr_regs[] = {3, 9, 17, 25, 33, 41};
    for (unsigned reg = 0; reg < sizeof(chr_regs); ++reg)
        write_mem((uint16_t)(0x7EF0 + reg), chr_regs[reg]);
    const uint8_t normal[] = {2, 3, 8, 9, 17, 25, 33, 41};
    const uint8_t reversed[] = {17, 25, 33, 41, 2, 3, 8, 9};
    for (unsigned slot = 0; slot < sizeof(normal); ++slot)
        BOARD_CHECK(ppu_read((uint16_t)(slot * 0x400)) == normal[slot]);
    write_mem(0x7EF6, 3);
    for (unsigned slot = 0; slot < sizeof(reversed); ++slot)
        BOARD_CHECK(ppu_read((uint16_t)(slot * 0x400)) == reversed[slot]);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    ppu_write(0x2000, 0x73);
    BOARD_CHECK(ppu_read(0x2800) == 0x73);
    write_mem(0x7EF6, 0);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    ppu_write(0x2000, 0x84);
    BOARD_CHECK(ppu_read(0x2400) == 0x84);
    write_mem(0x7EFD, 0xFF);
    write_mem(0x7EEE, 0);
    write_mem(0x7FFA, 0);
    write_mem(0x8000, 0);
    BOARD_CHECK(read_mem(0x8000) == 40 && ppu_read(0) == 2);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 40 && read_mem(0xC000) == 40);
    BOARD_CHECK(ppu_read(0) == 2 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    board_image_free(&image);
    return 0;
}

static int test_taito552_ram(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 552, 0x20000, 0, true));
    image.data[6] |= 2;
    image.data[10] = 0x70;
    BOARD_CHECK(board_image_add_trainer(&image, 0x9A));
    BOARD_CHECK(board_image_load(&image) == 0);
    static const uint16_t addresses[] = {0x6000, 0x6400, 0x6800, 0x6C00, 0x7000};
    for (unsigned index = 0; index < sizeof(addresses) / sizeof(addresses[0]); ++index) {
        write_mem(addresses[index], 0xFF);
        write_mem(0x4800, 0xA5);
        BOARD_CHECK(read_mem(addresses[index]) == 0xA5);
    }
    BOARD_CHECK(taito_store(0x7EF7, 0xCA) == 0);
    BOARD_CHECK(read_mem(0x6000) == 0 && read_mem(0x6400) == 0);
    write_mem(0x6000, 0x31);
    write_mem(0x6400, 0x42);
    write_mem(0x4800, 0xB6);
    BOARD_CHECK(read_mem(0x6800) == 0xB6);
    write_mem(0x7EF8, 0x69);
    write_mem(0x6800, 0x53);
    write_mem(0x6C00, 0x64);
    write_mem(0x7EF9, 0x84);
    BOARD_CHECK(read_mem(0x7000) == 0x9A);
    write_mem(0x7000, 0x75);
    // The permission registers cover 5 KiB. Declared extra RAM retains its
    // initial mapping outside that range, apart from the register addresses.
    write_mem(0x7400, 0x86);
    write_mem(0x7800, 0x97);
    BOARD_CHECK(read_mem(0x7400) == 0x86 && read_mem(0x7800) == 0x97);
    write_mem(0x7EF7, 0xCB);
    write_mem(0x7EF8, 0x68);
    write_mem(0x7EF9, 0x85);
    for (unsigned index = 0; index < sizeof(addresses) / sizeof(addresses[0]); ++index) {
        write_mem(addresses[index], 0xEE);
        write_mem(0x4800, 0xC7);
        BOARD_CHECK(read_mem(addresses[index]) == 0xC7);
    }
    BOARD_CHECK(read_mem(0x7400) == 0x86);
    write_mem(0x7EF7, 0xCA);
    write_mem(0x7EF8, 0x69);
    write_mem(0x7EF9, 0x84);
    for (unsigned index = 0; index < sizeof(addresses) / sizeof(addresses[0]); ++index)
        BOARD_CHECK(read_mem(addresses[index]) == 0x31 + index * 0x11);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x6000) == 0x31 && read_mem(0x7000) == 0x75);
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4800, 0xD8);
    BOARD_CHECK(read_mem(0x6000) == 0xD8);
    write_mem(0x7EF7, 0xCA);
    BOARD_CHECK(read_mem(0x6000) == 0);
    board_image_free(&image);
    return 0;
}

static int test_taito552_geometry(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 552, 0x14000, 0x2800, true));
    image.data[8] |= 0xF0;
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0xE000) == 18);
    write_mem(0x7EFA, 0x3F);
    BOARD_CHECK(read_mem(0x8000) == 6 && read_mem(0x9FFF) == 7);
    write_mem(0x7EF0, 15);
    write_mem(0x7EF1, 0xFF);
    write_mem(0x7EF2, 0xFD);
    BOARD_CHECK(ppu_read(0) == 4 && ppu_read(0x400) == 5);
    BOARD_CHECK(ppu_read(0x800) == 4 && ppu_read(0xC00) == 5 && ppu_read(0x1000) == 3);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x8000) == 6 && ppu_read(0x1000) == 3);
    board_image_set_unsupported_console(&image);
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x8000) == 6 && ppu_read(0x1000) == 3);
    image.data[0] = 0;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x8000) == 6 && ppu_read(0x1000) == 3);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 552, 0x1000, 0x200, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    for (unsigned window = 0; window < 8; ++window)
        BOARD_CHECK(read_mem((uint16_t)(0x8000 + window * 0x1000)) == 0);
    write_mem(0x7EFA, 0xFF);
    BOARD_CHECK(read_mem(0xFFFF) == 0);
    for (unsigned reg = 0; reg < 6; ++reg) write_mem((uint16_t)(0x7EF0 + reg), 0xFF);
    BOARD_CHECK(ppu_read(0x0F23) == 0 && ppu_read(0x1023) == 0x23);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 552, 0x20000, 0, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x6000, 0x65);
    // Without a save chip the save-RAM permission mapping cannot replace work RAM.
    BOARD_CHECK(read_mem(0x6000) == 0x65);
    write_mem(0x7EF7, 0);
    BOARD_CHECK(read_mem(0x6000) == 0x65);
    image.data[10] = 0;
    image.data[11] = 0;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x7EF7, 0xCA);
    write_mem(0x6000, 0x81);
    write_mem(0x4800, 0x92);
    BOARD_CHECK(read_mem(0x6000) == 0x92);
    write_mem(0x7EF0, 0xFF);
    BOARD_CHECK(ppu_read(0x1234) == 0x34);
    board_image_free(&image);
    return 0;
}

static int test_taito552_persistence(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 552, 0x20000, 0, true));
    image.data[6] |= 2;
    image.data[10] = 0x70;
    image.data[11] = 0x70;
    BOARD_CHECK(board_image_add_trainer(&image, 0x9A));
    char path[128], save[128], chr_save[136];
    snprintf(path, sizeof(path), "build/board-taito552-%lu-%lu.nes",
             (unsigned long)time(NULL), (unsigned long)clock());
    memcpy(save, path, strlen(path) + 1);
    strcpy(strrchr(save, '.'), ".sav");
    memcpy(chr_save, path, strlen(path) + 1);
    strcpy(strrchr(chr_save, '.'), ".chr.sav");
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t count = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(count == image.size && closed == 0);
    BOARD_CHECK(load_rom(path) == 0);
    write_mem(0x7EF7, 0xCA);
    write_mem(0x7EF8, 0x69);
    write_mem(0x7EF9, 0x84);
    BOARD_CHECK(read_mem(0x7000) == 0x9A);
    for (unsigned page = 0; page < 5; ++page)
        write_mem((uint16_t)(0x6000 + page * 0x400), (uint8_t)(0x40 + page));
    write_mem(0x7EF6, 0);
    ppu_write(0x0023, 0x71);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(path) == 0);
    write_mem(0x4800, 0xA7);
    BOARD_CHECK(read_mem(0x6000) == 0xA7);
    write_mem(0x7EF7, 0xCA);
    write_mem(0x7EF8, 0x69);
    write_mem(0x7EF9, 0x84);
    for (unsigned page = 0; page < 5; ++page)
        BOARD_CHECK(read_mem((uint16_t)(0x6000 + page * 0x400)) == 0x40 + page);
    write_mem(0x7EF6, 0);
    BOARD_CHECK(ppu_read(0x0023) == 0x71);
    BOARD_CHECK(unload_rom());
    file = fopen(save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0);
    long length = ftell(file);
    closed = fclose(file);
    BOARD_CHECK(length == 0x2000 && closed == 0);
    BOARD_CHECK(remove(save) == 0 && remove(chr_save) == 0 && remove(path) == 0);
    board_image_free(&image);
    return 0;
}

int test_board_taito_accuracy(void) {
    int failures = 0;
    failures += test_taito552_banks();
    failures += test_taito552_ram();
    failures += test_taito552_geometry();
    failures += test_taito552_persistence();
    unload_rom();
    printf("Taito X1-017 accuracy: 4 groups, %d failures\n", failures);
    return failures;
}
