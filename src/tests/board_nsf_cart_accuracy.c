/*
 * board_nsf_cart_accuracy.c - Mapper 31 cartridge bank-window regressions
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

static int test_nsf_cart_windows(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 31, 0x20000, 0x2000, false));
    image.data[6] |= 1;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4800, 0xA5);
    for (unsigned slot = 0; slot < 7; ++slot)
        BOARD_CHECK(read_mem((uint16_t)(0x8000 + slot * 0x1000)) == 0xA5);
    BOARD_CHECK(read_mem(0xF000) == 31 && read_mem(0xFFFF) == 31);
    BOARD_CHECK(ppu_read(0) == 0 && ppu_read(0x1FFF) == 7);
    for (unsigned slot = 0; slot < 8; ++slot) {
        uint8_t bank = (uint8_t)(7 + slot * 3);
        write_mem((uint16_t)(0x5FF8 + slot), bank);
        BOARD_CHECK(read_mem((uint16_t)(0x8000 + slot * 0x1000)) == bank);
        BOARD_CHECK(read_mem((uint16_t)(0x8FFF + slot * 0x1000)) == bank);
    }
    write_mem(0x4FFF, 0);
    write_mem(0x8000, 0);
    write_mem(0x6123, 0x81);
    BOARD_CHECK(read_mem(0x8000) == 7 && read_mem(0xF000) == 28);
    write_mem(0x5003, 31);
    BOARD_CHECK(read_mem(0xB000) == 31 && read_mem(0xA000) == 13);
    write_mem(0x5FFB, 0x2B);
    BOARD_CHECK(read_mem(0xB000) == 11 && read_mem(0xC000) == 19);
    write_mem(0x4800, 0xC6);
    BOARD_CHECK(read_mem(0x5007) == 0xC6);
    const uint8_t program[] = {0xA9, 0x24, 0x8D, 0x04, 0x5A, 0xAD, 0x00, 0xC0, 0x85, 0x10};
    for (unsigned byte = 0; byte < sizeof(program); ++byte)
        write_mem((uint16_t)(0x0200 + byte), program[byte]);
    cpu.pc = 0x0200;
    for (unsigned instruction = 0; instruction < 4; ++instruction)
        BOARD_CHECK(cpu_step(&cpu) > 0);
    BOARD_CHECK(read_mem(0x0010) == 4);
    ppu_write(0x2000, 0x93);
    BOARD_CHECK(ppu_read(0x2800) == 0x93 && cart_get_mirroring() == MIRROR_VERTICAL);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0xC000) == 4 && read_mem(0xB000) == 11 && read_mem(0x6123) == 0x81);
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4800, 0x5E);
    BOARD_CHECK(read_mem(0x8000) == 0x5E && read_mem(0xF000) == 31);
    board_image_free(&image);
    return 0;
}

static int test_nsf_cart_geometry(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 31, 0x200000, 0, true));
    image.data[8] |= 0xF0;
    BOARD_CHECK(board_image_load(&image) == 0);
    // Power-on selects bank 255, even when it is not the last ROM page.
    BOARD_CHECK(read_mem(0xF000) == 0xFF && read_mem(0xF001) == 0);
    write_mem(0x5000, 0xFE);
    BOARD_CHECK(read_mem(0x8000) == 0xFE && read_mem(0x8001) == 0);
    ppu_write(0x1234, 0xD7);
    BOARD_CHECK(ppu_read(0x1234) == 0xD7);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 31, 0xC000, 0x1000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0xF000) == 3);
    write_mem(0x5007, 31);
    BOARD_CHECK(read_mem(0xF000) == 7 && ppu_read(0x0FFF) == 3);
    BOARD_CHECK(ppu_read(0x1234) == 0x34);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 31, 0x2800, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0xF000) == 1);
    write_mem(0x5000, 2);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0x8FFF) == 0);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 31, 0xC00, 0, true));
    image.data[11] = 0;
    for (unsigned byte = 0; byte < 0xC00; ++byte)
        image.data[sizeof(iNESHeader) + byte] = (uint8_t)(byte >> 8);
    BOARD_CHECK(board_image_load(&image) == 0);
    for (unsigned offset = 0; offset < 0x8000; offset += 0x100) {
        write_mem(0x4800, 0xAC);
        uint8_t expected = offset < 0x7800 ? (uint8_t)((offset % 0xC00) >> 8) : 0xAC;
        BOARD_CHECK(read_mem((uint16_t)(0x8000 + offset)) == expected);
    }
    write_mem(0x5003, 0x3F);
    BOARD_CHECK(read_mem(0x8C00) == 0 && read_mem(0x93FF) == 7);
    BOARD_CHECK(ppu_read(0x0123) == 0x23);
    board_image_free(&image);
    return 0;
}

static int test_nsf_cart_replacement(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 31, 0x20000, 0, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x5000, 17);
    write_mem(0x7000, 0x36);
    ppu_write(0x0123, 0x87);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x8000) == 17 && read_mem(0x7000) == 0x36);
    image.data[7] |= 2;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x8000) == 17 && ppu_read(0x0123) == 0x87);
    image.data[0] = 0;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x7000) == 0x36 && ppu_read(0x0123) == 0x87);
    board_image_free(&image);
    return 0;
}

static int test_nsf_cart_save(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 31, 0x20000, 0, false));
    image.data[6] |= 2;
    BOARD_CHECK(board_image_add_trainer(&image, 0x14));
    char path[128], save[128];
    snprintf(path, sizeof(path), "build/board-nsf-cart-%lu-%lu.nes",
             (unsigned long)time(NULL), (unsigned long)clock());
    memcpy(save, path, strlen(path) + 1);
    strcpy(strrchr(save, '.'), ".sav");
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t count = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(count == image.size && closed == 0);
    BOARD_CHECK(load_rom(path) == 0 && read_mem(0x7000) == 0x14);
    write_mem(0x5000, 19);
    write_mem(0x7000, 0x72);
    ppu_write(0x1234, 0x84);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(path) == 0 && read_mem(0x7000) == 0x72);
    BOARD_CHECK(ppu_read(0x1234) == 0);
    write_mem(0x4800, 0x5E);
    BOARD_CHECK(read_mem(0x8000) == 0x5E && read_mem(0xF000) == 31);
    BOARD_CHECK(unload_rom());
    file = fopen(save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0);
    long length = ftell(file);
    closed = fclose(file);
    BOARD_CHECK(length == 0x2000 && closed == 0);
    BOARD_CHECK(remove(save) == 0 && remove(path) == 0);
    board_image_free(&image);
    return 0;
}

int test_board_nsf_cart_accuracy(void) {
    int failures = 0;
    failures += test_nsf_cart_windows();
    failures += test_nsf_cart_geometry();
    failures += test_nsf_cart_replacement();
    failures += test_nsf_cart_save();
    unload_rom();
    printf("NSF cartridge accuracy: 4 groups, %d failures\n", failures);
    return failures;
}
