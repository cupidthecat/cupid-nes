/*
 * board_jaleco_accuracy.c - Jaleco JF-13 cartridge bus regressions
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

static int test_jf13_registers(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 86, 0x20000, 0x10000, false));
    image.data[6] |= 1;
    BOARD_CHECK(board_image_add_trainer(&image, 0xA3));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 7);
    BOARD_CHECK(ppu_read(0x1234) == 0x34 && read_mem(0x7000) == 0xA3);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    for (unsigned prg = 0; prg < 4; ++prg) {
        for (unsigned chr = 0; chr < 8; ++chr) {
            uint8_t value = (uint8_t)((prg << 4) | (chr & 3) | ((chr & 4) << 4));
            write_mem((uint16_t)(0x6000 | (prg * 0x100) | chr), value);
            BOARD_CHECK(read_mem(0x8000) == prg * 8 && read_mem(0xFFFF) == prg * 8 + 7);
            BOARD_CHECK(ppu_read(0x0000) == chr * 8 && ppu_read(0x1FFF) == chr * 8 + 7);
            write_mem(0x6FFF, (uint8_t)(value | 0x8C));
            BOARD_CHECK(read_mem(0x8000) == prg * 8 && ppu_read(0) == chr * 8);
        }
    }
    write_mem(0x5000, 0);
    write_mem(0x8000, 0);
    write_mem(0x7000, 0);
    write_mem(0x7FFF, 0);
    BOARD_CHECK(read_mem(0x8000) == 24 && ppu_read(0) == 56);
    BOARD_CHECK(read_mem(0x6000) == 0 && read_mem(0x7000) == 0xA3);
    const uint8_t program[] = {0xA9, 0x51, 0x8D, 0xBC, 0x6A, 0xAD, 0x00, 0x80, 0x85, 0x10};
    for (unsigned byte = 0; byte < sizeof(program); ++byte)
        write_mem((uint16_t)(0x0200 + byte), program[byte]);
    cpu.pc = 0x0200;
    for (unsigned instruction = 0; instruction < 4; ++instruction)
        BOARD_CHECK(cpu_step(&cpu) > 0);
    BOARD_CHECK(read_mem(0x0010) == 8 && ppu_read(0) == 40);
    ppu_write(0x2000, 0x96);
    BOARD_CHECK(ppu_read(0x2800) == 0x96);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 8 && ppu_read(0) == 40);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL && read_mem(0x7000) == 0xA3);
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0);
    BOARD_CHECK(ppu_read(0x0123) == 0x23);
    board_image_free(&image);
    return 0;
}

static int test_jf13_geometry(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 86, 0x2000, 0x1000, true));
    image.data[8] |= 0xF0;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x6000, 0x63);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 1);
    BOARD_CHECK(ppu_read(0x0000) == 0 && ppu_read(0x0FFF) == 3);
    BOARD_CHECK(ppu_read(0x1234) == 0x34);
    ppu_write(0x0000, 0xEE);
    BOARD_CHECK(ppu_read(0x0000) == 0);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 86, 0x18000, 0x6000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x6000, 0x73);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 7);
    BOARD_CHECK(ppu_read(0) == 8 && ppu_read(0x1FFF) == 15);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 86, 0x14000, 0x2800, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x6000, 0x33);
    BOARD_CHECK(read_mem(0x8000) == 8 && read_mem(0xFFFF) == 15);
    BOARD_CHECK(ppu_read(0) == 0 && ppu_read(0x1FFF) == 7);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 86, 0x8000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    ppu_write(0x0123, 0x81);
    write_mem(0x6FFF, 0xFF);
    BOARD_CHECK(ppu_read(0x0123) == 0x81);
    image.data[11] = 0;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x6000, 0xFF);
    ppu_write(0x0123, 0xA9);
    BOARD_CHECK(ppu_read(0x0123) == 0x23);
    board_image_free(&image);
    return 0;
}

static int test_jf13_replacement(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 86, 0x20000, 0, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x6000, 0x20);
    ppu_write(0x0123, 0x87);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x8000) == 16 && ppu_read(0x0123) == 0x87);
    image.data[7] |= 2;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x8000) == 16 && ppu_read(0x0123) == 0x87);
    image.data[0] = 0;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x8000) == 16 && ppu_read(0x0123) == 0x87);
    board_image_free(&image);
    return 0;
}

static int test_jf13_trainer_and_save(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 86, 0x20000, 0, false));
    image.data[6] |= 2;
    BOARD_CHECK(board_image_add_trainer(&image, 0xA3));
    char path[128], save[128];
    snprintf(path, sizeof(path), "build/board-jf13-%lu-%lu.nes",
             (unsigned long)time(NULL), (unsigned long)clock());
    memcpy(save, path, strlen(path) + 1);
    strcpy(strrchr(save, '.'), ".sav");
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t count = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(count == image.size && closed == 0);
    file = fopen(save, "wb");
    BOARD_CHECK(file != NULL && fputc(0xB4, file) != EOF && fputc(0xC5, file) != EOF);
    BOARD_CHECK(fclose(file) == 0);
    BOARD_CHECK(load_rom(path) == 0);
    BOARD_CHECK(read_mem(0x6000) == 0xB4 && read_mem(0x6001) == 0xC5);
    BOARD_CHECK(read_mem(0x6002) == 0 && read_mem(0x7000) == 0xA3);
    write_mem(0x6000, 0xFF);
    write_mem(0x7000, 0);
    BOARD_CHECK(read_mem(0x8000) == 24 && read_mem(0x6000) == 0xB4);
    BOARD_CHECK(read_mem(0x7000) == 0xA3);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(path) == 0 && read_mem(0x6000) == 0xB4 && read_mem(0x7000) == 0xA3);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(remove(save) == 0 && remove(path) == 0);
    board_image_free(&image);
    return 0;
}

int test_board_jaleco_accuracy(void) {
    int failures = 0;
    failures += test_jf13_registers();
    failures += test_jf13_geometry();
    failures += test_jf13_replacement();
    failures += test_jf13_trainer_and_save();
    unload_rom();
    printf("Jaleco JF-13 accuracy: 4 groups, %d failures\n", failures);
    return failures;
}
