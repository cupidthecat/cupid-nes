/*
 * board_sealie_accuracy.c - Mapper 29 cartridge bus regressions
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

static int test_sealie_banks(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 29, 0x20000, 0, false));
    image.data[6] |= 1;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4800, 0xB5);
    BOARD_CHECK(read_mem(0x8000) == 0xB5 && read_mem(0xBFFF) == 0xB5);
    BOARD_CHECK(read_mem(0xC000) == 28 && read_mem(0xFFFF) == 31);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    write_mem(0x6123, 0x85);
    for (unsigned bank = 0; bank < 4; ++bank) {
        write_mem((uint16_t)(0xFFFF - bank), (uint8_t)bank);
        ppu_write(0x0123, (uint8_t)(0x40 + bank));
        ppu_write(0x1FFF, (uint8_t)(0x70 + bank));
    }
    for (unsigned prg = 0; prg < 8; ++prg) {
        for (unsigned chr = 0; chr < 4; ++chr) {
            write_mem(0x8000, (uint8_t)((prg << 2) | chr | 0xE0));
            BOARD_CHECK(read_mem(0x8000) == prg * 4 && read_mem(0xBFFF) == prg * 4 + 3);
            BOARD_CHECK(read_mem(0xC000) == 28 && read_mem(0xFFFF) == 31);
            BOARD_CHECK(ppu_read(0x0123) == 0x40 + chr && ppu_read(0x1FFF) == 0x70 + chr);
        }
    }
    const uint8_t code[] = {0xA9, 0x16, 0x8D, 0xBC, 0xCA, 0xAD, 0x00, 0x80, 0x85, 0x10};
    for (unsigned byte = 0; byte < sizeof(code); ++byte)
        write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    for (unsigned instruction = 0; instruction < 4; ++instruction) BOARD_CHECK(cpu_step(&cpu) > 0);
    BOARD_CHECK(read_mem(0x0010) == 20 && ppu_read(0x0123) == 0x42);
    write_mem(0x5000, 0);
    write_mem(0x7FFF, 0x98);
    BOARD_CHECK(read_mem(0x8000) == 20 && read_mem(0x6123) == 0x85 && read_mem(0x7FFF) == 0x98);
    ppu_write(0x2000, 0x31);
    ppu_write(0x2400, 0x52);
    BOARD_CHECK(ppu_read(0x2800) == 0x31 && ppu_read(0x2C00) == 0x52);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 20 && ppu_read(0x0123) == 0x42);
    BOARD_CHECK(read_mem(0x6123) == 0x85 && cart_get_mirroring() == MIRROR_VERTICAL);
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4800, 0x5C);
    BOARD_CHECK(read_mem(0x8000) == 0x5C && read_mem(0xC000) == 28);
    BOARD_CHECK(ppu_read(0x0123) == 0 && read_mem(0x6123) == 0);
    board_image_free(&image);
    return 0;
}

static int test_sealie_rom_geometry(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 29, 0x14000, 0x2800, true));
    image.data[8] |= 0xF0;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0xC000) == 16 && ppu_read(0x1234) == 0x34);
    write_mem(0x8000, 0x1F);
    BOARD_CHECK(read_mem(0x8000) == 8 && read_mem(0xFFFF) == 19);
    BOARD_CHECK(ppu_read(0) == 0 && ppu_read(0x1FFF) == 7);
    ppu_write(0x1234, 0xA3);
    BOARD_CHECK(ppu_read(0x1234) == 4);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    ppu_write(0x2000, 0x46);
    BOARD_CHECK(ppu_read(0x2400) == 0x46);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 29, 0x2000, 0x800, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    for (unsigned slot = 0; slot < 4; ++slot)
        BOARD_CHECK(read_mem((uint16_t)(0x8000 + slot * 0x2000)) == 0);
    write_mem(0x8000, 0xFF);
    BOARD_CHECK(read_mem(0xFFFF) == 1 && ppu_read(0x07FF) == 1);
    BOARD_CHECK(ppu_read(0x08C3) == 0xC3);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 29, 0x8000, 0, true));
    image.data[10] = 0;
    image.data[11] = 0;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x7000, 0x99);
    write_mem(0x4800, 0xA8);
    BOARD_CHECK(read_mem(0x7000) == 0xA8);
    write_mem(0x8000, 0xFF);
    ppu_write(0x0123, 0x72);
    BOARD_CHECK(ppu_read(0x0123) == 0x23);
    board_image_free(&image);
    return 0;
}

static int test_sealie_replacement(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 29, 0x20000, 0, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8000, 0x13);
    write_mem(0x7000, 0x46);
    ppu_write(0x1234, 0x57);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x8000) == 16 && read_mem(0x7000) == 0x46);
    board_image_set_unsupported_console(&image);
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x8000) == 16 && ppu_read(0x1234) == 0x57);
    image.data[0] = 0;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x7000) == 0x46 && ppu_read(0x1234) == 0x57);
    board_image_free(&image);
    return 0;
}

static int test_sealie_persistence(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 29, 0x20000, 0, true));
    image.data[6] |= 2;
    image.data[10] = 0x77;
    image.data[11] = 0x90;
    BOARD_CHECK(board_image_add_trainer(&image, 0x3C));
    char path[128], save[128], chr_save[136];
    snprintf(path, sizeof(path), "build/board-sealie-%lu-%lu.nes",
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
    file = fopen(save, "wb");
    BOARD_CHECK(file != NULL);
    uint8_t saved[0x2000];
    memset(saved, 0x5B, sizeof(saved));
    count = fwrite(saved, 1, sizeof(saved), file);
    closed = fclose(file);
    BOARD_CHECK(count == sizeof(saved) && closed == 0);
    BOARD_CHECK(load_rom(path) == 0 && read_mem(0x7000) == 0x3C);
    write_mem(0x7000, 0x8D);
    for (unsigned bank = 0; bank < 4; ++bank) {
        write_mem(0x8000, (uint8_t)bank);
        ppu_write(0x0123, (uint8_t)(0x71 + bank));
    }
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(path) == 0 && read_mem(0x7000) == 0x3C);
    for (unsigned bank = 0; bank < 4; ++bank) {
        write_mem(0x8000, (uint8_t)bank);
        BOARD_CHECK(ppu_read(0x0123) == 0x71 + bank);
    }
    BOARD_CHECK(unload_rom());
    file = fopen(save, "rb");
    BOARD_CHECK(file != NULL);
    count = fread(saved, 1, sizeof(saved), file);
    closed = fclose(file);
    BOARD_CHECK(count == sizeof(saved) && closed == 0);
    for (unsigned byte = 0; byte < sizeof(saved); ++byte) BOARD_CHECK(saved[byte] == 0x5B);
    file = fopen(chr_save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0);
    long length = ftell(file);
    closed = fclose(file);
    BOARD_CHECK(length == 0x8000 && closed == 0);
    BOARD_CHECK(remove(save) == 0 && remove(chr_save) == 0 && remove(path) == 0);
    board_image_free(&image);
    return 0;
}

int test_board_sealie_accuracy(void) {
    int failures = 0;
    failures += test_sealie_banks();
    failures += test_sealie_rom_geometry();
    failures += test_sealie_replacement();
    failures += test_sealie_persistence();
    unload_rom();
    printf("Sealie Computing accuracy: 4 groups, %d failures\n", failures);
    return failures;
}
