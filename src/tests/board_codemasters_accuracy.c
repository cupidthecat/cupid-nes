/*
 * board_codemasters_accuracy.c - Golden Five loader and cartridge bus tests
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

static int test_golden_five_registers(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 104, 0x200000, 0, false));
    image.data[6] |= 1;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x5000, 0xA6);
    BOARD_CHECK(read_mem(0x8000) == 0xA6 && read_mem(0xBFFF) == 0xA6);
    BOARD_CHECK(read_mem(0xC000) == 60 && read_mem(0xFFFF) == 63);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    write_mem(0x6123, 0x93);
    ppu_write(0x0123, 0x4A);
    ppu_write(0x2000, 0x27);
    ppu_write(0x2400, 0x86);
    BOARD_CHECK(ppu_read(0x2800) == 0x27 && ppu_read(0x2C00) == 0x86);

    // D3 enables outer writes. The middle 8KB window does not decode them.
    write_mem(0x8000, 7);
    BOARD_CHECK(read_mem(0x8000) == 7 && read_mem(0xC000) == 60);
    write_mem(0xA000, 0x0F);
    BOARD_CHECK(read_mem(0x8000) == 0x0F && read_mem(0xC000) == 60);
    write_mem(0x9FFF, 0x0A);
    BOARD_CHECK(read_mem(0x8000) == 128 && read_mem(0xC000) == 188);
    // ROM drives 0xBC here; a bus conflict would incorrectly select bank four.
    write_mem(0xC000, 5);
    BOARD_CHECK(read_mem(0x8000) == 148 && read_mem(0xC000) == 188);
    write_mem(0xBFFF, 0xFF);
    BOARD_CHECK(read_mem(0x8000) == 148 && read_mem(0xC000) == 188);
    write_mem(0x8000, 0x0F);
    BOARD_CHECK(read_mem(0x8000) == (uint8_t)468 && read_mem(0x8001) == 1);
    BOARD_CHECK(read_mem(0xC000) == (uint8_t)508 && read_mem(0xC001) == 1);
    write_mem(0xFFFF, 0xF2);
    BOARD_CHECK(read_mem(0x8000) == (uint8_t)456 && read_mem(0xBFFF) == (uint8_t)459);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == (uint8_t)456 && read_mem(0xC000) == (uint8_t)508);
    BOARD_CHECK(read_mem(0x6123) == 0x93 && ppu_read(0x0123) == 0x4A);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x5000, 0x52);
    BOARD_CHECK(read_mem(0x8000) == 0x52 && read_mem(0xC000) == 60);
    BOARD_CHECK(read_mem(0x6123) == 0 && ppu_read(0x0123) == 0);
    board_image_free(&image);
    return 0;
}

static int test_golden_five_geometry(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 104, 0xC000, 0, true));
    image.data[8] |= 0xF0;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0xC000) == 0);
    write_mem(0xC000, 4);
    BOARD_CHECK(read_mem(0x8000) == 4 && read_mem(0xC000) == 0);
    write_mem(0x8000, 0x09);
    BOARD_CHECK(read_mem(0x8000) == 8 && read_mem(0xC000) == 4);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 104, 0x5000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0xC000) == 0 && read_mem(0xFFFF) == 3);
    write_mem(0xC000, 15);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xBFFF) == 3);
    // The board never selects CHR ROM; only the default CHR RAM window is wired.
    BOARD_CHECK(ppu_read(0x1234) == 0x34);
    ppu_write(0x1234, 0xA7);
    BOARD_CHECK(ppu_read(0x1234) == 0x34);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 104, 0x2000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    for (unsigned window = 0; window < 4; ++window) {
        BOARD_CHECK(read_mem((uint16_t)(0x8000 + window * 0x2000)) == 0);
        BOARD_CHECK(read_mem((uint16_t)(0x9FFF + window * 0x2000)) == 1);
    }
    write_mem(0xC000, 0xFF);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 1);
    image.data[11] = 0;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(ppu_read(0x1456) == 0x56);
    board_image_free(&image);
    return 0;
}

static int test_golden_five_replacement(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 104, 0x80000, 0, false));
    BOARD_CHECK(board_image_add_trainer(&image, 0xD9));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x7000) == 0xD9 && read_mem(0x71FF) == 0xD9);
    write_mem(0xC000, 7);
    write_mem(0x7123, 0x63);
    ppu_write(0x0222, 0x74);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x8000) == 28 && read_mem(0x7123) == 0x63);
    BOARD_CHECK(ppu_read(0x0222) == 0x74);
    board_image_set_unsupported_console(&image);
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x8000) == 28 && read_mem(0x7123) == 0x63);
    image.data[0] = 0;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x8000) == 28 && ppu_read(0x0222) == 0x74);
    board_image_free(&image);
    return 0;
}

static int test_golden_five_saves(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 104, 0x40000, 0, false));
    image.data[6] |= 2;
    BOARD_CHECK(board_image_add_trainer(&image, 0x39));
    char path[128], save[128];
    snprintf(path, sizeof(path), "build/board-golden-five-%lu-%lu.nes",
             (unsigned long)time(NULL), (unsigned long)clock());
    memcpy(save, path, strlen(path) + 1);
    strcpy(strrchr(save, '.'), ".sav");
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t count = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(count == image.size && closed == 0);
    BOARD_CHECK(load_rom(path) == 0);
    BOARD_CHECK(read_mem(0x7000) == 0x39 && read_mem(0x71FF) == 0x39);
    write_mem(0x7000, 0xB2);
    write_mem(0x6FFF, 0xC4);
    ppu_write(0x1234, 0x67);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(path) == 0);
    BOARD_CHECK(read_mem(0x7000) == 0xB2 && read_mem(0x6FFF) == 0xC4);
    BOARD_CHECK(read_mem(0x71FF) == 0x39 && ppu_read(0x1234) == 0);
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

int test_board_codemasters_accuracy(void) {
    int failures = 0;
    failures += test_golden_five_registers();
    failures += test_golden_five_geometry();
    failures += test_golden_five_replacement();
    failures += test_golden_five_saves();
    unload_rom();
    printf("Golden Five accuracy: 4 groups, %d failures\n", failures);
    return failures;
}
