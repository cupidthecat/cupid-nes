/*
 * board_racermate_accuracy.c - Racermate bank, IRQ, and save regressions
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

static int racermate_store(uint16_t address, uint8_t value) {
    const uint8_t code[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(code); ++byte)
        write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int racermate_nops(unsigned count) {
    write_mem(0x0300, 0xEA);
    for (unsigned instruction = 0; instruction < count; ++instruction) {
        cpu.pc = 0x0300;
        BOARD_CHECK(cpu_step(&cpu) == 2);
    }
    return 0;
}

static int test_racermate_banks(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 168, 0x20000, 0, false));
    image.data[6] |= 1;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4800, 0xA5);
    BOARD_CHECK(read_mem(0x8000) == 0xA5 && read_mem(0xBFFF) == 0xA5);
    BOARD_CHECK(read_mem(0xC000) == 28 && read_mem(0xFFFF) == 31);
    for (unsigned bank = 0; bank < 16; ++bank) {
        write_mem(0x8000, (uint8_t)bank);
        ppu_write(0x1123, (uint8_t)(0x60 + bank));
        ppu_write(0x1FFF, (uint8_t)(0x80 + bank));
    }
    for (unsigned prg = 0; prg < 4; ++prg) {
        for (unsigned chr = 0; chr < 16; ++chr) {
            write_mem(0xBFFF, (uint8_t)((prg << 6) | chr | 0x30));
            BOARD_CHECK(read_mem(0x8000) == prg * 4 && read_mem(0xBFFF) == prg * 4 + 3);
            BOARD_CHECK(read_mem(0xC000) == 28 && read_mem(0xFFFF) == 31);
            BOARD_CHECK(ppu_read(0x1123) == 0x60 + chr && ppu_read(0x1FFF) == 0x80 + chr);
            BOARD_CHECK(ppu_read(0x0123) == 0x60 && ppu_read(0x0FFF) == 0x80);
        }
    }
    BOARD_CHECK(racermate_store(0xA321, 0x82) == 0);
    BOARD_CHECK(read_mem(0x8000) == 8 && ppu_read(0x1123) == 0x62);
    write_mem(0xC000, 0xFF);
    write_mem(0xFFFF, 0);
    write_mem(0x7000, 0x72);
    BOARD_CHECK(read_mem(0x8000) == 8 && ppu_read(0x1123) == 0x62);
    ppu_write(0x2000, 0x45);
    BOARD_CHECK(ppu_read(0x2800) == 0x45 && cart_get_mirroring() == MIRROR_VERTICAL);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 8 && ppu_read(0x1123) == 0x62 && read_mem(0x7000) == 0x72);
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4800, 0x5A);
    BOARD_CHECK(read_mem(0x8000) == 0x5A && ppu_read(0x0123) == 0);
    board_image_free(&image);
    return 0;
}

static int test_racermate_irq(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 168, 0x10000, 0, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(racermate_nops(32764) == 0 && !cart_irq_pending());
    BOARD_CHECK(racermate_nops(1) == 0 && cart_irq_pending());
    BOARD_CHECK(racermate_store(0xC000, 0) == 0 && !cart_irq_pending());
    BOARD_CHECK(racermate_nops(511) == 0 && !cart_irq_pending());
    BOARD_CHECK(racermate_nops(1) == 0 && cart_irq_pending());
    BOARD_CHECK(racermate_store(0xFFFF, 0xA5) == 0 && !cart_irq_pending());
    BOARD_CHECK(racermate_nops(510) == 0 && !cart_irq_pending());
    write_mem(0x0300, 0x8D);
    write_mem(0x0301, 0);
    write_mem(0x0302, 4);
    cpu.pc = 0x0300;
    BOARD_CHECK(cpu_step(&cpu) == 4 && cart_irq_pending());
    BOARD_CHECK(racermate_store(0xC321, 0) == 0 && !cart_irq_pending());
    BOARD_CHECK(racermate_nops(300) == 0);
    BOARD_CHECK(racermate_store(0x4014, 0) == 0 && !cart_irq_pending());
    write_mem(0x0300, 0xEA);
    cpu.pc = 0x0300;
    BOARD_CHECK(cpu_step(&cpu) >= 515 && cart_irq_pending());
    BOARD_CHECK(racermate_store(0xC000, 0) == 0);
    BOARD_CHECK(racermate_nops(509) == 0 && !cart_irq_pending());
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cart_irq_pending());
    BOARD_CHECK(board_image_load(&image) == 0 && !cart_irq_pending());
    BOARD_CHECK(racermate_nops(512) == 0 && !cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static int test_racermate_geometry(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 168, 0x14000, 0x2800, true));
    image.data[8] |= 0xF0;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0xC000) == 16 && ppu_read(0x0FFF) == 3);
    BOARD_CHECK(ppu_read(0x1456) == 0x56);
    write_mem(0xBFFF, 0xCF);
    BOARD_CHECK(read_mem(0x8000) == 12 && read_mem(0xFFFF) == 19);
    BOARD_CHECK(ppu_read(0x1000) == 4 && ppu_read(0x1FFF) == 7);
    ppu_write(0x1123, 0xA5);
    BOARD_CHECK(ppu_read(0x1123) == 4);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x8000) == 12 && ppu_read(0x1123) == 4);
    board_image_set_unsupported_console(&image);
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x8000) == 12 && ppu_read(0x1123) == 4);
    image.data[0] = 0;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x8000) == 12 && ppu_read(0x1123) == 4);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 168, 0x2000, 0x800, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 1);
    BOARD_CHECK(ppu_read(0x07FF) == 1 && ppu_read(0x0845) == 0x45);
    write_mem(0x8000, 0xFF);
    BOARD_CHECK(ppu_read(0x0FFF) == 1 && ppu_read(0x1045) == 0x45);
    image.data[11] = 0;
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 168, 0x8000, 0, true));
    image.data[11] = 0;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8000, 0xFF);
    ppu_write(0x1123, 0xA5);
    BOARD_CHECK(ppu_read(0x1123) == 0x23);
    board_image_free(&image);
    return 0;
}

static int test_racermate_persistence_case(bool nes20) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 168, 0x10000, 0, nes20));
    if (nes20) {
        image.data[6] |= 2;
        image.data[10] = 0x70;
        image.data[11] = 0x99;
    }
    BOARD_CHECK(board_image_add_trainer(&image, 0x9A));
    char path[128], save[128], chr_save[136];
    snprintf(path, sizeof(path), "build/board-racermate-%u-%lu-%lu.nes", nes20 ? 1u : 0u,
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
    BOARD_CHECK(load_rom(path) == 0 && read_mem(0x7000) == 0x9A);
    write_mem(0x7000, 0x75);
    for (unsigned bank = 0; bank < 16; ++bank) {
        write_mem(0x8000, (uint8_t)bank);
        ppu_write(0x1123, (uint8_t)(0x60 + bank));
    }
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(path) == 0 && read_mem(0x7000) == (nes20 ? 0x75 : 0x9A));
    for (unsigned bank = 0; bank < 16; ++bank) {
        write_mem(0x8000, (uint8_t)bank);
        BOARD_CHECK(ppu_read(0x1123) == (bank < 8 ? 0 : 0x60 + bank));
    }
    BOARD_CHECK(unload_rom());
    file = fopen(chr_save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0);
    long length = ftell(file);
    BOARD_CHECK(fseek(file, 0x123, SEEK_SET) == 0 && fgetc(file) == 0x68);
    closed = fclose(file);
    BOARD_CHECK(length == 0x8000 && closed == 0);
    if (nes20) {
        file = fopen(save, "rb");
        BOARD_CHECK(file != NULL && fseek(file, 0x1000, SEEK_SET) == 0 && fgetc(file) == 0x75);
        BOARD_CHECK(fclose(file) == 0 && remove(save) == 0);
    } else {
        BOARD_CHECK(fopen(save, "rb") == NULL);
    }
    BOARD_CHECK(remove(chr_save) == 0 && remove(path) == 0);
    board_image_free(&image);
    return 0;
}

int test_board_racermate_accuracy(void) {
    int failures = 0;
    failures += test_racermate_banks();
    failures += test_racermate_irq();
    failures += test_racermate_geometry();
    failures += test_racermate_persistence_case(false);
    failures += test_racermate_persistence_case(true);
    unload_rom();
    printf("Racermate accuracy: 5 groups, %d failures\n", failures);
    return failures;
}
