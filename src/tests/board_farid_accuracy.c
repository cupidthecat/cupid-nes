/*
 * board_farid_accuracy.c - Farid multicart bank and serial bus regressions
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

static int farid_store(uint16_t address, uint8_t value) {
    const uint8_t code[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(code); ++byte)
        write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int farid_serial(uint16_t address, uint8_t value) {
    for (unsigned bit = 0; bit < 5; ++bit)
        BOARD_CHECK(farid_store(address, (uint8_t)((value >> bit) & 1)) == 0);
    return 0;
}

static void farid_conflict_bytes(BoardImage *image, size_t prg_bytes) {
    for (size_t bank = 0; bank * 0x4000 + 3 < prg_bytes; ++bank) {
        image->data[sizeof(iNESHeader) + bank * 0x4000 + 2] = 0xFF;
        image->data[sizeof(iNESHeader) + bank * 0x4000 + 3] = 0x95;
    }
}

static int test_farid_slrom_registers(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 323, 0x100000, 0x100000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 28);
    BOARD_CHECK(ppu_read(0) == 0 && ppu_read(0x1000) == 4);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    BOARD_CHECK(farid_store(0x6000, 0x20) == 0);
    BOARD_CHECK(read_mem(0x8000) == 64 && read_mem(0xC000) == 92);
    BOARD_CHECK(ppu_read(0) == 0 && ppu_read(1) == 1);
    BOARD_CHECK(farid_serial(0xE000, 3) == 0);
    BOARD_CHECK(read_mem(0x8000) == 76 && read_mem(0xC000) == 92);
    BOARD_CHECK(farid_serial(0x8000, 0x1E) == 0);
    BOARD_CHECK(farid_serial(0xA000, 5) == 0 && farid_serial(0xC000, 10) == 0);
    BOARD_CHECK(ppu_read(0) == 20 && ppu_read(1) == 1);
    BOARD_CHECK(ppu_read(0x1000) == 40 && ppu_read(0x1001) == 1);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    ppu_write(0x2000, 0x96);
    BOARD_CHECK(ppu_read(0x2800) == 0x96);
    BOARD_CHECK(farid_store(0x7FFF, 0x58) == 0);
    BOARD_CHECK(read_mem(0x8000) == 172 && read_mem(0xC000) == 188);
    BOARD_CHECK(farid_store(0x6123, 0x10) == 0);
    BOARD_CHECK(read_mem(0x8000) == 172 && ppu_read(1) == 2);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 12 && read_mem(0xC000) == 28);
    BOARD_CHECK(ppu_read(0) == 20 && ppu_read(1) == 0);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);

    BOARD_CHECK(farid_serial(0xE000, 0x13) == 0);
    BOARD_CHECK(farid_store(0x6000, 0x70) == 0);
    BOARD_CHECK(read_mem(0x8000) == 12);
    write_mem(0x5000, 0xA6);
    BOARD_CHECK(read_mem(0x6000) == 0xA6);
    BOARD_CHECK(farid_serial(0xE000, 3) == 0 && farid_store(0x6000, 0x20) == 0);
    BOARD_CHECK(read_mem(0x8000) == 76 && read_mem(0x6000) == 0);

    // The paired-page path bypasses the outer PRG transform in 32 KiB mode.
    BOARD_CHECK(farid_serial(0x8000, 0x12) == 0);
    BOARD_CHECK(read_mem(0x8000) == 8 && read_mem(0xC000) == 12);
    BOARD_CHECK(ppu_read(1) == 1);
    BOARD_CHECK(farid_serial(0x8000, 0x1A) == 0);
    BOARD_CHECK(read_mem(0x8000) == 64 && read_mem(0xC000) == 76);

    // Reset clears the outer latch but retains a partly written serial word.
    BOARD_CHECK(farid_store(0x8000, 0x80) == 0);
    BOARD_CHECK(farid_store(0xE000, 1) == 0 && farid_store(0xE000, 0) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(farid_store(0xE000, 0) == 0 && farid_store(0xE000, 1) == 0);
    BOARD_CHECK(farid_store(0xE000, 0) == 0);
    BOARD_CHECK(read_mem(0x8000) == 4 && read_mem(0xC000) == 28);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x8000) == 4 && cart_get_mirroring() == MIRROR_VERTICAL);
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_SINGLE0 && ppu_read(0) == 0);
    board_image_free(&image);
    return 0;
}

static int test_farid_slrom_serial_timing(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 323, 0x20000, 0, true));
    image.data[sizeof(iNESHeader) + 7 * 0x4000 + 0x2124] = 0x7F;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(farid_store(0x8000, 0x80) == 0);
    write_mem(0x0300, 0xEE);
    write_mem(0x0301, 0x23);
    write_mem(0x0302, 0xE1);
    cpu.pc = 0x0300;
    BOARD_CHECK(cpu_step(&cpu) == 6);
    BOARD_CHECK(farid_store(0xE000, 0) == 0 && farid_store(0xE000, 1) == 0);
    BOARD_CHECK(farid_store(0xE000, 0) == 0 && farid_store(0xE000, 0) == 0);
    BOARD_CHECK(read_mem(0x8000) == 16);
    BOARD_CHECK(farid_store(0x8000, 0x80) == 0);
    write_mem(0x0301, 0x24);
    cpu.pc = 0x0300;
    BOARD_CHECK(cpu_step(&cpu) == 6);
    BOARD_CHECK(farid_serial(0xE000, 6) == 0);
    BOARD_CHECK(read_mem(0x8000) == 24);
    board_image_free(&image);
    return 0;
}

static int test_farid_unrom_registers(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 324, 0x100000, 0, true));
    image.data[10] = 7;
    image.data[6] |= 1;
    farid_conflict_bytes(&image, 0x100000);
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 28);
    BOARD_CHECK(farid_store(0x8002, 0x13) == 0);
    BOARD_CHECK(read_mem(0x8000) == 12 && read_mem(0xC000) == 28);
    BOARD_CHECK(farid_store(0x8002, 0xA5) == 0);
    BOARD_CHECK(read_mem(0x8000) == 84 && read_mem(0xC000) == 92);
    BOARD_CHECK(farid_store(0x8002, 0xE2) == 0);
    BOARD_CHECK(read_mem(0x8000) == 72 && read_mem(0xC000) == 92);
    BOARD_CHECK(farid_store(0x8002, 0x42) == 0 && farid_store(0x8002, 0xDA) == 0);
    BOARD_CHECK(read_mem(0x8000) == 168 && read_mem(0xC000) == 188);
    BOARD_CHECK(farid_store(0x8002, 1) == 0 && farid_store(0x8002, 0xB3) == 0);
    BOARD_CHECK(read_mem(0x8000) == 172 && read_mem(0xC000) == 188);
    write_mem(0x6123, 0x85);
    ppu_write(0x1234, 0x96);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 172 && read_mem(0xC000) == 188);
    BOARD_CHECK(read_mem(0x6123) == 0x85 && ppu_read(0x1234) == 0x96);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    BOARD_CHECK(farid_store(0x8002, 0xE4) == 0);
    BOARD_CHECK(read_mem(0x8000) == 16 && read_mem(0xC000) == 28);
    BOARD_CHECK(farid_store(0x8002, 0) == 0 && farid_store(0x8002, 0xD1) == 0);
    BOARD_CHECK(read_mem(0x8000) == 164 && read_mem(0xC000) == 188);
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(farid_store(0x8003, 0xFF) == 0);
    BOARD_CHECK(read_mem(0x8000) == 52 && read_mem(0xC000) == 60);
    board_image_free(&image);
    return 0;
}

static int test_farid_geometry_and_replacement(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 323, 0x100000, 0x2000, true));
    image.data[8] |= 0x50;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(farid_store(0x6000, 0x30) == 0 && farid_serial(0xE000, 7) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 4);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 323, 0x6000, 0x1800, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 3);
    BOARD_CHECK(ppu_read(0) == 0 && ppu_read(0x1000) == 0 && ppu_read(0x1FFF) == 3);
    BOARD_CHECK(farid_serial(0xE000, 7) == 0 && read_mem(0x8000) == 0);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 323, 0x8000, 0x200, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(ppu_read(0x0234) == 0 && ppu_read(0x0456) == 0x56);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 324, 0xC000, 0x2000, true));
    image.data[8] |= 0xF0;
    farid_conflict_bytes(&image, 0xC000);
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0xC000) == 4);
    BOARD_CHECK(farid_store(0x8002, 0x9F) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 0);
    ppu_write(0x1234, 0xFF);
    BOARD_CHECK(ppu_read(0x1234) == 4);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0xC000) == 0 && ppu_read(0x1234) == 4);
    board_image_set_unsupported_console(&image);
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0xC000) == 0 && ppu_read(0x1234) == 4);
    image.data[0] = 0;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0xC000) == 0 && ppu_read(0x1234) == 4);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 324, 0x2000, 0, true));
    farid_conflict_bytes(&image, 0x2000);
    BOARD_CHECK(board_image_load(&image) == 0);
    for (unsigned offset = 0; offset < 0x8000; offset += 0x1000)
        BOARD_CHECK(read_mem((uint16_t)(0x8000 + offset)) == (offset / 0x1000) % 2);
    BOARD_CHECK(farid_store(0x8002, 0xFF) == 0 && read_mem(0xFFFF) == 1);
    board_image_free(&image);
    return 0;
}

static int test_farid_save_ownership(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 323, 0x20000, 0, true));
    image.data[6] |= 2;
    image.data[10] = 0x77;
    BOARD_CHECK(board_image_add_trainer(&image, 0x9A));
    char path[128], save[128];
    snprintf(path, sizeof(path), "build/board-farid-%lu-%lu.nes",
             (unsigned long)time(NULL), (unsigned long)clock());
    memcpy(save, path, strlen(path) + 1);
    strcpy(strrchr(save, '.'), ".sav");
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t count = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(count == image.size && closed == 0);
    file = fopen(save, "wb");
    BOARD_CHECK(file != NULL);
    uint8_t saved[0x2000];
    memset(saved, 0x5A, sizeof(saved));
    count = fwrite(saved, 1, sizeof(saved), file);
    closed = fclose(file);
    BOARD_CHECK(count == sizeof(saved) && closed == 0);
    BOARD_CHECK(load_rom(path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu));
    BOARD_CHECK(read_mem(0x7000) == 0x5A);
    BOARD_CHECK(farid_serial(0xA000, 8) == 0 && read_mem(0x7000) == 0x9A);
    BOARD_CHECK(farid_store(0x6123, 0x36) == 0 && read_mem(0x6123) == 0);
    BOARD_CHECK(farid_serial(0xA000, 0) == 0 && read_mem(0x7000) == 0x5A);
    BOARD_CHECK(farid_store(0x7000, 0xF7) == 0 && read_mem(0x7000) == 0x5A);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(path) == 0 && read_mem(0x7000) == 0x5A);
    BOARD_CHECK(unload_rom());
    file = fopen(save, "rb");
    BOARD_CHECK(file != NULL);
    count = fread(saved, 1, sizeof(saved), file);
    closed = fclose(file);
    BOARD_CHECK(count == sizeof(saved) && closed == 0);
    for (unsigned byte = 0; byte < sizeof(saved); ++byte) BOARD_CHECK(saved[byte] == 0x5A);
    BOARD_CHECK(remove(save) == 0 && remove(path) == 0);
    board_image_free(&image);
    return 0;
}

static int test_farid_unrom_save(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 324, 0x20000, 0, true));
    image.data[6] |= 2;
    image.data[10] = 0x70;
    farid_conflict_bytes(&image, 0x20000);
    BOARD_CHECK(board_image_add_trainer(&image, 0x9A));
    char path[128], save[128];
    snprintf(path, sizeof(path), "build/board-farid-unrom-%lu-%lu.nes",
             (unsigned long)time(NULL), (unsigned long)clock());
    memcpy(save, path, strlen(path) + 1);
    strcpy(strrchr(save, '.'), ".sav");
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t count = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(count == image.size && closed == 0);
    BOARD_CHECK(load_rom(path) == 0 && read_mem(0x7000) == 0x9A);
    write_mem(0x7123, 0x46);
    ppu_write(0x1234, 0x85);
    write_mem(0x8002, 0xA3);
    BOARD_CHECK(read_mem(0x8000) == 12);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(path) == 0 && read_mem(0x7123) == 0x46);
    BOARD_CHECK(read_mem(0x7000) == 0x9A && ppu_read(0x1234) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 28);
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

int test_board_farid_accuracy(void) {
    int failures = 0;
    failures += test_farid_slrom_registers();
    failures += test_farid_slrom_serial_timing();
    failures += test_farid_unrom_registers();
    failures += test_farid_geometry_and_replacement();
    failures += test_farid_save_ownership();
    failures += test_farid_unrom_save();
    unload_rom();
    printf("Farid accuracy: 6 groups, %d failures\n", failures);
    return failures;
}
