/*
 * board_sachen_accuracy.c - Sachen cartridge bus and protection regressions
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

static int sachen_store(uint16_t address, uint8_t value) {
    const uint8_t code[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(code); ++byte)
        write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static void sachen_register(uint8_t number, uint8_t value) {
    write_mem(0x4100, number);
    write_mem(0x4101, value);
}

static int test_sachen133(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 133, 0x10000, 0x8000, false));
    BOARD_CHECK(board_image_add_trainer(&image, 0x65));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0) == 0);
    BOARD_CHECK(sachen_store(0x4100, 7) == 0);
    BOARD_CHECK(read_mem(0x8000) == 8 && read_mem(0xFFFF) == 15 && ppu_read(0) == 24);
    write_mem(0x6100, 0);
    write_mem(0x5EFE, 0);
    BOARD_CHECK(read_mem(0x8000) == 8 && ppu_read(0) == 24);
    write_mem(0xC1FF, 4);
    BOARD_CHECK(read_mem(0x8000) == 8 && ppu_read(0) == 0);
    write_mem(0x5FFF, 2);
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0) == 16);
    write_mem(0x7000, 0xA7);
    BOARD_CHECK(read_mem(0x7000) == 0x65);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0) == 16 && read_mem(0x7000) == 0x65);
    board_image_free(&image);
    return 0;
}

static int test_sachen143(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 143, 0x8000, 0x2000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 4 && read_mem(0xFFFF) == 7);
    write_mem(0x4020, 0xA5);
    BOARD_CHECK(read_mem(0x4020) == 0xA5);
    BOARD_CHECK(read_mem(0x4100) == 0x7F && read_mem(0x4101) == 0x7E);
    BOARD_CHECK(read_mem(0x41FF) == 0x40 && read_mem(0x5FFF) == 0x40);
    BOARD_CHECK(sachen_store(0x4101, 0xFF) == 0);
    BOARD_CHECK(read_mem(0x4101) == 0x7E && ppu_read(0x1FFF) == 7);
    write_mem(0x6123, 0xAB);
    BOARD_CHECK(read_mem(0x6123) == 0xAB);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x4100) == 0x7F && read_mem(0x6123) == 0xAB);
    board_image_free(&image);
    return 0;
}

static int test_sachen145_148_149(void) {
    const unsigned ids[] = {145, 148, 149};
    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[i], 0x10000, 0x10000, false));
        image.data[16 + 0x100] = 0x0B;
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0x1234) == 0x34);
        ppu_write(0x1234, 0xE7);
        BOARD_CHECK(ppu_read(0x1234) == 0x34);
        if (ids[i] == 148) {
            BOARD_CHECK(sachen_store(0x8100, 0xFF) == 0);
            BOARD_CHECK(read_mem(0x8000) == 8 && ppu_read(0) == 24);
            write_mem(0x8000, 0xFF);
            BOARD_CHECK(read_mem(0x8000) == 8 && ppu_read(0) == 0);
            write_mem(0x8100, 0);
            BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0) == 0);
            write_mem(0x8100, 0xFF);
        } else if (ids[i] == 145) {
            BOARD_CHECK(sachen_store(0x5100, 0x80) == 0);
            write_mem(0x5000, 0);
            write_mem(0x8000, 0);
            BOARD_CHECK(ppu_read(0) == 8);
            write_mem(0x7FFF, 0);
            BOARD_CHECK(ppu_read(0) == 0);
            write_mem(0x4100, 0x80);
        } else {
            BOARD_CHECK(sachen_store(0x8000, 0x80) == 0);
            write_mem(0x7FFF, 0);
            BOARD_CHECK(ppu_read(0) == 8);
            write_mem(0xFFFF, 0);
            BOARD_CHECK(ppu_read(0) == 0);
            write_mem(0xFFFF, 0x80);
        }
        uint8_t expected = ids[i] == 148 ? 24 : 8;
        BOARD_CHECK(ppu_read(0) == expected && ppu_read(0x1FFF) == expected + 7);
        ppu_write(0x0123, 0xEE);
        BOARD_CHECK(ppu_read(0x0123) == expected);
        cpu_soft_reset(&cpu);
        BOARD_CHECK(ppu_read(0) == expected);
        BOARD_CHECK(board_image_load(&image) == 0 && ppu_read(0x1234) == 0x34);
        board_image_free(&image);
    }
    return 0;
}

static int test_sachen136(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 136, 0x10000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4800, 0x84);
    BOARD_CHECK(read_mem(0x4100) == 0xB0);
    write_mem(0x4102, 0x25);
    write_mem(0x4101, 0);
    BOARD_CHECK(sachen_store(0x4100, 0) == 0);
    write_mem(0x4800, 0xC0);
    BOARD_CHECK(read_mem(0x4100) == 0xE5 && ppu_read(0) == 0);
    write_mem(0x8000, 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0) == 40 && ppu_read(1) == 1);
    write_mem(0x4103, 1);
    write_mem(0x4100, 0);
    write_mem(0x5100, 0);
    write_mem(0xA123, 0);
    BOARD_CHECK(ppu_read(0) == 56 && ppu_read(1) == 1);
    write_mem(0x4101, 1);
    write_mem(0x4103, 0);
    write_mem(0x4102, 0x12);
    write_mem(0x4100, 0);
    write_mem(0x4800, 0x40);
    BOARD_CHECK(read_mem(0x4100) == 0x6D);
    write_mem(0x4800, 0xA6);
    BOARD_CHECK(read_mem(0x4101) == 0xA6 && read_mem(0x5EFE) == 0xA6);
    write_mem(0xFFFF, 0);
    BOARD_CHECK(ppu_read(0) == 232 && read_mem(0x8000) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(ppu_read(0) == 232);
    BOARD_CHECK(board_image_load(&image) == 0 && ppu_read(0) == 0);
    board_image_free(&image);
    return 0;
}

static int test_sachen147(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 147, 0x20000, 0x20000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4101, 0);
    write_mem(0x4102, 0x9C);
    BOARD_CHECK(sachen_store(0x4100, 0) == 0);
    BOARD_CHECK(read_mem(0x4100) == 0x9C && ppu_read(0) == 0 && read_mem(0x8000) == 0);
    write_mem(0xA000, 0);
    BOARD_CHECK(read_mem(0x8000) == 24 && ppu_read(0) == 24);
    write_mem(0x4103, 4);
    write_mem(0x4100, 0);
    BOARD_CHECK(read_mem(0x4100) == 0xA0 && ppu_read(0) == 24);
    write_mem(0x8000, 0);
    BOARD_CHECK(read_mem(0x8000) == 16 && ppu_read(0) == 32);
    write_mem(0x4103, 0);
    write_mem(0x4102, 0x43);
    write_mem(0x4100, 0);
    write_mem(0xFFFF, 0);
    BOARD_CHECK(read_mem(0x4100) == 0x43 && read_mem(0x8000) == 0 && ppu_read(0) == 64);
    write_mem(0x4800, 0xE7);
    BOARD_CHECK(read_mem(0x4102) == 0xE7 && read_mem(0x4020) == 0xE7);
    write_mem(0x6123, 0x96);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(ppu_read(0) == 64 && read_mem(0x6123) == 0x96);
    board_image_free(&image);
    return 0;
}

static int test_sachen8259(void) {
    const unsigned ids[] = {137, 138, 139, 141};
    const uint8_t pages[4][4] = {{1, 18, 19, 28}, {34, 36, 38, 40},
                                {136, 146, 156, 166}, {68, 74, 76, 82}};
    const uint8_t simple[4][4] = {{1, 17, 17, 25}, {34, 34, 34, 34},
                                 {136, 138, 140, 142}, {68, 70, 68, 70}};
    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[i], 0x40000, 0x80000, true));
        BOARD_CHECK(board_image_load(&image) == 0 && ppu_read(0x1234) == 0x34);
        write_mem(0x4100, 5);
        write_mem(0x5EFF, 3);
        BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0x1234) == 0x34);
        BOARD_CHECK(sachen_store(0x4101, 3) == 0);
        for (uint8_t reg = 0; reg < 4; ++reg) sachen_register(reg, reg + 1);
        sachen_register(4, ids[i] == 137 ? 7 : 2);
        sachen_register(6, 1);
        BOARD_CHECK(read_mem(0x8000) == 24 && read_mem(0xFFFF) == 31);
        unsigned slot_size = ids[i] == 137 ? 0x400 : 0x800;
        for (unsigned slot = 0; slot < 4; ++slot) {
            BOARD_CHECK(ppu_read((uint16_t)(slot * slot_size)) == pages[i][slot]);
            BOARD_CHECK(ppu_read((uint16_t)(slot * slot_size + slot_size - 1))
                        == pages[i][slot] + (ids[i] == 137 ? 0 : 1));
        }
        if (ids[i] == 137) {
            BOARD_CHECK(ppu_read(0x1000) == 252 && ppu_read(0x1001) == 1);
            BOARD_CHECK(ppu_read(0x1FFF) == 255);
        }
        sachen_register(7, 4);
        ppu_write(0x2000, 0x31);
        ppu_write(0x2400, 0x42);
        BOARD_CHECK(ppu_read(0x2800) == 0x42 && ppu_read(0x2C00) == 0x42);
        BOARD_CHECK(ppu_read(0x3000) == 0x31 && ppu_read(0x3400) == 0x42);
        sachen_register(7, 6);
        ppu_write(0x2800, 0x63);
        BOARD_CHECK(ppu_read(0x2000) == 0x63 && ppu_read(0x2C00) == 0x63);
        write_mem(0x7FFE, 7);
        write_mem(0x7FFF, 7);
        BOARD_CHECK(cart_get_mirroring() == (ids[i] == 137 ? MIRROR_HORIZONTAL : MIRROR_VERTICAL));
        for (unsigned slot = 0; slot < 4; ++slot)
            BOARD_CHECK(ppu_read((uint16_t)(slot * slot_size)) == simple[i][slot]);
        write_mem(0x8000, 0);
        cpu_soft_reset(&cpu);
        BOARD_CHECK(read_mem(0x8000) == 24 && ppu_read((uint16_t)(3 * slot_size)) == simple[i][3]);
        BOARD_CHECK(board_image_load(&image) == 0 && ppu_read(0x1234) == 0x34);
        board_image_free(&image);
    }
    return 0;
}

static int test_sachen8259_ram(void) {
    const unsigned ids[] = {138, 139, 141};
    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[i], 0x10000, 0, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        ppu_write(0x0023, 0xA5);
        ppu_write(0x1823, 0xB6);
        sachen_register(0, 3);
        sachen_register(4, 7);
        sachen_register(7, 1);
        BOARD_CHECK(ppu_read(0x0023) == 0xA5 && ppu_read(0x1823) == 0xB6);
        board_image_free(&image);
        BOARD_CHECK(board_image_create(&image, ids[i], 0x10000, 0x20000, true));
        image.data[11] = 7;
        BOARD_CHECK(board_image_load(&image) == 0);
        sachen_register(0, 1);
        sachen_register(5, 1);
        ppu_write(0x0023, 0xA5);
        BOARD_CHECK(read_mem(0x8000) == 8 && ppu_read(0x0023) == 0x23 && ppu_read(0x1823) == 0x23);
        board_image_free(&image);
    }
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 137, 0x10000, 0, true));
    image.data[11] = 9;
    BOARD_CHECK(board_image_load(&image) == 0);
    sachen_register(1, 2);
    ppu_write(0x0423, 0xA5);
    sachen_register(1, 3);
    BOARD_CHECK(ppu_read(0x0423) == 0);
    ppu_write(0x0423, 0xB6);
    sachen_register(1, 2);
    BOARD_CHECK(ppu_read(0x0423) == 0xA5);
    sachen_register(1, 3);
    BOARD_CHECK(ppu_read(0x0423) == 0xB6);
    board_image_free(&image);
    return 0;
}

static int test_sachen150(void) {
    BoardImage image;
    BOARD_CHECK(cart_set_dip_switches(0));
    BOARD_CHECK(board_image_create(&image, 150, 0x20000, 0x20000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    ppu_write(0x2000, 0x11);
    ppu_write(0x2C00, 0x22);
    BOARD_CHECK(ppu_read(0x2400) == 0x11 && ppu_read(0x2800) == 0x11 && ppu_read(0x3000) == 0x11);
    sachen_register(4, 1);
    sachen_register(6, 3);
    sachen_register(5, 3);
    BOARD_CHECK(read_mem(0x8000) == 24 && ppu_read(0) == 56);
    write_mem(0xC100, 0);
    sachen_register(7, 4);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    write_mem(0x4800, 0xD0);
    BOARD_CHECK(read_mem(0x4101) == 0xD4);
    write_mem(0x4800, 0xD0);
    BOARD_CHECK(read_mem(0x4100) == 0xD0 && read_mem(0x6000) == 0xD0);
    BOARD_CHECK(cart_set_dip_switches(1));
    write_mem(0x4100, 1);
    BOARD_CHECK(sachen_store(0x4101, 2) == 0);
    BOARD_CHECK(read_mem(0x8000) == 16 && ppu_read(0) == 56);
    write_mem(0x4800, 0xA0);
    BOARD_CHECK(read_mem(0x4101) == 0xA2);
    write_mem(0x4800, 0xA4);
    BOARD_CHECK(read_mem(0x7FFF) == 0xA6);
    BOARD_CHECK(cart_set_dip_switches(0));
    write_mem(0x4800, 0xA0);
    BOARD_CHECK(read_mem(0x4101) == 0xA6);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 16 && ppu_read(0) == 56);
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0 && ppu_read(0) == 0);
    board_image_free(&image);
    return 0;
}

static int test_sachen_geometry_and_replacement(void) {
    const unsigned ids[] = {133, 136, 137, 138, 139, 141, 143, 145, 147, 148, 149, 150, 243};
    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[i], 0x6000, 0x2800, true));
        image.data[8] |= 0xF0;
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == (ids[i] == 143 ? 0 : 4));
        write_mem(0x4800, 0xA5);
        BOARD_CHECK(read_mem(0xFFFF) == (ids[i] == 143 ? 3 : 0xA5));
        uint8_t previous = ppu_read(0x1234);
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
        BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0x1234) == previous);
        image.data[7] |= 2;
        BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
        BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0x1234) == previous);
        image.data[0] = 0;
        BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
        BOARD_CHECK(ppu_read(0x1234) == previous);
        board_image_free(&image);
        BOARD_CHECK(board_image_create(&image, ids[i], 0x2000, 0x100, true));
        BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0xFFFF) == 1);
        if (ids[i] == 133 || ids[i] == 145) write_mem(0x4100, 0);
        else if (ids[i] == 136 || ids[i] == 147 || ids[i] == 148 || ids[i] == 149) write_mem(0x8000, 0);
        else if (ids[i] != 143) sachen_register(5, 0);
        BOARD_CHECK(ppu_read(0x0023) == 0 && ppu_read(0x0823) == 0x23);
        board_image_free(&image);
    }
    return 0;
}

static int test_sachen_persistence(void) {
    const unsigned ids[] = {133, 136, 137, 138, 139, 141, 143, 145, 147, 148, 149, 150, 243};
    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[i], 0x10000, 0, true));
        image.data[6] |= 2;
        image.data[10] = 0x70;
        image.data[11] = 0x70;
        BOARD_CHECK(board_image_add_trainer(&image, 0x44));
        char path[128], save[128], chr_save[136];
        snprintf(path, sizeof(path), "build/board-sachen-%u-%lu-%lu.nes", ids[i],
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
        uint8_t seed[0x2000] = {0};
        seed[0] = 0xB4;
        seed[0x1000] = 0x9A;
        file = fopen(save, "wb");
        BOARD_CHECK(file != NULL);
        count = fwrite(seed, 1, sizeof(seed), file);
        closed = fclose(file);
        BOARD_CHECK(count == sizeof(seed) && closed == 0);
        BOARD_CHECK(load_rom(path) == 0);
        ppu_power_on(&ppu);
        BOARD_CHECK(cpu_power_on(&cpu));
        if (ids[i] != 150 && ids[i] != 243) BOARD_CHECK(read_mem(0x7000) == 0x9A);
        write_mem(0x7000, 0x5E);
        ppu_write(0x0023, 0x6F);
        BOARD_CHECK(unload_rom());
        BOARD_CHECK(load_rom(path) == 0 && ppu_read(0x0023) == 0x6F);
        bool writable = ids[i] == 136 || ids[i] == 143 || ids[i] == 147 || ids[i] == 148 || ids[i] == 149;
        if (ids[i] != 150 && ids[i] != 243) BOARD_CHECK(read_mem(0x7000) == (writable ? 0x5E : 0x9A));
        BOARD_CHECK(unload_rom());
        file = fopen(save, "rb");
        BOARD_CHECK(file != NULL);
        count = fread(seed, 1, sizeof(seed), file);
        closed = fclose(file);
        BOARD_CHECK(count == sizeof(seed) && closed == 0 && seed[0] == 0xB4);
        BOARD_CHECK(seed[0x1000] == (writable ? 0x5E : 0x9A));
        file = fopen(chr_save, "rb");
        BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0);
        long length = ftell(file);
        closed = fclose(file);
        BOARD_CHECK(length == 0x2000 && closed == 0);
        BOARD_CHECK(remove(path) == 0 && remove(save) == 0 && remove(chr_save) == 0);
        board_image_free(&image);
    }
    return 0;
}

int test_board_sachen_accuracy(void) {
    int failures = 0;
    failures += test_sachen133();
    failures += test_sachen143();
    failures += test_sachen145_148_149();
    failures += test_sachen136();
    failures += test_sachen147();
    failures += test_sachen8259();
    failures += test_sachen8259_ram();
    failures += test_sachen150();
    cart_set_dip_switches(0);
    failures += test_sachen_geometry_and_replacement();
    failures += test_sachen_persistence();
    unload_rom();
    printf("Sachen accuracy: 10 groups, %d failures\n", failures);
    return failures;
}
