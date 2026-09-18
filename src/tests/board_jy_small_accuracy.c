/*
 * board_jy_small_accuracy.c - JY 35 and 91 bank and interrupt regressions
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

static int jy_store(uint16_t address, uint8_t value) {
    const uint8_t code[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(code); ++byte) write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int jy_cpu_cycles(unsigned cycles) {
    if (cycles == 2) write_mem(0x0300, 0xEA);
    else {
        write_mem(0x0300, 0x4C);
        write_mem(0x0301, 0);
        write_mem(0x0302, 3);
    }
    cpu.pc = 0x0300;
    BOARD_CHECK(cpu_step(&cpu) == (int)cycles);
    return 0;
}

static void jy_ppu_edge(unsigned dots) {
    cart_notify_ppu_address(0, ppu.total_cycles);
    ppu_step_dots((int)dots);
    cart_notify_ppu_address(0x1000, ppu.total_cycles);
}

static int jy_cpu_edge(unsigned cycles) {
    cart_notify_ppu_address(0, ppu.total_cycles);
    BOARD_CHECK(jy_cpu_cycles(cycles) == 0);
    cart_notify_ppu_address(0x1000, ppu.total_cycles);
    return 0;
}

static int test_jy35_banks(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 35, 0x200000, 0x40000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4018, 0xA5);
    BOARD_CHECK(read_mem(0x8000) == 0xA5 && read_mem(0xA000) == 0xA5 && read_mem(0xC000) == 0xA5);
    BOARD_CHECK(read_mem(0xE000) == 254 && read_mem(0xE001) == 1 && ppu_read(0x1234) == 0x34);
    for (unsigned slot = 0; slot < 4; ++slot) {
        BOARD_CHECK(jy_store((uint16_t)(0x8FF8 + slot), (uint8_t)(0x81 + slot)) == 0);
        uint16_t address = (uint16_t)(0x8000 + slot * 0x2000);
        BOARD_CHECK(read_mem(address) == 2 + slot * 2 && read_mem(address + 1) == 1);
    }
    for (unsigned slot = 0; slot < 8; ++slot) {
        BOARD_CHECK(jy_store((uint16_t)(0x9FF8 + slot), (uint8_t)(11 + slot)) == 0);
        BOARD_CHECK(ppu_read((uint16_t)(slot * 0x400)) == 11 + slot);
    }
    write_mem(0x8004, 0);
    write_mem(0xA000, 0);
    BOARD_CHECK(read_mem(0x8000) == 2 && ppu_read(0) == 11);
    write_mem(0xDFF9, 0);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    write_mem(0xD001, 1);
    write_mem(0xD000, 0);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    ppu_write(0x0023, 0xAB);
    BOARD_CHECK(ppu_read(0x0023) == 11);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 2 && ppu_read(0) == 11 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    board_image_free(&image);
    return 0;
}

static int test_jy91_banks(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 91, 0x80000, 0x80000, false));
    BOARD_CHECK(board_image_add_trainer(&image, 0x67));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4018, 0xA5);
    BOARD_CHECK(read_mem(0x8000) == 0xA5 && read_mem(0xA000) == 0xA5 && ppu_read(0x1234) == 0x34);
    BOARD_CHECK(read_mem(0xC000) == 124 && read_mem(0xE000) == 126 && read_mem(0x7000) == 0x67);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    BOARD_CHECK(jy_store(0x7FFC, 0xF7) == 0);
    BOARD_CHECK(jy_store(0x7FFD, 0xE8) == 0);
    BOARD_CHECK(read_mem(0x8000) == 14 && read_mem(0xA000) == 16);
    for (unsigned slot = 0; slot < 4; ++slot) {
        BOARD_CHECK(jy_store((uint16_t)(0x6F00 + slot), (uint8_t)(0x80 + slot)) == 0);
        uint16_t address = (uint16_t)(slot * 0x800);
        BOARD_CHECK(ppu_read(address) == slot * 2 && ppu_read(address + 1) == 1);
        BOARD_CHECK(ppu_read(address + 0x7FF) == slot * 2 + 1);
    }
    write_mem(0xA000, 0);
    write_mem(0x8000, 0);
    BOARD_CHECK(read_mem(0x8000) == 14 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    BOARD_CHECK(jy_store(0x7000, 5) == 0 && read_mem(0x7000) == 0x67 && read_mem(0x8000) == 10);
    ppu_write(0x0123, 0xAB);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 10 && ppu_read(0x0123) == 0 && ppu_read(1) == 1);
    board_image_free(&image);
    return 0;
}

static int test_jy35_irq(void) {
    BoardImage image;
    nes_set_region(NES_REGION_NTSC);
    BOARD_CHECK(board_image_create(&image, 35, 0x8000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    cart_notify_ppu_address(0x1000, ppu.total_cycles);
    write_mem(0xC005, 1);
    BOARD_CHECK(jy_store(0xC003, 0) == 0);
    jy_ppu_edge(9);
    BOARD_CHECK(!cart_irq_pending());
    jy_ppu_edge(10);
    BOARD_CHECK(cart_irq_pending());
    BOARD_CHECK(jy_store(0xC002, 0) == 0 && !cart_irq_pending());
    jy_ppu_edge(100);
    BOARD_CHECK(!cart_irq_pending());
    write_mem(0xC005, 0);
    write_mem(0xC003, 0);
    for (unsigned edge = 1; edge <= 256; ++edge) {
        jy_ppu_edge(10);
        BOARD_CHECK(cart_irq_pending() == (edge == 256));
    }
    write_mem(0xCFFa, 0);
    BOARD_CHECK(!cart_irq_pending());
    write_mem(0xCFFD, 2);
    write_mem(0xCFFB, 0);
    jy_ppu_edge(10);
    BOARD_CHECK(!cart_irq_pending());
    cpu_soft_reset(&cpu);
    jy_ppu_edge(10);
    BOARD_CHECK(cart_irq_pending());
    write_mem(0xC002, 0);
    write_mem(0xC005, 1);
    write_mem(0xC003, 0);
    ppu.scanline = 260;
    ppu.dot = 335;
    jy_ppu_edge(9);
    BOARD_CHECK(ppu.scanline == 261 && ppu.dot == 3 && !cart_irq_pending());
    jy_ppu_edge(10);
    BOARD_CHECK(cart_irq_pending());
    BOARD_CHECK(board_image_load(&image) == 0 && !cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static int test_jy91_irq(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 91, 0x8000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cart_set_mmc3_revision_name("standard"));
    cart_notify_ppu_address(0x1000, ppu.total_cycles);
    BOARD_CHECK(jy_store(0x7003, 0) == 0);
    BOARD_CHECK(jy_cpu_edge(2) == 0 && !cart_irq_pending());
    for (unsigned edge = 1; edge <= 8; ++edge) {
        BOARD_CHECK(jy_cpu_edge(3) == 0);
        BOARD_CHECK(cart_irq_pending() == (edge == 8));
    }
    (void)read_mem(0x7002);
    BOARD_CHECK(cart_irq_pending());
    BOARD_CHECK(jy_store(0x7002, 0) == 0 && !cart_irq_pending());
    write_mem(0xE001, 0);
    for (unsigned edge = 0; edge < 10; ++edge) BOARD_CHECK(jy_cpu_edge(3) == 0 && !cart_irq_pending());
    write_mem(0x7FFF, 0);
    for (unsigned edge = 0; edge < 4; ++edge) BOARD_CHECK(jy_cpu_edge(3) == 0 && !cart_irq_pending());
    cpu_soft_reset(&cpu);
    for (unsigned edge = 0; edge < 4; ++edge) {
        BOARD_CHECK(jy_cpu_edge(3) == 0);
        BOARD_CHECK(cart_irq_pending() == (edge == 3));
    }
    write_mem(0x7002, 0);
    write_mem(0x7003, 0);
    for (unsigned edge = 0; edge < 4; ++edge) BOARD_CHECK(jy_cpu_edge(3) == 0 && !cart_irq_pending());
    write_mem(0x7003, 0);
    for (unsigned edge = 1; edge <= 8; ++edge) {
        BOARD_CHECK(jy_cpu_edge(3) == 0);
        BOARD_CHECK(cart_irq_pending() == (edge == 8));
    }
    BOARD_CHECK(board_image_load(&image) == 0 && !cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static int test_jy_rendered_irq(void) {
    const unsigned ids[] = {35, 91};
    bool startup = ppu_startup_write_restriction_enabled();
    ppu_set_startup_write_restriction(false);
    for (unsigned board = 0; board < sizeof(ids) / sizeof(ids[0]); ++board) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[board], 0x8000, 0, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        if (ids[board] == 35) {
            write_mem(0xC005, 1);
            BOARD_CHECK(jy_store(0xC003, 0) == 0);
        } else BOARD_CHECK(jy_store(0x7003, 0) == 0);
        BOARD_CHECK(jy_store(0x2000, 8) == 0);
        BOARD_CHECK(jy_store(0x2001, 0x18) == 0);
        for (unsigned step = 0; step < 2000 && !cart_irq_pending(); ++step)
            BOARD_CHECK(jy_cpu_cycles(2) == 0);
        BOARD_CHECK(cart_irq_pending());
        BOARD_CHECK(jy_store(0x2001, 0) == 0);
        board_image_free(&image);
    }
    ppu_set_startup_write_restriction(startup);
    return 0;
}

static int test_jy_geometry(void) {
    const unsigned ids[] = {35, 91};
    for (unsigned board = 0; board < sizeof(ids) / sizeof(ids[0]); ++board) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[board], 0x6000, 0x2800, true));
        image.data[8] |= 0xF0;
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(read_mem(0xE000) == 4);
        uint16_t prg_reg = ids[board] == 35 ? 0x8000 : 0x7000;
        uint16_t chr_reg = ids[board] == 35 ? 0x9000 : 0x6000;
        write_mem(prg_reg, 5);
        write_mem(chr_reg, 7);
        uint8_t chr = ids[board] == 35 ? 7 : 4;
        BOARD_CHECK(read_mem(0x8000) == 4 && ppu_read(0x0023) == chr);
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
        BOARD_CHECK(read_mem(0x8000) == 4 && ppu_read(0x0023) == chr);
        image.data[7] |= 2;
        BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
        BOARD_CHECK(read_mem(0x8000) == 4 && ppu_read(0x0023) == chr);
        image.data[0] = 0;
        BOARD_CHECK(load_rom_memory(image.data, image.size) < 0 && ppu_read(0x0023) == chr);
        board_image_free(&image);
        BOARD_CHECK(board_image_create(&image, ids[board], 0x2000, 0x100, true));
        BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0xFFFF) == 1);
        unsigned slot = ids[board] == 35 ? 7 : 3;
        write_mem((uint16_t)(chr_reg + slot), 0xFF);
        BOARD_CHECK(ppu_read((uint16_t)(slot * 0x100 + 0x23)) == 0 && ppu_read(0x1823) == 0x23);
        write_mem(0x4018, 0xA5);
        BOARD_CHECK(read_mem(0x6000) == 0xA5);
        board_image_free(&image);
    }
    return 0;
}

static int test_jy_persistence(void) {
    const unsigned ids[] = {35, 91};
    for (unsigned board = 0; board < sizeof(ids) / sizeof(ids[0]); ++board) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[board], 0x10000, 0, true));
        image.data[6] |= 2;
        image.data[10] = image.data[11] = 0x70;
        BOARD_CHECK(board_image_add_trainer(&image, 0x65));
        char path[128], save[128], chr_save[136];
        snprintf(path, sizeof(path), "build/board-jy-%u-%lu-%lu.nes", ids[board],
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
        uint8_t saved[0x2000] = {0};
        saved[0x1000] = 0x76;
        file = fopen(save, "wb");
        BOARD_CHECK(file != NULL);
        count = fwrite(saved, 1, sizeof(saved), file);
        closed = fclose(file);
        BOARD_CHECK(count == sizeof(saved) && closed == 0);
        BOARD_CHECK(load_rom(path) == 0);
        ppu_power_on(&ppu);
        BOARD_CHECK(cpu_power_on(&cpu) && read_mem(0x7000) == 0x76);
        BOARD_CHECK(jy_store(0x7000, 0xAB) == 0);
        ppu_write(0x0023, 0xA5);
        uint16_t chr_reg = ids[board] == 35 ? 0x9000 : 0x6000;
        write_mem(chr_reg, 3);
        ppu_write(0x0023, 0xB6);
        BOARD_CHECK(unload_rom());
        BOARD_CHECK(load_rom(path) == 0 && ppu_read(0x0023) == 0xA5);
        BOARD_CHECK(read_mem(0x7000) == (ids[board] == 35 ? 0xAB : 0x76));
        write_mem(chr_reg, 3);
        BOARD_CHECK(ppu_read(0x0023) == 0xB6 && unload_rom());
        file = fopen(save, "rb");
        BOARD_CHECK(file != NULL);
        count = fread(saved, 1, sizeof(saved), file);
        closed = fclose(file);
        BOARD_CHECK(count == sizeof(saved) && closed == 0);
        BOARD_CHECK(saved[0x1000] == (ids[board] == 35 ? 0xAB : 0x76));
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

int test_board_jy_small_accuracy(void) {
    int failures = 0;
    failures += test_jy35_banks();
    failures += test_jy91_banks();
    failures += test_jy35_irq();
    failures += test_jy91_irq();
    failures += test_jy_rendered_irq();
    failures += test_jy_geometry();
    failures += test_jy_persistence();
    unload_rom();
    printf("JY 35/91 accuracy: 7 groups, %d failures\n", failures);
    return failures;
}
