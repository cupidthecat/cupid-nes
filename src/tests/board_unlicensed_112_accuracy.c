/*
 * board_unlicensed_112_accuracy.c - Multicart latches, reset modes, and PPU IRQs
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

static int store112(uint16_t address, uint8_t value) {
    const uint8_t code[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(code); ++byte) write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int nops112(unsigned count) {
    write_mem(0x0300, 0xEA);
    for (unsigned instruction = 0; instruction < count; ++instruction) {
        cpu.pc = 0x0300;
        BOARD_CHECK(cpu_step(&cpu) == 2);
    }
    return 0;
}

static int mirror112(Mirroring expected) {
    BOARD_CHECK(cart_get_mirroring() == expected);
    ppu_write(0x2000, 0xA5);
    ppu_write(expected == MIRROR_HORIZONTAL ? 0x2800 : 0x2400, 0x5A);
    BOARD_CHECK(ppu_read(expected == MIRROR_HORIZONTAL ? 0x2400 : 0x2800) == 0xA5);
    BOARD_CHECK(ppu_read(0x2C00) == 0x5A && ppu_read(0x3000) == 0xA5);
    return 0;
}

static int test_213_214_216_addresses(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 213, 0x20000, 0x10000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0);
    BOARD_CHECK(store112(0xFFDE, 0) == 0);
    BOARD_CHECK(read_mem(0x8000) == 24 && read_mem(0xFFFF) == 31 && ppu_read(0) == 24);
    BOARD_CHECK(store112(0x801C, 0xFF) == 0);
    BOARD_CHECK(read_mem(0x8000) == 16 && read_mem(0xFFFF) == 23 && ppu_read(0) == 24);
    BOARD_CHECK(store112(0x7FFF, 0) == 0 && read_mem(0x8000) == 16);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 16 && ppu_read(0) == 24 && mirror112(MIRROR_HORIZONTAL) == 0);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 214, 0x10000, 0x8000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0xC000) == 0);
    BOARD_CHECK(store112(0xFFFE, 0) == 0);
    BOARD_CHECK(read_mem(0x8000) == 12 && read_mem(0xC000) == 12 && ppu_read(0) == 16);
    BOARD_CHECK(store112(0xFFF9, 0xFF) == 0);
    BOARD_CHECK(read_mem(0x8000) == 8 && read_mem(0xFFFF) == 11 && ppu_read(0) == 8);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 8 && ppu_read(0) == 8 && mirror112(MIRROR_HORIZONTAL) == 0);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 216, 0x10000, 0x10000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(store112(0xFFFF, 0xFF) == 0);
    BOARD_CHECK(read_mem(0x8000) == 8 && read_mem(0xFFFF) == 15 && ppu_read(0) == 56);
    write_mem(0x4018, 0xA5);
    BOARD_CHECK(read_mem(0x5000) == 0);
    write_mem(0x4018, 0xA5);
    BOARD_CHECK(read_mem(0x5001) == 0xA5);
    BOARD_CHECK(store112(0x5001, 0) == 0 && read_mem(0x8000) == 8 && ppu_read(0) == 56);
    BOARD_CHECK(store112(0x5000, 0xFF) == 0 && read_mem(0x8000) == 0 && ppu_read(0) == 0);
    BOARD_CHECK(store112(0x6123, 0x5A) == 0 && read_mem(0x6123) == 0x5A);
    BOARD_CHECK(store112(0x800B, 0) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 8 && ppu_read(0) == 40 && read_mem(0x6123) == 0x5A);
    board_image_free(&image);
    return 0;
}

static void address222(uint16_t address, uint32_t frame_cycle) {
    ppu.scanline = frame_cycle / 341 ? (int)(frame_cycle / 341) - 1 : 261;
    ppu.dot = frame_cycle % 341;
    cart_notify_ppu_address(address, frame_cycle);
}

static int test_222_banks_and_irq(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 222, 0x40000, 0x20000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cart_cpu_read_bus(0x8000, 0xA5) == 0xA5 && cart_cpu_read_bus(0xA000, 0x5A) == 0x5A);
    BOARD_CHECK(read_mem(0xC000) == 60 && read_mem(0xFFFF) == 63 && ppu_read(0x0123) == 0x23);
    BOARD_CHECK(store112(0x8FFC, 5) == 0 && store112(0xAFFC, 9) == 0);
    BOARD_CHECK(read_mem(0x8000) == 10 && read_mem(0xA000) == 18);
    const uint16_t registers[] = {0xBFFC, 0xBFFE, 0xCFFC, 0xCFFE, 0xDFFC, 0xDFFE, 0xEFFC, 0xEFFE};
    for (unsigned slot = 0; slot < 8; ++slot)
        BOARD_CHECK(store112(registers[slot], (uint8_t)(3 + slot * 2)) == 0);
    for (unsigned slot = 0; slot < 8; ++slot) {
        BOARD_CHECK(ppu_read((uint16_t)(slot * 0x400)) == 3 + slot * 2);
        ppu_write((uint16_t)(slot * 0x400), 0xCC);
        BOARD_CHECK(ppu_read((uint16_t)(slot * 0x400)) == 3 + slot * 2);
    }
    BOARD_CHECK(store112(0x9FFC, 1) == 0 && mirror112(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store112(0x9000, 0) == 0 && mirror112(MIRROR_VERTICAL) == 0);
    write_mem(0xF000, 239);
    address222(0x1000, 10);
    address222(0, 20);
    address222(0x1000, 29);
    BOARD_CHECK(!cart_irq_pending());
    address222(0, 40);
    address222(0x1000, 50);
    BOARD_CHECK(cart_irq_pending());
    write_mem(0xF003, 0);
    BOARD_CHECK(cart_irq_pending());
    write_mem(0xFFFC, 238);
    BOARD_CHECK(!cart_irq_pending());
    address222(0x1000, 90);
    address222(0, 100);
    address222(0, 105);
    address222(0x1000, 109);
    BOARD_CHECK(!cart_irq_pending());
    address222(0, 120);
    address222(0x1000, 130);
    BOARD_CHECK(!cart_irq_pending());
    address222(0, 140);
    address222(0x1000, 150);
    BOARD_CHECK(cart_irq_pending());
    write_mem(0xF000, 0);
    for (unsigned edge = 0; edge < 256; ++edge) {
        address222(0, 200 + edge * 20);
        address222(0x1000, 210 + edge * 20);
    }
    BOARD_CHECK(!cart_irq_pending());
    write_mem(0xF000, 255);
    address222(0, 5400);
    address222(0x1000, 5410);
    BOARD_CHECK(cart_irq_pending());
    write_mem(0xF000, 239);
    address222(0, 89337);
    address222(0x1000, 5);
    BOARD_CHECK(cart_irq_pending());
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cart_irq_pending() && read_mem(0x8000) == 10 && ppu_read(0) == 3);
    write_mem(0xF000, 0);
    board_image_free(&image);
    return 0;
}

static int test_225_outer_banks(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 225, 0x200000, 0x100000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0 && read_mem(0xC000) == 4);
    BOARD_CHECK(store112(0xD143, 0) == 0);
    BOARD_CHECK(read_mem(0x8000) == 20 && read_mem(0x8001) == 1 && read_mem(0xC000) == 20);
    BOARD_CHECK(ppu_read(0) == 24 && ppu_read(1) == 2 && mirror112(MIRROR_VERTICAL) == 0);
    BOARD_CHECK(store112(0xF143, 0xFF) == 0 && mirror112(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(read_mem(0x8000) == 20 && ppu_read(0) == 24);
    BOARD_CHECK(store112(0xC1C5, 0) == 0);
    BOARD_CHECK(read_mem(0x8000) == 24 && read_mem(0x8001) == 1 && read_mem(0xC000) == 28);
    BOARD_CHECK(ppu_read(0) == 40 && ppu_read(1) == 2);
    BOARD_CHECK(store112(0x81C5, 0xFF) == 0);
    BOARD_CHECK(read_mem(0x8000) == 24 && read_mem(0x8001) == 0 && read_mem(0xC000) == 28);
    BOARD_CHECK(ppu_read(0) == 40 && ppu_read(1) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 24 && ppu_read(0) == 40 && mirror112(MIRROR_VERTICAL) == 0);
    board_image_free(&image);
    return 0;
}

static int test_226_233_reset_latches(void) {
    BoardImage image;
    for (unsigned variant = 0; variant < 2; ++variant) {
        BOARD_CHECK(board_image_create(&image, variant ? 233 : 226, 0x200000, 0, true));
        image.data[11] = 7;
        BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0 && read_mem(0xC000) == 4);
        ppu_write(0x0123, 0x5A);
        BOARD_CHECK(store112(0xFFFF, 1) == 0 && store112(0xFFFE, 0xE5) == 0);
        BOARD_CHECK(read_mem(0x8000) == (variant ? 20 : 148) && read_mem(0x8001) == 1);
        BOARD_CHECK(read_mem(0xC000) == (variant ? 20 : 148) && mirror112(MIRROR_VERTICAL) == 0);
        BOARD_CHECK(store112(0x8000, 0xC5) == 0);
        BOARD_CHECK(read_mem(0x8000) == (variant ? 16 : 144) && read_mem(0xC000) == (variant ? 20 : 148));
        cpu_soft_reset(&cpu);
        BOARD_CHECK(read_mem(0x8000) == (variant ? 128 : 0) && read_mem(0xC000) == (variant ? 132 : 4));
        BOARD_CHECK(read_mem(0x8001) == 0 && ppu_read(0x0123) == 0x5A && mirror112(MIRROR_VERTICAL) == 0);
        BOARD_CHECK(store112(0x8000, variant ? 0xA3 : 0x25) == 0);
        BOARD_CHECK(read_mem(0x8000) == (variant ? 140 : 20) && read_mem(0x8001) == 0);
        BOARD_CHECK(mirror112(MIRROR_HORIZONTAL) == 0);
        if (variant) {
            BOARD_CHECK(store112(0x8001, 1) == 0 && read_mem(0x8000) == 140 && read_mem(0x8001) == 1);
            cpu_soft_reset(&cpu);
            BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 4 && mirror112(MIRROR_HORIZONTAL) == 0);
            cpu_soft_reset(&cpu);
            BOARD_CHECK(read_mem(0x8000) == 128 && read_mem(0xC000) == 132);
        }
        BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0 && read_mem(0xC000) == 4);
        board_image_free(&image);
    }
    return 0;
}

static int test_227_bank_modes(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 227, 0x100000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    const uint16_t addresses[] = {0x8000, 0x8080, 0x8081, 0x808D, 0x809C, 0x810C, 0x830D, 0x831E, 0x817F};
    const uint8_t lower[] = {0, 0, 0, 8, 28, 140, 136, 156, 248};
    const uint8_t upper[] = {0, 0, 4, 12, 28, 128, 156, 156, 224};
    for (unsigned mode = 0; mode < sizeof(lower); ++mode) {
        BOARD_CHECK(store112(addresses[mode], (uint8_t)(mode * 31)) == 0);
        BOARD_CHECK(read_mem(0x8000) == lower[mode] && read_mem(0xC000) == upper[mode]);
        BOARD_CHECK(read_mem(0xBFFF) == lower[mode] + 3 && read_mem(0xFFFF) == upper[mode] + 3);
        BOARD_CHECK(ppu_read(0x0123) == 0x23);
        BOARD_CHECK(mirror112(mode >= 7 ? MIRROR_HORIZONTAL : MIRROR_VERTICAL) == 0);
    }
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 248 && read_mem(0xC000) == 224);
    board_image_free(&image);
    BOARD_CHECK(board_image_create(&image, 227, 0x100000, 0, true));
    image.data[11] = 7;
    BOARD_CHECK(board_image_load(&image) == 0);
    ppu_write(0x0123, 0x5A);
    for (unsigned mode = 0; mode < sizeof(lower); ++mode) {
        BOARD_CHECK(store112(addresses[mode], 0xFF) == 0 && ppu_read(0x0123) == 0x5A);
        ppu_write(0x1123, (uint8_t)mode);
        BOARD_CHECK(ppu_read(0x1123) == mode);
    }
    board_image_free(&image);
    return 0;
}

static int test_228_chip_select(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 228, 0x180000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && mirror112(MIRROR_VERTICAL) == 0);
    const uint16_t addresses[] = {0x8000, 0x8800, 0x9000, 0x9800};
    const uint8_t low[] = {0, 128, 0, 0}, high[] = {0, 0, 1, 1};
    for (unsigned chip = 0; chip < 4; ++chip) {
        BOARD_CHECK(store112(addresses[chip], 0) == 0);
        BOARD_CHECK(read_mem(0x8000) == low[chip] && read_mem(0x8001) == high[chip]);
        BOARD_CHECK(read_mem(0xC000) == low[chip] + 4 && read_mem(0xC001) == high[chip]);
    }
    BOARD_CHECK(store112(0x98ED, 2) == 0);
    BOARD_CHECK(read_mem(0x8000) == 12 && read_mem(0x8001) == 1 && read_mem(0xC000) == 12);
    BOARD_CHECK(ppu_read(0) == 176 && ppu_read(1) == 1);
    BOARD_CHECK(store112(0xB8ED, 0xFE) == 0 && mirror112(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(read_mem(0x8000) == 12 && ppu_read(0) == 176);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 4 && ppu_read(0) == 0);
    BOARD_CHECK(mirror112(MIRROR_VERTICAL) == 0);
    board_image_free(&image);
    return 0;
}

static int test_229_230_231_modes(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 229, 0x80000, 0x200000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(store112(0xFFFF, 0) == 0);
    BOARD_CHECK(read_mem(0x8000) == 124 && read_mem(0xC000) == 124 && ppu_read(0) == 248 && ppu_read(1) == 7);
    BOARD_CHECK(mirror112(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store112(0x8021, 0xFF) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 4 && ppu_read(0) == 8 && ppu_read(1) == 1);
    BOARD_CHECK(store112(0x8001, 0) == 0 && ppu_read(0) == 8 && ppu_read(1) == 0);
    BOARD_CHECK(store112(0x8002, 0xFF) == 0 && read_mem(0x8000) == 8 && read_mem(0xC000) == 8);
    BOARD_CHECK(store112(0xFF80, 0) == 0 && ppu_read(0) == 0 && ppu_read(1) == 4);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 4 && ppu_read(1) == 4);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 230, 0xA0000, 0, true));
    image.data[11] = 7;
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0 && read_mem(0xC000) == 28);
    ppu_write(0x0123, 0x5A);
    BOARD_CHECK(store112(0xFFFF, 0xE5) == 0 && read_mem(0x8000) == 20 && read_mem(0xC000) == 28);
    BOARD_CHECK(mirror112(MIRROR_VERTICAL) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 32 && read_mem(0xC000) == 36 && mirror112(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store112(0x8000, 0x25) == 0 && read_mem(0x8000) == 52 && read_mem(0xC000) == 52);
    BOARD_CHECK(store112(0xFFFF, 0x45) == 0 && read_mem(0x8000) == 48 && read_mem(0xC000) == 52);
    BOARD_CHECK(mirror112(MIRROR_VERTICAL) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 28 && ppu_read(0x0123) == 0x5A);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 32 && read_mem(0xC000) == 36);
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0 && read_mem(0xC000) == 28);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 231, 0x80000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0 && read_mem(0xC000) == 0);
    BOARD_CHECK(store112(0x80AB, 0) == 0 && read_mem(0x8000) == 40 && read_mem(0xC000) == 44);
    BOARD_CHECK(mirror112(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store112(0x801E, 0xFF) == 0 && read_mem(0x8000) == 120 && read_mem(0xC000) == 120);
    BOARD_CHECK(store112(0x803E, 0) == 0 && read_mem(0x8000) == 120 && read_mem(0xC000) == 124);
    BOARD_CHECK(mirror112(MIRROR_VERTICAL) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 0 && ppu_read(0x1FFF) == 7);
    BOARD_CHECK(mirror112(MIRROR_VERTICAL) == 0);
    board_image_free(&image);
    return 0;
}

static int test_222_ppu_bus_and_rendering(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 222, 0x8000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    bool startup = ppu_startup_write_restriction_enabled();
    ppu_set_startup_write_restriction(false);
    write_mem(0xF000, 239);
    BOARD_CHECK(store112(0x2006, 0) == 0 && store112(0x2006, 0) == 0);
    BOARD_CHECK(nops112(8) == 0 && !cart_irq_pending());
    BOARD_CHECK(store112(0x2006, 0x10) == 0 && store112(0x2006, 0) == 0);
    BOARD_CHECK(nops112(3) == 0 && cart_irq_pending());
    BOARD_CHECK(board_image_load(&image) == 0);
    ppu_set_startup_write_restriction(false);
    write_mem(0xF000, 238);
    BOARD_CHECK(store112(0x2000, 8) == 0 && store112(0x2001, 0x18) == 0);
    unsigned elapsed = 0;
    while (!cart_irq_pending() && elapsed < 2000) {
        BOARD_CHECK(nops112(1) == 0);
        ++elapsed;
    }
    BOARD_CHECK(cart_irq_pending() && elapsed < 2000 && ppu.scanline < 20);
    write_mem(0xF000, 0);
    BOARD_CHECK(nops112(1000) == 0 && !cart_irq_pending());
    BOARD_CHECK(store112(0x2001, 0) == 0);
    ppu_set_startup_write_restriction(startup);
    board_image_free(&image);
    return 0;
}

static int test_discrete112_small_geometry(void) {
    const unsigned ids[] = {213, 214, 216, 222, 225, 226, 227, 228, 229, 230, 231, 233};
    for (unsigned index = 0; index < sizeof(ids) / sizeof(ids[0]); ++index) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[index], 0x6000, 0x600, true));
        image.data[8] |= 0xF0;
        memset(image.data + 16, 0xA7, 0x6000);
        memset(image.data + 16 + 0x6000, 0xB8, 0x600);
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(store112(ids[index] == 222 ? 0x8000 : 0xFFFF, 0xFF) == 0);
        if (ids[index] == 222) BOARD_CHECK(store112(0xB000, 0xFF) == 0);
        uint8_t pattern = ids[index] == 227 ? 0x23 : 0xB8;
        BOARD_CHECK(read_mem(0x8000) == 0xA7 && ppu_read(0x0123) == pattern);
        BOARD_CHECK(ppu_read(0x1F23) == 0x23);
        ppu_write(0x0123, 0xCC);
        BOARD_CHECK(ppu_read(0x0123) == pattern);
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
        BOARD_CHECK(read_mem(0x8000) == 0xA7 && ppu_read(0x0123) == pattern);
        image.data[7] |= 2;
        BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
        BOARD_CHECK(read_mem(0x8000) == 0xA7 && ppu_read(0x0123) == pattern);
        board_image_free(&image);

        BOARD_CHECK(board_image_create(&image, ids[index], 0x6000, 0, true));
        image.data[11] = 3;
        BOARD_CHECK(board_image_load(&image) == 0);
        ppu_write(0x0123, 0x5A);
        BOARD_CHECK(ppu_read(0x1F23) == 0x5A);
        BOARD_CHECK(store112(ids[index] == 222 ? 0xB000 : 0xFFFF, 0xFF) == 0);
        ppu_write(0x1F23, 0xA5);
        BOARD_CHECK(ppu_read(0x0123) == 0xA5);
        board_image_free(&image);
    }
    return 0;
}

static int test_228_persistent_ram(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 228, 0x40000, 0, true));
    image.data[6] |= 2;
    image.data[10] = 0x70;
    image.data[11] = 0x77;
    BOARD_CHECK(board_image_add_trainer(&image, 0x35));
    char stem[100], path[120], save[120], chr_save[128];
    snprintf(stem, sizeof(stem), "build/discrete112-%lu-%lu", (unsigned long)time(NULL), (unsigned long)clock());
    snprintf(path, sizeof(path), "%s.nes", stem);
    snprintf(save, sizeof(save), "%s.sav", stem);
    snprintf(chr_save, sizeof(chr_save), "%s.chr.sav", stem);
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t written = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(written == image.size && closed == 0 && load_rom(path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu) && read_mem(0x7000) == 0x35);
    BOARD_CHECK(store112(0x7000, 0xE8) == 0);
    ppu_write(0x0123, 0x5A);
    BOARD_CHECK(store112(0x8000, 1) == 0);
    ppu_write(0x0123, 0xC7);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(ppu_read(0x0123) == 0x5A && read_mem(0x7000) == 0xE8);
    BOARD_CHECK(store112(0x8000, 1) == 0 && ppu_read(0x0123) == 0xC7);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(ppu_read(0x0123) == 0xC7 && read_mem(0x7000) == 0xE8);
    BOARD_CHECK(unload_rom() && load_rom(path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu) && read_mem(0x7000) == 0xE8 && ppu_read(0x0123) == 0);
    BOARD_CHECK(store112(0x8000, 1) == 0 && ppu_read(0x0123) == 0xC7);
    BOARD_CHECK(unload_rom());
    file = fopen(save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0 && ftell(file) == 0x2000);
    BOARD_CHECK(fseek(file, 0x1000, SEEK_SET) == 0 && fgetc(file) == 0xE8 && fclose(file) == 0);
    file = fopen(chr_save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0 && ftell(file) == 0x2000);
    BOARD_CHECK(fseek(file, 0x0123, SEEK_SET) == 0 && fgetc(file) == 0xC7 && fclose(file) == 0);
    BOARD_CHECK(remove(path) == 0 && remove(save) == 0 && remove(chr_save) == 0);
    board_image_free(&image);
    return 0;
}

int test_board_unlicensed_112_accuracy(void) {
    int failures = 0;
    failures += test_213_214_216_addresses();
    failures += test_222_banks_and_irq();
    failures += test_225_outer_banks();
    failures += test_226_233_reset_latches();
    failures += test_227_bank_modes();
    failures += test_228_chip_select();
    failures += test_229_230_231_modes();
    failures += test_222_ppu_bus_and_rendering();
    failures += test_discrete112_small_geometry();
    failures += test_228_persistent_ram();
    unload_rom();
    printf("Discrete cartridge group 112: 10 groups, %d failures\n", failures);
    return failures;
}
