/*
 * board_unlicensed_113_accuracy.c - Cartridge latches, DIP reads, timers, and DAC writes
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
#include "../rom/board.h"
#include "../apu/apu.h"
#include <time.h>

static int store113(uint16_t address, uint8_t value) {
    const uint8_t code[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(code); ++byte) write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int load113(uint16_t address, uint8_t expected) {
    write_mem(0x0200, 0xAD);
    write_mem(0x0201, (uint8_t)address);
    write_mem(0x0202, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 4 && cpu.a == expected);
    return 0;
}

static int nops113(unsigned count) {
    write_mem(0x0300, 0xEA);
    for (unsigned step = 0; step < count; ++step) {
        cpu.pc = 0x0300;
        BOARD_CHECK(cpu_step(&cpu) == 2);
    }
    return 0;
}

static int mirror113(Mirroring expected) {
    BOARD_CHECK(cart_get_mirroring() == expected);
    ppu_write(0x2000, 0xA5);
    if (expected == MIRROR_HORIZONTAL || expected == MIRROR_VERTICAL) {
        ppu_write(expected == MIRROR_HORIZONTAL ? 0x2800 : 0x2400, 0x5A);
        BOARD_CHECK(ppu_read(expected == MIRROR_HORIZONTAL ? 0x2400 : 0x2800) == 0xA5);
        BOARD_CHECK(ppu_read(0x2C00) == 0x5A);
    } else {
        BOARD_CHECK(ppu_read(0x2400) == 0xA5 && ppu_read(0x2800) == 0xA5 && ppu_read(0x2C00) == 0xA5);
    }
    BOARD_CHECK(ppu_read(0x3000) == 0xA5);
    return 0;
}

static int test_234_read_latches_and_conflicts(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 234, 0x80000, 0x80000, true));
    uint8_t *prg = image.data + 16;
    prg[0x7F80] = 0x82;
    prg[2 * 0x8000 + 0x7FE8] = 0x71;
    prg[2 * 0x8000 + 0x7F9F] = 0xFF;
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0);
    BOARD_CHECK(load113(0xFF7F, 7) == 0 && read_mem(0x8000) == 0);
    BOARD_CHECK(load113(0xFF80, 0x82) == 0 && read_mem(0x8000) == 16 && ppu_read(0) == 64);
    BOARD_CHECK(mirror113(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(load113(0xFFE8, 0x71) == 0 && read_mem(0x8000) == 16 && ppu_read(0) == 88);
    BOARD_CHECK(store113(0xFF9F, 0) == 0 && read_mem(0x8000) == 16);
    BOARD_CHECK(store113(0xFFE7, 0) == 0 && ppu_read(0) == 88);
    BOARD_CHECK(store113(0xFFF9, 0) == 0 && ppu_read(0) == 88);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 16 && ppu_read(0) == 88);

    prg[0x7F80] = 0x40;
    prg[0x7F81] = 0x46;
    prg[6 * 0x8000 + 0x7FE8] = 0x71;
    prg[7 * 0x8000 + 0x7FE8] = 0x11;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(store113(0xFF80, 0xFF) == 0 && read_mem(0x8000) == 0 && ppu_read(0) == 0);
    BOARD_CHECK(store113(0xFF81, 0xFF) == 0 && read_mem(0x8000) == 48 && ppu_read(0) == 192);
    BOARD_CHECK(load113(0xFFE8, 0x71) == 0 && read_mem(0x8000) == 56 && ppu_read(0) == 248);
    BOARD_CHECK(store113(0xFFE8, 0xFF) == 0 && read_mem(0x8000) == 56 && ppu_read(0) == 200);
    BOARD_CHECK(mirror113(MIRROR_VERTICAL) == 0);
    board_image_free(&image);
    return 0;
}

static int test_235_chip_select_and_open_bus(void) {
    const size_t sizes[] = {0x100000, 0x200000, 0x400000, 0x800000};
    const int expected[4][4] = {{0, -1, -1, -1}, {0, -1, 64, -1}, {0, -1, 64, 128}, {0, 64, 128, 192}};
    for (unsigned shape = 0; shape < 4; ++shape) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, 235, sizes[shape], 0x2000, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        for (unsigned chip = 0; chip < 4; ++chip) {
            BOARD_CHECK(store113((uint16_t)(0x8000 + chip * 0x100), 0xFF) == 0);
            if (expected[shape][chip] < 0) {
                BOARD_CHECK(cart_cpu_read_bus(0x8000, 0xA5) == 0xA5);
                BOARD_CHECK(cart_cpu_read_bus(0xFFFF, 0x5A) == 0x5A);
            } else {
                unsigned page = (unsigned)expected[shape][chip] * 4;
                BOARD_CHECK(read_mem(0x8000) == (uint8_t)page && read_mem(0x8001) == page / 256);
                BOARD_CHECK(read_mem(0xC000) == (uint8_t)(page + 4));
            }
        }
        BOARD_CHECK(store113(0x9813, 0) == 0);
        BOARD_CHECK(read_mem(0x8000) == 156 && read_mem(0xC000) == 156 && read_mem(0xFFFF) == 159);
        BOARD_CHECK(store113(0xA400, 0) == 0 && mirror113(MIRROR_SINGLE0) == 0);
        BOARD_CHECK(store113(0xA000, 0) == 0 && mirror113(MIRROR_HORIZONTAL) == 0);
        BOARD_CHECK(store113(0x8100, 0) == 0);
        cpu_soft_reset(&cpu);
        BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 4 && ppu_read(0x1FFF) == 7);
        board_image_free(&image);
    }
    return 0;
}

static int test_236_chr_outer_and_dips(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 236, 0x80000, 0x10000, true));
    for (unsigned nibble = 0; nibble < 16; ++nibble)
        image.data[16 + 3 * 0x4000 + 0x120 + nibble] = (uint8_t)(0xA0 + nibble);
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0xC000) == 28);
    BOARD_CHECK(store113(0x8025, 0) == 0 && ppu_read(0) == 40 && mirror113(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store113(0xC035, 0xFF) == 0 && read_mem(0x8000) == 20 && read_mem(0xC000) == 20);
    BOARD_CHECK(store113(0xC025, 0) == 0 && read_mem(0x8000) == 16 && read_mem(0xC000) == 20);
    BOARD_CHECK(store113(0xC013, 0) == 0 && cart_set_dip_switches(10));
    BOARD_CHECK(load113(0x8123, 0xAA) == 0 && read_mem(0x812F) == 0xAA);
    BOARD_CHECK(cart_set_dip_switches(5) && read_mem(0x8120) == 0xA5);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8123) == 0xA3 && read_mem(0xC000) == 28 && ppu_read(0) == 40);
    BOARD_CHECK(cart_set_dip_switches(0));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 236, 0x80000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    ppu_write(0x123, 0x5A);
    BOARD_CHECK(store113(0x8003, 0xFF) == 0 && read_mem(0x8000) == 96 && read_mem(0xC000) == 124);
    BOARD_CHECK(store113(0xC032, 0) == 0 && read_mem(0x8000) == 104 && read_mem(0xC000) == 104);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 104 && read_mem(0xC000) == 104 && ppu_read(0x123) == 0x5A);
    BOARD_CHECK(store113(0xC002, 0) == 0 && read_mem(0x8000) == 8 && read_mem(0xC000) == 28);
    board_image_free(&image);
    return 0;
}

static int test_240_241_244_bank_wiring(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 240, 0x80000, 0x20000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(store113(0x4020, 0xA5) == 0 && read_mem(0x8000) == 80 && ppu_read(0) == 40);
    BOARD_CHECK(store113(0x401F, 0) == 0 && read_mem(0x8000) == 80);
    BOARD_CHECK(store113(0x6000, 0x5A) == 0 && read_mem(0x6000) == 0x5A && read_mem(0x8000) == 80);
    BOARD_CHECK(store113(0x5FFF, 0x3C) == 0 && read_mem(0x8000) == 24 && ppu_read(0) == 96);
    BOARD_CHECK(store113(0xFFFF, 0) == 0 && read_mem(0x8000) == 24);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 24 && ppu_read(0) == 96 && mirror113(MIRROR_HORIZONTAL) == 0);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 241, 0x800000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(store113(0xFFFF, 0xA5) == 0);
    BOARD_CHECK(read_mem(0x8000) == 40 && read_mem(0x8001) == 5 && read_mem(0xFFFF) == 47);
    BOARD_CHECK(ppu_read(0x1FFF) == 7 && store113(0x7FFF, 0) == 0 && read_mem(0x8001) == 5);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8001) == 5);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 244, 0x20000, 0x10000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    const uint8_t writes[] = {0x03, 0x13, 0x21, 0x31, 0x09, 0x19, 0x2A, 0x3A, 0x49, 0x59, 0x69, 0x7E};
    const uint8_t results[] = {24, 0, 16, 8, 8, 16, 32, 8, 32, 16, 48, 8};
    for (unsigned choice = 0; choice < sizeof(writes); ++choice) {
        BOARD_CHECK(store113((uint16_t)(0x8000 + choice * 0x777), writes[choice]) == 0);
        BOARD_CHECK((choice < 4 ? read_mem(0x8000) : ppu_read(0)) == results[choice]);
        BOARD_CHECK(choice < 4 ? ppu_read(0) == 0 : read_mem(0x8000) == 8);
    }
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 8 && ppu_read(0) == 8);
    board_image_free(&image);
    return 0;
}

static int test_246_register_overlay(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 246, 0x80000, 0x20000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_add_trainer(&image, 0x3E));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0xE000) == 126);
    BOARD_CHECK(cart_cpu_read_bus(0x8000, 0xA5) == 0xA5 && ppu_read(0x123) == 0x23);
    BOARD_CHECK(read_mem(0x7000) == 0x3E && store113(0x67F8, 5) == 0 && read_mem(0x8000) == 10);
    BOARD_CHECK(read_mem(0x67F8) == 0 && store113(0x67FB, 9) == 0 && read_mem(0xE000) == 18);
    BOARD_CHECK(store113(0x6800, 0x5A) == 0 && read_mem(0x6800) == 0x5A && read_mem(0x8000) == 10);
    for (unsigned slot = 0; slot < 4; ++slot) {
        BOARD_CHECK(store113((uint16_t)(0x67FC + slot), (uint8_t)(slot * 3 + 5)) == 0);
        BOARD_CHECK(ppu_read((uint16_t)(slot * 0x800)) == (slot * 3 + 5) * 2);
    }
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 10 && read_mem(0xE000) == 126 && ppu_read(0) == 10);
    BOARD_CHECK(read_mem(0x7000) == 0x3E && read_mem(0x6800) == 0x5A);
    board_image_free(&image);
    return 0;
}

static int test_255_261_265_address_modes(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 255, 0x200000, 0x100000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(store113(0xD143, 0) == 0 && read_mem(0x8000) == 20 && read_mem(0x8001) == 1);
    BOARD_CHECK(read_mem(0xC000) == 20 && ppu_read(0) == 24 && ppu_read(1) == 2);
    BOARD_CHECK(store113(0xE143, 0xFF) == 0 && read_mem(0x8000) == 16 && read_mem(0xC000) == 20);
    BOARD_CHECK(mirror113(MIRROR_HORIZONTAL) == 0);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 261, 0x80000, 0x20000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0 && read_mem(0xC000) == 0);
    BOARD_CHECK(store113(0x82B9, 0) == 0 && read_mem(0x8000) == 44 && read_mem(0xC000) == 44);
    BOARD_CHECK(ppu_read(0) == 72 && mirror113(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store113(0x82C5, 0xFF) == 0 && read_mem(0x8000) == 40 && read_mem(0xC000) == 44);
    BOARD_CHECK(ppu_read(0) == 40 && mirror113(MIRROR_VERTICAL) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 0 && ppu_read(0) == 0);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 265, 0x100000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0xC000) == 28);
    ppu_write(0x123, 0x5A);
    BOARD_CHECK(store113(0x8162, 3) == 0 && read_mem(0x8000) == 236 && read_mem(0xC000) == 252);
    BOARD_CHECK(store113(0xA0A2, 5) == 0 && read_mem(0x8000) == 52 && read_mem(0xC000) == 52);
    BOARD_CHECK(store113(0x8000, 2) == 0 && read_mem(0x8000) == 40 && read_mem(0xC000) == 40);
    BOARD_CHECK(mirror113(MIRROR_HORIZONTAL) == 0 && ppu_read(0x123) == 0x5A);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(store113(0x8100, 7) == 0 && read_mem(0x8000) == 60 && read_mem(0xC000) == 60);
    BOARD_CHECK(board_image_load(&image) == 0 && store113(0x8100, 7) == 0 && read_mem(0x8000) == 156);
    board_image_free(&image);
    return 0;
}

static int test_yoko_registers_and_cpu_timer(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 264, 0x40000, 0x20000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0xC000) == 60 && read_mem(0xE000) == 62);
    BOARD_CHECK(cart_set_dip_switches(2));
    BOARD_CHECK(cart_cpu_read_bus(0x5000, 0xA5) == 0xA6 && cart_cpu_read_bus(0x53FF, 0x5A) == 0x5A);
    BOARD_CHECK(store113(0x5403, 0xD7) == 0 && read_mem(0x5FFF) == 0xD7);
    BOARD_CHECK(store113(0x53FF, 0) == 0 && read_mem(0x5FFF) == 0xD7);
    BOARD_CHECK(store113(0xF0E8, 5) == 0 && read_mem(0x8000) == 20 && read_mem(0xC000) == 60);
    BOARD_CHECK(store113(0x8400, 9) == 0 && read_mem(0x8000) == 16 && read_mem(0xE000) == 22);
    BOARD_CHECK(mirror113(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store113(0x8000, 8) == 0 && store113(0x8400, 0x10) == 0);
    BOARD_CHECK(store113(0xFC08, 3) == 0 && store113(0xFC09, 5) == 0 && store113(0xFC0A, 7) == 0);
    BOARD_CHECK(read_mem(0x8000) == 38 && read_mem(0xA000) == 42 && read_mem(0xC000) == 46 && read_mem(0xE000) == 62);
    const uint16_t chr_regs[] = {0x8C10, 0x8C11, 0x8C16, 0x8C17};
    for (unsigned slot = 0; slot < 4; ++slot) {
        BOARD_CHECK(store113(chr_regs[slot], (uint8_t)(slot * 4 + 3)) == 0);
        BOARD_CHECK(ppu_read((uint16_t)(slot * 0x800)) == (slot * 4 + 3) * 2);
    }
    write_mem(0x8400, 0x90);
    write_mem(0x8800, 8);
    BOARD_CHECK(store113(0x8801, 0) == 0);
    BOARD_CHECK(nops113(3) == 0 && !cart_irq_pending());
    BOARD_CHECK(nops113(1) == 0 && cart_irq_pending());
    write_mem(0x8800, 0);
    BOARD_CHECK(!cart_irq_pending() && nops113(20) == 0 && !cart_irq_pending());
    write_mem(0x8801, 0);
    BOARD_CHECK(nops113(32767) == 0 && !cart_irq_pending());
    BOARD_CHECK(nops113(1) == 0 && cart_irq_pending());
    cpu_soft_reset(&cpu);
    BOARD_CHECK(!cart_irq_pending() && read_mem(0x8000) == 38 && read_mem(0x5FFF) == 0xD7);
    BOARD_CHECK(store113(0x8C10, 9) == 0 && read_mem(0x8000) == 0 && read_mem(0xC000) == 60);
    write_mem(0x8800, 0);
    write_mem(0x8801, 0);
    BOARD_CHECK(nops113(10) == 0 && !cart_irq_pending() && cart_set_dip_switches(0));
    board_image_free(&image);
    return 0;
}

static int test_city_fighter_banks_dac_and_irq(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 266, 0x20000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0 && read_mem(0xC000) == 0 && read_mem(0xE000) == 6);
    BOARD_CHECK(store113(0x9000, 9) == 0 && read_mem(0x8000) == 16 && read_mem(0xC000) == 16);
    BOARD_CHECK(store113(0xCFFF, 1) == 0 && read_mem(0xC000) == 20 && read_mem(0xE000) == 22);
    BOARD_CHECK(mirror113(MIRROR_HORIZONTAL) == 0);
    const uint16_t chr_regs[] = {0xD000, 0xD008, 0xA000, 0xA008, 0xB000, 0xB008, 0xE000, 0xE008};
    for (unsigned slot = 0; slot < 8; ++slot) {
        BOARD_CHECK(store113(chr_regs[slot], (uint8_t)(slot + 1)) == 0);
        BOARD_CHECK(store113((uint16_t)(chr_regs[slot] + 4), (uint8_t)(slot + 3)) == 0);
        BOARD_CHECK(ppu_read((uint16_t)(slot * 0x400)) == (slot + 3) * 16 + slot + 1);
    }
    apu_power_on(&apu);
    BOARD_CHECK(store113(0x9804, 0x1B) == 0 && apu.dmc.output_level == 88 && read_mem(0x8000) == 16);
    BOARD_CHECK(store113(0x9008, 4) == 0 && apu.dmc.output_level == 88 && read_mem(0x8000) == 8);
    BOARD_CHECK(store113(0x9FFC, 0x17) == 0 && apu.dmc.output_level == 56);
    BOARD_CHECK(store113(0x9000, 6) == 0 && mirror113(MIRROR_SINGLE0) == 0);
    BOARD_CHECK(store113(0x9000, 7) == 0 && mirror113(MIRROR_SINGLE1) == 0);
    write_mem(0xF000, 4);
    write_mem(0xF004, 0);
    BOARD_CHECK(store113(0xF008, 2) == 0);
    BOARD_CHECK(nops113(3) == 0 && !cart_irq_pending());
    BOARD_CHECK(nops113(1) == 0 && cart_irq_pending());
    write_mem(0xF008, 2);
    BOARD_CHECK(!cart_irq_pending() && nops113(32767) == 0 && !cart_irq_pending());
    BOARD_CHECK(nops113(1) == 0 && cart_irq_pending());
    cpu_soft_reset(&cpu);
    BOARD_CHECK(!cart_irq_pending() && read_mem(0x8000) == 8 && read_mem(0xC000) == 12 && ppu_read(0) == 49);
    write_mem(0xF008, 0);
    BOARD_CHECK(nops113(10) == 0 && !cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static int test_discrete113_geometry_and_replacement(void) {
    const unsigned ids[] = {234, 235, 236, 240, 241, 244, 246, 255, 261, 264, 265, 266};
    for (unsigned item = 0; item < sizeof(ids) / sizeof(ids[0]); ++item) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[item], 0x6000, 0x600, true));
        image.data[8] |= 0xF0;
        memset(image.data + 16, 0xA7, 0x6000);
        memset(image.data + 16 + 0x6000, 0xB8, 0x600);
        BOARD_CHECK(board_image_load(&image) == 0);
        uint16_t reg = ids[item] == 234 ? 0xFF80 : ids[item] == 240 ? 0x4020 : ids[item] == 246 ? 0x6000 : 0x8000;
        BOARD_CHECK(store113(reg, 0xFF) == 0);
        if (ids[item] == 246) BOARD_CHECK(store113(0x6004, 0xFF) == 0);
        BOARD_CHECK(read_mem(0x8000) == 0xA7 && ppu_read(0x123) == 0xB8);
        ppu_write(0x123, 0xCC);
        BOARD_CHECK(ppu_read(0x123) == 0xB8);
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
        BOARD_CHECK(read_mem(0x8000) == 0xA7 && ppu_read(0x123) == 0xB8);
        board_image_set_unsupported_console(&image);
        BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
        BOARD_CHECK(read_mem(0x8000) == 0xA7 && ppu_read(0x123) == 0xB8);
        board_image_free(&image);
        BOARD_CHECK(board_image_create(&image, ids[item], 0x6000, 0, true));
        image.data[11] = 3;
        BOARD_CHECK(board_image_load(&image) == 0);
        ppu_write(0x123, 0x5A);
        BOARD_CHECK(ppu_read(0x1F23) == 0x5A);
        BOARD_CHECK(store113(reg, 0xFF) == 0);
        ppu_write(0x1F23, 0xA5);
        BOARD_CHECK(ppu_read(0x123) == 0xA5);
        board_image_free(&image);
    }
    return 0;
}

static int test_246_persistent_ram(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 246, 0x80000, 0, true));
    image.data[6] |= 2;
    image.data[10] = 0x70;
    image.data[11] = 0x77;
    BOARD_CHECK(board_image_add_trainer(&image, 0x3E));
    char stem[96], path[112], save[112], chr_save[120];
    snprintf(stem, sizeof(stem), "build/discrete113-%lu-%lu", (unsigned long)time(NULL), (unsigned long)clock());
    snprintf(path, sizeof(path), "%s.nes", stem);
    snprintf(save, sizeof(save), "%s.sav", stem);
    snprintf(chr_save, sizeof(chr_save), "%s.chr.sav", stem);
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t written = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(written == image.size && closed == 0 && load_rom(path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu) && read_mem(0x7000) == 0x3E);
    BOARD_CHECK(store113(0x7000, 0xA5) == 0 && store113(0x6004, 5) == 0);
    ppu_write(0x123, 0x6D);
    BOARD_CHECK(store113(0x6004, 0) == 0);
    ppu_write(0x123, 0x7C);
    BOARD_CHECK(read_mem(0x6004) == 0 && load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x7000) == 0xA5 && ppu_read(0x123) == 0x7C);
    BOARD_CHECK(unload_rom() && load_rom(path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu) && read_mem(0x7000) == 0xA5 && ppu_read(0x123) == 0);
    BOARD_CHECK(store113(0x6004, 5) == 0 && ppu_read(0x123) == 0x6D);
    BOARD_CHECK(unload_rom());
    file = fopen(save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0 && ftell(file) == 0x2000);
    BOARD_CHECK(fseek(file, 0x1000, SEEK_SET) == 0 && fgetc(file) == 0xA5 && fclose(file) == 0);
    file = fopen(chr_save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0 && ftell(file) == 0x2000);
    BOARD_CHECK(fseek(file, 0x923, SEEK_SET) == 0 && fgetc(file) == 0x6D && fclose(file) == 0);
    BOARD_CHECK(remove(path) == 0 && remove(save) == 0 && remove(chr_save) == 0);
    board_image_free(&image);
    return 0;
}

static int test_discrete113_invalid_buffers(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 244, 0x20000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && store113(0x8000, 3) == 0);
    iNESHeader header;
    memcpy(&header, image.data, sizeof(header));
    uint8_t *prg = image.data + sizeof(header);
    uint8_t *chr = prg + 0x20000;
    BOARD_CHECK(board_create(NULL, prg, 0x20000, chr, 0x2000) == NULL);
    BOARD_CHECK(board_create(&header, NULL, 0x20000, chr, 0x2000) == NULL);
    BOARD_CHECK(board_create(&header, prg, 0, chr, 0x2000) == NULL);
    BOARD_CHECK(board_create(&header, prg, 0x20000, NULL, 0x2000) == NULL);
    BOARD_CHECK(board_create(&header, prg, 0x20000, chr, 0) == NULL);
#if SIZE_MAX > UINT32_MAX
    BOARD_CHECK(board_create(&header, prg, (size_t)UINT32_MAX + 1, chr, 0x2000) == NULL);
    BOARD_CHECK(board_create(&header, prg, 0x20000, chr, (size_t)UINT32_MAX + 1) == NULL);
#endif
    BOARD_CHECK(read_mem(0x8000) == 24 && ppu_read(0x1FFF) == 7);
    board_image_free(&image);
    return 0;
}

int test_board_unlicensed_113_accuracy(void) {
    int failures = 0;
    failures += test_234_read_latches_and_conflicts();
    failures += test_235_chip_select_and_open_bus();
    failures += test_236_chr_outer_and_dips();
    failures += test_240_241_244_bank_wiring();
    failures += test_246_register_overlay();
    failures += test_255_261_265_address_modes();
    failures += test_yoko_registers_and_cpu_timer();
    failures += test_city_fighter_banks_dac_and_irq();
    failures += test_discrete113_geometry_and_replacement();
    failures += test_246_persistent_ram();
    failures += test_discrete113_invalid_buffers();
    unload_rom();
    printf("Discrete cartridge group 113: 11 groups, %d failures\n", failures);
    return failures;
}
