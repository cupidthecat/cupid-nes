/*
 * board_unlicensed_110_accuracy.c - Unlicensed board regressions for issue 110
 *
 * Copyright (C) 2026 Francis Hagan
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#include "board_tests.h"
#include <time.h>

static bool prg8_is(uint16_t address, unsigned bank) {
    return read_mem(address) == (uint8_t)(bank * 2)
        && read_mem((uint16_t)(address + 1)) == (uint8_t)(bank >> 7);
}

static bool prg16_is(uint16_t address, unsigned bank) {
    return read_mem(address) == (uint8_t)(bank * 4)
        && read_mem((uint16_t)(address + 1)) == (uint8_t)(bank >> 6);
}

static bool prg32_is(unsigned bank) {
    return read_mem(0x8000) == (uint8_t)(bank * 8)
        && read_mem(0x8001) == (uint8_t)(bank >> 5);
}

static bool chr1_is(uint16_t address, unsigned bank) {
    return ppu_read(address) == (uint8_t)bank
        && ppu_read((uint16_t)(address + 1)) == (uint8_t)(bank >> 8);
}

static bool chr8_is(unsigned bank) {
    return ppu_read(0) == (uint8_t)(bank * 8)
        && ppu_read(1) == (uint8_t)(bank >> 5);
}

static bool cpu_write_abs(uint16_t address, uint8_t value) {
    write_mem(0x0200, 0xA9); write_mem(0x0201, value);
    write_mem(0x0202, 0x8D); write_mem(0x0203, (uint8_t)address);
    write_mem(0x0204, (uint8_t)(address >> 8)); cpu.pc = 0x0200;
    return cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4;
}

static bool cpu_nop(void) {
    write_mem(0x0200, 0xEA); cpu.pc = 0x0200;
    return cpu_step(&cpu) == 2;
}

static bool advance_three_cpu_cycles(void) {
    write_mem(0x0200, 0x4C); write_mem(0x0201, 0); write_mem(0x0202, 2);
    cpu.pc = 0x0200; return cpu_step(&cpu) == 3;
}

static bool qualified_edge(void) {
    (void)ppu_read(0x1000);
    (void)ppu_read(0);
    if (!cpu_nop() || !cpu_nop()) return false;
    (void)ppu_read(0x1000);
    return true;
}

static int test_60_62(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 60, 0x10000, 0x8000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_is(0x8000, 0) && chr8_is(0));
    for (unsigned bank = 1; bank < 4; ++bank) {
        cpu_soft_reset(&cpu);
        BOARD_CHECK(prg16_is(0x8000, bank) && prg16_is(0xC000, bank) && chr8_is(bank));
    }
    cpu_soft_reset(&cpu); BOARD_CHECK(prg16_is(0x8000, 0) && chr8_is(0));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 62, 0x100000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8583, 2);
    BOARD_CHECK(prg16_is(0x8000, 4) && prg16_is(0xC000, 5));
    BOARD_CHECK(chr8_is(14) && cart_get_mirroring() == MIRROR_HORIZONTAL);
    write_mem(0x85A3, 1);
    BOARD_CHECK(prg16_is(0x8000, 5) && prg16_is(0xC000, 5) && chr8_is(13));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg16_is(0x8000, 0) && prg16_is(0xC000, 1) && chr8_is(0));
    board_image_free(&image);
    return 0;
}

static int test_83(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 83, 0x80000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cart_set_dip_switches(3));
    BOARD_CHECK(cart_cpu_read_bus(0x5000, 0xA4) == 0xA7);
    write_mem(0x5102, 0x5A); BOARD_CHECK(cart_cpu_read_bus(0x5102, 0) == 0x5A);
    write_mem(0x8000, 0x12);
    BOARD_CHECK(prg8_is(0x8000, 0x24) && prg8_is(0xA000, 0x25));
    BOARD_CHECK(prg8_is(0xC000, 0x3E) && prg8_is(0xE000, 0x3F));
    write_mem(0x8312, 7); BOARD_CHECK(chr1_is(0x0800, 0x107));
    write_mem(0x8300, 5); BOARD_CHECK(prg8_is(0x8000, 5));
    write_mem(0x8100, 0x81); BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    BOARD_CHECK(cpu_write_abs(0x8200, 2) && cpu_write_abs(0x8201, 0));
    BOARD_CHECK(!cart_irq_pending() && cpu_nop() && cart_irq_pending());
    write_mem(0x8200, 0); BOARD_CHECK(!cart_irq_pending());
    cart_set_dip_switches(0); board_image_free(&image); return 0;
}

static int test_103_106(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 103, 0x20000, 0x2000, false));
    BOARD_CHECK(board_image_load(&image) == 0 && prg8_is(0x8000, 12));
    write_mem(0x6000, 0x5A); write_mem(0xB800, 0x66);
    BOARD_CHECK(read_mem(0x6000) == 0x5A && read_mem(0xB800) == 0x66);
    write_mem(0x8000, 3); write_mem(0xF000, 0x10);
    BOARD_CHECK(prg8_is(0x6000, 3));
    write_mem(0x6000, 0xA5); BOARD_CHECK(prg8_is(0x6000, 3));
    write_mem(0xF000, 0); BOARD_CHECK(read_mem(0x6000) == 0xA5);
    write_mem(0xE000, 8); BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 106, 0x40000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8008, 3); write_mem(0x8009, 4); write_mem(0x800A, 5); write_mem(0x800B, 6);
    BOARD_CHECK(prg8_is(0x8000, 0x13) && prg8_is(0xA000, 4)
                && prg8_is(0xC000, 5) && prg8_is(0xE000, 0x16));
    write_mem(0x8000, 6); write_mem(0x8001, 8); write_mem(0x8004, 9);
    BOARD_CHECK(chr1_is(0x0000, 6) && chr1_is(0x0400, 9) && chr1_is(0x1000, 9));
    BOARD_CHECK(cpu_write_abs(0x800D, 0) && cpu_write_abs(0x800E, 0xFE)
                && cpu_write_abs(0x800F, 0xFF));
    BOARD_CHECK(!cart_irq_pending() && cpu_nop() && cart_irq_pending());
    write_mem(0x800D, 0); BOARD_CHECK(!cart_irq_pending());
    board_image_free(&image); return 0;
}

static int test_107_108_120(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 107, 0x40000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg32_is(0) && chr8_is(0));
    write_mem(0x8000, 5); BOARD_CHECK(prg32_is(2) && chr8_is(5));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 108, 0x20000, 0x4000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg8_is(0x6000, 15) && prg8_is(0x8000, 12));
    write_mem(0x8000, 5); BOARD_CHECK(prg8_is(0x6000, 5) && chr8_is(1));
    write_mem(0x9000, 0); BOARD_CHECK(prg8_is(0x6000, 5) && chr8_is(0));
    write_mem(0xF000, 7); BOARD_CHECK(prg8_is(0x6000, 7) && chr8_is(1));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 120, 0x18000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg8_is(0x6000, 0) && prg8_is(0x8000, 8));
    write_mem(0x41FF, 3); BOARD_CHECK(prg8_is(0x6000, 3) && prg8_is(0xE000, 11));
    board_image_free(&image); return 0;
}

static int test_116(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 116, 0x80000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8000, 5); write_mem(0xA000, 6); write_mem(0x9000, 1);
    BOARD_CHECK(prg8_is(0x8000, 5) && prg8_is(0xA000, 6));
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    write_mem(0xB000, 3); write_mem(0xB001, 2); BOARD_CHECK(chr1_is(0, 0x23));

    write_mem(0x4100, 1); write_mem(0x8000, 6); write_mem(0x8001, 7);
    BOARD_CHECK(prg8_is(0x8000, 7));
    BOARD_CHECK(cpu_write_abs(0xC000, 1) && cpu_write_abs(0xC001, 0)
                && cpu_write_abs(0xE001, 0));
    BOARD_CHECK(qualified_edge() && !cart_irq_pending());
    BOARD_CHECK(qualified_edge() && cart_irq_pending());
    write_mem(0xE000, 0); BOARD_CHECK(!cart_irq_pending());

    write_mem(0x4101, 2);
    write_mem(0xE000, 1); write_mem(0xE000, 1); write_mem(0xE000, 0);
    write_mem(0xE000, 0); write_mem(0xE000, 0);
    BOARD_CHECK(prg8_is(0x8000, 6) && prg8_is(0xA000, 7));
    board_image_free(&image); return 0;
}

static int test_117_156(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 117, 0x40000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x8000, 3); write_mem(0x8001, 4); write_mem(0xA005, 9);
    BOARD_CHECK(prg8_is(0x8000, 3) && prg8_is(0xA000, 4) && chr1_is(0x1400, 9));
    write_mem(0xD000, 1); BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    BOARD_CHECK(cpu_write_abs(0xC001, 1) && cpu_write_abs(0xC003, 0)
                && cpu_write_abs(0xE000, 1));
    BOARD_CHECK(qualified_edge() && cart_irq_pending());
    write_mem(0xC002, 0); BOARD_CHECK(!cart_irq_pending());
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 156, 0x20000, 0x100000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg16_is(0xC000, 7));
    write_mem(0xC010, 3); BOARD_CHECK(prg16_is(0x8000, 3));
    write_mem(0xC000, 0x34); write_mem(0xC004, 1); BOARD_CHECK(chr1_is(0, 0x134));
    write_mem(0xC014, 1); BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    board_image_free(&image); return 0;
}

static int test_163_and_geometry(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 163, 0x80000, 0x4000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && prg32_is(0));
    write_mem(0x5000, 3); write_mem(0x5200, 2); BOARD_CHECK(prg32_is(3));
    write_mem(0x5300, 0x40); write_mem(0x5100, 0x08);
    BOARD_CHECK(cart_cpu_read_bus(0x5100, 0) == (uint8_t)(0x40 | 0x08 | 3 | (2 ^ 0xFF)));
    BOARD_CHECK(cart_cpu_read_bus(0x5500, 0) == 0x43);
    write_mem(0x5101, 1); write_mem(0x5101, 0); BOARD_CHECK(cart_cpu_read_bus(0x5500, 0) == 0);
    write_mem(0x5000, 0x80); ppu.scanline = 127; ppu.dot = 257; cart_notify_ppu_address(0, 1);
    BOARD_CHECK(ppu_read(0) == 4 && ppu_read(0x1000) == 4);
    ppu.scanline = 239; ppu.dot = 257; cart_notify_ppu_address(0, 2);
    BOARD_CHECK(ppu_read(0) == 0 && ppu_read(0x1000) == 0);
    board_image_free(&image);

    static const unsigned ids[] = {60,62,83,103,106,107,108,116,117,120,156,163};
    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        BOARD_CHECK(board_image_create(&image, ids[i], 0x6000, 0x2800, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        uint8_t before = read_mem(0x8000);
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
        BOARD_CHECK(read_mem(0x8000) == before);
        board_image_free(&image);
    }
    return 0;
}

static int test_103_partial_work_ram_writes(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 103, 0x20000, 0, true));
    image.data[10] = 6;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cpu_write_abs(0x6000, 0xA5) && cpu_write_abs(0x6FFF, 0x5A));
    BOARD_CHECK(read_mem(0x6000) == 0xA5 && read_mem(0x6FFF) == 0x5A);
    BOARD_CHECK(read_mem(0x7000) == 0xA5 && read_mem(0x7FFF) == 0x5A);
    BOARD_CHECK(cpu_write_abs(0x7000, 0xCC) && cpu_write_abs(0x7FFF, 0xDD));
    BOARD_CHECK(read_mem(0x6000) == 0xA5 && read_mem(0x6FFF) == 0x5A);
    BOARD_CHECK(read_mem(0xB800) == 0xA5 && read_mem(0xC7FF) == 0x5A);
    BOARD_CHECK(cpu_write_abs(0xB800, 0xCC) && cpu_write_abs(0xD7FF, 0xDD));
    BOARD_CHECK(read_mem(0xB800) == 0xA5 && read_mem(0xD7FF) == 0x5A);
    BOARD_CHECK(cpu_write_abs(0x8000, 3) && cpu_write_abs(0xF000, 0x10));
    BOARD_CHECK(prg8_is(0x6000, 3) && cpu_write_abs(0x6000, 0x6D) && prg8_is(0x6000, 3));
    BOARD_CHECK(cpu_write_abs(0xF000, 0) && read_mem(0x6000) == 0x6D);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x6000) == 0x6D && read_mem(0x6FFF) == 0x5A);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0 && read_mem(0x6000) == 0x6D);
    board_image_free(&image);
    return 0;
}

static void arm_ppu_irq110(unsigned mapper) {
    if (mapper == 116) {
        write_mem(0x4100, 1);
        write_mem(0xE000, 0);
        write_mem(0xC000, 0);
        write_mem(0xC001, 0);
        write_mem(0xE001, 0);
    } else {
        write_mem(0xC002, 0);
        write_mem(0xC001, 1);
        write_mem(0xC003, 0);
        write_mem(0xE000, 1);
    }
}

static int test_116_117_ppu_dot_filter(void) {
    const unsigned ids[] = {116, 117};
    for (unsigned board = 0; board < 2; ++board) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[board], 0x80000, 0x2000, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        arm_ppu_irq110(ids[board]);
        ppu.v = 0;
        (void)ppu_read(0x1000); (void)ppu_read(0);
        uint64_t cpu_clock = cpu_get_bus_cycle();
        ppu_step_dots(9);
        (void)ppu_read(0x1000);
        BOARD_CHECK(!cart_irq_pending() && cpu_get_bus_cycle() == cpu_clock);
        (void)ppu_read(0);
        ppu_step_dots(10);
        (void)ppu_read(0x1000);
        BOARD_CHECK(cart_irq_pending() && cpu_get_bus_cycle() == cpu_clock);

        arm_ppu_irq110(ids[board]);
        ppu.v = 0x1000;
        ppu_step_dots(20);
        (void)ppu_read(0x1000);
        BOARD_CHECK(!cart_irq_pending());
        ppu.v = 0;
        (void)ppu_read(0);
        ppu_step_dots(5);
        (void)ppu_read(0);
        ppu_step_dots(5);
        (void)ppu_read(0x1000);
        BOARD_CHECK(cart_irq_pending());

        arm_ppu_irq110(ids[board]);
        ppu.v = 0;
        (void)ppu_read(0x1000); (void)ppu_read(0);
        BOARD_CHECK(advance_three_cpu_cycles());
        (void)ppu_read(0x1000);
        BOARD_CHECK(!cart_irq_pending());
        BOARD_CHECK(qualified_edge() && cart_irq_pending());

        arm_ppu_irq110(ids[board]);
        ppu.scanline = 260; ppu.dot = 337;
        (void)ppu_read(0x1000); (void)ppu_read(0);
        ppu_step_dots(9);
        (void)ppu_read(0x1000);
        BOARD_CHECK(ppu.scanline == 261 && ppu.dot == 5 && !cart_irq_pending());
        (void)ppu_read(0); ppu_step_dots(10); (void)ppu_read(0x1000);
        BOARD_CHECK(cart_irq_pending());

        arm_ppu_irq110(ids[board]);
        ppu.scanline = 0; ppu.dot = 257; ppu.v = 0; ppu.ctrl = 0x08;
        ppu.mask = 0x18; ppu.rendering_enabled = true; ppu.fetches_enabled = true;
        (void)ppu_read(0x1000); (void)ppu_read(0);
        ppu_step_dots(6);
        BOARD_CHECK(!cart_irq_pending());
        ppu_step_dots(341);
        BOARD_CHECK(cart_irq_pending());
        board_image_free(&image);
    }
    return 0;
}

static int test_156_initial_mapping_and_chr_registers(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 156, 0x40000, 0x100000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && cart_cpu_read_bus(0x8000, 0xA5) == 0xA5);
    BOARD_CHECK(prg16_is(0xC000, 15) && cart_get_mirroring() == MIRROR_SINGLE0);
    BOARD_CHECK(ppu_read(0x123) == 0x23 && ppu_read(0x1ABC) == 0xBC);
    BOARD_CHECK(cpu_write_abs(0xC010, 3) && prg16_is(0x8000, 3) && ppu_read(0x123) == 0x23);
    BOARD_CHECK(cpu_write_abs(0xC015, 0xFF) && ppu_read(0x1ABC) == 0xBC);
    BOARD_CHECK(cpu_write_abs(0xC000, 0x34));
    BOARD_CHECK(chr1_is(0, 0x34) && chr1_is(0x400, 0) && chr1_is(0x1C00, 0));
    BOARD_CHECK(cpu_write_abs(0xC004, 1) && chr1_is(0, 0x134));
    BOARD_CHECK(cpu_write_abs(0xC00B, 0x56) && cpu_write_abs(0xC00F, 2));
    BOARD_CHECK(chr1_is(0x1C00, 0x256) && chr1_is(0, 0x134));
    ppu_write(0x123, 0xCC);
    BOARD_CHECK(ppu_read(0x123) == 0x34);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg16_is(0x8000, 3) && chr1_is(0, 0x134) && chr1_is(0x1C00, 0x256));
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 156, 0x40000, 0, true));
    image.data[11] = 3;
    BOARD_CHECK(board_image_load(&image) == 0);
    ppu_write(0x123, 0x6D);
    BOARD_CHECK(ppu_read(0x1F23) == 0x6D && cpu_write_abs(0xC000, 0xFF));
    BOARD_CHECK(ppu_read(0x123) == 0x6D);
    board_image_free(&image);
    return 0;
}

static int test_163_rendering_and_prerender_writes(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 163, 0x80000, 0x4000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && cpu_write_abs(0x5000, 0x80));
    ppu.scanline = 127; ppu.dot = 253; ppu.v = 0;
    ppu.mask = 0x18; ppu.rendering_enabled = true; ppu.fetches_enabled = true;
    ppu_step_dots(3);
    BOARD_CHECK(ppu_read(0) == 0 && ppu_read(0x1000) == 0);
    ppu_step_dots(4);
    BOARD_CHECK(ppu_read(0) == 4 && ppu_read(0x1000) == 4);
    ppu.scanline = 239; ppu.dot = 256;
    ppu_step_dots(4);
    BOARD_CHECK(ppu_read(0) == 0 && ppu_read(0x1000) == 0);
    ppu.mask = 0; ppu.rendering_enabled = false; ppu.fetches_enabled = false;
    ppu.scanline = 261; ppu.dot = 0;
    BOARD_CHECK(cpu_write_abs(0x5000, 0) && ppu_read(0) == 0 && ppu_read(0x1000) == 4);
    BOARD_CHECK(cpu_write_abs(0x5000, 0x80));
    ppu.scanline = 127; ppu.dot = 257; (void)ppu_read(0);
    BOARD_CHECK(ppu_read(0) == 4 && ppu_read(0x1000) == 4);
    ppu.scanline = 128; ppu.dot = 0;
    BOARD_CHECK(cpu_write_abs(0x5000, 0) && ppu_read(0) == 4 && ppu_read(0x1000) == 4);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(ppu_read(0) == 4 && ppu_read(0x1000) == 4);
    board_image_free(&image);
    return 0;
}

static int test_103_work_save_and_trainer_ownership(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 103, 0x20000, 0, true));
    image.data[6] |= 2;
    image.data[10] = 0x78;
    BOARD_CHECK(board_image_add_trainer(&image, 0x3E));
    char stem[96], path[112], save[112];
    snprintf(stem, sizeof(stem), "build/discrete110-%lu-%lu", (unsigned long)time(NULL), (unsigned long)clock());
    snprintf(path, sizeof(path), "%s.nes", stem);
    snprintf(save, sizeof(save), "%s.sav", stem);
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t written = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(written == image.size && closed == 0);
    uint8_t saved[0x2000];
    memset(saved, 0xA5, sizeof(saved));
    file = fopen(save, "wb");
    BOARD_CHECK(file != NULL);
    written = fwrite(saved, 1, sizeof(saved), file);
    closed = fclose(file);
    BOARD_CHECK(written == sizeof(saved) && closed == 0 && load_rom(path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu) && read_mem(0x7000) == 0x3E && read_mem(0x6000) == 0);
    BOARD_CHECK(cpu_write_abs(0x7000, 0x6D) && cpu_write_abs(0xB800, 0x7C));
    BOARD_CHECK(cpu_write_abs(0x8000, 3) && cpu_write_abs(0xF000, 0x10) && prg8_is(0x6000, 3));
    BOARD_CHECK(cpu_write_abs(0x7000, 0x5A) && load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(cpu_write_abs(0xF000, 0) && read_mem(0x7000) == 0x5A && read_mem(0xB800) == 0x7C);
    BOARD_CHECK(unload_rom() && load_rom(path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu) && read_mem(0x7000) == 0x3E && read_mem(0xB800) == 0 && unload_rom());
    file = fopen(save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0 && ftell(file) == 0x2000);
    BOARD_CHECK(fseek(file, 0, SEEK_SET) == 0 && fread(saved, 1, sizeof(saved), file) == sizeof(saved));
    BOARD_CHECK(fclose(file) == 0);
    for (unsigned byte = 0; byte < sizeof(saved); ++byte) BOARD_CHECK(saved[byte] == 0xA5);
    BOARD_CHECK(remove(path) == 0 && remove(save) == 0);
    board_image_free(&image);
    return 0;
}

int test_board_unlicensed_110_accuracy(void) {
    int failures = 0;
    failures += test_60_62(); failures += test_83(); failures += test_103_106();
    failures += test_107_108_120(); failures += test_116(); failures += test_117_156();
    failures += test_163_and_geometry();
    failures += test_103_partial_work_ram_writes();
    failures += test_116_117_ppu_dot_filter();
    failures += test_156_initial_mapping_and_chr_registers();
    failures += test_163_rendering_and_prerender_writes();
    failures += test_103_work_save_and_trainer_ownership();
    unload_rom(); cart_set_dip_switches(0);
    printf("Unlicensed #110 accuracy: 12 groups, %d failures\n", failures);
    return failures;
}
