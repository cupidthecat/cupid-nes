/*
 * board_kaiser_accuracy.c - Kaiser banking, timers, reset, and RAM regressions
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

static bool kaiser_image(BoardImage *image, unsigned mapper, size_t prg, size_t chr, bool nes20) {
    if (!board_image_create(image, mapper, prg, chr, nes20)) return false;
    // Two-kilobyte labels distinguish the smallest Kaiser PRG slots.
    for (size_t offset = 0; offset < prg; ++offset)
        image->data[16 + offset] = (offset & 0x7FF) == 1 ? (uint8_t)(offset >> 19) : (uint8_t)(offset >> 11);
    return true;
}

static int kaiser_store(uint16_t address, uint8_t value) {
    const uint8_t code[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(code); ++byte) write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int kaiser_nops(unsigned count) {
    write_mem(0x0300, 0xEA);
    for (unsigned i = 0; i < count; ++i) {
        cpu.pc = 0x0300;
        BOARD_CHECK(cpu_step(&cpu) == 2);
    }
    return 0;
}

static void kaiser202_reload(uint16_t value) {
    write_mem(0x8000, value & 15);
    write_mem(0x9000, (value >> 4) & 15);
    write_mem(0xA000, (value >> 8) & 15);
    write_mem(0xB000, value >> 12);
}

static int test_kaiser202_banks(void) {
    const unsigned ids[] = {56, 142};
    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        BoardImage image;
        BOARD_CHECK(kaiser_image(&image, ids[i], 0x40000, 0x10000, false));
        BOARD_CHECK(board_image_load(&image) == 0);
        write_mem(0x4018, 0xA5);
        BOARD_CHECK(read_mem(0x8000) == 0xA5 && read_mem(0xC000) == 0xA5);
        BOARD_CHECK(read_mem(0xE000) == 124 && ppu_read(0x0123) == 0x23);
        for (uint8_t reg = 1; reg <= 4; ++reg) {
            write_mem(0xE000, reg);
            BOARD_CHECK(kaiser_store(0xF400, (uint8_t)(reg + 4)) == 0);
        }
        BOARD_CHECK(read_mem(0x8000) == 20 && read_mem(0xA000) == 24 && read_mem(0xC000) == 28);
        write_mem(0xE000, 5);
        write_mem(0xF400, 4);
        BOARD_CHECK(read_mem(0x6000) == 32);
        write_mem(0x6000, 0xAB);
        BOARD_CHECK(read_mem(0x6000) == 32);
        write_mem(0xE000, 0);
        for (unsigned bank = 0; bank < 4; ++bank) write_mem((uint16_t)(0xF000 + bank), 0x10);
        BOARD_CHECK(read_mem(0x8000) == (ids[i] == 56 ? 84 : 20));
        BOARD_CHECK(read_mem(0x6000) == (ids[i] == 56 ? 96 : 32));
        BOARD_CHECK(read_mem(0xE000) == 124);
        write_mem(0xF800, 1);
        write_mem(0xFC00, 35);
        write_mem(0xFFFF, 7);
        if (ids[i] == 56) {
            BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
            BOARD_CHECK(ppu_read(0x0123) == 35 && ppu_read(0x1FFF) == 7);
        } else {
            BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
            BOARD_CHECK(ppu_read(0x0123) == 0x23 && ppu_read(0x1FFF) == 0xFF);
        }
        write_mem(0xE000, 5);
        write_mem(0xF400, 0);
        write_mem(0x6123, 0x96);
        BOARD_CHECK(read_mem(0x6123) == 0x96);
        cpu_soft_reset(&cpu);
        BOARD_CHECK(read_mem(0x6123) == 0x96 && read_mem(0x8000) == (ids[i] == 56 ? 84 : 20));
        BOARD_CHECK(board_image_load(&image) == 0);
        write_mem(0x4018, 0xA5);
        BOARD_CHECK(read_mem(0x8000) == 0xA5 && ppu_read(0x0123) == 0x23);
        board_image_free(&image);
    }
    return 0;
}

static int test_kaiser202_irq(void) {
    const unsigned ids[] = {56, 142};
    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); ++i) {
        BoardImage image;
        BOARD_CHECK(kaiser_image(&image, ids[i], 0x8000, 0, true));
        BOARD_CHECK(board_image_load(&image) == 0 && !cart_irq_pending());
        kaiser202_reload(0xFFFB);
        BOARD_CHECK(kaiser_store(0xC000, 2) == 0);
        BOARD_CHECK(kaiser_nops(1) == 0 && !cart_irq_pending());
        BOARD_CHECK(kaiser_nops(1) == 0 && cart_irq_pending());
        BOARD_CHECK(kaiser_store(0xD000, 0) == 0 && !cart_irq_pending());
        BOARD_CHECK(kaiser_nops(100) == 0 && !cart_irq_pending());
        BOARD_CHECK(kaiser_store(0xCFFF, 2) == 0);
        write_mem(0x0300, 0x8D);
        write_mem(0x0301, 0);
        write_mem(0x0302, 4);
        cpu.pc = 0x0300;
        BOARD_CHECK(cpu_step(&cpu) == 4 && cart_irq_pending());
        BOARD_CHECK(kaiser_store(0xC000, 0) == 0 && !cart_irq_pending());
        kaiser202_reload(0xFFFF);
        BOARD_CHECK(kaiser_store(0xC000, 2) == 0);
        BOARD_CHECK(kaiser_nops(32767) == 0 && !cart_irq_pending());
        BOARD_CHECK(kaiser_nops(1) == 0 && cart_irq_pending());
        BOARD_CHECK(kaiser_store(0xC000, 1) == 0 && !cart_irq_pending());
        BOARD_CHECK(kaiser_nops(100) == 0 && !cart_irq_pending());
        kaiser202_reload(0xFFF8);
        BOARD_CHECK(kaiser_store(0xC000, 2) == 0);
        cpu_soft_reset(&cpu);
        BOARD_CHECK(cart_irq_pending());
        write_mem(0xC000, 0);
        kaiser202_reload(0xFEFF);
        BOARD_CHECK(kaiser_store(0xC000, 2) == 0);
        BOARD_CHECK(kaiser_store(0x4014, 0) == 0 && !cart_irq_pending());
        write_mem(0x0300, 0xEA);
        cpu.pc = 0x0300;
        BOARD_CHECK(cpu_step(&cpu) >= 515 && cart_irq_pending());
        BOARD_CHECK(board_image_load(&image) == 0 && !cart_irq_pending());
        board_image_free(&image);
    }
    return 0;
}

static int test_kaiser171_312_346(void) {
    BoardImage image;
    BOARD_CHECK(kaiser_image(&image, 171, 0x10000, 0x10000, false));
    BOARD_CHECK(board_image_load(&image) == 0 && ppu_read(0x1234) == 0x34);
    write_mem(0xEFFF, 8);
    BOARD_CHECK(ppu_read(0x0123) == 0x23);
    BOARD_CHECK(kaiser_store(0xF000, 3) == 0);
    BOARD_CHECK(ppu_read(0x0123) == 12 && ppu_read(0x1234) == 0x34);
    write_mem(0xFFF0, 4);
    BOARD_CHECK(ppu_read(0x0123) == 12 && ppu_read(0x1234) == 16);
    write_mem(0xF070, 5);
    write_mem(0xF080, 7);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(ppu_read(0x0123) == 20 && ppu_read(0x1234) == 28 && read_mem(0x8000) == 0);
    board_image_free(&image);

    BOARD_CHECK(kaiser_image(&image, 312, 0x400000, 0x2000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_add_trainer(&image, 0x55));
    BOARD_CHECK(board_image_load(&image) == 0 && cart_get_mirroring() == MIRROR_VERTICAL);
    BOARD_CHECK(read_mem(0xC000) == 248 && read_mem(0xC001) == 7);
    BOARD_CHECK(kaiser_store(0x7FFF, 3) == 0 && read_mem(0x8000) == 24);
    write_mem(0x7000, 0x80);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0x8001) == 4 && read_mem(0x7000) == 0x55);
    write_mem(0x8000, 1);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    write_mem(0xFFFF, 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL && read_mem(0x8001) == 4);
    board_image_free(&image);

    BOARD_CHECK(kaiser_image(&image, 346, 0x10000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 16);
    write_mem(0xE0A1, 0);
    BOARD_CHECK(read_mem(0x8000) == 16);
    BOARD_CHECK(kaiser_store(0xE0A0, 0xFF) == 0 && read_mem(0x8000) == 0);
    write_mem(0xEE36, 0);
    write_mem(0xEE34, 0);
    BOARD_CHECK(read_mem(0x8000) == 16);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 16 && ppu_read(0x1FFF) == 7);
    board_image_free(&image);
    return 0;
}

static int test_kaiser175_read_latch(void) {
    BoardImage image;
    BOARD_CHECK(kaiser_image(&image, 175, 0x20000, 0x20000, false));
    BOARD_CHECK(load_rom_memory(image.data, image.size) == 0);
    write_mem(0x4018, 0xA5);
    BOARD_CHECK(read_mem(0xC000) == 0xA5 && ppu_read(0x1234) == 0x34);
    write_mem(0xA000, 3);
    BOARD_CHECK(read_mem(0x8000) == 0);
    write_mem(0x4018, 0xA5);
    BOARD_CHECK(read_mem(0xFFFB) == 0xA5);
    BOARD_CHECK(read_mem(0xFFFC) == 31);
    BOARD_CHECK(read_mem(0x8000) == 24 && read_mem(0xC000) == 24 && ppu_read(0) == 24);
    write_mem(0xA000, 5);
    BOARD_CHECK(read_mem(0xFFFD) == 31 && ppu_read(0) == 24);
    BOARD_CHECK(read_mem(0xFFFC) == 47 && read_mem(0x8000) == 40 && ppu_read(0) == 40);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu));
    write_mem(0xA000, 3);
    (void)read_mem(0xFFFC);
    BOARD_CHECK(kaiser_store(0x8000, 4) == 0 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    write_mem(0x8001, 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 0 && ppu_read(0) == 0);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    board_image_free(&image);
    return 0;
}

static int test_kaiser303(void) {
    BoardImage image;
    BOARD_CHECK(kaiser_image(&image, 303, 0x20000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0 && cart_get_mirroring() == MIRROR_VERTICAL);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 16);
    write_mem(0x4A4C, 0);
    write_mem(0x4025, 8);
    BOARD_CHECK(read_mem(0x8000) == 0 && cart_get_mirroring() == MIRROR_VERTICAL);
    BOARD_CHECK(kaiser_store(0x51FE, 0) == 0);
    BOARD_CHECK(read_mem(0x8000) == 56 && read_mem(0xC000) == 16);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    write_mem(0x4020, 4);
    BOARD_CHECK(kaiser_store(0x4021, 0) == 0);
    BOARD_CHECK(kaiser_nops(1) == 0 && !cart_irq_pending());
    BOARD_CHECK(kaiser_nops(1) == 0 && cart_irq_pending());
    write_mem(0x4018, 0xA5);
    BOARD_CHECK(read_mem(0x4031) == 0xA5 && cart_irq_pending());
    BOARD_CHECK(read_mem(0x4030) == 1 && !cart_irq_pending());
    BOARD_CHECK(read_mem(0x4030) == 0);
    write_mem(0x4020, 0);
    BOARD_CHECK(kaiser_store(0x4021, 0) == 0 && kaiser_nops(1000) == 0 && !cart_irq_pending());
    // The enabled zero counter resumes when its low byte changes. The LDA/STA
    // sequence consumes six clocks before its high-byte write, leaving ten.
    write_mem(0x4020, 16);
    BOARD_CHECK(kaiser_store(0x4021, 0) == 0 && kaiser_nops(2) == 0);
    BOARD_CHECK(read_mem(0x4030) == 0);
    BOARD_CHECK(kaiser_nops(3) == 0 && cart_irq_pending());
    write_mem(0x4020, 5);
    BOARD_CHECK(!cart_irq_pending());
    BOARD_CHECK(kaiser_store(0x4021, 0) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cart_irq_pending() && read_mem(0x8000) == 56 && read_mem(0x4030) == 1);
    write_mem(0x4020, 0);
    BOARD_CHECK(kaiser_store(0x4021, 1) == 0);
    BOARD_CHECK(kaiser_store(0x4014, 0) == 0 && !cart_irq_pending());
    write_mem(0x0300, 0xEA);
    cpu.pc = 0x0300;
    BOARD_CHECK(cpu_step(&cpu) >= 515 && cart_irq_pending());
    BOARD_CHECK(board_image_load(&image) == 0 && !cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static int test_kaiser306(void) {
    BoardImage image;
    BOARD_CHECK(kaiser_image(&image, 306, 0x20000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x6000) == 32 && read_mem(0x8000) == 48 && read_mem(0xE000) == 60);
    const uint16_t addresses[] = {0xD943, 0xD953, 0xD973, 0xD903, 0xD933, 0xD937, 0xD93F};
    const uint8_t pages[] = {0, 16, 44, 44, 32, 36, 44};
    for (unsigned i = 0; i < sizeof(pages); ++i) {
        BOARD_CHECK(kaiser_store(addresses[i], (uint8_t)i) == 0);
        BOARD_CHECK(read_mem(0x6000) == pages[i] && read_mem(0x7FFF) == pages[i] + 3);
    }
    write_mem(0xD942, 0xFF);
    write_mem(0x6000, 0xFF);
    BOARD_CHECK(read_mem(0x6000) == 44);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x6000) == 44 && read_mem(0x8000) == 48 && ppu_read(0x1FFF) == 7);
    board_image_free(&image);
    return 0;
}

static int test_kaiser305(void) {
    BoardImage image;
    BOARD_CHECK(kaiser_image(&image, 305, 0x20000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    for (unsigned slot = 0; slot < 16; ++slot)
        BOARD_CHECK(read_mem((uint16_t)(0x8000 + slot * 0x800)) == 15 - slot);
    const uint8_t banks[] = {5, 9, 29, 33};
    for (unsigned slot = 0; slot < 4; ++slot) {
        BOARD_CHECK(read_mem((uint16_t)(0x6000 + slot * 0x800)) == 0);
        BOARD_CHECK(kaiser_store((uint16_t)(0x8000 + slot * 0x800), banks[slot]) == 0);
        BOARD_CHECK(read_mem((uint16_t)(0x6000 + slot * 0x800)) == banks[slot]);
    }
    write_mem(0xA000, 7);
    write_mem(0x6000, 0xFF);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x6000) == 7 && read_mem(0x7800) == 33 && read_mem(0xF800) == 0);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL && ppu_read(0x1FFF) == 7);
    board_image_free(&image);
    return 0;
}

static int test_kaiser302(void) {
    BoardImage image;
    BOARD_CHECK(kaiser_image(&image, 302, 0x80000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xA000) == 52 && read_mem(0xF800) == 63);
    const uint16_t regs[] = {0xB000, 0xB002, 0xC000, 0xC002, 0xD000, 0xD002, 0xE000, 0xE002};
    const uint8_t banks[] = {0x13, 0x24, 0x35, 0x46, 0x57, 0x68, 0x79, 0x8A};
    for (unsigned slot = 0; slot < 8; ++slot) {
        write_mem(regs[slot], (uint8_t)(0xF0 | (banks[slot] & 15)));
        BOARD_CHECK(kaiser_store((uint16_t)(regs[slot] + 1), (uint8_t)(0xE0 | (banks[slot] >> 4))) == 0);
        uint16_t address = (uint16_t)(slot < 4 ? 0x8000 + slot * 0x800 : 0x6000 + (slot - 4) * 0x800);
        BOARD_CHECK(read_mem(address) == banks[slot]);
    }
    write_mem(0xB000, 9);
    BOARD_CHECK(read_mem(0x8000) == 25);
    write_mem(0xB001, 15);
    BOARD_CHECK(read_mem(0x8000) == 249);
    write_mem(0xF000, 0);
    write_mem(0x6000, 0);
    BOARD_CHECK(read_mem(0x8000) == 249 && read_mem(0x6000) == 0x57);
    write_mem(0x9F03, 1);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    write_mem(0x8000, 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL && read_mem(0x8000) == 249);
    BOARD_CHECK(read_mem(0xA000) == 52 && read_mem(0xF800) == 63);
    board_image_free(&image);
    return 0;
}

static int test_kaiser307(void) {
    BoardImage image;
    BOARD_CHECK(kaiser_image(&image, 307, 0x10000, 0x2000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_add_trainer(&image, 0x44));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x6000) == 0 && read_mem(0xB000) == 0x44 && read_mem(0x7000) == 30);
    BOARD_CHECK(read_mem(0xA000) == 24 && read_mem(0xE000) == 28 && read_mem(0xF000) == 30);
    write_mem(0x6123, 0x12);
    BOARD_CHECK(kaiser_store(0xB123, 0x34) == 0);
    BOARD_CHECK(read_mem(0x6123) == 0x12 && read_mem(0xB123) == 0x34);
    write_mem(0xA000, 0xFF);
    write_mem(0x7000, 0xFF);
    BOARD_CHECK(read_mem(0xA000) == 24 && read_mem(0x7000) == 30);
    write_mem(0x8000, 6);
    write_mem(0x8001, 3);
    write_mem(0x8000, 7);
    write_mem(0x8001, 4);
    BOARD_CHECK(read_mem(0x8000) == 12 && read_mem(0x9000) == 14);
    BOARD_CHECK(read_mem(0xC000) == 16 && read_mem(0xD000) == 18);
    for (uint8_t reg = 2; reg < 6; ++reg) {
        write_mem(0x8000, reg);
        write_mem(0x8001, reg < 4 ? 1 : 0);
    }
    ppu_write(0x2000, 0xA5);
    ppu_write(0x2400, 0xB6);
    BOARD_CHECK(ppu_read(0x2800) == 0xA5 && ppu_read(0x2C00) == 0xB6);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0xB123) == 0x34 && read_mem(0x8000) == 12 && ppu_read(0x2800) == 0xA5);
    board_image_free(&image);
    BOARD_CHECK(kaiser_image(&image, 307, 0x10000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4018, 0xA5);
    BOARD_CHECK(read_mem(0x6000) == 0xA5 && read_mem(0xB000) == 0xA5);
    board_image_free(&image);
    return 0;
}

static int test_kaiser_geometry_and_replacement(void) {
    const unsigned ids[] = {56, 142, 171, 175, 302, 303, 305, 306, 307, 312, 346};
    const uint16_t addresses[] = {0x6000, 0x7FFF, 0x8000, 0xBFFF, 0xC000, 0xE000, 0xFFFF};
    for (unsigned board = 0; board < sizeof(ids) / sizeof(ids[0]); ++board) {
        BoardImage image;
        BOARD_CHECK(kaiser_image(&image, ids[board], 0x6000, 0x2800, true));
        image.data[8] |= 0xF0;
        BOARD_CHECK(board_image_load(&image) == 0);
        uint8_t previous[sizeof(addresses) / sizeof(addresses[0])];
        for (unsigned i = 0; i < sizeof(previous); ++i) {
            write_mem(0x4018, 0xC9);
            previous[i] = read_mem(addresses[i]);
        }
        uint8_t chr = ppu_read(0x1234);
        for (unsigned invalid = 0; invalid < 3; ++invalid) {
            if (invalid == 1) board_image_set_unsupported_console(&image);
            if (invalid == 2) image.data[0] = 0;
            BOARD_CHECK(load_rom_memory(image.data, image.size - (invalid == 0)) < 0);
            for (unsigned i = 0; i < sizeof(previous); ++i) {
                write_mem(0x4018, 0xC9);
                BOARD_CHECK(read_mem(addresses[i]) == previous[i]);
            }
            BOARD_CHECK(ppu_read(0x1234) == chr);
        }
        board_image_free(&image);
        BOARD_CHECK(kaiser_image(&image, ids[board], 0x2000, 0x100, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(ppu_read(0x0823) == 0x23);
        board_image_free(&image);
    }
    return 0;
}

static int test_kaiser_persistence(void) {
    const unsigned ids[] = {56, 142, 171, 175, 302, 303, 305, 306, 307, 312, 346};
    for (unsigned board = 0; board < sizeof(ids) / sizeof(ids[0]); ++board) {
        BoardImage image;
        BOARD_CHECK(kaiser_image(&image, ids[board], 0x20000, 0, true));
        image.data[6] |= 2;
        image.data[10] = image.data[11] = 0x70;
        BOARD_CHECK(board_image_add_trainer(&image, 0x44));
        char path[128], save[128], chr_save[136];
        snprintf(path, sizeof(path), "build/board-kaiser-%u-%lu-%lu.nes", ids[board],
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
        saved[0x23] = 0x66;
        file = fopen(save, "wb");
        BOARD_CHECK(file != NULL);
        count = fwrite(saved, 1, sizeof(saved), file);
        closed = fclose(file);
        BOARD_CHECK(count == sizeof(saved) && closed == 0);
        BOARD_CHECK(load_rom(path) == 0);
        ppu_power_on(&ppu);
        BOARD_CHECK(cpu_power_on(&cpu));
        write_mem(0x6023, 0x99);
        ppu_write(0x0023, 0x81);
        BOARD_CHECK(unload_rom());
        BOARD_CHECK(load_rom(path) == 0 && ppu_read(0x0023) == 0x81);
        BOARD_CHECK(unload_rom());
        file = fopen(save, "rb");
        BOARD_CHECK(file != NULL);
        count = fread(saved, 1, sizeof(saved), file);
        closed = fclose(file);
        BOARD_CHECK(count == sizeof(saved) && closed == 0);
        bool blocked = ids[board] == 302 || ids[board] == 305 || ids[board] == 306 || ids[board] == 312;
        BOARD_CHECK(saved[0x23] == (blocked ? 0x66 : 0x99));
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

int test_board_kaiser_accuracy(void) {
    int failures = 0;
    failures += test_kaiser202_banks();
    failures += test_kaiser202_irq();
    failures += test_kaiser171_312_346();
    failures += test_kaiser175_read_latch();
    failures += test_kaiser303();
    failures += test_kaiser306();
    failures += test_kaiser305();
    failures += test_kaiser302();
    failures += test_kaiser307();
    failures += test_kaiser_geometry_and_replacement();
    failures += test_kaiser_persistence();
    unload_rom();
    printf("Kaiser accuracy: 11 groups, %d failures\n", failures);
    return failures;
}
