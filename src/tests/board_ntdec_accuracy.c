/*
 * board_ntdec_accuracy.c - NTDEC cartridge banking, storage, and IRQ tests
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

static int ntdec_store(uint16_t address, uint8_t value) {
    const uint8_t code[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(code); ++byte)
        write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int ntdec_nops(unsigned count) {
    write_mem(0x0300, 0xEA);
    for (unsigned instruction = 0; instruction < count; ++instruction) {
        cpu.pc = 0x0300;
        BOARD_CHECK(cpu_step(&cpu) == 2);
    }
    return 0;
}

static int test_caltron41(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 41, 0x40000, 0x20000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 7 && ppu_read(0) == 0);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    BOARD_CHECK(ntdec_store(0x600E, 0xFF) == 0);
    BOARD_CHECK(read_mem(0x8000) == 48 && ppu_read(0) == 32);
    write_mem(0xFFFF, 3);
    BOARD_CHECK(ppu_read(0) == 56 && ppu_read(0x1FFF) == 63);
    write_mem(0x6011, 0);
    BOARD_CHECK(read_mem(0x8000) == 8 && ppu_read(0) == 88);
    write_mem(0x8000, 0);
    BOARD_CHECK(ppu_read(0) == 88);
    write_mem(0x6024, 0);
    BOARD_CHECK(read_mem(0x8000) == 32 && ppu_read(0) == 24);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    ppu_write(0x2000, 0x84);
    BOARD_CHECK(ppu_read(0x2400) == 0x84);
    write_mem(0x6800, 0x93);
    BOARD_CHECK(read_mem(0x6800) == 0x93 && read_mem(0x8000) == 32);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0) == 0);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL && read_mem(0x6800) == 0x93);
    board_image_free(&image);
    return 0;
}

static int test_ntdec63(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 63, 0x200000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xA000) == 2);
    BOARD_CHECK(read_mem(0xC000) == 0 && read_mem(0xE000) == 2);
    BOARD_CHECK(ppu_read(0x1234) == 0x34);
    BOARD_CHECK(ntdec_store(0x8082, 0) == 0);
    BOARD_CHECK(read_mem(0x8000) == 128 && read_mem(0xA000) == 130);
    BOARD_CHECK(read_mem(0xC000) == 132 && read_mem(0xE000) == 134);
    write_mem(0x8084, 0xFF);
    BOARD_CHECK(read_mem(0x8000) == 132 && read_mem(0xA000) == 134);
    BOARD_CHECK(read_mem(0xC000) == 132 && read_mem(0xE000) == 134);
    write_mem(0x889A, 0);
    BOARD_CHECK(read_mem(0x8000) == 152 && read_mem(0xA000) == 154);
    BOARD_CHECK(read_mem(0xC000) == 156 && read_mem(0xE000) == 54);
    write_mem(0x8383, 0);
    write_mem(0x4800, 0xA5);
    BOARD_CHECK(read_mem(0x8000) == 0xA5 && read_mem(0xBFFF) == 0xA5);
    BOARD_CHECK(read_mem(0xC000) == 132 && read_mem(0xE000) == 134);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    cpu_soft_reset(&cpu);
    write_mem(0x4800, 0xB6);
    BOARD_CHECK(read_mem(0x8000) == 0xB6 && read_mem(0xC000) == 132);
    write_mem(0x8000, 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 0);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    board_image_free(&image);
    return 0;
}

static void ntdec112_register(uint8_t number, uint8_t value) {
    write_mem(0x8000, number);
    write_mem(0xA000, value);
}

static int test_ntdec112(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 112, 0x80000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xA000) == 0);
    BOARD_CHECK(read_mem(0xC000) == 124 && read_mem(0xE000) == 126);
    BOARD_CHECK(ppu_read(0) == 0 && ppu_read(0x400) == 1 && ppu_read(0x800) == 0);
    ntdec112_register(0, 11);
    ntdec112_register(1, 17);
    BOARD_CHECK(read_mem(0x8000) == 22 && read_mem(0xA000) == 34);
    ntdec112_register(2, 3);
    ntdec112_register(3, 9);
    ntdec112_register(4, 5);
    ntdec112_register(5, 7);
    ntdec112_register(6, 11);
    ntdec112_register(7, 13);
    BOARD_CHECK(ppu_read(0) == 3 && ppu_read(0x400) == 4);
    BOARD_CHECK(ppu_read(0x800) == 9 && ppu_read(0xC00) == 10);
    BOARD_CHECK(ntdec_store(0xC000, 0xA0) == 0);
    BOARD_CHECK(ppu_read(0x1000) == 5 && ppu_read(0x1001) == 0);
    BOARD_CHECK(ppu_read(0x1400) == 7 && ppu_read(0x1401) == 1);
    BOARD_CHECK(ppu_read(0x1800) == 11 && ppu_read(0x1801) == 0);
    BOARD_CHECK(ppu_read(0x1C00) == 13 && ppu_read(0x1C01) == 1);
    write_mem(0xE000, 1);
    write_mem(0xE001, 0);
    write_mem(0xC001, 0);
    write_mem(0x8001, 0);
    write_mem(0xA001, 0);
    write_mem(0x5FFE, 0xFF);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    BOARD_CHECK(read_mem(0x8000) == 22 && ppu_read(0x1401) == 1);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0xA000) == 34 && ppu_read(0) == 3 && ppu_read(0x1C01) == 1);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    board_image_free(&image);
    return 0;
}

static int test_ntdec174(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 174, 0x20000, 0x10000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 0 && ppu_read(0) == 0);
    BOARD_CHECK(ntdec_store(0x805B, 0xFF) == 0);
    BOARD_CHECK(read_mem(0x8000) == 20 && read_mem(0xC000) == 20 && ppu_read(0) == 40);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    write_mem(0x80CA, 0);
    BOARD_CHECK(read_mem(0x8000) == 16 && read_mem(0xC000) == 20 && ppu_read(0) == 40);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 16 && read_mem(0xC000) == 20 && ppu_read(0) == 40);
    board_image_free(&image);
    return 0;
}

static int test_ntdec193(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 193, 0x20000, 0x10000, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4800, 0xA6);
    BOARD_CHECK(read_mem(0x8000) == 0xA6 && read_mem(0xA000) == 26);
    BOARD_CHECK(read_mem(0xC000) == 28 && read_mem(0xE000) == 30);
    BOARD_CHECK(ppu_read(0x1234) == 0x34);
    BOARD_CHECK(ntdec_store(0x6003, 5) == 0 && read_mem(0x8000) == 10);
    write_mem(0x7FFF, 7);
    BOARD_CHECK(read_mem(0x8000) == 14 && read_mem(0xE000) == 30);
    write_mem(0x6000, 7);
    write_mem(0x6001, 13);
    write_mem(0x6002, 19);
    BOARD_CHECK(ppu_read(0) == 6 && ppu_read(0x7FF) == 7);
    BOARD_CHECK(ppu_read(0x800) == 8 && ppu_read(0xFFF) == 9);
    BOARD_CHECK(ppu_read(0x1000) == 12 && ppu_read(0x17FF) == 13);
    BOARD_CHECK(ppu_read(0x1800) == 18 && ppu_read(0x1FFF) == 19);
    BOARD_CHECK(read_mem(0x6000) == 0 && read_mem(0x6003) == 0);
    write_mem(0x8000, 0);
    BOARD_CHECK(read_mem(0x8000) == 14 && ppu_read(0) == 6);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 14 && ppu_read(0x1800) == 18);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    board_image_free(&image);
    return 0;
}

static int test_ntdec221(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 221, 0x100000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 0);
    write_mem(0xC003, 0xFF);
    BOARD_CHECK(read_mem(0x8000) == 12 && read_mem(0xC000) == 12);
    write_mem(0x80A2, 0);
    BOARD_CHECK(read_mem(0x8000) == 168 && read_mem(0xC000) == 172);
    BOARD_CHECK(ntdec_store(0x81A3, 0xAA) == 0);
    BOARD_CHECK(read_mem(0x8000) == 172 && read_mem(0xC000) == 188);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    write_mem(0xC006, 0);
    BOARD_CHECK(read_mem(0x8000) == 184 && read_mem(0xC000) == 188);
    write_mem(0x80A0, 0xFF);
    BOARD_CHECK(read_mem(0x8000) == 184 && read_mem(0xC000) == 184);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL && ppu_read(0x1FFF) == 7);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 184 && read_mem(0xC000) == 184);
    board_image_free(&image);
    return 0;
}

static int test_ntdec290(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 290, 0x80000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 4 && ppu_read(0) == 0);
    BOARD_CHECK(ntdec_store(0xA345, 0xFF) == 0);
    BOARD_CHECK(read_mem(0x8000) == 32 && read_mem(0xC000) == 36 && ppu_read(0) == 232);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    write_mem(0xA7C3, 0);
    BOARD_CHECK(read_mem(0x8000) == 36 && read_mem(0xC000) == 36 && ppu_read(0) == 216);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 4 && ppu_read(0) == 0);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    board_image_free(&image);
    return 0;
}

static int test_tf1201_banks(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 298, 0x40000, 0x40000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xA000) == 0);
    BOARD_CHECK(read_mem(0xC000) == 60 && read_mem(0xE000) == 62 && ppu_read(0x1C00) == 0);
    write_mem(0x8000, 5);
    write_mem(0xA000, 7);
    BOARD_CHECK(read_mem(0x8000) == 10 && read_mem(0xA000) == 14);
    BOARD_CHECK(ntdec_store(0x9004, 2) == 0);
    BOARD_CHECK(read_mem(0x8000) == 60 && read_mem(0xC000) == 10);
    write_mem(0xB000, 0xA);
    write_mem(0xB008, 3);
    write_mem(0xB004, 7);
    write_mem(0xB00C, 4);
    BOARD_CHECK(ppu_read(0) == 58 && ppu_read(0x400) == 71);
    for (unsigned slot = 2; slot < 8; ++slot) {
        uint16_t address = (uint16_t)(0xB000 + (slot / 2) * 0x1000 + (slot & 1));
        write_mem(address, (uint8_t)(slot + 1));
        write_mem((uint16_t)(address | 2), (uint8_t)(slot + 2));
    }
    BOARD_CHECK(ppu_read(0x800) == 0x43 && ppu_read(0xC00) == 0x54);
    BOARD_CHECK(ppu_read(0x1000) == 0x65 && ppu_read(0x1400) == 0x76);
    BOARD_CHECK(ppu_read(0x1800) == 0x87 && ppu_read(0x1C00) == 0x98);
    write_mem(0x9000, 1);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 60 && read_mem(0xC000) == 10);
    BOARD_CHECK(ppu_read(0x1C00) == 0x98 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    board_image_free(&image);
    return 0;
}

static int test_tf1201_irq(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 298, 0x20000, 0, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(ntdec_store(0xF000, 0x0E) == 0 && ntdec_store(0xF008, 0x0F) == 0);
    BOARD_CHECK(ntdec_store(0xF004, 2) == 0);
    BOARD_CHECK(ntdec_nops(113) == 0 && !cart_irq_pending());
    BOARD_CHECK(ntdec_nops(1) == 0 && cart_irq_pending());
    BOARD_CHECK(ntdec_store(0xF00C, 0) == 0 && !cart_irq_pending());
    BOARD_CHECK(ntdec_nops(200) == 0 && !cart_irq_pending());
    BOARD_CHECK(ntdec_nops(14500) == 0 && cart_irq_pending());
    BOARD_CHECK(ntdec_store(0xF001, 2) == 0 && !cart_irq_pending());
    BOARD_CHECK(ntdec_nops(113) == 0 && !cart_irq_pending());
    BOARD_CHECK(ntdec_nops(1) == 0 && cart_irq_pending());
    BOARD_CHECK(ntdec_store(0xF001, 0) == 0 && !cart_irq_pending());
    BOARD_CHECK(ntdec_nops(1000) == 0 && !cart_irq_pending());

    BOARD_CHECK(ntdec_store(0xF000, 15) == 0 && ntdec_store(0xF001, 2) == 0);
    BOARD_CHECK(ntdec_nops(55) == 0 && !cart_irq_pending());
    write_mem(0x0300, 0x8D);
    write_mem(0x0301, 0);
    write_mem(0x0302, 4);
    cpu.pc = 0x0300;
    BOARD_CHECK(cpu_step(&cpu) == 4 && cart_irq_pending());
    BOARD_CHECK(ntdec_store(0xF001, 0) == 0);
    BOARD_CHECK(ntdec_store(0xF000, 12) == 0 && ntdec_store(0xF001, 2) == 0);
    BOARD_CHECK(ntdec_store(0x4014, 0) == 0 && !cart_irq_pending());
    write_mem(0x0300, 0xEA);
    cpu.pc = 0x0300;
    BOARD_CHECK(cpu_step(&cpu) >= 515 && cart_irq_pending());

    BOARD_CHECK(ntdec_store(0xF001, 0) == 0);
    BOARD_CHECK(ntdec_store(0xF000, 15) == 0 && ntdec_store(0xF001, 2) == 0);
    BOARD_CHECK(ntdec_nops(54) == 0 && !cart_irq_pending());
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cart_irq_pending());
    BOARD_CHECK(board_image_load(&image) == 0 && !cart_irq_pending());
    board_image_free(&image);
    return 0;
}

static int test_ntdec_geometry_and_replacement(void) {
    static const unsigned ids[] = {41, 63, 112, 174, 193, 221, 290, 298};
    static const int startup[8][4] = {
        {0, 2, 4, -1}, {0, 2, 0, 2}, {0, 0, 2, 4}, {0, 2, 0, 2},
        {-1, 0, 2, 4}, {0, 2, 0, 2}, {0, 2, 0, 2}, {0, 0, 2, 4}
    };
    static const uint8_t chr_last[] = {7, 0xFF, 0, 7, 0xFF, 7, 7, 0};
    for (unsigned board = 0; board < sizeof(ids) / sizeof(ids[0]); ++board) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[board], 0x6000, 0x2800, true));
        image.data[8] |= 0xF0;
        BOARD_CHECK(board_image_load(&image) == 0);
        for (unsigned slot = 0; slot < 4; ++slot) {
            write_mem(0x4800, 0x9B);
            uint8_t expected = startup[board][slot] < 0 ? 0x9B : (uint8_t)startup[board][slot];
            BOARD_CHECK(read_mem((uint16_t)(0x8000 + slot * 0x2000)) == expected);
        }
        BOARD_CHECK(ppu_read(0x1FFF) == chr_last[board]);
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
        BOARD_CHECK(ppu_read(0x1FFF) == chr_last[board]);
        image.data[7] |= 2;
        BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
        BOARD_CHECK(ppu_read(0x1FFF) == chr_last[board]);
        image.data[0] = 0;
        BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
        BOARD_CHECK(ppu_read(0x1FFF) == chr_last[board]);
        board_image_free(&image);
        BOARD_CHECK(board_image_create(&image, ids[board], 0x2000, 0x100, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        write_mem(0x4800, 0xA5);
        BOARD_CHECK(read_mem(0x8000) == (ids[board] == 193 ? 0xA5 : 0));
        BOARD_CHECK(read_mem(0xFFFF) == 1);
        BOARD_CHECK(ppu_read(0x345) == ((ids[board] == 112 || ids[board] == 298) ? 0 : 0x45));
        BOARD_CHECK(ppu_read(0x845) == 0x45);
        board_image_free(&image);
    }
    return 0;
}

static int test_ntdec_persistence(void) {
    static const unsigned ids[] = {41, 63, 112, 174, 193, 221, 290, 298};
    for (unsigned board = 0; board < sizeof(ids) / sizeof(ids[0]); ++board) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[board], 0x20000, 0, true));
        image.data[6] |= 2;
        image.data[10] = 0x70;
        image.data[11] = 0x70;
        BOARD_CHECK(board_image_add_trainer(&image, 0x9A));
        char path[128], save[128], chr_save[136];
        snprintf(path, sizeof(path), "build/board-ntdec-%u-%lu-%lu.nes", ids[board],
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
        if (ids[board] == 193) {
            // Its writes select banks, so the existing save must stay unchanged.
            uint8_t seed[0x2000] = {0};
            seed[0] = 0xB4;
            memset(seed + 0x1000, 0x9A, 512);
            file = fopen(save, "wb");
            BOARD_CHECK(file != NULL);
            count = fwrite(seed, 1, sizeof(seed), file);
            closed = fclose(file);
            BOARD_CHECK(count == sizeof(seed) && closed == 0);
        }
        BOARD_CHECK(load_rom(path) == 0);
        ppu_power_on(&ppu);
        BOARD_CHECK(cpu_power_on(&cpu) && read_mem(0x7000) == 0x9A);
        write_mem(0x7000, 0x5E);
        ppu_write(0x0023, 0x6F);
        BOARD_CHECK(unload_rom());
        BOARD_CHECK(load_rom(path) == 0);
        ppu_power_on(&ppu);
        BOARD_CHECK(cpu_power_on(&cpu));
        BOARD_CHECK(read_mem(0x7000) == (ids[board] == 193 ? 0x9A : 0x5E));
        if (ids[board] == 193) {
            BOARD_CHECK(read_mem(0x6000) == 0xB4);
            write_mem(0x7000, 0x5E);
        }
        BOARD_CHECK(ppu_read(0x0023) == 0x6F);
        BOARD_CHECK(unload_rom());
        file = fopen(chr_save, "rb");
        BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0);
        long length = ftell(file);
        closed = fclose(file);
        BOARD_CHECK(length == 0x2000 && closed == 0);
        BOARD_CHECK(remove(save) == 0 && remove(chr_save) == 0 && remove(path) == 0);
        board_image_free(&image);
    }
    return 0;
}

int test_board_ntdec_accuracy(void) {
    int failures = 0;
    failures += test_caltron41();
    failures += test_ntdec63();
    failures += test_ntdec112();
    failures += test_ntdec174();
    failures += test_ntdec193();
    failures += test_ntdec221();
    failures += test_ntdec290();
    failures += test_tf1201_banks();
    failures += test_tf1201_irq();
    failures += test_ntdec_geometry_and_replacement();
    failures += test_ntdec_persistence();
    unload_rom();
    printf("NTDEC accuracy: 11 groups, %d failures\n", failures);
    return failures;
}
