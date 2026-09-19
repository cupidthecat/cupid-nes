/*
 * board_unlicensed_111_accuracy.c - Subor and discrete cartridge regressions
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

static int store111(uint16_t address, uint8_t value) {
    const uint8_t code[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(code); ++byte) write_mem((uint16_t)(0x0200 + byte), code[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int mirror111(Mirroring expected) {
    BOARD_CHECK(cart_get_mirroring() == expected);
    ppu_write(0x2000, 0xA5);
    ppu_write(expected == MIRROR_HORIZONTAL ? 0x2800 : 0x2400, 0x5A);
    BOARD_CHECK(ppu_read(expected == MIRROR_HORIZONTAL ? 0x2400 : 0x2800) == 0xA5);
    BOARD_CHECK(ppu_read(0x2C00) == 0x5A && ppu_read(0x3000) == 0xA5);
    return 0;
}

static int test_subor_banks(void) {
    for (unsigned mapper = 166; mapper <= 167; ++mapper) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, mapper, 0x100000, 0x2000, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == (mapper == 166 ? 28 : 128));
        BOARD_CHECK(store111(0x9FFF, 0xFF) == 0 && read_mem(0x8000) == 128);
        BOARD_CHECK(store111(0xDFFF, 7) == 0 && store111(0xFFFF, 2) == 0);
        BOARD_CHECK(read_mem(0x8000) == 148 && read_mem(0xBFFF) == 151);
        BOARD_CHECK(store111(0xBFFF, 8) == 0);
        BOARD_CHECK(read_mem(0x8000) == (mapper == 166 ? 144 : 148));
        BOARD_CHECK(read_mem(0xC000) == (mapper == 166 ? 148 : 144));
        BOARD_CHECK(store111(0xBFFF, 0xFC) == 0);
        BOARD_CHECK(read_mem(0x8000) == (mapper == 166 ? 16 : 20));
        BOARD_CHECK(read_mem(0xC000) == (mapper == 166 ? 20 : 16));
        BOARD_CHECK(store111(0xA000, 4) == 0);
        BOARD_CHECK(read_mem(0x8000) == 124 && read_mem(0xC000) == 148);
        BOARD_CHECK(store111(0xC000, 0xE7) == 0 && read_mem(0xC000) == 148);
        BOARD_CHECK(ppu_read(0) == 0 && ppu_read(0x1FFF) == 7);
        BOARD_CHECK(mirror111(MIRROR_HORIZONTAL) == 0);
        cpu_soft_reset(&cpu);
        BOARD_CHECK(read_mem(0x8000) == 124 && read_mem(0xC000) == 148);
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == (mapper == 166 ? 28 : 128));
        board_image_free(&image);
    }
    return 0;
}

static int test_mapper170_permissions(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 170, 0x10000, 0x2000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_add_trainer(&image, 0x35));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x7000) == 0x35 && read_mem(0x7001) == 0x70 && read_mem(0x7777) == 0x77);
    BOARD_CHECK(store111(0x6502, 0x40) == 0);
    BOARD_CHECK(read_mem(0x6502) == 0 && read_mem(0x7001) == 0xF0 && read_mem(0x7777) == 0xF7);
    BOARD_CHECK(store111(0x7000, 0x80) == 0);
    BOARD_CHECK(read_mem(0x7000) == 0x35 && read_mem(0x7001) == 0x70);
    BOARD_CHECK(store111(0x6503, 0x40) == 0 && read_mem(0x6503) == 0x40 && read_mem(0x7777) == 0x77);
    BOARD_CHECK(store111(0x6502, 0x7F) == 0 && read_mem(0x7777) == 0xF7);
    BOARD_CHECK(store111(0x7001, 0x5A) == 0 && read_mem(0x7001) == 0xF0);
    BOARD_CHECK(store111(0x7777, 0xA5) == 0 && read_mem(0x7777) == 0xF7);
    BOARD_CHECK(store111(0xFFFF, 0xFF) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 7 && ppu_read(0x1FFF) == 7);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x7001) == 0x70 && read_mem(0x7777) == 0x77 && read_mem(0x6503) == 0x40);
    board_image_free(&image);
    return 0;
}

static int test_henggedianzi_banks(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 177, 0x200000, 0x2000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(mirror111(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store111(0xFFFF, 0x21) == 0);
    BOARD_CHECK(read_mem(0x8000) == 8 && read_mem(0x8001) == 1 && read_mem(0xFFFF) == 15);
    BOARD_CHECK(mirror111(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store111(0x8000, 0x1F) == 0 && read_mem(0x8000) == 248);
    BOARD_CHECK(mirror111(MIRROR_VERTICAL) == 0 && ppu_read(0) == 0);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 248 && mirror111(MIRROR_VERTICAL) == 0);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 179, 0x200000, 0x2000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0 && mirror111(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store111(0x5FFF, 0x43) == 0);
    BOARD_CHECK(read_mem(0x8000) == 8 && read_mem(0x8001) == 1);
    BOARD_CHECK(store111(0x6000, 0xA5) == 0 && read_mem(0x6000) == 0xA5);
    BOARD_CHECK(store111(0x7FFF, 0x5A) == 0 && read_mem(0x7FFF) == 0x5A);
    BOARD_CHECK(store111(0x8000, 0) == 0 && mirror111(MIRROR_VERTICAL) == 0);
    BOARD_CHECK(read_mem(0x8000) == 8 && read_mem(0x8001) == 1);
    BOARD_CHECK(store111(0xFFFF, 1) == 0 && mirror111(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(store111(0x5000, 0x42) == 0 && read_mem(0x8001) == 1);
    BOARD_CHECK(store111(0x4FFF, 0xFF) == 0 && read_mem(0x8001) == 1);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8001) == 1 && read_mem(0x6000) == 0xA5 && ppu_read(0x1FFF) == 7);
    board_image_free(&image);
    return 0;
}

static int test_magickid_banks(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 190, 0x40000, 0x80000, true));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == 0 && mirror111(MIRROR_VERTICAL) == 0);
    for (unsigned page = 0; page < 8; ++page) BOARD_CHECK(ppu_read((uint16_t)(page * 0x400)) == page);
    BOARD_CHECK(store111(0x9FFF, 0xFE) == 0 && read_mem(0x8000) == 24 && read_mem(0xC000) == 0);
    BOARD_CHECK(store111(0xDFFF, 0xFA) == 0 && read_mem(0x8000) == 40 && read_mem(0xC000) == 0);
    const uint16_t registers[] = {0xA000, 0xBFFD, 0xE002, 0xFFFF};
    const uint8_t banks[] = {0x81, 0x42, 0x23, 0xE4};
    const uint8_t low[] = {2, 132, 70, 200}, high[] = {1, 0, 0, 1};
    for (unsigned slot = 0; slot < 4; ++slot) BOARD_CHECK(store111(registers[slot], banks[slot]) == 0);
    for (unsigned slot = 0; slot < 4; ++slot) {
        uint16_t address = (uint16_t)(slot * 0x800);
        BOARD_CHECK(ppu_read(address) == low[slot] && ppu_read((uint16_t)(address + 1)) == high[slot]);
        BOARD_CHECK(ppu_read((uint16_t)(address + 0x7FF)) == low[slot] + 1);
        ppu_write(address, 0xAA);
        BOARD_CHECK(ppu_read(address) == low[slot]);
    }
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 40 && read_mem(0xC000) == 0 && ppu_read(0x1800) == 200);
    BOARD_CHECK(board_image_load(&image) == 0 && ppu_read(0x1800) == 6 && read_mem(0x8000) == 0);
    board_image_free(&image);
    return 0;
}

static int test_mapper200_204(void) {
    BoardImage image;
    for (unsigned mapper = 200; mapper <= 204; ++mapper) {
        BOARD_CHECK(board_image_create(&image, mapper, 0x100000, 0x10000, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xC000) == (mapper == 201 ? 4 : 0));
        BOARD_CHECK(mirror111(mapper == 204 ? MIRROR_VERTICAL : MIRROR_HORIZONTAL) == 0);
        if (mapper == 200) {
            BOARD_CHECK(store111(0xFFF5, 0xFF) == 0);
            BOARD_CHECK(read_mem(0x8000) == 20 && read_mem(0xC000) == 20 && ppu_read(0) == 40);
            BOARD_CHECK(mirror111(MIRROR_HORIZONTAL) == 0);
            BOARD_CHECK(store111(0x800D, 0) == 0 && mirror111(MIRROR_VERTICAL) == 0);
            BOARD_CHECK(read_mem(0x8000) == 20 && ppu_read(0) == 40);
        } else if (mapper == 201) {
            BOARD_CHECK(store111(0xFFF6, 0) == 0);
            BOARD_CHECK(read_mem(0x8000) == 16 && read_mem(0xC000) == 20 && ppu_read(0) == 48);
            BOARD_CHECK(store111(0xFFF6, 0xFF) == 0 && read_mem(0x8000) == 16 && ppu_read(0) == 48);
        } else if (mapper == 202) {
            BOARD_CHECK(store111(0xFFF9, 0) == 0);
            BOARD_CHECK(read_mem(0x8000) == 16 && read_mem(0xC000) == 20 && ppu_read(0) == 32);
            BOARD_CHECK(mirror111(MIRROR_HORIZONTAL) == 0);
            BOARD_CHECK(store111(0x800E, 0xFF) == 0);
            BOARD_CHECK(read_mem(0x8000) == 28 && read_mem(0xC000) == 28 && ppu_read(0) == 56);
            BOARD_CHECK(mirror111(MIRROR_VERTICAL) == 0);
        } else if (mapper == 203) {
            BOARD_CHECK(store111(0xFFFF, 0xD7) == 0);
            BOARD_CHECK(read_mem(0x8000) == 212 && read_mem(0xC000) == 212 && ppu_read(0) == 24);
            BOARD_CHECK(mirror111(MIRROR_HORIZONTAL) == 0);
        } else {
            BOARD_CHECK(store111(0x8015, 0) == 0);
            BOARD_CHECK(read_mem(0x8000) == 20 && read_mem(0xC000) == 20 && ppu_read(0) == 40);
            BOARD_CHECK(mirror111(MIRROR_HORIZONTAL) == 0);
            BOARD_CHECK(store111(0xFFE7, 0xFF) == 0);
            BOARD_CHECK(read_mem(0x8000) == 24 && read_mem(0xC000) == 28 && ppu_read(0) == 48);
            BOARD_CHECK(mirror111(MIRROR_VERTICAL) == 0);
        }
        uint8_t prg = read_mem(0x8000), chr = ppu_read(0);
        Mirroring mirroring = cart_get_mirroring();
        cpu_soft_reset(&cpu);
        BOARD_CHECK(read_mem(0x8000) == prg && ppu_read(0) == chr && cart_get_mirroring() == mirroring);
        board_image_free(&image);
    }
    return 0;
}

static int test_mapper212_read_mask(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 212, 0x40000, 0x10000, true));
    image.data[10] = 7;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(store111(0x6000, 0x25) == 0 && store111(0x6010, 0x35) == 0);
    BOARD_CHECK(store111(0x7020, 0x46) == 0 && store111(0x7FF0, 0x57) == 0);
    BOARD_CHECK(read_mem(0x6000) == 0xA5 && read_mem(0x6010) == 0x35);
    BOARD_CHECK(read_mem(0x7020) == 0xC6 && read_mem(0x7FF0) == 0x57);
    BOARD_CHECK(store111(0x800D, 0xFF) == 0 && mirror111(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(read_mem(0x8000) == 20 && read_mem(0xC000) == 20 && ppu_read(0) == 40);
    BOARD_CHECK(store111(0xC006, 0) == 0 && mirror111(MIRROR_VERTICAL) == 0);
    BOARD_CHECK(read_mem(0x8000) == 24 && read_mem(0xC000) == 28 && ppu_read(0) == 48);
    BOARD_CHECK(read_mem(0xBFFF) == 27 && read_mem(0xFFFF) == 31);
    BOARD_CHECK(store111(0xFFFF, 0xFF) == 0 && mirror111(MIRROR_HORIZONTAL) == 0);
    BOARD_CHECK(read_mem(0x8000) == 24 && read_mem(0xC000) == 28 && ppu_read(0) == 56);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 24 && ppu_read(0) == 56 && read_mem(0x6000) == 0xA5);
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x8000) == 0 && ppu_read(0) == 0);
    image.data[10] = 0;
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x4018, 0xA5);
    BOARD_CHECK(read_mem(0x6000) == 0x80 && read_mem(0x6010) == 0);
    BOARD_CHECK(store111(0x7000, 0xFF) == 0 && read_mem(0x7000) == 0x80);
    board_image_free(&image);
    return 0;
}

static int test_discrete111_small_geometry(void) {
    const unsigned ids[] = {166, 167, 170, 177, 179, 190, 200, 201, 202, 203, 204, 212};
    const uint16_t regs[] = {0x9FFF, 0x9FFF, 0x6502, 0xFFFF, 0x5000, 0xA000,
                              0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
    for (unsigned index = 0; index < sizeof(ids) / sizeof(ids[0]); ++index) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, ids[index], 0x6000, 0x600, true));
        image.data[8] |= 0xF0;
        memset(image.data + 16, 0xA7, 0x6000);
        memset(image.data + 16 + 0x6000, 0xB8, 0x600);
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(store111(regs[index], 0xFF) == 0);
        BOARD_CHECK(read_mem(0x8000) == 0xA7 && ppu_read(0x0123) == 0xB8);
        BOARD_CHECK(ppu_read(0x1F23) == 0x23);
        ppu_write(0x0123, 0xCC);
        BOARD_CHECK(ppu_read(0x0123) == 0xB8);
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
        BOARD_CHECK(read_mem(0x8000) == 0xA7 && ppu_read(0x0123) == 0xB8);
        board_image_set_unsupported_console(&image);
        BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
        BOARD_CHECK(read_mem(0x8000) == 0xA7 && ppu_read(0x0123) == 0xB8);
        board_image_free(&image);

        BOARD_CHECK(board_image_create(&image, ids[index], 0x6000, 0, true));
        image.data[11] = 3;
        BOARD_CHECK(board_image_load(&image) == 0);
        ppu_write(0x0123, 0x5A);
        BOARD_CHECK(ppu_read(0x1F23) == 0x5A);
        BOARD_CHECK(store111(regs[index], 0xFF) == 0);
        ppu_write(0x1F23, 0xA5);
        BOARD_CHECK(ppu_read(0x0123) == 0xA5);
        board_image_free(&image);
    }
    return 0;
}

static int test_discrete111_battery_and_trainer(void) {
    const unsigned mappers[] = {170, 212};
    for (unsigned index = 0; index < 2; ++index) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, mappers[index], 0x8000, 0x2000, true));
        image.data[6] |= 2;
        image.data[10] = 0x70;
        BOARD_CHECK(board_image_add_trainer(&image, 0x35));
        char stem[100], path[120], save[120];
        snprintf(stem, sizeof(stem), "build/discrete111-%u-%lu-%lu", mappers[index],
                 (unsigned long)time(NULL), (unsigned long)clock());
        snprintf(path, sizeof(path), "%s.nes", stem);
        snprintf(save, sizeof(save), "%s.sav", stem);
        FILE *file = fopen(path, "wb");
        BOARD_CHECK(file != NULL);
        size_t written = fwrite(image.data, 1, image.size, file);
        int closed = fclose(file);
        BOARD_CHECK(written == image.size && closed == 0 && load_rom(path) == 0);
        ppu_power_on(&ppu);
        BOARD_CHECK(cpu_power_on(&cpu));
        BOARD_CHECK(read_mem(0x7000) == (index ? 0xB5 : 0x35));
        BOARD_CHECK(store111(0x7001, 0x5A) == 0 && store111(0x7777, 0x6B) == 0);
        BOARD_CHECK(store111(0x6502, 0x40) == 0);
        BOARD_CHECK(read_mem(0x7001) == (index ? 0xDA : 0xF0));
        BOARD_CHECK(unload_rom());
        file = fopen(save, "rb");
        BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0 && ftell(file) == 0x2000);
        BOARD_CHECK(fseek(file, 0x1001, SEEK_SET) == 0 && fgetc(file) == 0x5A);
        BOARD_CHECK(fseek(file, 0x1777, SEEK_SET) == 0 && fgetc(file) == 0x6B && fclose(file) == 0);
        BOARD_CHECK(load_rom(path) == 0);
        ppu_power_on(&ppu);
        BOARD_CHECK(cpu_power_on(&cpu));
        // Battery data is loaded after the trainer, preserving saved bytes.
        BOARD_CHECK(read_mem(0x7001) == (index ? 0xDA : 0x70));
        BOARD_CHECK(read_mem(0x7777) == (index ? 0x6B : 0x77));
        BOARD_CHECK(unload_rom() && remove(path) == 0 && remove(save) == 0);
        board_image_free(&image);
    }
    return 0;
}

int test_board_unlicensed_111_accuracy(void) {
    int failures = 0;
    failures += test_subor_banks();
    failures += test_mapper170_permissions();
    failures += test_henggedianzi_banks();
    failures += test_magickid_banks();
    failures += test_mapper200_204();
    failures += test_mapper212_read_mask();
    failures += test_discrete111_small_geometry();
    failures += test_discrete111_battery_and_trainer();
    unload_rom();
    printf("Discrete cartridge group 111: 8 groups, %d failures\n", failures);
    return failures;
}
