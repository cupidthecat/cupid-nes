/*
 * default_prg_ram_geometry_accuracy.c - Fixed PRG-RAM window geometry regressions
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
#include "../apu/apu.h"
#include <time.h>

static const unsigned fixed_ram_boards[] = {
    4, 9, 10, 15, 18, 21, 22, 23, 24, 25, 26, 27, 28, 32, 33, 48,
    64, 65, 67, 68, 71, 72, 73, 74, 75, 76, 78, 85, 88, 89, 92, 93,
    95, 96, 97, 105, 118, 119, 151, 154, 158, 185, 191, 192, 194,
    195, 206, 232
};

static const unsigned register_owned_ram_boards[] = {87, 101, 140, 184};

static bool write_file(const char *path, const uint8_t *bytes, size_t length);
static bool read_file_exact(const char *path, uint8_t *bytes, size_t length);

static bool cpu_store(uint16_t address, uint8_t value) {
    write_mem(0x0200, 0xA9);
    write_mem(0x0201, value);
    write_mem(0x0202, 0x8D);
    write_mem(0x0203, (uint8_t)address);
    write_mem(0x0204, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4;
}

static bool cpu_load_is(uint16_t address, uint8_t expected) {
    write_mem(0x0200, 0xAD);
    write_mem(0x0201, (uint8_t)address);
    write_mem(0x0202, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 4 && cpu.a == expected;
}

static bool power_cart(void) {
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    return cpu_power_on(&cpu);
}

static uint8_t mixed_chr_shift(unsigned mapper) {
    switch (mapper) {
        case 74: case 191: case 194: return 5; /* 2 KiB */
        case 192: case 195: return 6;          /* 4 KiB */
        case 119: return 7;                    /* 8 KiB */
        default: return 0;
    }
}

static bool make_image(BoardImage *image, unsigned mapper, uint8_t ram_sizes,
                       bool trainer) {
    size_t chr_bytes = 0x2000;
    if (mapper == 28 || mapper == 96) chr_bytes = 0;
    if (!board_image_create(image, mapper, 0x20000, chr_bytes, true)) return false;
    iNESHeader *header = (iNESHeader *)image->data;
    header->flags10 = ram_sizes;
    if (ram_sizes & 0xF0u) header->flags6 |= 2;
    if (mapper == 28) header->zero[0] = 7;     /* 8 KiB CHR RAM */
    if (mapper == 96) header->zero[0] = 9;     /* 32 KiB CHR RAM */
    uint8_t extra_chr = mixed_chr_shift(mapper);
    if (extra_chr) header->zero[0] = extra_chr;
    return !trainer || board_image_add_trainer(image, 0xC1);
}

static void enable_ram(unsigned mapper) {
    switch (mapper) {
        case 4: case 74: case 118: case 119: case 191: case 192:
        case 194: case 195:
            cart_cpu_write(0xA001, 0x80);
            break;
        case 68:
            cart_cpu_write(0xF000, 0x10);
            break;
        case 85:
            cart_cpu_write(0xE000, 0x80);
            break;
        default:
            break;
    }
}

static void exercise_bank_register(unsigned mapper) {
    switch (mapper) {
        case 4: case 74: case 118: case 119: case 191: case 192:
        case 194: case 195:
            cart_cpu_write(0x8000, 6);
            cart_cpu_write(0x8001, 1);
            break;
        case 9: case 10: cart_cpu_write(0xA000, 1); break;
        case 15: case 18: case 32: case 33: case 48: case 64: case 65:
        case 71: case 75: case 89: case 93: case 97: case 105: case 151:
        case 158:
            cart_cpu_write(0x8000, 1);
            break;
        case 21: case 22: case 23: case 25: case 27:
            cart_cpu_write(0x8000, 1);
            break;
        case 24: case 26:
            cart_cpu_write(0x8000, 1);
            cart_cpu_write(0xB003, 0x80);
            break;
        case 28:
            cart_cpu_write(0x5000, 0x81);
            cart_cpu_write(0x8000, 1);
            break;
        case 67: cart_cpu_write(0xF800, 1); break;
        case 68: cart_cpu_write(0xF000, 0x11); break;
        case 73: cart_cpu_write(0xF000, 1); break;
        case 76: case 88: case 95: case 154: case 206:
            cart_cpu_write(0x8000, 6);
            cart_cpu_write(0x8001, 1);
            break;
        case 85:
            cart_cpu_write(0x8000, 1);
            cart_cpu_write(0xE000, 0x80);
            break;
        case 232: cart_cpu_write(0xC000, 1); break;
        default:
            break;
    }
}

static int test_fixed_window_matrix(void) {
    /* 8 KiB work + 16 KiB save is deliberately unequal and non-power-of-two
       in total. The battery-backed save chip owns the fixed CPU window; the
       trainer remains on the separate work chip. */
    for (size_t i = 0; i < sizeof(fixed_ram_boards) / sizeof(fixed_ram_boards[0]); ++i) {
        unsigned mapper = fixed_ram_boards[i];
        BoardImage image;
        BOARD_CHECK(make_image(&image, mapper, 0x87, true));
        BOARD_CHECK(board_image_load(&image) == 0 && power_cart());
        enable_ram(mapper);
        BOARD_CHECK(cpu_load_is(0x7000, 0));
        BOARD_CHECK(cpu_store(0x6001, 0xA3));
        BOARD_CHECK(cpu_store(0x7FFF, 0x5D));
        BOARD_CHECK(cpu_load_is(0x6001, 0xA3));
        BOARD_CHECK(cpu_load_is(0x7001, 0));
        BOARD_CHECK(cpu_load_is(0x7FFF, 0x5D));

        exercise_bank_register(mapper);
        enable_ram(mapper);
        BOARD_CHECK(cpu_load_is(0x6001, 0xA3));
        BOARD_CHECK(cpu_load_is(0x7FFF, 0x5D));

        uint8_t *previous_prg = prg_rom;
        uint32_t previous_crc = rom_file_crc32();
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) == -1);
        BOARD_CHECK(prg_rom == previous_prg && rom_file_crc32() == previous_crc);
        BOARD_CHECK(cpu_load_is(0x6001, 0xA3));
        BOARD_CHECK(unload_rom());
        board_image_free(&image);
    }
    return 0;
}

static int test_register_owned_read_windows(void) {
    for (size_t i = 0; i < sizeof(register_owned_ram_boards) / sizeof(register_owned_ram_boards[0]); ++i) {
        BoardImage image;
        unsigned mapper = register_owned_ram_boards[i];
        BOARD_CHECK(make_image(&image, mapper, 0x80, false)); /* 16 KiB save RAM. */
        char rom_path[160], save_path[160];
        int used = snprintf(rom_path, sizeof(rom_path), "build/default-prg-read-%u-%lu-%lu.nes",
                            mapper, (unsigned long)time(NULL), (unsigned long)clock());
        BOARD_CHECK(used > 0 && (size_t)used < sizeof(rom_path));
        memcpy(save_path, rom_path, (size_t)used + 1);
        strcpy(strrchr(save_path, '.'), ".sav");
        uint8_t saved[0x4000] = {0};
        saved[0] = 0x42;
        saved[0x1000] = 0x69;
        BOARD_CHECK(write_file(rom_path, image.data, image.size));
        BOARD_CHECK(write_file(save_path, saved, sizeof(saved)));
        BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
        BOARD_CHECK(cpu_load_is(0x6000, 0x42) && cpu_load_is(0x7000, 0x69));

        /* These writes belong to mapper registers. They must not mutate the
           RAM bytes that are still visible on reads from the same addresses. */
        BOARD_CHECK(cpu_store(0x6000, 0x03));
        BOARD_CHECK(cpu_load_is(0x6000, 0x42) && cpu_load_is(0x7000, 0x69));
        BOARD_CHECK(unload_rom());
        BOARD_CHECK(remove(save_path) == 0 && remove(rom_path) == 0);
        board_image_free(&image);
    }
    return 0;
}

static int test_bank_controls_preserve_ram(void) {
    static const unsigned boards[] = {28, 68, 85, 206, 232};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        BoardImage image;
        unsigned mapper = boards[i];
        BOARD_CHECK(make_image(&image, mapper, 0x08, false));
        BOARD_CHECK(board_image_load(&image) == 0 && power_cart());
        enable_ram(mapper);
        BOARD_CHECK(cpu_store(0x6002, 0xB4));
        uint8_t before = cart_cpu_read(0x8000);
        exercise_bank_register(mapper);
        if (mapper == 68 || mapper == 206 || mapper == 232)
            BOARD_CHECK(cart_cpu_read(0x8000) != before);
        enable_ram(mapper);
        BOARD_CHECK(cpu_load_is(0x6002, 0xB4));
        BOARD_CHECK(unload_rom());
        board_image_free(&image);
    }
    return 0;
}

static bool write_file(const char *path, const uint8_t *bytes, size_t length) {
    FILE *file = fopen(path, "wb");
    if (!file) return false;
    bool ok = fwrite(bytes, 1, length, file) == length;
    return fclose(file) == 0 && ok;
}

static bool read_file_exact(const char *path, uint8_t *bytes, size_t length) {
    FILE *file = fopen(path, "rb");
    if (!file) return false;
    bool ok = fread(bytes, 1, length, file) == length && fgetc(file) == EOF;
    return fclose(file) == 0 && ok;
}

static int test_save_persistence_and_trainer(void) {
    BoardImage image;
    BOARD_CHECK(make_image(&image, 33, 0x80, true)); /* save-only 16 KiB */
    char rom_path[160], save_path[160];
    int used = snprintf(rom_path, sizeof(rom_path), "build/default-prg-ram-%lu-%lu.nes",
                        (unsigned long)time(NULL), (unsigned long)clock());
    BOARD_CHECK(used > 0 && (size_t)used < sizeof(rom_path));
    memcpy(save_path, rom_path, (size_t)used + 1);
    strcpy(strrchr(save_path, '.'), ".sav");
    BOARD_CHECK(write_file(rom_path, image.data, image.size));
    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
    BOARD_CHECK(cpu_load_is(0x7000, 0xC1) && cpu_load_is(0x71FF, 0xC1));
    BOARD_CHECK(cpu_store(0x6001, 0xD2) && cpu_store(0x7FFF, 0x88));
    cart_battery_flush();
    uint8_t saved[0x4000];
    BOARD_CHECK(read_file_exact(save_path, saved, sizeof(saved)));
    BOARD_CHECK(saved[1] == 0xD2 && saved[0x1FFF] == 0x88);
    BOARD_CHECK(saved[0x1000] == 0xC1 && saved[0x11FF] == 0xC1);
    for (size_t i = 0x2000; i < sizeof(saved); ++i) BOARD_CHECK(saved[i] == 0);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
    BOARD_CHECK(cpu_load_is(0x6001, 0xD2) && cpu_load_is(0x7FFF, 0x88));
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(remove(save_path) == 0 && remove(rom_path) == 0);
    board_image_free(&image);
    return 0;
}

static int test_small_pages_and_open_bus(void) {
    BoardImage image;
    BOARD_CHECK(make_image(&image, 32, 0x02, false)); /* 256-byte work page */
    BOARD_CHECK(board_image_load(&image) == 0 && power_cart());
    BOARD_CHECK(cpu_store(0x6000, 0xA6) && cpu_store(0x60FF, 0x5C));
    BOARD_CHECK(cpu_load_is(0x6100, 0xA6) && cpu_load_is(0x7FFF, 0x5C));
    BOARD_CHECK(unload_rom());
    board_image_free(&image);

    BOARD_CHECK(make_image(&image, 32, 0x01, false)); /* 128 bytes: no CPU bus page */
    BOARD_CHECK(board_image_load(&image) == 0 && power_cart());
    BOARD_CHECK(cart_cpu_read_bus(0x6000, 0xD1) == 0xD1);
    BOARD_CHECK(cart_cpu_read_bus(0x7FFF, 0xE2) == 0xE2);
    BOARD_CHECK(cpu_store(0x6000, 0x77));
    BOARD_CHECK(cart_cpu_read_bus(0x6000, 0xC3) == 0xC3);
    BOARD_CHECK(unload_rom());
    board_image_free(&image);
    return 0;
}

int test_default_prg_ram_geometry_accuracy(void) {
    int failures = test_fixed_window_matrix();
    failures += test_register_owned_read_windows();
    failures += test_bank_controls_preserve_ram();
    failures += test_save_persistence_and_trainer();
    failures += test_small_pages_and_open_bus();
    unload_rom();
    printf("Default PRG-RAM geometry: 5 groups, %d failures\n", failures);
    return failures;
}
