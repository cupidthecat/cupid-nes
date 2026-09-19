/*
 * board_mmc3_mixed_chr_accuracy.c - MMC3 mixed CHR source and geometry regressions
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
#include "../system/hardware.h"
#include <time.h>

typedef struct {
    unsigned mapper;
    uint8_t first_ram_bank;
    uint8_t last_ram_bank;
    uint8_t default_ram_shift;
    unsigned default_ram_pages;
} MixedChrCase;

static const MixedChrCase mixed_chr_cases[] = {
    {74,  0x08, 0x09, 5, 2},
    {119, 0x40, 0x7F, 7, 8},
    {191, 0x80, 0xFF, 5, 2},
    {192, 0x08, 0x0B, 6, 4},
    {194, 0x00, 0x01, 5, 2},
    {195, 0x00, 0x03, 6, 4},
};

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

static bool cpu_three_cycles(void) {
    write_mem(0x0200, 0x4C);
    write_mem(0x0201, 0x00);
    write_mem(0x0202, 0x02);
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 3;
}

static bool power_cart(void) {
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    return cpu_power_on(&cpu);
}

static bool select_chr_bank(uint8_t bank) {
    return cpu_store(0x8000, 2) && cpu_store(0x8001, bank);
}

static bool select_chr_pair(uint8_t first_bank) {
    return cpu_store(0x8000, 0) && cpu_store(0x8001, first_bank);
}

static uint8_t outside_ram_bank(const MixedChrCase *c) {
    return c->first_ram_bank ? (uint8_t)(c->first_ram_bank - 1)
                             : (uint8_t)(c->last_ram_bank + 1);
}

static bool irq_once(void) {
    if (!cpu_store(0xC000, 0) || !cpu_store(0xC001, 0)
        || !cpu_store(0xE001, 0)) return false;
    cart_notify_ppu_address(0x0000, 1);
    if (!cpu_three_cycles()) return false;
    cart_notify_ppu_address(0x1000, 2);
    if (!cart_irq_pending()) return false;
    if (!cpu_store(0xE000, 0)) return false;
    return !cart_irq_pending();
}

static bool write_file(const char *path, const uint8_t *bytes, size_t size) {
    FILE *file = fopen(path, "wb");
    if (!file) return false;
    bool ok = fwrite(bytes, 1, size, file) == size;
    return fclose(file) == 0 && ok;
}

static long file_size(const char *path) {
    FILE *file = fopen(path, "rb");
    if (!file) return -1;
    long size = fseek(file, 0, SEEK_END) == 0 ? ftell(file) : -1;
    fclose(file);
    return size;
}

static int file_byte(const char *path, long offset) {
    FILE *file = fopen(path, "rb");
    if (!file) return -1;
    int value = fseek(file, offset, SEEK_SET) == 0 ? fgetc(file) : -1;
    fclose(file);
    return value;
}

static int test_ranges_cpu_ppu_and_irq(void) {
    for (size_t n = 0; n < sizeof(mixed_chr_cases) / sizeof(mixed_chr_cases[0]); ++n) {
        const MixedChrCase *c = &mixed_chr_cases[n];
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, c->mapper, 0x20000, 0x40000, false));
        BOARD_CHECK(board_image_load(&image) == 0);

        BOARD_CHECK(cpu_store(0x8000, 6) && cpu_store(0x8001, 3));
        BOARD_CHECK(cpu_load_is(0x8000, 6) && cpu_load_is(0xC000, 28));
        BOARD_CHECK(cpu_store(0xA001, 0x80));
        BOARD_CHECK(cpu_store(0x6123, (uint8_t)(0x50 + n)));
        BOARD_CHECK(cpu_load_is(0x6123, (uint8_t)(0x50 + n)));

        uint8_t outside = outside_ram_bank(c);
        BOARD_CHECK(select_chr_bank(outside));
        BOARD_CHECK(ppu_read(0x1000) == outside);
        ppu_write(0x1000, 0xEE);
        BOARD_CHECK(ppu_read(0x1000) == outside);

        BOARD_CHECK(select_chr_pair(c->first_ram_bank));
        ppu_write(0x0000, (uint8_t)(0xA0 + n));
        ppu_write(0x0400, (uint8_t)(0xB0 + n));
        BOARD_CHECK(ppu_read(0x0000) == (uint8_t)(0xA0 + n));
        BOARD_CHECK(ppu_read(0x0400) == (uint8_t)(0xB0 + n));

        BOARD_CHECK(select_chr_bank(c->last_ram_bank));
        ppu_write(0x1000, (uint8_t)(0xC0 + n));
        BOARD_CHECK(ppu_read(0x1000) == (uint8_t)(0xC0 + n));
        BOARD_CHECK(select_chr_bank(c->first_ram_bank));
        BOARD_CHECK(ppu_read(0x1000) == (uint8_t)(0xA0 + n));

        unsigned range_pages = (unsigned)c->last_ram_bank - c->first_ram_bank + 1;
        if (range_pages > c->default_ram_pages) {
            BOARD_CHECK(select_chr_bank((uint8_t)(c->first_ram_bank + c->default_ram_pages)));
            BOARD_CHECK(ppu_read(0x1000) == (uint8_t)(0xA0 + n));
        }

        BOARD_CHECK(select_chr_bank(c->first_ram_bank));
        cart->reset();
        BOARD_CHECK(cpu_load_is(0x8000, 6));
        BOARD_CHECK(ppu_read(0x1000) == (uint8_t)(0xA0 + n));

        BOARD_CHECK(irq_once());
        board_image_free(&image);
    }
    return 0;
}

static int test_ram_only_default_source_and_truncation(void) {
    for (size_t n = 0; n < sizeof(mixed_chr_cases) / sizeof(mixed_chr_cases[0]); ++n) {
        const MixedChrCase *c = &mixed_chr_cases[n];
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, c->mapper, 0x20000, 0, false));
        BOARD_CHECK(board_image_load(&image) == 0);

        uint8_t outside = outside_ram_bank(c);
        BOARD_CHECK(select_chr_bank(outside));
        ppu_write(0x1000, (uint8_t)(0x60 + n));
        BOARD_CHECK(ppu_read(0x1000) == (uint8_t)(0x60 + n));

        BOARD_CHECK(select_chr_bank(c->first_ram_bank));
        ppu_write(0x1000, (uint8_t)(0x80 + n));
        BOARD_CHECK(ppu_read(0x1000) == (uint8_t)(0x80 + n));
        BOARD_CHECK(cpu_store(0x8000, 6) && cpu_store(0x8001, 3));
        BOARD_CHECK(cpu_load_is(0x8000, 6));

        uint8_t *previous_prg = prg_rom;
        uint32_t previous_crc = rom_file_crc32();
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) == -1);
        BOARD_CHECK(prg_rom == previous_prg && rom_file_crc32() == previous_crc);
        BOARD_CHECK(cpu_load_is(0x8000, 6));
        BOARD_CHECK(select_chr_bank(c->first_ram_bank));
        BOARD_CHECK(ppu_read(0x1000) == (uint8_t)(0x80 + n));

        board_image_set_unsupported_console(&image);
        BOARD_CHECK(load_rom_memory(image.data, image.size) == -1);
        BOARD_CHECK(prg_rom == previous_prg && rom_file_crc32() == previous_crc);
        BOARD_CHECK(cpu_load_is(0x8000, 6));
        BOARD_CHECK(select_chr_bank(c->first_ram_bank));
        BOARD_CHECK(ppu_read(0x1000) == (uint8_t)(0x80 + n));
        board_image_free(&image);
    }
    return 0;
}

static int test_declared_chr_geometries(void) {
    for (size_t n = 0; n < sizeof(mixed_chr_cases) / sizeof(mixed_chr_cases[0]); ++n) {
        const MixedChrCase *c = &mixed_chr_cases[n];
        BoardImage image;
        iNESHeader *header;

        BOARD_CHECK(board_image_create(&image, c->mapper, 0x20000, 0x40000, true));
        header = (iNESHeader *)image.data;
        header->flags10 = 7;
        header->zero[0] = 0;
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(select_chr_bank(c->first_ram_bank));
        BOARD_CHECK(ppu_read(0x1000) == 4);
        ppu_write(0x1000, 0xEE);
        BOARD_CHECK(ppu_read(0x1000) == 4);
        board_image_free(&image);

        BOARD_CHECK(board_image_create(&image, c->mapper, 0x20000, 0x40000, true));
        header = (iNESHeader *)image.data;
        header->flags10 = 7;
        header->zero[0] = 3; /* 512 bytes: smaller than one MMC3 CHR page. */
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(select_chr_bank(c->first_ram_bank));
        ppu_write(0x0812, (uint8_t)(0x90 + n));
        BOARD_CHECK(ppu_read(0x0812) == (uint8_t)(0x90 + n));
        BOARD_CHECK(ppu_read(0x1012) == 4);
        board_image_free(&image);

        BOARD_CHECK(board_image_create(&image, c->mapper, 0x20000, 0x40000, true));
        header = (iNESHeader *)image.data;
        header->flags10 = 7;
        header->zero[0] = 4; /* 1 KiB: exactly one MMC3 CHR-RAM page. */
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(select_chr_bank(c->first_ram_bank));
        ppu_write(0x1000, (uint8_t)(0x70 + n));
        BOARD_CHECK(select_chr_bank((uint8_t)(c->first_ram_bank + 1)));
        BOARD_CHECK(ppu_read(0x1000) == (uint8_t)(0x70 + n));
        board_image_free(&image);

        BOARD_CHECK(board_image_create(&image, c->mapper, 0x20000, 0x40000, true));
        header = (iNESHeader *)image.data;
        header->flags10 = 7;
        header->zero[0] = 8; /* 16 KiB, larger than every board default here. */
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(select_chr_bank(c->first_ram_bank));
        ppu_write(0x1000, (uint8_t)(0xA0 + n));
        BOARD_CHECK(select_chr_bank((uint8_t)(c->first_ram_bank + 1)));
        ppu_write(0x1000, (uint8_t)(0xB0 + n));
        BOARD_CHECK(select_chr_bank(c->first_ram_bank));
        BOARD_CHECK(ppu_read(0x1000) == (uint8_t)(0xA0 + n));
        BOARD_CHECK(select_chr_bank((uint8_t)(c->first_ram_bank + 1)));
        BOARD_CHECK(ppu_read(0x1000) == (uint8_t)(0xB0 + n));
        board_image_free(&image);

        BOARD_CHECK(board_image_create(&image, c->mapper, 0x20000, 0x40000, true));
        header = (iNESHeader *)image.data;
        header->flags6 |= 2;
        header->flags10 = 0;
        header->zero[0] = (uint8_t)(c->default_ram_shift << 4);
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(select_chr_bank(c->first_ram_bank));
        ppu_write(0x1000, (uint8_t)(0xD0 + n));
        BOARD_CHECK(ppu_read(0x1000) == (uint8_t)(0xD0 + n));
        board_image_free(&image);

        BOARD_CHECK(board_image_create(&image, c->mapper, 0x20000, 0, true));
        header = (iNESHeader *)image.data;
        header->flags10 = 7;
        header->zero[0] = 0;
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(select_chr_bank(c->first_ram_bank));
        BOARD_CHECK(ppu_read(0x1123) == 0x23);
        ppu_write(0x1123, 0x5A);
        BOARD_CHECK(ppu_read(0x1123) == 0x23);
        board_image_free(&image);
    }
    return 0;
}

static int test_chr_nvram_persistence_and_short_save(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 119, 0x20000, 0x40000, true));
    iNESHeader *header = (iNESHeader *)image.data;
    header->flags6 |= 2;
    header->flags10 = 0;
    header->zero[0] = 0x44; /* 1 KiB volatile RAM followed by 1 KiB NVRAM. */

    char rom_path[160], save_path[160];
    int used = snprintf(rom_path, sizeof(rom_path), "build/mmc3-mixed-chr-%lu-%lu.nes",
                        (unsigned long)time(NULL), (unsigned long)clock());
    BOARD_CHECK(used > 0 && (size_t)used < sizeof(rom_path));
    memcpy(save_path, rom_path, (size_t)used + 1);
    strcpy(strrchr(save_path, '.'), ".chr.sav");
    BOARD_CHECK(write_file(rom_path, image.data, image.size));
    BOARD_CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_ZERO));

    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
    BOARD_CHECK(select_chr_bank(0x40));
    ppu_write(0x1000, 0xA1);
    BOARD_CHECK(select_chr_bank(0x41));
    ppu_write(0x1056, 0xD4);
    cart_battery_flush();
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(file_size(save_path) == 0x400 && file_byte(save_path, 0x56) == 0xD4);

    uint8_t short_save[32];
    memset(short_save, 0xA6, sizeof(short_save));
    BOARD_CHECK(write_file(save_path, short_save, sizeof(short_save)));
    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
    BOARD_CHECK(select_chr_bank(0x40));
    BOARD_CHECK(ppu_read(0x1000) == 0);
    BOARD_CHECK(select_chr_bank(0x41));
    BOARD_CHECK(ppu_read(0x1000) == 0xA6 && ppu_read(0x1020) == 0);
    ppu_write(0x1020, 0xE4);
    cart_battery_flush();
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(file_size(save_path) == 0x400);
    BOARD_CHECK(file_byte(save_path, 0) == 0xA6 && file_byte(save_path, 0x20) == 0xE4);

    BOARD_CHECK(remove(save_path) == 0 && remove(rom_path) == 0);
    BOARD_CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_DEFAULT));
    board_image_free(&image);
    return 0;
}

int test_board_mmc3_mixed_chr_accuracy(void) {
    int failures = 0;
    failures += test_ranges_cpu_ppu_and_irq();
    failures += test_ram_only_default_source_and_truncation();
    failures += test_declared_chr_geometries();
    failures += test_chr_nvram_persistence_and_short_save();
    unload_rom();
    (void)nes_set_ram_power_on_state(NES_RAM_POWER_DEFAULT);
    printf("MMC3 mixed CHR: 4 groups, %d failures\n", failures);
    return failures;
}
