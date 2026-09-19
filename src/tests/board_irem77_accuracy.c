/*
 * board_irem77_accuracy.c - Irem LROG017 CHR source and banking regressions
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
#include "../system/hardware.h"
#include <time.h>

static bool write_file(const char *path, const uint8_t *data, size_t size) {
    FILE *file = fopen(path, "wb");
    if (!file) return false;
    bool ok = fwrite(data, 1, size, file) == size;
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

static uint8_t *prg_data(BoardImage *image) {
    return image->data + sizeof(iNESHeader);
}

static int test_rom_and_default_ram(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 77, 0x10000, 0x4000, false));
    prg_data(&image)[0x100] = 0xFF; // Permit an unmasked register write at $8100.
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_FOUR);
    BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 7);
    BOARD_CHECK(ppu_read(0x0000) == 0 && ppu_read(0x0400) == 1);

    ppu_write(0x0801, 0x35);
    ppu_write(0x1002, 0x53);
    ppu_write(0x1803, 0x69);
    BOARD_CHECK(ppu_read(0x0801) == 0x35 && ppu_read(0x1002) == 0x53);
    BOARD_CHECK(ppu_read(0x1803) == 0x69);
    write_mem(0x6123, 0xA6);
    BOARD_CHECK(read_mem(0x6123) == 0xA6);

    ppu_write(0x2000, 0x10);
    ppu_write(0x2400, 0x20);
    ppu_write(0x2800, 0x30);
    ppu_write(0x2C00, 0x40);
    BOARD_CHECK(ppu_read(0x2000) == 0x10 && ppu_read(0x2400) == 0x20);
    BOARD_CHECK(ppu_read(0x2800) == 0x30 && ppu_read(0x2C00) == 0x40);

    write_mem(0x8000, 0xFF); // ROM drives zero, so the conflict keeps bank zero.
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0x0000) == 0);
    write_mem(0x8100, 0x21);
    BOARD_CHECK(read_mem(0x8000) == 8 && read_mem(0xFFFF) == 15);
    BOARD_CHECK(ppu_read(0x0000) == 4 && ppu_read(0x0400) == 5);
    BOARD_CHECK(ppu_read(0x0801) == 0x35 && ppu_read(0x1002) == 0x53);
    BOARD_CHECK(ppu_read(0x1803) == 0x69);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 8 && ppu_read(0x0000) == 4);
    BOARD_CHECK(read_mem(0x6123) == 0xA6 && cart_get_mirroring() == MIRROR_FOUR);
    board_image_free(&image);
    return 0;
}

static int test_shrunk_ram_overlap(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 77, 0x10000, 0x0800, true));
    ((iNESHeader *)image.data)->zero[0] = 4; // 1 KiB volatile CHR RAM.
    prg_data(&image)[0x100] = 0xFF;
    BOARD_CHECK(board_image_load(&image) == 0);

    BOARD_CHECK(ppu_read(0x0056) == 0);
    ppu_write(0x0456, 0xA5);
    BOARD_CHECK(ppu_read(0x0456) == 0xA5);
    BOARD_CHECK(ppu_read(0x0856) == 0xA5 && ppu_read(0x0C56) == 0xA5);
    ppu_write(0x0056, 0x96);
    BOARD_CHECK(ppu_read(0x0056) == 0);
    BOARD_CHECK(ppu_read(0x1012) == 0x12); // No fourth RAM slot at the shrunk page size.

    write_mem(0x8100, 0x10); // Default source remaps a complete 2 KiB ROM page.
    BOARD_CHECK(ppu_read(0x0056) == 0 && ppu_read(0x0456) == 1);
    ppu_write(0x0456, 0x5A);
    BOARD_CHECK(ppu_read(0x0456) == 1); // $0400-$07FF became read-only ROM again.
    BOARD_CHECK(ppu_read(0x0856) == 0xA5 && ppu_read(0x0C56) == 0xA5);
    board_image_free(&image);
    return 0;
}

static int test_legacy_ram_only_aliases(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 77, 0x10000, 0, false));
    prg_data(&image)[0x100] = 0xFF;
    BOARD_CHECK(board_image_load(&image) == 0);

    ppu_write(0x0123, 0xA5);
    BOARD_CHECK(ppu_read(0x0923) == 0xA5); // Default bank 0 aliases explicit RAM slot 1.
    ppu_write(0x1123, 0x5C);
    BOARD_CHECK(ppu_read(0x1123) == 0x5C && ppu_read(0x0123) == 0xA5);
    write_mem(0x8100, 0x10); // With no CHR ROM, Default selects RAM page 1.
    BOARD_CHECK(ppu_read(0x0123) == 0x5C && ppu_read(0x1123) == 0x5C);
    BOARD_CHECK(ppu_read(0x0923) == 0xA5);

    uint8_t before_prg = read_mem(0x8000);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) == -1);
    BOARD_CHECK(read_mem(0x8000) == before_prg);
    BOARD_CHECK(ppu_read(0x0123) == 0x5C && ppu_read(0x0923) == 0xA5);
    board_image_free(&image);
    return 0;
}

static int test_explicit_zero_and_tiny_ram(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 77, 0x8000, 0x0800, true));
    ((iNESHeader *)image.data)->zero[0] = 0; // Explicitly no CHR RAM.
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(ppu_read(0x0000) == 0 && ppu_read(0x0400) == 1);
    BOARD_CHECK(ppu_read(0x0812) == 0x12);
    ppu_write(0x0812, 0xA6);
    BOARD_CHECK(ppu_read(0x0812) == 0x12);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 77, 0x8000, 0, true));
    ((iNESHeader *)image.data)->zero[0] = 1; // 128 bytes: below the 256-byte mapping granularity.
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(ppu_read(0x0123) == 0x23 && ppu_read(0x0812) == 0x12);
    ppu_write(0x0123, 0xA6);
    BOARD_CHECK(ppu_read(0x0123) == 0x23);
    board_image_free(&image);
    return 0;
}

static int test_chr_nvram_persistence(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 77, 0x8000, 0x0800, true));
    iNESHeader *header = (iNESHeader *)image.data;
    header->flags6 |= 2;
    header->zero[0] = 0x44; // Separate 1 KiB volatile and battery-backed CHR RAM.

    char rom_path[160], save_path[160];
    unsigned long stamp = (unsigned long)time(NULL);
    int used = snprintf(rom_path, sizeof(rom_path), "build/irem77-%lu-%lu.nes",
                        stamp, (unsigned long)clock());
    BOARD_CHECK(used > 0 && (size_t)used < sizeof(rom_path));
    memcpy(save_path, rom_path, (size_t)used + 1);
    strcpy(strrchr(save_path, '.'), ".chr.sav");
    BOARD_CHECK(write_file(rom_path, image.data, image.size));

    BOARD_CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_ZERO));
    BOARD_CHECK(load_rom(rom_path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu));
    ppu_write(0x0923, 0xA1); // Volatile half of the combined 2 KiB page.
    ppu_write(0x0C56, 0xD4); // Battery-backed half.
    BOARD_CHECK(ppu_read(0x1123) == 0xA1 && ppu_read(0x1456) == 0xD4);
    cart_battery_flush();
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(file_size(save_path) == 0x400 && file_byte(save_path, 0x56) == 0xD4);

    BOARD_CHECK(load_rom(rom_path) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu));
    BOARD_CHECK(ppu_read(0x0923) == 0 && ppu_read(0x1123) == 0);
    BOARD_CHECK(ppu_read(0x0C56) == 0xD4 && ppu_read(0x1456) == 0xD4);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(remove(save_path) == 0 && remove(rom_path) == 0);
    BOARD_CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_DEFAULT));
    board_image_free(&image);
    return 0;
}

int test_board_irem77_accuracy(void) {
    int failures = 0;
    failures += test_rom_and_default_ram();
    failures += test_shrunk_ram_overlap();
    failures += test_legacy_ram_only_aliases();
    failures += test_explicit_zero_and_tiny_ram();
    failures += test_chr_nvram_persistence();
    unload_rom();
    printf("Irem LROG017: 5 groups, %d failures\n", failures);
    return failures;
}
