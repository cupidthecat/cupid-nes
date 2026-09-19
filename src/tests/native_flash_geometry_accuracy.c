/*
 * native_flash_geometry_accuracy.c - Physical flash addresses and partial pages
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

static bool store_byte(uint16_t address, uint8_t value) {
    write_mem(0x0200, 0xA9);
    write_mem(0x0201, value);
    write_mem(0x0202, 0x8D);
    write_mem(0x0203, (uint8_t)address);
    write_mem(0x0204, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4;
}

static bool load_byte_is(uint16_t address, uint8_t expected) {
    write_mem(0x0200, 0xAD);
    write_mem(0x0201, (uint8_t)address);
    write_mem(0x0202, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 4 && cpu.a == expected;
}

static bool power_flash_cart(void) {
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    return cpu_power_on(&cpu);
}

static bool make_flash_image(BoardImage *image, unsigned mapper, size_t size) {
    if (!board_image_create(image, mapper, size, 0, true)) return false;
    iNESHeader *header = (iNESHeader *)image->data;
    if (mapper == 30) {
        header->prg_ram_size = 0x10;
        header->flags6 |= 2;
    } else {
        header->zero[0] = 8;
    }
    memset(image->data + sizeof(*header), 0xFF, size);
    image->data[sizeof(*header) + 0x123] = 0xF3;
    if (size > 0x4123) image->data[sizeof(*header) + 0x4123] = 0xC5;
    return true;
}

static bool select_bank(unsigned mapper, uint8_t bank) {
    return store_byte(mapper == 30 ? 0xC000 : 0x5000, bank);
}

static bool flash_command(unsigned mapper, uint8_t command) {
    if (mapper == 30) {
        return select_bank(mapper, 1) && store_byte(0x9555, 0xAA)
            && select_bank(mapper, 0) && store_byte(0xAAAA, 0x55)
            && select_bank(mapper, 1) && store_byte(0x9555, command);
    }
    return store_byte(0xD555, 0xAA) && store_byte(0xAAAA, 0x55)
        && store_byte(0xD555, command);
}

static bool program_byte(unsigned mapper, uint8_t bank, uint16_t offset, uint8_t value) {
    return flash_command(mapper, 0xA0) && select_bank(mapper, bank)
        && store_byte((uint16_t)(0x8000u + offset), value);
}

static bool erase_sector(unsigned mapper, uint8_t bank, uint16_t offset) {
    if (!flash_command(mapper, 0x80)) return false;
    if (mapper == 30) {
        if (!select_bank(mapper, 1) || !store_byte(0x9555, 0xAA)
            || !select_bank(mapper, 0) || !store_byte(0xAAAA, 0x55)) return false;
    } else if (!store_byte(0xD555, 0xAA) || !store_byte(0xAAAA, 0x55)) {
        return false;
    }
    return select_bank(mapper, bank) && store_byte((uint16_t)(0x8000u + offset), 0x30);
}

static int test_flash_partial_native_pages(void) {
    const unsigned mappers[] = {30, 111};
    for (size_t i = 0; i < sizeof(mappers) / sizeof(mappers[0]); ++i) {
        unsigned mapper = mappers[i];
        size_t bytes = mapper == 30 ? 0xA000 : 0xC000;
        uint8_t tail_bank = mapper == 30 ? 2 : 1;
        BoardImage image;
        BOARD_CHECK(make_flash_image(&image, mapper, bytes));
        BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_flash_cart());
        BOARD_CHECK(select_bank(mapper, tail_bank));
        BOARD_CHECK(load_byte_is(0x8123, 0xF3) && load_byte_is(0xC123, 0xC5));

        // The tail cannot form a selectable native ROM page, but its physical
        // flash bytes exist and remain writable without changing the wrapped read.
        BOARD_CHECK(program_byte(mapper, tail_bank, 0x123, 0xA6));
        BOARD_CHECK(prg_rom[0x8123] == 0xA6 && prg_rom[0x123] == 0xF3);
        BOARD_CHECK(load_byte_is(0x8123, 0xF3));
        BOARD_CHECK(program_byte(mapper, 0, 0x123, 0xA2));
        BOARD_CHECK(select_bank(mapper, tail_bank) && load_byte_is(0x8123, 0xA2));
        BOARD_CHECK(prg_rom[0x8123] == 0xA6);
        BOARD_CHECK(erase_sector(mapper, tail_bank, 0x123));
        BOARD_CHECK(prg_rom[0x8123] == 0xFF && prg_rom[0x123] == 0xA2);
        BOARD_CHECK(program_byte(mapper, tail_bank, 0x123, 0xA6));
        cart_ppu_write(0x0123, 0x5C);

        Mapper *previous_cart = cart;
        uint8_t *previous_prg = prg_rom;
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) == -1);
        BOARD_CHECK(cart == previous_cart && prg_rom == previous_prg);
        BOARD_CHECK(prg_rom[0x123] == 0xA2 && prg_rom[0x8123] == 0xA6);
        BOARD_CHECK(load_byte_is(0x8123, 0xA2) && cart_ppu_read(0x0123) == 0x5C);
        cpu_soft_reset(&cpu);
        BOARD_CHECK(prg_rom[0x123] == 0xA2 && prg_rom[0x8123] == 0xA6);
        BOARD_CHECK(unload_rom());
        board_image_free(&image);
    }
    return 0;
}

static int test_flash_reduced_images(void) {
    const unsigned mappers[] = {30, 111};
    for (size_t i = 0; i < sizeof(mappers) / sizeof(mappers[0]); ++i) {
        unsigned mapper = mappers[i];
        size_t bytes = mapper == 30 ? 0x2000 : 0x4000;
        BoardImage image;
        BOARD_CHECK(make_flash_image(&image, mapper, bytes));
        BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_flash_cart());
        BOARD_CHECK(load_byte_is(0x8123, 0xF3) && load_byte_is(0xC123, 0xF3));
        BOARD_CHECK(program_byte(mapper, 0, 0x123, 0xA2));
        BOARD_CHECK(load_byte_is(0x8123, 0xA2) && load_byte_is(0xC123, 0xA2));
        BOARD_CHECK(program_byte(mapper, 1, 0x123, 0));
        BOARD_CHECK(prg_rom[0x123] == 0xA2 && load_byte_is(0x8123, 0xA2));
        BOARD_CHECK(power_flash_cart() && prg_rom[0x123] == 0xA2);
        uint8_t *previous = prg_rom;
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) == -1);
        BOARD_CHECK(prg_rom == previous && load_byte_is(0xC123, 0xA2));
        BOARD_CHECK(unload_rom());
        board_image_free(&image);

        // A partially populated 4 KiB sector may be programmed byte by byte,
        // but an erase command cannot erase beyond the physical allocation.
        BOARD_CHECK(make_flash_image(&image, mapper, 0x2800));
        BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_flash_cart());
        BOARD_CHECK(program_byte(mapper, 0, 0x2123, 0xA6));
        BOARD_CHECK(prg_rom[0x2123] == 0xA6);
        BOARD_CHECK(erase_sector(mapper, 0, 0x2123));
        BOARD_CHECK(prg_rom[0x2123] == 0xA6);
        BOARD_CHECK(program_byte(mapper, 0, 0x123, 0xA2));
        BOARD_CHECK(erase_sector(mapper, 0, 0x123) && prg_rom[0x123] == 0xFF);
        BOARD_CHECK(load_byte_is(0xF923, 0xF9));
        BOARD_CHECK(unload_rom());
        board_image_free(&image);
    }
    return 0;
}

static bool write_file(const char *path, const uint8_t *data, size_t size) {
    FILE *file = fopen(path, "wb");
    if (!file) return false;
    bool written = fwrite(data, 1, size, file) == size;
    return fclose(file) == 0 && written;
}

static bool read_file_exact(const char *path, uint8_t *data, size_t size) {
    FILE *file = fopen(path, "rb");
    if (!file) return false;
    bool read = fread(data, 1, size, file) == size;
    int trailing = fgetc(file);
    return fclose(file) == 0 && read && trailing == EOF;
}

static int test_flash_partial_page_persistence(void) {
    const unsigned mappers[] = {30, 111};
    for (size_t i = 0; i < sizeof(mappers) / sizeof(mappers[0]); ++i) {
        unsigned mapper = mappers[i];
        size_t bytes = mapper == 30 ? 0xA000 : 0xC000;
        uint8_t tail_bank = mapper == 30 ? 2 : 1;
        BoardImage image;
        BOARD_CHECK(make_flash_image(&image, mapper, bytes));
        uint8_t *saved = (uint8_t *)malloc(bytes);
        BOARD_CHECK(saved != NULL);
        char rom_path[160], save_path[160];
        int used = snprintf(rom_path, sizeof(rom_path), "build/native-flash-%u-%lu-%lu.nes",
                            mapper, (unsigned long)time(NULL), (unsigned long)clock());
        BOARD_CHECK(used > 0 && (size_t)used + 6 < sizeof(save_path));
        memcpy(save_path, rom_path, (size_t)used + 1);
        strcpy(strrchr(save_path, '.'), ".flash.sav");
        BOARD_CHECK(write_file(rom_path, image.data, image.size));
        BOARD_CHECK(load_rom(rom_path) == 0 && power_flash_cart());
        BOARD_CHECK(program_byte(mapper, tail_bank, 0x123, 0xA6));
        BOARD_CHECK(program_byte(mapper, 0, 0x123, 0xA2));
        uint8_t *previous = prg_rom;
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) == -1);
        BOARD_CHECK(prg_rom == previous && prg_rom[0x8123] == 0xA6);
        cart_battery_flush();
        BOARD_CHECK(read_file_exact(save_path, saved, bytes));
        BOARD_CHECK(saved[0x123] == 0xA2 && saved[0x4123] == 0xC5 && saved[0x8123] == 0xA6);
        BOARD_CHECK(saved[bytes - 1] == 0xFF);
        BOARD_CHECK(unload_rom());
        BOARD_CHECK(load_rom(rom_path) == 0 && power_flash_cart());
        BOARD_CHECK(prg_rom[0x123] == 0xA2 && prg_rom[0x8123] == 0xA6);
        BOARD_CHECK(select_bank(mapper, tail_bank) && load_byte_is(0x8123, 0xA2));
        cpu_soft_reset(&cpu);
        BOARD_CHECK(prg_rom[0x8123] == 0xA6);
        BOARD_CHECK(unload_rom() && remove(save_path) == 0);
        BOARD_CHECK(load_rom(rom_path) == 0 && power_flash_cart());
        BOARD_CHECK(prg_rom[0x123] == 0xF3 && prg_rom[0x8123] == 0xFF);
        BOARD_CHECK(unload_rom() && remove(rom_path) == 0);
        free(saved);
        board_image_free(&image);
    }
    return 0;
}

int test_native_flash_geometry_accuracy(void) {
    int failures = test_flash_partial_native_pages();
    failures += test_flash_reduced_images();
    failures += test_flash_partial_page_persistence();
    unload_rom();
    printf("Native flash geometry: 3 groups, %d failures\n", failures);
    return failures;
}
