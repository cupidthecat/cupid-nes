/*
 * mapper30_111_prg_ram_accuracy.c - Explicit PRG RAM on flash cartridge boards
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

static bool make_image(BoardImage *image, unsigned mapper, uint8_t prg_ram_sizes,
                       bool trainer) {
    if (!board_image_create(image, mapper, 0x10000, 0, true)) return false;
    iNESHeader *header = (iNESHeader *)image->data;
    header->flags10 = prg_ram_sizes;
    if (prg_ram_sizes & 0xF0u) header->flags6 |= 2;
    return !trainer || board_image_add_trainer(image, 0xC1);
}

static bool write_file(const char *path, const uint8_t *bytes, size_t size) {
    FILE *file = fopen(path, "wb");
    if (!file) return false;
    bool ok = fwrite(bytes, 1, size, file) == size;
    return fclose(file) == 0 && ok;
}

static bool read_file_exact(const char *path, uint8_t *bytes, size_t size) {
    FILE *file = fopen(path, "rb");
    if (!file) return false;
    bool ok = fread(bytes, 1, size, file) == size && fgetc(file) == EOF;
    return fclose(file) == 0 && ok;
}

static bool select_flash_bank(unsigned mapper, uint8_t bank) {
    return cpu_store(mapper == 30 ? 0xC000 : 0x5000, bank);
}

static bool flash_command(unsigned mapper, uint8_t command) {
    if (mapper == 30) {
        return select_flash_bank(mapper, 1) && cpu_store(0x9555, 0xAA)
            && select_flash_bank(mapper, 0) && cpu_store(0xAAAA, 0x55)
            && select_flash_bank(mapper, 1) && cpu_store(0x9555, command);
    }
    return cpu_store(0xD555, 0xAA) && cpu_store(0xAAAA, 0x55)
        && cpu_store(0xD555, command);
}

static bool program_flash(unsigned mapper, uint8_t bank, uint16_t offset, uint8_t value) {
    if (mapper == 111 && !select_flash_bank(mapper, 0)) return false;
    return flash_command(mapper, 0xA0) && select_flash_bank(mapper, bank)
        && cpu_store((uint16_t)(0x8000u + offset), value);
}

static int test_explicit_ram_geometry(void) {
    const unsigned mappers[] = {30, 111};
    const uint8_t layouts[] = {0x00, 0x01, 0x02, 0x08};

    for (size_t m = 0; m < sizeof(mappers) / sizeof(mappers[0]); ++m) {
        unsigned mapper = mappers[m];
        for (size_t n = 0; n < sizeof(layouts) / sizeof(layouts[0]); ++n) {
            BoardImage image;
            BOARD_CHECK(make_image(&image, mapper, layouts[n], false));
            BOARD_CHECK(board_image_load(&image) == 0 && power_cart());

            if (layouts[n] <= 1) {
                BOARD_CHECK(cart_cpu_read_bus(0x6000, 0xD1) == 0xD1);
                BOARD_CHECK(cpu_store(0x6000, 0xA6));
                BOARD_CHECK(cart_cpu_read_bus(0x6000, 0xE2) == 0xE2);
                if (mapper == 111) {
                    BOARD_CHECK(cart_cpu_read_bus(0x5000, 1) == 0);
                    BOARD_CHECK(cart_cpu_read(0x8000) == 8);
                    BOARD_CHECK(cart_cpu_read_bus(0x7000, 0) == 0);
                    BOARD_CHECK(cart_cpu_read(0x8000) == 0);
                }
            } else if (layouts[n] == 2) {
                BOARD_CHECK(cpu_store(0x6000, 0xA6));
                BOARD_CHECK(cpu_store(0x60FF, 0x5C));
                BOARD_CHECK(cpu_load_is(0x6100, 0xA6));
                if (mapper == 30) {
                    BOARD_CHECK(cpu_load_is(0x7FFF, 0x5C));
                } else {
                    BOARD_CHECK(cart_cpu_read_bus(0x7000, 1) == 0xA6);
                    BOARD_CHECK(cart_cpu_read(0x8000) == 8);
                    BOARD_CHECK(cpu_store(0x7000, 0));
                    BOARD_CHECK(cpu_load_is(0x6000, 0xA6));
                    BOARD_CHECK(cart_cpu_read_bus(0x7000, 0) == 0xA6);
                }
            } else {
                BOARD_CHECK(cpu_store(0x6000, 0xA6));
                BOARD_CHECK(cpu_store(0x6FFF, 0x5C));
                BOARD_CHECK(cpu_load_is(0x6000, 0xA6));
                BOARD_CHECK(cpu_load_is(0x6FFF, 0x5C));
                if (mapper == 30) {
                    BOARD_CHECK(cpu_store(0x7FFF, 0xE4));
                    BOARD_CHECK(cpu_load_is(0x7FFF, 0xE4));
                } else {
                    BOARD_CHECK(cart_cpu_read_bus(0x7000, 1) == 0);
                    BOARD_CHECK(cart_cpu_read(0x8000) == 8);
                    BOARD_CHECK(cpu_store(0x7000, 0));
                    BOARD_CHECK(cart_cpu_read_bus(0x7000, 0) == 0);
                }
            }

            BOARD_CHECK(unload_rom());
            board_image_free(&image);
        }
    }
    return 0;
}

static int test_unequal_chips_and_failed_replacement(void) {
    const unsigned mappers[] = {30, 111};
    for (size_t m = 0; m < sizeof(mappers) / sizeof(mappers[0]); ++m) {
        unsigned mapper = mappers[m];
        BoardImage image;
        /* 8 KiB work RAM plus 16 KiB save RAM. The battery-backed chip owns
           the CPU window while the trainer initializes the separate work chip. */
        BOARD_CHECK(make_image(&image, mapper, 0x87, true));
        BOARD_CHECK(board_image_load(&image) == 0 && power_cart());
        BOARD_CHECK(cpu_load_is(0x7000, 0));
        BOARD_CHECK(cpu_store(0x6001, 0xD2));
        BOARD_CHECK(cpu_load_is(0x6001, 0xD2));
        if (mapper == 30) {
            BOARD_CHECK(cpu_store(0x7FFF, 0x88));
            BOARD_CHECK(cpu_load_is(0x7FFF, 0x88));
        } else {
            BOARD_CHECK(cpu_store(0x7FFF, 3));
            BOARD_CHECK(cart_cpu_read_bus(0x7FFF, 0) == 0);
        }

        uint8_t *previous_prg = prg_rom;
        uint32_t previous_crc = rom_file_crc32();
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) == -1);
        BOARD_CHECK(prg_rom == previous_prg && rom_file_crc32() == previous_crc);
        BOARD_CHECK(cpu_load_is(0x6001, 0xD2));
        if (mapper == 30) BOARD_CHECK(cpu_load_is(0x7FFF, 0x88));

        BOARD_CHECK(unload_rom());
        board_image_free(&image);
    }
    return 0;
}

static int test_save_and_flash_persistence(void) {
    const unsigned mappers[] = {30, 111};
    for (size_t m = 0; m < sizeof(mappers) / sizeof(mappers[0]); ++m) {
        unsigned mapper = mappers[m];
        BoardImage image;
        BOARD_CHECK(make_image(&image, mapper, 0x80, true)); /* 16 KiB save RAM. */
        iNESHeader *header = (iNESHeader *)image.data;
        size_t trainer_bytes = (header->flags6 & 4u) ? 512u : 0u;
        memset(image.data + sizeof(*header) + trainer_bytes, 0xFF, 0x10000);

        char rom_path[160], save_path[160], flash_path[160];
        int used = snprintf(rom_path, sizeof(rom_path), "build/prg-ram-flash-%u-%lu-%lu.nes",
                            mapper, (unsigned long)time(NULL), (unsigned long)clock());
        BOARD_CHECK(used > 0 && (size_t)used + 10 < sizeof(save_path));
        memcpy(save_path, rom_path, (size_t)used + 1);
        memcpy(flash_path, rom_path, (size_t)used + 1);
        strcpy(strrchr(save_path, '.'), ".sav");
        strcpy(strrchr(flash_path, '.'), ".flash.sav");
        BOARD_CHECK(write_file(rom_path, image.data, image.size));

        uint8_t short_save[0x1024];
        memset(short_save, 0xA6, sizeof(short_save));
        BOARD_CHECK(write_file(save_path, short_save, sizeof(short_save)));
        BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
        BOARD_CHECK(cpu_load_is(0x7023, 0xA6));
        BOARD_CHECK(cpu_load_is(0x7123, 0xC1));
        BOARD_CHECK(cpu_store(0x6001, 0xD2));
        BOARD_CHECK(cpu_store(0x6FFF, 0x88));
        if (mapper == 30) {
            BOARD_CHECK(cpu_store(0x7001, 0xE3));
        } else {
            BOARD_CHECK(cpu_store(0x7001, 3));
            BOARD_CHECK(cart_cpu_read_bus(0x7001, 0) == 0xA6);
        }
        BOARD_CHECK(program_flash(mapper, 0, 0x0123, 0xA2));
        BOARD_CHECK(select_flash_bank(mapper, 0));
        BOARD_CHECK(cpu_load_is(0x8123, 0xA2));
        cart_battery_flush();

        uint8_t saved[0x4000];
        uint8_t flashed[0x10000];
        BOARD_CHECK(read_file_exact(save_path, saved, sizeof(saved)));
        BOARD_CHECK(read_file_exact(flash_path, flashed, sizeof(flashed)));
        BOARD_CHECK(saved[1] == 0xD2 && saved[0x0FFF] == 0x88);
        BOARD_CHECK(saved[0x1001] == (mapper == 30 ? 0xE3 : 0xA6));
        BOARD_CHECK(saved[0x1023] == 0xA6 && saved[0x1123] == 0xC1);
        BOARD_CHECK(flashed[0x0123] == 0xA2);

        BOARD_CHECK(unload_rom());
        BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
        BOARD_CHECK(cpu_load_is(0x6001, 0xD2) && cpu_load_is(0x6FFF, 0x88));
        BOARD_CHECK(select_flash_bank(mapper, 0) && cpu_load_is(0x8123, 0xA2));
        BOARD_CHECK(unload_rom());

        const uint8_t one_byte_save = 0x7D;
        BOARD_CHECK(write_file(save_path, &one_byte_save, 1));
        BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
        BOARD_CHECK(cpu_load_is(0x6000, 0x7D) && cpu_load_is(0x6001, 0));
        BOARD_CHECK(cpu_load_is(0x7000, 0xC1));
        BOARD_CHECK(select_flash_bank(mapper, 0) && cpu_load_is(0x8123, 0xA2));

        BOARD_CHECK(cpu_store(0x6002, 0xE4));
        uint8_t *previous_prg = prg_rom;
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) == -1);
        BOARD_CHECK(prg_rom == previous_prg);
        BOARD_CHECK(cpu_load_is(0x6002, 0xE4));
        BOARD_CHECK(select_flash_bank(mapper, 0) && cpu_load_is(0x8123, 0xA2));

        BOARD_CHECK(unload_rom());
        BOARD_CHECK(remove(save_path) == 0 && remove(flash_path) == 0 && remove(rom_path) == 0);
        board_image_free(&image);
    }
    return 0;
}

int test_mapper30_111_prg_ram_accuracy(void) {
    int failures = test_explicit_ram_geometry();
    failures += test_unequal_chips_and_failed_replacement();
    failures += test_save_and_flash_persistence();
    unload_rom();
    printf("Mapper 30/111 PRG RAM: 3 groups, %d failures\n", failures);
    return failures;
}
