/*
 * native_ram_accuracy.c - Independent RAM on cartridges with a fixed RAM window
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
#include "../rom/game_db.h"
#include <time.h>

static const unsigned default_ram_boards[] = {
    0, 2, 3, 7, 11, 13, 66, 79, 94, 113, 144, 146, 180
};

static bool cpu_store(uint16_t address, uint8_t value) {
    write_mem(0x0200, 0xA9); // LDA #value; STA address.
    write_mem(0x0201, value);
    write_mem(0x0202, 0x8D);
    write_mem(0x0203, (uint8_t)address);
    write_mem(0x0204, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4;
}

static bool cpu_load_is(uint16_t address, uint8_t expected) {
    write_mem(0x0200, 0xAD); // LDA address.
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

static bool make_ram_image(BoardImage *image, unsigned mapper, uint8_t ram_sizes) {
    if (!board_image_create(image, mapper, 0x8000, mapper == 13 ? 0 : 0x2000, true))
        return false;
    iNESHeader *header = (iNESHeader *)image->data;
    header->flags10 = ram_sizes;
    if (ram_sizes & 0xF0) header->flags6 |= 2;
    if (mapper == 13) header->zero[0] = 8; // CPROM's 16 KiB CHR RAM.
    return board_image_add_trainer(image, 0xC1);
}

static int test_default_ram_geometry(void) {
    // Unequal physical chips are valid even when their combined size is not
    // a power of two. Only the selected chip supplies the $6000-$7FFF window.
    const uint8_t layouts[] = {0x08, 0x87, 0x65};
    for (size_t board = 0; board < sizeof(default_ram_boards) / sizeof(default_ram_boards[0]); ++board) {
        for (size_t layout = 0; layout < sizeof(layouts); ++layout) {
            BoardImage image;
            BOARD_CHECK(make_ram_image(&image, default_ram_boards[board], layouts[layout]));
            BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart());
            BOARD_CHECK(cpu_load_is(0x7000, layout == 0 ? 0xC1 : 0));
            BOARD_CHECK(cpu_store(0x6001, 0xA3));
            BOARD_CHECK(cpu_load_is(0x6001, 0xA3));
            BOARD_CHECK(cpu_load_is(0x7001, layout == 0 ? 0xC1 : layout == 2 ? 0xA3 : 0));
            BOARD_CHECK(cpu_store(0x7FFF, 0x5D));
            BOARD_CHECK(cpu_load_is(0x7FFF, 0x5D));
            if (layout == 2) BOARD_CHECK(cpu_load_is(0x6FFF, 0x5D));
            BOARD_CHECK(cpu_store(0x8000, 0x1F));
            BOARD_CHECK(cpu_store(0x4100, 0x08));
            BOARD_CHECK(cpu_load_is(0x6001, 0xA3) && cpu_load_is(0x7FFF, 0x5D));
            ppu_soft_reset(&ppu);
            apu_soft_reset(&apu);
            cpu_soft_reset(&cpu);
            BOARD_CHECK(cpu_load_is(0x6001, 0xA3));
            BOARD_CHECK(power_cart() && cpu_load_is(0x7FFF, 0x5D));
            BOARD_CHECK(cpu_load_is(0x5FFF, 0x5F));

            uint8_t *previous_prg = prg_rom;
            uint32_t previous_crc = rom_file_crc32();
            BOARD_CHECK(load_rom_memory(image.data, image.size - 1) == -1);
            BOARD_CHECK(prg_rom == previous_prg && rom_file_crc32() == previous_crc);
            BOARD_CHECK(cpu_load_is(0x6001, 0xA3));
            BOARD_CHECK(unload_rom());
            BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart());
            BOARD_CHECK(cpu_load_is(0x6001, 0));
            BOARD_CHECK(cpu_load_is(0x7000, layout == 0 ? 0xC1 : 0));
            BOARD_CHECK(unload_rom());
            board_image_free(&image);
        }
    }
    return 0;
}

static bool write_file(const char *path, const uint8_t *bytes, size_t length) {
    FILE *file = fopen(path, "wb");
    if (!file) return false;
    bool written = fwrite(bytes, 1, length, file) == length;
    return fclose(file) == 0 && written;
}

static bool read_file_exact(const char *path, uint8_t *bytes, size_t length) {
    FILE *file = fopen(path, "rb");
    if (!file) return false;
    bool read = fread(bytes, 1, length, file) == length;
    int trailing = fgetc(file);
    return fclose(file) == 0 && read && trailing == EOF;
}

static int test_default_ram_persistence(void) {
    uint8_t contents[0x4000];
    uint8_t persisted[0x4000];
    memset(contents, 0xA5, 0x2000);
    memset(contents + 0x2000, 0xB7, 0x2000);
    contents[0x1000] = 0x44;

    for (size_t board = 0; board < sizeof(default_ram_boards) / sizeof(default_ram_boards[0]); ++board) {
        BoardImage image;
        unsigned mapper = default_ram_boards[board];
        BOARD_CHECK(make_ram_image(&image, mapper, 0x87));
        char rom_path[160], save_path[160];
        int used = snprintf(rom_path, sizeof(rom_path), "build/native-ram-%u-%lu-%lu.nes",
                            mapper, (unsigned long)time(NULL), (unsigned long)clock());
        BOARD_CHECK(used > 0 && (size_t)used < sizeof(rom_path));
        memcpy(save_path, rom_path, (size_t)used + 1);
        strcpy(strrchr(save_path, '.'), ".sav");
        BOARD_CHECK(write_file(rom_path, image.data, image.size));
        BOARD_CHECK(write_file(save_path, contents, sizeof(contents)));
        BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
        BOARD_CHECK(cpu_load_is(0x6000, 0xA5) && cpu_load_is(0x7000, 0x44));
        BOARD_CHECK(cpu_store(0x6001, 0xD2) && cpu_store(0x7FFF, 0x88));
        cart_battery_flush();
        BOARD_CHECK(read_file_exact(save_path, persisted, sizeof(persisted)));
        BOARD_CHECK(persisted[1] == 0xD2 && persisted[0x1FFF] == 0x88);
        BOARD_CHECK(persisted[0x1000] == 0x44);
        BOARD_CHECK(memcmp(persisted + 0x2000, contents + 0x2000, 0x2000) == 0);
        BOARD_CHECK(unload_rom());
        BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
        BOARD_CHECK(cpu_load_is(0x6001, 0xD2) && cpu_load_is(0x7FFF, 0x88));
        BOARD_CHECK(unload_rom());

        // A short save overlays only its bytes. The trainer belongs to the
        // separate work chip and must not leak into the selected save chip.
        memset(persisted, 0x33, 32);
        BOARD_CHECK(write_file(save_path, persisted, 32));
        BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
        BOARD_CHECK(cpu_load_is(0x6000, 0x33) && cpu_load_is(0x6020, 0));
        BOARD_CHECK(cpu_load_is(0x7000, 0));
        BOARD_CHECK(cpu_store(0x6020, 0xE4));
        cart_battery_flush();
        BOARD_CHECK(read_file_exact(save_path, persisted, sizeof(persisted)));
        BOARD_CHECK(persisted[0x20] == 0xE4 && persisted[0x1000] == 0);
        for (size_t i = 0x2000; i < sizeof(persisted); ++i) BOARD_CHECK(persisted[i] == 0);
        BOARD_CHECK(unload_rom());
        BOARD_CHECK(remove(save_path) == 0);

        BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
        BOARD_CHECK(cpu_load_is(0x6001, 0) && cpu_load_is(0x7000, 0));
        BOARD_CHECK(unload_rom());
        BOARD_CHECK(remove(rom_path) == 0);
        board_image_free(&image);
    }
    return 0;
}

static int test_default_ram_trainer_fallback(void) {
    BoardImage image;
    BOARD_CHECK(make_ram_image(&image, 0, 0x80)); // No work chip, 16 KiB save chip.
    char rom_path[160], save_path[160];
    int used = snprintf(rom_path, sizeof(rom_path), "build/native-ram-trainer-%lu-%lu.nes",
                        (unsigned long)time(NULL), (unsigned long)clock());
    BOARD_CHECK(used > 0 && (size_t)used < sizeof(rom_path));
    memcpy(save_path, rom_path, (size_t)used + 1);
    strcpy(strrchr(save_path, '.'), ".sav");
    uint8_t short_save[32];
    memset(short_save, 0xA6, sizeof(short_save));
    BOARD_CHECK(write_file(rom_path, image.data, image.size));
    BOARD_CHECK(write_file(save_path, short_save, sizeof(short_save)));
    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
    BOARD_CHECK(cpu_load_is(0x6000, 0xA6) && cpu_load_is(0x6020, 0));
    BOARD_CHECK(cpu_load_is(0x7000, 0xC1) && cpu_load_is(0x71FF, 0xC1));
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(remove(save_path) == 0 && remove(rom_path) == 0);
    board_image_free(&image);
    return 0;
}

static int test_default_ram_database_pages(void) {
    static const struct { unsigned work, save; } layouts[] = {
        {0x0C00, 0}, {0x2000, 0x1800}, {0x80, 0}, {0, 0x80}
    };
    bool previous_overrides = rom_database_overrides_enabled();
    rom_database_set_overrides(true);
    for (size_t layout = 0; layout < sizeof(layouts) / sizeof(layouts[0]); ++layout) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0x2000, false));
        uint32_t crc = game_db_crc32(image.data + 16, image.size - 16);
        char row[256];
        int length = snprintf(row, sizeof(row),
            "%08X,NesNtsc,TEST,,,0,b32768,b8192,0,b%u,b%u,%u,h,1,N,0,0,0\n",
            (unsigned)crc, layouts[layout].work, layouts[layout].save,
            layouts[layout].save ? 1u : 0u);
        BOARD_CHECK(length > 0 && (size_t)length < sizeof(row));
        BOARD_CHECK(rom_database_load_memory(row, (size_t)length));
        BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart());
        BOARD_CHECK(rom_metadata_source() == ROM_METADATA_DATABASE);
        BOARD_CHECK(cpu_store(0x6001, 0x83));
        if (layout < 2) {
            BOARD_CHECK(cpu_load_is(0x6001, 0x83));
            BOARD_CHECK(cpu_load_is(0x6C01, layout == 0 ? 0x83 : 0));
            BOARD_CHECK(cpu_store(0x77FF, 0xC7) && cpu_load_is(0x77FF, 0xC7));
            if (layout == 0) BOARD_CHECK(cpu_load_is(0x6BFF, 0xC7));
            BOARD_CHECK(cpu_load_is(0x7801, 0x78));
            BOARD_CHECK(cpu_store(0x7801, 0xD5));
            BOARD_CHECK(cpu_load_is(0x6001, 0x83) && cpu_load_is(0x7801, 0x78));
        } else {
            // The mapping granularity is 256 bytes. A declared 128-byte chip
            // cannot supply a CPU page and does not become mirrored RAM.
            BOARD_CHECK(cpu_load_is(0x6001, 0x60) && cpu_load_is(0x6081, 0x60));
        }
        BOARD_CHECK(unload_rom());
        board_image_free(&image);
    }
    rom_database_clear();
    rom_database_set_overrides(previous_overrides);
    return 0;
}

int test_native_ram_accuracy(void) {
    int failures = test_default_ram_geometry();
    failures += test_default_ram_persistence();
    failures += test_default_ram_trainer_fallback();
    failures += test_default_ram_database_pages();
    unload_rom();
    printf("Native RAM sources: 4 groups, %d failures\n", failures);
    return failures;
}
