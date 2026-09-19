/*
 * board_magic_floor_accuracy.c - Shared pattern and nametable memory tests
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

static int test_magic_floor_wiring(void) {
    static const uint8_t flags[] = {0, 1, 8, 9};
    static const uint8_t pattern_pages[4][8] = {
        {0, 0, 1, 1, 0, 0, 1, 1}, {0, 1, 0, 1, 0, 1, 0, 1},
        {0, 0, 0, 0, 1, 1, 1, 1}, {0, 0, 0, 0, 0, 0, 0, 0}
    };
    static const uint8_t nametable_pages[4][4] = {
        {0, 0, 1, 1}, {0, 1, 0, 1}, {0, 0, 0, 0}, {1, 1, 1, 1}
    };
    static const Mirroring mirrors[] = {
        MIRROR_HORIZONTAL, MIRROR_VERTICAL, MIRROR_SINGLE0, MIRROR_SINGLE1
    };
    for (unsigned variant = 0; variant < 4; ++variant) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, 218, 0x8000, 0, true));
        image.data[6] |= flags[variant];
        image.data[8] |= 0xF0;
        image.data[11] = 0; // CIRAM supplies patterns without a CHR chip.
        BOARD_CHECK(board_image_load(&image) == 0);
        BOARD_CHECK(cart_get_mirroring() == mirrors[variant]);
        uint8_t contents[2] = {0, 0};
        for (unsigned page = 0; page < 8; ++page) {
            uint8_t value = (uint8_t)(0x40 + page);
            ppu_write((uint16_t)(page * 0x400 + 0x2A), value);
            contents[pattern_pages[variant][page]] = value;
        }
        for (unsigned page = 0; page < 8; ++page)
            BOARD_CHECK(ppu_read((uint16_t)(page * 0x400 + 0x2A))
                        == contents[pattern_pages[variant][page]]);
        for (unsigned page = 0; page < 4; ++page)
            BOARD_CHECK(ppu_read((uint16_t)(0x2000 + page * 0x400 + 0x2A))
                        == contents[nametable_pages[variant][page]]);
        for (unsigned page = 0; page < 4; ++page) {
            uint8_t value = (uint8_t)(0x80 + page);
            ppu_write((uint16_t)(0x2000 + page * 0x400 + 0x2A), value);
            contents[nametable_pages[variant][page]] = value;
        }
        for (unsigned page = 0; page < 8; ++page)
            BOARD_CHECK(ppu_read((uint16_t)(page * 0x400 + 0x2A))
                        == contents[pattern_pages[variant][page]]);
        for (unsigned page = 0; page < 4; ++page)
            BOARD_CHECK(ppu_read((uint16_t)(0x3000 + page * 0x400 + 0x2A))
                        == contents[nametable_pages[variant][page]]);

        // CPU accesses to PPUDATA must reach the same shared CIRAM pages.
        const uint8_t program[] = {
            0xAD, 0x02, 0x20,       // LDA PPUSTATUS: reset the address latch.
            0xA9, 0x00, 0x8D, 0x06, 0x20,
            0xA9, 0x2A, 0x8D, 0x06, 0x20,
            0xA9, 0xE5, 0x8D, 0x07, 0x20,
            0xA9, 0x20, 0x8D, 0x06, 0x20,
            0xA9, 0x2A, 0x8D, 0x06, 0x20,
            0xAD, 0x07, 0x20,       // Discard the buffered byte.
            0xAD, 0x07, 0x20, 0x85, 0x10
        };
        for (unsigned byte = 0; byte < sizeof(program); ++byte)
            write_mem((uint16_t)(0x0200 + byte), program[byte]);
        cpu.pc = 0x0200;
        for (unsigned instruction = 0; instruction < 14; ++instruction)
            BOARD_CHECK(cpu_step(&cpu) > 0);
        contents[0] = 0xE5;
        BOARD_CHECK(read_mem(0x0010) == contents[nametable_pages[variant][0]]);
        write_mem(0x8000, 0xFF);
        write_mem(0xFFFF, 0x5A);
        BOARD_CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 7);
        cpu_soft_reset(&cpu);
        BOARD_CHECK(cart_get_mirroring() == mirrors[variant]);
        BOARD_CHECK(ppu_read(0x002A) == 0xE5);
        BOARD_CHECK(board_image_load(&image) == 0 && ppu_read(0x002A) == 0);
        board_image_free(&image);
    }
    return 0;
}

static int test_magic_floor_geometry(void) {
    static const size_t sizes[] = {0x2000, 0x4000, 0x5000, 0x8000, 0xC000, 0x18000};
    for (unsigned fixture = 0; fixture < sizeof(sizes) / sizeof(sizes[0]); ++fixture) {
        BoardImage image;
        size_t bytes = sizes[fixture];
        BOARD_CHECK(board_image_create(&image, 218, bytes, 0x2000, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        for (unsigned offset = 0; offset < 0x8000; offset += 0x1000) {
            write_mem(0x5000, 0xAD);
            if (bytes < 0x8000 && offset >= (0x8000 / bytes) * bytes) {
                BOARD_CHECK(read_mem((uint16_t)(0x8000 + offset)) == 0xAD);
            } else {
                unsigned page = (unsigned)((bytes < 0x8000 ? offset % bytes : offset) / 0x1000);
                BOARD_CHECK(read_mem((uint16_t)(0x8000 + offset)) == page);
            }
        }
        // A declared CHR ROM is not connected to the pattern address bus.
        ppu_write(0x1234, 0xB6);
        BOARD_CHECK(ppu_read(0x0234) == 0xB6 && ppu_read(0x2234) == 0xB6);
        board_image_free(&image);
    }
    return 0;
}

static int test_magic_floor_replacement(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 218, 0x8000, 0, false));
    BOARD_CHECK(board_image_load(&image) == 0);
    write_mem(0x6123, 0x8A);
    ppu_write(0x2345, 0x9B);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(read_mem(0x6123) == 0x8A && ppu_read(0x0345) == 0x9B);
    board_image_set_unsupported_console(&image);
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x8000) == 0 && ppu_read(0x0345) == 0x9B);
    image.data[0] = 0;
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(read_mem(0x6123) == 0x8A && ppu_read(0x2345) == 0x9B);
    board_image_free(&image);
    return 0;
}

static int test_magic_floor_save(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 218, 0x8000, 0, false));
    image.data[6] |= 2;
    BOARD_CHECK(board_image_add_trainer(&image, 0x19));
    char path[128], save[128];
    snprintf(path, sizeof(path), "build/board-magic-floor-%lu-%lu.nes",
             (unsigned long)time(NULL), (unsigned long)clock());
    memcpy(save, path, strlen(path) + 1);
    strcpy(strrchr(save, '.'), ".sav");
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    size_t count = fwrite(image.data, 1, image.size, file);
    int closed = fclose(file);
    BOARD_CHECK(count == image.size && closed == 0);
    BOARD_CHECK(load_rom(path) == 0 && read_mem(0x7000) == 0x19);
    write_mem(0x7000, 0x71);
    ppu_write(0x0123, 0xE3);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(path) == 0 && read_mem(0x7000) == 0x71);
    BOARD_CHECK(ppu_read(0x0123) == 0);
    BOARD_CHECK(unload_rom());
    file = fopen(save, "rb");
    BOARD_CHECK(file != NULL && fseek(file, 0, SEEK_END) == 0);
    long length = ftell(file);
    closed = fclose(file);
    BOARD_CHECK(length == 0x2000 && closed == 0);
    BOARD_CHECK(remove(save) == 0 && remove(path) == 0);
    board_image_free(&image);
    return 0;
}

int test_board_magic_floor_accuracy(void) {
    int failures = 0;
    failures += test_magic_floor_wiring();
    failures += test_magic_floor_geometry();
    failures += test_magic_floor_replacement();
    failures += test_magic_floor_save();
    unload_rom();
    printf("Magic Floor accuracy: 4 groups, %d failures\n", failures);
    return failures;
}
