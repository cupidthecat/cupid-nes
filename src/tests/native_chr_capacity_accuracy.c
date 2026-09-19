/*
 * native_chr_capacity_accuracy.c - Declared CHR chips and reachable bank windows
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

static const unsigned fixed_boards[] = {0, 2, 7, 15, 71, 73, 94, 97, 180, 232};

static bool cpu_store(uint16_t address, uint8_t value) {
    write_mem(0x0200, 0xA9);
    write_mem(0x0201, value);
    write_mem(0x0202, 0x8D);
    write_mem(0x0203, (uint8_t)address);
    write_mem(0x0204, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4;
}

static bool power_cart(void) {
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    return cpu_power_on(&cpu);
}

static bool wait_for_ppu_register_writes(void) {
    write_mem(0x0200, 0x4C);
    write_mem(0x0201, 0);
    write_mem(0x0202, 2);
    cpu.pc = 0x0200;
    // PPUADDR writes are ignored during the first frame after power-on.
    for (unsigned instruction = 0; instruction < 10000; ++instruction)
        if (cpu_step(&cpu) != 3) return false;
    return true;
}

static bool capacity_image(BoardImage *image, unsigned mapper,
                           unsigned ram_shift, bool battery) {
    if (!board_image_create(image, mapper, 0x20000, 0, true)) return false;
    iNESHeader *header = (iNESHeader *)image->data;
    header->zero[0] = (uint8_t)(battery ? ram_shift << 4 : ram_shift);
    if (battery) header->flags6 |= 2;
    // Unmasked bank writes remain possible on boards with ROM bus conflicts.
    memset(image->data + sizeof(*header), 0xFF, 0x20000);
    return true;
}

static bool select_fixed_board(unsigned mapper, uint8_t value) {
    // Mapper 15 modes 1 and 2 permit CHR writes; modes 0 and 3 protect RAM.
    return cpu_store(mapper == 15 ? 0xF001 : 0xF000, value);
}

static int test_fixed_windows_and_replacement(void) {
    for (size_t i = 0; i < sizeof(fixed_boards) / sizeof(fixed_boards[0]); ++i) {
        unsigned mapper = fixed_boards[i];
        BoardImage image;
        BOARD_CHECK(capacity_image(&image, mapper, 9, false)); // 32 KiB RAM.
        BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart());
        BOARD_CHECK(chr_size == 0x8000 && select_fixed_board(mapper, 0));
        ppu_write(0x0123, 0xA6);
        ppu_write(0x1FFF, 0x5C);
        BOARD_CHECK(ppu_read(0x0123) == 0xA6 && ppu_read(0x1FFF) == 0x5C);
        BOARD_CHECK(select_fixed_board(mapper, 0xFF));
        BOARD_CHECK(ppu_read(0x0123) == 0xA6 && ppu_read(0x1FFF) == 0x5C);
        BOARD_CHECK(chr_rom[0x2123] == 0 && chr_rom[0x7FFF] == 0);
        if (mapper == 15) {
            BOARD_CHECK(cpu_store(0x8003, 0));
            ppu_write(0x0123, 0x17);
            BOARD_CHECK(ppu_read(0x0123) == 0xA6);
        }
        uint8_t *previous_prg = prg_rom;
        uint8_t *previous_chr = chr_rom;
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) == -1);
        BOARD_CHECK(prg_rom == previous_prg && chr_rom == previous_chr);
        BOARD_CHECK(ppu_read(0x0123) == 0xA6 && ppu_read(0x1FFF) == 0x5C);
        cpu_soft_reset(&cpu);
        BOARD_CHECK(ppu_read(0x0123) == 0xA6 && ppu_read(0x1FFF) == 0x5C);
        BOARD_CHECK(unload_rom());
        BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart());
        BOARD_CHECK(ppu_read(0x0123) == 0 && ppu_read(0x1FFF) == 0);
        BOARD_CHECK(unload_rom());
        board_image_free(&image);
    }
    return 0;
}

static int test_masked_chr_banks(void) {
    const unsigned mappers[] = {13, 66};
    for (size_t i = 0; i < sizeof(mappers) / sizeof(mappers[0]); ++i) {
        unsigned mapper = mappers[i];
        size_t page_size = mapper == 13 ? 0x1000 : 0x2000;
        uint16_t window = mapper == 13 ? 0x1000 : 0;
        BoardImage image;
        BOARD_CHECK(capacity_image(&image, mapper, mapper == 13 ? 9 : 10, false));
        BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart());
        BOARD_CHECK(chr_size == 8 * page_size);
        for (uint8_t bank = 0; bank < 4; ++bank) {
            BOARD_CHECK(cpu_store(0x8000, bank));
            ppu_write((uint16_t)(window + 0x123), (uint8_t)(0xA0 + bank));
            ppu_write((uint16_t)(window + page_size - 1), (uint8_t)(0x50 + bank));
        }
        for (uint8_t bank = 0; bank < 8; ++bank) {
            BOARD_CHECK(cpu_store(0x8000, bank));
            BOARD_CHECK(ppu_read((uint16_t)(window + 0x123)) == 0xA0 + (bank & 3));
            BOARD_CHECK(ppu_read((uint16_t)(window + page_size - 1)) == 0x50 + (bank & 3));
            if (mapper == 13) BOARD_CHECK(ppu_read(0x0123) == 0xA0);
        }
        BOARD_CHECK(chr_rom[4 * page_size + 0x123] == 0 && chr_rom[chr_size - 1] == 0);
        uint8_t *previous_chr = chr_rom;
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) == -1);
        BOARD_CHECK(chr_rom == previous_chr);
        BOARD_CHECK(ppu_read((uint16_t)(window + 0x123)) == 0xA3);
        BOARD_CHECK(unload_rom());
        board_image_free(&image);
    }
    return 0;
}

static int test_latched_short_chr_rom(void) {
    static const struct {
        size_t bytes;
        uint16_t action_end;
        uint16_t oeka_slot;
        uint16_t oeka_end;
        uint8_t action_value;
        uint8_t oeka_upper;
        uint8_t oeka_latched;
    } cases[] = {
        {0x0100, 0x0100, 0x0100, 0x0200, 0x91, 0x91, 0x91},
        {0x0400, 0x0400, 0x0400, 0x0800, 0x91, 0x91, 0x91},
        {0x0800, 0x0800, 0x0800, 0x1000, 0x91, 0x91, 0x91},
        {0x0C00, 0x0C00, 0x0C00, 0x1800, 0x91, 0x91, 0x91},
        {0x1000, 0x1000, 0x1000, 0x2000, 0x91, 0x91, 0x91},
        {0x1800, 0x1800, 0x1000, 0x2000, 0x91, 0x91, 0x91},
        {0x2000, 0x2000, 0x1000, 0x2000, 0x91, 0x95, 0x95},
        {0x4000, 0x2000, 0x1000, 0x2000, 0x99, 0x9D, 0x95}
    };
    const unsigned mappers[] = {28, 96};
    for (size_t m = 0; m < sizeof(mappers) / sizeof(mappers[0]); ++m) {
        unsigned mapper = mappers[m];
        for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c) {
            for (unsigned sidecar = 0; sidecar < 2; ++sidecar) {
                BoardImage image;
                BOARD_CHECK(board_image_create(&image, mapper, 0x20000, cases[c].bytes, true));
                iNESHeader *header = (iNESHeader *)image.data;
                header->zero[0] = sidecar ? 9 : 0;
                memset(image.data + sizeof(*header), 0xFF, 0x20000);
                uint8_t *chr = image.data + sizeof(*header) + 0x20000;
                for (size_t b = 0; b < cases[c].bytes; ++b)
                    chr[b] = (uint8_t)(0x91 + b / 0x400);
                BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart());
                BOARD_CHECK(ppu_read(0x007B) == 0x7B);
                ppu_write(0x007B, 0x5C);
                BOARD_CHECK(ppu_read(0x007B) == 0x7B);
                if (mapper == 28) BOARD_CHECK(cpu_store(0x5000, 0));
                BOARD_CHECK(cpu_store(0x8000, mapper == 28 ? 3 : 4));
                uint8_t expected = mapper == 28 ? cases[c].action_value : 0x91;
                BOARD_CHECK(ppu_read(0x007B) == expected);
                ppu_write(0x007B, 0x5C);
                BOARD_CHECK(ppu_read(0x007B) == expected);
                uint16_t mapped_end = mapper == 28 ? cases[c].action_end : cases[c].oeka_end;
                if (mapped_end < 0x2000) {
                    BOARD_CHECK(ppu_read((uint16_t)(mapped_end + 0x33)) == 0x33);
                    ppu_write((uint16_t)(mapped_end + 0x33), 0x5C);
                    BOARD_CHECK(ppu_read(0x0033) == expected);
                }
                if (mapper == 96) {
                    BOARD_CHECK(ppu_read((uint16_t)(cases[c].oeka_slot + 0x7B)) == cases[c].oeka_upper);
                    // PPUADDR crossing into a nametable updates the lower bank.
                    BOARD_CHECK(wait_for_ppu_register_writes());
                    BOARD_CHECK(cpu_store(0x2006, 0) && cpu_store(0x2006, 0));
                    BOARD_CHECK(cpu_store(0x2006, 0x21) && cpu_store(0x2006, 0));
                    write_mem(0x0200, 0xEA);
                    cpu.pc = 0x0200;
                    BOARD_CHECK(cpu_step(&cpu) == 2 && ppu.bus_address == 0x2100);
                    expected = cases[c].oeka_latched;
                    BOARD_CHECK(ppu_read(0x007B) == expected);
                    BOARD_CHECK(ppu_read((uint16_t)(cases[c].oeka_slot + 0x7B)) == cases[c].oeka_upper);
                }
                Mapper *previous = cart;
                uint8_t *previous_prg = prg_rom;
                BOARD_CHECK(load_rom_memory(image.data, image.size - 1) == -1);
                BOARD_CHECK(cart == previous && prg_rom == previous_prg);
                BOARD_CHECK(ppu_read(0x007B) == expected);
                cpu_soft_reset(&cpu);
                BOARD_CHECK(ppu_read(0x007B) == expected);
                BOARD_CHECK(unload_rom());
                board_image_free(&image);
            }
        }
    }
    return 0;
}

static bool write_file(const char *path, const uint8_t *data, size_t size) {
    FILE *file = fopen(path, "wb");
    if (!file) return false;
    bool ok = fwrite(data, 1, size, file) == size;
    return fclose(file) == 0 && ok;
}

static bool read_file_exact(const char *path, uint8_t *data, size_t size) {
    FILE *file = fopen(path, "rb");
    if (!file) return false;
    bool ok = fread(data, 1, size, file) == size;
    int trailing = fgetc(file);
    return fclose(file) == 0 && ok && trailing == EOF;
}

static int test_unreachable_nvram_persistence(void) {
    const unsigned mappers[] = {0, 2, 7, 13, 15, 66, 71, 73, 94, 97, 180, 232};
    for (size_t i = 0; i < sizeof(mappers) / sizeof(mappers[0]); ++i) {
        unsigned mapper = mappers[i];
        unsigned ram_shift = mapper == 66 ? 10 : 9;
        size_t bytes = (size_t)64 << ram_shift;
        BoardImage image;
        BOARD_CHECK(capacity_image(&image, mapper, ram_shift, true));
        uint8_t *saved = (uint8_t *)calloc(bytes, 1);
        uint8_t *expected = (uint8_t *)calloc(bytes, 1);
        BOARD_CHECK(saved != NULL && expected != NULL);
        expected[bytes - 1] = 0xD7; // A real save byte outside every reachable bank.
        expected[bytes - 0x101] = 0x8C;
        char rom_path[160], save_path[168];
        int used = snprintf(rom_path, sizeof(rom_path), "build/chr-capacity-%u-%lu-%lu.nes",
                            mapper, (unsigned long)time(NULL), (unsigned long)clock());
        BOARD_CHECK(used > 0 && (size_t)used < sizeof(rom_path));
        memcpy(save_path, rom_path, (size_t)used + 1);
        strcpy(strrchr(save_path, '.'), ".chr.sav");
        BOARD_CHECK(write_file(rom_path, image.data, image.size));
        BOARD_CHECK(write_file(save_path, expected, bytes));
        BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
        BOARD_CHECK(chr_size == bytes && chr_rom[bytes - 1] == 0xD7);
        size_t physical_offset = 0x123;
        uint16_t ppu_address = 0x123;
        if (mapper == 13 || mapper == 66) {
            size_t page_size = mapper == 13 ? 0x1000 : 0x2000;
            BOARD_CHECK(cpu_store(0x8000, 7)); // D2 is not a bank-select line.
            physical_offset += 3 * page_size;
            if (mapper == 13) ppu_address += 0x1000;
        } else {
            BOARD_CHECK(select_fixed_board(mapper, 0xFF));
        }
        ppu_write(ppu_address, 0xA6);
        expected[physical_offset] = 0xA6;
        BOARD_CHECK(ppu_read(ppu_address) == 0xA6);
        cart_battery_flush();
        BOARD_CHECK(read_file_exact(save_path, saved, bytes));
        BOARD_CHECK(memcmp(saved, expected, bytes) == 0);
        BOARD_CHECK(unload_rom());
        BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
        if (mapper == 13 || mapper == 66) BOARD_CHECK(cpu_store(0x8000, 3));
        BOARD_CHECK(ppu_read(ppu_address) == 0xA6 && chr_rom[bytes - 1] == 0xD7);
        BOARD_CHECK(unload_rom() && remove(save_path) == 0);
        BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
        BOARD_CHECK(chr_rom[physical_offset] == 0 && chr_rom[bytes - 1] == 0);
        BOARD_CHECK(unload_rom() && remove(rom_path) == 0);
        free(saved);
        free(expected);
        board_image_free(&image);
    }
    return 0;
}

int test_native_chr_capacity_accuracy(void) {
    if (!nes_set_ram_power_on_state(NES_RAM_POWER_ZERO)) return 1;
    int failures = test_fixed_windows_and_replacement();
    failures += test_masked_chr_banks();
    failures += test_latched_short_chr_rom();
    failures += test_unreachable_nvram_persistence();
    unload_rom();
    if (!nes_set_ram_power_on_state(NES_RAM_POWER_DEFAULT)) ++failures;
    printf("Native CHR capacity: 4 groups, %d failures\n", failures);
    return failures;
}
