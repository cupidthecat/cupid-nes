/*
 * native_mixed_chr_accuracy.c - Native mixed CHR RAM and NVRAM regressions
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

static bool cpu_store(uint16_t address, uint8_t value) {
    write_mem(0x0200, 0xA9);
    write_mem(0x0201, value);
    write_mem(0x0202, 0x8D);
    write_mem(0x0203, (uint8_t)address);
    write_mem(0x0204, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4;
}

static bool cpu_load(uint16_t address, uint8_t *value) {
    if (!value) {
        return false;
    }
    write_mem(0x0200, 0xAD);
    write_mem(0x0201, (uint8_t)address);
    write_mem(0x0202, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    if (cpu_step(&cpu) != 4) {
        return false;
    }
    *value = cpu.a;
    return true;
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
    for (unsigned instruction = 0; instruction < 10000; ++instruction) {
        if (cpu_step(&cpu) != 3) {
            return false;
        }
    }
    return true;
}

static bool set_ppu_address(uint16_t address) {
    return cpu_store(0x2006, (uint8_t)(address >> 8)) && cpu_store(0x2006, (uint8_t)address);
}

static bool ppudata_write(uint16_t address, uint8_t value) {
    if (!set_ppu_address(address) || !cpu_store(0x2007, value)) {
        return false;
    }
    // Finish the delayed PPU bus write before inspecting memory or selecting another bus address.
    write_mem(0x0200, 0xEA);
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 2;
}

static bool ppudata_read(uint16_t address, uint8_t *value) {
    uint8_t buffered;
    return set_ppu_address(address) && cpu_load(0x2007, &buffered) && cpu_load(0x2007, value);
}

static bool write_file(const char *path, const uint8_t *data, size_t size) {
    FILE *file = fopen(path, "wb");
    if (!file) {
        return false;
    }
    bool ok = fwrite(data, 1, size, file) == size;
    return fclose(file) == 0 && ok;
}

static bool read_file_exact(const char *path, uint8_t *data, size_t size) {
    FILE *file = fopen(path, "rb");
    if (!file) {
        return false;
    }
    bool ok = fread(data, 1, size, file) == size;
    int trailing = fgetc(file);
    return fclose(file) == 0 && ok && trailing == EOF;
}

static bool file_absent(const char *path) {
    FILE *file = fopen(path, "rb");
    if (!file) {
        return true;
    }
    fclose(file);
    return false;
}

static bool make_chr_ram_image(BoardImage *image, unsigned mapper, uint8_t chr_ram_shift, uint8_t chr_nvram_shift) {
    if (!board_image_create(image, mapper, 0x20000, 0, true)) {
        return false;
    }
    iNESHeader *header = (iNESHeader *)image->data;
    header->zero[0] = (uint8_t)(chr_ram_shift | (chr_nvram_shift << 4));
    if (chr_nvram_shift) {
        header->flags6 |= 2;
    }
    memset(image->data + sizeof(*header), 0xFF, 0x20000);
    return true;
}

static bool make_paths(char *rom_path, size_t rom_size, char *save_path, size_t save_size, const char *tag) {
    int used = snprintf(rom_path, rom_size, "build/mixed-chr-%s-%lu-%lu.nes", tag, (unsigned long)time(NULL),
                        (unsigned long)clock());
    if (used <= 0 || (size_t)used >= rom_size || (size_t)used + 5 >= save_size) {
        return false;
    }
    memcpy(save_path, rom_path, (size_t)used + 1);
    strcpy(strrchr(save_path, '.'), ".chr.sav");
    return true;
}

static int test_fixed_mixed_persistence(void) {
    BoardImage image;
    BOARD_CHECK(make_chr_ram_image(&image, 0, 6, 6)); // 4 KiB work + 4 KiB save.
    char rom_path[160], save_path[168];
    BOARD_CHECK(make_paths(rom_path, sizeof(rom_path), save_path, sizeof(save_path), "fixed"));
    BOARD_CHECK(write_file(rom_path, image.data, image.size));

    uint8_t initial[0x1000], saved[0x1000];
    memset(initial, 0xA5, sizeof(initial));
    initial[0x123] = 0xB6;
    BOARD_CHECK(write_file(save_path, initial, sizeof(initial)));
    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart() && wait_for_ppu_register_writes());
    BOARD_CHECK(chr_size == 0x2000 && chr_rom[0x0123] == 0 && chr_rom[0x1123] == 0xB6);

    uint8_t value = 0;
    BOARD_CHECK(ppudata_write(0x0123, 0x4C) && ppudata_read(0x0123, &value) && value == 0x4C);
    cart_battery_flush();
    BOARD_CHECK(read_file_exact(save_path, saved, sizeof(saved)));
    BOARD_CHECK(memcmp(saved, initial, sizeof(saved)) == 0);

    BOARD_CHECK(ppudata_write(0x1123, 0xD2) && ppudata_read(0x1123, &value) && value == 0xD2);
    cart_battery_flush();
    initial[0x123] = 0xD2;
    BOARD_CHECK(read_file_exact(save_path, saved, sizeof(saved)));
    BOARD_CHECK(memcmp(saved, initial, sizeof(saved)) == 0);

    BoardImage rejected;
    BOARD_CHECK(board_image_create(&rejected, 30, 0x20000, 0x2000, true));
    iNESHeader *rejected_header = (iNESHeader *)rejected.data;
    rejected_header->zero[0] = 0x66;
    rejected_header->flags6 |= 0x0B;
    uint8_t *previous_chr = chr_rom;
    Mapper *previous_cart = cart;
    BOARD_CHECK(load_rom_memory(rejected.data, rejected.size) == -1);
    BOARD_CHECK(chr_rom == previous_chr && cart == previous_cart);
    BOARD_CHECK(ppu_read(0x0123) == 0x4C && ppu_read(0x1123) == 0xD2);
    board_image_free(&rejected);

    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
    BOARD_CHECK(chr_rom[0x0123] == 0 && chr_rom[0x1123] == 0xD2);
    BOARD_CHECK(unload_rom());

    memset(saved, 0x33, 32);
    BOARD_CHECK(write_file(save_path, saved, 32));
    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
    BOARD_CHECK(chr_rom[0] == 0 && chr_rom[0x1000] == 0x33 && chr_rom[0x101F] == 0x33);
    BOARD_CHECK(chr_rom[0x1020] == 0 && chr_rom[0x1FFF] == 0);
    ppu_write(0x1020, 0xE4);
    cart_battery_flush();
    memset(saved, 0, sizeof(saved));
    memset(saved, 0x33, 32);
    saved[0x20] = 0xE4;
    BOARD_CHECK(read_file_exact(save_path, initial, sizeof(initial)));
    BOARD_CHECK(memcmp(initial, saved, sizeof(saved)) == 0);
    BOARD_CHECK(unload_rom() && remove(save_path) == 0);

    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
    BOARD_CHECK(chr_rom[0x0123] == 0 && chr_rom[0x1123] == 0);
    BOARD_CHECK(unload_rom() && remove(rom_path) == 0);
    board_image_free(&image);
    return 0;
}

static int test_banked_unequal_mixed_layout(void) {
    BoardImage image;
    BOARD_CHECK(make_chr_ram_image(&image, 13, 6, 7)); // 4 KiB work + 8 KiB save.
    char rom_path[160], save_path[168];
    BOARD_CHECK(make_paths(rom_path, sizeof(rom_path), save_path, sizeof(save_path), "cprom"));
    BOARD_CHECK(write_file(rom_path, image.data, image.size));
    (void)remove(save_path);
    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart() && wait_for_ppu_register_writes());
    BOARD_CHECK(chr_size == 0x3000);

    uint8_t value = 0;
    BOARD_CHECK(cpu_store(0x8000, 0));
    BOARD_CHECK(ppudata_write(0x1123, 0x31) && ppudata_read(0x1123, &value) && value == 0x31);
    BOARD_CHECK(chr_rom[0x0123] == 0x31);
    ppu_write(0x1FFF, 0x41);
    BOARD_CHECK(chr_rom[0x0FFF] == 0x41);
    cart_battery_flush();
    BOARD_CHECK(file_absent(save_path));

    BOARD_CHECK(cpu_store(0x8000, 1));
    BOARD_CHECK(ppudata_write(0x1123, 0x52));
    ppu_write(0x1FFF, 0x62);
    BOARD_CHECK(chr_rom[0x1123] == 0x52);
    BOARD_CHECK(chr_rom[0x1FFF] == 0x62);
    BOARD_CHECK(cpu_store(0x8000, 2));
    BOARD_CHECK(ppudata_write(0x1123, 0x73) && chr_rom[0x2123] == 0x73);
    BOARD_CHECK(cpu_store(0x8000, 3));
    BOARD_CHECK(ppudata_read(0x1123, &value) && value == 0x31);

    uint8_t expected[0x2000] = {0};
    uint8_t saved[0x2000];
    expected[0x0123] = 0x52;
    expected[0x0FFF] = 0x62;
    expected[0x1123] = 0x73;
    cart_battery_flush();
    BOARD_CHECK(read_file_exact(save_path, saved, sizeof(saved)));
    BOARD_CHECK(memcmp(saved, expected, sizeof(saved)) == 0);

    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
    BOARD_CHECK(chr_rom[0x0123] == 0 && chr_rom[0x0FFF] == 0);
    BOARD_CHECK(chr_rom[0x1123] == 0x52 && chr_rom[0x1FFF] == 0x62 && chr_rom[0x2123] == 0x73);
    BOARD_CHECK(unload_rom() && remove(save_path) == 0 && remove(rom_path) == 0);
    board_image_free(&image);
    return 0;
}

static int test_unrom512_mixed_nametable_persistence(void) {
    BoardImage image;
    BOARD_CHECK(make_chr_ram_image(&image, 30, 8, 8)); // 16 KiB work + 16 KiB save.
    ((iNESHeader *)image.data)->flags6 |= 9;
    char rom_path[160], save_path[168];
    BOARD_CHECK(make_paths(rom_path, sizeof(rom_path), save_path, sizeof(save_path), "unrom512"));
    BOARD_CHECK(write_file(rom_path, image.data, image.size));
    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart() && wait_for_ppu_register_writes());
    BOARD_CHECK(chr_size == 0x8000 && cart_get_mirroring() == MIRROR_FOUR);

    uint8_t value = 0;
    BOARD_CHECK(ppudata_write(0x0123, 0x4C));
    BOARD_CHECK(ppudata_read(0x0123, &value) && value == 0x4C);
    cart_battery_flush();
    BOARD_CHECK(file_absent(save_path));

    BOARD_CHECK(ppudata_write(0x2000, 0xA6));
    BOARD_CHECK(ppudata_write(0x3123, 0xB7));
    BOARD_CHECK(ppudata_read(0x2000, &value) && value == 0xA6);
    BOARD_CHECK(chr_rom[0x6000] == 0xA6 && chr_rom[0x7123] == 0xB7);
    // The last pattern bank aliases the CHR-backed nametable allocation.
    BOARD_CHECK(cpu_store(0xC000, 0x60));
    BOARD_CHECK(ppudata_read(0x0000, &value) && value == 0xA6);
    BOARD_CHECK(ppudata_write(0x1FFF, 0xC8));
    BOARD_CHECK(chr_rom[0x7FFF] == 0xC8);

    uint8_t expected[0x4000] = {0};
    uint8_t saved[0x4000];
    expected[0x2000] = 0xA6;
    expected[0x3123] = 0xB7;
    expected[0x3FFF] = 0xC8;
    cart_battery_flush();
    BOARD_CHECK(read_file_exact(save_path, saved, sizeof(saved)));
    BOARD_CHECK(memcmp(saved, expected, sizeof(saved)) == 0);
    BOARD_CHECK(unload_rom());

    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart() && wait_for_ppu_register_writes());
    BOARD_CHECK(chr_rom[0x0123] == 0 && chr_rom[0x6000] == 0xA6);
    BOARD_CHECK(ppudata_read(0x3123, &value) && value == 0xB7);
    BOARD_CHECK(cpu_store(0xC000, 0x60));
    BOARD_CHECK(ppudata_read(0x1FFF, &value) && value == 0xC8);
    BOARD_CHECK(unload_rom() && remove(save_path) == 0 && remove(rom_path) == 0);
    board_image_free(&image);
    return 0;
}

static int test_single_kind_and_protected_open_bus(void) {
    static const struct {
        uint8_t work_shift;
        uint8_t save_shift;
    } single[] = {{7, 0}, {0, 7}};

    for (size_t i = 0; i < sizeof(single) / sizeof(single[0]); ++i) {
        BoardImage image;
        BOARD_CHECK(make_chr_ram_image(&image, 0, single[i].work_shift, single[i].save_shift));
        BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart());
        BOARD_CHECK(chr_size == 0x2000);
        ppu_write(0x0123, (uint8_t)(0x81 + i));
        BOARD_CHECK(ppu_read(0x0123) == (uint8_t)(0x81 + i));
        BOARD_CHECK(unload_rom());
        board_image_free(&image);
    }

    BoardImage protected_image;
    BOARD_CHECK(make_chr_ram_image(&protected_image, 185, 6, 6));
    BOARD_CHECK(load_rom_memory(protected_image.data, protected_image.size) == 0 && power_cart());
    ppu_write(0x0122, 0xC7);
    BOARD_CHECK(ppu_read(0x0122) == 0xC7);
    BOARD_CHECK(cpu_store(0x8000, 0));
    BOARD_CHECK(ppu_read(0x0122) == 0x23);
    ppu_write(0x0122, 0x4A);
    BOARD_CHECK(ppu_read(0x0122) == 0x23 && chr_rom[0x0122] == 0xC7);
    BOARD_CHECK(unload_rom());
    board_image_free(&protected_image);
    return 0;
}

static int check_partial_chr_window(size_t mapped_end, size_t period) {
    for (size_t offset = 0; offset < chr_size; ++offset)
        chr_rom[offset] = (uint8_t)(0x40u + (offset / 0x100u) * 7u);

    for (unsigned address = 0; address < 0x2000; ++address) {
        uint8_t expected = address < mapped_end
            ? chr_rom[address % period] : (uint8_t)address;
        BOARD_CHECK(cart_ppu_read((uint16_t)address) == expected);
    }

    uint8_t before[0x2000];
    BOARD_CHECK(chr_size <= sizeof(before));
    memcpy(before, chr_rom, chr_size);
    for (unsigned address = (unsigned)mapped_end; address < 0x2000; ++address)
        cart_ppu_write((uint16_t)address, 0xE3);
    BOARD_CHECK(memcmp(before, chr_rom, chr_size) == 0);

    if (mapped_end) {
        cart_ppu_write(0x53, 0xA7);
        BOARD_CHECK(chr_rom[0x53] == 0xA7);
        for (size_t address = 0x53; address < mapped_end; address += period)
            BOARD_CHECK(cart_ppu_read((uint16_t)address) == 0xA7);
    }
    return 0;
}

static int test_partial_chr_windows(void) {
    static const struct {
        unsigned mapper;
        uint8_t work_shift, save_shift;
        size_t mapped_end, period;
    } cases[] = {
        {0, 5, 6, 0x1800, 0x1800}, // One 6 KiB page, then an open 2 KiB tail.
        {0, 4, 5, 0x1800, 0x0C00}, // Two complete 3 KiB copies.
        {0, 2, 3, 0x1E00, 0x0300}, // Ten complete 768-byte copies.
        {0, 1, 0, 0, 1},          // A 128-byte page cannot occupy a 256-byte slot.
        {0, 1, 2, 0, 1},          // Neither can a 384-byte page.
        {0, 5, 5, 0x2000, 0x1000}, // Exact divisors still mirror across the full window.
        {0, 6, 6, 0x2000, 0x2000},
        {2, 5, 6, 0x1800, 0x1800},
        {3, 4, 5, 0x1800, 0x0C00},
        {3, 1, 2, 0, 1},
        {13, 5, 6, 0x2000, 0x1000}, // CPROM maps complete 4 KiB banks.
        {13, 1, 2, 0, 1},
        {16, 5, 6, 0x2000, 0x1800}, // Six 1 KiB banks, then the first two again.
        {67, 4, 5, 0x2000, 0x0800}, // A 3 KiB allocation has one complete 2 KiB bank.
        {85, 4, 5, 0x2000, 0x0C00},
        {93, 5, 6, 0x1800, 0x1800},
        {185, 1, 2, 0, 1},
    };
    int failures = 0;
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        BoardImage image;
        BOARD_CHECK(make_chr_ram_image(&image, cases[i].mapper,
                                       cases[i].work_shift, cases[i].save_shift));
        int result = load_rom_memory(image.data, image.size);
        if (result == 0) result = check_partial_chr_window(cases[i].mapped_end, cases[i].period);
        if (result != 0) {
            fprintf(stderr, "Partial CHR case %zu: mapper %u, work shift %u, save shift %u\n",
                    i, cases[i].mapper, cases[i].work_shift, cases[i].save_shift);
            ++failures;
        }
        unload_rom();
        board_image_free(&image);
    }
    return failures;
}

static int test_partial_chr_persistence(void) {
    BoardImage image;
    BOARD_CHECK(make_chr_ram_image(&image, 0, 4, 5)); // 1 KiB work + 2 KiB save.
    char rom_path[160], save_path[168];
    BOARD_CHECK(make_paths(rom_path, sizeof(rom_path), save_path, sizeof(save_path), "partial"));
    BOARD_CHECK(write_file(rom_path, image.data, image.size));
    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart() && wait_for_ppu_register_writes());
    BOARD_CHECK(chr_size == 0x0C00);
    BOARD_CHECK(ppudata_write(0x1C53, 0xD2));
    cart_battery_flush();
    BOARD_CHECK(file_absent(save_path) && chr_rom[0x0453] == 0);
    uint8_t value;
    BOARD_CHECK(ppudata_read(0x1C53, &value) && value == 0x53);
    BOARD_CHECK(ppudata_write(0x1053, 0xB7));
    BOARD_CHECK(chr_rom[0x0453] == 0xB7);
    uint8_t saved[0x0800], expected[0x0800] = {0};
    expected[0x53] = 0xB7;
    cart_battery_flush();
    BOARD_CHECK(read_file_exact(save_path, saved, sizeof(saved)));
    BOARD_CHECK(memcmp(saved, expected, sizeof(saved)) == 0);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
    BOARD_CHECK(ppu_read(0x0453) == 0xB7 && ppu_read(0x1053) == 0xB7);
    BOARD_CHECK(ppu_read(0x1C53) == 0x53);
    BOARD_CHECK(unload_rom() && remove(save_path) == 0 && remove(rom_path) == 0);
    board_image_free(&image);

    BOARD_CHECK(make_chr_ram_image(&image, 13, 5, 6)); // 2 KiB work + 4 KiB save.
    BOARD_CHECK(make_paths(rom_path, sizeof(rom_path), save_path, sizeof(save_path), "partial-bank"));
    BOARD_CHECK(write_file(rom_path, image.data, image.size));
    uint8_t bank_saved[0x1000], bank_expected[0x1000];
    memset(bank_expected, 0xA5, sizeof(bank_expected));
    BOARD_CHECK(write_file(save_path, bank_expected, sizeof(bank_expected)));
    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart() && wait_for_ppu_register_writes());
    BOARD_CHECK(cpu_store(0x8000, 1));
    BOARD_CHECK(ppudata_write(0x1833, 0xC8));
    BOARD_CHECK(chr_rom[0x0833] == 0xC8 && chr_rom[0x1033] == 0xA5);
    bank_expected[0x33] = 0xC8;
    cart_battery_flush();
    BOARD_CHECK(read_file_exact(save_path, bank_saved, sizeof(bank_saved)));
    BOARD_CHECK(memcmp(bank_saved, bank_expected, sizeof(bank_saved)) == 0);
    BOARD_CHECK(unload_rom());
    BOARD_CHECK(load_rom(rom_path) == 0 && power_cart());
    BOARD_CHECK(chr_rom[0x0833] == 0xC8 && chr_rom[0x1033] == 0xA5);
    BOARD_CHECK(unload_rom() && remove(save_path) == 0 && remove(rom_path) == 0);
    board_image_free(&image);
    return 0;
}

static int check_vrc6_partial_nametables(unsigned mapper, uint8_t work_shift,
                                        uint8_t save_shift, size_t chr_coverage) {
    BoardImage image;
    BOARD_CHECK(make_chr_ram_image(&image, mapper, work_shift, save_shift));
    BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart()
                && wait_for_ppu_register_writes());
    memset(chr_rom, 0xC4, chr_size);
    memset(ppu_vram, 0x41, 0x400);
    memset(ppu_vram + 0x400, 0x82, 0x400);
    BOARD_CHECK(cpu_store(0xB003, 0x2B)); // Select CIRAM page B in all four tables.
    BOARD_CHECK(cpu_store(0xB003, 0x12)); // A partial CHR mapping must retain the rest.
    for (unsigned nt = 0; nt < 4; ++nt) {
        for (unsigned in = 0; in < 0x400; ++in) {
            uint16_t address = (uint16_t)(0x2000 + nt * 0x400 + in);
            uint8_t expected = in < chr_coverage ? 0xC4 : 0x82;
            BOARD_CHECK(ppu_read(address) == expected);
            if (address < 0x2F00)
                BOARD_CHECK(ppu_read((uint16_t)(address + 0x1000)) == expected);
        }
    }

    BOARD_CHECK(ppudata_write(0x2353, 0xD6));
    BOARD_CHECK(ppu_vram[0x753] == 0xD6 && chr_rom[0x53] == 0xC4);
    BOARD_CHECK(ppudata_write(0x2053, 0xA7));
    if (chr_coverage) BOARD_CHECK(chr_rom[0x53] == 0xA7 && ppu_vram[0x453] == 0x82);
    else BOARD_CHECK(chr_rom[0x53] == 0xC4 && ppu_vram[0x453] == 0xA7);

    uint16_t chr_register = mapper == 26 ? 0xE001 : 0xE002;
    BOARD_CHECK(cpu_store(chr_register, 1));
    BOARD_CHECK(ppu_read(0x2353) == 0xD6 && ppu_read(0x2F53) == 0xD6);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(ppu_read(0x2353) == 0xD6);
    BOARD_CHECK(cpu_store(0xB003, 0x28)); // Replace all chunks with CIRAM page A.
    BOARD_CHECK(cpu_store(0xB003, 0x12));
    BOARD_CHECK(ppu_read(0x2353) == 0x41 && ppu_read(0x2F53) == 0x41);
    BOARD_CHECK(unload_rom());
    board_image_free(&image);
    return 0;
}

static int test_vrc6_partial_nametables(void) {
    int failures = 0;
    const unsigned mappers[] = {24, 26};
    for (size_t i = 0; i < sizeof(mappers) / sizeof(mappers[0]); ++i) {
        failures += check_vrc6_partial_nametables(mappers[i], 1, 2, 0);
        failures += check_vrc6_partial_nametables(mappers[i], 2, 3, 0x300);
    }
    return failures;
}

static int test_sunsoft4_partial_nametables(void) {
    BoardImage image;
    BOARD_CHECK(make_chr_ram_image(&image, 68, 1, 2));
    BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart()
                && wait_for_ppu_register_writes());
    static const struct { uint8_t bank; size_t base, coverage; } cases[] = {
        {1, 0, 0x100}, {0, 0x80, 0x100}, {2, 0x100, 0},
    };
    BOARD_CHECK(chr_size == 0x180);
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); ++c) {
        memset(chr_rom, 0xC4, chr_size);
        BOARD_CHECK(cpu_store(0xC000, cases[c].bank) && cpu_store(0xD000, cases[c].bank));
        BOARD_CHECK(cpu_store(0xE000, 0x11));
        for (unsigned nt = 0; nt < 4; ++nt) {
            for (unsigned in = 0; in < 0x400; ++in) {
                uint16_t address = (uint16_t)(0x2000 + nt * 0x400 + in);
                uint8_t expected = in < cases[c].coverage ? 0xC4 : (uint8_t)address;
                BOARD_CHECK(ppu_read(address) == expected);
            }
        }
        uint8_t before[0x180];
        memcpy(before, chr_rom, sizeof(before));
        BOARD_CHECK(ppudata_write(0x2153, 0xD7));
        BOARD_CHECK(memcmp(before, chr_rom, sizeof(before)) == 0);
        BOARD_CHECK(ppudata_write(0x2053, 0xA9));
        if (cases[c].coverage) BOARD_CHECK(chr_rom[cases[c].base + 0x53] == 0xA9);
        else BOARD_CHECK(memcmp(before, chr_rom, sizeof(before)) == 0);
    }
    BOARD_CHECK(unload_rom());
    board_image_free(&image);
    return 0;
}

static int test_rom_adjacent_storage_stays_separate(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0x2000, true));
    iNESHeader *header = (iNESHeader *)image.data;
    header->zero[0] = 0x66;
    header->flags6 |= 2;
    BOARD_CHECK(load_rom_memory(image.data, image.size) == 0 && power_cart());
    BOARD_CHECK(chr_size == 0x2000 && ppu_read(0x0123) == 0 && ppu_read(0x1123) == 4);
    ppu_write(0x1123, 0xE9);
    BOARD_CHECK(ppu_read(0x1123) == 4);
    BOARD_CHECK(unload_rom());
    board_image_free(&image);
    return 0;
}

int test_native_mixed_chr_accuracy(void) {
    if (!nes_set_ram_power_on_state(NES_RAM_POWER_ZERO)) {
        return 1;
    }
    int failures = test_fixed_mixed_persistence();
    failures += test_banked_unequal_mixed_layout();
    failures += test_unrom512_mixed_nametable_persistence();
    failures += test_single_kind_and_protected_open_bus();
    failures += test_rom_adjacent_storage_stays_separate();
    failures += test_partial_chr_windows();
    failures += test_partial_chr_persistence();
    failures += test_vrc6_partial_nametables();
    failures += test_sunsoft4_partial_nametables();
    unload_rom();
    if (!nes_set_ram_power_on_state(NES_RAM_POWER_DEFAULT)) {
        ++failures;
    }
    printf("Native mixed CHR storage: 9 groups, %d failures\n", failures);
    return failures;
}
