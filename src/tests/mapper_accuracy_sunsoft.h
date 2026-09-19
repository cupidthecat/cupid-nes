/*
 * mapper_accuracy_sunsoft.h - Sunsoft mapper regression tests
 *
 * Author: @frankischilling
 *
 * This private header contains regression tests for supported Sunsoft cartridge
 * boards and audio hardware.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef MAPPER_ACCURACY_SUNSOFT_H
#define MAPPER_ACCURACY_SUNSOFT_H

static int test_sunsoft_discrete_boards(void) {
    CHECK(fixture(89, 0x20000, 0x20000, false) == 89);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read(0xC000) == 14);
    cart_cpu_write(0x8ABC, 0xD5);
    CHECK(cart_cpu_read(0x8000) == 10 && cart_cpu_read(0xC000) == 14);
    CHECK(cart_ppu_read(0) == 13 * 8 && cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0x8000, 0x55);
    CHECK(cart_cpu_read(0x8000) == 10 && cart_ppu_read(0) == 5 * 8);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0x8000, 0x5D);
    CHECK(cart_cpu_read(0x8000) == 10 && cart_ppu_read(0) == 5 * 8);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    cart_cpu_write(0x8000, 0x6D);
    CHECK(cart_cpu_read(0x8000) == 12 && cart_ppu_read(0) == 5 * 8);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    cart_cpu_write(0xFFFF, 0x7A);
    CHECK(cart_cpu_read(0x8000) == 14 && cart_ppu_read(0) == 2 * 8);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    uint8_t prg_before = cart_cpu_read(0x8000);
    cart_cpu_write(0x6000, 0xA6);
    CHECK(cart_cpu_read(0x6000) == 0xA6 && cart_cpu_read(0x8000) == prg_before);
    cart->reset();
    CHECK(cart_cpu_read_bus(0x8000, 0x69) == 0x69);

    CHECK(fixture(93, 0x20000, 0x2000, false) == 93);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0x8000, 0x51);
    CHECK(cart_cpu_read(0x8000) == 10 && cart_cpu_read(0xC000) == 14);
    CHECK(cart_ppu_read(0x0123) == 0);
    cart_cpu_write(0xFFFF, 0x70);
    CHECK(cart_cpu_read(0x8000) == 14 && cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0x9000, 0x31);
    CHECK(cart_cpu_read(0x8000) == 6 && cart_ppu_read(0x1FFF) == 7);
    cart->reset();
    CHECK(cart_ppu_read(0x0123) == 0x23 && cart_cpu_read_bus(0x8000, 0x35) == 0x35);

    CHECK(fixture(93, 0x20000, 0x2000, true) == 93);
    cart_ppu_write(0x0123, 0xA6);
    CHECK(cart_ppu_read(0x0123) == 0xA6);
    cart_cpu_write(0x8000, 0);
    cart_ppu_write(0x0123, 0x53);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0x8000, 1);
    CHECK(cart_ppu_read(0x0123) == 0xA6);
    cart->reset();
    CHECK(cart_ppu_read(0x0123) == 0xA6);

    CHECK(fixture(184, 0x8000, 0x8000, false) == 184);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xFFFF) == 3);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0x5FFF, 0x76);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0x6000, 0x26);
    CHECK(cart_ppu_read(0) == 24 && cart_ppu_read(0x1000) == 24);
    cart_cpu_write(0x7FFF, 0x31);
    CHECK(cart_ppu_read(0) == 4 && cart_ppu_read(0x1000) == 28);
    cart_cpu_write(0x8000, 0x77);
    CHECK(cart_ppu_read(0) == 4 && cart_cpu_read(0x8000) == 0);
    cart->reset();
    CHECK(cart_ppu_read(0x0123) == 0x23);
    return 0;
}

static int test_sunsoft_shrunk_chr_pages(void) {
    size_t size;
    iNESHeader h;
    uint8_t *image;
    uint8_t *prg;
    uint8_t *chr;

    // Sunsoft 3 normally uses four 2 KiB CHR slots. With a 1 KiB ROM the
    // mapper page size shrinks to 1 KiB, so the four written slots occupy only
    // $0000-$0FFF and the upper pattern table remains open bus.
    h = header_for(67, 0x8000, false);
    h.flags7 |= 0x08;
    h.chr_rom_chunks = 0x28; // 1 * 2^10 = 1 KiB.
    h.flags9 = 0xF0;
    image = image_for(&h, 0x8000, 0x0400, &size);
    CHECK(image != NULL);
    prg = image + sizeof(h);
    chr = prg + 0x8000;
    memset(prg, 0x11, 0x4000);
    memset(prg + 0x4000, 0x22, 0x4000);
    memset(chr, 0x71, 0x0400);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0x8800, 7);
    cart_cpu_write(0x9800, 6);
    cart_cpu_write(0xA800, 5);
    cart_cpu_write(0xB800, 4);
    CHECK(cart_ppu_read(0x0123) == 0x71 && cart_ppu_read(0x0523) == 0x71);
    CHECK(cart_ppu_read(0x0923) == 0x71 && cart_ppu_read(0x0D23) == 0x71);
    CHECK(cart_ppu_read(0x1123) == 0x23);
    cart_cpu_write(0xE800, 3);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    cart_cpu_write(0xF800, 1);
    CHECK(cart_cpu_read(0x8000) == 0x22 && cart_cpu_read(0xC000) == 0x22);

    // Sunsoft 4 has the same four pattern slots. A 512-byte image shrinks each
    // slot to 512 bytes; CHR-backed nametables also expose only bytes that
    // exist in the selected physical image.
    h = header_for(68, 0x8000, false);
    h.flags7 |= 0x08;
    h.chr_rom_chunks = 0x24; // 512 bytes.
    h.flags9 = 0xF0;
    image = image_for(&h, 0x8000, 0x0200, &size);
    CHECK(image != NULL);
    prg = image + sizeof(h);
    chr = prg + 0x8000;
    memset(prg, 0x31, 0x4000);
    memset(prg + 0x4000, 0x42, 0x4000);
    memset(chr, 0x82, 0x0200);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    cart_cpu_write(0x8000, 1);
    cart_cpu_write(0x9000, 2);
    cart_cpu_write(0xA000, 3);
    cart_cpu_write(0xB000, 4);
    CHECK(cart_ppu_read(0x0100) == 0x82 && cart_ppu_read(0x0300) == 0x82);
    CHECK(cart_ppu_read(0x0500) == 0x82 && cart_ppu_read(0x0700) == 0x82);
    CHECK(cart_ppu_read(0x0900) == 0x00);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xD000, 0);
    cart_cpu_write(0xE000, 0x10);
    uint8_t nt[0x1000] = {0};
    CHECK(cart_nt_read(0x2001, nt) == 0x82);
    CHECK(cart_nt_read(0x2223, nt) == 0x23);
    cart_cpu_write(0xE000, 0x13);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);

    // Mapper 89 has one 8 KiB CHR slot. With 2 KiB of CHR, the selected slot
    // occupies only $0000-$07FF; bank and mirroring writes leave the rest open.
    h = header_for(89, 0x8000, false);
    h.flags7 |= 0x08;
    h.chr_rom_chunks = 0x2C; // 2 KiB.
    h.flags9 = 0xF0;
    image = image_for(&h, 0x8000, 0x0800, &size);
    CHECK(image != NULL);
    memset(image + sizeof(h) + 0x8000, 0x93, 0x0800);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    cart_cpu_write(0x8000, 0x5D);
    CHECK(cart_ppu_read(0x0123) == 0x93 && cart_ppu_read(0x0923) == 0x23);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);

    // Mapper 93 gates its single CHR slot. The slot shrinks with the image and
    // disabling it removes the entire pattern-table mapping.
    h = header_for(93, 0x8000, false);
    h.flags7 |= 0x08;
    h.chr_rom_chunks = 0x2C;
    h.flags9 = 0xF0;
    image = image_for(&h, 0x8000, 0x0800, &size);
    CHECK(image != NULL);
    memset(image + sizeof(h) + 0x8000, 0xA4, 0x0800);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    cart_cpu_write(0x8000, 0x11);
    CHECK(cart_ppu_read(0x0123) == 0xA4 && cart_ppu_read(0x0923) == 0x23);
    cart_cpu_write(0x8000, 0x10);
    CHECK(cart_ppu_read(0x0123) == 0x23);

    // A small RAM chip initially aliases throughout both pattern tables. An
    // enable write retains that mapping; disabling it removes every alias.
    h = header_for(93, 0x8000, true);
    h.flags7 |= 8;
    h.zero[0] = 3;
    image = image_for(&h, 0x8000, 0, &size);
    CHECK(image != NULL && load_rom_memory(image, size) == 0);
    free(image);
    cart_ppu_write(0x1F23, 0x5A);
    CHECK(cart_ppu_read(0x0123) == 0x5A && cart_ppu_read(0x1F23) == 0x5A);
    cart_cpu_write(0x8000, 1);
    cart_ppu_write(0x1F23, 0xA5);
    CHECK(cart_ppu_read(0x0123) == 0xA5 && cart_ppu_read(0x1F23) == 0xA5);
    cart_cpu_write(0x8000, 0);
    cart_ppu_write(0x1F23, 0xEE);
    CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x1F23) == 0x23);
    cart_cpu_write(0x8000, 1);
    CHECK(cart_ppu_read(0x0123) == 0xA5 && cart_ppu_read(0x1F23) == 0x23);
    cart_ppu_write(0x1F23, 0xEE);
    CHECK(cart_ppu_read(0x0123) == 0xA5);
    ppu_power_on(&ppu);
    CHECK(cpu_power_on(&cpu));
    cpu_soft_reset(&cpu);
    CHECK(cart_ppu_read(0x0123) == 0xA5 && cart_ppu_read(0x1F23) == 0x23);

    // With CHR RAM, the base mapper starts with the whole pattern table mapped.
    // Enabling the Sunsoft slot before any disable only replaces the low slot,
    // leaving the inherited high aliases in place. A disable removes the full
    // mapping; re-enabling later restores only the reduced low slot.
    CHECK(fixture(93, 0x8000, 0x0800, true) == 93);
    cart_ppu_write(0x0923, 0x53);
    CHECK(cart_ppu_read(0x0123) == 0x53 && cart_ppu_read(0x0923) == 0x53);
    cart_cpu_write(0x8000, 0x11);
    cart_ppu_write(0x0923, 0xA6);
    CHECK(cart_ppu_read(0x0123) == 0xA6 && cart_ppu_read(0x0923) == 0xA6);
    cart_cpu_write(0x8000, 0x10);
    CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x0923) == 0x23);
    cart_cpu_write(0x8000, 0x11);
    CHECK(cart_ppu_read(0x0123) == 0xA6 && cart_ppu_read(0x0923) == 0x23);
    cart_ppu_write(0x0123, 0x69);
    CHECK(cart_ppu_read(0x0123) == 0x69 && cart_ppu_read(0x0923) == 0x23);

    // Mapper 184 exposes two 4 KiB slots. A 2 KiB image shrinks both to 2 KiB,
    // so a register write maps $0000-$0FFF and leaves $1000-$1FFF open.
    h = header_for(184, 0x8000, false);
    h.flags7 |= 0x08;
    h.chr_rom_chunks = 0x2C;
    h.flags9 = 0xF0;
    image = image_for(&h, 0x8000, 0x0800, &size);
    CHECK(image != NULL);
    memset(image + sizeof(h) + 0x8000, 0xB5, 0x0800);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0x6000, 0x31);
    CHECK(cart_ppu_read(0x0123) == 0xB5 && cart_ppu_read(0x0923) == 0xB5);
    CHECK(cart_ppu_read(0x1123) == 0x23);
    return 0;
}

static int test_sunsoft_discrete_loader_rejection(void) {
    CHECK(fixture(89, 0x20000, 0x20000, false) == 89);
    cart_cpu_write(0x8000, 0x31);
    Mapper *previous = cart;
    iNESHeader h = header_for(89, 0x20000, false);
    h.flags7 |= 0x08;
    h.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x20000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 6);
    return 0;
}

static int test_sunsoft_discrete_image_loading(void) {
    const unsigned boards[] = {89, 93, 184};
    for (unsigned i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        bool ram = boards[i] == 93;
        iNESHeader h = header_for(boards[i], 0x8000, ram);
        h.flags7 |= 8;
        if (ram) h.zero[0] = 7;
        size_t image_size;
        uint8_t *image = image_for(&h, 0x8000, ram ? 0 : 0x2000, &image_size);
        CHECK(image != NULL);
        CHECK(load_rom_memory(image, image_size) == 0);
        CHECK(rom_mapper_number(&ines_header) == (int)boards[i]);
        write_mem(boards[i] == 184 ? 0x6000 : 0x8000, 1);
        CHECK(read_mem(0xC000) == 0x5C);
        if (ram) {
            cart_ppu_write(0x123, 0x69);
            CHECK(cart_ppu_read(0x123) == 0x69);
        } else {
            CHECK(cart_ppu_read(0x123) == 0xA5);
        }
        Mapper *previous = cart;
        CHECK(load_rom_memory(image, image_size - 1) == -1);
        CHECK(cart == previous && read_mem(0xC000) == 0x5C);
        free(image);
        CHECK(unload_rom());
    }
    return 0;
}

static int test_taito_x1005_and_207(void) {
    const unsigned mappers[] = {80, 207};
    for (size_t i = 0; i < sizeof(mappers) / sizeof(mappers[0]); ++i) {
        CHECK(fixture(mappers[i], 0x40000, 0x40000, false) == (int)mappers[i]);
        CHECK(cart != NULL && cart->clock != NULL && cart->reset != NULL);
        CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
        CHECK(cart_cpu_read_bus(0xA000, 0x69) == 0x69);
        CHECK(cart_cpu_read_bus(0xC000, 0xA6) == 0xA6);
        CHECK(cart_cpu_read(0xE000) == 31);
        CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x1456) == 0x56);

        cart_cpu_write(0x7EFA, 3);
        cart_cpu_write(0x7EFC, 5);
        cart_cpu_write(0x7EFE, 7);
        CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xA000) == 5);
        CHECK(cart_cpu_read(0xC000) == 7 && cart_cpu_read(0xE000) == 31);

        cart_cpu_write(0x7EF0, 0x84);
        cart_cpu_write(0x7EF1, 0x46);
        cart_cpu_write(0x7EF2, 9);
        cart_cpu_write(0x7EF3, 11);
        cart_cpu_write(0x7EF4, 13);
        cart_cpu_write(0x7EF5, 15);
        const uint8_t expected_chr[] = {0x84, 0x85, 0x46, 0x47, 9, 11, 13, 15};
        for (unsigned slot = 0; slot < 8; ++slot)
            CHECK(cart_ppu_read((uint16_t)(slot * 0x400u)) == expected_chr[slot]);

        CHECK(cart_cpu_read_bus(0x7F00, 0x35) == 0x35);
        cart_cpu_write(0x7F00, 0xA6);
        cart_cpu_write(0x7EF8, 0xA3);
        CHECK(cart_cpu_read(0x7F00) == 0 && cart_cpu_read(0x7F80) == 0);
        cart_cpu_write(0x7F00, 0x35);
        CHECK(cart_cpu_read(0x7F00) == 0x35 && cart_cpu_read(0x7F80) == 0x35);
        cart_cpu_write(0x7F80, 0x53);
        CHECK(cart_cpu_read(0x7F00) == 0x53 && cart_cpu_read(0x7F80) == 0x53);
        cart_cpu_write(0x7EF9, 0);
        CHECK(cart_cpu_read_bus(0x7F00, 0x96) == 0x96);
        cart_cpu_write(0x7EF8, 0xA3);
        CHECK(cart_cpu_read(0x7F00) == 0x53 && cart_cpu_read(0x7F80) == 0x53);

        if (mappers[i] == 80) {
            cart_cpu_write(0x7EF6, 1);
            CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
            cart_cpu_write(0x7EF7, 0);
            CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
        } else {
            uint8_t nt[0x1000] = {0};
            cart_nt_write(0x2001, 0xA1, nt);
            cart_nt_write(0x2801, 0xB2, nt);
            CHECK(cart_nt_read(0x2001, nt) == 0xA1 && cart_nt_read(0x2401, nt) == 0xA1);
            CHECK(cart_nt_read(0x2801, nt) == 0xB2 && cart_nt_read(0x2C01, nt) == 0xB2);
            Mirroring before = cart_get_mirroring();
            cart_cpu_write(0x7EF6, 1);
            CHECK(cart_get_mirroring() == before);
        }

        cart->reset();
        CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xA000) == 5);
        CHECK(cart_cpu_read(0xC000) == 7 && cart_cpu_read(0xE000) == 31);
        CHECK(cart_cpu_read(0x7F00) == 0x53 && cart_cpu_read(0x7F80) == 0x53);
        CHECK(cart_ppu_read(0x0123) == 0x84);
    }
    return 0;
}

static int test_taito_x1017_banks_chr_and_ram(void) {
    CHECK(fixture(82, 0x40000, 0x40000, false) == 82);
    CHECK(cart != NULL && cart->clock != NULL && cart->reset != NULL);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_ppu_read(0x0123) == 0x23);

    cart_cpu_write(0x7EFA, 12);
    cart_cpu_write(0x7EFB, 20);
    cart_cpu_write(0x7EFC, 28);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xA000) == 5);
    CHECK(cart_cpu_read(0xC000) == 7 && cart_cpu_read(0xE000) == 31);

    const uint8_t chr_regs[] = {9, 13, 15, 18, 21, 24};
    for (unsigned i = 0; i < 6; ++i)
        cart_cpu_write((uint16_t)(0x7EF0u + i), chr_regs[i]);
    CHECK(cart_ppu_read(0x0000) == 8 && cart_ppu_read(0x0400) == 9);
    CHECK(cart_ppu_read(0x0800) == 12 && cart_ppu_read(0x0C00) == 13);
    CHECK(cart_ppu_read(0x1000) == 15 && cart_ppu_read(0x1400) == 18);
    CHECK(cart_ppu_read(0x1800) == 21 && cart_ppu_read(0x1C00) == 24);

    cart_cpu_write(0x7EF6, 3);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    CHECK(cart_ppu_read(0x0000) == 15 && cart_ppu_read(0x0400) == 18);
    CHECK(cart_ppu_read(0x0800) == 21 && cart_ppu_read(0x0C00) == 24);
    CHECK(cart_ppu_read(0x1000) == 8 && cart_ppu_read(0x1400) == 9);
    CHECK(cart_ppu_read(0x1800) == 12 && cart_ppu_read(0x1C00) == 13);

    CHECK(cart_cpu_read(0x6000) == 0);
    cart_cpu_write(0x7EF7, 0xCA);
    cart_cpu_write(0x6000, 0x35);
    cart_cpu_write(0x6400, 0x53);
    CHECK(cart_cpu_read(0x6000) == 0x35 && cart_cpu_read(0x6400) == 0x53);
    CHECK(cart_cpu_read(0x6800) == 0);
    cart_cpu_write(0x7EF8, 0x69);
    cart_cpu_write(0x6800, 0xA6);
    cart_cpu_write(0x6C00, 0x7A);
    CHECK(cart_cpu_read(0x6800) == 0xA6 && cart_cpu_read(0x6C00) == 0x7A);
    CHECK(cart_cpu_read(0x7000) == 0);
    cart_cpu_write(0x7EF9, 0x84);
    cart_cpu_write(0x7000, 0x96);
    CHECK(cart_cpu_read(0x7000) == 0x96);
    cart_cpu_write(0x7400, 0x35);
    CHECK(cart_cpu_read(0x7400) == 0x35);

    cart_cpu_write(0x7EF8, 0x68);
    CHECK(cart_cpu_read(0x6800) == 0xA6 && cart_cpu_read(0x6C00) == 0x7A);
    cart_cpu_write(0x7EF8, 0x69);
    CHECK(cart_cpu_read(0x6800) == 0xA6 && cart_cpu_read(0x6C00) == 0x7A);

    cart->reset();
    CHECK(cart_cpu_read(0x6000) == 0x35 && cart_cpu_read(0x6800) == 0xA6);
    CHECK(cart_cpu_read(0x7000) == 0x96 && cart_cpu_read(0x7400) == 0x35);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xA000) == 5);
    CHECK(cart_cpu_read(0xC000) == 7 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_ppu_read(0x0123) == 15);
    return 0;
}

static int test_taito_x1_persistence_and_loader(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);

    iNESHeader h80 = header_for(80, 0x40000, false);
    h80.flags6 |= 2;
    CHECK(fixture_with_header(&h80, 0x40000, 0x40000) == 80);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0x7EF8, 0xA3);
    cart_cpu_write(0x7F00, 0xA6);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x100);
    CHECK(saved_byte(paths.prg_save, 0) == 0xA6 && saved_byte(paths.prg_save, 0x80) == 0xA6);
    CHECK(fixture_with_header(&h80, 0x40000, 0x40000) == 80);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0x7EF8, 0xA3);
    CHECK(cart_cpu_read(0x7F00) == 0xA6 && cart_cpu_read(0x7F80) == 0xA6);

    cart_battery_shutdown();
    CHECK(remove(paths.prg_save) == 0);
    iNESHeader oversized80 = header_for(80, 0x40000, false);
    oversized80.flags7 |= 8;
    oversized80.flags6 |= 2;
    oversized80.flags10 = 0x97; /* 8 KiB work plus declared 32 KiB save RAM. */
    CHECK(fixture_with_header(&oversized80, 0x40000, 0x40000) == 80);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0x7EF8, 0xA3);
    cart_cpu_write(0x7F00, 0x5C);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x100);
    CHECK(saved_byte(paths.prg_save, 0) == 0x5C && saved_byte(paths.prg_save, 0x80) == 0x5C);
    CHECK(save_fixture_end(&paths) == 0);

    CHECK(save_fixture_begin(&paths) == 0);
    iNESHeader h82 = header_for(82, 0x40000, false);
    h82.flags6 |= 2;
    CHECK(fixture_with_header(&h82, 0x40000, 0x40000) == 82);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0x7EF7, 0xCA);
    cart_cpu_write(0x7EF8, 0x69);
    cart_cpu_write(0x7EF9, 0x84);
    cart_cpu_write(0x6000, 0x35);
    cart_cpu_write(0x6800, 0x53);
    cart_cpu_write(0x7000, 0x96);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x1400);
    CHECK(fixture_with_header(&h82, 0x40000, 0x40000) == 82);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0x7EF7, 0xCA);
    cart_cpu_write(0x7EF8, 0x69);
    cart_cpu_write(0x7EF9, 0x84);
    CHECK(cart_cpu_read(0x6000) == 0x35 && cart_cpu_read(0x6800) == 0x53);
    CHECK(cart_cpu_read(0x7000) == 0x96);

    /* Short saves fill only their prefix. A later write expands the file to
       the mapper's full default 5 KiB save allocation. */
    cart_battery_shutdown();
    CHECK(remove(paths.prg_save) == 0);
    FILE *fp = fopen(paths.prg_save, "wb");
    CHECK(fp != NULL);
    const uint8_t short_save[] = {0xA6, 0x53};
    CHECK(fwrite(short_save, 1, sizeof(short_save), fp) == sizeof(short_save));
    CHECK(fclose(fp) == 0);
    CHECK(fixture_with_header(&h82, 0x40000, 0x40000) == 82);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0x7EF7, 0xCA);
    CHECK(cart_cpu_read(0x6000) == 0xA6 && cart_cpu_read(0x6001) == 0x53);
    CHECK(cart_cpu_read(0x6002) == 0);
    cart_cpu_write(0x6002, 0x71);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x1400);
    CHECK(saved_byte(paths.prg_save, 0) == 0xA6 && saved_byte(paths.prg_save, 2) == 0x71);

    /* An oversized NES 2.0 save allocation keeps bytes beyond the CPU window
       for persistence. Register writes at $7EF8 still target the mapper while
       reads at the same address come from the underlying save mapping. CHR
       sidecars remain separate when CHR ROM is present. */
    cart_battery_shutdown();
    CHECK(remove(paths.prg_save) == 0);
    uint8_t large_save[0x8000];
    memset(large_save, 0, sizeof(large_save));
    large_save[0x1EF8] = 0xE7;
    large_save[0x3000] = 0xC3;
    fp = fopen(paths.prg_save, "wb");
    CHECK(fp != NULL);
    CHECK(fwrite(large_save, 1, sizeof(large_save), fp) == sizeof(large_save));
    CHECK(fclose(fp) == 0);
    uint8_t chr_sidecar[0x2000];
    memset(chr_sidecar, 0x6B, sizeof(chr_sidecar));
    fp = fopen(paths.chr_save, "wb");
    CHECK(fp != NULL);
    CHECK(fwrite(chr_sidecar, 1, sizeof(chr_sidecar), fp) == sizeof(chr_sidecar));
    CHECK(fclose(fp) == 0);
    iNESHeader oversized82 = header_for(82, 0x40000, false);
    oversized82.flags7 |= 8;
    oversized82.flags6 |= 2;
    oversized82.flags10 = 0x90; /* 32 KiB save RAM. */
    oversized82.zero[0] = 0x77; /* 8 KiB CHR RAM plus 8 KiB CHR NVRAM. */
    CHECK(fixture_with_header(&oversized82, 0x40000, 0x40000) == 82);
    cart_battery_configure(paths.rom, true);
    CHECK(cart_cpu_read(0x7EF8) == 0xE7);
    cart_cpu_write(0x7EF8, 0x69);
    CHECK(cart_cpu_read(0x7EF8) == 0xE7);
    cart_cpu_write(0x6800, 0xD4);
    CHECK(cart_cpu_read(0x6800) == 0xD4);
    cart_cpu_write(0x7EF0, 3);
    CHECK(cart_ppu_read(0x0123) == 2);
    cart_ppu_write(0x0123, 0x91);
    CHECK(cart_ppu_read(0x0123) == 2);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x8000);
    CHECK(saved_byte(paths.prg_save, 0x1EF8) == 0xE7);
    CHECK(saved_byte(paths.prg_save, 0x3000) == 0xC3);
    CHECK(saved_file_size(paths.chr_save) == 0x2000);
    CHECK(saved_byte(paths.chr_save, 0) == 0x6B);
    CHECK(save_fixture_end(&paths) == 0);

    CHECK(fixture(207, 0x40000, 0x40000, false) == 207);
    cart_cpu_write(0x7EFA, 3);
    Mapper *previous = cart;
    iNESHeader invalid = header_for(207, 0x40000, false);
    invalid.flags7 |= 8;
    invalid.flags10 = 7;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, NULL, 0) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);

    iNESHeader valid80 = header_for(80, 0x40000, false);
    valid80.flags7 |= 8;
    valid80.flags10 = 2;
    CHECK(fixture_with_header(&valid80, 0x40000, 0x40000) == 80);
    cart_cpu_write(0x6000, 0x42);
    CHECK(cart_cpu_read(0x6000) == 0x42);
    cart_cpu_write(0x7EF8, 0xA3);
    CHECK(cart_cpu_read(0x7F00) == 0x42 && cart_cpu_read(0x7F80) == 0);
    cart_cpu_write(0x7F00, 0x54);
    CHECK(cart_cpu_read(0x7F00) == 0x54 && cart_cpu_read(0x7F80) == 0x54);
    iNESHeader valid82 = header_for(82, 0x40000, false);
    valid82.flags7 |= 8;
    valid82.flags10 = 0;
    CHECK(fixture_with_header(&valid82, 0x40000, 0x40000) == 82);
    cart_cpu_write(0x7EF7, 0xCA);
    cart_cpu_write(0x7EF8, 0x69);
    cart_cpu_write(0x7EF9, 0x84);
    CHECK(cart_cpu_read_bus(0x6000, 0xA1) == 0xA1);
    CHECK(cart_cpu_read_bus(0x7000, 0xB2) == 0xB2);
    CHECK(cart_cpu_read_bus(0x7400, 0xC3) == 0xC3);

    iNESHeader small82 = header_for(82, 0x40000, false);
    small82.flags7 |= 8;
    small82.flags6 |= 2;
    small82.flags10 = 0x25; /* 2 KiB work RAM plus a 256-byte save chip. */
    CHECK(fixture_with_header(&small82, 0x40000, 0x40000) == 82);
    CHECK(cart_cpu_read_bus(0x6000, 0xD4) == 0xD4);
    cart_cpu_write(0x7EF7, 0xCA);
    cart_cpu_write(0x6000, 0x51);
    CHECK(cart_cpu_read(0x6000) == 0x51 && cart_cpu_read(0x6100) == 0x51);
    CHECK(cart_cpu_read(0x6400) == 0x51);
    cart_cpu_write(0x7400, 0x62);
    CHECK(cart_cpu_read(0x7400) == 0x62);

    iNESHeader valid207 = header_for(207, 0x40000, false);
    valid207.flags7 |= 8;
    valid207.flags10 = 2;
    CHECK(fixture_with_header(&valid207, 0x40000, 0x40000) == 207);
    return 0;
}

static int test_irem97_prg_and_mirroring(void) {
    CHECK(fixture(97, 0x40000, 0x2000, false) == 97);
    CHECK(cart != NULL && cart->clock == NULL && cart->reset == NULL);
    CHECK(cart_cpu_read(0x8000) == 30 && cart_cpu_read(0xA000) == 31);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0);
    cart_cpu_write(0x6000, 0xA6);
    cart_cpu_write(0x7FFF, 0x69);
    CHECK(cart_cpu_read(0x6000) == 0xA6 && cart_cpu_read(0x7FFF) == 0x69);
    CHECK(cart_cpu_read(0x8000) == 30 && cart_cpu_read(0xC000) == 30);
    CHECK(cart_ppu_read(0x0000) == 0 && cart_ppu_read(0x1C00) == 7);

    cart_cpu_write(0x8000, 0x03);
    CHECK(cart_cpu_read(0x8000) == 30 && cart_cpu_read(0xC000) == 6);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0x9FFF, 0x43);
    CHECK(cart_cpu_read(0xC000) == 6 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    cart_cpu_write(0xC123, 0x83);
    CHECK(cart_cpu_read(0xC000) == 6 && cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0xFFFF, 0xC3);
    CHECK(cart_cpu_read(0x8000) == 30 && cart_cpu_read(0xC000) == 6);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);

    iNESHeader ram = header_for(97, 0x40000, true);
    CHECK(fixture_with_header(&ram, 0x40000, 0x2000) == 97);
    cart_ppu_write(0x0123, 0xA6);
    CHECK(cart_ppu_read(0x0123) == 0xA6);
    cart_cpu_write(0x8000, 5);
    CHECK(cart_ppu_read(0x0123) == 0xA6 && cart_cpu_read(0xC000) == 10);
    return 0;
}

static int test_irem97_loader_validation(void) {
    iNESHeader h97 = header_for(97, 0x4000, false);
    size_t size;
    uint8_t *image = image_for(&h97, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(rom_mapper_number(&ines_header) == 97);
    CHECK(cart_cpu_read(0x8000) == 0x5C && cart_cpu_read(0xC000) == 0x5C);
    unload_rom();

    CHECK(fixture(97, 0x40000, 0x2000, false) == 97);
    cart_cpu_write(0x8000, 3);
    iNESHeader invalid97 = header_for(97, 0x40000, false);
    fixture_chr[0x0123] = 0x47;
    fixture_chr[0x2123] = 0x8B;
    CHECK(mapper_init_from_header(&invalid97, fixture_prg, 0x40000, fixture_chr, 0x4000) == 97);
    CHECK(cart_ppu_read(0x0123) == 0x47);
    cart_cpu_write(0x8000, 3);
    CHECK(cart_cpu_read(0xC000) == 6 && cart_ppu_read(0x0123) == 0x47);
    return 0;
}

static int test_irem97_cpu_ram_layouts(void) {
    const unsigned boards[] = {97};
    const uint8_t program[] = {
        0xA9, 0x35, 0x8D, 0x00, 0x60, 0xAD, 0x00, 0x60,
        0xA9, 0xA6, 0x8D, 0xFF, 0x7F, 0xAD, 0xFF, 0x7F,
        0xA9, 0x53, 0x8D, 0x00, 0x68, 0xAD, 0x00, 0x60,
        0xAD, 0x00, 0x68, 0xAD, 0xFF, 0x5F
    };
    for (size_t board = 0; board < sizeof(boards) / sizeof(boards[0]); ++board) {
        for (unsigned layout = 0; layout < 4; ++layout) {
            iNESHeader h = header_for(boards[board], 0x8000, false);
            bool has_ram = layout != 1;
            bool mirrored_ram = layout == 3;
            if (layout) {
                h.flags7 |= 8;
                h.flags10 = layout == 2 ? 7 : layout == 3 ? 5 : 0;
            }
            size_t size;
            uint8_t *image = image_for(&h, 0x8000, 0x2000, &size);
            CHECK(image != NULL);
            memcpy(image + sizeof(h) + 0x4100, program, sizeof(program));
            image[sizeof(h) + 0x7FFC] = 0x00;
            image[sizeof(h) + 0x7FFD] = 0xC1;
            int loaded = load_rom_memory(image, size);
            free(image);
            CHECK(loaded == 0 && rom_mapper_number(&ines_header) == (int)boards[board]);
            nes_set_region(NES_REGION_NTSC);
            ppu_power_on(&ppu);
            apu_power_on(&apu);
            CHECK(cpu_power_on(&cpu) && cpu.pc == 0xC100);
            CHECK(cpu_step(&cpu) == 2);
            CHECK(cpu_step(&cpu) == 4);
            CHECK(cpu_step(&cpu) == 4 && cpu.a == (has_ram ? 0x35 : 0x60));
            CHECK(cpu_step(&cpu) == 2);
            CHECK(cpu_step(&cpu) == 4);
            CHECK(cpu_step(&cpu) == 4 && cpu.a == (has_ram ? 0xA6 : 0x7F));
            CHECK(cpu_step(&cpu) == 2);
            CHECK(cpu_step(&cpu) == 4);
            CHECK(cpu_step(&cpu) == 4 && cpu.a ==
                  (has_ram ? (mirrored_ram ? 0x53 : 0x35) : 0x60));
            CHECK(cpu_step(&cpu) == 4 && cpu.a == (has_ram ? 0x53 : 0x68));
            CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x5F);
            ppu_soft_reset(&ppu);
            apu_soft_reset(&apu);
            cpu_soft_reset(&cpu);
            CHECK(cpu.pc == 0xC100);
            CHECK(cart_cpu_read_bus(0x6000, 0x96) ==
                  (has_ram ? (mirrored_ram ? 0x53 : 0x35) : 0x96));
            CHECK(cart_cpu_read_bus(0x7FFF, 0x69) == (has_ram ? 0xA6 : 0x69));
            CHECK(unload_rom());
        }
    }
    return 0;
}

static int irem97_persistence_cases(const SaveFixture *paths) {
    const unsigned boards[] = {97};
    for (size_t board = 0; board < sizeof(boards) / sizeof(boards[0]); ++board) {
        for (unsigned nes2 = 0; nes2 < 2; ++nes2) {
            iNESHeader h = header_for(boards[board], 0x8000, false);
            h.flags6 |= 2;
            if (nes2) {
                h.flags7 |= 8;
                h.flags10 = 0x70;
            }
            size_t size;
            uint8_t *image = image_for(&h, 0x8000, 0x2000, &size);
            CHECK(image != NULL);
            FILE *fp = fopen(paths->rom, "wb");
            CHECK(fp != NULL);
            size_t written = fwrite(image, 1, size, fp);
            int closed = fclose(fp);
            free(image);
            CHECK(written == size && closed == 0);
            CHECK(load_rom(paths->rom) == 0);
            write_mem(0x6000, (uint8_t)(0x35 + board + nes2));
            write_mem(0x7FFF, (uint8_t)(0xA6 - board - nes2));
            CHECK(unload_rom());
            CHECK(saved_file_size(paths->prg_save) == 0x2000);
            CHECK(saved_byte(paths->prg_save, 0) == (int)(0x35 + board + nes2));
            CHECK(saved_byte(paths->prg_save, 0x1FFF) == (int)(0xA6 - board - nes2));
            CHECK(load_rom(paths->rom) == 0);
            CHECK(read_mem(0x6000) == 0x35 + board + nes2);
            CHECK(read_mem(0x7FFF) == 0xA6 - board - nes2);
            CHECK(unload_rom());
        }
    }
    return 0;
}

static int test_irem97_ram_persistence(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    int result = irem97_persistence_cases(&paths);
    unload_rom();
    return result | save_fixture_end(&paths);
}

static int test_jaleco72_92_latches_cpu_and_bus_conflicts(void) {
    const unsigned mappers[] = {72, 92};
    const size_t prg_sizes[] = {0x20000, 0x40000};

    for (size_t board = 0; board < 2; ++board) {
        unsigned mapper = mappers[board];
        size_t prg_bytes = prg_sizes[board];
        CHECK(fixture(mapper, prg_bytes, 0x20000, false) == (int)mapper);
        CHECK(cart != NULL && cart->clock == NULL && cart->reset != NULL);
        CHECK(cart_ppu_read(0) == 0);
        CHECK(cart_cpu_read(mapper == 72 ? 0x8001 : 0xC001)
              == (mapper == 72 ? 0 : 30));

        nes_set_region(NES_REGION_NTSC);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        cpu_power_on(&cpu);

        fixture_prg[0] = 0x7F;
        const uint8_t conflict_program[] = {
            0xA9, 0x85,             // LDA #$85
            0x8D, 0x00, 0x80        // STA $8000; ROM clears D7.
        };
        for (size_t i = 0; i < sizeof(conflict_program); ++i)
            write_mem((uint16_t)(0x0200 + i), conflict_program[i]);
        cpu.pc = 0x0200;
        CHECK(cpu_step(&cpu) == 2);
        CHECK(cpu_step(&cpu) == 4);
        CHECK(cart_cpu_read(mapper == 72 ? 0x8001 : 0xC001)
              == (mapper == 72 ? 0 : 30));

        fixture_prg[0] = 0xFF;
        fixture_prg[3 * 0x4000] = 0xFF;
        fixture_prg[5 * 0x4000] = 0xFF;
        const uint8_t latch_program[] = {
            0xA9, 0x83, 0x8D, 0x00, 0x80,
            0xA9, 0x87, 0x8D, 0x00, 0x80,
            0xA9, 0x03, 0x8D, 0x00, 0x80,
            0xA9, 0x85, 0x8D, 0x00, 0x80,
            0xA9, 0x44, 0x8D, 0x00, 0x80,
            0xA9, 0x47, 0x8D, 0x00, 0x80,
            0xA9, 0x07, 0x8D, 0x00, 0x80,
            0xA9, 0x42, 0x8D, 0x00, 0x80
        };
        for (size_t i = 0; i < sizeof(latch_program); ++i)
            write_mem((uint16_t)(0x0300 + i), latch_program[i]);
        cpu.pc = 0x0300;

        CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
        CHECK(cart_cpu_read(mapper == 72 ? 0x8001 : 0xC001) == 6);
        CHECK(cart_cpu_read(mapper == 72 ? 0xC001 : 0x8001)
              == (mapper == 72 ? 14 : 0));
        CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
        CHECK(cart_cpu_read(mapper == 72 ? 0x8001 : 0xC001) == 6);
        CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
        CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
        CHECK(cart_cpu_read(mapper == 72 ? 0x8001 : 0xC001) == 10);

        CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
        CHECK(cart_ppu_read(0) == 32);
        CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
        CHECK(cart_ppu_read(0) == 32);
        CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
        CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
        CHECK(cart_ppu_read(0) == 16);

        cart->reset();
        CHECK(cart_ppu_read(0) == 0);
        CHECK(cart_cpu_read(mapper == 72 ? 0x8001 : 0xC001)
              == (mapper == 72 ? 0 : 30));
    }
    return 0;
}

static int test_jaleco78_banking_and_submapper_mirroring(void) {
    CHECK(fixture(78, 0x20000, 0x20000, false) == 78);
    fixture_prg[0] = 0xFF;
    cart_cpu_write(0x8000, 0x5B);
    CHECK(cart_cpu_read(0x8001) == 6 && cart_ppu_read(0) == 40);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    fixture_prg[3 * 0x4000] = 0xFF;
    cart_cpu_write(0x8000, 0x23);
    CHECK(cart_cpu_read(0x8001) == 6 && cart_ppu_read(0) == 16);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    cart->reset();
    CHECK(cart_cpu_read(0x8001) == 0 && cart_ppu_read(0) == 0);

    iNESHeader h = header_for(78, 0x20000, false);
    h.flags7 |= 8;
    h.prg_ram_size = 0x10;
    CHECK(fixture_with_header(&h, 0x20000, 0x20000) == 78);
    fixture_prg[0] = 0xFF;
    cart_cpu_write(0x8000, 0x5B);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);

    h.prg_ram_size = 0x30;
    CHECK(fixture_with_header(&h, 0x20000, 0x20000) == 78);
    fixture_prg[0] = 0xFF;
    cart_cpu_write(0x8000, 0x5B);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    fixture_prg[3 * 0x4000] = 0xFF;
    cart_cpu_write(0x8000, 0x53);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    return 0;
}

static int test_jaleco87_101_140_banks_and_register_ranges(void) {
    CHECK(fixture(87, 0x8000, 0x8000, false) == 87);
    cart_cpu_write(0x5FFF, 3);
    CHECK(cart_ppu_read(0) == 0);
    cart_cpu_write(0x6000, 1);
    CHECK(cart_ppu_read(0) == 16);
    cart_cpu_write(0x7FFF, 2);
    CHECK(cart_ppu_read(0) == 8);
    cart_cpu_write(0x8000, 3);
    CHECK(cart_ppu_read(0) == 8);
    cart->reset();
    CHECK(cart_ppu_read(0) == 0);

    CHECK(fixture(101, 0x8000, 0x80000, false) == 101);
    cart_cpu_write(0x5FFF, 5);
    CHECK(cart_ppu_read(0) == 0);
    cart_cpu_write(0x6000, 5);
    CHECK(cart_ppu_read(0) == 40);
    cart_cpu_write(0x7FFF, 0x3F);
    CHECK(cart_ppu_read(0) == 0xF8);
    cart_cpu_write(0x8000, 1);
    CHECK(cart_ppu_read(0) == 0xF8);
    cart->reset();
    CHECK(cart_ppu_read(0) == 0);

    CHECK(fixture(140, 0x20000, 0x20000, false) == 140);
    cart_cpu_write(0x6000, 0x23);
    CHECK(cart_cpu_read(0x8001) == 8 && cart_ppu_read(0) == 24);
    cart_cpu_write(0x7FFF, 0x13);
    CHECK(cart_cpu_read(0x8001) == 4 && cart_ppu_read(0) == 24);
    cart_cpu_write(0x6000, 0x15);
    CHECK(cart_cpu_read(0x8001) == 4 && cart_ppu_read(0) == 40);
    cart_cpu_write(0x8000, 0x32);
    CHECK(cart_cpu_read(0x8001) == 4 && cart_ppu_read(0) == 40);
    cart->reset();
    CHECK(cart_cpu_read(0x8001) == 0 && cart_ppu_read(0) == 0);
    return 0;
}

static int jaleco_cpu_store_load(uint16_t addr, uint8_t value, uint8_t *loaded) {
    const uint8_t program[] = {
        0xA9, value,
        0x8D, (uint8_t)addr, (uint8_t)(addr >> 8),
        0xA9, 0xFF,
        0xAD, (uint8_t)addr, (uint8_t)(addr >> 8)
    };
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    for (size_t i = 0; i < sizeof(program); ++i)
        write_mem((uint16_t)(0x0200u + i), program[i]);
    cpu.pc = 0x0200;
    CHECK(cpu_step(&cpu) == 2);
    CHECK(cpu_step(&cpu) == 4);
    CHECK(cpu_step(&cpu) == 2);
    CHECK(cpu_step(&cpu) == 4);
    *loaded = cpu.a;
    return 0;
}

static int jaleco_cpu_load(uint16_t addr, uint8_t *loaded) {
    const uint8_t program[] = {
        0xAD, (uint8_t)addr, (uint8_t)(addr >> 8)
    };
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    for (size_t i = 0; i < sizeof(program); ++i)
        write_mem((uint16_t)(0x0200u + i), program[i]);
    cpu.pc = 0x0200;
    CHECK(cpu_step(&cpu) == 4);
    *loaded = cpu.a;
    return 0;
}

static int test_jaleco_discrete_ram_cpu_paths(void) {
    static const struct {
        unsigned mapper;
        size_t prg_bytes;
        size_t chr_bytes;
        bool register_window;
        uint8_t register_value;
        uint8_t expected_chr;
        uint8_t expected_prg;
    } cases[] = {
        {72,  0x20000, 0x20000, false, 0xA6, 0, 0},
        {78,  0x20000, 0x20000, false, 0xA6, 0, 0},
        {87,  0x08000, 0x08000, true,  0x01, 16, 0},
        {92,  0x40000, 0x20000, false, 0xA6, 0, 0},
        {101, 0x08000, 0x80000, true,  0x05, 40, 0},
        {140, 0x20000, 0x20000, true,  0x23, 24, 8}
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        const unsigned mapper = cases[i].mapper;
        uint8_t loaded = 0xFF;

        // Legacy headers inherit the board's conventional 8 KiB work RAM.
        CHECK(fixture(mapper, cases[i].prg_bytes, cases[i].chr_bytes, false) == (int)mapper);
        CHECK(jaleco_cpu_store_load(0x6123, cases[i].register_value, &loaded) == 0);
        CHECK(loaded == (cases[i].register_window ? 0 : cases[i].register_value));
        if (cases[i].register_window) {
            CHECK(cart_ppu_read(0) == cases[i].expected_chr);
            if (mapper == 140) CHECK(cart_cpu_read(0x8001) == cases[i].expected_prg);
        }
        cart->reset();
        CHECK(jaleco_cpu_load(0x6123, &loaded) == 0);
        CHECK(loaded == (cases[i].register_window ? 0 : cases[i].register_value));
        if (cases[i].register_window) {
            CHECK(cart_ppu_read(0) == 0);
            if (mapper == 140) CHECK(cart_cpu_read(0x8001) == 0);
        }

        // NES 2.0 can explicitly remove PRG RAM; the CPU then sees its open bus.
        iNESHeader h = header_for(mapper, cases[i].prg_bytes, false);
        h.flags7 |= 0x08;
        h.flags10 = 0;
        CHECK(fixture_with_header(&h, cases[i].prg_bytes, cases[i].chr_bytes) == (int)mapper);
        CHECK(jaleco_cpu_store_load(0x6123, cases[i].register_value, &loaded) == 0);
        CHECK(loaded == 0x61);
        if (cases[i].register_window) {
            CHECK(cart_ppu_read(0) == cases[i].expected_chr);
            if (mapper == 140) CHECK(cart_cpu_read(0x8001) == cases[i].expected_prg);
        }

        // Declared 8 KiB work RAM restores reads. Only 72/78/92 write that RAM;
        // 87/101/140 keep the same addresses as write-only mapper registers.
        h.flags10 = 7;
        CHECK(fixture_with_header(&h, cases[i].prg_bytes, cases[i].chr_bytes) == (int)mapper);
        CHECK(jaleco_cpu_store_load(0x6123, cases[i].register_value, &loaded) == 0);
        CHECK(loaded == (cases[i].register_window ? 0 : cases[i].register_value));
        if (cases[i].register_window) {
            CHECK(cart_ppu_read(0) == cases[i].expected_chr);
            if (mapper == 140) CHECK(cart_cpu_read(0x8001) == cases[i].expected_prg);
        }
        cart->reset();
        CHECK(jaleco_cpu_load(0x6123, &loaded) == 0);
        CHECK(loaded == (cases[i].register_window ? 0 : cases[i].register_value));
        if (cases[i].register_window) {
            CHECK(cart_ppu_read(0) == 0);
            if (mapper == 140) CHECK(cart_cpu_read(0x8001) == 0);
        }
    }
    return 0;
}

static int test_jaleco_intercepted_ram_reads_and_soft_reset(void) {
    static const struct {
        unsigned mapper;
        size_t prg_bytes;
        size_t chr_bytes;
        uint8_t register_value;
        uint8_t expected_chr_bank;
        uint8_t expected_prg_bank;
    } cases[] = {
        {87,  0x08000, 0x08000, 0x01, 2, 0},
        {101, 0x08000, 0x80000, 0x05, 5, 0},
        {140, 0x20000, 0x20000, 0x23, 3, 2}
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        for (unsigned nes2 = 0; nes2 < 2; ++nes2) {
            iNESHeader h = header_for(cases[i].mapper, cases[i].prg_bytes, false);
            h.flags6 |= 0x04; // Trainer seeds work RAM before the CPU starts.
            h.chr_rom_chunks = (uint8_t)(cases[i].chr_bytes / 0x2000u);
            if (nes2) {
                h.flags7 |= 0x08;
                h.flags10 = 7;
            }

            size_t image_size;
            uint8_t *image = image_for(&h, cases[i].prg_bytes, cases[i].chr_bytes, &image_size);
            CHECK(image != NULL);
            const size_t trainer_offset = sizeof(h);
            const size_t prg_offset = trainer_offset + 512;
            const size_t chr_offset = prg_offset + cases[i].prg_bytes;
            const uint8_t ram_value = (uint8_t)(0xA7u + nes2);
            image[trainer_offset + 0x123] = ram_value;
            for (size_t bank = 0; bank < cases[i].chr_bytes / 0x2000u; ++bank)
                memset(image + chr_offset + bank * 0x2000u, (int)bank, 0x2000u);
            if (cases[i].mapper == 140) {
                for (size_t bank = 0; bank < cases[i].prg_bytes / 0x8000u; ++bank)
                    memset(image + prg_offset + bank * 0x8000u, (int)(0x40u + bank), 0x8000u);
            }
            CHECK(load_rom_memory(image, image_size) == 0);
            free(image);

            nes_set_region(NES_REGION_NTSC);
            ppu_power_on(&ppu);
            apu_power_on(&apu);
            CHECK(cpu_power_on(&cpu));

            const uint8_t program[] = {
                0xA9, cases[i].register_value,
                0x8D, 0x23, 0x71,
                0xAD, 0x23, 0x71
            };
            for (size_t j = 0; j < sizeof(program); ++j)
                write_mem((uint16_t)(0x0200u + j), program[j]);
            cpu.pc = 0x0200;
            CHECK(cpu_step(&cpu) == 2);
            CHECK(cpu_step(&cpu) == 4);
            CHECK(cpu_step(&cpu) == 4 && cpu.a == ram_value);
            CHECK(cart_ppu_read(0) == cases[i].expected_chr_bank);
            if (cases[i].mapper == 140)
                CHECK(cart_cpu_read(0x8000) == (uint8_t)(0x40u + cases[i].expected_prg_bank));

            // A console soft reset leaves both the mapper latch and work RAM live.
            cpu_soft_reset(&cpu);
            CHECK(cart_ppu_read(0) == cases[i].expected_chr_bank);
            if (cases[i].mapper == 140)
                CHECK(cart_cpu_read(0x8000) == (uint8_t)(0x40u + cases[i].expected_prg_bank));
            write_mem(0x0300, 0xAD);
            write_mem(0x0301, 0x23);
            write_mem(0x0302, 0x71);
            cpu.pc = 0x0300;
            CHECK(cpu_step(&cpu) == 4 && cpu.a == ram_value);

            // The mapper's own reset clears its latch but still does not clear RAM.
            cart->reset();
            CHECK(cart_ppu_read(0) == 0);
            if (cases[i].mapper == 140) CHECK(cart_cpu_read(0x8000) == 0x40);
            cpu.pc = 0x0300;
            CHECK(cpu_step(&cpu) == 4 && cpu.a == ram_value);
        }
    }
    return 0;
}

static int test_jaleco_discrete_loader_validation(void) {
    const unsigned mappers[] = {72, 78, 87, 92, 101, 140};
    for (size_t i = 0; i < sizeof(mappers) / sizeof(mappers[0]); ++i) {
        iNESHeader h = header_for(mappers[i], 0x8000, false);
        size_t size;
        uint8_t *image = image_for(&h, 0x8000, 0x2000, &size);
        CHECK(image != NULL);
        CHECK(load_rom_memory(image, size) == 0);
        free(image);
        CHECK(rom_mapper_number(&ines_header) == (int)mappers[i]);
        CHECK(cart != NULL && cart->clock == NULL);
        unload_rom();
    }

    const unsigned mirrored[] = {87, 101, 140};
    for (size_t i = 0; i < sizeof(mirrored) / sizeof(mirrored[0]); ++i) {
        iNESHeader h = header_for(mirrored[i], 0x4000, false);
        size_t size;
        uint8_t *image = image_for(&h, 0x4000, 0x2000, &size);
        CHECK(image != NULL);
        CHECK(load_rom_memory(image, size) == 0);
        free(image);
        CHECK(cart_cpu_read(0x8000) == 0x5C && cart_cpu_read(0xC000) == 0x5C);
        cart_cpu_write(0x6000, mirrored[i] == 140 ? 0x31 : 3);
        CHECK(cart_cpu_read(0x8000) == 0x5C && cart_cpu_read(0xC000) == 0x5C);
        unload_rom();
    }

    CHECK(fixture(18, 0x20000, 0x20000, false) == 18);
    cart_cpu_write(0x8000, 3);
    CHECK(cart_cpu_read(0x8000) == 3);
    Mapper *previous = cart;

    iNESHeader invalid = header_for(78, 0x8000, false);
    invalid.flags7 |= 8;
    invalid.prg_ram_size = 0x20;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x8000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);

    invalid = header_for(72, 0x8000, false);
    invalid.flags7 |= 8;
    invalid.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x8000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);

    return 0;
}

static int test_cnrom185_submapper_latches_and_ppu_bus(void) {
    CHECK(fixture(185, 0x8000, 0x2000, false) == 185);
    CHECK(cart != NULL && cart->clock == NULL && cart->reset != NULL);
    fixture_prg[0] = 0xFF;
    fixture_chr[0x0122] = 0xA6;
    ppu_power_on(&ppu);
    CHECK(ppu_read(0x0122) == 0xA6);

    cart_cpu_write(0x8000, 0x00);
    CHECK(ppu_read(0x0122) == 0x23);
    CHECK(ppu_read(0x01A6) == 0xA7);
    CHECK(ppu_read(0x1FFE) == 0xFF);
    cart_cpu_write(0x8000, 0x01);
    CHECK(ppu_read(0x0122) == 0xA6);
    cart_cpu_write(0x8000, 0x13);
    CHECK(ppu_read(0x0122) == 0x23);
    cart_cpu_write(0x8000, 0x23);
    CHECK(ppu_read(0x0122) == 0xA6);

    for (unsigned submapper = 4; submapper <= 7; ++submapper) {
        iNESHeader h = header_for(185, 0x8000, false);
        h.flags7 |= 8;
        h.prg_ram_size = (uint8_t)(submapper << 4);
        CHECK(fixture_with_header(&h, 0x8000, 0x2000) == 185);
        fixture_prg[0] = 0xFF;
        fixture_chr[0x0122] = 0xA6;
        ppu_power_on(&ppu);

        uint8_t enabled = (uint8_t)(submapper - 4u);
        uint8_t disabled = (uint8_t)((enabled + 1u) & 3u);
        for (unsigned latch = 0; latch <= 0xFF; ++latch) {
            cart_cpu_write(0x8000, (uint8_t)latch);
            CHECK(ppu_read(0x0122) == ((latch & 3u) == enabled ? 0xA6 : 0x23));
        }
        cart_cpu_write(0x8000, enabled);
        CHECK(ppu_read(0x0122) == 0xA6);
        cart_cpu_write(0x8000, disabled);
        CHECK(ppu_read(0x0122) == 0x23);
        cart->reset();
        CHECK(ppu_read(0x0122) == 0xA6);
    }
    return 0;
}

static int test_cnrom185_cpu_bus_conflict_control(void) {
    for (unsigned submapper = 4; submapper <= 7; ++submapper) {
        iNESHeader h = header_for(185, 0x8000, false);
        h.flags7 |= 8;
        h.prg_ram_size = (uint8_t)(submapper << 4);
        CHECK(fixture_with_header(&h, 0x8000, 0x2000) == 185);
        fixture_chr[0x0122] = 0xA6;

        nes_set_region(NES_REGION_NTSC);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        cpu_power_on(&cpu);

        uint8_t enabled = (uint8_t)(submapper - 4u);
        uint8_t disabled = (uint8_t)((enabled + 1u) & 3u);
        uint8_t program[] = {
            0xA9, disabled,
            0x8D, 0x00, 0x80,
            0xA9, 0xFF,
            0x8D, 0x00, 0x80
        };
        for (size_t i = 0; i < sizeof(program); ++i)
            write_mem((uint16_t)(0x0200 + i), program[i]);
        cpu.pc = 0x0200;

        fixture_prg[0] = 0xFF;
        CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
        CHECK(ppu_read(0x0122) == 0x23);

        fixture_prg[0] = enabled;
        CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
        CHECK(ppu_read(0x0122) == 0xA6);

        fixture_prg[0] = 0xFF;
        cart_cpu_write(0x8000, disabled);
        CHECK(ppu_read(0x0122) == 0x23);
        ppu_soft_reset(&ppu);
        apu_soft_reset(&apu);
        cpu_soft_reset(&cpu);
        CHECK(ppu_read(0x0122) == 0x23);
        cart->reset();
        CHECK(ppu_read(0x0122) == 0xA6);
    }
    return 0;
}

static int test_cnrom185_loader_and_cnrom_regression(void) {
    for (size_t prg_bytes = 0x4000; prg_bytes <= 0x8000; prg_bytes += 0x4000) {
        iNESHeader h = header_for(185, prg_bytes, false);
        size_t size;
        uint8_t *image = image_for(&h, prg_bytes, 0x2000, &size);
        CHECK(image != NULL);
        CHECK(load_rom_memory(image, size) == 0);
        free(image);
        CHECK(rom_mapper_number(&ines_header) == 185);
        CHECK(cart_cpu_read(0x8000) == 0x5C && cart_cpu_read(0xC000) == 0x5C);
        ppu_power_on(&ppu);
        CHECK(ppu_read(0) == 0xA5);
        unload_rom();
    }

    for (unsigned submapper = 4; submapper <= 7; ++submapper) {
        iNESHeader h = header_for(185, 0x8000, false);
        h.flags7 |= 8;
        h.prg_ram_size = (uint8_t)(submapper << 4);
        size_t size;
        uint8_t *image = image_for(&h, 0x8000, 0x2000, &size);
        CHECK(image != NULL);
        CHECK(load_rom_memory(image, size) == 0);
        free(image);
        ppu_power_on(&ppu);
        CHECK(rom_mapper_number(&ines_header) == 185 && ppu_read(0) == 0xA5);
        unload_rom();
    }

    // CHR-RAM-only protected CNROM uses the same protection latch. Enabled
    // reads and writes reach RAM; disabled pattern-table accesses stay open bus.
    {
        iNESHeader ram_only = header_for(185, 0x8000, true);
        size_t size;
        uint8_t *image = image_for(&ram_only, 0x8000, 0, &size);
        CHECK(image != NULL);
        image[sizeof(ram_only)] = 0xFF;
        CHECK(load_rom_memory(image, size) == 0);
        ppu_power_on(&ppu);
        ppu_write(0x0122, 0xA6);
        CHECK(ppu_read(0x0122) == 0xA6);
        cart_cpu_write(0x8000, 0x00);
        ppu_write(0x0122, 0xB7);
        CHECK(ppu_read(0x0122) == 0x23);
        cart_cpu_write(0x8000, 0x01);
        CHECK(ppu_read(0x0122) == 0xA6);
        ppu_write(0x0122, 0xC8);
        CHECK(ppu_read(0x0122) == 0xC8);
        Mapper *ram_cart = cart;
        uint8_t *ram_prg = prg_rom;
        CHECK(load_rom_memory(image, size - 1) == -1);
        free(image);
        CHECK(cart == ram_cart && prg_rom == ram_prg && ppu_read(0x0122) == 0xC8);
    }

    CHECK(fixture(3, 0x8000, 0x8000, false) == 3);
    cart_cpu_write(0x8000, 2);
    CHECK(cart_ppu_read(0) == 16);
    Mapper *previous = cart;

    for (unsigned submapper = 1; submapper <= 3; ++submapper) {
        iNESHeader invalid = header_for(185, 0x8000, false);
        invalid.flags7 |= 8;
        invalid.prg_ram_size = (uint8_t)(submapper << 4);
        CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x8000, fixture_chr, 0x2000) == -1);
        CHECK(cart == previous && cart_ppu_read(0) == 16);
    }

    return 0;
}

#endif // MAPPER_ACCURACY_SUNSOFT_H

