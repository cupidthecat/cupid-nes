/*
 * mapper_accuracy_native.h - Native mapper regression tests
 *
 * Author: @frankischilling
 *
 * These checks exercise mapper behavior through the cartridge interface used by the
 * emulator core.
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
#ifndef MAPPER_ACCURACY_NATIVE_H
#define MAPPER_ACCURACY_NATIVE_H

static int test_gxrom_banks_reset_and_ram(void) {
    CHECK(fixture(66, 0x20000, 0x8000, false) == 66);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_ppu_read(0x0000) == 0);
    cart_cpu_write(0x8000, 0x31);
    CHECK(cart_cpu_read(0x8000) == 12 && cart_ppu_read(0x0000) == 8);
    cart_cpu_write(0xFFFF, 0x23);
    CHECK(cart_cpu_read(0x8000) == 8 && cart_ppu_read(0x0000) == 24);

    fixture_prg[2 * 0x8000] = 0x00;
    cart_cpu_write(0x8000, 0x11); // No bus conflict: visible ROM zero must not clear register bits.
    CHECK(cart_cpu_read(0x8000) == 4 && cart_ppu_read(0x0000) == 8);

    cart_cpu_write(0x6123, 0xA5);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    uint8_t before = cart_ppu_read(0x0123);
    cart_ppu_write(0x0123, 0x5A);
    CHECK(cart_ppu_read(0x0123) == before);
    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 0 && cart_ppu_read(0x0000) == 0);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    return 0;
}

static int test_gxrom_chr_ram_and_loader(void) {
    CHECK(fixture(66, 0x10000, 0x2000, true) == 66);
    cart_ppu_write(0x0123, 0xA6);
    cart_cpu_write(0x8000, 0x33);
    CHECK(cart_cpu_read(0x8000) == 4);
    CHECK(cart_ppu_read(0x0123) == 0xA6);

    iNESHeader h = header_for(66, 0x20000, false);
    h.chr_rom_chunks = 4;
    size_t size;
    uint8_t *image = image_for(&h, 0x20000, 0x8000, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && rom_mapper_number(&ines_header) == 66);
    cart_cpu_write(0x8000, 0x32);
    CHECK(cart_cpu_read(0x8000) == 0x5C && cart_ppu_read(0x0000) == 0xA5);

    Mapper *previous = cart;
    h.flags7 |= 0x08;
    h.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x8000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0x5C);
    return 0;
}

static int test_mapper71_variants_and_mirroring(void) {
    CHECK(fixture(71, 0x20000, 0x2000, true) == 71);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 14);
    cart_cpu_write(0x8000, 3);
    CHECK(cart_cpu_read(0x8000) == 6);
    cart_cpu_write(0x9000, 0x10);
    CHECK(cart_cpu_read(0x8000) == 6 && cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0xA000, 0);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1 && cart_cpu_read(0x8000) == 6);
    uint8_t nt[0x1000] = {0};
    cart_nt_write(0x2001, 0x5A, nt);
    CHECK(nt[0x401] == 0x5A && cart_nt_read(0x2C01, nt) == 0x5A);
    cart_cpu_write(0xA000, 0x10);
    CHECK(cart_nt_read(0x2001, nt) == 0);
    cart_nt_write(0x2401, 0xB6, nt);
    CHECK(nt[1] == 0xB6 && nt[0x401] == 0x5A);
    cart_cpu_write(0xA000, 0);
    CHECK(cart_nt_read(0x2801, nt) == 0x5A);
    cart_cpu_write(0xC000, 5);
    CHECK(cart_cpu_read(0x8000) == 10 && cart_cpu_read(0xC000) == 14);
    cart_cpu_write(0x6123, 0xA5);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    cart_ppu_write(0x0123, 0x5A);
    CHECK(cart_ppu_read(0x0123) == 0x5A);
    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 0 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    cart_cpu_write(0x8000, 2); // Legacy mode returns to ordinary bank decoding on reset.
    CHECK(cart_cpu_read(0x8000) == 4);

    iNESHeader h = header_for(71, 0x20000, true);
    h.flags7 |= 0x08;
    h.prg_ram_size = 0x10;
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 71);
    cart_cpu_write(0x8000, 0x10);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0xB000, 0);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    cart_cpu_write(0xC000, 6);
    CHECK(cart_cpu_read(0x8000) == 12);
    cart->reset();
    cart_cpu_write(0x8000, 0x10);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_get_mirroring() == MIRROR_SINGLE0);
    return 0;
}

static int test_mapper71_loader_and_chr_rom(void) {
    iNESHeader h = header_for(71, 0x20000, false);
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 71);
    uint8_t chr_before = cart_ppu_read(0x0123);
    cart_ppu_write(0x0123, 0xA6);
    CHECK(cart_ppu_read(0x0123) == chr_before);
    cart_cpu_write(0xC000, 2); // No bus conflict on the BF909x bank register.
    CHECK(cart_cpu_read(0x8000) == 4);
    Mapper *previous = cart;

    iNESHeader invalid = h;
    invalid.flags7 |= 0x08;
    invalid.prg_ram_size = 0x20;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x20000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 4);
    fixture_chr[0x0123] = 0x35;
    fixture_chr[0x2123] = 0x69;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x4000) == 71);
    CHECK(cart_ppu_read(0x0123) == 0x35);
    cart_cpu_write(0xC000, 3);
    CHECK(cart_cpu_read(0x8000) == 6 && cart_ppu_read(0x0123) == 0x35);
    return 0;
}

static int test_mapper71_image_loading(void) {
    for (unsigned submapper = 0; submapper <= 1; ++submapper) {
        iNESHeader h = header_for(71, 0x20000, true);
        h.flags7 |= 0x08;
        h.prg_ram_size = (uint8_t)(submapper << 4);
        h.flags10 = 7;
        h.zero[0] = 7;
        size_t size;
        uint8_t *image = image_for(&h, 0x20000, 0, &size);
        CHECK(image != NULL);
        for (size_t i = 0; i < 0x20000; ++i)
            image[sizeof(h) + i] = (uint8_t)(i / 0x4000);
        CHECK(load_rom_memory(image, size) == 0);
        CHECK(rom_mapper_number(&ines_header) == 71 && cart_cpu_read(0xC000) == 7);
        cart_cpu_write(0x8000, 3);
        CHECK(cart_cpu_read(0x8000) == (submapper ? 0 : 3));
        cart_cpu_write(0xC000, 5);
        cart_cpu_write(0x6123, 0xA6);
        cart_ppu_write(0x0123, 0xC7);
        Mapper *previous = cart;
        iNESHeader active = ines_header;
        image[8] = 0x20;
        CHECK(load_rom_memory(image, size) == -1);
        memcpy(image, &h, sizeof(h));
        CHECK(load_rom_memory(image, size - 1) == -1);
        free(image);
        CHECK(cart == previous && memcmp(&ines_header, &active, sizeof(active)) == 0);
        CHECK(cart_cpu_read(0x8000) == 5 && cart_cpu_read(0xC000) == 7);
        CHECK(cart_cpu_read(0x6123) == 0xA6 && cart_ppu_read(0x0123) == 0xC7);
    }
    return 0;
}

static int test_legacy_loader_ram_and_large_prg_metadata(void) {
    const struct {
        unsigned mapper;
        size_t prg_bytes;
        size_t chr_bytes;
        bool chr_ram;
    } ram_cases[] = {
        {34, 0x40000, 0x2000, true},
        {66, 0x20000, 0x2000, false},
        {71, 0x20000, 0x2000, true},
        {206, 0x20000, 0x2000, false},
    };
    for (size_t i = 0; i < sizeof(ram_cases) / sizeof(ram_cases[0]); ++i) {
        iNESHeader h = header_for(ram_cases[i].mapper, ram_cases[i].prg_bytes,
                                  ram_cases[i].chr_ram);
        h.prg_ram_size = 2; // Legacy byte 8 does not override the board's 8 KiB default.
        CHECK(fixture_with_header(&h, ram_cases[i].prg_bytes, ram_cases[i].chr_bytes)
              == (int)ram_cases[i].mapper);
        cart_cpu_write(0x6123, 0xA5);
        cart_cpu_write(0x7FFF, 0x5A);
        CHECK(cart_cpu_read(0x6123) == 0xA5 && cart_cpu_read(0x7FFF) == 0x5A);
    }

    iNESHeader bnrom = header_for(34, 0x400000, true);
    CHECK(bnrom.prg_rom_chunks == 0);
    size_t bnrom_size;
    uint8_t *bnrom_image = image_for(&bnrom, 0x400000, 0, &bnrom_size);
    CHECK(bnrom_image != NULL);
    for (size_t bank = 0; bank < 128; ++bank)
        memset(bnrom_image + sizeof(bnrom) + bank * 0x8000, (uint8_t)bank, 0x8000);
    bnrom_image[sizeof(bnrom) + 0x7FFF] = 0xFF;
    CHECK(load_rom_memory(bnrom_image, bnrom_size) == 0);
    CHECK(cart_cpu_read(0x8000) == 0);
    cart_cpu_write(0xFFFF, 0x7F);
    CHECK(cart_cpu_read(0x8000) == 0x7F);
    Mapper *previous = cart;
    CHECK(load_rom_memory(bnrom_image, bnrom_size - 1) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0x7F);
    free(bnrom_image);

    iNESHeader bf909x = header_for(71, 0x400000, false);
    CHECK(bf909x.prg_rom_chunks == 0);
    size_t bf909x_size;
    uint8_t *bf909x_image = image_for(&bf909x, 0x400000, 0x2000, &bf909x_size);
    CHECK(bf909x_image != NULL);
    for (size_t bank = 0; bank < 256; ++bank)
        memset(bf909x_image + sizeof(bf909x) + bank * 0x4000, (uint8_t)bank, 0x4000);
    CHECK(load_rom_memory(bf909x_image, bf909x_size) == 0);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 0xFF);
    cart_cpu_write(0xC000, 0xFE);
    CHECK(cart_cpu_read(0x8000) == 0xFE && cart_cpu_read(0xC000) == 0xFF);
    previous = cart;
    CHECK(load_rom_memory(bf909x_image, bf909x_size - 1) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0xFE && cart_cpu_read(0xC000) == 0xFF);
    free(bf909x_image);

    iNESHeader nes2_zero = mapper34_header(2, true, true);
    nes2_zero.prg_rom_chunks = 0;
    size_t nes2_zero_size;
    uint8_t *nes2_zero_image = image_for(&nes2_zero, 0, 0, &nes2_zero_size);
    CHECK(nes2_zero_image != NULL);
    previous = cart;
    CHECK(load_rom_memory(nes2_zero_image, nes2_zero_size) == -1);
    free(nes2_zero_image);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0xFE && cart_cpu_read(0xC000) == 0xFF);
    return 0;
}

static void namco108_write_bank_at(uint16_t select_address, uint16_t data_address,
                                   uint8_t reg, uint8_t value) {
    cart_cpu_write(select_address, (uint8_t)(0xF8u | reg));
    cart_cpu_write(data_address, value);
}

static void namco108_write_bank(uint8_t reg, uint8_t value) {
    namco108_write_bank_at(0x9FFEu, 0x9FFFu, reg, value);
}

static int test_namco108_banks_aliases_and_irq_absence(void) {
    CHECK(fixture(206, 0x200000, 0x40000, false) == 206);
    CHECK(cart != NULL && cart->clock == NULL);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 254 && cart_cpu_read(0xE000) == 255);
    CHECK(cart_ppu_read(0x0000) == 0 && cart_ppu_read(0x0400) == 1);
    CHECK(cart_ppu_read(0x0800) == 2 && cart_ppu_read(0x0C00) == 3);
    CHECK(cart_ppu_read(0x1000) == 4 && cart_ppu_read(0x1C00) == 7);

    // Only A0 distinguishes the selector and data ports anywhere in $8000-$9FFF.
    namco108_write_bank_at(0x8ABCu, 0x91FDu, 6, 0xC3);
    namco108_write_bank_at(0x9000u, 0x8001u, 7, 0x85);
    CHECK(cart_cpu_read(0x8000) == 0xC3 && cart_cpu_read(0xA000) == 0x85);
    CHECK(cart_cpu_read(0xC000) == 254 && cart_cpu_read(0xE000) == 255);

    // Only the two paired CHR registers ignore the low data bit.
    namco108_write_bank(0, 0x49);
    namco108_write_bank(1, 0x4D);
    namco108_write_bank(2, 0x85);
    namco108_write_bank(3, 0xA2);
    namco108_write_bank(4, 0xC7);
    namco108_write_bank(5, 0xF8);
    CHECK(cart_ppu_read(0x0000) == 0x48 && cart_ppu_read(0x0400) == 0x49);
    CHECK(cart_ppu_read(0x0800) == 0x4C && cart_ppu_read(0x0C00) == 0x4D);
    CHECK(cart_ppu_read(0x1000) == 0x85 && cart_ppu_read(0x1400) == 0xA2);
    CHECK(cart_ppu_read(0x1800) == 0xC7 && cart_ppu_read(0x1C00) == 0xF8);

    Mirroring mirr = cart_get_mirroring();
    cart_cpu_write(0xA000, 1);
    cart_cpu_write(0xA001, 0);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    for (unsigned i = 0; i < 8; ++i) {
        cart_notify_ppu_address(0x0000, i * 12);
        cart_notify_ppu_address(0x1000, i * 12 + 9);
    }
    CHECK(cart_get_mirroring() == mirr && !cart_irq_pending());

    cart_cpu_write(0x6123, 0xA5);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 254 && cart_cpu_read(0xE000) == 255);
    CHECK(cart_ppu_read(0x0000) == 0 && cart_ppu_read(0x1000) == 4);
    CHECK(cart_cpu_read(0x6123) == 0xA5 && cart_get_mirroring() == mirr);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_namco108_submapper_loader_and_chr_ram(void) {
    iNESHeader h = header_for(206, 0x8000, false);
    h.flags7 |= 0x08;
    h.prg_ram_size = 0x10;
    h.flags6 |= 1;
    CHECK(fixture_with_header(&h, 0x8000, 0x40000) == 206);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 2 && cart_cpu_read(0xE000) == 3);
    namco108_write_bank(6, 3);
    namco108_write_bank(7, 2);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 2 && cart_cpu_read(0xE000) == 3);
    cart_cpu_write(0xA000, 0);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xE000) == 3);

    iNESHeader ram_h = header_for(206, 0x8000, true);
    ram_h.flags7 |= 0x08;
    ram_h.flags10 = 0;
    ram_h.zero[0] = 12;
    CHECK(fixture_with_header(&ram_h, 0x8000, 0x40000) == 206);
    namco108_write_bank(2, 0x85);
    cart_ppu_write(0x1000, 0xA6);
    CHECK(cart_ppu_read(0x1000) == 0xA6);
    namco108_write_bank(2, 5);
    CHECK(cart_ppu_read(0x1000) == 5);
    namco108_write_bank(2, 0x85);
    CHECK(cart_ppu_read(0x1000) == 0xA6);

    iNESHeader load_h = header_for(206, 0x200000, false);
    load_h.chr_rom_chunks = 32;
    size_t size;
    uint8_t *image = image_for(&load_h, 0x200000, 0x40000, &size);
    CHECK(image != NULL);
    for (size_t i = 0; i < 0x200000; ++i)
        image[sizeof(load_h) + i] = (uint8_t)(i / 0x2000);
    for (size_t i = 0; i < 0x40000; ++i)
        image[sizeof(load_h) + 0x200000 + i] = (uint8_t)(i / 0x400);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(rom_mapper_number(&ines_header) == 206);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xE000) == 255);
    namco108_write_bank(6, 0xC3);
    namco108_write_bank(2, 0x85);
    cart_cpu_write(0x6123, 0xA6);
    CHECK(cart_cpu_read(0x8000) == 0xC3 && cart_ppu_read(0x1000) == 0x85);

    Mapper *previous = cart;
    iNESHeader active = ines_header;
    iNESHeader invalid = load_h;
    invalid.flags7 |= 0x08;
    invalid.prg_ram_size = 0x20;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x200000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0xC3);

    invalid = h;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x10000, fixture_chr, 0x40000) == 206);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 2 && cart_cpu_read(0xE000) == 3);
    namco108_write_bank(6, 7);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xE000) == 3);
    CHECK(mapper_init_from_header(&active, prg_rom, 0x200000, chr_rom, 0x40000) == 206);
    namco108_write_bank(6, 0xC3);
    namco108_write_bank(2, 0x85);
    cart_cpu_write(0x6123, 0xA6);
    previous = cart;
    CHECK(cart_cpu_read(0x8000) == 0xC3 && cart_ppu_read(0x1000) == 0x85);
    image = image_for(&load_h, 0x200000, 0x40000, &size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, size - 1) == -1);
    free(image);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0xC3);
    CHECK(memcmp(&ines_header, &active, sizeof(active)) == 0);
    CHECK(cart_ppu_read(0x1000) == 0x85 && cart_cpu_read(0x6123) == 0xA6);
    return 0;
}

static void jaleco18_write_bank(uint16_t reg, uint8_t value) {
    uint16_t alias = (uint16_t)(reg | 0x0FFCu);
    cart_cpu_write(alias, value & 0x0F);
    cart_cpu_write((uint16_t)(alias | 1u), value >> 4);
}

static void jaleco18_write_irq_reload(uint16_t value) {
    for (unsigned nibble = 0; nibble < 4; ++nibble)
        cart_cpu_write((uint16_t)(0xE000u + nibble), (uint8_t)(value >> (nibble * 4)));
}

static int test_irregular_native_page_safety(void) {
    static const unsigned sixteen_kib_boards[] = {10, 16, 67, 68};
    size_t size;

    // A 24 KiB image contains one complete 16 KiB mapper page plus an 8 KiB
    // tail. The tail is not a second bank on MMC4, Bandai FCG, or Sunsoft 3/4.
    for (size_t i = 0; i < sizeof(sixteen_kib_boards) / sizeof(sixteen_kib_boards[0]); ++i) {
        unsigned mapper = sixteen_kib_boards[i];
        iNESHeader h = header_for(mapper, 0x4000, false);
        h.flags7 |= 0x08;
        h.prg_rom_chunks = 0x35; // 3 * 2^13 = 24 KiB.
        h.flags9 = 0x0F;
        uint8_t *image = image_for(&h, 0x6000, 0x2000, &size);
        CHECK(image != NULL);
        uint8_t *prg = image + sizeof(h);
        memset(prg, 0x11, 0x2000);
        memset(prg + 0x2000, 0x22, 0x2000);
        memset(prg + 0x4000, 0x33, 0x2000);
        CHECK(load_rom_memory(image, size) == 0);
        free(image);
        CHECK(cart_cpu_read(0xC000) == 0x11 && cart_cpu_read(0xE000) == 0x22);
        switch (mapper) {
            case 10: cart_cpu_write(0xA000, 7); break;
            case 16: cart_cpu_write(0x8008, 7); break;
            case 67: cart_cpu_write(0xF800, 7); break;
            case 68: cart_cpu_write(0xF000, 7); break;
        }
        CHECK(cart_cpu_read(0xC000) == 0x11 && cart_cpu_read(0xE000) == 0x22);
        CHECK(cart_cpu_read(0x8000) == 0x11);
    }

    // Mapper 18 has 8 KiB pages, so all three pages in the same 24 KiB image
    // are real banks. Register writes must remain visible instead of being
    // hidden behind whole-image repetition.
    iNESHeader h = header_for(18, 0x4000, false);
    h.flags7 |= 0x08;
    h.prg_rom_chunks = 0x35;
    h.flags9 = 0x0F;
    uint8_t *image = image_for(&h, 0x6000, 0x2000, &size);
    CHECK(image != NULL);
    uint8_t *prg = image + sizeof(h);
    memset(prg, 0x41, 0x2000);
    memset(prg + 0x2000, 0x52, 0x2000);
    memset(prg + 0x4000, 0x63, 0x2000);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_cpu_read(0xE000) == 0x63);
    jaleco18_write_bank(0x8000, 1);
    CHECK(cart_cpu_read(0x8000) == 0x52);
    jaleco18_write_bank(0x8000, 2);
    CHECK(cart_cpu_read(0x8000) == 0x63);

    // Mapper 77 has a 32 KiB PRG page. A 24 KiB image is one repeated whole
    // image followed by open bus; its 8 KiB tail is part of that image, not a
    // separately selectable mapper page.
    h = header_for(77, 0x4000, false);
    h.flags7 |= 0x08;
    h.prg_rom_chunks = 0x35;
    h.flags9 = 0x0F;
    h.zero[0] = 7; // The board also carries its 8 KiB CHR RAM beside CHR ROM.
    image = image_for(&h, 0x6000, 0x2000, &size);
    CHECK(image != NULL);
    prg = image + sizeof(h);
    memset(prg, 0xFF, 0x2000);
    memset(prg + 0x2000, 0x72, 0x2000);
    memset(prg + 0x4000, 0x83, 0x2000);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_cpu_read(0x8000) == 0xFF && cart_cpu_read(0xA000) == 0x72);
    CHECK(cart_cpu_read(0xC000) == 0x83 && cart_cpu_read_bus(0xE000, 0xA6) == 0xA6);
    cart_cpu_write(0x8000, 0x21);
    CHECK(cart_cpu_read(0xC000) == 0x83 && cart_cpu_read_bus(0xE000, 0x5A) == 0x5A);

    // Boards with an explicit shrinking CHR window remain usable below their
    // native page size. Color Dreams maps a 4 KiB ROM at $0000-$0FFF and
    // leaves the upper pattern table open even after a CHR bank write.
    h = header_for(11, 0x8000, false);
    h.flags7 |= 0x08;
    h.chr_rom_chunks = 0x30; // 1 * 2^12 = 4 KiB.
    h.flags9 = 0xF0;
    image = image_for(&h, 0x8000, 0x1000, &size);
    CHECK(image != NULL);
    prg = image + sizeof(h);
    memset(prg, 0xFF, 0x8000);
    memset(prg + 0x8000, 0x6A, 0x1000);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_ppu_read(0x0123) == 0x6A && cart_ppu_read(0x1123) == 0x23);
    cart_cpu_write(0x8000, 0x10);
    CHECK(cart_ppu_read(0x0123) == 0x6A && cart_ppu_read(0x1123) == 0x23);

    // FME-7 shrinks its 1 KiB CHR pages to the physical ROM. A 512-byte ROM
    // remains bankable in slot zero and the following slot stays open bus.
    h = header_for(69, 0x8000, false);
    h.flags7 |= 0x08;
    h.chr_rom_chunks = 0x24; // 1 * 2^9 = 512 bytes.
    h.flags9 = 0xF0;
    image = image_for(&h, 0x8000, 0x0200, &size);
    CHECK(image != NULL);
    memset(image + sizeof(h) + 0x8000, 0x7B, 0x0200);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    sunsoft69_command(0, 1);
    CHECK(cart_ppu_read(0x0123) == 0x7B && cart_ppu_read(0x0323) == 0x23);

    return 0;
}

static int test_nintendo_shrunk_chr_pages(void) {
    size_t size;
    iNESHeader h;
    uint8_t *image;
    uint8_t *chr;

    // MMC1's two 4 KiB CHR slots shrink with the physical image. A 1 KiB ROM
    // therefore occupies $0000-$07FF after the two selected slots are mapped;
    // the rest of the pattern table remains open bus.
    h = header_for(1, 0x8000, false);
    h.flags7 |= 0x08;
    h.chr_rom_chunks = 0x28; // 1 KiB.
    h.flags9 = 0xF0;
    image = image_for(&h, 0x8000, 0x0400, &size);
    CHECK(image != NULL);
    chr = image + sizeof(h) + 0x8000;
    memset(chr, 0x61, 0x0400);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_ppu_read(0x0123) == 0x61 && cart_ppu_read(0x0523) == 0x61);
    CHECK(cart_ppu_read(0x0923) == 0x23);
    serial_write(0x8000, 0x1F);
    serial_write(0xA000, 7);
    serial_write(0xC000, 6);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    CHECK(cart_ppu_read(0x0123) == 0x61 && cart_ppu_read(0x0523) == 0x61);
    CHECK(cart_ppu_read(0x1123) == 0x23);

    // MMC2 and MMC4 start with their CHR slots unmapped. Register writes map
    // two shrunken 2 KiB slots at the bottom of PPU space; latch addresses may
    // still change the selected page even when those addresses are uncovered.
    const unsigned latch_boards[] = {9, 10};
    for (size_t i = 0; i < sizeof(latch_boards) / sizeof(latch_boards[0]); ++i) {
        h = header_for(latch_boards[i], 0x8000, false);
        h.flags7 |= 0x08;
        h.chr_rom_chunks = 0x2C; // 2 KiB.
        h.flags9 = 0xF0;
        image = image_for(&h, 0x8000, 0x0800, &size);
        CHECK(image != NULL);
        chr = image + sizeof(h) + 0x8000;
        memset(chr, (int)(0x72 + i), 0x0800);
        CHECK(load_rom_memory(image, size) == 0);
        free(image);
        CHECK(cart_ppu_read(0x0123) == 0x23);
        cart_cpu_write(0xB000, 3);
        cart_cpu_write(0xD000, 5);
        CHECK(cart_ppu_read(0x0123) == (uint8_t)(0x72 + i));
        CHECK(cart_ppu_read(0x0923) == (uint8_t)(0x72 + i));
        CHECK(cart_ppu_read(0x1123) == 0x23);
        (void)cart_ppu_read(0x0FD8);
        cart_cpu_write(0xC000, 7);
        CHECK(cart_ppu_read(0x0123) == (uint8_t)(0x72 + i));
        cart_cpu_write(0xF000, 1);
        CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    }

    // CNROM's single 8 KiB slot shrinks to the image. Bank writes wrap over
    // the complete-page count without mirroring data into the uncovered area.
    h = header_for(3, 0x8000, false);
    h.flags7 |= 0x08;
    h.chr_rom_chunks = 0x2C;
    h.flags9 = 0xF0;
    image = image_for(&h, 0x8000, 0x0800, &size);
    CHECK(image != NULL);
    memset(image + sizeof(h), 0xFF, 0x8000);
    memset(image + sizeof(h) + 0x8000, 0x84, 0x0800);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_ppu_read(0x0123) == 0x84 && cart_ppu_read(0x0923) == 0x23);
    cart_cpu_write(0x8000, 0xFF);
    CHECK(cart_ppu_read(0x0123) == 0x84 && cart_ppu_read(0x1123) == 0x23);

    // Protected CNROM uses the same shrinking slot and then applies its D0
    // pull-up behavior when the protection latch disables CHR output.
    h = header_for(185, 0x8000, false);
    h.flags7 |= 0x08;
    h.chr_rom_chunks = 0x2C;
    h.flags9 = 0xF0;
    image = image_for(&h, 0x8000, 0x0800, &size);
    CHECK(image != NULL);
    memset(image + sizeof(h), 0xFF, 0x8000);
    memset(image + sizeof(h) + 0x8000, 0x95, 0x0800);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_ppu_read(0x0123) == 0x95 && cart_ppu_read(0x0923) == 0x23);
    cart_cpu_write(0x8000, 0);
    CHECK(cart_ppu_read(0x0122) == 0x23 && cart_ppu_read(0x1122) == 0x23);

    // AxROM selects CHR page zero once at initialization. Its existing NROM
    // read path already exposes a short page followed by open bus.
    h = header_for(7, 0x8000, false);
    h.flags7 |= 0x08;
    h.chr_rom_chunks = 0x2C;
    h.flags9 = 0xF0;
    image = image_for(&h, 0x8000, 0x0800, &size);
    CHECK(image != NULL);
    memset(image + sizeof(h) + 0x8000, 0xA6, 0x0800);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_ppu_read(0x0123) == 0xA6 && cart_ppu_read(0x0923) == 0x23);
    cart_cpu_write(0x8000, 0x10);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);

    // CPROM maps one fixed and one switchable 4 KiB slot. With a 2 KiB ROM,
    // the fixed slot occupies $0000-$07FF and the switchable slot appears at
    // $0800-$0FFF only after the board register is written.
    h = header_for(13, 0x8000, false);
    h.flags7 |= 0x08;
    h.chr_rom_chunks = 0x2C;
    h.flags9 = 0xF0;
    image = image_for(&h, 0x8000, 0x0800, &size);
    CHECK(image != NULL);
    memset(image + sizeof(h), 0xFF, 0x8000);
    memset(image + sizeof(h) + 0x8000, 0xB7, 0x0800);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_ppu_read(0x0123) == 0xB7 && cart_ppu_read(0x0923) == 0x23);
    cart_cpu_write(0x8000, 3);
    CHECK(cart_ppu_read(0x0923) == 0xB7 && cart_ppu_read(0x1123) == 0x23);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    return 0;
}

static int test_jaleco18_banks_ram_and_mirroring(void) {
    CHECK(fixture(18, 0x40000, 0x20000, false) == 18);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_cpu_read_bus(0xC000, 0x69) == 0x69 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_ppu_read(0x0012) == 0x12 && cart_ppu_read(0x1C34) == 0x34);

    jaleco18_write_bank(0x8000, 0x13);
    jaleco18_write_bank(0x8002, 0x0B);
    jaleco18_write_bank(0x9000, 0x1E);
    CHECK(cart_cpu_read(0x8000) == 0x13 && cart_cpu_read(0xA000) == 0x0B);
    CHECK(cart_cpu_read(0xC000) == 0x1E && cart_cpu_read(0xE000) == 31);

    static const uint16_t chr_regs[8] = {
        0xA000, 0xA002, 0xB000, 0xB002, 0xC000, 0xC002, 0xD000, 0xD002
    };
    for (unsigned slot = 0; slot < 8; ++slot) {
        uint8_t bank = (uint8_t)(0x20 + slot * 3);
        jaleco18_write_bank(chr_regs[slot], bank);
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == bank);
    }

    cart_cpu_write(0x6123, 0xA6);
    CHECK(cart_cpu_read(0x6123) == 0xA6);
    for (unsigned mode = 0; mode < 4; ++mode) {
        cart_cpu_write(0xFFFE, (uint8_t)mode);
        CHECK(cart_get_mirroring() == (Mirroring)mode);
    }

    cart->reset();
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_ppu_read(0x0012) == 0x12 && !cart_irq_pending());
    CHECK(cart_cpu_read(0x6123) == 0xA6 && cart_get_mirroring() == MIRROR_HORIZONTAL);

    iNESHeader h = header_for(18, 0x4000, false);
    h.flags7 = 0x18;
    h.flags10 = 0;
    CHECK(fixture_with_header(&h, 0x4000, 0x2000) == 18);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    cart_cpu_write(0x6000, 0xA5);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    h = header_for(18, 0x20000, true);
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 18);
    cart_ppu_write(0x0123, 0xA6);
    cart_ppu_write(0x0523, 0x69);
    CHECK(cart_ppu_read(0x0123) == 0xA6 && cart_ppu_read(0x0523) == 0x69);
    cart_cpu_write(0xA003, 0); // The unwritten CHR register contains zero, regardless of initial RAM wiring.
    CHECK(cart_ppu_read(0x0523) == 0xA6);
    cart_ppu_write(0x0523, 0x35);
    CHECK(cart_ppu_read(0x0123) == 0x35);
    return 0;
}

static int test_jaleco18_irq_and_cpu_clock(void) {
    CHECK(fixture(18, 0x4000, 0x2000, false) == 18);
    CHECK(cart != NULL && cart->clock != NULL);
    static const struct { uint16_t reload; uint8_t control; } modes[] = {
        {0x1002, 0x01}, {0x0102, 0x03}, {0x0012, 0x05}, {0x0002, 0x09}
    };
    for (unsigned i = 0; i < sizeof(modes) / sizeof(modes[0]); ++i) {
        jaleco18_write_irq_reload(modes[i].reload);
        cart_cpu_write(0xF000, 0);
        cart_cpu_write(0xF001, modes[i].control);
        uint16_t mask = i == 0 ? 0xFFFF : i == 1 ? 0x0FFF : i == 2 ? 0x00FF : 0x000F;
        unsigned count = modes[i].reload & mask;
        cart->clock((int)(count - 1));
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
        cart_cpu_write(0xFFF0, 0); // $F000 alias reloads and acknowledges.
        CHECK(!cart_irq_pending());
    }

    jaleco18_write_irq_reload(3);
    cart_cpu_write(0xF000, 0);
    cart_cpu_write(0xF001, 1);
    cart->clock(1);
    cart_cpu_write(0xF001, 0);
    CHECK(!cart_irq_pending());
    cart->clock(20);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xF001, 1);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xF001, 0);
    CHECK(!cart_irq_pending());

    fixture_prg[0] = 0xEA;
    jaleco18_write_bank(0x8000, 0);
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cpu.pc = 0x8000;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
    jaleco18_write_irq_reload(2);
    cart_cpu_write(0xF000, 0);
    cart_cpu_write(0xF001, 1);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
    fixture_prg[0x3FFE] = 0;
    fixture_prg[0x3FFF] = 3;
    for (unsigned i = 0; i < 8; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
    cpu.pc = 0x0200;
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    for (unsigned step = 0; step < 3 && cpu.pc != 0x0300; ++step) (void)cpu_step(&cpu);
    CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG) && cart_irq_pending());
    cart_cpu_write(0xF001, 0);
    cpu_soft_reset(&cpu);
    CHECK(cart_cpu_read(0x8000) == 0xEA);
    return 0;
}

static int test_jaleco18_irq_width_transitions(void) {
    CHECK(fixture(18, 0x20000, 0x2000, false) == 18);
    const unsigned period[] = {65536, 4096, 256, 16, 16, 256};
    const uint8_t control[] = {1, 3, 5, 9, 15, 7};
    for (unsigned mode = 0; mode < sizeof(control); ++mode) {
        jaleco18_write_irq_reload(0);
        cart_cpu_write(0xF000, 0);
        cart_cpu_write(0xF001, control[mode]);
        cart->clock((int)(period[mode] - 1));
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
    }
    jaleco18_write_irq_reload(0x1230);
    cart_cpu_write(0xF000, 0);
    cart_cpu_write(0xF001, 9);
    cart->clock(16);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xF001, 1);
    cart->clock(0x122F);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending()); // Narrow counters leave the upper bits intact.
    return 0;
}

static int test_jaleco18_loader_validation(void) {
    CHECK(fixture(18, 0x4000, 0x2000, false) == 18);
    cart_cpu_write(0x6000, 0xA7);
    jaleco18_write_bank(0x8000, 1);
    Mapper *previous = cart;
    iNESHeader unsupported = header_for(18, 0x4000, false);
    unsupported.flags7 |= 0x08;
    unsupported.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&unsupported, fixture_prg, 0x4000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 1 && cart_cpu_read(0x6000) == 0xA7);

    return 0;
}

static int test_irem32_banks_variants_and_ram(void) {
    CHECK(fixture(32, 0x40000, 0x20000, false) == 32);
    CHECK(cart != NULL && cart->clock == NULL);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x1C12) == 0x12);

    cart_cpu_write(0x8ABC, 0x3F);
    CHECK(cart_cpu_read_bus(0xA000, 0x56) == 0x56);
    cart_cpu_write(0xA7FF, 0x22);
    CHECK(cart_cpu_read(0x8000) == 31 && cart_cpu_read(0xA000) == 2);
    cart_cpu_write(0x9ACE, 0x02);
    CHECK(cart_cpu_read(0x8000) == 30 && cart_cpu_read(0xC000) == 31);
    CHECK(cart_cpu_read(0xA000) == 2 && cart_cpu_read(0xE000) == 31);
    cart_cpu_write(0x9FFF, 0x01);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL && cart_cpu_read(0x8000) == 31);
    CHECK(cart_cpu_read(0xC000) == 30);
    cart_cpu_write(0x9000, 0);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);

    for (unsigned slot = 0; slot < 8; ++slot) {
        uint8_t bank = (uint8_t)(0x30 + slot);
        cart_cpu_write((uint16_t)(0xBFF8u + slot), bank);
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == bank);
    }
    cart_cpu_write(0x6000, 0xA6);
    CHECK(cart_cpu_read(0x6000) == 0xA6 && !cart_irq_pending());

    cart->reset();
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL && cart_cpu_read(0x6000) == 0xA6);
    cart_cpu_write(0x9000, 2);
    CHECK(cart_cpu_read(0x8000) == 30 && cart_cpu_read(0xA000) == 0);
    CHECK(cart_cpu_read(0xC000) == 0);

    iNESHeader h = header_for(32, 0x40000, false);
    h.flags7 |= 8;
    h.prg_ram_size = 0x10; // Submapper 1: fixed 8+8+16F PRG layout.
    h.flags10 = 7;
    CHECK(fixture_with_header(&h, 0x40000, 0x20000) == 32);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0x8000, 5);
    cart_cpu_write(0xA000, 7);
    cart_cpu_write(0x9000, 2);
    CHECK(cart_cpu_read(0x8000) == 5 && cart_cpu_read(0xA000) == 7);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);

    Mapper *previous = cart;
    uint8_t previous_bank = cart_cpu_read(0x8000);
    h.prg_ram_size = 0x20;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x40000, fixture_chr, 0x20000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == previous_bank);
    return 0;
}

static int test_irem65_banks_decode_and_ram(void) {
    CHECK(fixture(65, 0x40000, 0x20000, false) == 65);
    CHECK(cart != NULL && cart->clock != NULL);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x1C12) == 0x12);

    cart_cpu_write(0x8000, 9);
    cart_cpu_write(0xA000, 11);
    cart_cpu_write(0xC000, 13);
    CHECK(cart_cpu_read(0x8000) == 9 && cart_cpu_read(0xA000) == 11);
    CHECK(cart_cpu_read(0xC000) == 13 && cart_cpu_read(0xE000) == 31);
    cart_cpu_write(0x8FFF, 4);
    cart_cpu_write(0xA001, 5);
    cart_cpu_write(0xC001, 6);
    CHECK(cart_cpu_read(0x8000) == 9 && cart_cpu_read(0xA000) == 11 && cart_cpu_read(0xC000) == 13);

    cart_cpu_write(0x9001, 0x80);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    cart_cpu_write(0x9001, 0);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0x9000, 0x80);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);

    for (unsigned slot = 0; slot < 8; ++slot) {
        uint8_t bank = (uint8_t)(0x40 + slot);
        cart_cpu_write((uint16_t)(0xB000u + slot), bank);
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == bank);
    }
    cart_cpu_write(0xB008, 0x22);
    CHECK(cart_ppu_read(0) == 0x40);
    cart_cpu_write(0x6123, 0x5A);
    CHECK(cart_cpu_read(0x6123) == 0x5A);
    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_cpu_read(0x6123) == 0x5A && !cart_irq_pending());
    CHECK(fixture(65, 0x6000, 0x2000, false) == 65);
    CHECK(cart_cpu_read(0xC000) == 2 && cart_cpu_read(0xE000) == 2);
    return 0;
}

static int test_irem65_irq_and_loader_validation(void) {
    CHECK(fixture(65, 0x4000, 0x2000, false) == 65);
    cart_cpu_write(0x9005, 0x01);
    cart_cpu_write(0x9006, 0x02);
    cart_cpu_write(0x9004, 0);
    cart_cpu_write(0x9003, 0x80);
    cart->clock(0x101);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart->clock(8);
    CHECK(cart_irq_pending()); // Expiration disables the one-shot counter.
    cart_cpu_write(0x9003, 0);
    CHECK(!cart_irq_pending());

    cart_cpu_write(0x9005, 0);
    cart_cpu_write(0x9006, 3);
    cart_cpu_write(0x9004, 0);
    cart_cpu_write(0x9003, 0x80);
    cart->clock(1);
    cart_cpu_write(0x9004, 0);
    CHECK(!cart_irq_pending());
    cart->clock(2);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());

    cart_cpu_write(0x9003, 0);
    fixture_prg[0] = 0xEA;
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cpu.pc = 0x8000;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
    cart_cpu_write(0x9005, 0);
    cart_cpu_write(0x9006, 2);
    cart_cpu_write(0x9004, 0);
    cart_cpu_write(0x9003, 0x80);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());

    fixture_prg[0x3FFE] = 0;
    fixture_prg[0x3FFF] = 3;
    for (unsigned i = 0; i < 8; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
    cpu.pc = 0x0200;
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    for (unsigned step = 0; step < 3 && cpu.pc != 0x0300; ++step) (void)cpu_step(&cpu);
    CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG) && cart_irq_pending());
    write_mem(0x9007, 0);
    CHECK(cart_irq_pending());

    cart_cpu_write(0x9003, 0);
    cart_cpu_write(0x6000, 0xA7);
    Mapper *previous = cart;
    iNESHeader h = header_for(65, 0x4000, false);
    h.flags7 |= 8;
    h.prg_ram_size = 0x10; /* Unsupported NES 2.0 submapper must preserve the active board. */
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x6000) == 0xA7);
    return 0;
}

#endif // MAPPER_ACCURACY_NATIVE_H

