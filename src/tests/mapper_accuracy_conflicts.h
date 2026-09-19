/*
 * mapper_accuracy_conflicts.h - Mapper bus conflict regression tests
 *
 * Author: @frankischilling
 *
 * These checks verify cartridge bus conflicts and the register writes they affect.
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
#ifndef MAPPER_ACCURACY_CONFLICTS_H
#define MAPPER_ACCURACY_CONFLICTS_H

static int test_colordreams_bus_conflicts(void) {
    const uint16_t write_addr = 0x8123;
    const size_t bank_offset = write_addr - 0x8000;

    CHECK(fixture(11, 0x20000, 0x20000, false) == 11);

    // The first write must be masked by the byte in the bank mapped before the write.
    fixture_prg[bank_offset] = 0x52;
    fixture_prg[3 * 0x8000 + bank_offset] = 0x01;
    cart_cpu_write(write_addr, 0xF3);
    CHECK(cart_cpu_read(0x8000) == 8);
    CHECK(cart_ppu_read(0) == 40);

    // A ROM byte with every bit set leaves the CPU value unchanged.
    fixture_prg[2 * 0x8000 + bank_offset] = 0xFF;
    cart_cpu_write(write_addr, 0x31);
    CHECK(cart_cpu_read(0x8000) == 4);
    CHECK(cart_ppu_read(0) == 24);

    // Once bank 1 is mapped, the next conflict must use bank 1 at the write address.
    fixture_prg[1 * 0x8000 + bank_offset] = 0xA2;
    cart_cpu_write(write_addr, 0xF3);
    CHECK(cart_cpu_read(0x8000) == 8);
    CHECK(cart_ppu_read(0) == 80);
    return 0;
}

static int test_bus_conflict_submappers(void) {
    const unsigned boards[] = {2, 3, 7};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        for (unsigned submapper = 1; submapper <= 2; ++submapper) {
            iNESHeader h = header_for(boards[i], 0x20000, false);
            h.flags7 |= 0x08;
            h.prg_ram_size = (uint8_t)(submapper << 4);
            CHECK(fixture_with_header(&h, 0x20000, 0x8000) == (int)boards[i]);
            fixture_prg[0] = 1;
            cart_cpu_write(0x8000, 3);
            unsigned selected = submapper == 2 ? 1 : 3;
            if (boards[i] == 3) CHECK(cart_ppu_read(0) == selected * 8);
            else CHECK(cart_cpu_read(0x8001) == selected * (boards[i] == 2 ? 2 : 4));
        }
    }
    return 0;
}

static int test_mapper15_modes(void) {
    static const struct { uint16_t address; uint8_t value, pages[4]; } cases[] = {
        {0x8000, 0x02, {4, 5, 6, 7}}, {0x8000, 0x82, {5, 4, 7, 6}},
        {0x8001, 0x02, {4, 5, 14, 15}}, {0x8001, 0x82, {5, 6, 15, 16}},
        {0x8002, 0x02, {4, 4, 4, 4}}, {0x8002, 0x82, {5, 5, 5, 5}},
        {0x8003, 0x02, {4, 5, 4, 5}}, {0x8003, 0x82, {5, 6, 5, 6}},
        {0x8000, 0x42, {132, 133, 134, 135}}
    };
    CHECK(fixture(15, sizeof(fixture_prg), 0x2000, true) == 15);
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        cart_cpu_write(cases[i].address, cases[i].value);
        for (unsigned slot = 0; slot < 4; ++slot)
            CHECK(cart_cpu_read((uint16_t)(0x8000 + slot * 0x2000)) == cases[i].pages[slot]);
        CHECK(cart_get_mirroring() == ((cases[i].value & 0x40) ? MIRROR_HORIZONTAL : MIRROR_VERTICAL));
    }
    for (unsigned mode = 0; mode < 4; ++mode) {
        cart_cpu_write(0x8002, 0);
        cart_ppu_write(0, 0x42);
        cart_cpu_write((uint16_t)(0x8000 | mode), 0);
        cart_ppu_write(0, 0x69);
        CHECK(cart_ppu_read(0) == ((mode == 1 || mode == 2) ? 0x69 : 0x42));
    }
    return 0;
}

static iNESHeader action53_header(size_t prg_bytes, uint8_t chr_ram_shift) {
    iNESHeader h = header_for(28, prg_bytes, true);
    h.flags7 |= 0x08;
    h.prg_ram_size = 0;
    h.flags10 = 0;
    h.zero[0] = chr_ram_shift;
    return h;
}

static int test_action53_banks_and_mirroring(void) {
    iNESHeader h = action53_header(0x200000, 9); // 32KB CHR-RAM.
    CHECK(fixture_with_header(&h, 0x200000, 0x8000) == 28);

    // Power-up guarantees only the fixed last 16KB page at $C000.
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read(0xC000) == 0xFE);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL && cart->reset == NULL);

    // $81 selects the outer register. Mode $38 is 16KB banking with $8000 fixed.
    cart_cpu_write(0x5000, 0x81);
    CHECK(cart_cpu_read_bus(0x8123, 0x96) == 0x96);
    cart_cpu_write(0x8000, 0x12);
    cart_cpu_write(0x5000, 0x80);
    cart_cpu_write(0x8000, 0x38);
    cart_cpu_write(0x5000, 0x01);
    cart_cpu_write(0x8000, 0x06);
    CHECK(cart_cpu_read(0x8000) == 0x48 && cart_cpu_read(0xC000) == 0x4C);

    // Slot-select flips which 16KB half is fixed; 32KB mode keeps the pair adjacent.
    cart_cpu_write(0x5000, 0x80);
    cart_cpu_write(0x8000, 0x3C);
    CHECK(cart_cpu_read(0x8000) == 0x4C && cart_cpu_read(0xC000) == 0x4A);
    cart_cpu_write(0x8000, 0x20);
    CHECK(cart_cpu_read(0x8000) == 0x48 && cart_cpu_read(0xC000) == 0x4A);

    // Mirroring can be one-screen, vertical, or horizontal depending on mode and data bit.
    cart_cpu_write(0x8000, 0x00);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0x5000, 0x01);
    cart_cpu_write(0x8000, 0x10);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    cart_cpu_write(0x5000, 0x80);
    cart_cpu_write(0x8000, 0x02);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0x8000, 0x03);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);

    // Four 8KB CHR-RAM pages are independently writable; upper selector bits alias.
    cart_cpu_write(0x5000, 0x00);
    cart_cpu_write(0x8000, 0x00);
    cart_ppu_write(0x0123, 0xA0);
    cart_cpu_write(0x8000, 0x01);
    cart_ppu_write(0x0123, 0xB1);
    cart_cpu_write(0x8000, 0x02);
    cart_ppu_write(0x0123, 0xC2);
    cart_cpu_write(0x8000, 0x03);
    cart_ppu_write(0x0123, 0xD3);
    cart_cpu_write(0x8000, 0x05);
    CHECK(cart_ppu_read(0x0123) == 0xB1);

    // Addresses outside $5000-$5FFF do not change the selected register.
    cart_cpu_write(0x5000, 0x00);
    cart_cpu_write(0x4FFF, 0x81);
    cart_cpu_write(0x8000, 0x02);
    CHECK(cart_ppu_read(0x0123) == 0xC2);

    // Larger declared CHR RAM is accepted; the two-bit selector reaches the
    // first four 8 KiB pages and leaves later physical pages unreachable.
    size_t image_size;
    iNESHeader valid = action53_header(0x20000, 9);
    uint8_t *image = image_for(&valid, 0x20000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    free(image);
    iNESHeader large = action53_header(0x20000, 10); // 64 KiB CHR RAM.
    image = image_for(&large, 0x20000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    free(image);
    cart_cpu_write(0x5000, 0);
    cart_cpu_write(0x8000, 3);
    cart_ppu_write(0x0123, 0xD4);
    cart_cpu_write(0x8000, 7);
    CHECK(cart_ppu_read(0x0123) == 0xD4); // Selector 7 aliases physical page 3.
    cart_cpu_write(0x8000, 0);
    CHECK(cart_ppu_read(0x0123) == 0);
    cart_cpu_write(0x8000, 4);
    CHECK(cart_ppu_read(0x0123) == 0); // Selector 4 aliases page 0.
    cart_cpu_write(0x8000, 3);
    CHECK(cart_ppu_read(0x0123) == 0xD4);
    CHECK(cart_cpu_read(0xC000) == 0x5C);
    unload_rom();
    return 0;
}

static int test_action53_game_sizes(void) {
    iNESHeader h = action53_header(0x200000, 9);
    CHECK(fixture_with_header(&h, 0x200000, 0x8000) == 28);
    cart_cpu_write(0x5FFF, 0x81);
    cart_cpu_write(0xFFFF, 0x13);
    cart_cpu_write(0x5000, 1);
    cart_cpu_write(0x8000, 13);
    const uint8_t switched16[] = {39, 37, 37, 45};
    const uint8_t pair32[] = {38, 38, 34, 42};
    for (unsigned game = 0; game < 4; ++game) {
        cart_cpu_write(0x5000, 0x80);
        cart_cpu_write(0x8000, (uint8_t)(game << 4));
        CHECK(cart_cpu_read(0x8000) == pair32[game] * 2);
        CHECK(cart_cpu_read(0xC000) == (pair32[game] + 1) * 2);
        for (unsigned slot = 0; slot < 2; ++slot) {
            cart_cpu_write(0x8000, (uint8_t)((game << 4) | 8 | (slot << 2)));
            CHECK(cart_cpu_read(slot ? 0x8000 : 0xC000) == switched16[game] * 2);
            CHECK(cart_cpu_read(slot ? 0xC000 : 0x8000) == (38 + slot) * 2);
        }
    }
    return 0;
}

static int test_action53_largest_image(void) {
    iNESHeader h = action53_header(0x800000, 9);
    h.flags9 = 2;
    size_t bytes;
    uint8_t *image = image_for(&h, 0x800000, 0, &bytes);
    CHECK(image != NULL);
    for (size_t bank = 0; bank < 512; ++bank) {
        image[sizeof(h) + bank * 0x4000] = (uint8_t)bank;
        image[sizeof(h) + bank * 0x4000 + 1] = (uint8_t)(bank >> 8);
    }
    int loaded = load_rom_memory(image, bytes);
    free(image);
    CHECK(loaded == 0);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
    CHECK(cart_cpu_read(0xC000) == 0xFF && cart_cpu_read(0xC001) == 1);
    cart_cpu_write(0x5000, 0x81);
    cart_cpu_write(0x8000, 0xFF);
    CHECK(cart_cpu_read(0x8000) == 0xFE && cart_cpu_read(0x8001) == 1);
    CHECK(cart_cpu_read(0xC000) == 0xFF && cart_cpu_read(0xC001) == 1);
    uint8_t *old_prg = prg_rom;
    h = action53_header(0x20000, 7); // 8 KiB CHR RAM is present beside CHR ROM.
    h.chr_rom_chunks = 1;
    image = image_for(&h, 0x20000, 0x2000, &bytes);
    CHECK(image != NULL);
    memset(image + sizeof(h) + 0x20000, 0xA6, 0x2000);
    loaded = load_rom_memory(image, bytes);
    CHECK(loaded == 0 && prg_rom != old_prg);
    CHECK(cart_ppu_read(0x0123) == 0x23); // CHR ROM has not been selected yet.
    cart_cpu_write(0x5000, 0);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0x8000, 3);
    CHECK(cart_ppu_read(0x0123) == 0xA6);
    Mapper *previous = cart;
    uint8_t *previous_prg = prg_rom;
    CHECK(load_rom_memory(image, bytes - 1) == -1);
    free(image);
    CHECK(cart == previous && prg_rom == previous_prg && cart_ppu_read(0x0123) == 0xA6);
    return 0;
}

static iNESHeader unrom512_header(uint8_t submapper, bool battery, uint8_t chr_ram_shift) {
    iNESHeader h = header_for(30, 0x40000, true);
    h.flags7 |= 0x08;
    h.prg_ram_size = (uint8_t)(submapper << 4);
    h.flags10 = 0;
    h.zero[0] = chr_ram_shift;
    if (battery) h.flags6 |= 0x02;
    return h;
}

static iNESHeader m111_header(void) {
    iNESHeader h = header_for(111, 0x80000, true);
    h.flags7 |= 0x08;
    h.flags10 = 0;
    h.zero[0] = 8; // 16 KiB CHR RAM.
    return h;
}

static iNESHeader m96_header(void) {
    iNESHeader h = header_for(96, 0x20000, true);
    h.flags7 |= 0x08;
    h.flags10 = 0;
    h.zero[0] = 9; // 32 KiB CHR RAM.
    return h;
}

static void m111_flash_command(uint8_t command) {
    cart_cpu_write(0xD555, 0xAA);
    cart_cpu_write(0xAAAA, 0x55);
    cart_cpu_write(0xD555, command);
}

static void m111_flash_erase_prefix(void) {
    m111_flash_command(0x80);
    cart_cpu_write(0xD555, 0xAA);
    cart_cpu_write(0xAAAA, 0x55);
}

static void unrom512_flash_command(uint8_t command) {
    cart_cpu_write(0xC000, 0x01);
    cart_cpu_write(0x9555, 0xAA);
    cart_cpu_write(0xC000, 0x00);
    cart_cpu_write(0xAAAA, 0x55);
    cart_cpu_write(0xC000, 0x01);
    cart_cpu_write(0x9555, command);
}

static void unrom512_flash_erase_prefix(void) {
    unrom512_flash_command(0x80);
    cart_cpu_write(0xC000, 0x01);
    cart_cpu_write(0x9555, 0xAA);
    cart_cpu_write(0xC000, 0x00);
    cart_cpu_write(0xAAAA, 0x55);
}

static int test_unrom512_banks_flash_and_mirroring(void) {
    iNESHeader legacy = header_for(30, 0x40000, true);
    RomRamSizes ram;
    CHECK(rom_ram_sizes(&legacy, &ram) == 0);
    CHECK(ram.prg_ram == 0 && ram.prg_nvram == 0 && ram.chr_ram == 0x8000 && ram.chr_nvram == 0);
    CHECK(fixture_with_header(&legacy, 0x40000, 0x8000) == 30);

    iNESHeader h = unrom512_header(1, true, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 30);

    cart_cpu_write(0xC000, 0x03);
    CHECK(cart_cpu_read(0x8000) == 6 && cart_cpu_read(0xC000) == 30);
    cart_cpu_write(0xC000, 0x63);
    cart_ppu_write(0x0123, 0xD3);
    cart_cpu_write(0xC000, 0x23);
    cart_ppu_write(0x0123, 0xB1);
    cart_cpu_write(0xC000, 0x63);
    CHECK(cart_ppu_read(0x0123) == 0xD3);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL); // Submapper 1 ignores bit 7.

    // A valid byte-program command can only clear flash bits.
    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 0x03);
    CHECK(cart_cpu_read(0x8123) == 6);
    cart_cpu_write(0x8123, 0x04);
    CHECK(cart_cpu_read(0x8123) == 0x04);
    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 0x03);
    cart_cpu_write(0x8123, 0xFF);
    CHECK(cart_cpu_read(0x8123) == 0x04);

    // Invalid and interrupted unlock sequences do not alter program storage.
    cart_cpu_write(0xC000, 0x05);
    uint8_t untouched = cart_cpu_read(0x8234);
    cart_cpu_write(0xC000, 0x01);
    cart_cpu_write(0x9555, 0xAA);
    cart_cpu_write(0xC000, 0x00);
    cart_cpu_write(0xAAAA, 0x54);
    cart_cpu_write(0xC000, 0x05);
    cart_cpu_write(0x8234, 0x00);
    CHECK(cart_cpu_read(0x8234) == untouched);
    cart_cpu_write(0xC000, 0x05);
    untouched = cart_cpu_read(0x8234);
    cart_cpu_write(0xC000, 0x01);
    cart_cpu_write(0x9554, 0xAA); // Unlock data at the wrong physical address.
    cart_cpu_write(0xC000, 0x00);
    cart_cpu_write(0xAAAA, 0x55);
    cart_cpu_write(0xC000, 0x01);
    cart_cpu_write(0x9555, 0xA0);
    cart_cpu_write(0xC000, 0x05);
    cart_cpu_write(0x8234, 0x00);
    CHECK(cart_cpu_read(0x8234) == untouched);

    // Software ID mode responds through the currently selected physical bank.
    unrom512_flash_command(0x90);
    cart_cpu_write(0xC000, 0x00);
    CHECK(cart_cpu_read(0x8000) == 0xBF && cart_cpu_read(0x8001) == 0xB7);
    cart_cpu_write(0x8000, 0xF0);
    CHECK(cart_cpu_read(0x8000) == 0);

    // Sector erase uses the selected physical bank and restores the whole 4KB sector.
    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 0x04);
    cart_cpu_write(0x8123, 0x00);
    CHECK(cart_cpu_read(0x8123) == 0x00);
    unrom512_flash_erase_prefix();
    cart_cpu_write(0xC000, 0x04);
    cart_cpu_write(0x8123, 0x30);
    CHECK(cart_cpu_read(0x8123) == 0xFF && cart_cpu_read(0x8FFF) == 0xFF);

    // Chip erase follows the six-write sequence as well.
    unrom512_flash_erase_prefix();
    cart_cpu_write(0xC000, 0x01);
    cart_cpu_write(0x9555, 0x10);
    CHECK(cart_cpu_read(0x8000) == 0xFF && cart_cpu_read(0xC000) == 0xFF);

    // Read-only boards treat the same addresses as bank writes and keep PRG immutable.
    h = unrom512_header(1, false, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    cart_cpu_write(0x8000, 0x03);
    CHECK(cart_cpu_read(0x8000) == 6);
    uint8_t readonly = fixture_prg[3 * 0x4000 + 0x123];
    unrom512_flash_command(0xA0);
    cart_cpu_write(0x8000, 0x03);
    cart_cpu_write(0x8123, 0x00);
    CHECK(fixture_prg[3 * 0x4000 + 0x123] == readonly);

    // CHR ROM remains unmapped until the board's latch selects a page. A
    // separately declared CHR RAM chip does not replace the ROM source.
    {
        iNESHeader rom_source = unrom512_header(1, false, 7);
        rom_source.chr_rom_chunks = 2;
        size_t source_size;
        uint8_t *source_image = image_for(&rom_source, 0x40000, 0x4000, &source_size);
        CHECK(source_image != NULL);
        uint8_t *source_chr = source_image + sizeof(rom_source) + 0x40000;
        memset(source_chr, 0xA6, 0x2000);
        memset(source_chr + 0x2000, 0xB7, 0x2000);
        CHECK(load_rom_memory(source_image, source_size) == 0);
        ppu_power_on(&ppu);
        CHECK(ppu_read(0x0123) == 0x23);
        ppu_write(0x0123, 0x5D);
        CHECK(ppu_read(0x0123) == 0x23);
        cart_cpu_write(0xC000, 0x20);
        CHECK(ppu_read(0x0123) == 0xB7);
        ppu_write(0x0123, 0x5D);
        CHECK(ppu_read(0x0123) == 0xB7);
        Mapper *source_cart = cart;
        uint8_t *source_prg = prg_rom;
        CHECK(load_rom_memory(source_image, source_size - 1) == -1);
        free(source_image);
        CHECK(cart == source_cart && prg_rom == source_prg && ppu_read(0x0123) == 0xB7);
    }

    // Switchable one-screen and submapper 3 H/V mirroring use latch bit 7.
    h = unrom512_header(1, false, 9);
    h.flags6 |= 0x08;
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0x8000, 0x80);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    h = unrom512_header(3, false, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0x8000, 0x00);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    cart_cpu_write(0x8000, 0x80);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);

    // Submapper 4 reserves $8000-$BFFF for its LED register.
    h = unrom512_header(4, false, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    cart_cpu_write(0xC000, 0x03);
    CHECK(cart_cpu_read(0x8000) == 6);
    cart_cpu_write(0x8000, 0x07);
    CHECK(cart_cpu_read(0x8000) == 6);

    // Four-screen boards use the top CHR-RAM page as cartridge nametable RAM.
    h = unrom512_header(1, false, 9);
    h.flags6 |= 0x09;
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    uint8_t nt[0x1000] = {0};
    cart_nt_write(0x2000, 0x20, nt);
    cart_nt_write(0x2C00, 0x2C, nt);
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x2C00, nt) == 0x2C);
    CHECK(fixture_chr[0x6000] == 0x20 && fixture_chr[0x6C00] == 0x2C);
    cart_nt_write(0x3000, 0x30, nt);
    cart_nt_write(0x3E00, 0x3E, nt);
    CHECK(cart_nt_read(0x3000, nt) == 0x30 && cart_nt_read(0x3E00, nt) == 0x3E);
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && fixture_chr[0x7000] == 0x30);
    ppu_power_on(&ppu);
    ppu_write(0x2E12, 0xA6);
    ppu_write(0x3E12, 0x69);
    CHECK(ppu_read(0x2E12) == 0xA6 && ppu_read(0x3E12) == 0x69);
    CHECK(fixture_chr[0x6E12] == 0xA6 && fixture_chr[0x7E12] == 0x69);
    cart_cpu_write(0x8000, 0x60);
    CHECK(cart_ppu_read(0x0E12) == 0xA6 && cart_ppu_read(0x1E12) == 0x69);
    ppu_write(0x3F00, 0x2A);
    CHECK(ppu_read(0x3F00) == 0x2A && fixture_chr[0x7F00] != 0x2A);

    // Malformed flash metadata still fails transactionally. Four-screen boards
    // with less than 32 KiB CHR RAM fall back to cartridge nametable RAM.
    iNESHeader valid = unrom512_header(1, false, 9);
    size_t image_size;
    uint8_t *image = image_for(&valid, 0x40000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    free(image);
    uint8_t *previous_prg = prg_rom;
    uint8_t *previous_chr = chr_rom;
    iNESHeader previous_header = ines_header;
    iNESHeader invalid = unrom512_header(5, true, 9);
    image = image_for(&invalid, 0x40000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == -1);
    free(image);
    CHECK(prg_rom == previous_prg && chr_rom == previous_chr);
    CHECK(memcmp(&ines_header, &previous_header, sizeof(ines_header)) == 0);
    iNESHeader small_four = unrom512_header(1, false, 8);
    small_four.flags6 |= 0x09;
    image = image_for(&small_four, 0x40000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    free(image);
    uint8_t small_nt[0x1000] = {0};
    cart_nt_write(0x2001, 0x35, small_nt);
    cart_nt_write(0x2C01, 0x53, small_nt);
    CHECK(cart_nt_read(0x2001, small_nt) == 0x35);
    CHECK(cart_nt_read(0x2C01, small_nt) == 0x53);
    unload_rom();
    return 0;
}

static int test_unrom512_physical_flash_address(void) {
    iNESHeader h = unrom512_header(1, true, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    cart_cpu_write(0xC000, 0x13);
    CHECK(cart_cpu_read(0x8123) == 6);
    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 0x13);
    cart_cpu_write(0x8123, 0);
    CHECK(cart_cpu_read(0x8123) == 6); // Unpopulated flash addresses do not wrap for writes.
    unrom512_flash_erase_prefix();
    cart_cpu_write(0xC000, 0x13);
    cart_cpu_write(0x8123, 0x30);
    CHECK(cart_cpu_read(0x8123) == 6);
    cart_cpu_write(0xC000, 3);
    CHECK(cart_cpu_read(0x8123) == 6);

    // The same physical address exists on a 512KB flash chip.
    h.prg_rom_chunks = 32;
    CHECK(fixture_with_header(&h, 0x80000, 0x8000) == 30);
    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 0x13);
    cart_cpu_write(0x8123, 0);
    CHECK(cart_cpu_read(0x8123) == 0);
    cart_cpu_write(0xC000, 3);
    CHECK(cart_cpu_read(0x8123) == 6);
    unrom512_flash_erase_prefix();
    cart_cpu_write(0xC000, 0x13);
    cart_cpu_write(0x8123, 0x30);
    CHECK(cart_cpu_read(0x8123) == 0xFF && cart_cpu_read(0x9000) == 38);

    // Undefined commands consume their third command cycle without changing ID mode.
    unrom512_flash_command(0x90);
    unrom512_flash_command(0x00);
    cart_cpu_write(0x8000, 0xF0);
    CHECK(cart_cpu_read(0x8000) == 0xBF);
    cart_cpu_write(0x8000, 0xF0);
    CHECK(cart_cpu_read(0x8000) == 2);
    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 3);
    cart_cpu_write(0x8123, 0xF0); // Program data $F0 is not an ID-exit command.
    CHECK(cart_cpu_read(0x8123) == 0);

    for (size_t prg_bytes = 0x4000; prg_bytes <= 0x80000; prg_bytes *= 2) {
        h.prg_rom_chunks = (uint8_t)(prg_bytes / 0x4000);
        h.flags6 &= (uint8_t)~2u;
        CHECK(fixture_with_header(&h, prg_bytes, 0x8000) == 30);
        cart_cpu_write(0x8000, 31);
        CHECK(cart_cpu_read(0x8000) == (prg_bytes / 0x4000 - 1) * 2);
        CHECK(cart_cpu_read(0xC000) == (prg_bytes / 0x4000 - 1) * 2);
    }
    h = unrom512_header(2, true, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    memset(fixture_prg, 0xFF, 0x40000);
    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 3);
    cart_cpu_write(0x8123, 0x96);
    CHECK(cart_cpu_read(0x8123) == 0x96);
    return 0;
}

static int test_unrom512_cpu_flash(void) {
    iNESHeader h = unrom512_header(1, true, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    const uint8_t program[] = {
        0xA9, 1, 0x8D, 0, 0xC0,
        0xA9, 0xAA, 0x8D, 0x55, 0x95,
        0xA9, 0, 0x8D, 0, 0xC0,
        0xA9, 0x55, 0x8D, 0xAA, 0xAA,
        0xA9, 1, 0x8D, 0, 0xC0,
        0xA9, 0xA0, 0x8D, 0x55, 0x95,
        0xA9, 3, 0x8D, 0, 0xC0,
        0xA9, 4, 0x8D, 0x23, 0x81,
        0xAD, 0x23, 0x81
    };
    for (size_t i = 0; i < sizeof(program); ++i)
        write_mem((uint16_t)(0x0200 + i), program[i]);
    cpu.pc = 0x0200;
    for (unsigned i = 0; i < 8; ++i) {
        CHECK(cpu_step(&cpu) == 2);
        CHECK(cpu_step(&cpu) == 4);
    }
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 4);
    CHECK(cart_cpu_read(0xC000) == 30);
    cpu_soft_reset(&cpu);
    CHECK(cart_cpu_read(0x8123) == 4);
    return 0;
}

static int test_mapper111_banks_flash_and_nametables(void) {
    iNESHeader legacy = header_for(111, 0x80000, true);
    RomRamSizes ram;
    CHECK(rom_ram_sizes(&legacy, &ram) == 0);
    CHECK(ram.prg_ram == 0 && ram.prg_nvram == 0 && ram.chr_ram == 0x4000 && ram.chr_nvram == 0);
    CHECK(fixture_with_header(&legacy, 0x80000, 0x4000) == 111);

    iNESHeader h = m111_header();
    CHECK(fixture_with_header(&h, 0x80000, 0x4000) == 111);
    CHECK(cart_get_mirroring() == MIRROR_FOUR && cart_cpu_read(0x8000) == 0);

    cart_cpu_write(0x5000, 0x03);
    CHECK(cart_cpu_read(0x8000) == 12);
    cart_cpu_write(0x7000, 0x06);
    CHECK(cart_cpu_read(0x8000) == 24);

    CHECK(cart_cpu_read_bus(0x5000, 0x21) == 0);
    CHECK(cart_cpu_read(0x8000) == 4);
    CHECK(cart_cpu_read_bus(0x7FFF, 0x32) == 0);
    CHECK(cart_cpu_read(0x8000) == 8);

    cart_cpu_write(0x5000, 0x00);
    cart_ppu_write(0x0123, 0xA1);
    cart_cpu_write(0x5000, 0x10);
    cart_ppu_write(0x0123, 0xB2);
    CHECK(cart_ppu_read(0x0123) == 0xB2);
    cart_cpu_write(0x5000, 0x00);
    CHECK(cart_ppu_read(0x0123) == 0xA1);

    uint8_t nt[0x1000] = {0};
    cart_nt_write(0x2000, 0x20, nt);
    cart_nt_write(0x3C00, 0x27, nt);
    cart_cpu_write(0x5000, 0x20);
    CHECK(cart_nt_read(0x2000, nt) == 0 && cart_nt_read(0x3C00, nt) == 0);
    cart_nt_write(0x2000, 0x30, nt);
    cart_nt_write(0x3C00, 0x37, nt);
    cart_cpu_write(0x5000, 0x00);
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x3C00, nt) == 0x27);
    cart_cpu_write(0x5000, 0x20);
    CHECK(cart_nt_read(0x2000, nt) == 0x30 && cart_nt_read(0x3C00, nt) == 0x37);

    cart_cpu_write(0x5000, 0x03);
    m111_flash_command(0x90);
    CHECK(cart_cpu_read(0x8000) == 0xBF && cart_cpu_read(0x8001) == 0xB7);
    cart_cpu_write(0x8000, 0xF0);
    CHECK(cart_cpu_read(0x8000) == 12);

    m111_flash_command(0xA0);
    cart_cpu_write(0x8123, 0x00);
    CHECK(cart_cpu_read(0x8123) == 0x00);
    m111_flash_command(0xA0);
    cart_cpu_write(0x8123, 0xFF);
    CHECK(cart_cpu_read(0x8123) == 0x00);

    uint8_t untouched = cart_cpu_read(0x9234);
    cart_cpu_write(0xD555, 0xAA);
    cart_cpu_write(0xAAAA, 0x54);
    cart_cpu_write(0x9234, 0x00);
    CHECK(cart_cpu_read(0x9234) == untouched);

    m111_flash_erase_prefix();
    cart_cpu_write(0x8123, 0x30);
    CHECK(cart_cpu_read(0x8123) == 0xFF);

    cart_cpu_write(0x5000, 0x10);
    cart_ppu_write(0x0456, 0xD4);
    cart_cpu_write(0x5000, 0x02);
    CHECK(cart_cpu_read(0x8000) == 8);
    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 0);
    cart_cpu_write(0x5000, 0x10);
    CHECK(cart_ppu_read(0x0456) == 0xD4);

    m111_flash_command(0xA0);
    cart_cpu_write(0x8123, 0x00);
    m111_flash_erase_prefix();
    cart_cpu_write(0xD555, 0x10);
    CHECK(cart_cpu_read(0x8123) == 0xFF);

    size_t image_size;
    uint8_t *image = image_for(&h, 0x80000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    free(image);
    iNESHeader smaller_chr = h;
    smaller_chr.zero[0] = 7; // 8 KiB CHR RAM.
    image = image_for(&smaller_chr, 0x80000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    free(image);
    cart_ppu_write(0x0123, 0x69);
    CHECK(cart_ppu_read(0x0123) == 0x69);
    cart_cpu_write(0x5000, 0x10);
    CHECK(cart_ppu_read(0x0123) == 0x69);

    // With CHR ROM present, the latch selects ROM pages even when the header
    // also declares the board's CHR RAM. Writes leave the selected ROM intact.
    {
        iNESHeader rom_source = h;
        rom_source.chr_rom_chunks = 2;
        rom_source.zero[0] = 8;
        size_t source_size;
        uint8_t *source_image = image_for(&rom_source, 0x80000, 0x4000, &source_size);
        CHECK(source_image != NULL);
        uint8_t *source_chr = source_image + sizeof(rom_source) + 0x80000;
        memset(source_chr, 0xA6, 0x2000);
        memset(source_chr + 0x2000, 0xB7, 0x2000);
        CHECK(load_rom_memory(source_image, source_size) == 0);
        ppu_power_on(&ppu);
        CHECK(ppu_read(0x0123) == 0xA6);
        ppu_write(0x0123, 0x5D);
        CHECK(ppu_read(0x0123) == 0xA6);
        cart_cpu_write(0x5000, 0x10);
        CHECK(ppu_read(0x0123) == 0xB7);
        ppu_write(0x0123, 0x5D);
        CHECK(ppu_read(0x0123) == 0xB7);
        Mapper *source_cart = cart;
        uint8_t *source_prg = prg_rom;
        CHECK(load_rom_memory(source_image, source_size - 1) == -1);
        free(source_image);
        CHECK(cart == source_cart && prg_rom == source_prg && ppu_read(0x0123) == 0xB7);
    }

    iNESHeader invalid = h;
    invalid.flags10 = 7;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x80000, fixture_chr, 0x4000) == 111);
    invalid = h;
    invalid.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x80000, fixture_chr, 0x4000) == -1);
    return 0;
}

static int test_mapper111_cpu_soft_reset_preserves_state(void) {
    iNESHeader h = m111_header();
    CHECK(fixture_with_header(&h, 0x80000, 0x4000) == 111);
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);

    // Console soft reset only resets the CPU/APU/PPU. The GTROM latch and
    // flash command state remain live across that reset.
    cart_cpu_write(0x5000, 0x03);
    CHECK(cart_cpu_read(0x8000) == 12);
    m111_flash_command(0x90);
    CHECK(cart_cpu_read(0x8000) == 0xBF);
    cpu_soft_reset(&cpu);
    CHECK(cart_cpu_read(0x8000) == 0xBF);
    cart_cpu_write(0x8000, 0xF0);
    CHECK(cart_cpu_read(0x8000) == 12);

    m111_flash_command(0xA0);
    cpu_soft_reset(&cpu);
    cart_cpu_write(0x8123, 0x04);
    CHECK(cart_cpu_read(0x8123) == 0x04);
    CHECK(cart_cpu_read(0x8000) == 12);
    return 0;
}

static int test_mapper96_banks_latch_and_loader(void) {
    iNESHeader legacy = header_for(96, 0x20000, true);
    RomRamSizes ram;
    CHECK(rom_ram_sizes(&legacy, &ram) == 0);
    CHECK(ram.prg_ram == 0x2000 && ram.prg_nvram == 0 && ram.chr_ram == 0x8000 && ram.chr_nvram == 0);
    CHECK(fixture_with_header(&legacy, 0x20000, 0x8000) == 96);

    cart_cpu_write(0x6000, 0x35);
    cart_cpu_write(0x7FFF, 0x53);
    CHECK(cart_cpu_read(0x6000) == 0x35 && cart_cpu_read(0x7FFF) == 0x53);

    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xE000) == 3);
    CHECK(cart_ppu_read(0x0000) == 0 && cart_ppu_read(0x1000) == 4);

    // The board has ROM bus conflicts. Bank 0 drives $03 at $E000, so a
    // write of $07 reaches the mapper as $03 and cannot set the outer CHR bit.
    cart_cpu_write(0xE000, 0x07);
    CHECK(cart_cpu_read(0x8000) == 12);
    CHECK(cart_ppu_read(0x0000) == 0 && cart_ppu_read(0x1000) == 12);

    // Permit explicit register values at one test address while keeping the
    // tagged bank starts intact for PRG selection checks.
    for (unsigned bank = 0; bank < 4; ++bank)
        fixture_prg[bank * 0x8000u + 0x7FFEu] = 0xFF;
    for (unsigned bank = 0; bank < 4; ++bank) {
        cart_cpu_write(0xFFFE, (uint8_t)bank);
        CHECK(cart_cpu_read(0x8000) == bank * 4);
    }

    // Bit 2 selects the outer group. The upper pattern-table half stays on
    // outer|3 while the lower half follows the nametable-entry latch.
    cart_cpu_write(0xFFFE, 0x04);
    CHECK(cart_cpu_read(0x8000) == 0);
    CHECK(cart_ppu_read(0x0000) == 16 && cart_ppu_read(0x1000) == 28);
    cart_ppu_write(0x0123, 0xA1);
    cart_ppu_write(0x1123, 0xC3);

    (void)ppu_read(0x1000);
    (void)ppu_read(0x2200);
    CHECK(cart_ppu_read(0x0000) == 24);
    cart_ppu_write(0x0123, 0xB2);
    CHECK(cart_ppu_read(0x0123) == 0xB2 && cart_ppu_read(0x1123) == 0xC3);

    (void)ppu_read(0x2300); // Staying in $2xxx does not relatch inner bank 3.
    CHECK(cart_ppu_read(0x0123) == 0xB2);
    (void)ppu_read(0x3000);
    (void)ppu_read(0x2000);
    CHECK(cart_ppu_read(0x0123) == 0xA1 && cart_ppu_read(0x1123) == 0xC3);

    // A normal console soft reset preserves mapper state.
    cart_cpu_write(0xFFFE, 0x06);
    (void)ppu_read(0x3000);
    (void)ppu_read(0x2300);
    CHECK(cart_cpu_read(0x8000) == 8 && cart_ppu_read(0x0000) == 28);
    ppu_soft_reset(&ppu);
    apu_soft_reset(&apu);
    cpu_soft_reset(&cpu);
    CHECK(cart_cpu_read(0x8000) == 8 && cart_ppu_read(0x0000) == 28);

    // Legacy loading supplies the board's default 8 KiB work RAM and 32 KiB CHR RAM.
    size_t image_size;
    uint8_t *image = image_for(&legacy, 0x20000, 0, &image_size);
    CHECK(image != NULL);
    const uint8_t legacy_ram_program[] = {
        0xA9, 0x35,             // LDA #$35
        0x8D, 0x00, 0x60,       // STA $6000
        0xAD, 0x00, 0x60,       // LDA $6000
        0xA9, 0x53,             // LDA #$53
        0x8D, 0xFF, 0x7F,       // STA $7FFF
        0xAD, 0xFF, 0x7F        // LDA $7FFF
    };
    memcpy(image + sizeof(legacy) + 0x0100, legacy_ram_program, sizeof(legacy_ram_program));
    image[sizeof(legacy) + 0x7FFC] = 0x00;
    image[sizeof(legacy) + 0x7FFD] = 0x81;
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    CHECK(rom_mapper_number(&ines_header) == 96 && chr_size == 0x8000);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu) && cpu.pc == 0x8100);
    CHECK(cpu_step(&cpu) == 2);
    CHECK(cpu_step(&cpu) == 4);
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x35);
    CHECK(cpu_step(&cpu) == 2);
    CHECK(cpu_step(&cpu) == 4);
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x53);

    iNESHeader h = m96_header();
    image = image_for(&h, 0x20000, 0, &image_size);
    CHECK(image != NULL);
    const uint8_t no_ram_program[] = {
        0xA9, 0x69,             // LDA #$69
        0x8D, 0x00, 0x60,       // STA $6000
        0xAD, 0x00, 0x60        // LDA $6000; open bus keeps the operand high byte.
    };
    memcpy(image + sizeof(h) + 0x0100, no_ram_program, sizeof(no_ram_program));
    image[sizeof(h) + 0x7FFC] = 0x00;
    image[sizeof(h) + 0x7FFD] = 0x81;
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu) && cpu.pc == 0x8100);
    CHECK(cpu_step(&cpu) == 2);
    CHECK(cpu_step(&cpu) == 4);
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x60);
    CHECK(cart_cpu_read_bus(0x6000, 0xA6) == 0xA6);
    cart_cpu_write(0x6000, 0x69);
    CHECK(cart_cpu_read_bus(0x6000, 0xA6) == 0xA6);
    uint8_t *previous_prg = prg_rom;
    uint8_t *previous_chr = chr_rom;

    iNESHeader smaller_chr = h;
    smaller_chr.zero[0] = 8; // 16 KiB CHR RAM.
    image = image_for(&smaller_chr, 0x20000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    free(image);
    cart_ppu_write(0x0123, 0x79);
    CHECK(cart_ppu_read(0x0123) == 0x79);
    previous_prg = prg_rom;
    previous_chr = chr_rom;

    // CHR ROM starts unmapped even when a separate CHR RAM chip is declared.
    // The first mapper write selects ROM; the sidecar RAM remains independent.
    iNESHeader rom_with_sidecar = h;
    rom_with_sidecar.chr_rom_chunks = 1;
    rom_with_sidecar.zero[0] = 7;
    image = image_for(&rom_with_sidecar, 0x20000, 0x2000, &image_size);
    CHECK(image != NULL);
    image[sizeof(rom_with_sidecar) + 0x7FFE] = 0xFF;
    memset(image + sizeof(rom_with_sidecar) + 0x20000, 0xA6, 0x2000);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    cart_ppu_write(0x0123, 0x5D);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0xFFFE, 0);
    CHECK(cart_ppu_read(0x0123) == 0xA6);
    cart_ppu_write(0x0123, 0x5D);
    CHECK(cart_ppu_read(0x0123) == 0xA6);

    iNESHeader with_work_ram = h;
    with_work_ram.flags10 = 7;
    image = image_for(&with_work_ram, 0x20000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    free(image);
    cart_cpu_write(0x6000, 0x96);
    cart_cpu_write(0x7FFF, 0x69);
    CHECK(cart_cpu_read(0x6000) == 0x96 && cart_cpu_read(0x7FFF) == 0x69);

    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 96);
    previous_prg = prg_rom;
    previous_chr = chr_rom;
    iNESHeader invalid = h;
    invalid.prg_ram_size = 0x10;
    image = image_for(&invalid, 0x20000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == -1);
    free(image);
    CHECK(prg_rom == previous_prg && chr_rom == previous_chr);

    iNESHeader smaller_prg = h;
    smaller_prg.prg_rom_chunks = 4;
    image = image_for(&smaller_prg, 0x10000, 0, &image_size);
    CHECK(image != NULL);
    memset(image + sizeof(smaller_prg), 0x31, 0x8000);
    memset(image + sizeof(smaller_prg) + 0x8000, 0x52, 0x8000);
    image[sizeof(smaller_prg) + 0x7FFE] = 0xFF;
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    CHECK(cart_cpu_read(0x8100) == 0x31);
    cart_cpu_write(0xFFFE, 1);
    CHECK(cart_cpu_read(0x8100) == 0x52);
    previous_prg = prg_rom;
    previous_chr = chr_rom;

    iNESHeader chr_rom = h;
    chr_rom.chr_rom_chunks = 1;
    chr_rom.zero[0] = 0;
    image = image_for(&chr_rom, 0x20000, 0x2000, &image_size);
    CHECK(image != NULL);
    image[sizeof(chr_rom) + 0x7FFE] = 0xFF;
    memset(image + sizeof(chr_rom) + 0x20000, 0xB7, 0x2000);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    CHECK(rom_mapper_number(&ines_header) == 96 && cart_ppu_read(0x0123) == 0x23);
    cart_ppu_write(0x0123, 0x35);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0xFFFE, 0);
    CHECK(cart_ppu_read(0x0123) == 0xB7);
    return 0;
}

static void mmc5_enter_frame(uint8_t *nt) {
    (void)cart_nt_read(0x2000, nt);
    (void)cart_nt_read(0x2000, nt);
    (void)cart_nt_read(0x2000, nt);
    (void)cart_nt_read(0x23C0, nt);
    (void)cart_nt_read(0x2001, nt);
}

static void mmc5_next_scanline(uint8_t *nt) {
    (void)cart_nt_read(0x2000, nt);
    (void)cart_nt_read(0x2000, nt);
    (void)cart_nt_read(0x2000, nt);
    (void)cart_nt_read(0x23C0, nt);
}

static int test_mmc5_memory_windows(void) {
    CHECK(fixture(5, 0x20000, 0x8000, false) == 5);
    CHECK(cart_cpu_read(0xE000) == 15);
    cart_cpu_write(0x5102, 2);
    cart_cpu_write(0x5103, 1);
    cart_cpu_write(0x6000, 0xA5);
    CHECK(cart_cpu_read(0x8000) == 0xA5);
    cart_cpu_write(0x8000, 0x5A);
    CHECK(cart_cpu_read(0x6000) == 0x5A);
    cart_cpu_write(0x5102, 0);
    cart_cpu_write(0x8000, 0x11);
    CHECK(cart_cpu_read(0x8000) == 0x5A);
    cart_cpu_write(0x5102, 2);
    cart_cpu_write(0x5114, 0x82);
    cart_cpu_write(0x8000, 0x11);
    CHECK(cart_cpu_read(0x8000) == 2 && cart_cpu_read(0x6000) == 0x5A);
    cart_cpu_write(0x5117, 3);
    CHECK(cart_cpu_read(0xE000) == 3); // $5117 cannot select RAM.
    cart_cpu_write(0x5100, 0);
    cart_cpu_write(0x5117, 8);
    CHECK(cart_cpu_read(0x8000) == 8 && cart_cpu_read(0xE000) == 11);
    cart_cpu_write(0x5100, 1);
    cart_cpu_write(0x5115, 0x84);
    CHECK(cart_cpu_read(0x8000) == 4 && cart_cpu_read(0xA000) == 5);
    CHECK(cart_cpu_read(0xC000) == 8 && cart_cpu_read(0xE000) == 9);
    cart_cpu_write(0x5115, 0);
    CHECK(cart_cpu_read(0x8000) == 0x5A && cart_cpu_read(0xA000) == 0);
    cart_cpu_write(0x5100, 2);
    cart_cpu_write(0x5115, 0x84);
    cart_cpu_write(0x5116, 0);
    CHECK(cart_cpu_read(0x8000) == 4 && cart_cpu_read(0xA000) == 5);
    CHECK(cart_cpu_read(0xC000) == 0x5A && cart_cpu_read(0xE000) == 8);
    return 0;
}

static int test_mmc5_exram_and_irq(void) {
    CHECK(fixture(5, 0x20000, 0x8000, false) == 5);
    uint8_t nt[0x1000] = {0};
    cart_cpu_write(0x5104, 2);
    cart_cpu_write(0x5C00, 0xA5);
    CHECK(cart_cpu_read(0x5C00) == 0xA5);
    cart_cpu_write(0x5104, 3);
    cart_cpu_write(0x5C00, 0x11);
    CHECK(cart_cpu_read(0x5C00) == 0xA5);
    cart_cpu_write(0x5104, 0);
    cart_cpu_write(0x5C00, 0x55);
    cart_cpu_write(0x5104, 2);
    CHECK(cart_cpu_read(0x5C00) == 0);
    mmc5_enter_frame(nt);
    cart_cpu_write(0x5104, 0);
    cart_cpu_write(0x5C00, 0x55);
    cart_cpu_write(0x5104, 2);
    CHECK(cart_cpu_read(0x5C00) == 0x55);
    cart_cpu_write(0x5203, 1);
    cart_cpu_write(0x5204, 0x80);
    mmc5_next_scanline(nt);
    CHECK(cart_irq_pending());
    CHECK(cart_cpu_read(0x5204) == 0xC0 && !cart_irq_pending());
    cart_cpu_write(0x5203, 2);
    mmc5_next_scanline(nt);
    CHECK(cart_irq_pending());
    (void)cart_cpu_read(0xFFFA);
    CHECK(!cart_irq_pending() && cart_cpu_read(0x5204) == 0);
    mmc5_enter_frame(nt);
    CHECK(cart_cpu_read(0x5204) == 0x40);
    CHECK(cart != NULL && cart->clock != NULL);
    cart->clock(2);
    CHECK(cart_cpu_read(0x5204) == 0x40);
    cart->clock(1);
    CHECK(cart_cpu_read(0x5204) == 0);
    cart_cpu_write(0x5105, 0xFF);
    cart_cpu_write(0x5106, 0x37);
    cart_cpu_write(0x5107, 2);
    CHECK(cart_nt_read(0x2000, nt) == 0x37 && cart_nt_read(0x23C0, nt) == 0xAA);
    cart_cpu_write(0x5205, 0xFF);
    cart_cpu_write(0x5206, 0xFF);
    CHECK(cart_cpu_read(0x5205) == 1 && cart_cpu_read(0x5206) == 0xFE);
    return 0;
}

static int test_mmc5_chr_fetch_modes(void) {
    CHECK(fixture(5, 0x20000, 0x20000, false) == 5);
    cart_set_ppu_fetch_source(CART_PPU_FETCH_CPU);
    CHECK(cart_ppu_read(0x0000) == 0 && cart_ppu_read(0x1FFF) == 7);

    cart_cpu_write(0x5101, 3);
    cart_cpu_write(0x5120, 4);
    cart_cpu_write(0x5127, 11);
    cart_cpu_write(0x5128, 20);
    cart_cpu_write(0x5129, 21);
    cart_cpu_write(0x512A, 22);
    cart_cpu_write(0x512B, 23);
    CHECK(cart_ppu_read(0x0000) == 4); // 8x8 mode forces set A even after a B write.
    write_mem(0x2000, 0x20);
    CHECK(cart_ppu_read(0x0000) == 4); // The 8x8 B write must not reappear later.
    cart_cpu_write(0x5128, 20);
    cart_cpu_write(0x5129, 21);
    cart_cpu_write(0x512A, 22);
    cart_cpu_write(0x512B, 23);
    CHECK(cart_ppu_read(0x0000) == 20 && cart_ppu_read(0x1000) == 20);
    write_mem(0x2008, 0); // PPU sees the mirror; MMC5 still sees large sprites.
    CHECK(ppu.ctrl == 0 && cart_ppu_read(0x0000) == 20);
    cart_set_ppu_fetch_source(CART_PPU_FETCH_BG);
    CHECK(cart_ppu_read(0x0C00) == 23);
    cart_set_ppu_fetch_source(CART_PPU_FETCH_SPRITE);
    CHECK(cart_ppu_read(0x0000) == 4 && cart_ppu_read(0x1C00) == 11);
    cart_set_ppu_fetch_source(CART_PPU_FETCH_CPU);
    write_mem(0x2000, 0);
    CHECK(cart_ppu_read(0x0000) == 4);
    write_mem(0x3FF8, 0x20);
    CHECK(ppu.ctrl == 0x20 && cart_ppu_read(0x0000) == 4);
    write_mem(0x2000, 0);

    cart_cpu_write(0x5101, 0);
    cart_cpu_write(0x5127, 2);
    cart_set_ppu_fetch_source(CART_PPU_FETCH_SPRITE);
    CHECK(cart_ppu_read(0x0000) == 16 && cart_ppu_read(0x1C00) == 23);
    cart_cpu_write(0x5101, 1);
    cart_cpu_write(0x5123, 3);
    cart_cpu_write(0x5127, 5);
    CHECK(cart_ppu_read(0x0000) == 12 && cart_ppu_read(0x1000) == 20);
    cart_cpu_write(0x5101, 2);
    cart_cpu_write(0x5121, 2);
    cart_cpu_write(0x5123, 4);
    CHECK(cart_ppu_read(0x0000) == 4 && cart_ppu_read(0x0800) == 8);
    return 0;
}

static int test_mmc5_extended_rendering(void) {
    CHECK(fixture(5, 0x20000, 0x20000, false) == 5);
    uint8_t nt[0x1000] = {0};
    nt[5] = 0x2A;
    cart_cpu_write(0x5104, 2);
    cart_cpu_write(0x5C05, 0x83); // Palette 2, 4KB CHR bank 3.
    mmc5_enter_frame(nt);
    cart_cpu_write(0x5104, 1);
    CHECK(cart_nt_read(0x2005, nt) == 0x2A);
    CHECK(cart_nt_read(0x23C0, nt) == 0xAA);
    CHECK(cart_ppu_read(0x0123) == 12);
    CHECK(cart_ppu_read(0x012B) == 12);
    CHECK(cart_ppu_read(0x0123) == 0);
    cart_cpu_write(0x5104, 2);
    cart_cpu_write(0x5C00, 0x5A);
    cart_cpu_write(0x5C01, 0x6B);
    cart_cpu_write(0x5FC0, 0x02);
    fixture_chr[0x4000 + 0x120] = 0x90;
    fixture_chr[0x4000 + 0x121] = 0x91;
    cart_cpu_write(0x5200, 0x84); // Left-side split through column 3.
    cart_cpu_write(0x5201, 0);
    cart_cpu_write(0x5202, 4);
    cart_cpu_write(0x5104, 0);
    uint8_t split_tile = 0;
    for (unsigned i = 0; i < 46; ++i)
        split_tile = cart_nt_read((uint16_t)(0x2100 + (i & 0x1Fu)), nt);
    CHECK(split_tile == 0x5A);
    CHECK(cart_nt_read(0x23C0, nt) == 0xAA);
    CHECK(cart_ppu_read(0x0123) == 0x90);
    CHECK(cart_nt_read(0x211E, nt) == 0x6B);
    CHECK(cart_ppu_read(0x0123) == 0x91);
    cart->clock(3);
    CHECK((cart_cpu_read(0x5204) & 0x40) == 0);
    CHECK(cart_ppu_read(0x0123) == 0);

    CHECK(fixture(5, 0x20000, 0x20000, false) == 5);
    uint8_t repeated_nt[0x1000] = {0};
    repeated_nt[5] = 0x2A;
    cart_cpu_write(0x5104, 2);
    cart_cpu_write(0x5C05, 0x83);
    mmc5_enter_frame(repeated_nt);
    cart_cpu_write(0x5104, 1);
    fixture_chr[0x3000 + 0x3C0] = 0x6D;
    CHECK(cart_nt_read(0x2005, repeated_nt) == 0x2A);
    CHECK(cart_nt_read(0x23C0, repeated_nt) == 0xAA);
    CHECK(cart_nt_read(0x23C0, repeated_nt) == 0x6D);
    CHECK(cart_nt_read(0x23C0, repeated_nt) == 0x6D);
    return 0;
}

static void mmc5_prepare_render(void) {
    ppu_power_on(&ppu);
    ppu.scanline = (int)nes_timing()->scanlines - 1;
    ppu.dot = 0;
    ppu.v = 0;
    ppu.t = 0;
    ppu.x = 0;
    ppu.ctrl = 0;
    ppu.mask = 0x0A; // Background, including the left eight pixels.
    ppu.rendering_enabled = true;
    ppu.fetches_enabled = true;
    ppu_palette[0] = 0x0F;
    ppu_begin_frame_render(framebuffer);
}

static int test_mmc5_rendered_ppu_paths(void) {
    CHECK(fixture(5, 0x20000, 0x20000, false) == 5);
    memset(fixture_chr, 0, sizeof(fixture_chr));
    mmc5_prepare_render();
    for (unsigned tile = 0; tile < 64; ++tile) ppu_vram[tile] = 1;
    for (unsigned row = 0; row < 8; ++row) {
        fixture_chr[0x3000 + 0x10 + row] = 0xFF;
        fixture_chr[0x3000 + 0x18 + row] = 0;
    }
    cart_cpu_write(0x5104, 2);
    for (unsigned tile = 0; tile < 0x3C0; ++tile)
        cart_cpu_write((uint16_t)(0x5C00 + tile), 0xC3); // Palette 3, 4KB bank 3.
    cart_cpu_write(0x5104, 1);
    cart_cpu_write(0x5203, 1);
    cart_cpu_write(0x5204, 0x80);
    ppu_palette[13] = 0x21;
    ppu_palette[14] = 0x16;
    ppu_palette[15] = 0x30;
    ppu_step_dots(341 * 3);
    CHECK(framebuffer[1 * 256] == get_color(0x21));
    CHECK(framebuffer[1 * 256 + 7] == get_color(0x21));
    CHECK(bg_opaque[1 * 256] && bg_opaque[1 * 256 + 7]);
    CHECK(cart_irq_pending());
    CHECK((cart_cpu_read(0x5204) & 0xC0) == 0xC0);

    CHECK(fixture(5, 0x20000, 0x20000, false) == 5);
    memset(fixture_chr, 0, sizeof(fixture_chr));
    mmc5_prepare_render();
    for (unsigned tile = 0; tile < 0x3C0; ++tile) ppu_vram[tile] = 1;
    for (unsigned row = 0; row < 8; ++row) {
        fixture_chr[0x0010 + row] = 0xFF;          // Normal tile 1.
        fixture_chr[0x0018 + row] = 0;
        fixture_chr[0x4000 + 0x20 + row] = 0xFF; // Split tile 2 in bank 4.
        fixture_chr[0x4000 + 0x28 + row] = 0;
    }
    cart_cpu_write(0x5104, 2);
    for (unsigned tile = 0; tile < 0x3C0; ++tile)
        cart_cpu_write((uint16_t)(0x5C00 + tile), 2);
    for (unsigned attr = 0x3C0; attr < 0x400; ++attr)
        cart_cpu_write((uint16_t)(0x5C00 + attr), 0x55); // Split palette 1.
    cart_cpu_write(0x5104, 0);
    cart_cpu_write(0x5200, 0x88); // Left-side split through tile column 7.
    cart_cpu_write(0x5201, 0);
    cart_cpu_write(0x5202, 4);
    ppu_palette[1] = 0x11; // Normal palette 0, pixel 1.
    ppu_palette[5] = 0x21; // Split palette 1, pixel 1.
    ppu_step_dots(341 * 3);
    CHECK(framebuffer[1 * 256 + 63] == get_color(0x21));
    CHECK(framebuffer[1 * 256 + 64] == get_color(0x11));
    CHECK(bg_opaque[1 * 256 + 63] && bg_opaque[1 * 256 + 64]);

    CHECK(fixture(5, 0x20000, 0x20000, false) == 5);
    memset(fixture_chr, 0, sizeof(fixture_chr));
    mmc5_prepare_render();
    for (unsigned tile = 0; tile < 0x3C0; ++tile) ppu_vram[tile] = 1;
    for (unsigned row = 0; row < 8; ++row) {
        fixture_chr[0x0010 + row] = 0xFF;
        fixture_chr[0x4000 + 0x20 + row] = 0xFF;
    }
    cart_cpu_write(0x5104, 2);
    for (unsigned tile = 0; tile < 0x3C0; ++tile)
        cart_cpu_write((uint16_t)(0x5C00 + tile), 2);
    for (unsigned attr = 0x3C0; attr < 0x400; ++attr)
        cart_cpu_write((uint16_t)(0x5C00 + attr), 0x55);
    cart_cpu_write(0x5104, 0);
    cart_cpu_write(0x5200, 0xC8); // Same boundary, split on the right side.
    cart_cpu_write(0x5202, 4);
    ppu_palette[1] = 0x11;
    ppu_palette[5] = 0x21;
    ppu_step_dots(341 * 3);
    CHECK(framebuffer[1 * 256 + 63] == get_color(0x11));
    CHECK(framebuffer[1 * 256 + 64] == get_color(0x21));

    CHECK(fixture(5, 0x20000, 0x20000, false) == 5);
    memset(fixture_chr, 0, sizeof(fixture_chr));
    uint8_t physical_nt[0x1000] = {0};
    cart_cpu_write(0x5104, 2);
    for (unsigned tile = 0; tile < 0x3C0; ++tile)
        cart_cpu_write((uint16_t)(0x5C00 + tile), 0xC3); // Palette 3, 4KB bank 3.
    mmc5_enter_frame(physical_nt);
    // Place the mapper immediately before the final sprite slot's two dummy
    // nametable reads without tripping its three-identical-read scanline detector.
    for (unsigned read = 0; read < 45; ++read)
        (void)cart_nt_read((uint16_t)(0x2000 + (read & 1u)), physical_nt);
    cart_cpu_write(0x5104, 1);
    for (unsigned row = 0; row < 8; ++row) fixture_chr[0x3008 + row] = 0x5A;
    ppu_power_on(&ppu);
    memset(ppu.secondary_oam, 0, sizeof(ppu.secondary_oam));
    ppu.scanline = 0;
    ppu.dot = 313;
    ppu.mask = 0x18;
    ppu.rendering_enabled = true;
    ppu.fetches_enabled = true;
    // The first dummy read remains in the sprite CHR-A window; the second moves
    // the mapper out of it and arms extended attributes. The following physical
    // CHR reads therefore consume the palette slot and then the selected bank.
    ppu_step_dots(8);
    CHECK(ppu.dot == 321);
    CHECK(ppu.sprite_pattern_lo[7] == 0xFF);
    CHECK(ppu.sprite_pattern_hi[7] == 0x5A);
    return 0;
}

static int test_mmc5_audio_and_pcm(void) {
    CHECK(fixture(5, 0x20000, 0x8000, false) == 5);
    CHECK(cart != NULL && cart->clock != NULL && cart_expansion_audio() == 0.0f);

    // The fixed envelope/length divider clocks immediately, then every floor(CPU/240) cycles.
    // A high-register write stages its length reload until the end of the next CPU clock.
    int frame_period = (int)(nes_timing()->cpu_hz / 240.0);
    cart_cpu_write(0x5015, 0x01);
    cart_cpu_write(0x5000, 0x10);
    cart_cpu_write(0x5002, 0);
    cart_cpu_write(0x5003, 0x18); // Length-table entry 3 starts at 2.
    CHECK((cart_cpu_read(0x5015) & 1) == 0);
    cart->clock(1);
    CHECK((cart_cpu_read(0x5015) & 1) != 0);
    cart->clock(frame_period);
    CHECK((cart_cpu_read(0x5015) & 1) != 0);
    cart->clock(frame_period);
    CHECK((cart_cpu_read(0x5015) & 1) == 0);

    // Reload and length-halt writes collide with the old counter on the next
    // CPU clock: the frame tick sees the old halt state, then reload arbitration
    // and the new halt state commit at the end of that clock.
    CHECK(fixture(5, 0x20000, 0x8000, false) == 5);
    cart_cpu_write(0x5015, 0x01);
    cart_cpu_write(0x5000, 0x10);
    cart_cpu_write(0x5003, 0x38); // Length 6.
    cart->clock(1);
    cart->clock(frame_period); // 6 -> 5.
    cart->clock(frame_period - 1);
    cart_cpu_write(0x5000, 0x30); // Stage halt.
    cart_cpu_write(0x5003, 0x18); // Stage length 2 with previous counter 5.
    cart->clock(1);               // Old counter becomes 4; reload loses collision.
    cart_cpu_write(0x5000, 0x10);
    cart->clock(1);               // Commit unhalt before the next frame tick.
    cart->clock(frame_period - 1); // 4 -> 3.
    cart->clock(frame_period);     // 3 -> 2.
    cart->clock(frame_period);     // 2 -> 1.
    CHECK((cart_cpu_read(0x5015) & 1) != 0);
    cart->clock(frame_period);     // 1 -> 0.
    CHECK((cart_cpu_read(0x5015) & 1) == 0);

    CHECK(fixture(5, 0x20000, 0x8000, false) == 5);
    cart_cpu_write(0x5015, 0x01);
    cart_cpu_write(0x5000, 0xDF); // Duty 3, constant volume 15.
    cart_cpu_write(0x5002, 0);
    cart_cpu_write(0x5003, 0x08); // Load length with a period below 8.
    cart->clock(1);
    CHECK(cart_expansion_audio() == 0.0f);
    cart->clock(4);
    float one_pulse = cart_expansion_audio();
    CHECK(one_pulse < 0.0f); // MMC5 pulses are not muted for periods below 8.
    CHECK(one_pulse < -0.125f && one_pulse > -0.127f);
    cart_cpu_write(0x5001, 0xFF); // No sweep unit.
    CHECK(cart_expansion_audio() == one_pulse);

    CHECK(fixture(5, 0x20000, 0x8000, false) == 5);
    cart_cpu_write(0x5015, 0x03);
    cart_cpu_write(0x5000, 0xDF);
    cart_cpu_write(0x5002, 0);
    cart_cpu_write(0x5003, 0x08);
    cart_cpu_write(0x5004, 0xDF);
    cart_cpu_write(0x5006, 0);
    cart_cpu_write(0x5007, 0x08);
    cart->clock(5);
    CHECK(cart_expansion_audio() < one_pulse);
    CHECK(cart_expansion_audio() < -0.251f && cart_expansion_audio() > -0.253f);
    cart_cpu_write(0x5015, 0);
    cart->clock(2);
    CHECK(cart_expansion_audio() == 0.0f);

    cart_cpu_write(0x5010, 0x00);
    cart_cpu_write(0x5011, 0);
    CHECK(!cart_irq_pending()); // Trip state is retained while IRQ output is disabled.
    cart_cpu_write(0x5010, 0x80);
    CHECK(cart_irq_pending());
    cart_cpu_write(0x5011, 0x40);
    CHECK(!cart_irq_pending()); // A nonzero DAC assignment clears the trip state.
    cart_cpu_write(0x5010, 0x00);
    cart_cpu_write(0x5011, 0);
    CHECK(!cart_irq_pending());
    // Status masks disabled IRQs but still acknowledges the trip.
    CHECK(cart_cpu_read(0x5010) == 0x01);
    cart_cpu_write(0x5010, 0x80);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0x5011, 0);
    CHECK(cart_irq_pending());
    CHECK(cart_cpu_read(0x5010) == 0x81 && !cart_irq_pending());
    cart_cpu_write(0x5011, 0x80);
    float pcm = cart_expansion_audio();
    CHECK(pcm < -0.357f && pcm > -0.360f);
    cart_cpu_write(0x5010, 0x81);
    cart_cpu_write(0x5011, 0x20); // Direct DAC writes are ignored in read mode.
    CHECK(cart_expansion_audio() == pcm);
    cart_cpu_write(0x5114, 0x81);
    CHECK(cart_cpu_read(0x8000) == 1 && !cart_irq_pending());
    float pcm_one = cart_expansion_audio();
    CHECK(pcm_one < 0.0f && pcm_one > pcm);
    cart_cpu_write(0x5114, 0x80);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_irq_pending());
    CHECK(cart_expansion_audio() == pcm_one);
    CHECK(cart_cpu_read(0x5010) == 0x81 && !cart_irq_pending());
    return 0;
}

#endif // MAPPER_ACCURACY_CONFLICTS_H

