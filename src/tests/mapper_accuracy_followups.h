/*
 * mapper_accuracy_followups.h - Mapper edge case regression tests
 *
 * Author: @frankischilling
 *
 * These checks cover mapper timing, banking, and bus edge cases found during follow-up
 * testing.
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
#ifndef MAPPER_ACCURACY_FOLLOWUPS_H
#define MAPPER_ACCURACY_FOLLOWUPS_H

static int discrete_cpu_store(uint16_t address, uint8_t value) {
    const uint8_t program[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (size_t i = 0; i < sizeof(program); ++i)
        write_mem((uint16_t)(0x0200 + i), program[i]);
    cpu.pc = 0x0200;
    CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int discrete_cpu_load(uint16_t address, uint8_t expected) {
    write_mem(0x0300, 0xAD);
    write_mem(0x0301, (uint8_t)address);
    write_mem(0x0302, (uint8_t)(address >> 8));
    cpu.pc = 0x0300;
    CHECK(cpu_step(&cpu) == 4 && cpu.a == expected);
    return 0;
}

static int test_colordreams_high_banks_and_mapper144(void) {
    const unsigned boards[] = {11, 144};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        CHECK(fixture(boards[i], 0x80000, 0x20000, false) == (int)boards[i]);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        CHECK(cpu_power_on(&cpu));
        fixture_prg[0x123] = 0xED;
        CHECK(discrete_cpu_store(0x8123, 0xDD) == 0);
        CHECK(discrete_cpu_load(0x8100, 52) == 0); // PRG bank 13, including D2 and D3.
        CHECK(ppu_read(0x0123) == 96); // CHR bank 12 after the ROM bus conflict.
        CHECK(fixture_prg[0x123] == 0xED);

        fixture_prg[13 * 0x8000 + 0x123] = 0xFE;
        CHECK(discrete_cpu_store(0x8123, 0) == 0);
        CHECK(discrete_cpu_load(0x8100, 0) == 0);
        CHECK(ppu_read(0x0123) == 0);

        // Mapper 144 takes D0 from the ROM even when the CPU writes zero.
        fixture_prg[0x123] = 1;
        CHECK(discrete_cpu_store(0x8123, 0) == 0);
        CHECK(discrete_cpu_load(0x8100, boards[i] == 144 ? 4 : 0) == 0);
        fixture_prg[(boards[i] == 144 ? 0x8000 : 0) + 0x123] = 0xF0;
        CHECK(discrete_cpu_store(0x8123, 0xFF) == 0);
        CHECK(discrete_cpu_load(0x8100, 0) == 0);
        CHECK(ppu_read(0x0123) == 120);
        ppu_write(0x0123, 0xA5);
        CHECK(ppu_read(0x0123) == 120);
        ppu_soft_reset(&ppu);
        apu_soft_reset(&apu);
        cpu_soft_reset(&cpu);
        CHECK(ppu_read(0x0123) == 120);
        cart->reset();
        CHECK(ppu_read(0x0123) == 0);
    }
    return 0;
}

static int test_unrom94_180_cpu_banks(void) {
    const unsigned boards[] = {94, 180};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        CHECK(fixture(boards[i], i ? 0x100000 : 0x20000, 0x2000, false) == (int)boards[i]);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        CHECK(cpu_power_on(&cpu));
        CHECK(discrete_cpu_load(0x8100, 0) == 0);
        CHECK(discrete_cpu_load(0xC100, i ? 0 : 14) == 0);
        fixture_prg[0x123] = 0; // Register writes are not ANDed with PRG ROM.
        CHECK(discrete_cpu_store(0x8123, 0xB4) == 0);
        CHECK(discrete_cpu_load(0x8100, i ? 0 : 10) == 0);
        CHECK(discrete_cpu_load(0xC100, i ? 104 : 14) == 0);
        CHECK(discrete_cpu_store(0xFFFF, 0x23) == 0);
        CHECK(discrete_cpu_load(0x8100, 0) == 0);
        CHECK(discrete_cpu_load(0xC100, i ? 70 : 14) == 0);
        ppu_soft_reset(&ppu);
        apu_soft_reset(&apu);
        cpu_soft_reset(&cpu);
        CHECK(discrete_cpu_load(0xC100, i ? 70 : 14) == 0);
        ppu_write(0x0123, 0xA5);
        CHECK(ppu_read(0x0123) == 0);

        CHECK(fixture(boards[i], 0x8000, 0x2000, true) == (int)boards[i]);
        ppu_write(0x1FFF, 0xA6);
        cart_cpu_write(0x8000, 0xFF);
        CHECK(ppu_read(0x1FFF) == 0xA6);
        cart->reset();
        CHECK(cart_cpu_read(0xC100) == (i ? 0 : 2));
    }
    return 0;
}

static int test_nina_cpu_decode_and_mirroring(void) {
    const unsigned boards[] = {79, 113, 146};
    const uint16_t ignored[] = {0x4200, 0x5000, 0x5EFF, 0x6000, 0x8000, 0xFFFF};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        bool multicart = boards[i] == 113;
        CHECK(fixture(boards[i], multicart ? 0x40000 : 0x10000,
                      multicart ? 0x20000 : 0x10000, false) == (int)boards[i]);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        CHECK(cpu_power_on(&cpu));
        CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
        CHECK(discrete_cpu_store(0x4100, 0xCB) == 0);
        CHECK(discrete_cpu_load(0x8100, 4) == 0);
        CHECK(ppu_read(0x0123) == (multicart ? 88 : 24));
        CHECK(cart_get_mirroring() == (multicart ? MIRROR_VERTICAL : MIRROR_HORIZONTAL));
        ppu_write(0x2000, 0xA1);
        ppu_write(0x2400, 0xB2);
        CHECK(ppu_read(0x2000) == (multicart ? 0xA1 : 0xB2));
        if (multicart) CHECK(ppu_read(0x2800) == 0xA1);
        for (size_t j = 0; j < sizeof(ignored) / sizeof(ignored[0]); ++j) {
            CHECK(discrete_cpu_store(ignored[j], 0) == 0);
            CHECK(discrete_cpu_load(0x8100, 4) == 0);
            CHECK(ppu_read(0x0123) == (multicart ? 88 : 24));
        }
        CHECK(discrete_cpu_store(0x5FFF, 0x7C) == 0);
        CHECK(discrete_cpu_load(0x8100, multicart ? 28 : 4) == 0);
        CHECK(ppu_read(0x0123) == (multicart ? 96 : 32));
        CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
        CHECK(discrete_cpu_store(0x4301, 0x48) == 0);
        CHECK(discrete_cpu_load(0x8100, 4) == 0);
        CHECK(ppu_read(0x0123) == (multicart ? 64 : 0));
        ppu_write(0x0123, 0xA6);
        CHECK(ppu_read(0x0123) == (multicart ? 64 : 0));
        ppu_soft_reset(&ppu);
        apu_soft_reset(&apu);
        cpu_soft_reset(&cpu);
        CHECK(discrete_cpu_load(0x8100, 4) == 0);
        CHECK(ppu_read(0x0123) == (multicart ? 64 : 0));
        cart->reset();
        CHECK(cart_cpu_read(0x8100) == 0 && ppu_read(0x0123) == 0);
    }
    return 0;
}

static int test_nina_chr_ram_banks(void) {
    const unsigned boards[] = {79, 113, 146};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        bool multicart = boards[i] == 113;
        iNESHeader h = header_for(boards[i], 0x10000, true);
        h.flags7 |= 8;
        h.zero[0] = multicart ? 11 : 10;
        CHECK(fixture_with_header(&h, 0x10000, multicart ? 0x20000 : 0x10000) == (int)boards[i]);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        CHECK(cpu_power_on(&cpu));
        CHECK(discrete_cpu_store(0x4100, 0x41) == 0);
        ppu_write(0x0123, 0xA5);
        CHECK(discrete_cpu_store(0x4100, 0) == 0);
        CHECK(ppu_read(0x0123) == 0);
        ppu_write(0x0123, 0x5A);
        CHECK(discrete_cpu_store(0x4100, 0x41) == 0);
        CHECK(ppu_read(0x0123) == 0xA5);
        ppu_soft_reset(&ppu);
        apu_soft_reset(&apu);
        cpu_soft_reset(&cpu);
        CHECK(ppu_read(0x0123) == 0xA5);
        cart->reset();
        CHECK(ppu_read(0x0123) == 0x5A);
    }
    return 0;
}

static int test_discrete_followup_loader_and_ram(void) {
    const unsigned boards[] = {79, 94, 113, 144, 146, 180};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        for (unsigned layout = 0; layout < 4; ++layout) {
            iNESHeader h = header_for(boards[i], 0x8000, false);
            h.flags6 |= 1;
            if (layout == 0) h.flags6 |= 4;
            else {
                h.flags7 |= 8;
                h.flags10 = layout == 1 ? 0 : layout == 2 ? 7 : 5;
            }
            size_t size;
            uint8_t *image = image_for(&h, 0x8000, 0x2000, &size);
            CHECK(image != NULL);
            size_t offset = sizeof(h) + (layout == 0 ? 512 : 0);
            image[offset + 0x3FFC] = image[offset + 0x7FFC] = 0x00;
            image[offset + 0x3FFD] = image[offset + 0x7FFD] = 0x81;
            CHECK(load_rom_memory(image, size) == 0);
            CHECK(rom_mapper_number(&ines_header) == (int)boards[i]);
            CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
            ppu_power_on(&ppu);
            apu_power_on(&apu);
            CHECK(cpu_power_on(&cpu) && cpu.pc == 0x8100);
            CHECK(discrete_cpu_load(0x7123, layout == 0 ? 0x23 : layout == 1 ? 0x71 : 0) == 0);
            CHECK(discrete_cpu_store(0x6000, 0x37) == 0);
            CHECK(discrete_cpu_load(0x6000, layout == 1 ? 0x60 : 0x37) == 0);
            CHECK(discrete_cpu_store(0x6800, 0xA6) == 0);
            CHECK(discrete_cpu_load(0x6000, layout == 1 ? 0x60 : layout == 3 ? 0xA6 : 0x37) == 0);
            CHECK(discrete_cpu_load(0x6800, layout == 1 ? 0x68 : 0xA6) == 0);
            CHECK(discrete_cpu_store(0x7FFF, 0xD5) == 0);
            CHECK(discrete_cpu_load(0x7FFF, layout == 1 ? 0x7F : 0xD5) == 0);
            CHECK(discrete_cpu_load(0x5FFF, 0x5F) == 0);
            ppu_soft_reset(&ppu);
            apu_soft_reset(&apu);
            cpu_soft_reset(&cpu);
            CHECK(discrete_cpu_load(0x7FFF, layout == 1 ? 0x7F : 0xD5) == 0);

            Mapper *previous = cart;
            iNESHeader active = ines_header;
            CHECK(load_rom_memory(image, size - 1) == -1);
            uint8_t signature = image[0];
            image[0] = 0;
            CHECK(load_rom_memory(image, size) == -1);
            image[0] = signature;
            iNESHeader invalid_ram = h;
            invalid_ram.flags7 |= 8;
            invalid_ram.flags6 &= (uint8_t)~2u;
            invalid_ram.flags10 = 0x77;
            memcpy(image, &invalid_ram, sizeof(invalid_ram));
            // A save chip requires the battery flag, even when separate work
            // and save chips are supported by the board's fixed RAM window.
            CHECK(load_rom_memory(image, size) == -1);
            free(image);
            CHECK(cart == previous && memcmp(&active, &ines_header, sizeof(active)) == 0);
            CHECK(discrete_cpu_load(0x7FFF, layout == 1 ? 0x7F : 0xD5) == 0);
            CHECK(unload_rom());
        }
    }
    return 0;
}

static int test_mapper180_full_prg_range(void) {
    iNESHeader h = header_for(180, 0x400000, true);
    size_t size;
    uint8_t *image = image_for(&h, 0x400000, 0, &size);
    CHECK(image != NULL);
    for (unsigned bank = 0; bank < 256; ++bank) {
        uint8_t *page = image + sizeof(h) + bank * 0x4000;
        memset(page, (int)bank, 0x4000);
        page[0x3FFC] = 0x00;
        page[0x3FFD] = 0x81;
    }
    int loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu) && cpu.pc == 0x8100);
    CHECK(discrete_cpu_load(0xC100, 0) == 0);
    CHECK(discrete_cpu_store(0x8000, 0xC7) == 0);
    CHECK(discrete_cpu_load(0xC100, 0xC7) == 0);
    CHECK(discrete_cpu_load(0x8100, 0) == 0);
    CHECK(discrete_cpu_store(0xFFFF, 0x80) == 0);
    CHECK(discrete_cpu_load(0xC100, 0x80) == 0);
    ppu_soft_reset(&ppu);
    apu_soft_reset(&apu);
    cpu_soft_reset(&cpu);
    CHECK(cpu.pc == 0x8100);
    CHECK(discrete_cpu_load(0xC100, 0x80) == 0);
    CHECK(unload_rom());
    return 0;
}

static int test_uxrom94_180_sub16_prg_geometry(void) {
    const unsigned boards[] = {94, 180};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        iNESHeader h = header_for(boards[i], 0x8000, false);
        h.flags7 |= 8;
        h.prg_rom_chunks = 0x31; // 3 * 2^12 = 12 KiB.
        h.chr_rom_chunks = 0x30; // 1 * 2^12 = 4 KiB.
        h.flags9 = 0xFF;
        size_t size;
        uint8_t *image = image_for(&h, 0x3000, 0x1000, &size);
        CHECK(image != NULL);
        uint8_t *prg = image + sizeof(h);
        uint8_t *chr = prg + 0x3000;
        memset(prg, 0x21, 0x1000);
        memset(prg + 0x1000, 0x42, 0x1000);
        memset(prg + 0x2000, 0x63, 0x1000);
        memset(chr, 0xA3, 0x1000);

        CHECK(load_rom_memory(image, size) == 0);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        CHECK(cpu_power_on(&cpu));
        CHECK(discrete_cpu_load(0x8100, 0x21) == 0);
        CHECK(discrete_cpu_load(0x9100, 0x42) == 0);
        CHECK(discrete_cpu_load(0xA100, 0x63) == 0);
        CHECK(discrete_cpu_load(0xB100, 0x21) == 0);
        CHECK(discrete_cpu_load(0xC100, 0x42) == 0);
        CHECK(discrete_cpu_load(0xD100, 0x63) == 0);
        CHECK(discrete_cpu_load(0xE100, 0xE1) == 0);
        CHECK(ppu_read(0x0123) == 0xA3 && ppu_read(0x1123) == 0x23);

        CHECK(discrete_cpu_store(0x8123, i ? 0xA5 : 0x1C) == 0);
        CHECK(discrete_cpu_load(0x8100, 0x21) == 0);
        CHECK(discrete_cpu_load(0xC100, 0x42) == 0);
        CHECK(discrete_cpu_load(0xE100, 0xE1) == 0);

        Mapper *previous = cart;
        uint8_t *previous_prg = prg_rom;
        CHECK(load_rom_memory(image, size - 1) == -1);
        CHECK(cart == previous && prg_rom == previous_prg);
        CHECK(discrete_cpu_load(0x8100, 0x21) == 0);
        CHECK(discrete_cpu_load(0xD100, 0x63) == 0);
        CHECK(discrete_cpu_load(0xE100, 0xE1) == 0);
        CHECK(ppu_read(0x0123) == 0xA3 && ppu_read(0x1123) == 0x23);
        free(image);
        CHECK(unload_rom());
    }
    return 0;
}

static int test_native_8k_prg_page_geometry(void) {
    // MMC3 has a complete 8 KiB native page here. All four CPU slots therefore
    // wrap to that page even after a bank-register write.
    iNESHeader h = header_for(4, 0x4000, false);
    h.flags7 |= 8;
    h.prg_rom_chunks = 0x34; // 8 KiB.
    h.flags9 = 0x0F;
    size_t size;
    uint8_t *image = image_for(&h, 0x2000, 0x2000, &size);
    CHECK(image != NULL);
    memset(image + sizeof(h), 0x41, 0x2000);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_cpu_read(0x8100) == 0x41 && cart_cpu_read(0xA100) == 0x41);
    CHECK(cart_cpu_read(0xC100) == 0x41 && cart_cpu_read(0xE100) == 0x41);
    cart_cpu_write(0x8000, 6);
    cart_cpu_write(0x8001, 0x7F);
    CHECK(cart_cpu_read(0x8100) == 0x41 && cart_cpu_read(0xC100) == 0x41);

    // Jaleco 18 starts with its three switchable slots disconnected. A 12 KiB
    // image contributes one selectable 8 KiB page; the trailing 4 KiB cannot
    // form another hardware page and remains unreachable.
    h = header_for(18, 0x4000, false);
    h.flags7 |= 8;
    h.prg_rom_chunks = 0x31; // 3 * 2^12 = 12 KiB.
    h.flags9 = 0x0F;
    image = image_for(&h, 0x3000, 0x2000, &size);
    CHECK(image != NULL);
    uint8_t *prg = image + sizeof(h);
    memset(prg, 0x52, 0x2000);
    memset(prg + 0x2000, 0xA7, 0x1000);
    CHECK(load_rom_memory(image, size) == 0);
    CHECK(cart_cpu_read_bus(0x8100, 0xD1) == 0xD1);
    CHECK(cart_cpu_read_bus(0xA100, 0xD2) == 0xD2);
    CHECK(cart_cpu_read_bus(0xC100, 0xD3) == 0xD3);
    CHECK(cart_cpu_read(0xE100) == 0x52);
    cart_cpu_write(0x8000, 0x0F);
    cart_cpu_write(0x8001, 0x0F);
    CHECK(cart_cpu_read(0x8100) == 0x52);
    CHECK(cart_cpu_read(0xE100) == 0x52);

    Mapper *previous = cart;
    uint8_t *previous_prg = prg_rom;
    CHECK(load_rom_memory(image, size - 1) == -1);
    free(image);
    CHECK(cart == previous && prg_rom == previous_prg);
    CHECK(cart_cpu_read(0x8100) == 0x52 && cart_cpu_read(0xE100) == 0x52);
    CHECK(unload_rom());
    return 0;
}

static int test_native_reduced_prg_page_geometry(void) {
    // Native 16 KiB and 32 KiB page mappers shrink their PRG page to a smaller
    // physical image. Whole copies repeat through $8000-$FFFF; a partial copy
    // at the end of the CPU window stays open bus.
    const unsigned boards[] = {1, 3};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        iNESHeader h = header_for(boards[i], 0x4000, false);
        h.flags7 |= 8;
        h.prg_rom_chunks = 0x31; // 3 * 2^12 = 12 KiB.
        h.flags9 = 0x0F;
        size_t size;
        uint8_t *image = image_for(&h, 0x3000, 0x2000, &size);
        CHECK(image != NULL);
        uint8_t *prg = image + sizeof(h);
        memset(prg, 0x31, 0x1000);
        memset(prg + 0x1000, 0x52, 0x1000);
        memset(prg + 0x2000, 0x73, 0x1000);
        CHECK(load_rom_memory(image, size) == 0);
        free(image);

        ppu_power_on(&ppu);
        apu_power_on(&apu);
        CHECK(cpu_power_on(&cpu));
        CHECK(discrete_cpu_load(0x8100, 0x31) == 0);
        CHECK(discrete_cpu_load(0x9100, 0x52) == 0);
        CHECK(discrete_cpu_load(0xA100, 0x73) == 0);
        CHECK(discrete_cpu_load(0xB100, 0x31) == 0);
        CHECK(discrete_cpu_load(0xC100, 0x52) == 0);
        CHECK(discrete_cpu_load(0xD100, 0x73) == 0);
        CHECK(discrete_cpu_load(0xE100, 0xE1) == 0);

        if (boards[i] == 1) {
            serial_write(0xE000, 7);
        } else {
            cart_cpu_write(0x8000, 3);
        }
        CHECK(discrete_cpu_load(0x8100, 0x31) == 0);
        CHECK(discrete_cpu_load(0xD100, 0x73) == 0);
        CHECK(discrete_cpu_load(0xE100, 0xE1) == 0);
        CHECK(unload_rom());
    }

    // MMC5 has its own reduced-page read path because PCM read mode observes
    // cartridge reads. A 6 KiB image repeats five times; the final 2 KiB of
    // the 32 KiB CPU window remains open bus.
    iNESHeader h = header_for(5, 0x4000, false);
    h.flags7 |= 8;
    h.prg_rom_chunks = 0x2D; // 3 * 2^11 = 6 KiB.
    h.flags9 = 0x0F;
    size_t size;
    uint8_t *image = image_for(&h, 0x1800, 0x2000, &size);
    CHECK(image != NULL);
    uint8_t *prg = image + sizeof(h);
    memset(prg, 0x41, 0x0800);
    memset(prg + 0x0800, 0x62, 0x0800);
    memset(prg + 0x1000, 0x83, 0x0800);
    CHECK(load_rom_memory(image, size) == 0);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    CHECK(cart_cpu_read_bus(0x8100, 0xD1) == 0xD1);
    CHECK(discrete_cpu_load(0xF100, 0x83) == 0);
    CHECK(discrete_cpu_load(0xF900, 0xF9) == 0);
    cart_cpu_write(0x5114, 0xFF);
    cart_cpu_write(0x5117, 0x80);
    CHECK(discrete_cpu_load(0x8100, 0x41) == 0);
    CHECK(discrete_cpu_load(0x8900, 0x62) == 0);
    CHECK(discrete_cpu_load(0x9100, 0x83) == 0);
    CHECK(discrete_cpu_load(0x9900, 0x41) == 0);
    CHECK(discrete_cpu_load(0xF900, 0xF9) == 0);

    Mapper *previous = cart;
    uint8_t *previous_prg = prg_rom;
    CHECK(load_rom_memory(image, size - 1) == -1);
    free(image);
    CHECK(cart == previous && prg_rom == previous_prg);
    CHECK(discrete_cpu_load(0xF100, 0x83) == 0);
    CHECK(discrete_cpu_load(0xF900, 0xF9) == 0);
    CHECK(unload_rom());
    return 0;
}

static int test_remaining_native_geometry_caps(void) {
    size_t size;
    uint8_t *image;

    // Action 53 shrinks a 16 KiB native PRG page to a 12 KiB image. Two full
    // copies fit in the CPU window and the final 8 KiB remains open bus.
    iNESHeader h = action53_header(0x4000, 5);
    h.prg_rom_chunks = 0x31;
    h.flags9 = (uint8_t)((h.flags9 & 0xF0u) | 0x0Fu);
    image = image_for(&h, 0x3000, 0, &size);
    CHECK(image != NULL);
    uint8_t *prg = image + sizeof(h);
    memset(prg, 0x21, 0x1000);
    memset(prg + 0x1000, 0x42, 0x1000);
    memset(prg + 0x2000, 0x63, 0x1000);
    CHECK(load_rom_memory(image, size) == 0);
    ppu_power_on(&ppu); apu_power_on(&apu); CHECK(cpu_power_on(&cpu));
    CHECK(discrete_cpu_load(0x8100, 0x21) == 0);
    CHECK(discrete_cpu_load(0xD100, 0x63) == 0);
    CHECK(discrete_cpu_load(0xE100, 0xE1) == 0);
    cart_ppu_write(0x0123, 0x91);
    CHECK(cart_ppu_read(0x0123) == 0x91 && cart_ppu_read(0x0923) == 0x91);
    cart_cpu_write(0x5000, 1); cart_cpu_write(0x8000, 7);
    CHECK(discrete_cpu_load(0x8100, 0x21) == 0);
    Mapper *previous = cart; uint8_t *previous_prg = prg_rom;
    CHECK(load_rom_memory(image, size - 1) == -1);
    CHECK(cart == previous && prg_rom == previous_prg);
    free(image);

    // UNROM 512 accepts an irregular three-page image. Reads wrap complete
    // 16 KiB pages, while flash writes keep their raw physical bank address.
    h = unrom512_header(1, true, 5);
    h.prg_rom_chunks = 0x39; h.flags9 = (uint8_t)((h.flags9 & 0xF0u) | 0x0Fu);
    image = image_for(&h, 0xC000, 0, &size);
    CHECK(image != NULL); prg = image + sizeof(h);
    memset(prg, 0x31, 0x4000); memset(prg + 0x4000, 0x52, 0x4000); memset(prg + 0x8000, 0x73, 0x4000);
    CHECK(load_rom_memory(image, size) == 0);
    ppu_power_on(&ppu); apu_power_on(&apu); CHECK(cpu_power_on(&cpu));
    cart_ppu_write(0x0123, 0xA2);
    CHECK(cart_ppu_read(0x0123) == 0xA2 && cart_ppu_read(0x0923) == 0xA2);
    cart_cpu_write(0xC000, 7);
    CHECK(discrete_cpu_load(0x8100, 0x52) == 0 && discrete_cpu_load(0xC100, 0x73) == 0);
    unrom512_flash_command(0xA0); cart_cpu_write(0xC000, 7); cart_cpu_write(0x8123, 0x00);
    CHECK(cart_cpu_read(0x8123) == 0x52);
    unrom512_flash_command(0xA0); cart_cpu_write(0xC000, 1); cart_cpu_write(0x8123, 0x00);
    CHECK(cart_cpu_read(0x8123) == 0x00);
    previous = cart; previous_prg = prg_rom;
    CHECK(load_rom_memory(image, size - 1) == -1 && cart == previous && prg_rom == previous_prg);
    free(image);

    // Oeka Kids accepts a non-128 KiB PRG image. The complete 32 KiB page is
    // banked normally and the trailing 16 KiB cannot form another page.
    h = m96_header(); h.zero[0] = 5; h.prg_rom_chunks = 0x39; h.flags9 = (uint8_t)((h.flags9 & 0xF0u) | 0x0Fu);
    image = image_for(&h, 0xC000, 0, &size); CHECK(image != NULL); prg = image + sizeof(h);
    memset(prg, 0x41, 0x8000); memset(prg + 0x8000, 0x82, 0x4000); prg[0x7FFE] = 0xFF;
    CHECK(load_rom_memory(image, size) == 0);
    ppu_power_on(&ppu); apu_power_on(&apu); CHECK(cpu_power_on(&cpu));
    CHECK(discrete_cpu_load(0x8100, 0x41) == 0 && discrete_cpu_load(0xE100, 0x41) == 0);
    CHECK(discrete_cpu_store(0xFFFE, 7) == 0 && discrete_cpu_load(0x8100, 0x41) == 0);
    cart_ppu_write(0x0123, 0xA6);
    CHECK(cart_ppu_read(0x0123) == 0xA6 && cart_ppu_read(0x0923) == 0xA6);
    previous = cart; previous_prg = prg_rom;
    CHECK(load_rom_memory(image, size - 1) == -1 && cart == previous && prg_rom == previous_prg);
    free(image);

    // GTROM has the same complete-page/read versus physical-flash distinction
    // with 32 KiB pages.
    h = m111_header(); h.zero[0] = 5; h.prg_rom_chunks = 0x39; h.flags9 = (uint8_t)((h.flags9 & 0xF0u) | 0x0Fu);
    image = image_for(&h, 0xC000, 0, &size); CHECK(image != NULL); prg = image + sizeof(h);
    memset(prg, 0x51, 0x8000); memset(prg + 0x8000, 0x92, 0x4000);
    CHECK(load_rom_memory(image, size) == 0);
    ppu_power_on(&ppu); apu_power_on(&apu); CHECK(cpu_power_on(&cpu));
    cart_cpu_write(0x5000, 3); CHECK(discrete_cpu_load(0x8100, 0x51) == 0);
    m111_flash_command(0xA0); cart_cpu_write(0x8123, 0x00); CHECK(cart_cpu_read(0x8123) == 0x51);
    cart_cpu_write(0x5000, 0); m111_flash_command(0xA0); cart_cpu_write(0x8123, 0x00); CHECK(cart_cpu_read(0x8123) == 0x00);
    cart_cpu_write(0x5000, 0x10); cart_ppu_write(0x0123, 0xB7);
    CHECK(cart_ppu_read(0x0123) == 0xB7 && cart_ppu_read(0x0923) == 0xB7);
    previous = cart; previous_prg = prg_rom;
    CHECK(load_rom_memory(image, size - 1) == -1 && cart == previous && prg_rom == previous_prg);
    free(image);

    // Namco 108 submapper 1 fixes four CPU slots starting at page zero. With
    // two physical 8 KiB pages the slots wrap 0,1,0,1.
    h = header_for(206, 0x4000, false); h.flags7 |= 8; h.prg_ram_size = 0x10;
    image = image_for(&h, 0x4000, 0x2000, &size); CHECK(image != NULL); prg = image + sizeof(h);
    memset(prg, 0x61, 0x2000); memset(prg + 0x2000, 0x82, 0x2000);
    uint8_t *chr = prg + 0x4000;
    for (unsigned bank = 0; bank < 8; ++bank) memset(chr + bank * 0x0400, (int)bank, 0x0400);
    CHECK(load_rom_memory(image, size) == 0);
    ppu_power_on(&ppu); apu_power_on(&apu); CHECK(cpu_power_on(&cpu));
    CHECK(discrete_cpu_load(0x8100, 0x61) == 0 && discrete_cpu_load(0xA100, 0x82) == 0);
    CHECK(discrete_cpu_load(0xC100, 0x61) == 0 && discrete_cpu_load(0xE100, 0x82) == 0);
    namco108_write_bank(6, 1); CHECK(discrete_cpu_load(0x8100, 0x61) == 0);
    namco108_write_bank(2, 5); CHECK(cart_ppu_read(0x1000) == 5);
    previous = cart; previous_prg = prg_rom;
    CHECK(load_rom_memory(image, size - 1) == -1 && cart == previous && prg_rom == previous_prg);
    free(image);

    // Mapper 105's fixed CHR RAM can be smaller than MMC1's nominal 4 KiB
    // page. A 2 KiB device remains writable and aliases across PPU space.
    h = header_for(105, 0x40000, true); h.flags7 |= 8; h.flags10 = 7;
    h.zero[0] = 5; // 2 KiB CHR RAM.
    image = image_for(&h, 0x40000, 0, &size); CHECK(image != NULL);
    CHECK(load_rom_memory(image, size) == 0);
    cart_ppu_write(0x0123, 0xA5);
    CHECK(cart_ppu_read(0x0123) == 0xA5 && cart_ppu_read(0x0923) == 0xA5);
    cart_ppu_write(0x0923, 0xB6);
    CHECK(cart_ppu_read(0x0123) == 0xB6 && cart_ppu_read(0x1123) == 0xB6);
    serial_write(0xA000, 7); CHECK(cart_ppu_read(0x0123) == 0xB6);
    previous = cart; previous_prg = prg_rom;
    CHECK(load_rom_memory(image, size - 1) == -1 && cart == previous && prg_rom == previous_prg);
    free(image);

    // TxSROM also shrinks its native 1 KiB CHR pages. Eight 512-byte slots
    // cover the first 4 KiB and the upper pattern-table half stays open bus.
    h = header_for(118, 0x20000, false); h.flags7 |= 8; h.flags10 = 7;
    h.chr_rom_chunks = 0x24; h.flags9 = (uint8_t)((h.flags9 & 0x0Fu) | 0xF0u);
    image = image_for(&h, 0x20000, 0x0200, &size); CHECK(image != NULL);
    memset(image + sizeof(h) + 0x20000, 0xB6, 0x0200);
    CHECK(load_rom_memory(image, size) == 0);
    CHECK(cart_ppu_read(0x0123) == 0xB6 && cart_ppu_read(0x1123) == 0x23);
    cart_cpu_write(0x8000, 2); cart_cpu_write(0x8001, 7);
    CHECK(cart_ppu_read(0x0123) == 0xB6);
    previous = cart; previous_prg = prg_rom;
    CHECK(load_rom_memory(image, size - 1) == -1 && cart == previous && prg_rom == previous_prg);
    free(image);

    // With 512 bytes of CHR RAM, TxSROM's selected 512-byte pages cover the
    // first 4 KiB and the default RAM mapping aliases through the upper half.
    h = header_for(118, 0x20000, true); h.flags7 |= 8; h.flags10 = 7;
    h.zero[0] = 3; // 512-byte CHR RAM.
    image = image_for(&h, 0x20000, 0, &size); CHECK(image != NULL);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    cart_ppu_write(0x0123, 0xC7);
    CHECK(cart_ppu_read(0x0123) == 0xC7 && cart_ppu_read(0x1123) == 0xC7);
    cart_ppu_write(0x1123, 0x6D);
    CHECK(cart_ppu_read(0x0123) == 0x6D);
    cart_cpu_write(0x8000, 2); cart_cpu_write(0x8001, 7);
    CHECK(cart_ppu_read(0x0123) == 0x6D);

    // LROG017 shrinks its ROM-backed 2 KiB slot while retaining the separate
    // writable CHR region above $07FF.
    h = header_for(77, 0x8000, false); h.flags7 |= 8; h.zero[0] = 7;
    h.chr_rom_chunks = 0x28; h.flags9 = (uint8_t)((h.flags9 & 0x0Fu) | 0xF0u);
    image = image_for(&h, 0x8000, 0x0400, &size); CHECK(image != NULL);
    memset(image + sizeof(h) + 0x8000, 0xC7, 0x0400);
    CHECK(load_rom_memory(image, size) == 0);
    CHECK(cart_ppu_read(0x0123) == 0xC7 && cart_ppu_read(0x0523) == 0x23);
    cart_ppu_write(0x0801, 0x35); CHECK(cart_ppu_read(0x0801) == 0x35);
    previous = cart; previous_prg = prg_rom;
    CHECK(load_rom_memory(image, size - 1) == -1 && cart == previous && prg_rom == previous_prg);
    free(image);
    CHECK(unload_rom());
    return 0;
}

static void small_chr_board_bank_write(unsigned mapper) {
    switch (mapper) {
        case 66:  cart_cpu_write(0x8000, 0x30); break;
        case 71:  cart_cpu_write(0xC000, 0x03); break;
        case 72:  cart_cpu_write(0x8000, 0xC3); break;
        case 78:  cart_cpu_write(0x8000, 0x53); break;
        case 87:  cart_cpu_write(0x6000, 0x01); break;
        case 92:  cart_cpu_write(0x8000, 0xC3); break;
        case 97:  cart_cpu_write(0x8000, 0x03); break;
        case 101: cart_cpu_write(0x6000, 0x05); break;
        case 140: cart_cpu_write(0x6000, 0x23); break;
        case 232:
            cart_cpu_write(0x8000, 0x08);
            cart_cpu_write(0xC000, 0x02);
            break;
    }
}

static int small_chr_board_cpu_bank_check(unsigned mapper) {
    switch (mapper) {
        case 66:  return cart_cpu_read(0x8001) == 12;
        case 71:  return cart_cpu_read(0x8001) == 6;
        case 72:  return cart_cpu_read(0x8001) == 6;
        case 78:  return cart_cpu_read(0x8001) == 6;
        case 87:  return cart_cpu_read(0x8001) == 0;
        case 92:  return cart_cpu_read(0xC001) == 6;
        case 97:  return cart_cpu_read(0xC001) == 6;
        case 101: return cart_cpu_read(0x8001) == 0;
        case 140: return cart_cpu_read(0x8001) == 8;
        case 232: return cart_cpu_read(0x8001) == 12 && cart_cpu_read(0xC001) == 14;
        default:  return 0;
    }
}

static int test_native_shrunk_chr8_windows(void) {
    static const struct {
        unsigned mapper;
        size_t prg_bytes;
    } boards[] = {
        {66, 0x20000}, {71, 0x20000}, {72, 0x20000}, {78, 0x20000},
        {87, 0x08000}, {92, 0x40000}, {97, 0x40000}, {101, 0x08000},
        {140, 0x20000}, {232, 0x40000}
    };

    for (size_t board = 0; board < sizeof(boards) / sizeof(boards[0]); ++board) {
        unsigned mapper = boards[board].mapper;
        size_t prg_bytes = boards[board].prg_bytes;
        iNESHeader h = header_for(mapper, prg_bytes, false);
        h.flags7 |= 0x08;
        h.chr_rom_chunks = 0x30; // NES 2.0 exponent encoding: 4 KiB CHR ROM.
        h.flags9 = 0xF0;

        size_t size;
        uint8_t *image = image_for(&h, prg_bytes, 0x1000, &size);
        CHECK(image != NULL);
        uint8_t *prg = image + sizeof(h);
        uint8_t *chr = prg + prg_bytes;
        for (size_t page = 0; page < prg_bytes / 0x2000; ++page)
            memset(prg + page * 0x2000, (int)page, 0x2000);
        memset(chr, 0xC5, 0x1000);
        if (mapper == 72 || mapper == 78 || mapper == 92) prg[0] = 0xFF;

        CHECK(load_rom_memory(image, size) == 0);
        free(image);
        CHECK(rom_mapper_number(&ines_header) == (int)mapper && chr_size == 0x1000);
        ppu_power_on(&ppu);
        CHECK(ppu_read(0x0123) == 0xC5);
        CHECK(ppu_read(0x1123) == 0x23); // Upper half remains open bus.
        ppu_write(0x0123, 0x5A);
        CHECK(ppu_read(0x0123) == 0xC5); // CHR ROM remains read-only.

        small_chr_board_bank_write(mapper);
        CHECK(small_chr_board_cpu_bank_check(mapper));
        CHECK(ppu_read(0x0123) == 0xC5 && ppu_read(0x1123) == 0x23);

        // A truncated replacement is rejected before activation and leaves the
        // selected CPU bank and shrunken PPU mapping intact.
        image = image_for(&h, prg_bytes, 0x1000, &size);
        CHECK(image != NULL);
        CHECK(load_rom_memory(image, size - 1) == -1);
        free(image);
        CHECK(rom_mapper_number(&ines_header) == (int)mapper);
        CHECK(small_chr_board_cpu_bank_check(mapper));
        CHECK(ppu_read(0x0123) == 0xC5 && ppu_read(0x1123) == 0x23);

        // A declared 4 KiB CHR-RAM chip mirrors through the pattern-table
        // window. Mapper bank writes cannot select storage that is not there.
        h = header_for(mapper, prg_bytes, true);
        h.flags7 |= 0x08;
        h.zero[0] = 6; // 64 << 6 = 4 KiB volatile CHR RAM.
        image = image_for(&h, prg_bytes, 0, &size);
        CHECK(image != NULL);
        prg = image + sizeof(h);
        for (size_t page = 0; page < prg_bytes / 0x2000; ++page)
            memset(prg + page * 0x2000, (int)page, 0x2000);
        if (mapper == 72 || mapper == 78 || mapper == 92) prg[0] = 0xFF;
        CHECK(load_rom_memory(image, size) == 0);
        free(image);
        CHECK(chr_size == 0x1000);
        ppu_power_on(&ppu);
        ppu_write(0x0123, 0xA6);
        CHECK(ppu_read(0x0123) == 0xA6 && ppu_read(0x1123) == 0xA6);
        small_chr_board_bank_write(mapper);
        CHECK(ppu_read(0x0123) == 0xA6 && ppu_read(0x1123) == 0xA6);
        ppu_write(0x1123, 0x69);
        CHECK(ppu_read(0x0123) == 0x69 && ppu_read(0x1123) == 0x69);
    }

    // Fixed-CHR boards 71 and 97 accept larger physical images as well. Their
    // single visible 8 KiB page stays on the first physical page.
    const unsigned fixed_boards[] = {71, 97};
    for (size_t i = 0; i < sizeof(fixed_boards) / sizeof(fixed_boards[0]); ++i) {
        unsigned mapper = fixed_boards[i];
        size_t prg_bytes = mapper == 71 ? 0x20000 : 0x40000;
        iNESHeader h = header_for(mapper, prg_bytes, false);
        h.chr_rom_chunks = 2;
        size_t size;
        uint8_t *image = image_for(&h, prg_bytes, 0x4000, &size);
        CHECK(image != NULL);
        memset(image + sizeof(h) + prg_bytes, 0x41, 0x2000);
        memset(image + sizeof(h) + prg_bytes + 0x2000, 0x82, 0x2000);
        CHECK(load_rom_memory(image, size) == 0);
        free(image);
        CHECK(ppu_read(0x0000) == 0x41 && ppu_read(0x1FFF) == 0x41);
        small_chr_board_bank_write(mapper);
        CHECK(ppu_read(0x0000) == 0x41 && ppu_read(0x1FFF) == 0x41);
    }
    return 0;
}

static int test_discrete_followup_page_geometry(void) {
    // NES 2.0 exponent sizes: 48 KiB PRG and 12 KiB CHR. Both contain one
    // complete mapper page plus a trailing partial page, which is not selected.
    const unsigned paged_boards[] = {79, 11, 144};
    for (size_t board = 0; board < sizeof(paged_boards) / sizeof(paged_boards[0]); ++board) {
        unsigned mapper = paged_boards[board];
        iNESHeader h = header_for(mapper, 0x8000, false);
        h.flags7 |= 8;
        h.prg_rom_chunks = 0x39; // 3 * 2^14 = 48 KiB.
        h.chr_rom_chunks = 0x31; // 3 * 2^12 = 12 KiB.
        h.flags9 = 0xFF;
        size_t size;
        uint8_t *image = image_for(&h, 0xC000, 0x3000, &size);
        CHECK(image != NULL);
        uint8_t *prg = image + sizeof(h);
        uint8_t *chr = prg + 0xC000;
        memset(prg, 0x21, 0x8000);
        memset(prg + 0x8000, 0x42, 0x4000);
        memset(chr, 0x31, 0x2000);
        memset(chr + 0x2000, 0x62, 0x1000);
        if (mapper == 11 || mapper == 144) prg[0x123] = 0xFF;
        CHECK(load_rom_memory(image, size) == 0);
        free(image);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        CHECK(cpu_power_on(&cpu));
        if (mapper == 79) {
            CHECK(discrete_cpu_store(0x4100, 0x09) == 0);
        } else {
            CHECK(discrete_cpu_store(0x8123, 0x11) == 0);
        }
        CHECK(discrete_cpu_load(0x8100, 0x21) == 0);
        CHECK(discrete_cpu_load(0xC100, 0x21) == 0);
        CHECK(ppu_read(0x0123) == 0x31 && ppu_read(0x1123) == 0x31);
        CHECK(unload_rom());
    }

    // A 24 KiB PRG image is smaller than NINA's 32 KiB window. One whole
    // copy maps at $8000-$DFFF and the remaining $E000-$FFFF stays open bus.
    // A smaller CHR ROM maps its physical bytes and leaves the upper range open.
    iNESHeader h = header_for(79, 0x8000, false);
    h.flags7 |= 8;
    h.prg_rom_chunks = 0x35; // 3 * 2^13 = 24 KiB.
    h.chr_rom_chunks = 0x30; // 1 * 2^12 = 4 KiB.
    h.flags9 = 0xFF;
    size_t size;
    uint8_t *image = image_for(&h, 0x6000, 0x1000, &size);
    CHECK(image != NULL);
    memset(image + sizeof(h), 0x52, 0x6000);
    memset(image + sizeof(h) + 0x6000, 0x64, 0x1000);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    CHECK(discrete_cpu_load(0x8100, 0x52) == 0);
    CHECK(discrete_cpu_load(0xC100, 0x52) == 0);
    CHECK(discrete_cpu_load(0xE100, 0xE1) == 0);
    CHECK(ppu_read(0x0123) == 0x64 && ppu_read(0x1123) == 0x23);
    CHECK(unload_rom());

    // Exactly 16 KiB is mirrored twice across a 32 KiB NINA PRG window.
    h = header_for(79, 0x8000, false);
    h.flags7 |= 8;
    h.prg_rom_chunks = 0x38; // 1 * 2^14 = 16 KiB.
    h.chr_rom_chunks = 0x30;
    h.flags9 = 0xFF;
    image = image_for(&h, 0x4000, 0x1000, &size);
    CHECK(image != NULL);
    memset(image + sizeof(h), 0x71, 0x4000);
    memset(image + sizeof(h) + 0x4000, 0x83, 0x1000);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    CHECK(discrete_cpu_load(0x8100, 0x71) == 0);
    CHECK(discrete_cpu_load(0xC100, 0x71) == 0);
    CHECK(ppu_read(0x0123) == 0x83 && ppu_read(0x1123) == 0x23);
    CHECK(unload_rom());

    // UxROM variants use the same smaller-CHR-ROM mapping rule.
    h = header_for(94, 0x8000, false);
    h.flags7 |= 8;
    h.prg_rom_chunks = 0x38;
    h.chr_rom_chunks = 0x30;
    h.flags9 = 0xFF;
    image = image_for(&h, 0x4000, 0x1000, &size);
    CHECK(image != NULL);
    memset(image + sizeof(h), 0x91, 0x4000);
    memset(image + sizeof(h) + 0x4000, 0xA3, 0x1000);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    CHECK(discrete_cpu_load(0x8100, 0x91) == 0);
    CHECK(discrete_cpu_load(0xC100, 0x91) == 0);
    CHECK(ppu_read(0x0123) == 0xA3 && ppu_read(0x1123) == 0x23);
    CHECK(unload_rom());

    // Oversized images are accepted; address lines simply cannot select every
    // complete page. Mapper 113 reaches its normal bank range within the image.
    h = header_for(113, 0x80000, false);
    h.flags7 |= 8;
    h.chr_rom_chunks = 32;
    image = image_for(&h, 0x80000, 0x40000, &size);
    CHECK(image != NULL);
    uint8_t *oversized_prg = image + sizeof(h);
    uint8_t *oversized_chr = oversized_prg + 0x80000;
    for (unsigned bank = 0; bank < 16; ++bank)
        memset(oversized_prg + bank * 0x8000, 0x40 + bank, 0x8000);
    for (unsigned bank = 0; bank < 32; ++bank)
        memset(oversized_chr + bank * 0x2000, 0x80 + bank, 0x2000);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    CHECK(discrete_cpu_store(0x4100, 0x7F) == 0);
    CHECK(discrete_cpu_load(0x8100, 0x47) == 0);
    CHECK(ppu_read(0x0123) == 0x8F);
    CHECK(unload_rom());

    // Each newly added board accepts ROMs beyond the former artificial cap.
    // Banking still exposes only the pages its address lines can select.
    static const struct {
        unsigned mapper;
        size_t prg_bytes;
        size_t chr_bytes;
    } oversized[] = {
        {79,  0x18000,  0x12000},
        {94,  0x28000,  0x04000},
        {113, 0x80000,  0x40000},
        {144, 0x100000, 0x40000},
        {146, 0x18000,  0x12000},
        {180, 0x440000, 0x04000}
    };
    for (size_t i = 0; i < sizeof(oversized) / sizeof(oversized[0]); ++i) {
        h = header_for(oversized[i].mapper, 0x8000, false);
        h.flags7 |= 8;
        size_t prg_units = oversized[i].prg_bytes / 0x4000;
        h.prg_rom_chunks = (uint8_t)prg_units;
        h.chr_rom_chunks = (uint8_t)(oversized[i].chr_bytes / 0x2000);
        h.flags9 = (uint8_t)((prg_units >> 8) & 0x0F);
        image = image_for(&h, oversized[i].prg_bytes, oversized[i].chr_bytes, &size);
        CHECK(image != NULL);
        CHECK(load_rom_memory(image, size) == 0);
        free(image);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        CHECK(cpu_power_on(&cpu));
        CHECK(discrete_cpu_load(0x8100, 0x5C) == 0);
        CHECK(ppu_read(0x0123) == 0xA5);
        CHECK(unload_rom());
    }
    return 0;
}

static int test_discrete_followup_ignored_submappers(void) {
    const unsigned boards[] = {79, 94, 113, 144, 146, 180};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        iNESHeader h = header_for(boards[i], 0x8000, false);
        h.flags7 |= 8;
        h.prg_ram_size = 0xF0;
        size_t size;
        uint8_t *image = image_for(&h, 0x8000, 0x2000, &size);
        CHECK(image != NULL);
        CHECK(load_rom_memory(image, size) == 0);
        free(image);
        CHECK(rom_mapper_number(&ines_header) == (int)boards[i]);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        CHECK(cpu_power_on(&cpu));
        CHECK(discrete_cpu_load(0x8100, 0x5C) == 0);
        CHECK(ppu_read(0x0123) == 0xA5);
        CHECK(unload_rom());
    }
    return 0;
}

static int discrete_followup_save_cases(const SaveFixture *paths) {
    const unsigned boards[] = {79, 94, 113, 144, 146, 180};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        for (unsigned nes2 = 0; nes2 < 2; ++nes2) {
            iNESHeader h = header_for(boards[i], 0x8000, nes2 != 0);
            h.flags6 |= 2;
            if (nes2) {
                h.flags7 |= 8;
                h.flags10 = 0x70;
                h.zero[0] = 0x70;
            }
            size_t size;
            uint8_t *image = image_for(&h, 0x8000, nes2 ? 0 : 0x2000, &size);
            CHECK(image != NULL);
            FILE *file = fopen(paths->rom, "wb");
            CHECK(file != NULL);
            size_t written = fwrite(image, 1, size, file);
            int closed = fclose(file);
            free(image);
            CHECK(written == size && closed == 0 && load_rom(paths->rom) == 0);
            ppu_power_on(&ppu);
            apu_power_on(&apu);
            CHECK(cpu_power_on(&cpu));
            CHECK(discrete_cpu_store(0x6000, (uint8_t)(0x31 + i + nes2)) == 0);
            CHECK(discrete_cpu_store(0x7FFF, (uint8_t)(0xD5 - i - nes2)) == 0);
            if (nes2) ppu_write(0x0123, (uint8_t)(0xA6 + i));
            CHECK(unload_rom());
            CHECK(saved_file_size(paths->prg_save) == 0x2000);
            CHECK(saved_byte(paths->prg_save, 0) == (int)(0x31 + i + nes2));
            CHECK(saved_byte(paths->prg_save, 0x1FFF) == (int)(0xD5 - i - nes2));
            if (nes2) {
                CHECK(saved_file_size(paths->chr_save) == 0x2000);
                CHECK(saved_byte(paths->chr_save, 0x123) == (int)(0xA6 + i));
            }
            CHECK(load_rom(paths->rom) == 0);
            CHECK(read_mem(0x6000) == 0x31 + i + nes2);
            CHECK(read_mem(0x7FFF) == 0xD5 - i - nes2);
            if (nes2) CHECK(ppu_read(0x0123) == 0xA6 + i);
            CHECK(unload_rom());
        }
    }
    return 0;
}

static int test_discrete_followup_saves(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    int result = discrete_followup_save_cases(&paths);
    unload_rom();
    return result | save_fixture_end(&paths);
}

static int trainer_save_overlay_case(const SaveFixture *paths, unsigned mapper,
                                     bool volatile_ram, size_t save_length) {
    CHECK(unload_rom());
    CHECK(remove(paths->prg_save) == 0 || errno == ENOENT);
    iNESHeader h = header_for(mapper, 0x8000, false);
    h.flags7 |= 8;
    h.flags6 |= 6; // Battery and a 512-byte trainer.
    h.flags10 = volatile_ram ? 7 : 0x70;
    size_t image_size;
    uint8_t *image = image_for(&h, 0x8000, 0x2000, &image_size);
    CHECK(image != NULL);
    memset(image + sizeof(h), 0x3C, 512);
    size_t prg_offset = sizeof(h) + 512;
    image[prg_offset + 0x7FFC] = 0x00;
    image[prg_offset + 0x7FFD] = 0x81;
    FILE *fp = fopen(paths->rom, "wb");
    CHECK(fp != NULL);
    size_t written = fwrite(image, 1, image_size, fp);
    int closed = fclose(fp);
    free(image);
    CHECK(written == image_size && closed == 0);

    if (save_length != SIZE_MAX) {
        uint8_t saved[0x2400];
        CHECK(save_length <= sizeof(saved));
        memset(saved, 0xA6, sizeof(saved));
        fp = fopen(paths->prg_save, "wb");
        CHECK(fp != NULL);
        written = fwrite(saved, 1, save_length, fp);
        closed = fclose(fp);
        CHECK(written == save_length && closed == 0);
    }
    CHECK(load_rom(paths->rom) == 0);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu) && cpu.pc == 0x8100);
    size_t loaded = volatile_ram || save_length == SIZE_MAX ? 0 : save_length;
    uint8_t early_trainer = loaded > 0x1023 ? 0xA6 : 0x3C;
    uint8_t late_trainer = loaded > 0x1123 ? 0xA6 : 0x3C;
    CHECK(discrete_cpu_load(0x6023, loaded > 0x23 ? 0xA6 : 0) == 0);
    CHECK(discrete_cpu_load(0x7023, early_trainer) == 0);
    CHECK(discrete_cpu_load(0x7123, late_trainer) == 0);
    CHECK(discrete_cpu_load(0x7223, loaded > 0x1223 ? 0xA6 : 0) == 0);
    CHECK(discrete_cpu_load(0x7FFF, loaded >= 0x2000 ? 0xA6 : 0) == 0);
    if (mapper == 5) {
        CHECK(discrete_cpu_store(0x5104, 2) == 0);
        CHECK(discrete_cpu_load(0x5C23, loaded > 0x2023 ? 0xA6 : 0) == 0);
        CHECK(discrete_cpu_store(0x5102, 2) == 0);
        CHECK(discrete_cpu_store(0x5103, 1) == 0);
    } else if (mapper == 19) {
        CHECK(discrete_cpu_store(0xF800, 0x23) == 0);
        CHECK(discrete_cpu_load(0x4800, loaded > 0x2023 ? 0xA6 : 0) == 0);
        CHECK(discrete_cpu_store(0xF800, 0x40) == 0); // Enable PRG RAM writes.
    }

    uint8_t malformed[sizeof(iNESHeader)] = {0};
    uint8_t *active_prg = prg_rom;
    CHECK(load_rom_memory(malformed, sizeof(malformed)) == -1);
    CHECK(prg_rom == active_prg);
    CHECK(discrete_cpu_load(0x7123, late_trainer) == 0);
    CHECK(discrete_cpu_store(0x6023, 0x5D) == 0);
    CHECK(discrete_cpu_load(0x6023, 0x5D) == 0);
    CHECK(unload_rom());
    if (!volatile_ram) {
        long expected_size = mapper == 5 ? 0x2400 : mapper == 19 ? 0x2080 : 0x2000;
        CHECK(saved_file_size(paths->prg_save) == expected_size);
        CHECK(saved_byte(paths->prg_save, 0x23) == 0x5D);
        CHECK(saved_byte(paths->prg_save, 0x1023) == early_trainer);
        CHECK(saved_byte(paths->prg_save, 0x1123) == late_trainer);
        if (mapper == 5 || mapper == 19)
            CHECK(saved_byte(paths->prg_save, 0x2023) == (loaded > 0x2023 ? 0xA6 : 0));
    }
    CHECK(load_rom(paths->rom) == 0);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    CHECK(discrete_cpu_load(0x6023, volatile_ram ? 0 : 0x5D) == 0);
    CHECK(discrete_cpu_load(0x7023, early_trainer) == 0);
    CHECK(discrete_cpu_load(0x7123, late_trainer) == 0);
    CHECK(unload_rom());
    return 0;
}

static int test_trainer_and_existing_saves(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    const unsigned boards[] = {0, 79, 5, 19};
    const size_t lengths[] = {SIZE_MAX, 0, 0x800, 0x1100, 0x2000};
    int result = 0;
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]) && !result; ++i) {
        for (size_t j = 0; j < sizeof(lengths) / sizeof(lengths[0]) && !result; ++j) {
            size_t length = lengths[j];
            if (length == 0x2000 && boards[i] == 5) length = 0x2400;
            if (length == 0x2000 && boards[i] == 19) length = 0x2080;
            result = trainer_save_overlay_case(&paths, boards[i], false, length);
        }
    }
    unload_rom();
    return result | save_fixture_end(&paths);
}

static int test_trainer_volatile_ram_ignores_save(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    int result = trainer_save_overlay_case(&paths, 0, true, 0x2000);
    if (!result) result = trainer_save_overlay_case(&paths, 79, true, 0x2000);
    unload_rom();
    return result | save_fixture_end(&paths);
}

static int test_game_database_and_headerless_loading(void) {
    CHECK(unload_rom());
    rom_database_clear();
    rom_database_set_overrides(true);
    joypad_set_configuration_overrides(0);
    CHECK(joypad_set_adapter(NES_ADAPTER_NONE));
    CHECK(joypad_set_port_device(0, NES_PORT_GAMEPAD));
    CHECK(joypad_set_port_device(1, NES_PORT_GAMEPAD));
    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));

    iNESHeader legacy = header_for(0, 0x4000, false);
    size_t corrected_size = 0;
    uint8_t *corrected = image_for(&legacy, 0x8000, 0x2000, &corrected_size);
    CHECK(corrected != NULL);
    for (size_t i = 0; i < 0x8000; ++i)
        corrected[sizeof(legacy) + i] = (uint8_t)(i / 0x4000);
    uint32_t corrected_payload_crc = game_db_crc32(corrected + sizeof(legacy),
                                                   corrected_size - sizeof(legacy));
    uint32_t corrected_file_crc = game_db_crc32(corrected, corrected_size);
    uint32_t corrected_prg_crc = game_db_crc32(corrected + sizeof(legacy), 0x8000);
    char database[512];
    int length = snprintf(database, sizeof(database),
        "%08X,Famicom,UOROM,TEST,MMC1B,2,32,8,0,8,0,0,v,59,N,2,0,0\n",
        (unsigned)corrected_payload_crc);
    CHECK(length > 0 && (size_t)length < sizeof(database));
    CHECK(rom_database_load_memory(database, (size_t)length));
    CHECK(load_rom_memory(corrected, corrected_size) == 0);
    CHECK(rom_metadata_source() == ROM_METADATA_DATABASE);
    CHECK(rom_mapper_number(&ines_header) == 2 && (ines_header.prg_ram_size >> 4) == 2);
    CHECK(prg_size == 0x8000 && chr_size == 0x2000);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL && nes_timing()->region == NES_REGION_NTSC);
    CHECK(rom_file_crc32() == corrected_file_crc);
    CHECK(rom_prg_crc32() == corrected_prg_crc && rom_prg_chr_crc32() == corrected_payload_crc);
    CHECK(joypad_expansion_device() == NES_EXPANSION_FCNS_CONTROLLER);
    cart_cpu_write(0x6123, 0xA7);
    CHECK(cart_cpu_read(0x6123) == 0xA7);
    cart_cpu_write(0x8000, 1);
    CHECK(cart_cpu_read(0x8000) == 1); // Database disabled the board's bus conflict.

    GameDbEntry looked_up = {0};
    CHECK(game_db_lookup(corrected_payload_crc, &looked_up));
    CHECK(looked_up.mapper == 2 && looked_up.submapper_present && looked_up.submapper == 2);
    CHECK(strcmp(looked_up.board, "UOROM") == 0 && looked_up.bus_conflicts == 0);

    /* A database correction does not turn a legacy cartridge into NES 2.0.
       Mapper 16 accepts the legacy 8 KiB RAM default before resolving its
       serial-memory device; NES 2.0 interprets that declaration differently. */
    iNESHeader bandai_header = header_for(0, 0x8000, false);
    size_t bandai_size = 0;
    uint8_t *bandai_image = image_for(&bandai_header, 0x8000, 0x2000, &bandai_size);
    CHECK(bandai_image != NULL);
    bandai_image[sizeof(bandai_header)] = 0x4B;
    uint32_t bandai_crc = game_db_crc32(bandai_image + sizeof(bandai_header),
                                        bandai_size - sizeof(bandai_header));
    length = snprintf(database, sizeof(database),
        "%08X,NesNtsc,BANDAI-FCG,,,16,32,8,,,,0,h,1,N,,0,0\n", (unsigned)bandai_crc);
    CHECK(length > 0 && (size_t)length < sizeof(database));
    CHECK(rom_database_load_memory(database, (size_t)length));
    CHECK(load_rom_memory(bandai_image, bandai_size) == 0);
    CHECK(rom_metadata_source() == ROM_METADATA_DATABASE && rom_mapper_number(&ines_header) == 16);
    cart_cpu_write(0x8008, 0);
    CHECK(cart_cpu_read(0x8000) == 0x4B);
    free(bandai_image);

    /* Native board implementations must still honor explicit validated DB RAM
       sizes after retaining legacy mapper semantics. */
    iNESHeader bandai70_header = header_for(0, 0x8000, false);
    size_t bandai70_size = 0;
    uint8_t *bandai70_image = image_for(&bandai70_header, 0x8000, 0x2000, &bandai70_size);
    CHECK(bandai70_image != NULL);
    uint32_t bandai70_crc = game_db_crc32(bandai70_image + sizeof(bandai70_header),
                                          bandai70_size - sizeof(bandai70_header));
    length = snprintf(database, sizeof(database),
        "%08X,NesNtsc,BANDAI-74161,,,70,32,8,0,0,0,0,v,1,N,0,0,0\n",
        (unsigned)bandai70_crc);
    CHECK(length > 0 && (size_t)length < sizeof(database));
    CHECK(rom_database_load_memory(database, (size_t)length));
    CHECK(load_rom_memory(bandai70_image, bandai70_size) == 0);
    CHECK(rom_metadata_source() == ROM_METADATA_DATABASE && rom_mapper_number(&ines_header) == 70);
    cart_cpu_write(0x6123, 0xA6);
    CHECK(cart_cpu_read_bus(0x6123, 0x69) == 0x69);
    free(bandai70_image);

    iNESHeader blank_ram_header = header_for(0, 0x4000, false);
    size_t blank_ram_size = 0;
    uint8_t *blank_ram_image = image_for(&blank_ram_header, 0x4000, 0x2000, &blank_ram_size);
    CHECK(blank_ram_image != NULL);
    uint32_t blank_ram_crc = game_db_crc32(blank_ram_image + sizeof(blank_ram_header),
                                           blank_ram_size - sizeof(blank_ram_header));
    length = snprintf(database, sizeof(database),
        "%08X,NesNtsc,NROM,,,0,16,8,,,,0,h,1,N,,0,0\n", (unsigned)blank_ram_crc);
    CHECK(length > 0 && (size_t)length < sizeof(database));
    CHECK(rom_database_load_memory(database, (size_t)length));
    CHECK(load_rom_memory(blank_ram_image, blank_ram_size) == 0);
    cart_cpu_write(0x6123, 0x6A);
    CHECK(cart_cpu_read(0x6123) == 0x6A); // Blank DB RAM keeps legacy 8 KiB work RAM.
    free(blank_ram_image);

    iNESHeader battery_header = header_for(0, 0x4000, false);
    battery_header.flags6 |= 0x02;
    size_t battery_size = 0;
    uint8_t *battery_image = image_for(&battery_header, 0x4000, 0x2000, &battery_size);
    CHECK(battery_image != NULL);
    uint32_t battery_crc = game_db_crc32(battery_image + sizeof(battery_header),
                                         battery_size - sizeof(battery_header));
    length = snprintf(database, sizeof(database),
        "%08X,NesNtsc,NROM,,,0,16,8,,,,0,h,1,N,,0,0\n", (unsigned)battery_crc);
    CHECK(length > 0 && (size_t)length < sizeof(database));
    CHECK(rom_database_load_memory(database, (size_t)length));
    CHECK(load_rom_memory(battery_image, battery_size) == 0);
    CHECK((ines_header.flags6 & 0x02) != 0);
    cart_cpu_write(0x6456, 0xA9);
    CHECK(cart_cpu_read(0x6456) == 0xA9); // Original battery keeps legacy save-RAM default.
    free(battery_image);

    iNESHeader chr_ram_header = header_for(0, 0x4000, true);
    size_t chr_ram_image_size = 0;
    uint8_t *chr_ram_image = image_for(&chr_ram_header, 0x4000, 0, &chr_ram_image_size);
    CHECK(chr_ram_image != NULL);
    uint32_t chr_ram_crc = game_db_crc32(chr_ram_image + sizeof(chr_ram_header),
                                         chr_ram_image_size - sizeof(chr_ram_header));
    length = snprintf(database, sizeof(database),
        "%08X,NesNtsc,NROM,,,0,16,0,4,,,0,h,1,N,,0,0\n", (unsigned)chr_ram_crc);
    CHECK(length > 0 && (size_t)length < sizeof(database));
    CHECK(rom_database_load_memory(database, (size_t)length));
    CHECK(load_rom_memory(chr_ram_image, chr_ram_image_size) == 0);
    CHECK(chr_size == 4096);
    ppu_write(0x0123, 0xB7);
    CHECK(ppu_read(0x0123) == 0xB7);
    free(chr_ram_image);

    iNESHeader exact_header = header_for(0, 0x4000, false);
    const size_t exact_prg = 12345, exact_chr = 3000;
    size_t exact_size = 0;
    uint8_t *exact_image = image_for(&exact_header, exact_prg, exact_chr, &exact_size);
    CHECK(exact_image != NULL);
    exact_image[sizeof(exact_header)] = 0x37;
    exact_image[sizeof(exact_header) + exact_prg] = 0xA4;
    uint32_t exact_crc = game_db_crc32(exact_image + sizeof(exact_header),
                                       exact_size - sizeof(exact_header));
    length = snprintf(database, sizeof(database),
        "%08X,NesNtsc,NROM,,,0,b12345,b3000,0,8,0,0,h,1,N,,0,0\n", (unsigned)exact_crc);
    CHECK(length > 0 && (size_t)length < sizeof(database));
    CHECK(rom_database_load_memory(database, (size_t)length));
    CHECK(load_rom_memory(exact_image, exact_size) == 0);
    CHECK(prg_size == exact_prg && chr_size == exact_chr);
    CHECK(cart_cpu_read(0x8000) == 0x37 && ppu_read(0) == 0xA4);
    free(exact_image);

    iNESHeader nes2 = header_for(0, 0x8000, false);
    nes2.flags7 |= 0x08;
    size_t nes2_size = 0;
    uint8_t *nes2_image = image_for(&nes2, 0x8000, 0x2000, &nes2_size);
    CHECK(nes2_image != NULL);
    memcpy(nes2_image + sizeof(nes2), corrected + sizeof(legacy), 0xA000);
    CHECK(load_rom_memory(nes2_image, nes2_size) == 0);
    CHECK(rom_metadata_source() == ROM_METADATA_NES20 && rom_mapper_number(&ines_header) == 0);
    free(nes2_image);

    rom_database_set_overrides(false);
    iNESHeader plain = header_for(0, 0x8000, false);
    size_t plain_size = 0;
    uint8_t *plain_image = image_for(&plain, 0x8000, 0x2000, &plain_size);
    CHECK(plain_image != NULL);
    memcpy(plain_image + sizeof(plain), corrected + sizeof(legacy), 0xA000);
    CHECK(load_rom_memory(plain_image, plain_size) == 0);
    CHECK(rom_metadata_source() == ROM_METADATA_INES && rom_mapper_number(&ines_header) == 0);
    free(plain_image);
    rom_database_set_overrides(true);

    iNESHeader vs_header = header_for(0, 0x4000, false);
    size_t vs_size = 0;
    uint8_t *vs_image = image_for(&vs_header, 0x4000, 0x2000, &vs_size);
    CHECK(vs_image != NULL);
    uint32_t vs_crc = game_db_crc32(vs_image + sizeof(vs_header), vs_size - sizeof(vs_header));
    length = snprintf(database, sizeof(database),
        "%08X,VsSystem,NROM,,,0,16,8,0,0,0,0,h,1,N,0,0,2\n", (unsigned)vs_crc);
    CHECK(length > 0 && (size_t)length < sizeof(database));
    CHECK(rom_database_load_memory(database, (size_t)length));
    CHECK(load_rom_memory(vs_image, vs_size) == 0);
    CHECK(rom_metadata_source() == ROM_METADATA_DATABASE && vs_enabled());
    CHECK(vs_ppu_model() == VS_PPU_2C04_0001);
    free(vs_image);

    uint8_t *headerless = (uint8_t *)malloc(0x6000);
    CHECK(headerless != NULL);
    memset(headerless, 0x35, 0x4000);
    memset(headerless + 0x4000, 0xC7, 0x2000);
    uint32_t headerless_crc = game_db_crc32(headerless, 0x6000);
    length = snprintf(database, sizeof(database),
        "%08X,NesPal,NROM,,,0,16,8,0,8,0,0,h,1,N,,0,0\n", (unsigned)headerless_crc);
    CHECK(length > 0 && (size_t)length < sizeof(database));

    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    char database_path[128];
    snprintf(database_path, sizeof(database_path), "%s/games.txt", paths.directory);
    FILE *database_file = fopen(database_path, "wb");
    CHECK(database_file != NULL);
    size_t database_written = fwrite(database, 1, (size_t)length, database_file);
    int database_closed = fclose(database_file);
    CHECK(database_written == (size_t)length && database_closed == 0);
    CHECK(rom_database_load_file(database_path));

    rom_database_set_overrides(false); // Headerless lookup remains available.
    CHECK(load_rom_memory(headerless, 0x6000) == 0);
    CHECK(rom_metadata_source() == ROM_METADATA_DATABASE_HEADERLESS);
    CHECK(rom_file_crc32() == headerless_crc && rom_prg_chr_crc32() == headerless_crc);
    CHECK(nes_timing()->region == NES_REGION_PAL && rom_mapper_number(&ines_header) == 0);
    cart_cpu_write(0x6123, 0x6D);
    CHECK(cart_cpu_read(0x6123) == 0x6D);

    uint8_t unknown[0x6000];
    memcpy(unknown, headerless, sizeof(unknown));
    unknown[0] ^= 0xFF;
    uint8_t *active_prg = prg_rom;
    Mapper *active_cart = cart;
    CHECK(load_rom_memory(unknown, sizeof(unknown)) == -1);
    CHECK(prg_rom == active_prg && cart == active_cart && cart_cpu_read(0x6123) == 0x6D);

    CHECK(!rom_database_load_memory("bad", 3));
    CHECK(load_rom_memory(headerless, 0x6000) == 0);
    CHECK(rom_metadata_source() == ROM_METADATA_DATABASE_HEADERLESS);

    rom_database_set_overrides(true);
    rom_database_clear();
    CHECK(remove(database_path) == 0);
    CHECK(unload_rom());
    free(headerless);
    free(corrected);
    CHECK(save_fixture_end(&paths) == 0);
    return 0;
}

static int test_unif_loading_and_named_boards(void) {
    CHECK(unload_rom());
    rom_database_clear();
    rom_database_set_overrides(true);

    uint8_t prg0[0x4000], prg1[0x4000], chr0[0x1000], chr1[0x1000];
    memset(prg0, 0x11, sizeof(prg0));
    memset(prg1, 0x22, sizeof(prg1));
    memset(chr0, 0x33, sizeof(chr0));
    memset(chr1, 0x44, sizeof(chr1));
    const char nrom_board[] = "NES-NROM-256";
    const uint8_t tv_pal = 1, battery = 1, vertical = 1;

    UnifFixture unif;
    CHECK(unif_fixture_begin(&unif) == 0);
    CHECK(unif_fixture_chunk(&unif, "PRG1", prg1, sizeof(prg1)) == 0);
    CHECK(unif_fixture_chunk(&unif, "MAPR", nrom_board, sizeof(nrom_board)) == 0);
    CHECK(unif_fixture_chunk(&unif, "CHR1", chr1, sizeof(chr1)) == 0);
    CHECK(unif_fixture_chunk(&unif, "TVCI", &tv_pal, 1) == 0);
    CHECK(unif_fixture_chunk(&unif, "BATR", &battery, 1) == 0);
    CHECK(unif_fixture_chunk(&unif, "MIRR", &vertical, 1) == 0);
    CHECK(unif_fixture_chunk(&unif, "PRG0", prg0, sizeof(prg0)) == 0);
    CHECK(unif_fixture_chunk(&unif, "CHR0", chr0, sizeof(chr0)) == 0);
    CHECK(load_rom_memory(unif.data, unif.size) == 0);
    CHECK(rom_metadata_source() == ROM_METADATA_UNIF);
    CHECK(rom_mapper_number(&ines_header) == 0);
    CHECK(prg_size == 0x8000 && chr_size == 0x2000);
    CHECK(cart_cpu_read(0x8000) == 0x11 && cart_cpu_read(0xC000) == 0x22);
    CHECK(cart_ppu_read(0x0000) == 0x33 && cart_ppu_read(0x1000) == 0x44);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL && nes_timing()->region == NES_REGION_PAL);
    cart_cpu_write(0x6123, 0xA6);
    CHECK(cart_cpu_read(0x6123) == 0xA6);
    uint8_t ordered_prg[0x8000];
    memcpy(ordered_prg, prg0, sizeof(prg0));
    memcpy(ordered_prg + sizeof(prg0), prg1, sizeof(prg1));
    CHECK(rom_prg_crc32() == game_db_crc32(ordered_prg, sizeof(ordered_prg)));
    CHECK(rom_file_crc32() == game_db_crc32(unif.data, unif.size));
    unif_fixture_end(&unif);

    CHECK(unif_board_mapper_id("UNL-NROM") == 0);
    CHECK(unif_board_mapper_id("HVC-SLROM") == 1);
    CHECK(unif_board_mapper_id("BTL-MARIO1-MALEE2") == UNIF_BOARD_MALEE);
    CHECK(unif_board_mapper_id("BMC-SSS-NROM-256") == UNIF_BOARD_SSS_NROM_256);
    CHECK(unif_board_mapper_id("NOT-A-BOARD") == UNIF_BOARD_UNKNOWN);

    char database[256];
    int db_len = snprintf(database, sizeof(database),
        "12345678,NesNtsc,NES-MARIO1-MALEE2,,,65000,34,8,0,0,0,0,h,1,N,,0,0\n");
    CHECK(db_len > 0 && (size_t)db_len < sizeof(database));
    CHECK(rom_database_load_memory(database, (size_t)db_len));
    GameDbEntry resolved = {0};
    CHECK(game_db_lookup(0x12345678u, &resolved));
    CHECK(resolved.mapper == UNIF_BOARD_MALEE);
    rom_database_clear();

    uint8_t malee_prg[0x8800], malee_chr[0x2000];
    for (size_t page = 0; page < sizeof(malee_prg) / 0x800; ++page)
        memset(malee_prg + page * 0x800, (int)page, 0x800);
    memset(malee_chr, 0x5C, sizeof(malee_chr));
    const char malee_board[] = "UNL-MARIO1-MALEE2";
    CHECK(unif_fixture_begin(&unif) == 0);
    CHECK(unif_fixture_chunk(&unif, "CHR0", malee_chr, sizeof(malee_chr)) == 0);
    CHECK(unif_fixture_chunk(&unif, "MAPR", malee_board, sizeof(malee_board)) == 0);
    CHECK(unif_fixture_chunk(&unif, "PRG0", malee_prg, sizeof(malee_prg)) == 0);
    CHECK(load_rom_memory(unif.data, unif.size) == 0);
    CHECK(rom_metadata_source() == ROM_METADATA_UNIF);
    CHECK(cart_cpu_read(0x6000) == 16 && cart_cpu_read(0x8000) == 0);
    CHECK(cart_ppu_read(0x0123) == 0x5C);
    unif_fixture_end(&unif);

    uint8_t box_prg[0x8000], box_chr[0x2000];
    memset(box_prg, 0x71, sizeof(box_prg));
    memset(box_chr, 0x2E, sizeof(box_chr));
    const char box_board[] = "SSS-NROM-256";
    CHECK(unif_fixture_begin(&unif) == 0);
    CHECK(unif_fixture_chunk(&unif, "MAPR", box_board, sizeof(box_board)) == 0);
    CHECK(unif_fixture_chunk(&unif, "PRG0", box_prg, sizeof(box_prg)) == 0);
    CHECK(unif_fixture_chunk(&unif, "CHR0", box_chr, sizeof(box_chr)) == 0);
    CHECK(load_rom_memory(unif.data, unif.size) == 0);
    CHECK(cart_cpu_read_bus(0x5007, 0x00) == 0x22);
    uint8_t *active_prg = prg_rom;
    Mapper *active_cart = cart;
    unif_fixture_end(&unif);

    const char unknown_board[] = "BMC-NOT-A-BOARD";
    CHECK(unif_fixture_begin(&unif) == 0);
    CHECK(unif_fixture_chunk(&unif, "MAPR", unknown_board, sizeof(unknown_board)) == 0);
    CHECK(unif_fixture_chunk(&unif, "PRG0", box_prg, sizeof(box_prg)) == 0);
    CHECK(load_rom_memory(unif.data, unif.size) == -1);
    CHECK(prg_rom == active_prg && cart == active_cart);
    CHECK(cart_cpu_read_bus(0x5007, 0x00) == 0x22);
    unif_fixture_end(&unif);

    CHECK(unif_fixture_begin(&unif) == 0);
    CHECK(unif_fixture_chunk(&unif, "MAPR", nrom_board, sizeof(nrom_board)) == 0);
    CHECK(load_rom_memory(unif.data, unif.size) == -1);
    CHECK(prg_rom == active_prg && cart == active_cart);
    unif_fixture_end(&unif);

    uint8_t malformed[40] = {0};
    memcpy(malformed, "UNIF", 4);
    memcpy(malformed + 32, "MAPR", 4);
    malformed[36] = 0x40;
    CHECK(load_rom_memory(malformed, sizeof(malformed)) == -1);
    CHECK(prg_rom == active_prg && cart == active_cart);

    CHECK(unif_fixture_begin(&unif) == 0);
    CHECK(unif_fixture_chunk(&unif, "MAPR", nrom_board, sizeof(nrom_board)) == 0);
    CHECK(unif_fixture_chunk(&unif, "PRGZ", prg0, sizeof(prg0)) == 0);
    CHECK(load_rom_memory(unif.data, unif.size) == -1);
    CHECK(prg_rom == active_prg && cart == active_cart);
    unif_fixture_end(&unif);

    CHECK(unload_rom());
    return 0;
}

static int test_cartridge_unload(void) {
    iNESHeader h = header_for(0, 0x4000, true);
    size_t image_size;
    uint8_t *image = image_for(&h, 0x4000, 0, &image_size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, image_size);
    free(image);
    CHECK(loaded == 0 && cart != NULL && prg_rom != NULL && chr_rom != NULL);
    unload_rom();
    unload_rom();
    CHECK(cart == NULL && prg_rom == NULL && chr_rom == NULL);
    CHECK(prg_size == 0 && chr_size == 0 && !cart_irq_pending() && mirroring_mode == 0);
    const iNESHeader empty = {0};
    CHECK(memcmp(&ines_header, &empty, sizeof(empty)) == 0);
    CHECK(cart_cpu_read_bus(0x8000, 0xA6) == 0xA6);
    CHECK(fixture(0, 0x4000, 0x2000, true) == 0);
    cart_ppu_write(0, 0xA6);
    unload_rom(); // External fixture arrays remain caller-owned.
    CHECK(fixture_chr[0] == 0xA6 && fixture_prg[0x3FFF] == 1);
    CHECK(fixture(0, 0x4000, 0x2000, true) == 0);
    CHECK(cart_cpu_read(0xFFFF) == 1);
    return 0;
}

#endif // MAPPER_ACCURACY_FOLLOWUPS_H

