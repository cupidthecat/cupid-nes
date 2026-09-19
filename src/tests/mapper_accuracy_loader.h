/*
 * mapper_accuracy_loader.h - Cartridge loader regression tests
 *
 * Author: @frankischilling
 *
 * This private header contains regression tests for cartridge headers, mapper
 * selection, and invalid images.
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
#ifndef MAPPER_ACCURACY_LOADER_H
#define MAPPER_ACCURACY_LOADER_H

static int test_header_and_mapper_rejection(void) {
    CHECK(fixture(0, 0x4000, 0x2000, true) == 0);
    iNESHeader h = header_for(0, 0x4000, true);
    Mapper *previous = cart;
    h.flags7 = 0x08;
    h.prg_ram_size = 1;
    CHECK(rom_mapper_number(&h) == 0x100);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x4000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0xFFFF) == 1);
    h.flags6 = 0x40;
    h.prg_ram_size = 0x20;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x4000, fixture_chr, 0x2000) == -1);
    h.flags7 = 0xFC;
    CHECK(rom_mapper_number(&h) == 4); // Legacy header ignores contaminated upper mapper bits.
    h.flags7 = 0xF0;
    CHECK(rom_mapper_number(&h) == 0xF4);
    CHECK(mapper_init_from_header(NULL, fixture_prg, 0x4000, fixture_chr, 0x2000) == -1);
    CHECK(mapper_init_from_header(&h, fixture_prg, 0, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous);
    return 0;
}

static uint8_t *image_for(const iNESHeader *h, size_t prg_bytes, size_t chr_bytes, size_t *size) {
    size_t trainer_bytes = (h->flags6 & 4) ? 512 : 0;
    *size = sizeof(*h) + trainer_bytes + prg_bytes + chr_bytes;
    uint8_t *image = (uint8_t *)calloc(1, *size);
    if (!image) return NULL;
    memcpy(image, h, sizeof(*h));
    for (size_t i = 0; i < trainer_bytes; ++i) image[sizeof(*h) + i] = (uint8_t)i;
    memset(image + sizeof(*h) + trainer_bytes, 0x5C, prg_bytes);
    memset(image + sizeof(*h) + trainer_bytes + prg_bytes, 0xA5, chr_bytes);
    return image;
}

static int test_mmc1a_ram_revision(void) {
    const unsigned boards[] = {1, 155};
    for (size_t board = 0; board < sizeof(boards) / sizeof(boards[0]); ++board) {
        for (unsigned nes2 = 0; nes2 < 2; ++nes2) {
            iNESHeader h = header_for(boards[board], 0x20000, false);
            h.chr_rom_chunks = 4;
            if (nes2) {
                h.flags7 |= 8;
                h.flags10 = 7;
            }
            size_t size;
            uint8_t *image = image_for(&h, 0x20000, 0x8000, &size);
            CHECK(image != NULL);
            for (size_t i = 0; i < 0x20000; ++i)
                image[sizeof(h) + i] = (uint8_t)(i / 0x2000);
            for (size_t i = 0; i < 0x8000; ++i)
                image[sizeof(h) + 0x20000 + i] = (uint8_t)(i / 0x0400);
            int loaded = load_rom_memory(image, size);
            free(image);
            CHECK(loaded == 0 && rom_mapper_number(&ines_header) == (int)boards[board]);

            cart_cpu_write(0x6123, 0xA6);
            serial_write(0xE000, 0x13);
            CHECK(cart_cpu_read(0x8000) == 6 && cart_cpu_read(0xC000) == 14);
            CHECK(cart_cpu_read_bus(0x6123, 0x56) == (board ? 0xA6 : 0x56));
            cart_cpu_write(0x6123, 0x55);
            serial_write(0xE000, 3);
            CHECK(cart_cpu_read(0x6123) == (board ? 0x55 : 0xA6));

            static const uint8_t prg_pages[4][2] = {{4, 6}, {4, 6}, {0, 6}, {6, 14}};
            static const Mirroring mirrors[4] = {
                MIRROR_SINGLE0, MIRROR_SINGLE1, MIRROR_VERTICAL, MIRROR_HORIZONTAL
            };
            serial_write(0xA000, 1);
            serial_write(0xC000, 3);
            for (unsigned mode = 0; mode < 4; ++mode) {
                serial_write(0x8000, (uint8_t)(0x10 | (mode << 2) | mode));
                CHECK(cart_cpu_read(0x8000) == prg_pages[mode][0]);
                CHECK(cart_cpu_read(0xC000) == prg_pages[mode][1]);
                CHECK(cart_get_mirroring() == mirrors[mode]);
                CHECK(cart_ppu_read(0) == 4 && cart_ppu_read(0x1000) == 12);
            }
            serial_write(0x8000, 0x0C);
            serial_write(0xA000, 3);
            CHECK(cart_ppu_read(0) == 8 && cart_ppu_read(0x1000) == 12);
            cart_ppu_write(0, 0x77);
            CHECK(cart_ppu_read(0) == 8);
            cart->reset();
            CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 14);
            CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
            CHECK(cart_cpu_read(0x6123) == (board ? 0x55 : 0xA6));
        }
    }
    return 0;
}

static int test_mmc1a_cpu_serial_writes(void) {
    CHECK(fixture(155, 0x20000, 0x2000, true) == 155);
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    for (uint16_t addr = 0; addr < 0x0800; ++addr) write_mem(addr, 0);
    fixture_prg[0x1E000] = 1;
    const uint8_t program[] = {
        0xEE, 0x00, 0xE0,       // INC $E000: serial bit 1, then an ignored adjacent write.
        0xA9, 0x00,             // LDA #0
        0x8D, 0x00, 0xE0,       // STA $E000
        0x8D, 0x00, 0xE0,
        0x8D, 0x00, 0xE0,
        0xA9, 0x01,             // LDA #1
        0x8D, 0x00, 0xE0,       // Commit $11, selecting PRG bank 1 with bit 4 set.
        0xA9, 0xA6,
        0x8D, 0x23, 0x61,       // STA $6123
        0xAD, 0x23, 0x61        // LDA $6123
    };
    for (size_t i = 0; i < sizeof(program); ++i)
        write_mem((uint16_t)(0x0200 + i), program[i]);
    cpu.pc = 0x0200;
    const int cycles[] = {6, 2, 4, 4, 4, 2, 4, 2, 4, 4};
    for (size_t i = 0; i < sizeof(cycles) / sizeof(cycles[0]); ++i)
        CHECK(cpu_step(&cpu) == cycles[i]);
    CHECK(cpu.a == 0xA6 && cart_cpu_read(0x6123) == 0xA6);
    CHECK(cart_cpu_read(0x8000) == 2 && cart_cpu_read(0xC000) == 14);

    // An adjacent reset-bit write is accepted and discards the partial shift.
    serial_write(0x8000, 3);
    fixture_prg[0x4000] = 0x7F;
    const uint8_t reset_program[] = {0xEE, 0x00, 0xC0}; // INC $C000 writes $7F, then $80.
    for (size_t i = 0; i < sizeof(reset_program); ++i)
        write_mem((uint16_t)(0x0200 + i), reset_program[i]);
    cpu.pc = 0x0200;
    CHECK(cpu_step(&cpu) == 6);
    CHECK(cart_cpu_read(0xC000) == 14 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    serial_write(0xE000, 2);
    CHECK(cart_cpu_read(0x8000) == 4 && cart_cpu_read(0x6123) == 0xA6);
    cart_ppu_write(0x0123, 0x69);
    CHECK(cart_ppu_read(0x0123) == 0x69);
    return 0;
}

static int test_mmc1a_ram_layouts_and_loader(void) {
    iNESHeader h = header_for(155, 0x80000, true);
    h.flags7 |= 8;
    h.flags10 = 9;
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x80000, 0x2000) == 155);
    serial_write(0xE000, 0x10);
    for (unsigned bank = 0; bank < 4; ++bank) {
        serial_write(0xA000, (uint8_t)(0x10 | (bank << 2)));
        cart_cpu_write(0x6000, (uint8_t)(0xA0 + bank));
        cart_cpu_write(0x7FFF, (uint8_t)(0xB0 + bank));
    }
    for (unsigned bank = 0; bank < 4; ++bank) {
        serial_write(0xA000, (uint8_t)(0x10 | (bank << 2)));
        CHECK(cart_cpu_read(0x6000) == 0xA0 + bank && cart_cpu_read(0x7FFF) == 0xB0 + bank);
        CHECK(cart_cpu_read(0x8000) == 32 && cart_cpu_read(0xC000) == 62);
    }
    serial_write(0x8000, 0x1C);
    serial_write(0xC000, 4);
    CHECK(cart_cpu_read(0x6000) == 0xA1);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 30);
    serial_write(0x8000, 0x0C);
    CHECK(cart_cpu_read(0x6000) == 0xA3 && cart_cpu_read(0xC000) == 62);

    h.flags6 |= 2;
    h.flags10 = 0x77;
    CHECK(fixture_with_header(&h, 0x80000, 0x2000) == 155);
    serial_write(0xE000, 0x10);
    cart_cpu_write(0x6000, 0x35);
    serial_write(0xA000, 8);
    cart_cpu_write(0x6000, 0x73);
    serial_write(0xA000, 4);
    CHECK(cart_cpu_read(0x6000) == 0x35);
    serial_write(0xA000, 8);
    CHECK(cart_cpu_read(0x6000) == 0x73);

    size_t size;
    uint8_t *image = image_for(&h, 0x80000, 0, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0);
    serial_write(0xE000, 0x10);
    cart_cpu_write(0x6000, 0x96);
    uint8_t *previous_prg = prg_rom;
    uint8_t *previous_chr = chr_rom;
    iNESHeader invalid[] = {h, h, h, h};
    invalid[0].prg_ram_size = 0x50; // Fixed-PRG submapper 5 belongs to mapper 1.
    invalid[1].flags10 = 10;        // More than 32KB of PRG-RAM.
    invalid[2].zero[0] = 0x77;     // Separate volatile and nonvolatile CHR chips.
    invalid[3].flags6 &= (uint8_t)~2u;
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        image = image_for(&invalid[i], 0x80000, 0, &size);
        CHECK(image != NULL);
        loaded = load_rom_memory(image, size);
        free(image);
        CHECK(loaded == -1 && prg_rom == previous_prg && chr_rom == previous_chr);
        CHECK(cart_cpu_read(0x6000) == 0x96);
    }

    h.flags6 &= (uint8_t)~2u;
    h.flags10 = 0;
    CHECK(fixture_with_header(&h, 0x80000, 0x2000) == 155);
    serial_write(0xE000, 0x10);
    cart_cpu_write(0x6000, 0x77);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    return 0;
}

static iNESHeader mcacc_header(void) {
    iNESHeader h = header_for(4, 0x20000, false);
    h.flags7 = 8;
    h.prg_ram_size = 0x30;
    h.flags10 = 7;
    h.chr_rom_chunks = 4;
    return h;
}

static void mcacc_falling_edge(uint64_t cycle) {
    cart_notify_ppu_address(0x1000, cycle);
    cart_notify_ppu_address(0x2000, cycle + 1);
}

static int test_mcacc_irq_divider(void) {
    iNESHeader h = mcacc_header();
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_cpu_write(0xC000, 2);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x2000, 0);
    for (unsigned pulse = 1; pulse <= 17; ++pulse) {
        uint64_t cycle = (uint64_t)pulse * 4;
        cart_notify_ppu_address(0x1000, cycle);
        cart_notify_ppu_address(0x1001, cycle + 1);
        CHECK(!cart_irq_pending());
        cart_notify_ppu_address(0x2000, cycle + 2);
        cart_notify_ppu_address(0x2001, cycle + 3);
        CHECK(cart_irq_pending() == (pulse == 17));
    }
    cart_cpu_write(0xE000, 0);
    CHECK(!cart_irq_pending());

    cart->reset();
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xE001, 0);
    mcacc_falling_edge(100);
    CHECK(cart_irq_pending()); // A one-dot high pulse is sufficient.
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    CHECK(!cart_irq_pending()); // Acknowledgment and enable do not reset divider phase.
    for (unsigned pulse = 1; pulse <= 8; ++pulse) {
        mcacc_falling_edge(100 + (uint64_t)pulse * 2);
        CHECK(cart_irq_pending() == (pulse == 8));
    }

    // Reset clears the remembered high level and the divider as well as the IRQ.
    cart_notify_ppu_address(0x1000, 120);
    cart->reset();
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x2000, 121);
    CHECK(!cart_irq_pending());
    mcacc_falling_edge(122);
    CHECK(cart_irq_pending());
    return 0;
}

static int test_mcacc_irq_reload_and_enable(void) {
    iNESHeader h = mcacc_header();
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_cpu_write(0xC000, 1);
    for (unsigned pulse = 0; pulse < 9; ++pulse) mcacc_falling_edge((uint64_t)pulse * 2);
    CHECK(!cart_irq_pending()); // Disabled IRQ output does not stop counting.
    cart_cpu_write(0xE001, 0);
    for (unsigned pulse = 1; pulse <= 16; ++pulse) {
        mcacc_falling_edge(18 + (uint64_t)pulse * 2);
        CHECK(cart_irq_pending() == (pulse == 16));
    }
    cart_cpu_write(0xE000, 0);

    cart->reset();
    cart_cpu_write(0xC000, 4);
    cart_cpu_write(0xE001, 0);
    for (unsigned pulse = 0; pulse < 3; ++pulse) mcacc_falling_edge(100 + (uint64_t)pulse * 2);
    cart_cpu_write(0xC000, 0);
    cart_notify_ppu_address(0x1000, 108);
    cart_cpu_write(0xDFFF, 0); // $C001 mirror resets the divider without forgetting A12.
    CHECK(!cart_irq_pending());
    cart_notify_ppu_address(0x2000, 109);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xC001, 0);
    CHECK(cart_irq_pending()); // Reload requests do not acknowledge an asserted IRQ.
    cart_cpu_write(0xFFFE, 0); // $E000 mirror acknowledges it.
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xFFFF, 0);
    cart_notify_ppu_address(0x2001, 110);
    CHECK(!cart_irq_pending());
    mcacc_falling_edge(111);
    CHECK(cart_irq_pending());
    return 0;
}

static int test_mcacc_banks_ram_and_loader(void) {
    iNESHeader h = mcacc_header();
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 14 && cart_cpu_read(0xE000) == 15);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0x9FFE, 6);
    cart_cpu_write(0x9FFF, 3);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xC000) == 14);
    cart_cpu_write(0x8000, 0x46);
    CHECK(cart_cpu_read(0x8000) == 14 && cart_cpu_read(0xC000) == 3);
    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 5);
    CHECK(cart_ppu_read(0) == 4 && cart_ppu_read(0x0400) == 5);
    cart_cpu_write(0x8000, 0x80);
    CHECK(cart_ppu_read(0x1000) == 4 && cart_ppu_read(0x1400) == 5);
    cart_ppu_write(0x1000, 0x77);
    CHECK(cart_ppu_read(0x1000) == 4);
    cart_cpu_write(0xA000, 1);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);

    // MC-ACC leaves its populated RAM window writable across $A001 settings.
    for (unsigned protect = 0; protect <= 0xC0; protect += 0x40) {
        cart_cpu_write(0xA001, (uint8_t)protect);
        cart_cpu_write(0x6000, (uint8_t)(protect | 0x35));
        cart_cpu_write(0x7FFF, (uint8_t)(protect | 0x1A));
        CHECK(cart_cpu_read(0x6000) == (protect | 0x35));
        CHECK(cart_cpu_read(0x7FFF) == (protect | 0x1A));
    }
    cart->reset();
    CHECK(cart_cpu_read(0x6000) == 0xF5 && !cart_irq_pending());

    size_t size;
    uint8_t *image = image_for(&h, 0x20000, 0x8000, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && ines_header.prg_ram_size == 0x30);
    cart_cpu_write(0x6000, 0x96);
    uint8_t *previous = prg_rom;
    iNESHeader invalid[] = {h, h};
    invalid[0].prg_ram_size = 0x20;
    invalid[1].prg_ram_size = 0x40;
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        image = image_for(&invalid[i], 0x20000, 0x8000, &size);
        CHECK(image != NULL);
        loaded = load_rom_memory(image, size);
        free(image);
        CHECK(loaded == -1 && prg_rom == previous && cart_cpu_read(0x6000) == 0x96);
    }
    h.flags10 = 0;
    h.flags6 |= 8;
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_cpu_write(0x6000, 0x77);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    cart_cpu_write(0xA000, 1);
    CHECK(cart_get_mirroring() == MIRROR_FOUR);
    return 0;
}

static int mcacc_cpu_ppu_address(uint16_t address) {
    const uint8_t program[] = {
        0xA9, (uint8_t)(address >> 8), 0x8D, 0x06, 0x20,
        0xA9, (uint8_t)address, 0x8D, 0x06, 0x20,
        0xEA // Let the delayed PPU address assignment reach the physical bus.
    };
    for (size_t i = 0; i < sizeof(program); ++i)
        write_mem((uint16_t)(0x0200 + i), program[i]);
    cpu.pc = 0x0200;
    const int cycles[] = {2, 4, 2, 4, 2};
    for (size_t i = 0; i < sizeof(cycles) / sizeof(cycles[0]); ++i)
        CHECK(cpu_step(&cpu) == cycles[i]);
    return 0;
}

static int test_mcacc_cpu_ppu_irq_path(void) {
    iNESHeader h = mcacc_header();
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    ppu_step_dots(341 * 262 * 2);
    write_mem(0x2001, 0);
    (void)read_mem(0x2002);
    fixture_prg[0x1FFFE] = 0x00;
    fixture_prg[0x1FFFF] = 0x03;
    write_mem(0xC000, 1);
    write_mem(0xC001, 0);
    write_mem(0xE001, 0);
    for (unsigned pulse = 1; pulse <= 9; ++pulse) {
        CHECK(mcacc_cpu_ppu_address(0x1000) == 0);
        CHECK(!cart_irq_pending());
        CHECK(mcacc_cpu_ppu_address(0x2000) == 0);
        CHECK(cart_irq_pending() == (pulse == 9));
    }
    for (unsigned i = 0; i < 8; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
    cpu.pc = 0x0200;
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    for (unsigned step = 0; step < 3 && cpu.pc != 0x0300; ++step) (void)cpu_step(&cpu);
    CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG));
    CHECK(cart_irq_pending());
    write_mem(0xE000, 0);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_taito_loader_transaction(void) {
    size_t image_size;
    iNESHeader h = header_for(33, 0x8000, false);
    uint8_t *image = image_for(&h, 0x8000, 0x2000, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    image = NULL;
    CHECK(rom_mapper_number(&ines_header) == 33 && cart != NULL);
    h = header_for(48, 0x8000, false);
    h.flags7 |= 0x08;
    h.prg_ram_size = 0x10; // Supported mapper 48 submapper 1.
    image = image_for(&h, 0x8000, 0x2000, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    image = NULL;
    CHECK(rom_mapper_number(&ines_header) == 48 && cart != NULL && cart->clock != NULL);
    uint8_t *previous_prg = prg_rom;
    uint8_t *previous_chr = chr_rom;

    h.prg_ram_size = 0x20; // Unknown mapper 48 submapper 2.
    image = image_for(&h, 0x8000, 0x2000, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == -1);
    free(image);
    CHECK(prg_rom == previous_prg && chr_rom == previous_chr);
    CHECK(rom_mapper_number(&ines_header) == 48);
    return 0;
}

static iNESHeader tqrom_header(size_t chr_bytes) {
    iNESHeader h = header_for(119, 0x20000, false);
    h.flags7 |= 0x08;
    h.flags10 = 7; // 8KB volatile PRG-RAM.
    h.zero[0] = 7; // 8KB volatile CHR-RAM alongside CHR-ROM.
    h.chr_rom_chunks = (uint8_t)(chr_bytes / 0x2000);
    return h;
}

static uint8_t *tqrom_image(const iNESHeader *h, size_t chr_bytes, size_t *size) {
    uint8_t *image = image_for(h, 0x20000, chr_bytes, size);
    if (!image) return NULL;
    size_t prg_offset = sizeof(*h);
    size_t chr_offset = prg_offset + 0x20000;
    for (size_t i = 0; i < 0x20000; ++i)
        image[prg_offset + i] = (uint8_t)(i / 0x2000);
    for (size_t i = 0; i < chr_bytes; ++i)
        image[chr_offset + i] = (uint8_t)(i / 0x0400);
    return image;
}

static int test_tqrom_mixed_chr_memory(void) {
    const size_t chr_bytes = 0x40000;
    iNESHeader h = tqrom_header(chr_bytes);
    size_t image_size;
    uint8_t *image = tqrom_image(&h, chr_bytes, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == 0);

    // TQROM keeps MMC3 PRG banking and work RAM behavior. IRQ timing for the
    // shared board path is covered with elapsed CPU cycles in the dedicated test.
    cart_cpu_write(0x8000, 6);
    cart_cpu_write(0x8001, 3);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xC000) == 14);
    cart_cpu_write(0xA001, 0x80);
    cart_cpu_write(0x6000, 0xA6);
    CHECK(cart_cpu_read(0x6000) == 0xA6);
    cart_cpu_write(0xA000, 1);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    cart_cpu_write(0xA000, 0);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    // CHR pages $40-$7F select the board's 8KB RAM before ROM bank wrapping.
    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 0x40);
    cart_ppu_write(0x0000, 0xD0);
    cart_ppu_write(0x0400, 0xD1);
    CHECK(cart_ppu_read(0x0000) == 0xD0 && cart_ppu_read(0x0400) == 0xD1);

    cart_cpu_write(0x8000, 2);
    cart_cpu_write(0x8001, 0x43);
    cart_ppu_write(0x1000, 0xD3);
    CHECK(cart_ppu_read(0x1000) == 0xD3);
    cart_cpu_write(0x8001, 0x80);
    CHECK(cart_ppu_read(0x1000) == 0x80);
    cart_ppu_write(0x1000, 0xEE);
    CHECK(cart_ppu_read(0x1000) == 0x80); // CHR-ROM stays read-only.
    cart_cpu_write(0x8001, 0x43);
    CHECK(cart_ppu_read(0x1000) == 0xD3);
    cart_cpu_write(0x8001, 0x4B);
    CHECK(cart_ppu_read(0x1000) == 0xD3); // RAM banks alias every eight pages.

    // Inverted CHR mode moves the 1KB and paired registers to the opposite half.
    cart_cpu_write(0x8000, 0x82);
    cart_cpu_write(0x8001, 0x45);
    cart_ppu_write(0x0000, 0xE5);
    CHECK(cart_ppu_read(0x0000) == 0xE5);
    cart_cpu_write(0x8001, 0x85);
    CHECK(cart_ppu_read(0x0000) == 0x85);
    cart_ppu_write(0x0000, 0x5E);
    CHECK(cart_ppu_read(0x0000) == 0x85);
    cart_cpu_write(0x8001, 0x45);
    CHECK(cart_ppu_read(0x0000) == 0xE5);
    cart_cpu_write(0x8000, 0x80);
    cart_cpu_write(0x8001, 0x40);
    CHECK(cart_ppu_read(0x1000) == 0xD0 && cart_ppu_read(0x1400) == 0xD1);

    // The shared MMC3 board retains its bank registers across a mapper reset.
    cart->reset();
    CHECK(!cart_irq_pending() && cart_ppu_read(0x0000) == 0xE5);
    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 0x40);
    CHECK(cart_ppu_read(0x0000) == 0xD0);

    // Replacing the cartridge clears mapper-owned CHR-RAM.
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    image = NULL;
    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 0x40);
    CHECK(cart_ppu_read(0x0000) == 0);

    // Legacy mapper 119 headers imply the board's fixed 8KB CHR-RAM.
    iNESHeader legacy = header_for(119, 0x20000, false);
    legacy.chr_rom_chunks = (uint8_t)(chr_bytes / 0x2000);
    image = tqrom_image(&legacy, chr_bytes, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    image = NULL;
    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 0x40);
    CHECK(cart_ppu_read(0x0000) == 0);
    cart_ppu_write(0x0000, 0x6C);

    return 0;
}

static int test_mmc3_mixed_chr_variants(void) {
    static const struct {
        unsigned mapper;
        uint8_t first_ram_bank;
        uint8_t last_ram_bank;
        uint8_t ram_shift;
        unsigned ram_pages;
    } cases[] = {
        {74,  0x08, 0x09, 5, 2},
        {191, 0x80, 0xFF, 5, 2},
        {192, 0x08, 0x0B, 6, 4},
        {194, 0x00, 0x01, 5, 2},
        {195, 0x00, 0x03, 6, 4}
    };
    const size_t chr_bytes = 0x40000;

    for (size_t n = 0; n < sizeof(cases) / sizeof(cases[0]); ++n) {
        iNESHeader h = header_for(cases[n].mapper, 0x20000, false);
        h.flags7 |= 0x08;
        h.flags10 = 7;
        h.zero[0] = cases[n].ram_shift;
        h.chr_rom_chunks = (uint8_t)(chr_bytes / 0x2000);
        size_t image_size;
        uint8_t *image = tqrom_image(&h, chr_bytes, &image_size);
        CHECK(image != NULL && load_rom_memory(image, image_size) == 0);

        cart_cpu_write(0x8000, 6);
        cart_cpu_write(0x8001, 3);
        CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xC000) == 14);
        cart_cpu_write(0xA001, 0x80);
        cart_cpu_write(0x6000, (uint8_t)(0x50 + n));
        CHECK(cart_cpu_read(0x6000) == 0x50 + n);

        uint8_t rom_bank = cases[n].first_ram_bank
                         ? (uint8_t)(cases[n].first_ram_bank - 1)
                         : (uint8_t)(cases[n].last_ram_bank + 1);
        cart_cpu_write(0x8000, 2);
        cart_cpu_write(0x8001, rom_bank);
        CHECK(cart_ppu_read(0x1000) == rom_bank);
        cart_ppu_write(0x1000, 0xEE);
        CHECK(cart_ppu_read(0x1000) == rom_bank);

        if (cases[n].last_ram_bank < 0xFF) {
            uint8_t following = (uint8_t)(cases[n].last_ram_bank + 1);
            cart_cpu_write(0x8001, following);
            CHECK(cart_ppu_read(0x1000) == following);
            cart_ppu_write(0x1000, 0xEE);
            CHECK(cart_ppu_read(0x1000) == following);
        }

        cart_cpu_write(0x8001, cases[n].first_ram_bank);
        cart_ppu_write(0x1000, (uint8_t)(0xA0 + n));
        CHECK(cart_ppu_read(0x1000) == 0xA0 + n);
        cart_cpu_write(0x8001, cases[n].last_ram_bank);
        cart_ppu_write(0x1000, (uint8_t)(0xB0 + n));
        CHECK(cart_ppu_read(0x1000) == 0xB0 + n);
        cart_cpu_write(0x8001, cases[n].first_ram_bank);
        CHECK(cart_ppu_read(0x1000) == 0xA0 + n);

        if ((unsigned)(cases[n].last_ram_bank - cases[n].first_ram_bank + 1) > cases[n].ram_pages) {
            cart_cpu_write(0x8001, (uint8_t)(cases[n].first_ram_bank + cases[n].ram_pages));
            CHECK(cart_ppu_read(0x1000) == 0xA0 + n);
        }

        cart_cpu_write(0x8000, 0);
        cart_cpu_write(0x8001, cases[n].first_ram_bank);
        cart_ppu_write(0x0000, (uint8_t)(0xC0 + n));
        cart_ppu_write(0x0400, (uint8_t)(0xD0 + n));
        CHECK(cart_ppu_read(0x0000) == 0xC0 + n && cart_ppu_read(0x0400) == 0xD0 + n);

        cart_cpu_write(0x8000, 0x82);
        cart_cpu_write(0x8001, cases[n].first_ram_bank);
        cart_ppu_write(0x0000, (uint8_t)(0xE0 + n));
        CHECK(cart_ppu_read(0x0000) == 0xE0 + n);

        cart->reset();
        cart_cpu_write(0x8000, 2);
        cart_cpu_write(0x8001, cases[n].first_ram_bank);
        CHECK(cart_ppu_read(0x1000) == 0xE0 + n);

        h.zero[0] = 0;
        uint8_t *zero_ram = tqrom_image(&h, chr_bytes, &image_size);
        CHECK(zero_ram != NULL && load_rom_memory(zero_ram, image_size) == 0);
        free(zero_ram);
        cart_cpu_write(0x8000, 2);
        cart_cpu_write(0x8001, cases[n].first_ram_bank);
        CHECK(cart_ppu_read(0x1000) == 4);

        iNESHeader legacy = header_for(cases[n].mapper, 0x20000, false);
        legacy.chr_rom_chunks = (uint8_t)(chr_bytes / 0x2000);
        uint8_t *legacy_image = tqrom_image(&legacy, chr_bytes, &image_size);
        CHECK(legacy_image != NULL && load_rom_memory(legacy_image, image_size) == 0);
        free(legacy_image);
        cart_cpu_write(0x8000, 2);
        cart_cpu_write(0x8001, cases[n].first_ram_bank);
        cart_ppu_write(0x1000, 0x69);
        CHECK(cart_ppu_read(0x1000) == 0x69);
        free(image);
    }
    return 0;
}

static iNESHeader jy_header(unsigned mapper) {
    iNESHeader h = header_for(mapper, 0x100000, false);
    h.flags7 |= 0x08;
    h.prg_rom_chunks = 0x40; // 1 MiB PRG-ROM.
    h.chr_rom_chunks = 0x40; // 512 KiB CHR-ROM.
    return h;
}

static void jy_write_chr(unsigned index, uint16_t bank) {
    cart_cpu_write((uint16_t)(0x9000 + index), (uint8_t)bank);
    cart_cpu_write((uint16_t)(0xA000 + index), (uint8_t)(bank >> 8));
}

static void jy_arm_irq(uint8_t source) {
    cart_cpu_write(0xC002, 0);
    cart_cpu_write(0xC006, 0x55);
    cart_cpu_write(0xC004, 0x52); // XOR -> prescaler 7.
    cart_cpu_write(0xC005, 0xAA); // XOR -> counter $FF.
    cart_cpu_write(0xC001, (uint8_t)(0x44 | source)); // Up, 3-bit prescaler.
    cart_cpu_write(0xC003, 0);
}

static int test_jy_prg_chr_modes(void) {
    iNESHeader h = jy_header(90);
    CHECK(fixture_with_header(&h, 0x100000, 0x80000) == 90);

    // Mode 0 fixes the last 32 KiB unless bit 2 selects register 3.
    CHECK(cart_cpu_read(0x8000) == 60 && cart_cpu_read(0xA000) == 61);
    CHECK(cart_cpu_read(0xC000) == 62 && cart_cpu_read(0xE000) == 63);
    cart_cpu_write(0x8003, 5);
    cart_cpu_write(0xD000, 0x04);
    CHECK(cart_cpu_read(0x8000) == 5 && cart_cpu_read(0xA000) == 6);
    CHECK(cart_cpu_read(0xC000) == 7 && cart_cpu_read(0xE000) == 8);

    // Mode 1 is 16 KiB + 16 KiB, with independently selectable upper pair.
    cart_cpu_write(0x8001, 7);
    cart_cpu_write(0x8003, 9);
    cart_cpu_write(0xD000, 0x01);
    CHECK(cart_cpu_read(0x8000) == 14 && cart_cpu_read(0xA000) == 15);
    CHECK(cart_cpu_read(0xC000) == 62 && cart_cpu_read(0xE000) == 63);
    cart_cpu_write(0xD000, 0x85); // Register-3 upper pair + $6000 PRG window.
    CHECK(cart_cpu_read(0xC000) == 9 && cart_cpu_read(0xE000) == 10);
    CHECK(cart_cpu_read(0x6000) == 19);

    // Modes 2 and 3 expose 8 KiB registers; mode 3 reverses register bit order.
    cart_cpu_write(0x8000, 3);
    cart_cpu_write(0x8001, 4);
    cart_cpu_write(0x8002, 5);
    cart_cpu_write(0x8003, 6);
    cart_cpu_write(0xD000, 0x02);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xA000) == 4);
    CHECK(cart_cpu_read(0xC000) == 5 && cart_cpu_read(0xE000) == 63);
    cart_cpu_write(0xD000, 0x86);
    CHECK(cart_cpu_read(0xE000) == 6 && cart_cpu_read(0x6000) == 6);

    cart_cpu_write(0x8000, 1);
    cart_cpu_write(0x8001, 2);
    cart_cpu_write(0x8002, 4);
    cart_cpu_write(0x8003, 0x10);
    cart_cpu_write(0xD000, 0x03);
    CHECK(cart_cpu_read(0x8000) == 64 && cart_cpu_read(0xA000) == 32);
    CHECK(cart_cpu_read(0xC000) == 16 && cart_cpu_read(0xE000) == 63);
    cart_cpu_write(0xD000, 0x87);
    CHECK(cart_cpu_read(0xE000) == 4 && cart_cpu_read(0x6000) == 4);

    // CHR mode 0: one 8 KiB bank with block bits.
    cart_cpu_write(0x9000, 3);
    cart_cpu_write(0xD003, 0x01); // Block 1, block mode.
    cart_cpu_write(0xD000, 0x00);
    CHECK(cart_ppu_read(0x0000) == (uint8_t)(35 * 8));
    CHECK(cart_ppu_read(0x1C00) == (uint8_t)(35 * 8 + 7));

    // CHR mode 1: two 4 KiB windows, selected by the latch registers.
    cart_cpu_write(0x9000, 1);
    cart_cpu_write(0x9004, 2);
    cart_cpu_write(0xD003, 0x01);
    cart_cpu_write(0xD000, 0x08);
    CHECK(cart_ppu_read(0x0000) == (uint8_t)(65 * 4));
    CHECK(cart_ppu_read(0x1000) == (uint8_t)(66 * 4));

    // CHR mode 2 uses registers 0/2/4/6 as four 2 KiB windows.
    cart_cpu_write(0x9000, 1);
    cart_cpu_write(0x9002, 2);
    cart_cpu_write(0x9004, 3);
    cart_cpu_write(0x9006, 4);
    cart_cpu_write(0xD003, 0x01);
    cart_cpu_write(0xD000, 0x10);
    CHECK(cart_ppu_read(0x0000) == (uint8_t)(129 * 2));
    CHECK(cart_ppu_read(0x0800) == (uint8_t)(130 * 2));
    CHECK(cart_ppu_read(0x1000) == (uint8_t)(131 * 2));
    CHECK(cart_ppu_read(0x1800) == (uint8_t)(132 * 2));

    // CHR mode 3 exposes all eight registers and the high-byte fields.
    for (unsigned i = 0; i < 8; ++i) jy_write_chr(i, (uint16_t)(0x100 + i));
    cart_cpu_write(0xD003, 0x20); // High-register mode.
    cart_cpu_write(0xD000, 0x18);
    for (unsigned i = 0; i < 8; ++i)
        CHECK(cart_ppu_read((uint16_t)(i * 0x400)) == i);
    jy_write_chr(0, 0x101);
    jy_write_chr(1, 0x102);
    jy_write_chr(2, 0x107);
    jy_write_chr(3, 0x108);
    cart_cpu_write(0xD003, 0xA0); // Mirror registers 2/3 from 0/1.
    CHECK(cart_ppu_read(0x0800) == 1 && cart_ppu_read(0x0C00) == 2);

    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 60 && cart_cpu_read(0xE000) == 63);
    CHECK(cart_ppu_read(0) == 0 && !cart_irq_pending());
    return 0;
}

static int test_jy_mapper_variants_and_readback(void) {
    uint8_t nt[0x1000] = {0};
    for (unsigned page = 0; page < 4; ++page) nt[page * 0x400] = (uint8_t)(0x10 + page);

    iNESHeader h = jy_header(90);
    CHECK(fixture_with_header(&h, 0x100000, 0x80000) == 90);
    cart_cpu_write(0x5800, 0xFF);
    cart_cpu_write(0x5801, 0xFE);
    CHECK(cart_cpu_read(0x5800) == 0x02 && cart_cpu_read(0x5801) == 0xFD);
    cart_cpu_write(0x5803, 0xA6);
    CHECK(cart_cpu_read(0x5803) == 0xA6);
    CHECK(cart_cpu_read(0x5000) == 0 && cart_cpu_read_bus(0x5002, 0x5A) == 0x5A);

    // Mapper 90 always uses its simple mirroring register.
    cart_cpu_write(0xD000, 0x20);
    cart_cpu_write(0xB000, 0x81);
    cart_cpu_write(0xB004, 0x01);
    cart_cpu_write(0xD001, 1);
    CHECK(cart_nt_read(0x2000, nt) == 0x10 && cart_nt_read(0x2400, nt) == 0x10);
    CHECK(cart_nt_read(0x2800, nt) == 0x11 && cart_nt_read(0x2C00, nt) == 0x11);

    // Mapper 209 only enters advanced nametable mode when requested and has MMC2-style CHR latches.
    h = jy_header(209);
    CHECK(fixture_with_header(&h, 0x100000, 0x80000) == 209);
    jy_write_chr(0, 1); jy_write_chr(2, 3); jy_write_chr(4, 5); jy_write_chr(6, 7);
    cart_cpu_write(0xD003, 0x20);
    cart_cpu_write(0xD000, 0x08);
    CHECK(cart_ppu_read(0) == 4 && cart_ppu_read(0x1000) == 20);
    cart_notify_ppu_address(0x0FE8, 1);
    cart_notify_ppu_address(0x1FE8, 2);
    CHECK(cart_ppu_read(0) == 12 && cart_ppu_read(0x1000) == 28);
    cart_notify_ppu_address(0x0FD8, 3);
    cart_notify_ppu_address(0x1FD8, 4);
    CHECK(cart_ppu_read(0) == 4 && cart_ppu_read(0x1000) == 20);

    cart_cpu_write(0xB000, 0x82); // CHR page $82 when bit 7 mismatches D002.
    cart_cpu_write(0xB004, 0);
    cart_cpu_write(0xD002, 0);
    cart_cpu_write(0xD000, 0x28);
    CHECK(cart_nt_read(0x2000, nt) == 0x82);
    cart_cpu_write(0xD002, 0x80);
    CHECK(cart_nt_read(0x2000, nt) == 0x10); // Matching select bit uses CIRAM page 0.
    cart_nt_write(0x2000, 0x66, nt);
    CHECK(nt[0] == 0x66);

    // Mapper 211 behaves as advanced nametable mode even when D000 bit 5 is clear.
    h = jy_header(211);
    CHECK(fixture_with_header(&h, 0x100000, 0x80000) == 211);
    cart_cpu_write(0xB000, 0x81);
    cart_cpu_write(0xB001, 0x80);
    cart_cpu_write(0xB002, 0x81);
    cart_cpu_write(0xB003, 0x80);
    cart_cpu_write(0xD002, 0x80);
    CHECK(cart_nt_read(0x2000, nt) == nt[0x400]);
    CHECK(cart_nt_read(0x2400, nt) == nt[0]);
    cart_cpu_write(0xD002, 0);
    CHECK(cart_nt_read(0x2000, nt) == 0x81); // Bit mismatch selects CHR page $81.
    cart_cpu_write(0xD000, 0x40); // Disable NT RAM forces CHR reads for every quadrant.
    CHECK(cart_nt_read(0x2400, nt) == 0x80);
    cart_nt_write(0x2000, 0x77, nt);
    CHECK(nt[0x400] == 0x77); // Writes still reach the selected CIRAM page.
    return 0;
}

static int test_jy_chr_ram_nametable_reads(void) {
    const unsigned boards[] = {90, 209, 211};
    for (unsigned i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        iNESHeader h = header_for(boards[i], 0x8000, true);
        h.flags7 |= 8;
        h.zero[0] = 7;
        size_t size;
        uint8_t *image = image_for(&h, 0x8000, 0, &size);
        CHECK(image != NULL);
        int loaded = load_rom_memory(image, size);
        free(image);
        CHECK(loaded == 0 && rom_mapper_number(&ines_header) == (int)boards[i]);
        uint8_t nt[0x1000] = {0};
        nt[0x123] = 0xA6;
        cart_ppu_write(0x0123, 0x5A);
        CHECK(cart_ppu_read(0x0123) == 0x5A);
        cart_cpu_write(0xD000, 0x60);
        cart_cpu_write(0xB000, 0);
        cart_cpu_write(0xB004, 0);
        // The advanced nametable read port selects ROM, never pattern RAM.
        CHECK(cart_nt_read(0x2123, nt) == (boards[i] == 90 ? 0xA6 : 0));
        cart_nt_write(0x2123, 0x69, nt);
        CHECK(nt[0x123] == 0x69 && cart_ppu_read(0x0123) == 0x5A);
        cart_cpu_write(0xD000, 0x20);
        CHECK(cart_nt_read(0x2123, nt) == 0x69);
    }
    return 0;
}

static int test_jy_irq_sources_and_cpu_delivery(void) {
    iNESHeader h = jy_header(90);
    CHECK(fixture_with_header(&h, 0x100000, 0x80000) == 90);

    jy_arm_irq(0);
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xC002, 0);
    CHECK(!cart_irq_pending());

    jy_arm_irq(1);
    cart_notify_ppu_address(0x0000, 0);
    cart_notify_ppu_address(0x1000, 1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xC002, 0);
    cart_notify_ppu_address(0x1001, 2);
    CHECK(!cart_irq_pending()); // Held-high A12 is not another edge.

    // Rendering nametable and pattern reads both clock source 2.
    jy_arm_irq(2);
    ppu_power_on(&ppu);
    ppu.scanline = 0; ppu.dot = 1; ppu.v = 0;
    ppu.mask = 0x08; ppu.rendering_enabled = true; ppu.fetches_enabled = true;
    ppu_step_dots(2);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xC002, 0);
    jy_arm_irq(2);
    ppu.dot = 5;
    ppu_step_dots(2);
    CHECK(cart_irq_pending());

    // CPU $2007 reads use the CPU fetch source and do not clock source 2.
    cart_cpu_write(0xC002, 0);
    jy_arm_irq(2);
    CHECK(!cart_irq_pending());
    apu_power_on(&apu);
    CHECK(!cart_irq_pending());
    ppu_power_on(&ppu);
    CHECK(!cart_irq_pending());
    cpu_power_on(&cpu);
    CHECK(!cart_irq_pending());
    write_mem(0x2006, 0x00);
    CHECK(!cart_irq_pending());
    write_mem(0x2006, 0x00);
    CHECK(!cart_irq_pending());
    (void)read_mem(0x2007);
    CHECK(!cart_irq_pending());

    // Direct mapper clocks are not CPU writes; an executed STA is.
    jy_arm_irq(3);
    cart->clock(8);
    CHECK(!cart_irq_pending());
    write_mem(0x0200, 0xA9); write_mem(0x0201, 0x42); // LDA #$42
    write_mem(0x0202, 0x85); write_mem(0x0203, 0x10); // STA $10
    cpu.pc = 0x0200;
    CHECK(cpu_step(&cpu) == 2 && !cart_irq_pending());
    CHECK(cpu_step(&cpu) == 3 && cart_irq_pending() && read_mem(0x0010) == 0x42);

    // OAM DMA cycles remain DMA reads for this source, including its PUT phases.
    cart_cpu_write(0xC002, 0);
    write_mem(0x4014, 0x02);
    write_mem(0x0204, 0xEA);
    jy_arm_irq(3);
    cpu.pc = 0x0204;
    int dma_cycles = cpu_step(&cpu);
    CHECK(dma_cycles >= 515 && dma_cycles <= 516);
    CHECK(!cart_irq_pending());

    // Counter XOR and downward counting boundaries.
    cart_cpu_write(0xC002, 0);
    cart_cpu_write(0xC006, 0xA5);
    cart_cpu_write(0xC004, 0xA4); // XOR -> 1.
    cart_cpu_write(0xC005, 0xA5); // XOR -> 0.
    cart_cpu_write(0xC001, 0x84); // Down, 3-bit prescaler, CPU clock.
    cart_cpu_write(0xC000, 1);
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xC000, 0);
    CHECK(!cart_irq_pending());

    // Asserted JY IRQ reaches the production CPU interrupt path.
    cart->reset();
    fixture_prg[0x7FFFE] = 0x00;
    fixture_prg[0x7FFFF] = 0x03;
    cpu_power_on(&cpu);
    cpu.pc = 0x0200;
    cpu.status = UNUSED_FLAG | INTERRUPT_FLAG;
    write_mem(0x0200, 0xEA);
    write_mem(0x0201, 0xEA);
    jy_arm_irq(0);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    for (unsigned i = 0; i < 3 && cpu.pc != 0x0300; ++i) (void)cpu_step(&cpu);
    CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG));
    return 0;
}

static int test_jy_loader_and_reset(void) {
    iNESHeader h = jy_header(211);
    size_t size;
    uint8_t *image = image_for(&h, 0x100000, 0x80000, &size);
    CHECK(image != NULL && load_rom_memory(image, size) == 0);
    free(image);
    cart_cpu_write(0x5803, 0x91);
    cart_cpu_write(0xD001, 3);
    CHECK(cart_cpu_read(0x5803) == 0x91 && cart_get_mirroring() == MIRROR_SINGLE1);
    cart->reset();
    CHECK(cart_cpu_read(0x5803) == 0 && !cart_irq_pending());

    uint8_t *previous_prg = prg_rom;
    uint8_t *previous_chr = chr_rom;
    h.prg_ram_size = 0x10; // Unsupported nonzero submapper.
    image = image_for(&h, 0x100000, 0x80000, &size);
    CHECK(image != NULL && load_rom_memory(image, size) == -1);
    free(image);
    CHECK(prg_rom == previous_prg && chr_rom == previous_chr && rom_mapper_number(&ines_header) == 211);

    h = jy_header(90);
    h.flags10 = 7; // These boards do not expose CPU PRG-RAM.
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x100000, fixture_chr, 0x80000) == -1);
    CHECK(prg_rom == previous_prg && chr_rom == previous_chr);
    return 0;
}

static uint8_t *txsrom_image(size_t *size) {
    const size_t prg_bytes = 0x20000, chr_bytes = 0x40000;
    iNESHeader h = header_for(118, prg_bytes, false);
    h.flags7 |= 8;
    h.flags10 = 7;
    h.chr_rom_chunks = (uint8_t)(chr_bytes / 0x2000);
    uint8_t *image = image_for(&h, prg_bytes, chr_bytes, size);
    if (!image) return NULL;
    for (size_t i = 0; i < prg_bytes; ++i) image[sizeof(h) + i] = (uint8_t)(i / 0x2000);
    for (size_t i = 0; i < chr_bytes; ++i) image[sizeof(h) + prg_bytes + i] = (uint8_t)(i / 0x0400);
    return image;
}

static int test_txsrom_nametable_routing(void) {
    size_t image_size;
    uint8_t *image = txsrom_image(&image_size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, image_size);
    free(image);
    CHECK(loaded == 0);
    const uint16_t offset = 0x012;
    memset(ppu_vram, 0, NT_RAM_SIZE);
    ppu_vram[offset] = 0x10;
    ppu_vram[0x400 + offset] = 0x20;
    cart_cpu_write(0x9FFE, 0);
    cart_cpu_write(0x9FFF, 0x80);
    cart_cpu_write(0x8000, 1);
    cart_cpu_write(0x8001, 0);
    CHECK(cart_ppu_read(0) == 0x80 && cart_ppu_read(0x0400) == 0x81);
    CHECK(ppu_read(0x2000 + offset) == 0x20 && ppu_read(0x2400 + offset) == 0x20);
    CHECK(ppu_read(0x2800 + offset) == 0x10 && ppu_read(0x2C00 + offset) == 0x10);
    ppu_write(0x2400 + offset, 0xA1);
    ppu_write(0x2C00 + offset, 0xB2);
    CHECK(ppu_vram[0x400 + offset] == 0xA1 && ppu_vram[offset] == 0xB2);
    CHECK(ppu_read(0x3000 + offset) == 0xA1 && ppu_read(0x3800 + offset) == 0xB2);

    cart_cpu_write(0x8000, 0x82);
    CHECK(ppu_read(0x2000 + offset) == 0xA1 && ppu_read(0x2400 + offset) == 0xA1);
    CHECK(ppu_read(0x2800 + offset) == 0xB2 && ppu_read(0x2C00 + offset) == 0xB2);
    cart_cpu_write(0x8001, 0x80);
    cart_cpu_write(0x8000, 0x83);
    cart_cpu_write(0x8001, 0);
    cart_cpu_write(0x8000, 0x84);
    cart_cpu_write(0x8001, 0x80);
    cart_cpu_write(0x8000, 0x85);
    cart_cpu_write(0x8001, 0);
    CHECK(cart_ppu_read(0) == 0x80 && cart_ppu_read(0x0800) == 0x80);
    CHECK(ppu_read(0x2000 + offset) == 0xA1 && ppu_read(0x2400 + offset) == 0xB2);
    CHECK(ppu_read(0x2800 + offset) == 0xA1 && ppu_read(0x2C00 + offset) == 0xB2);
    for (unsigned reg = 0; reg < 8; ++reg) {
        if (reg >= 2 && reg <= 5) continue;
        cart_cpu_write(0x8000, (uint8_t)(0x80 | reg));
        cart_cpu_write(0x8001, 0xFF);
        CHECK(ppu_read(0x2000 + offset) == 0xA1 && ppu_read(0x2400 + offset) == 0xB2);
        CHECK(ppu_read(0x2800 + offset) == 0xA1 && ppu_read(0x2C00 + offset) == 0xB2);
    }
    cart_cpu_write(0xA000, 1);
    cart_cpu_write(0xBFFE, 0);
    CHECK(ppu_read(0x2000 + offset) == 0xA1 && ppu_read(0x2400 + offset) == 0xB2);
    CHECK(ppu_read(0x2800 + offset) == 0xA1 && ppu_read(0x2C00 + offset) == 0xB2);
    cart_cpu_write(0x8000, 6);
    cart_cpu_write(0x8001, 3);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xC000) == 14);
    cart_cpu_write(0xA001, 0x80);
    cart_cpu_write(0x6000, 0x6A);
    CHECK(cart_cpu_read(0x6000) == 0x6A);
    cart_cpu_write(0xA001, 0xC0);
    cart_cpu_write(0x6000, 0xFF);
    CHECK(cart_cpu_read(0x6000) == 0x6A);
    cart_cpu_write(0xA001, 0);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    // Nametable accesses above have already driven A12 at the running PPU time.
    uint64_t irq_cycle = ppu.total_cycles;
    cart_notify_ppu_address(0x1000, irq_cycle);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x2000, irq_cycle + 3);
    cart_notify_ppu_address(0x1000, irq_cycle + 9);
    CHECK(!cart_irq_pending());
    a12_pulse(irq_cycle + 12);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_txsrom_startup_ram_and_loader(void) {
    const uint8_t flags[] = {0, 1, 8};
    const uint8_t pages[][4] = {{0, 0, 1, 1}, {0, 1, 0, 1}, {0, 1, 2, 3}};
    uint8_t nt[0x1000] = {0};
    for (unsigned page = 0; page < 4; ++page) nt[page * 0x400] = (uint8_t)(0x10 + page);
    for (unsigned i = 0; i < sizeof(flags); ++i) {
        iNESHeader h = header_for(118, 0x20000, true);
        h.flags6 |= flags[i];
        CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 118);
        for (unsigned page = 0; page < 4; ++page)
            CHECK(cart_nt_read((uint16_t)(0x2000 + page * 0x400), nt) == 0x10 + pages[i][page]);
        cart_cpu_write(0x8000, 0);
        cart_cpu_write(0x8001, 0x84);
        cart_ppu_write(0x0123, 0x96);
        cart_cpu_write(0x8001, 4);
        CHECK(cart_ppu_read(0x0123) == 0x96);
        CHECK(cart_nt_read(0x2000, nt) == 0x10 && cart_nt_read(0x2400, nt) == 0x10);
        cart_cpu_write(0x8001, 0x80);
        cart->reset();
        for (unsigned page = 0; page < 4; ++page)
            CHECK(cart_nt_read((uint16_t)(0x2000 + page * 0x400), nt) == 0x10 + pages[i][page]);
    }
    size_t bytes;
    uint8_t *image = txsrom_image(&bytes);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, bytes);
    CHECK(loaded == 0);
    uint8_t *old_prg = prg_rom, *old_chr = chr_rom;
    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 0x80);
    iNESHeader *h = (iNESHeader *)image;
    h->prg_ram_size = 0x10;
    CHECK(load_rom_memory(image, bytes) == -1);
    h->prg_ram_size = 0;
    h->flags10 = 7;
    CHECK(load_rom_memory(image, bytes - 1) == -1);
    free(image);
    CHECK(prg_rom == old_prg && chr_rom == old_chr);
    CHECK(cart_ppu_read(0) == 0x80 && cart_nt_read(0x2000, nt) == 0x11);
    return 0;
}

#endif // MAPPER_ACCURACY_LOADER_H

