/*
 * mapper_accuracy_persistence.h - Cartridge persistence regression tests
 *
 * Author: @frankischilling
 *
 * These checks verify cartridge RAM loading, saving, and battery backed storage.
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
#ifndef MAPPER_ACCURACY_PERSISTENCE_H
#define MAPPER_ACCURACY_PERSISTENCE_H

static int test_chr_nvram_writers(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    int result = chr_writer_cases(&paths);
    return result | save_fixture_end(&paths);
}

static iNESHeader mmc6_header(bool battery) {
    iNESHeader h = header_for(4, 0x20000, false);
    h.flags7 = 8;
    h.prg_ram_size = 0x10;
    h.flags10 = battery ? 0x40 : 4;
    if (battery) h.flags6 |= 2;
    return h;
}

static int test_mmc6_ram_mirroring(void) {
    iNESHeader h = mmc6_header(false);
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_cpu_write(0xA001, 0xF0); // Writes are ignored until global RAM enable.
    cart_cpu_write(0x8000, 0x20);
    CHECK(cart_cpu_read_bus(0x7000, 0xA6) == 0xA6);
    cart_cpu_write(0xA001, 0xF0);
    for (unsigned offset = 0; offset < 0x400; ++offset)
        cart_cpu_write((uint16_t)(0x7000 + offset), (uint8_t)(offset ^ (offset >> 1) ^ 0x5A));
    for (unsigned offset = 0; offset < 0x1000; ++offset) {
        unsigned index = offset & 0x3FF;
        uint8_t expected = (uint8_t)(index ^ (index >> 1) ^ 0x5A);
        CHECK(cart_cpu_read_bus((uint16_t)(0x7000 + offset), 0xA6) == expected);
        CHECK(cart_cpu_read_bus((uint16_t)(0x6000 + offset), 0xA6) == 0xA6);
        cart_cpu_write((uint16_t)(0x6000 + offset), 0);
    }
    CHECK(cart_cpu_read(0x7000) == 0x5A);
    cart_cpu_write(0x7FFF, 0x91);
    CHECK(cart_cpu_read(0x73FF) == 0x91 && cart_cpu_read(0x77FF) == 0x91);
    cart_cpu_write(0x8000, 0);
    CHECK(cart_cpu_read_bus(0x7000, 0xD3) == 0xD3);
    cart_cpu_write(0xA001, 0xF0);
    cart_cpu_write(0x8000, 0x20);
    CHECK(cart_cpu_read_bus(0x7000, 0xD3) == 0xD3); // Disabled state cleared the protect latch.
    cart_cpu_write(0xA001, 0xF0);
    CHECK(cart_cpu_read(0x7000) == 0x5A && cart_cpu_read(0x7FFF) == 0x91);
    h.flags10 = 7;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x8000) == -1);
    CHECK(cart_cpu_read(0x7FFF) == 0x91); // MMC6 cannot be expanded to 8KB by a header.
    h.flags10 = 0;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x8000) == -1);
    return 0;
}

static int test_mmc6_protection(void) {
    iNESHeader h = mmc6_header(false);
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_cpu_write(0x8000, 0x20);
    for (unsigned protection = 0; protection <= 0xF0; protection += 0x10) {
        cart_cpu_write(0xA001, 0xF0);
        cart_cpu_write(0x7001, 0x11);
        cart_cpu_write(0x7201, 0x22);
        cart_cpu_write(0xBFFF, (uint8_t)(protection | 0x0F)); // Register mirror; low bits ignored.
        cart_cpu_write(0x7401, 0x33);
        cart_cpu_write(0x7601, 0x44);
        bool first_write = (protection & 0x30) == 0x30;
        bool second_write = (protection & 0xC0) == 0xC0;
        uint8_t first_data = first_write ? 0x33 : 0x11;
        uint8_t second_data = second_write ? 0x44 : 0x22;
        uint8_t first_read = !(protection & 0xA0) ? 0xA6
                           : (protection & 0x20) ? first_data : 0;
        uint8_t second_read = !(protection & 0xA0) ? 0xA6
                            : (protection & 0x80) ? second_data : 0;
        CHECK(cart_cpu_read_bus(0x7C01, 0xA6) == first_read);
        CHECK(cart_cpu_read_bus(0x7E01, 0xA6) == second_read);
        cart_cpu_write(0xA001, 0xF0);
        CHECK(cart_cpu_read(0x7001) == first_data && cart_cpu_read(0x7201) == second_data);
    }
    return 0;
}

static int test_mmc6_banks_and_irq(void) {
    for (unsigned submapper = 0; submapper < 2; ++submapper) {
        iNESHeader h = mmc6_header(false);
        h.prg_ram_size = (uint8_t)(submapper << 4);
        if (!submapper) h.flags10 = 7;
        CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
        cart_cpu_write(0x9FFE, 0x26);
        cart_cpu_write(0x9FFF, 3);
        CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xC000) == 14);
        cart_cpu_write(0x8000, 0x66);
        CHECK(cart_cpu_read(0x8000) == 14 && cart_cpu_read(0xC000) == 3);
        cart_cpu_write(0x8000, 0xA0);
        cart_cpu_write(0x8001, 5);
        CHECK(cart_ppu_read(0x1000) == 4 && cart_ppu_read(0x1400) == 5);
        cart_cpu_write(0xA000, 1);
        CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
        cart_cpu_write(0xA000, 0);
        CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
        cart_cpu_write(0xC000, 1);
        cart_cpu_write(0xC001, 0);
        cart_cpu_write(0xE001, 0);
        a12_pulse(0);
        CHECK(!cart_irq_pending());
        cart_cpu_write(0xC000, 0);
        a12_pulse(12);
        CHECK(cart_irq_pending());
        cart_cpu_write(0xE000, 0);
        cart_cpu_write(0xE001, 0);
        a12_pulse(24);
        CHECK(cart_irq_pending()); // Sharp MMC3 and MMC6 both reassert on a zero reload.
        cart_cpu_write(0xE000, 0);
        cart_cpu_write(0xC001, 0);
        cart_cpu_write(0x8000, 0); // Disabling RAM does not disable the IRQ counter.
        cart_cpu_write(0xE001, 0);
        a12_pulse(36);
        CHECK(cart_irq_pending());
    }
    return 0;
}

static void sunsoft69_command(uint8_t command, uint8_t value) {
    cart_cpu_write(0x8000, command);
    cart_cpu_write(0xA000, value);
}

static void sunsoft5b_register(uint8_t reg, uint8_t value) {
    cart_cpu_write(0xC000, reg);
    cart_cpu_write(0xE000, value);
}

static void prepare_mapper_cpu_nops(void) {
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    for (unsigned i = 0; i < 0x200; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
    cpu.pc = 0x0200;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
}

static void run_mapper_nops(unsigned count) {
    for (unsigned i = 0; i < count; ++i) (void)cpu_step(&cpu);
}

static int test_sunsoft69_banks_ram_and_startup(void) {
    CHECK(fixture(69, 0x80000, 0x20000, false) == 69);
    CHECK(cart != NULL && cart->clock != NULL);
    CHECK(cart_cpu_read(0x6000) == 0);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
    CHECK(cart_cpu_read_bus(0xA000, 0x56) == 0x56);
    CHECK(cart_cpu_read_bus(0xC000, 0x56) == 0x56);
    CHECK(cart_cpu_read(0xE000) == 63);
    CHECK(cart_ppu_read(0x0123) == 0x23); // CHR-ROM starts unmapped.

    for (unsigned slot = 0; slot < 8; ++slot) {
        sunsoft69_command((uint8_t)slot, (uint8_t)(24 + slot));
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == (uint8_t)(24 + slot));
    }
    sunsoft69_command(9, 5);
    sunsoft69_command(10, 17);
    sunsoft69_command(11, 31);
    CHECK(cart_cpu_read(0x8000) == 5);
    CHECK(cart_cpu_read(0xA000) == 17);
    CHECK(cart_cpu_read(0xC000) == 31);
    CHECK(cart_cpu_read(0xE000) == 63);

    sunsoft69_command(12, 0);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    sunsoft69_command(12, 1);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    sunsoft69_command(12, 2);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    sunsoft69_command(12, 3);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);

    iNESHeader h = header_for(69, 0x20000, false);
    h.flags7 |= 0x08; // NES 2.0
    h.flags10 = 9;    // 32 KiB volatile PRG-RAM
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 69);
    CHECK(cart_cpu_read(0x6000) == 0); // Command 8 powers up as PRG-ROM bank 0.
    sunsoft69_command(8, 0x40);
    CHECK(cart_cpu_read_bus(0x6123, 0x56) == 0x56);
    cart_cpu_write(0x6123, 0x11);
    sunsoft69_command(8, 0xC3);
    cart_cpu_write(0x6123, 0xA5);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    sunsoft69_command(8, 0xC0);
    CHECK(cart_cpu_read(0x6123) == 0);
    sunsoft69_command(8, 0x43);
    CHECK(cart_cpu_read_bus(0x6123, 0x56) == 0x56);
    cart_cpu_write(0x6123, 0x33);
    sunsoft69_command(8, 0xC3);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    sunsoft69_command(8, 3);
    CHECK(cart_cpu_read(0x6123) == 3);

    CHECK(fixture(69, 0x20000, 0x2000, true) == 69);
    CHECK(cart_ppu_read(0x0423) == 0); // Allocated CHR RAM starts cleared.
    cart_ppu_write(0x0423, 0x71);
    sunsoft69_command(0, 1);
    CHECK(cart_ppu_read(0x0023) == 0x71);
    sunsoft69_command(0, 7);
    cart_ppu_write(0x0123, 0xA6);
    CHECK(cart_ppu_read(0x0123) == 0xA6);
    sunsoft69_command(0, 0);
    CHECK(cart_ppu_read(0x0123) == 0);
    sunsoft69_command(0, 7);
    CHECK(cart_ppu_read(0x0123) == 0xA6);
    return 0;
}

static int test_sunsoft69_legacy_ram_defaults(void) {
    for (unsigned battery = 0; battery < 2; ++battery) {
        iNESHeader h = header_for(69, 0x20000, false);
        h.flags6 |= (uint8_t)(battery << 1);
        h.prg_ram_size = 0;
        RomRamSizes sizes;
        CHECK(rom_ram_sizes(&h, &sizes) == 0);
        CHECK(sizes.prg_ram == (battery ? 0u : 0x8000u));
        CHECK(sizes.prg_nvram == (battery ? 0x8000u : 0u));
        CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 69);
        for (unsigned bank = 0; bank < 4; ++bank) {
            sunsoft69_command(8, (uint8_t)(0xC0u | bank));
            cart_cpu_write(0x6123, (uint8_t)(0xA0u + bank));
        }
        for (unsigned bank = 0; bank < 4; ++bank) {
            sunsoft69_command(8, (uint8_t)(0xC0u | bank));
            CHECK(cart_cpu_read(0x6123) == (uint8_t)(0xA0u + bank));
        }
        h.prg_ram_size = 1;
        CHECK(rom_ram_sizes(&h, &sizes) == 0);
        CHECK(sizes.prg_ram + sizes.prg_nvram == 0x8000);
    }
    return 0;
}

static int test_sunsoft69_irq_cpu_clock(void) {
    CHECK(fixture(69, 0x20000, 0x2000, false) == 69);
    prepare_mapper_cpu_nops();

    sunsoft69_command(14, 1);
    sunsoft69_command(15, 0);
    sunsoft69_command(13, 0x81);
    CHECK(!cart_irq_pending());
    CHECK(cpu_step(&cpu) == 2);
    CHECK(cart_irq_pending()); // 0001 -> 0000 -> FFFF on the two NOP cycles.

    sunsoft69_command(13, 0x80);
    CHECK(!cart_irq_pending());
    sunsoft69_command(14, 0);
    sunsoft69_command(15, 0);
    CHECK(cpu_step(&cpu) == 2);
    CHECK(!cart_irq_pending()); // Counter runs, IRQ output is disabled.
    sunsoft69_command(13, 0x01);
    CHECK(!cart_irq_pending()); // Enabling output later does not replay the old edge.

    sunsoft69_command(14, 0);
    sunsoft69_command(15, 0);
    sunsoft69_command(13, 0x81);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
    sunsoft69_command(13, 0x00);
    CHECK(!cart_irq_pending());
    run_mapper_nops(4);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_sunsoft5b_tone_noise_envelope(void) {
    CHECK(fixture(69, 0x20000, 0x2000, false) == 69);
    sunsoft5b_register(0, 1);
    sunsoft5b_register(1, 0);
    sunsoft5b_register(7, 0x3E); // A tone enabled, all noise and B/C tone disabled.
    sunsoft5b_register(8, 15);
    prepare_mapper_cpu_nops();
    // CPU power-on contributed seven mapper clocks. Eight more leave the
    // shared divide-by-16 phase one clock short of its first PSG tick.
    run_mapper_nops(4);
    CHECK(cart_expansion_audio() == 0.0f);
    run_mapper_nops(1); // Cross clock 16 and toggle tone A high.
    float full_level = cart_expansion_audio();
    CHECK(full_level < -0.125f && full_level > -0.127f);
    cart_cpu_write(0xC000, 0x18); // Nonzero selector high nibble blocks data writes.
    cart_cpu_write(0xE000, 0);
    CHECK(cart_expansion_audio() == full_level);
    run_mapper_nops(7);
    CHECK(cart_expansion_audio() == full_level);
    run_mapper_nops(1); // Cross clock 32 and toggle tone A low.
    CHECK(cart_expansion_audio() == 0.0f);

    CHECK(fixture(69, 0x20000, 0x2000, false) == 69);
    sunsoft5b_register(6, 1);
    sunsoft5b_register(7, 0x37); // Constant tone gate, A noise enabled; B/C muted.
    sunsoft5b_register(8, 15);
    prepare_mapper_cpu_nops();
    float noise_high = cart_expansion_audio();
    CHECK(noise_high < -0.125f && noise_high > -0.127f);
    run_mapper_nops(12); // Global clock 31: only one half of the noise period elapsed.
    CHECK(cart_expansion_audio() == noise_high);
    run_mapper_nops(1); // Cross global clock 32 and advance the 17-bit LFSR once.
    CHECK(cart_expansion_audio() == 0.0f);

    CHECK(fixture(69, 0x20000, 0x2000, false) == 69);
    sunsoft5b_register(7, 0x3F); // Disabled generators are high mixer inputs.
    sunsoft5b_register(8, 0x10); // Channel A uses the shared envelope.
    sunsoft5b_register(0x0B, 1);
    sunsoft5b_register(0x0C, 0);
    sunsoft5b_register(0x0D, 0x0C); // Repeating rising saw envelope.
    prepare_mapper_cpu_nops();
    CHECK(cart_expansion_audio() == 0.0f);
    run_mapper_nops(4);
    CHECK(cart_expansion_audio() == 0.0f);
    run_mapper_nops(1); // Cross clock 16: level 1 still aliases silence.
    CHECK(cart_expansion_audio() == 0.0f);
    run_mapper_nops(8); // Level 2 is the first nonzero 1.5 dB step.
    float first_envelope_level = cart_expansion_audio();
    CHECK(first_envelope_level < -0.0008f && first_envelope_level > -0.0009f);
    run_mapper_nops(29 * 8); // Reach level 31 without reading private PSG state.
    CHECK(cart_expansion_audio() < -0.125f && cart_expansion_audio() > -0.127f);
    run_mapper_nops(8); // The next envelope period restarts the saw at zero.
    CHECK(cart_expansion_audio() == 0.0f);
    return 0;
}

static int test_sunsoft69_persistence_and_loader(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    iNESHeader h = header_for(69, 0x20000, false);
    h.flags7 |= 0x08;
    h.flags6 |= 0x02;
    h.flags10 = 0x90; // 32 KiB PRG-NVRAM
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 69);
    cart_battery_configure(paths.rom, true);
    sunsoft69_command(8, 0xC2);
    cart_cpu_write(0x6123, 0xA5);
    sunsoft69_command(8, 0xC3);
    cart_cpu_write(0x6123, 0x5A);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x8000);
    CHECK(saved_byte(paths.prg_save, 0x4000 + 0x123) == 0xA5);
    CHECK(saved_byte(paths.prg_save, 0x6000 + 0x123) == 0x5A);

    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 69);
    cart_battery_configure(paths.rom, true);
    sunsoft69_command(8, 0xC2);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    sunsoft69_command(8, 0xC3);
    CHECK(cart_cpu_read(0x6123) == 0x5A);

    // NES 2.0 may describe independent work and save chips. FME-7 selects
    // the save chip when the battery flag is present; the work chip remains
    // allocated but unmapped. A trainer initializes work RAM before the save
    // file is loaded, so it must not overwrite the selected save chip.
    cart_battery_shutdown();
    CHECK(remove(paths.prg_save) == 0);
    uint8_t seeded[0x2000];
    memset(seeded, 0xA6, sizeof(seeded));
    FILE *fp = fopen(paths.prg_save, "wb");
    CHECK(fp != NULL);
    CHECK(fwrite(seeded, 1, sizeof(seeded), fp) == sizeof(seeded));
    CHECK(fclose(fp) == 0);

    h = header_for(69, 0x20000, false);
    h.flags7 |= 0x08;
    h.flags6 |= 0x06; // Battery plus trainer.
    h.flags10 = 0x77; // Independent 8 KiB work RAM and 8 KiB save RAM.
    size_t image_size;
    uint8_t *image = image_for(&h, 0x20000, 0x2000, &image_size);
    CHECK(image != NULL);
    fp = fopen(paths.rom, "wb");
    CHECK(fp != NULL);
    CHECK(fwrite(image, 1, image_size, fp) == image_size);
    CHECK(fclose(fp) == 0);
    free(image);
    CHECK(load_rom(paths.rom) == 0);
    sunsoft69_command(8, 0xC0);
    CHECK(cart_cpu_read(0x7000) == 0xA6 && cart_cpu_read(0x71FF) == 0xA6);
    cart_cpu_write(0x7123, 0x53);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x2000);
    CHECK(saved_byte(paths.prg_save, 0x1123) == 0x53);
    CHECK(unload_rom());
    CHECK(load_rom(paths.rom) == 0);
    sunsoft69_command(8, 0xC0);
    CHECK(cart_cpu_read(0x7123) == 0x53);

    Mapper *previous = cart;
    sunsoft69_command(9, 3);
    CHECK(cart_cpu_read(0x8000) == 0x5C);
    image = image_for(&h, 0x20000, 0x2000, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size - 1) == -1);
    free(image);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 0x5C);
    h.flags10 = 0xE0; // 1 MiB NVRAM; the six-bit selector reaches its first 512 KiB.
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x20000, fixture_chr, 0x2000) == 69);
    sunsoft69_command(8, 0xFF);
    cart_cpu_write(0x6123, 0xD2);
    CHECK(cart_cpu_read(0x6123) == 0xD2);
    sunsoft69_command(8, 0xC0);
    CHECK(cart_cpu_read(0x6123) == 0);
    sunsoft69_command(8, 0xFF);
    CHECK(cart_cpu_read(0x6123) == 0xD2);
    return save_fixture_end(&paths);
}

static iNESHeader namco210_header(unsigned submapper, bool ram) {
    iNESHeader h = header_for(210, 0x20000, false);
    h.flags7 |= 0x08;
    h.prg_ram_size = (uint8_t)(submapper << 4);
    h.flags10 = ram ? 7 : 0;
    return h;
}

static void namco163_ram_write(uint8_t address, uint8_t value) {
    cart_cpu_write(0xF800, address);
    cart_cpu_write(0x4800, value);
}

static int test_namco163_banks_ram_and_nametables(void) {
    CHECK(fixture(19, 0x40000, 0x20000, false) == 19);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
    CHECK(cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_cpu_read_bus(0xC000, 0x69) == 0x69);
    CHECK(cart_cpu_read(0xE000) == 31);

    cart_cpu_write(0xE000, 3);
    cart_cpu_write(0xE800, 5);
    cart_cpu_write(0xF000, 7);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xA000) == 5);
    CHECK(cart_cpu_read(0xC000) == 7 && cart_cpu_read(0xE000) == 31);

    cart_cpu_write(0x8000, 4);
    cart_cpu_write(0xA800, 9);
    CHECK(cart_ppu_read(0x0000) == 4 && cart_ppu_read(0x1400) == 9);

    memset(ppu_vram, 0, sizeof(ppu_vram));
    ppu_vram[0x400] = 0xA5;
    cart_cpu_write(0x8000, 0xE1);
    CHECK(cart_ppu_read(0x0000) == 0xA5);
    cart_cpu_write(0xE800, 0x45); // Force low pattern banks to CHR even for $E0-$FF values.
    cart_cpu_write(0x8000, 0xE1);
    CHECK(cart_ppu_read(0x0000) == (0xE1u % 0x80u));

    ppu_vram[0] = 0x35;
    cart_cpu_write(0xC000, 0xE0);
    cart_cpu_write(0xC800, 3);
    CHECK(cart_nt_read(0x2000, ppu_vram) == 0x35);
    CHECK(cart_nt_read(0x2400, ppu_vram) == 3);
    cart_nt_write(0x2001, 0x53, ppu_vram);
    CHECK(ppu_vram[1] == 0x53);

    cart_cpu_write(0xF800, 0x40); // Global RAM writes enabled, all four 2 KiB blocks writable.
    cart_cpu_write(0x6123, 0x11);
    cart_cpu_write(0x6923, 0x22);
    cart_cpu_write(0x7123, 0x33);
    cart_cpu_write(0x7923, 0x44);
    CHECK(cart_cpu_read(0x6123) == 0x11 && cart_cpu_read(0x6923) == 0x22);
    CHECK(cart_cpu_read(0x7123) == 0x33 && cart_cpu_read(0x7923) == 0x44);
    cart_cpu_write(0xF800, 0x45); // Protect blocks 0 and 2 while keeping global write enable.
    cart_cpu_write(0x6123, 0xA1);
    cart_cpu_write(0x6923, 0xA2);
    cart_cpu_write(0x7123, 0xA3);
    cart_cpu_write(0x7923, 0xA4);
    CHECK(cart_cpu_read(0x6123) == 0x11 && cart_cpu_read(0x6923) == 0xA2);
    CHECK(cart_cpu_read(0x7123) == 0x33 && cart_cpu_read(0x7923) == 0xA4);
    cart_cpu_write(0xF800, 0x05); // Global write disable preserves readable RAM.
    cart_cpu_write(0x6923, 0x55);
    CHECK(cart_cpu_read(0x6923) == 0xA2);
    return 0;
}

static int test_namco163_irq_and_audio(void) {
    CHECK(fixture(19, 0x20000, 0x20000, false) == 19);
    CHECK(cart != NULL && cart->clock != NULL && cart_expansion_audio() == 0.0f);

    cart_cpu_write(0x5000, 0xFD);
    cart_cpu_write(0x5800, 0xFF);
    cart->clock(1);
    CHECK(!cart_irq_pending() && cart_cpu_read(0x5000) == 0xFE);
    cart->clock(1);
    CHECK(cart_irq_pending() && cart_cpu_read(0x5000) == 0xFF);
    cart_cpu_write(0x5000, 0);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0x5800, 0x7F);
    cart->clock(8);
    CHECK(!cart_irq_pending() && cart_cpu_read(0x5800) == 0x7F);

    fixture_prg[0] = 0xEA;
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cart_cpu_write(0xE000, 0);
    cpu.pc = 0x8000;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
    cart_cpu_write(0x5000, 0xFD);
    cart_cpu_write(0x5800, 0xFF);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
    cart_cpu_write(0x5800, 0);

    cart_cpu_write(0xF800, 0x80);
    cart_cpu_write(0x4800, 0x12);
    cart_cpu_write(0x4800, 0x34);
    cart_cpu_write(0xF800, 0x80);
    CHECK(cart_cpu_read(0x4800) == 0x12);
    CHECK(cart_cpu_read(0x4800) == 0x34);

    cart->reset();
    namco163_ram_write(0x00, 0x00);
    namco163_ram_write(0x78, 0x01);
    namco163_ram_write(0x79, 0x00);
    namco163_ram_write(0x7A, 0x00);
    namco163_ram_write(0x7B, 0x00);
    namco163_ram_write(0x7C, 0x00);
    namco163_ram_write(0x7D, 0x00);
    namco163_ram_write(0x7E, 0x00);
    namco163_ram_write(0x7F, 0x0F);
    cart->clock(14);
    CHECK(cart_expansion_audio() == 0.0f);
    cart->clock(1);
    CHECK(cart_expansion_audio() > 0.47f && cart_expansion_audio() < 0.49f);

    float held = cart_expansion_audio();
    cart_cpu_write(0xE000, 0x40);
    cart->clock(30);
    CHECK(cart_expansion_audio() == held);

    cart->reset();
    namco163_ram_write(0x78, 1);
    namco163_ram_write(0x70, 2);
    namco163_ram_write(0x7F, 0x10); // Channels seven and six share the 15-cycle sequencer.
    cart->clock(14);
    cart_cpu_write(0xF800, 0x79);
    CHECK(cart_cpu_read(0x4800) == 0);
    cart_cpu_write(0xE000, 0x40);
    cart->clock(60);
    cart_cpu_write(0xE000, 0);
    cart->clock(1);
    cart_cpu_write(0xF800, 0x79);
    CHECK(cart_cpu_read(0x4800) == 1);
    cart_cpu_write(0xF800, 0x71);
    CHECK(cart_cpu_read(0x4800) == 0);
    cart->clock(15);
    cart_cpu_write(0xF800, 0x71);
    CHECK(cart_cpu_read(0x4800) == 2);
    cart->clock(15);
    cart_cpu_write(0xF800, 0x79);
    CHECK(cart_cpu_read(0x4800) == 2);
    return 0;
}

static int test_namco175_340_variants(void) {
    iNESHeader h = namco210_header(1, true);
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 210);
    CHECK(cart_cpu_read(0xE000) == 15);
    CHECK(cart_cpu_read_bus(0x4800, 0xA6) == 0xA6);
    CHECK(cart_cpu_read_bus(0x5000, 0x53) == 0x53);
    cart_cpu_write(0x6123, 0x11);
    CHECK(cart_cpu_read(0x6123) == 0);
    cart_cpu_write(0xC000, 1);
    cart_cpu_write(0x6123, 0xA5);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0x6123, 0x5A);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    cart_cpu_write(0x5000, 0xFF);
    cart_cpu_write(0x5800, 0xFF);
    cart->clock(8);
    CHECK(!cart_irq_pending() && cart_expansion_audio() == 0.0f);

    h = namco210_header(2, true);
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 210);
    CHECK(cart_cpu_read_bus(0x6000, 0x69) == 0x69);
    cart_cpu_write(0x6000, 0xA5);
    CHECK(cart_cpu_read_bus(0x6000, 0x69) == 0x69);
    cart_cpu_write(0xE000, 0x03);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0xE000, 0x43);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0xE000, 0x83);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    cart_cpu_write(0xE000, 0xC3);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    memset(ppu_vram, 0, sizeof(ppu_vram));
    ppu_vram[0x400] = 0x96;
    cart_cpu_write(0xC000, 0xE1);
    CHECK(cart_nt_read(0x2000, ppu_vram) == 0x96);
    cart_cpu_write(0xE000, 0x03);
    CHECK(cart_nt_read(0x2000, ppu_vram) == 0);
    CHECK(cart_nt_read(0x2400, ppu_vram) == 0);
    cart_cpu_write(0x5000, 0xFF);
    cart_cpu_write(0x5800, 0xFF);
    cart->clock(8);
    CHECK(!cart_irq_pending() && cart_cpu_read_bus(0x5000, 0x53) == 0x53);
    return 0;
}

static int test_namco163_persistence_and_loader(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    iNESHeader h = header_for(19, 0x20000, false);
    h.flags6 |= 0x02;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 19);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0xF800, 0x40);
    cart_cpu_write(0x6123, 0xA5);
    namco163_ram_write(0, 0x5A);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x2080);
    CHECK(saved_byte(paths.prg_save, 0x123) == 0xA5);
    CHECK(saved_byte(paths.prg_save, 0x2000) == 0x5A);

    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 19);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0xF800, 0x40);
    CHECK(cart_cpu_read(0x6123) == 0xA5);
    cart_cpu_write(0xF800, 0);
    CHECK(cart_cpu_read(0x4800) == 0x5A);

    // Phase bytes are writable audio RAM, including writes performed by the chip.
    namco163_ram_write(0x78, 1);
    cart_battery_flush();
    CHECK(saved_byte(paths.prg_save, 0x2079) == 0);
    cart->clock(15);
    cart_battery_flush();
    CHECK(saved_byte(paths.prg_save, 0x2079) == 1);
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 19);
    cart_battery_configure(paths.rom, true);
    cart->clock(15); // No CPU write after loading the saved audio state.
    cart_battery_flush();
    CHECK(saved_byte(paths.prg_save, 0x2079) == 2);

    cart_cpu_write(0xE000, 3);
    Mapper *previous = cart;
    iNESHeader invalid = namco210_header(3, true);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x20000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);
    return save_fixture_end(&paths);
}

static int test_namco163_source_page_geometry(void) {
    iNESHeader h = header_for(19, 0x20000, false);
    h.flags7 |= 0x08;
    h.chr_rom_chunks = 0x24; // NES 2.0 exponent encoding: 512-byte CHR-ROM.
    h.flags9 = (uint8_t)((h.flags9 & 0x0Fu) | 0xF0u);

    size_t size;
    uint8_t *image = image_for(&h, 0x20000, 0x0200, &size);
    CHECK(image != NULL);
    uint8_t *prg = image + sizeof(h);
    uint8_t *chr = prg + 0x20000;
    for (size_t page = 0; page < 0x20000 / 0x2000; ++page)
        memset(prg + page * 0x2000, (int)page, 0x2000);
    memset(chr, 0xC3, 0x0200);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(chr_size == 0x0200);
    ppu_power_on(&ppu);
    CHECK(cpu_power_on(&cpu));

    cart_cpu_write(0xE000, 3);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xE000) == 15);

    // A 512-byte source shrinks each logical CHR page and its destination
    // spacing. Unselected chunks remain open bus.
    CHECK(cart_ppu_read(0x0012) == 0x12);
    cart_cpu_write(0x8000, 0x35);
    CHECK(cart_ppu_read(0x0012) == 0xC3);
    CHECK(cart_ppu_read(0x0212) == 0x12);
    cart_cpu_write(0x8800, 0x7A);
    CHECK(cart_ppu_read(0x0212) == 0xC3);
    cart_cpu_write(0xB800, 0x53);
    CHECK(cart_ppu_read(0x0E12) == 0xC3);
    CHECK(cart_ppu_read(0x1012) == 0x12);

    // CIRAM keeps its native 1 KiB page size. Replacing the same logical
    // slot with shrunken CHR only overwrites the 512-byte destination range.
    memset(ppu_vram, 0, sizeof(ppu_vram));
    ppu_vram[0x0412] = 0xA6;
    cart_cpu_write(0xA000, 0xE1);
    CHECK(cart_ppu_read(0x1012) == 0xA6);
    cart_ppu_write(0x1013, 0x69);
    CHECK(ppu_vram[0x0413] == 0x69);
    cart_cpu_write(0xE800, 0x80);
    cart_cpu_write(0xA000, 0xE1);
    CHECK(cart_ppu_read(0x0812) == 0xC3);
    CHECK(cart_ppu_read(0x1012) == 0xA6);

    // Nametable CHR registers use the CHR source page size too. A CIRAM
    // selection for the same logical slot maps the normal nametable range.
    ppu_vram[0x0012] = 0x35;
    cart_cpu_write(0xC000, 0x00);
    CHECK(cart_ppu_read(0x1012) == 0xC3);
    CHECK(cart_nt_read(0x2012, ppu_vram) == 0x35);
    ppu_vram[0x0412] = 0x96;
    cart_cpu_write(0xC000, 0xE1);
    CHECK(cart_nt_read(0x2012, ppu_vram) == 0x96);
    cart_nt_write(0x2013, 0x5A, ppu_vram);
    CHECK(ppu_vram[0x0413] == 0x5A);

    // CPU soft reset retains the board registers and partial CHR/CIRAM maps.
    cpu_soft_reset(&cpu);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xE000) == 15);
    CHECK(cart_ppu_read(0x0012) == 0xC3);
    CHECK(cart_nt_read(0x2012, ppu_vram) == 0x96);
    CHECK(cart_nt_read(0x2013, ppu_vram) == 0x5A);

    // Reinitializing the mapper removes those mappings. CHR ROM returns to
    // open bus until a bank register selects it again.
    cart->reset();
    CHECK(cart_ppu_read(0x0012) == 0x12);
    CHECK(cart_cpu_read(0xE000) == 15);

    // Mapper 210 uses the same shrunken CHR source geometry.
    h = namco210_header(1, false);
    h.chr_rom_chunks = 0x24;
    h.flags9 = (uint8_t)((h.flags9 & 0x0Fu) | 0xF0u);
    image = image_for(&h, 0x20000, 0x0200, &size);
    CHECK(image != NULL);
    chr = image + sizeof(h) + 0x20000;
    memset(chr, 0x87, 0x0200);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    cart_cpu_write(0x8000, 0xE1);
    CHECK(cart_ppu_read(0x0012) == 0x87);
    CHECK(cart_ppu_read(0x0212) == 0x12);

    // A source page whose size is not a multiple of the mapper's 256-byte
    // mapping granularity is accepted, but a bank write cannot map a partial
    // hardware page.
    h = header_for(19, 0x20000, false);
    h.flags7 |= 0x08;
    h.chr_rom_chunks = 0x1D; // NES 2.0 exponent encoding: 384-byte CHR-ROM.
    h.flags9 = (uint8_t)((h.flags9 & 0x0Fu) | 0xF0u);
    image = image_for(&h, 0x20000, 0x0180, &size);
    CHECK(image != NULL);
    memset(image + sizeof(h) + 0x20000, 0xD2, 0x0180);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    cart_cpu_write(0x8000, 0);
    CHECK(cart_ppu_read(0x0012) == 0x12);

    // Truncated replacements remain transactional after the new geometry is
    // accepted.
    h.chr_rom_chunks = 0x24;
    image = image_for(&h, 0x20000, 0x0200, &size);
    CHECK(image != NULL);
    Mapper *previous = cart;
    CHECK(load_rom_memory(image, size - 1) == -1);
    free(image);
    CHECK(cart == previous && cart_ppu_read(0x0012) == 0x12);
    return 0;
}

static iNESHeader mapper34_header(unsigned submapper, bool chr_ram, bool nes2) {
    iNESHeader h = header_for(34, 0x40000, chr_ram);
    if (nes2) {
        h.flags7 |= 0x08;
        h.prg_ram_size = (uint8_t)(submapper << 4);
        h.flags10 = 7;
        if (chr_ram) h.zero[0] = 7;
    }
    return h;
}

static int test_mapper34_bnrom_banking_and_ram(void) {
    iNESHeader h = mapper34_header(2, true, true);
    CHECK(fixture_with_header(&h, 0x40000, 0x2000) == 34);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xE000) == 3);
    CHECK(cart_ppu_read(0x0123) == 0);
    cart_ppu_write(0x0123, 0xA5);
    CHECK(cart_ppu_read(0x0123) == 0xA5);
    cart_cpu_write(0x6123, 0x5A);
    CHECK(cart_cpu_read(0x6123) == 0x5A);

    cart_cpu_write(0xE000, 3); // Bank-0 ROM drives 3 here, so the bus-conflicted write keeps 3.
    CHECK(cart_cpu_read(0x8000) == 12 && cart_cpu_read(0xE000) == 15);
    cart_cpu_write(0x8000, 7); // Current ROM drives 12; 7 & 12 selects bank 4.
    CHECK(cart_cpu_read(0x8000) == 16);
    cart_cpu_write(0xE000, 0xFF); // Current ROM drives 19; wrapped bank value selects bank 3.
    CHECK(cart_cpu_read(0x8000) == 12);

    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 12 && cart_cpu_read(0x6123) == 0x5A);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    return 0;
}

static int test_mapper34_nina_banks_and_selection(void) {
    iNESHeader h = mapper34_header(1, false, true);
    CHECK(fixture_with_header(&h, 0x40000, 0x20000) == 34);
    CHECK(cart_cpu_read(0x8000) == 0);
    CHECK(cart_ppu_read(0x0000) == 0x00 && cart_ppu_read(0x1001) == 0x01);

    cart_cpu_write(0x7FFD, 5);
    cart_cpu_write(0x7FFE, 9);
    cart_cpu_write(0x7FFF, 17);
    CHECK(cart_cpu_read(0x8000) == 20);
    CHECK(cart_ppu_read(0x0000) == 36 && cart_ppu_read(0x1000) == 68);
    CHECK(cart_cpu_read(0x7FFD) == 5 && cart_cpu_read(0x7FFE) == 9 && cart_cpu_read(0x7FFF) == 17);

    cart_cpu_write(0x7FFC, 0xA5);
    CHECK(cart_cpu_read(0x7FFC) == 0xA5 && cart_cpu_read(0x8000) == 20);
    cart_cpu_write(0x8000, 2); // NINA does not decode the BNROM register range.
    CHECK(cart_cpu_read(0x8000) == 20);
    cart_cpu_write(0x7FFE, 0xFF);
    CHECK(cart_ppu_read(0x0000) == 124); // 255 wraps across 32 4 KiB CHR banks.

    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 20);
    CHECK(cart_ppu_read(0x0000) == 124 && cart_ppu_read(0x1001) == 68);
    CHECK(cart_cpu_read(0x7FFC) == 0xA5);

    iNESHeader legacy_nina = header_for(34, 0x40000, false);
    CHECK(fixture_with_header(&legacy_nina, 0x40000, 0x2000) == 34);
    cart_cpu_write(0x7FFD, 2);
    CHECK(cart_cpu_read(0x8000) == 8);

    iNESHeader legacy_bnrom = header_for(34, 0x40000, true);
    CHECK(fixture_with_header(&legacy_bnrom, 0x40000, 0x2000) == 34);
    cart_cpu_write(0xE000, 2);
    CHECK(cart_cpu_read(0x8000) == 8);
    return 0;
}

static int test_mapper34_loader_preserves_cart(void) {
    iNESHeader good = mapper34_header(1, false, true);
    CHECK(fixture_with_header(&good, 0x40000, 0x2000) == 34);
    cart_cpu_write(0x7FFD, 3);
    cart_cpu_write(0x6123, 0xA6);
    Mapper *previous = cart;

    iNESHeader invalid = mapper34_header(3, false, true);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 12 && cart_cpu_read(0x6123) == 0xA6);

    iNESHeader compatible = mapper34_header(2, false, true);
    fixture_chr[0x0123] = 0x35;
    CHECK(mapper_init_from_header(&compatible, fixture_prg, 0x40000, fixture_chr, 0x2000) == 34);
    CHECK(cart_ppu_read(0x0123) == 0x35);
    cart_cpu_write(0xE000, 3);
    CHECK(cart_cpu_read(0x8000) == 12 && cart_ppu_read(0x0123) == 0x35);

    compatible = mapper34_header(1, true, true);
    CHECK(mapper_init_from_header(&compatible, fixture_prg, 0x40000, fixture_chr, 0x2000) == 34);
    cart_ppu_write(0x0123, 0x53);
    CHECK(cart_ppu_read(0x0123) == 0x53);
    cart_cpu_write(0x7FFE, 1);
    cart_ppu_write(0x0123, 0xA6);
    CHECK(cart_ppu_read(0x0123) == 0xA6);
    cart_cpu_write(0x7FFE, 0);
    CHECK(cart_ppu_read(0x0123) == 0x53);

    return 0;
}

static int test_mmc3_mixed_chr_source_page_geometry(void) {
    iNESHeader h = header_for(74, 0x20000, false);
    h.flags7 |= 0x08;
    h.flags10 = 7;  // 8 KiB volatile PRG-RAM.
    h.zero[0] = 5; // 2 KiB volatile CHR-RAM for mapper 74.
    h.chr_rom_chunks = 0x24; // NES 2.0 exponent encoding: 512-byte CHR-ROM.
    h.flags9 = (uint8_t)((h.flags9 & 0x0Fu) | 0xF0u);

    size_t image_size;
    uint8_t *image = tqrom_image(&h, 0x0200, &image_size);
    CHECK(image != NULL);
    uint8_t *chr = image + sizeof(h) + 0x20000;
    memset(chr, 0x5A, 0x0200);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);

    // The ROM page shrinks to 512 bytes, so MMC3 logical slots 0-7 initially
    // cover only $0000-$0FFF. The upper half remains unmapped.
    CHECK(cart_ppu_read(0x0812) == 0x5A);
    CHECK(cart_ppu_read(0x1012) == 0x12);

    // Register 2 is logical slot 4 in CHR mode 0. Selecting mapper 74's RAM
    // bank maps a 1 KiB RAM page at slot*1 KiB = $1000, independently of the
    // existing 512-byte ROM slot at $0800.
    cart_cpu_write(0x8000, 2);
    cart_cpu_write(0x8001, 0x08);
    cart_ppu_write(0x1012, 0xA6);
    CHECK(cart_ppu_read(0x1012) == 0xA6);
    CHECK(cart_ppu_read(0x0812) == 0x5A);
    CHECK(cart_ppu_read(0x1412) == 0x12);

    // Switching that logical slot back to ROM overwrites only the 512-byte
    // destination range. The RAM mapping at $1000 is outside it and remains,
    // matching the source mapper's range-overwrite behavior.
    cart_cpu_write(0x8001, 0x07);
    CHECK(cart_ppu_read(0x0812) == 0x5A);
    CHECK(cart_ppu_read(0x1012) == 0xA6);
    cart_ppu_write(0x0812, 0xFF);
    CHECK(cart_ppu_read(0x0812) == 0x5A);

    // CPU banking remains live while the PPU source map is mixed.
    cart_cpu_write(0x8000, 6);
    cart_cpu_write(0x8001, 3);
    CHECK(cart_cpu_read(0x8000) == 3);

    // Explicit zero CHR RAM is valid. Selecting the RAM bank cannot replace
    // the existing shrunken ROM page because there is no RAM page to map.
    iNESHeader zero_ram = h;
    zero_ram.zero[0] = 0;
    image = tqrom_image(&zero_ram, 0x0200, &image_size);
    CHECK(image != NULL);
    memset(image + sizeof(zero_ram) + 0x20000, 0x3C, 0x0200);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    cart_cpu_write(0x8000, 2);
    cart_cpu_write(0x8001, 0x08);
    CHECK(cart_ppu_read(0x0812) == 0x3C && cart_ppu_read(0x1012) == 0x12);
    return 0;
}

static int test_mapper34_image_loading(void) {
    for (unsigned submapper = 1; submapper <= 2; ++submapper) {
        bool chr_ram = submapper == 2;
        size_t chr_bytes = chr_ram ? 0 : 0x10000;
        iNESHeader h = mapper34_header(submapper, chr_ram, true);
        h.chr_rom_chunks = (uint8_t)(chr_bytes / 0x2000);
        size_t size;
        uint8_t *image = image_for(&h, 0x40000, chr_bytes, &size);
        CHECK(image != NULL);
        for (size_t i = 0; i < 0x40000; ++i)
            image[sizeof(h) + i] = (uint8_t)(i / 0x2000);
        for (size_t i = 0; i < chr_bytes; ++i)
            image[sizeof(h) + 0x40000 + i] = (uint8_t)(i / 0x0400);
        CHECK(load_rom_memory(image, size) == 0);
        CHECK(rom_mapper_number(&ines_header) == 34);
        cart_cpu_write(chr_ram ? 0xE000 : 0x7FFD, 3);
        cart_cpu_write(0x6123, 0xA6);
        CHECK(cart_cpu_read(0x8000) == 12 && cart_cpu_read(0x6123) == 0xA6);
        if (chr_ram) {
            cart_ppu_write(0x0123, 0xE4);
        } else {
            cart_cpu_write(0x7FFE, 5);
            cart_cpu_write(0x7FFF, 7);
            CHECK(cart_ppu_read(0x0123) == 20 && cart_ppu_read(0x1123) == 28);
            cart_ppu_write(0x0123, 0xE4);
            CHECK(cart_ppu_read(0x0123) == 20);
        }
        Mapper *previous = cart;
        iNESHeader active = ines_header;
        image[8] = 0x30; // Unsupported board metadata must leave the selected banks and RAM intact.
        CHECK(load_rom_memory(image, size) == -1);
        memcpy(image, &h, sizeof(h));
        CHECK(load_rom_memory(image, size - 1) == -1);
        free(image);
        CHECK(cart == previous && memcmp(&ines_header, &active, sizeof(active)) == 0);
        CHECK(cart_cpu_read(0x8000) == 12 && cart_cpu_read(0x6123) == 0xA6);
        CHECK(cart_ppu_read(0x0123) == (chr_ram ? 0xE4 : 20));
    }
    return 0;
}

#endif // MAPPER_ACCURACY_PERSISTENCE_H

