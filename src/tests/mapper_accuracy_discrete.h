static int test_simple_mapper_registers(void) {
    CHECK(fixture(2, 0x100000, 0x2000, true) == 2);
    cart_cpu_write(0x8000, 32);
    CHECK(cart_cpu_read(0x8000) == 64 && cart_cpu_read(0xFFFF) == 127);
    CHECK(fixture(7, 0x80000, 0x2000, true) == 7);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    cart_cpu_write(0x8000, 0x18);
    CHECK(cart_cpu_read(0x8000) == 32 && cart_get_mirroring() == MIRROR_SINGLE1);
    CHECK(fixture(11, 0x20000, 0x20000, false) == 11);
    fixture_prg[0] = 0xFF;
    cart_cpu_write(0x8000, 0x52);
    CHECK(cart_cpu_read(0x8000) == 8 && cart_ppu_read(0) == 40);
    CHECK(fixture(13, 0x8000, 0x4000, true) == 13);
    for (unsigned bank = 0; bank < 4; ++bank) {
        cart_cpu_write(0x8000, (uint8_t)bank);
        cart_ppu_write(0x1000, (uint8_t)(0xA0 + bank));
    }
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL && cart_ppu_read(0) == 0xA0);
    for (unsigned bank = 0; bank < 4; ++bank) {
        cart_cpu_write(0x8000, (uint8_t)bank);
        CHECK(cart_ppu_read(0x1000) == 0xA0 + bank);
    }
    return 0;
}

static uint16_t vrc6_test_addr(unsigned mapper, uint16_t logical) {
    if (mapper != 26) return logical;
    return (uint16_t)((logical & 0xFFFCu) | ((logical & 1u) << 1) | ((logical & 2u) >> 1));
}

static void vrc6_test_write(unsigned mapper, uint16_t logical, uint8_t value) {
    cart_cpu_write(vrc6_test_addr(mapper, logical), value);
}

static int test_vrc6_startup_banks_wiring_and_ram(void) {
    static const unsigned mappers[] = {24, 26};
    for (size_t variant = 0; variant < sizeof(mappers) / sizeof(mappers[0]); ++variant) {
        unsigned mapper = mappers[variant];
        CHECK(fixture(mapper, 0x40000, 0x40000, false) == (int)mapper);
        CHECK(cart != NULL && cart->clock != NULL);

        // Only the final $E000-$FFFF PRG window is selected at power-on.
        CHECK(cart_cpu_read_bus(0x8000, 0x58) == 0x58);
        CHECK(cart_cpu_read_bus(0xA000, 0xA5) == 0xA5);
        CHECK(cart_cpu_read_bus(0xC000, 0x3C) == 0x3C);
        CHECK(cart_cpu_read(0xE000) == 31);
        CHECK(cart_ppu_read(0x0400) == 0); // CHR-ROM windows start unselected.
        cart_cpu_write(0x6123, 0xA6);
        CHECK(cart_cpu_read_bus(0x6123, 0x58) == 0xA6);

        vrc6_test_write(mapper, 0x8000, 3);
        vrc6_test_write(mapper, 0xC000, 9);
        CHECK(cart_cpu_read(0x8000) == 6 && cart_cpu_read(0xA000) == 7);
        CHECK(cart_cpu_read(0xC000) == 9 && cart_cpu_read(0xE000) == 31);

        // The VRC6b swaps A0/A1; these logical writes must behave identically.
        vrc6_test_write(mapper, 0xD000, 1);
        CHECK(cart_cpu_read_bus(0x6123, 0x58) == 0x58);
        cart_cpu_write(0x6123, 0xFF);
        vrc6_test_write(mapper, 0xD001, 3);
        vrc6_test_write(mapper, 0xD002, 5);
        vrc6_test_write(mapper, 0xD003, 7);
        vrc6_test_write(mapper, 0xE000, 9);
        vrc6_test_write(mapper, 0xE001, 11);
        vrc6_test_write(mapper, 0xE002, 13);
        vrc6_test_write(mapper, 0xE003, 15);
        vrc6_test_write(mapper, 0xB003, 0x00);
        static const uint8_t chr_pages[8] = {1, 3, 5, 7, 9, 11, 13, 15};
        for (unsigned slot = 0; slot < 8; ++slot)
            CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == chr_pages[slot]);

        // Mode 1 duplicates 1 KiB pages unless bit 5 enables adjacent pages.
        vrc6_test_write(mapper, 0xD000, 4);
        vrc6_test_write(mapper, 0xB003, 0x01);
        CHECK(cart_ppu_read(0x0000) == 4 && cart_ppu_read(0x0400) == 4);
        vrc6_test_write(mapper, 0xB003, 0x21);
        CHECK(cart_ppu_read(0x0000) == 4 && cart_ppu_read(0x0400) == 5);

        // Mode 2 keeps four 1 KiB pages and pairs the upper half.
        vrc6_test_write(mapper, 0xE000, 8);
        vrc6_test_write(mapper, 0xB003, 0x22);
        CHECK(cart_ppu_read(0x0000) == 4);
        CHECK(cart_ppu_read(0x1000) == 8 && cart_ppu_read(0x1400) == 9);

        // A PPU banking write applies B003.7 to the initial RAM mapping.
        CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
        cart_cpu_write(0x6000, 0xA5);
        CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
        vrc6_test_write(mapper, 0xB003, 0x80);
        CHECK(cart_cpu_read(0x6000) == 0);
        CHECK(cart_cpu_read(0x6123) == 0xA6);
        cart_cpu_write(0x6123, 0xA5);
        CHECK(cart_cpu_read(0x6123) == 0xA5);
        vrc6_test_write(mapper, 0xB003, 0x00);
        CHECK(cart_cpu_read_bus(0x6123, 0x56) == 0x56);
        vrc6_test_write(mapper, 0xB003, 0x80);
        CHECK(cart_cpu_read(0x6123) == 0xA5);

        // Audio registers use the same A0/A1 wiring variant.
        vrc6_test_write(mapper, 0x9000, 0x8F);
        vrc6_test_write(mapper, 0x9002, 0x80);
        float pulse = cart_expansion_audio();
        CHECK(pulse < -0.224f && pulse > -0.226f);
    }

    // CHR-RAM is linear at startup, then follows programmed CHR banks.
    for (size_t variant = 0; variant < sizeof(mappers) / sizeof(mappers[0]); ++variant) {
        unsigned mapper = mappers[variant];
        CHECK(fixture(mapper, 0x40000, 0x2000, true) == (int)mapper);
        cart_ppu_write(0x0400, 0xA6);
        CHECK(cart_ppu_read(0x0400) == 0xA6);
        vrc6_test_write(mapper, 0xD000, 3);
        cart_ppu_write(0x0000, 0x53);
        vrc6_test_write(mapper, 0xD000, 4);
        cart_ppu_write(0x0000, 0x64);
        vrc6_test_write(mapper, 0xD000, 3);
        CHECK(cart_ppu_read(0x0000) == 0x53);
        vrc6_test_write(mapper, 0xD000, 4);
        CHECK(cart_ppu_read(0x0000) == 0x64);
    }
    return 0;
}

static int test_vrc6_nametable_modes(void) {
    static const unsigned mappers[] = {24, 26};
    for (size_t variant = 0; variant < sizeof(mappers) / sizeof(mappers[0]); ++variant) {
        unsigned mapper = mappers[variant];
        CHECK(fixture(mapper, 0x40000, 0x40000, false) == (int)mapper);
        uint8_t nt[0x1000] = {0};
        nt[0x000] = 0x11;
        nt[0x400] = 0x22;

        // Register-controlled CIRAM routing.
        vrc6_test_write(mapper, 0xE000, 0);
        vrc6_test_write(mapper, 0xE001, 1);
        vrc6_test_write(mapper, 0xE002, 1);
        vrc6_test_write(mapper, 0xE003, 0);
        vrc6_test_write(mapper, 0xB003, 0x01);
        CHECK(cart_nt_read(0x2000, nt) == 0x11 && cart_nt_read(0x2400, nt) == 0x22);
        CHECK(cart_nt_read(0x2800, nt) == 0x22 && cart_nt_read(0x2C00, nt) == 0x11);
        cart_nt_write(0x2C10, 0x66, nt);
        CHECK(nt[0x010] == 0x66 && cart_nt_read(0x2010, nt) == 0x66);

        vrc6_test_write(mapper, 0xB003, 0x20);
        CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
        CHECK(cart_nt_read(0x2000, nt) == 0x11 && cart_nt_read(0x2400, nt) == 0x22);
        CHECK(cart_nt_read(0x2800, nt) == 0x11 && cart_nt_read(0x2C00, nt) == 0x22);
        vrc6_test_write(mapper, 0xB003, 0x23);
        CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
        CHECK(cart_nt_read(0x2000, nt) == 0x11 && cart_nt_read(0x2400, nt) == 0x11);
        CHECK(cart_nt_read(0x2800, nt) == 0x22 && cart_nt_read(0x2C00, nt) == 0x22);
        vrc6_test_write(mapper, 0xB003, 0x28);
        CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
        vrc6_test_write(mapper, 0xB003, 0x2B);
        CHECK(cart_get_mirroring() == MIRROR_SINGLE1);

        // B003.4 maps nametable reads through CHR-ROM; writes stay read-only.
        vrc6_test_write(mapper, 0xE000, 20);
        vrc6_test_write(mapper, 0xE001, 21);
        vrc6_test_write(mapper, 0xE002, 22);
        vrc6_test_write(mapper, 0xE003, 23);
        vrc6_test_write(mapper, 0xB003, 0x11);
        CHECK(cart_nt_read(0x2000, nt) == 20 && cart_nt_read(0x2400, nt) == 21);
        CHECK(cart_nt_read(0x2800, nt) == 22 && cart_nt_read(0x2C00, nt) == 23);
        cart_nt_write(0x2000, 0xEE, nt);
        CHECK(cart_nt_read(0x2000, nt) == 20);
    }
    return 0;
}

static int test_vrc6_irq_timing(void) {
    static const unsigned mappers[] = {24, 26};
    for (size_t variant = 0; variant < sizeof(mappers) / sizeof(mappers[0]); ++variant) {
        unsigned mapper = mappers[variant];
        CHECK(fixture(mapper, 0x40000, 0x40000, false) == (int)mapper);

        vrc6_test_write(mapper, 0xF000, 0xFF);
        vrc6_test_write(mapper, 0xF001, 0x02);
        cart->clock(113);
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());

        vrc6_test_write(mapper, 0xF000, 0xFE);
        vrc6_test_write(mapper, 0xF001, 0x07);
        cart->clock(1);
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
        vrc6_test_write(mapper, 0xF002, 0);
        CHECK(!cart_irq_pending());
        cart->clock(2);
        CHECK(cart_irq_pending());

        vrc6_test_write(mapper, 0xF001, 0x06);
        cart->clock(2);
        CHECK(cart_irq_pending());
        vrc6_test_write(mapper, 0xF002, 0);
        CHECK(!cart_irq_pending());
        cart->clock(4);
        CHECK(!cart_irq_pending());
    }
    return 0;
}

static int test_vrc6_pulse_and_saw_audio(void) {
    CHECK(fixture(24, 0x40000, 0x40000, false) == 24);

    vrc6_test_write(24, 0x9000, 0x1F);
    vrc6_test_write(24, 0x9001, 15);
    vrc6_test_write(24, 0x9002, 0x80);
    float pulse = cart_expansion_audio();
    CHECK(pulse < -0.224f && pulse > -0.226f);
    cart->clock(16);
    CHECK(cart_expansion_audio() == pulse);
    cart->clock(1);
    CHECK(cart_expansion_audio() == 0.0f);

    // Frequency shift and halt affect timing but halt preserves the current level.
    CHECK(fixture(24, 0x40000, 0x40000, false) == 24);
    vrc6_test_write(24, 0x9003, 0x02);
    vrc6_test_write(24, 0x9000, 0x1F);
    vrc6_test_write(24, 0x9001, 15);
    vrc6_test_write(24, 0x9002, 0x80);
    cart->clock(1);
    pulse = cart_expansion_audio();
    CHECK(pulse < 0.0f);
    vrc6_test_write(24, 0x9003, 0x03);
    cart->clock(100);
    CHECK(cart_expansion_audio() == pulse);
    vrc6_test_write(24, 0x9003, 0x02);
    cart->clock(1);
    CHECK(cart_expansion_audio() == 0.0f);

    CHECK(fixture(24, 0x40000, 0x40000, false) == 24);
    vrc6_test_write(24, 0x9003, 0x04);
    vrc6_test_write(24, 0x9000, 0x1F);
    vrc6_test_write(24, 0x9001, 0xFF);
    vrc6_test_write(24, 0x9002, 0x80);
    cart->clock(1);
    CHECK(cart_expansion_audio() < 0.0f);
    cart->clock(1);
    CHECK(cart_expansion_audio() == 0.0f);

    // Pulse 2 is independent and disabling a pulse resets its duty phase.
    vrc6_test_write(24, 0x9002, 0x00);
    vrc6_test_write(24, 0xA000, 0x85);
    vrc6_test_write(24, 0xA002, 0x80);
    float pulse2 = cart_expansion_audio();
    CHECK(pulse2 < -0.074f && pulse2 > -0.076f);

    // Saw adds its six-bit rate every second divider step and resets after 14 steps.
    CHECK(fixture(24, 0x40000, 0x40000, false) == 24);
    vrc6_test_write(24, 0xB000, 8);
    vrc6_test_write(24, 0xB001, 0);
    vrc6_test_write(24, 0xB002, 0x80);
    CHECK(cart_expansion_audio() == 0.0f);
    cart->clock(1);
    CHECK(cart_expansion_audio() == 0.0f);
    cart->clock(1);
    float saw = cart_expansion_audio();
    CHECK(saw < -0.014f && saw > -0.016f);
    cart->clock(12);
    CHECK(cart_expansion_audio() == 0.0f);
    vrc6_test_write(24, 0xB002, 0x00);
    CHECK(cart_expansion_audio() == 0.0f);
    vrc6_test_write(24, 0xB002, 0x80);
    cart->clock(2);
    CHECK(cart_expansion_audio() == saw);

    CHECK(fixture(24, 0x40000, 0x40000, false) == 24);
    vrc6_test_write(24, 0x9000, 0x1F);
    vrc6_test_write(24, 0x9001, 15);
    vrc6_test_write(24, 0x9002, 0x80);
    prepare_mapper_cpu_nops();
    apu_audio_init_state(&apu, 44100);
    run_mapper_nops(128);
    float reconstructed[4];
    apu_audio_pull(&apu, reconstructed, 4);
    float reconstructed_energy = 0.0f;
    for (unsigned i = 0; i < 4; ++i)
        reconstructed_energy += reconstructed[i] < 0.0f ? -reconstructed[i] : reconstructed[i];
    CHECK(reconstructed_energy > 0.00001f);
    CHECK(apu.audio_transition_count > 0);
    return 0;
}

static int test_vrc6_loader_rejection_preserves_cart(void) {
    CHECK(fixture(24, 0x40000, 0x40000, false) == 24);
    vrc6_test_write(24, 0x8000, 3);
    vrc6_test_write(24, 0xB003, 0x80);
    cart_cpu_write(0x6123, 0xA7);
    Mapper *previous = cart;

    iNESHeader invalid = header_for(24, 0x40000, false);
    invalid.flags7 |= 0x08;
    invalid.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 6 && cart_cpu_read(0x6123) == 0xA7);
    return 0;
}

static int test_vrc6_cpu_boot_and_odd_prg_size(void) {
    const unsigned mappers[] = {24, 26};
    const uint16_t registers[] = {0x8000, 0x9000, 0x9002, 0xF000, 0xF001};
    const uint8_t values[] = {1, 0x8F, 0x80, 0xFE, 0x06};
    for (unsigned variant = 0; variant < 2; ++variant) {
        for (unsigned region = 0; region < 3; ++region) {
            unsigned mapper = mappers[variant];
            iNESHeader h = header_for(mapper, 0x6000, true);
            h.flags7 |= 0x08;
            h.prg_rom_chunks = (13u << 2) | 1u; // NES 2.0: 2^13 * 3 bytes, three 8 KiB pages.
            h.flags9 = 0x0F;
            h.flags10 = 7;
            h.zero[0] = 7;
            h.zero[1] = region == 2 ? 3 : (uint8_t)region;
            size_t size;
            uint8_t *image = image_for(&h, 0x6000, 0, &size);
            CHECK(image != NULL);
            for (size_t i = 0; i < 0x6000; ++i)
                image[sizeof(h) + i] = (uint8_t)(i / 0x2000);
            size_t cursor = sizeof(h) + 0x4000;
            for (unsigned reg = 0; reg < sizeof(values); ++reg) {
                uint16_t address = vrc6_test_addr(mapper, registers[reg]);
                image[cursor++] = 0xA9;
                image[cursor++] = values[reg];
                image[cursor++] = 0x8D;
                image[cursor++] = (uint8_t)address;
                image[cursor++] = (uint8_t)(address >> 8);
            }
            memset(image + cursor, 0xEA, 16);
            image[sizeof(h) + 0x5FFC] = 0x00;
            image[sizeof(h) + 0x5FFD] = 0xE0;
            image[sizeof(h) + 0x5FFE] = 0x00;
            image[sizeof(h) + 0x5FFF] = 0x03;
            CHECK(load_rom_memory(image, size) == 0);
            ppu_power_on(&ppu);
            apu_power_on(&apu);
            cpu_power_on(&cpu);
            CHECK(cpu.pc == 0xE000);
            for (unsigned instruction = 0; instruction < 10; ++instruction) cpu_step(&cpu);
            CHECK(cart_cpu_read(0x8100) == 2 && cart_cpu_read(0xA100) == 0);
            CHECK(cart_expansion_audio() < -0.224f && cart_expansion_audio() > -0.226f);
            cpu_step(&cpu);
            CHECK(cart_irq_pending());
            cpu.status &= (uint8_t)~INTERRUPT_FLAG;
            for (unsigned step = 0; step < 4 && cpu.pc != 0x0300; ++step) cpu_step(&cpu);
            CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG));

            Mapper *previous = cart;
            iNESHeader active = ines_header;
            image[8] = 0x10;
            CHECK(load_rom_memory(image, size) == -1);
            memcpy(image, &h, sizeof(h));
            CHECK(load_rom_memory(image, size - 1) == -1);
            free(image);
            CHECK(cart == previous && memcmp(&ines_header, &active, sizeof(active)) == 0);
            CHECK(cart_cpu_read(0x8100) == 2 && cart_irq_pending());
            vrc6_test_write(mapper, 0xF001, 0);
            vrc6_test_write(mapper, 0x8000, 0xFF);
            CHECK(cart_cpu_read(0x8100) == 0 && cart_cpu_read(0xA100) == 1);
        }
    }
    nes_set_region(NES_REGION_NTSC);
    return 0;
}

