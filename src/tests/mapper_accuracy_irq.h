static int test_irem_ram_and_irq_boundaries(void) {
    const unsigned mappers[] = {32, 65};
    for (unsigned i = 0; i < 2; ++i) {
        CHECK(fixture(mappers[i], 0x20000, 0x2000, true) == (int)mappers[i]);
        cart_ppu_write(0x0123, 0xA6);
        cart_ppu_write(0x0523, 0x69);
        CHECK(cart_ppu_read(0x0123) == 0xA6 && cart_ppu_read(0x0523) == 0x69);
        cart_cpu_write(0xB001, 0);
        CHECK(cart_ppu_read(0x0523) == 0xA6);
        cart_ppu_write(0x0523, 0x35);
        CHECK(cart_ppu_read(0x0123) == 0x35);
    }
    cart_cpu_write(0x9003, 0x80);
    cart->clock(65535);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0x9004, 0);
    cart->clock(65536);
    CHECK(!cart_irq_pending()); // Reload acknowledges but does not re-enable an expired counter.
    cart_cpu_write(0x9003, 0x80);
    cart->clock(65536);
    CHECK(cart_irq_pending());
    cart_cpu_write(0x9006, 3);
    cart_cpu_write(0x9004, 0);
    cart_cpu_write(0x9003, 0x80);
    cart->clock(1);
    cart_cpu_write(0x9003, 0);
    cart->clock(100);
    cart_cpu_write(0x9003, 0x80);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0x9004, 0);
    cart_cpu_write(0x9003, 0x80);
    cart->clock(1);
    cart_cpu_write(0x9005, 0xFF);
    cart_cpu_write(0x9006, 0xFF);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending()); // Reload-latch writes leave the running counter unchanged.
    return 0;
}

static int test_cartridge_bus_reads(void) {
    mapper_shutdown();
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
    CHECK(fixture(0, 0x4000, 0x2000, true) == 0);
    fixture_prg[0] = 0xFF;
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0xFF);
    cart_cpu_write(0x6000, 0xFF);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0xFF);
    CHECK(cart_cpu_read_bus(0x4020, 0x56) == 0x56);
    CHECK(cart_cpu_read_bus(0x5FFF, 0x56) == 0x56);
    CHECK(cart_cpu_read(0x5FFF) == 0xFF); // Bus input from the preceding call does not leak.
    iNESHeader h = header_for(0, 0x4000, true);
    h.flags7 = 8;
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x4000, 0x2000) == 0);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    CHECK(fixture(1, 0x4000, 0x2000, true) == 1);
    cart_cpu_write(0x6000, 0xFF);
    serial_write(0xE000, 0x10);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    serial_write(0xE000, 0);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0xFF);
    CHECK(fixture(4, 0x4000, 0x2000, true) == 4);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    cart_cpu_write(0xA001, 0x80);
    cart_cpu_write(0x6000, 0xFF);
    cart_cpu_write(0xA001, 0xC0);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0xFF);
    cart_cpu_write(0xA001, 0);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);

    h = header_for(5, 0x20000, true);
    h.flags7 = 8;
    h.flags10 = 7;
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 5);
    CHECK(cart_cpu_read_bus(0x5203, 0x56) == 0x56);
    cart_cpu_write(0x5104, 2);
    cart_cpu_write(0x5C00, 0xFF);
    CHECK(cart_cpu_read_bus(0x5C00, 0x56) == 0xFF);
    cart_cpu_write(0x5104, 0);
    CHECK(cart_cpu_read_bus(0x5C00, 0x56) == 0x56);
    cart_cpu_write(0x5205, 0xFF);
    cart_cpu_write(0x5206, 1);
    CHECK(cart_cpu_read_bus(0x5205, 0x56) == 0xFF);
    CHECK(cart_cpu_read_bus(0x5206, 0x56) == 0);
    CHECK(cart_cpu_read_bus(0x5204, 0xFF) == 0x3F);
    cart_cpu_write(0x5203, 1);
    cart_cpu_write(0x5204, 0x80);
    uint8_t mmc5_nt[0x1000] = {0};
    mmc5_enter_frame(mmc5_nt);
    mmc5_next_scanline(mmc5_nt);
    CHECK(cart_irq_pending());
    CHECK(cart_cpu_read_bus(0x5204, 0x96) == 0xD6);
    CHECK(!cart_irq_pending());
    CHECK(cart_cpu_read_bus(0x5204, 0x96) == 0x56);
    cart_cpu_write(0x5113, 4);
    cart_cpu_write(0x5114, 4);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
    cart_cpu_write(0x5114, 0x80);
    fixture_prg[0] = 0xFF;
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0xFF);
    return 0;
}

static int mmc6_persistence_cases(const SaveFixture *paths) {
    iNESHeader h = mmc6_header(true);
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_battery_configure(paths->rom, true);
    cart_cpu_write(0x8000, 0x20);
    cart_cpu_write(0xA001, 0xF0);
    cart_cpu_write(0x7000, 0x35);
    cart_cpu_write(0x7200, 0x53);
    cart_cpu_write(0x7FFF, 0xFF);
    cart_battery_flush();
    CHECK(saved_file_size(paths->prg_save) == 0x400);
    CHECK(saved_byte(paths->prg_save, 0) == 0x35 && saved_byte(paths->prg_save, 0x200) == 0x53);
    CHECK(saved_byte(paths->prg_save, 0x3FF) == 0xFF);
    CHECK(saved_file_size(paths->chr_save) == -1);
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_battery_configure(paths->rom, true);
    cart_cpu_write(0x8000, 0x20);
    cart_cpu_write(0xA001, 0xF0);
    CHECK(cart_cpu_read(0x7400) == 0x35 && cart_cpu_read(0x7600) == 0x53);
    CHECK(cart_cpu_read_bus(0x7FFF, 0x56) == 0xFF);
    return 0;
}

static int test_mmc6_persistence(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    int result = mmc6_persistence_cases(&paths);
    return result | save_fixture_end(&paths);
}

typedef struct {
    unsigned mapper;
    uint8_t submapper;
    uint16_t reg1_offset;
    uint16_t reg2_offset;
    uint16_t reg3_offset;
    bool has_irq;
    bool vrc2;
    bool vrc2a;
} Vrc24Case;

static const Vrc24Case vrc24_cases[] = {
    {21, 0, 0x0002, 0x0004, 0x0006, true,  false, false},
    {21, 1, 0x0002, 0x0004, 0x0006, true,  false, false},
    {21, 2, 0x0040, 0x0080, 0x00C0, true,  false, false},
    {22, 0, 0x0002, 0x0001, 0x0003, false, true,  true },
    {23, 0, 0x0001, 0x0002, 0x0003, true,  false, false},
    {23, 1, 0x0001, 0x0002, 0x0003, true,  false, false},
    {23, 2, 0x0004, 0x0008, 0x000C, true,  false, false},
    {23, 3, 0x0001, 0x0002, 0x0003, false, true,  false},
    {25, 0, 0x0002, 0x0001, 0x0003, true,  false, false},
    {25, 1, 0x0002, 0x0001, 0x0003, true,  false, false},
    {25, 2, 0x0008, 0x0004, 0x000C, true,  false, false},
    {25, 3, 0x0002, 0x0001, 0x0003, false, true,  false},
    {27, 0, 0x0001, 0x0002, 0x0003, true,  false, false},
    {183,0, 0x0004, 0x0008, 0x000C, true,  false, false},
};

static iNESHeader vrc24_header(unsigned mapper, uint8_t submapper, bool ram) {
    iNESHeader h = header_for(mapper, 0x40000, false);
    h.flags7 |= 0x08;
    h.prg_ram_size = (uint8_t)(submapper << 4);
    h.flags10 = ram ? 7 : 0;
    return h;
}

static int test_vrc24_variant_register_wiring(void) {
    for (size_t i = 0; i < sizeof(vrc24_cases) / sizeof(vrc24_cases[0]); ++i) {
        const Vrc24Case *tc = &vrc24_cases[i];
        iNESHeader h = vrc24_header(tc->mapper, tc->submapper, true);
        CHECK(fixture_with_header(&h, 0x40000, 0x40000) == (int)tc->mapper);
        CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 0);
        CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);

        cart_cpu_write(0x8000, 3);
        cart_cpu_write(0xA000, 5);
        CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xA000) == 5);
        CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);

        cart_cpu_write(0x9000, tc->vrc2 ? 1 : 2);
        CHECK(cart_get_mirroring() == (tc->vrc2 ? MIRROR_HORIZONTAL : MIRROR_SINGLE0));
        cart_cpu_write(0x9000, 0);
        CHECK(cart_get_mirroring() == MIRROR_VERTICAL);

        cart_cpu_write(0xB000, 2);
        cart_cpu_write((uint16_t)(0xB000u + tc->reg1_offset), 1);
        uint8_t expected_chr = tc->vrc2a ? 9 : 0x12;
        CHECK(cart_ppu_read(0x0000) == expected_chr);

        if (!tc->vrc2 && !(tc->mapper == 23 && tc->submapper == 0)) {
            cart_cpu_write((uint16_t)(0x9000u + tc->reg2_offset), 2);
            CHECK(cart_cpu_read(0x8000) == 30 && cart_cpu_read(0xA000) == 5);
            CHECK(cart_cpu_read(0xC000) == 3 && cart_cpu_read(0xE000) == 31);
        }
        CHECK((cart->clock != NULL) == tc->has_irq);
    }

    // Legacy submapper-zero boards accept both known address-line aliases.
    CHECK(fixture(21, 0x40000, 0x40000, false) == 21);
    cart_cpu_write(0xB000, 2);
    cart_cpu_write(0xB002, 1);
    CHECK(cart_ppu_read(0) == 0x12);
    cart_cpu_write(0xB040, 2);
    CHECK(cart_ppu_read(0) == 0x22);

    CHECK(fixture(23, 0x40000, 0x40000, false) == 23);
    cart_cpu_write(0xB000, 2);
    cart_cpu_write(0xB001, 1);
    CHECK(cart_ppu_read(0) == 0x12);
    cart_cpu_write(0xB004, 2);
    CHECK(cart_ppu_read(0) == 0x22);

    CHECK(fixture(25, 0x40000, 0x40000, false) == 25);
    cart_cpu_write(0xB000, 2);
    cart_cpu_write(0xB002, 1);
    CHECK(cart_ppu_read(0) == 0x12);
    cart_cpu_write(0xB008, 2);
    CHECK(cart_ppu_read(0) == 0x22);
    return 0;
}

static int test_vrc24_ram_latch_and_mapper183_window(void) {
    iNESHeader h = vrc24_header(22, 0, false);
    CHECK(fixture_with_header(&h, 0x40000, 0x40000) == 22);
    CHECK(cart_cpu_read_bus(0x6000, 0xA6) == 0xA6);
    cart_cpu_write(0x6000, 1);
    CHECK(cart_cpu_read_bus(0x6000, 0xA6) == 0xA7);
    cart_cpu_write(0x6FFF, 0);
    CHECK(cart_cpu_read_bus(0x6123, 0x5B) == 0x5A);
    CHECK(cart_cpu_read_bus(0x7000, 0x5B) == 0x5B);

    h = vrc24_header(23, 3, true);
    CHECK(fixture_with_header(&h, 0x40000, 0x40000) == 23);
    cart_cpu_write(0x6123, 0xA5);
    CHECK(cart_cpu_read(0x6123) == 0xA5);

    h = vrc24_header(183, 0, false);
    CHECK(fixture_with_header(&h, 0x40000, 0x40000) == 183);
    CHECK(cart_cpu_read(0x6000) == 0);
    cart_cpu_write(0x6005, 0xFF);
    CHECK(cart_cpu_read(0x6000) == 5 && cart_cpu_read(0x7FFF) == 5);
    cart_cpu_write(0x7FFE, 0);
    CHECK(cart_cpu_read(0x6000) == 14);
    return 0;
}

static int test_vrc24_irq_variants_and_phase(void) {
    for (size_t i = 0; i < sizeof(vrc24_cases) / sizeof(vrc24_cases[0]); ++i) {
        const Vrc24Case *tc = &vrc24_cases[i];
        iNESHeader h = vrc24_header(tc->mapper, tc->submapper, true);
        CHECK(fixture_with_header(&h, 0x40000, 0x40000) == (int)tc->mapper);
        if (!tc->has_irq) {
            CHECK(cart->clock == NULL);
            cart_cpu_write(0xF000, 0x0E);
            cart_cpu_write((uint16_t)(0xF000u + tc->reg1_offset), 0x0F);
            CHECK(!cart_irq_pending());
            continue;
        }

        cart_cpu_write(0xF000, 0x0E);
        cart_cpu_write((uint16_t)(0xF000u + tc->reg1_offset), 0x0F);
        cart_cpu_write((uint16_t)(0xF000u + tc->reg2_offset), 0x07);
        cart->clock(1);
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
        cart_cpu_write((uint16_t)(0xF000u + tc->reg3_offset), 0);
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
    }

    iNESHeader h = vrc24_header(21, 1, true);
    CHECK(fixture_with_header(&h, 0x40000, 0x40000) == 21);
    cart_cpu_write(0xF000, 0x0F);
    cart_cpu_write(0xF002, 0x0F);
    cart_cpu_write(0xF004, 0x02);
    cart->clock(113);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xF006, 0);
    CHECK(!cart_irq_pending());
    cart->clock(228);
    CHECK(!cart_irq_pending());

    fixture_prg[0] = 0xEA;
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cart_cpu_write(0x8000, 0);
    cpu.pc = 0x8000;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
    cart_cpu_write(0xF000, 0x0E);
    cart_cpu_write(0xF002, 0x0F);
    cart_cpu_write(0xF004, 0x06);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
    return 0;
}

static int test_vrc24_loader_rejection_preserves_cart(void) {
    iNESHeader active = vrc24_header(21, 1, true);
    CHECK(fixture_with_header(&active, 0x40000, 0x40000) == 21);
    cart_cpu_write(0x8000, 3);
    cart_cpu_write(0x6123, 0xA7);
    Mapper *previous = cart;

    iNESHeader invalid = vrc24_header(21, 3, true);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3 && cart_cpu_read(0x6123) == 0xA7);

    return 0;
}

static iNESHeader vrc7_header(uint8_t submapper, bool chr_ram, bool ram) {
    iNESHeader h = header_for(85, 0x80000, chr_ram);
    h.flags7 |= 0x08;
    h.prg_ram_size = (uint8_t)(submapper << 4);
    h.flags10 = ram ? 7 : 0;
    if (chr_ram) h.zero[0] = 7;
    else h.chr_rom_chunks = 0x20;
    return h;
}

static void vrc7_audio_write(uint8_t reg, uint8_t value) {
    cart_cpu_write(0x9010, reg);
    cart_cpu_write(0x9030, value);
}

static int test_vrc7_banks_wiring_and_ram(void) {
    for (unsigned submapper = 0; submapper <= 2; ++submapper) {
        iNESHeader h = vrc7_header((uint8_t)submapper, false, true);
        CHECK(fixture_with_header(&h, 0x80000, 0x40000) == 85);
        CHECK(cart != NULL && cart->clock != NULL);
        CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
        CHECK(cart_cpu_read_bus(0xA000, 0xA6) == 0xA6);
        CHECK(cart_cpu_read_bus(0xC000, 0x3C) == 0x3C);
        CHECK(cart_cpu_read(0xE000) == 63);
        CHECK(cart_ppu_read(0x0400) == 0); // CHR-ROM pages start unmapped.

        cart_cpu_write(0x8000, 3);
        cart_cpu_write(0x9000, 9);
        CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xC000) == 9);
        if (submapper == 0) {
            cart_cpu_write(0x8008, 4);
            CHECK(cart_cpu_read(0xA000) == 4);
            cart_cpu_write(0x8010, 6);
            CHECK(cart_cpu_read(0xA000) == 6);
            cart_cpu_write(0xA008, 5);
            CHECK(cart_ppu_read(0x0400) == 5);
            cart_cpu_write(0xA010, 7);
            CHECK(cart_ppu_read(0x0400) == 7);
        } else if (submapper == 1) {
            cart_cpu_write(0x8008, 5);
            CHECK(cart_cpu_read(0xA000) == 5);
            cart_cpu_write(0x8010, 7); // A4 is not connected on VRC7b.
            CHECK(cart_cpu_read(0x8000) == 7 && cart_cpu_read(0xA000) == 5);
            cart_cpu_write(0xA008, 6);
            CHECK(cart_ppu_read(0x0400) == 6);
            cart_cpu_write(0xA010, 8);
            CHECK(cart_ppu_read(0x0000) == 8 && cart_ppu_read(0x0400) == 6);
        } else {
            cart_cpu_write(0x8008, 5); // A3 is not connected on VRC7a.
            CHECK(cart_cpu_read(0x8000) == 5);
            cart_cpu_write(0x8010, 7);
            CHECK(cart_cpu_read(0xA000) == 7);
            cart_cpu_write(0xA008, 6);
            CHECK(cart_ppu_read(0x0000) == 6);
            cart_cpu_write(0xA010, 8);
            CHECK(cart_ppu_read(0x0400) == 8);
        }

        CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
        cart_cpu_write(0x6000, 0xA5);
        CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
        cart_cpu_write(0xE000, 0x82);
        CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
        cart_cpu_write(0x6123, 0xA5);
        CHECK(cart_cpu_read(0x6123) == 0xA5);
        cart_cpu_write(0xE000, 0x81);
        CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL && cart_cpu_read(0x6123) == 0xA5);
        cart_cpu_write(0xE000, 0x00);
        CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
        CHECK(cart_cpu_read_bus(0x6123, 0x56) == 0x56);
    }

    // CHR-RAM is linearly available before bank registers are written.
    iNESHeader ram_header = vrc7_header(2, true, true);
    CHECK(fixture_with_header(&ram_header, 0x80000, 0x2000) == 85);
    cart_ppu_write(0x0400, 0xA7);
    CHECK(cart_ppu_read(0x0400) == 0xA7);
    cart_cpu_write(0xA010, 3);
    cart_ppu_write(0x0400, 0x53);
    CHECK(cart_ppu_read(0x0C00) == 0x53);
    ram_header.zero[0] = 6; // A declared 4KB RAM aliases across the 8KB pattern window.
    CHECK(fixture_with_header(&ram_header, 0x80000, 0x1000) == 85);
    cart_ppu_write(0x1C23, 0xB6);
    CHECK(cart_ppu_read(0x0C23) == 0xB6);
    cart_cpu_write(0xA010, 7);
    CHECK(cart_ppu_read(0x0423) == 0xB6);
    return 0;
}

static int test_vrc7_irq_timing(void) {
    for (unsigned submapper = 0; submapper <= 2; ++submapper) {
        iNESHeader h = vrc7_header((uint8_t)submapper, false, true);
        CHECK(fixture_with_header(&h, 0x80000, 0x40000) == 85);
        uint16_t secondary = submapper == 2 ? 0x10u : 0x08u;
        cart_cpu_write((uint16_t)(0xE000u + secondary), 0xFE);
        cart_cpu_write(0xF000, 0x07);
        cart->clock(1);
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
        cart_cpu_write((uint16_t)(0xF000u + secondary), 0);
        CHECK(!cart_irq_pending());
        cart->clock(2);
        CHECK(cart_irq_pending());
    }

    iNESHeader h = vrc7_header(1, false, true);
    CHECK(fixture_with_header(&h, 0x80000, 0x40000) == 85);
    cart_cpu_write(0xE008, 0xFF);
    cart_cpu_write(0xF000, 0x02);
    cart->clock(113);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xF008, 0);
    CHECK(!cart_irq_pending());

    fixture_prg[0] = 0xEA;
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cart_cpu_write(0x8000, 0);
    cpu.pc = 0x8000;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
    cart_cpu_write(0xE008, 0xFE);
    cart_cpu_write(0xF000, 0x06);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
    return 0;
}

static void vrc7_program_custom_patch(void) {
    static const uint8_t patch[8] = {0x01,0x01,0x10,0x00,0xF5,0xF5,0x4F,0x4F};
    for (unsigned reg = 0; reg < 8; ++reg) vrc7_audio_write((uint8_t)reg, patch[reg]);
}

static void vrc7_key_channel0(uint8_t instrument, bool key_on) {
    vrc7_audio_write(0x30, (uint8_t)(instrument << 4));
    vrc7_audio_write(0x10, 0x80);
    vrc7_audio_write(0x20, (uint8_t)(0x08 | (key_on ? 0x10 : 0)));
}

static int test_vrc7_fm_audio(void) {
    iNESHeader h = vrc7_header(2, false, true);
    CHECK(fixture_with_header(&h, 0x80000, 0x40000) == 85);
    nes_set_region(NES_REGION_NTSC);
    CHECK(cart_expansion_audio() == 0.0f);

    vrc7_program_custom_patch();
    vrc7_key_channel0(0, true);
    cart->clock(35);
    CHECK(cart_expansion_audio() == 0.0f);
    cart->clock(1);
    CHECK(cart_expansion_audio() == 0.0f); // First chip tick starts the envelope at zero output.

    static const int16_t expected_trace[16] = {
        -693, -453, -787, -1120, -1394, -1645, -1828, -1956,
        -2031, -2037, -1988, -1873, -1708, -1492, -1231, -934
    };
    float custom_trace[16];
    float custom_energy = 0.0f;
    for (unsigned i = 0; i < 16; ++i) {
        cart->clock(36);
        custom_trace[i] = cart_expansion_audio();
        CHECK(custom_trace[i] == (float)expected_trace[i] * (1.0f / 5000.0f));
        custom_energy += custom_trace[i] < 0.0f ? -custom_trace[i] : custom_trace[i];
    }
    CHECK(custom_energy > 0.001f);

    cart->reset();
    nes_set_region(NES_REGION_PAL);
    vrc7_program_custom_patch();
    vrc7_key_channel0(0, true);
    cart->clock(34);
    for (unsigned i = 0; i < 16; ++i) {
        float previous = cart_expansion_audio();
        cart->clock(33);
        CHECK(cart_expansion_audio() == previous);
        cart->clock(1);
        CHECK(cart_expansion_audio() == custom_trace[i]);
    }

    cart->reset();
    nes_set_region(NES_REGION_NTSC);
    vrc7_program_custom_patch();
    vrc7_key_channel0(0, true);
    cart->clock(36);
    for (unsigned i = 0; i < 16; ++i) {
        cart->clock(36);
        CHECK(cart_expansion_audio() == custom_trace[i]);
    }

    cart->reset();
    nes_set_region(NES_REGION_NTSC);
    vrc7_key_channel0(1, true);
    float preset_energy = 0.0f;
    bool differs = false;
    cart->clock(36 * 32);
    for (unsigned i = 0; i < 16; ++i) {
        cart->clock(36);
        float sample = cart_expansion_audio();
        preset_energy += sample < 0.0f ? -sample : sample;
        if (sample != custom_trace[i]) differs = true;
    }
    CHECK(preset_energy > 0.0f && differs);

    cart->reset();
    nes_set_region(NES_REGION_NTSC);
    vrc7_program_custom_patch();
    vrc7_key_channel0(0, true);
    cart->clock(36 * 32);
    CHECK(cart_expansion_audio() != 0.0f);
    vrc7_audio_write(0x20, 0x08); // Key off enters release on both operators.
    cart->clock(36 * 16384);
    CHECK(cart_expansion_audio() == 0.0f);

    vrc7_key_channel0(0, true);
    cart->clock(36 * 8);
    CHECK(cart_expansion_audio() != 0.0f);
    cart_cpu_write(0xE000, 0x40);
    CHECK(cart_expansion_audio() == 0.0f);
    vrc7_audio_write(0x20, 0x08); // Ignored while audio is muted.
    cart->clock(36 * 8);
    CHECK(cart_expansion_audio() == 0.0f);
    cart_cpu_write(0xE000, 0x00);
    cart->clock(36);
    CHECK(cart_expansion_audio() != 0.0f);
    return 0;
}

static int test_vrc7_register_boundaries(void) {
    iNESHeader h = vrc7_header(2, false, true);
    CHECK(fixture_with_header(&h, 0x80000, 0x40000) == 85);
    nes_set_region(NES_REGION_NTSC);
    vrc7_program_custom_patch();
    vrc7_key_channel0(0, true);
    float trace[128];
    for (unsigned i = 0; i < 128; ++i) {
        cart->clock(36);
        trace[i] = cart_expansion_audio();
    }
    cart->reset();
    vrc7_program_custom_patch();
    vrc7_key_channel0(0, true);
    for (unsigned i = 0; i < 128; ++i) {
        for (unsigned reg = 0x40; reg <= 0xFF; ++reg)
            vrc7_audio_write((uint8_t)reg, (uint8_t)(i ^ reg));
        cart->clock(36);
        CHECK(cart_expansion_audio() == trace[i]);
    }

    // High aliases inside the 64-register window still reach channel zero.
    cart->reset();
    vrc7_program_custom_patch();
    vrc7_audio_write(0x39, 0);
    vrc7_audio_write(0x19, 0x80);
    vrc7_audio_write(0x29, 0x18);
    for (unsigned i = 0; i < 128; ++i) {
        cart->clock(36);
        CHECK(cart_expansion_audio() == trace[i]);
    }
    return 0;
}

static int test_vrc7_loader_rejection_preserves_cart(void) {
    iNESHeader active = vrc7_header(1, false, true);
    CHECK(fixture_with_header(&active, 0x80000, 0x40000) == 85);
    cart_cpu_write(0x8000, 3);
    cart_cpu_write(0xE000, 0x80);
    cart_cpu_write(0x6123, 0xA7);
    Mapper *previous = cart;

    iNESHeader invalid = vrc7_header(3, false, true);
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x80000, fixture_chr, 0x40000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3 && cart_cpu_read(0x6123) == 0xA7);

    return 0;
}

static int test_namco108_variants(void) {
    CHECK(fixture(76, 0x200000, 0x80000, false) == 76);
    CHECK(cart != NULL && cart->clock == NULL);
    CHECK(cart_ppu_read(0x0000) == 8 && cart_ppu_read(0x0800) == 10);
    CHECK(cart_ppu_read(0x1000) == 12 && cart_ppu_read(0x1800) == 14);
    namco108_write_bank(2, 3);
    namco108_write_bank(3, 7);
    namco108_write_bank(4, 11);
    namco108_write_bank(5, 15);
    CHECK(cart_ppu_read(0x0000) == 6 && cart_ppu_read(0x0400) == 7);
    CHECK(cart_ppu_read(0x0800) == 14 && cart_ppu_read(0x0C00) == 15);
    CHECK(cart_ppu_read(0x1000) == 22 && cart_ppu_read(0x1800) == 30);
    namco108_write_bank(6, 0x33);
    namco108_write_bank(7, 0x55);
    CHECK(cart_cpu_read(0x8000) == 0x33 && cart_cpu_read(0xA000) == 0x55);
    CHECK(cart_cpu_read(0xC000) == 254 && cart_cpu_read(0xE000) == 255);

    CHECK(fixture(88, 0x200000, 0x20000, false) == 88);
    CHECK(cart_ppu_read(0x1000) == 0x44 && cart_ppu_read(0x1C00) == 0x47);
    namco108_write_bank(0, 0x7F);
    namco108_write_bank(1, 0x43);
    namco108_write_bank(2, 0x05);
    namco108_write_bank(3, 0x3A);
    CHECK(cart_ppu_read(0x0000) == 0x3E && cart_ppu_read(0x0400) == 0x3F);
    CHECK(cart_ppu_read(0x0800) == 0x02 && cart_ppu_read(0x0C00) == 0x03);
    CHECK(cart_ppu_read(0x1000) == 0x45 && cart_ppu_read(0x1400) == 0x7A);
    namco108_write_bank(4, 0x09);
    namco108_write_bank(5, 0xBF);
    CHECK(cart_ppu_read(0x1800) == 0x49 && cart_ppu_read(0x1C00) == 0x7F);
    cart->reset();
    CHECK(cart_ppu_read(0x1000) == 0x44 && cart_ppu_read(0x1C00) == 0x47);

    CHECK(fixture(95, 0x200000, 0x40000, false) == 95);
    uint8_t nt[0x1000] = {0};
    nt[0] = 0x10;
    nt[0x400] = 0x20;
    CHECK(cart_nt_read(0x2000, nt) == 0x10 && cart_nt_read(0x2800, nt) == 0x20);
    cart_cpu_write(0xA001, 0); // An odd register write also connects the nametable selectors.
    CHECK(cart_nt_read(0x2000, nt) == 0x10 && cart_nt_read(0x2800, nt) == 0x10);
    cart->reset();
    CHECK(cart_nt_read(0x2800, nt) == 0x20);
    namco108_write_bank(0, 0x20);
    namco108_write_bank(1, 0x02);
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x2400, nt) == 0x20);
    CHECK(cart_nt_read(0x2800, nt) == 0x10 && cart_nt_read(0x2C00, nt) == 0x10);
    namco108_write_bank(1, 0x22);
    CHECK(cart_nt_read(0x2800, nt) == 0x20 && cart_nt_read(0x2C00, nt) == 0x20);
    cart_nt_write(0x2001, 0x35, nt);
    CHECK(nt[0x401] == 0x35 && cart_nt_read(0x2401, nt) == 0x35);

    CHECK(fixture(154, 0x200000, 0x20000, false) == 154);
    CHECK(cart_ppu_read(0x1000) == 0x44 && cart_ppu_read(0x1C00) == 0x47);
    cart_cpu_write(0x8000, 0x40);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    cart_cpu_write(0x8001, 0x45);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE1 && cart_ppu_read(0) == 4);
    cart_cpu_write(0xA000, 0);
    CHECK(cart_get_mirroring() == MIRROR_SINGLE0 && cart_ppu_read(0) == 4);
    cart_notify_ppu_address(0, 0);
    cart_notify_ppu_address(0x1000, 12);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_namco108_variant_image_loading(void) {
    const unsigned boards[] = {76, 88, 95, 154};
    for (unsigned i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        iNESHeader h = header_for(boards[i], 0x20000, false);
        h.flags7 |= 8;
        h.chr_rom_chunks = 16;
        size_t image_size;
        uint8_t *image = image_for(&h, 0x20000, 0x20000, &image_size);
        CHECK(image != NULL);
        for (unsigned bank = 0; bank < 16; ++bank)
            memset(image + sizeof(h) + bank * 0x2000, (int)bank, 0x2000);
        CHECK(load_rom_memory(image, image_size) == 0);
        free(image);
        CHECK(rom_mapper_number(&ines_header) == (int)boards[i]);
        CHECK(cart != NULL && cart->clock == NULL);
        for (unsigned reg = 6; reg <= 7; ++reg) {
            write_mem(0x9FFE, (uint8_t)(0xC0 | reg));
            write_mem(0x9FFF, (uint8_t)(3 + reg));
            CHECK(read_mem((uint16_t)(0x8000 + (reg - 6) * 0x2000)) == 3 + reg);
        }
        CHECK(read_mem(0xC000) == 14 && read_mem(0xE000) == 15);
        write_mem(0xC000, 0);
        write_mem(0xC001, 0);
        write_mem(0xE001, 0);
        cart_notify_ppu_address(0, 0);
        cart_notify_ppu_address(0x1000, 12);
        CHECK(!cart_irq_pending());
        CHECK(unload_rom());
    }
    return 0;
}

static int test_namco108_variant_loader_rejection(void) {
    CHECK(fixture(76, 0x200000, 0x80000, false) == 76);
    namco108_write_bank(6, 3);
    Mapper *previous = cart;
    iNESHeader h = header_for(154, 0x200000, false);
    h.flags7 |= 8;
    h.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x200000, fixture_chr, 0x20000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);
    return 0;
}

static int test_native_small_chr_bank_windows(void) {
    const unsigned vrc1_mappers[] = {75, 151};
    for (size_t i = 0; i < sizeof(vrc1_mappers) / sizeof(vrc1_mappers[0]); ++i) {
        CHECK(fixture(vrc1_mappers[i], 0x20000, 0x0800, false) == (int)vrc1_mappers[i]);
        fixture_chr[0x0123] = 0xA6;
        CHECK(cart_ppu_read(0x0123) == 0x23);
        cart_cpu_write(0xE000, 7);
        CHECK(cart_ppu_read(0x0123) == 0xA6);
        CHECK(cart_ppu_read(0x0923) == 0xA6);
        CHECK(cart_ppu_read(0x1123) == 0x23);
    }

    CHECK(fixture(73, 0x20000, 0x0800, false) == 73);
    fixture_chr[0x0123] = 0x53;
    CHECK(cart_ppu_read(0x0123) == 0x53);
    CHECK(cart_ppu_read(0x0923) == 0x23);

    CHECK(fixture(73, 0x20000, 0x0800, true) == 73);
    cart_ppu_write(0x1123, 0x69);
    CHECK(cart_ppu_read(0x0123) == 0x69);
    CHECK(cart_ppu_read(0x1123) == 0x69);

    iNESHeader bnrom = mapper34_header(2, false, true);
    CHECK(fixture_with_header(&bnrom, 0x40000, 0x0800) == 34);
    fixture_chr[0x0123] = 0x35;
    CHECK(cart_ppu_read(0x0123) == 0x35);
    CHECK(cart_ppu_read(0x0923) == 0x23);

    iNESHeader nina = mapper34_header(1, false, true);
    CHECK(fixture_with_header(&nina, 0x40000, 0x0800) == 34);
    fixture_chr[0x0123] = 0x96;
    CHECK(cart_ppu_read(0x0123) == 0x23);
    cart_cpu_write(0x7FFE, 0x35);
    CHECK(cart_ppu_read(0x0123) == 0x96);
    CHECK(cart_ppu_read(0x0923) == 0x23);
    cart_cpu_write(0x7FFF, 0x53);
    CHECK(cart_ppu_read(0x0923) == 0x96);
    CHECK(cart_ppu_read(0x1123) == 0x23);

    const unsigned namco_mappers[] = {76, 88, 95, 154, 206};
    for (size_t i = 0; i < sizeof(namco_mappers) / sizeof(namco_mappers[0]); ++i) {
        unsigned mapper = namco_mappers[i];
        size_t chr_bytes = 0x0200;
        unsigned slots = mapper == 76 ? 4u : 8u;
        uint16_t uncovered = (uint16_t)(chr_bytes * slots + 0x0123u);
        uint16_t register2_slot = mapper == 76 ? 0x0123u
            : (uint16_t)(chr_bytes * 4u + 0x0123u);
        CHECK(fixture(mapper, 0x20000, chr_bytes, false) == (int)mapper);
        fixture_chr[0x0123] = 0xC3;
        CHECK(cart_ppu_read(0x0123) == 0xC3);
        CHECK(cart_ppu_read((uint16_t)(chr_bytes + 0x0123u)) == 0xC3);
        namco108_write_bank(2, 0x85);
        CHECK(cart_ppu_read(register2_slot) == 0xC3);
        CHECK(cart_ppu_read(uncovered) == (uint8_t)uncovered);
    }

    CHECK(fixture(206, 0x20000, 0x0200, true) == 206);
    cart_ppu_write(0x1123, 0x5A);
    CHECK(cart_ppu_read(0x0123) == 0x5A);
    CHECK(cart_ppu_read(0x1123) == 0x5A);
    namco108_write_bank(2, 0x85);
    CHECK(cart_ppu_read(0x0123) == 0x5A);
    return 0;
}

static int test_native_small_chr_1k_families(void) {
    const unsigned taito_mappers[] = {33, 48};
    for (size_t i = 0; i < sizeof(taito_mappers) / sizeof(taito_mappers[0]); ++i) {
        CHECK(fixture(taito_mappers[i], 0x20000, 0x0200, false) == (int)taito_mappers[i]);
        fixture_chr[0x12] = 0xA6;
        CHECK(cart_ppu_read(0x0812) == 0x12);
        cart_cpu_write(0xA000, 7);
        CHECK(cart_ppu_read(0x0812) == 0xA6);
        CHECK(cart_ppu_read(0x1012) == 0x12);
    }

    const unsigned taito_x1_mappers[] = {80, 207};
    for (size_t i = 0; i < sizeof(taito_x1_mappers) / sizeof(taito_x1_mappers[0]); ++i) {
        CHECK(fixture(taito_x1_mappers[i], 0x20000, 0x0200, false) == (int)taito_x1_mappers[i]);
        fixture_chr[0x12] = 0x53;
        CHECK(cart_ppu_read(0x0812) == 0x12);
        cart_cpu_write(0x7EF2, 5);
        CHECK(cart_ppu_read(0x0812) == 0x53);
        CHECK(cart_ppu_read(0x1012) == 0x12);
    }

    CHECK(fixture(82, 0x20000, 0x0200, false) == 82);
    fixture_chr[0x12] = 0x96;
    CHECK(cart_ppu_read(0x0812) == 0x12);
    cart_cpu_write(0x7EF2, 3);
    CHECK(cart_ppu_read(0x0812) == 0x96);
    CHECK(cart_ppu_read(0x1012) == 0x12);

    const unsigned rambo_mappers[] = {64, 158};
    for (size_t i = 0; i < sizeof(rambo_mappers) / sizeof(rambo_mappers[0]); ++i) {
        CHECK(fixture(rambo_mappers[i], 0x20000, 0x0200, false) == (int)rambo_mappers[i]);
        fixture_chr[0x12] = 0x69;
        CHECK(cart_ppu_read(0x0812) == 0x69);
        cart_cpu_write(0x8000, 2);
        cart_cpu_write(0x8001, 0x35);
        CHECK(cart_ppu_read(0x0812) == 0x69);
        CHECK(cart_ppu_read(0x1012) == 0x12);
    }

    CHECK(fixture(18, 0x20000, 0x0200, false) == 18);
    fixture_chr[0x12] = 0xC3;
    CHECK(cart_ppu_read(0x0812) == 0x12);
    cart_cpu_write(0xC000, 5);
    cart_cpu_write(0xC001, 3);
    CHECK(cart_ppu_read(0x0812) == 0xC3);
    CHECK(cart_ppu_read(0x1012) == 0x12);

    CHECK(fixture(32, 0x20000, 0x0200, false) == 32);
    fixture_chr[0x12] = 0x5A;
    CHECK(cart_ppu_read(0x0812) == 0x12);
    cart_cpu_write(0xB004, 7);
    CHECK(cart_ppu_read(0x0812) == 0x5A);
    CHECK(cart_ppu_read(0x1012) == 0x12);

    CHECK(fixture(65, 0x20000, 0x0200, false) == 65);
    fixture_chr[0x12] = 0x3C;
    CHECK(cart_ppu_read(0x0812) == 0x12);
    cart_cpu_write(0xB004, 9);
    CHECK(cart_ppu_read(0x0812) == 0x3C);
    CHECK(cart_ppu_read(0x1012) == 0x12);

    const unsigned vrc24_mappers[] = {21, 22, 23, 25, 27, 183};
    for (size_t i = 0; i < sizeof(vrc24_mappers) / sizeof(vrc24_mappers[0]); ++i) {
        CHECK(fixture(vrc24_mappers[i], 0x20000, 0x0200, false) == (int)vrc24_mappers[i]);
        fixture_chr[0x12] = 0x87;
        CHECK(cart_ppu_read(0x0812) == 0x87);
        CHECK(cart_ppu_read(0x1012) == 0x12);
    }

    CHECK(fixture(85, 0x20000, 0x0200, false) == 85);
    fixture_chr[0x12] = 0xD2;
    CHECK(cart_ppu_read(0x0812) == 0x12);
    cart_cpu_write(0xC000, 0x6A);
    CHECK(cart_ppu_read(0x0812) == 0xD2);
    CHECK(cart_ppu_read(0x1012) == 0x12);

    CHECK(fixture(85, 0x20000, 0x0200, true) == 85);
    cart_ppu_write(0x1012, 0x4D);
    CHECK(cart_ppu_read(0x0012) == 0x4D);
    CHECK(cart_ppu_read(0x1012) == 0x4D);
    cart_cpu_write(0xC000, 3);
    cart_ppu_write(0x0812, 0xB4);
    CHECK(cart_ppu_read(0x0812) == 0xB4);
    return 0;
}

static int test_remaining_small_chr_single_source(void) {
    const unsigned vrc6_mappers[] = {24, 26};
    for (size_t i = 0; i < sizeof(vrc6_mappers) / sizeof(vrc6_mappers[0]); ++i) {
        unsigned mapper = vrc6_mappers[i];
        CHECK(fixture(mapper, 0x40000, 0x0200, false) == (int)mapper);
        fixture_chr[0x12] = 0xA6;
        CHECK(cart_ppu_read(0x0812) == 0x12);
        vrc6_test_write(mapper, 0xD000, 5);
        CHECK(cart_ppu_read(0x0012) == 0xA6);
        CHECK(cart_ppu_read(0x1012) == 0x12);
        vrc6_test_write(mapper, 0xB003, 0x10);
        uint8_t nt[0x1000] = {0};
        CHECK(cart_nt_read(0x2012, nt) == 0xA6);

        CHECK(fixture(mapper, 0x40000, 0x0200, true) == (int)mapper);
        cart_ppu_write(0x1012, 0x53);
        CHECK(cart_ppu_read(0x0012) == 0x53 && cart_ppu_read(0x1012) == 0x53);
        vrc6_test_write(mapper, 0xD000, 3);
        cart_ppu_write(0x0012, 0x69);
        CHECK(cart_ppu_read(0x0012) == 0x69 && cart_ppu_read(0x1012) == 0x69);
    }

    const unsigned jy_mappers[] = {90, 209, 211};
    for (size_t i = 0; i < sizeof(jy_mappers) / sizeof(jy_mappers[0]); ++i) {
        unsigned mapper = jy_mappers[i];
        CHECK(fixture(mapper, 0x8000, 0x0200, false) == (int)mapper);
        fixture_chr[0x12] = 0x96;
        CHECK(cart_ppu_read(0x0812) == 0x96);
        CHECK(cart_ppu_read(0x1012) == 0x12);
        cart_cpu_write(0xD000, 0x18);
        jy_write_chr(4, 0x123);
        CHECK(cart_ppu_read(0x0812) == 0x96);

        CHECK(fixture(mapper, 0x8000, 0x0200, true) == (int)mapper);
        cart_ppu_write(0x1012, 0x3C);
        CHECK(cart_ppu_read(0x0012) == 0x3C && cart_ppu_read(0x1012) == 0x3C);
        cart_cpu_write(0xD000, 0x18);
        jy_write_chr(0, 7);
        cart_ppu_write(0x0012, 0xC3);
        CHECK(cart_ppu_read(0x0012) == 0xC3 && cart_ppu_read(0x1012) == 0xC3);
    }

    CHECK(fixture(16, 0x20000, 0x0200, false) == 16);
    fixture_chr[0x12] = 0x5A;
    CHECK(cart_ppu_read(0x0812) == 0x12);
    cart_cpu_write(0x8004, 9);
    CHECK(cart_ppu_read(0x0812) == 0x5A && cart_ppu_read(0x1012) == 0x12);

    CHECK(fixture(16, 0x20000, 0x0200, true) == 16);
    cart_ppu_write(0x1012, 0xD2);
    CHECK(cart_ppu_read(0x0012) == 0xD2 && cart_ppu_read(0x1012) == 0xD2);
    cart_cpu_write(0x8004, 7);
    CHECK(cart_ppu_read(0x0812) == 0xD2);
    return 0;
}

static int test_vrc1_banks_mirroring_and_reset(void) {
    const unsigned mappers[] = {75, 151};
    for (size_t i = 0; i < sizeof(mappers) / sizeof(mappers[0]); ++i) {
        CHECK(fixture(mappers[i], 0x20000, 0x20000, false) == (int)mappers[i]);
        CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
        CHECK(cart_cpu_read_bus(0xA000, 0x69) == 0x69);
        CHECK(cart_cpu_read_bus(0xC000, 0xA6) == 0xA6);
        CHECK(cart_cpu_read(0xE000) == 15);
        CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x1456) == 0x56);

        cart_cpu_write(0x8123, 3);
        cart_cpu_write(0xAFFF, 5);
        cart_cpu_write(0xC456, 7);
        CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xA000) == 5);
        CHECK(cart_cpu_read(0xC000) == 7 && cart_cpu_read(0xE000) == 15);

        cart_cpu_write(0xE000, 3);
        CHECK(cart_ppu_read(0) == 12 && cart_ppu_read(0x1000) == 0);
        cart_cpu_write(0xF000, 4);
        CHECK(cart_ppu_read(0) == 12 && cart_ppu_read(0x1000) == 16);
        cart_cpu_write(0x9000, 0x06);
        CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
        CHECK(cart_ppu_read(0) == 76 && cart_ppu_read(0x1000) == 80);
        cart_cpu_write(0xEABC, 9);
        CHECK(cart_ppu_read(0) == 100 && cart_ppu_read(0x1000) == 80);
        cart_cpu_write(0xF123, 2);
        CHECK(cart_ppu_read(0) == 100 && cart_ppu_read(0x1000) == 72);
        cart_cpu_write(0x9000, 1);
        CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
        CHECK(cart_ppu_read(0) == 36 && cart_ppu_read(0x1000) == 8);

        cart->reset();
        CHECK(cart_cpu_read_bus(0x8000, 0x35) == 0x35);
        CHECK(cart_cpu_read_bus(0xA000, 0x53) == 0x53);
        CHECK(cart_cpu_read_bus(0xC000, 0x96) == 0x96);
        CHECK(cart_cpu_read(0xE000) == 15);
        CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x1456) == 0x56);
        CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    }

    iNESHeader four = header_for(75, 0x20000, false);
    four.flags6 |= 0x08;
    CHECK(fixture_with_header(&four, 0x20000, 0x20000) == 75);
    CHECK(cart_get_mirroring() == MIRROR_FOUR);
    cart_cpu_write(0x9000, 1);
    CHECK(cart_get_mirroring() == MIRROR_FOUR);

    iNESHeader ram = header_for(75, 0x20000, true);
    CHECK(fixture_with_header(&ram, 0x20000, 0x2000) == 75);
    cart_ppu_write(0x0123, 0xA6);
    cart_ppu_write(0x1123, 0x69);
    CHECK(cart_ppu_read(0x0123) == 0xA6 && cart_ppu_read(0x1123) == 0x69);
    cart_cpu_write(0xE000, 1);
    cart_ppu_write(0x0123, 0x35);
    CHECK(cart_ppu_read(0x0123) == 0x35 && cart_ppu_read(0x1123) == 0xA6);
    cart_cpu_write(0xF000, 1);
    CHECK(cart_ppu_read(0x1123) == 0x35);
    return 0;
}

static int test_vrc1_loader_rejection_preserves_cart(void) {
    CHECK(fixture(75, 0x20000, 0x20000, false) == 75);
    cart_cpu_write(0x8000, 3);
    Mapper *previous = cart;

    iNESHeader invalid = header_for(75, 0x20000, false);
    invalid.flags7 |= 0x08;
    invalid.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x20000, fixture_chr, 0x20000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x8000) == 3);
    return 0;
}

static void vrc3_write_reload(uint16_t reload) {
    cart_cpu_write(0x8000, (uint8_t)reload);
    cart_cpu_write(0x9000, (uint8_t)(reload >> 4));
    cart_cpu_write(0xA000, (uint8_t)(reload >> 8));
    cart_cpu_write(0xB000, (uint8_t)(reload >> 12));
}

static int test_vrc3_banks_irq_and_reset(void) {
    CHECK(fixture(73, 0x20000, 0x2000, false) == 73);
    CHECK(cart != NULL && cart->clock != NULL);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
    CHECK(cart_cpu_read(0xC000) == 14 && cart_cpu_read(0xE000) == 15);
    CHECK(cart_ppu_read(0x0123) == 0 && cart_ppu_read(0x1C12) == 7);

    cart_cpu_write(0xFABC, 3);
    CHECK(cart_cpu_read(0x8000) == 6 && cart_cpu_read(0xA000) == 7);
    CHECK(cart_cpu_read(0xC000) == 14 && cart_cpu_read(0xE000) == 15);

    vrc3_write_reload(0x1234);
    cart_cpu_write(0x8000, 0xFA);
    cart_cpu_write(0x9000, 0xEB);
    cart_cpu_write(0xA000, 0xDC);
    cart_cpu_write(0xB000, 0xCD);
    cart_cpu_write(0xC000, 2);
    cart->clock((int)(0x10000u - 0xDCBAu - 1u));
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());

    vrc3_write_reload(0xFFFE);
    cart_cpu_write(0xC000, 3);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xDFFF, 0);
    CHECK(!cart_irq_pending());
    cart->clock(2);
    CHECK(cart_irq_pending());

    cart_cpu_write(0xC000, 0);
    CHECK(!cart_irq_pending());
    cart->clock(32);
    CHECK(!cart_irq_pending());

    vrc3_write_reload(0xABFE);
    cart_cpu_write(0xC000, 6);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xD000, 0);
    CHECK(!cart_irq_pending());
    cart->clock(4);
    CHECK(!cart_irq_pending());

    cart_cpu_write(0x6000, 0xA6);
    cart->reset();
    CHECK(!cart_irq_pending() && cart_cpu_read(0x6000) == 0xA6);
    CHECK(cart_cpu_read_bus(0x8000, 0x35) == 0x35 && cart_cpu_read(0xC000) == 14);
    return 0;
}

static int test_vrc3_cpu_irq_and_loader(void) {
    CHECK(fixture(73, 0x4000, 0x2000, false) == 73);
    fixture_prg[0] = 0xEA;
    fixture_prg[0x3FFE] = 0;
    fixture_prg[0x3FFF] = 3;
    cart_cpu_write(0xF000, 0);
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cpu.pc = 0x8000;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
    vrc3_write_reload(0xFFFE);
    cart_cpu_write(0xC000, 2);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());

    for (unsigned i = 0; i < 8; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
    cpu.pc = 0x0200;
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    for (unsigned step = 0; step < 3 && cpu.pc != 0x0300; ++step) (void)cpu_step(&cpu);
    CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG) && cart_irq_pending());
    cart_cpu_write(0xD000, 0);
    CHECK(!cart_irq_pending());

    cart_cpu_write(0x6000, 0xA7);
    Mapper *previous = cart;
    iNESHeader invalid = header_for(73, 0x20000, false);
    invalid.flags7 |= 0x08;
    invalid.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x20000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x6000) == 0xA7);
    return 0;
}

static int test_sunsoft3_banks_mirroring_and_irq(void) {
    CHECK(fixture(67, 0x40000, 0x80000, false) == 67);
    CHECK(cart != NULL && cart->clock != NULL);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);
    CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x18A6) == 0xA6);

    const uint16_t chr_regs[] = {0x8ABC, 0x9FFF, 0xA923, 0xBFFE};
    for (unsigned slot = 0; slot < 4; ++slot) {
        uint8_t bank = (uint8_t)(5 + slot * 3);
        cart_cpu_write(chr_regs[slot], bank);
        CHECK(cart_ppu_read((uint16_t)(slot * 0x800)) == (uint8_t)(bank * 2));
        CHECK(cart_ppu_read((uint16_t)(slot * 0x800 + 0x7FF)) == (uint8_t)(bank * 2 + 1));
    }
    uint8_t before = cart_ppu_read(0);
    cart_cpu_write(0x9000, 0x7F);
    CHECK(cart_ppu_read(0) == before);

    cart_cpu_write(0xFFFF, 7);
    CHECK(cart_cpu_read(0x8000) == 14 && cart_cpu_read(0xA000) == 15);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_cpu_read(0xE000) == 31);

    uint8_t nt[0x1000] = {0};
    cart_cpu_write(0xE800, 0);
    cart_nt_write(0x2000, 0x35, nt);
    CHECK(cart_nt_read(0x2800, nt) == 0x35 && cart_nt_read(0x2400, nt) == 0);
    memset(nt, 0, sizeof(nt));
    cart_cpu_write(0xEFFF, 1);
    cart_nt_write(0x2000, 0x53, nt);
    CHECK(cart_nt_read(0x2400, nt) == 0x53 && cart_nt_read(0x2800, nt) == 0);
    memset(nt, 0, sizeof(nt));
    cart_cpu_write(0xE800, 2);
    cart_nt_write(0x2000, 0x69, nt);
    CHECK(cart_nt_read(0x2C00, nt) == 0x69 && cart_get_mirroring() == MIRROR_SINGLE0);
    memset(nt, 0, sizeof(nt));
    cart_cpu_write(0xE800, 3);
    cart_nt_write(0x2400, 0x96, nt);
    CHECK(cart_nt_read(0x2C00, nt) == 0x96 && cart_get_mirroring() == MIRROR_SINGLE1);

    cart_cpu_write(0xC800, 0x12);
    cart_cpu_write(0xD800, 0);
    cart_cpu_write(0xC800, 0);
    cart_cpu_write(0xCFFF, 1);
    cart_cpu_write(0xD800, 0x10);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart->clock(16);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xD800, 0);
    CHECK(!cart_irq_pending());
    cart->clock(16);
    CHECK(!cart_irq_pending());

    cart->reset();
    CHECK(!cart_irq_pending() && cart_cpu_read_bus(0x8000, 0xA6) == 0xA6);
    CHECK(cart_cpu_read(0xC000) == 30 && cart_ppu_read(0x0123) == 0x23);
    return 0;
}

static int test_sunsoft3_cpu_irq_and_loader(void) {
    CHECK(fixture(67, 0x4000, 0x2000, false) == 67);
    fixture_prg[0] = 0xEA;
    fixture_prg[0x3FFE] = 0;
    fixture_prg[0x3FFF] = 3;
    cart_cpu_write(0xF800, 0);
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
    cpu.pc = 0x8000;
    cpu.status = INTERRUPT_FLAG | UNUSED_FLAG;
    cart_cpu_write(0xC800, 0);
    cart_cpu_write(0xC800, 1);
    cart_cpu_write(0xD800, 0x10);
    CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());

    for (unsigned i = 0; i < 8; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
    cpu.pc = 0x0200;
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    for (unsigned step = 0; step < 3 && cpu.pc != 0x0300; ++step) (void)cpu_step(&cpu);
    CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG));

    cart_cpu_write(0xD800, 0);
    cart_cpu_write(0x6000, 0xA7);
    Mapper *previous = cart;
    iNESHeader invalid = header_for(67, 0x40000, false);
    invalid.flags7 |= 0x08;
    invalid.prg_ram_size = 0x10;
    CHECK(mapper_init_from_header(&invalid, fixture_prg, 0x40000, fixture_chr, 0x2000) == -1);
    CHECK(cart == previous && cart_cpu_read(0x6000) == 0xA7);
    return 0;
}

static int test_sunsoft4_banks_nametables_and_timer(void) {
    CHECK(fixture(68, 0x40000, 0x40000, false) == 68);
    CHECK(cart != NULL && cart->clock != NULL);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 14);
    CHECK(cart_cpu_read(0xE000) == 15 && cart_cpu_read_bus(0x6000, 0x56) == 0x56);
    CHECK(cart_ppu_read(0x0123) == 0x23 && cart_ppu_read(0x18A6) == 0xA6);

    for (unsigned slot = 0; slot < 4; ++slot) {
        uint8_t bank = (uint8_t)(3 + slot * 5);
        cart_cpu_write((uint16_t)(0x8000u + slot * 0x1000u), bank);
        CHECK(cart_ppu_read((uint16_t)(slot * 0x800u)) == (uint8_t)(bank * 2));
    }

    uint8_t nt[0x1000] = {0};
    cart_cpu_write(0xC000, 1);
    cart_cpu_write(0xD000, 2);
    cart_cpu_write(0xE000, 0x10);
    CHECK(cart_nt_read(0x2000, nt) == 0x81 && cart_nt_read(0x2400, nt) == 0x82);
    CHECK(cart_nt_read(0x2800, nt) == 0x81 && cart_nt_read(0x2C00, nt) == 0x82);
    cart_cpu_write(0xE000, 0x11);
    CHECK(cart_nt_read(0x2000, nt) == 0x81 && cart_nt_read(0x2400, nt) == 0x81);
    CHECK(cart_nt_read(0x2800, nt) == 0x82 && cart_nt_read(0x2C00, nt) == 0x82);
    for (unsigned mode = 2; mode <= 3; ++mode) {
        cart_cpu_write(0xE000, (uint8_t)(0x10 | mode));
        for (unsigned page = 0; page < 4; ++page)
            CHECK(cart_nt_read((uint16_t)(0x2000 + page * 0x400), nt) == 0x7F + mode);
    }
    cart_cpu_write(0xE000, 0);
    cart_nt_write(0x2000, 0x35, nt);
    CHECK(cart_nt_read(0x2800, nt) == 0x35);
    cart_cpu_write(0xE000, 0x10);
    CHECK(cart_nt_read(0x2000, nt) == 0x81);
    cart_cpu_write(0xE000, 0);
    CHECK(cart_nt_read(0x2000, nt) == 0x35);

    cart_cpu_write(0xF000, 0x10);
    CHECK(cart_cpu_read_bus(0x8000, 0x69) == 0x69);
    cart_cpu_write(0x6123, 0xA6);
    CHECK(cart_cpu_read(0x6123) == 0xA6 && cart_cpu_read(0x8000) == 16);
    cart->clock(107519);
    CHECK(cart_cpu_read(0x8000) == 16);
    cart->clock(1);
    CHECK(cart_cpu_read_bus(0x8000, 0x53) == 0x53);
    cart_cpu_write(0x6123, 0xA7);
    CHECK(cart_cpu_read(0x8000) == 16 && cart_cpu_read(0x6123) == 0xA7);
    cart_cpu_write(0xF000, 0x1B);
    CHECK(cart_cpu_read(0x8000) == 6 && cart_cpu_read(0xC000) == 14);
    cart->clock(107520);
    CHECK(cart_cpu_read(0x8000) == 6);
    cart_cpu_write(0xF000, 0x08);
    CHECK(cart_cpu_read_bus(0x6123, 0x96) == 0x96 && cart_cpu_read(0x8000) == 0);
    cart_cpu_write(0x6000, 0x22);
    cart_cpu_write(0xF000, 0x18);
    CHECK(cart_cpu_read(0x6123) == 0xA7);

    cart->reset();
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 14);
    CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56 && cart_get_mirroring() == MIRROR_HORIZONTAL);
    return 0;
}

static int test_sunsoft4_chr_ram_persistence_and_loader(void) {
    iNESHeader ram = header_for(68, 0x20000, true);
    CHECK(fixture_with_header(&ram, 0x20000, 0x2000) == 68);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xD000, 1);
    cart_cpu_write(0xE000, 0x10);
    uint8_t nt[0x1000] = {0};
    cart_nt_write(0x2001, 0x35, nt);
    cart_nt_write(0x2401, 0x53, nt);
    CHECK(cart_nt_read(0x2001, nt) == 0x35 && cart_nt_read(0x2401, nt) == 0x53);
    cart_cpu_write(0xE000, 0);
    CHECK(cart_nt_read(0x2001, nt) == 0 && cart_nt_read(0x2401, nt) == 0);
    cart_cpu_write(0xE000, 0x10);
    CHECK(cart_nt_read(0x2001, nt) == 0x35 && cart_nt_read(0x2401, nt) == 0x53);

    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    iNESHeader save = header_for(68, 0x20000, false);
    save.flags7 |= 8;
    save.flags6 |= 2;
    save.flags10 = 0x70;
    save.chr_rom_chunks = 0x20;
    CHECK(fixture_with_header(&save, 0x20000, 0x40000) == 68);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0xF000, 0x18);
    cart_cpu_write(0x6000, 0xA6);
    cart_cpu_write(0x7FFF, 0x69);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x2000);
    CHECK(fixture_with_header(&save, 0x20000, 0x40000) == 68);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0xF000, 0x18);
    CHECK(cart_cpu_read(0x6000) == 0xA6 && cart_cpu_read(0x7FFF) == 0x69);
    int result = save_fixture_end(&paths);
    CHECK(result == 0);

    // Four-screen headers are accepted. Until the mapper selects CHR-backed
    // nametables, all four cartridge nametable pages remain independent.
    iNESHeader four = header_for(68, 0x20000, false);
    four.flags6 |= 0x08;
    size_t four_size;
    uint8_t *four_image = image_for(&four, 0x20000, 0x2000, &four_size);
    CHECK(four_image != NULL);
    uint8_t *four_chr = four_image + sizeof(four) + 0x20000;
    memset(four_chr + 0x0400, 0x31, 0x0400);
    memset(four_chr + 0x0800, 0x52, 0x0400);
    CHECK(load_rom_memory(four_image, four_size) == 0);
    free(four_image);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    ppu_write(0x2001, 0x21);
    ppu_write(0x2401, 0x42);
    ppu_write(0x2801, 0x63);
    ppu_write(0x2C01, 0x84);
    CHECK(ppu_read(0x2001) == 0x21 && ppu_read(0x2401) == 0x42);
    CHECK(ppu_read(0x2801) == 0x63 && ppu_read(0x2C01) == 0x84);
    CHECK(discrete_cpu_store(0xC000, 1) == 0);
    CHECK(discrete_cpu_store(0xD000, 2) == 0);
    CHECK(discrete_cpu_store(0xE000, 0x10) == 0);
    CHECK(ppu_read(0x2001) == 0x31 && ppu_read(0x2401) == 0x52);
    CHECK(ppu_read(0x2801) == 0x31 && ppu_read(0x2C01) == 0x52);

    Mapper *previous = cart;
    uint8_t *previous_prg = prg_rom;
    iNESHeader invalid = four;
    invalid.flags7 |= 8;
    invalid.prg_ram_size = 0x10;
    four_image = image_for(&invalid, 0x20000, 0x2000, &four_size);
    CHECK(four_image != NULL && load_rom_memory(four_image, four_size) == -1);
    free(four_image);
    CHECK(cart == previous && prg_rom == previous_prg && ppu_read(0x2001) == 0x31);
    return 0;
}

static int test_sunsoft4_cpu_licensed_reads(void) {
    iNESHeader h = header_for(68, 0x40000, false);
    size_t image_size;
    uint8_t *image = image_for(&h, 0x40000, 0x2000, &image_size);
    CHECK(image != NULL);
    image[sizeof(h) + 8 * 0x4000] = 0xA6;
    image[sizeof(h) + 7 * 0x4000 + 0x3FFC] = 0;
    image[sizeof(h) + 7 * 0x4000 + 0x3FFD] = 2;
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu) && cpu.pc == 0x0200);
    const uint8_t read_program[] = {0xAD, 0x00, 0x80};
    const uint8_t refresh_program[] = {0x8D, 0x00, 0x60};
    for (unsigned i = 0; i < 3; ++i) {
        write_mem((uint16_t)(0x0200 + i), read_program[i]);
        write_mem((uint16_t)(0x0300 + i), refresh_program[i]);
    }
    write_mem(0xF000, 0x10);
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x80); // Unlicensed ROM leaves the operand high byte on the bus.
    cpu.pc = 0x0300;
    cpu.a = 0x53;
    CHECK(cpu_step(&cpu) == 4 && read_mem(0x6000) == 0x53);
    cart->clock(107515);
    cpu.pc = 0x0200;
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0xA6);
    cpu.pc = 0x0200;
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x80);
    cpu.pc = 0x0300;
    CHECK(cpu_step(&cpu) == 4);
    cart->clock(107516);
    cpu.pc = 0x0200;
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x80); // Expiry on the final data cycle blocks that read.
    CHECK(unload_rom());
    return 0;
}

