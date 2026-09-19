static int test_small_cartridges(void) {
    const unsigned boards[] = {0, 3, 13};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        CHECK(fixture(boards[i], 0x4000, 0x4000, boards[i] == 13) == (int)boards[i]);
        CHECK(cart_cpu_read(0x8000) == 0);
        CHECK(cart_cpu_read(0xBFFF) == 1);
        CHECK(cart_cpu_read(0xC000) == 0);
        CHECK(cart_cpu_read(0xFFFF) == 1);
    }
    CHECK(fixture(0, 0x4000, 0x1000, false) == 0);
    CHECK(cart_ppu_read(0x0FFF) == 3 && cart_ppu_read(0x1FFF) == 0xFF);
    cart_ppu_write(0x1FFF, 0xA5);
    CHECK(cart_ppu_read(0x0FFF) == 3);
    CHECK(fixture(0, 0x4000, 0x1000, true) == 0);
    cart_ppu_write(0x1FFF, 0xA5);
    CHECK(cart_ppu_read(0x0FFF) == 0xA5);
    return 0;
}

static int test_mmc1_banks_and_ram(void) {
    CHECK(fixture(1, 0x20000, 0x8000, false) == 1);
    CHECK(cart_cpu_read(0xC000) == 14);
    serial_write(0xE000, 3);
    CHECK(cart_cpu_read(0x8000) == 6 && cart_cpu_read(0xC000) == 14);
    serial_write(0x8000, 0x00);
    CHECK(cart_cpu_read(0x8000) == 4 && cart_cpu_read(0xC000) == 6);
    serial_write(0x8000, 0x0A);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 6);
    serial_write(0x8000, 0x1F);
    serial_write(0xA000, 1);
    serial_write(0xC000, 3);
    CHECK(cart_ppu_read(0x0000) == 4 && cart_ppu_read(0x1000) == 12);
    serial_write(0x8000, 0x0F);
    serial_write(0xA000, 3);
    CHECK(cart_ppu_read(0x0000) == 8 && cart_ppu_read(0x1000) == 12);
    cart_cpu_write(0x6123, 0xA6);
    serial_write(0xE000, 0x13);
    CHECK(cart_cpu_read(0x6123) == 0xFF);
    cart_cpu_write(0x6123, 0x55);
    CHECK(cart_cpu_read(0x8000) == 6);
    serial_write(0xE000, 3);
    CHECK(cart_cpu_read(0x6123) == 0xA6);
    CHECK(fixture(1, 0x4000, 0x1000, false) == 1);
    serial_write(0x8000, 0);
    serial_write(0xE000, 3);
    serial_write(0xA000, 3);
    CHECK(cart_cpu_read(0xC000) == 0 && cart_ppu_read(0x1FFF) == 3);
    return 0;
}

static int test_mmc1_serial_timing(void) {
    CHECK(fixture(1, 0x20000, 0x2000, true) == 1);
    cpu_total_cycles = 100;
    cart_cpu_write(0xE000, 1);
    cpu_total_cycles = 101;
    cart_cpu_write(0xE000, 0); // Consecutive RMW write must not enter the serial buffer.
    for (unsigned i = 0; i < 4; ++i) {
        cpu_total_cycles += 2;
        cart_cpu_write(0xE000, 0);
    }
    CHECK(cart_cpu_read(0x8000) == 2);
    serial_write(0x8000, 3);
    cpu_total_cycles += 2;
    cart_cpu_write(0xE000, 1);
    cpu_total_cycles++;
    cart_cpu_write(0x8000, 0x80); // Reset is accepted even on the consecutive cycle.
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    CHECK(cart_cpu_read(0xC000) == 14);
    serial_write(0xE000, 2);
    CHECK(cart_cpu_read(0x8000) == 4);
    return 0;
}

static int test_mmc1_outer_and_fixed_banks(void) {
    CHECK(fixture(1, 0x80000, 0x2000, true) == 1);
    CHECK(cart_cpu_read(0xC000) == 30);
    serial_write(0xA000, 0x10);
    CHECK(cart_cpu_read(0x8000) == 32 && cart_cpu_read(0xC000) == 62);
    serial_write(0x8000, 0x1C);
    serial_write(0xC000, 0);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 30);
    serial_write(0xA000, 0x10);
    CHECK(cart_cpu_read(0xC000) == 62);
    iNESHeader h = header_for(1, 0x8000, true);
    h.flags7 = 0x08;
    h.prg_ram_size = 0x50;
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x8000, 0x2000) == 1);
    serial_write(0xE000, 1);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 2);
    return 0;
}

static void latch_banks(void) {
    cart_cpu_write(0xB000, 1);
    cart_cpu_write(0xC000, 2);
    cart_cpu_write(0xD000, 3);
    cart_cpu_write(0xE000, 4);
}

static uint8_t mmc_latch_bus_read(uint16_t address, uint64_t *cycle) {
    cart_notify_ppu_address(address, (*cycle)++);
    return cart_ppu_read(address);
}

static void mmc_latch_bus_write(uint16_t address, uint8_t value, uint64_t *cycle) {
    cart_notify_ppu_address(address, (*cycle)++);
    cart_ppu_write(address, value);
}

static int test_mmc_latch_address_notifications(void) {
    for (unsigned mapper = 9; mapper <= 10; ++mapper) {
        uint64_t cycle = 1;
        CHECK(fixture(mapper, 0x20000, 0x8000, false) == (int)mapper);

        // Power-on CHR registers are not mapped until a latch transition is
        // followed by another VRAM address. The transition itself is delayed
        // even when it came from the address bus without a pattern-table read.
        CHECK(cart_ppu_read(0x1400) == 0);
        cart_notify_ppu_address(0x0FD8, cycle++);
        CHECK(cart_ppu_read(0x1400) == 0);
        cart_notify_ppu_address(0x1400, cycle++);
        CHECK(cart_ppu_read(0x1400) == 1);

        CHECK(fixture(mapper, 0x20000, 0x8000, false) == (int)mapper);
        latch_banks();
        CHECK(cart_ppu_read(0) == 8);

        // Nametable addresses never select a latch, even when their lower
        // thirteen bits match a pattern-table trigger. They can still apply
        // an update that was pending from an earlier pattern-table address.
        const uint16_t nametable_aliases[] = {0x2FD8, 0x2FDF, 0x2FE8, 0x2FEF,
                                              0x3FD8, 0x3FDF, 0x3FE8, 0x3FEF};
        for (unsigned index = 0; index < sizeof(nametable_aliases) / sizeof(nametable_aliases[0]); ++index) {
            cart_notify_ppu_address(nametable_aliases[index], cycle++);
            cart_notify_ppu_address(0, cycle++);
            CHECK(cart_ppu_read(0) == 8 && cart_ppu_read(0x1000) == 16);
        }
        cart_notify_ppu_address(0x0FD8, cycle++);
        cart_notify_ppu_address(0x2FE8, cycle++);
        CHECK(cart_ppu_read(0) == 4);
        cart_notify_ppu_address(0, cycle++);
        CHECK(cart_ppu_read(0) == 4);
        cart_notify_ppu_address(0x0FE8, cycle++);
        cart_notify_ppu_address(0x2000, cycle++);
        CHECK(cart_ppu_read(0) == 8);

        // A CPU PPUADDR write changes the cartridge address pins after the
        // PPU's normal three-clock delay. It can trigger the latch without a
        // CHR read, and the following bus address performs the bank update.
        ppu_power_on(&ppu);
        ppu_reg_write(PPUADDR, 0x0F);
        ppu_reg_write(PPUADDR, 0xD8);
        ppu_step_dots(2);
        CHECK(cart_ppu_read(0) == 8);
        ppu_step_dots(1);
        CHECK(cart_ppu_read(0) == 8);
        ppu_reg_write(PPUADDR, 0x00);
        ppu_reg_write(PPUADDR, 0x00);
        ppu_step_dots(3);
        CHECK(cart_ppu_read(0) == 4);

        // Writes use the same address hook. The triggering write still uses
        // the selected page; the next address commits the new latch state.
        CHECK(fixture(mapper, 0x20000, 0x8000, true) == (int)mapper);
        latch_banks();
        cycle = 1;
        cart_ppu_write(0, 0x53);
        mmc_latch_bus_write(0x0FD8, 0xA6, &cycle);
        CHECK(cart_ppu_read(0) == 0x53);
        cart_notify_ppu_address(0, cycle++);
        CHECK(cart_ppu_read(0) == 4);
        cart_notify_ppu_address(0x0FE8, cycle++);
        CHECK(cart_ppu_read(0) == 4);
        cart_notify_ppu_address(0, cycle++);
        CHECK(cart_ppu_read(0) == 0x53);

        // A console CPU soft reset does not reset MMC2/MMC4 state. A pending
        // latch update therefore remains pending until the next VRAM address.
        CHECK(fixture(mapper, 0x20000, 0x8000, false) == (int)mapper);
        latch_banks();
        cycle = 1;
        CHECK(cart_ppu_read(0) == 8);
        cart_notify_ppu_address(0x0FD8, cycle++);
        cpu_soft_reset(&cpu);
        CHECK(cart_ppu_read(0) == 8);
        cart_notify_ppu_address(0, cycle++);
        CHECK(cart_ppu_read(0) == 4);

        // A pending latch update is committed by any following PPU address,
        // including a nametable access. Nametable addresses that alias a latch
        // after masking to 13 bits must not start another latch transition.
        CHECK(fixture(mapper, 0x20000, 0x8000, false) == (int)mapper);
        latch_banks();
        cycle = 1;
        cart_notify_ppu_address(0x0FD8, cycle++);
        cart_notify_ppu_address(0x2FE8, cycle++);
        CHECK(cart_ppu_read(0) == 4);
        cart_notify_ppu_address(0, cycle++);
        CHECK(cart_ppu_read(0) == 4);

        cart_notify_ppu_address(0x1FD8, cycle++);
        cart_notify_ppu_address(0x3FE8, cycle++);
        CHECK(cart_ppu_read(0x1000) == 12);
        cart_notify_ppu_address(0, cycle++);
        CHECK(cart_ppu_read(0x1000) == 12);
    }
    return 0;
}

static int test_mmc2_banks_and_latches(void) {
    uint64_t cycle = 1;
    CHECK(fixture(9, 0x20000, 0x8000, false) == 9);
    cart_cpu_write(0xA000, 2);
    CHECK(cart_cpu_read(0x8000) == 2);
    CHECK(cart_cpu_read(0xA000) == 13);
    CHECK(cart_cpu_read(0xC000) == 14);
    CHECK(cart_cpu_read(0xFFFF) == 15);
    latch_banks();
    CHECK(cart_ppu_read(0) == 8 && cart_ppu_read(0x1000) == 16);
    CHECK(mmc_latch_bus_read(0x0FD9, &cycle) == 11); // Only $0FD8 is decoded in the left table.
    CHECK(cart_ppu_read(0) == 8);
    CHECK(mmc_latch_bus_read(0x0FD8, &cycle) == 11); // Triggering read still uses the old bank.
    CHECK(mmc_latch_bus_read(0, &cycle) == 4);
    CHECK(mmc_latch_bus_read(0x0FE8, &cycle) == 7);
    CHECK(mmc_latch_bus_read(0, &cycle) == 8);
    CHECK(mmc_latch_bus_read(0x1FDF, &cycle) == 19);
    CHECK(mmc_latch_bus_read(0x1000, &cycle) == 12);
    CHECK(mmc_latch_bus_read(0x1FEF, &cycle) == 15);
    CHECK(mmc_latch_bus_read(0x1000, &cycle) == 16);
    cart_ppu_write(0x1000, 0x55);
    CHECK(cart_ppu_read(0x1000) == 16);
    cart_cpu_write(0xF000, 1);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    return 0;
}

static int test_mmc4_latches_and_chr_ram(void) {
    uint64_t cycle = 1;
    CHECK(fixture(10, 0x20000, 0x8000, false) == 10);
    cart_cpu_write(0xA000, 2);
    CHECK(cart_cpu_read(0x8000) == 4 && cart_cpu_read(0xFFFF) == 15);
    latch_banks();
    CHECK(cart_ppu_read(0) == 8 && cart_ppu_read(0x1000) == 16);
    CHECK(mmc_latch_bus_read(0x0FDF, &cycle) == 11);
    CHECK(mmc_latch_bus_read(0, &cycle) == 4 && cart_ppu_read(0x1000) == 16);
    CHECK(mmc_latch_bus_read(0x0FEF, &cycle) == 7);
    CHECK(mmc_latch_bus_read(0, &cycle) == 8);
    for (unsigned mapper = 9; mapper <= 10; ++mapper) {
        CHECK(fixture(mapper, 0x20000, 0x8000, true) == (int)mapper);
        latch_banks();
        cycle = 1;
        cart_ppu_write(0x1000, 0xA5);
        (void)mmc_latch_bus_read(0x1FD8, &cycle);
        CHECK(mmc_latch_bus_read(0x1000, &cycle) == 12);
        cart_ppu_write(0x1000, 0x5A);
        (void)mmc_latch_bus_read(0x1FE8, &cycle);
        CHECK(mmc_latch_bus_read(0x1000, &cycle) == 0xA5);
        (void)mmc_latch_bus_read(0x1FD8, &cycle);
        CHECK(mmc_latch_bus_read(0x1000, &cycle) == 0x5A);
    }
    return 0;
}

static int test_mmc3_banks_and_protection(void) {
    iNESHeader h = header_for(4, 0x20000, true);
    h.flags6 |= 8;
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 4);
    cart_cpu_write(0x8000, 6);
    cart_cpu_write(0x8001, 3);
    CHECK(cart_cpu_read(0x8000) == 3 && cart_cpu_read(0xC000) == 14);
    cart_cpu_write(0x8000, 0x46);
    CHECK(cart_cpu_read(0x8000) == 14 && cart_cpu_read(0xC000) == 3);
    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 5);
    CHECK(cart_ppu_read(0) == 4 && cart_ppu_read(0x0400) == 5);
    cart_cpu_write(0x8000, 0x80);
    CHECK(cart_ppu_read(0x1000) == 4 && cart_ppu_read(0x1400) == 5);
    cart_ppu_write(0x1400, 0x56);
    cart_cpu_write(0x8000, 0);
    CHECK(cart_ppu_read(0x0400) == 0x56);
    cart_cpu_write(0xA001, 0x80);
    cart_cpu_write(0x6000, 0xAB);
    cart_cpu_write(0xA001, 0xC0);
    cart_cpu_write(0x6000, 0x55);
    CHECK(cart_cpu_read(0x6000) == 0xAB);
    cart_cpu_write(0xA001, 0);
    CHECK(cart_cpu_read(0x6000) == 0xFF);
    cart_cpu_write(0x6000, 0x11);
    cart_cpu_write(0xA001, 0x80);
    CHECK(cart_cpu_read(0x6000) == 0xAB);
    cart_cpu_write(0xA000, 1);
    CHECK(cart_get_mirroring() == MIRROR_FOUR);
    uint8_t nt[0x1000] = {0};
    for (unsigned i = 0; i < 4; ++i) cart_nt_write((uint16_t)(0x2000 + i * 0x400), (uint8_t)(i + 1), nt);
    for (unsigned i = 0; i < 4; ++i) CHECK(cart_nt_read((uint16_t)(0x2000 + i * 0x400), nt) == i + 1);
    return 0;
}

static void a12_pulse(uint64_t cycle) {
    cart_notify_ppu_address(0x2000, cycle);
    cart_notify_ppu_address(0x1000, cycle + 9);
}

static int test_mmc3_irq_edges(void) {
    CHECK(fixture(4, 0x20000, 0x2000, false) == 4);
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    cart_cpu_write(0xC000, 2);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(0); // Loading the counter does not assert a nonzero latch.
    CHECK(!cart_irq_pending());
    cart_notify_ppu_address(0x2000, 12);
    cart_notify_ppu_address(0x1000, 18); // Two CPU clocks is insufficient.
    cart_notify_ppu_address(0x1000, 30); // A held-high address is not another edge.
    CHECK(!cart_irq_pending());
    a12_pulse(36);
    CHECK(!cart_irq_pending());
    cart_notify_ppu_address(0x2000, 48);
    cart_notify_ppu_address(0x2001, 54); // Repeated low addresses do not restart the filter.
    cart_notify_ppu_address(0x1000, 57);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    a12_pulse(60);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xE001, 0);
    cart_notify_scanline();
    CHECK(!cart_irq_pending());
    a12_pulse(72);
    CHECK(cart_irq_pending());
    cpu_soft_reset(&cpu);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_mapper105_competition_board(void) {
    iNESHeader h = header_for(105, 0x40000, true);
    CHECK(cart_set_dip_switches(0));
    CHECK(fixture_with_header(&h, 0x40000, 0x2000) == 105);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 2);

    serial_write(0xA000, 0x00);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 2);
    serial_write(0xA000, 0x10);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 2);
    serial_write(0xA000, 0x06);
    CHECK(cart_cpu_read(0x8000) == 12 && cart_cpu_read(0xC000) == 14);

    serial_write(0xA000, 0x08);
    CHECK(cart_cpu_read(0x8000) == 16 && cart_cpu_read(0xC000) == 30);
    serial_write(0xE000, 3);
    CHECK(cart_cpu_read(0x8000) == 22 && cart_cpu_read(0xC000) == 30);
    serial_write(0x8000, 0x08);
    CHECK(cart_cpu_read(0x8000) == 16 && cart_cpu_read(0xC000) == 22);
    serial_write(0x8000, 0x00);
    CHECK(cart_cpu_read(0x8000) == 20 && cart_cpu_read(0xC000) == 22);

    cart_cpu_write(0x6000, 0xA5);
    CHECK(cart_cpu_read(0x6000) == 0xA5);
    serial_write(0xE000, 0x10);
    CHECK(cart_cpu_read_bus(0x6000, 0x5A) == 0x5A);
    serial_write(0xE000, 0);
    CHECK(cart_cpu_read(0x6000) == 0xA5);

    serial_write(0xA000, 0x00);
    CHECK(!cart_irq_pending());
    cart->clock(0x1FFFFFFF);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_irq_ack();
    cart->clock(100);
    CHECK(!cart_irq_pending());

    serial_write(0xA000, 0x10);
    CHECK(!cart_irq_pending());
    CHECK(cart_set_dip_switches(3));
    serial_write(0xA000, 0x00);
    cart->clock(0x25FFFFFF);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());

    cart->reset();
    CHECK(!cart_irq_pending() && cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 2);
    CHECK(cart_set_dip_switches(0));

    size_t image_size;
    uint8_t *image = image_for(&h, 0x40000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    free(image);
    uint8_t *previous = prg_rom;
    uint8_t previous_value = cart_cpu_read(0x8000);
    h.flags7 |= 0x08;
    h.prg_ram_size = 0x10;
    h.zero[0] = 7;
    image = image_for(&h, 0x40000, 0, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == -1);
    free(image);
    CHECK(prg_rom == previous && cart_cpu_read(0x8000) == previous_value);
    return 0;
}

static int test_mapper105_fixed_chr_and_serial_timing(void) {
    CHECK(fixture(105, 0x40000, 0x2000, true) == 105);
    cart_ppu_write(0x0003, 0xA5);
    cart_ppu_write(0x1003, 0x5A);
    serial_write(0x8000, 0x1C);
    serial_write(0xA000, 3);
    serial_write(0xC000, 0);
    CHECK(cart_ppu_read(0x0003) == 0xA5 && cart_ppu_read(0x1003) == 0x5A);
    cart_ppu_write(0x0003, 0x7B);
    CHECK(fixture_chr[3] == 0x7B && fixture_chr[0x1003] == 0x5A);

    CHECK(cart_set_dip_switches(0));
    cart->reset();
    serial_write(0xA000, 0);
    cart->clock(0x20000000);
    CHECK(cart_irq_pending());
    cart_irq_ack();
    ++cpu_total_cycles;
    cart_cpu_write(0xC000, 0);
    cart->clock(1);
    CHECK(!cart_irq_pending()); // The consecutive write must not shift or restart the timer.
    for (unsigned bit = 0; bit < 4; ++bit) {
        cpu_total_cycles += 2;
        cart_cpu_write(0xC000, 0);
        cart->clock(1);
        CHECK(!cart_irq_pending());
    }
    cpu_total_cycles += 2;
    cart_cpu_write(0xC000, 0);
    cart->clock(1);
    CHECK(cart_irq_pending()); // A completed register write updates timer control.
    serial_write(0xA000, 0x10);
    CHECK(!cart_irq_pending());
    cpu_total_cycles += 2;
    cart_cpu_write(0x8000, 0x80);
    CHECK(!cart_irq_pending() && cart_ppu_read(0x0003) == 0x7B);

    CHECK(fixture(105, 0x40000, 0x2000, false) == 105);
    CHECK(cart_ppu_read(0x0123) == 0x23);
    serial_write(0xA000, 0x1F);
    cart_ppu_write(0x0123, 0x69);
    CHECK(cart_ppu_read(0x0123) == 0x23 && fixture_chr[0x0123] == 0);
    return 0;
}

static int test_mapper105_dips_and_cpu_irq(void) {
    CHECK(fixture(105, 0x40000, 0x2000, true) == 105);
    for (unsigned dips = 0; dips < 16; ++dips) {
        CHECK(cart_set_dip_switches(dips));
        CHECK(!cart_set_dip_switches(256) && cart_dip_switches() == dips);
        cart->reset();
        serial_write(0xA000, 0);
        uint32_t limit = 0x20000000u | (dips << 25);
        cart->clock((int)(limit - 1));
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
    }
    cart->reset();
    CHECK(cart_set_dip_switches(15));
    serial_write(0xA000, 0);
    cart->clock(0x21000000);
    CHECK(!cart_irq_pending());
    CHECK(cart_set_dip_switches(0));
    cart->clock(1);
    CHECK(cart_irq_pending());

    cart->reset();
    fixture_prg[0x7FFC] = 0;
    fixture_prg[0x7FFD] = 2;
    fixture_prg[0x7FFE] = 0;
    fixture_prg[0x7FFF] = 3;
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu) && cpu.pc == 0x0200);
    write_mem(0x0200, 0xEA);
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    serial_write(0xA000, 0);
    cart->clock(0x1FFFFFFF);
    CHECK(cpu_step(&cpu) == 9 && cpu.pc == 0x0300 && cart_irq_pending());
    serial_write(0xA000, 0x10);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_mapper232_multicart_banks(void) {
    iNESHeader h = header_for(232, 0x40000, true);
    for (unsigned submapper = 0; submapper <= 1; ++submapper) {
        if (submapper) {
            h.flags7 |= 0x08;
            h.prg_ram_size = 0x10;
            h.flags10 = 7;
            h.zero[0] = 7;
        }
        CHECK(fixture_with_header(&h, 0x40000, 0x2000) == 232);
        CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 6);
        static const uint8_t standard_values[4] = {0x00, 0x08, 0x10, 0x18};
        static const uint8_t swapped_values[4] = {0x00, 0x10, 0x08, 0x18};
        for (unsigned block = 0; block < 4; ++block) {
            uint8_t outer = submapper ? swapped_values[block] : standard_values[block];
            cart_cpu_write(0x8000, (uint8_t)(outer | 0xE7));
            for (unsigned page = 0; page < 4; ++page) {
                cart_cpu_write(0xC000, (uint8_t)(0xFC | page));
                CHECK(cart_cpu_read(0x8000) == 2u * (block * 4u + page));
                CHECK(cart_cpu_read(0xC000) == 2u * (block * 4u + 3u));
            }
        }
        cart_ppu_write(0x1234, (uint8_t)(0x70 + submapper));
        CHECK(cart_ppu_read(0x1234) == 0x70 + submapper);
        cart->reset();
        CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xC000) == 6);
    }

    CHECK(fixture(71, 0x40000, 0x2000, true) == 71);
    cart_cpu_write(0xC000, 2);
    CHECK(cart_cpu_read(0x8000) == 4);

    h.flags7 |= 0x08;
    h.prg_ram_size = 0x20;
    h.flags10 = 7;
    h.zero[0] = 7;
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x40000, fixture_chr, 0x2000) == -1);
    CHECK(cart_cpu_read(0x8000) == 4);
    return 0;
}

static int test_mapper232_loader_and_cpu_bus(void) {
    for (unsigned submapper = 0; submapper <= 1; ++submapper) {
        iNESHeader h = header_for(232, 0x40000, true);
        h.flags6 |= 1;
        h.flags7 |= 0x08;
        h.prg_ram_size = (uint8_t)(submapper << 4);
        h.zero[0] = 7;
        size_t size;
        uint8_t *image = image_for(&h, 0x40000, 0, &size);
        CHECK(image != NULL);
        for (size_t i = 0; i < 0x40000; ++i)
            image[sizeof(h) + i] = (uint8_t)(i / 0x4000);
        CHECK(load_rom_memory(image, size) == 0);
        CHECK(rom_mapper_number(&ines_header) == 232);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        CHECK(cpu_power_on(&cpu));
        CHECK(read_mem(0x8000) == 0 && read_mem(0xFFFF) == 3);
        CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
        write_mem(0xBFFF, submapper ? 0x10 : 0x08);
        write_mem(0xFFFF, 0xFE);
        CHECK(read_mem(0x8000) == 6 && read_mem(0xBFFF) == 6);
        CHECK(read_mem(0xC000) == 7 && read_mem(0xFFFF) == 7);
        write_mem(0x7FFF, 0x18);
        CHECK(read_mem(0x8000) == 6 && cart_get_mirroring() == MIRROR_VERTICAL);
        cart_ppu_write(0x1FFF, 0xA6);

        Mapper *previous = cart;
        uint8_t *previous_prg = prg_rom;
        image[8] = 0x20;
        CHECK(load_rom_memory(image, size) == -1);
        memcpy(image, &h, sizeof(h));
        CHECK(load_rom_memory(image, size - 1) == -1);
        CHECK(cart == previous && prg_rom == previous_prg);
        CHECK(read_mem(0x8000) == 6 && read_mem(0xFFFF) == 7);
        CHECK(cart_ppu_read(0x1FFF) == 0xA6);
        image[11] = 8; // A 16 KiB chip retains the fixed first 8 KiB window.
        CHECK(load_rom_memory(image, size) == 0 && chr_size == 0x4000);
        cart_ppu_write(0x1FFF, 0x69);
        CHECK(cart_ppu_read(0x1FFF) == 0x69 && chr_rom[0x3FFF] == 0);
        free(image);
    }
    return 0;
}

static int test_mmc3_revision_a_irq(void) {
    iNESHeader h = header_for(4, 0x20000, false);
    size_t image_size = 0;
    uint8_t *image = image_for(&h, 0x20000, 0x2000, &image_size);
    CHECK(image != NULL);

    CHECK(cart_set_mmc3_revision_name("standard"));
    CHECK(load_rom_memory(image, image_size) == 0);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(0);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(18);
    CHECK(cart_irq_pending());

    CHECK(cart_set_mmc3_revision_name("a"));
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(0);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(18);
    CHECK(!cart_irq_pending());

    cart_cpu_write(0xC000, 1);
    cart_cpu_write(0xC001, 0);
    a12_pulse(36);
    CHECK(!cart_irq_pending());
    a12_pulse(54);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    a12_pulse(72);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xE001, 0);
    a12_pulse(90);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(108);
    CHECK(!cart_irq_pending());

    CHECK(!cart_set_mmc3_revision_name("unknown"));
    CHECK(strcmp(cart_mmc3_revision_name(), "a") == 0);

    h.flags7 |= 0x08;
    h.prg_ram_size = 0x10;
    h.flags10 = 4;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 4);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(126);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    a12_pulse(144);
    CHECK(cart_irq_pending()); // MMC6 retains repeated zero-counter IRQs under profile A.

    h.prg_ram_size = 0x30;
    h.flags10 = 0;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 4);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x1000, 0);
    cart_notify_ppu_address(0x2000, 1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    for (uint64_t edge = 1; edge <= 8; ++edge) {
        cart_notify_ppu_address(0x1000, edge * 2);
        cart_notify_ppu_address(0x2000, edge * 2 + 1);
        CHECK(cart_irq_pending() == (edge == 8));
    }
    CHECK(cart_set_mmc3_revision_name("standard"));
    return 0;
}

static int test_mmc3_revision_a_cpu_irq(void) {
    CHECK(cart_set_mmc3_revision_name("a"));
    CHECK(fixture(4, 0x20000, 0x2000, false) == 4);
    fixture_prg[0x1FFFC] = 0;
    fixture_prg[0x1FFFD] = 2;
    fixture_prg[0x1FFFE] = 0;
    fixture_prg[0x1FFFF] = 3;
    nes_set_region(NES_REGION_NTSC);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    cart->reset();
    for (unsigned i = 0; i < 16; ++i) write_mem((uint16_t)(0x200 + i), 0xEA);
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    uint64_t edge = ppu.total_cycles;
    cart_notify_ppu_address(0x2000, edge);
    cart_notify_ppu_address(0x1000, edge + 6);
    CHECK(!cart_irq_pending());
    CHECK(cpu_step(&cpu) == 2 && cpu.pc == 0x201);
    edge = ppu.total_cycles;
    cart_notify_ppu_address(0x2000, edge);
    cart_notify_ppu_address(0x1000, edge + 9);
    CHECK(cart_irq_pending());
    CHECK(cpu_step(&cpu) == 9 && cpu.pc == 0x300);
    CHECK(read_mem(0x1FC) == 0x02); // IRQ follows the NOP's final polling cycle.
    cart_cpu_write(0xE000, 0);
    CHECK(!cart_irq_pending());
    CHECK(cart_set_mmc3_revision_name("standard"));
    return 0;
}

static int test_mmc3_render_trace(void) {
    CHECK(fixture(4, 0x20000, 0x2000, false) == 4);
    cart_cpu_write(0xC000, 1);
    cart_cpu_write(0xE001, 0);
    // BG at $0000, sprites at $1000: short sprite-fetch gaps are filtered.
    for (unsigned dot = 1; dot < 257; dot += 8) {
        cart_notify_ppu_address(0x2000, dot);
        cart_notify_ppu_address(0, dot + 4);
    }
    for (unsigned dot = 257; dot < 321; dot += 8) {
        cart_notify_ppu_address(0x2000, dot);
        cart_notify_ppu_address(0x1000, dot + 4);
    }
    CHECK(!cart_irq_pending()); // Exactly one qualified edge loaded the latch.
    cart_notify_ppu_address(0x2000, 321);
    cart_notify_ppu_address(0x1000, 602);
    CHECK(cart_irq_pending());

    CHECK(fixture(4, 0x20000, 0x2000, false) == 4);
    cart_cpu_write(0xC000, 1);
    cart_cpu_write(0xE001, 0);
    // Omitting the dot-0 pattern address would produce a spurious qualified
    // nine-dot gap from the dummy nametable fetch to the next line's BG fetch.
    cart_notify_ppu_address(0x2000, 321);
    cart_notify_ppu_address(0x1000, 325);
    CHECK(!cart_irq_pending());
    cart_notify_ppu_address(0x2000, 337);
    cart_notify_ppu_address(0x1000, 346);
    CHECK(!cart_irq_pending()); // First qualifying edge loads one.
    cart_notify_ppu_address(0x2000, 598);
    cart_notify_ppu_address(0x1000, 666);
    CHECK(cart_irq_pending());

    CHECK(fixture(4, 0x20000, 0x2000, false) == 4);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x1000, 333);
    cart_notify_ppu_address(0x2000, 337);
    // Visible dot 0 drives a pattern address without reading CHR. It splits
    // the low interval, so neither this pulse nor the dot-5 fetch clocks IRQ.
    cart_notify_ppu_address(0x1000, 341);
    cart_notify_ppu_address(0x2000, 342);
    cart_notify_ppu_address(0x1000, 346);
    CHECK(!cart_irq_pending());
    cart_notify_ppu_address(0x2000, 598);
    cart_notify_ppu_address(0x1000, 666);
    CHECK(cart_irq_pending()); // The long sprite-fetch interval still qualifies.
    return 0;
}

static int test_taito_banks_aliases_and_mirroring(void) {
    CHECK(fixture(33, 0x80000, 0x20000, false) == 33);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_cpu_read(0xC000) == 62 && cart_cpu_read(0xE000) == 63);

    cart_cpu_write(0x9000, 0x7F); // $9000 aliases $8000.
    cart_cpu_write(0xD001, 0x42); // $D001 aliases $8001.
    CHECK(cart_cpu_read(0x8000) == 63 && cart_cpu_read(0xA000) == 2);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    cart_cpu_write(0x8000, 3);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);

    cart_cpu_write(0x9002, 3);
    cart_cpu_write(0xD003, 5);
    cart_cpu_write(0xB000, 12);
    cart_cpu_write(0xB001, 13);
    cart_cpu_write(0xF002, 14);
    cart_cpu_write(0xF003, 15);
    static const uint8_t expected_chr[] = {6, 7, 10, 11, 12, 13, 14, 15};
    for (unsigned slot = 0; slot < 8; ++slot)
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == expected_chr[slot]);

    uint8_t nt[0x1000] = {0};
    cart_nt_write(0x2000, 0x33, nt);
    CHECK(cart_nt_read(0x2800, nt) == 0x33);
    CHECK(cart_nt_read(0x2400, nt) == 0);
    CHECK(!cart_irq_pending());
    a12_pulse(0);
    a12_pulse(12);
    CHECK(!cart_irq_pending() && cart->clock == NULL);

    cart->reset();
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_ppu_read(0x0012) == 0x12 && cart_ppu_read(0x1C34) == 0x34);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL && !cart_irq_pending());

    CHECK(fixture(48, 0x80000, 0x20000, false) == 48);
    CHECK(cart != NULL && cart->clock != NULL);
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_ppu_read(0x0012) == 0x12 && cart_ppu_read(0x1C34) == 0x34);
    cart_cpu_write(0xE000, 0);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0x9000, 0x41); // Bank write does not carry mapper 33's mirroring bit.
    cart_cpu_write(0x9001, 4);
    CHECK(cart_cpu_read(0x8000) == 1 && cart_cpu_read(0xA000) == 4);
    CHECK(cart_cpu_read(0xC000) == 62 && cart_cpu_read(0xE000) == 63);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0x9002, 7);
    cart_cpu_write(0x9003, 9);
    cart_cpu_write(0xB000, 20);
    cart_cpu_write(0xB001, 21);
    cart_cpu_write(0xB002, 22);
    cart_cpu_write(0xB003, 23);
    static const uint8_t expected_chr48[] = {14, 15, 18, 19, 20, 21, 22, 23};
    for (unsigned slot = 0; slot < 8; ++slot)
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == expected_chr48[slot]);
    cart_cpu_write(0xE001, 0x40); // Only $E000 is decoded in this register group.
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    cart_cpu_write(0xF000, 0x40); // $F000 aliases $E000.
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);

    memset(nt, 0, sizeof(nt));
    cart_nt_write(0x2000, 0x48, nt);
    CHECK(cart_nt_read(0x2400, nt) == 0x48);
    CHECK(cart_nt_read(0x2800, nt) == 0);
    cart->reset();
    CHECK(cart_cpu_read_bus(0x8000, 0x56) == 0x56 && cart_cpu_read_bus(0xA000, 0x96) == 0x96);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL && !cart_irq_pending());
    return 0;
}

static int test_taito48_irq(void) {
    CHECK(fixture(48, 0x20000, 0x2000, false) == 48);
    cart_cpu_write(0xC000, 0xFD); // Inverted reload value is 2 on the original board.
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xC002, 0);
    a12_pulse(0);
    CHECK(!cart_irq_pending());
    cart_notify_ppu_address(0x2000, 12);
    cart_notify_ppu_address(0x1000, 18); // Two CPU clocks low does not qualify.
    CHECK(!cart_irq_pending());
    a12_pulse(24);
    CHECK(!cart_irq_pending());
    a12_pulse(36);
    CHECK(!cart_irq_pending()); // Counter reached zero, but assertion is delayed.
    cart->clock(21);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());

    cart_cpu_write(0xD000, 0xFF); // $D000 aliases $C000 and acknowledges the line.
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xD001, 0);
    cart_cpu_write(0xD002, 0);
    a12_pulse(48); // Reload zero schedules an IRQ on this qualifying edge.
    cart->clock(22);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xD003, 0);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xC001, 0);
    a12_pulse(60);
    cart->clock(22);
    CHECK(!cart_irq_pending()); // Disabled counter activity cannot schedule another IRQ.

    cart_cpu_write(0xC002, 0);
    cart_cpu_write(0xC001, 0);
    a12_pulse(72);
    cart->clock(5);
    cart_cpu_write(0xC003, 0);
    cart->clock(16);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending()); // Disabling the counter does not cancel an already latched delay.

    iNESHeader h = header_for(48, 0x20000, false);
    h.flags7 |= 0x08;
    h.prg_ram_size = 0x10; // Submapper 1.
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 48);
    cart_cpu_write(0xC000, 0xFF); // Submapper 1 adds one after inversion.
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xC002, 0);
    a12_pulse(0);
    cart->clock(6);
    CHECK(!cart_irq_pending()); // First edge loaded one, so no delay was scheduled.
    a12_pulse(12);
    cart->clock(5);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart->reset();
    CHECK(!cart_irq_pending());
    cart->clock(32);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_taito48_cpu_irq(void) {
    for (unsigned submapper = 0; submapper < 2; ++submapper) {
        iNESHeader h = header_for(48, 0x20000, false);
        h.flags7 |= 8;
        h.prg_ram_size = (uint8_t)(submapper << 4);
        CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 48);
        fixture_prg[0x1FFFC] = 0;
        fixture_prg[0x1FFFD] = 2;
        fixture_prg[0x1FFFE] = 0;
        fixture_prg[0x1FFFF] = 3;
        nes_set_region(NES_REGION_NTSC);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        cpu_power_on(&cpu);
        cart->reset();
        for (unsigned i = 0; i < 32; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
        cpu.status &= (uint8_t)~INTERRUPT_FLAG;
        cart_cpu_write(0xC000, submapper ? 0 : 0xFF);
        cart_cpu_write(0xC001, 0);
        cart_cpu_write(0xC002, 0);
        a12_pulse(0);
        unsigned delay = submapper ? 6 : 22;
        for (unsigned elapsed = 2; elapsed < delay; elapsed += 2) {
            CHECK(cpu_step(&cpu) == 2);
            CHECK(!cart_irq_pending());
        }
        CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
        for (unsigned step = 0; step < 3 && cpu.pc != 0x0300; ++step) (void)cpu_step(&cpu);
        CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG));
        CHECK(cart_irq_pending());
        write_mem(0xC003, 0);
        CHECK(!cart_irq_pending());
        write_mem(0x8000, 3);
        cpu_soft_reset(&cpu);
        CHECK(cart_cpu_read(0x8000) == 3);
    }
    return 0;
}

static int test_rambo1_banks_and_modes(void) {
    CHECK(fixture(64, 0x200000, 0x20000, false) == 64);
    CHECK(cart != NULL && cart->clock != NULL);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 2 && cart_cpu_read(0xE000) == 0xFF);
    for (unsigned slot = 0; slot < 8; ++slot)
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == slot);

    cart_cpu_write(0x9FFE, 6);
    cart_cpu_write(0x9FFF, 5);
    cart_cpu_write(0x8000, 7);
    cart_cpu_write(0x8001, 7);
    cart_cpu_write(0x8000, 15);
    cart_cpu_write(0x8001, 9);
    CHECK(cart_cpu_read(0x8000) == 5 && cart_cpu_read(0xA000) == 7);
    CHECK(cart_cpu_read(0xC000) == 9 && cart_cpu_read(0xE000) == 0xFF);
    cart_cpu_write(0x8000, 0x46);
    CHECK(cart_cpu_read(0x8000) == 9 && cart_cpu_read(0xA000) == 7);
    CHECK(cart_cpu_read(0xC000) == 5 && cart_cpu_read(0xE000) == 0xFF);

    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 10);
    cart_cpu_write(0x8000, 1);
    cart_cpu_write(0x8001, 20);
    for (unsigned reg = 2; reg <= 5; ++reg) {
        cart_cpu_write(0x8000, (uint8_t)reg);
        cart_cpu_write(0x8001, (uint8_t)(28 + reg));
    }
    static const uint8_t paired_chr[] = {10, 11, 20, 21, 30, 31, 32, 33};
    for (unsigned slot = 0; slot < 8; ++slot)
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == paired_chr[slot]);

    cart_cpu_write(0x8000, 0x28);
    cart_cpu_write(0x8001, 12);
    cart_cpu_write(0x8000, 0x29);
    cart_cpu_write(0x8001, 22);
    static const uint8_t one_k_chr[] = {10, 12, 20, 22, 30, 31, 32, 33};
    for (unsigned slot = 0; slot < 8; ++slot)
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == one_k_chr[slot]);
    cart_cpu_write(0x8000, 0xA0);
    static const uint8_t inverted_chr[] = {30, 31, 32, 33, 10, 12, 20, 22};
    for (unsigned slot = 0; slot < 8; ++slot)
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == inverted_chr[slot]);

    uint8_t nt[0x1000] = {0};
    cart_cpu_write(0xA000, 1);
    CHECK(cart_get_mirroring() == MIRROR_HORIZONTAL);
    cart_nt_write(0x2000, 0x64, nt);
    CHECK(cart_nt_read(0x2400, nt) == 0x64 && cart_nt_read(0x2800, nt) == 0);
    cart_cpu_write(0xA000, 0);
    CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    memset(nt, 0, sizeof(nt));
    cart_nt_write(0x2000, 0x46, nt);
    CHECK(cart_nt_read(0x2800, nt) == 0x46 && cart_nt_read(0x2400, nt) == 0);

    cart->reset();
    CHECK(!cart_irq_pending() && cart_get_mirroring() == MIRROR_VERTICAL);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0xA000) == 1);
    CHECK(cart_cpu_read(0xC000) == 2 && cart_cpu_read(0xE000) == 0xFF);
    for (unsigned slot = 0; slot < 8; ++slot)
        CHECK(cart_ppu_read((uint16_t)(slot * 0x400)) == slot);
    return 0;
}

static int test_rambo1_irq_sources(void) {
    CHECK(fixture(64, 0x20000, 0x2000, false) == 64);

    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 1);
    cart_cpu_write(0xE001, 0);
    cart->clock(3);
    CHECK(!cart_irq_pending());
    cart->clock(1); // Divide-by-four counter clock schedules the one-cycle IRQ delay.
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    CHECK(!cart_irq_pending());

    cart_cpu_write(0xC001, 1);
    cart_cpu_write(0xE001, 0);
    cart->clock(2);
    cart_notify_ppu_address(0x2000, 0);
    cart_notify_ppu_address(0x1000, 60); // PPU edges are ignored in CPU-cycle mode.
    cart_cpu_write(0xC001, 0);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1); // The old CPU prescaler is allowed one final counter clock.
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);

    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x2000, 100);
    cart_notify_ppu_address(0x1000, 129);
    cart->clock(2);
    CHECK(!cart_irq_pending()); // Twenty-nine PPU cycles low is too short.
    cart_notify_ppu_address(0x2000, 140);
    cart_notify_ppu_address(0x1000, 170);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);

    cart_cpu_write(0xC000, 2);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    for (unsigned edge = 0; edge < 3; ++edge) {
        uint64_t low = 200 + edge * 40;
        cart_notify_ppu_address(0x2000, low);
        cart_notify_ppu_address(0x1000, low + 30);
        cart->clock(2);
        CHECK(!cart_irq_pending());
    }
    cart_notify_ppu_address(0x2000, 320);
    cart_notify_ppu_address(0x1000, 350);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart->clock(1);
    CHECK(cart_irq_pending());
    cart_cpu_write(0xE000, 0);

    cart_cpu_write(0xC001, 1);
    cart_cpu_write(0xE001, 0);
    cart->clock(4);
    cart->reset();
    CHECK(!cart_irq_pending());
    cart->clock(16);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_rambo158_nametables(void) {
    CHECK(fixture(158, 0x20000, 0x20000, false) == 158);
    uint8_t nt[0x1000] = {0};
    nt[0] = 0x10;
    nt[0x400] = 0x20;
    CHECK(cart_nt_read(0x2000, nt) == 0x10 && cart_nt_read(0x2400, nt) == 0x20);
    CHECK(cart_nt_read(0x2800, nt) == 0x10 && cart_nt_read(0x2C00, nt) == 0x20);

    cart_cpu_write(0x8000, 0);
    cart_cpu_write(0x8001, 0x80);
    cart_cpu_write(0x8000, 1);
    cart_cpu_write(0x8001, 0x00);
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x2400, nt) == 0x20);
    CHECK(cart_nt_read(0x2800, nt) == 0x10 && cart_nt_read(0x2C00, nt) == 0x10);
    cart_cpu_write(0xA000, 1); // Mapper 158 does not use the RAMBO-1 mirroring register.
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x2800, nt) == 0x10);

    cart_cpu_write(0x8000, 0x82);
    cart_cpu_write(0x8001, 0x80);
    cart_cpu_write(0x8000, 0x83);
    cart_cpu_write(0x8001, 0x00);
    cart_cpu_write(0x8000, 0x84);
    cart_cpu_write(0x8001, 0x80);
    cart_cpu_write(0x8000, 0x85);
    cart_cpu_write(0x8001, 0x00);
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x2400, nt) == 0x10);
    CHECK(cart_nt_read(0x2800, nt) == 0x20 && cart_nt_read(0x2C00, nt) == 0x10);

    cart_cpu_write(0x8000, 0x80);
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x2400, nt) == 0x10);
    cart_cpu_write(0x8000, 0x28);
    cart_cpu_write(0x8001, 0x80); // The nametable latch decodes only the lower three register bits.
    CHECK(cart_nt_read(0x2000, nt) == 0x20 && cart_nt_read(0x2400, nt) == 0x20);
    cart_cpu_write(0x8000, 0xA3);
    cart_cpu_write(0x8001, 0x00);

    cart_nt_write(0x2001, 0xA1, nt);
    cart_nt_write(0x2401, 0xB1, nt);
    CHECK(nt[0x401] == 0xA1 && nt[1] == 0xB1);
    cart->reset();
    CHECK(cart_nt_read(0x2000, nt) == 0x10 && cart_nt_read(0x2400, nt) == 0x20);
    CHECK(cart_nt_read(0x2800, nt) == 0x10 && cart_nt_read(0x2C00, nt) == 0x20);
    CHECK(!cart_irq_pending());
    return 0;
}

static int test_rambo1_ram_and_odd_chr_banks(void) {
    const unsigned boards[] = {64, 158};
    for (unsigned i = 0; i < 2; ++i) {
        CHECK(fixture(boards[i], 0x20000, 0x8000, true) == (int)boards[i]);
        cart_cpu_write(0x8000, 0);
        cart_cpu_write(0x8001, 5);
        CHECK(cart_ppu_read(0) == 5 && cart_ppu_read(0x0400) == 5);
        cart_ppu_write(0x0123, 0xA6);
        CHECK(cart_ppu_read(0x0523) == 0xA6);
        cart_cpu_write(0x8000, 0x28);
        cart_cpu_write(0x8001, 6);
        CHECK(cart_ppu_read(0x0123) == 0xA6 && cart_ppu_read(0x0523) == 6);
        cart_ppu_write(0x0523, 0x69);
        cart_cpu_write(0x8000, 0xA0);
        CHECK(cart_ppu_read(0x1123) == 0xA6 && cart_ppu_read(0x1523) == 0x69);
        cart_cpu_write(0x6000, 0x35);
        cart_cpu_write(0x7FFF, 0x53);
        for (unsigned protection = 0; protection <= 0xC0; protection += 0x40) {
            cart_cpu_write(0xA001, (uint8_t)protection);
            CHECK(cart_cpu_read(0x6000) == 0x35 && cart_cpu_read(0x7FFF) == 0x53);
            cart_cpu_write(0x6123, (uint8_t)(protection | 0x15));
            CHECK(cart_cpu_read(0x6123) == (protection | 0x15));
        }
        CHECK(cart_cpu_read_bus(0x5000, 0x96) == 0x96);
        cart->reset();
        CHECK(cart_cpu_read(0x6000) == 0x35 && cart_ppu_read(0x1523) == 0xA6);
        iNESHeader h = header_for(boards[i], 0x20000, false);
        h.flags7 |= 8;
        CHECK(fixture_with_header(&h, 0x20000, 0x2000) == (int)boards[i]);
        cart_cpu_write(0x6000, 0xFF);
        CHECK(cart_cpu_read_bus(0x6000, 0x56) == 0x56);
        cart_ppu_write(0, 0xFF);
        CHECK(cart_ppu_read(0) == 0);
    }
    return 0;
}

static int test_rambo1_irq_boundaries(void) {
    CHECK(fixture(64, 0x20000, 0x2000, false) == 64);
    const uint8_t reloads[] = {0, 1, 2, 254, 255};
    const unsigned first_clocks[] = {1, 2, 4, 256, 1};
    const unsigned later_clocks[] = {1, 2, 3, 255, 256};
    for (unsigned test = 0; test < sizeof(reloads); ++test) {
        cart->reset();
        cart_cpu_write(0xC000, reloads[test]);
        cart_cpu_write(0xC001, 1);
        cart_cpu_write(0xE001, 0);
        cart->clock((int)(first_clocks[test] * 4));
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
        cart_cpu_write(0xE000, 0);
        cart_cpu_write(0xE001, 0);
        cart->clock((int)(later_clocks[test] * 4 - 1));
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
    }

    for (unsigned phase = 0; phase < 4; ++phase) {
        cart->reset();
        cart_cpu_write(0xC000, 0);
        cart_cpu_write(0xC001, 1);
        cart_cpu_write(0xE001, 0);
        cart->clock((int)phase);
        cart_cpu_write(0xC001, 0);
        cart->clock((int)(4 - phase));
        CHECK(!cart_irq_pending());
        cart->clock(1);
        CHECK(cart_irq_pending());
        cart_cpu_write(0xE000, 0);
        cart_cpu_write(0xE001, 0);
        cart->clock(16);
        CHECK(!cart_irq_pending()); // Switching to PPU mode allows exactly one final CPU counter clock.
    }

    cart->reset();
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 1);
    cart_cpu_write(0xE001, 0);
    cart->clock(3);
    cart_cpu_write(0xC001, 1);
    cart->clock(4);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart->clock(1);
    CHECK(cart_irq_pending()); // An IRQ already in the output delay survives acknowledgment.
    cart_cpu_write(0xE000, 0);
    cart->clock(20);
    CHECK(!cart_irq_pending());

    cart->reset();
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    cart_notify_ppu_address(0x2000, 89330);
    cart_notify_ppu_address(0x2001, 89342);
    cart_notify_ppu_address(0x1000, 89360);
    cart->clock(1);
    CHECK(!cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart->clock(1);
    CHECK(cart_irq_pending());
    return 0;
}

static int test_rambo1_cpu_and_rendering_irq(void) {
    const unsigned boards[] = {64, 158};
    for (unsigned board = 0; board < 2; ++board) {
        CHECK(fixture(boards[board], 0x20000, 0x2000, false) == (int)boards[board]);
        fixture_prg[0x1FFFC] = 0;
        fixture_prg[0x1FFFD] = 2;
        fixture_prg[0x1FFFE] = 0;
        fixture_prg[0x1FFFF] = 3;
        nes_set_region(NES_REGION_NTSC);
        ppu_power_on(&ppu);
        apu_power_on(&apu);
        cpu_power_on(&cpu);
        for (unsigned i = 0; i < 16; ++i) write_mem((uint16_t)(0x0200 + i), 0xEA);
        cart_cpu_write(0xC000, 0);
        cart_cpu_write(0xC001, 1);
        cart_cpu_write(0xE001, 0);
        CHECK(cpu_step(&cpu) == 2 && !cart_irq_pending());
        CHECK(cpu_step(&cpu) == 2 && !cart_irq_pending());
        CHECK(cpu_step(&cpu) == 2 && cart_irq_pending());
        cpu.status &= (uint8_t)~INTERRUPT_FLAG;
        for (unsigned step = 0; step < 3 && cpu.pc != 0x0300; ++step) (void)cpu_step(&cpu);
        CHECK(cpu.pc == 0x0300 && (cpu.status & INTERRUPT_FLAG));
        cart_cpu_write(0xE000, 0);
        CHECK(!cart_irq_pending());

        ppu_power_on(&ppu);
        ppu_step_dots(341 * 262 * 2);
        write_mem(0x2000, 8);
        write_mem(0x2001, 0x18);
        cart->reset();
        cpu.pc = 0x0200;
        cpu.status = (uint8_t)(UNUSED_FLAG | INTERRUPT_FLAG);
        write_mem(0x0200, 0x4C);
        write_mem(0x0201, 0);
        write_mem(0x0202, 2);
        cart_cpu_write(0xC000, 0);
        cart_cpu_write(0xC001, 0);
        cart_cpu_write(0xE001, 0);
        for (unsigned step = 0; step < 1000 && !cart_irq_pending(); ++step) (void)cpu_step(&cpu);
        CHECK(cart_irq_pending()); // Rendering fetches reach the cartridge's physical A12 filter.
        cart_cpu_write(0xE000, 0);
        cart_cpu_write(0x8000, 6);
        cart_cpu_write(0x8001, 5);
        cpu_soft_reset(&cpu);
        CHECK(cart_cpu_read(0x8000) == 5);
    }
    return 0;
}

static int test_rambo1_loader(void) {
    const unsigned boards[] = {64, 158};
    for (unsigned board = 0; board < 2; ++board) {
        iNESHeader h = header_for(boards[board], 0x20000, false);
        h.flags7 |= 8;
        h.chr_rom_chunks = 32;
        size_t size;
        uint8_t *image = image_for(&h, 0x20000, 0x40000, &size);
        CHECK(image != NULL);
        image[sizeof(h) + 0x20000 + 0x3FC00] = 0x96;
        int loaded = load_rom_memory(image, size);
        free(image);
        CHECK(loaded == 0 && rom_mapper_number(&ines_header) == (int)boards[board]);
        cart_cpu_write(0x8000, 2);
        cart_cpu_write(0x8001, 0xFF);
        CHECK(cart_ppu_read(0x1000) == 0x96);
        cart_cpu_write(0xC001, 1);
        cart_cpu_write(0xE001, 0);
        cart->clock(5);
        CHECK(cart_irq_pending());
        uint8_t *old_prg = prg_rom, *old_chr = chr_rom;
        iNESHeader rejected[] = {h};
        rejected[0].prg_ram_size = 0x10;
        for (unsigned i = 0; i < sizeof(rejected) / sizeof(rejected[0]); ++i) {
            image = image_for(&rejected[i], 0x20000, (size_t)rejected[i].chr_rom_chunks * 0x2000, &size);
            CHECK(image != NULL);
            loaded = load_rom_memory(image, size);
            free(image);
            CHECK(loaded == -1 && prg_rom == old_prg && chr_rom == old_chr && cart_irq_pending());
            CHECK(cart_ppu_read(0x1000) == 0x96);
        }
        image = image_for(&h, 0x20000, 0x40000, &size);
        CHECK(image != NULL);
        loaded = load_rom_memory(image, size - 1);
        free(image);
        CHECK(loaded == -1 && prg_rom == old_prg && cart_irq_pending());
    }
    return 0;
}

