static int test_loader_trainers_and_sizes(void) {
    iNESHeader h = header_for(0, 0x4000, true);
    h.flags6 |= 4;
    size_t size;
    uint8_t *image = image_for(&h, 0x4000, 0, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    memset(image, 0, size);
    free(image);
    CHECK(loaded == 0 && prg_size == 0x4000);
    CHECK(cart_cpu_read(0x8000) == 0x5C && cart_cpu_read(0xC000) == 0x5C);
    CHECK(cart_cpu_read(0x7001) == 1 && cart_cpu_read(0x71FF) == 0xFF);
    CHECK(cart_cpu_read(0x7200) == 0);

    h = header_for(0, 0x4000, false);
    h.flags7 = 0x08;
    h.flags9 = 0xF0;
    h.chr_rom_chunks = 12 << 2; // 2^12 bytes of CHR-ROM.
    image = image_for(&h, 0x4000, 0x1000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && chr_size == 0x1000);
    CHECK(cart_ppu_read(0) == 0xA5 && cart_ppu_read(0x1FFF) == 0xFF);
    cart_ppu_write(0x1FFF, 0x55);
    CHECK(cart_ppu_read(0x0FFF) == 0xA5);

    h.flags9 = 0x10;
    h.chr_rom_chunks = 0; // The extended count makes this ROM, not RAM.
    image = image_for(&h, 0x4000, 0x200000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && chr_size == 0x200000);
    cart_ppu_write(0, 0x55);
    CHECK(cart_ppu_read(0) == 0xA5);

    h = header_for(13, 0x8000, true);
    image = image_for(&h, 0x8000, 0, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && chr_size == 0x4000);
    for (unsigned i = 0; i < 4; ++i) {
        cart_cpu_write(0x8000, (uint8_t)i);
        cart_ppu_write(0x1000, (uint8_t)(i + 1));
    }
    for (unsigned i = 0; i < 4; ++i) {
        cart_cpu_write(0x8000, (uint8_t)i);
        CHECK(cart_ppu_read(0x1000) == i + 1);
    }
    h = header_for(0, 0x4000, true);
    h.flags7 = 0x08;
    h.zero[0] = 6; // 4KB CHR-RAM, mirrored through the PPU's 8KB window.
    image = image_for(&h, 0x4000, 0, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && chr_size == 0x1000);
    cart_ppu_write(0x1000, 0x91);
    CHECK(cart_ppu_read(0) == 0x91);

    // On two-socket MMC5 layouts the trainer initializes volatile RAM before
    // battery data is loaded into the save socket.
    h = header_for(5, 0x20000, false);
    h.flags7 = 8;
    h.flags6 |= 0x06;
    h.flags10 = 0x77;
    image = image_for(&h, 0x20000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && cart_cpu_read(0x7001) == 0 && cart_cpu_read(0x71FF) == 0);
    cart_cpu_write(0x5113, 4);
    CHECK(cart_cpu_read(0x7000) == 0 && cart_cpu_read(0x7001) == 1 && cart_cpu_read(0x71FF) == 0xFF);
    return 0;
}

static int test_loader_rejection_preserves_cart(void) {
    iNESHeader h = header_for(0, 0x4000, true);
    size_t size;
    uint8_t *image = image_for(&h, 0x4000, 0, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0);
    uint8_t *previous_prg = prg_rom;
    uint8_t *previous_chr = chr_rom;
    iNESHeader previous_header = ines_header;
    CHECK(load_rom_memory(NULL, 0) == -1);
    CHECK(load_rom_memory((const uint8_t *)&h, sizeof(h) - 1) == -1);
    CHECK(load_rom_memory((const uint8_t *)&h, sizeof(h)) == -1);
    h.signature[0] = 0;
    CHECK(load_rom_memory((const uint8_t *)&h, sizeof(h)) == -1);
    h.signature[0] = 'N';
    h.flags6 = 4;
    CHECK(load_rom_memory((const uint8_t *)&h, sizeof(h)) == -1);
    h.flags6 = 0;
    h.flags7 = 0x08;
    h.flags9 = 0x0F;
    h.prg_rom_chunks = 0xFF; // 7 * 2^63 overflows size_t on 64-bit hosts.
    CHECK(load_rom_memory((const uint8_t *)&h, sizeof(h)) == -1);
    h.prg_rom_chunks = 32 << 2; // Representable on 64-bit, but no 4GB payload follows.
    CHECK(load_rom_memory((const uint8_t *)&h, sizeof(h)) == -1);
    h = header_for(0, 0x4000, false);
    image = image_for(&h, 0x4000, 0, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size); // Header declares missing CHR bytes.
    free(image);
    CHECK(loaded == -1);
    h = header_for(0, 0x4000, true);
    h.flags7 = 8;
    h.prg_ram_size = 1;
    image = image_for(&h, 0x4000, 0, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size); // Mapper $100 must not alias mapper zero.
    free(image);
    CHECK(loaded == -1);
    CHECK(prg_rom == previous_prg && chr_rom == previous_chr);
    CHECK(memcmp(&ines_header, &previous_header, sizeof(ines_header)) == 0);
    CHECK(cart_cpu_read(0xFFFF) == 0x5C);
    return 0;
}

static int test_loader_small_and_irregular_rom_pages(void) {
    size_t size;
    iNESHeader h = header_for(0, 0x4000, false);
    h.flags7 = 0x08;
    h.prg_rom_chunks = 0x1C; // 1 * 2^7 = 128 bytes.
    h.flags9 = 0x0F;
    uint8_t *image = image_for(&h, 0x80, 0x2000, &size);
    CHECK(image != NULL);
    uint8_t *prg = image + sizeof(h);
    for (unsigned i = 0; i < 0x80; ++i) prg[i] = (uint8_t)i;
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(prg_size == 0x100);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_cpu_read(0x807F) == 0x7F);
    CHECK(cart_cpu_read(0x8080) == 0 && cart_cpu_read(0x80FF) == 0x7F);

    h = header_for(0, 0x4000, false);
    h.flags7 = 0x08;
    h.prg_rom_chunks = 0x34; // 1 * 2^13 = 8 KiB.
    h.flags9 = 0x0F;
    image = image_for(&h, 0x2000, 0x2000, &size);
    CHECK(image != NULL);
    prg = image + sizeof(h);
    memset(prg, 0x41, 0x2000);
    prg[0x100] = 0xA6;
    prg[0x1FFC] = 0x00;
    prg[0x1FFD] = 0x81;
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu) && cpu.pc == 0x8100);
    CHECK(read_mem(0x8100) == 0xA6 && read_mem(0xA100) == 0xA6);

    h = header_for(0, 0x4000, false);
    h.flags7 = 0x08;
    h.prg_rom_chunks = 0x31; // 3 * 2^12 = 12 KiB.
    h.flags9 = 0x0F;
    image = image_for(&h, 0x3000, 0x2000, &size);
    CHECK(image != NULL);
    memset(image + sizeof(h), 0x52, 0x3000);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_cpu_read(0x8100) == 0x52 && cart_cpu_read(0xB100) == 0x52);
    CHECK(cart_cpu_read_bus(0xE100, 0xD7) == 0xD7);

    // The 32 KiB discrete-window boards use the same whole-image repetition
    // rule for small PRG payloads. These paths are explicitly safe below the
    // legacy 16 KiB minimum.
    const unsigned small_discrete[] = {11, 79, 94, 113, 144, 146, 180};
    for (size_t i = 0; i < sizeof(small_discrete) / sizeof(small_discrete[0]); ++i) {
        h = header_for(small_discrete[i], 0x4000, false);
        h.flags7 |= 0x08;
        h.prg_rom_chunks = 0x34; // 8 KiB.
        h.flags9 = 0x0F;
        image = image_for(&h, 0x2000, 0x2000, &size);
        CHECK(image != NULL);
        memset(image + sizeof(h), (int)(0x60 + i), 0x2000);
        CHECK(load_rom_memory(image, size) == 0);
        free(image);
        CHECK(cart_cpu_read(0x8100) == (uint8_t)(0x60 + i));
        CHECK(cart_cpu_read(0xA100) == (uint8_t)(0x60 + i));
        CHECK(cart_cpu_read(0xC100) == (uint8_t)(0x60 + i));
        CHECK(cart_cpu_read(0xE100) == (uint8_t)(0x60 + i));
    }

    h = header_for(69, 0x4000, false);
    h.flags7 |= 0x08;
    h.prg_rom_chunks = 0x32; // 5 * 2^12 = 20 KiB: two full 8 KiB pages plus a tail.
    h.chr_rom_chunks = 0x26; // 5 * 2^9 = 2560 bytes: two full 1 KiB pages plus a tail.
    h.flags9 = 0xFF;
    image = image_for(&h, 0x5000, 0x0A00, &size);
    CHECK(image != NULL);
    prg = image + sizeof(h);
    uint8_t *chr = prg + 0x5000;
    memset(prg, 0x10, 0x2000);
    memset(prg + 0x2000, 0x20, 0x2000);
    memset(prg + 0x4000, 0x30, 0x1000);
    memset(chr, 0x40, 0x400);
    memset(chr + 0x400, 0x50, 0x400);
    memset(chr + 0x800, 0x60, 0x200);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_cpu_read(0xE000) == 0x20);
    sunsoft69_command(9, 3);
    CHECK(cart_cpu_read(0x8000) == 0x20);
    sunsoft69_command(9, 2);
    CHECK(cart_cpu_read(0x8000) == 0x10);
    sunsoft69_command(0, 1);
    CHECK(cart_ppu_read(0) == 0x50);
    sunsoft69_command(0, 2);
    CHECK(cart_ppu_read(0) == 0x40);

    h = header_for(69, 0x4000, false);
    h.flags7 |= 0x08;
    h.prg_rom_chunks = 0x46; // 5 * 2^17 = 640 KiB.
    h.flags9 = 0x0F;
    image = image_for(&h, 0xA0000, 0x2000, &size);
    CHECK(image != NULL);
    prg = image + sizeof(h);
    for (unsigned bank = 0; bank < 80; ++bank)
        memset(prg + bank * 0x2000, (int)bank, 0x2000);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_cpu_read(0xE000) == 79);
    sunsoft69_command(9, 63);
    CHECK(cart_cpu_read(0x8000) == 63);

    Mapper *previous = cart;
    uint8_t *previous_prg = prg_rom;
    h.prg_ram_size = 0x10; // FME-7 does not decode a submapper number.
    image = image_for(&h, 0xA0000, 0x2000, &size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, size - 1) == -1);
    CHECK(cart == previous && prg_rom == previous_prg && cart_cpu_read(0xE000) == 79);
    CHECK(load_rom_memory(image, size) == 0);
    sunsoft69_command(9, 63);
    CHECK(cart_cpu_read(0x8000) == 0x5C);
    free(image);
    return 0;
}

static int test_loader_existing_board_page_geometry(void) {
    size_t size;

    // Mapper 24 uses 8 KiB PRG pages and 1 KiB CHR pages. An oversized NES 2.0
    // image remains valid even when the register width cannot select every page;
    // the fixed PRG window still points at the final complete physical page.
    iNESHeader h = header_for(24, 0x4000, false);
    h.flags7 |= 0x08;
    h.prg_rom_chunks = 0x46; // 5 * 2^17 = 640 KiB.
    h.chr_rom_chunks = 0x46; // 5 * 2^17 = 640 KiB.
    h.flags9 = 0xFF;
    uint8_t *image = image_for(&h, 0xA0000, 0xA0000, &size);
    CHECK(image != NULL);
    uint8_t *prg = image + sizeof(h);
    uint8_t *chr = prg + 0xA0000;
    for (unsigned bank = 0; bank < 80; ++bank)
        memset(prg + bank * 0x2000, (int)bank, 0x2000);
    for (unsigned bank = 0; bank < 640; ++bank)
        memset(chr + bank * 0x0400, (int)bank, 0x0400);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_cpu_read(0xE000) == 79);
    vrc6_test_write(24, 0x8000, 15);
    CHECK(cart_cpu_read(0x8000) == 30 && cart_cpu_read(0xA000) == 31);
    vrc6_test_write(24, 0xD000, 0xFF);
    vrc6_test_write(24, 0xB003, 0);
    CHECK(cart_ppu_read(0) == 0xFF);

    // Exponent-encoded tails do not form extra mapper pages. Two complete PRG
    // and CHR pages are selectable here; the 4 KiB PRG and 512-byte CHR tails
    // remain outside the board's page count.
    h = header_for(24, 0x4000, false);
    h.flags7 |= 0x08;
    h.prg_rom_chunks = 0x32; // 5 * 2^12 = 20 KiB.
    h.chr_rom_chunks = 0x26; // 5 * 2^9 = 2560 bytes.
    h.flags9 = 0xFF;
    image = image_for(&h, 0x5000, 0x0A00, &size);
    CHECK(image != NULL);
    prg = image + sizeof(h);
    chr = prg + 0x5000;
    memset(prg, 0x10, 0x2000);
    memset(prg + 0x2000, 0x20, 0x2000);
    memset(prg + 0x4000, 0x30, 0x1000);
    memset(chr, 0x40, 0x0400);
    memset(chr + 0x0400, 0x50, 0x0400);
    memset(chr + 0x0800, 0x60, 0x0200);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    CHECK(cart_cpu_read(0xE000) == 0x20);
    vrc6_test_write(24, 0x8000, 1);
    CHECK(cart_cpu_read(0x8000) == 0x10 && cart_cpu_read(0xA000) == 0x20);
    vrc6_test_write(24, 0xD000, 1);
    vrc6_test_write(24, 0xB003, 0);
    CHECK(cart_ppu_read(0) == 0x50);
    vrc6_test_write(24, 0xD000, 2);
    CHECK(cart_ppu_read(0) == 0x40);

    // Mapper 66 selects complete 32 KiB PRG and 8 KiB CHR pages. A 48 KiB /
    // 12 KiB image contains one selectable page of each type plus a tail;
    // selecting higher banks wraps to those complete pages instead of folding
    // the trailing bytes into the banked windows.
    h = header_for(66, 0x8000, false);
    h.flags7 |= 0x08;
    h.prg_rom_chunks = 0x39; // 3 * 2^14 = 48 KiB.
    h.chr_rom_chunks = 0x31; // 3 * 2^12 = 12 KiB.
    h.flags9 = 0xFF;
    image = image_for(&h, 0xC000, 0x3000, &size);
    CHECK(image != NULL);
    prg = image + sizeof(h);
    chr = prg + 0xC000;
    memset(prg, 0x21, 0x8000);
    memset(prg + 0x8000, 0x42, 0x4000);
    memset(chr, 0x31, 0x2000);
    memset(chr + 0x2000, 0x62, 0x1000);
    CHECK(load_rom_memory(image, size) == 0);
    free(image);
    cart_cpu_write(0x8000, 0x33);
    CHECK(cart_cpu_read(0x8000) == 0x21 && cart_cpu_read(0xE000) == 0x21);
    CHECK(cart_ppu_read(0) == 0x31 && cart_ppu_read(0x1FFF) == 0x31);
    return 0;
}

static int test_loader_region_and_console_type(void) {
    iNESHeader h = header_for(0, 0x4000, false);
    h.flags9 = 1;
    size_t size;
    uint8_t *image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && nes_timing()->region == NES_REGION_PAL);

    h = header_for(0, 0x4000, false);
    h.flags7 = 0x04; // Archaic iNES: later header bytes are unreliable padding.
    h.flags9 = 1;
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && nes_timing()->region == NES_REGION_NTSC);

    h = header_for(0, 0x4000, false);
    h.flags7 = 0x08;
    h.zero[1] = 3;
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && nes_timing()->region == NES_REGION_DENDY);
    Mapper *previous_cart = cart;
    uint8_t *previous_prg = prg_rom;

    h.flags7 = 0x09; // VS hardware requires NTSC rather than the current Dendy region.
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == -1 && cart == previous_cart && prg_rom == previous_prg);
    CHECK(nes_timing()->region == NES_REGION_DENDY);

    h.flags7 = 0x0B;
    h.zero[2] = 5; // An extended console subtype not provided by this core.
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == -1 && cart == previous_cart && nes_timing()->region == NES_REGION_DENDY);

    h.zero[1] = 2; // Dual-compatible cartridges default to NTSC timing.
    h.zero[2] = 0; // Extended console type 0 is the ordinary NES family.
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && nes_timing()->region == NES_REGION_NTSC);

    h = header_for(0, 0x4000, false);
    h.flags7 = 2; // PlayChoice-10 in a clean legacy header.
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    uint8_t *playchoice = (uint8_t *)realloc(image, size + 0x2000);
    CHECK(playchoice != NULL);
    image = playchoice;
    memset(image + size, 0xE7, 0x2000); // Cabinet payload follows cartridge PRG/CHR data.
    loaded = load_rom_memory(image, size + 0x2000);
    free(image);
    CHECK(loaded == 0 && nes_timing()->region == NES_REGION_NTSC);
    CHECK(cart_cpu_read(0x8123) == 0x5C && cart_ppu_read(0x0123) == 0xA5);

    h = header_for(0, 0x4000, false);
    h.flags7 = 0x0A; // NES 2.0 direct PlayChoice console type.
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && nes_timing()->region == NES_REGION_NTSC);
    CHECK(cart_cpu_read(0x8123) == 0x5C && cart_ppu_read(0x0123) == 0xA5);

    h.flags7 = 0x0B;
    h.zero[2] = 2; // NES 2.0 extended PlayChoice subtype.
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && nes_timing()->region == NES_REGION_NTSC);
    CHECK(cart_cpu_read(0x8123) == 0x5C && cart_ppu_read(0x0123) == 0xA5);

    previous_cart = cart;
    previous_prg = prg_rom;
    h.zero[2] = 5;
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == -1 && cart == previous_cart && prg_rom == previous_prg);
    CHECK(cart_cpu_read(0x8123) == 0x5C && cart_ppu_read(0x0123) == 0xA5);
    unload_rom();
    CHECK(nes_timing()->region == NES_REGION_NTSC);
    return 0;
}

static int test_ram_header_sizes(void) {
    RomRamSizes ram;
    iNESHeader h = header_for(0, 0x4000, true);
    CHECK(rom_ram_sizes(&h, &ram) == 0);
    CHECK(ram.prg_ram == 0x2000 && ram.prg_nvram == 0);
    CHECK(ram.chr_ram == 0x2000 && ram.chr_nvram == 0);
    h.flags6 |= 2;
    h.prg_ram_size = 4;
    CHECK(rom_ram_sizes(&h, &ram) == 0);
    CHECK(ram.prg_ram == 0 && ram.prg_nvram == 0x2000);
    h = header_for(13, 0x8000, true);
    CHECK(rom_ram_sizes(&h, &ram) == 0 && ram.chr_ram == 0x4000);
    h = header_for(5, 0x20000, true);
    CHECK(rom_ram_sizes(&h, &ram) == 0 && ram.prg_ram == 0x10000 && ram.prg_nvram == 0);
    h.flags6 |= 2;
    CHECK(rom_ram_sizes(&h, &ram) == 0 && ram.prg_ram == 0 && ram.prg_nvram == 0x10000);

    h.flags7 = 8;
    h.prg_ram_size = 0x50; // Byte 8 is mapper metadata in NES 2.0.
    for (unsigned shift = 0; shift < 16; ++shift) {
        h.flags10 = (uint8_t)(shift | (shift << 4));
        h.zero[0] = h.flags10;
        size_t expected = shift ? (size_t)64 << shift : 0;
        CHECK(rom_ram_sizes(&h, &ram) == 0);
        CHECK(ram.prg_ram == expected && ram.prg_nvram == expected);
        CHECK(ram.chr_ram == expected && ram.chr_nvram == expected);
    }
    CHECK(rom_ram_sizes(NULL, &ram) == -1 && rom_ram_sizes(&h, NULL) == -1);
    return 0;
}

static int test_prg_ram_capacity(void) {
    iNESHeader h = header_for(0, 0x4000, true);
    h.flags7 = 8;
    h.zero[0] = 7;
    h.flags10 = 1; // 128 bytes cannot supply a complete 256-byte bus page.
    CHECK(fixture_with_header(&h, 0x4000, 0x2000) == 0);
    cart_cpu_write(0x6000, 0xA6);
    cart_cpu_write(0x607F, 0x56);
    CHECK(cart_cpu_read_bus(0x6080, 0xD1) == 0xD1);
    CHECK(cart_cpu_read_bus(0x7FFF, 0xE2) == 0xE2);
    h.flags10 = 2; // A complete 256-byte page repeats through the RAM window.
    CHECK(fixture_with_header(&h, 0x4000, 0x2000) == 0);
    cart_cpu_write(0x6000, 0xA6);
    cart_cpu_write(0x60FF, 0x56);
    CHECK(cart_cpu_read(0x6100) == 0xA6 && cart_cpu_read(0x7FFF) == 0x56);
    cart_cpu_write(0x7F00, 0x91);
    CHECK(cart_cpu_read(0x6000) == 0x91);
    h.flags10 = 0;
    CHECK(fixture_with_header(&h, 0x4000, 0x2000) == 0);
    cart_cpu_write(0x6000, 0xA6);
    CHECK(cart_cpu_read(0x6000) == 0xFF && cart_cpu_read(0x7FFF) == 0xFF);
    h.flags10 = 7;
    CHECK(fixture_with_header(&h, 0x4000, 0x2000) == 0);
    cart_cpu_write(0x6000, 0xA6);
    CHECK(cart_cpu_read(0x6080) == 0);

    h.flags10 = 8; // The first 8 KiB of a 16 KiB chip supplies the fixed window.
    CHECK(fixture_with_header(&h, 0x4000, 0x2000) == 0);
    cart_cpu_write(0x6000, 0xA6);
    cart_cpu_write(0x7FFF, 0x56);
    CHECK(cart_cpu_read(0x6000) == 0xA6 && cart_cpu_read(0x7FFF) == 0x56);
    h.flags10 = 0x77;
    h.flags6 |= 2;
    CHECK(fixture_with_header(&h, 0x4000, 0x2000) == 0);
    cart_cpu_write(0x6000, 0xC3);
    CHECK(cart_cpu_read(0x6000) == 0xC3);
    h.flags6 &= (uint8_t)~2u; // A nonvolatile chip still requires a battery flag.
    CHECK(mapper_init_from_header(&h, fixture_prg, 0x4000, fixture_chr, 0x2000) == -1);
    CHECK(cart_cpu_read(0x6000) == 0xC3);
    return 0;
}

static int test_mmc1_banked_ram(void) {
    iNESHeader h = header_for(1, 0x20000, true);
    h.flags7 |= 8;
    h.prg_ram_size = 0;
    h.flags6 |= 2;
    h.flags10 = 0x90; // SXROM: 32KB battery-backed PRG RAM.
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    for (unsigned bank = 0; bank < 4; ++bank) {
        serial_write(0xA000, (uint8_t)(bank << 2));
        cart_cpu_write(0x6000, (uint8_t)(0xA0 + bank));
        cart_cpu_write(0x7FFF, (uint8_t)(0xB0 + bank));
    }
    for (unsigned bank = 0; bank < 4; ++bank) {
        serial_write(0xA000, (uint8_t)(bank << 2));
        CHECK(cart_cpu_read(0x6000) == 0xA0 + bank && cart_cpu_read(0x7FFF) == 0xB0 + bank);
    }
    serial_write(0x8000, 0x1C);
    serial_write(0xC000, 4);
    CHECK(cart_cpu_read(0x6000) == 0xA1);
    serial_write(0x8000, 0x0C); // In 8KB CHR mode only CHR0 supplies RAM bank bits.
    CHECK(cart_cpu_read(0x6000) == 0xA3);
    serial_write(0xE000, 0x10);
    cart_cpu_write(0x6000, 0x55);
    CHECK(cart_cpu_read(0x6000) == 0xFF);
    serial_write(0xE000, 0);
    CHECK(cart_cpu_read(0x6000) == 0xA3);

    h.flags10 = 8; // 16KB volatile PRG RAM.
    h.flags6 &= (uint8_t)~2u;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    cart_cpu_write(0x6000, 1);
    serial_write(0xA000, 4);
    cart_cpu_write(0x6000, 2);
    serial_write(0xA000, 8);
    CHECK(cart_cpu_read(0x6000) == 1);
    serial_write(0xA000, 4);
    CHECK(cart_cpu_read(0x6000) == 2);

    h.flags6 |= 2;
    h.flags10 = 0x77; // SOROM: one 8KB work chip and one 8KB battery chip.
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    cart_cpu_write(0x6000, 0x35);
    serial_write(0xA000, 8);
    cart_cpu_write(0x6000, 0x73);
    serial_write(0xA000, 4); // Bit 2 does not select the second SOROM chip.
    CHECK(cart_cpu_read(0x6000) == 0x35);
    serial_write(0x8000, 0x1C);
    serial_write(0xC000, 8);
    CHECK(cart_cpu_read(0x6000) == 0x73);
    serial_write(0x8000, 0x0C);
    CHECK(cart_cpu_read(0x6000) == 0x35);
    return 0;
}

static void mmc5_unlock_ram(void) {
    cart_cpu_write(0x5102, 2);
    cart_cpu_write(0x5103, 1);
}

static int test_mmc5_banked_ram(void) {
    iNESHeader h = header_for(5, 0x20000, true);
    h.flags7 = 8;
    h.zero[0] = 7;
    const uint8_t sizes[] = {7, 9, 10, 11}; // 8KB, 32KB, 64KB, 128KB chips.
    for (size_t chip = 0; chip < sizeof(sizes); ++chip) {
        for (unsigned persistent = 0; persistent < 2; ++persistent) {
            h.flags10 = (uint8_t)(sizes[chip] << (persistent ? 4 : 0));
            h.flags6 = (uint8_t)(0x50 | (persistent ? 2 : 0));
            CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 5);
            mmc5_unlock_ram();
            unsigned banks = (unsigned)(((size_t)64 << sizes[chip]) / 0x2000);
            for (unsigned bank = 0; bank < banks; ++bank) {
                cart_cpu_write(0x5113, (uint8_t)bank);
                cart_cpu_write(0x6000, (uint8_t)(0x90 + bank));
                cart_cpu_write(0x7FFF, (uint8_t)(0xB0 + bank));
            }
            for (unsigned bank = 0; bank < banks; ++bank) {
                cart_cpu_write(0x5113, (uint8_t)bank);
                CHECK(cart_cpu_read(0x6000) == 0x90 + bank);
                CHECK(cart_cpu_read(0x7FFF) == 0xB0 + bank);
                cart_cpu_write(0x5114, (uint8_t)bank);
                CHECK(cart_cpu_read(0x8000) == 0x90 + bank);
                cart_cpu_write(0x8000, (uint8_t)(0xC0 + bank));
                CHECK(cart_cpu_read(0x6000) == 0xC0 + bank);
            }
            if (banks <= 4) {
                cart_cpu_write(0x5113, 4);
                CHECK(cart_cpu_read(0x6000) == 0xFF); // Unpopulated second socket.
                cart_cpu_write(0x5114, 4);
                cart_cpu_write(0x8000, 0x77);
                CHECK(cart_cpu_read(0x8000) == 0xFF);
                cart_cpu_write(0x5113, 0);
                CHECK(cart_cpu_read(0x6000) == 0xC0);
            }
            if (banks == 1) {
                cart_cpu_write(0x5113, 3);
                CHECK(cart_cpu_read(0x6000) == 0xC0);
            } else {
                cart_cpu_write(0x5100, 1);
                cart_cpu_write(0x5115, 3); // Ignore low bit for a 16KB window.
                CHECK(cart_cpu_read(0x8000) == 0xC2 && cart_cpu_read(0xA000) == 0xC3);
                cart_cpu_write(0xBFFF, 0x65);
                cart_cpu_write(0x5113, 3);
                CHECK(cart_cpu_read(0x7FFF) == 0x65);
                cart_cpu_write(0x5100, 2);
                cart_cpu_write(0x5116, 1);
                CHECK(cart_cpu_read(0xC000) == 0xC1);
                cart_cpu_write(0x5103, 0);
                cart_cpu_write(0xC000, 0x55);
                CHECK(cart_cpu_read(0xC000) == 0xC1);
            }
        }
    }
    h.flags6 = 0x52;
    h.flags10 = 0x77;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 5);
    mmc5_unlock_ram();
    cart_cpu_write(0x6000, 0x12);
    cart_cpu_write(0x5113, 4);
    cart_cpu_write(0x6000, 0x34);
    for (unsigned bank = 0; bank < 8; ++bank) {
        cart_cpu_write(0x5113, (uint8_t)bank);
        CHECK(cart_cpu_read(0x6000) == (bank < 4 ? 0x12 : 0x34));
    }
    h = header_for(5, 0x20000, true);
    h.prg_ram_size = 8; // Legacy header: eight linearly banked 8KB pages.
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 5);
    mmc5_unlock_ram();
    for (unsigned bank = 0; bank < 8; ++bank) {
        cart_cpu_write(0x5113, (uint8_t)bank);
        cart_cpu_write(0x6000, (uint8_t)(bank + 1));
    }
    for (unsigned bank = 0; bank < 8; ++bank) {
        cart_cpu_write(0x5113, (uint8_t)bank);
        CHECK(cart_cpu_read(0x6000) == bank + 1);
    }
    return 0;
}

static int test_loader_ram_layouts(void) {
    iNESHeader h = header_for(1, 0x4000, true);
    h.flags7 = 8;
    h.flags10 = 9;
    h.zero[0] = 7;
    size_t size;
    uint8_t *image = image_for(&h, 0x4000, 0, &size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0);
    serial_write(0xA000, 12);
    cart_cpu_write(0x6000, 0xA7);
    uint8_t *previous_prg = prg_rom;
    uint8_t *previous_chr = chr_rom;
    static const struct { uint8_t prg, chr, rom, flags; } invalid[] = {
        {10, 7, 0, 0}, // More RAM than MMC1 can address.
        {0x99, 7, 0, 2}, // Separate large RAM chips lack board selection.
        {7, 0x77, 0, 2}, // Mixed volatile/nonvolatile CHR chips.
        {7, 0, 0, 0}, // NES 2.0 explicitly declares no CHR memory.
        {0x70, 7, 0, 0} // NVRAM requires the battery flag.
    };
    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        h.flags10 = invalid[i].prg;
        h.zero[0] = invalid[i].chr;
        h.chr_rom_chunks = invalid[i].rom;
        h.flags6 = (uint8_t)(0x10 | invalid[i].flags);
        image = image_for(&h, 0x4000, invalid[i].rom ? 0x2000 : 0, &size);
        CHECK(image != NULL);
        loaded = load_rom_memory(image, size);
        free(image);
        CHECK(loaded == -1 && prg_rom == previous_prg && chr_rom == previous_chr);
        CHECK(cart_cpu_read(0x6000) == 0xA7);
    }

    // CHR RAM declared beside CHR ROM is separate storage. MMC1 has no
    // selector for it in this layout, so the ROM remains mapped and read-only.
    h.flags10 = 7;
    h.zero[0] = 0x0C;
    h.chr_rom_chunks = 1;
    h.flags6 = 0x10;
    image = image_for(&h, 0x4000, 0x2000, &size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, size);
    free(image);
    CHECK(loaded == 0 && cart_ppu_read(0x0123) == 0xA5);
    cart_ppu_write(0x0123, 0x53);
    CHECK(cart_ppu_read(0x0123) == 0xA5);
    return 0;
}

typedef struct {
    char directory[96];
    char rom[128], prg_save[128], chr_save[128], flash_save[128];
} SaveFixture;

static int save_fixture_begin(SaveFixture *paths) {
    for (unsigned attempt = 0; attempt < 1000; ++attempt) {
        snprintf(paths->directory, sizeof(paths->directory), ".mapper-ram-test-%llu-%u",
                 (unsigned long long)time(NULL), attempt);
#ifdef _WIN32
        int result = _mkdir(paths->directory);
#else
        int result = mkdir(paths->directory, 0700);
#endif
        if (result == 0) {
            snprintf(paths->rom, sizeof(paths->rom), "%s/cart.nes", paths->directory);
            snprintf(paths->prg_save, sizeof(paths->prg_save), "%s/cart.sav", paths->directory);
            snprintf(paths->chr_save, sizeof(paths->chr_save), "%s/cart.chr.sav", paths->directory);
            snprintf(paths->flash_save, sizeof(paths->flash_save), "%s/cart.flash.sav", paths->directory);
            return 0;
        }
        if (errno != EEXIST) return -1;
    }
    return -1;
}

static int save_fixture_end(const SaveFixture *paths) {
    cart_battery_shutdown();
    int result = 0;
    if (remove(paths->rom) != 0 && errno != ENOENT) result = 1;
    if (remove(paths->prg_save) != 0 && errno != ENOENT) result = 1;
    if (remove(paths->chr_save) != 0 && errno != ENOENT) result = 1;
    if (remove(paths->flash_save) != 0 && errno != ENOENT) result = 1;
#ifdef _WIN32
    if (_rmdir(paths->directory) != 0) result = 1;
#else
    if (rmdir(paths->directory) != 0) result = 1;
#endif
    return result;
}

static long saved_file_size(const char *path) {
    FILE *fp = fopen(path, "rb");
    if (!fp) return -1;
    long size = fseek(fp, 0, SEEK_END) == 0 ? ftell(fp) : -1;
    fclose(fp);
    return size;
}

static int saved_byte(const char *path, long offset) {
    FILE *fp = fopen(path, "rb");
    if (!fp) return -1;
    int value = fseek(fp, offset, SEEK_SET) == 0 ? fgetc(fp) : -1;
    fclose(fp);
    return value;
}

static int test_extended_ram_layouts_and_ownership(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);

    uint8_t seeded[0x2000];
    memset(seeded, 0x5A, sizeof(seeded));
    FILE *fp = fopen(paths.prg_save, "wb");
    CHECK(fp != NULL);
    CHECK(fwrite(seeded, 1, sizeof(seeded), fp) == sizeof(seeded));
    CHECK(fclose(fp) == 0);
    memset(seeded, 0x6B, sizeof(seeded));
    fp = fopen(paths.chr_save, "wb");
    CHECK(fp != NULL);
    CHECK(fwrite(seeded, 1, sizeof(seeded), fp) == sizeof(seeded));
    CHECK(fclose(fp) == 0);

    // MMC1 accepts independent 16 KiB work RAM and 8 KiB save RAM. With a
    // battery present, non-SOROM layouts select the save chip and leave the
    // work chip unmapped. Declared CHR RAM/NVRAM beside CHR ROM stays separate.
    iNESHeader h = header_for(1, 0x20000, false);
    h.flags7 |= 0x08;
    h.flags6 |= 0x02;
    h.flags10 = 0x78;
    h.zero[0] = 0x77;
    size_t image_size;
    uint8_t *image = image_for(&h, 0x20000, 0x2000, &image_size);
    CHECK(image != NULL);
    fp = fopen(paths.rom, "wb");
    CHECK(fp != NULL);
    CHECK(fwrite(image, 1, image_size, fp) == image_size);
    CHECK(fclose(fp) == 0);
    free(image);
    CHECK(load_rom(paths.rom) == 0);
    CHECK(cart_cpu_read(0x6000) == 0x5A);
    CHECK(cart_ppu_read(0) == 0xA5);
    cart_ppu_write(0, 0x17);
    CHECK(cart_ppu_read(0) == 0xA5);
    serial_write(0xA000, 4);
    cart_cpu_write(0x6000, 0x71);
    serial_write(0xA000, 0);
    CHECK(cart_cpu_read(0x6000) == 0x71);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x2000);
    CHECK(saved_byte(paths.prg_save, 0) == 0x71);
    CHECK(saved_file_size(paths.chr_save) == 0x2000);
    CHECK(saved_byte(paths.chr_save, 0) == 0x6B);
    CHECK(unload_rom());

    // MMC5 copies a trainer into volatile RAM before loading battery data.
    // Bank 0 selects the save socket and bank 4 selects the work socket in
    // the 8 KiB + 8 KiB two-socket layout.
    CHECK(remove(paths.prg_save) == 0);
    CHECK(remove(paths.chr_save) == 0);
    memset(seeded, 0xA6, sizeof(seeded));
    fp = fopen(paths.prg_save, "wb");
    CHECK(fp != NULL);
    CHECK(fwrite(seeded, 1, sizeof(seeded), fp) == sizeof(seeded));
    CHECK(fclose(fp) == 0);
    h = header_for(5, 0x20000, true);
    h.flags7 |= 0x08;
    h.flags6 |= 0x06;
    h.flags10 = 0x77;
    h.zero[0] = 7;
    image = image_for(&h, 0x20000, 0, &image_size);
    CHECK(image != NULL);
    fp = fopen(paths.rom, "wb");
    CHECK(fp != NULL);
    CHECK(fwrite(image, 1, image_size, fp) == image_size);
    CHECK(fclose(fp) == 0);
    free(image);
    CHECK(load_rom(paths.rom) == 0);
    mmc5_unlock_ram();
    cart_cpu_write(0x5113, 0);
    CHECK(cart_cpu_read(0x7123) == 0xA6);
    cart_cpu_write(0x5113, 4);
    CHECK(cart_cpu_read(0x7123) == 0x23);
    cart_cpu_write(0x7123, 0x44);
    cart_cpu_write(0x5113, 0);
    cart_cpu_write(0x7123, 0x55);
    cart_battery_flush();
    CHECK(saved_file_size(paths.prg_save) == 0x2400);
    CHECK(saved_byte(paths.prg_save, 0x1123) == 0x55);
    CHECK(unload_rom());
    CHECK(load_rom(paths.rom) == 0);
    mmc5_unlock_ram();
    cart_cpu_write(0x5113, 0);
    CHECK(cart_cpu_read(0x7123) == 0x55);
    cart_cpu_write(0x5113, 4);
    CHECK(cart_cpu_read(0x7123) == 0x23);
    CHECK(unload_rom());

    // A single 16 KiB MMC5 RAM chip mirrors through all eight low bank
    // selectors. When either physical chip is 64 KiB or larger and a save
    // chip exists, the save chip remains selected for the four-bit bank path.
    h = header_for(5, 0x20000, true);
    h.flags7 |= 0x08;
    h.flags10 = 8;
    h.zero[0] = 7;
    image = image_for(&h, 0x20000, 0, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    mmc5_unlock_ram();
    cart_cpu_write(0x5113, 0);
    cart_cpu_write(0x6000, 0x31);
    cart_cpu_write(0x5113, 1);
    cart_cpu_write(0x6000, 0x42);
    cart_cpu_write(0x5113, 2);
    CHECK(cart_cpu_read(0x6000) == 0x31);
    cart_cpu_write(0x5113, 7);
    CHECK(cart_cpu_read(0x6000) == 0x42);

    h.flags6 |= 0x02;
    h.flags10 = 0x7A; // 64 KiB work RAM plus 8 KiB save RAM.
    image = image_for(&h, 0x20000, 0, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    mmc5_unlock_ram();
    cart_cpu_write(0x5113, 0);
    cart_cpu_write(0x6000, 0x19);
    cart_cpu_write(0x5113, 8);
    CHECK(cart_cpu_read(0x6000) == 0x19);
    cart_cpu_write(0x6000, 0x2A);
    cart_cpu_write(0x5113, 0);
    CHECK(cart_cpu_read(0x6000) == 0x2A);

    unload_rom();
    return save_fixture_end(&paths);
}

static int test_unmapped_chr_storage_ownership(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);

    uint8_t seeded[0x2000];
    memset(seeded, 0x6B, sizeof(seeded));
    FILE *fp = fopen(paths.chr_save, "wb");
    CHECK(fp != NULL);
    CHECK(fwrite(seeded, 1, sizeof(seeded), fp) == sizeof(seeded));
    CHECK(fclose(fp) == 0);

    iNESHeader h = header_for(69, 0x20000, false);
    h.flags7 |= 0x08;
    h.flags6 |= 0x02;
    h.flags10 = 0;
    h.zero[0] = 0x77; // Separate 8 KiB CHR work RAM and 8 KiB CHR NVRAM beside ROM.
    size_t image_size;
    uint8_t *image = image_for(&h, 0x20000, 0x2000, &image_size);
    CHECK(image != NULL);
    FILE *rom = fopen(paths.rom, "wb");
    CHECK(rom != NULL);
    CHECK(fwrite(image, 1, image_size, rom) == image_size);
    CHECK(fclose(rom) == 0);
    free(image);

    CHECK(load_rom(paths.rom) == 0);
    sunsoft69_command(0, 0);
    CHECK(cart_ppu_read(0x0123) == 0xA5);
    cart_ppu_write(0x0123, 0x53);
    CHECK(cart_ppu_read(0x0123) == 0xA5);
    cart_battery_flush();
    CHECK(saved_file_size(paths.chr_save) == 0x2000);
    CHECK(saved_byte(paths.chr_save, 0) == 0x6B);
    CHECK(saved_file_size(paths.prg_save) == -1);

    Mapper *previous = cart;
    iNESHeader invalid = h;
    invalid.flags6 &= (uint8_t)~0x02u;
    image = image_for(&invalid, 0x20000, 0x2000, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == -1);
    free(image);
    CHECK(cart == previous && cart_ppu_read(0x0123) == 0xA5);

    // Storage that exceeds the mapper's CHR-ROM selector range is still a
    // valid independent device when no PPU register can select it.
    h.zero[0] = 0xDD; // 512 KiB volatile plus 512 KiB nonvolatile CHR RAM.
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 69);
    sunsoft69_command(0, 0);
    CHECK(cart_ppu_read(0x0123) == 0);

    unload_rom();
    return save_fixture_end(&paths);
}

static int test_sunsoft184_inherited_ram_reads(void) {
    iNESHeader h = header_for(184, 0x8000, false);
    h.flags7 |= 0x08;
    h.flags10 = 7; // 8 KiB volatile PRG-RAM.
    h.flags6 |= 0x04; // Trainer preloads RAM even though CPU writes are mapper registers.
    h.chr_rom_chunks = 4;
    size_t image_size;
    uint8_t *image = image_for(&h, 0x8000, 0x8000, &image_size);
    CHECK(image != NULL);
    size_t prg_offset = sizeof(h) + 512;
    size_t chr_offset = sizeof(h) + 512 + 0x8000;
    for (unsigned bank = 0; bank < 8; ++bank)
        memset(image + chr_offset + bank * 0x1000, (int)bank, 0x1000);
    const uint8_t ram_program[] = {
        0xAD, 0x23, 0x71,       // LDA $7123: trainer-backed PRG-RAM read.
        0xA9, 0x31,             // LDA #$31
        0x8D, 0x23, 0x71,       // STA $7123: mapper register write, not RAM write.
        0xAD, 0x23, 0x71        // LDA $7123: RAM value must remain unchanged.
    };
    memcpy(image + prg_offset + 0x0100, ram_program, sizeof(ram_program));
    image[prg_offset + 0x7FFC] = 0x00;
    image[prg_offset + 0x7FFD] = 0x81;
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);

    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu) && cpu.pc == 0x8100);
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x23);
    CHECK(cpu_step(&cpu) == 2 && cpu.a == 0x31);
    CHECK(cpu_step(&cpu) == 4);
    CHECK(cart_ppu_read(0x0000) == 1 && cart_ppu_read(0x1000) == 7);
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x23);
    CHECK(read_mem(0x7123) == 0x23);
    write_mem(0x7123, 0x31);
    CHECK(read_mem(0x7123) == 0x23);
    CHECK(cart_ppu_read(0x0000) == 1 && cart_ppu_read(0x1000) == 7);

    h.flags6 &= (uint8_t)~0x04u;
    h.flags10 = 0;
    image = image_for(&h, 0x8000, 0x8000, &image_size);
    CHECK(image != NULL);
    chr_offset = sizeof(h) + 0x8000;
    for (unsigned bank = 0; bank < 8; ++bank)
        memset(image + chr_offset + bank * 0x1000, (int)bank, 0x1000);
    const uint8_t no_ram_program[] = {
        0xA9, 0x26,             // LDA #$26
        0x8D, 0x23, 0x61,       // STA $6123: mapper register still receives writes.
        0xAD, 0x23, 0x61        // LDA $6123: open bus keeps operand high byte $61.
    };
    memcpy(image + sizeof(h) + 0x0100, no_ram_program, sizeof(no_ram_program));
    image[sizeof(h) + 0x7FFC] = 0x00;
    image[sizeof(h) + 0x7FFD] = 0x81;
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu) && cpu.pc == 0x8100);
    CHECK(cpu_step(&cpu) == 2 && cpu.a == 0x26);
    CHECK(cpu_step(&cpu) == 4);
    CHECK(cart_ppu_read(0x0000) == 6 && cart_ppu_read(0x1000) == 6);
    CHECK(cpu_step(&cpu) == 4 && cpu.a == 0x61);
    write_mem(0x4018, 0xA6);
    CHECK(read_mem(0x6123) == 0xA6);

    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    uint8_t save[0x2000];
    for (size_t i = 0; i < sizeof(save); ++i) save[i] = (uint8_t)(i ^ 0xA5u);
    FILE *fp = fopen(paths.prg_save, "wb");
    CHECK(fp != NULL);
    size_t written = fwrite(save, 1, sizeof(save), fp);
    int closed = fclose(fp);
    CHECK(written == sizeof(save) && closed == 0);

    h = header_for(184, 0x8000, false);
    h.flags6 |= 0x02;
    h.chr_rom_chunks = 4;
    image = image_for(&h, 0x8000, 0x8000, &image_size);
    CHECK(image != NULL);
    const uint8_t save_program[] = {
        0xAD, 0x23, 0x61        // LDA $6123: battery-backed inherited RAM read.
    };
    memcpy(image + sizeof(h) + 0x0100, save_program, sizeof(save_program));
    image[sizeof(h) + 0x7FFC] = 0x00;
    image[sizeof(h) + 0x7FFD] = 0x81;
    CHECK(load_rom_memory(image, image_size) == 0);
    cart_battery_configure(paths.rom, true);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu) && cpu.pc == 0x8100);
    CHECK(cpu_step(&cpu) == 4 && cpu.a == save[0x123]);
    CHECK(read_mem(0x6123) == save[0x123]);
    write_mem(0x6123, 0x26);
    CHECK(read_mem(0x6123) == save[0x123]);
    cart_battery_flush();
    CHECK(saved_byte(paths.prg_save, 0x123) == save[0x123]);

    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    cart_battery_configure(paths.rom, true);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    CHECK(read_mem(0x6123) == save[0x123]);
    return save_fixture_end(&paths);
}

static int prg_persistence_cases(const SaveFixture *paths) {
    uint8_t legacy_save[0x2000];
    for (size_t i = 0; i < sizeof(legacy_save); ++i) legacy_save[i] = (uint8_t)(i ^ 0xA5);
    FILE *fp = fopen(paths->prg_save, "wb");
    CHECK(fp != NULL);
    size_t written = fwrite(legacy_save, 1, sizeof(legacy_save), fp);
    int closed = fclose(fp);
    CHECK(written == sizeof(legacy_save) && closed == 0);
    iNESHeader h = header_for(1, 0x20000, true);
    h.flags7 |= 8;
    h.prg_ram_size = 0;
    h.flags6 |= 2;
    h.flags10 = 0x90;
    h.zero[0] = 7;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    cart_battery_configure(paths->rom, true);
    CHECK(cart_cpu_read(0x6000) == 0xA5 && cart_cpu_read(0x7FFF) == 0x5A);
    serial_write(0xA000, 12);
    CHECK(cart_cpu_read(0x6000) == 0 && cart_cpu_read(0x7FFF) == 0);
    cart_cpu_write(0x6000, 0x73);
    cart_cpu_write(0x7FFF, 0x39);
    cart_ppu_write(0, 0x22); // CHR-RAM is volatile.
    cart_battery_flush();
    CHECK(saved_file_size(paths->prg_save) == 0x8000);
    CHECK(saved_file_size(paths->chr_save) == -1);
    uint8_t restored_prefix[sizeof(legacy_save)];
    fp = fopen(paths->prg_save, "rb");
    CHECK(fp != NULL);
    size_t bytes_read = fread(restored_prefix, 1, sizeof(restored_prefix), fp);
    closed = fclose(fp);
    CHECK(bytes_read == sizeof(restored_prefix) && closed == 0);
    CHECK(memcmp(restored_prefix, legacy_save, sizeof(legacy_save)) == 0);
    CHECK(saved_byte(paths->prg_save, 0x6000) == 0x73 && saved_byte(paths->prg_save, 0x7FFF) == 0x39);
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    cart_battery_configure(paths->rom, true);
    serial_write(0xA000, 12);
    CHECK(cart_cpu_read(0x6000) == 0x73 && cart_cpu_read(0x7FFF) == 0x39);

    h.flags10 = 0x77;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    cart_battery_configure(paths->rom, true);
    cart_cpu_write(0x6000, 0x66);
    serial_write(0xA000, 8);
    cart_cpu_write(0x6000, 0x99);
    cart_battery_flush();
    CHECK(saved_file_size(paths->prg_save) == 0x2000);
    CHECK(saved_byte(paths->prg_save, 0) == 0x66);
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 1);
    cart_battery_configure(paths->rom, true);
    CHECK(cart_cpu_read(0x6000) == 0x66);
    serial_write(0xA000, 8);
    CHECK(cart_cpu_read(0x6000) == 0); // The work chip was not serialized.
    return 0;
}

static int test_prg_nvram_persistence(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    int result = prg_persistence_cases(&paths);
    return result | save_fixture_end(&paths);
}

static int test_mapper96_legacy_nvram_persistence(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);

    iNESHeader h = header_for(96, 0x20000, true);
    h.flags6 |= 0x02;
    size_t image_size;
    uint8_t *image = image_for(&h, 0x20000, 0, &image_size);
    CHECK(image != NULL);
    FILE *fp = fopen(paths.rom, "wb");
    CHECK(fp != NULL);
    size_t written = fwrite(image, 1, image_size, fp);
    int closed = fclose(fp);
    free(image);
    CHECK(written == image_size && closed == 0);

    CHECK(load_rom(paths.rom) == 0 && rom_mapper_number(&ines_header) == 96);
    write_mem(0x6000, 0x35);
    write_mem(0x7FFF, 0xA6);
    CHECK(read_mem(0x6000) == 0x35 && read_mem(0x7FFF) == 0xA6);
    CHECK(unload_rom());
    CHECK(saved_file_size(paths.prg_save) == 0x2000);
    CHECK(saved_byte(paths.prg_save, 0) == 0x35);
    CHECK(saved_byte(paths.prg_save, 0x1FFF) == 0xA6);

    CHECK(load_rom(paths.rom) == 0);
    CHECK(read_mem(0x6000) == 0x35 && read_mem(0x7FFF) == 0xA6);

    CHECK(unload_rom());
    return save_fixture_end(&paths);
}

static int vs_nvram_persistence_cases(const SaveFixture *paths) {
    iNESHeader h = header_for(99, 0x8000, false);
    h.flags7 |= 0x09;
    h.flags6 |= 2;
    h.prg_ram_size = 0;
    h.flags10 = 0x50; // Explicit 2 KiB battery-backed RAM.
    size_t image_size;
    uint8_t *image = image_for(&h, 0x8000, 0x2000, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    cart_battery_configure(paths->rom, true);
    cart_cpu_write(0x6000, 0x62);
    cart_cpu_write(0x67FF, 0xC3);
    cart_battery_flush();
    CHECK(saved_file_size(paths->prg_save) == 0x800);
    CHECK(saved_byte(paths->prg_save, 0) == 0x62 && saved_byte(paths->prg_save, 0x7FF) == 0xC3);
    CHECK(unload_rom());
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    cart_battery_configure(paths->rom, true);
    CHECK(cart_cpu_read(0x6000) == 0x62 && cart_cpu_read(0x67FF) == 0xC3);
    return 0;
}

static int test_vs_nvram_persistence(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    int result = vs_nvram_persistence_cases(&paths);
    unload_rom();
    return result | save_fixture_end(&paths);
}

static int test_unrom512_flash_persistence(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    iNESHeader h = unrom512_header(1, true, 9);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    cart_battery_configure(paths.rom, true);

    // Latching banks alone must not create or dirty a flash save.
    cart_cpu_write(0xC000, 0x07);
    cart_cpu_write(0xC000, 0x02);
    cart_battery_flush();
    CHECK(saved_file_size(paths.flash_save) == -1);

    // A failed replacement keeps the flash dirty so a later flush can retry.
#ifdef _WIN32
    CHECK(_mkdir(paths.flash_save) == 0);
#else
    CHECK(mkdir(paths.flash_save, 0700) == 0);
#endif

    unrom512_flash_command(0xA0);
    cart_cpu_write(0xC000, 0x03);
    cart_cpu_write(0x8123, 0x04);
    CHECK(cart_cpu_read(0x8123) == 0x04);
    cart_battery_flush();
#ifdef _WIN32
    CHECK(_rmdir(paths.flash_save) == 0);
#else
    CHECK(rmdir(paths.flash_save) == 0);
#endif
    cart_battery_flush();
    CHECK(saved_file_size(paths.flash_save) == 0x40000);
    CHECK(saved_byte(paths.flash_save, 3 * 0x4000 + 0x123) == 0x04);
    CHECK(saved_file_size(paths.prg_save) == -1);

    // Reinitialization restores pristine fixture bytes before the flash save is loaded.
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    CHECK(cart_cpu_read(0x8123) == 0);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0xC000, 0x03);
    CHECK(cart_cpu_read(0x8123) == 0x04);

    // An erased sector is also persisted and restored.
    unrom512_flash_erase_prefix();
    cart_cpu_write(0xC000, 0x03);
    cart_cpu_write(0x8123, 0x30);
    CHECK(cart_cpu_read(0x8123) == 0xFF);
    cart_battery_flush();
    CHECK(saved_byte(paths.flash_save, 3 * 0x4000 + 0x123) == 0xFF);
    CHECK(fixture_with_header(&h, 0x40000, 0x8000) == 30);
    cart_battery_configure(paths.rom, true);
    cart_cpu_write(0xC000, 0x03);
    CHECK(cart_cpu_read(0x8123) == 0xFF);

    return save_fixture_end(&paths);
}

static int test_mapper111_flash_persistence(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    iNESHeader h = m111_header();
    CHECK(fixture_with_header(&h, 0x80000, 0x4000) == 111);

    cart_battery_configure(paths.rom, false);
    cart_cpu_write(0x5000, 0x03);
    m111_flash_command(0xA0);
    cart_cpu_write(0x8123, 0x00);
    cart_battery_flush();
    CHECK(saved_file_size(paths.flash_save) == 0x80000);
    CHECK(saved_byte(paths.flash_save, 3 * 0x8000 + 0x123) == 0x00);

    CHECK(fixture_with_header(&h, 0x80000, 0x4000) == 111);
    cart_battery_configure(paths.rom, false);
    cart_cpu_write(0x5000, 0x03);
    CHECK(cart_cpu_read(0x8123) == 0x00);

    cart_battery_shutdown();
    CHECK(remove(paths.flash_save) == 0);
#ifdef _WIN32
    CHECK(_mkdir(paths.flash_save) == 0);
#else
    CHECK(mkdir(paths.flash_save, 0700) == 0);
#endif
    cart_battery_configure(paths.rom, false);
    cart_cpu_write(0x5000, 0x04);
    m111_flash_command(0xA0);
    cart_cpu_write(0x8456, 0x00);
    cart_battery_flush();
#ifdef _WIN32
    CHECK(_rmdir(paths.flash_save) == 0);
#else
    CHECK(rmdir(paths.flash_save) == 0);
#endif
    cart_battery_flush();
    CHECK(saved_file_size(paths.flash_save) == 0x80000);
    CHECK(saved_byte(paths.flash_save, 4 * 0x8000 + 0x456) == 0x00);

    return save_fixture_end(&paths);
}

static int mmc5_persistence_cases(const SaveFixture *paths) {
    iNESHeader h = header_for(5, 0x20000, true);
    h.flags6 |= 2;
    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 5);
    cart_battery_configure(paths->rom, true);
    cart_cpu_write(0x5102, 2);
    cart_cpu_write(0x5103, 1);
    cart_cpu_write(0x5113, 0);
    cart_cpu_write(0x6000, 0x35);
    cart_cpu_write(0x5104, 2);
    cart_cpu_write(0x5C00, 0x53);
    cart_cpu_write(0x5FFF, 0xA7);
    cart_battery_flush();
    CHECK(saved_file_size(paths->prg_save) == 0x10400);
    CHECK(saved_byte(paths->prg_save, 0) == 0x35);
    CHECK(saved_byte(paths->prg_save, 0x10000) == 0x53);
    CHECK(saved_byte(paths->prg_save, 0x103FF) == 0xA7);

    CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 5);
    cart_battery_configure(paths->rom, true);
    cart_cpu_write(0x5113, 0);
    cart_cpu_write(0x5104, 2);
    CHECK(cart_cpu_read(0x6000) == 0x35);
    CHECK(cart_cpu_read(0x5C00) == 0x53 && cart_cpu_read(0x5FFF) == 0xA7);

    // Old saves may end in PRG RAM or partway through the appended ExRAM.
    const size_t short_sizes[] = {3, 0x10002};
    for (unsigned i = 0; i < sizeof(short_sizes) / sizeof(short_sizes[0]); ++i) {
        cart_battery_shutdown();
        uint8_t *contents = (uint8_t *)calloc(1, short_sizes[i]);
        CHECK(contents != NULL);
        contents[0] = 0x42;
        contents[short_sizes[i] - 1] = 0xA3;
        FILE *fp = fopen(paths->prg_save, "wb");
        if (!fp) { free(contents); return 1; }
        size_t written = fwrite(contents, 1, short_sizes[i], fp);
        int closed = fclose(fp);
        free(contents);
        CHECK(written == short_sizes[i] && closed == 0);
        CHECK(fixture_with_header(&h, 0x20000, 0x2000) == 5);
        cart_battery_configure(paths->rom, true);
        cart_cpu_write(0x5113, 0);
        cart_cpu_write(0x5104, 2);
        CHECK(cart_cpu_read(0x6000) == 0x42 && cart_cpu_read(0x7FFF) == 0);
        CHECK(cart_cpu_read(0x6002) == (i ? 0 : 0xA3));
        CHECK(cart_cpu_read(0x5C01) == (i ? 0xA3 : 0));
        CHECK(cart_cpu_read(0x5C02) == 0 && cart_cpu_read(0x5FFF) == 0);
    }
    return 0;
}

static int test_mmc5_persistence(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    int result = mmc5_persistence_cases(&paths);
    return result | save_fixture_end(&paths);
}

static int chr_persistence_cases(const SaveFixture *paths) {
    iNESHeader h = header_for(1, 0x20000, true);
    h.flags7 = 8;
    h.flags6 |= 2;
    h.flags10 = 7;
    h.zero[0] = 0x90; // 32KB CHR-NVRAM, plus a separate volatile PRG chip.
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 1);
    cart_battery_configure(paths->rom, true);
    serial_write(0x8000, 0x1C);
    for (unsigned bank = 0; bank < 8; ++bank) {
        serial_write(0xA000, (uint8_t)bank);
        cart_ppu_write(0, (uint8_t)(0x30 + bank));
        cart_ppu_write(0x0FFF, (uint8_t)(0x50 + bank));
    }
    cart_cpu_write(0x6000, 0xA6);
    cart_battery_flush();
    CHECK(saved_file_size(paths->prg_save) == -1);
    CHECK(saved_file_size(paths->chr_save) == 0x8000);
    CHECK(saved_byte(paths->chr_save, 0x7000) == 0x37 && saved_byte(paths->chr_save, 0x7FFF) == 0x57);
    CHECK(fixture_with_header(&h, 0x20000, 0x8000) == 1);
    cart_battery_configure(paths->rom, true);
    CHECK(cart_cpu_read(0x6000) == 0);
    serial_write(0x8000, 0x1C);
    for (unsigned bank = 0; bank < 8; ++bank) {
        serial_write(0xA000, (uint8_t)bank);
        CHECK(cart_ppu_read(0) == 0x30 + bank && cart_ppu_read(0x0FFF) == 0x50 + bank);
    }

    // Replacing a loaded ROM must flush both old chips before freeing their storage.
    h.flags10 = 0x70;
    h.zero[0] = 0x70;
    size_t image_size;
    uint8_t *image = image_for(&h, 0x20000, 0, &image_size);
    CHECK(image != NULL);
    int loaded = load_rom_memory(image, image_size);
    free(image);
    CHECK(loaded == 0);
    cart_battery_configure(paths->rom, true);
    cart_cpu_write(0x6000, 0x8D);
    cart_ppu_write(0, 0xD8);
    h = header_for(0, 0x4000, true);
    image = image_for(&h, 0x4000, 0, &image_size);
    CHECK(image != NULL);
    loaded = load_rom_memory(image, image_size);
    free(image);
    CHECK(loaded == 0 && cart_cpu_read(0x6000) == 0 && cart_ppu_read(0) == 0);
    CHECK(saved_file_size(paths->prg_save) == 0x2000);
    CHECK(saved_file_size(paths->chr_save) == 0x2000);
    CHECK(saved_byte(paths->prg_save, 0) == 0x8D && saved_byte(paths->chr_save, 0) == 0xD8);
    return 0;
}

static int test_chr_nvram_persistence(void) {
    SaveFixture paths;
    CHECK(save_fixture_begin(&paths) == 0);
    int result = chr_persistence_cases(&paths);
    return result | save_fixture_end(&paths);
}

static int chr_writer_cases(const SaveFixture *paths) {
    const unsigned boards[] = {0, 1, 2, 3, 4, 5, 7, 9, 10, 11, 13, 15};
    for (size_t i = 0; i < sizeof(boards) / sizeof(boards[0]); ++i) {
        iNESHeader h = header_for(boards[i], 0x8000, true);
        h.flags7 |= 8;
        h.flags6 |= 2;
        h.zero[0] = boards[i] == 13 ? 0x80 : 0x70;
        size_t bytes = boards[i] == 13 ? 0x4000 : 0x2000;
        CHECK(fixture_with_header(&h, 0x8000, bytes) == (int)boards[i]);
        cart_battery_configure(paths->rom, true);
        if (boards[i] == 15) cart_cpu_write(0x8002, 0);
        cart_ppu_write(7, 0xA6);
        CHECK(cart_ppu_read(7) == 0xA6);
        cart_battery_flush();
        CHECK(saved_file_size(paths->chr_save) == (long)bytes);
        CHECK(saved_byte(paths->chr_save, 7) == 0xA6);
        CHECK(saved_file_size(paths->prg_save) == -1);
        cart_battery_shutdown();
        CHECK(remove(paths->chr_save) == 0);
    }
    return 0;
}

