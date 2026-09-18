/*
 * unif_accuracy.c - Named cartridge boards and UNIF metadata regressions
 *
 * Copyright (C) 2026 Francis Hagan
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#include "board_tests.h"
#include "../rom/game_db.h"
#include "../rom/unif.h"
#include "../system/hardware.h"
#include "../system/vs_system.h"

static bool append_chunk(BoardImage *image, const char id[4], const void *bytes, size_t size) {
    if (size > UINT32_MAX || image->size > SIZE_MAX - 8 - size) return false;
    uint8_t *grown = (uint8_t *)realloc(image->data, image->size + 8 + size);
    if (!grown) return false;
    image->data = grown;
    uint8_t *chunk = grown + image->size;
    memcpy(chunk, id, 4);
    for (unsigned i = 0; i < 4; ++i) chunk[4 + i] = (uint8_t)(size >> (i * 8));
    if (size) memcpy(chunk + 8, bytes, size);
    image->size += 8 + size;
    return true;
}

static void fill_payload(uint8_t *bytes, size_t prg_bytes, size_t chr_bytes) {
    for (size_t i = 0; i < prg_bytes; ++i)
        bytes[i] = (i & 0x1FFF) == 1 ? (uint8_t)(i >> 21) : (uint8_t)(i >> 13);
    for (size_t i = 0; i < chr_bytes; ++i)
        bytes[prg_bytes + i] = (i & 0x3FF) == 1 ? (uint8_t)(i >> 18) : (uint8_t)(i >> 10);
}

static bool make_unif(BoardImage *image, const char *name, size_t prg_bytes,
                       size_t chr_bytes, uint32_t *crc) {
    *image = (BoardImage){0};
    if (!prg_bytes || chr_bytes > SIZE_MAX - prg_bytes) return false;
    uint8_t *payload = (uint8_t *)malloc(prg_bytes + chr_bytes);
    image->data = (uint8_t *)calloc(1, 32);
    if (!payload || !image->data) {
        free(payload);
        board_image_free(image);
        return false;
    }
    image->size = 32;
    memcpy(image->data, "UNIF", 4);
    image->data[4] = 7;
    fill_payload(payload, prg_bytes, chr_bytes);
    if (crc) *crc = game_db_crc32(payload, prg_bytes + chr_bytes);
    bool ok = append_chunk(image, "CHR0", payload + prg_bytes, chr_bytes)
           && append_chunk(image, "MAPR", name, strlen(name) + 1)
           && append_chunk(image, "PRG0", payload, prg_bytes);
    free(payload);
    if (!ok) board_image_free(image);
    return ok;
}

static bool write_absolute(uint16_t address, uint8_t value) {
    write_mem(0x0200, 0xA9);
    write_mem(0x0201, value);
    write_mem(0x0202, 0x8D);
    write_mem(0x0203, (uint8_t)address);
    write_mem(0x0204, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4;
}

static bool read_absolute(uint16_t address) {
    write_mem(0x0200, 0xAD);
    write_mem(0x0201, (uint8_t)address);
    write_mem(0x0202, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 4;
}

static bool read_absolute_is(uint16_t address, uint8_t value) {
    return read_absolute(address) && cpu.a == value;
}

static bool prg_bank_is(uint16_t address, unsigned bank) {
    return read_mem(address) == (uint8_t)bank
        && read_mem((uint16_t)(address + 1)) == (uint8_t)(bank >> 8);
}

static bool chr_bank_is(uint16_t address, unsigned bank) {
    return ppu_read(address) == (uint8_t)bank
        && ppu_read((uint16_t)(address + 1)) == (uint8_t)(bank >> 8);
}

static bool a12_edge(unsigned low_cycles) {
    cart_notify_ppu_address(0x1000, cpu_get_bus_cycle() * 3);
    cart_notify_ppu_address(0x0000, cpu_get_bus_cycle() * 3 + 1);
    write_mem(0x0200, low_cycles == 3 ? 0x4C : 0xEA);
    write_mem(0x0201, 0x00);
    write_mem(0x0202, 0x02);
    cpu.pc = 0x0200;
    if (cpu_step(&cpu) != (int)low_cycles) return false;
    cart_notify_ppu_address(0x1000, cpu_get_bus_cycle() * 3);
    return true;
}

static int test_discrete_named_boards(void) {
    BoardImage image;
    BOARD_CHECK(make_unif(&image, "UNL-GS-2013", 0x80000, 0x2000, NULL));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg_bank_is(0x6000, 31) && prg_bank_is(0x8000, 60)
                && prg_bank_is(0xE000, 63));
    BOARD_CHECK(write_absolute(0x8000, 2));
    BOARD_CHECK(prg_bank_is(0x8000, 8) && prg_bank_is(0xE000, 11));
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg_bank_is(0x6000, 31) && prg_bank_is(0x8000, 60));
    board_image_free(&image);

    BOARD_CHECK(make_unif(&image, "CC-21", 0x8000, 0x2000, NULL));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(chr_bank_is(0x0000, 0) && chr_bank_is(0x1000, 0));
    BOARD_CHECK(write_absolute(0x8000, 1));
    BOARD_CHECK(chr_bank_is(0x0000, 4) && chr_bank_is(0x1000, 4));
    BOARD_CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    ppu_write(0x2000, 0xA6);
    BOARD_CHECK(ppu_read(0x2400) == 0xA6 && ppu_read(0x2C00) == 0xA6);
    BOARD_CHECK(write_absolute(0x8002, 1));
    BOARD_CHECK(chr_bank_is(0x0000, 0) && cart_get_mirroring() == MIRROR_SINGLE0);
    BOARD_CHECK(write_absolute(0x8003, 0));
    BOARD_CHECK(chr_bank_is(0x1000, 4));
    board_image_free(&image);
    BOARD_CHECK(make_unif(&image, "CC-21", 0x8000, 0x4000, NULL));
    BOARD_CHECK(board_image_load(&image) == 0 && write_absolute(0x8000, 1));
    BOARD_CHECK(chr_bank_is(0x0000, 8) && chr_bank_is(0x1000, 12));
    board_image_free(&image);

    BOARD_CHECK(make_unif(&image, "AC08", 0x20000, 0x2000, NULL));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg_bank_is(0x6000, 0) && prg_bank_is(0x8000, 12)
                && prg_bank_is(0xE000, 15));
    BOARD_CHECK(write_absolute(0x8001, 10) && prg_bank_is(0x6000, 5));
    BOARD_CHECK(write_absolute(0x8000, 9) && prg_bank_is(0x6000, 9));
    BOARD_CHECK(write_absolute(0x4025, 8) && cart_get_mirroring() == MIRROR_HORIZONTAL);
    BOARD_CHECK(write_absolute(0x4025, 0) && cart_get_mirroring() == MIRROR_VERTICAL);
    board_image_free(&image);

    BOARD_CHECK(make_unif(&image, "PUZZLE", 0x10000, 0x10000, NULL));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(write_absolute(0x4100, 13));
    BOARD_CHECK(prg_bank_is(0x8000, 4) && prg_bank_is(0xE000, 7));
    BOARD_CHECK(chr_bank_is(0x0000, 40));
    ppu_write(0x0000, 0xEE);
    BOARD_CHECK(chr_bank_is(0x0000, 40));
    BOARD_CHECK(write_absolute(0x409F, 0) && prg_bank_is(0x8000, 4));
    board_image_free(&image);

    BOARD_CHECK(make_unif(&image, "255in1", 0x20000, 0x10000, NULL));
    BOARD_CHECK(board_image_load(&image) == 0 && write_absolute(0x800D, 0));
    BOARD_CHECK(prg_bank_is(0x8000, 12) && prg_bank_is(0xE000, 15));
    BOARD_CHECK(chr_bank_is(0x0000, 40));
    board_image_free(&image);
    return 0;
}

static int test_ghostbusters_open_bus(void) {
    BoardImage image;
    BOARD_CHECK(make_unif(&image, "BMC-Ghostbusters63in1", 0x200000, 0, NULL));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg_bank_is(0x8000, 0) && prg_bank_is(0xC000, 2));
    ppu_write(0x0000, 0xA7);
    BOARD_CHECK(ppu_read(0x0000) == 0xA7);
    BOARD_CHECK(write_absolute(0x8000, 0x25));
    BOARD_CHECK(prg_bank_is(0x8000, 10) && prg_bank_is(0xC000, 10));
    BOARD_CHECK(write_absolute(0x8000, 0x80));
    write_mem(0x0040, 0xA6);
    BOARD_CHECK(read_mem(0x8000) == 0xA6 && read_mem(0xFFFF) == 0xA6);
    BOARD_CHECK(write_absolute(0x8001, 1));
    BOARD_CHECK(prg_bank_is(0x8000, 128) && prg_bank_is(0xC000, 130));
    BOARD_CHECK(write_absolute(0x8000, 0x40));
    BOARD_CHECK(prg_bank_is(0x8000, 64) && cart_get_mirroring() == MIRROR_VERTICAL);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(prg_bank_is(0x8000, 0) && cart_get_mirroring() == MIRROR_HORIZONTAL);
    BOARD_CHECK(ppu_read(0x0000) == 0xA7);
    board_image_free(&image);
    return 0;
}

static int test_8237a_banks_and_irq(void) {
    BoardImage image;
    BOARD_CHECK(make_unif(&image, "UNL-8237A", 0x200000, 0x200000, NULL));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(prg_bank_is(0x8000, 96) && prg_bank_is(0xE000, 127));
    BOARD_CHECK(chr_bank_is(0x0000, 256));
    BOARD_CHECK(write_absolute(0x5001, 8));
    BOARD_CHECK(prg_bank_is(0x8000, 128) && chr_bank_is(0x0000, 1024));
    BOARD_CHECK(write_absolute(0x8000, 6) && write_absolute(0x8001, 5));
    BOARD_CHECK(prg_bank_is(0x8000, 133));
    BOARD_CHECK(write_absolute(0x5000, 0xA1));
    BOARD_CHECK(prg_bank_is(0x8000, 128) && prg_bank_is(0xA000, 129)
                && prg_bank_is(0xC000, 130) && prg_bank_is(0xE000, 131));
    BOARD_CHECK(write_absolute(0x5000, 0x81));
    BOARD_CHECK(prg_bank_is(0x8000, 130) && prg_bank_is(0xC000, 130));
    BOARD_CHECK(write_absolute(0x5000, 0x40) && write_absolute(0x5001, 0x32));
    BOARD_CHECK(prg_bank_is(0x8000, 85) && chr_bank_is(0x0000, 384));

    BOARD_CHECK(write_absolute(0x5000, 0) && write_absolute(0x5001, 0));
    BOARD_CHECK(write_absolute(0x5007, 1));
    BOARD_CHECK(write_absolute(0xA000, 6) && write_absolute(0xC000, 9));
    BOARD_CHECK(chr_bank_is(0x1800, 9));
    BOARD_CHECK(write_absolute(0x8001, 1) && cart_get_mirroring() == MIRROR_HORIZONTAL);
    BOARD_CHECK(write_absolute(0xA001, 1) && write_absolute(0xC001, 0));
    BOARD_CHECK(write_absolute(0xE001, 0));
    BOARD_CHECK(a12_edge(2) && !cart_irq_pending());
    BOARD_CHECK(a12_edge(3) && !cart_irq_pending());
    BOARD_CHECK(a12_edge(3) && cart_irq_pending());
    BOARD_CHECK(write_absolute(0xE000, 0) && !cart_irq_pending());
    uint8_t *previous_prg = prg_rom;
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
    BOARD_CHECK(prg_rom == previous_prg && chr_bank_is(0x1800, 9));
    board_image_free(&image);
    return 0;
}

static int test_famicombox_cpu_ram(void) {
    BoardImage image;
    NesRamPowerOnState previous_power = nes_ram_power_on_state();
    BOARD_CHECK(nes_set_ram_power_on_state(NES_RAM_POWER_ZERO));
    BOARD_CHECK(make_unif(&image, "SSS-NROM-256", 0x8000, 0x2000, NULL));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cart_cpu_ram_8k() != NULL);
    static const uint16_t addresses[] = {0x0033, 0x0833, 0x1033, 0x1833};
    static const uint8_t values[] = {0x40, 0x61, 0x82, 0xA3};
    for (unsigned i = 0; i < 4; ++i) BOARD_CHECK(write_absolute(addresses[i], values[i]));
    for (unsigned i = 0; i < 4; ++i) {
        BOARD_CHECK(read_absolute_is(addresses[i], values[i]));
        BOARD_CHECK(cpu_peek_internal_ram(addresses[i]) == values[i]);
    }
    BOARD_CHECK(read_mem(0x5000) == 0xFF && read_mem(0x5007) == 0x22);
    cpu_soft_reset(&cpu);
    for (unsigned i = 0; i < 4; ++i) BOARD_CHECK(read_mem(addresses[i]) == values[i]);
    BOARD_CHECK(cpu_power_on(&cpu));
    for (unsigned i = 0; i < 4; ++i) BOARD_CHECK(read_mem(addresses[i]) == 0);
    board_image_free(&image);
    BOARD_CHECK(make_unif(&image, "NROM", 0x8000, 0x2000, NULL));
    BOARD_CHECK(board_image_load(&image) == 0 && cart_cpu_ram_8k() == NULL);
    write_mem(0x0033, 0x57);
    BOARD_CHECK(read_mem(0x0833) == 0x57 && read_mem(0x1833) == 0x57);
    board_image_free(&image);
    BOARD_CHECK(nes_set_ram_power_on_state(previous_power));
    return 0;
}

static int test_database_resolution_before_allocation(void) {
    BoardImage image;
    uint32_t crc = 0;
    BOARD_CHECK(make_unif(&image, "UNKNOWN-CPROM", 0x8000, 0, &crc));
    const uint8_t single = 2;
    BOARD_CHECK(append_chunk(&image, "MIRR", &single, 1));
    char row[256];
    int length = snprintf(row, sizeof(row),
        "%08X,NesPal,CPROM,,,13,0,0,16,0,0,0,,1,N,,0,0\n", (unsigned)crc);
    BOARD_CHECK(length > 0 && (size_t)length < sizeof(row));
    BOARD_CHECK(rom_database_load_memory(row, (size_t)length));
    rom_database_set_overrides(true);
    nes_set_region(NES_REGION_PAL);
    BOARD_CHECK(cpu_set_startup_alignment(14, 4));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(rom_metadata_source() == ROM_METADATA_DATABASE);
    BOARD_CHECK(nes_timing()->region == NES_REGION_PAL && chr_size == 0x4000);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_VERTICAL);
    BOARD_CHECK(write_absolute(0x8000, 3));
    ppu_write(0x1000, 0xA9);
    BOARD_CHECK(ppu_read(0x1000) == 0xA9 && ppu_read(0x0000) == 0);
    BOARD_CHECK(write_absolute(0x8000, 0) && ppu_read(0x1000) == 0);
    BOARD_CHECK(write_absolute(0x8000, 3) && ppu_read(0x1000) == 0xA9);
    uint8_t *previous_prg = prg_rom;
    rom_database_set_overrides(false);
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0 && prg_rom == previous_prg);
    rom_database_set_overrides(true);
    length = snprintf(row, sizeof(row),
        "%08X,NesNtsc,CPROM,,,13,32,0,16,0,0,0,,1,N,,0,0\n", (unsigned)crc);
    BOARD_CHECK(length > 0 && (size_t)length < sizeof(row));
    BOARD_CHECK(rom_database_load_memory(row, (size_t)length));
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0);
    BOARD_CHECK(prg_rom == previous_prg && nes_timing()->region == NES_REGION_PAL);
    BOARD_CHECK(ppu_read(0x1000) == 0xA9);
    cpu_use_default_startup_alignment();
    length = snprintf(row, sizeof(row),
        "%08X,NesNtsc,NROM,,,0,32,0,8,0,0,0,,1,N,,0,0\n", (unsigned)crc);
    BOARD_CHECK(length > 0 && (size_t)length < sizeof(row));
    BOARD_CHECK(rom_database_load_memory(row, (size_t)length));
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_SINGLE0);
    rom_database_clear();
    board_image_free(&image);
    return 0;
}

static int test_database_input_and_vs(void) {
    BoardImage image;
    uint32_t crc = 0;
    NesInputConfiguration saved = {joypad_adapter(),
        {joypad_port_device(0), joypad_port_device(1)}, joypad_expansion_device()};
    uint8_t saved_overrides = joypad_configuration_overrides();
    joypad_set_configuration_overrides(0);
    BOARD_CHECK(make_unif(&image, "NROM", 0x8000, 0x2000, &crc));
    char row[256];
    int length = snprintf(row, sizeof(row),
        "%08X,NesNtsc,NROM,,,0,0,0,0,0,0,0,h,43,N,,0,0\n", (unsigned)crc);
    BOARD_CHECK(length > 0 && (size_t)length < sizeof(row));
    BOARD_CHECK(rom_database_load_memory(row, (size_t)length));
    BOARD_CHECK(board_image_load(&image) == 0 && !vs_enabled());
    BOARD_CHECK(prg_size == 0x8000 && chr_size == 0x2000 && chr_bank_is(0x1000, 4));
    ppu_write(0x1000, 0xEE);
    BOARD_CHECK(chr_bank_is(0x1000, 4));
    BOARD_CHECK(joypad_port_device(0) == NES_PORT_SNES_CONTROLLER
                && joypad_port_device(1) == NES_PORT_SNES_CONTROLLER);
    BOARD_CHECK(joypad_set_snes_button(0, SNES_BUTTON_A, true));
    BOARD_CHECK(write_absolute(0x4016, 1) && write_absolute(0x4016, 0));
    for (unsigned i = 0; i < 8; ++i) BOARD_CHECK(read_absolute(0x4016));
    BOARD_CHECK(read_absolute(0x4016) && (cpu.a & 1) == 1);
    BOARD_CHECK(joypad_set_snes_button(0, SNES_BUTTON_A, false));

    length = snprintf(row, sizeof(row),
        "%08X,VsSystem,NROM,,,0,32,8,0,0,0,0,h,1,N,,0,2\n", (unsigned)crc);
    BOARD_CHECK(length > 0 && (size_t)length < sizeof(row));
    BOARD_CHECK(rom_database_load_memory(row, (size_t)length));
    BOARD_CHECK(board_image_load(&image) == 0 && vs_enabled());
    BOARD_CHECK(vs_ppu_model() == VS_PPU_2C04_0001);
    uint8_t *previous_prg = prg_rom;
    length = snprintf(row, sizeof(row),
        "%08X,VsSystem,NROM,,,0,32,8,0,0,0,0,h,18,N,,0,2\n", (unsigned)crc);
    BOARD_CHECK(length > 0 && (size_t)length < sizeof(row));
    BOARD_CHECK(rom_database_load_memory(row, (size_t)length));
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0 && prg_rom == previous_prg);
    BOARD_CHECK(vs_enabled() && vs_ppu_model() == VS_PPU_2C04_0001);
    BOARD_CHECK(unload_rom());
    rom_database_clear();
    BOARD_CHECK(joypad_apply_configuration(&saved));
    joypad_set_configuration_overrides(saved_overrides);
    board_image_free(&image);
    return 0;
}

static int test_named_database_and_file_loader(void) {
    const size_t prg_bytes = 0x8800, chr_bytes = 0x2000;
    uint8_t *raw = (uint8_t *)malloc(prg_bytes + chr_bytes);
    BOARD_CHECK(raw != NULL);
    fill_payload(raw, prg_bytes, chr_bytes);
    char row[256];
    int length = snprintf(row, sizeof(row),
        "%08X,NesNtsc,UNL-MARIO1-MALEE2,,,65000,34,8,0,0,0,0,h,1,N,,0,0\n",
        (unsigned)game_db_crc32(raw, prg_bytes + chr_bytes));
    BOARD_CHECK(length > 0 && (size_t)length < sizeof(row));
    BOARD_CHECK(rom_database_load_memory(row, (size_t)length));
    BOARD_CHECK(load_rom_memory(raw, prg_bytes + chr_bytes) == 0);
    BOARD_CHECK(rom_metadata_source() == ROM_METADATA_DATABASE_HEADERLESS);
    BOARD_CHECK(read_mem(0x6000) == 4 && read_mem(0x8000) == 0);
    free(raw);
    rom_database_clear();

    BoardImage image;
    BOARD_CHECK(make_unif(&image, " NES - NROM ", 0x8000, 0x2000, NULL));
    uint8_t replacement[0x2000];
    memset(replacement, 0xD3, sizeof(replacement));
    BOARD_CHECK(append_chunk(&image, "NAME", "unused", 6));
    BOARD_CHECK(append_chunk(&image, "CHR0", replacement, sizeof(replacement)));
    const char *path = "build/unif-runtime-fixture.unif";
    FILE *file = fopen(path, "wb");
    BOARD_CHECK(file != NULL);
    bool written = fwrite(image.data, 1, image.size, file) == image.size;
    BOARD_CHECK(fclose(file) == 0 && written);
    BOARD_CHECK(load_rom(path) == 0);
    BOARD_CHECK(rom_metadata_source() == ROM_METADATA_UNIF && ppu_read(0x0123) == 0xD3);
    BOARD_CHECK(remove(path) == 0);
    uint8_t *previous_prg = prg_rom;
    BOARD_CHECK(append_chunk(&image, "TVCI", NULL, 0));
    BOARD_CHECK(load_rom_memory(image.data, image.size) < 0 && prg_rom == previous_prg);
    BOARD_CHECK(ppu_read(0x0123) == 0xD3);
    board_image_free(&image);
    return 0;
}

int test_unif_accuracy(void) {
    int (*tests[])(void) = {
        test_discrete_named_boards, test_ghostbusters_open_bus,
        test_8237a_banks_and_irq, test_famicombox_cpu_ram,
        test_database_resolution_before_allocation, test_database_input_and_vs,
        test_named_database_and_file_loader
    };
    NesInputConfiguration saved_input = {joypad_adapter(),
        {joypad_port_device(0), joypad_port_device(1)}, joypad_expansion_device()};
    uint8_t saved_overrides = joypad_configuration_overrides();
    NesRamPowerOnState saved_power = nes_ram_power_on_state();
    int failures = 0;
    unsigned count = (unsigned)(sizeof(tests) / sizeof(tests[0]));
    for (unsigned i = 0; i < count; ++i) {
        rom_database_clear();
        rom_database_set_overrides(true);
        cpu_use_default_startup_alignment();
        failures += tests[i]();
        unload_rom();
        (void)joypad_apply_configuration(&saved_input);
        joypad_set_configuration_overrides(saved_overrides);
        (void)nes_set_ram_power_on_state(saved_power);
    }
    cpu_use_default_startup_alignment();
    rom_database_clear();
    unload_rom();
    printf("UNIF accuracy: %u groups, %d failures\n", count, failures);
    return failures;
}
