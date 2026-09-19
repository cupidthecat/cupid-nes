/*
 * rom_database_defaults_accuracy.c - Database defaults and cartridge initialization
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 * This program is distributed without any warranty; see the GNU General
 * Public License for details. See <https://www.gnu.org/licenses/>.
 */
#include "board_tests.h"
#include "../joypad/joypad.h"
#include "../rom/game_db.h"
#include "../system/hardware.h"
#include "../system/vs_system.h"

static int database_entry_input(const BoardImage *image, unsigned mapper, size_t prg, size_t chr,
                                const char *chr_ram, const char *work, const char *save,
                                bool battery, const char *mirror, const char *submapper,
                                const char *system, unsigned input) {
    size_t offset = sizeof(iNESHeader) + (image->data[6] & 4 ? 512u : 0u);
    uint32_t crc = game_db_crc32(image->data + offset, image->size - offset);
    char row[512];
    int length = snprintf(row, sizeof(row),
        "%08X,%s,TEST,,,%u,b%u,b%u,%s,%s,%s,%u,%s,%u,N,%s,0,0\n",
        (unsigned)crc, system, mapper, (unsigned)prg, (unsigned)chr,
        chr_ram, work, save, battery ? 1 : 0, mirror, input, submapper);
    BOARD_CHECK(length > 0 && (size_t)length < sizeof(row));
    BOARD_CHECK(rom_database_load_memory(row, (size_t)length));
    return 0;
}

static int database_entry(const BoardImage *image, unsigned mapper, size_t prg, size_t chr,
                          const char *chr_ram, const char *work, const char *save,
                          bool battery, const char *mirror, const char *submapper,
                          const char *system) {
    return database_entry_input(image, mapper, prg, chr, chr_ram, work, save,
                                battery, mirror, submapper, system, 1);
}

static int database_store(uint16_t address, uint8_t value) {
    const uint8_t program[] = {0xA9, value, 0x8D, (uint8_t)address, (uint8_t)(address >> 8)};
    for (unsigned byte = 0; byte < sizeof(program); ++byte)
        write_mem((uint16_t)(0x0200 + byte), program[byte]);
    cpu.pc = 0x0200;
    BOARD_CHECK(cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4);
    return 0;
}

static int database_ram_banks(void) {
    for (unsigned bank = 0; bank < 4; ++bank) {
        BOARD_CHECK(database_store(0x8000, (uint8_t)((bank << 6) | bank)) == 0);
        BOARD_CHECK(read_mem(0x8000) == bank * 8 && read_mem(0x6000) == 0 && read_mem(0x7FFF) == 0);
        BOARD_CHECK(database_store(0x6000, (uint8_t)(0xA0 + bank)) == 0);
        BOARD_CHECK(database_store(0x7FFF, (uint8_t)(0xB0 + bank)) == 0);
    }
    for (unsigned bank = 0; bank < 4; ++bank) {
        BOARD_CHECK(database_store(0xFFFF, (uint8_t)((bank << 6) | bank)) == 0);
        BOARD_CHECK(read_mem(0x6000) == 0xA0 + bank && read_mem(0x7FFF) == 0xB0 + bank);
    }
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x8000) == 24 && read_mem(0x6000) == 0xA3 && read_mem(0x7FFF) == 0xB3);
    return 0;
}

static int test_database_329_unspecified_and_explicit_ram(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 0, 0x100000, 0, false));
    BOARD_CHECK(database_entry(&image, 329, 0x100000, 0, "", "", "", false, "h", "", "") == 0);
    BOARD_CHECK(board_image_load(&image) == 0 && rom_mapper_number(&ines_header) == 329);
    BOARD_CHECK(database_ram_banks() == 0);
    ppu_write(0x123, 0x6D);
    BOARD_CHECK(ppu_read(0x123) == 0x6D && chr_size == 0x2000);
    BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0 && read_mem(0x6000) == 0xA3);
    BOARD_CHECK(ppu_read(0x123) == 0x6D);

    BOARD_CHECK(load_rom_memory(image.data + 16, image.size - 16) == 0);
    ppu_power_on(&ppu);
    BOARD_CHECK(cpu_power_on(&cpu) && rom_metadata_source() == ROM_METADATA_DATABASE_HEADERLESS);
    BOARD_CHECK(database_ram_banks() == 0);
    BOARD_CHECK(database_entry(&image, 329, 0x100000, 0, "0", "0", "0", false, "h", "", "NesNtsc") == 0);
    BOARD_CHECK(board_image_load(&image) == 0 && database_ram_banks() == 0);

    BOARD_CHECK(database_entry(&image, 329, 0x100000, 0, "0", "0", "0", false, "h", "0", "NesNtsc") == 0);
    BOARD_CHECK(board_image_load(&image) == 0 && database_store(0x6000, 0x6D) == 0);
    BOARD_CHECK(cart_cpu_read_bus(0x6000, 0xA5) == 0xA5 && cart_cpu_read_bus(0x7FFF, 0x5A) == 0x5A);
    ppu_write(0x123, 0xCC);
    BOARD_CHECK(ppu_read(0x123) == 0x23);

    const char *validation[] = {"", "0"};
    for (unsigned check = 0; check < 2; ++check) {
        BOARD_CHECK(database_entry(&image, 329, 0x100000, 0, "8", "b512", "0", false,
                                   "h", validation[check], "NesNtsc") == 0);
        BOARD_CHECK(board_image_load(&image) == 0 && database_store(0x6000, 0x6D) == 0);
        BOARD_CHECK(read_mem(0x7E00) == 0x6D && database_store(0xFFFF, 0xC3) == 0);
        BOARD_CHECK(read_mem(0x6000) == 0x6D && read_mem(0x8000) == 24);
        BOARD_CHECK(database_store(0x61FF, 0xA5) == 0 && read_mem(0x7FFF) == 0xA5);
    }
    board_image_free(&image);
    return 0;
}

static int test_database_board_specific_defaults(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 0, 0x20000, 0, false));
    BOARD_CHECK(board_image_add_trainer(&image, 0x3E));
    BOARD_CHECK(database_entry(&image, 103, 0x20000, 0, "", "", "", false, "h", "", "NesNtsc") == 0);
    BOARD_CHECK(board_image_load(&image) == 0 && read_mem(0x7000) == 0x3E);
    BOARD_CHECK(database_store(0x6000, 0xA5) == 0 && database_store(0xB800, 0x5A) == 0);
    BOARD_CHECK(read_mem(0x6000) == 0xA5 && read_mem(0xB800) == 0x5A);
    BOARD_CHECK(database_store(0x8000, 3) == 0 && database_store(0xF000, 0x10) == 0);
    BOARD_CHECK(read_mem(0x6000) == 6 && database_store(0x6000, 0x6D) == 0);
    BOARD_CHECK(database_store(0xF000, 0) == 0 && read_mem(0x6000) == 0x6D && read_mem(0xB800) == 0x5A);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(read_mem(0x6000) == 0x6D && read_mem(0xB800) == 0x5A && read_mem(0x7000) == 0x3E);
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 0, 0x20000, 0x2000, false));
    BOARD_CHECK(database_entry(&image, 69, 0x20000, 0x2000, "", "", "", false, "h", "", "NesNtsc") == 0);
    BOARD_CHECK(board_image_load(&image) == 0);
    for (unsigned bank = 0; bank < 4; ++bank) {
        BOARD_CHECK(database_store(0x8000, 8) == 0 && database_store(0xA000, (uint8_t)(0xC0 | bank)) == 0);
        BOARD_CHECK(read_mem(0x6000) == 0 && database_store(0x6000, (uint8_t)(0xA0 + bank)) == 0);
    }
    for (unsigned bank = 0; bank < 4; ++bank) {
        BOARD_CHECK(database_store(0x8000, 8) == 0 && database_store(0xA000, (uint8_t)(0xC0 | bank)) == 0);
        BOARD_CHECK(read_mem(0x6000) == 0xA0 + bank);
    }
    board_image_free(&image);

    BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0, false));
    BOARD_CHECK(database_entry(&image, 13, 0x8000, 0, "", "", "", false, "h", "", "NesNtsc") == 0);
    BOARD_CHECK(board_image_load(&image) == 0 && chr_size == 0x4000);
    for (unsigned bank = 0; bank < 4; ++bank) {
        BOARD_CHECK(database_store(0x8000, (uint8_t)bank) == 0);
        BOARD_CHECK(ppu_read(0x1123) == 0);
        ppu_write(0x1123, (uint8_t)(0xB0 + bank));
    }
    for (unsigned bank = 0; bank < 4; ++bank) {
        BOARD_CHECK(database_store(0x8000, (uint8_t)bank) == 0);
        BOARD_CHECK(ppu_read(0x1123) == 0xB0 + bank);
    }
    board_image_free(&image);
    return 0;
}

static int test_database_startup_mirroring_order(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 0, 0x40000, 0x2000, false));
    BOARD_CHECK(database_entry(&image, 156, 0x40000, 0x2000, "", "", "", false, "v", "", "NesNtsc") == 0);
    BOARD_CHECK(board_image_load(&image) == 0 && cart_get_mirroring() == MIRROR_SINGLE0);
    ppu_write(0x2000, 0xA5);
    BOARD_CHECK(ppu_read(0x2400) == 0xA5 && ppu_read(0x2800) == 0xA5 && ppu_read(0x2C00) == 0xA5);
    BOARD_CHECK(ppu_read(0x123) == 0x23 && database_store(0xC000, 3) == 0);
    BOARD_CHECK(ppu_read(0) == 3 && cart_get_mirroring() == MIRROR_SINGLE0);

    BOARD_CHECK(database_entry(&image, 285, 0x40000, 0x2000, "", "", "", false, "h", "", "NesNtsc") == 0);
    BOARD_CHECK(board_image_load(&image) == 0 && cart_get_mirroring() == MIRROR_VERTICAL);
    ppu_write(0x2000, 0xA5);
    ppu_write(0x2400, 0x5A);
    BOARD_CHECK(ppu_read(0x2800) == 0xA5 && ppu_read(0x2C00) == 0x5A);

    BOARD_CHECK(database_entry(&image, 240, 0x40000, 0x2000, "", "", "", false, "1", "", "NesNtsc") == 0);
    BOARD_CHECK(board_image_load(&image) == 0 && cart_get_mirroring() == MIRROR_SINGLE1);
    ppu_write(0x2000, 0x6D);
    BOARD_CHECK(ppu_read(0x2400) == 0x6D && ppu_read(0x2800) == 0x6D && ppu_read(0x2C00) == 0x6D);
    cpu_soft_reset(&cpu);
    BOARD_CHECK(cart_get_mirroring() == MIRROR_SINGLE1);
    board_image_free(&image);
    return 0;
}

static int test_database_default_console_and_explicit_region(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0x2000, false));
    image.data[9] = 1;
    BOARD_CHECK(database_entry(&image, 240, 0x8000, 0x2000, "", "", "", false, "h", "", "") == 0);
    BOARD_CHECK(board_image_load(&image) == 0 && nes_timing()->region == NES_REGION_NTSC && !vs_enabled());
    image.data[7] |= 1;
    BOARD_CHECK(board_image_load(&image) == 0 && !vs_enabled() && (ines_header.flags7 & 3) == 0);
    BOARD_CHECK(database_entry(&image, 240, 0x8000, 0x2000, "", "", "", false, "h", "", "NesUnknown") == 0);
    BOARD_CHECK(board_image_load(&image) == 0 && nes_timing()->region == NES_REGION_NTSC && !vs_enabled());
    BOARD_CHECK(database_entry(&image, 240, 0x8000, 0x2000, "", "", "", false, "h", "", "NesPal") == 0);
    BOARD_CHECK(board_image_load(&image) == 0 && nes_timing()->region == NES_REGION_PAL && !vs_enabled());
    BOARD_CHECK(database_entry(&image, 240, 0x8000, 0x2000, "", "", "", false, "h", "", "Dendy") == 0);
    BOARD_CHECK(board_image_load(&image) == 0 && nes_timing()->region == NES_REGION_DENDY && !vs_enabled());
    board_image_free(&image);
    return 0;
}

static int test_database_zapper_system_routing(void) {
    BoardImage image;
    BOARD_CHECK(board_image_create(&image, 0, 0x8000, 0x2000, false));

    NesConsoleModel saved_model = nes_console_model();
    uint8_t saved_overrides = joypad_configuration_overrides();
    NesInputAdapter saved_adapter = joypad_adapter();
    NesPortDevice saved_port1 = joypad_port_device(0);
    NesPortDevice saved_port2 = joypad_port_device(1);
    NesExpansionDevice saved_expansion = joypad_expansion_device();

    joypad_set_configuration_overrides(0);
    BOARD_CHECK(nes_set_console_model(NES_CONSOLE_NES001));
    BOARD_CHECK(joypad_set_adapter(NES_ADAPTER_NONE));
    BOARD_CHECK(joypad_set_port_device(0, NES_PORT_GAMEPAD));
    BOARD_CHECK(joypad_set_port_device(1, NES_PORT_GAMEPAD));
    BOARD_CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    BOARD_CHECK(database_entry_input(&image, 0, 0x8000, 0x2000, "", "", "", false,
                                     "h", "", "Famicom", 8) == 0);
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(joypad_port_device(1) == NES_PORT_GAMEPAD
                && joypad_expansion_device() == NES_EXPANSION_ZAPPER);
    BOARD_CHECK(joypad_set_zapper(2, -1, -1, true));
    write_mem(0x4018, 0);
    BOARD_CHECK((read_mem(0x4017) & 0x10u) != 0);

    BOARD_CHECK(nes_set_console_model(NES_CONSOLE_HVC001));
    BOARD_CHECK(joypad_set_port_device(1, NES_PORT_GAMEPAD));
    BOARD_CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    BOARD_CHECK(database_entry_input(&image, 0, 0x8000, 0x2000, "", "", "", false,
                                     "h", "", "NesNtsc", 8) == 0);
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(joypad_port_device(1) == NES_PORT_ZAPPER
                && joypad_expansion_device() == NES_EXPANSION_NONE);
    BOARD_CHECK(joypad_set_zapper(1, -1, -1, true));
    write_mem(0x4018, 0);
    BOARD_CHECK((read_mem(0x4017) & 0x10u) != 0);

    BOARD_CHECK(nes_set_console_model(NES_CONSOLE_NES001));
    BOARD_CHECK(joypad_set_port_device(1, NES_PORT_GAMEPAD));
    BOARD_CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    BOARD_CHECK(database_entry_input(&image, 0, 0x8000, 0x2000, "", "", "", false,
                                     "h", "", "Dendy", 8) == 0);
    BOARD_CHECK(board_image_load(&image) == 0 && nes_timing()->region == NES_REGION_DENDY);
    BOARD_CHECK(joypad_port_device(1) == NES_PORT_GAMEPAD
                && joypad_expansion_device() == NES_EXPANSION_ZAPPER);

    BOARD_CHECK(nes_set_console_model(NES_CONSOLE_HVC001));
    BOARD_CHECK(joypad_set_port_device(1, NES_PORT_GAMEPAD));
    BOARD_CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    BOARD_CHECK(database_entry_input(&image, 0, 0x8000, 0x2000, "", "", "", false,
                                     "h", "", "NesUnknown", 8) == 0);
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(joypad_port_device(1) == NES_PORT_ZAPPER
                && joypad_expansion_device() == NES_EXPANSION_NONE);

    joypad_set_configuration_overrides(NES_INPUT_OVERRIDE_PORT2);
    BOARD_CHECK(joypad_set_port_device(1, NES_PORT_GAMEPAD));
    BOARD_CHECK(database_entry_input(&image, 0, 0x8000, 0x2000, "", "", "", false,
                                     "h", "", "NesNtsc", 8) == 0);
    BOARD_CHECK(board_image_load(&image) == 0 && joypad_port_device(1) == NES_PORT_GAMEPAD);

    joypad_set_configuration_overrides(NES_INPUT_OVERRIDE_EXPANSION);
    BOARD_CHECK(nes_set_console_model(NES_CONSOLE_NES001));
    BOARD_CHECK(joypad_set_expansion_device(NES_EXPANSION_FAMILY_BASIC));
    BOARD_CHECK(database_entry_input(&image, 0, 0x8000, 0x2000, "", "", "", false,
                                     "h", "", "Famicom", 8) == 0);
    BOARD_CHECK(board_image_load(&image) == 0
                && joypad_expansion_device() == NES_EXPANSION_FAMILY_BASIC);

    joypad_set_configuration_overrides(0);
    BOARD_CHECK(joypad_set_port_device(1, NES_PORT_GAMEPAD));
    BOARD_CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    BOARD_CHECK(database_entry_input(&image, 0, 0x8000, 0x2000, "", "", "", false,
                                     "h", "", "Famicom", 8) == 0);
    BOARD_CHECK(load_rom_memory(image.data + sizeof(iNESHeader), image.size - sizeof(iNESHeader)) == 0);
    BOARD_CHECK(rom_metadata_source() == ROM_METADATA_DATABASE_HEADERLESS
                && joypad_expansion_device() == NES_EXPANSION_ZAPPER);

    BOARD_CHECK(database_entry_input(&image, 4095, 0x8000, 0x2000, "", "", "", false,
                                     "h", "", "NesNtsc", 8) == 0);
    BOARD_CHECK(board_image_load(&image) < 0);
    BOARD_CHECK(joypad_port_device(1) == NES_PORT_GAMEPAD
                && joypad_expansion_device() == NES_EXPANSION_ZAPPER);

    /* Explicit NES 2.0 metadata uses the selected console, even when the
       payload matches a database record for the other connector. */
    BOARD_CHECK(database_entry_input(&image, 0, 0x8000, 0x2000, "", "", "", false,
                                     "h", "", "Famicom", 8) == 0);
    image.data[7] = 0x08;
    image.data[15] = 0x08;
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(rom_metadata_source() == ROM_METADATA_NES20);
    BOARD_CHECK(joypad_port_device(1) == NES_PORT_ZAPPER
                && joypad_expansion_device() == NES_EXPANSION_NONE);
    BOARD_CHECK(nes_set_console_model(NES_CONSOLE_HVC001));
    BOARD_CHECK(database_entry_input(&image, 0, 0x8000, 0x2000, "", "", "", false,
                                     "h", "", "NesNtsc", 8) == 0);
    BOARD_CHECK(board_image_load(&image) == 0);
    BOARD_CHECK(joypad_port_device(1) == NES_PORT_GAMEPAD
                && joypad_expansion_device() == NES_EXPANSION_ZAPPER);

    joypad_set_configuration_overrides(0);
    BOARD_CHECK(joypad_set_adapter(saved_adapter));
    BOARD_CHECK(joypad_set_port_device(0, saved_port1));
    BOARD_CHECK(joypad_set_port_device(1, saved_port2));
    BOARD_CHECK(joypad_set_expansion_device(saved_expansion));
    joypad_set_configuration_overrides(saved_overrides);
    BOARD_CHECK(nes_set_console_model(saved_model));
    board_image_free(&image);
    return 0;
}

int test_rom_database_defaults_accuracy(void) {
    rom_database_clear();
    rom_database_set_overrides(true);
    int failures = 0;
    failures += test_database_329_unspecified_and_explicit_ram();
    failures += test_database_board_specific_defaults();
    failures += test_database_startup_mirroring_order();
    failures += test_database_default_console_and_explicit_region();
    failures += test_database_zapper_system_routing();
    unload_rom();
    rom_database_clear();
    rom_database_set_overrides(true);
    printf("Cartridge database defaults: 5 groups, %d failures\n", failures);
    return failures;
}
