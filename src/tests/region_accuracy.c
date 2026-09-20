/*
 * region_accuracy.c - Region selection through production image loaders
 *
 * Author: @frankischilling
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License, version 3 or later.
 * This program is distributed without any warranty; see the license for details.
 */
#include "../rom/rom.h"
#include "../rom/mapper.h"
#include "../rom/game_db.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../apu/apu.h"
#include "../system/timing.h"
#include "../system/hardware.h"
#include "../system/vs_system.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t image[16 + 0x4000 + 0x2000];
static uint8_t unif[0x8000];
static uint8_t disk[65500];
static uint8_t bios[0x40000];
static unsigned checks;

#define CHECK(condition) do { \
    ++checks; \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static const NesRegionMode modes[] = {
    NES_REGION_MODE_AUTO, NES_REGION_MODE_NTSC, NES_REGION_MODE_PAL, NES_REGION_MODE_DENDY
};

static void make_image(bool nes2, uint8_t region) {
    memset(image, 0, sizeof(image));
    memcpy(image, "NES\x1A", 4);
    image[4] = 1;
    image[5] = 1;
    image[7] = nes2 ? 8 : 0;
    image[nes2 ? 12 : 9] = region;
    memset(image + 16, 0xEA, 0x4000);
    image[16 + 0x3FFC] = 0;
    image[16 + 0x3FFD] = 0x80;
}

static bool power_machine(void) {
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    return cpu_power_on(&cpu);
}

static size_t append_unif(size_t offset, const char tag[4], const void *data, size_t size) {
    if (offset > sizeof(unif) - 8 || size > sizeof(unif) - offset - 8) return 0;
    memcpy(unif + offset, tag, 4);
    for (unsigned byte = 0; byte < 4; ++byte)
        unif[offset + 4 + byte] = (uint8_t)(size >> (8 * byte));
    memcpy(unif + offset + 8, data, size);
    return offset + 8 + size;
}

static size_t make_unif(uint8_t tv) {
    memset(unif, 0, sizeof(unif));
    memcpy(unif, "UNIF", 4);
    unif[4] = 7;
    size_t offset = append_unif(32, "MAPR", "NROM", 5);
    if (offset) offset = append_unif(offset, "PRG0", image + 16, 0x4000);
    if (offset) offset = append_unif(offset, "CHR0", image + 16 + 0x4000, 0x2000);
    if (offset) offset = append_unif(offset, "TVCI", &tv, 1);
    return offset;
}

static int header_modes(void) {
    static const NesRegion detected[] = {
        NES_REGION_NTSC, NES_REGION_PAL, NES_REGION_NTSC, NES_REGION_DENDY
    };
    for (unsigned nes2 = 0; nes2 < 2; ++nes2) {
        for (unsigned timing = 0; timing < (nes2 ? 4u : 2u); ++timing) {
            make_image(nes2 != 0, (uint8_t)timing);
            const NesRegion expected[] = {
                detected[timing], NES_REGION_NTSC, NES_REGION_PAL, NES_REGION_DENDY
            };
            for (unsigned mode = 0; mode < 4; ++mode) {
                CHECK(nes_set_region_mode(modes[mode]));
                CHECK(load_rom_memory(image, sizeof(image)) == 0);
                CHECK(nes_timing()->region == expected[mode]);
                CHECK(memcmp(&ines_header, image, sizeof(ines_header)) == 0);
                CHECK(rom_metadata_source() == (nes2 ? ROM_METADATA_NES20 : ROM_METADATA_INES));
            }
        }
    }
    return 0;
}

static int unif_modes(void) {
    make_image(false, 0);
    for (uint8_t tv = 0; tv < 3; ++tv) {
        size_t size = make_unif(tv);
        CHECK(size != 0);
        const NesRegion expected[] = {
            tv == 1 ? NES_REGION_PAL : NES_REGION_NTSC,
            NES_REGION_NTSC, NES_REGION_PAL, NES_REGION_DENDY
        };
        iNESHeader metadata = {0};
        for (unsigned mode = 0; mode < 4; ++mode) {
            CHECK(nes_set_region_mode(modes[mode]));
            CHECK(load_rom_memory(unif, size) == 0);
            CHECK(nes_timing()->region == expected[mode]);
            CHECK(rom_metadata_source() == ROM_METADATA_UNIF);
            if (!mode) metadata = ines_header;
            else CHECK(memcmp(&metadata, &ines_header, sizeof(metadata)) == 0);
        }
    }
    return 0;
}

static int database_modes(void) {
    static const char *const systems[] = {"NesNtsc", "NesPal", "Dendy"};
    static const NesRegion detected[] = {NES_REGION_NTSC, NES_REGION_PAL, NES_REGION_DENDY};
    make_image(false, 0);
    uint32_t crc = game_db_crc32(image + 16, sizeof(image) - 16);
    size_t unif_size = make_unif(0);
    CHECK(unif_size != 0);
    for (unsigned system = 0; system < 3; ++system) {
        char row[256];
        int length = snprintf(row, sizeof(row),
            "%08X,%s,TEST,,,0,b16384,b8192,,b8192,b0,0,h,1,N,0,0,0\n",
            (unsigned)crc, systems[system]);
        CHECK(length > 0 && (size_t)length < sizeof(row));
        CHECK(rom_database_load_memory(row, (size_t)length));
        const NesRegion expected[] = {
            detected[system], NES_REGION_NTSC, NES_REGION_PAL, NES_REGION_DENDY
        };
        for (unsigned format = 0; format < 3; ++format) {
            iNESHeader metadata = {0};
            for (unsigned mode = 0; mode < 4; ++mode) {
                CHECK(nes_set_region_mode(modes[mode]));
                const uint8_t *data = format == 2 ? unif : image + (format == 1 ? 16 : 0);
                size_t size = format == 2 ? unif_size : sizeof(image) - (format == 1 ? 16 : 0);
                CHECK(load_rom_memory(data, size) == 0);
                CHECK(nes_timing()->region == expected[mode]);
                CHECK(rom_metadata_source() == (format == 1 ? ROM_METADATA_DATABASE_HEADERLESS
                                                            : ROM_METADATA_DATABASE));
                if (!mode) metadata = ines_header;
                else CHECK(memcmp(&metadata, &ines_header, sizeof(metadata)) == 0);
            }
        }
    }
    // NES 2.0 keeps its declared metadata even when a legacy database row matches.
    image[7] = 8;
    image[12] = 1;
    CHECK(nes_set_region_mode(NES_REGION_MODE_AUTO));
    CHECK(load_rom_memory(image, sizeof(image)) == 0);
    CHECK(nes_timing()->region == NES_REGION_PAL && rom_metadata_source() == ROM_METADATA_NES20);
    CHECK(nes_set_region_mode(NES_REGION_MODE_DENDY));
    CHECK(load_rom_memory(image, sizeof(image)) == 0);
    CHECK(nes_timing()->region == NES_REGION_DENDY && ines_header.zero[1] == 1);
    rom_database_clear();
    return 0;
}

static int machine_clocks(void) {
    static const NesRegion expected[] = {NES_REGION_NTSC, NES_REGION_PAL, NES_REGION_DENDY};
    static const unsigned ten_cpu_dots[] = {30, 32, 30};
    static const unsigned lines[] = {262, 312, 312};
    static const unsigned vblank[] = {241, 241, 291};
    static const unsigned rendered_odd_frame[] = {89341, 106392, 106392};
    static const unsigned noise[] = {4068, 3778, 4068};
    static const unsigned dmc[] = {54, 50, 54};
    static const double cpu_hz[] = {1789773.0, 1662607.0, 1773448.0};
    make_image(true, 0);
    for (unsigned mode = 0; mode < 3; ++mode) {
        CHECK(nes_set_region_mode(modes[mode + 1]));
        CHECK(load_rom_memory(image, sizeof(image)) == 0 && power_machine());
        CHECK(nes_timing()->region == expected[mode]);
        uint64_t dots = ppu.total_cycles;
        uint64_t cpu_cycles = cpu_total_cycles;
        for (unsigned instruction = 0; instruction < 5; ++instruction)
            CHECK(cpu_step(&cpu) == 2);
        CHECK(cpu_total_cycles - cpu_cycles == 10);
        CHECK(ppu.total_cycles - dots == ten_cpu_dots[mode]);
        apu_write(0x400E, 15);
        apu_write(0x4010, 15);
        CHECK(apu.noise.period == noise[mode] && apu.dmc.timer_reload + 1u == dmc[mode]);
        CHECK(apu.sample_rate > 1.0);
        CHECK(fabs(apu.cycles_per_sample * apu.sample_rate - cpu_hz[mode]) < 0.001);

        ppu_power_on(&ppu);
        ppu.scanline = 0;
        ppu.status = 0;
        ppu_step_dots(vblank[mode] * 341u + 1u);
        CHECK(!(ppu.status & 0x80));
        ppu_step_dots(1);
        CHECK(ppu.status & 0x80);
        ppu_step_dots((lines[mode] - vblank[mode]) * 341u - 2u);
        CHECK(ppu.frame_complete && ppu.scanline == 0 && ppu.dot == 0);

        ppu_power_on(&ppu);
        ppu.scanline = 0;
        ppu.odd_frame = true;
        ppu_reg_write(0x2001, 0x18);
        ppu_step_dots(rendered_odd_frame[mode] - 1);
        CHECK(!ppu.frame_complete);
        ppu_step_dots(1);
        CHECK(ppu.frame_complete && ppu.scanline == 0 && ppu.dot == 0);

        CHECK(nes_set_region_mode(NES_REGION_MODE_AUTO));
        ppu_soft_reset(&ppu);
        apu_soft_reset(&apu);
        cpu_soft_reset(&cpu);
        CHECK(nes_timing()->region == expected[mode]);
        apu_write(0x400E, 15);
        apu_write(0x4010, 15);
        CHECK(apu.noise.period == noise[mode] && apu.dmc.timer_reload + 1u == dmc[mode]);
        CHECK(fabs(apu.cycles_per_sample * apu.sample_rate - cpu_hz[mode]) < 0.001);
        dots = ppu.total_cycles;
        for (unsigned instruction = 0; instruction < 5; ++instruction)
            CHECK(cpu_step(&cpu) == 2);
        CHECK(ppu.total_cycles - dots == ten_cpu_dots[mode]);
    }
    return 0;
}

static int replacement_and_alignment(void) {
    make_image(true, 1);
    CHECK(nes_set_console_model(NES_CONSOLE_HVC001));
    CHECK(nes_set_region_mode(NES_REGION_MODE_AUTO));
    CHECK(load_rom_memory(image, sizeof(image)) == 0 && power_machine());
    CHECK(nes_timing()->region == NES_REGION_PAL && nes_console_model() == NES_CONSOLE_HVC001);
    CHECK(cpu_set_startup_alignment(15, 4));
    CHECK(nes_set_region_mode(NES_REGION_MODE_NTSC));
    Mapper *active = cart;
    uint32_t crc = rom_file_crc32();
    write_mem(0x0010, 0xA7);
    uint64_t cycles = cpu_total_cycles;
    CHECK(load_rom_memory(image, sizeof(image)) < 0);
    CHECK(cart == active && rom_file_crc32() == crc && nes_timing()->region == NES_REGION_PAL);
    CHECK(cpu_total_cycles == cycles && read_mem(0x0010) == 0xA7);
    CHECK(nes_set_region_mode(NES_REGION_MODE_PAL));
    image[12] = 0;
    CHECK(load_rom_memory(image, sizeof(image)) == 0 && power_machine());
    CHECK(cpu_get_startup_alignment().cpu_offset == 15 && cpu_get_startup_alignment().ppu_phase == 4);
    CHECK(nes_timing()->region == NES_REGION_PAL && ines_header.zero[1] == 0);
    cpu_use_default_startup_alignment();
    CHECK(nes_set_region_mode(NES_REGION_MODE_DENDY));
    active = cart;
    CHECK(load_rom_memory(image, sizeof(image) - 1) < 0);
    CHECK(cart == active && nes_timing()->region == NES_REGION_PAL);
    CHECK(!nes_set_region_mode((NesRegionMode)-1));
    CHECK(!nes_set_region_mode_name("PAL") && !nes_set_region_mode_name(NULL));
    CHECK(nes_region_mode() == NES_REGION_MODE_DENDY && nes_timing()->region == NES_REGION_PAL);
    CHECK(load_rom_memory(image, sizeof(image)) == 0 && power_machine());
    CHECK(nes_timing()->region == NES_REGION_DENDY && nes_console_model() == NES_CONSOLE_HVC001);
    unload_rom();
    CHECK(nes_region_mode() == NES_REGION_MODE_DENDY);
    CHECK(load_rom_memory(image, sizeof(image)) == 0 && nes_timing()->region == NES_REGION_DENDY);
    return 0;
}

static int special_hardware(void) {
    make_image(true, 0);
    CHECK(nes_set_region_mode(NES_REGION_MODE_PAL));
    CHECK(load_rom_memory(image, sizeof(image)) == 0 && power_machine());
    Mapper *active = cart;
    uint32_t crc = rom_file_crc32();
    image[7] = 9; // Supported single VS cabinet, ordinary 2C03 PPU.
    for (unsigned mode = 2; mode < 4; ++mode) {
        CHECK(nes_set_region_mode(modes[mode]));
        CHECK(load_rom_memory(image, sizeof(image)) < 0);
        CHECK(cart == active && rom_file_crc32() == crc && !vs_enabled());
        CHECK(nes_timing()->region == NES_REGION_PAL);
    }
    image[12] = 1;
    CHECK(nes_set_region_mode(NES_REGION_MODE_NTSC));
    CHECK(load_rom_memory(image, sizeof(image)) == 0 && vs_enabled());
    CHECK(nes_timing()->region == NES_REGION_NTSC && ines_header.zero[1] == 1);
    image[12] = 0;
    CHECK(nes_set_region_mode(NES_REGION_MODE_AUTO));
    CHECK(load_rom_memory(image, sizeof(image)) == 0 && vs_enabled());
    CHECK(nes_timing()->region == NES_REGION_NTSC);

    memset(disk, 0, sizeof(disk));
    disk[0] = 1;
    disk[1] = 0x2A;
    disk[56] = 2;
    memset(bios, 0xEA, sizeof(bios));
    bios[0x1FFC] = 0;
    bios[0x1FFD] = 0xE0;
    active = cart;
    for (unsigned mode = 2; mode < 4; ++mode) {
        CHECK(nes_set_region_mode(modes[mode]));
        CHECK(load_fds_memory(disk, sizeof(disk), bios, 0x2000, NULL, false) < 0);
        CHECK(cart == active && vs_enabled() && nes_timing()->region == NES_REGION_NTSC);
    }
    CHECK(nes_set_region_mode(NES_REGION_MODE_AUTO));
    CHECK(load_fds_memory(disk, sizeof(disk), bios, 0x2000, NULL, false) == 0);
    CHECK(rom_is_fds() && !vs_enabled() && nes_timing()->region == NES_REGION_NTSC);
    CHECK(nes_set_region_mode(NES_REGION_MODE_NTSC));
    CHECK(load_fds_memory(disk, sizeof(disk), bios, 0x2000, NULL, false) == 0);
    CHECK(rom_is_fds() && nes_timing()->region == NES_REGION_NTSC);

    const uint8_t tape[34] = {
        'S','T','B','X', 4,0,0,0, 0,1,0,0,
        'P','A','G','E', 14,0,0,0, 0,0,0,0, 0,0,0,0, 0xC5,1,2,3,4,5
    };
    active = cart;
    for (unsigned mode = 2; mode < 4; ++mode) {
        CHECK(nes_set_region_mode(modes[mode]));
        CHECK(load_studybox_memory(tape, sizeof(tape), bios, sizeof(bios)) < 0);
        CHECK(cart == active && rom_is_fds() && nes_timing()->region == NES_REGION_NTSC);
    }
    CHECK(nes_set_region_mode(NES_REGION_MODE_NTSC));
    CHECK(load_studybox_memory(tape, sizeof(tape), bios, sizeof(bios)) == 0);
    CHECK(rom_is_studybox() && !rom_is_fds() && nes_timing()->region == NES_REGION_NTSC);
    CHECK(nes_set_region_mode(NES_REGION_MODE_AUTO));
    CHECK(load_studybox_memory(tape, sizeof(tape), bios, sizeof(bios)) == 0);
    CHECK(rom_is_studybox() && nes_timing()->region == NES_REGION_NTSC);
    return 0;
}

int test_region_accuracy(void) {
    const NesRegionMode saved_mode = nes_region_mode();
    const NesRegion saved_region = nes_timing()->region;
    const NesConsoleModel saved_console = nes_console_model();
    const bool saved_overrides = rom_database_overrides_enabled();
    const bool saved_restriction = ppu_startup_write_restriction_enabled();
    const bool saved_suppression = ppu_reset_suppression_enabled();
    int (*const tests[])(void) = {
        header_modes, unif_modes, database_modes, machine_clocks,
        replacement_and_alignment, special_hardware
    };
    int failures = 0;
    checks = 0;
    ppu_set_startup_write_restriction(false);
    ppu_set_reset_suppression(false);
    for (unsigned test = 0; test < sizeof(tests) / sizeof(tests[0]); ++test) {
        unload_rom();
        rom_database_clear();
        rom_database_set_overrides(true);
        cpu_use_default_startup_alignment();
        nes_set_region_mode(NES_REGION_MODE_AUTO);
        failures += tests[test]();
    }
    unload_rom();
    rom_database_clear();
    rom_database_set_overrides(saved_overrides);
    cpu_use_default_startup_alignment();
    nes_set_region_mode(saved_mode);
    nes_set_region(saved_region);
    nes_set_console_model(saved_console);
    ppu_set_startup_write_restriction(saved_restriction);
    ppu_set_reset_suppression(saved_suppression);
    printf("Region selection: %u checks, %d failures\n", checks, failures);
    return failures;
}
