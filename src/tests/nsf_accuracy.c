/*
 * nsf_accuracy.c - NSF and NSFe loader/playback regression tests
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or any later version.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../rom/mapper.h"
#include "../rom/nsf.h"
#include "../rom/rom.h"
#include "../system/timing.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); \
        return 1; \
    } \
} while (0)

static void put16(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static void put32(uint8_t *p, uint32_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

static size_t make_nsf(uint8_t *out, size_t capacity, uint8_t region, uint8_t chips,
                       uint16_t ntsc_speed, uint16_t pal_speed, uint8_t starting_song,
                       const uint8_t *payload, size_t payload_size) {
    if (!out || capacity < 0x80 + payload_size) return 0;
    memset(out, 0, 0x80 + payload_size);
    memcpy(out, "NESM\x1A", 5);
    out[5] = 1;
    out[6] = 2;
    out[7] = starting_song;
    put16(out + 8, 0x8000);
    put16(out + 10, 0x8000);
    put16(out + 12, 0x8010);
    memcpy(out + 14, "Cupid NSF test", 14);
    put16(out + 110, ntsc_speed);
    put16(out + 120, pal_speed);
    out[122] = region;
    out[123] = chips;
    memcpy(out + 0x80, payload, payload_size);
    return 0x80 + payload_size;
}

static void make_play_program(uint8_t *payload, size_t size) {
    memset(payload, 0xEA, size);
    payload[0x00] = 0x8D; payload[0x01] = 0x00; payload[0x02] = 0x60; /* STA $6000 */
    payload[0x03] = 0x8E; payload[0x04] = 0x02; payload[0x05] = 0x60; /* STX $6002 */
    payload[0x06] = 0x60;                                           /* RTS */
    payload[0x10] = 0xEE; payload[0x11] = 0x01; payload[0x12] = 0x60; /* INC $6001 */
    payload[0x13] = 0x60;                                           /* RTS */
}

static int power_music(void) {
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    return cpu_power_on(&cpu) ? 0 : -1;
}

static int run_until_init(uint8_t expected_song) {
    for (unsigned i = 0; i < 32; ++i) {
        if (cart_cpu_read(0x6000) == expected_song && cpu.pc == 0x4108) return 0;
        (void)cpu_step(&cpu);
    }
    return -1;
}

static int test_nsf_init_play_and_tracks(void) {
    cpu_use_default_startup_alignment();
    uint8_t payload[0x100];
    uint8_t image[0x180];
    make_play_program(payload, sizeof(payload));
    size_t size = make_nsf(image, sizeof(image), 0, 0, 1000, 1000, 2,
                           payload, sizeof(payload));
    CHECK(size != 0 && load_rom_memory(image, size) == 0);
    CHECK(rom_is_nsf() && rom_metadata_source() == ROM_METADATA_NSF);
    CHECK(cart_cpu_read_bus(0x4120, 0xA5) == 0);
    CHECK(cart_cpu_read_bus(0x41FF, 0x5A) == 0);
    CHECK(power_music() == 0 && run_until_init(1) == 0);
    CHECK(cart_cpu_read(0x6002) == 0);

    uint64_t start = cpu_total_cycles;
    for (unsigned i = 0; i < 1000 && cart_cpu_read(0x6001) == 0; ++i)
        (void)cpu_step(&cpu);
    CHECK(cart_cpu_read(0x6001) == 1);
    uint64_t elapsed = cpu_total_cycles - start;
    uint64_t expected = (uint64_t)(nes_timing()->cpu_hz / 1000.0);
    CHECK(elapsed + 32 >= expected && elapsed <= expected + 64);

    CHECK(rom_nsf_current_track() == 1);
    CHECK(rom_nsf_select_track(0));
    CHECK(cart_cpu_read(0x6001) == 0);
    CHECK(run_until_init(0) == 0 && rom_nsf_current_track() == 0);
    return 0;
}

static int test_nsf_pal_and_banking(void) {
    cpu_use_default_startup_alignment();
    uint8_t payload[0x2000];
    uint8_t image[0x2080];
    make_play_program(payload, sizeof(payload));
    memcpy(payload + 0x1000, payload, 0x100);
    payload[0x20] = 0x11;
    payload[0x1020] = 0x22;
    size_t size = make_nsf(image, sizeof(image), 1, 0, 1000, 65535, 1,
                           payload, sizeof(payload));
    image[112] = 1;
    CHECK(load_rom_memory(image, size) == 0 && nes_timing()->region == NES_REGION_PAL);
    CHECK(power_music() == 0 && run_until_init(0) == 0);
    CHECK(cart_cpu_read(0x6002) == 1);
    CHECK(cart_cpu_read(0x8020) == 0x22);

    cart_cpu_write(0x6005, 0xA7);
    CHECK(cart_cpu_read(0x6005) == 0xA7);
    cart_cpu_write(0x5FF6, 1);
    CHECK(cart_cpu_read(0x6020) == 0x22);
    cart_cpu_write(0x6020, 0x6C);
    CHECK(cart_cpu_read(0x8020) == 0x6C);
    cart_cpu_write(0x5FF7, 0);
    CHECK(cart_cpu_read(0x7020) == 0x11);
    cart_cpu_write(0x7020, 0x3D);
    cart_cpu_write(0x5FF8, 0);
    CHECK(cart_cpu_read(0x8020) == 0x3D);

    uint16_t reload = (uint16_t)(uint32_t)(65535.0 * nes_timing()->cpu_hz / 1000000.0);
    CHECK(reload > 1);
    cart_cpu_write(0x4100, 0);
    for (uint32_t i = 1; i < reload; ++i) {
        cart_clock_cpu_cycle(false);
        CHECK(!cart_irq_pending());
    }
    cart_clock_cpu_cycle(false);
    CHECK(cart_irq_pending());
    cart_irq_ack();
    return 0;
}

static int test_nsf_zero_reload_and_irq_mask(void) {
    cpu_use_default_startup_alignment();
    const uint16_t wrapped_zero_speed = 36617;
    const NesTiming *timing = nes_timing_for_region(NES_REGION_NTSC);
    CHECK((uint16_t)(uint32_t)(wrapped_zero_speed * timing->cpu_hz / 1000000.0) == 0);

    uint8_t payload[0x100];
    uint8_t image[0x180];
    make_play_program(payload, sizeof(payload));
    size_t size = make_nsf(image, sizeof(image), 0, 0,
                           wrapped_zero_speed, 1000, 1, payload, sizeof(payload));
    CHECK(size != 0 && load_rom_memory(image, size) == 0);
    CHECK(power_music() == 0 && run_until_init(0) == 0);
    CHECK(cart_cpu_read(0x6001) == 0);
    cart_cpu_write(0x4100, 0);
    for (unsigned i = 0; i < 512; ++i) cart_clock_cpu_cycle(false);
    CHECK(!cart_irq_pending() && cart_cpu_read(0x6001) == 0);

    apu.frame_irq = true;
    apu.frame_irq_source = true;
    apu.irq_inhibit = false;
    apu.dmc.irq_flag = false;
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    for (unsigned i = 0; i < 16; ++i) (void)cpu_step(&cpu);
    CHECK(cart_cpu_read(0x6001) == 0);
    apu_clear_frame_irq(&apu);

    apu.dmc.irq_flag = true;
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    for (unsigned i = 0; i < 16; ++i) (void)cpu_step(&cpu);
    CHECK(cart_cpu_read(0x6001) == 0);
    apu.dmc.irq_flag = false;

    uint8_t rom[16 + 0x4000 + 0x2000] = {0};
    memcpy(rom, "NES\x1A", 4);
    rom[4] = 1;
    rom[5] = 1;
    rom[16 + 0x3FFC] = 0x00;
    rom[16 + 0x3FFD] = 0x02;
    rom[16 + 0x3FFE] = 0x00;
    rom[16 + 0x3FFF] = 0x03;
    CHECK(load_rom_memory(rom, sizeof(rom)) == 0 && !cart_nsf_active());
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    write_mem(0x0200, 0xEA);
    cpu.pc = 0x0200;
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    apu.dmc.irq_flag = true;
    (void)cpu_step(&cpu);
    CHECK(cpu.pc == 0x0300);
    apu.dmc.irq_flag = false;
    return 0;
}

static int test_nsf_ppu_clock_only_and_restore(void) {
    bool saved_startup_restriction = ppu_startup_write_restriction_enabled();
    bool saved_reset_suppression = ppu_reset_suppression_enabled();
    ppu_set_startup_write_restriction(false);
    ppu_set_reset_suppression(true);
    cpu_use_default_startup_alignment();

    const uint16_t wrapped_zero_speed = 36617;
    uint8_t payload[0x1000];
    uint8_t image[0x1080];
    make_play_program(payload, sizeof(payload));
    payload[0x20] = 0xE6; payload[0x21] = 0x04; /* INC $04 */
    payload[0x22] = 0x40;                         /* RTI */
    payload[0xFFA] = 0x20; payload[0xFFB] = 0x80; /* NMI -> $8020 */
    size_t size = make_nsf(image, sizeof(image), 0, 0,
                           wrapped_zero_speed, 1000, 1, payload, sizeof(payload));
    CHECK(size != 0 && load_rom_memory(image, size) == 0);
    CHECK(power_music() == 0 && run_until_init(0) == 0);

    ppu.ctrl = 0x80;
    ppu.mask = 0x18;
    ppu.status = 0xE0;
    ppu.v = 0x2345;
    ppu.total_cycles = 12345;
    ppu.frame_count = 9;
    ppu.scanline = 7;
    ppu.dot = 17;
    ppu.oam[0] = 0xA5;
    ppu_soft_reset(&ppu);
    CHECK(ppu.ctrl == 0 && ppu.mask == 0 && ppu.status == 0);
    CHECK(ppu.v == 0x2345 && ppu.total_cycles == 0 && ppu.frame_count == 0);
    CHECK(ppu.scanline == (int)nes_timing()->scanlines - 1 && ppu.dot == 340);
    CHECK(ppu.oam[0] == 0xA5 && !ppu.nmi_out);

    ppu.pixel_indices[0] = 0x2A;
    ppu_reg_write(PPUCTRL, 0x80);
    ppu_reg_write(PPUMASK, 0x18);
    start_frame();
    ppu_step_dots(1);
    CHECK(ppu.scanline == 0 && ppu.dot == 0 && !ppu.frame_complete);
    const int first_frame_clocks = 240 * 341;
    ppu_step_dots(first_frame_clocks - 1);
    CHECK(!ppu.frame_complete && ppu.scanline == 239 && ppu.dot == 340);
    ppu_step_dots(1);
    CHECK(ppu.frame_complete && ppu.scanline == 240 && ppu.dot == 0);
    CHECK(ppu.frame_count == 1 && !(ppu.status & 0x80) && !ppu.nmi_out);
    CHECK(ppu.pixel_indices[0] == 0x2A && ppu.rendering_enabled && !ppu.fetches_enabled);

    uint64_t completed_at = ppu.total_cycles;
    start_frame();
    int full_frame_clocks = (int)nes_timing()->scanlines * 341;
    ppu_step_dots(full_frame_clocks - 1);
    CHECK(!ppu.frame_complete);
    ppu_step_dots(1);
    CHECK(ppu.frame_complete && ppu.frame_count == 2);
    CHECK(ppu.total_cycles - completed_at == (uint64_t)full_frame_clocks);
    CHECK(!(ppu.status & 0x80) && !ppu.nmi_out && ppu.pixel_indices[0] == 0x2A);

    ppu_soft_reset(&ppu);
    ppu_reg_write(PPUCTRL, 0x80);
    ppu_reg_write(PPUMASK, 0x18);
    write_mem(0x0004, 0);
    uint64_t target_frame = ppu.frame_count + 2;
    start_frame();
    for (unsigned instructions = 0;
         instructions < 30000 && ppu.frame_count < target_frame;
         ++instructions) {
        (void)cpu_step(&cpu);
        if (ppu.frame_complete && ppu.frame_count < target_frame) start_frame();
    }
    CHECK(ppu.frame_count == target_frame);
    CHECK(read_mem(0x0004) == 0 && !(ppu.status & 0x80) && !ppu.nmi_out);
    CHECK(ppu.pixel_indices[0] == 0x0F);

    ppu_set_reset_suppression(saved_reset_suppression);
    ppu_set_startup_write_restriction(saved_startup_restriction);

    uint8_t rom[16 + 0x4000 + 0x2000] = {0};
    memcpy(rom, "NES\x1A", 4);
    rom[4] = 1;
    rom[5] = 1;
    uint8_t *prg = rom + 16;
    prg[0x0000] = 0x4C; prg[0x0001] = 0x00; prg[0x0002] = 0x80; /* JMP $8000 */
    prg[0x0100] = 0xE6; prg[0x0101] = 0x10; prg[0x0102] = 0x40; /* INC $10; RTI */
    prg[0x3FFA] = 0x00; prg[0x3FFB] = 0x81;
    prg[0x3FFC] = 0x00; prg[0x3FFD] = 0x80;
    prg[0x3FFE] = 0x00; prg[0x3FFF] = 0x80;
    CHECK(load_rom_memory(rom, sizeof(rom)) == 0 && !cart_nsf_active());
    ppu_set_startup_write_restriction(false);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    CHECK(read_mem(0xFFFA) == 0x00 && read_mem(0xFFFB) == 0x81);
    write_mem(0x0010, 0);
    ppu.pixel_indices[0] = 0x2A;
    ppu_reg_write(PPUCTRL, 0x80);
    ppu_reg_write(PPUMASK, 0x18);
    CHECK(ppu.ctrl == 0x80 && ppu.mask == 0x18);
    for (unsigned clocks = 0; clocks < 100000 && !ppu.nmi_out; ++clocks)
        ppu_step_dots(1);
    CHECK(ppu.nmi_out && (ppu.status & 0x80));
    CHECK(ppu.pixel_indices[0] != 0x2A && ppu.fetches_enabled);
    for (unsigned instructions = 0; instructions < 16 && read_mem(0x0010) == 0; ++instructions)
        (void)cpu_step(&cpu);
    CHECK(read_mem(0x0010) > 0);

    ppu_set_startup_write_restriction(saved_startup_restriction);
    return 0;
}

static size_t append_chunk(uint8_t *out, size_t capacity, size_t offset,
                           const char id[4], const uint8_t *payload, size_t length) {
    if (length > UINT32_MAX || offset > capacity || 8 + length > capacity - offset) return 0;
    put32(out + offset, (uint32_t)length);
    memcpy(out + offset + 4, id, 4);
    if (length) memcpy(out + offset + 8, payload, length);
    return offset + 8 + length;
}

static size_t make_nsfe(uint8_t *out, size_t capacity, const uint8_t *program,
                        size_t program_size, bool required_unknown) {
    if (capacity < 4) return 0;
    memcpy(out, "NSFE", 4);
    size_t off = 4;
    uint8_t info[10] = {0};
    put16(info, 0x8000); put16(info + 2, 0x8000); put16(info + 4, 0x8010);
    info[6] = 0; info[7] = 0; info[8] = 2; info[9] = 0;
    off = append_chunk(out, capacity, off, "INFO", info, sizeof(info));
    if (!off) return 0;
    uint8_t banks[8] = {0};
    off = append_chunk(out, capacity, off, "BANK", banks, sizeof(banks));
    uint8_t times[8], fades[8];
    put32(times, 1); put32(times + 4, 2000);
    put32(fades, 1); put32(fades + 4, 600);
    off = append_chunk(out, capacity, off, "time", times, sizeof(times));
    off = append_chunk(out, capacity, off, "fade", fades, sizeof(fades));
    static const uint8_t labels[] = {'O','n','e',0,'T','w','o',0};
    static const uint8_t auth[] = "Game\0Artist\0Copyright\0Ripper\0";
    off = append_chunk(out, capacity, off, "tlbl", labels, sizeof(labels));
    off = append_chunk(out, capacity, off, "auth", auth, sizeof(auth));
    if (required_unknown) off = append_chunk(out, capacity, off, "XBAD", NULL, 0);
    off = append_chunk(out, capacity, off, "DATA", program, program_size);
    off = append_chunk(out, capacity, off, "NEND", NULL, 0);
    return off;
}

static int test_nsfe_metadata_and_required_chunks(void) {
    cpu_use_default_startup_alignment();
    uint8_t program[0x100];
    uint8_t file[0x400];
    make_play_program(program, sizeof(program));
    size_t size = make_nsfe(file, sizeof(file), program, sizeof(program), false);
    CHECK(size != 0);
    NsfImage parsed;
    CHECK(nsf_parse_image(file, size, &parsed));
    CHECK(parsed.metadata.nsfe && parsed.metadata.total_songs == 2);
    CHECK(strcmp(parsed.metadata.track_names[0], "One") == 0);
    CHECK(strcmp(parsed.metadata.track_names[1], "Two") == 0);
    CHECK(strcmp(parsed.metadata.title, "Game") == 0);
    CHECK(strcmp(parsed.metadata.artist, "Artist") == 0);
    CHECK(parsed.metadata.track_length[0] == 1 && parsed.metadata.track_fade[0] == 1);
    nsf_image_free(&parsed);
    CHECK(load_rom_memory(file, size) == 0 && power_music() == 0);
    const NsfMetadata *metadata = rom_nsf_metadata();
    CHECK(metadata && metadata->nsfe && metadata->total_songs == 2);
    CHECK(strcmp(metadata->track_names[1], "Two") == 0);
    CHECK(metadata->track_length[1] == 2000 && metadata->track_fade[1] == 600);
    CHECK(run_until_init(0) == 0);

    size = make_nsfe(file, sizeof(file), program, sizeof(program), true);
    CHECK(size != 0 && !nsf_parse_image(file, size, &parsed));
    return 0;
}

static int load_audio_nsf(uint8_t sound_chip) {
    uint8_t payload[0x100];
    uint8_t image[0x180];
    make_play_program(payload, sizeof(payload));
    size_t size = make_nsf(image, sizeof(image), 0, sound_chip,
                           1000, 1000, 1, payload, sizeof(payload));
    cpu_use_default_startup_alignment();
    if (!size || load_rom_memory(image, size) != 0 || !cart_nsf_active()) return -1;
    return power_music();
}

static int test_nsf_expansion_audio(void) {
    CHECK(load_audio_nsf(NSF_SOUND_MMC5) == 0);
    cart_cpu_write(0x5205, 13);
    cart_cpu_write(0x5206, 7);
    CHECK(cart_cpu_read(0x5205) == 91 && cart_cpu_read(0x5206) == 0);
    cart_cpu_write(0x5011, 0x20);
    CHECK(cart_expansion_audio() < -0.001f);

    CHECK(load_audio_nsf(NSF_SOUND_FDS) == 0);
    cart_cpu_write(0x4089, 0x80);
    for (uint16_t address = 0x4040; address <= 0x407F; ++address)
        cart_cpu_write(address, 0x3F);
    CHECK((cart_cpu_read(0x4040) & 0x3F) == 0x3F);
    cart_cpu_write(0x4089, 0);
    cart_cpu_write(0x4080, 0xBF);
    cart_cpu_write(0x4082, 0xFF);
    cart_cpu_write(0x4083, 0x0F);
    cart_clock_cpu_cycle(false);
    CHECK(cart_expansion_audio() < -0.001f);

    CHECK(load_audio_nsf(NSF_SOUND_NAMCO163) == 0);
    cart_cpu_write(0xF800, 0x80);
    cart_cpu_write(0x4800, 0xFF);
    cart_cpu_write(0xF800, 0x00);
    CHECK(cart_cpu_read(0x4800) == 0xFF);
    static const uint8_t channel_registers[][2] = {
        {0x78, 0x01}, {0x7A, 0x00}, {0x7C, 0x00}, {0x7E, 0x00}, {0x7F, 0x0F}
    };
    for (size_t i = 0; i < sizeof(channel_registers) / sizeof(channel_registers[0]); ++i) {
        cart_cpu_write(0xF800, channel_registers[i][0]);
        cart_cpu_write(0x4800, channel_registers[i][1]);
    }
    for (unsigned i = 0; i < 15; ++i) cart_clock_cpu_cycle(false);
    CHECK(cart_expansion_audio() < -0.001f);

    CHECK(load_audio_nsf(NSF_SOUND_VRC6) == 0);
    cart_cpu_write(0x9000, 0x8F);
    cart_cpu_write(0x9001, 0x00);
    cart_cpu_write(0x9002, 0x80);
    CHECK(cart_expansion_audio() < -0.001f);

    CHECK(load_audio_nsf(NSF_SOUND_SUNSOFT5B) == 0);
    cart_cpu_write(0xC000, 8);
    cart_cpu_write(0xE000, 0x0F);
    cart_cpu_write(0xC000, 7);
    cart_cpu_write(0xE000, 0x3F);
    CHECK(cart_expansion_audio() < -0.001f);

    CHECK(load_audio_nsf(NSF_SOUND_VRC7) == 0);
    cart_cpu_write(0x9010, 0x30);
    cart_cpu_write(0x9030, 0x10);
    cart_cpu_write(0x9010, 0x10);
    cart_cpu_write(0x9030, 0x80);
    cart_cpu_write(0x9010, 0x20);
    cart_cpu_write(0x9030, 0x15);
    float peak = 0.0f;
    for (unsigned i = 0; i < 10000; ++i) {
        cart_clock_cpu_cycle(false);
        float sample = fabsf(cart_expansion_audio());
        if (sample > peak) peak = sample;
    }
    CHECK(peak > 0.0001f);
    return 0;
}

static int test_nsfe_fade_and_replacement(void) {
    cpu_use_default_startup_alignment();
    uint8_t program[0x100];
    uint8_t file[0x400];
    make_play_program(program, sizeof(program));
    size_t size = make_nsfe(file, sizeof(file), program, sizeof(program), false);
    CHECK(load_rom_memory(file, size) == 0 && power_music() == 0);
    for (unsigned i = 0; i < 1500; ++i) (void)cpu_step(&cpu);
    CHECK(cart_audio_gain() == 0.0f);

    uint8_t rom[16 + 0x4000 + 0x2000] = {0};
    memcpy(rom, "NES\x1A", 4);
    rom[4] = 1; rom[5] = 1;
    CHECK(load_rom_memory(rom, sizeof(rom)) == 0);
    CHECK(!rom_is_nsf() && !cart_nsf_active());
    CHECK(cart_audio_gain() == 1.0f && cart_expansion_audio() == 0.0f);
    return 0;
}

static int test_nsf_invalid_images(void) {
    cpu_use_default_startup_alignment();
    uint8_t payload[0x20] = {0x60};
    uint8_t image[0xA0];
    NsfImage parsed = {0};
    CHECK(!nsf_parse_image(NULL, 1, &parsed));
    CHECK(!nsf_parse_image(image, sizeof(image), NULL));
    size_t size = make_nsf(image, sizeof(image), 0, 0, 1000, 1000, 1,
                           payload, sizeof(payload));
    CHECK(load_rom_memory(image, size) == 0 && power_music() == 0);
    cart_cpu_write(0x6003, 0xA5);
    uint32_t crc = rom_file_crc32();

    put16(image + 8, 0x5000);
    CHECK(load_rom_memory(image, size) == -1);
    CHECK(rom_is_nsf() && rom_file_crc32() == crc && cart_cpu_read(0x6003) == 0xA5);

    size = make_nsf(image, sizeof(image), 0, 0, 1000, 1000, 1,
                    payload, sizeof(payload));
    image[6] = 0;
    CHECK(load_rom_memory(image, size) == -1);
    CHECK(rom_is_nsf() && rom_file_crc32() == crc && cart_cpu_read(0x6003) == 0xA5);

    uint8_t broken_nsfe[12] = {'N','S','F','E'};
    put32(broken_nsfe + 4, 32);
    memcpy(broken_nsfe + 8, "INFO", 4);
    CHECK(load_rom_memory(broken_nsfe, sizeof(broken_nsfe)) == -1);
    CHECK(rom_is_nsf() && rom_file_crc32() == crc && cart_cpu_read(0x6003) == 0xA5);

    size = make_nsf(image, sizeof(image), 0x80, 0x40, 1000, 1000, 1,
                    payload, sizeof(payload));
    image[5] = 0;
    CHECK(load_rom_memory(image, size) == 0);
    const NsfMetadata *metadata = rom_nsf_metadata();
    CHECK(metadata && metadata->region_flags == 0x80 && metadata->sound_chips == 0x40);
    CHECK(cart_expansion_audio() == 0.0f);
    return 0;
}

int test_nsf_accuracy(void) {
    int failures = 0;
    failures += test_nsf_init_play_and_tracks();
    failures += test_nsf_pal_and_banking();
    failures += test_nsf_zero_reload_and_irq_mask();
    failures += test_nsf_ppu_clock_only_and_restore();
    failures += test_nsfe_metadata_and_required_chunks();
    failures += test_nsf_expansion_audio();
    failures += test_nsfe_fade_and_replacement();
    failures += test_nsf_invalid_images();
    unload_rom();
    printf("NSF/NSFe accuracy: 8 groups, %d failures\n", failures);
    return failures;
}
