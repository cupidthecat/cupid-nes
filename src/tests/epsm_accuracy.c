/*
 * epsm_accuracy.c - EPSM loading, bus, timing, IRQ, and audio regressions
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator, licensed under the GNU General
 * Public License, version 3 or any later version.
 */
#include "../apu/epsm.h"
#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../ppu/ppu.h"
#include "../rom/rom.h"
#include "../rom/mapper.h"
#include "../system/hardware.h"
#include "../system/timing.h"
#include "../system/vs_system.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

extern uint8_t ram[0x800];

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); return 1; \
} } while (0)

enum { IMAGE_BYTES = 16 + 0x4000 + 0x2000, AUDIO_FRAMES = 1024 };

static uint8_t *epsm_image(NesRegion region) {
    uint8_t *image = calloc(1, IMAGE_BYTES);
    if (!image) return NULL;
    memcpy(image, "NES\x1A", 4);
    image[4] = 1;
    image[5] = 1;
    image[7] = 0x0B;
    image[12] = region == NES_REGION_PAL ? 1 : region == NES_REGION_DENDY ? 3 : 0;
    image[13] = 4;
    image[16] = 0x4C;
    image[17] = 0;
    image[18] = 0x80;
    image[16 + 0x3FFA] = 0;
    image[16 + 0x3FFB] = 0x80;
    image[16 + 0x3FFC] = 0;
    image[16 + 0x3FFD] = 0x80;
    image[16 + 0x3FFE] = 0;
    image[16 + 0x3FFF] = 3;
    return image;
}

static int load_epsm(NesRegion region) {
    uint8_t *image = epsm_image(region);
    if (!image) return -1;
    int result = load_rom_memory(image, IMAGE_BYTES);
    free(image);
    if (result != 0) return result;
    cpu_use_default_startup_alignment();
    cpu_set_test_mode(false);
    memset(ram, 0, 0x800);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    return cpu_power_on(&cpu) ? 0 : -1;
}

static void reg_write(unsigned bank, uint8_t reg, uint8_t value) {
    uint16_t port = bank ? 0x401E : 0x401C;
    write_mem(port, reg);
    write_mem((uint16_t)(port + 1), value);
}

static void protocol_write(unsigned port, uint8_t value) {
    uint8_t select = (uint8_t)(((port & 2u) << 1) | ((port & 1u) << 3));
    write_mem(0x4016, (uint8_t)((value & 0xF0u) | select | 2u));
    write_mem(0x4016, (uint8_t)((value << 4) | select));
}

static void protocol_reg(unsigned bank, uint8_t reg, uint8_t value) {
    protocol_write(bank ? 2 : 0, reg);
    protocol_write(bank ? 3 : 1, value);
}

static void ssg_tone(void (*write_register)(unsigned, uint8_t, uint8_t), uint8_t volume) {
    write_register(0, 0, 32);
    write_register(0, 1, 0);
    write_register(0, 7, 0x3E);
    write_register(0, 8, volume);
}

static void fm_tone(void (*write_register)(unsigned, uint8_t, uint8_t),
                    unsigned bank, uint8_t pan) {
    write_register(0, 0x29, 0x83); // Enable all six FM channels and both timer IRQs.
    for (unsigned op = 0; op < 4; ++op) {
        uint8_t offset = (uint8_t)(op * 4);
        write_register(bank, (uint8_t)(0x30 + offset), 1);
        write_register(bank, (uint8_t)(0x40 + offset), 0x18);
        write_register(bank, (uint8_t)(0x50 + offset), 0x1F);
        write_register(bank, (uint8_t)(0x60 + offset), 0);
        write_register(bank, (uint8_t)(0x70 + offset), 0);
        write_register(bank, (uint8_t)(0x80 + offset), 0x0F);
    }
    write_register(bank, 0xB0, 7);
    write_register(bank, 0xB4, pan);
    write_register(bank, 0xA4, 0x22);
    write_register(bank, 0xA0, 0x69);
    write_register(0, 0x28, (uint8_t)(0xF0 | (bank ? 4 : 0)));
}

static int capture_audio(float *output, int frames) {
    apu_audio_init_state(&apu, 44100);
    uint64_t start = cpu_total_cycles;
    while (((atomic_load(&apu.ring_w) - atomic_load(&apu.ring_r)) & (APU_RING_CAP - 1u))
           < (unsigned)frames) {
        CHECK(cpu_step(&cpu) > 0 && !cpu.halted);
        CHECK(cpu_total_cycles - start < 1000000);
    }
    apu_sdl_stereo_callback(NULL, (uint8_t *)output, (int)(frames * 2 * sizeof(float)));
    for (int i = 0; i < frames * 2; ++i) CHECK(isfinite(output[i]));
    return 0;
}

static double channel_energy(const float *samples, unsigned channel, unsigned frames) {
    double energy = 0;
    for (unsigned frame = 0; frame < frames; ++frame) {
        double sample = samples[2 * frame + channel];
        energy += sample * sample;
    }
    return energy;
}

static int test_epsm_metadata_and_failure_preservation(void) {
    CHECK(epsm_set_adpcm_rom(NULL, 0));
    for (NesRegion region = NES_REGION_NTSC; region <= NES_REGION_DENDY; ++region) {
        CHECK(load_epsm(region) == 0);
        CHECK(epsm_enabled() && !epsm_has_adpcm_rom() && !vs_enabled());
        CHECK(nes_timing()->region == region && cpu.pc == 0x8000);
    }
    uint8_t *image = epsm_image(NES_REGION_PAL);
    CHECK(image != NULL);
    Mapper *previous = cart;
    uint8_t *previous_prg = prg_rom;
    uint64_t before = epsm_clock_count();
    CHECK(load_rom_memory(image, IMAGE_BYTES - 1) == -1);
    CHECK(epsm_enabled() && epsm_clock_count() == before && prg_rom == previous_prg);
    image[6] = 0xF0;
    image[7] |= 0xF0;
    image[8] = 0x0F; // Synthetic unsupported mapper 4095.
    CHECK(load_rom_memory(image, IMAGE_BYTES) == -1);
    CHECK(cart == previous && prg_rom == previous_prg && epsm_enabled());
    CHECK(epsm_clock_count() == before && nes_timing()->region == NES_REGION_DENDY);
    image[6] = 0;
    image[7] &= 0x0F;
    image[8] = 0;
    image[13] = 5;
    CHECK(load_rom_memory(image, IMAGE_BYTES) == -1);
    CHECK(cart == previous && epsm_enabled() && epsm_clock_count() == before);
    image[7] = 8;
    image[13] = 4; // Byte 13 alone does not enable hardware on an ordinary NES.
    CHECK(load_rom_memory(image, IMAGE_BYTES) == 0);
    CHECK(!epsm_enabled());
    image[7] = 0x0B;
    CHECK(load_rom_memory(image, IMAGE_BYTES) == 0 && epsm_enabled());
    image[7] = 9;
    image[12] = 0;
    image[13] = 0;
    CHECK(load_rom_memory(image, IMAGE_BYTES) == 0);
    CHECK(vs_enabled() && !epsm_enabled());
    free(image);
    CHECK(load_epsm(NES_REGION_NTSC) == 0);
    before = epsm_clock_count();
    uint8_t *disk = calloc(1, 65500);
    CHECK(disk != NULL);
    uint8_t bios[0x2000];
    memset(bios, 0xEA, sizeof(bios));
    disk[0] = 1;
    disk[1] = 0x2A;
    disk[55] = 0x40;
    disk[56] = 2;
    CHECK(load_fds_memory(disk, 65500, bios, sizeof(bios) - 1, NULL, false) == -1);
    CHECK(epsm_enabled() && epsm_clock_count() == before);
    CHECK(load_fds_memory(disk, 65500, bios, sizeof(bios), NULL, false) == 0);
    CHECK(rom_is_fds() && !epsm_enabled());
    free(disk);
    CHECK(unload_rom());
    CHECK(!epsm_enabled() && !epsm_irq_pending());
    return 0;
}

static int test_epsm_stereo_and_bus_protocol(void) {
    float direct[AUDIO_FRAMES * 2], serial[AUDIO_FRAMES * 2];
    CHECK(load_epsm(NES_REGION_NTSC) == 0);
    ssg_tone(reg_write, 0x0F);
    fm_tone(reg_write, 0, 0x80);
    fm_tone(reg_write, 1, 0x40);
    CHECK(capture_audio(direct, AUDIO_FRAMES) == 0);
    CHECK(channel_energy(direct, 0, AUDIO_FRAMES) > 0.01);
    CHECK(channel_energy(direct, 1, AUDIO_FRAMES) > 0.01);

    CHECK(load_epsm(NES_REGION_NTSC) == 0);
    ssg_tone(protocol_reg, 0x0F);
    fm_tone(protocol_reg, 0, 0x80);
    fm_tone(protocol_reg, 1, 0x40);
    CHECK(capture_audio(serial, AUDIO_FRAMES) == 0);
    CHECK(memcmp(direct, serial, sizeof(direct)) == 0);

    CHECK(load_epsm(NES_REGION_NTSC) == 0);
    fm_tone(reg_write, 1, 0x80);
    CHECK(capture_audio(direct, AUDIO_FRAMES) == 0);
    CHECK(channel_energy(direct, 0, AUDIO_FRAMES) > 0.01);
    CHECK(channel_energy(direct, 1, AUDIO_FRAMES) == 0);
    reg_write(1, 0xB4, 0x40);
    CHECK(capture_audio(serial, AUDIO_FRAMES) == 0);
    CHECK(channel_energy(serial, 1, AUDIO_FRAMES) > 0.01);
    CHECK(channel_energy(serial + 64, 0, AUDIO_FRAMES - 32) == 0);

    // The callback keeps its own last sample when the producer underruns.
    float tail[16];
    apu_audio_init_state(&apu, 44100);
    apu.last_read_sample = 0.25f;
    apu.last_read_side = 0.125f;
    apu_sdl_stereo_callback(NULL, (uint8_t *)tail, (int)sizeof(tail));
    for (unsigned i = 0; i < 8; ++i) CHECK(tail[i * 2] == 0.375f && tail[i * 2 + 1] == 0.125f);
    CHECK(unload_rom());
    return 0;
}

static int test_epsm_delayed_out_uses_live_bus(void) {
    static const uint8_t program[] = {
        0x24, 0x00,       // BIT $00 aligns the first write to the even CPU cycle.
        0xA9, 0x02, 0x8D, 0x16, 0x40, 0xEA,
        0xA9, 0x00, 0x8D, 0x16, 0x40, 0xEA,
        0x4C, 0x0E, 0x80
    };
    float serial[AUDIO_FRAMES * 2], direct[AUDIO_FRAMES * 2];
    CHECK(load_epsm(NES_REGION_NTSC) == 0);
    memcpy(prg_rom, program, sizeof(program));
    ssg_tone(reg_write, 0);
    write_mem(0x401C, 8);
    for (unsigned instruction = 0; instruction < 7; ++instruction) CHECK(cpu_step(&cpu) > 0);
    CHECK(cpu_total_cycles == 26);
    CHECK(capture_audio(serial, AUDIO_FRAMES) == 0);

    CHECK(load_epsm(NES_REGION_NTSC) == 0);
    memcpy(prg_rom, program, sizeof(program));
    prg_rom[5] = prg_rom[11] = 0; // Use unrelated APU writes with the same CPU timing.
    ssg_tone(reg_write, 0);
    for (unsigned instruction = 0; instruction < 7; ++instruction) CHECK(cpu_step(&cpu) > 0);
    CHECK(cpu_total_cycles == 26);
    // At both delayed OUT edges, the fetched NOP opcode $EA is on the CPU bus.
    // Its high nibble supplies E/E and its address bits select the low data port.
    reg_write(0, 8, 0xEE);
    CHECK(capture_audio(direct, AUDIO_FRAMES) == 0);
    CHECK(channel_energy(direct, 0, AUDIO_FRAMES) > 0.01);
    CHECK(memcmp(serial, direct, sizeof(serial)) == 0);
    CHECK(unload_rom());
    return 0;
}

static int test_epsm_controller_and_open_bus(void) {
    CHECK(load_epsm(NES_REGION_NTSC) == 0);
    CHECK(nes_set_console_model_name("famicom"));
    CHECK(joypad_set_port_device(0, NES_PORT_GAMEPAD));
    CHECK(joypad_set_port_device(1, NES_PORT_GAMEPAD));
    CHECK(joypad_set_expansion_device(NES_EXPANSION_NONE));
    CHECK(joypad_set_adapter(NES_ADAPTER_NONE));
    joypad_player(0)->buttons = 0xA5;
    joypad_player(1)->buttons = 0x5A;
    write_mem(0x4016, 1);
    write_mem(0x4016, 0);
    uint8_t first = 0, second = 0;
    for (unsigned bit = 0; bit < 8; ++bit) {
        first |= (read_mem(0x4016) & 1u) << bit;
        second |= (read_mem(0x4017) & 1u) << bit;
    }
    CHECK(first == 0xA5 && second == 0x5A);
    for (uint16_t address = 0x4018; address <= 0x401F; ++address) {
        write_mem(0x401B, 0xA5);
        CHECK(read_mem(address) == 0xA5);
    }
    ssg_tone(reg_write, 0x0F);
    // Repeated high OUT1 writes must not commit the low data nibble.
    write_mem(0x4016, 0x02);
    for (unsigned i = 0; i < 8; ++i) write_mem(0x4016, 0xFA);
    write_mem(0x4016, 0x80); // Original rising edge selected address zero, value $08.
    write_mem(0x401D, 0);    // Volume is now zero.
    float muted[AUDIO_FRAMES * 2];
    CHECK(capture_audio(muted, AUDIO_FRAMES) == 0);
    CHECK(channel_energy(muted, 0, AUDIO_FRAMES) == 0);
    CHECK(unload_rom());
    CHECK(nes_set_console_model_name("nes-001"));
    return 0;
}

static int test_epsm_power_and_soft_reset(void) {
    float before[AUDIO_FRAMES * 2], after[AUDIO_FRAMES * 2];
    CHECK(load_epsm(NES_REGION_NTSC) == 0);
    ssg_tone(reg_write, 0x0F);
    fm_tone(reg_write, 0, 0x80);
    CHECK(capture_audio(before, AUDIO_FRAMES) == 0);
    cpu_soft_reset(&cpu);
    CHECK(capture_audio(after, AUDIO_FRAMES) == 0);
    CHECK(channel_energy(after, 0, AUDIO_FRAMES) > 0.01);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    ssg_tone(reg_write, 0x0F);
    fm_tone(reg_write, 0, 0x80);
    CHECK(capture_audio(after, AUDIO_FRAMES) == 0);
    CHECK(memcmp(before, after, sizeof(before)) == 0);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    CHECK(cpu_power_on(&cpu));
    CHECK(capture_audio(after, AUDIO_FRAMES) == 0);
    CHECK(channel_energy(after, 0, AUDIO_FRAMES) == 0);
    CHECK(channel_energy(after, 1, AUDIO_FRAMES) == 0);
    CHECK(unload_rom());
    return 0;
}

static int test_epsm_clocks_and_irq(void) {
    for (NesRegion region = NES_REGION_NTSC; region <= NES_REGION_DENDY; ++region) {
        CHECK(load_epsm(region) == 0);
        const NesTiming *timing = nes_timing();
        for (unsigned instruction = 0; instruction < 1000; ++instruction) CHECK(cpu_step(&cpu) == 3);
        uint64_t master = cpu_total_cycles * timing->cpu_divider + timing->ppu_divider;
        uint32_t frequency = (uint32_t)(timing->cpu_hz * timing->cpu_divider);
        CHECK(epsm_clock_count() == master * EPSM_CLOCK_RATE / frequency);
    }
    CHECK(load_epsm(NES_REGION_NTSC) == 0);
    epsm_power_on();
    reg_write(0, 0x24, 0xFF);
    reg_write(0, 0x25, 3);
    reg_write(0, 0x27, 5);
    epsm_clock_master(143, EPSM_CLOCK_RATE);
    CHECK(!epsm_irq_pending());
    epsm_clock_master(1, EPSM_CLOCK_RATE);
    CHECK(epsm_irq_pending());
    reg_write(0, 0x27, 0x15);
    CHECK(!epsm_irq_pending());
    epsm_clock_master(144, EPSM_CLOCK_RATE);
    CHECK(epsm_irq_pending());
    apu.frame_irq_source = true;
    reg_write(0, 0x27, 0x30);
    CHECK(!epsm_irq_pending() && apu.frame_irq_source);
    apu.frame_irq_source = false;

    epsm_power_on();
    reg_write(0, 0x26, 0xFF);
    reg_write(0, 0x27, 0x0A);
    epsm_clock_master(2303, EPSM_CLOCK_RATE);
    CHECK(!epsm_irq_pending());
    epsm_clock_master(1, EPSM_CLOCK_RATE);
    CHECK(epsm_irq_pending());
    uint64_t before = epsm_clock_count();
    cpu.status &= (uint8_t)~INTERRUPT_FLAG;
    CHECK(cpu_step(&cpu) == 10 && cpu.pc == 0x0300);
    CHECK(epsm_clock_count() > before && epsm_irq_pending());
    cpu_soft_reset(&cpu);
    CHECK(epsm_irq_pending() && cpu.pc == 0x8000);
    before = epsm_clock_count();
    nes_set_region(NES_REGION_PAL);
    CHECK(cpu_step(&cpu) == 3);
    uint64_t advanced = epsm_clock_count() - before;
    CHECK(advanced >= 14 && advanced <= 15);
    CHECK(epsm_irq_pending());
    CHECK(cpu_power_on(&cpu));
    CHECK(!epsm_irq_pending());
    CHECK(unload_rom());
    return 0;
}

static int write_firmware_fixture(const char *path, size_t bytes, uint8_t value) {
    FILE *file = fopen(path, "wb");
    if (!file) return -1;
    bool ok = true;
    for (size_t i = 0; i < bytes && ok; ++i) ok = fputc(value, file) != EOF;
    if (fclose(file) != 0) ok = false;
    return ok ? 0 : -1;
}

static void rhythm_tone(unsigned voice) {
    reg_write(0, 0x11, 0x3F);
    reg_write(0, (uint8_t)(0x18 + voice), 0xDF);
    reg_write(0, 0x10, (uint8_t)(1u << voice));
}

static int test_epsm_firmware_and_rhythm(void) {
    char path[128];
    snprintf(path, sizeof(path), "build/epsm-firmware-%llu-%lu.bin",
             (unsigned long long)time(NULL), (unsigned long)clock());
    CHECK(epsm_set_adpcm_rom(NULL, 0));
    CHECK(write_firmware_fixture(path, EPSM_ADPCM_ROM_SIZE, 0x17) == 0);
    CHECK(epsm_load_adpcm_file(path));
    CHECK(load_epsm(NES_REGION_NTSC) == 0 && epsm_has_adpcm_rom());
    CHECK(epsm_set_adpcm_rom(NULL, 0));
    CHECK(epsm_has_adpcm_rom()); // Active firmware is owned by the loaded device.
    float first[AUDIO_FRAMES * 2], second[AUDIO_FRAMES * 2];
    rhythm_tone(5); // The last voice reads through the end of the 8 KiB ROM.
    CHECK(capture_audio(first, AUDIO_FRAMES) == 0);
    CHECK(channel_energy(first, 0, AUDIO_FRAMES) > 0.001);

    CHECK(write_firmware_fixture(path, EPSM_ADPCM_ROM_SIZE, 0x71) == 0);
    CHECK(epsm_load_adpcm_file(path));
    uint64_t before = epsm_clock_count();
    CHECK(write_firmware_fixture(path, EPSM_ADPCM_ROM_SIZE - 1, 0) == 0);
    CHECK(!epsm_load_adpcm_file(path));
    CHECK(write_firmware_fixture(path, EPSM_ADPCM_ROM_SIZE + 1, 0) == 0);
    CHECK(!epsm_load_adpcm_file(path));
    CHECK(remove(path) == 0);
    CHECK(!epsm_load_adpcm_file(path) && !epsm_load_adpcm_file(NULL));
    CHECK(epsm_clock_count() == before && epsm_has_adpcm_rom());
    CHECK(load_epsm(NES_REGION_NTSC) == 0 && epsm_has_adpcm_rom());
    rhythm_tone(5);
    CHECK(capture_audio(second, AUDIO_FRAMES) == 0);
    CHECK(channel_energy(second, 0, AUDIO_FRAMES) > 0.001);
    CHECK(memcmp(first, second, sizeof(first)) != 0);
    uint8_t expected_rom[EPSM_ADPCM_ROM_SIZE];
    memset(expected_rom, 0x71, sizeof(expected_rom));
    CHECK(!epsm_set_adpcm_rom(NULL, sizeof(expected_rom)));
    CHECK(!epsm_set_adpcm_rom(expected_rom, sizeof(expected_rom) - 1));
    CHECK(epsm_set_adpcm_rom(expected_rom, sizeof(expected_rom)));
    CHECK(load_epsm(NES_REGION_NTSC) == 0);
    rhythm_tone(5);
    CHECK(capture_audio(first, AUDIO_FRAMES) == 0);
    CHECK(memcmp(first, second, sizeof(first)) == 0);
    CHECK(epsm_set_adpcm_rom(NULL, 0));
    CHECK(load_epsm(NES_REGION_NTSC) == 0 && !epsm_has_adpcm_rom());
    CHECK(unload_rom());
    return 0;
}

int test_epsm_accuracy(void) {
    static int (*const tests[])(void) = {
        test_epsm_metadata_and_failure_preservation,
        test_epsm_stereo_and_bus_protocol,
        test_epsm_delayed_out_uses_live_bus,
        test_epsm_controller_and_open_bus,
        test_epsm_power_and_soft_reset,
        test_epsm_clocks_and_irq,
        test_epsm_firmware_and_rhythm
    };
    int failures = 0;
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) failures += tests[i]();
    unload_rom();
    epsm_set_adpcm_rom(NULL, 0);
    printf("EPSM accuracy: %zu groups, %d failures\n", sizeof(tests) / sizeof(tests[0]), failures);
    return failures;
}
