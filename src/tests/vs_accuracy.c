/*
 * vs_accuracy.c - VS System hardware regression tests
 *
 * Author: @frankischilling
 *
 * These tests load synthetic VS cartridges through the production loader and
 * exercise PPU variants, cabinet inputs, protection hardware, mapper 99 and
 * dual-system execution through the real CPU bus.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../joypad/joypad.h"
#include "../ppu/ppu.h"
#include "../rom/mapper.h"
#include "../rom/rom.h"
#include "../system/vs_system.h"
#include "../../include/globals.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern uint8_t ram[0x800];

#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #expr); \
        return 1; \
    } \
} while (0)

static iNESHeader nes20_vs_header(unsigned mapper, uint8_t prg16, uint8_t chr8,
                                  VsSystemType type, uint8_t ppu_code, uint8_t input) {
    iNESHeader h;
    memset(&h, 0, sizeof(h));
    memcpy(h.signature, "NES\x1A", 4);
    h.prg_rom_chunks = prg16;
    h.chr_rom_chunks = chr8;
    h.flags6 = (uint8_t)((mapper & 0x0Fu) << 4);
    h.flags7 = (uint8_t)((mapper & 0xF0u) | 0x09u);
    h.prg_ram_size = (uint8_t)((mapper >> 8) & 0x0Fu);
    h.zero[2] = (uint8_t)(((unsigned)type << 4) | ppu_code);
    h.zero[4] = input;
    return h;
}

static iNESHeader legacy_vs99_header(void) {
    iNESHeader h;
    memset(&h, 0, sizeof(h));
    memcpy(h.signature, "NES\x1A", 4);
    h.prg_rom_chunks = 4;
    h.chr_rom_chunks = 4;
    h.flags6 = 0x30;
    h.flags7 = 0x61;
    return h;
}

static uint8_t *build_image(const iNESHeader *header, size_t prg_size,
                            size_t chr_size, size_t *image_size) {
    if (!header || !image_size) return NULL;
    if (prg_size > SIZE_MAX - sizeof(*header)
        || chr_size > SIZE_MAX - sizeof(*header) - prg_size) return NULL;
    *image_size = sizeof(*header) + prg_size + chr_size;
    uint8_t *image = (uint8_t *)calloc(1, *image_size);
    if (!image) return NULL;
    memcpy(image, header, sizeof(*header));
    return image;
}

static void set_vector(uint8_t *prg, size_t offset, uint16_t vector) {
    prg[offset] = (uint8_t)vector;
    prg[offset + 1] = (uint8_t)(vector >> 8);
}

static int load_vs_program(VsSystemType type, uint8_t ppu_code, uint8_t input,
                           const uint8_t *program, size_t program_size) {
    iNESHeader h = nes20_vs_header(0, 2, 1, type, ppu_code, input);
    size_t image_size;
    uint8_t *image = build_image(&h, 0x8000, 0x2000, &image_size);
    if (!image) return -1;
    uint8_t *prg = image + sizeof(h);
    memset(prg, 0xEA, 0x8000);
    if (program_size > 0x7000) {
        free(image);
        return -1;
    }
    memcpy(prg, program, program_size);
    set_vector(prg, 0x7FFA, 0x8000);
    set_vector(prg, 0x7FFC, 0x8000);
    set_vector(prg, 0x7FFE, 0x8000);
    int result = load_rom_memory(image, image_size);
    free(image);
    return result;
}

static void power_main(void) {
    memset(ram, 0, 0x800);
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    cpu_power_on(&cpu);
}

static int test_vs_metadata_transaction(void) {
    static const uint8_t loop[] = {0x4C, 0x00, 0x80};
    unload_rom();
    CHECK(load_vs_program(VS_TYPE_DEFAULT, 0, VS_INPUT_STANDARD, loop, sizeof(loop)) == 0);
    CHECK(vs_enabled() && !vs_dual_system() && vs_ppu_model() == VS_PPU_2C03);
    Mapper *previous = cart;
    uint8_t *previous_prg = prg_rom;

    iNESHeader invalid = nes20_vs_header(0, 2, 1, VS_TYPE_DEFAULT, 13, VS_INPUT_STANDARD);
    size_t image_size;
    uint8_t *image = build_image(&invalid, 0x8000, 0x2000, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == -1);
    free(image);
    CHECK(cart == previous && prg_rom == previous_prg && vs_enabled());

    invalid = nes20_vs_header(3, 2, 1, VS_TYPE_DEFAULT, 0, VS_INPUT_STANDARD);
    image = build_image(&invalid, 0x8000, 0x2000, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == -1);
    free(image);
    CHECK(cart == previous && prg_rom == previous_prg && vs_enabled());

    iNESHeader archaic;
    memset(&archaic, 0, sizeof(archaic));
    memcpy(archaic.signature, "NES\x1A", 4);
    archaic.prg_rom_chunks = 2;
    archaic.chr_rom_chunks = 1;
    archaic.flags7 = 0x06;
    image = build_image(&archaic, 0x8000, 0x2000, &image_size);
    CHECK(image != NULL);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    CHECK(!vs_enabled());
    unload_rom();
    CHECK(!vs_enabled());
    return 0;
}

static int test_vs_ppu_models(void) {
    static const uint8_t program[] = {
        0xA9,0x18, 0x8D,0x00,0x20,
        0xA9,0x80, 0x8D,0x01,0x20,
        0xAD,0x02,0x20, 0x85,0x00,
        0x4C,0x0F,0x80
    };
    CHECK(load_vs_program(VS_TYPE_DEFAULT, 9, VS_INPUT_STANDARD, program, sizeof(program)) == 0);
    power_main();
    for (unsigned i = 0; i < 6; ++i) cpu_step(&cpu);
    CHECK(ppu.mask == 0x18 && ppu.ctrl == 0x80);
    CHECK(ram[0] == 0x3D);
    CHECK(get_color(1) == 0xFF002491u);
    unload_rom();

    static const uint8_t loop[] = {0x4C,0x00,0x80};
    CHECK(load_vs_program(VS_TYPE_DEFAULT, 2, VS_INPUT_STANDARD, loop, sizeof(loop)) == 0);
    power_main();
    CHECK(get_color(0) == 0xFFFFB6B6u);
    CHECK(get_color(6) == 0xFF484848u);
    CHECK(get_color(17) == 0xFFDAB66Du);
    unload_rom();
    return 0;
}

static int test_vs_inputs_and_protection(void) {
    static const uint8_t input_program[] = {
        0xA9,0x01, 0x8D,0x16,0x40,
        0xA9,0x00, 0x8D,0x16,0x40,
        0xAD,0x16,0x40, 0x85,0x00,
        0xAD,0x17,0x40, 0x85,0x01,
        0x4C,0x16,0x80
    };
    CHECK(load_vs_program(VS_TYPE_DEFAULT, 0, VS_INPUT_SWAPPED,
                          input_program, sizeof(input_program)) == 0);
    CHECK(vs_set_dip_switches(0x00A5));
    CHECK(vs_set_coin(0, true));
    CHECK(vs_set_service(0, true));
    CHECK(joypad_set_player(0, BTN_A, false));
    CHECK(joypad_set_player(1, BTN_A, true));
    power_main();
    for (unsigned i = 0; i < 8; ++i) cpu_step(&cpu);
    CHECK(ram[0] == 0x2D);
    CHECK(ram[1] == 0xA4);
    joypad_set_player(1, BTN_A, false);
    unload_rom();

    static const uint8_t protection_program[] = {
        0xAD,0x00,0x5E,
        0xAD,0x01,0x5E, 0x85,0x00,
        0xAD,0x01,0x5E, 0x85,0x01,
        0x4C,0x0C,0x80
    };
    CHECK(load_vs_program(VS_TYPE_TKO_BOXING, 0, VS_INPUT_STANDARD,
                          protection_program, sizeof(protection_program)) == 0);
    power_main();
    for (unsigned i = 0; i < 6; ++i) cpu_step(&cpu);
    CHECK(ram[0] == 0xFF && ram[1] == 0xBF);
    unload_rom();

    static const uint8_t loop[] = {0x4C,0x00,0x80};
    CHECK(load_vs_program(VS_TYPE_RBI_BASEBALL, 0, VS_INPUT_STANDARD, loop, sizeof(loop)) == 0);
    power_main();
    (void)read_mem(0x5E00);
    for (unsigned i = 0; i < 4; ++i) (void)read_mem(0x5E01);
    CHECK(read_mem(0x5E01) == 0xB4);
    unload_rom();

    static const uint8_t xevious_program[] = {
        0xAD,0x00,0x5E, 0x85,0x00,
        0xAD,0x01,0x5E, 0x85,0x01,
        0xAD,0x20,0x40, 0x85,0x02,
        0xAD,0x00,0x50, 0x85,0x03,
        0x4C,0x14,0x80
    };
    CHECK(load_vs_program(VS_TYPE_SUPER_XEVIOUS, 0, VS_INPUT_STANDARD,
                          xevious_program, sizeof(xevious_program)) == 0);
    power_main();
    for (unsigned i = 0; i < 8; ++i) cpu_step(&cpu);
    CHECK(ram[0] == 0x00); // $5E00 resets the protection sequence.
    CHECK(ram[1] == 0x00); // $5E01 is a fixed zero and does not advance Xevious protection.
    CHECK(ram[2] == 0x05 && ram[3] == 0x01);
    unload_rom();

    static const uint8_t open_window_program[] = {
        0xA9,0xA5,
        0xAD,0x20,0x40, 0x85,0x00,
        0xAD,0xFF,0x5F, 0x85,0x01,
        0x4C,0x0C,0x80
    };
    CHECK(load_vs_program(VS_TYPE_DEFAULT, 0, VS_INPUT_STANDARD,
                          open_window_program, sizeof(open_window_program)) == 0);
    power_main();
    for (unsigned i = 0; i < 6; ++i) cpu_step(&cpu);
    CHECK(ram[0] == 0x00 && ram[1] == 0x00);
    unload_rom();
    return 0;
}

static int test_mapper99_banks(void) {
    iNESHeader h = nes20_vs_header(99, 0x36, 2, VS_TYPE_DEFAULT, 0, VS_INPUT_STANDARD);
    h.flags9 = 0x0F; // Exponent/multiplier encoding: 5 * 8192 = 40 KiB PRG.
    h.flags10 = 5;   // 2 KiB work RAM.
    size_t image_size;
    uint8_t *image = build_image(&h, 0xA000, 0x4000, &image_size);
    CHECK(image != NULL);
    uint8_t *prg = image + sizeof(h);
    uint8_t *chr = prg + 0xA000;
    for (unsigned bank = 0; bank < 5; ++bank)
        memset(prg + bank * 0x2000, (int)bank, 0x2000);
    memset(chr, 0x10, 0x2000);
    memset(chr + 0x2000, 0x11, 0x2000);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    CHECK(rom_mapper_number(&ines_header) == 99 && !vs_dual_system());
    CHECK(cart_cpu_read(0x8000) == 0 && cart_ppu_read(0) == 0x10);
    write_mem(0x4016, 0x04);
    CHECK(cart_cpu_read(0x8000) == 4 && cart_ppu_read(0) == 0x11);
    write_mem(0x4016, 0x00);
    CHECK(cart_cpu_read(0x8000) == 0 && cart_ppu_read(0) == 0x10);
    unload_rom();

    h = legacy_vs99_header();
    h.prg_rom_chunks = 3;
    h.chr_rom_chunks = 2;
    image = build_image(&h, 0xC000, 0x4000, &image_size);
    CHECK(image != NULL);
    prg = image + sizeof(h);
    chr = prg + 0xC000;
    for (unsigned bank = 0; bank < 6; ++bank)
        memset(prg + bank * 0x2000, (int)bank, 0x2000);
    memset(chr, 0x20, 0x2000);
    memset(chr + 0x2000, 0x21, 0x2000);
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    CHECK(!vs_dual_system());
    CHECK(cart_cpu_read(0x8000) == 0 && cart_ppu_read(0) == 0x20);
    write_mem(0x4016, 0x04);
    CHECK(cart_cpu_read(0x8000) == 4 && cart_ppu_read(0) == 0x21);
    unload_rom();
    return 0;
}

static int test_vs_dual_execution(void) {
    iNESHeader h = legacy_vs99_header();
    size_t image_size;
    uint8_t *image = build_image(&h, 0x10000, 0x8000, &image_size);
    CHECK(image != NULL);
    uint8_t *prg = image + sizeof(h);
    memset(prg, 0xEA, 0x10000);

    static const uint8_t main_program[] = {
        0xA9,0x00, 0x8D,0x16,0x40,
        0xA2,0xFF, 0xCA, 0xD0,0xFD,
        0xA9,0x02, 0x8D,0x16,0x40,
        0xAD,0x00,0x60, 0x85,0x00,
        0xAD,0x01,0x60, 0x85,0x01,
        0xAD,0x02,0x60, 0x85,0x02,
        0xAD,0x03,0x60, 0x85,0x03,
        0x4C,0x23,0x80
    };
    static const uint8_t sub_program[] = {
        0x58,
        0xA9,0x66, 0x8D,0x00,0x02,
        0xA9,0x00, 0x8D,0x03,0x20,
        0xA9,0x02, 0x8D,0x14,0x40,
        0xAD,0x04,0x20, 0x8D,0x01,0x60,
        0xA9,0x01, 0x8D,0x15,0x40,
        0xA9,0x30, 0x8D,0x00,0x40,
        0xA9,0x08, 0x8D,0x02,0x40,
        0xA9,0x08, 0x8D,0x03,0x40,
        0xAD,0x15,0x40, 0x29,0x01, 0x8D,0x02,0x60,
        0xA9,0x01, 0x8D,0x16,0x40,
        0xA9,0x00, 0x8D,0x16,0x40,
        0xAD,0x16,0x40, 0x29,0x01, 0x8D,0x03,0x60,
        0x4C,0x44,0x80
    };
    static const uint8_t irq_handler[] = {
        0xAA,
        0x68, 0x09,0x04, 0x48,
        0xA9,0x5A, 0x8D,0x00,0x60,
        0x8A,
        0x40
    };
    memcpy(prg, main_program, sizeof(main_program));
    memcpy(prg + 4 * 0x2000, sub_program, sizeof(sub_program));
    memcpy(prg + 4 * 0x2000 + 0x100, irq_handler, sizeof(irq_handler));
    set_vector(prg, 0x7FFA, 0x8000);
    set_vector(prg, 0x7FFC, 0x8000);
    set_vector(prg, 0x7FFE, 0x8000);
    set_vector(prg, 0xFFFA, 0x8000);
    set_vector(prg, 0xFFFC, 0x8000);
    set_vector(prg, 0xFFFE, 0x8100);

    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    CHECK(vs_dual_system() && rom_mapper_number(&ines_header) == 99);
    CHECK(joypad_set_player(2, BTN_A, true));
    power_main();
    vs_power_on_secondary();
    for (unsigned i = 0; i < 800; ++i) vs_cpu_step();
    CHECK(ram[0] == 0x5A); // Sub CPU serviced the peer IRQ and wrote shared RAM.
    CHECK(ram[1] == 0x66); // Sub OAM DMA used the sub PPU, then read the copied byte.
    CHECK(ram[2] == 0x01); // Sub APU register state is live on the sub CPU bus.
    CHECK(ram[3] == 0x01); // Players 3/4 are routed to the sub controller ports.
    CHECK(ppu.oam[0] == 0xFF); // Sub DMA did not overwrite the main PPU OAM.
    CHECK(!apu.pulse1.enabled); // Sub APU writes did not alter the main APU instance.
    uint64_t main_cycles = vs_side_cpu_cycles(0);
    uint64_t sub_cycles = vs_side_cpu_cycles(1);
    CHECK(main_cycles > sub_cycles ? main_cycles - sub_cycles <= 5 : sub_cycles - main_cycles <= 6);

    uint64_t main_frame = vs_side_frame_count(0);
    vs_start_frame();
    for (unsigned i = 0; i < 40000 && !ppu.frame_complete; ++i) vs_cpu_step();
    CHECK(ppu.frame_complete && vs_side_frame_count(0) == main_frame + 1);
    CHECK(vs_side_frame_count(1) >= vs_side_frame_count(0));

    write_mem(0x6000, 0x00);
    vs_soft_reset();
    CHECK(!vs_shared_ram_access_allowed());
    CHECK(!vs_external_irq_pending());
    for (unsigned i = 0; i < 8; ++i) vs_cpu_step();
    write_mem(0x4016, 0x02);
    CHECK(vs_shared_ram_access_allowed());
    CHECK(cart_cpu_read(0x6000) == 0x5A); // Reset restored the sub IRQ and its initial RAM ownership.
    joypad_set_player(2, BTN_A, false);
    unload_rom();
    return 0;
}

static int test_vs_reset_and_declared_ram(void) {
    static const uint8_t loop[] = {0x4C, 0x00, 0x80};
    CHECK(load_vs_program(VS_TYPE_TKO_BOXING, 0, VS_INPUT_STANDARD, loop, sizeof(loop)) == 0);
    power_main();
    (void)read_mem(0x5E00);
    CHECK(read_mem(0x5E01) == 0xFF && read_mem(0x5E01) == 0xBF);
    ppu_soft_reset(&ppu);
    apu_soft_reset(&apu);
    cpu_soft_reset(&cpu);
    vs_soft_reset();
    CHECK(read_mem(0x5E01) == 0xFF);
    CHECK(unload_rom());

    iNESHeader h = nes20_vs_header(99, 2, 1, VS_TYPE_DEFAULT, 0, VS_INPUT_STANDARD);
    size_t image_size;
    uint8_t *image = build_image(&h, 0x8000, 0x2000, &image_size);
    CHECK(image != NULL && load_rom_memory(image, image_size) == 0);
    cart_cpu_write(0x6000, 0x39);
    CHECK(cart_cpu_read_bus(0x6000, 0xA7) == 0xA7);
    CHECK(cart_cpu_read_bus(0x6000, 0x5C) == 0x5C);

    h.flags10 = 6; // Explicit 4 KiB work RAM must not become the 2 KiB legacy default.
    memcpy(image, &h, sizeof(h));
    CHECK(load_rom_memory(image, image_size) == 0);
    cart_cpu_write(0x6000, 0x24);
    cart_cpu_write(0x6800, 0x68);
    CHECK(cart_cpu_read(0x6000) == 0x24 && cart_cpu_read(0x6800) == 0x68);
    CHECK(cart_cpu_read(0x7000) == 0x24 && cart_cpu_read(0x7800) == 0x68);
    Mapper *previous = cart;
    uint8_t *previous_prg = prg_rom;
    h.flags10 = 8; // This board implementation cannot address more than 8 KiB.
    memcpy(image, &h, sizeof(h));
    CHECK(load_rom_memory(image, image_size) == -1);
    CHECK(cart == previous && prg_rom == previous_prg && vs_enabled());
    CHECK(cart_cpu_read(0x6000) == 0x24 && cart_cpu_read(0x6800) == 0x68);
    h.flags6 |= 2;
    h.flags10 = 0x55; // Separate work and save chips are not silently collapsed.
    memcpy(image, &h, sizeof(h));
    CHECK(load_rom_memory(image, image_size) == -1);
    CHECK(cart == previous && prg_rom == previous_prg);
    free(image);
    CHECK(unload_rom());
    return 0;
}

static int test_vs_dual_video_and_audio(void) {
    iNESHeader h = legacy_vs99_header();
    size_t image_size;
    uint8_t *image = build_image(&h, 0x10000, 0x8000, &image_size);
    CHECK(image != NULL);
    uint8_t *prg = image + sizeof(h);
    memset(prg, 0xEA, 0x10000);
    static const uint8_t paint[] = {
        0xA9,0x3F, 0x8D,0x06,0x20,
        0xA9,0x00, 0x8D,0x06,0x20,
        0xA9,0x01, 0x8D,0x07,0x20,
        0xA9,0x00, 0x8D,0x06,0x20, 0x8D,0x06,0x20
    };
    static const uint8_t pulse[] = {
        0xA9,0x01, 0x8D,0x15,0x40,
        0xA9,0xBF, 0x8D,0x00,0x40,
        0xA9,0x40, 0x8D,0x02,0x40,
        0xA9,0x00, 0x8D,0x03,0x40,
        0x4C,0x2B,0x80
    };
    memcpy(prg, paint, sizeof(paint));
    prg[23] = 0x4C; prg[24] = 0x17; prg[25] = 0x80;
    memcpy(prg + 0x8000, paint, sizeof(paint));
    prg[0x800B] = 2;
    memcpy(prg + 0x8000 + sizeof(paint), pulse, sizeof(pulse));
    for (size_t side = 0; side < 2; ++side) {
        size_t base = side * 0x8000;
        set_vector(prg, base + 0x7FFA, 0x8000);
        set_vector(prg, base + 0x7FFC, 0x8000);
        set_vector(prg, base + 0x7FFE, 0x8000);
    }
    CHECK(load_rom_memory(image, image_size) == 0);
    free(image);
    power_main();
    vs_power_on_secondary();
    vs_audio_init(48000);
    CHECK(vs_side_apu(0) == &apu && vs_side_apu(1) != NULL && vs_side_apu(2) == NULL);
    CHECK(vs_side_apu(0)->sample_rate == 48000 && vs_side_apu(1)->sample_rate == 48000);
    for (unsigned frame = 0; frame < 2; ++frame) {
        vs_start_frame();
        for (unsigned i = 0; i < 40000 && !ppu.frame_complete; ++i) vs_cpu_step();
        CHECK(ppu.frame_complete);
    }
    CHECK(vs_video_width() == 2 * SCREEN_WIDTH);
    const uint32_t *pixels = vs_video_framebuffer();
    size_t pixel = 100 * vs_video_width() + 100;
    CHECK(pixels[pixel] == 0xFF002491u);
    CHECK(pixels[pixel + SCREEN_WIDTH] == 0xFF0000DAu);
    CHECK(!apu.pulse1.enabled && vs_side_apu(1)->pulse1.enabled);

    uint64_t main_cycles = vs_side_cpu_cycles(0), sub_cycles = vs_side_cpu_cycles(1);
    float samples[513];
    vs_audio_callback(NULL, (uint8_t *)samples, sizeof(samples));
    bool audible = false;
    for (size_t i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        CHECK(isfinite(samples[i]));
        if (fabsf(samples[i]) > 0.000001f) audible = true;
    }
    CHECK(audible); // Only the secondary cabinet can supply this sound.
    CHECK(vs_side_cpu_cycles(0) == main_cycles && vs_side_cpu_cycles(1) == sub_cycles);
    CHECK(vs_active_side() == 0);
    CHECK(atomic_load(&vs_side_apu(0)->ring_r) == 513);
    CHECK(atomic_load(&vs_side_apu(1)->ring_r) == 513);
    CHECK(unload_rom());
    CHECK(vs_video_width() == SCREEN_WIDTH && vs_video_framebuffer() == framebuffer);
    return 0;
}

int test_vs_accuracy(void) {
    static int (*const tests[])(void) = {
        test_vs_metadata_transaction,
        test_vs_ppu_models,
        test_vs_inputs_and_protection,
        test_mapper99_banks,
        test_vs_dual_execution,
        test_vs_reset_and_declared_ram,
        test_vs_dual_video_and_audio
    };
    int failures = 0;
    for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) failures += tests[i]();
    unload_rom();
    printf("VS System accuracy: %zu groups, %d failures\n",
           sizeof(tests) / sizeof(tests[0]), failures);
    return failures;
}
