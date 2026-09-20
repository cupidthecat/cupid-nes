/*
 * vs_zapper_palette_accuracy.h - VS light sensor palette regressions
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "../ui/palette_tool.h"

// Fixed sensor levels for 2C03/2C05 and the four 2C04 palette variants.
static const uint16_t vs_sensor_brightness[5][64] = {
    {306, 178, 205, 223, 218, 174, 114, 115, 104, 83,  82,  87,  141, 0,   0,   0,   519, 333, 385, 410, 390, 336,
     262, 231, 216, 191, 159, 193, 265, 0,   0,   0,   764, 531, 545, 571, 604, 568, 495, 426, 378, 352, 368, 423,
     499, 0,   0,   0,   764, 670, 676, 687, 700, 684, 655, 628, 605, 596, 604, 626, 658, 0,   0,   0},
    {684, 571, 262, 545, 265, 83,  0,   336, 764, 306, 426, 174, 218, 378, 104, 764, 531, 0,   0,   352, 658, 676,
     655, 385, 0,   423, 0,   0,   552, 237, 604, 178, 0,   670, 687, 368, 499, 141, 193, 390, 0,   115, 700, 114,
     410, 205, 495, 0,   0,   191, 519, 82,  596, 223, 628, 231, 0,   333, 87,  0,   605, 568, 216, 604},
    {0,   426, 216, 596, 604, 568, 265, 670, 262, 410, 605, 700, 764, 571, 658, 87,  0,   531, 114, 552, 193, 352,
     0,   545, 0,   604, 0,   423, 676, 104, 0,   223, 218, 655, 495, 687, 333, 0,   519, 205, 390, 0,   306, 83,
     385, 0,   378, 764, 0,   0,   368, 231, 141, 178, 336, 191, 0,   499, 115, 628, 684, 174, 82,  237},
    {390, 568, 604, 519, 87,  764, 670, 83,  178, 0,   655, 104, 336, 552, 0,   658, 545, 265, 174, 385, 191, 216,
     231, 193, 306, 223, 0,   205, 262, 114, 700, 684, 571, 0,   0,   628, 0,   426, 495, 764, 352, 218, 531, 604,
     333, 237, 0,   0,   499, 0,   596, 687, 115, 368, 378, 0,   82,  0,   676, 605, 410, 423, 0,   141},
    {216, 223, 265, 378, 0,   684, 178, 231, 519, 0,   368, 0,   655, 628, 87,  596, 568, 0,   385, 700, 0,   0,
     114, 495, 0,   193, 545, 191, 218, 0,   604, 531, 174, 82,  115, 205, 410, 390, 306, 336, 141, 552, 333, 0,
     0,   605, 237, 604, 687, 764, 104, 262, 0,   423, 764, 658, 0,   426, 571, 670, 352, 676, 499, 83}};

static const uint8_t vs_sensor_read_program[] = {0xA9, 0x01, 0x8D, 0x16, 0x40, // Strobe high.
                                                 0xA9, 0x00, 0x8D, 0x16, 0x40, // Latch the sensor and trigger.
                                                 0xA2, 0x00, 0xAD, 0x16, 0x40, 0x29, 0x01, 0x95,
                                                 0x10, 0xE8, 0xE0, 0x0A, 0xD0, 0xF4, 0x02};

static uint8_t latch_vs_sensor_on_bus(void) {
    uint8_t report = 0;
    write_mem(0x4016, 1);
    write_mem(0x4016, 0);
    for (unsigned bit = 0; bit < 8; ++bit) {
        report |= (uint8_t)((read_mem(0x4016) & 1u) << bit);
    }

    return report;
}

static int test_vs_zapper_palette_levels(void) {
    unsigned saved_radius = joypad_zapper_radius();
    CHECK(joypad_set_zapper_radius(0));
    for (uint8_t code = 0; code < 16; ++code) {
        CHECK(load_vs_program(VS_TYPE_DEFAULT, code, VS_INPUT_ZAPPER, vs_sensor_read_program,
                              sizeof(vs_sensor_read_program)) == 0);
        power_main();
        unsigned palette = code >= 2 && code <= 5 ? code - 1u : 0;
        for (uint8_t color = 0; color < 64; ++color) {
            vs_sensor_pixel(32, 20, color);
            uint16_t expected_level = vs_sensor_brightness[palette][color];
            CHECK(ppu.pixel_indices[20 * 256 + 32] == color);
            CHECK(ppu_pixel_brightness(32, 20) == expected_level);

            // Grayscale selects a palette column before the hardware remap.
            ppu.mask = 1;
            CHECK(ppu_pixel_brightness(32, 20) == vs_sensor_brightness[palette][color & 0x30]);
            ppu.mask = 0xE0;
            CHECK(ppu_pixel_brightness(32, 20) == expected_level);
            ppu.mask = 0;
            bool trigger = (color & 1u) != 0;
            CHECK(joypad_set_zapper(0, 32, 20, trigger));
            uint8_t expected_report = (uint8_t)(0x10 | (expected_level >= 85 ? 0x40 : 0) | (trigger ? 0x80 : 0));

            cpu.pc = 0x8000;
            cpu.halted = false;
            memset(ram + 0x10, 0xFF, 10);
            for (unsigned instruction = 0; instruction < 100 && !cpu.halted; ++instruction) {
                cpu_step(&cpu);
            }

            CHECK(cpu.halted);
            for (unsigned bit = 0; bit < 10; ++bit) {
                CHECK(ram[0x10 + bit] == (bit < 8 ? ((expected_report >> bit) & 1u) : 0));
            }
        }

        CHECK(unload_rom());
    }

    CHECK(joypad_set_zapper_radius(saved_radius));
    CHECK(ppu_pixel_brightness(256, 0) == 0 && ppu_pixel_brightness(0, 240) == 0);
    return 0;
}

static int test_vs_zapper_palette_beam(void) {
    static const uint8_t loop[] = {0x4C, 0x00, 0x80};
    unsigned saved_radius = joypad_zapper_radius();
    CHECK(load_vs_program(VS_TYPE_DEFAULT, 2, VS_INPUT_ZAPPER, loop, sizeof(loop)) == 0);
    power_main();
    CHECK(joypad_set_zapper_radius(0));
    CHECK(joypad_set_zapper(0, 40, 30, true));
    vs_sensor_pixel(40, 30, 0x0D); // Bright on this PPU, black on a home console.
    CHECK(latch_vs_sensor_on_bus() == 0xD0);
    ppu.dot = 41;
    CHECK(latch_vs_sensor_on_bus() == 0x90);
    ppu.dot = 42;
    CHECK(latch_vs_sensor_on_bus() == 0xD0);
    ppu.scanline = 29;
    CHECK(latch_vs_sensor_on_bus() == 0x90);
    ppu.scanline = 50;
    CHECK(latch_vs_sensor_on_bus() == 0xD0);
    ppu.scanline = 51;
    CHECK(latch_vs_sensor_on_bus() == 0x90);

    vs_sensor_pixel(40, 30, 0x33); // Level 82 stays below the threshold of 85.
    CHECK(latch_vs_sensor_on_bus() == 0x90);
    vs_sensor_pixel(40, 30, 0x3A); // Level 87 lies just above that threshold.
    CHECK(latch_vs_sensor_on_bus() == 0xD0);
    vs_sensor_pixel(39, 30, 0x33);
    ppu.dot = 42;
    CHECK(joypad_set_zapper(0, 39, 30, false));
    CHECK(latch_vs_sensor_on_bus() == 0x10);
    CHECK(joypad_set_zapper_radius(1));
    CHECK(latch_vs_sensor_on_bus() == 0x50);
    CHECK(joypad_set_zapper(0, -1, 30, false));
    CHECK(latch_vs_sensor_on_bus() == 0x10);

    CHECK(joypad_set_zapper_radius(saved_radius));
    CHECK(unload_rom());
    return 0;
}

static int test_vs_zapper_palette_isolation(void) {
    static const uint8_t loop[] = {0x4C, 0x00, 0x80};
    uint32_t saved_palette[64];
    uint32_t saved_emphasis[8][64];
    bool saved_have_emphasis = ppu__have_emphasis_tables;
    memcpy(saved_palette, ppu__active_palette_base, sizeof(saved_palette));
    memcpy(saved_emphasis, ppu__emphasis_palettes, sizeof(saved_emphasis));
    CHECK(load_vs_program(VS_TYPE_DEFAULT, 0, VS_INPUT_ZAPPER, loop, sizeof(loop)) == 0);
    power_main();
    CHECK(joypad_set_zapper(0, 32, 20, false));
    vs_sensor_pixel(32, 20, 0x2D);
    CHECK(ppu_pixel_brightness(32, 20) == 0);

    // Display tables and framebuffer edits must not turn the sensor on.
    for (unsigned color = 0; color < 64; ++color) {
        ppu__active_palette_base[color] = 0xFFFFFFFFu;
        for (unsigned emphasis = 0; emphasis < 8; ++emphasis) {
            ppu__emphasis_palettes[emphasis][color] = 0xFFFFFFFFu;
        }
    }

    ppu__have_emphasis_tables = true;
    framebuffer[20 * 256 + 32] = 0xFFFFFFFFu;
    ppu.mask = 0xE0;
    CHECK(ppu_pixel_brightness(32, 20) == 0);
    CHECK(latch_vs_sensor_on_bus() == 0x10);

    iNESHeader invalid = nes20_vs_header(0, 2, 1, VS_TYPE_DEFAULT, 2, VS_INPUT_ZAPPER);
    CHECK(load_rom_memory((const uint8_t *)&invalid, sizeof(invalid)) == -1);
    CHECK(vs_enabled() && vs_ppu_model() == VS_PPU_2C03);
    CHECK(ppu_pixel_brightness(32, 20) == 0);
    CHECK(unload_rom());
    CHECK(!vs_enabled());
    vs_sensor_pixel(32, 20, 0x2D);
    CHECK(ppu_pixel_brightness(32, 20) == 237);

    memcpy(ppu__active_palette_base, saved_palette, sizeof(saved_palette));
    memcpy(ppu__emphasis_palettes, saved_emphasis, sizeof(saved_emphasis));
    ppu__have_emphasis_tables = saved_have_emphasis;
    return 0;
}
