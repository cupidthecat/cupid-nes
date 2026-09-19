/*
 * mmc5_extended_geometry_accuracy.c - Extended MMC5 fetch address regressions
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
#include "../apu/apu.h"
#include "../system/timing.h"
#include "../../include/globals.h"

static bool cpu_store(uint16_t address, uint8_t value) {
    write_mem(0x0200, 0xA9);
    write_mem(0x0201, value);
    write_mem(0x0202, 0x8D);
    write_mem(0x0203, (uint8_t)address);
    write_mem(0x0204, (uint8_t)(address >> 8));
    cpu.pc = 0x0200;
    return cpu_step(&cpu) == 2 && cpu_step(&cpu) == 4;
}

static void enter_frame(void) {
    (void)ppu_read(0x2000);
    (void)ppu_read(0x2000);
    (void)ppu_read(0x2000);
    (void)ppu_read(0x23C0);
    (void)ppu_read(0x2001);
}

static int test_extended_fetch_address_bits(void) {
    static const struct {
        size_t chr_bytes;
        uint8_t selected_bank;
        uint8_t expected_page;
    } cases[] = {
        {0x2000, 3, 4},
        {0x3000, 1, 0}, {0x3000, 3, 8}, {0x3000, 7, 8},
        {0x5000, 3, 0}, {0x5000, 5, 16}, {0x5000, 7, 16},
        {0x7000, 3, 8}, {0x7000, 5, 16}, {0x7000, 7, 24}
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, 5, 0x8000, cases[i].chr_bytes, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        apu_power_on(&apu);
        BOARD_CHECK(cpu_store(0x5104, 2));
        BOARD_CHECK(cpu_store(0x5C05, (uint8_t)(0x80 | cases[i].selected_bank)));
        BOARD_CHECK(cpu_store(0x5104, 1));
        ppu_vram[5] = 0x2A;
        enter_frame();
        BOARD_CHECK(ppu_read(0x2005) == 0x2A);
        BOARD_CHECK(ppu_read(0x23C0) == 0xAA);
        BOARD_CHECK(ppu_read(0x0123) == cases[i].expected_page);
        BOARD_CHECK(ppu_read(0x012B) == cases[i].expected_page);
        // The third pattern read returns to ordinary 1 KiB bank selection.
        BOARD_CHECK(ppu_read(0x0123) == 0);
        ppu_write(0x0123, 0xEE);
        BOARD_CHECK(ppu_read(0x2005) == 0x2A);
        BOARD_CHECK(ppu_read(0x23C0) == 0xAA);
        BOARD_CHECK(ppu_read(0x0123) == cases[i].expected_page);
        BOARD_CHECK(ppu_read(0x012B) == cases[i].expected_page);
        board_image_free(&image);
    }
    return 0;
}

static int test_split_fetch_address_bits(void) {
    static const struct {
        size_t chr_bytes;
        uint8_t expected_page;
    } cases[] = {{0x2000, 4}, {0x3000, 8}, {0x5000, 16}, {0x7000, 24}};
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, 5, 0x8000, cases[i].chr_bytes, true));
        BOARD_CHECK(board_image_load(&image) == 0);
        apu_power_on(&apu);
        BOARD_CHECK(cpu_store(0x5104, 2));
        BOARD_CHECK(cpu_store(0x5C00, 0x5A));
        BOARD_CHECK(cpu_store(0x5FC0, 2));
        BOARD_CHECK(cpu_store(0x5200, 0x84));
        BOARD_CHECK(cpu_store(0x5201, 0));
        BOARD_CHECK(cpu_store(0x5202, 7));
        BOARD_CHECK(cpu_store(0x5104, 0));
        enter_frame();
        uint8_t tile = 0;
        // Alternating nametable addresses advance to column zero without
        // accidentally starting another scanline through three identical reads.
        for (unsigned fetch = 0; fetch < 47; ++fetch)
            tile = ppu_read((uint16_t)(0x2100 + (fetch & 0x1F)));
        BOARD_CHECK(tile == 0x5A);
        BOARD_CHECK(ppu_read(0x23C0) == 0xAA);
        BOARD_CHECK(ppu_read(0x0123) == cases[i].expected_page);
        BOARD_CHECK(ppu_read(0x012B) == cases[i].expected_page);
        uint8_t *previous_prg = prg_rom;
        BOARD_CHECK(load_rom_memory(image.data, image.size - 1) < 0);
        BOARD_CHECK(prg_rom == previous_prg);
        BOARD_CHECK(ppu_read(0x0123) == cases[i].expected_page);
        board_image_free(&image);
    }
    return 0;
}

static int test_rendered_irregular_chr(void) {
    for (unsigned split = 0; split < 2; ++split) {
        BoardImage image;
        BOARD_CHECK(board_image_create(&image, 5, 0x8000, 0x3000, true));
        uint8_t *chr = image.data + sizeof(iNESHeader) + 0x8000;
        memset(chr, 0, 0x3000);
        for (unsigned row = 0; row < 8; ++row) {
            // The selected physical page produces pixel 1. The two pages that
            // modulo addressing would select instead produce pixel 2.
            chr[0x0018 + row] = 0xFF;
            chr[0x1018 + row] = 0xFF;
            chr[0x1028 + row] = 0xFF;
            chr[0x2010 + row] = 0xFF;
            chr[0x2020 + row] = 0xFF;
        }
        BOARD_CHECK(board_image_load(&image) == 0);
        apu_power_on(&apu);
        BOARD_CHECK(cpu_store(0x5104, 2));
        for (unsigned tile = 0; tile < 0x3C0; ++tile)
            BOARD_CHECK(cpu_store((uint16_t)(0x5C00 + tile), split ? 2 : 0xC3));
        if (split) {
            for (unsigned attribute = 0x3C0; attribute < 0x400; ++attribute)
                BOARD_CHECK(cpu_store((uint16_t)(0x5C00 + attribute), 0x55));
            BOARD_CHECK(cpu_store(0x5200, 0x88));
            BOARD_CHECK(cpu_store(0x5202, 7));
        }
        BOARD_CHECK(cpu_store(0x5104, split ? 0 : 1));
        ppu_power_on(&ppu);
        memset(ppu_vram, 1, 0x3C0);
        ppu.scanline = (int)nes_timing()->scanlines - 1;
        ppu.dot = 0;
        ppu.ctrl = 0;
        ppu.mask = 0x0A;
        ppu.rendering_enabled = true;
        ppu.fetches_enabled = true;
        ppu_palette[0] = 0x0F;
        ppu_palette[2] = 0x16;
        ppu_palette[5] = ppu_palette[13] = 0x21;
        ppu_palette[6] = ppu_palette[14] = 0x16;
        ppu_begin_frame_render(framebuffer);
        ppu_step_dots(341 * 3);
        BOARD_CHECK(framebuffer[256] == get_color(0x21));
        BOARD_CHECK(framebuffer[256 + 63] == get_color(0x21));
        BOARD_CHECK(bg_opaque[256] && bg_opaque[256 + 63]);
        BOARD_CHECK(framebuffer[256 + 64] == get_color(split ? 0x16 : 0x21));
        board_image_free(&image);
    }
    return 0;
}

int test_mmc5_extended_geometry_accuracy(void) {
    int failures = test_extended_fetch_address_bits();
    failures += test_split_fetch_address_bits();
    failures += test_rendered_irregular_chr();
    unload_rom();
    printf("MMC5 extended geometry: 3 groups, %d failures\n", failures);
    return failures;
}
