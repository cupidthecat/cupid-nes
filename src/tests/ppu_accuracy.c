/*
 * ppu_accuracy.c - PPU hardware regression tests
 *
 * Author: @frankischilling
 *
 * This file tests PPU registers, memory transfers, fetch timing, rendering, scrolling,
 * sprite evaluation, palette behavior, vblank and NMI timing, and region specific clocks.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "../ppu/ppu.h"
#include "../cpu/cpu.h"
#include "../apu/apu.h"
#include "../rom/mapper.h"
#include "../system/hardware.h"
#include "../system/timing.h"
#include "../video/ntsc_composite.h"
#include "../../include/globals.h"
#include <stdio.h>
#include <string.h>

static int checks;
static int failures;
extern uint8_t ram[0x0800];
static uint8_t test_prg[0x8000];
static uint8_t test_chr[0x2000];
static uint16_t pattern_addresses[128];
static uint64_t pattern_clocks[128];
static unsigned pattern_reads;
static uint8_t (*pattern_reader)(uint16_t);
static uint16_t composite_fixture[SCREEN_WIDTH * SCREEN_HEIGHT];
static uint32_t composite_phase0[NTSC_COMPOSITE_WIDTH * NTSC_COMPOSITE_HEIGHT];
static uint32_t composite_phase1[NTSC_COMPOSITE_WIDTH * NTSC_COMPOSITE_HEIGHT];

#define CHECK(name, condition) do { \
    checks++; \
    if (!(condition)) { \
        failures++; \
        fprintf(stderr, "PPU:%d: %s\n", __LINE__, name); \
    } \
} while (0)

static void set_render_mask(uint8_t mask) {
    ppu.mask = mask;
    ppu.rendering_enabled = (mask & 0x18) != 0;
    ppu.fetches_enabled = ppu.rendering_enabled;
}

static void reset_video_region(unsigned mapper, NesRegion region) {
    nes_set_region(region);
    memset(test_prg, 0, sizeof(test_prg));
    memset(test_chr, 0, sizeof(test_chr));
    memset(ppu_vram, 0, sizeof(ppu_vram));
    memset(framebuffer, 0, sizeof(framebuffer));
    iNESHeader header = {0};
    memcpy(header.signature, "NES\x1A", 4);
    header.prg_rom_chunks = 2;
    header.flags6 = (uint8_t)(mapper << 4);
    header.flags7 = (uint8_t)(mapper & 0xF0);
    CHECK("synthetic cartridge initializes", mapper_init_from_header(&header, test_prg, sizeof(test_prg), test_chr, sizeof(test_chr)) == (int)mapper);
    ines_header = header;
    cpu_total_cycles = 0;
    apu_reset(&apu);
    ppu_reset(&ppu);
    cpu_power_on(&cpu);
    // Register and fetch tests choose their own PPU dot origin after the CPU
    // has completed its real reset reads with rendering disabled.
    ppu_reset(&ppu);
}

static void reset_video(unsigned mapper) {
    reset_video_region(mapper, NES_REGION_NTSC);
}

static uint64_t composite_hash(const uint32_t *pixels) {
    uint64_t hash = UINT64_C(1469598103934665603);
    for (size_t i = 0; i < (size_t)NTSC_COMPOSITE_WIDTH * NTSC_COMPOSITE_HEIGHT; ++i) {
        uint32_t pixel = pixels[i];
        for (unsigned byte = 0; byte < 4; ++byte) {
            hash ^= (uint8_t)(pixel >> (byte * 8));
            hash *= UINT64_C(1099511628211);
        }
    }
    return hash;
}

static void test_ntsc_composite_video(void) {
    CHECK("NTSC composite filter is available for ordinary NTSC output",
          ntsc_composite_supported(NES_REGION_NTSC, false));
    CHECK("PAL keeps the direct renderer when NTSC composite is requested",
          !ntsc_composite_supported(NES_REGION_PAL, false));
    CHECK("Dendy keeps the direct renderer when NTSC composite is requested",
          !ntsc_composite_supported(NES_REGION_DENDY, false));
    CHECK("VS RGB hardware keeps its native direct renderer",
          !ntsc_composite_supported(NES_REGION_NTSC, true));

    static const uint8_t colors[4] = {0x16, 0x27, 0x30, 0x0D};
    for (unsigned y = 0; y < SCREEN_HEIGHT; ++y) {
        for (unsigned x = 0; x < SCREEN_WIDTH; ++x) {
            unsigned band = (x / 32u + y / 40u) & 7u;
            uint16_t color = colors[(x / 64u + y / 60u) & 3u];
            composite_fixture[y * SCREEN_WIDTH + x] = color | (uint16_t)(band << 6);
        }
    }
    ntsc_composite_filter_frame(composite_fixture, 0, composite_phase0);
    ntsc_composite_filter_frame(composite_fixture, 1, composite_phase1);
    uint64_t phase0_hash = composite_hash(composite_phase0);
    uint64_t phase1_hash = composite_hash(composite_phase1);
    CHECK("phase-zero composite fixture has stable decoded output",
          phase0_hash == UINT64_C(0x1294327F4F883767));
    CHECK("phase-one composite fixture has stable decoded output",
          phase1_hash == UINT64_C(0x13C66FEFCD8EA1EB));
    CHECK("two-times composite output duplicates each decoded source row",
          memcmp(composite_phase0, composite_phase0 + NTSC_COMPOSITE_WIDTH,
                 NTSC_COMPOSITE_WIDTH * sizeof(uint32_t)) == 0);

    // This frame covers all palette/emphasis values, starting with hue 12.
    // Its hashes come from the independently compiled signal decoder.
    for (unsigned y = 0; y < SCREEN_HEIGHT; ++y)
        for (unsigned x = 0; x < SCREEN_WIDTH; ++x)
            composite_fixture[y * SCREEN_WIDTH + x] = (uint16_t)((x * 29u + y * 67u + 12u) & 511u);
    static const uint64_t signal_hashes[3] = {
        UINT64_C(0xA645DCD5E139059F), UINT64_C(0xCA5E00539D7CD747),
        UINT64_C(0x6CA5D1B39F66106F)
    };
    for (uint8_t phase = 0; phase < 3; ++phase) {
        ntsc_composite_filter_frame(composite_fixture, phase, composite_phase0);
        CHECK("all signal levels retain the decoder's initial carrier phase",
              composite_hash(composite_phase0) == signal_hashes[phase]);
    }

    reset_video(0);
    ppu.scanline = 0;
    ppu.dot = 1;
    ppu.v = 0x3F05;
    ppu_write(0x3F05, 0x2A);
    ppu.mask = 0x21;
    ppu.rendering_enabled = false;
    ppu.fetches_enabled = false;
    ppu_step_dots(1);
    CHECK("PPU captures raw palette index separately from composite metadata",
          ppu.pixel_indices[0] == 0x2A && ppu.pixel_signal[0] == 0x60);
    ppu.mask = 0xC0;
    ppu_step_dots(1);
    CHECK("PPU captures emphasis independently for the following pixel",
          ppu.pixel_indices[1] == 0x2A && ppu.pixel_signal[1] == 0x1AA);

    uint8_t saved_status = ppu.status;
    uint8_t saved_mask = ppu.mask;
    int saved_scanline = ppu.scanline;
    int saved_dot = ppu.dot;
    uint64_t saved_clocks = ppu.total_cycles;
    uint32_t saved_direct_pixel = framebuffer[0];
    uint16_t saved_brightness = ppu_pixel_brightness(0, 0);
    ntsc_composite_filter_frame(ppu.pixel_signal, ppu.completed_video_phase, composite_phase0);
    CHECK("composite presentation leaves PPU timing and registers unchanged",
          ppu.status == saved_status && ppu.mask == saved_mask
          && ppu.scanline == saved_scanline && ppu.dot == saved_dot
          && ppu.total_cycles == saved_clocks);
    CHECK("composite presentation leaves direct framebuffer and light-sensor pixels unchanged",
          framebuffer[0] == saved_direct_pixel && ppu.pixel_indices[0] == 0x2A
          && ppu_pixel_brightness(0, 0) == saved_brightness);
}

typedef struct {
    uint16_t v;
    uint16_t t;
    uint8_t x;
    uint8_t w;
    uint8_t ctrl;
} PpuWriteState;

static void cpu_sta_abs(uint16_t address, uint8_t value) {
    // STA abs fetches the high address byte immediately before its write.
    // PPU register mirrors therefore let tests choose a distinct preceding
    // CPU bus value without changing which PPU register receives the write.
    test_prg[0] = 0x8D;
    test_prg[1] = (uint8_t)address;
    test_prg[2] = (uint8_t)(address >> 8);
    cpu.pc = 0x8000;
    cpu.a = value;
    cpu_step(&cpu);
}

static uint8_t cpu_lda_abs(uint16_t address) {
    test_prg[0] = 0xAD;
    test_prg[1] = (uint8_t)address;
    test_prg[2] = (uint8_t)(address >> 8);
    cpu.pc = 0x8000;
    cpu_step(&cpu);
    return cpu.a;
}

static PpuWriteState cpu_sta_ppu(NesRegion region, int scanline, int start_dot,
                                  uint8_t render_mask, uint16_t initial_v,
                                  uint16_t initial_t, uint8_t initial_x,
                                  uint8_t initial_w, uint16_t address, uint8_t value) {
    reset_video_region(0, region);
    ppu.scanline = scanline;
    ppu.dot = start_dot;
    ppu.v = initial_v;
    ppu.t = initial_t;
    ppu.x = initial_x;
    ppu.w = initial_w;
    set_render_mask(render_mask);
    cpu_sta_abs(address, value);

    PpuWriteState state = {ppu.v, ppu.t, ppu.x, ppu.w, ppu.ctrl};
    return state;
}

static void seed_distinct_oam_rows(void) {
    for (unsigned row = 0; row < 32; ++row) {
        for (unsigned byte = 0; byte < 8; ++byte)
            ppu.oam[row * 8 + byte] = (uint8_t)(row * 8 + byte);
        ppu.secondary_oam[row] = (uint8_t)(0x80 | row);
    }
}

static bool oam_row_matches_seed(unsigned dest_row, unsigned source_row) {
    for (unsigned byte = 0; byte < 8; ++byte) {
        if (ppu.oam[dest_row * 8 + byte] != (uint8_t)(source_row * 8 + byte))
            return false;
    }
    return true;
}

static bool oam_row_matches_decay_value(unsigned row) {
    for (unsigned byte = 0; byte < 8; ++byte) {
        uint8_t address = (uint8_t)(row * 8 + byte);
        uint8_t expected = (address & 3) == 2 ? (uint8_t)(address & 0xE3) : address;
        if (ppu.oam[address] != expected) return false;
    }
    return true;
}

static uint8_t read_data(void) {
    uint8_t value = ppu_reg_read(PPUDATA);
    ppu_step_dots(6);
    return value;
}

static void write_data(uint8_t value) {
    ppu_reg_write(PPUDATA, value);
    ppu_step_dots(6);
}

static void set_data_address(uint16_t address) {
    ppu.v = address;
    ppu.bus_address = address & 0x3FFF;
}

static uint8_t trace_pattern_read(uint16_t addr) {
    if (pattern_reads < 128) {
        pattern_addresses[pattern_reads] = addr;
        pattern_clocks[pattern_reads] = ppu.total_cycles;
    }
    pattern_reads++;
    return pattern_reader(addr);
}

static void prepare_overlap(int x) {
    reset_video(0);
    ppu.scanline = 1;
    ppu.dot = x + 1;
    set_render_mask(0x1E);
    ppu.bg_shift_lo = 0x8000;
    ppu.sprite_count = 1;
    ppu.sprite_valid_mask = ppu.sprite_active_mask = 1;
    ppu.sprite_zero_on_line = true;
    ppu.sprite_positions[0] = (uint8_t)x;
    ppu.sprite_start_dot[0] = (uint16_t)x + 1;
    ppu.sprite_pattern_lo[0] = 0x80;
    ppu.sprite_attributes[0] = 0x20;
    ppu_write(0x3F01, 0x01);
    ppu_write(0x3F11, 0x21);
}

#include "ppu_accuracy_registers.h"
#include "ppu_accuracy_video.h"
#include "ppu_accuracy_sprite_wrap.h"

int test_ppu_accuracy(void) {
    checks = failures = 0;
    reset_video(0);
    ppu_write(0x3F10, 0xFF);
    CHECK("palette stores six bits", ppu_read(0x3F00) == 0x3F);
    CHECK("palette universal-color mirror", ppu_read(0x3F30) == 0x3F);
    ppu_write(0x3F24, 0x52);
    CHECK("palette entry four mirror", ppu_read(0x3F14) == 0x12);
    CHECK("palette mirror backing stays consistent", ppu_palette[4] == ppu_palette[0x14]);

    ppu_write(0x2000, 0xAB);
    set_data_address(0x2000);
    ppu.ppudata_buffer = 0x35;
    CHECK("ordinary PPUDATA read returns the previous buffer", read_data() == 0x35);
    CHECK("PPUDATA refills its buffer from VRAM", ppu.ppudata_buffer == 0xAB);
    CHECK("PPUDATA puts the returned value on open bus", ppu_reg_read(0x2000) == 0x35);
    CHECK("status low bits use returned PPUDATA value", (ppu_reg_read(0x2002) & 0x1F) == 0x15);
    CHECK("ordinary PPUDATA read increments address", ppu.v == 0x2001);
    ppu_write(0x2F00, 0x5A);
    ppu_write(0x3F00, 0x21);
    set_data_address(0x3F00);
    ppu_reg_write(0x2000, 0xC0);
    CHECK("palette read bypasses buffer and retains high open-bus bits", read_data() == 0xE1);
    CHECK("palette read refills buffer from underlying nametable", ppu.ppudata_buffer == 0x5A);
    ppu_reg_write(0x2001, 1);
    ppu_reg_write(0x2000, 0xC0);
    set_data_address(0x3F00);
    CHECK("grayscale also masks CPU palette reads", read_data() == 0xE0);
    set_data_address(0x3F00);
    ppu.ppudata_buffer = 0x71;
    write_data(0x18);
    CHECK("palette writes do not refill PPUDATA buffer", ppu.ppudata_buffer == 0x71);
    set_data_address(0x7FFF);
    ppu.ctrl = 0;
    read_data();
    CHECK("PPUDATA address wraps at fifteen bits", ppu.v == 0);
    set_data_address(0x7FE0);
    ppu.ctrl = 4;
    write_data(0);
    CHECK("vertical PPUDATA increment also wraps at fifteen bits", ppu.v == 0);

    reset_video(0);
    ppu.scanline = 20;
    ppu.dot = 305;
    set_render_mask(8);
    set_data_address(0x73BF);
    read_data();
    CHECK("render-time PPUDATA read wraps coarse X and Y together", ppu.v == 0x0C00);
    set_data_address(0x73FF);
    ppu.ctrl = 4;
    write_data(0);
    CHECK("coarse Y 31 wraps without vertical nametable toggle", ppu.v == 0x0400);
    set_render_mask(0x10);
    ppu.dot = 305;
    set_data_address(0);
    read_data();
    CHECK("sprite-only rendering also uses rendering address increment", ppu.v == 0x1001);
    ppu.scanline = 261;
    ppu.dot = 305;
    set_data_address(0);
    write_data(0);
    CHECK("pre-render PPUDATA write uses rendering address increment", ppu.v == 0x1001);

    reset_video(0);
    ppu_reg_write(0x2003, 2);
    ppu_reg_write(0x2004, 0xFF);
    CHECK("OAM attribute storage masks unimplemented bits", ppu.oam[2] == 0xE3);
    ppu_reg_write(0x2003, 2);
    CHECK("OAMDATA attribute read masks unimplemented bits", ppu_reg_read(0x2004) == 0xE3);
    ppu_reg_write(0x2003, 0xFF);
    ppu_reg_write(0x2004, 0x17);
    CHECK("OAM address increments with eight-bit wrap", ppu.oam_addr == 0 && ppu.oam[255] == 0x17);
    ppu.scanline = 20;
    set_render_mask(8);
    ppu.oam_addr = 3;
    uint8_t old_oam = ppu.oam[3];
    ppu_reg_write(0x2004, 0x55);
    CHECK("render-time OAMDATA write does not overwrite primary OAM", ppu.oam[3] == old_oam);
    CHECK("render-time OAMDATA write performs the rendering increment", ppu.oam_addr == 4);
    ppu.dot = 1;
    ppu_step_dots(2);
    CHECK("OAMDATA reads the secondary-OAM clear bus", ppu_reg_read(0x2004) == 0xFF);

    reset_video(0);
    for (int i = 0; i < 256; ++i) write_mem(0x0200 + i, (uint8_t)i);
    ppu.oam_addr = 0xF0;
    ppu_oam_dma(2);
    CHECK("OAM DMA waits for the next CPU read cycle", ppu.oam[0xF0] == 0xFF);
    cpu_step(&cpu);
    CHECK("OAM DMA preserves its wrapped starting address", ppu.oam_addr == 0xF0);
    for (int i = 0; i < 256; ++i) {
        unsigned address = (0xF0 + i) & 255;
        uint8_t expected = (address & 3) == 2 ? (uint8_t)(i & 0xE3) : (uint8_t)i;
        CHECK("OAM DMA copies through hardware attribute storage", ppu.oam[address] == expected);
    }

    reset_video(0);
    ppu.scanline = 0;
    set_render_mask(0x10);
    for (int i = 0; i < 9; ++i) {
        ppu.oam[i * 4] = 0;
        ppu.oam[i * 4 + 1] = 0;
        ppu.oam[i * 4 + 2] = 0;
        ppu.oam[i * 4 + 3] = (uint8_t)(i * 8);
    }
    ppu_step_dots(129);
    CHECK("sprite overflow waits for ninth-sprite evaluation", !(ppu.status & 0x20));
    ppu_step_dots(2);
    CHECK("ninth-sprite evaluation stages overflow on its condition clock", ppu.sprite_status_pending & 0x20);
    CHECK("sprite overflow is not externally visible on the condition clock", !(ppu.status & 0x20));
    ppu_step_dots(1);
    CHECK("ninth-sprite evaluation sets overflow", ppu.status & 0x20);
    memset(ppu.oam, 0xFF, sizeof(ppu.oam));
    ppu_step_dots(341 * 2);
    CHECK("sprite overflow persists across later scanlines", ppu.status & 0x20);
    start_frame();
    CHECK("host frame boundary does not clear sprite status", ppu.status & 0x20);
    ppu.scanline = 261;
    ppu.dot = 1;
    ppu_step_dots(1);
    CHECK("pre-render status clear removes sprite overflow", !(ppu.status & 0x20));

    reset_video(0);
    ppu.scanline = 0;
    set_render_mask(0x10);
    for (int i = 0; i < 8; ++i) memset(&ppu.oam[i * 4], 0, 4);
    ppu.oam[9 * 4 + 1] = 0;
    ppu_step_dots(133);
    CHECK("overflow diagonal match stages the flag on its condition clock", ppu.sprite_status_pending & 0x20);
    ppu_step_dots(1);
    CHECK("overflow diagonal scan can mistake a tile number for Y", ppu.status & 0x20);
    reset_video(0);
    ppu.scanline = 0;
    memset(ppu.oam, 0, 9 * 4);
    ppu_step_dots(257);
    CHECK("disabled rendering does not evaluate sprite overflow", !(ppu.status & 0x20));

    reset_video(0);
    ppu.scanline = 0;
    set_render_mask(0x14);
    ppu.oam[0] = 0;
    ppu.oam[1] = 1;
    ppu.oam[2] = 0;
    ppu.oam[3] = 8;
    test_chr[16] = 0x80;
    ppu_write(0x3F11, 0x21);
    ppu_step_dots(341);
    CHECK("sprites are evaluated for the following scanline", ppu.sprite_count == 1 && ppu.sprite_zero_on_line);
    CHECK("sprite pattern is loaded during sprite fetch slots", ppu.sprite_pattern_lo[0] == 0x80);
    test_chr[16] = 0;
    ppu.oam[3] = 100;
    ppu_step_dots(10);
    CHECK("visible sprite uses cached pattern and X position", framebuffer[256 + 8] == get_color(0x21));

    reset_video(0);
    ppu.scanline = 0;
    set_render_mask(0x14);
    ppu.ctrl = 0x20;
    ppu.oam[0] = 0;
    ppu.oam[1] = 3;
    ppu.oam[2] = 0xC0;
    ppu.oam[3] = 8;
    test_chr[0x1037] = 0x80;
    ppu_write(0x3F11, 0x21);
    ppu_step_dots(341 + 17);
    CHECK("8x16 sprite uses tile-bit pattern table and flipped lower half", framebuffer[256 + 15] == get_color(0x21));
    CHECK("horizontal sprite flip moves the opaque pixel", framebuffer[256 + 8] == get_color(ppu_palette[0]));

    prepare_overlap(8);
    ppu_step_dots(1);
    CHECK("sprite-zero overlap stages hit before the status output clock", ppu.sprite_status_pending & 0x40);
    CHECK("sprite-zero hit is not externally visible on the overlap clock", !(ppu.status & 0x40));
    ppu_step_dots(1);
    CHECK("sprite-zero hit ignores sprite background priority", ppu.status & 0x40);
    CHECK("behind-background sprite still displays background", framebuffer[256 + 8] == get_color(0x01));
    start_frame();
    CHECK("host frame boundary does not clear sprite-zero hit", ppu.status & 0x40);
    prepare_overlap(255);
    ppu_step_dots(1);
    CHECK("sprite-zero hit excludes pixel 255", !(ppu.status & 0x40));
    prepare_overlap(0);
    set_render_mask(0x1C);
    ppu_step_dots(1);
    CHECK("background left-edge mask prevents sprite-zero hit", !(ppu.status & 0x40));
    prepare_overlap(0);
    set_render_mask(0x1A);
    ppu_step_dots(1);
    CHECK("sprite left-edge mask prevents sprite-zero hit", !(ppu.status & 0x40));
    prepare_overlap(0);
    ppu_step_dots(1);
    CHECK("enabled left edges stage sprite-zero hit at pixel zero", ppu.sprite_status_pending & 0x40);
    ppu_step_dots(1);
    CHECK("enabled left edges permit sprite-zero hit at pixel zero", ppu.status & 0x40);
    prepare_overlap(8);
    ppu.sprite_count = 2;
    ppu.sprite_valid_mask = ppu.sprite_active_mask = 3;
    ppu.sprite_positions[1] = 8;
    ppu.sprite_attributes[1] = 0;
    ppu.sprite_pattern_hi[1] = 0x80;
    ppu_step_dots(1);
    CHECK("first opaque sprite wins before background priority is applied", framebuffer[256 + 8] == get_color(0x01));

    reset_video(0);
    ppu.scanline = (int)nes_timing()->scanlines - 1;
    ppu.dot = 1;
    ppu.sprite_status_pending = 0x60;
    ppu_step_dots(1);
    CHECK("pre-render status clear cancels pending sprite flags", !(ppu.status & 0x60) && !ppu.sprite_status_pending);

    for (int fine_x = 0; fine_x <= 3; fine_x += 3) {
        reset_video(0);
        set_render_mask(0x0A);
        ppu.x = (uint8_t)fine_x;
        for (int tile = 0; tile < 32; ++tile) ppu_write(0x2000 + tile, (uint8_t)(tile & 1));
        test_chr[0] = 0x80;
        test_chr[16] = 0x01;
        ppu_step_dots(341);
        CHECK("pre-render fetch primes two complete background tiles", ppu.bg_shift_lo == 0x8001);
        ppu_step_dots(33);
        for (int x = 0; x < 32; ++x) {
            int position = (x + fine_x) & 15;
            CHECK("background pipeline preserves tile edges with fine scrolling", bg_opaque[x] == (position == 0 || position == 15));
        }
    }
    reset_video(0);
    set_render_mask(0x0A);
    ppu.x = 3;
    test_chr[0] = 0xFF;
    ppu_write(0x23C0, 0xE4);
    ppu_write(0x3F01, 0x01);
    ppu_write(0x3F05, 0x11);
    ppu_step_dots(341 + 33);
    CHECK("background attribute shifter keeps the left palette", framebuffer[12] == get_color(0x01));
    CHECK("background attribute shifter scrolls palette boundaries", framebuffer[13] == get_color(0x11));

    reset_video(0);
    ppu.scanline = 0;
    ppu.dot = 1;
    ppu.v = 0x3F05;
    ppu_write(0x3F05, 0x21);
    ppu_step_dots(1);
    CHECK("forced blank displays palette address selected by v", framebuffer[0] == get_color(0x21));

    reset_video(0);
    ppu.scanline = 241;
    ppu.dot = 0;
    ppu_step_dots(1);
    ppu_reg_read(0x2002);
    ppu_step_dots(1);
    CHECK("status read just before vblank suppresses its flag", !(ppu.status & 0x80));
    ppu.scanline = 261;
    ppu.dot = 1;
    ppu_step_dots(1);
    ppu.scanline = 241;
    ppu.dot = 1;
    ppu_step_dots(1);
    CHECK("vblank suppression does not leak into the next frame", ppu.status & 0x80);

    reset_video(0);
    ppu.scanline = 0;
    set_render_mask(8);
    ppu_step_dots(89341);
    CHECK("even rendered frame has not ended after 89341 clocks", !ppu.frame_complete);
    ppu_step_dots(1);
    CHECK("even rendered frame lasts 89342 clocks", ppu.frame_complete && ppu.scanline == 0 && ppu.dot == 0);
    CHECK("completed even frame retains its starting NTSC carrier phase",
          ppu.completed_video_phase == 0 && ppu.frame_video_phase == 2);
    start_frame();
    CHECK("host frame start does not toggle hardware parity", ppu.odd_frame);
    ppu_step_dots(89340);
    CHECK("odd rendered frame has not ended one clock early", !ppu.frame_complete);
    ppu_step_dots(1);
    CHECK("odd rendered frame skips pre-render clock 340", ppu.frame_complete && ppu.scanline == 0 && ppu.dot == 0);
    CHECK("PPU clock counter remains monotonic across frame skip", ppu.total_cycles == 89342u + 89341u);
    CHECK("odd-frame skipped clock advances the next carrier phase from executed clocks",
          ppu.completed_video_phase == 2 && ppu.frame_video_phase == 0);
    reset_video(0);
    ppu.scanline = 0;
    ppu.odd_frame = true;
    ppu_step_dots(89341);
    CHECK("disabled rendering prevents odd-frame skip", !ppu.frame_complete);
    ppu_step_dots(1);
    CHECK("blank odd frame still has 89342 clocks", ppu.frame_complete);

    reset_video(0);
    ppu.scanline = 0;
    set_render_mask(0x18);
    Mapper *saved_cart = cart;
    Mapper tracing_cart = *cart;
    pattern_reader = cart->ppu_read;
    tracing_cart.ppu_read = trace_pattern_read;
    cart = &tracing_cart;
    pattern_reads = 0;
    start_frame();
    CHECK("starting host frame does not read cartridge graphics", pattern_reads == 0);
    ppu_step_dots(341);
    CHECK("render scanline performs 68 background and 16 sprite pattern reads", pattern_reads == 84);
    CHECK("first background pattern data reads occur on clocks six and eight", pattern_clocks[0] == 6 && pattern_clocks[1] == 8);
    CHECK("sprite pattern data reads occur in sprite fetch slots", pattern_clocks[64] == 262 && pattern_clocks[79] == 320);
    CHECK("next-line prefetch data reads occur after sprite fetches", pattern_clocks[80] == 326 && pattern_clocks[83] == 336);
    CHECK("unused sprite slots still perform pattern fetches", (pattern_addresses[64] & 0x0FF0) == 0x0FF0);
    cart = saved_cart;

    reset_video(4);
    ppu.scanline = 0;
    set_render_mask(8);
    ppu.ctrl = 8;
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    ppu_step_dots(261);
    CHECK("MMC3 IRQ waits for physical sprite-pattern A12 edge", !cart_irq_pending());
    ppu_step_dots(1);
    CHECK("sprite fetch A12 edge clocks MMC3 even with sprites hidden", cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    ppu_step_dots(341 - 262);
    CHECK("short A12 gaps between sprite slots do not reclock MMC3", !cart_irq_pending());
    ppu_step_dots(262);
    CHECK("next scanline clocks MMC3 again", cart_irq_pending());
    cart_cpu_write(0xE000, 0);
    cart_cpu_write(0xE001, 0);
    set_render_mask(0);
    ppu_step_dots(341 * 2);
    CHECK("disabled rendering does not manufacture scanline IRQ clocks", !cart_irq_pending());

    reset_video(4);
    ppu.scanline = 1;
    set_render_mask(8);
    ppu.ctrl = 0x10;
    cart_cpu_write(0xC000, 1);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    ppu_step_dots(341 + 325);
    CHECK("dot-zero pattern address prevents an extra background A12 clock", !cart_irq_pending());
    ppu_step_dots(1);
    CHECK("background-table A12 clocks once per complete scanline", cart_irq_pending());

    test_register_pipeline();
    test_dot257_scroll_glitches();
    test_oam_row_corruption_profiles();
    test_startup_register_restriction();
    test_oam_decay_refresh();
    test_oamdata_read_profile();
    test_regional_video();
    test_video_reset();
    test_reset_suppression();
    test_power_on_ram_profiles();
    test_sprite_shifters();
    test_sprite_evaluation_wrap_profile();
    test_late_register_reads();
    test_ntsc_composite_video();
    printf("PPU: %d checks, %d failures\n", checks, failures);
    return failures;
}
