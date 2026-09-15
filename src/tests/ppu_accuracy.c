/* SPDX-License-Identifier: GPL-3.0-or-later
 * PPU register, rendering, fetch-bus, and frame-clock regressions.
 */
#include "../ppu/ppu.h"
#include "../cpu/cpu.h"
#include "../apu/apu.h"
#include "../rom/mapper.h"
#include "../system/timing.h"
#include "../../include/globals.h"
#include <stdio.h>
#include <string.h>

static int checks;
static int failures;
static uint8_t test_prg[0x8000];
static uint8_t test_chr[0x2000];
static uint16_t pattern_addresses[128];
static uint64_t pattern_clocks[128];
static unsigned pattern_reads;
static uint8_t (*pattern_reader)(uint16_t);

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

static void reset_video(unsigned mapper) {
    nes_set_region(NES_REGION_NTSC);
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

static void test_register_pipeline(void) {
    reset_video(0);
    ppu.scanline = 241;
    ppu.dot = 20;
    ppu_write(0x2000, 0xAB);
    set_data_address(0x2000);
    ppu.ppudata_buffer = 0x35;
    CHECK("PPUDATA returns the old buffer before the external read", ppu_reg_read(PPUDATA) == 0x35);
    ppu_step_dots(4);
    CHECK("PPUDATA refill waits through four PPU clocks", ppu.ppudata_buffer == 0x35 && ppu.v == 0x2000);
    ppu_step_dots(1);
    CHECK("PPUDATA external read completes on the fifth clock", ppu.ppudata_buffer == 0xAB && ppu.v == 0x2000);
    ppu_step_dots(1);
    CHECK("PPUDATA increments v one clock after its external read", ppu.v == 0x2001 && ppu.bus_address == 0x2001);

    set_data_address(0x2000);
    ppu.ppudata_buffer = 0x57;
    CHECK("first consecutive PPUDATA read starts a transfer", ppu_reg_read(PPUDATA) == 0x57);
    ppu_step_dots(3);
    CHECK("next CPU-cycle PPUDATA read returns the IO latch", ppu_reg_read(PPUDATA) == 0x57);
    ppu_step_dots(3);
    CHECK("ignored PPUDATA read does not restart the transfer or increment twice", ppu.v == 0x2001 && ppu.ppudata_buffer == 0xAB && !ppu.data_read_delay);
    CHECK("PPUDATA accepts another read after the recovery interval", ppu_reg_read(PPUDATA) == 0xAB);
    ppu_step_dots(6);

    set_data_address(0x2000);
    ppu_reg_write(PPUDATA, 0x62);
    ppu_step_dots(4);
    CHECK("PPUDATA write does not reach memory in its first four clocks", ppu_read(0x2000) == 0xAB);
    ppu_step_dots(1);
    CHECK("PPUDATA write reaches memory before the address increments", ppu_read(0x2000) == 0x62 && ppu.v == 0x2000);
    ppu_step_dots(1);
    CHECK("PPUDATA write address increments on the sixth clock", ppu.v == 0x2001);

    set_data_address(0x1111);
    ppu.w = 0;
    ppu_reg_write(PPUADDR, 0x21);
    ppu_reg_write(PPUADDR, 0x40);
    CHECK("PPUADDR writes update t before v", ppu.t == 0x2140 && ppu.v == 0x1111);
    ppu_step_dots(2);
    CHECK("PPUADDR leaves the external bus unchanged for two clocks", ppu.v == 0x1111 && ppu.bus_address == 0x1111);
    ppu_step_dots(1);
    CHECK("PPUADDR loads v and its bus on the third clock", ppu.v == 0x2140 && ppu.bus_address == 0x2140);

    reset_video(0);
    ppu.scanline = 20;
    ppu.dot = 254;
    set_render_mask(8);
    ppu.v = 0x73BF;
    ppu_reg_write(PPUADDR, 0x21);
    ppu_reg_write(PPUADDR, 0x1F);
    ppu_step_dots(3);
    CHECK("PPUADDR colliding with dot 256 uses the incremented horizontal bits", ppu.v == 0x2100 && ppu.t == 0x2100);

    reset_video(4);
    ppu.scanline = 241;
    ppu.dot = 20;
    ppu.total_cycles = 32;
    cart_notify_ppu_address(0, 0);
    cart_cpu_write(0xC000, 0);
    cart_cpu_write(0xC001, 0);
    cart_cpu_write(0xE001, 0);
    ppu_reg_write(PPUADDR, 0x10);
    ppu_reg_write(PPUADDR, 0);
    ppu_step_dots(2);
    CHECK("MMC3 cannot see an uncommitted PPUADDR write", !cart_irq_pending());
    ppu_step_dots(1);
    CHECK("delayed PPUADDR bus transition clocks MMC3 A12", cart_irq_pending());
}

static void test_regional_video(void) {
    reset_video(0);
    ppu.mask = 0x20;
    uint32_t red_emphasis = get_color(0x20);
    ppu.mask = 0x40;
    uint32_t green_emphasis = get_color(0x20);
    for (NesRegion region = NES_REGION_PAL; region <= NES_REGION_DENDY; ++region) {
        nes_set_region(region);
        ppu_power_on(&ppu);
        CHECK("50 Hz hardware starts with scanline 311 as pre-render", ppu.scanline == 311);
        ppu.scanline = 0;
        ppu.odd_frame = true;
        set_render_mask(0x18);
        ppu_step_dots(106391);
        CHECK("PAL and Dendy rendering does not skip an odd-frame dot", !ppu.frame_complete);
        ppu_step_dots(1);
        CHECK("PAL and Dendy frames contain exactly 312 scanlines", ppu.frame_complete && ppu.scanline == 0 && ppu.dot == 0);
        ppu.scanline = (int)nes_timing()->vblank_scanline;
        ppu.dot = 1;
        ppu.status = 0;
        ppu_step_dots(1);
        CHECK("regional vblank starts at its hardware scanline", ppu.status & 0x80);
        ppu.scanline = 311;
        ppu.dot = 1;
        ppu.status |= 0x60;
        ppu_step_dots(1);
        CHECK("regional pre-render clears vblank and sprite flags", !(ppu.status & 0xE0));
        ppu.mask = 0x20;
        CHECK("PAL and Dendy emphasis bit five controls green", get_color(0x20) == green_emphasis);
        ppu.mask = 0x40;
        CHECK("PAL and Dendy emphasis bit six controls red", get_color(0x20) == red_emphasis);
    }
    ppu_power_on(&ppu);
    ppu.scanline = 241;
    ppu.dot = 1;
    ppu_step_dots(1);
    CHECK("Dendy does not start vblank at the NTSC scanline", !(ppu.status & 0x80));

    nes_set_region(NES_REGION_PAL);
    ppu_power_on(&ppu);
    ppu.scanline = 265;
    ppu.dot = 0;
    ppu_step_dots(2);
    CHECK("PAL OAM refresh skips scanline clock zero", ppu.oam_addr == 0);
    ppu_step_dots(1);
    CHECK("PAL OAM refresh first clocks at dot two", ppu.oam_addr == 1);
    ppu_step_dots(338);
    CHECK("PAL vblank refresh clocks OAM 170 times per scanline", ppu.oam_addr == 170);
    nes_set_region(NES_REGION_DENDY);
    ppu_power_on(&ppu);
    ppu.scanline = 265;
    ppu_step_dots(341);
    CHECK("Dendy does not perform PAL OAM refresh", ppu.oam_addr == 0);

    nes_set_region(NES_REGION_PAL);
    ppu_power_on(&ppu);
    ppu_step(1);
    CHECK("PAL standalone stepping preserves fractional PPU clocks", ppu.total_cycles == 3);
    ppu_step(1);
    ppu_step(1);
    ppu_step(1);
    ppu_step(1);
    CHECK("five PAL CPU clocks produce sixteen PPU clocks", ppu.total_cycles == 16);
    ppu_step(0);
    ppu_step(-1);
    CHECK("nonpositive CPU clock requests do not advance the PPU", ppu.total_cycles == 16);
    nes_set_region(NES_REGION_NTSC);
}

static void test_video_reset(void) {
    reset_video(0);
    ppu.oam[3] = 0x72;
    ppu.secondary_oam[7] = 0x42;
    ppu_write(0x3F05, 0x21);
    ppu_write(0x2000, 0x68);
    ppu.v = 0x27A5;
    ppu.status = 0xE0;
    ppu.ctrl = 0xFF;
    ppu.mask = 0xFF;
    ppu.w = 1;
    ppu.data_write_delay = 3;
    ppu.total_cycles = 1234;
    ppu_soft_reset(&ppu);
    CHECK("soft reset preserves primary and secondary OAM", ppu.oam[3] == 0x72 && ppu.secondary_oam[7] == 0x42);
    CHECK("soft reset preserves palette and nametable memory", ppu_read(0x3F05) == 0x21 && ppu_read(0x2000) == 0x68);
    CHECK("soft reset preserves v and status until hardware clears them", ppu.v == 0x27A5 && ppu.status == 0xE0);
    CHECK("soft reset clears control latches and outstanding register transfers", !ppu.ctrl && !ppu.mask && !ppu.w && !ppu.data_write_delay);
    CHECK("soft reset keeps cartridge clock timestamps monotonic", ppu.total_cycles == 1234);
    ppu_power_on(&ppu);
    CHECK("power on initializes the address and status latches", !ppu.v && !ppu.status);
    CHECK("power on initializes nametable RAM separately from soft reset", ppu_read(0x2000) == 0);
}

static void test_sprite_shifters(void) {
    reset_video(0);
    ppu.scanline = 0;
    set_render_mask(0x14);
    memset(ppu.oam, 0xFF, sizeof(ppu.oam));
    memset(ppu.oam, 0, 4);
    ppu.oam[1] = 1;
    test_chr[16] = 0xFF;
    ppu_write(0x3F11, 0x21);
    ppu_step_dots(341 + 2);
    CHECK("first sprite pixel consumes one pattern bit", ppu.sprite_pattern_lo[0] == 0xFE);
    set_render_mask(0);
    ppu_step_dots(10);
    CHECK("disabled rendering holds sprite pattern data", ppu.sprite_pattern_lo[0] == 0xFE);
    set_render_mask(0x14);
    ppu_step_dots(1);
    CHECK("reenabling rendering resumes a partially consumed sprite", framebuffer[256 + 11] == get_color(0x21));
    ppu_step_dots(7);
    CHECK("a resumed sprite emits only its remaining pattern bits", framebuffer[256 + 17] == get_color(0x21)
          && framebuffer[256 + 18] == get_color(ppu_palette[0]));

    reset_video(0);
    ppu.scanline = 0;
    set_render_mask(8);
    memset(ppu.oam, 0, 4);
    ppu.oam[1] = 1;
    test_chr[16] = 0x80;
    ppu_step_dots(341 + 2);
    CHECK("hidden sprites still shift while background rendering runs", ppu.sprite_pattern_lo[0] == 0);

    prepare_overlap(8);
    ppu.bg_shift_lo = 0;
    ppu.sprite_count = 2;
    ppu.sprite_valid_mask = ppu.sprite_active_mask = 3;
    ppu.sprite_pattern_lo[0] = 0x80;
    ppu.sprite_pattern_lo[1] = 0;
    ppu.sprite_pattern_hi[1] = 0x40;
    ppu.sprite_attributes[1] = 0;
    ppu_write(0x3F12, 0x31);
    ppu_step_dots(2);
    CHECK("lower-priority sprite shifts even when another sprite wins the pixel", framebuffer[256 + 9] == get_color(0x31));

    reset_video(0);
    ppu.scanline = 1;
    ppu.dot = 2;
    set_render_mask(8);
    ppu_step_dots(1);
    CHECK("background high-plane shifter has a pull-up input", (ppu.bg_shift_hi & 1) && !(ppu.bg_shift_lo & 1));
}

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
    CHECK("enabled left edges permit sprite-zero hit at pixel zero", ppu.status & 0x40);
    prepare_overlap(8);
    ppu.sprite_count = 2;
    ppu.sprite_valid_mask = ppu.sprite_active_mask = 3;
    ppu.sprite_positions[1] = 8;
    ppu.sprite_attributes[1] = 0;
    ppu.sprite_pattern_hi[1] = 0x80;
    ppu_step_dots(1);
    CHECK("first opaque sprite wins before background priority is applied", framebuffer[256 + 8] == get_color(0x01));

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
    start_frame();
    CHECK("host frame start does not toggle hardware parity", ppu.odd_frame);
    ppu_step_dots(89340);
    CHECK("odd rendered frame has not ended one clock early", !ppu.frame_complete);
    ppu_step_dots(1);
    CHECK("odd rendered frame skips pre-render clock 340", ppu.frame_complete && ppu.scanline == 0 && ppu.dot == 0);
    CHECK("PPU clock counter remains monotonic across frame skip", ppu.total_cycles == 89342u + 89341u);
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
    CHECK("first background pattern reads occur on clocks five and seven", pattern_clocks[0] == 5 && pattern_clocks[1] == 7);
    CHECK("sprite pattern reads occur in sprite fetch slots", pattern_clocks[64] == 261 && pattern_clocks[79] == 319);
    CHECK("next-line prefetch reads occur after sprite fetches", pattern_clocks[80] == 325 && pattern_clocks[83] == 335);
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
    test_regional_video();
    test_video_reset();
    test_sprite_shifters();
    printf("PPU: %d checks, %d failures\n", checks, failures);
    return failures;
}
