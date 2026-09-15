/* SPDX-License-Identifier: GPL-3.0-or-later
 * PPU register, rendering, fetch-bus, and frame-clock regressions.
 */
#include "../ppu/ppu.h"
#include "../cpu/cpu.h"
#include "../apu/apu.h"
#include "../rom/mapper.h"
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
    cpu_reset(&cpu);
    cpu_total_cycles = 0;
    apu_reset(&apu);
    ppu_reset(&ppu);
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
    ppu.sprite_zero_on_line = true;
    ppu.sprite_positions[0] = (uint8_t)x;
    ppu.sprite_pattern_lo[0] = 0x80;
    ppu.sprite_attributes[0] = 0x20;
    ppu_write(0x3F01, 0x01);
    ppu_write(0x3F11, 0x21);
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
    ppu.v = 0x2000;
    ppu.ppudata_buffer = 0x35;
    CHECK("ordinary PPUDATA read returns the previous buffer", ppu_reg_read(0x2007) == 0x35);
    CHECK("PPUDATA refills its buffer from VRAM", ppu.ppudata_buffer == 0xAB);
    CHECK("PPUDATA puts the returned value on open bus", ppu_reg_read(0x2000) == 0x35);
    CHECK("status low bits use returned PPUDATA value", (ppu_reg_read(0x2002) & 0x1F) == 0x15);
    CHECK("ordinary PPUDATA read increments address", ppu.v == 0x2001);
    ppu_write(0x2F00, 0x5A);
    ppu_write(0x3F00, 0x21);
    ppu.v = 0x3F00;
    ppu_reg_write(0x2000, 0xC0);
    CHECK("palette read bypasses buffer and retains high open-bus bits", ppu_reg_read(0x2007) == 0xE1);
    CHECK("palette read refills buffer from underlying nametable", ppu.ppudata_buffer == 0x5A);
    ppu_reg_write(0x2001, 1);
    ppu_reg_write(0x2000, 0xC0);
    ppu.v = 0x3F00;
    CHECK("grayscale also masks CPU palette reads", ppu_reg_read(0x2007) == 0xE0);
    ppu.v = 0x3F00;
    ppu.ppudata_buffer = 0x71;
    ppu_reg_write(0x2007, 0x18);
    CHECK("palette writes do not refill PPUDATA buffer", ppu.ppudata_buffer == 0x71);
    ppu.v = 0x7FFF;
    ppu.ctrl = 0;
    ppu_reg_read(0x2007);
    CHECK("PPUDATA address wraps at fifteen bits", ppu.v == 0);
    ppu.v = 0x7FE0;
    ppu.ctrl = 4;
    ppu_reg_write(0x2007, 0);
    CHECK("vertical PPUDATA increment also wraps at fifteen bits", ppu.v == 0);

    reset_video(0);
    ppu.scanline = 20;
    set_render_mask(8);
    ppu.v = 0x73BF;
    ppu_reg_read(0x2007);
    CHECK("render-time PPUDATA read wraps coarse X and Y together", ppu.v == 0x0C00);
    ppu.v = 0x73FF;
    ppu.ctrl = 4;
    ppu_reg_write(0x2007, 0);
    CHECK("coarse Y 31 wraps without vertical nametable toggle", ppu.v == 0x0400);
    set_render_mask(0x10);
    ppu.v = 0;
    ppu_reg_read(0x2007);
    CHECK("sprite-only rendering also uses rendering address increment", ppu.v == 0x1001);
    ppu.scanline = 261;
    ppu.v = 0;
    ppu_reg_write(0x2007, 0);
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

    printf("PPU: %d checks, %d failures\n", checks, failures);
    return failures;
}
