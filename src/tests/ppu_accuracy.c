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

static void test_dot257_scroll_glitches(void) {
    const uint8_t render = 0x08;

    PpuWriteState ctrl_before = cpu_sta_ppu(NES_REGION_NTSC, 20, 246, render,
                                             0, 0x0012, 3, 0, 0x3FF8, 0x02);
    PpuWriteState ctrl_during = cpu_sta_ppu(NES_REGION_NTSC, 20, 247, render,
                                             0, 0x0012, 3, 0, 0x3FF8, 0x02);
    PpuWriteState ctrl_after = cpu_sta_ppu(NES_REGION_NTSC, 20, 248, render,
                                            0, 0x0012, 3, 0, 0x3FF8, 0x02);
    CHECK("PPUCTRL before dot 257 feeds its normal t value into horizontal reload",
          (ctrl_before.v & 0x041F) == 0x0012);
    CHECK("PPUCTRL on dot 257 takes nametable bit zero from the previous CPU bus",
          (ctrl_during.v & 0x041F) == 0x0412);
    CHECK("PPUCTRL after dot 257 leaves the completed horizontal reload alone",
          (ctrl_after.v & 0x041F) == 0x0012);
    CHECK("dot-257 PPUCTRL still performs the normal control and t updates",
          ctrl_during.ctrl == 0x02 && ctrl_during.t == 0x0812
          && ctrl_during.x == 3 && ctrl_during.w == 0);

    PpuWriteState ctrl_low_bus = cpu_sta_ppu(NES_REGION_NTSC, 20, 247, render,
                                              0, 0x0012, 3, 0, 0x2000, 0x02);
    CHECK("dot-257 PPUCTRL uses pre-write CPU bus data instead of the written byte",
          (ctrl_low_bus.v & 0x041F) == 0x0012
          && ((ctrl_during.v ^ ctrl_low_bus.v) & 0x7FFF) == 0x0400);

    PpuWriteState scroll_before = cpu_sta_ppu(NES_REGION_NTSC, 20, 246, render,
                                               0, 0x0412, 1, 0, 0x3FFD, 0xA5);
    PpuWriteState scroll_during = cpu_sta_ppu(NES_REGION_NTSC, 20, 247, render,
                                               0, 0x0412, 1, 0, 0x3FFD, 0xA5);
    PpuWriteState scroll_after = cpu_sta_ppu(NES_REGION_NTSC, 20, 248, render,
                                              0, 0x0412, 1, 0, 0x3FFD, 0xA5);
    CHECK("first PPUSCROLL before dot 257 feeds its new coarse X into reload",
          (scroll_before.v & 0x041F) == 0x0414);
    CHECK("first PPUSCROLL on dot 257 takes coarse X from the previous CPU bus",
          (scroll_during.v & 0x041F) == 0x0407);
    CHECK("first PPUSCROLL after dot 257 leaves the old coarse X reload alone",
          (scroll_after.v & 0x041F) == 0x0412);
    CHECK("dot-257 first PPUSCROLL still updates t, fine X, and the write toggle",
          scroll_during.t == 0x0414 && scroll_during.x == 5 && scroll_during.w == 1);

    PpuWriteState scroll_low_bus = cpu_sta_ppu(NES_REGION_NTSC, 20, 247, render,
                                                0, 0x0412, 1, 0, 0x2005, 0xA5);
    CHECK("dot-257 first PPUSCROLL preserves non-coarse-X bits while using CPU bus data",
          (scroll_low_bus.v & 0x041F) == 0x0404
          && ((scroll_during.v ^ scroll_low_bus.v) & (uint16_t)~0x001F) == 0);

    PpuWriteState addr_before = cpu_sta_ppu(NES_REGION_NTSC, 20, 246, render,
                                             0, 0x0412, 6, 0, 0x3FFE, 0x2A);
    PpuWriteState addr_during = cpu_sta_ppu(NES_REGION_NTSC, 20, 247, render,
                                             0, 0x0412, 6, 0, 0x3FFE, 0x2A);
    PpuWriteState addr_after = cpu_sta_ppu(NES_REGION_NTSC, 20, 248, render,
                                            0, 0x0412, 6, 0, 0x3FFE, 0x2A);
    CHECK("first PPUADDR before dot 257 changes the horizontal nametable reload normally",
          (addr_before.v & 0x0C00) == 0);
    CHECK("first PPUADDR on dot 257 takes both nametable bits from the previous CPU bus",
          (addr_during.v & 0x0C00) == 0x0C00);
    CHECK("first PPUADDR after dot 257 leaves the old horizontal nametable reload alone",
          (addr_after.v & 0x0C00) == 0x0400);
    CHECK("dot-257 first PPUADDR still updates t and the shared write toggle",
          addr_during.t == 0x2A12 && addr_during.x == 6 && addr_during.w == 1);

    PpuWriteState addr_low_bus = cpu_sta_ppu(NES_REGION_NTSC, 20, 247, render,
                                              0, 0x0412, 6, 0, 0x2006, 0x2A);
    CHECK("dot-257 first PPUADDR uses previous CPU bus nametable bits instead of write data",
          (addr_low_bus.v & 0x0C00) == 0
          && ((addr_during.v ^ addr_low_bus.v) & (uint16_t)~0x0C00) == 0);

    PpuWriteState blank_ctrl = cpu_sta_ppu(NES_REGION_NTSC, 20, 247, 0,
                                            0x1555, 0x0012, 3, 0, 0x3FF8, 0x02);
    PpuWriteState blank_scroll = cpu_sta_ppu(NES_REGION_NTSC, 20, 247, 0,
                                              0x1555, 0x0412, 1, 0, 0x3FFD, 0xA5);
    PpuWriteState blank_addr = cpu_sta_ppu(NES_REGION_NTSC, 20, 247, 0,
                                            0x1555, 0x0412, 6, 0, 0x3FFE, 0x2A);
    CHECK("disabled rendering prevents all first-write dot-257 scroll corruption",
          blank_ctrl.v == 0x1555 && blank_scroll.v == 0x1555 && blank_addr.v == 0x1555);
    CHECK("disabled rendering keeps normal first-write register updates",
          blank_ctrl.t == 0x0812 && blank_ctrl.ctrl == 0x02
          && blank_scroll.t == 0x0414 && blank_scroll.x == 5 && blank_scroll.w == 1
          && blank_addr.t == 0x2A12 && blank_addr.w == 1);

    PpuWriteState last_visible_ctrl = cpu_sta_ppu(NES_REGION_NTSC, 239, 247, render,
                                                   0, 0x0012, 3, 0, 0x3FF8, 0x02);
    PpuWriteState last_visible_scroll = cpu_sta_ppu(NES_REGION_NTSC, 239, 247, render,
                                                     0, 0x0412, 1, 0, 0x3FFD, 0xA5);
    PpuWriteState last_visible_addr = cpu_sta_ppu(NES_REGION_NTSC, 239, 247, render,
                                                   0, 0x0412, 6, 0, 0x3FFE, 0x2A);
    CHECK("dot-257 first-write corruption remains active on visible scanline 239",
          (last_visible_ctrl.v & 0x041F) == 0x0412
          && (last_visible_scroll.v & 0x041F) == 0x0407
          && (last_visible_addr.v & 0x0C00) == 0x0C00);

    PpuWriteState postrender_ctrl = cpu_sta_ppu(NES_REGION_NTSC, 240, 247, render,
                                                 0x1555, 0x0012, 3, 0, 0x3FF8, 0x02);
    PpuWriteState postrender_scroll = cpu_sta_ppu(NES_REGION_NTSC, 240, 247, render,
                                                   0x1555, 0x0412, 1, 0, 0x3FFD, 0xA5);
    PpuWriteState postrender_addr = cpu_sta_ppu(NES_REGION_NTSC, 240, 247, render,
                                                 0x1555, 0x0412, 6, 0, 0x3FFE, 0x2A);
    CHECK("post-render scanline 240 has no dot-257 first-write corruption",
          postrender_ctrl.v == 0x1555 && postrender_scroll.v == 0x1555
          && postrender_addr.v == 0x1555);

    int prerender = (int)nes_timing()->scanlines - 1;
    PpuWriteState prerender_ctrl = cpu_sta_ppu(NES_REGION_NTSC, prerender, 247, render,
                                                0, 0x0012, 3, 0, 0x3FF8, 0x02);
    PpuWriteState prerender_scroll = cpu_sta_ppu(NES_REGION_NTSC, prerender, 247, render,
                                                  0, 0x0412, 1, 0, 0x3FFD, 0xA5);
    PpuWriteState prerender_addr = cpu_sta_ppu(NES_REGION_NTSC, prerender, 247, render,
                                                0, 0x0412, 6, 0, 0x3FFE, 0x2A);
    CHECK("pre-render horizontal reload does not use the visible-line first-write glitch",
          (prerender_ctrl.v & 0x041F) == 0x0012
          && (prerender_scroll.v & 0x041F) == 0x0412
          && (prerender_addr.v & 0x0C00) == 0x0400);

    const NesRegion regions[] = {NES_REGION_NTSC, NES_REGION_PAL, NES_REGION_DENDY};
    for (size_t i = 0; i < sizeof(regions) / sizeof(regions[0]); ++i) {
        PpuWriteState regional_ctrl = cpu_sta_ppu(regions[i], 20, 247, render,
                                                   0, 0x0012, 3, 0, 0x3FF8, 0x02);
        PpuWriteState regional_scroll = cpu_sta_ppu(regions[i], 20, 247, render,
                                                     0, 0x0412, 1, 0, 0x3FFD, 0xA5);
        PpuWriteState regional_addr = cpu_sta_ppu(regions[i], 20, 247, render,
                                                   0, 0x0412, 6, 0, 0x3FFE, 0x2A);
        CHECK("regional CPU/PPU divider alignment reaches the PPUCTRL dot-257 window",
              (regional_ctrl.v & 0x041F) == 0x0412);
        CHECK("regional CPU/PPU divider alignment reaches the PPUSCROLL dot-257 window",
              (regional_scroll.v & 0x041F) == 0x0407);
        CHECK("regional CPU/PPU divider alignment reaches the PPUADDR dot-257 window",
              (regional_addr.v & 0x0C00) == 0x0C00);
    }
    nes_set_region(NES_REGION_NTSC);
}

static void test_oam_row_corruption_profiles(void) {
    PpuRevision saved_revision = ppu_revision();
    bool saved_worst_case = ppu_oam_row_corruption_worst_case();

    CHECK("late 2C02 revision name is accepted",
          ppu_set_revision_name("2c02e-plus")
          && ppu_revision() == PPU_REVISION_2C02_E_PLUS
          && strcmp(ppu_revision_name(), "2c02e-plus") == 0);
    CHECK("early 2C02 revision name is accepted",
          ppu_set_revision_name("2c02-pre-e")
          && ppu_revision() == PPU_REVISION_2C02_PRE_E
          && strcmp(ppu_revision_name(), "2c02-pre-e") == 0);
    CHECK("invalid PPU revisions are rejected",
          !ppu_set_revision((PpuRevision)-1)
          && !ppu_set_revision((PpuRevision)2)
          && !ppu_set_revision_name(NULL)
          && !ppu_set_revision_name("2c02-unknown"));
    ppu_set_oam_row_corruption_worst_case(true);
    ppu_reset(&ppu);
    CHECK("PPU corruption profile survives reset",
          ppu_revision() == PPU_REVISION_2C02_PRE_E
          && ppu_oam_row_corruption_worst_case());

    ppu_set_revision(PPU_REVISION_2C02_E_PLUS);
    ppu_set_oam_row_corruption_worst_case(false);
    reset_video(0);
    seed_distinct_oam_rows();
    ppu.scanline = 20;
    ppu.dot = 20;
    set_render_mask(0);
    ppu.oam_addr = 0x10;
    cpu_sta_abs(0x3FE3, 0x30);
    CHECK("default profile leaves $2003 row-corruption approximation disabled",
          oam_row_matches_seed(6, 6) && oam_row_matches_seed(7, 7)
          && ppu.secondary_oam[6] == 0x86 && ppu.secondary_oam[7] == 0x87
          && ppu.oam_addr == 0x30);

    ppu_set_oam_row_corruption_worst_case(true);
    reset_video(0);
    seed_distinct_oam_rows();
    ppu.scanline = 20;
    ppu.dot = 20;
    set_render_mask(0);
    ppu.oam_addr = 0x10;
    cpu_sta_abs(0x3FE3, 0x30);
    CHECK("worst-case $2003 uses the previous CPU bus as its intermediate row",
          oam_row_matches_seed(7, 2) && oam_row_matches_seed(6, 2)
          && ppu.secondary_oam[7] == 0x82 && ppu.secondary_oam[6] == 0x82);
    CHECK("worst-case $2003 leaves source and unrelated rows unchanged",
          oam_row_matches_seed(2, 2) && oam_row_matches_seed(5, 5)
          && ppu.secondary_oam[2] == 0x82 && ppu.secondary_oam[5] == 0x85
          && ppu.oam_addr == 0x30);

    reset_video(0);
    seed_distinct_oam_rows();
    ppu.scanline = 20;
    ppu.dot = 20;
    set_render_mask(0);
    ppu.oam_addr = 0x10;
    cpu_sta_abs(OAMADDR, 0x30);
    CHECK("$2003 mirror choice changes the previous-bus row without changing the write",
          oam_row_matches_seed(4, 2) && oam_row_matches_seed(6, 2)
          && oam_row_matches_seed(7, 7)
          && ppu.secondary_oam[4] == 0x82 && ppu.secondary_oam[6] == 0x82);

    int prerender = (int)nes_timing()->scanlines - 1;
    reset_video(0);
    seed_distinct_oam_rows();
    ppu.scanline = prerender;
    ppu.dot = 90;
    set_render_mask(8);
    ppu.oam_addr = 0x10;
    cpu_sta_abs(0x3FE3, 0x30);
    CHECK("render-time odd-clock $2003 takes the opt-in corruption path",
          oam_row_matches_seed(7, 2) && oam_row_matches_seed(6, 2));

    reset_video(0);
    seed_distinct_oam_rows();
    ppu.scanline = prerender;
    ppu.dot = 89;
    set_render_mask(8);
    ppu.oam_addr = 0x10;
    cpu_sta_abs(0x3FE3, 0x30);
    CHECK("render-time even-clock $2003 does not take the alignment-dependent path",
          oam_row_matches_seed(7, 7) && oam_row_matches_seed(6, 6));

    reset_video(0);
    seed_distinct_oam_rows();
    ppu.scanline = prerender;
    ppu.dot = 249;
    set_render_mask(8);
    ppu.oam_addr = 0x10;
    cpu_sta_abs(0x3FE3, 0x30);
    CHECK("render-time $2003 after dot 257 does not use the worst-case address path",
          oam_row_matches_seed(7, 7) && oam_row_matches_seed(6, 6));

    reset_video_region(0, NES_REGION_PAL);
    seed_distinct_oam_rows();
    ppu.scanline = 20;
    ppu.dot = 20;
    set_render_mask(0);
    ppu.oam_addr = 0x10;
    cpu_sta_abs(0x3FE3, 0x30);
    CHECK("PAL excludes $2003 row corruption even in worst-case mode",
          oam_row_matches_seed(7, 7) && oam_row_matches_seed(6, 6));

    reset_video_region(0, NES_REGION_PAL);
    seed_distinct_oam_rows();
    ppu.scanline = 20;
    ppu.dot = 100;
    ppu.mask = 8;
    ppu.rendering_enabled = true;
    ppu.fetches_enabled = false;
    ppu.oam_addr = 0x10;
    ppu.secondary_index = 5;
    ppu_step_dots(1);
    CHECK("PAL excludes rendering-transition row corruption",
          oam_row_matches_seed(5, 5) && ppu.secondary_oam[5] == 0x85);

    reset_video(0);
    seed_distinct_oam_rows();
    ppu.scanline = 20;
    ppu.dot = 88;
    set_render_mask(0);
    ppu.oam_addr = 0x10;
    ppu.secondary_index = 5;
    cpu_sta_abs(PPUMASK, 8);
    CHECK("real CPU mask write leaves the second rendering latch pending", ppu.dot == 100
          && ppu.rendering_enabled && !ppu.fetches_enabled);
    ppu_step_dots(1);
    CHECK("evaluation-window render-enable copies the selected primary row to the secondary row",
          oam_row_matches_seed(5, 2) && ppu.secondary_oam[5] == 0x82
          && oam_row_matches_seed(6, 6));

    reset_video(0);
    seed_distinct_oam_rows();
    ppu.scanline = 20;
    ppu.dot = 89;
    set_render_mask(0);
    ppu.oam_addr = 0x10;
    ppu.secondary_index = 5;
    cpu_sta_abs(PPUMASK, 8);
    ppu_step_dots(1);
    CHECK("odd evaluation-window render-enable transition does not copy an OAM row",
          oam_row_matches_seed(5, 5) && ppu.secondary_oam[5] == 0x85);

    reset_video(0);
    seed_distinct_oam_rows();
    ppu.scanline = prerender;
    ppu.dot = 88;
    set_render_mask(8);
    ppu.oam_addr = 0x10;
    ppu.secondary_index = 5;
    cpu_sta_abs(PPUMASK, 0);
    ppu_step_dots(1);
    CHECK("render-disable transition copies the selected secondary row to primary OAM",
          oam_row_matches_seed(2, 5) && ppu.secondary_oam[2] == 0x85
          && oam_row_matches_seed(6, 6));
    CHECK("render-disable row copy occurs before the same-clock OAM increment",
          ppu.oam_addr == 0x11);

    reset_video(0);
    seed_distinct_oam_rows();
    ppu.scanline = 20;
    ppu.dot = 248;
    set_render_mask(8);
    ppu.oam_addr = 0x78;
    ppu.secondary_index = 19;
    cpu_sta_abs(PPUMASK, 0);
    ppu_step_dots(1);
    CHECK("sprite-fetch transition uses the rows selected by fetch timing before corruption",
          oam_row_matches_seed(0, 3) && ppu.secondary_oam[0] == 0x83
          && ppu.oam_addr == 0 && ppu.secondary_index == 3);

    ppu_set_revision(PPU_REVISION_2C02_E_PLUS);
    reset_video(0);
    seed_distinct_oam_rows();
    prerender = (int)nes_timing()->scanlines - 1;
    ppu.scanline = prerender;
    ppu.dot = 0;
    set_render_mask(8);
    ppu.oam_addr = 0x10;
    ppu.secondary_index = 5;
    ppu_step_dots(1);
    CHECK("late 2C02 profile applies the pre-render row copy",
          oam_row_matches_seed(5, 2) && ppu.secondary_oam[5] == 0x82);

    ppu_set_revision(PPU_REVISION_2C02_PRE_E);
    reset_video(0);
    seed_distinct_oam_rows();
    prerender = (int)nes_timing()->scanlines - 1;
    ppu.scanline = prerender;
    ppu.dot = 0;
    set_render_mask(8);
    ppu.oam_addr = 0x10;
    ppu.secondary_index = 5;
    ppu_step_dots(1);
    CHECK("pre-E 2C02 profile excludes the pre-render row copy",
          oam_row_matches_seed(5, 5) && ppu.secondary_oam[5] == 0x85);

    ppu_set_revision(PPU_REVISION_2C02_E_PLUS);
    reset_video_region(0, NES_REGION_PAL);
    seed_distinct_oam_rows();
    prerender = (int)nes_timing()->scanlines - 1;
    ppu.scanline = prerender;
    ppu.dot = 0;
    set_render_mask(8);
    ppu.oam_addr = 0x10;
    ppu.secondary_index = 5;
    ppu_step_dots(1);
    CHECK("PAL excludes the late-revision pre-render row copy",
          oam_row_matches_seed(5, 5) && ppu.secondary_oam[5] == 0x85);

    ppu_set_revision(saved_revision);
    ppu_set_oam_row_corruption_worst_case(saved_worst_case);
    nes_set_region(NES_REGION_NTSC);
}

static void test_startup_register_restriction(void) {
    bool saved_restriction = ppu_startup_write_restriction_enabled();
    ppu_set_startup_write_restriction(true);

    reset_video(0);
    CHECK("startup write restriction profile survives PPU reset",
          ppu_startup_write_restriction_enabled() && ppu_startup_writes_restricted());

    ppu.ctrl = 0x14;
    ppu.mask = 0x04;
    ppu.t = 0x1234;
    ppu.x = 5;
    ppu.w = 0;
    ppu.status = 0x80;
    ppu.nmi_out = false;
    ppu.address_write_delay = 0;
    ppu_reg_write_cpu(PPUCTRL, 0x80, 0x5A);
    CHECK("startup-restricted PPUCTRL write only charges the PPU I/O latch",
          ppu.ctrl == 0x14 && ppu.t == 0x1234 && !ppu.nmi_out && ppu.open_bus == 0x80);
    ppu_reg_write_cpu(PPUMASK, 0x18, 0xA5);
    CHECK("startup-restricted PPUMASK write does not change rendering state",
          ppu.mask == 0x04 && !ppu.rendering_enabled && !ppu.fetches_enabled
          && ppu.open_bus == 0x18);
    ppu_reg_write_cpu(PPUSCROLL, 0x27, 0x33);
    ppu_reg_write_cpu(PPUSCROLL, 0xE8, 0x44);
    CHECK("startup-restricted PPUSCROLL writes do not advance the shared write toggle",
          ppu.t == 0x1234 && ppu.x == 5 && ppu.w == 0 && ppu.open_bus == 0xE8);
    ppu_reg_write_cpu(PPUADDR, 0x3F, 0x55);
    ppu_reg_write_cpu(PPUADDR, 0x20, 0x66);
    CHECK("startup-restricted PPUADDR writes do not queue an address transfer",
          ppu.t == 0x1234 && ppu.w == 0 && ppu.address_write_delay == 0
          && ppu.open_bus == 0x20);
    ppu_reg_write_cpu(OAMADDR, 0x48, 0);
    CHECK("startup restriction leaves OAMADDR writes available", ppu.oam_addr == 0x48);

    reset_video(0);
    uint64_t frame_clocks = (uint64_t)nes_timing()->scanlines * 341u;
    ppu_step_dots((int)(frame_clocks - 13));
    CHECK("startup restriction remains active before the release boundary",
          ppu_startup_writes_restricted()
          && ppu.scanline == (int)nes_timing()->scanlines - 2 && ppu.dot == 328);
    ppu.status = 0x80;
    cpu_sta_abs(PPUCTRL, 0x80);
    CHECK("last CPU write before startup release is ignored",
          ppu_startup_writes_restricted() && ppu.ctrl == 0 && !ppu.nmi_out
          && ppu.scanline == (int)nes_timing()->scanlines - 2 && ppu.dot == 340);
    ppu_step_dots(1);
    CHECK("startup access opens on entry to the next pre-render scanline",
          !ppu_startup_writes_restricted()
          && ppu.scanline == (int)nes_timing()->scanlines - 1 && ppu.dot == 0);
    cpu_sta_abs(PPUCTRL, 0x80);
    CHECK("first CPU write after startup release is accepted", ppu.ctrl == 0x80);
    ppu_reg_write_cpu(PPUCTRL, 0, 0);
    ppu.status = 0x80;
    ppu.nmi_out = false;
    ppu_reg_write_cpu(PPUCTRL, 0x80, 0);
    CHECK("released PPUCTRL can assert NMI during a later vblank",
          ppu.ctrl == 0x80 && ppu.nmi_out);

    const NesRegion regions[] = {NES_REGION_NTSC, NES_REGION_PAL, NES_REGION_DENDY};
    for (size_t i = 0; i < sizeof(regions) / sizeof(regions[0]); ++i) {
        reset_video_region(0, regions[i]);
        uint64_t clocks = (uint64_t)nes_timing()->scanlines * 341u;
        CHECK("regional startup profile begins with protected writes",
              ppu_startup_writes_restricted()
              && ppu.scanline == (int)nes_timing()->scanlines - 1 && ppu.dot == 0);
        ppu_step_dots((int)(clocks - 1));
        CHECK("regional startup profile stays protected through the last pre-release dot",
              ppu_startup_writes_restricted()
              && ppu.scanline == (int)nes_timing()->scanlines - 2 && ppu.dot == 340);
        ppu_step_dots(1);
        CHECK("regional startup profile releases at the next pre-render boundary",
              !ppu_startup_writes_restricted()
              && ppu.scanline == (int)nes_timing()->scanlines - 1 && ppu.dot == 0);
    }

    nes_set_region(NES_REGION_NTSC);
    ppu.total_cycles = 12345;
    ppu.oam[9] = 0xA7;
    ppu_soft_reset(&ppu);
    CHECK("soft reset restarts the selected startup restriction",
          ppu_startup_writes_restricted() && ppu.total_cycles == 12345
          && ppu.oam[9] == 0xA7);

    ppu_set_startup_write_restriction(false);
    ppu_soft_reset(&ppu);
    CHECK("compatibility profile leaves startup register writes unrestricted",
          !ppu_startup_write_restriction_enabled() && !ppu_startup_writes_restricted());
    ppu_reg_write_cpu(PPUCTRL, 0x91, 0);
    CHECK("compatibility profile accepts PPUCTRL immediately", ppu.ctrl == 0x91);

    ppu_set_startup_write_restriction(saved_restriction);
    nes_set_region(NES_REGION_NTSC);
}

static void test_oam_decay_refresh(void) {
    bool saved_decay = ppu_oam_decay_enabled();
    ppu_set_oam_decay(true);

    reset_video(0);
    memset(ppu.oam, 0xA5, sizeof(ppu.oam));
    ppu.oam_addr = 0x18;
    ppu.oam_decay_cycles[3] = 500;
    cpu_total_cycles = 5000;
    CHECK("OAM row remains intact at the 4500-cycle decay threshold",
          ppu_reg_read(OAMDATA) == 0xA5 && ppu.oam[0x18] == 0xA5);
    CHECK("fresh OAM read refreshes only the selected row timestamp",
          ppu.oam_decay_cycles[3] == 5000 && ppu.oam_decay_cycles[4] == 0);

    memset(&ppu.oam[4 * 8], 0xA5, 8);
    memset(&ppu.oam[5 * 8], 0x6C, 8);
    ppu.oam_decay_cycles[4] = 499;
    ppu.oam_decay_cycles[5] = 499;
    ppu.oam_addr = 4 * 8;
    CHECK("OAM row decays immediately after the 4500-cycle threshold",
          ppu_reg_read(OAMDATA) == 0x20 && oam_row_matches_decay_value(4));
    CHECK("decay clears nonexistent attribute bits and leaves unrelated rows alone",
          ppu.oam[0x22] == 0x22 && ppu.oam[0x26] == 0x22
          && ppu.oam[5 * 8] == 0x6C && ppu.oam_decay_cycles[5] == 499);
    CHECK("reading an already-expired row does not manufacture a refresh timestamp",
          ppu.oam_decay_cycles[4] == 499);

    memset(&ppu.oam[6 * 8], 0xC6, 8);
    memset(&ppu.oam[7 * 8], 0xD7, 8);
    ppu.oam_decay_cycles[6] = 1000;
    ppu.oam_decay_cycles[7] = 1000;
    cpu_total_cycles = 5400;
    ppu.oam_addr = 6 * 8;
    CHECK("refreshing one OAM row preserves that row", ppu_reg_read(OAMDATA) == 0xC6);
    CHECK("refreshing one OAM row does not refresh its neighbor",
          ppu.oam_decay_cycles[6] == 5400 && ppu.oam_decay_cycles[7] == 1000);
    cpu_total_cycles = 5501;
    ppu.oam_addr = 7 * 8;
    (void)ppu_reg_read(OAMDATA);
    CHECK("unrefreshed neighboring row can decay independently",
          oam_row_matches_decay_value(7) && ppu.oam[6 * 8] == 0xC6);

    reset_video(0);
    ppu.oam_addr = 8 * 8;
    ppu.oam[8 * 8] = 0x66;
    uint64_t before_cpu_read = ppu.oam_decay_cycles[8];
    CHECK("real CPU OAMDATA read uses the decay-aware primary OAM path",
          cpu_lda_abs(OAMDATA) == 0x66);
    CHECK("real CPU OAMDATA read refreshes the selected row",
          ppu.oam_decay_cycles[8] > before_cpu_read
          && ppu.oam_decay_cycles[9] == 0);

    ppu.oam_addr = 9 * 8;
    uint64_t before_cpu_write = ppu.oam_decay_cycles[9];
    cpu_sta_abs(OAMDATA, 0x7B);
    CHECK("real CPU OAMDATA write stores through the decay-aware path",
          ppu.oam[9 * 8] == 0x7B);
    CHECK("real CPU OAMDATA write refreshes its row",
          ppu.oam_decay_cycles[9] > before_cpu_write);

    reset_video(0);
    for (int i = 0; i < 256; ++i) write_mem(0x0200 + i, (uint8_t)i);
    ppu.oam_addr = 0;
    ppu_oam_dma(2);
    cpu_step(&cpu);
    bool dma_refreshed_all_rows = true;
    for (unsigned row = 0; row < 32; ++row)
        dma_refreshed_all_rows &= ppu.oam_decay_cycles[row] != 0;
    CHECK("OAM DMA refreshes every row through the production $2004 write path",
          dma_refreshed_all_rows);

    reset_video(0);
    cpu_total_cycles = 4300;
    ppu.scanline = 20;
    ppu.dot = 65;
    set_render_mask(0x10);
    ppu.oam_addr = 10 * 8;
    ppu.oam[10 * 8] = 20;
    ppu.oam_decay_cycles[10] = 100;
    ppu_step_dots(1);
    CHECK("sprite evaluation refreshes the primary OAM row it reads",
          ppu.oam_decay_cycles[10] == 4300);

    reset_video(0);
    cpu_total_cycles = 7000;
    ppu.scanline = 100;
    ppu.dot = 0;
    set_render_mask(0);
    ppu.oam_addr = 11 * 8;
    ppu.oam_decay_cycles[11] = 25;
    ppu.oam_decay_cycles[12] = 25;
    ppu_step_dots(1);
    CHECK("blanking refreshes the selected primary OAM row",
          ppu.oam_decay_cycles[11] == 7000 && ppu.oam_decay_cycles[12] == 25);

    reset_video_region(0, NES_REGION_PAL);
    cpu_total_cycles = 8000;
    ppu.scanline = 265;
    ppu.dot = 2;
    set_render_mask(0);
    ppu.oam_addr = 0x17;
    ppu.oam_decay_cycles[2] = 40;
    ppu.oam_decay_cycles[3] = 40;
    ppu_step_dots(1);
    CHECK("PAL late-vblank OAM clock advances the address", ppu.oam_addr == 0x18);
    CHECK("PAL late-vblank OAM clock refreshes the row it enters",
          ppu.oam_decay_cycles[3] == 8000 && ppu.oam_decay_cycles[2] == 40);

    reset_video_region(0, NES_REGION_DENDY);
    memset(&ppu.oam[13 * 8], 0x4D, 8);
    ppu.oam_addr = 13 * 8;
    ppu.oam_decay_cycles[13] = 0;
    cpu_total_cycles = PPU_OAM_DECAY_CPU_CYCLES + 1;
    (void)ppu_reg_read(OAMDATA);
    CHECK("Dendy opt-in profile uses the same deterministic OAM decay model",
          oam_row_matches_decay_value(13));

    nes_set_region(NES_REGION_NTSC);
    ppu.oam[14 * 8] = 0x9E;
    ppu.oam_decay_cycles[14] = 777;
    cpu_total_cycles = 9000;
    ppu_soft_reset(&ppu);
    CHECK("soft reset preserves OAM bytes while clearing decay timestamps",
          ppu.oam[14 * 8] == 0x9E && ppu.oam_decay_cycles[14] == 0);
    ppu.oam_addr = 14 * 8;
    (void)ppu_reg_read(OAMDATA);
    CHECK("cleared soft-reset timestamp participates in later deterministic decay",
          oam_row_matches_decay_value(14));

    ppu_set_oam_decay(false);
    reset_video(0);
    memset(&ppu.oam[15 * 8], 0xEF, 8);
    ppu.oam_addr = 15 * 8;
    ppu.oam_decay_cycles[15] = 0;
    cpu_total_cycles = PPU_OAM_DECAY_CPU_CYCLES * 4u;
    CHECK("compatibility profile leaves OAM decay disabled",
          ppu_reg_read(OAMDATA) == 0xEF && ppu.oam[15 * 8] == 0xEF);

    ppu_set_oam_decay(saved_decay);
    nes_set_region(NES_REGION_NTSC);
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

static void test_late_register_reads(void) {
    reset_video(0);
    ppu.scanline = (int)nes_timing()->scanlines - 1;
    ppu.dot = 1;
    ppu.status = 0xE0;
    uint8_t status = ppu_reg_read(PPUSTATUS);
    CHECK("status read latches vblank before the read finishes", (status & 0xE0) == 0xE0);
    ppu_step_dots(1);
    status = ppu_reg_read_finish(PPUSTATUS, status);
    CHECK("status read samples sprite flags at the end of the read", (status & 0xE0) == 0x80);

    reset_video(0);
    ppu.scanline = 0;
    ppu.dot = 65;
    set_render_mask(0x10);
    ppu.oam_addr = 0;
    ppu.oam_bus = 0x7F;
    ppu.oam_read_latch = 0x7F;
    ppu.oam[0] = 0x12;
    uint8_t oam = ppu_reg_read(OAMDATA);
    ppu_step_dots(1);
    CHECK("sprite evaluation advances the internal OAM bus", ppu.oam_bus == 0x12);
    oam = ppu_reg_read_finish(OAMDATA, oam);
    CHECK("OAMDATA read finishes from the CPU-facing OAM output latch", oam == 0x7F);
    ppu_step_dots(1);
    CHECK("OAM output latch follows the internal bus one PPU clock later", ppu.oam_read_latch == 0x12);
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
    test_regional_video();
    test_video_reset();
    test_sprite_shifters();
    test_late_register_reads();
    printf("PPU: %d checks, %d failures\n", checks, failures);
    return failures;
}
