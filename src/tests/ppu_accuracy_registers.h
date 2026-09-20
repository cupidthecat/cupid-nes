/*
 * ppu_accuracy_registers.h - PPU register regression tests
 *
 * Author: @frankischilling
 *
 * These checks exercise PPU registers, internal pipelines, and timing visible to the
 * CPU.
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
#ifndef PPU_ACCURACY_REGISTERS_H
#define PPU_ACCURACY_REGISTERS_H

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

static void test_oamdata_read_profile(void) {
    bool saved_disabled = ppu_oamdata_read_disabled();
    bool saved_decay = ppu_oam_decay_enabled();

    ppu_set_oamdata_read_disabled(false);
    ppu_set_oam_decay(false);
    reset_video(0);
    ppu_reg_write(PPUCTRL, 0xA5);
    ppu.oam_addr = 0x10;
    ppu.oam[0x10] = 0x3C;
    CHECK("enabled OAMDATA reads drive primary OAM during blanking", ppu_reg_read(OAMDATA) == 0x3C);

    reset_video(0);
    ppu.scanline = 20;
    ppu.dot = 66;
    set_render_mask(0x10);
    ppu.oam_bus = 0x42;
    ppu.oam_read_latch = 0x42;
    CHECK("enabled OAMDATA reads expose the sprite-evaluation bus",
          ppu_reg_read_finish(OAMDATA, ppu_reg_read(OAMDATA)) == 0x42);

    reset_video(0);
    ppu.scanline = 20;
    ppu.dot = 300;
    set_render_mask(0x10);
    ppu.secondary_index = 5;
    ppu.secondary_oam[5] = 0x6D;
    ppu.oam_read_latch = 0x6D;
    CHECK("enabled OAMDATA reads expose the sprite-fetch bus",
          ppu_reg_read_finish(OAMDATA, ppu_reg_read(OAMDATA)) == 0x6D);

    ppu_set_oamdata_read_disabled(true);
    ppu_set_oam_decay(true);
    reset_video(0);
    ppu_reg_write(PPUCTRL, 0xA5);
    ppu.scanline = 100;
    ppu.dot = 20;
    ppu.oam_addr = 0x18;
    memset(&ppu.oam[0x18], 0x77, 8);
    ppu.oam_decay_cycles[3] = 123;
    uint8_t blanking_value = cpu_lda_abs(OAMDATA);
    CHECK("disabled OAMDATA CPU reads preserve the PPU open bus during blanking",
          blanking_value == 0xA5 && ppu.open_bus == 0xA5);
    CHECK("disabled OAMDATA reads do not refresh or decay primary OAM",
          ppu.oam_decay_cycles[3] == 123 && ppu.oam[0x18] == 0x77);

    reset_video(0);
    ppu_reg_write(PPUCTRL, 0xA5);
    ppu.scanline = 20;
    ppu.dot = 66;
    set_render_mask(0x10);
    ppu.oam_bus = 0x12;
    ppu.oam_read_latch = 0x7F;
    uint8_t eval_value = ppu_reg_read(OAMDATA);
    CHECK("disabled OAMDATA reads preserve open bus during sprite evaluation",
          eval_value == 0xA5 && ppu.oam_bus == 0x12);
    CHECK("disabled OAMDATA read completion does not substitute the evaluation latch",
          ppu_reg_read_finish(OAMDATA, eval_value) == 0xA5);

    reset_video(0);
    ppu_reg_write(PPUCTRL, 0xA5);
    ppu.scanline = 20;
    ppu.dot = 300;
    set_render_mask(0x10);
    ppu.secondary_index = 5;
    ppu.secondary_oam[5] = 0x6D;
    ppu.oam_bus = 0x12;
    ppu.oam_read_latch = 0x6D;
    uint8_t fetch_value = ppu_reg_read(OAMDATA);
    CHECK("disabled OAMDATA reads preserve open bus during sprite fetches", fetch_value == 0xA5 && ppu.oam_bus == 0x12);
    CHECK("disabled OAMDATA fetch completion preserves open bus", ppu_reg_read_finish(OAMDATA, fetch_value) == 0xA5);

    reset_video(0);
    ppu_reg_write(PPUCTRL, 0x5A);
    uint64_t decay_cycles = (uint64_t)(nes_timing()->cpu_hz * 4.0 / nes_timing()->fps) + 1;
    cpu_total_cycles += decay_cycles;
    CHECK("disabled OAMDATA reads observe normal PPU open-bus decay", ppu_reg_read(OAMDATA) == 0x00);

    ppu_set_oamdata_read_disabled(saved_disabled);
    ppu_set_oam_decay(saved_decay);
}

static void test_palette_readback_profile(void) {
    bool saved_disabled = ppu_palette_readback_disabled();

    ppu_set_palette_readback_disabled(false);
    reset_video(0);
    ppu.scanline = 241;
    ppu.dot = 20;
    ppu_write(0x2F00, 0x5A);
    ppu_write(0x3F00, 0x21);
    set_data_address(0x3F00);
    ppu.ppudata_buffer = 0x71;
    ppu_reg_write(PPUCTRL, 0xC0);
    CHECK("enabled palette readback reaches palette RAM through a real CPU read", cpu_lda_abs(PPUDATA) == 0xE1);
    ppu_step_dots(6);
    CHECK("enabled palette readback still refills from the external nametable bus",
          ppu.ppudata_buffer == 0x5A && ppu.v == 0x3F01);

    set_data_address(0x3F00);
    ppu.ppudata_buffer = 0x71;
    ppu.mask = 1;
    ppu_reg_write(PPUCTRL, 0xC0);
    CHECK("enabled palette readback keeps grayscale and high open-bus handling", ppu_reg_read(PPUDATA) == 0xE0);
    ppu_step_dots(6);

    ppu_set_palette_readback_disabled(true);
    reset_video(0);
    ppu.scanline = 241;
    ppu.dot = 20;
    ppu_write(0x2F00, 0x5A);
    ppu_write(0x3F00, 0x21);
    set_data_address(0x3F00);
    ppu.ppudata_buffer = 0x71;
    ppu.mask = 1;
    ppu_reg_write(PPUCTRL, 0xC0);
    CHECK("disabled palette readback returns the buffered byte through a real CPU read", cpu_lda_abs(PPUDATA) == 0x71);
    ppu_step_dots(6);
    CHECK("disabled palette readback preserves external refill and address increment",
          ppu.ppudata_buffer == 0x5A && ppu.v == 0x3F01);

    set_data_address(0x3F00);
    ppu.ppudata_buffer = 0x65;
    ppu.mask = 1;
    ppu_reg_write(PPUCTRL, 0xC0);
    CHECK("disabled palette readback ignores palette grayscale and high open-bus bits", ppu_reg_read(PPUDATA) == 0x65);
    ppu_step_dots(3);
    uint8_t delay = ppu.data_read_delay;
    uint8_t cooldown = ppu.data_read_cooldown;
    CHECK("disabled palette readback keeps consecutive-read recovery open bus",
          ppu_reg_read(PPUDATA) == 0x65 && ppu.data_read_delay == delay && ppu.data_read_cooldown == cooldown);
    ppu_step_dots(3);
    CHECK("ignored consecutive palette read does not restart refill or increment",
          ppu.ppudata_buffer == 0x5A && ppu.v == 0x3F01 && !ppu.data_read_delay);
    CHECK("disabled palette readback accepts a read after cooldown", ppu_reg_read(PPUDATA) == 0x5A);
    ppu_step_dots(6);

    ppu_set_palette_readback_disabled(saved_disabled);
}

#endif // PPU_ACCURACY_REGISTERS_H
