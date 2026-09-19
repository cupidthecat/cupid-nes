/*
 * ppu_accuracy_video.h - PPU video regression tests
 *
 * Author: @frankischilling
 *
 * These checks cover regional video timing, color output, and rendering behavior.
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
#ifndef PPU_ACCURACY_VIDEO_H
#define PPU_ACCURACY_VIDEO_H

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

static void test_reset_suppression(void) {
    reset_video(0);
    CHECK("PPU reset suppression defaults off", !ppu_reset_suppression_enabled());

    ppu.ctrl = 0xA4;
    ppu.mask = 0x1E;
    ppu.status = 0xE0;
    ppu.v = 0x27A5;
    ppu.t = 0x1357;
    ppu.x = 5;
    ppu.w = 1;
    ppu.scanline = 123;
    ppu.dot = 211;
    ppu.odd_frame = true;
    ppu.rendering_enabled = true;
    ppu.fetches_enabled = true;
    ppu.total_cycles = 654321;
    ppu.cpu_clock_phase = 2;
    ppu.oam_decay_cycles[0] = 111;
    ppu.oam_decay_cycles[31] = 222;
    ppu.oam[7] = 0x6D;

    ppu_set_reset_suppression(true);
    CHECK("PPU reset suppression can be enabled", ppu_reset_suppression_enabled());
    ppu_soft_reset(&ppu);
    CHECK("suppressed soft reset preserves PPU registers and scroll latches",
          ppu.ctrl == 0xA4 && ppu.mask == 0x1E && ppu.status == 0xE0
          && ppu.v == 0x27A5 && ppu.t == 0x1357 && ppu.x == 5 && ppu.w == 1);
    CHECK("suppressed soft reset preserves raster and rendering state",
          ppu.scanline == 123 && ppu.dot == 211 && ppu.odd_frame
          && ppu.rendering_enabled && ppu.fetches_enabled && ppu.total_cycles == 654321);
    CHECK("suppressed soft reset still resets PPU clock and OAM-decay bookkeeping",
          ppu.cpu_clock_phase == 0 && ppu.oam_decay_cycles[0] == 0
          && ppu.oam_decay_cycles[31] == 0 && ppu.oam[7] == 0x6D);

    ppu_set_reset_suppression(false);
    ppu.ctrl = 0x80;
    ppu.mask = 0x18;
    ppu.w = 1;
    ppu_soft_reset(&ppu);
    CHECK("ordinary soft reset still clears control and render state when suppression is disabled",
          !ppu.ctrl && !ppu.mask && !ppu.w && !ppu.rendering_enabled && !ppu.fetches_enabled);
}

static bool all_bytes_equal(const uint8_t *bytes, size_t size, uint8_t value) {
    for (size_t i = 0; i < size; ++i) {
        if (bytes[i] != value) return false;
    }
    return true;
}

static void test_power_on_ram_profiles(void) {
    static const uint8_t boot_palette[PPU_PALETTE_SIZE] = {
        0x09,0x01,0x00,0x01,0x00,0x02,0x02,0x0D,
        0x08,0x10,0x08,0x24,0x00,0x00,0x04,0x2C,
        0x09,0x01,0x34,0x03,0x00,0x04,0x00,0x14,
        0x08,0x3A,0x00,0x02,0x00,0x20,0x2C,0x08
    };
    uint8_t first_ram[sizeof(ram)];
    uint8_t first_vram[sizeof(ppu_vram)];
    uint8_t first_oam[sizeof(ppu.oam)];
    uint8_t first_secondary[sizeof(ppu.secondary_oam)];
    uint8_t first_palette[sizeof(ppu_palette)];

    CHECK("power-on RAM state names reject unknown profiles",
          !nes_set_ram_power_on_state_name("unknown"));

    CHECK("default power-on RAM profile selects compatibility state",
          nes_set_ram_power_on_state(NES_RAM_POWER_DEFAULT));
    nes_set_randomize_vblank(false);
    memset(ram, 0xA5, sizeof(ram));
    memset(ppu_vram, 0xA5, sizeof(ppu_vram));
    memset(ppu.oam, 0xA5, sizeof(ppu.oam));
    ppu_power_on(&ppu);
    CHECK("default power-on profile keeps zeroed CPU and nametable RAM",
          cpu_power_on(&cpu) && all_bytes_equal(ram, sizeof(ram), 0)
          && all_bytes_equal(ppu_vram, sizeof(ppu_vram), 0));
    CHECK("default power-on profile keeps all-ones primary and secondary OAM",
          all_bytes_equal(ppu.oam, sizeof(ppu.oam), 0xFF)
          && all_bytes_equal(ppu.secondary_oam, sizeof(ppu.secondary_oam), 0xFF));
    CHECK("nonrandom power-on profiles keep the fixed boot palette",
          memcmp(ppu_palette, boot_palette, sizeof(boot_palette)) == 0);
    CHECK("power-on VBL randomization stays disabled independently", !(ppu.status & 0x80));

    CHECK("zero power-on RAM profile is selectable",
          nes_set_ram_power_on_state_name("zero"));
    ppu_power_on(&ppu);
    CHECK("zero profile clears CPU internal RAM on hard power-on",
          cpu_power_on(&cpu) && all_bytes_equal(ram, sizeof(ram), 0));
    CHECK("zero profile clears PPU nametable and OAM storage",
          all_bytes_equal(ppu_vram, sizeof(ppu_vram), 0)
          && all_bytes_equal(ppu.oam, sizeof(ppu.oam), 0)
          && all_bytes_equal(ppu.secondary_oam, sizeof(ppu.secondary_oam), 0));
    CHECK("zero profile still uses the fixed nonrandom palette",
          memcmp(ppu_palette, boot_palette, sizeof(boot_palette)) == 0);

    CHECK("ones power-on RAM profile is selectable",
          nes_set_ram_power_on_state_name("ones"));
    ppu_power_on(&ppu);
    CHECK("ones profile fills CPU internal RAM on hard power-on",
          cpu_power_on(&cpu) && all_bytes_equal(ram, sizeof(ram), 0xFF));
    CHECK("ones profile fills PPU nametable and OAM storage",
          all_bytes_equal(ppu_vram, sizeof(ppu_vram), 0xFF)
          && all_bytes_equal(ppu.oam, sizeof(ppu.oam), 0xFF)
          && all_bytes_equal(ppu.secondary_oam, sizeof(ppu.secondary_oam), 0xFF));
    CHECK("ones profile still uses the fixed nonrandom palette",
          memcmp(ppu_palette, boot_palette, sizeof(boot_palette)) == 0);

    CHECK("random power-on RAM profile is selectable",
          nes_set_ram_power_on_state_name("random"));
    nes_seed_power_on_random(0x12345678u);
    ppu_power_on(&ppu);
    CHECK("random profile initializes CPU internal RAM", cpu_power_on(&cpu));
    memcpy(first_ram, ram, sizeof(first_ram));
    memcpy(first_vram, ppu_vram, sizeof(first_vram));
    memcpy(first_oam, ppu.oam, sizeof(first_oam));
    memcpy(first_secondary, ppu.secondary_oam, sizeof(first_secondary));
    memcpy(first_palette, ppu_palette, sizeof(first_palette));
    CHECK("random profile does not collapse CPU or PPU RAM to a constant fill",
          !all_bytes_equal(ram, sizeof(ram), ram[0])
          && !all_bytes_equal(ppu_vram, sizeof(ppu_vram), ppu_vram[0])
          && !all_bytes_equal(ppu.oam, sizeof(ppu.oam), ppu.oam[0]));
    bool palette_range = true;
    for (unsigned i = 0; i < PPU_PALETTE_SIZE; ++i) {
        if (ppu_palette[i] > 0x3F) palette_range = false;
    }
    CHECK("random palette RAM is limited to six-bit PPU values", palette_range);

    nes_seed_power_on_random(0x12345678u);
    ppu_power_on(&ppu);
    CHECK("reseeded random profile initializes CPU internal RAM", cpu_power_on(&cpu));
    CHECK("same power-on seed reproduces CPU internal RAM",
          memcmp(ram, first_ram, sizeof(first_ram)) == 0);
    CHECK("same power-on seed reproduces PPU RAM areas",
          memcmp(ppu_vram, first_vram, sizeof(first_vram)) == 0
          && memcmp(ppu.oam, first_oam, sizeof(first_oam)) == 0
          && memcmp(ppu.secondary_oam, first_secondary, sizeof(first_secondary)) == 0
          && memcmp(ppu_palette, first_palette, sizeof(first_palette)) == 0);

    ram[0x21] = 0x6A;
    ppu_vram[0x123] = 0x5C;
    ppu_palette[7] = 0x2D;
    ppu.oam[9] = 0xA6;
    ppu_soft_reset(&ppu);
    cpu_soft_reset(&cpu);
    CHECK("soft reset does not rerun CPU or PPU power-on RAM initialization",
          ram[0x21] == 0x6A && ppu_vram[0x123] == 0x5C
          && ppu_palette[7] == 0x2D && ppu.oam[9] == 0xA6);

    CHECK("default RAM profile can randomize only the power-on VBL flag",
          nes_set_ram_power_on_state(NES_RAM_POWER_DEFAULT));
    nes_set_randomize_vblank(true);
    nes_seed_power_on_random(0xCAFEBABEu);
    bool expected_vblank = nes_power_on_random_bool();
    nes_seed_power_on_random(0xCAFEBABEu);
    ppu_power_on(&ppu);
    CHECK("power-on VBL uses the controlled random source independently of RAM",
          ((ppu.status & 0x80) != 0) == expected_vblank
          && all_bytes_equal(ppu_vram, sizeof(ppu_vram), 0));
    nes_set_randomize_vblank(false);
    nes_seed_power_on_random(0xCAFEBABEu);
    ppu_power_on(&ppu);
    CHECK("disabled power-on VBL randomization always clears the startup flag",
          !(ppu.status & 0x80));

    nes_set_ram_power_on_state(NES_RAM_POWER_DEFAULT);
    nes_set_randomize_vblank(false);
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

#endif // PPU_ACCURACY_VIDEO_H

