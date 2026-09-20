/*
 * ppu_accuracy_sprite_wrap.h - Early PPU sprite-evaluation wrap regressions
 *
 * Author: @frankischilling
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef PPU_ACCURACY_SPRITE_WRAP_H
#define PPU_ACCURACY_SPRITE_WRAP_H

static void seed_wrap_sprite(unsigned index, unsigned slot, bool tall) {
    ppu.oam[index * 4] = 20;
    ppu.oam[index * 4 + 1] = 1;
    ppu.oam[index * 4 + 2] = 0;
    ppu.oam[index * 4 + 3] = (uint8_t)(16 + slot * 16);
    test_chr[tall ? 0x1000 : 0x0010] = 0x80;
}

static void check_sprite_wrap_case(unsigned complete, bool early, bool final_y_in_range, bool tall) {
    ppu_set_sprite_eval_wrap_bug(early);
    ppu_set_revision(early ? PPU_REVISION_2C02_PRE_E : PPU_REVISION_2C02_E_PLUS);
    reset_video(0);
    CHECK("sprite wrap profile survives CPU and PPU reset", ppu_sprite_eval_wrap_bug_enabled() == early);
    for (unsigned sprite = 0; sprite < 64; ++sprite) {
        ppu.oam[sprite * 4] = 240;
    }

    // Evaluation has 96 byte-copy opportunities. Each complete sprite costs
    // three extra copies, leaving this Y as the last one read after wrapping.
    unsigned last = complete ? 31 - 3 * complete : 7;
    for (unsigned sprite = 0; sprite + 1 < complete; ++sprite) {
        seed_wrap_sprite(sprite, sprite, tall);
    }

    seed_wrap_sprite(last + (final_y_in_range ? 0 : 1), complete ? complete - 1 : 0, tall);
    ppu_reg_write(OAMADDR, complete ? 0 : 40 * 4);
    ppu.ctrl = tall ? 0x20 : 0;
    ppu.scanline = 20;
    ppu.dot = 0;
    set_render_mask(0x14);
    ppu_write(0x3F00, 0x09);
    ppu_write(0x3F11, 0x21);
    ppu_write(0x3F1D, 0x2A);
    unsigned pattern = tall ? 0x1FF7 : 0x0FF7;
    test_chr[pattern] = 1;

    Mapper *saved_cart = cart;
    Mapper tracing_cart = *cart;
    pattern_reader = cart->ppu_read;
    tracing_cart.ppu_read = trace_pattern_read;
    pattern_reads = 0;
    cart = &tracing_cart;
    ppu_step_dots(257);
    CHECK("sprite evaluation reaches the primary OAM wrap", ppu.eval_done);
    CHECK("wrapped Y scans preserve the next secondary-OAM slot",
          !ppu.secondary_oam_full && ppu.secondary_index == complete * 4);
    bool extra = early && final_y_in_range;
    CHECK("early wrapped scans retain their final Y coordinate", ppu.secondary_oam[complete * 4] == (extra ? 20 : 240));
    CHECK("wrapped sprite tile, attributes and X retain the secondary-OAM clear value",
          ppu.secondary_oam[complete * 4 + 1] == 0xFF && ppu.secondary_oam[complete * 4 + 2] == 0xFF &&
              ppu.secondary_oam[complete * 4 + 3] == 0xFF);
    CHECK("a partial wrapped entry does not set sprite overflow",
          !(ppu.status & 0x20) && !(ppu.sprite_status_pending & 0x20));

    ppu_step_dots(84);
    cart = saved_cart;
    CHECK("wrapped entries use the ordinary sprite fetch count", ppu.sprite_count == complete + (extra ? 1 : 0));
    CHECK("wrapped entry retains X=255 and palette/flip attributes", ppu.sprite_positions[complete] == 255 &&
                                                                         ppu.sprite_start_dot[complete] == 256 &&
                                                                         ppu.sprite_attributes[complete] == 0xFF);
    CHECK("wrapped Y alone cannot create a sprite-zero identity", ppu.sprite_zero_on_line == (complete >= 2));
    CHECK("sprite evaluation does not add cartridge pattern reads", pattern_reads == 84);
    if (extra) {
        CHECK("wrapped sprite fetches the vertically flipped final row of tile FF",
              pattern_addresses[64 + complete * 2] == pattern && pattern_addresses[65 + complete * 2] == pattern + 8);
        CHECK("wrapped sprite pattern reads retain normal fetch clocks",
              pattern_clocks[64 + complete * 2] == 262 + complete * 8 &&
                  pattern_clocks[65 + complete * 2] == 264 + complete * 8);
        CHECK("wrapped sprite loads its data into the normal pattern shifter",
              ppu.sprite_pattern_lo[complete] == 1 && ppu.sprite_pattern_hi[complete] == 0);
    }

    ppu_step_dots(257);
    CHECK("partial wrapped entry renders a palette-three pixel at X=255",
          ppu.pixel_indices[21 * 256 + 255] == (extra ? 0x2A : 0x09) &&
              framebuffer[21 * 256 + 255] == get_color(extra ? 0x2A : 0x09));
    CHECK("horizontal flip keeps the wrapped sprite off the preceding pixel",
          ppu.pixel_indices[21 * 256 + 254] == 0x09);
    CHECK("wrapped output does not manufacture sprite-zero or overflow flags", !(cpu_lda_abs(PPUSTATUS) & 0x60));
}

static void check_full_secondary_oam_wrap(bool early) {
    ppu_set_sprite_eval_wrap_bug(early);
    reset_video(0);
    for (unsigned sprite = 0; sprite < 64; ++sprite) {
        ppu.oam[sprite * 4] = 240;
    }

    for (unsigned sprite = 0; sprite < 8; ++sprite) {
        seed_wrap_sprite(sprite, sprite, false);
    }

    ppu.scanline = 20;
    ppu.dot = 0;
    set_render_mask(0x14);
    ppu_write(0x3F00, 0x09);
    ppu_write(0x3F1D, 0x2A);
    test_chr[0x0FF7] = 1;
    ppu_step_dots(257);
    CHECK("a full secondary OAM remains intact after wrapping",
          ppu.secondary_oam_full && memcmp(ppu.secondary_oam, ppu.oam, 32) == 0);
    CHECK("full secondary OAM skips whole sprites after evaluation wraps", ppu.eval_done && ppu.oam_addr == 32);
    ppu_step_dots(84);
    CHECK("a full secondary OAM fetches exactly its eight complete sprites", ppu.sprite_count == 8);
    ppu_step_dots(257);
    CHECK("a full secondary OAM cannot append a wrapped sprite", ppu.pixel_indices[21 * 256 + 255] == 0x09);
}

static void test_sprite_evaluation_wrap_profile(void) {
    bool saved_early = ppu_sprite_eval_wrap_bug_enabled();
    PpuRevision saved_revision = ppu_revision();
    for (unsigned complete = 0; complete < 8; ++complete) {
        for (unsigned tall = 0; tall < 2; ++tall) {
            check_sprite_wrap_case(complete, false, true, tall != 0);
            check_sprite_wrap_case(complete, true, true, tall != 0);
            check_sprite_wrap_case(complete, true, false, tall != 0);
        }
    }

    check_full_secondary_oam_wrap(false);
    check_full_secondary_oam_wrap(true);
    ppu_set_sprite_eval_wrap_bug(saved_early);
    ppu_set_revision(saved_revision);
}

#endif
