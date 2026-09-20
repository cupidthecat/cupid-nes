/*
 * apu_accuracy_noise_profile.h - Noise hardware profile regression tests
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

static void test_apu_noise_profile(void) {
    static const uint16_t ntsc_periods[16] = {4,   8,   16,  32,  64,  96,   128,  160,
                                              202, 254, 380, 508, 762, 1016, 2034, 4068};
    static const uint16_t pal_periods[16] = {4, 8, 14, 30, 60, 88, 118, 148, 188, 236, 354, 472, 708, 944, 1890, 3778};
    static const NesRegion regions[] = {NES_REGION_NTSC, NES_REGION_PAL, NES_REGION_DENDY};

    CHECK("noise mode profile defaults to standard behavior", !apu_noise_mode_disabled());

    reset_audio();
    write_mem(0x400E, 0x00);
    apu.noise.lfsr = 0x41;
    apu.noise.timer = 0;
    apu_step(&apu, 1);
    CHECK("standard long noise mode uses the bit-one tap", apu.noise.lfsr == 0x4020);

    reset_audio();
    write_mem(0x400E, 0x80);
    apu.noise.lfsr = 0x41;
    apu.noise.timer = 0;
    apu_step(&apu, 1);
    CHECK("standard short noise mode uses the bit-six tap", apu.noise.lfsr == 0x20);

    apu_set_disable_noise_mode(true);
    CHECK("oldest noise profile can be selected", apu_noise_mode_disabled());
    apu_power_on(&apu);
    CHECK("noise profile survives power-on", apu_noise_mode_disabled());
    apu_soft_reset(&apu);
    CHECK("noise profile survives soft reset", apu_noise_mode_disabled());
    apu_reset(&apu);
    CHECK("noise profile survives reset", apu_noise_mode_disabled());

    reset_audio();
    write_mem(0x400E, 0x80);
    CHECK("oldest profile retains the written noise mode bit", apu.noise.mode && (apu_read(0x400E) & 0x80));
    apu.noise.lfsr = 0x41;
    apu.noise.timer = 0;
    apu_step(&apu, 1);
    CHECK("oldest profile ignores short mode at an LFSR clock", apu.noise.lfsr == 0x4020);

    apu_set_disable_noise_mode(false);
    apu.noise.lfsr = 0x41;
    apu.noise.timer = 0;
    apu_step(&apu, 1);
    CHECK("noise mode selection is evaluated at each LFSR clock", apu.noise.lfsr == 0x20);

    apu_set_disable_noise_mode(true);
    for (unsigned region_index = 0; region_index < sizeof(regions) / sizeof(regions[0]); ++region_index) {
        const uint16_t *periods = regions[region_index] == NES_REGION_PAL ? pal_periods : ntsc_periods;
        reset_audio_region(regions[region_index]);
        for (unsigned rate = 0; rate < 16; ++rate) {
            write_mem(0x400E, (uint8_t)(0x80u | rate));
            CHECK("noise profile preserves regional period selection",
                  apu.noise.period_idx == rate && apu.noise.period == periods[rate] && apu.noise.mode);
        }
    }

    apu_set_disable_noise_mode(false);
    reset_audio();
}
