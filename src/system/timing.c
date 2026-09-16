/*
 * timing.c - NES region timing tables
 *
 * Author: @frankischilling
 *
 * This file stores NTSC, PAL, and Dendy timing values and selects the active hardware
 * region used by the CPU, PPU, APU, and frontend frame pacing code.
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
#include "timing.h"

static const NesTiming timings[] = {
    {NES_REGION_NTSC, 12, 4, 262, 241, 1789773.0, 1789773.0 * 3.0 / (341.0 * 262.0 - 0.5)},
    {NES_REGION_PAL, 16, 5, 312, 241, 1662607.0, 1662607.0 * 3.2 / (341.0 * 312.0)},
    {NES_REGION_DENDY, 15, 5, 312, 291, 1773448.0, 1773448.0 * 3.0 / (341.0 * 312.0)}
};

static const NesTiming *active_timing = &timings[NES_REGION_NTSC];

const NesTiming *nes_timing_for_region(NesRegion region) {
    if ((unsigned)region > NES_REGION_DENDY)
        region = NES_REGION_NTSC;
    return &timings[region];
}

const NesTiming *nes_timing(void) {
    return active_timing;
}

void nes_set_region(NesRegion region) {
    active_timing = nes_timing_for_region(region);
}
