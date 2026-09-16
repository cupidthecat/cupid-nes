/*
 * timing.h - NES region timing interface
 *
 * Author: @frankischilling
 *
 * This header defines the supported NES timing regions and the CPU, PPU, scanline,
 * vblank, frame rate, and clock values shared by the emulator subsystems.
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
#ifndef NES_TIMING_H
#define NES_TIMING_H

typedef enum {
    NES_REGION_NTSC,
    NES_REGION_PAL,
    NES_REGION_DENDY
} NesRegion;

typedef struct {
    NesRegion region;
    unsigned cpu_divider;
    unsigned ppu_divider;
    unsigned scanlines;
    unsigned vblank_scanline;
    double cpu_hz;
    double fps;
} NesTiming;

const NesTiming *nes_timing(void);
const NesTiming *nes_timing_for_region(NesRegion region);
// Select the region before powering on or resetting the emulated hardware.
void nes_set_region(NesRegion region);

#endif
