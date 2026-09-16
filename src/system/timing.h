/* SPDX-License-Identifier: GPL-3.0-or-later
 * Region timing values shared by the CPU, PPU, APU, and frontend.
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
