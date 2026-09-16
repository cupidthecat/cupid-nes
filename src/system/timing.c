/* SPDX-License-Identifier: GPL-3.0-or-later
 * Region-specific CPU, PPU, frame, and vblank timing tables.
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
