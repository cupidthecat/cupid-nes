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
#include <string.h>

static const NesTiming timings[] = {
    {NES_REGION_NTSC, 12, 4, 262, 241, 1789773.0, 1789773.0 * 3.0 / (341.0 * 262.0 - 0.5)},
    {NES_REGION_PAL, 16, 5, 312, 241, 1662607.0, 1662607.0 * 3.2 / (341.0 * 312.0)},
    {NES_REGION_DENDY, 15, 5, 312, 291, 1773448.0, 1773448.0 * 3.0 / (341.0 * 312.0)}
};

static const NesTiming *active_timing = &timings[NES_REGION_NTSC];

static NesRegionMode region_mode = NES_REGION_MODE_AUTO;

static const char *const region_mode_names[] = {"auto", "ntsc", "pal", "dendy"};

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

NesRegionMode nes_region_mode(void) {
    return region_mode;
}

bool nes_set_region_mode(NesRegionMode mode) {
    if ((unsigned)mode > NES_REGION_MODE_DENDY) {
        return false;
    }

    region_mode = mode;
    return true;
}

bool nes_set_region_mode_name(const char *name) {
    if (!name) {
        return false;
    }

    for (unsigned mode = 0; mode <= NES_REGION_MODE_DENDY; ++mode) {
        if (strcmp(name, region_mode_names[mode]) == 0) {
            return nes_set_region_mode((NesRegionMode)mode);
        }
    }

    return false;
}

const char *nes_region_mode_name(void) {
    return region_mode_names[region_mode];
}

const char *nes_region_name(NesRegion region) {
    switch (region) {
        case NES_REGION_NTSC: return "NTSC";
        case NES_REGION_PAL: return "PAL";
        case NES_REGION_DENDY: return "Dendy";
        default: return "unknown";
    }
}

NesRegion nes_resolve_region(NesRegion detected_region) {
    switch (region_mode) {
        case NES_REGION_MODE_NTSC: return NES_REGION_NTSC;
        case NES_REGION_MODE_PAL: return NES_REGION_PAL;
        case NES_REGION_MODE_DENDY: return NES_REGION_DENDY;
        case NES_REGION_MODE_AUTO:
        default: return nes_timing_for_region(detected_region)->region;
    }
}

static bool timing_state_decode(NesStateReader *reader, NesRegion *region,
                                NesRegionMode *mode) {
    uint8_t encoded_region, encoded_mode;
    if (!nes_state_read_u8(reader, &encoded_region)
        || !nes_state_read_u8(reader, &encoded_mode)
        || encoded_region > NES_REGION_DENDY
        || encoded_mode > NES_REGION_MODE_DENDY
        || nes_state_reader_remaining(reader) != 0) return false;
    *region = (NesRegion)encoded_region;
    *mode = (NesRegionMode)encoded_mode;
    return true;
}

bool timing_state_capture(NesStateWriter *writer) {
    return writer
        && nes_state_write_u8(writer, (uint8_t)active_timing->region)
        && nes_state_write_u8(writer, (uint8_t)region_mode);
}

bool timing_state_validate(NesStateReader *reader) {
    NesRegion region;
    NesRegionMode mode;
    return reader && timing_state_decode(reader, &region, &mode);
}

bool timing_state_apply(NesStateReader *reader) {
    NesRegion region;
    NesRegionMode mode;
    if (!reader || !timing_state_decode(reader, &region, &mode)) return false;
    active_timing = &timings[region];
    region_mode = mode;
    return true;
}
