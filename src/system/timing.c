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
#include "execution_policy.h"
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
    if (!nes_execution_allows_host_configuration()) return;
    active_timing = nes_timing_for_region(region);
}

NesRegionMode nes_region_mode(void) {
    return region_mode;
}

bool nes_set_region_mode(NesRegionMode mode) {
    if ((unsigned)mode > NES_REGION_MODE_DENDY) {
        return false;
    }
    if (!nes_execution_allows_host_configuration()) return false;

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

static NesOverclockConfig overclock_requested = {false, 0, 0, true};
static NesOverclockConfig overclock_active = {false, 0, 0, true};
static unsigned overclock_dots;
static bool overclock_pcm;

NesOverclockConfig nes_overclock_config(void) {
    return overclock_requested;
}

NesOverclockConfig nes_overclock_active_config(void) {
    return overclock_active;
}

static bool overclock_valid(const NesOverclockConfig *config) {
    return config && config->postrender_scanlines <= NES_OVERCLOCK_MAX_SCANLINES &&
           config->vblank_scanlines <= NES_OVERCLOCK_MAX_SCANLINES;
}

bool nes_set_overclock_config(const NesOverclockConfig *config) {
    if (!overclock_valid(config) || !nes_execution_allows_host_configuration()) {
        return false;
    }
    overclock_requested = *config;
    return true;
}

bool nes_overclock_extra_active(void) {
    return overclock_dots != 0;
}

unsigned nes_overclock_extra_dots(void) {
    return overclock_dots;
}

void nes_overclock_begin_frame(bool supported, bool dmc_active) {
    overclock_active = overclock_requested;
    if (!supported || (overclock_active.dmc_compatibility && (dmc_active || overclock_pcm))) {
        overclock_active.enabled = false;
    }
    overclock_pcm = false;
    overclock_dots = 0;
}

void nes_overclock_begin_extra(bool before_nmi) {
    if (overclock_active.enabled) {
        overclock_dots =
            341u * (before_nmi ? overclock_active.postrender_scanlines : overclock_active.vblank_scanlines);
    }
}

void nes_overclock_step_dot(void) {
    if (overclock_dots) {
        --overclock_dots;
    }
}

void nes_overclock_note_pcm_write(void) {
    if (overclock_requested.enabled || overclock_active.enabled) {
        overclock_pcm = true;
    }
}

static bool overclock_write(NesStateWriter *writer, const NesOverclockConfig *config) {
    return nes_state_write_bool(writer, config->enabled) && nes_state_write_u16(writer, config->postrender_scanlines) &&
           nes_state_write_u16(writer, config->vblank_scanlines) &&
           nes_state_write_bool(writer, config->dmc_compatibility);
}

static bool overclock_read(NesStateReader *reader, NesOverclockConfig *config) {
    return nes_state_read_bool(reader, &config->enabled) && nes_state_read_u16(reader, &config->postrender_scanlines) &&
           nes_state_read_u16(reader, &config->vblank_scanlines) &&
           nes_state_read_bool(reader, &config->dmc_compatibility) && overclock_valid(config);
}

static bool overclock_default(const NesOverclockConfig *config) {
    return !config->enabled && !config->postrender_scanlines && !config->vblank_scanlines && config->dmc_compatibility;
}

typedef struct {
    uint8_t region, mode;
    NesOverclockConfig requested, active;
    uint32_t dots;
    bool pcm;
} TimingState;

static bool timing_state_decode(NesStateReader *reader, TimingState *state) {
    memset(state, 0, sizeof(*state));
    state->requested.dmc_compatibility = state->active.dmc_compatibility = true;
    if (!nes_state_read_u8(reader, &state->region) || !nes_state_read_u8(reader, &state->mode) ||
        state->region > NES_REGION_DENDY || state->mode > NES_REGION_MODE_DENDY) {
        return false;
    }
    if (!nes_state_reader_remaining(reader)) {
        return true;
    }
    uint8_t version;
    return nes_state_read_u8(reader, &version) && version == 1 && overclock_read(reader, &state->requested) &&
           overclock_read(reader, &state->active) && nes_state_read_u32(reader, &state->dots) &&
           nes_state_read_bool(reader, &state->pcm) &&
           state->dots <= 341u * (state->active.postrender_scanlines > state->active.vblank_scanlines
                                      ? state->active.postrender_scanlines
                                      : state->active.vblank_scanlines) &&
           (!state->dots || state->active.enabled) && nes_state_reader_remaining(reader) == 0;
}

bool timing_state_capture(NesStateWriter *writer) {
    if (!writer || !nes_state_write_u8(writer, (uint8_t)active_timing->region) ||
        !nes_state_write_u8(writer, (uint8_t)region_mode)) {
        return false;
    }
    /* Preserve the legacy bytes for normal hardware timing. */
    if (overclock_default(&overclock_requested) && overclock_default(&overclock_active) && !overclock_dots &&
        !overclock_pcm) {
        return true;
    }
    return nes_state_write_u8(writer, 1) && overclock_write(writer, &overclock_requested) &&
           overclock_write(writer, &overclock_active) && nes_state_write_u32(writer, overclock_dots) &&
           nes_state_write_bool(writer, overclock_pcm);
}

bool timing_state_validate(NesStateReader *reader) {
    TimingState state;
    return reader && timing_state_decode(reader, &state);
}

bool timing_state_apply(NesStateReader *reader) {
    TimingState state;
    if (!reader || !timing_state_decode(reader, &state)) {
        return false;
    }
    active_timing = &timings[state.region];
    region_mode = (NesRegionMode)state.mode;
    overclock_requested = state.requested;
    overclock_active = state.active;
    overclock_dots = state.dots;
    overclock_pcm = state.pcm;
    return true;
}
