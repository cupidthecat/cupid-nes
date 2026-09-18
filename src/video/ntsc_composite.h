/*
 * ntsc_composite.h - Optional NTSC composite video reconstruction
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */
#ifndef NES_NTSC_COMPOSITE_H
#define NES_NTSC_COMPOSITE_H

#include <stdbool.h>
#include <stdint.h>
#include "../system/timing.h"

#define NTSC_COMPOSITE_WIDTH 512
#define NTSC_COMPOSITE_HEIGHT 480

bool ntsc_composite_supported(NesRegion region, bool vs_hardware);
void ntsc_composite_filter_frame(const uint16_t *ppu_signal, uint8_t video_phase,
                                 uint32_t *output);

#endif
