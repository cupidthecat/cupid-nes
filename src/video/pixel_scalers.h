/*
 * pixel_scalers.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Internal scaler dispatch. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_PIXEL_SCALERS_H
#define CUPID_PIXEL_SCALERS_H

#include "pixel_filter.h"
#include <vector>

/* One tightly packed screen, with validated dimensions and allocated output.
 * Allocation failures propagate to the presentation API's error handler. */
void nes_pixel_scaler_render(NesPixelFilterKind kind, unsigned factor, uint32_t *source, unsigned width,
                             unsigned height, uint32_t *output, std::vector<uint32_t> &scratch);

#endif
