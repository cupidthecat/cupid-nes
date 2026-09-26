/*
 * pixel_scalers.cpp
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Presentation scaler integration. SPDX-License-Identifier: GPL-3.0-or-later */
#include "pixel_scalers.h"
#include "../third_party/xbrz/xbrz.h"
#include "../third_party/hqx/hqx.h"
#include "../third_party/scale2x/scale2x.h"
#include "../third_party/scale2x/scale3x.h"
#include "../third_party/sai/SaiEagle.h"
#include <algorithm>
#include <mutex>

namespace {
void prescale(const uint32_t *source, unsigned width, unsigned height, unsigned factor, uint32_t *output) {
    size_t stride = static_cast<size_t>(width) * factor;
    for (unsigned y = 0; y < height; ++y) {
        uint32_t *row = output + y * factor * stride;
        for (unsigned x = 0; x < width; ++x) {
            std::fill_n(row + x * factor, factor, source[static_cast<size_t>(y) * width + x]);
        }

        for (unsigned repeat = 1; repeat < factor; ++repeat) {
            std::copy_n(row, stride, row + repeat * stride);
        }
    }
}

void scale_rows(const uint32_t *source, unsigned width, unsigned height, unsigned factor, uint32_t *output) {
    /* With one column, the left/right neighbors are equal and the Scale2x/3x
     * rules reduce to nearest-neighbor expansion. Their row APIs require >=2. */
    if (width == 1) {
        prescale(source, width, height, factor, output);
        return;
    }

    size_t stride = static_cast<size_t>(width) * factor;
    for (unsigned y = 0; y < height; ++y) {
        const uint32_t *above = source + static_cast<size_t>(y ? y - 1 : y) * width;
        const uint32_t *row = source + static_cast<size_t>(y) * width;
        const uint32_t *below = source + static_cast<size_t>(std::min(y + 1, height - 1)) * width;
        uint32_t *target = output + y * factor * stride;
        if (factor == 2) {
            scale2x_32_def(target, target + stride, above, row, below, width);
        } else {
            scale3x_32_def(target, target + stride, target + 2 * stride, above, row, below, width);
        }
    }
}
} // namespace

void nes_pixel_scaler_render(NesPixelFilterKind kind, unsigned factor, uint32_t *source, unsigned width,
                             unsigned height, uint32_t *output, std::vector<uint32_t> &scratch) {
    if (kind >= NES_PIXEL_FILTER_XBRZ2 && kind <= NES_PIXEL_FILTER_XBRZ6) {
        xbrz::scale(factor, source, output, static_cast<int>(width), static_cast<int>(height), xbrz::ColorFormat::ARGB);
    } else if (kind >= NES_PIXEL_FILTER_HQ2X && kind <= NES_PIXEL_FILTER_HQ4X) {
        static std::once_flag initialized;
        std::call_once(initialized, hqxInit);
        hqx(factor, source, output, static_cast<int>(width), static_cast<int>(height));
    } else if (kind == NES_PIXEL_FILTER_SCALE2X || kind == NES_PIXEL_FILTER_SCALE3X) {
        scale_rows(source, width, height, factor, output);
    } else if (kind == NES_PIXEL_FILTER_SCALE4X) {
        /* Two Scale2x passes avoid the upstream Scale4x helper's silent malloc
         * failure and also support a single visible row after overscan. */
        scratch.resize(static_cast<size_t>(width) * height * 4);
        scale_rows(source, width, height, 2, scratch.data());
        scale_rows(scratch.data(), width * 2, height * 2, 2, output);
    } else if (kind == NES_PIXEL_FILTER_2XSAI) {
        twoxsai_generic_xrgb8888(width, height, source, width, output, width * 2);
    } else if (kind == NES_PIXEL_FILTER_SUPER2XSAI) {
        supertwoxsai_generic_xrgb8888(width, height, source, width, output, width * 2);
    } else if (kind == NES_PIXEL_FILTER_SUPEREAGLE) {
        supereagle_generic_xrgb8888(width, height, source, width, output, width * 2);
    } else {
        prescale(source, width, height, factor, output);
    }
}
