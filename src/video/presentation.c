/*
 * presentation.c - Regional cropping and observed video layers
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "presentation.h"
#include "ntsc_composite.h"
#include <stdio.h>
#include <string.h>

static NesVideoPresentationSettings current = {{{0}}, true, true};
static uint32_t composed[2][NTSC_COMPOSITE_WIDTH * NTSC_COMPOSITE_HEIGHT];
static uint32_t output[2 * NTSC_COMPOSITE_WIDTH * NTSC_COMPOSITE_HEIGHT];
static uint16_t layer_signal[256 * 240];

unsigned nes_video_region_index(NesRegion region) {
    switch (region) {
        case NES_REGION_PAL: return 1;
        case NES_REGION_DENDY: return 2;
        default: return 0;
    }
}

bool nes_video_overscan_valid(NesVideoOverscan crop) {
    return crop.left < 256 && crop.right < 256 && crop.top < 240 && crop.bottom < 240
        && crop.left + crop.right < 256 && crop.top + crop.bottom < 240;
}

void nes_video_presentation_defaults(NesVideoPresentationSettings *settings) {
    if (!settings) return;
    memset(settings, 0, sizeof(*settings));
    settings->show_background = settings->show_sprites = true;
}

void nes_video_presentation_get(NesVideoPresentationSettings *settings) {
    if (settings) *settings = current;
}

bool nes_video_presentation_set(const NesVideoPresentationSettings *settings,
                                 char *error, size_t error_size) {
    if (!settings) return false;
    for (unsigned i = 0; i < 3; ++i) {
        if (!nes_video_overscan_valid(settings->overscan[i])) {
            if (error && error_size)
                snprintf(error, error_size, "Overscan must leave at least one pixel on each screen");
            return false;
        }
    }
    bool layers = !settings->show_background || !settings->show_sprites;
    if (!nes_video_trace_use(NES_VIDEO_TRACE_LAYERS, layers)) {
        if (error && error_size) snprintf(error, error_size, "Could not allocate video layer storage");
        return false;
    }
    current = *settings;
    if (error && error_size) error[0] = 0;
    return true;
}

bool nes_video_presentation_render(const NesVideoPresentationSource *source,
                                    NesVideoPresentationFrame *frame,
                                    char *error, size_t error_size) {
    if (!source || !frame || !source->pixels || !source->screens || source->screens > 2) {
        if (error && error_size) snprintf(error, error_size, "A completed one-screen or two-screen frame is required");
        return false;
    }
    bool filtered = source->composite && ntsc_composite_supported(source->region, source->vs_system);
    for (unsigned side = 0; filtered && side < source->screens; ++side) {
        if (!source->signals[side]) {
            if (error && error_size) snprintf(error, error_size, "The composite signal is not available for this frame");
            return false;
        }
    }
    NesVideoOverscan crop = current.overscan[nes_video_region_index(source->region)];
    bool layers = !current.show_background || !current.show_sprites;
    bool layers_ready = true;
    unsigned full_width = filtered ? NTSC_COMPOSITE_WIDTH : 256;
    unsigned full_height = filtered ? NTSC_COMPOSITE_HEIGHT : 240;
    unsigned x_start = crop.left * full_width / 256;
    unsigned x_end = ((256 - crop.right) * full_width + 255) / 256;
    unsigned y_start = crop.top * full_height / 240;
    unsigned y_end = ((240 - crop.bottom) * full_height + 239) / 240;
    unsigned side_width = x_end - x_start;
    unsigned height = y_end - y_start;

    for (unsigned side = 0; side < source->screens; ++side) {
        const NesVideoTraceFrame *trace = source->trace[side];
        bool traced = layers && trace && trace->complete && trace->pixels;
        if (layers && !traced) layers_ready = false;
        if (filtered) {
            const uint16_t *signal = source->signals[side];
            if (layers && traced) {
                for (unsigned i = 0; i < 256 * 240; ++i)
                    layer_signal[i] = nes_video_trace_layer_signal(trace->pixels + i,
                        current.show_background, current.show_sprites);
                signal = layer_signal;
            }
            if (!ntsc_composite_filter_frame_settings(signal, source->phases[side], composed[side],
                                                      source->ntsc_settings, error, error_size)) return false;
        } else {
            for (unsigned y = 0; y < 240; ++y) {
                for (unsigned x = 0; x < 256; ++x) {
                    unsigned pixel = y * 256 + x;
                    composed[side][pixel] = layers && traced
                        ? nes_video_trace_layer_rgb(trace->pixels + pixel,
                            current.show_background, current.show_sprites)
                        : source->pixels[y * 256 * source->screens + side * 256 + x];
                }
            }
        }
        for (unsigned y = 0; y < height; ++y) {
            memcpy(output + y * side_width * source->screens + side * side_width,
                   composed[side] + (y + y_start) * full_width + x_start,
                   side_width * sizeof(*output));
        }
    }
    *frame = (NesVideoPresentationFrame){output, side_width * source->screens, height,
                                        source->screens, crop, filtered, layers_ready};
    if (error && error_size) error[0] = 0;
    return true;
}

bool nes_video_presentation_aim(const NesVideoPresentationFrame *frame,
                                 int x, int y, unsigned *side, int *nes_x, int *nes_y) {
    if (nes_x) *nes_x = -1;
    if (nes_y) *nes_y = -1;
    if (!frame || !frame->screens || !frame->width || !frame->height
        || x < 0 || y < 0 || (unsigned)x >= frame->width || (unsigned)y >= frame->height
        || !nes_video_overscan_valid(frame->overscan)) return false;
    unsigned side_width = frame->width / frame->screens;
    if (!side_width) return false;
    if (side) *side = (unsigned)x / side_width;
    if (nes_x) *nes_x = (int)(frame->overscan.left
        + (uint64_t)((unsigned)x % side_width) * (256 - frame->overscan.left - frame->overscan.right) / side_width);
    if (nes_y) *nes_y = (int)(frame->overscan.top
        + (uint64_t)(unsigned)y * (240 - frame->overscan.top - frame->overscan.bottom) / frame->height);
    return true;
}
