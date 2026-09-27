/*
 * video_trace.c - Bounded host buffers for layers and replacement graphics
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "video_trace.h"
#include "../system/vs_system.h"
#include <stdlib.h>
#include <string.h>

enum { PIXELS = NES_VIDEO_TRACE_WIDTH * NES_VIDEO_TRACE_HEIGHT };
typedef struct {
    NesVideoPixel *working;
    NesVideoPixel *completed;
    NesVideoTraceFrame frame;
    NesVideoChr last_read;
    unsigned next_pixel;
    bool ordered;
    uint64_t generation;
} TraceSide;

bool nes_video_trace_active;
static unsigned users;
static uint64_t next_generation = 1;
static TraceSide sides[2];

static void release_buffers(void) {
    for (unsigned i = 0; i < 2; ++i) {
        free(sides[i].working);
        free(sides[i].completed);
        memset(&sides[i], 0, sizeof(sides[i]));
    }
}

bool nes_video_trace_use(unsigned user, bool enabled) {
    const unsigned known = NES_VIDEO_TRACE_HD | NES_VIDEO_TRACE_LAYERS | NES_VIDEO_TRACE_EXPORT | NES_VIDEO_TRACE_BUILDER;
    if (!user || (user & ~known)) return false;
    unsigned requested = enabled ? users | user : users & ~user;
    if (requested && !users) {
        TraceSide prepared[2] = {0};
        for (unsigned i = 0; i < 2; ++i) {
            prepared[i].working = calloc(PIXELS, sizeof(NesVideoPixel));
            prepared[i].completed = calloc(PIXELS, sizeof(NesVideoPixel));
            if (!prepared[i].working || !prepared[i].completed) {
                for (unsigned j = 0; j < 2; ++j) {
                    free(prepared[j].working);
                    free(prepared[j].completed);
                }
                return false;
            }
            prepared[i].frame.pixels = prepared[i].completed;
            prepared[i].generation = next_generation++;
        }
        memcpy(sides, prepared, sizeof(sides));
    } else if (!requested) {
        release_buffers();
    }
    users = requested;
    nes_video_trace_active = users != 0;
    return true;
}

void nes_video_trace_shutdown(void) {
    release_buffers();
    users = 0;
    nes_video_trace_active = false;
}

void nes_video_trace_reset_side(unsigned side) {
    if (side >= 2) return;
    TraceSide *trace = &sides[side];
    trace->generation = next_generation++;
    trace->frame.complete = false;
    trace->ordered = false;
    trace->next_pixel = 0;
    trace->last_read.valid = false;
}

uint64_t nes_video_trace_generation(unsigned side) {
    return side < 2 ? sides[side].generation : 0;
}

const NesVideoTraceFrame *nes_video_trace_frame(unsigned side) {
    return nes_video_trace_active && side < 2 ? &sides[side].frame : NULL;
}

void nes_video_trace_begin_read(unsigned side) {
    if (nes_video_trace_active && side < 2) sides[side].last_read.valid = false;
}

void nes_video_trace_chr_read(const uint8_t *bytes, size_t size, size_t offset, bool ram) {
    if (!nes_video_trace_active) return;
    unsigned side = vs_active_side();
    if (side >= 2) return;
    NesVideoChr *tile = &sides[side].last_read;
    tile->valid = false;
    size_t base = offset & ~(size_t)15;
    if (!bytes || offset >= size || base > size || size - base < 16 || base / 16 > UINT32_MAX) return;
    tile->index = (uint32_t)(base / 16);
    tile->offset = (uint8_t)(offset & 15u);
    memcpy(tile->bytes, bytes + base, sizeof(tile->bytes));
    tile->ram = ram;
    tile->valid = true;
}

const NesVideoChr *nes_video_trace_last_read(unsigned side) {
    return nes_video_trace_active && side < 2 ? &sides[side].last_read : NULL;
}

NesVideoPixel *nes_video_trace_pixel(unsigned side, unsigned x, unsigned y) {
    if (!nes_video_trace_active || side >= 2 || x >= NES_VIDEO_TRACE_WIDTH || y >= NES_VIDEO_TRACE_HEIGHT) return NULL;
    TraceSide *trace = &sides[side];
    unsigned index = y * NES_VIDEO_TRACE_WIDTH + x;
    if (!index) {
        trace->ordered = true;
        trace->next_pixel = 0;
    }
    if (index != trace->next_pixel) trace->ordered = false;
    trace->next_pixel = index + 1;
    NesVideoPixel *pixel = trace->working + index;
    memset(pixel, 0, sizeof(*pixel));
    return pixel;
}

void nes_video_trace_complete(unsigned side, uint64_t frame_number) {
    if (!nes_video_trace_active || side >= 2) return;
    TraceSide *trace = &sides[side];
    NesVideoPixel *completed = trace->working;
    trace->working = trace->completed;
    trace->completed = completed;
    trace->frame = (NesVideoTraceFrame){completed, frame_number, trace->generation,
                                       trace->ordered && trace->next_pixel == PIXELS};
    trace->next_pixel = 0;
    trace->ordered = false;
}

static bool visible_sprite(const NesVideoPixel *pixel, bool background, bool sprites) {
    return sprites && pixel->selected_sprite_index
        && (!(pixel->selected_sprite_attributes & 0x20)
            || !background || !pixel->background.color_index);
}

uint32_t nes_video_trace_layer_rgb(const NesVideoPixel *pixel, bool background, bool sprites) {
    if (!pixel) return 0xFF000000;
    if (background && sprites) return pixel->original_rgb;
    if (visible_sprite(pixel, background, sprites)) return pixel->selected_sprite_rgb;
    return background && pixel->background.color_index ? pixel->background.rgb : pixel->backdrop_rgb;
}

uint16_t nes_video_trace_layer_signal(const NesVideoPixel *pixel, bool background, bool sprites) {
    if (!pixel) return 0;
    if (background && sprites) return pixel->original_signal;
    uint16_t color = pixel->backdrop;
    if (visible_sprite(pixel, background, sprites)) color = pixel->selected_sprite_color;
    else if (background && pixel->background.color_index) color = pixel->background.color;
    color &= (pixel->mask & 1) ? 0x30u : 0x3Fu;
    return (uint16_t)(color | ((uint16_t)(pixel->mask & 0xE0u) << 1));
}
