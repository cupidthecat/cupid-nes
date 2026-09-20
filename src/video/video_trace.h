/*
 * video_trace.h - Presentation data collected from completed PPU fetches
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_VIDEO_TRACE_H
#define CUPID_VIDEO_TRACE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    NES_VIDEO_TRACE_HD = 1u,
    NES_VIDEO_TRACE_LAYERS = 2u,
    NES_VIDEO_TRACE_EXPORT = 4u,
    NES_VIDEO_TRACE_WIDTH = 256,
    NES_VIDEO_TRACE_HEIGHT = 240,
    NES_VIDEO_TRACE_SPRITES = 4
};

typedef struct {
    uint32_t index;
    uint8_t bytes[16];
    uint8_t offset;
    bool ram;
    bool valid;
} NesVideoChr;

typedef struct {
    NesVideoChr chr;
    uint32_t palette;
    uint32_t rgb;
    uint16_t address;
    uint8_t x;
    uint8_t y;
    uint8_t attributes;
    uint8_t palette_offset;
    uint8_t color_index;
    uint8_t color;
} NesVideoTile;

typedef struct {
    NesVideoTile background;
    NesVideoTile sprites[NES_VIDEO_TRACE_SPRITES];
    uint32_t backdrop_rgb;
    uint32_t original_rgb;
    uint32_t selected_sprite_rgb;
    uint16_t original_signal;
    uint16_t scroll_address;
    uint8_t fine_x;
    uint8_t backdrop;
    uint8_t mask;
    uint8_t sprite_count;
    uint8_t selected_sprite_color;
    uint8_t selected_sprite_index;
    uint8_t selected_sprite_attributes;
} NesVideoPixel;

typedef struct {
    const NesVideoPixel *pixels;
    uint64_t frame_number;
    uint64_t generation;
    bool complete;
} NesVideoTraceFrame;

/* Owned by the application thread. Hardware serializers deliberately exclude
 * these buffers: they describe presentation and never drive a device or bus. */
extern bool nes_video_trace_active;
bool nes_video_trace_use(unsigned user, bool enabled);
void nes_video_trace_shutdown(void);
void nes_video_trace_reset_side(unsigned side);
uint64_t nes_video_trace_generation(unsigned side);
const NesVideoTraceFrame *nes_video_trace_frame(unsigned side);

/* Core hooks observe bytes already read and do not perform emulated accesses. */
void nes_video_trace_begin_read(unsigned side);
void nes_video_trace_chr_read(const uint8_t *bytes, size_t size, size_t offset, bool ram);
const NesVideoChr *nes_video_trace_last_read(unsigned side);
NesVideoPixel *nes_video_trace_pixel(unsigned side, unsigned x, unsigned y);
void nes_video_trace_complete(unsigned side, uint64_t frame_number);

/* Recompose original layers for display controls. The raw PPU output and light
 * sensor retain their original pixels regardless of these switches. */
uint32_t nes_video_trace_layer_rgb(const NesVideoPixel *pixel, bool background, bool sprites);
uint16_t nes_video_trace_layer_signal(const NesVideoPixel *pixel, bool background, bool sprites);

#ifdef __cplusplus
}
#endif
#endif
