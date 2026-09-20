/*
 * presentation.h - Regional cropping and observed video layers
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_VIDEO_PRESENTATION_H
#define CUPID_VIDEO_PRESENTATION_H

#include "video_trace.h"
#include "../system/timing.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    unsigned left, right, top, bottom;
} NesVideoOverscan;

typedef struct {
    NesVideoOverscan overscan[3]; /* NTSC, PAL, Dendy. */
    bool show_background;
    bool show_sprites;
} NesVideoPresentationSettings;

typedef struct {
    const uint32_t *pixels; /* One or two uncropped 256 by 240 screens, side by side. */
    const uint16_t *signals[2];
    const NesVideoTraceFrame *trace[2];
    unsigned phases[2];
    unsigned screens;
    NesRegion region;
    bool vs_system;
    bool composite;
} NesVideoPresentationSource;

typedef struct {
    const uint32_t *pixels;
    unsigned width, height;
    unsigned screens;
    NesVideoOverscan overscan;
    bool filtered;
    bool layers_ready;
} NesVideoPresentationFrame;

void nes_video_presentation_defaults(NesVideoPresentationSettings *settings);
void nes_video_presentation_get(NesVideoPresentationSettings *settings);
bool nes_video_presentation_set(const NesVideoPresentationSettings *settings,
                                 char *error, size_t error_size);
bool nes_video_overscan_valid(NesVideoOverscan overscan);
unsigned nes_video_region_index(NesRegion region);

/* Output storage lasts until the next render call. No method reads a hardware
 * register or changes the original PPU framebuffer or light-sensor signal. */
bool nes_video_presentation_render(const NesVideoPresentationSource *source,
                                    NesVideoPresentationFrame *frame,
                                    char *error, size_t error_size);
bool nes_video_presentation_aim(const NesVideoPresentationFrame *frame,
                                 int x, int y, unsigned *side, int *nes_x, int *nes_y);

#ifdef __cplusplus
}
#endif
#endif
