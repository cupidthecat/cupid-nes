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
#include <stddef.h>
#include <stdint.h>
#include "../system/timing.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NTSC_COMPOSITE_WIDTH 512
#define NTSC_COMPOSITE_HEIGHT 480

typedef enum {
    NTSC_CONTROL_HUE,
    NTSC_CONTROL_BRIGHTNESS,
    NTSC_CONTROL_CONTRAST,
    NTSC_CONTROL_SATURATION,
    NTSC_CONTROL_LUMA_WIDTH,
    NTSC_CONTROL_I_WIDTH,
    NTSC_CONTROL_Q_WIDTH,
    NTSC_CONTROL_GAMMA,
    NTSC_CONTROL_SCANLINES,
    NTSC_CONTROL_FIELDS,
    NTSC_CONTROL_COUNT
} NtscCompositeControlId;

typedef enum { NTSC_FIELDS_SOURCE, NTSC_FIELDS_FIXED, NTSC_FIELDS_BLEND, NTSC_FIELDS_COUNT } NtscCompositeFields;

typedef struct {
    int hue, brightness, contrast, saturation;
    int y_width, i_width, q_width;
    int gamma, scanlines, fields;
} NtscCompositeSettings;

typedef struct {
    const char *key, *label, *unit;
    int minimum, maximum;
} NtscCompositeControl;

enum { NTSC_PRESET_COUNT = 4 };

void ntsc_composite_defaults(NtscCompositeSettings *settings);
bool ntsc_composite_validate(const NtscCompositeSettings *settings, char *error, size_t error_size);
const NtscCompositeControl *ntsc_composite_control_info(unsigned control);
int *ntsc_composite_control_value(NtscCompositeSettings *settings, unsigned control);
int ntsc_composite_control_get(const NtscCompositeSettings *settings, unsigned control);
bool ntsc_composite_preset(NtscCompositeSettings *settings, unsigned preset);
int ntsc_composite_preset_index(const NtscCompositeSettings *settings);
const char *ntsc_composite_preset_name(unsigned preset);
const char *ntsc_composite_field_name(unsigned fields);

bool ntsc_composite_supported(NesRegion region, bool vs_hardware);
void ntsc_composite_filter_frame(const uint16_t *ppu_signal, uint8_t video_phase, uint32_t *output);
/* NULL settings selects the original picture. Invalid input leaves output alone.
 * Picture controls never change the source signal, PPU state, or light sensor. */
bool ntsc_composite_filter_frame_settings(const uint16_t *ppu_signal, uint8_t video_phase, uint32_t *output,
                                          const NtscCompositeSettings *settings, char *error, size_t error_size);

#ifdef __cplusplus
}
#endif
#endif
