/*
 * ntsc_settings.c
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Composite picture controls and presets. SPDX-License-Identifier: GPL-3.0-or-later */
#include "ntsc_composite.h"
#include <stdio.h>

static const NtscCompositeSettings presets[NTSC_PRESET_COUNT] = {
    {0, 0, 100, 100, 36, 36, 36, 100, 0, NTSC_FIELDS_SOURCE},
    {0, 0, 100, 100, 12, 24, 24, 100, 0, NTSC_FIELDS_SOURCE},
    {0, 0, 100, 100, 48, 72, 72, 100, 25, NTSC_FIELDS_BLEND},
    {0, 0, 100, 0, 36, 36, 36, 100, 0, NTSC_FIELDS_SOURCE}};

static const NtscCompositeControl controls[NTSC_CONTROL_COUNT] = {
    {"hue", "NTSC hue", " degrees", -180, 180},
    {"brightness", "NTSC brightness", "%", -100, 100},
    {"contrast", "NTSC contrast", "%", 0, 200},
    {"saturation", "NTSC saturation", "%", 0, 200},
    {"luma_width", "NTSC luma filter width", " samples", 1, 96},
    {"i_width", "NTSC I chroma filter width", " samples", 12, 96},
    {"q_width", "NTSC Q chroma filter width", " samples", 12, 96},
    {"gamma", "NTSC gamma", "%", 25, 400},
    {"scanlines", "NTSC scanline strength", "%", 0, 100},
    {"fields", "NTSC field handling", "", 0, NTSC_FIELDS_COUNT - 1}};

static const size_t offsets[NTSC_CONTROL_COUNT] = {
    offsetof(NtscCompositeSettings, hue),       offsetof(NtscCompositeSettings, brightness),
    offsetof(NtscCompositeSettings, contrast),  offsetof(NtscCompositeSettings, saturation),
    offsetof(NtscCompositeSettings, y_width),   offsetof(NtscCompositeSettings, i_width),
    offsetof(NtscCompositeSettings, q_width),   offsetof(NtscCompositeSettings, gamma),
    offsetof(NtscCompositeSettings, scanlines), offsetof(NtscCompositeSettings, fields)};

void ntsc_composite_defaults(NtscCompositeSettings *settings) {
    if (settings) {
        *settings = presets[0];
    }
}

const NtscCompositeControl *ntsc_composite_control_info(unsigned control) {
    return control < NTSC_CONTROL_COUNT ? &controls[control] : NULL;
}

int *ntsc_composite_control_value(NtscCompositeSettings *settings, unsigned control) {
    return settings && control < NTSC_CONTROL_COUNT ? (int *)((unsigned char *)settings + offsets[control]) : NULL;
}

int ntsc_composite_control_get(const NtscCompositeSettings *settings, unsigned control) {
    return settings && control < NTSC_CONTROL_COUNT ? *(const int *)((const unsigned char *)settings + offsets[control])
                                                    : 0;
}

bool ntsc_composite_validate(const NtscCompositeSettings *settings, char *error, size_t error_size) {
    if (!settings) {
        if (error && error_size) {
            snprintf(error, error_size, "NTSC picture settings are required");
        }
        return false;
    }
    for (unsigned i = 0; i < NTSC_CONTROL_COUNT; ++i) {
        int value = ntsc_composite_control_get(settings, i);
        if (value < controls[i].minimum || value > controls[i].maximum) {
            if (error && error_size) {
                snprintf(error, error_size, "%s must be from %d to %d", controls[i].label, controls[i].minimum,
                         controls[i].maximum);
            }
            return false;
        }
    }
    if (error && error_size) {
        error[0] = 0;
    }
    return true;
}

bool ntsc_composite_preset(NtscCompositeSettings *settings, unsigned preset) {
    if (!settings || preset >= NTSC_PRESET_COUNT) {
        return false;
    }
    *settings = presets[preset];
    return true;
}

int ntsc_composite_preset_index(const NtscCompositeSettings *settings) {
    if (!settings) {
        return -1;
    }
    for (unsigned preset = 0; preset < NTSC_PRESET_COUNT; ++preset) {
        unsigned control = 0;
        while (control < NTSC_CONTROL_COUNT &&
               ntsc_composite_control_get(settings, control) == ntsc_composite_control_get(&presets[preset], control)) {
            ++control;
        }
        if (control == NTSC_CONTROL_COUNT) {
            return (int)preset;
        }
    }
    return -1;
}

const char *ntsc_composite_preset_name(unsigned preset) {
    static const char *const names[] = {"Default composite", "Sharp composite", "Soft television", "Monochrome"};
    return preset < NTSC_PRESET_COUNT ? names[preset] : "Custom";
}

const char *ntsc_composite_field_name(unsigned fields) {
    static const char *const names[] = {"Follow PPU phase", "Fixed phase", "Blend three phases"};
    return fields < NTSC_FIELDS_COUNT ? names[fields] : "Invalid field mode";
}
