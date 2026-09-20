/*
 * audio_mix.c - Presentation volume and stereo balance settings
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "audio_mix.h"
#include <stdio.h>
#include <string.h>

static NesAudioMixSettings current = {
    {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100},
    {0}, 100, false
};
static float gains[2][NES_AUDIO_CHANNEL_COUNT] = {
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
};
static bool default_channels = true;
static bool has_panning;

void nes_audio_mix_defaults(NesAudioMixSettings *settings) {
    if (!settings) return;
    memset(settings, 0, sizeof(*settings));
    for (unsigned i = 0; i < NES_AUDIO_CHANNEL_COUNT; ++i) settings->volume[i] = 100;
    settings->master_volume = 100;
}

void nes_audio_mix_get(NesAudioMixSettings *settings) {
    if (settings) *settings = current;
}

const char *nes_audio_channel_name(unsigned channel) {
    static const char *const names[NES_AUDIO_CHANNEL_COUNT] = {
        "Pulse 1", "Pulse 2", "Triangle", "Noise", "DMC", "FDS", "MMC5",
        "Namco 163", "Sunsoft 5B", "VRC6", "VRC7", "EPSM", "Cartridge PCM"
    };
    return channel < NES_AUDIO_CHANNEL_COUNT ? names[channel] : "Unknown channel";
}

bool nes_audio_mix_set(const NesAudioMixSettings *settings, char *error, size_t error_size) {
    if (!settings || settings->master_volume > 100) {
        if (error && error_size) snprintf(error, error_size, "Master volume must be from 0 to 100 percent");
        return false;
    }
    for (unsigned i = 0; i < NES_AUDIO_CHANNEL_COUNT; ++i) {
        if (settings->volume[i] > 200 || settings->pan[i] < -100 || settings->pan[i] > 100) {
            if (error && error_size)
                snprintf(error, error_size, "%s needs volume 0 to 200 and panning -100 to 100",
                         nes_audio_channel_name(i));
            return false;
        }
    }

    current = *settings;
    default_channels = true;
    has_panning = false;
    for (unsigned i = 0; i < NES_AUDIO_CHANNEL_COUNT; ++i) {
        float volume = (float)settings->volume[i] / 100.0f;
        float pan = (float)settings->pan[i] / 100.0f;
        gains[0][i] = volume * (1.0f - pan);
        gains[1][i] = volume * (1.0f + pan);
        if (settings->volume[i] != 100 || settings->pan[i] != 0) default_channels = false;
        if (settings->pan[i] != 0) has_panning = true;
    }
    if (error && error_size) error[0] = 0;
    return true;
}

bool nes_audio_mix_channels_default(void) { return default_channels; }
bool nes_audio_mix_has_panning(void) { return has_panning; }
const float *nes_audio_mix_side_gains(bool right) { return gains[right ? 1 : 0]; }

void nes_audio_mix_master(float *left, float *right) {
    float gain = current.muted ? 0.0f : (float)current.master_volume / 100.0f;
    if (left) *left *= gain;
    if (right) *right *= gain;
}
