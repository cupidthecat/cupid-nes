/*
 * audio_mix.h - Presentation volume and stereo balance settings
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_AUDIO_MIX_H
#define CUPID_AUDIO_MIX_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NES_AUDIO_PULSE1,
    NES_AUDIO_PULSE2,
    NES_AUDIO_TRIANGLE,
    NES_AUDIO_NOISE,
    NES_AUDIO_DMC,
    NES_AUDIO_FDS,
    NES_AUDIO_MMC5,
    NES_AUDIO_NAMCO163,
    NES_AUDIO_SUNSOFT5B,
    NES_AUDIO_VRC6,
    NES_AUDIO_VRC7,
    NES_AUDIO_EPSM,
    NES_AUDIO_CARTRIDGE_PCM,
    NES_AUDIO_CHANNEL_COUNT
} NesAudioChannel;

typedef struct {
    unsigned volume[NES_AUDIO_CHANNEL_COUNT]; /* Percent, 0 through 200. */
    int pan[NES_AUDIO_CHANNEL_COUNT];         /* -100 left through +100 right. */
    unsigned master_volume;                  /* Percent, 0 through 100. */
    bool muted;
} NesAudioMixSettings;

/* Settings belong to the application thread and survive machine state loads.
 * Apply them while emulation and the host audio callback are coordinated. */
void nes_audio_mix_defaults(NesAudioMixSettings *settings);
void nes_audio_mix_get(NesAudioMixSettings *settings);
bool nes_audio_mix_set(const NesAudioMixSettings *settings, char *error, size_t error_size);
const char *nes_audio_channel_name(unsigned channel);
bool nes_audio_mix_channels_default(void);
bool nes_audio_mix_has_panning(void);
const float *nes_audio_mix_side_gains(bool right);

/* Master volume and mute affect listening. Capture and silence detection use
 * the samples before this final gain. Native expansion stereo is retained. */
void nes_audio_mix_master(float *left, float *right);

#ifdef __cplusplus
}
#endif
#endif
