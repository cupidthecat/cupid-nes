/*
 * audio_output.h - Buffered host playback, independent of emulated audio state
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef NES_AUDIO_OUTPUT_H
#define NES_AUDIO_OUTPUT_H
#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct NesAudioOutput NesAudioOutput;
typedef struct {
    double queue_ms;
    uint64_t underruns, missing_frames, dropped_frames;
} NesAudioOutputStats;

NesAudioOutput *nes_audio_output_create(unsigned rate, unsigned block);
/* Close the device before destroying its callback's storage. */
void nes_audio_output_destroy(NesAudioOutput *output);
/* Producer-thread operations. Reset discards pre-transition audio at the next
 * callback without touching machine state or racing the consumer's cursor. */
void nes_audio_output_reset(NesAudioOutput *output);
void nes_audio_output_push(NesAudioOutput *output, const float *stereo, unsigned frames);
bool nes_audio_output_stats(const NesAudioOutput *output, NesAudioOutputStats *stats);
void nes_audio_output_callback(void *context, Uint8 *stream, int bytes);
#ifdef __cplusplus
}
#endif
#endif
