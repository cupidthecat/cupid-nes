/*
 * audio_observer.h - Emulation-thread access to completed audio samples
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef NES_AUDIO_OBSERVER_H
#define NES_AUDIO_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*NesAudioObserver)(void *context, unsigned machine,
                                 double sample_rate, float left, float right);

/* Registration, removal, and callbacks belong to the emulation thread.
 * Observers see the reconstructed stereo output before the host consumes it.
 * They must not clock or reset the machine from inside a callback. */
uint32_t nes_audio_observer_add(NesAudioObserver observer, void *context);
bool nes_audio_observer_remove(uint32_t token);
void nes_audio_observe(unsigned machine, double sample_rate, float left, float right);

#ifdef __cplusplus
}
#endif

#endif
