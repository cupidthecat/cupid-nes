/*
 * audio_observer.c - Emulation-thread access to completed audio samples
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "audio_observer.h"
#include "../system/execution_policy.h"
#include <math.h>
#include <stddef.h>

enum { AUDIO_OBSERVER_CAPACITY = 8 };

typedef struct {
    NesAudioObserver callback;
    void *context;
    uint32_t token;
} AudioObserverEntry;

static AudioObserverEntry observers[AUDIO_OBSERVER_CAPACITY];
static uint32_t next_token = 1;
static unsigned observer_count;
static bool dispatching;

uint32_t nes_audio_observer_add(NesAudioObserver observer, void *context) {
    if (!observer || dispatching || observer_count == AUDIO_OBSERVER_CAPACITY) return 0;

    for (size_t i = 0; i < AUDIO_OBSERVER_CAPACITY; ++i) {
        if (observers[i].callback) continue;

        uint32_t token;
        bool collision;
        do {
            token = next_token++;
            collision = token == 0;
            for (size_t j = 0; j < AUDIO_OBSERVER_CAPACITY; ++j)
                if (observers[j].callback && observers[j].token == token) collision = true;
        } while (collision);

        observers[i] = (AudioObserverEntry){observer, context, token};
        ++observer_count;
        return token;
    }

    return 0;
}

bool nes_audio_observer_remove(uint32_t token) {
    if (!token) return false;

    for (size_t i = 0; i < AUDIO_OBSERVER_CAPACITY; ++i) {
        if (observers[i].callback && observers[i].token == token) {
            observers[i] = (AudioObserverEntry){0};
            --observer_count;
            return true;
        }
    }

    return false;
}

void nes_audio_observe(unsigned machine, double sample_rate, float left, float right) {
    if (!observer_count || dispatching || !isfinite(sample_rate) || sample_rate <= 0.0
        || (nes_execution_policy() & (NES_EXECUTION_SPECULATIVE | NES_EXECUTION_REWIND))) return;

    dispatching = true;
    for (size_t i = 0; i < AUDIO_OBSERVER_CAPACITY; ++i) {
        if (observers[i].callback)
            observers[i].callback(observers[i].context, machine, sample_rate, left, right);
    }
    dispatching = false;
}
