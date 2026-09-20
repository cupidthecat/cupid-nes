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
enum { AUDIO_PROCESSOR_CAPACITY = 4 };

typedef struct {
    NesAudioObserver callback;
    void *context;
    uint32_t token;
} AudioObserverEntry;

static AudioObserverEntry observers[AUDIO_OBSERVER_CAPACITY];
typedef struct {
    NesAudioProcessor callback;
    void *context;
    uint32_t token;
} AudioProcessorEntry;
static AudioProcessorEntry processors[AUDIO_PROCESSOR_CAPACITY];
static uint32_t next_token = 1;
static unsigned observer_count;
static unsigned processor_count;
static bool dispatching;

static uint32_t allocate_token(void) {
    uint32_t token;
    bool collision;
    do {
        token = next_token++;
        collision = token == 0;
        for (size_t i = 0; i < AUDIO_OBSERVER_CAPACITY; ++i)
            if (observers[i].callback && observers[i].token == token) collision = true;
        for (size_t i = 0; i < AUDIO_PROCESSOR_CAPACITY; ++i)
            if (processors[i].callback && processors[i].token == token) collision = true;
    } while (collision);
    return token;
}

uint32_t nes_audio_observer_add(NesAudioObserver observer, void *context) {
    if (!observer || dispatching || observer_count == AUDIO_OBSERVER_CAPACITY) return 0;

    for (size_t i = 0; i < AUDIO_OBSERVER_CAPACITY; ++i) {
        if (observers[i].callback) continue;

        uint32_t token = allocate_token();

        observers[i] = (AudioObserverEntry){observer, context, token};
        ++observer_count;
        return token;
    }

    return 0;
}

uint32_t nes_audio_processor_add(NesAudioProcessor processor, void *context) {
    if (!processor || dispatching || processor_count == AUDIO_PROCESSOR_CAPACITY) return 0;
    for (size_t i = 0; i < AUDIO_PROCESSOR_CAPACITY; ++i) {
        if (processors[i].callback) continue;
        uint32_t token = allocate_token();
        processors[i] = (AudioProcessorEntry){processor, context, token};
        ++processor_count;
        return token;
    }
    return 0;
}

bool nes_audio_processor_remove(uint32_t token) {
    if (!token) return false;
    for (size_t i = 0; i < AUDIO_PROCESSOR_CAPACITY; ++i) {
        if (processors[i].callback && processors[i].token == token) {
            processors[i] = (AudioProcessorEntry){0};
            --processor_count;
            return true;
        }
    }
    return false;
}

void nes_audio_process(unsigned machine, double sample_rate, float *left, float *right) {
    if (!processor_count || dispatching || !left || !right || !isfinite(sample_rate) || sample_rate <= 0.0
        || (nes_execution_policy() & (NES_EXECUTION_SPECULATIVE | NES_EXECUTION_REWIND))) return;
    dispatching = true;
    for (size_t i = 0; i < AUDIO_PROCESSOR_CAPACITY; ++i)
        if (processors[i].callback) processors[i].callback(processors[i].context, machine, sample_rate, left, right);
    dispatching = false;
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
