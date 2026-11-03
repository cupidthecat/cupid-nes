/*
 * audio_output.c - Host audio reservoir and bounded device-clock correction
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "audio_output.h"
#include <math.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

/* Separate from serialized APU rings: even an 8192-frame device callback can
 * retain a full reservoir without changing save-state formats or captured samples. */
enum { OUTPUT_CAPACITY = 32768, OUTPUT_MASK = OUTPUT_CAPACITY - 1, FADE_FRAMES = 64 };
struct NesAudioOutput {
    float samples[OUTPUT_CAPACITY][2];
    _Atomic uint32_t read, write;
    _Atomic uint64_t reset;
    _Atomic uint64_t underruns, missing_frames, dropped_frames;
    unsigned rate, target;
    /* Consumer-owned state. The producer never writes these fields. */
    uint64_t seen_reset;
    unsigned fade;
    bool primed, rebuffering;
    double phase, filtered_error;
    float last[2];
};

NesAudioOutput *nes_audio_output_create(unsigned rate, unsigned block) {
    if (rate < 8000 || rate > 192000 || block < 64 || block > 8192) return NULL;
    NesAudioOutput *o = calloc(1, sizeof(*o));
    if (!o) return NULL;
    o->rate = rate;
    /* Retain 60 ms after a device block: a 40 ms stall plus the next
     * completed emulation frame must fit without draining playback. */
    o->target = block + rate * 60 / 1000;
    if (o->target < block * 2) o->target = block * 2;
    atomic_init(&o->read, 0);
    atomic_init(&o->write, 0);
    atomic_init(&o->reset, 0);
    atomic_init(&o->underruns, 0);
    atomic_init(&o->missing_frames, 0);
    atomic_init(&o->dropped_frames, 0);
    return o;
}

void nes_audio_output_destroy(NesAudioOutput *o) { free(o); }

void nes_audio_output_reset(NesAudioOutput *o) {
    if (!o) return;
    uint64_t serial = (atomic_load_explicit(&o->reset, memory_order_relaxed) >> 32) + 1;
    uint32_t boundary = atomic_load_explicit(&o->write, memory_order_relaxed);
    /* Publish the cursor and serial together, including repeated resets while
     * a paused device has not acknowledged an earlier transition. */
    atomic_store_explicit(&o->reset, (serial << 32) | boundary, memory_order_release);
}

void nes_audio_output_push(NesAudioOutput *o, const float *stereo, unsigned frames) {
    if (!o || !stereo) return;
    uint32_t w = atomic_load_explicit(&o->write, memory_order_relaxed);
    uint32_t r = atomic_load_explicit(&o->read, memory_order_acquire);
    unsigned room = OUTPUT_CAPACITY - (w - r);
    unsigned count = frames < room ? frames : room;
    for (unsigned i = 0; i < count; ++i) {
        o->samples[(w + i) & OUTPUT_MASK][0] = stereo[i * 2];
        o->samples[(w + i) & OUTPUT_MASK][1] = stereo[i * 2 + 1];
    }
    atomic_store_explicit(&o->write, w + count, memory_order_release);
    atomic_fetch_add_explicit(&o->dropped_frames, frames - count, memory_order_relaxed);
}

bool nes_audio_output_stats(const NesAudioOutput *o, NesAudioOutputStats *stats) {
    if (!o || !stats) return false;
    /* Read first: a concurrent consumer can only make this conservative. */
    uint32_t r = atomic_load_explicit(&o->read, memory_order_acquire);
    uint32_t w = atomic_load_explicit(&o->write, memory_order_acquire);
    unsigned queued = w - r;
    if (queued > OUTPUT_CAPACITY) queued = 0;
    *stats = (NesAudioOutputStats){queued * 1000.0 / o->rate,
        atomic_load_explicit(&o->underruns, memory_order_relaxed),
        atomic_load_explicit(&o->missing_frames, memory_order_relaxed),
        atomic_load_explicit(&o->dropped_frames, memory_order_relaxed)};
    return true;
}

void nes_audio_output_callback(void *context, Uint8 *stream, int bytes) {
    NesAudioOutput *o = context;
    if (!stream || bytes <= 0) return;
    memset(stream, 0, (size_t)bytes);
    if (!o) return;
    unsigned frames = (unsigned)bytes / (2 * sizeof(float));
    float *out = (float *)stream;
    uint64_t reset = atomic_load_explicit(&o->reset, memory_order_acquire);
    uint32_t r = atomic_load_explicit(&o->read, memory_order_relaxed);
    if (reset != o->seen_reset) {
        r = (uint32_t)reset;
        atomic_store_explicit(&o->read, r, memory_order_release);
        o->seen_reset = reset;
        o->primed = o->rebuffering = false;
        o->phase = o->filtered_error = 0;
        o->last[0] = o->last[1] = 0;
    }
    unsigned queued = atomic_load_explicit(&o->write, memory_order_acquire) - r;
    if (!o->primed) {
        if (queued < o->target) {
            /* Count silence caused by recovery, but exclude startup/resume. */
            if (o->rebuffering) atomic_fetch_add_explicit(&o->missing_frames, frames, memory_order_relaxed);
            return;
        }
        o->primed = true;
        o->rebuffering = false;
        o->fade = FADE_FRAMES;
        o->phase = o->filtered_error = 0;
    }
    /* Smooth block/frame quantization; correct only the host's playback clock.
     * Reconstructed/recorded audio and all emulated clocks remain untouched. */
    double smoothing = fmin(1.0, (double)frames / o->rate);
    o->filtered_error += smoothing * ((double)queued - o->target - o->filtered_error);
    double correction = o->filtered_error / (o->rate * 0.5);
    if (correction > 0.005) correction = 0.005;
    if (correction < -0.005) correction = -0.005;
    double ratio = 1.0 + correction;
    unsigned i = 0;
    for (; i < frames; ++i) {
        uint32_t w = atomic_load_explicit(&o->write, memory_order_acquire);
        unsigned advance = (unsigned)(o->phase + ratio);
        if (w - r < 2 || w - r < advance) break;
        for (unsigned side = 0; side < 2; ++side) {
            float a = o->samples[r & OUTPUT_MASK][side];
            float b = o->samples[(r + 1) & OUTPUT_MASK][side];
            float value = a + (b - a) * (float)o->phase;
            if (o->fade) value *= (float)(FADE_FRAMES - o->fade) / FADE_FRAMES;
            out[i * 2 + side] = o->last[side] = value;
        }
        if (o->fade) --o->fade;
        o->phase += ratio - advance;
        r += advance;
    }
    atomic_store_explicit(&o->read, r, memory_order_release);
    if (i < frames) {
        atomic_fetch_add_explicit(&o->underruns, 1, memory_order_relaxed);
        atomic_fetch_add_explicit(&o->missing_frames, frames - i, memory_order_relaxed);
        /* Fade to silence instead of holding an arbitrary nonzero sample. */
        unsigned fade = frames - i < FADE_FRAMES ? frames - i : FADE_FRAMES;
        for (unsigned j = 0; j < fade; ++j)
            for (unsigned side = 0; side < 2; ++side)
                out[(i + j) * 2 + side] = o->last[side] * (float)(fade - j - 1) / fade;
        o->last[0] = o->last[1] = 0;
        o->primed = false;
        o->rebuffering = true;
        o->phase = o->filtered_error = 0;
    }
}
