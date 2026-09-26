/*
 * frame_timing.c - Bounded host frame pacing measurements
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "frame_timing.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

void nes_frame_timing_reset(NesFrameTiming *timing) {
    if (timing) {
        memset(timing, 0, sizeof(*timing));
    }
}

double nes_frame_timing_elapsed(uint64_t begin, uint64_t end, uint64_t frequency) {
    return frequency && end >= begin ? (double)(end - begin) * 1000.0 / (double)frequency : 0;
}

bool nes_frame_timing_push(NesFrameTiming *timing, const NesHostFrameSample *sample) {
    if (!timing || !sample) {
        return false;
    }
    for (size_t i = 0; i < NES_FRAME_TIMING_METRICS; ++i) {
        if (!isfinite(sample->milliseconds[i]) || sample->milliseconds[i] < 0) {
            return false;
        }
    }
    if (sample->audio_available && (!isfinite(sample->audio_queue_ms) || sample->audio_queue_ms < 0)) {
        return false;
    }
    timing->samples[timing->next] = *sample;
    timing->next = (timing->next + 1) % NES_FRAME_TIMING_CAPACITY;
    if (timing->count < NES_FRAME_TIMING_CAPACITY) {
        ++timing->count;
    }
    return true;
}

static int compare(const void *a, const void *b) {
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

bool nes_frame_timing_summary(const NesFrameTiming *timing, NesFrameTimingSummary *summary) {
    if (!timing || !summary || timing->count > NES_FRAME_TIMING_CAPACITY || timing->next >= NES_FRAME_TIMING_CAPACITY) {
        return false;
    }
    memset(summary, 0, sizeof(*summary));
    summary->count = timing->count;
    if (!timing->count) {
        return true;
    }
    size_t recent = (timing->next + NES_FRAME_TIMING_CAPACITY - 1) % NES_FRAME_TIMING_CAPACITY;
    for (size_t m = 0; m < NES_FRAME_TIMING_METRICS; ++m) {
        double values[NES_FRAME_TIMING_CAPACITY], mean = 0;
        for (size_t i = 0; i < timing->count; ++i) {
            values[i] = timing->samples[i].milliseconds[m];
            mean += (values[i] - mean) / (double)(i + 1);
        }
        qsort(values, timing->count, sizeof(values[0]), compare);
        summary->metrics[m] = (NesHostMetric){timing->samples[recent].milliseconds[m], mean, values[0],
                                              values[timing->count - 1], values[(timing->count * 95 + 99) / 100 - 1]};
    }
    double interval = summary->metrics[NES_HOST_INTERVAL].average;
    summary->fps = interval > 0 ? 1000.0 / interval : 0;
    summary->audio_available = timing->samples[recent].audio_available;
    summary->audio_queue_ms = timing->samples[recent].audio_queue_ms;
    summary->audio_underruns = timing->samples[recent].audio_underruns;
    return true;
}
