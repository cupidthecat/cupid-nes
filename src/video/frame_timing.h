/*
 * frame_timing.h - Bounded host frame pacing measurements
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_FRAME_TIMING_H
#define CUPID_FRAME_TIMING_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
enum { NES_FRAME_TIMING_CAPACITY = 240, NES_FRAME_TIMING_METRICS = 5 };

enum { NES_HOST_EMULATION, NES_HOST_RENDER, NES_HOST_PRESENT, NES_HOST_PACING, NES_HOST_INTERVAL };

typedef struct {
    double milliseconds[NES_FRAME_TIMING_METRICS];
    double audio_queue_ms;
    uint64_t audio_underruns;
    bool audio_available;
} NesHostFrameSample;

typedef struct {
    NesHostFrameSample samples[NES_FRAME_TIMING_CAPACITY];
    size_t count, next;
} NesFrameTiming;

typedef struct {
    double recent, average, minimum, maximum, p95;
} NesHostMetric;

typedef struct {
    NesHostMetric metrics[NES_FRAME_TIMING_METRICS];
    size_t count;
    double fps, audio_queue_ms;
    uint64_t audio_underruns;
    bool audio_available;
} NesFrameTimingSummary;

void nes_frame_timing_reset(NesFrameTiming *timing);
bool nes_frame_timing_push(NesFrameTiming *timing, const NesHostFrameSample *sample);
bool nes_frame_timing_summary(const NesFrameTiming *timing, NesFrameTimingSummary *summary);
double nes_frame_timing_elapsed(uint64_t begin, uint64_t end, uint64_t frequency);
#ifdef __cplusplus
}
#endif
#endif
