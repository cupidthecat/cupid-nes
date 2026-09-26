/*
 * frame_timing_accuracy.c - Host pacing statistics regression tests
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../video/frame_timing.h"
#include <math.h>
#include <stdio.h>
#define CHECK(x)                                                                                                       \
    do {                                                                                                               \
        if (!(x)) {                                                                                                    \
            fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #x);                                                    \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

int run_frame_timing_accuracy_tests(void) {
    NesFrameTiming timing;
    NesFrameTimingSummary summary;
    nes_frame_timing_reset(&timing);
    CHECK(nes_frame_timing_summary(&timing, &summary) && summary.count == 0 && summary.fps == 0);
    for (unsigned i = 1; i <= 300; ++i) {
        NesHostFrameSample sample = {{i, i / 2.0, 0, 1, 20}, 15, i, true};
        CHECK(nes_frame_timing_push(&timing, &sample));
    }
    CHECK(nes_frame_timing_summary(&timing, &summary));
    CHECK(summary.count == 240 && summary.fps == 50);
    CHECK(summary.metrics[0].recent == 300 && summary.metrics[0].minimum == 61 && summary.metrics[0].maximum == 300 &&
          summary.metrics[0].p95 == 288 && fabs(summary.metrics[0].average - 180.5) < 0.00001);
    CHECK(summary.audio_available && summary.audio_underruns == 300 && summary.audio_queue_ms == 15);
    NesHostFrameSample invalid = {{NAN, 0, 0, 0, 0}, 0, 0, false};
    CHECK(!nes_frame_timing_push(&timing, &invalid));
    invalid.milliseconds[0] = -1;
    CHECK(!nes_frame_timing_push(&timing, &invalid));
    CHECK(nes_frame_timing_elapsed(100, 130, 1000) == 30);
    CHECK(nes_frame_timing_elapsed(130, 100, 1000) == 0 && nes_frame_timing_elapsed(100, 130, 0) == 0);
    nes_frame_timing_reset(&timing);
    CHECK(!nes_frame_timing_summary(NULL, &summary));
    puts("Frame timing regressions passed");
    return 0;
}
