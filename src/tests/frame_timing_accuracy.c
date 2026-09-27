/*
 * frame_timing_accuracy.c - Host pacing statistics regression tests
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../video/frame_timing.h"
#include <SDL2/SDL.h>
#include <math.h>
#include <stdio.h>
#define CHECK(x)                                                                                                       \
    do {                                                                                                               \
        if (!(x)) {                                                                                                    \
            fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #x);                                                    \
            return 1;                                                                                                  \
        }                                                                                                              \
    } while (0)

static void count_service(void *context) {
    ++*(unsigned *)context;
}

int run_frame_timing_accuracy_tests(void) {
    unsigned serviced = 0;
    double frequency = (double)SDL_GetPerformanceFrequency();
    double deadline = (double)SDL_GetPerformanceCounter() + frequency * 0.004;
    nes_frame_timing_wait(deadline, frequency, count_service, &serviced);
    CHECK((double)SDL_GetPerformanceCounter() >= deadline);
    nes_frame_timing_wait(NAN, frequency, count_service, &serviced);
    nes_frame_timing_wait(deadline, 0, NULL, NULL);
    nes_frame_timing_wait(0, frequency, NULL, NULL);
    CHECK(nes_frame_timing_sleep_ms(NAN) == 0 && nes_frame_timing_sleep_ms(INFINITY) == 0);
    CHECK(nes_frame_timing_sleep_ms(-1) == 0 && nes_frame_timing_sleep_ms(0.9) == 0);
    CHECK(nes_frame_timing_sleep_ms(1.999) == 0 && nes_frame_timing_sleep_ms(2) == 1);
    CHECK(nes_frame_timing_sleep_ms(4.99) == 3 && nes_frame_timing_sleep_ms(5) == 4);
    CHECK(nes_frame_timing_sleep_ms(100000) == 4);
    CHECK(nes_frame_timing_present(100, 110, 90, 1000));
    CHECK(!nes_frame_timing_present(120, 110, 100, 1000));
    CHECK(nes_frame_timing_present(150, 110, 100, 1000));
    CHECK(nes_frame_timing_present(120, 110, 0, 1000));
    CHECK(nes_frame_timing_present(90, 80, 100, 1000));
    /* The same emulation work keeps its rate as presentation gets more costly. */
    for (unsigned cost = 2; cost <= 26; cost += 12) {
        double now = 1, deadline = 1, last = 0;
        unsigned presentations = 0;
        for (unsigned frame = 0; frame < 600; ++frame) {
            now += 3;
            deadline += 1000.0 / 60;
            if (nes_frame_timing_present(now, deadline, last, 1000)) {
                last = now;
                now += cost;
                ++presentations;
            }
            if (now < deadline) now = deadline;
        }
        CHECK(fabs(now - 10001) < 50);
        CHECK(presentations >= 190 && presentations <= 600);
        if (cost > 16) CHECK(presentations < 600);
    }
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
    CHECK(summary.interval_jitter_ms == 0);
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
    /* Equal average rates can conceal alternating early and late frames. */
    NesHostFrameSample early = {{0, 0, 0, 0, 10}, 0, 0, false};
    NesHostFrameSample late = {{0, 0, 0, 0, 30}, 0, 0, false};
    CHECK(nes_frame_timing_push(&timing, &early) && nes_frame_timing_push(&timing, &late));
    CHECK(nes_frame_timing_summary(&timing, &summary) && summary.fps == 50 && summary.interval_jitter_ms == 10);
    CHECK(!nes_frame_timing_summary(NULL, &summary));
    puts("Frame timing regressions passed");
    return 0;
}
