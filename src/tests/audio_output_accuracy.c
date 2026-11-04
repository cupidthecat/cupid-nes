/*
 * audio_output_accuracy.c - Host buffering, clock drift and stall recovery
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "../audio/audio_output.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { fprintf(stderr, "Audio output %d: %s\n", __LINE__, #x); return 1; } } while (0)

static float source[8192 * 2], played[8192 * 2];
static void fill(float left, float right) {
    for (unsigned i = 0; i < 8192; ++i) { source[i * 2] = left; source[i * 2 + 1] = right; }
}
static void play(NesAudioOutput *o, unsigned frames) {
    nes_audio_output_callback(o, (Uint8 *)played, (int)(frames * 2 * sizeof(float)));
}

/* Independent emulation/device clocks, 13 ms core work, bounded catch-up and
 * a 40 ms producer stall. No real-time sleeps or dummy-driver timing involved. */
static int simulate(double drift) {
    NesAudioOutput *o = nes_audio_output_create(44100, 1024);
    CHECK(o);
    fill(.25f, -.5f);
    double delivery = 13, callback = 0, accumulated = 0;
    double frame_ms = 1000.0 / 60.0988, block_ms = 1024 * 1000.0 / (44100 * (1 + drift));
    unsigned frame = 0;
    while (callback < 120000) {
        if (delivery <= callback) {
            accumulated += 44100 * frame_ms / 1000;
            unsigned count = (unsigned)accumulated;
            accumulated -= count;
            nes_audio_output_push(o, source, count);
            ++frame;
            delivery = fmax(delivery + 13, frame * frame_ms + 13);
            if (frame == 300 || frame == 3000) delivery += 40;
        } else {
            play(o, 1024);
            for (unsigned i = 0; i < 2048; ++i) CHECK(isfinite(played[i]));
            callback += block_ms;
        }
    }
    NesAudioOutputStats stats;
    CHECK(nes_audio_output_stats(o, &stats));
    if (stats.underruns || stats.dropped_frames)
        fprintf(stderr, "drift %.4f: %llu underruns, %llu dropped, queue %.2f ms\n", drift,
            (unsigned long long)stats.underruns, (unsigned long long)stats.dropped_frames, stats.queue_ms);
    CHECK(stats.underruns == 0 && stats.dropped_frames == 0);
    CHECK(stats.queue_ms > 20 && stats.queue_ms < 110);
    nes_audio_output_destroy(o);
    return 0;
}

int run_audio_output_accuracy_tests(void) {
    CHECK(!nes_audio_output_create(0, 1024) && !nes_audio_output_create(44100, 0));
    NesAudioOutput *o = nes_audio_output_create(44100, 1024);
    CHECK(o);
    fill(.25f, -.5f);
    nes_audio_output_push(o, source, 1024);
    play(o, 1024);
    for (unsigned i = 0; i < 2048; ++i) CHECK(played[i] == 0); /* Prefill keeps its samples. */
    nes_audio_output_push(o, source, 4096);
    play(o, 1024);
    CHECK(played[0] == 0 && played[1] == 0); /* Fade in. */
    CHECK(fabsf(played[200] - .25f) < .00001f && fabsf(played[201] + .5f) < .00001f);
    NesAudioOutputStats stats;
    CHECK(nes_audio_output_stats(o, &stats) && stats.underruns == 0);
    /* No cached pre-transition output survives a machine change or resume. */
    nes_audio_output_reset(o);
    fill(-.75f, .125f);
    nes_audio_output_push(o, source, 4096);
    play(o, 1024);
    CHECK(fabsf(played[200] + .75f) < .00001f && fabsf(played[201] - .125f) < .00001f);
    for (unsigned i = 0; i < 5; ++i) play(o, 1024);
    CHECK(nes_audio_output_stats(o, &stats) && stats.underruns == 1 && stats.missing_frames > 0);
    CHECK(played[2046] == 0 && played[2047] == 0);
    nes_audio_output_push(o, source, 1024);
    play(o, 1024);
    CHECK(played[200] == 0 && played[201] == 0); /* Rebuffer, not repeated clicks. */
    nes_audio_output_push(o, source, 4096);
    play(o, 1024);
    CHECK(fabsf(played[200] + .75f) < .00001f);
    nes_audio_output_destroy(o);
    /* Largest supported device block exceeds the emulated APU ring size. */
    o = nes_audio_output_create(192000, 8192);
    CHECK(o);
    for (unsigned i = 0; i < 3; ++i) nes_audio_output_push(o, source, 8192);
    play(o, 8192);
    CHECK(nes_audio_output_stats(o, &stats) && stats.underruns == 0 && stats.dropped_frames == 0);
    nes_audio_output_destroy(o);
    CHECK(simulate(0) == 0 && simulate(.001) == 0 && simulate(-.001) == 0);
    puts("Host audio prefill, stereo, reset, underrun, large-block, drift and stall regressions passed");
    return 0;
}
