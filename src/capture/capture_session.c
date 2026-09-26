/*
 * capture_session.c - Emulated-time audio sampling for screenshots and recordings
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "capture_session.h"
#include "capture_gif.h"
#include "../audio/audio_observer.h"
#include "../rom/rom.h"
#include "../system/execution_policy.h"
#include "../system/timing.h"
#include "../system/vs_system.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum { CAPTURE_INPUT_SAMPLES = 65536 };

typedef struct {
    uint64_t cycle;
    float left;
    float right;
} CaptureSample;

typedef struct {
    CaptureSample *samples;
    CaptureSample previous;
    size_t count;
    size_t cursor;
    uint64_t start_cycle;
    bool have_previous;
} CaptureChannel;

typedef struct {
    NesCaptureStream *stream;
    NesCaptureGif *gif;
    uint32_t observer;
    uint32_t image_crc;
    NesRegion region;
    uint64_t sample_phase;
    unsigned machines;
    bool frame_open;
    NesFileResult frame_error;
    CaptureChannel channels[2];
    int16_t *pcm;
    size_t pcm_capacity;
} CaptureImplementation;

void nes_capture_options_defaults(NesCaptureOptions *options) {
    if (options) {
        *options = (NesCaptureOptions){.sample_rate = 44100,
                                       .byte_limit = UINT32_MAX,
                                       .displayed_output = true,
                                       .codec = NES_CAPTURE_CODEC_RAW,
                                       .compression_level = 6,
                                       .format = NES_CAPTURE_FORMAT_AVI,
                                       .gif_scale = 1};
    }
}

static uint32_t gcd32(uint32_t a, uint32_t b) {
    while (b) {
        uint32_t remainder = a % b;
        a = b;
        b = remainder;
    }
    return a;
}

NesCaptureFrameRate nes_capture_frame_rate(void) {
    const NesTiming *timing = nes_timing();
    uint32_t numerator = (uint32_t)timing->cpu_hz * timing->cpu_divider;
    uint32_t denominator = 341u * timing->scanlines * timing->ppu_divider;
    if (timing->region == NES_REGION_NTSC && !vs_enabled() && !rom_is_nsf()) {
        numerator *= 2u;
        denominator = denominator * 2u - timing->ppu_divider;
    }
    uint32_t divisor = gcd32(numerator, denominator);
    return (NesCaptureFrameRate){numerator / divisor, denominator / divisor};
}

static void release_implementation(NesCaptureSession *session, bool abort_stream) {
    CaptureImplementation *implementation = session->implementation;
    if (!implementation) {
        return;
    }
    (void)nes_audio_observer_remove(implementation->observer);
    if (abort_stream) {
        nes_capture_abort(&implementation->stream);
        nes_capture_gif_abort(&implementation->gif);
    }
    free(implementation->channels[0].samples);
    free(implementation->channels[1].samples);
    free(implementation->pcm);
    free(implementation);
    session->implementation = NULL;
    session->info.recording = false;
}

static void observe_audio(void *context, unsigned machine, double sample_rate, float left, float right) {
    (void)sample_rate;
    NesCaptureSession *session = context;
    CaptureImplementation *implementation = session->implementation;
    if (!implementation || !implementation->frame_open || machine >= implementation->machines ||
        implementation->frame_error != NES_FILE_OK) {
        return;
    }
    CaptureChannel *channel = &implementation->channels[machine];
    if (channel->count == CAPTURE_INPUT_SAMPLES) {
        implementation->frame_error = NES_FILE_TOO_LARGE;
        return;
    }
    uint64_t cycle = vs_side_cpu_cycles(machine);
    uint64_t previous = channel->count ? channel->samples[channel->count - 1u].cycle : channel->start_cycle;
    if (cycle < previous) {
        implementation->frame_error = NES_FILE_INVALID_ARGUMENT;
        return;
    }
    if (!isfinite(left)) {
        left = 0.0f;
    }
    if (!isfinite(right)) {
        right = 0.0f;
    }
    channel->samples[channel->count++] = (CaptureSample){cycle, left, right};
}

NesFileResult nes_capture_session_start(NesCaptureSession *session, const char *path, bool video,
                                        const NesCaptureFrame *frame, const NesCaptureOptions *options) {
    if (!session || !options) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    session->error[0] = '\0';
    if (session->implementation) {
        snprintf(session->error, sizeof(session->error), "Stop the current recording before starting another");
        return NES_FILE_INVALID_ARGUMENT;
    }
    if (nes_execution_policy() & (NES_EXECUTION_SPECULATIVE | NES_EXECUTION_REWIND)) {
        snprintf(session->error, sizeof(session->error), "Start capture after rewind or speculative execution ends");
        return NES_FILE_INVALID_ARGUMENT;
    }
    NesCaptureFrameRate rate = nes_capture_frame_rate();
    if (options->sample_rate < 8000 || options->sample_rate > 192000 || options->byte_limit > UINT32_MAX ||
        (unsigned)options->codec > NES_CAPTURE_CODEC_ZMBV || options->compression_level > 9 ||
        (unsigned)options->format > NES_CAPTURE_FORMAT_GIF || options->gif_scale < 1 || options->gif_scale > 4 ||
        (video && (!frame || !frame->pixels))) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    CaptureImplementation *implementation = calloc(1, sizeof(*implementation));
    if (!implementation) {
        return NES_FILE_OUT_OF_MEMORY;
    }
    implementation->machines = vs_dual_system() ? 2u : 1u;
    implementation->image_crc = rom_file_crc32();
    implementation->region = nes_timing()->region;
    implementation->pcm_capacity =
        (size_t)(((uint64_t)options->sample_rate * rate.denominator + rate.numerator - 1u) / rate.numerator);
    implementation->pcm = malloc(implementation->pcm_capacity * 2u * sizeof(*implementation->pcm));
    for (unsigned i = 0; i < implementation->machines; ++i) {
        implementation->channels[i].samples = malloc(CAPTURE_INPUT_SAMPLES * sizeof(CaptureSample));
    }
    session->implementation = implementation;
    if (!implementation->pcm || !implementation->channels[0].samples ||
        (implementation->machines == 2 && !implementation->channels[1].samples)) {
        release_implementation(session, true);
        return NES_FILE_OUT_OF_MEMORY;
    }
    NesFileResult result;
    if (video && options->format == NES_CAPTURE_FORMAT_GIF) {
        result = nes_capture_gif_open(path, frame->width, frame->height, options->gif_scale, rate.numerator,
                                      rate.denominator, options->byte_limit, &implementation->gif);
    } else if (video) {
        result = nes_capture_avi_open_codec(path, frame->width, frame->height, options->sample_rate, rate.numerator,
                                            rate.denominator, options->byte_limit, options->codec,
                                            options->compression_level, &implementation->stream);
    } else {
        result = nes_capture_wav_open(path, options->sample_rate, options->byte_limit, &implementation->stream);
    }
    if (result != NES_FILE_OK) {
        release_implementation(session, true);
        snprintf(session->error, sizeof(session->error), "%s", nes_file_result_message(result));
        return result;
    }
    implementation->observer = nes_audio_observer_add(observe_audio, session);
    if (!implementation->observer) {
        release_implementation(session, true);
        snprintf(session->error, sizeof(session->error), "The audio capture observer could not be attached");
        return NES_FILE_OUT_OF_MEMORY;
    }
    session->info = (NesCaptureInfo){.recording = true,
                                     .video = video,
                                     .width = video ? frame->width : 0,
                                     .height = video ? frame->height : 0,
                                     .sample_rate = options->sample_rate,
                                     .frame_rate = rate,
                                     .bytes = implementation->gif ? nes_capture_gif_size(implementation->gif)
                                                                  : nes_capture_file_size(implementation->stream)};
    return NES_FILE_OK;
}

NesFileResult nes_capture_session_stop(NesCaptureSession *session) {
    if (!session) {
        return NES_FILE_INVALID_ARGUMENT;
    }
    CaptureImplementation *implementation = session->implementation;
    if (!implementation) {
        return NES_FILE_OK;
    }
    session->info.bytes =
        implementation->gif ? nes_capture_gif_size(implementation->gif) : nes_capture_file_size(implementation->stream);
    NesFileResult result =
        implementation->gif ? nes_capture_gif_close(&implementation->gif) : nes_capture_close(&implementation->stream);
    if (result != NES_FILE_OK) {
        snprintf(session->error, sizeof(session->error), "Recording could not be finalized: %s",
                 nes_file_result_message(result));
    }
    release_implementation(session, false);
    return result;
}

static NesFileResult stop_with_error(NesCaptureSession *session, NesFileResult result, const char *message) {
    NesFileResult finalization = nes_capture_session_stop(session);
    if (finalization != NES_FILE_OK) {
        return finalization;
    }
    snprintf(session->error, sizeof(session->error), "%s; completed frames were finalized", message);
    return result;
}

bool nes_capture_session_begin_frame(NesCaptureSession *session) {
    if (!session || !session->implementation) {
        return false;
    }
    CaptureImplementation *implementation = session->implementation;
    if (implementation->image_crc != rom_file_crc32() || implementation->region != nes_timing()->region ||
        implementation->machines != (vs_dual_system() ? 2u : 1u) ||
        (nes_execution_policy() & (NES_EXECUTION_SPECULATIVE | NES_EXECUTION_REWIND))) {
        (void)stop_with_error(session, NES_FILE_INVALID_ARGUMENT,
                              "Recording stopped when the emulation session changed");
        return false;
    }
    if (implementation->frame_open) {
        return true;
    }
    for (unsigned i = 0; i < implementation->machines; ++i) {
        CaptureChannel *channel = &implementation->channels[i];
        channel->count = channel->cursor = 0;
        channel->start_cycle = vs_side_cpu_cycles(i);
        if (channel->have_previous && channel->previous.cycle > channel->start_cycle) {
            (void)stop_with_error(session, NES_FILE_INVALID_ARGUMENT,
                                  "Recording stopped after the machine clock was restored");
            return false;
        }
        if (!channel->have_previous) {
            channel->previous = (CaptureSample){channel->start_cycle, 0.0f, 0.0f};
            channel->have_previous = true;
        }
    }
    implementation->frame_error = NES_FILE_OK;
    implementation->frame_open = true;
    return true;
}

static double relative_cycle(uint64_t cycle, uint64_t start) {
    return cycle >= start ? (double)(cycle - start) : -(double)(start - cycle);
}

static CaptureSample interpolate(CaptureChannel *channel, double position) {
    while (channel->cursor < channel->count &&
           relative_cycle(channel->samples[channel->cursor].cycle, channel->start_cycle) <= position) {
        channel->previous = channel->samples[channel->cursor++];
    }
    CaptureSample sample = channel->previous;
    if (channel->cursor < channel->count) {
        const CaptureSample *next = &channel->samples[channel->cursor];
        double begin = relative_cycle(sample.cycle, channel->start_cycle);
        double end = relative_cycle(next->cycle, channel->start_cycle);
        double weight = end > begin ? (position - begin) / (end - begin) : 1.0;
        if (weight < 0.0) {
            weight = 0.0;
        }
        if (weight > 1.0) {
            weight = 1.0;
        }
        sample.left += (float)weight * (next->left - sample.left);
        sample.right += (float)weight * (next->right - sample.right);
    }
    return sample;
}

static int16_t pcm_sample(float value) {
    if (!isfinite(value)) {
        return 0;
    }
    if (value >= 1.0f) {
        return INT16_MAX;
    }
    if (value <= -1.0f) {
        return INT16_MIN;
    }
    return (int16_t)lrintf(value * (value < 0.0f ? 32768.0f : 32767.0f));
}

NesFileResult nes_capture_session_end_frame(NesCaptureSession *session, const NesCaptureFrame *frame) {
    if (!session || !session->implementation) {
        return NES_FILE_OK;
    }
    CaptureImplementation *implementation = session->implementation;
    if (!implementation->frame_open) {
        return NES_FILE_OK;
    }
    if (implementation->frame_error != NES_FILE_OK) {
        return stop_with_error(session, implementation->frame_error,
                               "Recording stopped because its input buffer or clock changed");
    }
    if (implementation->image_crc != rom_file_crc32() || implementation->region != nes_timing()->region) {
        return stop_with_error(session, NES_FILE_INVALID_ARGUMENT,
                               "Recording stopped when the emulation session changed");
    }
    uint64_t cycles[2] = {0};
    for (unsigned i = 0; i < implementation->machines; ++i) {
        uint64_t end = vs_side_cpu_cycles(i);
        if (end <= implementation->channels[i].start_cycle) {
            return stop_with_error(session, NES_FILE_INVALID_ARGUMENT,
                                   "Recording stopped because a completed frame had no forward emulation time");
        }
        cycles[i] = end - implementation->channels[i].start_cycle;
    }

    uint64_t step = (uint64_t)session->info.sample_rate * session->info.frame_rate.denominator;
    uint64_t phase = implementation->sample_phase;
    size_t frames = (size_t)((phase + step) / session->info.frame_rate.numerator);
    if (frames > implementation->pcm_capacity) {
        return stop_with_error(session, NES_FILE_TOO_LARGE, "The recording sample buffer could not hold this frame");
    }
    for (size_t i = 0; i < frames; ++i) {
        double fraction = (double)(((uint64_t)i + 1u) * session->info.frame_rate.numerator - phase) / (double)step;
        CaptureSample sample = interpolate(&implementation->channels[0], fraction * (double)cycles[0]);
        if (implementation->machines == 2) {
            CaptureSample second = interpolate(&implementation->channels[1], fraction * (double)cycles[1]);
            sample.left = 0.5f * (sample.left + second.left);
            sample.right = 0.5f * (sample.right + second.right);
        }
        implementation->pcm[i * 2u] = pcm_sample(sample.left);
        implementation->pcm[i * 2u + 1u] = pcm_sample(sample.right);
    }
    NesFileResult result = implementation->gif ? nes_capture_gif_frame(implementation->gif, frame)
                           : session->info.video
                               ? nes_capture_write_frame(implementation->stream, frame, implementation->pcm, frames)
                               : nes_capture_write_audio(implementation->stream, implementation->pcm, frames);
    if (result != NES_FILE_OK) {
        const char *message = result == NES_FILE_TOO_LARGE ? "Recording reached its configured file or index limit"
                              : result == NES_FILE_INVALID_ARGUMENT
                                  ? "Recording stopped when the output dimensions changed"
                                  : "Recording stopped after an output failure";
        return stop_with_error(session, result, message);
    }
    implementation->sample_phase = (phase + step) % session->info.frame_rate.numerator;
    implementation->frame_open = false;
    for (unsigned i = 0; i < implementation->machines; ++i) {
        CaptureChannel *channel = &implementation->channels[i];
        if (channel->count) {
            channel->previous = channel->samples[channel->count - 1u];
        }
    }
    ++session->info.completed_frames;
    session->info.audio_frames = nes_capture_audio_frames(implementation->stream);
    session->info.bytes =
        implementation->gif ? nes_capture_gif_size(implementation->gif) : nes_capture_file_size(implementation->stream);
    return NES_FILE_OK;
}

void nes_capture_session_discard(NesCaptureSession *session) {
    if (!session) {
        return;
    }
    release_implementation(session, true);
    memset(&session->info, 0, sizeof(session->info));
    session->error[0] = '\0';
}
