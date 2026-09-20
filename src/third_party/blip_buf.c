/* blip_buf 1.1.0. http://www.slack.net/~ant/
 *
 * Library Copyright (C) 2003-2009 Shay Green. This library is free software;
 * you can redistribute it and/or modify it under the terms of the GNU Lesser
 * General Public License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version. This
 * library is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU Lesser General Public License for details.
 */

#include "blip_buf.h"

#include <assert.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

#if ULONG_MAX / 0xFFFFFFFFUL > 0xFFFFFFFFUL
typedef unsigned long fixed_t;
enum { pre_shift = 32 };
#elif defined(ULLONG_MAX)
typedef unsigned long long fixed_t;
enum { pre_shift = 32 };
#else
typedef unsigned fixed_t;
enum { pre_shift = 0 };
#endif

enum { time_bits = pre_shift + 20 };
static fixed_t const time_unit = (fixed_t)1 << time_bits;

enum { bass_shift = 9 };
enum { end_frame_extra = 2 };
enum { half_width = 8 };
enum { buf_extra = half_width * 2 + end_frame_extra };
enum { phase_bits = 5 };
enum { phase_count = 1 << phase_bits };
enum { delta_bits = 15 };
enum { delta_unit = 1 << delta_bits };
enum { frac_bits = time_bits - pre_shift };
enum { max_sample = 32767, min_sample = -32768 };

typedef int buf_t;

struct blip_t {
    fixed_t factor;
    fixed_t offset;
    int avail;
    int size;
    int integrator;
};

#define SAMPLES(buffer) ((buf_t *)((buffer) + 1))
#define ARITH_SHIFT(value, shift) ((value) >> (shift))

static int clamp_sample(int sample) {
    if ((short)sample != sample) sample = ARITH_SHIFT(sample, 16) ^ max_sample;
    return sample;
}

static void check_assumptions(void) {
    int sample;
#if INT_MAX < 0x7FFFFFFF || UINT_MAX < 0xFFFFFFFFU
#error "int must be at least 32 bits"
#endif
    assert((-3 >> 1) == -2);
    sample = clamp_sample(max_sample * 2);
    assert(sample == max_sample);
    sample = clamp_sample(min_sample * 2);
    assert(sample == min_sample);
}

blip_t *blip_new(int size) {
    assert(size >= 0);
    blip_t *buffer = (blip_t *)malloc(sizeof(*buffer) + (size + buf_extra) * sizeof(buf_t));
    if (buffer) {
        buffer->factor = time_unit / (1 << 20);
        buffer->size = size;
        blip_clear(buffer);
        check_assumptions();
    }
    return buffer;
}

void blip_delete(blip_t *buffer) {
    if (!buffer) return;
    memset(buffer, 0, sizeof(*buffer));
    free(buffer);
}

void blip_set_rates(blip_t *buffer, double clock_rate, double sample_rate) {
    double factor = (double)time_unit * sample_rate / clock_rate;
    buffer->factor = (fixed_t)factor;
    assert(0.0 <= factor - (double)buffer->factor && factor - (double)buffer->factor < 1.0);
    if ((double)buffer->factor < factor) buffer->factor++;
}

void blip_clear(blip_t *buffer) {
    buffer->offset = buffer->factor / 2;
    buffer->avail = 0;
    buffer->integrator = 0;
    memset(SAMPLES(buffer), 0, (size_t)(buffer->size + buf_extra) * sizeof(buf_t));
}

void blip_end_frame(blip_t *buffer, unsigned clock_duration) {
    fixed_t offset = (fixed_t)clock_duration * buffer->factor + buffer->offset;
    buffer->avail += (int)(offset >> time_bits);
    buffer->offset = offset & (time_unit - 1);
    assert(buffer->avail <= buffer->size);
}

int blip_samples_avail(const blip_t *buffer) {
    return buffer->avail;
}

static void remove_samples(blip_t *buffer, int count) {
    buf_t *samples = SAMPLES(buffer);
    int remain = buffer->avail + buf_extra - count;
    buffer->avail -= count;
    memmove(samples, samples + count, (size_t)remain * sizeof(samples[0]));
    memset(samples + remain, 0, (size_t)count * sizeof(samples[0]));
}

int blip_read_samples(blip_t *buffer, short out[], int count, int stereo) {
    assert(count >= 0);
    if (count > buffer->avail) count = buffer->avail;
    if (!count) return 0;

    int step = stereo ? 2 : 1;
    const buf_t *input = SAMPLES(buffer);
    const buf_t *end = input + count;
    int sum = buffer->integrator;
    do {
        int sample = ARITH_SHIFT(sum, delta_bits);
        sum += *input++;
        sample = clamp_sample(sample);
        *out = (short)sample;
        out += step;
        sum -= sample * (1 << (delta_bits - bass_shift));
    } while (input != end);
    buffer->integrator = sum;
    remove_samples(buffer, count);
    return count;
}

size_t blip_state_sample_count(const blip_t *buffer) {
    return buffer && buffer->size >= 0 ? (size_t)buffer->size + buf_extra : 0;
}

bool blip_state_export(const blip_t *buffer, BlipStateHeader *header,
                       int32_t *samples, size_t sample_count) {
    if (!buffer || !header || buffer->size < 0 || buffer->avail < 0
        || buffer->avail > buffer->size) return false;
    size_t count = blip_state_sample_count(buffer);
    if (count != sample_count || (count && !samples)) return false;
    header->factor = (uint64_t)buffer->factor;
    header->offset = (uint64_t)buffer->offset;
    header->available = buffer->avail;
    header->size = buffer->size;
    header->integrator = buffer->integrator;
    for (size_t i = 0; i < count; ++i) samples[i] = SAMPLES(buffer)[i];
    return true;
}

bool blip_state_import(blip_t *buffer, const BlipStateHeader *header,
                       const int32_t *samples, size_t sample_count) {
    if (!buffer || !header || header->size != buffer->size
        || header->available < 0 || header->available > header->size
        || (uint64_t)(fixed_t)header->factor != header->factor
        || (uint64_t)(fixed_t)header->offset != header->offset) return false;
    size_t count = blip_state_sample_count(buffer);
    if (count != sample_count || (count && !samples)) return false;
    buffer->factor = (fixed_t)header->factor;
    buffer->offset = (fixed_t)header->offset;
    buffer->avail = header->available;
    buffer->integrator = header->integrator;
    for (size_t i = 0; i < count; ++i) SAMPLES(buffer)[i] = samples[i];
    return true;
}

static const short bl_step[phase_count + 1][half_width] = {
    {43,-115,350,-488,1136,-914,5861,21022},
    {44,-118,348,-473,1076,-799,5274,21001},
    {45,-121,344,-454,1011,-677,4706,20936},
    {46,-122,336,-431,942,-549,4156,20829},
    {47,-123,327,-404,868,-418,3629,20679},
    {47,-122,316,-375,792,-285,3124,20488},
    {47,-120,303,-344,714,-151,2644,20256},
    {46,-117,289,-310,634,-17,2188,19985},
    {46,-114,273,-275,553,117,1758,19675},
    {44,-108,255,-237,471,247,1356,19327},
    {43,-103,237,-199,390,373,981,18944},
    {42,-98,218,-160,310,495,633,18527},
    {40,-91,198,-121,231,611,314,18078},
    {38,-84,178,-81,153,722,22,17599},
    {36,-76,157,-43,80,824,-241,17092},
    {34,-68,135,-3,8,919,-476,16558},
    {32,-61,115,34,-60,1006,-683,16001},
    {29,-52,94,70,-123,1083,-862,15422},
    {27,-44,73,106,-184,1152,-1015,14824},
    {25,-36,53,139,-239,1211,-1142,14210},
    {22,-27,34,170,-290,1261,-1244,13582},
    {20,-20,16,199,-335,1301,-1322,12942},
    {18,-12,-3,226,-375,1331,-1376,12293},
    {15,-4,-19,250,-410,1351,-1408,11638},
    {13,3,-35,272,-439,1361,-1419,10979},
    {11,9,-49,292,-464,1362,-1410,10319},
    {9,16,-63,309,-483,1354,-1383,9660},
    {7,22,-75,322,-496,1337,-1339,9005},
    {6,26,-85,333,-504,1312,-1280,8355},
    {4,31,-94,341,-507,1278,-1205,7713},
    {3,35,-102,347,-506,1238,-1119,7082},
    {1,40,-110,350,-499,1190,-1021,6464},
    {0,43,-115,350,-488,1136,-914,5861}
};

void blip_add_delta(blip_t *buffer, unsigned clock_time, int delta) {
    unsigned fixed = (unsigned)(((fixed_t)clock_time * buffer->factor + buffer->offset) >> pre_shift);
    buf_t *output = SAMPLES(buffer) + buffer->avail + (fixed >> frac_bits);
    int phase_shift = frac_bits - phase_bits;
    int phase = (int)(fixed >> phase_shift) & (phase_count - 1);
    const short *input = bl_step[phase];
    const short *reverse = bl_step[phase_count - phase];
    int interpolation = (int)(fixed >> (phase_shift - delta_bits)) & (delta_unit - 1);
    int delta2 = (delta * interpolation) >> delta_bits;
    delta -= delta2;

    assert(output <= &SAMPLES(buffer)[buffer->size + end_frame_extra]);
    output[0] += input[0] * delta + input[half_width + 0] * delta2;
    output[1] += input[1] * delta + input[half_width + 1] * delta2;
    output[2] += input[2] * delta + input[half_width + 2] * delta2;
    output[3] += input[3] * delta + input[half_width + 3] * delta2;
    output[4] += input[4] * delta + input[half_width + 4] * delta2;
    output[5] += input[5] * delta + input[half_width + 5] * delta2;
    output[6] += input[6] * delta + input[half_width + 6] * delta2;
    output[7] += input[7] * delta + input[half_width + 7] * delta2;

    input = reverse;
    output[8] += input[7] * delta + input[7 - half_width] * delta2;
    output[9] += input[6] * delta + input[6 - half_width] * delta2;
    output[10] += input[5] * delta + input[5 - half_width] * delta2;
    output[11] += input[4] * delta + input[4 - half_width] * delta2;
    output[12] += input[3] * delta + input[3 - half_width] * delta2;
    output[13] += input[2] * delta + input[2 - half_width] * delta2;
    output[14] += input[1] * delta + input[1 - half_width] * delta2;
    output[15] += input[0] * delta + input[0 - half_width] * delta2;
}
