/*
 * namco163.c - Namco 163 wavetable audio
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <string.h>
#include "namco163.h"

static uint8_t channel_count_minus_one(const Namco163Audio *audio) {
    return (audio->ram[0x7F] >> 4) & 7u;
}

static uint8_t channel_base(unsigned channel) {
    return (uint8_t)(0x40u + channel * 8u);
}

static uint32_t channel_frequency(const Namco163Audio *audio, unsigned channel) {
    uint8_t base = channel_base(channel);
    return (uint32_t)audio->ram[base]
        | ((uint32_t)audio->ram[base + 2] << 8)
        | ((uint32_t)(audio->ram[base + 4] & 3u) << 16);
}

static uint32_t channel_phase(const Namco163Audio *audio, unsigned channel) {
    uint8_t base = channel_base(channel);
    return (uint32_t)audio->ram[base + 1]
        | ((uint32_t)audio->ram[base + 3] << 8)
        | ((uint32_t)audio->ram[base + 5] << 16);
}

static void set_channel_phase(Namco163Audio *audio, unsigned channel, uint32_t phase) {
    uint8_t base = channel_base(channel);
    audio->ram[base + 1] = (uint8_t)phase;
    audio->ram[base + 3] = (uint8_t)(phase >> 8);
    audio->ram[base + 5] = (uint8_t)(phase >> 16);
}

static uint16_t channel_wave_length(const Namco163Audio *audio, unsigned channel) {
    return (uint16_t)(256u - (audio->ram[channel_base(channel) + 4] & 0xFCu));
}

static void update_mixed_output(Namco163Audio *audio) {
    unsigned count = (unsigned)channel_count_minus_one(audio) + 1u;
    unsigned first = 8u - count;
    int sum = 0;
    for (unsigned channel = first; channel < 8; ++channel)
        sum += audio->channel_output[channel];
    audio->mixed_output = (int16_t)(sum / (int)count);
}

static bool update_channel(Namco163Audio *audio, unsigned channel) {
    uint8_t base = channel_base(channel);
    uint32_t phase = channel_phase(audio, channel);
    uint32_t previous_phase = phase;
    uint32_t frequency = channel_frequency(audio, channel);
    uint16_t wave_length = channel_wave_length(audio, channel);
    uint32_t modulus = (uint32_t)wave_length << 16;
    phase = (phase + frequency) % modulus;

    uint8_t sample_position = (uint8_t)(((phase >> 16) + audio->ram[base + 6]) & 0xFFu);
    uint8_t packed = audio->ram[sample_position >> 1];
    int sample = (sample_position & 1u) ? (packed >> 4) : (packed & 0x0Fu);
    int volume = audio->ram[base + 7] & 0x0Fu;
    audio->channel_output[channel] = (int16_t)((sample - 8) * volume);
    set_channel_phase(audio, channel, phase);
    update_mixed_output(audio);
    return phase != previous_phase;
}

void namco163_audio_reset(Namco163Audio *audio) {
    if (!audio) return;
    memset(audio, 0, sizeof(*audio));
    audio->current_channel = 7;
}

uint8_t namco163_audio_read_data(Namco163Audio *audio) {
    if (!audio) return 0;
    uint8_t value = audio->ram[audio->ram_position];
    if (audio->auto_increment && audio->ram_position != 0x7F) audio->ram_position++;
    return value;
}

bool namco163_audio_write_data(Namco163Audio *audio, uint8_t value) {
    if (!audio) return false;
    bool changed = audio->ram[audio->ram_position] != value;
    audio->ram[audio->ram_position] = value;
    if (audio->auto_increment && audio->ram_position != 0x7F) audio->ram_position++;
    return changed;
}

void namco163_audio_write_address(Namco163Audio *audio, uint8_t value) {
    if (!audio) return;
    audio->ram_position = value & 0x7Fu;
    audio->auto_increment = (value & 0x80u) != 0;
}

void namco163_audio_set_disabled(Namco163Audio *audio, bool disabled) {
    if (audio) audio->sound_disabled = disabled;
}

bool namco163_audio_clock(Namco163Audio *audio, int cpu_cycles) {
    if (!audio || cpu_cycles <= 0 || audio->sound_disabled) return false;
    bool changed = false;
    for (int cycle = 0; cycle < cpu_cycles; ++cycle) {
        audio->update_counter++;
        if (audio->update_counter != 15) continue;
        audio->update_counter = 0;
        changed |= update_channel(audio, (unsigned)audio->current_channel);
        audio->current_channel--;
        if (audio->current_channel < 7 - (int8_t)channel_count_minus_one(audio))
            audio->current_channel = 7;
    }
    return changed;
}

float namco163_audio_output(const Namco163Audio *audio) {
    if (!audio) return 0.0f;
    /* The DAC is centered around sample value 8. This gain matches the
     * cartridge expansion path while preserving the chip's bipolar output. */
    return -(float)audio->mixed_output * (20.0f / 5000.0f);
}

uint8_t *namco163_audio_ram(Namco163Audio *audio) {
    return audio ? audio->ram : NULL;
}
