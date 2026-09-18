/*
 * namco163.h - Namco 163 wavetable audio
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

#ifndef NAMCO163_H
#define NAMCO163_H

#include <stdbool.h>
#include <stdint.h>

#define NAMCO163_RAM_SIZE 0x80u

typedef struct {
    uint8_t ram[NAMCO163_RAM_SIZE];
    int16_t channel_output[8];
    uint8_t ram_position;
    bool auto_increment;
    uint8_t update_counter;
    int8_t current_channel;
    int16_t mixed_output;
    bool sound_disabled;
} Namco163Audio;

void namco163_audio_reset(Namco163Audio *audio);
uint8_t namco163_audio_read_data(Namco163Audio *audio);
bool namco163_audio_write_data(Namco163Audio *audio, uint8_t value);
void namco163_audio_write_address(Namco163Audio *audio, uint8_t value);
void namco163_audio_set_disabled(Namco163Audio *audio, bool disabled);
bool namco163_audio_clock(Namco163Audio *audio, int cpu_cycles);
float namco163_audio_output(const Namco163Audio *audio);
uint8_t *namco163_audio_ram(Namco163Audio *audio);

#endif
