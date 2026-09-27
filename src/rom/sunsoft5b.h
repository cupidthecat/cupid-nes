/*
 * sunsoft5b.h - Sunsoft 5B expansion audio
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef SUNSOFT5B_H
#define SUNSOFT5B_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint8_t registers[16];
    uint8_t selected_register;
    uint8_t master_divider;
    uint16_t tone_counter[3];
    bool tone_output[3];
    uint8_t noise_counter;
    bool noise_phase;
    uint32_t noise_lfsr;
    uint16_t envelope_counter;
    uint8_t envelope_level;
    bool envelope_attack;
    bool envelope_holding;
} Sunsoft5B;

void sunsoft5b_reset(Sunsoft5B *audio);
void sunsoft5b_write(Sunsoft5B *audio, uint16_t address, uint8_t value);
void sunsoft5b_clock(Sunsoft5B *audio, int cpu_cycles);
float sunsoft5b_output(const Sunsoft5B *audio);

#endif
