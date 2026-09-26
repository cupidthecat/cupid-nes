/*
 * vrc7_audio.h - VRC7 FM audio integration
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef VRC7_AUDIO_H
#define VRC7_AUDIO_H

#include <stdbool.h>
#include <stdint.h>
#include "../state/state_io.h"

typedef struct __OPLL OPLL;

typedef struct {
    OPLL *opll;
    uint8_t address;
    double clock_timer;
    float output;
    bool muted;
} Vrc7Fm;

bool vrc7_fm_init(Vrc7Fm *fm);
void vrc7_fm_destroy(Vrc7Fm *fm);
void vrc7_fm_reset(Vrc7Fm *fm);
void vrc7_fm_reset_chip(Vrc7Fm *fm);
void vrc7_fm_set_muted(Vrc7Fm *fm, bool muted);
void vrc7_fm_write_address(Vrc7Fm *fm, uint8_t value);
void vrc7_fm_write_data(Vrc7Fm *fm, uint8_t value);
void vrc7_fm_clock(Vrc7Fm *fm, int cpu_cycles, double cpu_hz);
float vrc7_fm_output(const Vrc7Fm *fm);
bool vrc7_fm_state_capture(NesStateWriter *writer, const Vrc7Fm *fm);
bool vrc7_fm_state_validate(const Vrc7Fm *target, NesStateReader *reader);
bool vrc7_fm_state_apply(Vrc7Fm *target, NesStateReader *reader);

#endif
