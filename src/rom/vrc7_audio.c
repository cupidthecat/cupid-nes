/*
 * vrc7_audio.c - VRC7 FM audio integration
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "vrc7_audio.h"

#include <stddef.h>
#include <string.h>

#include "emu2413.h"

#define VRC7_SAMPLE_RATE 49716.0
#define VRC7_OPLL_CLOCK  (49716u * 72u)
#define VRC7_OUTPUT_SCALE (1.0f / 5000.0f)

bool vrc7_fm_init(Vrc7Fm *fm) {
    if (!fm) return false;
    OPLL *opll = OPLL_new(VRC7_OPLL_CLOCK, (uint32_t)VRC7_SAMPLE_RATE);
    if (!opll) return false;
    *fm = (Vrc7Fm){.opll = opll};
    vrc7_fm_reset(fm);
    return true;
}

void vrc7_fm_destroy(Vrc7Fm *fm) {
    if (!fm) return;
    if (fm->opll) OPLL_delete(fm->opll);
    memset(fm, 0, sizeof(*fm));
}

void vrc7_fm_reset_chip(Vrc7Fm *fm) {
    if (fm && fm->opll) OPLL_reset(fm->opll);
}

void vrc7_fm_reset(Vrc7Fm *fm) {
    if (!fm || !fm->opll) return;
    vrc7_fm_reset_chip(fm);
    OPLL_setChipType(fm->opll, OPLL_VRC7_TONE);
    OPLL_resetPatch(fm->opll, OPLL_VRC7_TONE);
    fm->address = 0;
    fm->clock_timer = 0.0;
    fm->output = 0.0f;
    fm->muted = false;
}

void vrc7_fm_set_muted(Vrc7Fm *fm, bool muted) {
    if (!fm) return;
    fm->muted = muted;
}

void vrc7_fm_write_address(Vrc7Fm *fm, uint8_t value) {
    if (!fm || fm->muted) return;
    fm->address = value;
}

void vrc7_fm_write_data(Vrc7Fm *fm, uint8_t value) {
    if (!fm || !fm->opll || fm->muted) return;
    OPLL_writeReg(fm->opll, fm->address, value);
}

void vrc7_fm_clock(Vrc7Fm *fm, int cpu_cycles, double cpu_hz) {
    if (!fm || !fm->opll || cpu_cycles <= 0 || cpu_hz <= 0.0) return;
    for (int cycle = 0; cycle < cpu_cycles; ++cycle) {
        if (fm->clock_timer == 0.0) fm->clock_timer = cpu_hz / VRC7_SAMPLE_RATE;
        fm->clock_timer -= 1.0;
        if (fm->clock_timer <= 0.0) {
            int16_t sample = OPLL_calc(fm->opll);
            fm->output = (float)sample * VRC7_OUTPUT_SCALE;
            fm->clock_timer = cpu_hz / VRC7_SAMPLE_RATE;
        }
    }
}

float vrc7_fm_output(const Vrc7Fm *fm) {
    return fm && !fm->muted ? fm->output : 0.0f;
}

static bool vrc7_state_write_patch(NesStateWriter *writer, const OPLL_PATCH *patch) {
    const uint32_t *values = &patch->TL;
    for (unsigned i = 0; i < 14; ++i)
        if (!nes_state_write_u32(writer, values[i])) return false;
    return true;
}

static bool vrc7_state_read_patch(NesStateReader *reader, OPLL_PATCH *patch) {
    uint32_t *values = &patch->TL;
    for (unsigned i = 0; i < 14; ++i)
        if (!nes_state_read_u32(reader, &values[i])) return false;
    return true;
}

static bool vrc7_state_write_slot(NesStateWriter *writer, const OPLL_SLOT *slot) {
    return nes_state_write_u8(writer, slot->number)
        && nes_state_write_u8(writer, slot->type)
        && nes_state_write_u32(writer, (uint32_t)slot->output[0])
        && nes_state_write_u32(writer, (uint32_t)slot->output[1])
        && nes_state_write_u32(writer, slot->pg_phase)
        && nes_state_write_u32(writer, slot->pg_out)
        && nes_state_write_u8(writer, slot->pg_keep)
        && nes_state_write_u16(writer, slot->blk_fnum)
        && nes_state_write_u16(writer, slot->fnum)
        && nes_state_write_u8(writer, slot->blk)
        && nes_state_write_u8(writer, slot->eg_state)
        && nes_state_write_u32(writer, (uint32_t)slot->volume)
        && nes_state_write_u8(writer, slot->key_flag)
        && nes_state_write_u8(writer, slot->sus_flag)
        && nes_state_write_u16(writer, slot->tll)
        && nes_state_write_u8(writer, slot->rks)
        && nes_state_write_u8(writer, slot->eg_rate_h)
        && nes_state_write_u8(writer, slot->eg_rate_l)
        && nes_state_write_u32(writer, slot->eg_shift)
        && nes_state_write_u32(writer, slot->eg_out)
        && nes_state_write_u32(writer, slot->update_requests);
}

static bool vrc7_state_read_slot(NesStateReader *reader, OPLL_SLOT *slot) {
    uint32_t output0, output1, volume;
    if (!nes_state_read_u8(reader, &slot->number)
        || !nes_state_read_u8(reader, &slot->type)
        || !nes_state_read_u32(reader, &output0)
        || !nes_state_read_u32(reader, &output1)
        || !nes_state_read_u32(reader, &slot->pg_phase)
        || !nes_state_read_u32(reader, &slot->pg_out)
        || !nes_state_read_u8(reader, &slot->pg_keep)
        || !nes_state_read_u16(reader, &slot->blk_fnum)
        || !nes_state_read_u16(reader, &slot->fnum)
        || !nes_state_read_u8(reader, &slot->blk)
        || !nes_state_read_u8(reader, &slot->eg_state)
        || !nes_state_read_u32(reader, &volume)
        || !nes_state_read_u8(reader, &slot->key_flag)
        || !nes_state_read_u8(reader, &slot->sus_flag)
        || !nes_state_read_u16(reader, &slot->tll)
        || !nes_state_read_u8(reader, &slot->rks)
        || !nes_state_read_u8(reader, &slot->eg_rate_h)
        || !nes_state_read_u8(reader, &slot->eg_rate_l)
        || !nes_state_read_u32(reader, &slot->eg_shift)
        || !nes_state_read_u32(reader, &slot->eg_out)
        || !nes_state_read_u32(reader, &slot->update_requests)) return false;
    slot->output[0] = (int32_t)output0;
    slot->output[1] = (int32_t)output1;
    slot->volume = (int32_t)volume;
    slot->patch = NULL;
    slot->wave_table = NULL;
    return slot->number < 18 && slot->type <= 3 && slot->pg_keep <= 1
        && slot->blk <= 7 && slot->key_flag <= 1 && slot->sus_flag <= 1;
}

static bool vrc7_state_write_opll(NesStateWriter *writer, const OPLL *opll) {
    if (!writer || !opll || opll->conv) return false;
    if (!nes_state_write_u32(writer, opll->clk)
        || !nes_state_write_u32(writer, opll->rate)
        || !nes_state_write_u8(writer, opll->chip_type)
        || !nes_state_write_u32(writer, opll->adr)
        || !nes_state_write_f64(writer, opll->inp_step)
        || !nes_state_write_f64(writer, opll->out_step)
        || !nes_state_write_f64(writer, opll->out_time)
        || !nes_state_write_bytes(writer, opll->reg, sizeof(opll->reg))
        || !nes_state_write_u8(writer, opll->test_flag)
        || !nes_state_write_u32(writer, opll->slot_key_status)
        || !nes_state_write_u8(writer, opll->rhythm_mode)
        || !nes_state_write_u32(writer, opll->eg_counter)
        || !nes_state_write_u32(writer, opll->pm_phase)
        || !nes_state_write_u32(writer, (uint32_t)opll->am_phase)
        || !nes_state_write_u8(writer, opll->lfo_am)
        || !nes_state_write_u32(writer, opll->noise)
        || !nes_state_write_u8(writer, opll->short_noise)) return false;
    for (unsigned i = 0; i < 9; ++i)
        if (!nes_state_write_u32(writer, (uint32_t)opll->patch_number[i])) return false;
    for (unsigned i = 0; i < 18; ++i)
        if (!vrc7_state_write_slot(writer, &opll->slot[i])) return false;
    for (unsigned i = 0; i < 38; ++i)
        if (!vrc7_state_write_patch(writer, &opll->patch[i])) return false;
    if (!nes_state_write_bytes(writer, opll->pan, sizeof(opll->pan))) return false;
    for (unsigned i = 0; i < 16; ++i)
        for (unsigned side = 0; side < 2; ++side)
            if (!nes_state_write_f32(writer, opll->pan_fine[i][side])) return false;
    if (!nes_state_write_u32(writer, opll->mask)) return false;
    for (unsigned i = 0; i < 14; ++i)
        if (!nes_state_write_u16(writer, (uint16_t)opll->ch_out[i])) return false;
    return nes_state_write_u16(writer, (uint16_t)opll->mix_out[0])
        && nes_state_write_u16(writer, (uint16_t)opll->mix_out[1]);
}

static bool vrc7_state_read_opll(NesStateReader *reader, OPLL *opll) {
    uint32_t am_phase;
    memset(opll, 0, sizeof(*opll));
    if (!nes_state_read_u32(reader, &opll->clk)
        || !nes_state_read_u32(reader, &opll->rate)
        || !nes_state_read_u8(reader, &opll->chip_type)
        || !nes_state_read_u32(reader, &opll->adr)
        || !nes_state_read_f64(reader, &opll->inp_step)
        || !nes_state_read_f64(reader, &opll->out_step)
        || !nes_state_read_f64(reader, &opll->out_time)
        || !nes_state_read_bytes(reader, opll->reg, sizeof(opll->reg))
        || !nes_state_read_u8(reader, &opll->test_flag)
        || !nes_state_read_u32(reader, &opll->slot_key_status)
        || !nes_state_read_u8(reader, &opll->rhythm_mode)
        || !nes_state_read_u32(reader, &opll->eg_counter)
        || !nes_state_read_u32(reader, &opll->pm_phase)
        || !nes_state_read_u32(reader, &am_phase)
        || !nes_state_read_u8(reader, &opll->lfo_am)
        || !nes_state_read_u32(reader, &opll->noise)
        || !nes_state_read_u8(reader, &opll->short_noise)) return false;
    opll->am_phase = (int32_t)am_phase;
    for (unsigned i = 0; i < 9; ++i) {
        uint32_t patch_number;
        if (!nes_state_read_u32(reader, &patch_number) || patch_number > 18) return false;
        opll->patch_number[i] = (int32_t)patch_number;
    }
    for (unsigned i = 0; i < 18; ++i)
        if (!vrc7_state_read_slot(reader, &opll->slot[i])) return false;
    for (unsigned i = 0; i < 38; ++i)
        if (!vrc7_state_read_patch(reader, &opll->patch[i])) return false;
    if (!nes_state_read_bytes(reader, opll->pan, sizeof(opll->pan))) return false;
    for (unsigned i = 0; i < 16; ++i)
        for (unsigned side = 0; side < 2; ++side)
            if (!nes_state_read_f32(reader, &opll->pan_fine[i][side])) return false;
    if (!nes_state_read_u32(reader, &opll->mask)) return false;
    for (unsigned i = 0; i < 14; ++i) {
        uint16_t value;
        if (!nes_state_read_u16(reader, &value)) return false;
        opll->ch_out[i] = (int16_t)value;
    }
    for (unsigned i = 0; i < 2; ++i) {
        uint16_t value;
        if (!nes_state_read_u16(reader, &value)) return false;
        opll->mix_out[i] = (int16_t)value;
    }
    return opll->chip_type <= OPLL_VRC7_TONE && opll->rhythm_mode <= 1;
}

static bool vrc7_state_decode(const Vrc7Fm *target, NesStateReader *reader, Vrc7Fm *saved,
                              OPLL *saved_opll) {
    if (!target || !target->opll || target->opll->conv || !reader || !saved || !saved_opll
        || !nes_state_read_u8(reader, &saved->address)
        || !nes_state_read_f64(reader, &saved->clock_timer)
        || !nes_state_read_f32(reader, &saved->output)
        || !nes_state_read_bool(reader, &saved->muted)
        || !vrc7_state_read_opll(reader, saved_opll)
        || nes_state_reader_remaining(reader) != 0
        || saved_opll->clk != target->opll->clk
        || saved_opll->rate != target->opll->rate) return false;
    saved->opll = saved_opll;
    return true;
}

bool vrc7_fm_state_capture(NesStateWriter *writer, const Vrc7Fm *fm) {
    return writer && fm && fm->opll
        && nes_state_write_u8(writer, fm->address)
        && nes_state_write_f64(writer, fm->clock_timer)
        && nes_state_write_f32(writer, fm->output)
        && nes_state_write_bool(writer, fm->muted)
        && vrc7_state_write_opll(writer, fm->opll);
}

bool vrc7_fm_state_validate(const Vrc7Fm *target, NesStateReader *reader) {
    Vrc7Fm saved = {0};
    OPLL opll;
    return vrc7_state_decode(target, reader, &saved, &opll);
}

bool vrc7_fm_state_apply(Vrc7Fm *target, NesStateReader *reader) {
    Vrc7Fm saved = {0};
    OPLL opll;
    if (!vrc7_state_decode(target, reader, &saved, &opll)) return false;
    OPLL_RateConv *conv = target->opll->conv;
    *target->opll = opll;
    target->opll->conv = conv;
    OPLL_rebindState(target->opll);
    target->address = saved.address;
    target->clock_timer = saved.clock_timer;
    target->output = saved.output;
    target->muted = saved.muted;
    return true;
}
