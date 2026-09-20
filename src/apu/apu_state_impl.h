/*
 * apu_state_impl.h - Hardware and audio reconstruction state codec
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_APU_STATE_IMPL_H
#define CUPID_APU_STATE_IMPL_H

static bool apu_state_write_envelope(NesStateWriter *writer, const Envelope *value) {
    return nes_state_write_bool(writer, value->loop_envelope)
        && nes_state_write_bool(writer, value->constant_volume)
        && nes_state_write_u8(writer, value->volume)
        && nes_state_write_u8(writer, value->divider)
        && nes_state_write_u8(writer, value->decay)
        && nes_state_write_bool(writer, value->start_flag);
}

static bool apu_state_read_envelope(NesStateReader *reader, Envelope *value) {
    return nes_state_read_bool(reader, &value->loop_envelope)
        && nes_state_read_bool(reader, &value->constant_volume)
        && nes_state_read_u8(reader, &value->volume)
        && nes_state_read_u8(reader, &value->divider)
        && nes_state_read_u8(reader, &value->decay)
        && nes_state_read_bool(reader, &value->start_flag);
}

static bool apu_state_write_sweep(NesStateWriter *writer, const Sweep *value) {
    return nes_state_write_bool(writer, value->enabled)
        && nes_state_write_u8(writer, value->period)
        && nes_state_write_bool(writer, value->negate)
        && nes_state_write_u8(writer, value->shift)
        && nes_state_write_u8(writer, value->divider)
        && nes_state_write_bool(writer, value->reload);
}

static bool apu_state_read_sweep(NesStateReader *reader, Sweep *value) {
    return nes_state_read_bool(reader, &value->enabled)
        && nes_state_read_u8(reader, &value->period)
        && nes_state_read_bool(reader, &value->negate)
        && nes_state_read_u8(reader, &value->shift)
        && nes_state_read_u8(reader, &value->divider)
        && nes_state_read_bool(reader, &value->reload);
}

static bool apu_state_write_length(NesStateWriter *writer, const LengthCounter *value) {
    return nes_state_write_u8(writer, value->length)
        && nes_state_write_bool(writer, value->halt)
        && nes_state_write_bool(writer, value->next_halt)
        && nes_state_write_bool(writer, value->halt_pending)
        && nes_state_write_u8(writer, value->reload_value)
        && nes_state_write_u8(writer, value->previous_value);
}

static bool apu_state_read_length(NesStateReader *reader, LengthCounter *value) {
    return nes_state_read_u8(reader, &value->length)
        && nes_state_read_bool(reader, &value->halt)
        && nes_state_read_bool(reader, &value->next_halt)
        && nes_state_read_bool(reader, &value->halt_pending)
        && nes_state_read_u8(reader, &value->reload_value)
        && nes_state_read_u8(reader, &value->previous_value);
}

static bool apu_state_write_pulse(NesStateWriter *writer, const Pulse *value) {
    return apu_state_write_envelope(writer, &value->env)
        && apu_state_write_sweep(writer, &value->sweep)
        && apu_state_write_length(writer, &value->lc)
        && nes_state_write_u16(writer, value->timer)
        && nes_state_write_u16(writer, value->timer_reload)
        && nes_state_write_u8(writer, value->duty)
        && nes_state_write_u8(writer, value->duty_step)
        && nes_state_write_u8(writer, value->output_level)
        && nes_state_write_bool(writer, value->enabled);
}

static bool apu_state_read_pulse(NesStateReader *reader, Pulse *value) {
    return apu_state_read_envelope(reader, &value->env)
        && apu_state_read_sweep(reader, &value->sweep)
        && apu_state_read_length(reader, &value->lc)
        && nes_state_read_u16(reader, &value->timer)
        && nes_state_read_u16(reader, &value->timer_reload)
        && nes_state_read_u8(reader, &value->duty)
        && nes_state_read_u8(reader, &value->duty_step)
        && nes_state_read_u8(reader, &value->output_level)
        && nes_state_read_bool(reader, &value->enabled);
}

static bool apu_state_write_triangle(NesStateWriter *writer, const Triangle *value) {
    return apu_state_write_length(writer, &value->lc)
        && nes_state_write_bool(writer, value->control)
        && nes_state_write_u8(writer, value->linear_reload_val)
        && nes_state_write_u8(writer, value->linear_counter)
        && nes_state_write_bool(writer, value->linear_reload)
        && nes_state_write_u16(writer, value->timer)
        && nes_state_write_u16(writer, value->timer_reload)
        && nes_state_write_u8(writer, value->step)
        && nes_state_write_u8(writer, value->output_level)
        && nes_state_write_bool(writer, value->enabled);
}

static bool apu_state_read_triangle(NesStateReader *reader, Triangle *value) {
    return apu_state_read_length(reader, &value->lc)
        && nes_state_read_bool(reader, &value->control)
        && nes_state_read_u8(reader, &value->linear_reload_val)
        && nes_state_read_u8(reader, &value->linear_counter)
        && nes_state_read_bool(reader, &value->linear_reload)
        && nes_state_read_u16(reader, &value->timer)
        && nes_state_read_u16(reader, &value->timer_reload)
        && nes_state_read_u8(reader, &value->step)
        && nes_state_read_u8(reader, &value->output_level)
        && nes_state_read_bool(reader, &value->enabled);
}

static bool apu_state_write_noise(NesStateWriter *writer, const Noise *value) {
    return apu_state_write_envelope(writer, &value->env)
        && apu_state_write_length(writer, &value->lc)
        && nes_state_write_bool(writer, value->mode)
        && nes_state_write_u16(writer, value->lfsr)
        && nes_state_write_u16(writer, value->period)
        && nes_state_write_u8(writer, value->period_idx)
        && nes_state_write_u16(writer, value->timer)
        && nes_state_write_u8(writer, value->output_level)
        && nes_state_write_bool(writer, value->enabled);
}

static bool apu_state_read_noise(NesStateReader *reader, Noise *value) {
    return apu_state_read_envelope(reader, &value->env)
        && apu_state_read_length(reader, &value->lc)
        && nes_state_read_bool(reader, &value->mode)
        && nes_state_read_u16(reader, &value->lfsr)
        && nes_state_read_u16(reader, &value->period)
        && nes_state_read_u8(reader, &value->period_idx)
        && nes_state_read_u16(reader, &value->timer)
        && nes_state_read_u8(reader, &value->output_level)
        && nes_state_read_bool(reader, &value->enabled);
}

static bool apu_state_write_dmc(NesStateWriter *writer, const DMC *value) {
    return nes_state_write_bool(writer, value->enabled)
        && nes_state_write_bool(writer, value->irq_enable)
        && nes_state_write_bool(writer, value->irq_flag)
        && nes_state_write_bool(writer, value->loop)
        && nes_state_write_u8(writer, value->rate_index)
        && nes_state_write_u16(writer, value->timer)
        && nes_state_write_u16(writer, value->timer_reload)
        && nes_state_write_u8(writer, value->output_level)
        && nes_state_write_u8(writer, value->sample_addr_reg)
        && nes_state_write_u8(writer, value->sample_len_reg)
        && nes_state_write_u16(writer, value->sample_addr)
        && nes_state_write_u16(writer, value->sample_len)
        && nes_state_write_u16(writer, value->current_addr)
        && nes_state_write_u16(writer, value->bytes_remaining)
        && nes_state_write_u8(writer, value->shift_reg)
        && nes_state_write_u8(writer, value->bits_remaining)
        && nes_state_write_bool(writer, value->silence)
        && nes_state_write_u8(writer, value->sample_buffer)
        && nes_state_write_bool(writer, value->sample_buffer_empty)
        && nes_state_write_u8(writer, value->start_delay)
        && nes_state_write_u8(writer, value->disable_delay)
        && nes_state_write_bool(writer, value->dma_pending)
        && nes_state_write_bool(writer, value->dma_halt_started)
        && nes_state_write_bool(writer, value->dma_abort_requested);
}

static bool apu_state_read_dmc(NesStateReader *reader, DMC *value) {
    return nes_state_read_bool(reader, &value->enabled)
        && nes_state_read_bool(reader, &value->irq_enable)
        && nes_state_read_bool(reader, &value->irq_flag)
        && nes_state_read_bool(reader, &value->loop)
        && nes_state_read_u8(reader, &value->rate_index)
        && nes_state_read_u16(reader, &value->timer)
        && nes_state_read_u16(reader, &value->timer_reload)
        && nes_state_read_u8(reader, &value->output_level)
        && nes_state_read_u8(reader, &value->sample_addr_reg)
        && nes_state_read_u8(reader, &value->sample_len_reg)
        && nes_state_read_u16(reader, &value->sample_addr)
        && nes_state_read_u16(reader, &value->sample_len)
        && nes_state_read_u16(reader, &value->current_addr)
        && nes_state_read_u16(reader, &value->bytes_remaining)
        && nes_state_read_u8(reader, &value->shift_reg)
        && nes_state_read_u8(reader, &value->bits_remaining)
        && nes_state_read_bool(reader, &value->silence)
        && nes_state_read_u8(reader, &value->sample_buffer)
        && nes_state_read_bool(reader, &value->sample_buffer_empty)
        && nes_state_read_u8(reader, &value->start_delay)
        && nes_state_read_u8(reader, &value->disable_delay)
        && nes_state_read_bool(reader, &value->dma_pending)
        && nes_state_read_bool(reader, &value->dma_halt_started)
        && nes_state_read_bool(reader, &value->dma_abort_requested);
}

enum { APU_BLIP_STATE_SAMPLES = APU_RECONSTRUCTION_CAP + 18 };

typedef struct {
    APU state;
    bool has_reconstruction;
    BlipStateHeader reconstruction;
    int32_t reconstruction_samples[APU_BLIP_STATE_SAMPLES];
    bool has_right_reconstruction;
    BlipStateHeader right_reconstruction;
    int32_t right_reconstruction_samples[APU_BLIP_STATE_SAMPLES];
} ApuSavedMachine;

static bool apu_state_write_right(NesStateWriter *writer, const ApuRightOutput *right) {
    if (!nes_state_write_u32(writer, (uint32_t)right->level)
        || !nes_state_write_f32(writer, right->hp90_prev_in)
        || !nes_state_write_f32(writer, right->hp90_prev_out)
        || !nes_state_write_f32(writer, right->hp440_prev_in)
        || !nes_state_write_f32(writer, right->hp440_prev_out)
        || !nes_state_write_f32(writer, right->lp14k_prev_out)
        || !nes_state_write_bool(writer, right->reconstruction != NULL)) return false;
    if (!right->reconstruction) return true;
    BlipStateHeader header;
    int32_t samples[APU_BLIP_STATE_SAMPLES];
    if (blip_state_sample_count(right->reconstruction) != APU_BLIP_STATE_SAMPLES
        || !blip_state_export(right->reconstruction, &header, samples, APU_BLIP_STATE_SAMPLES)
        || !nes_state_write_u64(writer, header.factor)
        || !nes_state_write_u64(writer, header.offset)
        || !nes_state_write_u32(writer, (uint32_t)header.available)
        || !nes_state_write_u32(writer, (uint32_t)header.size)
        || !nes_state_write_u32(writer, (uint32_t)header.integrator)) return false;
    for (unsigned i = 0; i < APU_BLIP_STATE_SAMPLES; ++i)
        if (!nes_state_write_u32(writer, (uint32_t)samples[i])) return false;
    return true;
}

static bool apu_state_read_right(NesStateReader *reader, ApuSavedMachine *saved) {
    ApuRightOutput *right = &saved->state.right_output;
    uint32_t level;
    if (!nes_state_read_u32(reader, &level)
        || !nes_state_read_f32(reader, &right->hp90_prev_in)
        || !nes_state_read_f32(reader, &right->hp90_prev_out)
        || !nes_state_read_f32(reader, &right->hp440_prev_in)
        || !nes_state_read_f32(reader, &right->hp440_prev_out)
        || !nes_state_read_f32(reader, &right->lp14k_prev_out)
        || !nes_state_read_bool(reader, &saved->has_right_reconstruction)) return false;
    right->level = (int32_t)level;
    if (right->level < -32768 || right->level > 32767
        || !isfinite(right->hp90_prev_in) || !isfinite(right->hp90_prev_out)
        || !isfinite(right->hp440_prev_in) || !isfinite(right->hp440_prev_out)
        || !isfinite(right->lp14k_prev_out)) return false;
    if (!saved->has_right_reconstruction) return true;
    uint32_t available, size, integrator;
    if (!nes_state_read_u64(reader, &saved->right_reconstruction.factor)
        || !nes_state_read_u64(reader, &saved->right_reconstruction.offset)
        || !nes_state_read_u32(reader, &available)
        || !nes_state_read_u32(reader, &size)
        || !nes_state_read_u32(reader, &integrator)) return false;
    saved->right_reconstruction.available = (int32_t)available;
    saved->right_reconstruction.size = (int32_t)size;
    saved->right_reconstruction.integrator = (int32_t)integrator;
    if (size != APU_RECONSTRUCTION_CAP || available > size) return false;
    for (unsigned i = 0; i < APU_BLIP_STATE_SAMPLES; ++i) {
        uint32_t sample;
        if (!nes_state_read_u32(reader, &sample)) return false;
        saved->right_reconstruction_samples[i] = (int32_t)sample;
    }
    return true;
}

bool apu_machine_hardware_state_capture(NesStateWriter *writer, const APU *state) {
    if (!writer || !state
        || !nes_state_write_u32(writer, state->cycle_in_seq)
        || !nes_state_write_bool(writer, state->five_step)
        || !nes_state_write_bool(writer, state->irq_inhibit)
        || !nes_state_write_bool(writer, state->frame_irq)
        || !nes_state_write_bool(writer, state->frame_irq_source)
        || !nes_state_write_u8(writer, state->frame_irq_clear_delay)
        || !nes_state_write_u8(writer, state->frame_reset_delay)
        || !nes_state_write_bool(writer, state->frame_reset_pending)
        || !nes_state_write_bool(writer, state->cpu_cycle_odd)
        || !nes_state_write_bool(writer, state->frame_next_five_step)
        || !nes_state_write_u8(writer, state->frame_clock_block)
        || !apu_state_write_pulse(writer, &state->pulse1)
        || !apu_state_write_pulse(writer, &state->pulse2)
        || !apu_state_write_triangle(writer, &state->tri)
        || !apu_state_write_noise(writer, &state->noise)
        || !apu_state_write_dmc(writer, &state->dmc)
        || !nes_state_write_bytes(writer, state->regs, sizeof(state->regs))) return false;
    return true;
}

bool apu_hardware_state_capture(NesStateWriter *writer) {
    return writer && nes_state_write_u8(writer, (uint8_t)cpu_revision)
        && nes_state_write_bool(writer, disable_noise_mode)
        && nes_state_write_bool(writer, swap_duty_cycles)
        && apu_machine_hardware_state_capture(writer, main_apu);
}

static bool apu_state_write_machine(NesStateWriter *writer, const APU *state) {
    if (!apu_machine_hardware_state_capture(writer, state)
        || !nes_state_write_f64(writer, state->sample_rate)
        || !nes_state_write_f64(writer, state->cycles_per_sample)
        || !nes_state_write_f64(writer, state->sample_accum)
        || !nes_state_write_u32(writer, (uint32_t)state->reconstructed_level)
        || !nes_state_write_u64(writer, state->audio_transition_count)
        || !nes_state_write_f32(writer, state->hp90_alpha)
        || !nes_state_write_f32(writer, state->hp440_alpha)
        || !nes_state_write_f32(writer, state->lp14k_alpha)
        || !nes_state_write_f32(writer, state->hp90_prev_in)
        || !nes_state_write_f32(writer, state->hp90_prev_out)
        || !nes_state_write_f32(writer, state->hp440_prev_in)
        || !nes_state_write_f32(writer, state->hp440_prev_out)
        || !nes_state_write_f32(writer, state->lp14k_prev_out)
        || !nes_state_write_f32(writer, state->last_output_sample)
        || !nes_state_write_f32(writer, state->last_read_sample)
        || !nes_state_write_f32(writer, state->last_read_side)) return false;
    for (unsigned i = 0; i < APU_RING_CAP; ++i) {
        if (!nes_state_write_f32(writer, state->ring[i])
            || !nes_state_write_f32(writer, state->ring_side[i])) return false;
    }
    uint32_t ring_w = atomic_load_explicit(&state->ring_w, memory_order_acquire);
    uint32_t ring_r = atomic_load_explicit(&state->ring_r, memory_order_acquire);
    if (!nes_state_write_u32(writer, ring_w)
        || !nes_state_write_u32(writer, ring_r)
        || !nes_state_write_bool(writer, state->reconstruction != NULL)) return false;
    if (!state->reconstruction) return apu_state_write_right(writer, &state->right_output);
    BlipStateHeader header;
    int32_t samples[APU_BLIP_STATE_SAMPLES];
    if (blip_state_sample_count(state->reconstruction) != APU_BLIP_STATE_SAMPLES
        || !blip_state_export(state->reconstruction, &header, samples,
                              APU_BLIP_STATE_SAMPLES)
        || !nes_state_write_u64(writer, header.factor)
        || !nes_state_write_u64(writer, header.offset)
        || !nes_state_write_u32(writer, (uint32_t)header.available)
        || !nes_state_write_u32(writer, (uint32_t)header.size)
        || !nes_state_write_u32(writer, (uint32_t)header.integrator)) return false;
    for (unsigned i = 0; i < APU_BLIP_STATE_SAMPLES; ++i)
        if (!nes_state_write_u32(writer, (uint32_t)samples[i])) return false;
    return apu_state_write_right(writer, &state->right_output);
}

static bool apu_state_read_machine(NesStateReader *reader, ApuSavedMachine *saved) {
    uint32_t reconstructed, ring_w, ring_r;
    memset(saved, 0, sizeof(*saved));
    if (!nes_state_read_u32(reader, &saved->state.cycle_in_seq)
        || !nes_state_read_bool(reader, &saved->state.five_step)
        || !nes_state_read_bool(reader, &saved->state.irq_inhibit)
        || !nes_state_read_bool(reader, &saved->state.frame_irq)
        || !nes_state_read_bool(reader, &saved->state.frame_irq_source)
        || !nes_state_read_u8(reader, &saved->state.frame_irq_clear_delay)
        || !nes_state_read_u8(reader, &saved->state.frame_reset_delay)
        || !nes_state_read_bool(reader, &saved->state.frame_reset_pending)
        || !nes_state_read_bool(reader, &saved->state.cpu_cycle_odd)
        || !nes_state_read_bool(reader, &saved->state.frame_next_five_step)
        || !nes_state_read_u8(reader, &saved->state.frame_clock_block)
        || !apu_state_read_pulse(reader, &saved->state.pulse1)
        || !apu_state_read_pulse(reader, &saved->state.pulse2)
        || !apu_state_read_triangle(reader, &saved->state.tri)
        || !apu_state_read_noise(reader, &saved->state.noise)
        || !apu_state_read_dmc(reader, &saved->state.dmc)
        || !nes_state_read_bytes(reader, saved->state.regs, sizeof(saved->state.regs))
        || !nes_state_read_f64(reader, &saved->state.sample_rate)
        || !nes_state_read_f64(reader, &saved->state.cycles_per_sample)
        || !nes_state_read_f64(reader, &saved->state.sample_accum)
        || !nes_state_read_u32(reader, &reconstructed)
        || !nes_state_read_u64(reader, &saved->state.audio_transition_count)
        || !nes_state_read_f32(reader, &saved->state.hp90_alpha)
        || !nes_state_read_f32(reader, &saved->state.hp440_alpha)
        || !nes_state_read_f32(reader, &saved->state.lp14k_alpha)
        || !nes_state_read_f32(reader, &saved->state.hp90_prev_in)
        || !nes_state_read_f32(reader, &saved->state.hp90_prev_out)
        || !nes_state_read_f32(reader, &saved->state.hp440_prev_in)
        || !nes_state_read_f32(reader, &saved->state.hp440_prev_out)
        || !nes_state_read_f32(reader, &saved->state.lp14k_prev_out)
        || !nes_state_read_f32(reader, &saved->state.last_output_sample)
        || !nes_state_read_f32(reader, &saved->state.last_read_sample)
        || !nes_state_read_f32(reader, &saved->state.last_read_side)) return false;
    saved->state.reconstructed_level = (int32_t)reconstructed;
    for (unsigned i = 0; i < APU_RING_CAP; ++i) {
        if (!nes_state_read_f32(reader, &saved->state.ring[i])
            || !nes_state_read_f32(reader, &saved->state.ring_side[i])) return false;
    }
    if (!nes_state_read_u32(reader, &ring_w)
        || !nes_state_read_u32(reader, &ring_r)
        || !nes_state_read_bool(reader, &saved->has_reconstruction)
        || ring_w >= APU_RING_CAP || ring_r >= APU_RING_CAP
        || !isfinite(saved->state.sample_rate) || saved->state.sample_rate <= 1.0
        || !isfinite(saved->state.cycles_per_sample)
        || !isfinite(saved->state.sample_accum)) return false;
    atomic_init(&saved->state.ring_w, ring_w);
    atomic_init(&saved->state.ring_r, ring_r);
    if (saved->has_reconstruction) {
        uint32_t available, size, integrator;
        if (!nes_state_read_u64(reader, &saved->reconstruction.factor)
            || !nes_state_read_u64(reader, &saved->reconstruction.offset)
            || !nes_state_read_u32(reader, &available)
            || !nes_state_read_u32(reader, &size)
            || !nes_state_read_u32(reader, &integrator)) return false;
        saved->reconstruction.available = (int32_t)available;
        saved->reconstruction.size = (int32_t)size;
        saved->reconstruction.integrator = (int32_t)integrator;
        if (saved->reconstruction.size != APU_RECONSTRUCTION_CAP
            || saved->reconstruction.available < 0
            || saved->reconstruction.available > saved->reconstruction.size) return false;
        for (unsigned i = 0; i < APU_BLIP_STATE_SAMPLES; ++i) {
            uint32_t sample;
            if (!nes_state_read_u32(reader, &sample)) return false;
            saved->reconstruction_samples[i] = (int32_t)sample;
        }
    }
    return apu_state_read_right(reader, saved);
}

static bool apu_state_machine_compatible(const APU *target, const ApuSavedMachine *saved) {
    if (!target || !saved) return false;
    if (saved->has_reconstruction) {
        if (!target->reconstruction
            || blip_state_sample_count(target->reconstruction) != APU_BLIP_STATE_SAMPLES)
            return false;
    }
    if (saved->has_right_reconstruction
        && (!target->right_output.reconstruction
            || blip_state_sample_count(target->right_output.reconstruction) != APU_BLIP_STATE_SAMPLES))
        return false;
    return true;
}

bool apu_machine_state_capture(NesStateWriter *writer, const APU *state) {
    return apu_state_write_machine(writer, state);
}

bool apu_machine_state_validate(const APU *target, NesStateReader *reader) {
    ApuSavedMachine saved;
    return reader && apu_state_read_machine(reader, &saved)
        && nes_state_reader_remaining(reader) == 0
        && apu_state_machine_compatible(target, &saved);
}

bool apu_machine_state_apply(APU *target, NesStateReader *reader) {
    ApuSavedMachine saved;
    if (!reader || !apu_state_read_machine(reader, &saved)
        || nes_state_reader_remaining(reader) != 0
        || !apu_state_machine_compatible(target, &saved)) return false;
    blip_t *reconstruction = target->reconstruction;
    blip_t *right_reconstruction = target->right_output.reconstruction;
    uint32_t ring_w = atomic_load_explicit(&saved.state.ring_w, memory_order_relaxed);
    uint32_t ring_r = atomic_load_explicit(&saved.state.ring_r, memory_order_relaxed);
    *target = saved.state;
    target->reconstruction = reconstruction;
    target->right_output.reconstruction = right_reconstruction;
    atomic_store_explicit(&target->ring_w, ring_w, memory_order_relaxed);
    atomic_store_explicit(&target->ring_r, ring_r, memory_order_relaxed);
    if (saved.has_reconstruction
        && !blip_state_import(target->reconstruction, &saved.reconstruction,
                              saved.reconstruction_samples, APU_BLIP_STATE_SAMPLES)) return false;
    if (saved.has_right_reconstruction
        && !blip_state_import(target->right_output.reconstruction, &saved.right_reconstruction,
                              saved.right_reconstruction_samples, APU_BLIP_STATE_SAMPLES)) return false;
    return true;
}

bool apu_state_capture(NesStateWriter *writer) {
    return writer && apu_state_write_machine(writer, main_apu)
        && nes_state_write_u8(writer, (uint8_t)cpu_revision)
        && nes_state_write_bool(writer, disable_noise_mode)
        && nes_state_write_bool(writer, swap_duty_cycles);
}

static bool apu_state_decode_main(NesStateReader *reader, ApuSavedMachine *saved,
                                  ApuCpuRevision *revision, bool *disable_noise,
                                  bool *swap_duty) {
    uint8_t encoded_revision;
    if (!apu_state_read_machine(reader, saved)
        || !nes_state_read_u8(reader, &encoded_revision)
        || !nes_state_read_bool(reader, disable_noise)
        || !nes_state_read_bool(reader, swap_duty)
        || encoded_revision > APU_CPU_REVISION_LATE_2A03
        || nes_state_reader_remaining(reader) != 0) return false;
    *revision = (ApuCpuRevision)encoded_revision;
    return apu_state_machine_compatible(main_apu, saved);
}

bool apu_state_validate(NesStateReader *reader) {
    ApuSavedMachine saved;
    ApuCpuRevision revision;
    bool disable_noise, swap_duty;
    return reader && apu_state_decode_main(reader, &saved, &revision,
                                           &disable_noise, &swap_duty);
}

bool apu_state_apply(NesStateReader *reader) {
    ApuSavedMachine saved;
    ApuCpuRevision revision;
    bool disable_noise, swap_duty;
    if (!reader || !apu_state_decode_main(reader, &saved, &revision,
                                          &disable_noise, &swap_duty)) return false;
    blip_t *reconstruction = main_apu->reconstruction;
    blip_t *right_reconstruction = main_apu->right_output.reconstruction;
    uint32_t ring_w = atomic_load_explicit(&saved.state.ring_w, memory_order_relaxed);
    uint32_t ring_r = atomic_load_explicit(&saved.state.ring_r, memory_order_relaxed);
    *main_apu = saved.state;
    main_apu->reconstruction = reconstruction;
    main_apu->right_output.reconstruction = right_reconstruction;
    atomic_store_explicit(&main_apu->ring_w, ring_w, memory_order_relaxed);
    atomic_store_explicit(&main_apu->ring_r, ring_r, memory_order_relaxed);
    if (saved.has_reconstruction
        && !blip_state_import(main_apu->reconstruction, &saved.reconstruction,
                              saved.reconstruction_samples, APU_BLIP_STATE_SAMPLES)) return false;
    if (saved.has_right_reconstruction
        && !blip_state_import(main_apu->right_output.reconstruction, &saved.right_reconstruction,
                              saved.right_reconstruction_samples, APU_BLIP_STATE_SAMPLES)) return false;
    cpu_revision = revision;
    disable_noise_mode = disable_noise;
    swap_duty_cycles = swap_duty;
    apu_select_machine(NULL);
    return true;
}

#endif
