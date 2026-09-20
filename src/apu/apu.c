/*
 * apu.c - Audio Processing Unit (APU) emulation
 *
 * Author: @frankischilling
 *
 * This file implements the NES APU and its five audio channels: two pulse channels,
 * triangle, noise, and DMC. It also handles envelopes, sweep units, length counters,
 * frame sequencing, nonlinear mixing, audio buffering, and the SDL audio callback.
 *
 * This file is part of Cupid NES Emulator.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "apu.h"
#include "epsm.h"
#include "../audio/audio_observer.h"
#include "../audio/audio_mix.h"
#include "../third_party/blip_buf.h"
#include "../rom/mapper.h"
#include "../cpu/cpu.h"
#include "../system/timing.h"
#include "../system/execution_policy.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

APU apu;
static APU *const main_apu = &apu;
static APU *active_apu = &apu;
#define apu (*active_apu)
static ApuCpuRevision cpu_revision = APU_CPU_REVISION_EARLY_2A03;
static bool disable_noise_mode;
static bool swap_duty_cycles;

enum { APU_RECONSTRUCTION_CAP = 64, APU_RECONSTRUCTION_SCALE = 16384 };

void apu_select_machine(APU *state) {
    active_apu = state ? state : main_apu;
}

APU *apu_active_state(void) {
    return active_apu;
}

// Duty sequences
static const uint8_t DUTY_SEQ[4][8] = {
    {0,1,0,0,0,0,0,0}, // 12.5%
    {0,1,1,0,0,0,0,0}, // 25%
    {0,1,1,1,1,0,0,0}, // 50%
    {1,0,0,1,1,1,1,1}, // 75% (inverted 25%)
};

static const uint8_t TRI_SEQ[32] = {
    15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,
     0, 1, 2, 3, 4, 5,6,7,8,9,10,11,12,13,14,15
};

// Length counter table (from NES APU docs)
static const uint8_t LENGTH_TABLE[32] = {
    10,254, 20,  2, 40,  4, 80,  6, 160,  8, 60, 10, 14, 12, 26, 14,
    12, 16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30
};

static const uint16_t NOISE_PERIOD_NTSC[16] = {
    4,8,16,32,64,96,128,160,202,254,380,508,762,1016,2034,4068
};

static const uint16_t NOISE_PERIOD_PAL[16] = {
    4,8,14,30,60,88,118,148,188,236,354,472,708,944,1890,3778
};

static const uint16_t DMC_PERIOD_NTSC[16] = {
    428, 380, 340, 320, 286, 254, 226, 214,
    190, 160, 142, 128, 106,  84,  72,  54
};

static const uint16_t DMC_PERIOD_PAL[16] = {
    398, 354, 316, 298, 276, 236, 210, 198,
    176, 148, 132, 118,  98,  78,  66,  50
};

static const uint32_t FRAME_STEPS_NTSC[2][6] = {
    {7457, 14913, 22371, 29828, 29829, 29830},
    {7457, 14913, 22371, 29829, 37281, 37282},
};

static const uint32_t FRAME_STEPS_PAL[2][6] = {
    {8313, 16627, 24939, 33252, 33253, 33254},
    {8313, 16627, 24939, 33253, 41565, 41566},
};

static inline bool apu_uses_pal_tables(void) {
    return nes_timing()->region == NES_REGION_PAL;
}

static inline const uint16_t *noise_period_table(void) {
    return apu_uses_pal_tables() ? NOISE_PERIOD_PAL : NOISE_PERIOD_NTSC;
}

static inline const uint16_t *dmc_period_table(void) {
    return apu_uses_pal_tables() ? DMC_PERIOD_PAL : DMC_PERIOD_NTSC;
}

static inline const uint32_t (*frame_step_table(void))[6] {
    return apu_uses_pal_tables() ? FRAME_STEPS_PAL : FRAME_STEPS_NTSC;
}

static void sweep_clock(Pulse* p, bool is_ch2);
static void tri_linear_clock(Triangle* t);
static void dmc_restart_sample(DMC* d);
static void dmc_request_buffer(APU* a);
static void dmc_clock_output(APU* a);

// Audio ring buffer.
static inline uint32_t rb_next(uint32_t v){ return (v+1) & (APU_RING_CAP-1); }
static inline bool rb_push(APU* a, float s, float side){
    uint32_t w = atomic_load_explicit(&a->ring_w, memory_order_relaxed);
    uint32_t n = rb_next(w);
    if (n == atomic_load_explicit(&a->ring_r, memory_order_acquire)) return false;
    a->ring[w] = s;
    a->ring_side[w] = side;
    atomic_store_explicit(&a->ring_w, n, memory_order_release);
    return true;
}
static inline int rb_pull(APU* a, float* out, int n){
    int got = 0;
    uint32_t r = atomic_load_explicit(&a->ring_r, memory_order_relaxed);
    while (got < n && r != atomic_load_explicit(&a->ring_w, memory_order_acquire)) {
        out[got++] = a->ring[r];
        a->last_read_side = a->ring_side[r];
        r = rb_next(r);
        atomic_store_explicit(&a->ring_r, r, memory_order_release);
    }
    return got;
}

static inline float one_pole_hp(float x, float alpha, float *prev_in, float *prev_out) {
    float y = alpha * ((*prev_out) + x - (*prev_in));
    *prev_in = x;
    *prev_out = y;
    return y;
}

static inline float one_pole_lp(float x, float alpha, float *prev_out) {
    float y = (*prev_out) + alpha * (x - (*prev_out));
    *prev_out = y;
    return y;
}

static void apu_init_filter_coeffs(APU *a) {
    const double sr = (a->sample_rate > 1.0) ? a->sample_rate : 44100.0;
    const double dt = 1.0 / sr;
    const double pi = 3.14159265358979323846;

    const double rc_hp90 = 1.0 / (2.0 * pi * 90.0);
    const double rc_hp440 = 1.0 / (2.0 * pi * 440.0);
    const double rc_lp14k = 1.0 / (2.0 * pi * 14000.0);

    a->hp90_alpha = (float)(rc_hp90 / (rc_hp90 + dt));
    a->hp440_alpha = (float)(rc_hp440 / (rc_hp440 + dt));
    a->lp14k_alpha = (float)(dt / (rc_lp14k + dt));

    a->hp90_prev_in = 0.0f;
    a->hp90_prev_out = 0.0f;
    a->hp440_prev_in = 0.0f;
    a->hp440_prev_out = 0.0f;
    a->lp14k_prev_out = 0.0f;
    a->last_output_sample = 0.0f;
    a->right_output.hp90_prev_in = a->right_output.hp90_prev_out = 0.0f;
    a->right_output.hp440_prev_in = a->right_output.hp440_prev_out = 0.0f;
    a->right_output.lp14k_prev_out = 0.0f;
}

static inline float apu_post_filter(APU *a, float s) {
    s = one_pole_hp(s, a->hp90_alpha, &a->hp90_prev_in, &a->hp90_prev_out);
    s = one_pole_hp(s, a->hp440_alpha, &a->hp440_prev_in, &a->hp440_prev_out);
    s = one_pole_lp(s, a->lp14k_alpha, &a->lp14k_prev_out);
    return s;
}

static float apu_post_filter_right(APU *a, float sample) {
    ApuRightOutput *output = &a->right_output;
    sample = one_pole_hp(sample, a->hp90_alpha, &output->hp90_prev_in, &output->hp90_prev_out);
    sample = one_pole_hp(sample, a->hp440_alpha, &output->hp440_prev_in, &output->hp440_prev_out);
    return one_pole_lp(sample, a->lp14k_alpha, &output->lp14k_prev_out);
}

static void apu_reconstruction_reset(APU *a) {
    if (!a->reconstruction) a->reconstruction = blip_new(APU_RECONSTRUCTION_CAP);
    if (!a->reconstruction) return;
    blip_set_rates(a->reconstruction, nes_timing()->cpu_hz,
                   a->sample_rate > 1.0 ? a->sample_rate : 44100.0);
    blip_clear(a->reconstruction);
    a->reconstructed_level = 0;
    a->audio_transition_count = 0;
    if (!a->right_output.reconstruction)
        a->right_output.reconstruction = blip_new(APU_RECONSTRUCTION_CAP);
    if (a->right_output.reconstruction) {
        blip_set_rates(a->right_output.reconstruction, nes_timing()->cpu_hz,
                       a->sample_rate > 1.0 ? a->sample_rate : 44100.0);
        blip_clear(a->right_output.reconstruction);
    }
    a->right_output.level = 0;
}

void apu_audio_shutdown_state(APU *a) {
    if (!a) return;
    blip_delete(a->reconstruction);
    blip_delete(a->right_output.reconstruction);
    a->reconstruction = NULL;
    memset(&a->right_output, 0, sizeof(a->right_output));
    a->reconstructed_level = 0;
    a->audio_transition_count = 0;
}

// Envelope unit.
static void env_clock(Envelope* e) {
    if (e->start_flag) {
        e->start_flag = false;
        e->decay = 15;
        e->divider = e->volume;
    } else {
        if (e->divider == 0) {
            e->divider = e->volume;
            if (e->decay == 0) {
                if (e->loop_envelope) e->decay = 15;
            } else {
                e->decay--;
            }
        } else {
            e->divider--;
        }
    }
}
static inline uint8_t env_output(const Envelope* e) {
    return e->constant_volume ? e->volume : e->decay;
}

// Length counter.
static inline void length_clock(LengthCounter* l){
    if (!l->halt && l->length > 0) l->length--;
}

static void length_set_halt(LengthCounter *counter, bool halt) {
    counter->next_halt = halt;
    counter->halt_pending = true;
}

static void length_load(LengthCounter *counter, uint8_t value, bool enabled) {
    if (enabled) {
        counter->reload_value = LENGTH_TABLE[(value >> 3) & 31];
        counter->previous_value = counter->length;
    }
}

static void length_apply_write(LengthCounter *counter) {
    if (counter->reload_value) {
        // A simultaneous half-frame decrement wins when a nonzero counter
        // was running. A zero counter can still load on that clock.
        if (counter->length == counter->previous_value)
            counter->length = counter->reload_value;
        counter->reload_value = 0;
    }
    if (counter->halt_pending) {
        counter->halt = counter->next_halt;
        counter->halt_pending = false;
    }
}

static inline void apu_clock_quarter_frame(APU *a) {
    env_clock(&a->pulse1.env);
    env_clock(&a->pulse2.env);
    env_clock(&a->noise.env);
    tri_linear_clock(&a->tri);
}

static inline void apu_clock_half_frame(APU *a) {
    length_clock(&a->pulse1.lc);
    length_clock(&a->pulse2.lc);
    length_clock(&a->tri.lc);
    length_clock(&a->noise.lc);
    sweep_clock(&a->pulse1, false);
    sweep_clock(&a->pulse2, true);
}

// Pulse-channel sweep unit.
static inline uint16_t sweep_target(uint16_t t, const Sweep* s, bool ch2){
    uint16_t change = t >> s->shift;
    if (s->negate) {
        // Pulse 1 uses one's complement subtraction; pulse 2 uses two's complement.
        return ch2 ? (t - change) : (t - change - 1);
    } else {
        return t + change;
    }
}

static inline uint8_t pulse_current_output(const Pulse* p) {
    if (p->lc.length == 0) return 0;
    if (p->timer_reload < 8 || p->timer_reload > 0x7FF) return 0;
    if (!p->sweep.negate && sweep_target(p->timer_reload, &p->sweep, false) > 0x7FF)
        return 0;
    if (!DUTY_SEQ[p->duty][p->duty_step]) return 0;
    return env_output(&p->env);
}

static inline uint8_t noise_current_output(const Noise* n) {
    if (n->lc.length == 0 || (n->lfsr & 1)) return 0;
    return env_output(&n->env);
}

static void sweep_clock(Pulse* p, bool is_ch2){
    p->sweep.divider--;
    if (p->sweep.divider == 0) {
        if (p->sweep.enabled && p->sweep.shift && p->timer_reload >= 8) {
            uint16_t tgt = sweep_target(p->timer_reload, &p->sweep, is_ch2);
            if (tgt < 0x800) p->timer_reload = tgt;
        }
        p->sweep.divider = p->sweep.period;
    }
    if (p->sweep.reload) {
        p->sweep.divider = p->sweep.period;
        p->sweep.reload = false;
    }
}

// Triangle linear counter.
static void tri_linear_clock(Triangle* t){
    if (t->linear_reload) {
        t->linear_counter = t->linear_reload_val;
    } else if (t->linear_counter > 0) {
        t->linear_counter--;
    }
    if (!t->control) t->linear_reload = false;
}

// Power, reset, and audio initialization.
static void apu_reset_state(APU *a, bool soft_reset) {
    double sample_rate = a->sample_rate > 1.0 ? a->sample_rate : 44100.0;
    blip_t *reconstruction = a->reconstruction;
    blip_t *right_reconstruction = a->right_output.reconstruction;
    bool five_step = soft_reset ? a->five_step : false;
    uint8_t dmc_addr_reg = soft_reset ? a->dmc.sample_addr_reg : 0;
    uint8_t dmc_len_reg = soft_reset ? a->dmc.sample_len_reg : 0;
    uint16_t dmc_addr = soft_reset ? a->dmc.sample_addr : 0xC000;
    uint16_t dmc_len = soft_reset ? a->dmc.sample_len : 1;
    LengthCounter triangle_length = a->tri.lc;

    memset(a, 0, sizeof(*a));
    a->reconstruction = reconstruction;
    a->right_output.reconstruction = right_reconstruction;
    atomic_init(&a->ring_w, 0);
    atomic_init(&a->ring_r, 0);
    a->five_step = five_step;
    a->frame_next_five_step = five_step;
    a->frame_reset_pending = true;
    a->frame_reset_delay = 3;
    a->cpu_cycle_odd = false;
    if (soft_reset) {
        a->tri.lc = triangle_length;
        a->tri.enabled = false;
    }
    a->noise.lfsr = 1; // cannot be 0
    a->noise.period = noise_period_table()[0];
    a->noise.timer = a->noise.period - 1;
    a->dmc.enabled = false;
    a->dmc.sample_buffer_empty = true;
    a->dmc.silence = true;
    a->dmc.bits_remaining = 8;
    a->dmc.sample_addr_reg = dmc_addr_reg;
    a->dmc.sample_len_reg = dmc_len_reg;
    a->dmc.sample_addr = dmc_addr;
    a->dmc.sample_len = dmc_len;
    a->dmc.timer_reload = dmc_period_table()[0] - 1;
    // The DMC divider clocks on the CPU's get phase. Its even periods must
    // keep output clocks on odd completed CPU cycles across initialization.
    a->dmc.timer = a->dmc.timer_reload - ((cpu_get_bus_cycle() & 1) ? 0 : 1);
    a->sample_rate = sample_rate;
    a->cycles_per_sample = nes_timing()->cpu_hz / sample_rate;
    if (five_step) a->regs[0x17] = 0x80;
    a->regs[0x12] = dmc_addr_reg;
    a->regs[0x13] = dmc_len_reg;
    apu_init_filter_coeffs(a);
    apu_reconstruction_reset(a);
}

void apu_power_on(APU *a) {
    apu_reset_state(a, false);
}

void apu_soft_reset(APU *a) {
    apu_reset_state(a, true);
}

void apu_reset(APU *a) {
    apu_power_on(a);
}

void apu_audio_init(int sample_rate) {
    apu_audio_init_state(&apu, sample_rate);
}

void apu_audio_init_state(APU *state, int sample_rate) {
    if (!state || sample_rate <= 0) return;
    state->sample_rate = (double)sample_rate;
    state->cycles_per_sample = nes_timing()->cpu_hz / state->sample_rate;
    state->sample_accum = 0.0;
    state->last_read_sample = 0.0f;
    state->last_read_side = 0.0f;
    atomic_store_explicit(&state->ring_w, 0, memory_order_relaxed);
    atomic_store_explicit(&state->ring_r, 0, memory_order_relaxed);
    apu_init_filter_coeffs(state);
    apu_reconstruction_reset(state);
}

bool apu_set_cpu_revision(ApuCpuRevision revision) {
    if (revision != APU_CPU_REVISION_EARLY_2A03 &&
        revision != APU_CPU_REVISION_LATE_2A03) {
        return false;
    }
    if (!nes_execution_allows_host_configuration()) return false;
    cpu_revision = revision;
    return true;
}

ApuCpuRevision apu_get_cpu_revision(void) {
    return cpu_revision;
}

void apu_set_disable_noise_mode(bool enabled) {
    if (!nes_execution_allows_host_configuration()) return;
    disable_noise_mode = enabled;
}

bool apu_noise_mode_disabled(void) {
    return disable_noise_mode;
}

void apu_set_swap_duty_cycles(bool enabled) {
    if (!nes_execution_allows_host_configuration()) return;
    swap_duty_cycles = enabled;
}

bool apu_swap_duty_cycles_enabled(void) {
    return swap_duty_cycles;
}

// APU register access.
static inline void apu_write_4017(APU *a, uint8_t v) {
    a->regs[0x17] = v;
    a->frame_next_five_step = (v & 0x80) != 0;
    a->irq_inhibit = (v & 0x40) != 0;
    if (a->irq_inhibit) {
        a->frame_irq = false;
        a->frame_irq_source = false;
        a->frame_irq_clear_delay = 0;
    }

    // The mode and optional quarter/half clock take effect with the delayed reset.
    a->frame_reset_delay = (cpu_get_bus_cycle() & 1ULL) ? 4 : 3;
    a->frame_reset_pending = true;
}
static inline uint8_t apu_read_4015(APU *a) {
    uint8_t s = 0;
    if (a->pulse1.lc.length) s |= 0x01;
    if (a->pulse2.lc.length) s |= 0x02;
    if (a->tri.lc.length)    s |= 0x04;
    if (a->noise.lc.length)  s |= 0x08;
    if (a->dmc.bytes_remaining > 0) s |= 0x10;
    if (a->frame_irq)        s |= 0x40;
    if (a->dmc.irq_flag)     s |= 0x80;
    a->frame_irq_source = false;
    if (a->frame_irq && !a->frame_irq_clear_delay)
        a->frame_irq_clear_delay = (cpu_get_bus_cycle() & 1u) ? 2 : 1;
    return s;
}

uint8_t apu_debug_peek_status(void) {
    APU *a = apu_active_state();
    uint8_t s = 0;
    if (a->pulse1.lc.length) s |= 0x01;
    if (a->pulse2.lc.length) s |= 0x02;
    if (a->tri.lc.length) s |= 0x04;
    if (a->noise.lc.length) s |= 0x08;
    if (a->dmc.bytes_remaining > 0) s |= 0x10;
    if (a->frame_irq) s |= 0x40;
    if (a->dmc.irq_flag) s |= 0x80;
    return s;
}

static void dmc_restart_sample(DMC* d) {
    d->current_addr = d->sample_addr;
    d->bytes_remaining = d->sample_len;
}

static void dmc_request_buffer(APU* a) {
    DMC* d = &a->dmc;
    if (d->sample_buffer_empty && d->bytes_remaining && !d->start_delay && !d->dma_pending) {
        d->dma_pending = true;
        d->dma_halt_started = false;
        d->dma_abort_requested = false;
    }
}

bool apu_dmc_dma_pending(const APU *a) {
    return a->dmc.dma_pending;
}

uint16_t apu_dmc_dma_address(const APU *a) {
    return a->dmc.current_addr;
}

void apu_dmc_dma_complete(APU *a, uint8_t value) {
    DMC *d = &a->dmc;
    d->dma_pending = false;
    d->dma_halt_started = false;
    d->dma_abort_requested = false;
    if (!d->bytes_remaining) return;
    d->sample_buffer = value;
    d->sample_buffer_empty = false;

    d->current_addr++;
    if (d->current_addr == 0) d->current_addr = 0x8000;

    d->bytes_remaining--;
    if (d->bytes_remaining == 0) {
        if (d->loop) {
            dmc_restart_sample(d);
        } else if (d->irq_enable) {
            d->irq_flag = true;
        }
    }

    // Later CPU revisions can start a new transfer immediately when DMA
    // completion coincides with the output unit reloading its bit counter.
    if (cpu_revision == APU_CPU_REVISION_LATE_2A03 &&
        d->bits_remaining == 8 && d->timer == d->timer_reload) {
        d->shift_reg = d->sample_buffer;
        d->silence = false;
        d->sample_buffer_empty = true;
        if (d->sample_len == 1) dmc_restart_sample(d);
        dmc_request_buffer(a);
    // A one-byte non-looping sample fetched immediately before the output
    // shifter reloads can schedule a reload DMA that is stopped one CPU cycle
    // after it begins. This is the early-CPU one-cycle DMA behavior.
    } else if (d->sample_len == 1 && !d->loop && d->bits_remaining == 1 && d->timer < 2) {
        d->shift_reg = d->sample_buffer;
        d->sample_buffer_empty = false;
        dmc_restart_sample(d);
        d->disable_delay = 3;
    }
}

static void dmc_clock_output(APU* a) {
    DMC* d = &a->dmc;

    if (!d->silence) {
        if (d->shift_reg & 1) {
            if (d->output_level <= 125) d->output_level += 2;
        } else {
            if (d->output_level >= 2) d->output_level -= 2;
        }
    }

    d->shift_reg >>= 1;
    if (--d->bits_remaining == 0) {
        d->bits_remaining = 8;
        d->silence = d->sample_buffer_empty;
        if (!d->sample_buffer_empty) {
            d->shift_reg = d->sample_buffer;
            d->sample_buffer_empty = true;
            dmc_request_buffer(a);
        }
    }
}

static void pulse_write(Pulse *p, uint16_t reg, uint8_t v) {
    switch (reg & 3) {
    case 0: // $4000/$4004
        p->env.loop_envelope = (v & 0x20) != 0;
        length_set_halt(&p->lc, p->env.loop_envelope);
        p->env.constant_volume = (v & 0x10) != 0;
        p->env.volume = v & 0x0F;
        p->duty = (v >> 6) & 3;
        if (swap_duty_cycles) {
            p->duty = (uint8_t)(((p->duty & 0x02u) >> 1) | ((p->duty & 0x01u) << 1));
        }
        break;
    case 1: // sweep
        p->sweep.enabled = (v & 0x80) != 0;
        p->sweep.period = ((v >> 4) & 7) + 1;
        p->sweep.negate = (v & 0x08) != 0;
        p->sweep.shift = v & 7;
        p->sweep.reload = true;
        break;
    case 2: // timer low
        p->timer_reload = (p->timer_reload & 0x700) | v;
        break;
    case 3: // timer high + length load
        p->timer_reload = (p->timer_reload & 0xFF) | ((v & 7) << 8);
        p->duty_step = 0;
        length_load(&p->lc, v, p->enabled);
        p->env.start_flag = true;
        break;
    }
    // Pulse register writes refresh the DAC immediately. $4015 disable only
    // clears the length counter; the latched output changes on the next edge.
    p->output_level = pulse_current_output(p);
}
static void triangle_write(Triangle* t, uint16_t reg, uint8_t v){
    switch (reg & 3) {
        case 0:
            t->control = (v & 0x80) != 0;
            length_set_halt(&t->lc, t->control);
            t->linear_reload_val = v & 0x7F;
            break;
        case 2:
            t->timer_reload = (t->timer_reload & 0x700) | v;
            break;
        case 3:
            t->timer_reload = (t->timer_reload & 0xFF) | ((v & 7) << 8);
            length_load(&t->lc, v, t->enabled);
            t->linear_reload = true;
            break;
    }
}
static void noise_write(Noise* n, uint16_t reg, uint8_t v){
    switch (reg & 3) {
        case 0:
            n->env.loop_envelope = (v & 0x20) != 0;
            length_set_halt(&n->lc, n->env.loop_envelope);
            n->env.constant_volume = (v & 0x10) != 0;
            n->env.volume = v & 0x0F;
            break;
        case 2:
            n->mode = (v & 0x80) != 0;
            n->period_idx = v & 0x0F;
            n->period = noise_period_table()[n->period_idx];
            break;
        case 3:
            length_load(&n->lc, v, n->enabled);
            n->env.start_flag = true;
            break;
    }
}

void apu_write(uint16_t addr, uint8_t v){
    if (addr < 0x4000 || addr > 0x4017) return;
    apu.regs[addr - 0x4000] = v;

    if (addr <= 0x4003)        pulse_write(&apu.pulse1, addr, v);
    else if (addr <= 0x4007)   pulse_write(&apu.pulse2, addr, v);
    else if (addr <= 0x400B)   triangle_write(&apu.tri, addr, v);
    else if (addr <= 0x400F)   noise_write(&apu.noise, addr, v);
    else if (addr == 0x4010) {
        apu.dmc.irq_enable = (v & 0x80) != 0;
        apu.dmc.loop = (v & 0x40) != 0;
        apu.dmc.rate_index = v & 0x0F;
        apu.dmc.timer_reload = dmc_period_table()[apu.dmc.rate_index] - 1;
        if (!apu.dmc.irq_enable) apu.dmc.irq_flag = false;
    }
    else if (addr == 0x4011) {
        apu.dmc.output_level = v & 0x7F;
    }
    else if (addr == 0x4012) {
        apu.dmc.sample_addr_reg = v;
        apu.dmc.sample_addr = (uint16_t)(0xC000u + ((uint16_t)v << 6));
    }
    else if (addr == 0x4013) {
        apu.dmc.sample_len_reg = v;
        apu.dmc.sample_len = (uint16_t)(((uint16_t)v << 4) + 1u);
    }
    else if (addr == 0x4015) {
        apu.pulse1.enabled = (v & 0x01) != 0;
        apu.pulse2.enabled = (v & 0x02) != 0;
        apu.tri.enabled = (v & 0x04) != 0;
        apu.noise.enabled = (v & 0x08) != 0;
        LengthCounter *lengths[4] = {&apu.pulse1.lc, &apu.pulse2.lc, &apu.tri.lc, &apu.noise.lc};
        for (int channel = 0; channel < 4; ++channel) {
            if (!(v & (1 << channel))) {
                lengths[channel]->length = 0;
                lengths[channel]->reload_value = 0;
            }
        }
        apu.dmc.enabled    = (v & 0x10) != 0;
        apu.dmc.irq_flag = false;
        if (!apu.dmc.enabled) {
            if (!apu.dmc.disable_delay)
                apu.dmc.disable_delay = (cpu_get_bus_cycle() & 1) ? 3 : 2;
        } else if (apu.dmc.bytes_remaining == 0) {
            dmc_restart_sample(&apu.dmc);
            apu.dmc.start_delay = (cpu_get_bus_cycle() & 1) ? 3 : 2;
        }
    } else if (addr == 0x4017) {
        apu_write_4017(&apu, v);
    }
    apu_audio_refresh(&apu);
}
uint8_t apu_read(uint16_t addr){
    if (addr == 0x4015) return apu_read_4015(&apu);
    if (addr >= 0x4000 && addr <= 0x4017) return apu.regs[addr - 0x4000];
    return 0x00;
}

// Per-cycle APU clocks.
static inline void clock_pulse(Pulse* p){
    if (p->timer == 0) {
        p->timer = p->timer_reload;
        p->duty_step = (p->duty_step + 1) & 7;
        p->output_level = pulse_current_output(p);
    } else {
        p->timer--;
    }
}
static inline void clock_triangle(Triangle* t){
    if (t->timer == 0) {
        t->timer = t->timer_reload;
        if (t->lc.length && t->linear_counter) {
            t->step = (t->step + 1) % 32;
            t->output_level = TRI_SEQ[t->step];
        }
    } else {
        t->timer--;
    }
}

static inline void clock_noise(Noise *n) {
    if (n->period == 0) {
        return;
    }

    if (n->timer == 0) {
        n->timer = n->period - 1;
        // Feedback uses bit 0 and bit 1, or bit 6 in short mode.
        bool mode = disable_noise_mode ? false : n->mode;
        uint16_t bit0 = n->lfsr & 1;
        uint16_t bitX = (n->lfsr >> (mode ? 6 : 1)) & 1;
        uint16_t fb = bit0 ^ bitX;
        n->lfsr = (n->lfsr >> 1) | (fb << 14);
        n->output_level = noise_current_output(n);
    } else {
        n->timer--;
    }
}
// DAC-ish sample (0..1 per channel)
static inline float pulse_out(const Pulse* p){
    return (float)p->output_level;
}
static inline float triangle_out(const Triangle* t){
    return (float)t->output_level;
}
static inline float noise_out(const Noise* n){
    return (float)n->output_level;
}

static inline float dmc_out(const DMC* d){
    return (float)d->output_level; // 0..127
}

uint8_t apu_read_test_output(uint16_t addr) {
    switch (addr) {
        case 0x4018:
            return (uint8_t)((uint8_t)pulse_out(&apu.pulse1)
                           | ((uint8_t)pulse_out(&apu.pulse2) << 4));
        case 0x4019:
            return (uint8_t)((uint8_t)triangle_out(&apu.tri)
                           | ((uint8_t)noise_out(&apu.noise) << 4));
        case 0x401A:
            return (uint8_t)dmc_out(&apu.dmc);
        default:
            return 0;
    }
}

// Nonlinear mixer (NESdev): pulse & TND
static inline float mix_sample(float p1, float p2, float tri, float noi, float dmc){
    float pulse = (p1 + p2);
    float tnd   = tri/8227.0f + noi/12241.0f + dmc/22638.0f;
    float pulse_v = (pulse <= 0.0f) ? 0.0f : 95.88f / (8128.0f / pulse + 100.0f);
    float tnd_v   = (tnd   <= 0.0f) ? 0.0f : 159.79f / (1.0f / tnd + 100.0f);
    float s = pulse_v + tnd_v;
    // soft clip
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    return s;
}

static int apu_quantize_mix(float sample) {
    double scaled = (double)sample * APU_RECONSTRUCTION_SCALE;
    if (scaled > 32767.0) scaled = 32767.0;
    if (scaled < -32768.0) scaled = -32768.0;
    return (int)lrint(scaled);
}

static float apu_side_mix(const APU *a, const float *expansion, bool right) {
    const float *gain = nes_audio_mix_side_gains(right);
    float sample = mix_sample(pulse_out(&a->pulse1) * gain[NES_AUDIO_PULSE1],
                               pulse_out(&a->pulse2) * gain[NES_AUDIO_PULSE2],
                               triangle_out(&a->tri) * gain[NES_AUDIO_TRIANGLE],
                               noise_out(&a->noise) * gain[NES_AUDIO_NOISE],
                               dmc_out(&a->dmc) * gain[NES_AUDIO_DMC]);
    for (unsigned channel = NES_AUDIO_FDS; channel < NES_AUDIO_CHANNEL_COUNT; ++channel)
        sample += expansion[channel] * gain[channel];
    return sample * cart_audio_gain();
}

void apu_audio_refresh(APU *a) {
    if (!a) return;
    float left, right;
    if (nes_audio_mix_channels_default()) {
        left = mix_sample(pulse_out(&a->pulse1), pulse_out(&a->pulse2),
                           triangle_out(&a->tri), noise_out(&a->noise), dmc_out(&a->dmc));
        left += cart_expansion_audio();
        left *= cart_audio_gain();
        right = left;
    } else {
        float expansion[NES_AUDIO_CHANNEL_COUNT];
        cart_expansion_audio_channels(expansion);
        left = apu_side_mix(a, expansion, false);
        right = apu_side_mix(a, expansion, true);
    }
    int level = apu_quantize_mix(left);
    int delta = level - a->reconstructed_level;
    if (delta && a->reconstruction) {
        blip_add_delta(a->reconstruction, 0, delta);
        a->audio_transition_count++;
    }
    a->reconstructed_level = level;
    level = apu_quantize_mix(right);
    delta = level - a->right_output.level;
    if (delta && a->right_output.reconstruction)
        blip_add_delta(a->right_output.reconstruction, 0, delta);
    a->right_output.level = level;
}

static void apu_output_reconstructed_samples(APU *a) {
    if (!a->reconstruction) return;
    blip_end_frame(a->reconstruction, 1);
    if (a->right_output.reconstruction) blip_end_frame(a->right_output.reconstruction, 1);
    while (blip_samples_avail(a->reconstruction) > 0) {
        short reconstructed = 0, right_reconstructed = 0;
        if (blip_read_samples(a->reconstruction, &reconstructed, 1, 0) != 1) break;
        if (!a->right_output.reconstruction
            || blip_read_samples(a->right_output.reconstruction, &right_reconstructed, 1, 0) != 1)
            right_reconstructed = reconstructed;
        float left = apu_post_filter(a, (float)reconstructed / (float)APU_RECONSTRUCTION_SCALE);
        float right = apu_post_filter_right(a, (float)right_reconstructed / (float)APU_RECONSTRUCTION_SCALE);
        float epsm_left = 0.0f, epsm_right = 0.0f;
        if (a == main_apu) epsm_sample_stereo(&epsm_left, &epsm_right);
        left += epsm_left * nes_audio_mix_side_gains(false)[NES_AUDIO_EPSM];
        right += epsm_right * nes_audio_mix_side_gains(true)[NES_AUDIO_EPSM];
        nes_audio_process(a == main_apu ? 0u : 1u, a->sample_rate, &left, &right);
        if (left > 1.0f) left = 1.0f;
        if (left < -1.0f) left = -1.0f;
        if (right > 1.0f) right = 1.0f;
        if (right < -1.0f) right = -1.0f;
        nes_audio_observe(a == main_apu ? 0u : 1u, a->sample_rate, left, right);
        nes_audio_mix_master(&left, &right);
        float middle = (left + right) * 0.5f;
        float side = (left - right) * 0.5f;
        a->last_output_sample = middle;
        rb_push(a, middle, side);
    }
}

void apu_step(APU *a, int cpu_cycles){
    const uint32_t (*frame_steps)[6] = frame_step_table();
    for (int i=0; i<cpu_cycles; ++i) {
        if (a->frame_irq_clear_delay && --a->frame_irq_clear_delay == 0)
            a->frame_irq = false;
        if (a->dmc.disable_delay && --a->dmc.disable_delay == 0) {
            a->dmc.bytes_remaining = 0;
            if (a->dmc.dma_pending) {
                if (a->dmc.dma_halt_started) {
                    a->dmc.dma_abort_requested = true;
                } else {
                    a->dmc.dma_pending = false;
                    a->dmc.dma_abort_requested = false;
                }
            }
        }
        if (a->dmc.start_delay && --a->dmc.start_delay == 0)
            dmc_request_buffer(a);
        // Half-frame events also clock the quarter-frame units.
        a->cycle_in_seq++;
        unsigned frame_mode = a->five_step ? 1u : 0u;
        bool half_clock = a->cycle_in_seq == frame_steps[frame_mode][1] ||
                          a->cycle_in_seq == frame_steps[frame_mode][4];
        bool quarter_clock = half_clock || a->cycle_in_seq == frame_steps[frame_mode][0] ||
                             a->cycle_in_seq == frame_steps[frame_mode][2];
        if (quarter_clock && !a->frame_clock_block) {
            apu_clock_quarter_frame(a);
            if (half_clock) apu_clock_half_frame(a);
            a->frame_clock_block = 2;
        }
        if (!a->five_step && a->cycle_in_seq >= frame_steps[0][3]) {
            a->frame_irq = true;
            a->frame_irq_clear_delay = 0;
            if (!a->irq_inhibit) a->frame_irq_source = true;
            if (a->irq_inhibit && a->cycle_in_seq >= frame_steps[0][5]) {
                a->frame_irq = false;
                a->frame_irq_source = false;
            }
        }
        if (a->cycle_in_seq >= frame_steps[frame_mode][5])
            a->cycle_in_seq = 0;

        if (a->frame_reset_pending) {
            if (a->frame_reset_delay > 0) a->frame_reset_delay--;
            if (a->frame_reset_delay == 0) {
                a->five_step = a->frame_next_five_step;
                a->cycle_in_seq = 0;
                a->frame_reset_pending = false;
                if (a->five_step && !a->frame_clock_block) {
                    apu_clock_quarter_frame(a);
                    apu_clock_half_frame(a);
                    a->frame_clock_block = 2;
                }
            }
        }
        if (a->frame_clock_block) a->frame_clock_block--;

        length_apply_write(&a->pulse1.lc);
        length_apply_write(&a->pulse2.lc);
        length_apply_write(&a->tri.lc);
        length_apply_write(&a->noise.lc);

        // Pulse timers divide the CPU clock by two. Other periods are CPU cycles.
        if (!a->cpu_cycle_odd) {
            clock_pulse(&a->pulse1);
            clock_pulse(&a->pulse2);
        }
        clock_noise(&a->noise);
        clock_triangle(&a->tri);
        if (a->dmc.timer == 0) {
            a->dmc.timer = a->dmc.timer_reload;
            dmc_clock_output(a);
        } else {
            a->dmc.timer--;
        }

        // Record cycle-timed output changes, then advance the reconstruction by one CPU clock.
        a->sample_accum += 1.0;
        while (a->sample_accum >= a->cycles_per_sample) a->sample_accum -= a->cycles_per_sample;
        apu_audio_refresh(a);
        apu_output_reconstructed_samples(a);

        a->cpu_cycle_odd = !a->cpu_cycle_odd;
    }
}

// SDL audio callback.
void apu_audio_pull(APU *state, float *samples, int count) {
    if (!state || !samples || count <= 0) return;
    int got = rb_pull(state, samples, count);
    if (got) state->last_read_sample = samples[got - 1];
    for (int i = got; i < count; ++i) samples[i] = state->last_read_sample;
}

void apu_sdl_audio_callback(void *userdata, uint8_t *stream, int len){
    (void)userdata;
    float *out = (float*)stream;
    int frames = len / sizeof(float);
    apu_audio_pull(main_apu, out, frames);
}

void apu_audio_pull_stereo(APU *state, float *samples, int frames) {
    if (!state || !samples || frames <= 0) return;
    uint32_t read_index = atomic_load_explicit(&state->ring_r, memory_order_relaxed);
    int frame = 0;
    while (frame < frames
           && read_index != atomic_load_explicit(&state->ring_w, memory_order_acquire)) {
        float middle = state->ring[read_index];
        float side = state->ring_side[read_index];
        samples[frame * 2] = middle + side;
        samples[frame * 2 + 1] = middle - side;
        state->last_read_sample = middle;
        state->last_read_side = side;
        ++frame;
        read_index = rb_next(read_index);
        atomic_store_explicit(&state->ring_r, read_index, memory_order_release);
    }
    for (; frame < frames; ++frame) {
        samples[frame * 2] = state->last_read_sample + state->last_read_side;
        samples[frame * 2 + 1] = state->last_read_sample - state->last_read_side;
    }
}

void apu_sdl_stereo_callback(void *userdata, uint8_t *stream, int len) {
    (void)userdata;
    if (!stream || len <= 0) return;
    int frames = len / (int)(2 * sizeof(float));
    apu_audio_pull_stereo(main_apu, (float *)stream, frames);
    size_t bytes = (size_t)frames * 2 * sizeof(float);
    if (bytes < (size_t)len) memset(stream + bytes, 0, (size_t)len - bytes);
}

#include "apu_state_impl.h"
