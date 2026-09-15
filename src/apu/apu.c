/*
 * apu.c - Audio Processing Unit (APU) emulation
 * 
 * Author: @frankischilling
 * 
 * This file implements the NES APU (Audio Processing Unit) which handles sound generation.
 * It emulates all five sound channels: two pulse wave channels, one triangle wave, one noise
 * channel, and DMC (Delta Modulation Channel). Includes envelope generators, sweep units,
 * length counters, and frame sequencer. Features accurate timing and nonlinear mixing.
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
#include <string.h>
#include <math.h>
#include <stdio.h>

extern uint64_t cpu_total_cycles;

APU apu;

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

// Noise periods (NTSC) — index 0..15 -> timer reload values
static const uint16_t NOISE_PERIOD[16] = {
    4,8,16,32,64,96,128,160,202,254,380,508,762,1016,2034,4068
};

// DMC rates (NTSC CPU cycles per output bit)
static const uint16_t DMC_PERIOD[16] = {
    428, 380, 340, 320, 286, 254, 226, 214,
    190, 160, 142, 128, 106,  84,  72,  54
};

static void sweep_clock(Pulse* p, bool is_ch2);
static void tri_linear_clock(Triangle* t);
static void dmc_restart_sample(DMC* d);
static void dmc_request_buffer(APU* a);
static void dmc_clock_output(APU* a);

// ---------------- Ring buffer ----------------
static inline uint32_t rb_next(uint32_t v){ return (v+1) & (APU_RING_CAP-1); }
static inline bool rb_push(APU* a, float s){
    uint32_t w = a->ring_w, n = rb_next(w);
    if (n == a->ring_r) return false; // full, drop
    a->ring[w] = s; a->ring_w = n; return true;
}
static inline int rb_pull(APU* a, float* out, int n){
    int got=0; while (got<n && a->ring_r != a->ring_w) {
        out[got++] = a->ring[a->ring_r];
        a->ring_r = rb_next(a->ring_r);
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
}

static inline float apu_post_filter(APU *a, float s) {
    s = one_pole_hp(s, a->hp90_alpha, &a->hp90_prev_in, &a->hp90_prev_out);
    s = one_pole_hp(s, a->hp440_alpha, &a->hp440_prev_in, &a->hp440_prev_out);
    s = one_pole_lp(s, a->lp14k_alpha, &a->lp14k_prev_out);
    return s;
}

// ---------------- Envelope ----------------
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

// ---------------- Length counter ----------------
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

// ---------------- Sweep (pulse) ----------------
static inline uint16_t sweep_target(uint16_t t, const Sweep* s, bool ch2){
    uint16_t change = t >> s->shift;
    if (s->negate) {
        // Pulse 1 uses one's complement subtraction; pulse 2 uses two's complement.
        return ch2 ? (t - change) : (t - change - 1);
    } else {
        return t + change;
    }
}
static void sweep_clock(Pulse* p, bool is_ch2){
    if (p->sweep.divider == 0 && p->sweep.enabled &&
        p->sweep.shift && p->timer_reload >= 8) {
        uint16_t tgt = sweep_target(p->timer_reload, &p->sweep, is_ch2);
        if (tgt < 0x800) p->timer_reload = tgt;
    }
    if (p->sweep.divider == 0 || p->sweep.reload) {
        p->sweep.divider = p->sweep.period;
        p->sweep.reload = false;
    } else {
        p->sweep.divider--;
    }
}

// ---------------- Triangle linear counter ----------------
static void tri_linear_clock(Triangle* t){
    if (t->linear_reload) {
        t->linear_counter = t->linear_reload_val;
    } else if (t->linear_counter > 0) {
        t->linear_counter--;
    }
    if (!t->control) t->linear_reload = false;
}

// ---------------- Reset & init ----------------
void apu_reset(APU *a) {
    memset(a, 0, sizeof(*a));
    a->noise.lfsr = 1; // cannot be 0
    a->noise.period = NOISE_PERIOD[0];
    a->noise.timer = a->noise.period - 1;
    a->dmc.enabled = false;
    a->dmc.sample_buffer_empty = true;
    a->dmc.silence = true;
    a->dmc.bits_remaining = 8;
    a->dmc.sample_addr = 0xC000;
    a->dmc.sample_len = 1;
    a->dmc.timer_reload = DMC_PERIOD[0] - 1;
    // The DMC divider clocks on the CPU's get phase. Its even periods must
    // keep output clocks on odd completed CPU cycles across initialization.
    a->dmc.timer = a->dmc.timer_reload - ((cpu_total_cycles & 1) ? 0 : 1);
    a->cycles_per_sample = 1789773.0 / 44100.0;
    a->sample_rate = 44100.0;
    apu_init_filter_coeffs(a);
}
void apu_audio_init(int sample_rate) {
    apu.sample_rate = (double)sample_rate;
    apu.cycles_per_sample = 1789773.0 / apu.sample_rate;
    apu.sample_accum = 0.0;
    apu.ring_w = apu.ring_r = 0;
    apu_init_filter_coeffs(&apu);
}

// ---------------- Reads/Writes ----------------
static inline void apu_write_4017(APU *a, uint8_t v) {
    a->regs[0x17] = v;
    a->frame_next_five_step = (v & 0x80) != 0;
    a->irq_inhibit = (v & 0x40) != 0;
    if (a->irq_inhibit) a->frame_irq = false;

    // The mode and optional quarter/half clock take effect with the delayed reset.
    a->frame_reset_delay = (cpu_total_cycles & 1ULL) ? 4 : 3;
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
    a->frame_irq = false;
    return s;
}

static void dmc_restart_sample(DMC* d) {
    d->current_addr = d->sample_addr;
    d->bytes_remaining = d->sample_len;
}

static void dmc_request_buffer(APU* a) {
    DMC* d = &a->dmc;
    if (d->sample_buffer_empty && d->bytes_remaining && !d->start_delay)
        d->dma_pending = true;
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
static void pulse_write(Pulse* p, uint16_t reg, uint8_t v){
    switch (reg & 3) {
        case 0: // $4000/$4004
            p->env.loop_envelope = (v & 0x20) != 0;
            length_set_halt(&p->lc, p->env.loop_envelope);
            p->env.constant_volume = (v & 0x10) != 0;
            p->env.volume = v & 0x0F;
            p->duty = (v >> 6) & 3;
            break;
        case 1: // sweep
            p->sweep.enabled = (v & 0x80) != 0;
            p->sweep.period  = (v >> 4) & 7;
            p->sweep.negate  = (v & 0x08) != 0;
            p->sweep.shift   = v & 7;
            p->sweep.reload  = true;
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
            n->period = NOISE_PERIOD[n->period_idx];
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
        apu.dmc.timer_reload = DMC_PERIOD[apu.dmc.rate_index] - 1;
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
                apu.dmc.disable_delay = (cpu_total_cycles & 1) ? 3 : 2;
        } else if (apu.dmc.bytes_remaining == 0) {
            dmc_restart_sample(&apu.dmc);
            apu.dmc.start_delay = (cpu_total_cycles & 1) ? 3 : 2;
        }
    } else if (addr == 0x4017) {
        apu_write_4017(&apu, v);
    }
}
uint8_t apu_read(uint16_t addr){
    if (addr == 0x4015) return apu_read_4015(&apu);
    if (addr >= 0x4000 && addr <= 0x4017) return apu.regs[addr - 0x4000];
    return 0x00;
}

// ---------------- Per-cycle ticking ----------------
static inline void clock_pulse(Pulse* p){
    if (p->timer == 0) {
        p->timer = p->timer_reload;
        p->duty_step = (p->duty_step + 1) & 7;
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

static inline void clock_noise(Noise* n){
    if (n->period == 0) return;
    
    if (n->timer == 0) {
        n->timer = n->period - 1;
        // Feedback uses bit 0 and bit 1, or bit 6 in short mode.
        uint16_t bit0 = n->lfsr & 1;
        uint16_t bitX = (n->lfsr >> (n->mode ? 6 : 1)) & 1;
        uint16_t fb = bit0 ^ bitX;
        n->lfsr = (n->lfsr >> 1) | (fb << 14);
    } else {
        n->timer--;
    }
}
// DAC-ish sample (0..1 per channel)
static inline float pulse_out(const Pulse* p){
    if (!p->enabled || p->lc.length == 0) return 0.0f;
    if (p->timer_reload < 8 || p->timer_reload > 0x7FF) return 0.0f;
    // The adder can mute the channel even when sweep updates are disabled.
    if (!p->sweep.negate && sweep_target(p->timer_reload, &p->sweep, false) > 0x7FF)
        return 0.0f;
    uint8_t gate = DUTY_SEQ[p->duty][p->duty_step];
    if (!gate) return 0.0f;
    return (float)env_output(&p->env); // 0..15 raw DAC domain for nonlinear mixer
}
static inline float triangle_out(const Triangle* t){
    return (float)t->output_level;
}
static inline float noise_out(const Noise* n){
    if (!n->enabled || n->lc.length == 0) return 0.0f;
    // if lfsr bit0 is 1 -> output 0, else envelope
    if (n->lfsr & 1) return 0.0f;
    return (float)env_output(&n->env); // 0..15
}

static inline float dmc_out(const DMC* d){
    return (float)d->output_level; // 0..127
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

void apu_step(APU *a, int cpu_cycles){
    for (int i=0; i<cpu_cycles; ++i) {
        if (a->dmc.disable_delay && --a->dmc.disable_delay == 0) {
            a->dmc.bytes_remaining = 0;
            a->dmc.dma_pending = false;
        }
        if (a->dmc.start_delay && --a->dmc.start_delay == 0)
            dmc_request_buffer(a);
        // Half-frame events also clock the quarter-frame units.
        a->cycle_in_seq++;
        uint32_t terminal_clock = a->five_step ? 37281u : 29829u;
        bool half_clock = a->cycle_in_seq == 14913u || a->cycle_in_seq == terminal_clock;
        bool quarter_clock = half_clock || a->cycle_in_seq == 7457u || a->cycle_in_seq == 22371u;
        if (quarter_clock && !a->frame_clock_block) {
            apu_clock_quarter_frame(a);
            if (half_clock) apu_clock_half_frame(a);
            a->frame_clock_block = 2;
        }
        if (!a->five_step && !a->irq_inhibit && a->cycle_in_seq >= 29828u)
            a->frame_irq = true;
        if (a->cycle_in_seq >= (a->five_step ? APU_5STEP_PERIOD : APU_4STEP_PERIOD))
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

        // resample
        a->sample_accum += 1.0;
        if (a->sample_accum >= a->cycles_per_sample) {
            a->sample_accum -= a->cycles_per_sample;

            float p1 = pulse_out(&a->pulse1);
            float p2 = pulse_out(&a->pulse2);
            float tr = triangle_out(&a->tri);
            float nz = noise_out(&a->noise);
            float dm = dmc_out(&a->dmc);
            float s  = mix_sample(p1, p2, tr, nz, dm);
            s = apu_post_filter(a, s);

            if (s > 1.0f) s = 1.0f;
            if (s < -1.0f) s = -1.0f;
            a->last_output_sample = s;

            rb_push(a, s);
        }

        a->cpu_cycle_odd = !a->cpu_cycle_odd;
    }
}

// ---------------- SDL callback ----------------
void apu_sdl_audio_callback(void *userdata, uint8_t *stream, int len){
    (void)userdata;
    float *out = (float*)stream;
    int frames = len / sizeof(float);
    int got = rb_pull(&apu, out, frames);
    for (int i=got; i<frames; ++i) out[i] = apu.last_output_sample;
}
