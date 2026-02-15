/*
 * apu.c - Audio Processing Unit (APU) emulation
 * 
 * Author: @frankischilling
 * 
 * This file implements the NES APU (Audio Processing Unit) which handles sound generation.
 * It emulates all five sound channels: two pulse wave channels, one triangle wave, one noise
 * channel, and DMC (Delta Modulation Channel) stub. Includes envelope generators, sweep units,
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
#include <SDL2/SDL.h>

extern uint64_t cpu_total_cycles;
extern uint8_t read_mem(uint16_t addr);

#define APU_LOG(...) do {} while (0)

static bool apu_irq_probe_active = false;
static uint32_t apu_irq_probe_count = 0;
static uint64_t apu_irq_probe_start = 0;

static inline void apu_log_frame_irq_event(const char *src) {
    (void)src;
}

APU apu;

// Duty sequences
static const uint8_t DUTY_SEQ[4][8] = {
    {0,1,0,0,0,0,0,0}, // 12.5%
    {0,1,1,0,0,0,0,0}, // 25%
    {0,1,1,1,1,0,0,0}, // 50%
    {1,0,0,1,1,1,1,1}, // 25% neg (inverted 12.5 with phase)
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
    190, 160, 142, 128, 106,  85,  72,  54
};

static void sweep_clock(Pulse* p, bool is_ch2);
static void tri_linear_clock(Triangle* t);
static void dmc_restart_sample(DMC* d);
static void dmc_try_fill_buffer(APU* a);
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
        // channel 1: two's complement bug; channel 2: normal
        return ch2 ? (t - change) : (t - change - 1);
    } else {
        return t + change;
    }
}
static void sweep_clock(Pulse* p, bool is_ch2){
    if (p->sweep.reload) {
        if (p->sweep.divider == 0) p->sweep.divider = p->sweep.period;
        else                       p->sweep.divider--;
        p->sweep.reload = false;
        return;
    }
    if (p->sweep.divider == 0) {
        p->sweep.divider = p->sweep.period;
        if (p->sweep.enabled && p->sweep.shift && p->timer_reload >= 8) {
            uint16_t tgt = sweep_target(p->timer_reload, &p->sweep, is_ch2);
            if (tgt < 0x800) p->timer_reload = tgt;
        }
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
    a->dmc.enabled = false;
    a->dmc.sample_buffer_empty = true;
    a->dmc.silence = true;
    a->dmc.bits_remaining = 8;
    a->dmc.timer_reload = DMC_PERIOD[0];
    a->dmc.timer = a->dmc.timer_reload;
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
    bool new_five_step = (v & 0x80) != 0;
    bool write_odd = ((cpu_total_cycles & 1ULL) != 0);
    a->irq_inhibit = (v & 0x40) != 0;
    a->frame_irq = false;
    a->frame_irq_delay = 0;

    // In 5-step mode, writing $4017 clocks quarter+half frame immediately.
    if (new_five_step) {
        apu_clock_quarter_frame(a);
        apu_clock_half_frame(a);
    }

    a->five_step = new_five_step;
    if (!new_five_step) {
        a->mode0_first_frame = true;
    }

    // Frame sequencer reset occurs after 3 or 4 CPU cycles (clock jitter).
    // Even cycle write => 4-cycle delay, odd cycle write => 3-cycle delay.
        a->frame_reset_delay = write_odd ? 3 : 4;
    a->frame_reset_pending = true;
    APU_LOG("write $4017=%02X five=%d inhibit=%d odd=%d reset_delay=%u", v,
            new_five_step ? 1 : 0, a->irq_inhibit ? 1 : 0,
            write_odd ? 1 : 0, a->frame_reset_delay);

    if ((v & 0x80) == 0 && (v & 0x40) == 0) {
        apu_irq_probe_active = true;
        apu_irq_probe_count = 0;
        apu_irq_probe_start = cpu_total_cycles;
        APU_LOG("irq_probe armed (mode0, irq enabled)");
    }
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
    if (s & 0x40) {
        APU_LOG("read $4015 -> %02X (frame_irq consumed)", s);
    }
    a->frame_irq = false;
    return s;
}

static void dmc_restart_sample(DMC* d) {
    d->current_addr = d->sample_addr;
    d->bytes_remaining = d->sample_len;
}

static void dmc_try_fill_buffer(APU* a) {
    DMC* d = &a->dmc;
    if (!d->enabled) return;
    if (!d->sample_buffer_empty) return;
    if (d->bytes_remaining == 0) return;

    d->sample_buffer = read_mem(d->current_addr);
    d->sample_buffer_empty = false;
    a->dmc_dma_stall_cycles += 4;

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

    if (d->bits_remaining == 0) {
        d->bits_remaining = 8;
        if (d->sample_buffer_empty) {
            d->silence = true;
        } else {
            d->silence = false;
            d->shift_reg = d->sample_buffer;
            d->sample_buffer_empty = true;
            dmc_try_fill_buffer(a);
        }
    }

    if (!d->silence) {
        if (d->shift_reg & 1) {
            if (d->output_level <= 125) d->output_level += 2;
        } else {
            if (d->output_level >= 2) d->output_level -= 2;
        }
    }

    d->shift_reg >>= 1;
    if (d->bits_remaining > 0) d->bits_remaining--;

    if (d->bits_remaining == 0 && d->sample_buffer_empty && d->bytes_remaining == 0 && !d->loop && d->irq_enable) {
        d->irq_flag = true;
    }
}
static void pulse_write(Pulse* p, uint16_t reg, uint8_t v){
    switch (reg & 3) {
        case 0: // $4000/$4004
            p->env.loop_envelope = (v & 0x20) != 0;
            p->lc.halt = p->env.loop_envelope;
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
            if (p->enabled) p->lc.length = LENGTH_TABLE[(v >> 3) & 0x1F];
            p->env.start_flag = true;
            break;
    }
}
static void triangle_write(Triangle* t, uint16_t reg, uint8_t v){
    switch (reg & 3) {
        case 0:
            t->control = (v & 0x80) != 0;
            t->lc.halt = t->control;
            t->linear_reload_val = v & 0x7F;
            break;
        case 2:
            t->timer_reload = (t->timer_reload & 0x700) | v;
            break;
        case 3:
            t->timer_reload = (t->timer_reload & 0xFF) | ((v & 7) << 8);
            if (t->enabled) t->lc.length = LENGTH_TABLE[(v >> 3) & 0x1F];
            t->linear_reload = true;
            break;
    }
}
static void noise_write(Noise* n, uint16_t reg, uint8_t v){
    switch (reg & 3) {
        case 0:
            n->env.loop_envelope = (v & 0x20) != 0;
            n->lc.halt = n->env.loop_envelope;
            n->env.constant_volume = (v & 0x10) != 0;
            n->env.volume = v & 0x0F;
            break;
        case 2:
            n->mode = (v & 0x80) != 0;
            n->period_idx = v & 0x0F;
            n->period = NOISE_PERIOD[n->period_idx];
            break;
        case 3:
            if (n->enabled) n->lc.length = LENGTH_TABLE[(v >> 3) & 0x1F];
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
        apu.dmc.timer_reload = DMC_PERIOD[apu.dmc.rate_index];
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
        apu.pulse1.enabled = (v & 0x01) != 0; if (!apu.pulse1.enabled) apu.pulse1.lc.length = 0;
        apu.pulse2.enabled = (v & 0x02) != 0; if (!apu.pulse2.enabled) apu.pulse2.lc.length = 0;
        apu.tri.enabled    = (v & 0x04) != 0; if (!apu.tri.enabled)    apu.tri.lc.length = 0;
        apu.noise.enabled  = (v & 0x08) != 0; if (!apu.noise.enabled)  apu.noise.lc.length = 0;
        apu.dmc.enabled    = (v & 0x10) != 0;
        apu.dmc.irq_flag = false;
        if (!apu.dmc.enabled) {
            apu.dmc.bytes_remaining = 0;
        } else if (apu.dmc.bytes_remaining == 0) {
            dmc_restart_sample(&apu.dmc);
            dmc_try_fill_buffer(&apu);
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
        }
    } else {
        t->timer--;
    }
}

static inline void clock_noise(Noise* n){
    if (n->period == 0) return;
    
    if (n->timer == 0) {
        n->timer = n->period;
        // XOR taps: 1 and 6 when mode=1 (7-bit), else 1 and 14 (15-bit)
        uint16_t bit0 = n->lfsr & 1;
        uint16_t bitX = (n->mode ? ((n->lfsr >> 6) & 1) : ((n->lfsr >> 14) & 1));
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
    uint8_t gate = DUTY_SEQ[p->duty][p->duty_step];
    if (!gate) return 0.0f;
    return (float)env_output(&p->env); // 0..15 raw DAC domain for nonlinear mixer
}
static inline float triangle_out(const Triangle* t){
    if (!t->enabled || t->lc.length == 0 || t->linear_counter == 0) return 0.0f;
    // 32-step triangle: 0..15 descending, 16..31 ascending (or vice versa)
    static const uint8_t TRI_SEQ[32] = {
        15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,
         0, 1, 2, 3, 4, 5,6,7,8,9,10,11,12,13,14,15
    };
    return (float)TRI_SEQ[t->step]; // 0..15
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
        if (a->frame_irq_delay > 0) {
            a->frame_irq_delay--;
            if (a->frame_irq_delay == 0 && !a->irq_inhibit) {
                a->frame_irq = true;
                APU_LOG("frame_irq asserted");
                apu_log_frame_irq_event("delay");
            }
        }

        // Timer domains:
        // - Pulse + Noise run on APU cycles (every other CPU cycle)
        // - Triangle + DMC run on CPU cycles
        if (!a->cpu_cycle_odd) {
            clock_pulse(&a->pulse1);
            clock_pulse(&a->pulse2);
            clock_noise(&a->noise);
        }
        clock_triangle(&a->tri);
        dmc_try_fill_buffer(a);
        if (a->dmc.timer == 0) {
            a->dmc.timer = a->dmc.timer_reload;
            dmc_clock_output(a);
        } else {
            a->dmc.timer--;
        }

        // frame sequencer timing (CPU-cycle domain)
        a->cycle_in_seq++;
        // 4-step: quarter @ 7457, 22371; quarter+half @ 14913, 29829; wrap at 29832.
        // Terminal IRQ edge: first frame after $4017 reset at 29829, then 29830.
        if (!a->five_step) {
            const uint32_t term_irq_cycle = a->mode0_first_frame ? 29829u : 29830u;
            if (a->cycle_in_seq == 7457u || a->cycle_in_seq == 22371u) {
                // quarter: envelopes + triangle linear
                apu_clock_quarter_frame(a);
            } else if (a->cycle_in_seq == 14913u) {
                // quarter+half
                apu_clock_quarter_frame(a);
                apu_clock_half_frame(a);
            } else if (a->cycle_in_seq == term_irq_cycle) {
                // terminal IRQ edge
                if (!a->irq_inhibit) {
                    a->frame_irq = true;
                    a->frame_irq_delay = 0;
                    APU_LOG("frame_irq asserted at terminal edge");
                    apu_log_frame_irq_event("terminal");
                }
            } else if (a->cycle_in_seq == 29829u) {
                // terminal quarter+half clock
                apu_clock_quarter_frame(a);
                apu_clock_half_frame(a);
            }
            if (a->cycle_in_seq >= APU_4STEP_PERIOD) {
                a->cycle_in_seq = 0;
                a->mode0_first_frame = false;
            }
        } else {
            // 5-step (no IRQ): quarter @ 7457, 22371, 37281; half @ 14913, 37281
            if (a->cycle_in_seq == 7457u || a->cycle_in_seq == 22371u || a->cycle_in_seq == 37281u) {
                apu_clock_quarter_frame(a);
            }
            if (a->cycle_in_seq == 14913u || a->cycle_in_seq == 37281u) {
                apu_clock_half_frame(a);
            }
            if (a->cycle_in_seq >= APU_5STEP_PERIOD) a->cycle_in_seq = 0;
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

        if (a->frame_reset_pending) {
            if (a->frame_reset_delay > 0) {
                a->frame_reset_delay--;
            }
            if (a->frame_reset_delay == 0) {
                a->cycle_in_seq = 0;
                a->frame_reset_pending = false;
                APU_LOG("frame_reset applied");
            }
        }

        a->cpu_cycle_odd = !a->cpu_cycle_odd;
    }
}

int apu_take_dmc_dma_stall_cycles(APU *a) {
    int v = (int)a->dmc_dma_stall_cycles;
    a->dmc_dma_stall_cycles = 0;
    return v;
}

// ---------------- SDL callback ----------------
void apu_sdl_audio_callback(void *userdata, Uint8 *stream, int len){
    (void)userdata;
    float *out = (float*)stream;
    int frames = len / sizeof(float);
    int got = rb_pull(&apu, out, frames);
    for (int i=got; i<frames; ++i) out[i] = apu.last_output_sample;
}
