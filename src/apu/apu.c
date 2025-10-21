#include "apu.h"
#include <string.h>
#include <math.h>
#include <SDL2/SDL.h>

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
    a->dmc.enabled = false;
    a->noise.lfsr = 1; // cannot be 0
    a->cycles_per_sample = 1789773.0 / 44100.0;
    a->sample_rate = 44100.0;
}
void apu_audio_init(int sample_rate) {
    apu.sample_rate = (double)sample_rate;
    apu.cycles_per_sample = 1789773.0 / apu.sample_rate;
    apu.sample_accum = 0.0;
    apu.ring_w = apu.ring_r = 0;
}

// ---------------- Reads/Writes ----------------
static inline void apu_write_4017(APU *a, uint8_t v) {
    a->regs[0x17] = v;
    a->five_step   = (v & 0x80) != 0;
    a->irq_inhibit = (v & 0x40) != 0;
    if (a->irq_inhibit) a->frame_irq = false;
    a->cycle_in_seq = 0;
}
static inline uint8_t apu_read_4015(APU *a) {
    uint8_t s = 0;
    if (a->pulse1.lc.length) s |= 0x01;
    if (a->pulse2.lc.length) s |= 0x02;
    if (a->tri.lc.length)    s |= 0x04;
    if (a->noise.lc.length)  s |= 0x08;
    if (a->frame_irq)        s |= 0x40;
    // DMC IRQ would be bit7
    a->frame_irq = false;
    return s;
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
    else if (addr == 0x4010) { /* DMC cfg (stub) */ }
    else if (addr == 0x4011) { /* DMC DAC (stub) */ }
    else if (addr == 0x4012) { /* DMC addr (stub) */ }
    else if (addr == 0x4013) { /* DMC len  (stub) */ }
    else if (addr == 0x4015) {
        apu.pulse1.enabled = (v & 0x01) != 0; if (!apu.pulse1.enabled) apu.pulse1.lc.length = 0;
        apu.pulse2.enabled = (v & 0x02) != 0; if (!apu.pulse2.enabled) apu.pulse2.lc.length = 0;
        apu.tri.enabled    = (v & 0x04) != 0; if (!apu.tri.enabled)    apu.tri.lc.length = 0;
        apu.noise.enabled  = (v & 0x08) != 0; if (!apu.noise.enabled)  apu.noise.lc.length = 0;
        apu.dmc.enabled    = (v & 0x10) != 0; // stub
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
    static uint16_t div = 0;
    if (div == 0) div = n->period;
    if (--div == 0) {
        div = n->period;
        // XOR taps: 1 and 6 when mode=1 (7-bit), else 1 and 14 (15-bit)
        uint16_t bit0 = n->lfsr & 1;
        uint16_t bitX = (n->mode ? ((n->lfsr >> 6) & 1) : ((n->lfsr >> 14) & 1));
        uint16_t fb = bit0 ^ bitX;
        n->lfsr = (n->lfsr >> 1) | (fb << 14);
    }
}

// DAC-ish sample (0..1 per channel)
static inline float pulse_out(const Pulse* p){
    if (!p->enabled || p->lc.length == 0) return 0.0f;
    if (p->timer_reload < 8 || p->timer_reload > 0x7FF) return 0.0f;
    uint8_t gate = DUTY_SEQ[p->duty][p->duty_step];
    if (!gate) return 0.0f;
    float vol = env_output(&p->env) / 15.0f;
    return vol; // raw before nonlinear mixer
}
static inline float triangle_out(const Triangle* t){
    if (!t->enabled || t->lc.length == 0 || t->linear_counter == 0) return 0.0f;
    // 32-step triangle: 0..15 descending, 16..31 ascending (or vice versa)
    static const uint8_t TRI_SEQ[32] = {
        15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,
         0, 1, 2, 3, 4, 5,6,7,8,9,10,11,12,13,14,15
    };
    return TRI_SEQ[t->step] / 15.0f;
}
static inline float noise_out(const Noise* n){
    if (!n->enabled || n->lc.length == 0) return 0.0f;
    // if lfsr bit0 is 1 -> output 0, else envelope
    if (n->lfsr & 1) return 0.0f;
    return env_output(&n->env) / 15.0f;
}

// Nonlinear mixer (NESdev): pulse & TND
static inline float mix_sample(float p1, float p2, float tri, float noi /*, float dmc*/){
    float pulse = (p1 + p2);
    float tnd   = tri/8227.0f + noi/12241.0f /* + dmc/22638.0f*/;
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
        // tick sequencers
        clock_pulse(&a->pulse1);
        clock_pulse(&a->pulse2);
        clock_triangle(&a->tri);
        clock_noise(&a->noise);

        // frame sequencer timing
        a->cycle_in_seq++;
        // Quarter @ 3729, 11186; Quarter+Half @ 7457, 14916
        if (!a->five_step) {
            if (a->cycle_in_seq == 3729u || a->cycle_in_seq == 11186u) {
                // quarter: envelopes + triangle linear
                env_clock(&a->pulse1.env);
                env_clock(&a->pulse2.env);
                env_clock(&a->noise.env);
                tri_linear_clock(&a->tri);
            } else if (a->cycle_in_seq == 7457u) {
                // quarter+half
                env_clock(&a->pulse1.env);
                env_clock(&a->pulse2.env);
                env_clock(&a->noise.env);
                tri_linear_clock(&a->tri);
                length_clock(&a->pulse1.lc);
                length_clock(&a->pulse2.lc);
                length_clock(&a->tri.lc);
                length_clock(&a->noise.lc);
                sweep_clock(&a->pulse1, false);
                sweep_clock(&a->pulse2, true);
            } else if (a->cycle_in_seq == 14916u) {
                // quarter+half + IRQ edge
                env_clock(&a->pulse1.env);
                env_clock(&a->pulse2.env);
                env_clock(&a->noise.env);
                tri_linear_clock(&a->tri);
                length_clock(&a->pulse1.lc);
                length_clock(&a->pulse2.lc);
                length_clock(&a->tri.lc);
                length_clock(&a->noise.lc);
                sweep_clock(&a->pulse1, false);
                sweep_clock(&a->pulse2, true);
                if (!a->irq_inhibit) a->frame_irq = true;
                a->cycle_in_seq = 0;
            }
        } else {
            // 5-step (no IRQ), extra quarter at 18640
            if (a->cycle_in_seq == 3729u || a->cycle_in_seq == 11186u || a->cycle_in_seq == 18640u) {
                env_clock(&a->pulse1.env); env_clock(&a->pulse2.env); env_clock(&a->noise.env);
                tri_linear_clock(&a->tri);
                if (a->cycle_in_seq == 7457u || a->cycle_in_seq == 18640u) {
                    length_clock(&a->pulse1.lc); length_clock(&a->pulse2.lc);
                    length_clock(&a->tri.lc);    length_clock(&a->noise.lc);
                    sweep_clock(&a->pulse1, false); sweep_clock(&a->pulse2, true);
                }
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
            float s  = mix_sample(p1, p2, tr, nz);

            rb_push(a, s);
        }
    }
}

// ---------------- SDL callback ----------------
void apu_sdl_audio_callback(void *userdata, Uint8 *stream, int len){
    float *out = (float*)stream;
    int frames = len / sizeof(float);
    int got = rb_pull(&apu, out, frames);
    for (int i=got; i<frames; ++i) out[i] = 0.0f;
}
