/* SPDX-License-Identifier: GPL-3.0-or-later
 * Public APU state and interfaces used by the emulator core.
 */

#ifndef APU_H
#define APU_H
#include <stdint.h>
#include <stdbool.h>
#include <stdatomic.h>
// NTSC APU frame-sequencer constants (CPU cycles)
#define APU_4STEP_PERIOD 29830u
#define APU_5STEP_PERIOD 37282u

typedef struct {
    // Envelope (for pulse/noise)
    bool    loop_envelope;    // also "halt length"
    bool    constant_volume;
    uint8_t volume;           // 0..15
    uint8_t divider;          // envelope divider
    uint8_t decay;            // envelope decay counter
    bool    start_flag;       // envelope restart
} Envelope;

typedef struct {
    // Sweep (pulse only)
    bool    enabled;
    uint8_t period;    // divider period (1..8 after a register write)
    bool    negate;
    uint8_t shift;     // 0..7
    uint8_t divider;   // internal sweep divider
    bool    reload;    // reload flag
} Sweep;

typedef struct {
    // Length counter (all non-DMC channels)
    uint8_t length;      // 0 means silent if not halted
    bool    halt;        // "halt" == loop envelope / stop length decrement
    bool    next_halt;
    bool    halt_pending;
    uint8_t reload_value;
    uint8_t previous_value;
} LengthCounter;

typedef struct {
    // Pulse channel common
    Envelope env;
    Sweep    sweep;
    LengthCounter lc;

    uint16_t timer;      // 11-bit timer (store as 16)
    uint16_t timer_reload;
    uint8_t  duty;       // 0..3 (12.5/25/50/25% neg)
    uint8_t  duty_step;  // sequencer step 0..7
    bool     enabled;    // $4015 bit
} Pulse;

typedef struct {
    LengthCounter lc;
    bool    control;     // linear counter control (also halts length)
    uint8_t linear_reload_val;
    uint8_t linear_counter;
    bool    linear_reload;

    uint16_t timer;
    uint16_t timer_reload;
    uint8_t  step;       // 0..31 waveform step
    uint8_t  output_level; // DAC holds its last value when the sequencer stops
    bool     enabled;
} Triangle;

typedef struct {
    Envelope env;
    LengthCounter lc;

    bool     mode;       // 0: 15-bit, 1: 7-bit taps
    uint16_t lfsr;       // 15-bit shift register
    uint16_t period;     // current period from table
    uint8_t  period_idx; // 0..15
    uint16_t timer;
    bool     enabled;
} Noise;

typedef struct {
    bool enabled;
    bool irq_enable;
    bool irq_flag;
    bool loop;
    uint8_t rate_index;
    uint16_t timer;
    uint16_t timer_reload;

    uint8_t output_level; // 0..127

    uint8_t sample_addr_reg;
    uint8_t sample_len_reg;
    uint16_t sample_addr;
    uint16_t sample_len;

    uint16_t current_addr;
    uint16_t bytes_remaining;

    uint8_t shift_reg;
    uint8_t bits_remaining;
    bool silence;

    uint8_t sample_buffer;
    bool sample_buffer_empty;
    uint8_t start_delay;
    uint8_t disable_delay;
    bool dma_pending;
    bool dma_halt_started;
    bool dma_abort_requested;
} DMC;

typedef struct {
    // Frame sequencer
    uint32_t cycle_in_seq;
    bool five_step;
    bool irq_inhibit;
    bool frame_irq;
    uint8_t frame_irq_clear_delay;
    uint8_t frame_reset_delay;
    bool frame_reset_pending;
    bool cpu_cycle_odd;
    bool frame_next_five_step;
    uint8_t frame_clock_block;

    // Channels
    Pulse    pulse1, pulse2;
    Triangle tri;
    Noise    noise;
    DMC      dmc;

    // Register mirror (for reads)
    uint8_t regs[0x18];

    // Audio output/resampling
    double   sample_rate;    // e.g., 44100
    double   cycles_per_sample; // CPU cycles per audio sample
    double   sample_accum;   // accum CPU cycles towards next sample

    // Output filter state/coefs (NES-like analog chain approximation)
    float hp90_alpha;
    float hp440_alpha;
    float lp14k_alpha;
    float hp90_prev_in;
    float hp90_prev_out;
    float hp440_prev_in;
    float hp440_prev_out;
    float lp14k_prev_out;
    float last_output_sample;

    // Lockless ring buffer (very simple)
    #define APU_RING_CAP 8192
    float    ring[APU_RING_CAP];
    _Atomic uint32_t ring_w;
    _Atomic uint32_t ring_r;
} APU;

extern APU apu;

// lifecycle
void apu_power_on(APU *a);
void apu_soft_reset(APU *a);
void apu_reset(APU *a);
void apu_audio_init(int sample_rate);

// memory-mapped access
void    apu_write(uint16_t addr, uint8_t val);
uint8_t apu_read(uint16_t addr);

// ticking
void apu_step(APU *a, int cpu_cycles);

// IRQ
static inline bool apu_irq_pending(const APU *a) { return (a->frame_irq && !a->irq_inhibit) || a->dmc.irq_flag; }
static inline void apu_clear_frame_irq(APU *a) {
    ((APU*)a)->frame_irq = false;
    ((APU*)a)->frame_irq_clear_delay = 0;
}
bool apu_dmc_dma_pending(const APU *a);
uint16_t apu_dmc_dma_address(const APU *a);
void apu_dmc_dma_complete(APU *a, uint8_t value);

// SDL glue
void apu_sdl_audio_callback(void *userdata, uint8_t *stream, int len);

#endif
