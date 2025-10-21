#ifndef APU_H
#define APU_H
#include <stdint.h>
#include <stdbool.h>
#include <SDL2/SDL.h>
// NTSC APU frame-sequencer constants (CPU cycles)
#define APU_4STEP_PERIOD 14916u
#define APU_5STEP_PERIOD 18640u

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
    uint8_t period;    // divider period (0..7)
    bool    negate;
    uint8_t shift;     // 0..7
    uint8_t divider;   // internal sweep divider
    bool    reload;    // reload flag
} Sweep;

typedef struct {
    // Length counter (all non-DMC channels)
    uint8_t length;      // 0 means silent if not halted
    bool    halt;        // "halt" == loop envelope / stop length decrement
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
    bool     enabled;
} Triangle;

typedef struct {
    Envelope env;
    LengthCounter lc;

    bool     mode;       // 0: 15-bit, 1: 7-bit taps
    uint16_t lfsr;       // 15-bit shift register
    uint16_t period;     // current period from table
    uint8_t  period_idx; // 0..15
    bool     enabled;
} Noise;

// Minimal (stub) DMC holder; implement later if you want DK SFX perfect
typedef struct {
    bool enabled;
} DMC;

typedef struct {
    // Frame sequencer
    uint32_t cycle_in_seq;
    bool five_step;
    bool irq_inhibit;
    bool frame_irq;

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

    // Lockless ring buffer (very simple)
    #define APU_RING_CAP 8192
    float    ring[APU_RING_CAP];
    volatile uint32_t ring_w;
    volatile uint32_t ring_r;
} APU;

extern APU apu;

// lifecycle
void apu_reset(APU *a);
void apu_audio_init(int sample_rate);

// memory-mapped access
void    apu_write(uint16_t addr, uint8_t val);
uint8_t apu_read(uint16_t addr);

// ticking
void apu_step(APU *a, int cpu_cycles);

// IRQ
static inline bool apu_irq_pending(const APU *a) { return a->frame_irq; }
static inline void apu_clear_frame_irq(APU *a)   { ((APU*)a)->frame_irq = false; }

// SDL glue
void apu_sdl_audio_callback(void *userdata, Uint8 *stream, int len);

#endif
