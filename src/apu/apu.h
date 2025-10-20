#ifndef APU_H
#define APU_H

#include <stdint.h>
#include <stdbool.h>

// NTSC APU frame-sequencer constants (CPU cycles)
#define APU_4STEP_PERIOD 14916u    // sequence length
#define APU_5STEP_PERIOD 18640u

typedef struct {
    // Frame sequencer state
    uint32_t cycle_in_seq;   // cycles since last $4017 reset (mod sequence len)
    bool five_step;          // $4017 bit 7
    bool irq_inhibit;        // $4017 bit 6  (if set, frame IRQ disabled)
    bool frame_irq;          // latched frame interrupt flag (shown at $4015 bit 6)

    // We just latch registers for now (so games that read back get *something*)
    uint8_t regs[0x18];      // $4000–$4017 mirror (we only *use* $4015/$4017)

} APU;

extern APU apu;

void apu_reset(APU *a);

// Memory-mapped accessors (CPU bus)
void apu_write(uint16_t addr, uint8_t val);
uint8_t apu_read(uint16_t addr);

// Tick the APU by CPU cycles
void apu_step(APU *a, int cpu_cycles);

// IRQ line query/clear
static inline bool apu_irq_pending(const APU *a) { return a->frame_irq; }
static inline void apu_clear_frame_irq(APU *a)   { a->frame_irq = false; }

#endif
