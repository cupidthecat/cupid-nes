// apu.c 
// very basic 
#include "apu.h"

// Public singleton
APU apu;

// Quarter/Half frame event points (in CPU cycles) for reference.
// We’re not clocking envelopes/length counters yet, just the IRQ.
// 4-step: Q at 3729, 11186; Q+H at 7457, 14916; IRQ at 14916 if not inhibited
static const uint32_t EV4_QH[4] = { 3729u, 7457u, 11186u, 14914u };
// 5-step: Q at 3729, 11186; Q+H at 7457, 14916; extra step at 18640; no IRQ
static const uint32_t EV5_QH[5] = { 3729u, 7457u, 11186u, 14916u, 18640u };
void apu_reset(APU *a) {
    a->cycle_in_seq = 0;
    a->five_step    = false;
    a->irq_inhibit  = false;
    a->frame_irq    = false;

    for (int i = 0; i < 0x18; ++i) a->regs[i] = 0;
}

// $4017 write: frame counter
// bit7 = 1: 5-step; bit6 = 1: IRQ inhibit/clear; bits 0-5 unused
static inline void apu_write_4017(APU *a, uint8_t v) {
    a->regs[0x17] = v;
    a->five_step   = (v & 0x80) != 0;
    a->irq_inhibit = (v & 0x40) != 0;
    if (a->irq_inhibit) a->frame_irq = false; // writing IRQ disable clears flag
    // Writing $4017 resets the frame sequencer immediately on the 2A03
    a->cycle_in_seq = 0;
}

// $4015 write: channel enables (we just latch it now)
static inline void apu_write_4015(APU *a, uint8_t v) {
    a->regs[0x15] = v;
    // Real APU would enable/disable length counters here.
}

// $4015 read: status
// bit0..3 length counters (stubbed as 0 for now)
// bit6 = frame interrupt flag
// bit7 = DMC interrupt flag (0 in this minimal impl)
// Reading $4015 clears bit6 (frame IRQ flag).
static inline uint8_t apu_read_4015(APU *a) {
    uint8_t status = 0x00;
    if (a->frame_irq) status |= 0x40;
    a->frame_irq = false; // reading clears the frame IRQ flag
    return status;
}

void apu_write(uint16_t addr, uint8_t val) {
    if (addr < 0x4000 || addr > 0x4017) return;

    // Latch register value for debug/reads
    apu.regs[addr - 0x4000] = val;

    switch (addr) {
        case 0x4015: apu_write_4015(&apu, val); break;
        case 0x4017: apu_write_4017(&apu, val); break;
        default: /* $4000–$4013 channel regs: just latched for now */ break;
    }
}

uint8_t apu_read(uint16_t addr) {
    if (addr == 0x4015) return apu_read_4015(&apu);
    // $4016/$4017 are not APU reads here (joypad at $4016; $4017 usually open bus)
    if (addr >= 0x4000 && addr <= 0x4013) return apu.regs[addr - 0x4000]; // benign
    if (addr == 0x4017) return apu.regs[0x17]; // benign
    return 0x00;
}

// Tick the frame sequencer and set frame IRQ when appropriate.
void apu_step(APU *a, int cpu_cycles) {
    // Iterate per cycle to avoid missing edge cases across boundaries
    for (int i = 0; i < cpu_cycles; ++i) {
        a->cycle_in_seq++;

        if (!a->five_step) {
            // 4-step mode
            if (a->cycle_in_seq == EV4_QH[3]) {
                // End of step 4 -> frame IRQ (if not inhibited)
                if (!a->irq_inhibit) a->frame_irq = true;
                a->cycle_in_seq = 0; // wrap to new sequence
            } else if (a->cycle_in_seq > APU_4STEP_PERIOD) {
                // safety wrap
                a->cycle_in_seq %= APU_4STEP_PERIOD;
            }
        } else {
            // 5-step mode (no frame IRQ)
            if (a->cycle_in_seq == EV5_QH[4]) {
                a->cycle_in_seq = 0;
            } else if (a->cycle_in_seq > APU_5STEP_PERIOD) {
                a->cycle_in_seq %= APU_5STEP_PERIOD;
            }
        }
    }
}