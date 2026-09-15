/* NTSC audio register, divider, sequencer, and sample-reader regressions. */
#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../rom/mapper.h"
#include <stdio.h>

static int checks;
static int failures;
static unsigned sample_reads;
static uint16_t first_sample_address;
static uint16_t last_sample_address;

#define CHECK(name, condition) do { \
    checks++; \
    if (!(condition)) { \
        failures++; \
        fprintf(stderr, "APU:%d: %s\n", __LINE__, name); \
    } \
} while (0)

static void reset_audio(void) {
    cpu_total_cycles = 0;
    apu_reset(&apu);
}

static void start_pulse(void) {
    apu_write(0x4015, 1);
    apu_write(0x4000, 0);
    apu_write(0x4002, 100);
    apu_write(0x4003, 0);
}

static uint8_t sample_read(uint16_t address) {
    if (sample_reads++ == 0) first_sample_address = address;
    last_sample_address = address;
    return 0xFF;
}

// Test the reader/output boundary independently of CPU DMA arbitration.
// CPU regressions verify the halt, alignment, and shared OAM DMA cycles.
static void step_sample_reader(int cycles) {
    for (int i = 0; i < cycles; ++i) {
        cpu_total_cycles++;
        apu_step(&apu, 1);
        if (apu_dmc_dma_pending(&apu))
            apu_dmc_dma_complete(&apu, sample_read(apu_dmc_dma_address(&apu)));
    }
}

int test_apu_accuracy(void) {
    checks = failures = 0;

    reset_audio();
    CHECK("noise starts with a nonzero shift register", apu.noise.lfsr == 1);
    apu_step(&apu, 3);
    CHECK("noise period is measured in CPU clocks", apu.noise.lfsr == 1);
    apu_step(&apu, 1);
    CHECK("first noise clock occurs after four CPU clocks", apu.noise.lfsr == 0x4000);
    apu_step(&apu, 4);
    CHECK("noise divider repeats every four clocks", apu.noise.lfsr == 0x2000);
    apu.noise.lfsr = 3;
    apu.noise.timer = 0;
    apu_step(&apu, 1);
    CHECK("long noise mode taps bits zero and one", apu.noise.lfsr == 1);
    apu_write(0x400E, 0x80);
    apu.noise.lfsr = 0x41;
    apu.noise.timer = 0;
    apu_step(&apu, 1);
    CHECK("short noise mode taps bits zero and six", apu.noise.lfsr == 0x20);

    static const int noise_periods[16] = {
        4,8,16,32,64,96,128,160,202,254,380,508,762,1016,2034,4068
    };
    for (int rate = 0; rate < 16; ++rate) {
        reset_audio();
        apu_write(0x400E, (uint8_t)rate);
        apu.noise.timer = 0;
        apu_step(&apu, 1);
        apu_step(&apu, noise_periods[rate] - 1);
        CHECK("noise does not advance before the selected period", apu.noise.lfsr == 0x4000);
        apu_step(&apu, 1);
        CHECK("noise advances at the selected period", apu.noise.lfsr == 0x2000);
    }

    reset_audio();
    start_pulse();
    apu_step(&apu, 7456);
    CHECK("envelope waits for first quarter-frame", apu.pulse1.env.start_flag);
    apu_step(&apu, 1);
    CHECK("first quarter-frame starts envelope", apu.pulse1.env.decay == 15);
    CHECK("quarter-frame does not clock length", apu.pulse1.lc.length == 10);
    apu_step(&apu, 14913 - 7457);
    CHECK("half-frame also clocks envelope", apu.pulse1.env.decay == 14);
    CHECK("first half-frame clocks length", apu.pulse1.lc.length == 9);
    apu_step(&apu, 29827 - 14913);
    CHECK("frame IRQ does not assert early", !apu.frame_irq);
    apu_step(&apu, 1);
    CHECK("frame IRQ first asserts at clock 29828", apu_read(0x4015) & 0x40);
    CHECK("status read clears frame IRQ", !apu.frame_irq);
    apu_step(&apu, 1);
    CHECK("frame IRQ reasserts at clock 29829", apu_read(0x4015) & 0x40);
    CHECK("terminal half-frame is not lost to IRQ handling", apu.pulse1.lc.length == 8);
    CHECK("terminal quarter-frame clocks envelope", apu.pulse1.env.decay == 12);
    apu_step(&apu, 1);
    CHECK("frame IRQ reasserts at clock 29830", apu_read(0x4015) & 0x40);
    CHECK("four-step sequence lasts 29830 clocks", apu.cycle_in_seq == 0);
    apu_step(&apu, 1);
    CHECK("IRQ stops reasserting after terminal clocks", !apu.frame_irq);

    reset_audio();
    start_pulse();
    apu_write(0x4017, 0x80);
    CHECK("mode write does not change the active sequence immediately", !apu.five_step);
    CHECK("mode write does not prematurely apply a pending length load", !(apu_read(0x4015) & 1));
    apu_step(&apu, 2);
    CHECK("even-cycle mode write waits three clocks", !apu.five_step);
    apu_step(&apu, 1);
    CHECK("delayed mode write starts five-step sequence", apu.five_step && apu.cycle_in_seq == 0);
    CHECK("five-step reset clocks quarter and half units", apu.pulse1.lc.length == 9 && apu.pulse1.env.decay == 15);
    apu_step(&apu, 14913);
    CHECK("five-step clock 14913 includes quarter-frame", apu.pulse1.env.decay == 13);
    CHECK("five-step clock 14913 includes half-frame", apu.pulse1.lc.length == 8);
    apu_step(&apu, 37281 - 14913);
    CHECK("five-step terminal event clocks both units", apu.pulse1.lc.length == 7 && apu.pulse1.env.decay == 11);
    CHECK("five-step sequence never generates frame IRQ", !apu.frame_irq);
    apu_step(&apu, 1);
    CHECK("five-step sequence lasts 37282 clocks", apu.cycle_in_seq == 0);

    reset_audio();
    cpu_total_cycles = 1;
    apu_write(0x4017, 0x80);
    apu_step(&apu, 3);
    CHECK("odd-cycle mode write waits four clocks", !apu.five_step);
    apu_step(&apu, 1);
    CHECK("odd-cycle mode write applies on its fourth clock", apu.five_step);
    reset_audio();
    apu.frame_irq = true;
    apu_write(0x4017, 0x80);
    CHECK("bit six clear preserves a pending frame IRQ", apu.frame_irq);
    apu_write(0x4017, 0x40);
    CHECK("IRQ inhibit clears a pending frame IRQ immediately", !apu.frame_irq);
    reset_audio();
    start_pulse();
    apu.cycle_in_seq = 7455;
    apu_write(0x4017, 0x80);
    apu_step(&apu, 3);
    CHECK("adjacent sequencer reset does not double-clock envelope", apu.pulse1.env.decay == 15);
    CHECK("adjacent sequencer reset does not add a half-frame", apu.pulse1.lc.length == 10);

    for (int channel = 0; channel < 4; ++channel) {
        reset_audio();
        apu_write(0x4015, 0x0F);
        LengthCounter *lengths[4] = {&apu.pulse1.lc, &apu.pulse2.lc, &apu.tri.lc, &apu.noise.lc};
        LengthCounter *length = lengths[channel];
        uint16_t control = (uint16_t)(0x4000 + channel * 4);
        uint8_t halt = channel == 2 ? 0x80 : 0x20;
        length->length = 5;
        apu.cycle_in_seq = 14912;
        apu_write(control + 3, 8);
        apu_step(&apu, 1);
        CHECK("simultaneous length clock wins over nonzero reload", length->length == 4);
        length->length = 0;
        apu.cycle_in_seq = 14912;
        apu.frame_clock_block = 0;
        apu_write(control + 3, 8);
        apu_step(&apu, 1);
        CHECK("zero length counter can reload on a half-frame", length->length == 254);
        length->length = 5;
        apu.cycle_in_seq = 14912;
        apu.frame_clock_block = 0;
        apu_write(control, halt);
        apu_step(&apu, 1);
        CHECK("length halt applies after a simultaneous decrement", length->length == 4 && length->halt);
        apu.cycle_in_seq = 14912;
        apu.frame_clock_block = 0;
        apu_write(control, 0);
        apu_step(&apu, 1);
        CHECK("clearing length halt does not decrement on the same clock", length->length == 4 && !length->halt);
        apu_write(control + 3, 8);
        apu_write(0x4015, 0);
        apu_step(&apu, 1);
        CHECK("channel disable cancels a pending length reload", !length->length);
    }

    reset_audio();
    start_pulse();
    apu_write(0x4001, 0xD1);
    apu.cycle_in_seq = 14912;
    apu_step(&apu, 1);
    CHECK("sweep reload still updates period when divider expires", apu.pulse1.timer_reload == 150);
    CHECK("sweep reload loads the new divider", apu.pulse1.sweep.divider == 5);
    apu.pulse1.sweep.divider = 4;
    apu_write(0x4001, 0xD1);
    apu.cycle_in_seq = 14912;
    apu.frame_clock_block = 0;
    apu_step(&apu, 1);
    CHECK("reload replaces a running sweep divider", apu.pulse1.sweep.divider == 5);
    CHECK("running divider does not update the swept period", apu.pulse1.timer_reload == 150);

    reset_audio();
    apu_write(0x4015, 3);
    apu_write(0x4002, 100);
    apu_write(0x4003, 0);
    apu_write(0x4006, 100);
    apu_write(0x4007, 0);
    apu_write(0x4001, 0x89);
    apu_write(0x4005, 0x89);
    apu.cycle_in_seq = 14912;
    apu_step(&apu, 1);
    CHECK("pulse one negative sweep subtracts an extra one", apu.pulse1.timer_reload == 49);
    CHECK("pulse two negative sweep uses ordinary subtraction", apu.pulse2.timer_reload == 50);

    reset_audio();
    apu_write(0x4015, 1);
    apu_write(0x4000, 0xDF);
    apu_write(0x4002, 0);
    apu_write(0x4003, 5);
    apu.pulse1.timer = 500;
    apu.pulse1.duty_step = 3;
    apu_step(&apu, 100);
    CHECK("sweep overflow mutes a pulse with sweep disabled", apu.last_output_sample == 0.0f);
    apu_write(0x4001, 8);
    apu_step(&apu, 100);
    CHECK("negative sweep does not use positive overflow muting", apu.last_output_sample > 0.0f);

    reset_audio();
    apu_write(0x4015, 4);
    apu_write(0x4008, 0x81);
    apu_write(0x400A, 2);
    apu_write(0x400B, 0);
    apu.tri.linear_counter = 1;
    apu.tri.timer = 0;
    apu_step(&apu, 1);
    CHECK("triangle advances its DAC with the sequencer", apu.tri.output_level == 14);
    apu_write(0x4015, 0);
    apu_step(&apu, 100);
    CHECK("disabled triangle holds its last DAC level", apu.tri.output_level == 14 && apu.tri.step == 1);
    CHECK("held triangle level still enters the mixer", apu.last_output_sample > 0.0f);

    reset_audio();
    CHECK("DMC power-on sample address", apu.dmc.sample_addr == 0xC000);
    CHECK("DMC power-on sample length", apu.dmc.sample_len == 1);
    apu_step(&apu, 426);
    CHECK("DMC startup waits for its first get-phase clock", apu.dmc.bits_remaining == 8);
    apu_step(&apu, 1);
    CHECK("DMC startup output clock lands on odd CPU cycle 427", apu.dmc.bits_remaining == 7);
    apu_step(&apu, 428);
    CHECK("DMC rate does not gain an extra clock per bit", apu.dmc.bits_remaining == 6);

    cpu_total_cycles = 1;
    apu_reset(&apu);
    apu_step(&apu, 427);
    CHECK("odd-cycle DMC initialization retains a full first period", apu.dmc.bits_remaining == 8);
    apu_step(&apu, 1);
    CHECK("DMC output remains on get phase after odd-cycle initialization", apu.dmc.bits_remaining == 7);

    static const int dmc_periods[16] = {
        428,380,340,320,286,254,226,214,190,160,142,128,106,84,72,54
    };
    for (int rate = 0; rate < 16; ++rate) {
        reset_audio();
        apu_write(0x4010, (uint8_t)rate);
        apu.dmc.timer = 0;
        apu_step(&apu, 1);
        apu_step(&apu, dmc_periods[rate] - 1);
        CHECK("DMC holds its output-bit counter for the selected period", apu.dmc.bits_remaining == 7);
        apu_step(&apu, 1);
        CHECK("DMC advances after the exact selected period", apu.dmc.bits_remaining == 6);
    }

    reset_audio();
    apu_write(0x4010, 0x0F);
    apu.dmc.sample_buffer = 0xFF;
    apu.dmc.sample_buffer_empty = false;
    apu_step(&apu, 428 + 7 * 54);
    CHECK("DMC reloads the output register at the end of bit eight", apu.dmc.bits_remaining == 8 && !apu.dmc.silence && apu.dmc.sample_buffer_empty);
    CHECK("loading a DMC byte does not play a bit early", apu.dmc.output_level == 0);
    apu_step(&apu, 54);
    CHECK("DMC plays first loaded bit on the next timer clock", apu.dmc.output_level == 2);

    reset_audio();
    sample_reads = 0;
    apu_write(0x4010, 0x8F);
    apu_write(0x4015, 0x10);
    CHECK("DMC enable does not synchronously read CPU memory", !apu_dmc_dma_pending(&apu) && sample_reads == 0);
    apu_step(&apu, 1);
    CHECK("even-cycle DMC enable waits a full APU cycle", !apu_dmc_dma_pending(&apu));
    apu_step(&apu, 1);
    CHECK("DMC requests a read after its startup delay", apu_dmc_dma_pending(&apu));
    CHECK("DMC has not consumed memory before DMA completion", apu.dmc.bytes_remaining == 1 && !apu.dmc.irq_flag);
    apu_dmc_dma_complete(&apu, sample_read(apu_dmc_dma_address(&apu)));
    CHECK("DMC fetches from its power-on address", sample_reads == 1 && first_sample_address == 0xC000);
    CHECK("last DMC memory fetch asserts IRQ", apu.dmc.irq_flag);
    CHECK("sample reader status clears while output data remains", !(apu_read(0x4015) & 0x10));
    CHECK("status read preserves DMC IRQ", apu.dmc.irq_flag);
    CHECK("DMC completion clears the outstanding bus request", !apu_dmc_dma_pending(&apu));
    apu_write(0x4015, 0);
    step_sample_reader(10000);
    CHECK("completed DMC output cannot reassert a cleared IRQ", !apu.dmc.irq_flag);

    reset_audio();
    sample_reads = 0;
    apu_write(0x4010, 0x8F);
    apu_write(0x4012, 0xFF);
    apu_write(0x4013, 4);
    apu_write(0x4015, 0x10);
    step_sample_reader(30000);
    CHECK("DMC sample length register counts sixteen-byte units plus one", sample_reads == 65);
    CHECK("DMC sample reader wraps from FFFF to 8000", first_sample_address == 0xFFC0 && last_sample_address == 0x8000);
    CHECK("DMC wrapped address advances after final fetch", apu.dmc.current_addr == 0x8001);
    CHECK("DMC sample finishes with IRQ and no remaining bytes", apu.dmc.irq_flag && apu.dmc.bytes_remaining == 0);

    reset_audio();
    sample_reads = 0;
    apu_write(0x4010, 0xCF);
    apu_write(0x4015, 0x10);
    step_sample_reader(2000);
    CHECK("looping DMC restarts sample reader", sample_reads > 1 && apu.dmc.bytes_remaining == 1);
    CHECK("looping DMC never raises end-of-sample IRQ", !apu.dmc.irq_flag);
    reset_audio();
    cpu_total_cycles = 1;
    apu_write(0x4015, 0x10);
    apu_step(&apu, 2);
    CHECK("odd-cycle DMC enable waits three CPU clocks", !apu_dmc_dma_pending(&apu));
    apu_step(&apu, 1);
    CHECK("odd-cycle DMC request appears on the third clock", apu_dmc_dma_pending(&apu));
    apu_write(0x4015, 0);
    apu_step(&apu, 2);
    CHECK("DMC disable preserves a request until the delayed stop", apu_dmc_dma_pending(&apu));
    apu_step(&apu, 1);
    CHECK("DMC delayed disable cancels its pending bus request", !apu_dmc_dma_pending(&apu) && !apu.dmc.bytes_remaining);
    apu_dmc_dma_complete(&apu, 0x55);
    CHECK("aborted DMC completion cannot refill its buffer", apu.dmc.sample_buffer_empty);
    printf("APU: %d checks, %d failures\n", checks, failures);
    return failures;
}
