#include "vrc7_audio.h"

#include <stddef.h>
#include <string.h>

#include "emu2413.h"

#define VRC7_SAMPLE_RATE 49716.0
#define VRC7_OPLL_CLOCK  (49716u * 72u)
#define VRC7_OUTPUT_SCALE (1.0f / 5000.0f)

bool vrc7_fm_init(Vrc7Fm *fm) {
    if (!fm) return false;
    OPLL *opll = OPLL_new(VRC7_OPLL_CLOCK, (uint32_t)VRC7_SAMPLE_RATE);
    if (!opll) return false;
    *fm = (Vrc7Fm){.opll = opll};
    vrc7_fm_reset(fm);
    return true;
}

void vrc7_fm_destroy(Vrc7Fm *fm) {
    if (!fm) return;
    if (fm->opll) OPLL_delete(fm->opll);
    memset(fm, 0, sizeof(*fm));
}

void vrc7_fm_reset(Vrc7Fm *fm) {
    if (!fm || !fm->opll) return;
    OPLL_reset(fm->opll);
    OPLL_setChipType(fm->opll, OPLL_VRC7_TONE);
    OPLL_resetPatch(fm->opll, OPLL_VRC7_TONE);
    fm->address = 0;
    fm->clock_timer = 0.0;
    fm->output = 0.0f;
    fm->muted = false;
}

void vrc7_fm_set_muted(Vrc7Fm *fm, bool muted) {
    if (!fm) return;
    fm->muted = muted;
}

void vrc7_fm_write_address(Vrc7Fm *fm, uint8_t value) {
    if (!fm || fm->muted) return;
    fm->address = value;
}

void vrc7_fm_write_data(Vrc7Fm *fm, uint8_t value) {
    if (!fm || !fm->opll || fm->muted) return;
    OPLL_writeReg(fm->opll, fm->address, value);
}

void vrc7_fm_clock(Vrc7Fm *fm, int cpu_cycles, double cpu_hz) {
    if (!fm || !fm->opll || cpu_cycles <= 0 || cpu_hz <= 0.0) return;
    for (int cycle = 0; cycle < cpu_cycles; ++cycle) {
        if (fm->clock_timer == 0.0) fm->clock_timer = cpu_hz / VRC7_SAMPLE_RATE;
        fm->clock_timer -= 1.0;
        if (fm->clock_timer <= 0.0) {
            int16_t sample = OPLL_calc(fm->opll);
            fm->output = (float)sample * VRC7_OUTPUT_SCALE;
            fm->clock_timer = cpu_hz / VRC7_SAMPLE_RATE;
        }
    }
}

float vrc7_fm_output(const Vrc7Fm *fm) {
    return fm && !fm->muted ? fm->output : 0.0f;
}
