/*
 * machine_actions.c - Desktop frontend machine lifecycle actions
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "machine_actions.h"
#include "../apu/apu.h"
#include "../cpu/cpu.h"
#include "../debugger/debugger.h"
#include "../ppu/ppu.h"
#include "../replay/input_event.h"
#include "../system/vs_system.h"

bool frontend_machine_soft_reset(void) {
    NesInputEvent event = {.type = NES_INPUT_EVENT_SOFT_RESET};
    if (!nes_input_event_submit(&event)) return false;
    debugger_invalidate_memory();
    ppu_soft_reset(&ppu);
    apu_soft_reset(&apu);
    cpu_soft_reset(&cpu);
    vs_soft_reset();
    return true;
}

bool frontend_machine_power_cycle(void) {
    NesInputEvent event = {.type = NES_INPUT_EVENT_POWER_CYCLE};
    if (!nes_input_event_submit(&event)) return false;
    debugger_invalidate_memory();
    ppu_power_on(&ppu);
    apu_power_on(&apu);
    if (!cpu_power_on(&cpu)) return false;
    vs_power_on_secondary();
    return true;
}
