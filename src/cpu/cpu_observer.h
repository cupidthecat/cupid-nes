/*
 * cpu_observer.h - Host-only observation of completed CPU bus writes
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef CUPID_NES_CPU_OBSERVER_H
#define CUPID_NES_CPU_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*NesCpuWriteObserver)(void *context, uint16_t address, uint8_t value);

uint32_t nes_cpu_write_observer_add(NesCpuWriteObserver observer, void *context);
bool nes_cpu_write_observer_remove(uint32_t token);

/* Called after a real write has reached the emulated bus target. Observers are
 * host presentation/audio only and are suppressed during rewind/speculation. */
void nes_cpu_write_observe(uint16_t address, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif
