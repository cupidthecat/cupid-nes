/*
 * cpu_observer.c - Host-only observation of completed CPU bus writes
 * Author: @frankischilling
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "cpu_observer.h"
#include "../system/execution_policy.h"

#include <stddef.h>

enum { CPU_WRITE_OBSERVER_CAPACITY = 8 };

typedef struct {
    NesCpuWriteObserver callback;
    void *context;
    uint32_t token;
} CpuWriteObserverEntry;

static CpuWriteObserverEntry observers[CPU_WRITE_OBSERVER_CAPACITY];
static uint32_t next_token = 1;
static unsigned observer_count;
static bool dispatching;

uint32_t nes_cpu_write_observer_add(NesCpuWriteObserver observer, void *context) {
    if (!observer || dispatching || observer_count == CPU_WRITE_OBSERVER_CAPACITY) return 0;
    for (size_t i = 0; i < CPU_WRITE_OBSERVER_CAPACITY; ++i) {
        if (observers[i].callback) continue;
        uint32_t token;
        bool collision;
        do {
            token = next_token++;
            collision = token == 0;
            for (size_t j = 0; j < CPU_WRITE_OBSERVER_CAPACITY; ++j)
                if (observers[j].callback && observers[j].token == token) collision = true;
        } while (collision);
        observers[i] = (CpuWriteObserverEntry){observer, context, token};
        ++observer_count;
        return token;
    }
    return 0;
}

bool nes_cpu_write_observer_remove(uint32_t token) {
    if (!token) return false;
    for (size_t i = 0; i < CPU_WRITE_OBSERVER_CAPACITY; ++i) {
        if (observers[i].callback && observers[i].token == token) {
            observers[i] = (CpuWriteObserverEntry){0};
            --observer_count;
            return true;
        }
    }
    return false;
}

void nes_cpu_write_observe(uint16_t address, uint8_t value) {
    if (!observer_count || dispatching
        || (nes_execution_policy() & (NES_EXECUTION_SPECULATIVE | NES_EXECUTION_REWIND))) return;
    dispatching = true;
    for (size_t i = 0; i < CPU_WRITE_OBSERVER_CAPACITY; ++i)
        if (observers[i].callback) observers[i].callback(observers[i].context, address, value);
    dispatching = false;
}
