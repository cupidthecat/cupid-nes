/*
 * memory_view.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Bounded, observational memory access. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_DEBUG_MEMORY_VIEW_H
#define CUPID_DEBUG_MEMORY_VIEW_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef uint32_t DebugMemorySpace;
enum {
    DEBUG_MEMORY_RAM,
    DEBUG_MEMORY_CART_RAM,
    DEBUG_MEMORY_CPU,
    DEBUG_MEMORY_PPU,
    DEBUG_MEMORY_OAM,
    DEBUG_MEMORY_PALETTE,
    DEBUG_MEMORY_SPACE_COUNT
};

const char *debug_memory_name(DebugMemorySpace space);
bool debug_memory_bounds(DebugMemorySpace space, uint32_t *first, uint32_t *last);
bool debug_memory_cpu_space(DebugMemorySpace space);
/* Reject floating CPU addresses and range overflow without driving the bus. */
bool debug_memory_readable(DebugMemorySpace space, uint32_t first, uint32_t last);
/* Inclusive address range, with no wrapping or bus side effects. Invalid
 * arguments leave the destination unchanged. Values exclude cheat overlays. */
bool debug_memory_sample(DebugMemorySpace space, uint32_t first, uint32_t last, uint8_t *out, size_t capacity);

#endif
