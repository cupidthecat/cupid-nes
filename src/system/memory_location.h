/*
 * memory_location.h
 * Author: @frankischilling
 * This file is part of Cupid NES Emulator.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
/* Observational memory backing. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_MEMORY_LOCATION_H
#define CUPID_MEMORY_LOCATION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Borrowed pointers are valid only until the machine or its mapping changes.
 * Resolve and consume them under the same paused-machine lock. No bus access
 * is needed to inspect or edit a resolved byte. */
typedef struct {
    uint8_t *data, *mirror;
    const char *backing;
    size_t offset;
    bool *dirty;
    uint64_t *refresh;
    uint64_t refresh_cycle;
    uint8_t mask;
    bool writable;
} NesMemoryLocation;

static inline bool nes_memory_location(NesMemoryLocation *out, uint8_t *base, size_t size, size_t offset,
                                        const char *backing, bool writable, bool *dirty) {
    if (!out || !base || offset >= size) return false;
    out->data = base + offset;
    out->mirror = NULL;
    out->backing = backing;
    out->offset = offset;
    out->dirty = dirty;
    out->refresh = NULL;
    out->refresh_cycle = 0;
    out->mask = 0xFF;
    out->writable = writable;
    return true;
}

#endif
