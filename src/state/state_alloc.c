/*
 * state_alloc.c - Save-state preparation allocator
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#include "state_alloc.h"

#include <stdlib.h>

static int fail_after = -1;

void *nes_state_alloc(size_t size) {
    if (!size) return NULL;
    if (fail_after == 0) return NULL;
    if (fail_after > 0) --fail_after;
    return malloc(size);
}

void nes_state_dealloc(void *pointer) {
    free(pointer);
}

void nes_state_test_fail_allocation_after(int successful_allocations) {
    fail_after = successful_allocations < 0 ? -1 : successful_allocations;
}
