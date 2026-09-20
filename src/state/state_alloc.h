/*
 * state_alloc.h - Save-state preparation allocator
 *
 * Author: @frankischilling
 *
 * This file is part of Cupid NES Emulator and is distributed under the
 * GNU General Public License, version 3 or any later version.
 */
#ifndef CUPID_NES_STATE_ALLOC_H
#define CUPID_NES_STATE_ALLOC_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void *nes_state_alloc(size_t size);
void nes_state_dealloc(void *pointer);

/* Hardware tests use this to make restore-allocation failures deterministic. */
void nes_state_test_fail_allocation_after(int successful_allocations);

#ifdef __cplusplus
}
#endif

#endif
