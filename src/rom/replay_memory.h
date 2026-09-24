/* Isolated movie startup memory. SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef CUPID_REPLAY_MEMORY_H
#define CUPID_REPLAY_MEMORY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*CartReplayMemoryInitializer)(void *bytes, size_t size, bool default_zero, void *context);

/* These operations only change the active cartridge's allocated RAM. The movie
 * owner must retain a live restore point and suppress persistence first. */
void cart_replay_initialize_memory(CartReplayMemoryInitializer initialize, void *context);
size_t cart_replay_save_ram_size(void);
bool cart_replay_set_save_ram(const uint8_t *bytes, size_t size);

#ifdef __cplusplus
}
#endif
#endif
